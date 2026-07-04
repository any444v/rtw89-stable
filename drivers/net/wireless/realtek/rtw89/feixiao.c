// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/* feixiao.c — macOS (Feixiao kext) bridge for rtw89.
 *
 * Compiled only for the macOS port (RTW89_MACOS).  The Feixiao kext was
 * written against the rtw88 port and links against a fixed set of symbol
 * names (rtw88_* helpers plus a few rtw_* driver entry points).  This file
 * implements those same names on top of rtw89 so the kext sources build
 * against either driver unchanged.
 *
 * It lives inside the driver tree (not in the kext's compat layer) because
 * it needs the real rtw89 headers; the compat tree's linux/pci.h shadows
 * the driver's pci.h on the kext include path.
 */
#ifdef RTW89_MACOS

#include "cam.h"
#include "chan.h"
#include "core.h"
#include "debug.h"
#include "fw.h"
#include "mac.h"
#include "pci.h"
#include "ps.h"
#include "txrx.h"

/* The kext-facing API (declared in rtw88_compat.h, force-included) passes
 * an opaque "struct rtw_dev *"; here it is really a struct rtw89_dev. */
#define to_rtw89(p) ((struct rtw89_dev *)(p))

/* Globals shared with the generic compat layer (rtw88_compat.c). */
extern void *g_irq_dev_id;                       /* rtw89_dev of active IRQ */
extern struct ieee80211_hw *rtw88_get_hw(void);

/* Track-work kill switch consumed by core.c (macOS diagnostic patch). */
bool rtw89_disable_track_work;

/* ------------------------------------------------------------------ */
/*  PCI probe/remove                                                   */
/* ------------------------------------------------------------------ */

/* Per-chip PCI id tables exported by the rtw89 *e.c files under
 * RTW89_MACOS.  Each table is zero-terminated and its driver_data
 * carries the chip's rtw89_driver_info, exactly as the Linux PCI core
 * would pass it. */
extern const struct pci_device_id *rtw89_8851be_feixiao_ids;
extern const struct pci_device_id *rtw89_8852ae_feixiao_ids;
extern const struct pci_device_id *rtw89_8852be_feixiao_ids;
extern const struct pci_device_id *rtw89_8852bte_feixiao_ids;
extern const struct pci_device_id *rtw89_8852ce_feixiao_ids;
extern const struct pci_device_id *rtw89_8922ae_feixiao_ids;

static const struct pci_device_id *rtw89_feixiao_match(u16 device)
{
	const struct pci_device_id **tables[] = {
		&rtw89_8851be_feixiao_ids,
		&rtw89_8852ae_feixiao_ids,
		&rtw89_8852be_feixiao_ids,
		&rtw89_8852bte_feixiao_ids,
		&rtw89_8852ce_feixiao_ids,
		&rtw89_8922ae_feixiao_ids,
	};
	const struct pci_device_id *id;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(tables); i++)
		for (id = *tables[i]; id->vendor; id++)
			if (id->device == device)
				return id;
	return NULL;
}

/* The kext calls rtw_pci_probe() with a fake pci_device_id it built from
 * its own chip table; for rtw89 the real driver_data (rtw89_driver_info)
 * lives in the *e.c id tables, so match by PCI device id and ignore the
 * caller's driver_data. */
int rtw_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	const struct pci_device_id *real_id = rtw89_feixiao_match(pdev->device);

	if (!real_id) {
		pr_err("rtw89: no chip info for PCI device %04x\n",
		       pdev->device);
		return -ENODEV;
	}

	return rtw89_pci_probe(pdev, real_id);
}

void rtw_pci_remove(struct pci_dev *pdev)
{
	rtw89_pci_remove(pdev);
}

/* USB path not wired up for rtw89 yet; PCIe chips only. */
int rtw_usb_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	return -ENODEV;
}

void rtw_usb_disconnect(struct usb_interface *intf)
{
}

/* ------------------------------------------------------------------ */
/*  Interrupt / TX-ring helpers                                        */
/* ------------------------------------------------------------------ */

void rtw88_reenable_interrupt(void)
{
	if (g_irq_dev_id)
		rtw89_pci_enable_intr_lock(to_rtw89(g_irq_dev_id));
}

/* BE-ring free-slot count for the kext TX flow-control (backpressure)
 * decision.  Same math as pci.c's rtw89_pci_get_avail_txbd_num(), clamped
 * by the wd ring like __rtw89_pci_check_and_reclaim_tx_resource_noio();
 * read locklessly — a stale value only makes the kext stall or resume one
 * frame early, never corrupts state. */
u32 rtw88_be_ring_avail(struct rtw_dev *rtwdev_opaque)
{
	struct rtw89_dev *rtwdev = to_rtw89(rtwdev_opaque);
	struct rtw89_pci *rtwpci = (struct rtw89_pci *)rtwdev->priv;
	struct rtw89_pci_tx_ring *tx_ring;
	struct rtw89_pci_dma_ring *bd_ring;
	u32 avail;
	u8 txch;

	txch = rtw89_chip_get_ch_dma(rtwdev, RTW89_TX_QSEL_BE_0);
	tx_ring = &rtwpci->tx.rings[txch];
	bd_ring = &tx_ring->bd_ring;

	if (bd_ring->rp > bd_ring->wp)
		avail = bd_ring->rp - bd_ring->wp - 1;
	else
		avail = bd_ring->len - (bd_ring->wp - bd_ring->rp) - 1;

	return min(avail, tx_ring->wd_ring.curr_num);
}

/* Dump SW-side BE TX ring + device state.  Called by the kext's periodic
 * debug timer to diagnose a TX freeze.  Unlike the rtw88 version this
 * stays off the MMIO bus: register maps differ per rtw89 generation
 * (AX/BE), and the SW counters plus RPQ state already distinguish
 * "chip stopped consuming" from "no release reports coming back". */
void rtw88_debug_dump_tx_state(void)
{
	struct rtw89_dev *rtwdev;
	struct rtw89_pci *rtwpci;
	struct rtw89_pci_tx_ring *tx_ring;
	struct rtw89_pci_rx_ring *rpq;
	u8 txch;

	if (!g_irq_dev_id)
		return;
	rtwdev = to_rtw89(g_irq_dev_id);
	rtwpci = (struct rtw89_pci *)rtwdev->priv;

	txch = rtw89_chip_get_ch_dma(rtwdev, RTW89_TX_QSEL_BE_0);
	tx_ring = &rtwpci->tx.rings[txch];
	rpq = &rtwpci->rx.rings[RTW89_RXCH_RPQ];

	pr_info("rtw89: TXSTATE BE(ch%u) bd wp=%u rp=%u len=%u wd_avail=%u/%u rpq wp=%u rp=%u running=%d power=%d\n",
		txch,
		tx_ring->bd_ring.wp, tx_ring->bd_ring.rp,
		tx_ring->bd_ring.len,
		tx_ring->wd_ring.curr_num, tx_ring->wd_ring.page_num,
		rpq->bd_ring.wp, rpq->bd_ring.rp,
		rtwpci->running ? 1 : 0,
		test_bit(RTW89_FLAG_POWERON, rtwdev->flags) ? 1 : 0);
}

/* ------------------------------------------------------------------ */
/*  Scan bridges                                                       */
/* ------------------------------------------------------------------ */

bool rtw88_is_scanning(void)
{
	struct ieee80211_hw *hw = rtw88_get_hw();

	if (!hw || !hw->priv)
		return false;
	return to_rtw89(hw->priv)->scanning;
}

bool rtw88_hw_scan_supported(struct ieee80211_hw *hw)
{
	struct rtw89_dev *rtwdev;

	if (!hw || !hw->priv)
		return false;
	rtwdev = to_rtw89(hw->priv);
	return RTW89_CHK_FW_FEATURE(SCAN_OFFLOAD, &rtwdev->fw);
}

/* Mirrors rtw89_ops_sw_scan_start() (mac80211.c) minus the wiphy-lock
 * bookkeeping — the kext serializes these calls itself. */
void rtw88_sw_scan_start(struct ieee80211_hw *hw, struct ieee80211_vif *vif)
{
	struct rtw89_dev *rtwdev;
	struct rtw89_vif *rtwvif;
	struct rtw89_vif_link *rtwvif_link;

	if (!hw || !hw->priv || !vif)
		return;
	rtwdev = to_rtw89(hw->priv);
	rtwvif = (struct rtw89_vif *)vif->drv_priv;

	rtwvif_link = rtw89_get_designated_link(rtwvif);
	if (!rtwvif_link) {
		rtw89_err(rtwdev, "sw scan start: find no designated link\n");
		return;
	}

	rtw89_leave_lps(rtwdev);
	rtw89_core_scan_start(rtwdev, rtwvif_link, vif->addr, false);
}

void rtw88_sw_scan_switch_channel(struct ieee80211_hw *hw)
{
	struct rtw89_dev *rtwdev;

	if (!hw || !hw->priv)
		return;
	rtwdev = to_rtw89(hw->priv);

	/* rtw89 derives the channel from its entity state, not hw->conf —
	 * mirror the chandef the kext just wrote before programming. */
	rtw89_config_entity_chandef(rtwdev, RTW89_CHANCTX_0,
				    &hw->conf.chandef);
	rtw89_set_channel(rtwdev);
}

void rtw88_sw_scan_complete(struct ieee80211_hw *hw, struct ieee80211_vif *vif)
{
	struct rtw89_dev *rtwdev;
	struct rtw89_vif *rtwvif;
	struct rtw89_vif_link *rtwvif_link;

	if (!hw || !hw->priv || !vif)
		return;
	rtwdev = to_rtw89(hw->priv);
	rtwvif = (struct rtw89_vif *)vif->drv_priv;

	rtwvif_link = rtw89_get_designated_link(rtwvif);
	if (!rtwvif_link)
		return;

	rtw89_core_scan_complete(rtwdev, rtwvif_link, false);
}

/* ------------------------------------------------------------------ */
/*  Coex / connect helpers                                             */
/* ------------------------------------------------------------------ */

/* rtw88 semantics: stop the WiFi/BT coexistence engine from throttling
 * WiFi on machines with no functional BT controller.  rtw89 has no
 * efuse.btcoex flag; the equivalent is BTC manual control, which stops
 * the periodic coex algorithm from reacting to (never-arriving) BT
 * firmware replies.  Same mechanism as debugfs' btc_manual node. */
void rtw88_force_wifi_only(void)
{
	struct ieee80211_hw *hw = rtw88_get_hw();
	struct rtw89_dev *rtwdev;
	struct rtw89_btc *btc;

	if (!hw || !hw->priv)
		return;
	rtwdev = to_rtw89(hw->priv);
	btc = &rtwdev->btc;

	btc->manual_ctrl = true;
	if (btc->ver && btc->ver->fcxctrl == 7)
		btc->ctrl.ctrl_v7.manual = true;
	else
		btc->ctrl.ctrl.manual = true;

	pr_info("rtw89: forcing wifi-only (BTC manual control)\n");
}

/* Set channel + BSSID for the kext's connect flow (it bypasses mac80211's
 * bss_info_changed path).  Caller populates hw->conf.chandef first, same
 * contract as the rtw88 version. */
void rtw88_connect_hw_setup(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			    const uint8_t *bssid)
{
	struct rtw89_dev *rtwdev;
	struct rtw89_vif *rtwvif;
	struct rtw89_vif_link *rtwvif_link;

	if (!hw || !hw->priv || !vif || !bssid)
		return;
	rtwdev = to_rtw89(hw->priv);
	rtwvif = (struct rtw89_vif *)vif->drv_priv;

	rtwvif_link = rtw89_get_designated_link(rtwvif);
	if (!rtwvif_link)
		return;

	rtw89_leave_lps(rtwdev);

	/* rtw89 has no non-chanctx channel path: rtw89_set_channel() reads
	 * hal.chanctx[].chandef, which only mac80211's chanctx ops normally
	 * fill.  The kext bypasses those, so mirror hw->conf.chandef into
	 * the entity state here — otherwise the radio stays on the boot
	 * default channel and auth frames never reach the AP. */
	rtw89_config_entity_chandef(rtwdev, RTW89_CHANCTX_0,
				    &hw->conf.chandef);
	rtw89_set_channel(rtwdev);

	/* BSSID goes to the address CAM via H2C, not an MMIO port register
	 * like rtw88's PORT_SET_BSSID. */
	ether_addr_copy(rtwvif_link->bssid, bssid);
	rtw89_cam_bssid_changed(rtwdev, rtwvif_link);
	rtw89_fw_h2c_cam(rtwdev, rtwvif_link, NULL, NULL,
			 RTW89_ROLE_INFO_CHANGE);
}

/* Re-apply channel + BSSID after an internal reset while associated. */
void rtw88_restore_connected_hw(struct ieee80211_hw *hw,
				struct ieee80211_vif *vif,
				const uint8_t *bssid)
{
	rtw88_connect_hw_setup(hw, vif, bssid);
}

/* ------------------------------------------------------------------ */
/*  Info helpers                                                       */
/* ------------------------------------------------------------------ */

void rtw88_get_fw_version(struct rtw_dev *rtwdev_opaque, uint16_t *version,
			  uint8_t *sub_version)
{
	struct rtw89_dev *rtwdev = to_rtw89(rtwdev_opaque);
	const struct rtw89_fw_suit *fw_suit;

	if (version)
		*version = 0;
	if (sub_version)
		*sub_version = 0;
	if (!rtwdev)
		return;

	fw_suit = rtw89_fw_suit_get(rtwdev, RTW89_FW_NORMAL);
	if (version)
		*version = ((uint16_t)fw_suit->major_ver << 8) |
			   fw_suit->minor_ver;
	if (sub_version)
		*sub_version = fw_suit->sub_ver;
}

void rtw88_get_chip_name(struct rtw_dev *rtwdev_opaque, char *name_buf,
			 size_t buf_sz)
{
	struct rtw89_dev *rtwdev = to_rtw89(rtwdev_opaque);

	if (!name_buf || buf_sz == 0)
		return;
	name_buf[0] = '\0';
	if (!rtwdev || !rtwdev->chip) {
		strlcpy(name_buf, "Unknown", buf_sz);
		return;
	}

	switch (rtwdev->chip->chip_id) {
	case RTL8851B:
		strlcpy(name_buf, "RTL8851BE", buf_sz);
		break;
	case RTL8852A:
		strlcpy(name_buf, "RTL8852AE", buf_sz);
		break;
	case RTL8852B:
		strlcpy(name_buf, "RTL8852BE", buf_sz);
		break;
	case RTL8852BT:
		strlcpy(name_buf, "RTL8852BTE", buf_sz);
		break;
	case RTL8852C:
		strlcpy(name_buf, "RTL8852CE", buf_sz);
		break;
	case RTL8922A:
		strlcpy(name_buf, "RTL8922AE", buf_sz);
		break;
	default:
		strlcpy(name_buf, "RTL89xx (Unknown)", buf_sz);
		break;
	}
}

void rtw88_get_stats(struct rtw_dev *rtwdev_opaque, uint32_t *tx_bytes,
		     uint32_t *rx_bytes)
{
	struct rtw89_dev *rtwdev = to_rtw89(rtwdev_opaque);

	if (tx_bytes)
		*tx_bytes = 0;
	if (rx_bytes)
		*rx_bytes = 0;
	if (!rtwdev)
		return;
	if (tx_bytes)
		*tx_bytes = (uint32_t)rtwdev->stats.tx_unicast;
	if (rx_bytes)
		*rx_bytes = (uint32_t)rtwdev->stats.rx_unicast;
}

#endif /* RTW89_MACOS */
