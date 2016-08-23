/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corp.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

/**
 * @file bna_ethtool.c  QLogic BR-Series 10G PCIe Ethernet driver.
 */

#include "cna_os.h"

#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/ethtool.h>
#include <linux/rtnetlink.h>
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 31))
#include <linux/firmware.h>
#endif

#include "bna.h"

#include "bnad_compat.h"
#include "bnad_trcmod.h"
#include "bnad_ioctl_common.h"
#include "bnad.h"

#define BNAD_NUM_TXF_COUNTERS 12
#define BNAD_NUM_RXF_COUNTERS 10
#define BNAD_NUM_CQ_COUNTERS (3 + 5)
#define BNAD_NUM_RXQ_COUNTERS 6
#define BNAD_NUM_TXQ_COUNTERS 5

#define BNAD_ETHTOOL_STATS_NUM						\
	(sizeof(struct net_device_stats) / sizeof(unsigned long) +	\
	sizeof(struct bnad_drv_stats_s) / sizeof(uint64_t) +		\
	offsetof(struct bfi_enet_stats, rxf_stats[0]) / sizeof(uint64_t))

// static char *bnad_net_stats_strings[BNAD_ETHTOOL_STATS_NUM] = {
static char *bnad_net_stats_strings[] = {
	"rx_packets",
	"tx_packets",
	"rx_bytes",
	"tx_bytes",
	"rx_errors",
	"tx_errors",
	"rx_dropped",
	"tx_dropped",
	"multicast",
	"collisions",

	"rx_length_errors",
	"rx_over_errors",
	"rx_crc_errors",
	"rx_frame_errors",
	"rx_fifo_errors",
	"rx_missed_errors",

	"tx_aborted_errors",
	"tx_carrier_errors",
	"tx_fifo_errors",
	"tx_heartbeat_errors",
	"tx_window_errors",

	"rx_compressed",
	"tx_compressed",

	"netif_queue_stop",
	"netif_queue_wakeup",
	"netif_queue_stopped",
	"tso4",
	"tso6",
	"tso_err",
	"tcpcsum_offload",
	"udpcsum_offload",
	"csum_help",
	"tx_skb_too_short",
	"tx_skb_stopping",
	"tx_skb_max_vectors",
	"tx_skb_mss_too_long",
	"tx_skb_tso_too_short",
	"tx_skb_tso_prepare",
	"tx_skb_non_tso_too_long",
	"tx_skb_tcp_hdr",
	"tx_skb_udp_hdr",
	"tx_skb_csum_err",
	"tx_skb_headlen_too_long",
	"tx_skb_headlen_zero",
	"tx_skb_frag_zero",
	"tx_skb_len_mismatch",
	"hw_stats_updates",
	"netif_rx_dropped",
	"lro_flushed",
	"lro_aggregated",

	"link_toggle",
	"cee_toggle",

	"rxp_info_alloc_failed",
	"mbox_intr_disabled",
	"mbox_intr_enabled",
	"tx_unmap_q_alloc_failed",
	"rx_unmap_q_alloc_failed",
	"rxbuf_alloc_failed",

	"mac_stats_clr_cnt",
	"mac_frame_64",
	"mac_frame_65_127",
	"mac_frame_128_255",
	"mac_frame_256_511",
	"mac_frame_512_1023",
	"mac_frame_1024_1518",
	"mac_frame_1518_1522",
	"mac_rx_bytes",
	"mac_rx_packets",
	"mac_rx_fcs_error",
	"mac_rx_multicast",
	"mac_rx_broadcast",
	"mac_rx_control_frames",
	"mac_rx_pause",
	"mac_rx_unknown_opcode",
	"mac_rx_alignment_error",
	"mac_rx_frame_length_error",
	"mac_rx_code_error",
	"mac_rx_carrier_sense_error",
	"mac_rx_undersize",
	"mac_rx_oversize",
	"mac_rx_fragments",
	"mac_rx_jabber",
	"mac_rx_drop",

	"mac_tx_bytes",
	"mac_tx_packets",
	"mac_tx_multicast",
	"mac_tx_broadcast",
	"mac_tx_pause",
	"mac_tx_deferral",
	"mac_tx_excessive_deferral",
	"mac_tx_single_collision",
	"mac_tx_muliple_collision",
	"mac_tx_late_collision",
	"mac_tx_excessive_collision",
	"mac_tx_total_collision",
	"mac_tx_pause_honored",
	"mac_tx_drop",
	"mac_tx_jabber",
	"mac_tx_fcs_error",
	"mac_tx_control_frame",
	"mac_tx_oversize",
	"mac_tx_undersize",
	"mac_tx_fragments",

	"bpc_tx_pause_0",
	"bpc_tx_pause_1",
	"bpc_tx_pause_2",
	"bpc_tx_pause_3",
	"bpc_tx_pause_4",
	"bpc_tx_pause_5",
	"bpc_tx_pause_6",
	"bpc_tx_pause_7",
	"bpc_tx_zero_pause_0",
	"bpc_tx_zero_pause_1",
	"bpc_tx_zero_pause_2",
	"bpc_tx_zero_pause_3",
	"bpc_tx_zero_pause_4",
	"bpc_tx_zero_pause_5",
	"bpc_tx_zero_pause_6",
	"bpc_tx_zero_pause_7",
	"bpc_tx_first_pause_0",
	"bpc_tx_first_pause_1",
	"bpc_tx_first_pause_2",
	"bpc_tx_first_pause_3",
	"bpc_tx_first_pause_4",
	"bpc_tx_first_pause_5",
	"bpc_tx_first_pause_6",
	"bpc_tx_first_pause_7",

	"bpc_rx_pause_0",
	"bpc_rx_pause_1",
	"bpc_rx_pause_2",
	"bpc_rx_pause_3",
	"bpc_rx_pause_4",
	"bpc_rx_pause_5",
	"bpc_rx_pause_6",
	"bpc_rx_pause_7",
	"bpc_rx_zero_pause_0",
	"bpc_rx_zero_pause_1",
	"bpc_rx_zero_pause_2",
	"bpc_rx_zero_pause_3",
	"bpc_rx_zero_pause_4",
	"bpc_rx_zero_pause_5",
	"bpc_rx_zero_pause_6",
	"bpc_rx_zero_pause_7",
	"bpc_rx_first_pause_0",
	"bpc_rx_first_pause_1",
	"bpc_rx_first_pause_2",
	"bpc_rx_first_pause_3",
	"bpc_rx_first_pause_4",
	"bpc_rx_first_pause_5",
	"bpc_rx_first_pause_6",
	"bpc_rx_first_pause_7",

	"rad_rx_frames",
	"rad_rx_octets",
	"rad_rx_vlan_frames",
	"rad_rx_ucast",
	"rad_rx_ucast_octets",
	"rad_rx_ucast_vlan",
	"rad_rx_mcast",
	"rad_rx_mcast_octets",
	"rad_rx_mcast_vlan",
	"rad_rx_bcast",
	"rad_rx_bcast_octets",
	"rad_rx_bcast_vlan",
	"rad_rx_drops",

	"rlb_rad_rx_frames",
	"rlb_rad_rx_octets",
	"rlb_rad_rx_vlan_frames",
	"rlb_rad_rx_ucast",
	"rlb_rad_rx_ucast_octets",
	"rlb_rad_rx_ucast_vlan",
	"rlb_rad_rx_mcast",
	"rlb_rad_rx_mcast_octets",
	"rlb_rad_rx_mcast_vlan",
	"rlb_rad_rx_bcast",
	"rlb_rad_rx_bcast_octets",
	"rlb_rad_rx_bcast_vlan",
	"rlb_rad_rx_drops",

	"fc_rx_ucast_octets",
	"fc_rx_ucast",
	"fc_rx_ucast_vlan",
	"fc_rx_mcast_octets",
	"fc_rx_mcast",
	"fc_rx_mcast_vlan",
	"fc_rx_bcast_octets",
	"fc_rx_bcast",
	"fc_rx_bcast_vlan",

	"fc_tx_ucast_octets",
	"fc_tx_ucast",
	"fc_tx_ucast_vlan",
	"fc_tx_mcast_octets",
	"fc_tx_mcast",
	"fc_tx_mcast_vlan",
	"fc_tx_bcast_octets",
	"fc_tx_bcast",
	"fc_tx_bcast_vlan",
	"fc_tx_parity_errors",
	"fc_tx_timeout",
	"fc_tx_fid_parity_errors",
};


static int
bnad_get_settings(struct net_device *netdev, struct ethtool_cmd *cmd)
{
	cmd->supported = SUPPORTED_10000baseT_Full;
	cmd->advertising = ADVERTISED_10000baseT_Full;
	cmd->autoneg = AUTONEG_DISABLE;
	cmd->supported |= SUPPORTED_FIBRE;
	cmd->advertising |= ADVERTISED_FIBRE;
	cmd->port = PORT_FIBRE;
	cmd->phy_address = 0;

	if (netif_carrier_ok(netdev)) {
		cmd->speed = SPEED_10000;
		cmd->duplex = DUPLEX_FULL;
	} else {
		cmd->speed = -1;
		cmd->duplex = -1;
	}
	cmd->transceiver = XCVR_EXTERNAL;
	cmd->maxtxpkt = 0;
	cmd->maxrxpkt = 0;

	return 0;
}

static int
bnad_set_settings(struct net_device *netdev, struct ethtool_cmd *cmd)
{
	/* 10G full duplex setting supported only */
	if (cmd->autoneg == AUTONEG_ENABLE) {
		return -EOPNOTSUPP;
	} else {
		if ((cmd->speed == SPEED_10000) && (cmd->duplex == DUPLEX_FULL))
			return 0;
	}

	return -EOPNOTSUPP;
}

static void
bnad_get_drvinfo(struct net_device *netdev, struct ethtool_drvinfo *drvinfo)
{
	struct bnad_s *bnad = netdev_priv(netdev);
	struct bfa_ioc_attr_s *ioc_attr;
	unsigned long flags;

	strcpy(drvinfo->driver, BNAD_NAME);
	strcpy(drvinfo->version, BNAD_VERSION);

	ioc_attr = kzalloc(sizeof(*ioc_attr), GFP_KERNEL);
	if (ioc_attr) {
		memset(ioc_attr, 0, sizeof(*ioc_attr));
		spin_lock_irqsave(&bnad->bna_lock, flags);
		bfa_ioc_get_attr(&bnad->bna.ioceth.ioc, ioc_attr);
		spin_unlock_irqrestore(&bnad->bna_lock, flags);

		strncpy(drvinfo->fw_version, ioc_attr->adapter_attr.fw_ver,
			sizeof(drvinfo->fw_version) - 1);
		kfree(ioc_attr);
	}

	strncpy(drvinfo->bus_info, pci_name(bnad->pcidev), ETHTOOL_BUSINFO_LEN);
}

static void
bnad_get_wol(struct net_device *netdev, struct ethtool_wolinfo *wolinfo)
{
	wolinfo->supported = 0;
	wolinfo->wolopts = 0;
}

static uint32_t
bnad_get_msglevel(struct net_device *netdev)
{
	uint32_t msglevel;
	struct bnad_s *bnad = netdev_priv(netdev);
	msglevel = bnad_get_loglevel(bnad, 0);
	return msglevel;
}

static void
bnad_set_msglevel(struct net_device *netdev, u32 msglevel)
{
	struct bnad_s *bnad = netdev_priv(netdev);
	bnad_set_loglevel(bnad, msglevel, 0);
}

static int
bnad_get_coalesce(struct net_device *netdev, struct ethtool_coalesce *coalesce)
{
	struct bnad_s *bnad = netdev_priv(netdev);

	coalesce->use_adaptive_rx_coalesce =
		(bnad->cfg_flags & BNAD_CF_DIM_ENABLED) ? true : false;
	coalesce->rx_coalesce_usecs = bnad->rx_coalescing_timeo *
					BFI_COALESCING_TIMER_UNIT;
	coalesce->tx_coalesce_usecs = bnad->tx_coalescing_timeo *
					BFI_COALESCING_TIMER_UNIT;
	coalesce->tx_max_coalesced_frames = BFI_TX_INTERPKT_COUNT;

	return 0;
}

static int
bnad_set_coalesce(struct net_device *netdev, struct ethtool_coalesce *coalesce)
{
	struct bnad_s 		*bnad = netdev_priv(netdev);
	unsigned long		flags;

	if (coalesce->rx_coalesce_usecs < BFI_COALESCING_TIMER_UNIT ||
	    coalesce->rx_coalesce_usecs >
	    BFI_MAX_COALESCING_TIMEO * BFI_COALESCING_TIMER_UNIT)
		return -EINVAL;

	if (coalesce->tx_coalesce_usecs < BFI_COALESCING_TIMER_UNIT ||
	    coalesce->tx_coalesce_usecs >
	    BFI_MAX_COALESCING_TIMEO * BFI_COALESCING_TIMER_UNIT)
		return -EINVAL;

	bnad_conf_lock();

	spin_lock_irqsave(&bnad->bna_lock, flags);
	if (coalesce->use_adaptive_rx_coalesce) {
		if (!(bnad->cfg_flags & BNAD_CF_DIM_ENABLED))
			bnad->cfg_flags |= BNAD_CF_DIM_ENABLED;
	} else {
		if (bnad->cfg_flags & BNAD_CF_DIM_ENABLED) {
			bnad->cfg_flags &= ~BNAD_CF_DIM_ENABLED;
			bnad_rx_coalescing_timeo_set(bnad);
		}
	}
	if (bnad->tx_coalescing_timeo != coalesce->tx_coalesce_usecs /
					BFI_COALESCING_TIMER_UNIT) {
		bnad->tx_coalescing_timeo = coalesce->tx_coalesce_usecs /
						BFI_COALESCING_TIMER_UNIT;
		bnad_tx_coalescing_timeo_set(bnad);
	}

	if (bnad->rx_coalescing_timeo != coalesce->rx_coalesce_usecs /
					BFI_COALESCING_TIMER_UNIT) {
		bnad->rx_coalescing_timeo = coalesce->rx_coalesce_usecs /
						BFI_COALESCING_TIMER_UNIT;

		bnad_rx_coalescing_timeo_set(bnad);

	}

	/* Add Tx Inter-pkt DMA count?  */

	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	bnad_conf_unlock();
	return 0;
}

static void
bnad_get_ringparam(struct net_device *netdev,
		   struct ethtool_ringparam *ringparam)
{
	struct bnad_s *bnad = netdev_priv(netdev);

	ringparam->rx_max_pending = BNAD_MAX_RXQ_DEPTH;
	ringparam->rx_mini_max_pending = 0;
	ringparam->rx_jumbo_max_pending = 0;
	ringparam->tx_max_pending = BNAD_MAX_TXQ_DEPTH;

	ringparam->rx_pending = bnad->rxq_depth;
	ringparam->rx_mini_max_pending = 0;
	ringparam->rx_jumbo_max_pending = 0;
	ringparam->tx_pending = bnad->txq_depth;
}

static int
bnad_set_ringparam(struct net_device *netdev,
		   struct ethtool_ringparam *ringparam)
{
	struct bnad_s *bnad = netdev_priv(netdev);
	int i, current_err, err = 0;
	bool invalidate_netq;
	uint32_t rx_count;

	bnad_conf_lock();

	if (ringparam->rx_pending == bnad->rxq_depth &&
	    ringparam->tx_pending == bnad->txq_depth) {
		goto ret_err;
	}

	if (ringparam->rx_pending < BNAD_MIN_Q_DEPTH ||
	    ringparam->rx_pending > BNAD_MAX_RXQ_DEPTH ||
	    !BNA_POWER_OF_2(ringparam->rx_pending)) {
		err = -EINVAL;
		goto ret_err;
	}
	if (ringparam->tx_pending < BNAD_MIN_Q_DEPTH ||
	    ringparam->tx_pending > BNAD_MAX_TXQ_DEPTH ||
	    !BNA_POWER_OF_2(ringparam->tx_pending)) {
		err = -EINVAL;
		goto ret_err;
	}

	if (ringparam->rx_pending != bnad->rxq_depth) {
		bnad->rxq_depth = ringparam->rx_pending;
		if (!netif_running(netdev))
			goto ret_err;

		rx_count = bnad_reinit_rx(bnad);
		if (rx_count > 0)
			invalidate_netq = BFA_TRUE;
	}

	if (ringparam->tx_pending != bnad->txq_depth) {
		bnad->txq_depth = ringparam->tx_pending;
		if (!netif_running(netdev))
			goto ret_err;

		bnad_netdev_trans_start_set(netdev);

		for (i = 0; i < bnad->num_tx; i++) {
			if (!bnad->tx_info[i].tx)
				continue;
			invalidate_netq = BFA_TRUE;
			bnad_destroy_tx(bnad, i);
			current_err = bnad_setup_tx(bnad, i);
			if (current_err && !err)
				err = current_err;
		}
	}

	bnad_netq_reinit_needed(bnad, invalidate_netq);

ret_err:
	bnad_conf_unlock();
	return err;
}

static void
bnad_get_pauseparam(struct net_device *netdev,
		    struct ethtool_pauseparam *pauseparam)
{
	struct bnad_s *bnad = netdev_priv(netdev);

	pauseparam->autoneg = 0;
	pauseparam->rx_pause = bnad->bna.enet.pause_config.rx_pause;
	pauseparam->tx_pause = bnad->bna.enet.pause_config.tx_pause;
}

static int
bnad_set_pauseparam(struct net_device *netdev,
		    struct ethtool_pauseparam *pauseparam)
{
	struct bnad_s *bnad = netdev_priv(netdev);
	struct bna_pause_config_s pause_config;
	unsigned long flags;

	if (pauseparam->autoneg == AUTONEG_ENABLE)
		return -EINVAL;

	bnad_conf_lock();
	if (pauseparam->rx_pause != bnad->bna.enet.pause_config.rx_pause ||
	    pauseparam->tx_pause != bnad->bna.enet.pause_config.tx_pause) {
		pause_config.rx_pause = pauseparam->rx_pause;
		pause_config.tx_pause = pauseparam->tx_pause;
		spin_lock_irqsave(&bnad->bna_lock, flags);
		bna_enet_pause_config(&bnad->bna.enet, &pause_config, NULL);
		spin_unlock_irqrestore(&bnad->bna_lock, flags);
	}
	bnad_conf_unlock();
	return 0;
}

#if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 1, 0)
static u32
bnad_get_rx_csum(struct net_device *netdev)
{
	uint32_t rx_csum;
	struct bnad_s *bnad = netdev_priv(netdev);

	rx_csum = bnad->rx_csum;
	return rx_csum;
}

static int
bnad_set_rx_csum(struct net_device *netdev, u32 rx_csum)
{
	struct bnad_s *bnad = netdev_priv(netdev);

	bnad_conf_lock();
	bnad->rx_csum = rx_csum;
	bnad_conf_unlock();
	return 0;
}

static int
bnad_set_tx_csum(struct net_device *netdev, u32 tx_csum)
{
	struct bnad_s *bnad = netdev_priv(netdev);

	bnad_conf_lock();
	if (tx_csum) {
		netdev->features |= NETIF_F_IP_CSUM;
#ifdef NETIF_F_IPV6_CSUM
		netdev->features |= NETIF_F_IPV6_CSUM;
#endif
	} else {
		netdev->features &= ~NETIF_F_IP_CSUM;
#ifdef NETIF_F_IPV6_CSUM
		netdev->features &= ~NETIF_F_IPV6_CSUM;
#endif
	}
	bnad_conf_unlock();
	return 0;
}

#ifdef NETIF_F_TSO
static int
bnad_set_tso(struct net_device *netdev, u32 tso)
{
	struct bnad_s *bnad = netdev_priv(netdev);

	bnad_conf_lock();
	if (tso) {
		netdev->features |= NETIF_F_TSO;
#ifdef NETIF_F_TSO6
		netdev->features |= NETIF_F_TSO6;
#endif
	} else {
		netdev->features &= ~NETIF_F_TSO;
#ifdef NETIF_F_TSO6
		netdev->features &= ~NETIF_F_TSO6;
#endif
	}
	bnad_conf_unlock();
	return 0;
}
#endif
#endif

static void
bnad_get_strings(struct net_device *netdev, u32 stringset, u8 * string)
{
	struct bnad_s *bnad = netdev_priv(netdev);
	int i, j, q_num;
	uint32_t bmap;

	bnad_conf_lock();

	switch (stringset) {
	case ETH_SS_STATS:
		for (i = 0; i < BNAD_ETHTOOL_STATS_NUM; i++) {
			CNA_ASSERT(strlen(bnad_net_stats_strings[i]) <
				   ETH_GSTRING_LEN);
			memcpy(string, bnad_net_stats_strings[i],
			       ETH_GSTRING_LEN);
			string += ETH_GSTRING_LEN;
		}
		bmap = bna_tx_rid_mask(&bnad->bna);
		for (i = 0; bmap; i++) {
			if (bmap & 1) {
				sprintf(string, "txf%d_ucast_octets", i);
				string += ETH_GSTRING_LEN;
				sprintf(string, "txf%d_ucast", i);
				string += ETH_GSTRING_LEN;
				sprintf(string, "txf%d_ucast_vlan", i);
				string += ETH_GSTRING_LEN;
				sprintf(string, "txf%d_mcast_octets", i);
				string += ETH_GSTRING_LEN;
				sprintf(string, "txf%d_mcast", i);
				string += ETH_GSTRING_LEN;
				sprintf(string, "txf%d_mcast_vlan", i);
				string += ETH_GSTRING_LEN;
				sprintf(string, "txf%d_bcast_octets", i);
				string += ETH_GSTRING_LEN;
				sprintf(string, "txf%d_bcast", i);
				string += ETH_GSTRING_LEN;
				sprintf(string, "txf%d_bcast_vlan", i);
				string += ETH_GSTRING_LEN;
				sprintf(string, "txf%d_errors", i);
				string += ETH_GSTRING_LEN;
				sprintf(string, "txf%d_filter_vlan", i);
				string += ETH_GSTRING_LEN;
				sprintf(string, "txf%d_filter_mac_sa", i);
				string += ETH_GSTRING_LEN;
			}
			bmap >>= 1;
		}

		bmap = bna_rx_rid_mask(&bnad->bna);
		for (i = 0; bmap; i++) {
			if (bmap & 1) {
				sprintf(string, "rxf%d_ucast_octets", i);
				string += ETH_GSTRING_LEN;
				sprintf(string, "rxf%d_ucast", i);
				string += ETH_GSTRING_LEN;
				sprintf(string, "rxf%d_ucast_vlan", i);
				string += ETH_GSTRING_LEN;
				sprintf(string, "rxf%d_mcast_octets", i);
				string += ETH_GSTRING_LEN;
				sprintf(string, "rxf%d_mcast", i);
				string += ETH_GSTRING_LEN;
				sprintf(string, "rxf%d_mcast_vlan", i);
				string += ETH_GSTRING_LEN;
				sprintf(string, "rxf%d_bcast_octets", i);
				string += ETH_GSTRING_LEN;
				sprintf(string, "rxf%d_bcast", i);
				string += ETH_GSTRING_LEN;
				sprintf(string, "rxf%d_bcast_vlan", i);
				string += ETH_GSTRING_LEN;
				sprintf(string, "rxf%d_frame_drops", i);
				string += ETH_GSTRING_LEN;
			}
			bmap >>= 1;
		}

		q_num = 0;
		for (i = 0; i < bnad->num_rx; i++) {
			if (!bnad->rx_info[i].rx)
				continue;
			for (j = 0; j < bnad->num_rxp_per_rx; j++) {
				sprintf(string, "cq%d_producer_index", q_num);
				string += ETH_GSTRING_LEN;
				sprintf(string, "cq%d_consumer_index", q_num);
				string += ETH_GSTRING_LEN;
				sprintf(string, "cq%d_hw_producer_index",
					q_num);
				string += ETH_GSTRING_LEN;
				sprintf(string, "cq%d_intr", q_num);
				string += ETH_GSTRING_LEN;
				sprintf(string, "cq%d_poll", q_num);
				string += ETH_GSTRING_LEN;
				sprintf(string, "cq%d_schedule", q_num);
				string += ETH_GSTRING_LEN;
				sprintf(string, "cq%d_keep_poll", q_num);
				string += ETH_GSTRING_LEN;
				sprintf(string, "cq%d_complete", q_num);
				string += ETH_GSTRING_LEN;
				q_num++;
			}
		}

		q_num = 0;
		for (i = 0; i < bnad->num_rx; i++) {
			if (!bnad->rx_info[i].rx)
				continue;
			for (j = 0; j < bnad->num_rxp_per_rx; j++) {
				sprintf(string, "rxq%d_packets", q_num);
				string += ETH_GSTRING_LEN;
				sprintf(string, "rxq%d_bytes", q_num);
				string += ETH_GSTRING_LEN;
				sprintf(string, "rxq%d_packets_with_error",
								q_num);
				string += ETH_GSTRING_LEN;
				sprintf(string, "rxq%d_allocbuf_failed", q_num);
				string += ETH_GSTRING_LEN;
				sprintf(string, "rxq%d_producer_index", q_num);
				string += ETH_GSTRING_LEN;
				sprintf(string, "rxq%d_consumer_index", q_num);
				string += ETH_GSTRING_LEN;
				q_num++;
				if (bnad->rx_info[i].rx_ctrl[j].ccb &&
					bnad->rx_info[i].rx_ctrl[j].ccb->
					rcb[1] &&
					bnad->rx_info[i].rx_ctrl[j].ccb->
					rcb[1]->rxq) {
					sprintf(string, "rxq%d_packets", q_num);
					string += ETH_GSTRING_LEN;
					sprintf(string, "rxq%d_bytes", q_num);
					string += ETH_GSTRING_LEN;
					sprintf(string,
					"rxq%d_packets_with_error", q_num);
					string += ETH_GSTRING_LEN;
					sprintf(string, "rxq%d_allocbuf_failed",
								q_num);
					string += ETH_GSTRING_LEN;
					sprintf(string, "rxq%d_producer_index",
								q_num);
					string += ETH_GSTRING_LEN;
					sprintf(string, "rxq%d_consumer_index",
								q_num);
					string += ETH_GSTRING_LEN;
					q_num++;
				}
			}
		}

		q_num = 0;
		for (i = 0; i < bnad->num_tx; i++) {
			if (!bnad->tx_info[i].tx)
				continue;
			for (j = 0; j < bnad->num_txq_per_tx; j++) {
				sprintf(string, "txq%d_packets", q_num);
				string += ETH_GSTRING_LEN;
				sprintf(string, "txq%d_bytes", q_num);
				string += ETH_GSTRING_LEN;
				sprintf(string, "txq%d_producer_index", q_num);
				string += ETH_GSTRING_LEN;
				sprintf(string, "txq%d_consumer_index", q_num);
				string += ETH_GSTRING_LEN;
				sprintf(string, "txq%d_hw_consumer_index",
									q_num);
				string += ETH_GSTRING_LEN;
				q_num++;
			}
		}
		break;

	default:
		break;
	}

	bnad_conf_unlock();
}

static int
bnad_get_stats_count_locked(struct net_device *netdev)
{
	struct bnad_s *bnad = netdev_priv(netdev);
	int i, j, count = 0, rxf_active_num = 0, txf_active_num = 0;
	uint32_t bmap;

	bmap = bna_tx_rid_mask(&bnad->bna);
	for (i = 0; bmap; i++) {
		if (bmap & 1)
			txf_active_num++;
		bmap >>= 1;
	}
	bmap = bna_rx_rid_mask(&bnad->bna);
	for (i = 0; bmap; i++) {
		if (bmap & 1)
			rxf_active_num++;
		bmap >>= 1;
	}
	count = BNAD_ETHTOOL_STATS_NUM +
		txf_active_num * BNAD_NUM_TXF_COUNTERS +
		rxf_active_num * BNAD_NUM_RXF_COUNTERS;

	for (i = 0; i < bnad->num_rx; i++) {
		if (!bnad->rx_info[i].rx)
			continue;
		count += bnad->num_rxp_per_rx * BNAD_NUM_CQ_COUNTERS;
		count += bnad->num_rxp_per_rx * BNAD_NUM_RXQ_COUNTERS;
		for (j = 0; j < bnad->num_rxp_per_rx; j++)
			if (bnad->rx_info[i].rx_ctrl[j].ccb &&
				bnad->rx_info[i].rx_ctrl[j].ccb->rcb[1] &&
				bnad->rx_info[i].rx_ctrl[j].ccb->rcb[1]->rxq)
				count +=  BNAD_NUM_RXQ_COUNTERS;
	}

	for (i = 0; i < bnad->num_tx; i++) {
		if (!bnad->tx_info[i].tx)
			continue;
		count += bnad->num_txq_per_tx * BNAD_NUM_TXQ_COUNTERS;
	}
	return count;
}

static int
bnad_per_q_stats_fill(struct bnad_s *bnad, u64 *buf, int bi)
{
	int i, j;
	struct bna_rcb_s *rcb = NULL;
	struct bna_tcb_s *tcb = NULL;

	for (i = 0; i < bnad->num_rx; i++) {
		if (!bnad->rx_info[i].rx)
			continue;
		for (j = 0; j < bnad->num_rxp_per_rx; j++)
			if (bnad->rx_info[i].rx_ctrl[j].ccb &&
				bnad->rx_info[i].rx_ctrl[j].ccb->rcb[0] &&
				bnad->rx_info[i].rx_ctrl[j].ccb->rcb[0]->rxq) {
				buf[bi++] = bnad->rx_info[i].rx_ctrl[j].
						ccb->producer_index;
				buf[bi++] = 0; /* ccb->consumer_index */
				buf[bi++] = *(bnad->rx_info[i].rx_ctrl[j].
						ccb->hw_producer_index);


				buf[bi++] = bnad->rx_info[i].
						rx_ctrl[j].rx_intr_ctr;
				buf[bi++] = bnad->rx_info[i].
						rx_ctrl[j].rx_poll_ctr;
				buf[bi++] = bnad->rx_info[i].
						rx_ctrl[j].rx_schedule;
				buf[bi++] = bnad->rx_info[i].
						rx_ctrl[j].rx_keep_poll;
				buf[bi++] = bnad->rx_info[i].
						rx_ctrl[j].rx_complete;

			}
	}
	for (i = 0; i < bnad->num_rx; i++) {
		if (!bnad->rx_info[i].rx)
			continue;
		for (j = 0; j < bnad->num_rxp_per_rx; j++)
			if (bnad->rx_info[i].rx_ctrl[j].ccb) {
				if (bnad->rx_info[i].rx_ctrl[j].ccb->rcb[0] &&
					bnad->rx_info[i].rx_ctrl[j].ccb->
					rcb[0]->rxq) {
					rcb = bnad->rx_info[i].rx_ctrl[j].
							ccb->rcb[0];
					buf[bi++] = rcb->rxq->rx_packets;
					buf[bi++] = rcb->rxq->rx_bytes;
					buf[bi++] = rcb->rxq->
							rx_packets_with_error;
					buf[bi++] = rcb->rxq->
							rxbuf_alloc_failed;
					buf[bi++] = rcb->producer_index;
					buf[bi++] = rcb->consumer_index;
				}
				if (bnad->rx_info[i].rx_ctrl[j].ccb->rcb[1] &&
					bnad->rx_info[i].rx_ctrl[j].ccb->
					rcb[1]->rxq) {
					rcb = bnad->rx_info[i].rx_ctrl[j].
								ccb->rcb[1];
					buf[bi++] = rcb->rxq->rx_packets;
					buf[bi++] = rcb->rxq->rx_bytes;
					buf[bi++] = rcb->rxq->
							rx_packets_with_error;
					buf[bi++] = rcb->rxq->
							rxbuf_alloc_failed;
					buf[bi++] = rcb->producer_index;
					buf[bi++] = rcb->consumer_index;
				}
			}
	}

	for (i = 0; i < bnad->num_tx; i++) {
		if (!bnad->tx_info[i].tx)
			continue;
		for (j = 0; j < bnad->num_txq_per_tx; j++)
			if (bnad->tx_info[i].tcb[j] &&
				bnad->tx_info[i].tcb[j]->txq) {
				tcb = bnad->tx_info[i].tcb[j];
				buf[bi++] = tcb->txq->tx_packets;
				buf[bi++] = tcb->txq->tx_bytes;
				buf[bi++] = tcb->producer_index;
				buf[bi++] = tcb->consumer_index;
				buf[bi++] = *(tcb->hw_consumer_index);
			}
	}

	return bi;
}


static void
bnad_get_ethtool_stats(struct net_device *netdev, struct ethtool_stats *stats,
		       u64 *buf)
{
	struct bnad_s *bnad = netdev_priv(netdev);
	int i, j, bi;
	unsigned long *net_stats, flags;
	uint64_t *stats64;
	uint32_t bmap;

	bnad_conf_lock();
	if (bnad_get_stats_count_locked(netdev) != stats->n_stats) {
		bnad_conf_unlock();
		return;
	}

	/*
	 * Used bna_lock to sync reads from bna_stats, which is written
	 * under the same lock
	 */
	spin_lock_irqsave(&bnad->bna_lock, flags);
	bi = 0;
	memset(buf, 0, stats->n_stats * sizeof(u64));
	memset(&bnad->net_stats, 0, sizeof(struct net_device_stats));

	bnad_netdev_qstats_fill(bnad);
	bnad_netdev_hwstats_fill(bnad);

	/* Fill net_stats into ethtool buffers */
	net_stats = (unsigned long *)&bnad->net_stats;
	for (i = 0; i < sizeof(struct net_device_stats) / sizeof(unsigned long);
	     i++)
		buf[bi++] = net_stats[i];

	/* Get LRO stats from stack in driver stats structure */
	bnad_lro_stats_fill(bnad);

	/* Get netif_queue_stopped from stack */
	bnad->stats.drv_stats.netif_queue_stopped = netif_queue_stopped(netdev);

	/* Fill driver stats into ethtool buffers */
	stats64 = (uint64_t *)&bnad->stats.drv_stats;
	for (i = 0; i < sizeof(struct bnad_drv_stats_s) / sizeof(uint64_t); i++)
		buf[bi++] = stats64[i];

	/* Fill hardware stats excluding the rxf/txf into ethtool bufs */
	stats64 = (uint64_t *) &bnad->stats.bna_stats->hw_stats;
	for (i = 0;
	     i < offsetof(struct bfi_enet_stats, rxf_stats[0]) /
		sizeof(uint64_t);
	     i++)
		buf[bi++] = stats64[i];

	/* Fill txf stats into ethtool buffers */
	bmap = bna_tx_rid_mask(&bnad->bna);
	for (i = 0; bmap; i++) {
		if (bmap & 1) {
			stats64 = (uint64_t *)&bnad->stats.bna_stats->
						hw_stats.txf_stats[i];
			for (j = 0; j < sizeof(struct bfi_enet_stats_txf) /
					sizeof(uint64_t); j++)
				buf[bi++] = stats64[j];
		}
		bmap >>= 1;
	}

	/*  Fill rxf stats into ethtool buffers */
	bmap = bna_rx_rid_mask(&bnad->bna);
	for (i = 0; bmap; i++) {
		if (bmap & 1) {
			stats64 = (uint64_t *)&bnad->stats.bna_stats->
						hw_stats.rxf_stats[i];
			for (j = 0; j < sizeof(struct bfi_enet_stats_rxf) /
					sizeof(uint64_t); j++)
				buf[bi++] = stats64[j];
		}
		bmap >>= 1;
	}

	/* Fill per Q stats into ethtool buffers */
	bi = bnad_per_q_stats_fill(bnad, buf, bi);

	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	bnad_conf_unlock();
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 29)
static int
bnad_get_stats_count(struct net_device *netdev)
{
	struct bnad_s *bnad = netdev_priv(netdev);
	int count;

	bnad_conf_lock();
	count = bnad_get_stats_count_locked(netdev);
	bnad_conf_unlock();
	return count;
}
#else
static int
bnad_get_sset_count(struct net_device *netdev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return bnad_get_stats_count_locked(netdev);
	default:
		return -EOPNOTSUPP;
	}
}
#endif

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 31))
/*
 * The set_flags entry point is replaced by ndo_set_features in
 * 3.0.x based kernels.
 */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 0, 0))
static int bnad_set_flags(struct net_device *dev, u32 eflags)
{
#ifdef BNAD_LRO
	struct bnad_s *bnad = netdev_priv(dev);

	if (!netif_running(bnad->netdev))
		return 0;

	bnad_conf_lock();
	/* Turn on the LRO feature - if not enabled already */
	if ((eflags & ETH_FLAG_LRO) && (!(dev->features & NETIF_F_LRO))) {
		dev->features |= NETIF_F_LRO;
		bnad_lro_disable = 0;
		/* Destroy the rx object and re-initialize to enable lro */
		bnad_reinit_rx(bnad);
	} else if ((dev->features & NETIF_F_LRO) && !bnad_lro_disable) {
		/* Turn off the LRO feature - if enabled already */
		if (!(eflags & ETH_FLAG_LRO)) {
			dev->features &= ~NETIF_F_LRO;
			bnad_lro_disable = 1;
			/* Destroy the rx object and re-init to disable lro */
			bnad_reinit_rx(bnad);
		}
	}
	bnad_conf_unlock();
#endif
	return 0;
}
#endif

static u32
bnad_get_flash_partition_by_offset(struct bnad_s *bnad, u32 offset,
			u32 len, u32 *base_offset, u32 *is_base_off)
{
	bfa_flash_attr_t *flash_attr;
	struct bnad_ioctl_comp_s fcomp;
	u32 i, flash_part = 0, ret;
	unsigned long flags = 0;

	flash_attr = kzalloc(sizeof(bfa_flash_attr_t), GFP_KERNEL);
	if (!flash_attr)
		return -ENOMEM;

	fcomp.bnad = bnad;
	fcomp.comp_status = 0;

	init_completion(&fcomp.comp);
	spin_lock_irqsave(&bnad->bna_lock, flags);
	ret = bfa_flash_get_attr(&bnad->bna.flash, flash_attr,
				bnad_cb_completion, &fcomp);
	if (ret != BFA_STATUS_OK) {
		spin_unlock_irqrestore(&bnad->bna_lock, flags);
		kfree(flash_attr);
		goto out_err;
	}
	spin_unlock_irqrestore(&bnad->bna_lock, flags);
	wait_for_completion(&fcomp.comp);
	ret = fcomp.comp_status;

	/* Check for the flash type & base offset value */
	if (ret == BFA_STATUS_OK) {
		for (i = 0; i < flash_attr->npart; i++) {
			if ((offset + sizeof(u32)) >=
			     flash_attr->part[i].part_off &&
			   ((offset + len) <= (flash_attr->part[i].part_off +
			     flash_attr->part[i].part_size))) {
				flash_part = flash_attr->part[i].part_type;
				*base_offset = flash_attr->part[i].part_off;
				if (!offset || (offset + sizeof(u32)) ==
				    flash_attr->part[i].part_off)
					*is_base_off = 1;
				break;
			}
		}
	}
	kfree(flash_attr);
	return flash_part;
out_err:
	return -EINVAL;
}

static int
bnad_get_eeprom_len(struct net_device *netdev)
{
	return BFA_TOTAL_FLASH_SIZE;
}

static int
bnad_get_eeprom(struct net_device *netdev, struct ethtool_eeprom *eeprom,
		u8 *bytes)
{
	struct bnad_s *bnad = netdev_priv(netdev);
	struct bnad_ioctl_comp_s fcomp;
	u32 flash_part = 0, base_offset = 0, is_base_off = 0;
	unsigned long flags = 0;
	int ret = 0;

	/* Check if the flash read request is valid */
	if (eeprom->magic != (bnad->pcidev->vendor |
			     (bnad->pcidev->device << 16)))
		return -EFAULT;

	/* Query the flash partition based on the offset */
	flash_part = bnad_get_flash_partition_by_offset(bnad,
				eeprom->offset, eeprom->len,
				&base_offset, &is_base_off);
	if (flash_part <= 0)
		return -EFAULT;

	fcomp.bnad = bnad;
	fcomp.comp_status = 0;

	init_completion(&fcomp.comp);
	spin_lock_irqsave(&bnad->bna_lock, flags);
	ret = bfa_flash_read_part(&bnad->bna.flash, flash_part,
				bna_port_id_get(&bnad->bna), bytes, eeprom->len,
				eeprom->offset - base_offset,
				bnad_cb_completion, &fcomp, 0);
	if (ret != BFA_STATUS_OK) {
		spin_unlock_irqrestore(&bnad->bna_lock, flags);
		goto done;
	}

	spin_unlock_irqrestore(&bnad->bna_lock, flags);
	wait_for_completion(&fcomp.comp);
	ret = fcomp.comp_status;
done:
	return ret;
}

static int
bnad_set_eeprom(struct net_device *netdev, struct ethtool_eeprom *eeprom,
		u8 *bytes)
{
	struct bnad_s *bnad = netdev_priv(netdev);
	struct bnad_ioctl_comp_s fcomp;
	u32 flash_part = 0, base_offset = 0, is_base_off = 0;
	unsigned long flags = 0;
	int ret = 0;

	/* Check if the flash update request is valid */
	if (eeprom->magic != (bnad->pcidev->vendor |
			     (bnad->pcidev->device << 16)))
		return -EINVAL;

	/* Query the flash partition based on the offset */
	flash_part = bnad_get_flash_partition_by_offset(bnad,
				eeprom->offset, eeprom->len,
				&base_offset, &is_base_off);
	if (flash_part <= 0)
		return -EFAULT;

	/* Check if another flash partition write is happening */
	if ((bnad->cur_flash_part != flash_part) &&
	    (bnad->cur_flash_off != bnad->total_flashsz))
		return BFA_STATUS_DEVBUSY;

	/*
	 * Initial flash update request for this partition
	 * Allocate the total buffer and make necessary checks
	 * to prepare for flash update.
	 */
	if (is_base_off) {
		/* Initial 4 bytes hold the total flash update sz */
		bnad->total_flashsz = *(u32 *)bytes;
		bnad->flash_buffer = vmalloc(bnad->total_flashsz);
		if (!bnad->flash_buffer)
			return -ENOMEM;
		memset(bnad->flash_buffer, 0, bnad->total_flashsz);

		/* Copy the received flash buf into the temp buf */
		memcpy((char *)bnad->flash_buffer, (char *)bytes + sizeof(u32),
		       (eeprom->len - sizeof(u32)));

		/* Keep track of the current flash parition & offset info */
		bnad->cur_flash_part = flash_part;
		bnad->cur_flash_off = (eeprom->len - sizeof(u32));
	} else {
		if (!bnad->flash_buffer)
			return -EINVAL;

		if (bnad->cur_flash_off <= bnad->total_flashsz) {
			/* Copy the received flash buf into the temp buf */
			memcpy(((char *)bnad->flash_buffer +
			     bnad->cur_flash_off), (char *)bytes, eeprom->len);
		} else
			return -EINVAL;

		/* Increment the current flash offset */
		bnad->cur_flash_off += eeprom->len;
	}

	/* Do a flash update once we have the entire flash buffer */
	if (bnad->cur_flash_off == bnad->total_flashsz) {
		fcomp.bnad = bnad;
		fcomp.comp_status = 0;

		init_completion(&fcomp.comp);
		spin_lock_irqsave(&bnad->bna_lock, flags);
		ret = bfa_flash_update_part(&bnad->bna.flash,
					bnad->cur_flash_part,
					bna_port_id_get(&bnad->bna),
					bnad->flash_buffer,
					bnad->total_flashsz, 0,
					bnad_cb_completion, &fcomp, 0);
		if (ret != BFA_STATUS_OK) {
			spin_unlock_irqrestore(&bnad->bna_lock, flags);
			vfree(bnad->flash_buffer);
			bnad->flash_buffer = NULL;
			goto done;
		}

		spin_unlock_irqrestore(&bnad->bna_lock, flags);
		wait_for_completion(&fcomp.comp);
		ret = fcomp.comp_status;
		vfree(bnad->flash_buffer);
		bnad->flash_buffer = NULL;
	}
done:
	return ret;
}

static int
bnad_flash_device(struct net_device *netdev, struct ethtool_flash *eflash)
{
	struct bnad_s *bnad = netdev_priv(netdev);
	struct bnad_ioctl_comp_s fcomp;
	const struct firmware *fw;
	unsigned long flags = 0;
	int ret = 0;

	ret = request_firmware(&fw, eflash->data, &bnad->pcidev->dev);
	if (ret) {
		pr_err("Can't locate firmware %s\n", eflash->data);
		goto out;
	}

	fcomp.bnad = bnad;
	fcomp.comp_status = 0;

	init_completion(&fcomp.comp);
	spin_lock_irqsave(&bnad->bna_lock, flags);
	ret = bfa_flash_update_part(&bnad->bna.flash, BFA_FLASH_PART_FWIMG,
				bna_port_id_get(&bnad->bna),
				(u8 *)fw->data, fw->size, 0,
				bnad_cb_completion, &fcomp, 0);
	if (ret != BFA_STATUS_OK) {
		pr_err("BNA: Flash update failed with err: %d\n", ret);
		ret = -EIO;
		spin_unlock_irqrestore(&bnad->bna_lock, flags);
		goto out;
	}

	spin_unlock_irqrestore(&bnad->bna_lock, flags);
	wait_for_completion(&fcomp.comp);
	if (fcomp.comp_status != BFA_STATUS_OK) {
		ret = -EIO;
		pr_err("BNA: Firmware image update to flash failed with: %d\n",
			fcomp.comp_status);
	}
out:
	release_firmware(fw);
	return ret;
}
#endif

static struct ethtool_ops bnad_ethtool_ops = {
	.get_settings = bnad_get_settings,
	.set_settings = bnad_set_settings,
	.get_drvinfo = bnad_get_drvinfo,
	.get_wol = bnad_get_wol,
	.get_msglevel = bnad_get_msglevel,
	.set_msglevel = bnad_set_msglevel,
	.get_link = ethtool_op_get_link,
	.get_coalesce = bnad_get_coalesce,
	.set_coalesce = bnad_set_coalesce,
	.get_ringparam = bnad_get_ringparam,
	.set_ringparam = bnad_set_ringparam,
	.get_pauseparam = bnad_get_pauseparam,
	.set_pauseparam = bnad_set_pauseparam,
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 1, 0)
	.get_rx_csum = bnad_get_rx_csum,
	.set_rx_csum = bnad_set_rx_csum,
	.get_tx_csum = ethtool_op_get_tx_csum,
	.set_tx_csum = bnad_set_tx_csum,
	.get_sg = ethtool_op_get_sg,
	.set_sg = ethtool_op_set_sg,
#ifdef NETIF_F_TSO
	.get_tso = ethtool_op_get_tso,
	.set_tso = bnad_set_tso,
#endif
#endif
	.get_strings = bnad_get_strings,
	.get_ethtool_stats = bnad_get_ethtool_stats,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 29)
	.get_stats_count = bnad_get_stats_count,
#else
	.get_sset_count = bnad_get_sset_count,
#endif
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 31))
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 0, 0))
	.get_flags = ethtool_op_get_flags,
	.set_flags = bnad_set_flags,
#endif
	.get_eeprom_len = bnad_get_eeprom_len,
	.get_eeprom = bnad_get_eeprom,
	.set_eeprom = bnad_set_eeprom,
	.flash_device = bnad_flash_device,
#endif
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 5, 0))
	.get_ts_info = ethtool_op_get_ts_info
#endif
};

#if defined(RHEL_RELEASE_VERSION)
#if (RHEL_RELEASE_CODE >= RHEL_RELEASE_VERSION(6, 4)) && (!RHEL_RELEASE_CODE >= RHEL_RELEASE_VERSION(7, 0))
static const struct ethtool_ops_ext bnad_ethtool_ops_ext = {
	.size = sizeof(struct ethtool_ops_ext),
	.get_ts_info = ethtool_op_get_ts_info,
};
#endif
#endif

void
bnad_set_ethtool_ops(struct net_device *netdev)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 16, 0))
	SET_ETHTOOL_OPS(netdev, &bnad_ethtool_ops);
#else
	netdev->ethtool_ops = &bnad_ethtool_ops;
#endif
#if defined(RHEL_RELEASE_VERSION)
#if (RHEL_RELEASE_CODE >= RHEL_RELEASE_VERSION(6, 4)) && (!RHEL_RELEASE_CODE >= RHEL_RELEASE_VERSION(7, 0))
	set_ethtool_ops_ext(netdev, &bnad_ethtool_ops_ext);
#endif
#endif
}
