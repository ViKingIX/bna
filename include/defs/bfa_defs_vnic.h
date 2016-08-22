/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BFA_DEFS_VNIC_H__
#define __BFA_DEFS_VNIC_H__

#define BFA_VNIC_MAX_TXQ	8
#define BFA_VNIC_MAX_TXF	8

#define BFA_VNIC_MAX_RXQ	2
#define BFA_VNIC_MAX_RXP	16
#define BFA_VNIC_MAX_RXF	16
#define BFA_VNIC_MAX_UCMAC	64
#define BFA_VNIC_MAX_UCMAC_VLAN	64
#define BFA_VNIC_MAX_MCMAC	64
#define BFA_VNIC_MAX_RIT	128	/* BFI_ENET_RSS_RIT_MAX */

/**
 * Structures related to VNIC attributes
 */

enum bfa_vnic_offload_e {
	BFA_VNIC_TX_IPCSUM	= 0x1,
	BFA_VNIC_TX_TCPCSUM	= 0x2,
	BFA_VNIC_TX_UDPCSUM	= 0x4,
	BFA_VNIC_RX_IPCSUM	= 0x8,
	BFA_VNIC_RX_TCPCSUM	= 0x10,
	BFA_VNIC_RX_UDPCSUM	= 0x20,
	BFA_VNIC_LSO		= 0x40,
};

struct bfa_vnic_txq_info_s {
	uint32_t		id;
	uint32_t		hw_id;
	uint32_t		msix_vector;
	uint32_t		priority;
	uint32_t		q_depth;
	uint32_t		rsvd;
};

struct bfa_vnic_txf_info_s {
	uint32_t		id;
	uint32_t		hw_id;
	uint32_t		os_id;
	uint32_t		num_txq;
	struct bfa_vnic_txq_info_s txq_info[BFA_VNIC_MAX_TXQ];
};

struct bfa_vnic_rxq_info_s {
	uint32_t		id;
	uint32_t		hw_id;
	uint32_t		buf_size;
	uint32_t		q_depth;
};

struct bfa_vnic_rxp_info_s {
	uint32_t		cq_id;
	uint32_t		hw_cq_id;
	uint32_t		msix_vector;
	uint32_t		num_rxq;
	struct bfa_vnic_rxq_info_s rxq_info[BFA_VNIC_MAX_RXQ];
};

struct bfa_vnic_mac_vlan_s {
	mac_t		mac;
	uint32_t	vlan;
};

struct bfa_vnic_rxf_info_s {
	uint32_t		id;
	uint32_t		hw_id;
	uint32_t		os_id;
	uint32_t		num_ucmac;
	mac_t			ucmac_list[BFA_VNIC_MAX_UCMAC];
	uint32_t		num_ucmac_vlan;
	struct bfa_vnic_mac_vlan_s ucmac_vlan_list[BFA_VNIC_MAX_UCMAC_VLAN];
	uint32_t		num_mcmac;
	mac_t			mcmac_list[BFA_VNIC_MAX_MCMAC];
	uint8_t			promisc_status;
	uint8_t			default_status;
	uint8_t			vlan_filter_status;
	uint8_t			rss_status;
	uint32_t		num_rxp;
	uint32_t		rit_size;
	uint32_t		rsvd;
	uint8_t			rit[BFA_VNIC_MAX_RIT];
	struct bfa_vnic_rxp_info_s rxp_info[BFA_VNIC_MAX_RXP];
};

struct bfa_vnic_attr_s {
	uint16_t		mtu;
	uint16_t		iscsi_prio;
	uint16_t		def_nw_prio;
	uint16_t		num_txf;
	uint16_t		num_rxf;
	uint16_t		offloads;
	uint32_t		rsvd;
	struct bfa_vnic_txf_info_s txf_info[BFA_VNIC_MAX_TXF];
	struct bfa_vnic_rxf_info_s rxf_info[BFA_VNIC_MAX_RXF];
};


/**
 * Structures related to VNIC statistics
 */

struct bfa_vnic_stats_bpc_s {
	uint64_t		tx_pause[8];
	uint64_t		tx_zero_pause[8];
	uint64_t		tx_first_pause[8];

	uint64_t		rx_pause[8];
	uint64_t		rx_zero_pause[8];
	uint64_t		rx_first_pause[8];
};
typedef struct bfa_vnic_stats_bpc_s bfa_vnic_stats_bpc_t;

struct bfa_vnic_stats_rad_s {
	uint64_t		rx_frames;	/* Rx frames */
	uint64_t		rx_octets;	/* Rx octets */
	uint64_t		rx_vlan_frames;	/* Rx vlan frames */

	uint64_t		rx_ucast;	/* Rx ucast */
	uint64_t		rx_ucast_octets; /* Rx ucast_octets */
	uint64_t		rx_ucast_vlan;	/* Rx ucast vlan */

	uint64_t		rx_mcast;	/* Rx mcast */
	uint64_t		rx_mcast_octets; /* Rx mcast_octets */
	uint64_t		rx_mcast_vlan;	/* Rx mcast vlan */

	uint64_t		rx_bcast;	/* Rx bcast */
	uint64_t		rx_bcast_octets; /* Rx bcast_octets */
	uint64_t		rx_bcast_vlan;	/* Rx bcast vlan */

	uint64_t		rx_drops;	/* Rx drops */
};
typedef struct bfa_vnic_stats_rad_s bfa_vnic_stats_rad_t;

struct bfa_vnic_stats_txq_s {
	uint64_t		id;		/* Tx Queue */

	uint64_t		tx_stop;	/* Tx stops */
	uint64_t		tx_resume;	/* Tx resumes */

	uint64_t		lso4;		/* LSO4 packets */
	uint64_t		lso6;		/* LSO6 packets */
	uint64_t		lso_errors;	/* LSO packets */

	uint64_t		tx_bytes;	/* Tx bytes */
	uint64_t		tx_packets;	/* Tx packets */
	uint64_t		ip_cso;		/* IP4 cso packets */
	uint64_t		tcp_cso;	/* TCP cso packets */
	uint64_t		udp_cso;	/* UDP cso packets */
	uint64_t		cso_errors;	/* CSO errors */

	uint64_t		out_of_wi;	/* Out of wi */
	uint64_t		dma_map_errors;	/* DMA map errors */
	uint64_t		prod_idx;	/* Producer index */
	uint64_t		cons_idx;	/* Consumer index */
	uint64_t		unmap_prod_idx;	/* Unmapped producer index */
	uint64_t		unmap_cons_idx;	/* Unmapped consumer index */
	uint64_t		hw_cons_idx;	/* HW consumer index */
};
typedef struct bfa_vnic_stats_txq_s bfa_vnic_stats_txq_t;

struct bfa_vnic_stats_txf_s {
	uint64_t		id;		/* Tx Function */

	uint64_t		ucast_octets;	/* Ucast bytes */
	uint64_t		ucast;		/* Ucast packets */
	uint64_t		ucast_vlan;	/* Ucast vlan */

	uint64_t		mcast_octets;	/* Mcast bytes */
	uint64_t		mcast;		/* Mcast packets */
	uint64_t		mcast_vlan;	/* Mcast vlan */

	uint64_t		bcast_octets;	/* Bcast bytes */
	uint64_t		bcast;		/* Bcast packets */
	uint64_t		bcast_vlan;	/* Bcast vlan */

	uint64_t		errors;		/* Errors */
	uint64_t		filter_vlan;	/* Filter vlan */
	uint64_t		filter_mac_sa;	/* Filter MAC */

	uint64_t		num_txq;	/* Tx queue count */
};
typedef struct bfa_vnic_stats_txf_s bfa_vnic_stats_txf_t;

struct bfa_vnic_stats_rxq_s {
	uint64_t		id;		/* Rx Queue */

	uint64_t		rx_cleanup;	/* Rx cleanups */
	uint64_t		rx_post;	/* Rx posts */
	uint64_t		rx_schedule;	/* Rx schedules */

	uint64_t		low_buf;	/* Rx low buf count */
	uint64_t		buf_alloc_fail;	/* Rx alloc failures */

	uint64_t		rx_bytes;	/* Rx bytes */
	uint64_t		rx_packets;	/* Rx packets */
	uint64_t		rx_mac_err;	/* Rx mac errors */
	uint64_t		rx_csum_err;	/* Rx checksum errors */
	uint64_t		lro;		/* Rx lro */
	uint64_t		lro_flush;	/* Rx lro flush */

	uint64_t		prod_idx;	/* Producer index */
	uint64_t		cons_idx;	/* Consumer index */
};
typedef struct bfa_vnic_stats_rxq_s bfa_vnic_stats_rxq_t;

struct bfa_vnic_stats_rxp_s {
	uint64_t		id;		/* Rx Path */

	uint64_t		cq_prod_idx;	/* CQ producer index */
	uint64_t		cq_hw_idx;	/* CQ hw index */
	uint64_t		num_rxq;	/* Rx queue count */
	struct bfa_vnic_stats_rxq_s rxq_stats[2];
};
typedef struct bfa_vnic_stats_rxp_s bfa_vnic_stats_rxp_t;


struct bfa_vnic_stats_rxf_s {
	uint64_t		id;		/* Rx Function */

	uint64_t		ucast_octets;	/* Ucast bytes */
	uint64_t		ucast;		/* Ucast packets */
	uint64_t		ucast_vlan;	/* Ucast vlan */

	uint64_t		mcast_octets;	/* Mcast bytes */
	uint64_t		mcast;		/* Mcast packets */
	uint64_t		mcast_vlan;	/* Mcast vlan */

	uint64_t		bcast_octets;	/* Bcast bytes */
	uint64_t		bcast;		/* Bcast packets */
	uint64_t		bcast_vlan;	/* Bcast vlan */
	uint64_t		frame_drops;	/* Frame drops */

	uint64_t		num_rxp;	/* Rx path count */
};
typedef struct bfa_vnic_stats_rxf_s bfa_vnic_stats_rxf_t;

struct bfa_vnic_drvstats_s {
	uint64_t	tx_out_of_wis_count; /* Tx wi waitq count */
	uint64_t	tx_ctxt_waitq_count; /* Tx ctxt waitq count */
	uint64_t	tx_max_nbs_per_nbl; /* Max nbls per nbl */
};
typedef struct bfa_vnic_drvstats_s bfa_vnic_drvstats_t;

struct bfa_vnic_stats_s {
	uint64_t		link_toggle;	/* Link toggle count */
	uint64_t		cee_toggle;	/* CEE toggle count */
	struct bfa_vnic_stats_bpc_s bpc_stats;	/* BPC stats */
	struct bfa_vnic_stats_rad_s rad_stats;	/* RAD stats */
	struct bfa_vnic_stats_rad_s rlb_stats;	/* RLB stats */
	uint64_t		num_txf;	/* Tx function count */
	struct bfa_vnic_stats_txf_s txf_stats[8];
	struct bfa_vnic_stats_txq_s txq_stats[8];
	uint64_t		num_rxf;	/* Rx function count */
	struct bfa_vnic_stats_rxf_s rxf_stats[16];
	struct bfa_vnic_stats_rxp_s rxp_stats[16];
	struct bfa_vnic_drvstats_s drv_stats;
};
typedef struct bfa_vnic_stats_s bfa_vnic_stats_t;

#endif /* __BFA_DEFS_VNIC_H__ */
