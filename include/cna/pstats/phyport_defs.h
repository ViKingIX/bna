/*
 * Copyright (c) 2006-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved.
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __PHYPORT_DEFS_H__
#define __PHYPORT_DEFS_H__

#define BNA_TXF_ID_MAX  	64
#define BNA_RXF_ID_MAX  	64

/* Statistics */

/* TxF Frame Statistics */
struct bna_stats_txf
{
	uint64_t ucast_octets;	/* Txf ucast octets */
	uint64_t ucast;		/* Txf ucast packets */
	uint64_t ucast_vlan;	/* Txf ucast vlan */

	uint64_t mcast_octets;	/* Txf mcast octets */
	uint64_t mcast;		/* Txf mcast packets */
	uint64_t mcast_vlan;	/* Txf mcast vlan */
    
	uint64_t bcast_octets;	/* Txf bcast octets */
	uint64_t bcast;		/* Txf bcast packets */
	uint64_t bcast_vlan;	/* Txf bcast vlan */
    
	uint64_t errors;	/* Txf errors */
	uint64_t filter_vlan;      /* Txf VLAN filtered frames */
	uint64_t filter_mac_sa;    /* Txf SA check filtered frames */
};
typedef struct bna_stats_txf bna_stats_txf_t;

/* RxF Frame Statistics */
struct bna_stats_rxf
{
	uint64_t ucast_octets;	/* Rxf ucast octets */
	uint64_t ucast;		/* Rxf ucast packets */
	uint64_t ucast_vlan;	/* Rxf ucast vlan */
    
	uint64_t mcast_octets;	/* Rxf mcast octets */
	uint64_t mcast;		/* Rxf mcast packets */
	uint64_t mcast_vlan;	/* Rxf mcast vlan */
    
	uint64_t bcast_octets;	/* Rxf bcast octets */
	uint64_t bcast;		/* Rxf bcast packets */
	uint64_t bcast_vlan;	/* Rxf bcast vlan */
	uint64_t frame_drops;	/* Rxf frame drops */
};
typedef struct bna_stats_rxf bna_stats_rxf_t;


/* FC Tx Frame Statistics */
struct bna_stats_fc_tx
{
	uint64_t txf_ucast_octets;	/* txf_ucast_octets */
	uint64_t txf_ucast;		/* txf_ucast */
	uint64_t txf_ucast_vlan;	/* txf_ucast_vlan */

	uint64_t txf_mcast_octets;	/* txf_mcast_octets */
	uint64_t txf_mcast;		/* txf_mcast */
	uint64_t txf_mcast_vlan;	/* txf_mcast_vlan */
    
	uint64_t txf_bcast_octets;	/* txf_bcast_octets */
	uint64_t txf_bcast;		/* txf_bcast */
	uint64_t txf_bcast_vlan;	/* txf_bcast_vlan */
    
	uint64_t txf_parity_errors;	/* txf_parity_errors */
	uint64_t txf_timeout;		/* txf_timeout */
	uint64_t txf_fid_parity_errors;	/* txf_fid_parity_errors */
};
typedef struct bna_stats_fc_tx bna_stats_fc_tx_t;

/* FC Rx Frame Statistics */
struct bna_stats_fc_rx
{
	uint64_t rxf_ucast_octets;	/* rxf_ucast_octets */
	uint64_t rxf_ucast;		/* rxf_ucast */
	uint64_t rxf_ucast_vlan;	/* rxf_ucast_vlan */
    
	uint64_t rxf_mcast_octets;	/* rxf_mcast_octets */
	uint64_t rxf_mcast;		/* rxf_mcast */
	uint64_t rxf_mcast_vlan;	/* rxf_mcast_vlan */
    
	uint64_t rxf_bcast_octets;	/* rxf_bcast_octets */
	uint64_t rxf_bcast;		/* rxf_bcast */
	uint64_t rxf_bcast_vlan;	/* rxf_bcast_vlan */
};
typedef struct bna_stats_fc_rx bna_stats_fc_rx_t;

/* RAD Frame Statistics */
struct cna_stats_rad
{
	uint64_t rx_frames;		/* rx_frames */ 
	uint64_t rx_octets;		/* rx_octets */
	uint64_t rx_vlan_frames;	/* rx_vlan_frames */

	uint64_t rx_ucast;		/* rx_ucast */
	uint64_t rx_ucast_octets;	/* rx_ucast_octets */
	uint64_t rx_ucast_vlan;		/* rx_ucast_vlan */
    
	uint64_t rx_mcast;		/* rx_mcast */
	uint64_t rx_mcast_octets;	/* rx_mcast_octets */
	uint64_t rx_mcast_vlan;		/* rx_mcast_vlan */
    
	uint64_t rx_bcast;		/* rx_bcast */
	uint64_t rx_bcast_octets;	/* rx_bcast_octets */
	uint64_t rx_bcast_vlan;		/* rx_bcast_vlan */

	uint64_t rx_drops;		/* rx_drops */
};
typedef struct cna_stats_rad cna_stats_rad_t;

/* BPC Tx Registers */
struct cna_stats_bpc_tx
{
	uint64_t tx_pause[8];	/* tx_pause */
	uint64_t tx_zero_pause[8]; /*!< Pause cancellation */
	uint64_t tx_first_pause[8];/*!< Pause initiation rather than retention*/ 
};
typedef struct cna_stats_bpc_tx cna_stats_bpc_tx_t;


/* BPC Rx Registers */
struct cna_stats_bpc_rx
{
	uint64_t rx_pause[8]; /* rx_pause */
	uint64_t rx_zero_pause[8]; /*!< Pause cancellation */
	uint64_t rx_first_pause[8];/*!< Pause initiation rather than retention*/
};
typedef struct cna_stats_bpc_rx cna_stats_bpc_rx_t;


/* MAC Rx Statistics */
struct cna_stats_mac_rx
{
	uint64_t frame_64;		/* frame_64 */
	uint64_t frame_65_127;		/* frame_65_127 */
	uint64_t frame_128_255;		/* frame_128_255 */
	uint64_t frame_256_511;		/* frame_256_511 */
	uint64_t frame_512_1023;	/* frame_512_1023 */
	uint64_t frame_1024_1518;	/* frame_1024_1518 */
	uint64_t frame_1518_1522;	/* frame_1518_1522 */
	uint64_t rx_bytes;		/* rx_bytes */
	uint64_t rx_packets;		/* rx_packets */
	uint64_t rx_fcs_error;		/* rx_fcs_error */
	uint64_t rx_multicast;		/* rx_multicast */
	uint64_t rx_broadcast;		/* rx_broadcast */
	uint64_t rx_control_frames;	/* rx_control_frames */
	uint64_t rx_pause;		/* rx_pause */
	uint64_t rx_unknown_opcode;	/* rx_unknown_opcode */
	uint64_t rx_alignment_error;	/* rx_alignment_error */
	uint64_t rx_frame_length_error;	/* rx_frame_length_error */
	uint64_t rx_code_error;		/* rx_code_error */
	uint64_t rx_carrier_sense_error;/* rx_carrier_sense_error */
	uint64_t rx_undersize;		/* rx_undersize */
	uint64_t rx_oversize;		/* rx_oversize */
	uint64_t rx_fragments;		/* rx_fragments */
	uint64_t rx_jabber;		/* rx_jabber */
	uint64_t rx_drop;		/* rx_drop */
};
typedef struct cna_stats_mac_rx cna_stats_mac_rx_t;


/* MAC Tx Statistics */
struct cna_stats_mac_tx
{
	uint64_t tx_bytes;		/* tx_bytes */
	uint64_t tx_packets;		/* tx_packets */
	uint64_t tx_multicast;		/* tx_multicast */
	uint64_t tx_broadcast;		/* tx_broadcast */
	uint64_t tx_pause;		/* tx_pause */
	uint64_t tx_deferral;		/* tx_deferral */
	uint64_t tx_excessive_deferral;	/* tx_excessive_deferral */
	uint64_t tx_single_collision;	/* tx_single_collision */
	uint64_t tx_muliple_collision;	/* tx_muliple_collision */
	uint64_t tx_late_collision;	/* tx_late_collision */
	uint64_t tx_excessive_collision;/* tx_excessive_collision */
	uint64_t tx_total_collision;	/* tx_total_collision */
	uint64_t tx_pause_honored;	/* tx_pause_honored */
	uint64_t tx_drop;		/* tx_drop */
	uint64_t tx_jabber;		/* tx_jabber */
	uint64_t tx_fcs_error;		/* tx_fcs_error */
	uint64_t tx_control_frame;	/* tx_control_frame */
	uint64_t tx_oversize;		/* tx_oversize */
	uint64_t tx_undersize;		/* tx_undersize */
	uint64_t tx_fragments;		/* tx_fragments */
};
typedef struct cna_stats_mac_tx cna_stats_mac_tx_t;

/* Complete statistics */
struct bna_stats
{
	struct cna_stats_mac_rx	mac_rx_stats;
	struct cna_stats_bpc_rx	bpc_rx_stats;
	struct cna_stats_rad	rad_stats;
	struct bna_stats_fc_rx	fc_rx_stats;
	struct cna_stats_mac_tx	mac_tx_stats;
	struct cna_stats_bpc_tx	bpc_tx_stats;
	struct bna_stats_fc_tx	fc_tx_stats;
	struct bna_stats_rxf	rxf_stats[64];
	struct bna_stats_txf	txf_stats[64];
};
typedef struct bna_stats bna_stats_t;

struct bnad_vars {
	/* adapter variables */
	uint16_t miniport_state;
	uint16_t adapter_state;
	uint16_t mp_reconfig_state;
	uint16_t mp_init_state;
	uint32_t adapter_flag;

	/* tx_queue variables */
	uint16_t wi_count;
	uint16_t wi_outstanding;
	uint16_t vlan_id;
	uint8_t tx_can_send;
	uint8_t txq_state;
	uint64_t drv_tx_stats[16];

	/* rx variables */
	/* rx_dev */
	uint8_t rx_Q_set_count;
	uint8_t rx_state;
	uint32_t cpu_bitmap;

	struct _rx_q_set {
		uint8_t is_rss_enabled;
		uint8_t rx_can_receive;
		uint8_t state;
		uint32_t packet_filter;
		struct _rx_q {
			uint16_t rx_offset;
			uint8_t rx_q_id;
			uint8_t state;
			uint16_t rx_buffer_count;
			uint16_t rx_buffer_size;
			uint64_t drv_rx_stats[16];
		} rx_q[2];
	} rx_q_set[16];
};

#endif
