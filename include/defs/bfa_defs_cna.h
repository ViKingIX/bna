/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BFA_DEFS_CNA_H__
#define __BFA_DEFS_CNA_H__

#include <protocol/bfa_fc.h>
#include <defs/bfa_defs.h>
#include <defs/bfa_defs_svc.h>
#include <cna/pstats/phyport_defs.h>

/**
 * @brief
 * FC physical port statistics.
 */
struct bfa_port_fc_stats_s {
	uint64_t	secs_reset;	/*!< Seconds since stats is reset */
	uint64_t	tx_frames;	/*!< Tx frames			*/
	uint64_t	tx_words;	/*!< Tx words			*/
	uint64_t	tx_lip;		/*!< Tx LIP			*/
	uint64_t	tx_lip_f7f7;	/*!< Tx LIP_F7F7		*/
	uint64_t	tx_lip_f8f7;	/*!< Tx LIP_F8F7		*/
	uint64_t	tx_arbf0;	/*!< Tx ARB F0			*/
	uint64_t	tx_nos;		/*!< Tx NOS			*/
	uint64_t	tx_ols;		/*!< Tx OLS			*/
	uint64_t	tx_lr;		/*!< Tx LR			*/
	uint64_t	tx_lrr;		/*!< Tx LRR			*/
	uint64_t	rx_frames;	/*!< Rx frames			*/
	uint64_t	rx_words;	/*!< Rx words			*/
	uint64_t	lip_count;	/*!< Rx LIP			*/
	uint64_t	rx_lip_f7f7;	/*!< Rx LIP_F7F7		*/
	uint64_t	rx_lip_f8f7;	/*!< Rx LIP_F8F7		*/
	uint64_t	rx_arbf0;	/*!< Rx ARB F0			*/
	uint64_t	nos_count;	/*!< Rx NOS			*/
	uint64_t	ols_count;	/*!< Rx OLS			*/
	uint64_t	lr_count;	/*!< Rx LR			*/
	uint64_t	lrr_count;	/*!< Rx LRR			*/
	uint64_t	invalid_crcs;	/*!< Rx CRC err frames		*/
	uint64_t	invalid_crc_gd_eof; /*!< Rx CRC err good EOF frames */
	uint64_t	undersized_frm; /*!< Rx undersized frames	*/
	uint64_t	oversized_frm;	/*!< Rx oversized frames	*/
	uint64_t	bad_eof_frm;	/*!< Rx frames with bad EOF	*/
	uint64_t	error_frames;	/*!< Errored frames		*/
	uint64_t	dropped_frames;	/*!< Dropped frames		*/
	uint64_t	link_failures;	/*!< Link Failure (LF) count	*/
	uint64_t	loss_of_syncs;	/*!< Loss of sync count		*/
	uint64_t	loss_of_signals; /*!< Loss of signal count	*/
	uint64_t	primseq_errs;	/*!< Primitive sequence protocol err. */
	uint64_t	bad_os_count;	/*!< Invalid ordered sets	*/
	uint64_t	err_enc_out;	/*!< Encoding err nonframe_8b10b */
	uint64_t	err_enc;	/*!< Encoding err frame_8b10b	*/
	uint64_t	bbcr_frames_lost; /*!< BBCR Frames Lost */
	uint64_t	bbcr_rrdys_lost; /*!< BBCR RRDYs Lost */
	uint64_t	bbcr_link_resets; /*!< BBCR Link Resets */
	uint64_t	bbcr_frame_lost_intrs; /*!< BBCR Frame loss intrs */
	uint64_t	bbcr_rrdy_lost_intrs; /*!< BBCR Rrdy loss intrs */
	uint64_t	loop_timeouts;	/*!< Loop timeouts		*/
};
typedef struct bfa_port_fc_stats_s bfa_port_fc_stats_t;

/**
 * @brief
 * Eth Physical Port statistics.
 */
struct bfa_port_eth_stats_s {
	uint64_t	secs_reset;	/*!< Seconds since stats is reset */
	uint64_t	frame_64;	/*!< Frames 64 bytes		*/
	uint64_t	frame_65_127;	/*!< Frames 65-127 bytes	*/
	uint64_t	frame_128_255;	/*!< Frames 128-255 bytes	*/
	uint64_t	frame_256_511;	/*!< Frames 256-511 bytes	*/
	uint64_t	frame_512_1023;	/*!< Frames 512-1023 bytes	*/
	uint64_t	frame_1024_1518; /*!< Frames 1024-1518 bytes	*/
	uint64_t	frame_1519_1522; /*!< Frames 1519-1522 bytes	*/
	uint64_t	tx_bytes;	/*!< Tx bytes			*/
	uint64_t	tx_packets;	 /*!< Tx packets		*/
	uint64_t	tx_mcast_packets; /*!< Tx multicast packets	*/
	uint64_t	tx_bcast_packets; /*!< Tx broadcast packets	*/
	uint64_t	tx_control_frame; /*!< Tx control frame		*/
	uint64_t	tx_drop;	/*!< Tx drops			*/
	uint64_t	tx_jabber;	/*!< Tx jabber			*/
	uint64_t	tx_fcs_error;	/*!< Tx FCS errors		*/
	uint64_t	tx_fragments;	/*!< Tx fragments		*/
	uint64_t	rx_bytes;	/*!< Rx bytes			*/
	uint64_t	rx_packets;	/*!< Rx packets			*/
	uint64_t	rx_mcast_packets; /*!< Rx multicast packets	*/
	uint64_t	rx_bcast_packets; /*!< Rx broadcast packets	*/
	uint64_t	rx_control_frames; /*!< Rx control frames	*/
	uint64_t	rx_unknown_opcode; /*!< Rx unknown opcode	*/
	uint64_t	rx_drop;	/*!< Rx drops			*/
	uint64_t	rx_jabber;	/*!< Rx jabber			*/
	uint64_t	rx_fcs_error;	/*!< Rx FCS errors		*/
	uint64_t	rx_alignment_error; /*!< Rx alignment errors	*/
	uint64_t	rx_frame_length_error; /*!< Rx frame len errors	*/
	uint64_t	rx_code_error;	/*!< Rx code errors		*/
	uint64_t	rx_fragments;	/*!< Rx fragments		*/
	uint64_t	rx_pause;	/*!< Rx pause			*/
	uint64_t	rx_zero_pause;	/*!< Rx zero pause		*/
	uint64_t	tx_pause;	/*!< Tx pause			*/
	uint64_t	tx_zero_pause;	/*!< Tx zero pause		*/
	uint64_t	rx_fcoe_pause;	/*!< Rx FCoE pause		*/
	uint64_t	rx_fcoe_zero_pause; /*!< Rx FCoE zero pause	*/
	uint64_t	tx_fcoe_pause;	/*!< Tx FCoE pause		*/
	uint64_t	tx_fcoe_zero_pause; /*!< Tx FCoE zero pause	*/
	uint64_t	rx_iscsi_pause;	/*!< Rx iSCSI pause		*/
	uint64_t	rx_iscsi_zero_pause; /*!< Rx iSCSI zero pause	*/
	uint64_t	tx_iscsi_pause;	/*!< Tx iSCSI pause		*/
	uint64_t	tx_iscsi_zero_pause; /*!< Tx iSCSI zero pause	*/
};
typedef struct bfa_port_eth_stats_s bfa_port_eth_stats_t;

/**
 * @brief
 *		Port statistics.
 */
union bfa_port_stats_u {
	struct bfa_port_fc_stats_s	fc;
	struct bfa_port_eth_stats_s	eth;
};
typedef union bfa_port_stats_u bfa_port_stats_t;


#pragma pack(1)

#define BFA_CEE_LLDP_MAX_STRING_LEN (128)
#define BFA_CEE_DCBX_MAX_PRIORITY	(8)
#define BFA_CEE_DCBX_MAX_PGID		(8)

typedef enum {
	/* 0 is a reserved value. */
	BFA_LLDP_CHASSIS_ID_CHASSIS_COMP = 1,
	BFA_LLDP_CHASSIS_ID_INTF_ALIAS = 2,
	BFA_LLDP_CHASSIS_ID_PORT_COMP = 3,
	BFA_LLDP_CHASSIS_ID_MAC_ADDR = 4,
	BFA_LLDP_CHASSIS_ID_NET_ADDR = 5,
	BFA_LLDP_CHASSIS_ID_INTF_NAME = 6,
	BFA_LLDP_CHASSIS_ID_LOCAL = 7,
	/* Values 8-255 are reserved */
} bfa_lldp_chassis_id_sub_type_t;

typedef enum {
	/* 0 is a reserved value. */
	BFA_LLDP_PORT_ID_INTF_ALIAS = 1,
	BFA_LLDP_PORT_ID_PORT_COMP = 2,
	BFA_LLDP_PORT_ID_MAC_ADDR = 3,
	BFA_LLDP_PORT_ID_NET_ADDR = 4,
	BFA_LLDP_PORT_ID_INTF_NAME = 5,
	BFA_LLDP_PORT_ID_AGENT_CIRCUIT_ID = 6,
	BFA_LLDP_PORT_ID_LOCAL = 7,
	/* Values 8-255 are reserved */
} bfa_lldp_port_id_sub_type_t;

#define BFA_CEE_LLDP_SYS_CAP_OTHER	0x0001
#define BFA_CEE_LLDP_SYS_CAP_REPEATER	0x0002
#define BFA_CEE_LLDP_SYS_CAP_MAC_BRIDGE	0x0004
#define BFA_CEE_LLDP_SYS_CAP_WLAN_AP	0x0008
#define BFA_CEE_LLDP_SYS_CAP_ROUTER	0x0010
#define BFA_CEE_LLDP_SYS_CAP_TELEPHONE	0x0020
#define BFA_CEE_LLDP_SYS_CAP_DOCSIS_CD	0x0040
#define BFA_CEE_LLDP_SYS_CAP_STATION	0x0080
#define BFA_CEE_LLDP_SYS_CAP_CVLAN	0x0100
#define BFA_CEE_LLDP_SYS_CAP_SVLAN	0x0200
#define BFA_CEE_LLDP_SYS_CAP_TPMR	0x0400


/* LLDP string type */
struct bfa_cee_lldp_str_s {
	uint8_t sub_type;
	uint8_t len;
	uint8_t rsvd[2];
	uint8_t value[BFA_CEE_LLDP_MAX_STRING_LEN];
};
typedef struct bfa_cee_lldp_str_s bfa_cee_lldp_str_t;


/* LLDP paramters */
struct bfa_cee_lldp_cfg_s {
	struct bfa_cee_lldp_str_s chassis_id;
	struct bfa_cee_lldp_str_s port_id;
	struct bfa_cee_lldp_str_s port_desc;
	struct bfa_cee_lldp_str_s sys_name;
	struct bfa_cee_lldp_str_s sys_desc;
	struct bfa_cee_lldp_str_s mgmt_addr;
	uint16_t time_to_live;
	uint16_t enabled_system_cap;
};
typedef struct bfa_cee_lldp_cfg_s bfa_cee_lldp_cfg_t;

enum bfa_cee_dcbx_version_e {
	DCBX_PROTOCOL_PRECEE	= 1,
	DCBX_PROTOCOL_CEE	= 2,
};
typedef enum bfa_cee_dcbx_version_e bfa_cee_dcbx_version_t;

enum bfa_cee_lls_e {
	/* LLS is down because the TLV not sent by the peer */
	CEE_LLS_DOWN_NO_TLV = 0,
	/* LLS is down as advertised by the peer */
	CEE_LLS_DOWN	= 1,
	CEE_LLS_UP	= 2,
};
typedef enum bfa_cee_lls_e bfa_cee_lls_t;

/* CEE/DCBX parameters */
struct bfa_cee_dcbx_cfg_s {
	uint8_t pgid[BFA_CEE_DCBX_MAX_PRIORITY];
	uint8_t pg_percentage[BFA_CEE_DCBX_MAX_PGID];
	uint8_t pfc_primap; /* bitmap of priorties with PFC enabled */
	uint8_t fcoe_primap; /* bitmap of priorities used for FcoE traffic */
	uint8_t iscsi_primap; /* bitmap of priorities used for iSCSI traffic */
	uint8_t dcbx_version; /* operating version:CEE or preCEE */
	uint8_t lls_fcoe; /* FCoE Logical Link Status */
	uint8_t lls_lan; /* LAN Logical Link Status */
	uint8_t rsvd[2];
};
typedef struct bfa_cee_dcbx_cfg_s bfa_cee_dcbx_cfg_t;

/* CEE status */
/* Making this to tri-state for the benefit of port list command */
enum bfa_cee_status_e {
	CEE_UP = 0,
	CEE_PHY_UP = 1,
	CEE_LOOPBACK = 2,
	CEE_PHY_DOWN = 3,
};
typedef enum bfa_cee_status_e bfa_cee_status_t;

/* CEE Query */
struct bfa_cee_attr_s {
	uint8_t	cee_status;
	uint8_t error_reason;
	struct bfa_cee_lldp_cfg_s lldp_remote;
	struct bfa_cee_dcbx_cfg_s dcbx_remote;
	mac_t src_mac;
	uint8_t link_speed;
	uint8_t nw_priority;
	uint8_t filler[2];
};
typedef struct bfa_cee_attr_s bfa_cee_attr_t;

/* LLDP/DCBX/CEE Statistics */
struct bfa_cee_stats_s {
	uint32_t	lldp_tx_frames;		/*!< LLDP Tx Frames */
	uint32_t	lldp_rx_frames;		/*!< LLDP Rx Frames */
	uint32_t	lldp_rx_frames_invalid;	/*!< LLDP Rx Frames invalid */
	uint32_t	lldp_rx_frames_new;	/*!< LLDP Rx Frames new */
	uint32_t	lldp_tlvs_unrecognized;	/*!< LLDP Rx unrecog. TLVs */
	uint32_t	lldp_rx_shutdown_tlvs;	/*!< LLDP Rx shutdown TLVs */
	uint32_t	lldp_info_aged_out;	/*!< LLDP remote info aged */
	uint32_t	dcbx_phylink_ups;	/*!< DCBX phy link ups */
	uint32_t	dcbx_phylink_downs;	/*!< DCBX phy link downs */
	uint32_t	dcbx_rx_tlvs;		/*!< DCBX Rx TLVs */
	uint32_t	dcbx_rx_tlvs_invalid;	/*!< DCBX Rx TLVs invalid */
	uint32_t	dcbx_control_tlv_error;	/*!< DCBX control TLV errors */
	uint32_t	dcbx_feature_tlv_error;	/*!< DCBX feature TLV errors */
	uint32_t	dcbx_cee_cfg_new;	/*!< DCBX new CEE cfg rcvd */
	uint32_t	cee_status_down;	/*!< DCB status down */
	uint32_t	cee_status_up;		/*!< DCB status up */
	uint32_t	cee_hw_cfg_changed;	/*!< DCB hw cfg changed */
	uint32_t	cee_rx_invalid_cfg;	/*!< DCB invalid cfg */
};
typedef struct bfa_cee_stats_s bfa_cee_stats_t;

#pragma pack()


#endif	/* __BFA_DEFS_CNA_H__ */
