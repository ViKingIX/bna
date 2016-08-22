/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

/**
 * @file bfi_enet.h BNA Hardware and Firmware Interface
 */

/**
 *
 * Skipping statistics collection to avoid clutter.
 * Command is no longer needed:
 *	MTU
 * 	TxQ Stop
 * 	RxQ Stop
 *	RxF Enable/Disable
 *
 * HDS-off request is dynamic
 * keep structures as multiple of 32-bit fields for alignment.
 *
 * BEWARE:  All values must be written in big-endian.
 * FIXME: remove enet_id from msgq_mhdr and 'commonize'(error code)
 * enet commands
 */
#ifndef __BFI_ENET_H__
#define __BFI_ENET_H__

#include <defs/bfa_defs.h>
#include <bfi/bfi.h>
#include <defs/bfa_defs_cna.h>

#pragma pack(1)


/* TODO: consolidate all enet related  macros */
#define BFI_ENET_CFG_MAX		32	/* Max resources per PF */

#define BFI_ENET_TXQ_PRIO_MAX		8
#define BFI_ENET_RX_QSET_MAX		16
#define BFI_ENET_TXQ_WI_VECT_MAX	4

#define BFI_ENET_VLAN_ID_MAX		4096
#define BFI_ENET_VLAN_BLOCK_SIZE	512	/* in bits */
#define BFI_ENET_VLAN_BLOCKS_MAX					\
	(BFI_ENET_VLAN_ID_MAX / BFI_ENET_VLAN_BLOCK_SIZE)
#define BFI_ENET_VLAN_WORD_SIZE		32	/* in bits */
#define BFI_ENET_VLAN_WORDS_MAX						\
	(BFI_ENET_VLAN_BLOCK_SIZE / BFI_ENET_VLAN_WORD_SIZE)

#define BFI_ENET_RSS_RIT_MAX		128	/* entries */
#define BFI_ENET_RSS_KEY_LEN		10 	/* 32-bit words */

#define BFI_ENET_WOL_FRAME_ID_MAX	7
#define BFI_ENET_WOL_FRAME_LEN		128	/* bytes */
#define BFI_ENET_WOL_FRAME_MASK_LEN	(BFI_ENET_WOL_FRAME_LEN / 8)
#define BFI_ENET_WOL_MAGIC_ID_MAX	2
#define BFI_ENET_WOL_MAGIC_PASSWD_LEN	6	/* bytes */


/* TODO - move to bfi.h */
union bfi_addr_be_u {
	struct {
		uint32_t	addr_hi;	/* Most Significant 32-bits */
		uint32_t	addr_lo;	/* Least Significant 32-Bits */
	} a32;
};
typedef union bfi_addr_be_u bfi_addr_be_t;


/**
 *
 *	T X   Q U E U E   D E F I N E S
 *
 */
/* TxQ Vector (a.k.a. Tx-Buffer Descriptor) */
/* TxQ Entry Opcodes */
#define BFI_ENET_TXQ_WI_SEND 		(0x402)	/* Single Frame Transmission */
#define BFI_ENET_TXQ_WI_SEND_LSO 	(0x403)	/* Multi-Frame Transmission */
#define BFI_ENET_TXQ_WI_EXTENSION	(0x104)	/* Extension WI */
typedef uint16_t bfi_enet_txq_wi_opcode_t;


/* TxQ Entry Control Flags */
#define BFI_ENET_TXQ_WI_CF_FCOE_CRC  	(1 << 8)
#define BFI_ENET_TXQ_WI_CF_IPID_MODE 	(1 << 5)
#define BFI_ENET_TXQ_WI_CF_INS_PRIO  	(1 << 4)
#define BFI_ENET_TXQ_WI_CF_INS_VLAN  	(1 << 3)
#define BFI_ENET_TXQ_WI_CF_UDP_CKSUM 	(1 << 2)
#define BFI_ENET_TXQ_WI_CF_TCP_CKSUM 	(1 << 1)
#define BFI_ENET_TXQ_WI_CF_IP_CKSUM  	(1 << 0)
typedef uint16_t bfi_enet_txq_wi_ctrl_flag_t;

struct bfi_enet_txq_wi_base {
	uint8_t reserved;
	uint8_t num_vectors;	/* number of vectors present */
	bfi_enet_txq_wi_opcode_t opcode;
		/* BFI_ENET_TXQ_WI_SEND or BFI_ENET_TXQ_WI_SEND_LSO */
	bfi_enet_txq_wi_ctrl_flag_t  flags; 	/* OR of all the flags */
	uint16_t	l4_hdr_size_n_offset;
	uint16_t	vlan_tag;
	uint16_t	lso_mss;	/* Only 14 LSB are valid */
	uint32_t	frame_length;	/* Only 24 LSB are valid */
};


struct bfi_enet_txq_wi_ext {
	uint16_t	reserved;
	bfi_enet_txq_wi_opcode_t opcode;	/* BFI_ENET_TXQ_WI_EXTENSION */
	uint32_t	reserved2[3];
};


struct bfi_enet_txq_wi_vector  /* Tx Buffer Descriptor */
{
	uint16_t		reserved;
	uint16_t		length; /* Only 14 LSB are valid */
	union bfi_addr_be_u	addr;
};


/**
 *  TxQ Entry Structure
 *
 */
struct bfi_enet_txq_entry
{
	union {
		struct bfi_enet_txq_wi_base	base;
		struct bfi_enet_txq_wi_ext	ext;
	} wi;
	struct bfi_enet_txq_wi_vector vector[BFI_ENET_TXQ_WI_VECT_MAX];
};

#define wi_hdr  	wi.base
#define wi_ext_hdr  	wi.ext

#define BFI_ENET_TXQ_WI_L4_HDR_N_OFFSET(_hdr_size, _offset) \
		(((_hdr_size) << 10) | ((_offset) & 0x3FF))


/**
 *
 *   R X   Q U E U E   D E F I N E S
 *
 */
struct bfi_enet_rxq_entry
{
	union bfi_addr_be_u  rx_buffer;
};


/**
 *
 *   R X   C O M P L E T I O N   Q U E U E   D E F I N E S
 *
 */
/* CQ Entry Flags */
#define	BFI_ENET_CQ_EF_MAC_ERROR	(1 <<  0)
#define	BFI_ENET_CQ_EF_FCS_ERROR	(1 <<  1)
#define	BFI_ENET_CQ_EF_TOO_LONG 	(1 <<  2)
#define	BFI_ENET_CQ_EF_FC_CRC_OK	(1 <<  3)

#define	BFI_ENET_CQ_EF_RSVD1		(1 <<  4)
#define	BFI_ENET_CQ_EF_L4_CKSUM_OK	(1 <<  5)
#define	BFI_ENET_CQ_EF_L3_CKSUM_OK	(1 <<  6)
#define	BFI_ENET_CQ_EF_HDS_HEADER	(1 <<  7)

#define	BFI_ENET_CQ_EF_UDP		(1 <<  8)
#define	BFI_ENET_CQ_EF_TCP		(1 <<  9)
#define	BFI_ENET_CQ_EF_IP_OPTIONS	(1 << 10)
#define	BFI_ENET_CQ_EF_IPV6		(1 << 11)

#define	BFI_ENET_CQ_EF_IPV4		(1 << 12)
#define	BFI_ENET_CQ_EF_VLAN		(1 << 13)
#define	BFI_ENET_CQ_EF_RSS		(1 << 14)
#define	BFI_ENET_CQ_EF_RSVD2		(1 << 15)

#define	BFI_ENET_CQ_EF_MCAST_MATCH	(1 << 16)
#define	BFI_ENET_CQ_EF_MCAST		(1 << 17)
#define BFI_ENET_CQ_EF_BCAST		(1 << 18)
#define	BFI_ENET_CQ_EF_REMOTE		(1 << 19)

#define	BFI_ENET_CQ_EF_LOCAL		(1 << 20)
typedef uint32_t bfi_enet_cq_e_flag_t;


/* CQ Entry Structure */
struct bfi_enet_cq_entry
{
	bfi_enet_cq_e_flag_t flags;
	uint16_t	vlan_tag;
	uint16_t	length;
	uint32_t	rss_hash;
	uint8_t 	valid;
	uint8_t 	reserved1;
	uint8_t 	reserved2;
	uint8_t 	rxq_id;
};


/**
 *
 *   E N E T   C O N T R O L   P A T H   C O M M A N D S
 *
 */
struct bfi_enet_q {
	union bfi_addr_u	pg_tbl;
	union bfi_addr_u	first_entry;
	uint16_t		pages;	/* # of pages */
	uint16_t		page_sz;
};


struct bfi_enet_txq {
	struct bfi_enet_q	q;
	uint8_t			priority;
	uint8_t			rsvd[3];
};


struct bfi_enet_rxq {
	struct bfi_enet_q	q;
	uint16_t		rx_buffer_size;
	uint16_t		rsvd;
};


struct bfi_enet_cq {
	struct bfi_enet_q	q;
};


struct bfi_enet_ib_cfg {
	uint8_t		int_pkt_dma;
	uint8_t		int_enabled;
	uint8_t		int_pkt_enabled;
	uint8_t		continuous_coalescing;
	uint8_t		msix;
	uint8_t		rsvd[3];
	uint32_t	coalescing_timeout;
	uint32_t	inter_pkt_timeout;
	uint8_t		inter_pkt_count;
	uint8_t		rsvd1[3];
};


struct bfi_enet_ib {
	union bfi_addr_u	index_addr;
	union {
		uint16_t	msix_index;
		uint16_t 	intx_bitmask;
	} intr;
	uint16_t 		rsvd;
};


/**
 * @brief
 *	"enums" for all ENET command messages
 */
typedef enum {
	/* Rx Commands */
	BFI_ENET_H2I_RX_CFG_SET_REQ = 1,
	BFI_ENET_H2I_RX_CFG_CLR_REQ = 2,

	BFI_ENET_H2I_RIT_CFG_REQ = 3,
	BFI_ENET_H2I_RSS_CFG_REQ = 4,
	BFI_ENET_H2I_RSS_ENABLE_REQ = 5,
	BFI_ENET_H2I_RX_PROMISCUOUS_REQ = 6,
	BFI_ENET_H2I_RX_DEFAULT_REQ = 7,

	BFI_ENET_H2I_MAC_UCAST_SET_REQ = 8,
	BFI_ENET_H2I_MAC_UCAST_CLR_REQ = 9,
	BFI_ENET_H2I_MAC_UCAST_ADD_REQ = 10,
	BFI_ENET_H2I_MAC_UCAST_DEL_REQ = 11,

	BFI_ENET_H2I_MAC_MCAST_ADD_REQ = 12,
	BFI_ENET_H2I_MAC_MCAST_DEL_REQ = 13,
	BFI_ENET_H2I_MAC_MCAST_FILTER_REQ = 14,

	BFI_ENET_H2I_RX_VLAN_SET_REQ = 15,
	BFI_ENET_H2I_RX_VLAN_STRIP_ENABLE_REQ = 16,

	/* Tx Commands */
	BFI_ENET_H2I_TX_CFG_SET_REQ = 17,
	BFI_ENET_H2I_TX_CFG_CLR_REQ = 18,

	/* Port Commands */
	BFI_ENET_H2I_PORT_ADMIN_UP_REQ = 19,
	BFI_ENET_H2I_SET_PAUSE_REQ = 20,
	BFI_ENET_H2I_DIAG_LOOPBACK_REQ = 21,

	/* Get Attributes Command */
	BFI_ENET_H2I_GET_ATTR_REQ = 22,

	/*  Statistics Commands */
	BFI_ENET_H2I_STATS_GET_REQ = 23,
	BFI_ENET_H2I_STATS_CLR_REQ = 24,

	BFI_ENET_H2I_WOL_MAGIC_REQ = 25,
	BFI_ENET_H2I_WOL_FRAME_REQ = 26,

	BFI_ENET_H2I_MAX = 27,
} bfi_enet_h2i_msgs_t;

typedef enum {
	/* Rx Responses */
	BFI_ENET_I2H_RX_CFG_SET_RSP =
		BFA_I2HM(BFI_ENET_H2I_RX_CFG_SET_REQ),
	BFI_ENET_I2H_RX_CFG_CLR_RSP =
		BFA_I2HM(BFI_ENET_H2I_RX_CFG_CLR_REQ),

	BFI_ENET_I2H_RIT_CFG_RSP =
		BFA_I2HM(BFI_ENET_H2I_RIT_CFG_REQ),
	BFI_ENET_I2H_RSS_CFG_RSP =
		BFA_I2HM(BFI_ENET_H2I_RSS_CFG_REQ),
	BFI_ENET_I2H_RSS_ENABLE_RSP =
		BFA_I2HM(BFI_ENET_H2I_RSS_ENABLE_REQ),
	BFI_ENET_I2H_RX_PROMISCUOUS_RSP =
		BFA_I2HM(BFI_ENET_H2I_RX_PROMISCUOUS_REQ),
	BFI_ENET_I2H_RX_DEFAULT_RSP =
		BFA_I2HM(BFI_ENET_H2I_RX_DEFAULT_REQ),

	BFI_ENET_I2H_MAC_UCAST_SET_RSP =
		BFA_I2HM(BFI_ENET_H2I_MAC_UCAST_SET_REQ),
	BFI_ENET_I2H_MAC_UCAST_CLR_RSP =
		BFA_I2HM(BFI_ENET_H2I_MAC_UCAST_CLR_REQ),
	BFI_ENET_I2H_MAC_UCAST_ADD_RSP =
		BFA_I2HM(BFI_ENET_H2I_MAC_UCAST_ADD_REQ),
	BFI_ENET_I2H_MAC_UCAST_DEL_RSP =
		BFA_I2HM(BFI_ENET_H2I_MAC_UCAST_DEL_REQ),

	BFI_ENET_I2H_MAC_MCAST_ADD_RSP =
		BFA_I2HM(BFI_ENET_H2I_MAC_MCAST_ADD_REQ),
	BFI_ENET_I2H_MAC_MCAST_DEL_RSP =
		BFA_I2HM(BFI_ENET_H2I_MAC_MCAST_DEL_REQ),
	BFI_ENET_I2H_MAC_MCAST_FILTER_RSP =
		BFA_I2HM(BFI_ENET_H2I_MAC_MCAST_FILTER_REQ),

	BFI_ENET_I2H_RX_VLAN_SET_RSP =
		BFA_I2HM(BFI_ENET_H2I_RX_VLAN_SET_REQ),

	BFI_ENET_I2H_RX_VLAN_STRIP_ENABLE_RSP =
		BFA_I2HM(BFI_ENET_H2I_RX_VLAN_STRIP_ENABLE_REQ),

	/* Tx Responses */
	BFI_ENET_I2H_TX_CFG_SET_RSP =
		BFA_I2HM(BFI_ENET_H2I_TX_CFG_SET_REQ),
	BFI_ENET_I2H_TX_CFG_CLR_RSP =
		BFA_I2HM(BFI_ENET_H2I_TX_CFG_CLR_REQ),

	/* Port Responses */
	BFI_ENET_I2H_PORT_ADMIN_RSP =
		BFA_I2HM(BFI_ENET_H2I_PORT_ADMIN_UP_REQ),

	BFI_ENET_I2H_SET_PAUSE_RSP =
		BFA_I2HM(BFI_ENET_H2I_SET_PAUSE_REQ),
	BFI_ENET_I2H_DIAG_LOOPBACK_RSP =
		BFA_I2HM(BFI_ENET_H2I_DIAG_LOOPBACK_REQ),

	/*  Attributes Response */
	BFI_ENET_I2H_GET_ATTR_RSP =
		BFA_I2HM(BFI_ENET_H2I_GET_ATTR_REQ),

	/* Statistics Responses */
	BFI_ENET_I2H_STATS_GET_RSP =
		BFA_I2HM(BFI_ENET_H2I_STATS_GET_REQ),
	BFI_ENET_I2H_STATS_CLR_RSP =
		BFA_I2HM(BFI_ENET_H2I_STATS_CLR_REQ),

	BFI_ENET_I2H_WOL_MAGIC_RSP =
		BFA_I2HM(BFI_ENET_H2I_WOL_MAGIC_REQ),
	BFI_ENET_I2H_WOL_FRAME_RSP =
		BFA_I2HM(BFI_ENET_H2I_WOL_FRAME_REQ),

	/* AENs */

	BFI_ENET_I2H_LINK_DOWN_AEN = BFA_I2HM(BFI_ENET_H2I_MAX),
	BFI_ENET_I2H_LINK_UP_AEN = BFA_I2HM(BFI_ENET_H2I_MAX + 1),

	BFI_ENET_I2H_PORT_ENABLE_AEN = BFA_I2HM(BFI_ENET_H2I_MAX + 2),
	BFI_ENET_I2H_PORT_DISABLE_AEN = BFA_I2HM(BFI_ENET_H2I_MAX + 3),

	BFI_ENET_I2H_BW_UPDATE_AEN = BFA_I2HM(BFI_ENET_H2I_MAX + 4),
} bfi_enet_i2h_msgs_t;

/**
 * @brief
 *  The following error codes can be returned
 *  by the enet commands
 */
typedef enum {
	BFI_ENET_CMD_OK		= 0,
	BFI_ENET_CMD_FAIL	= 1,
	BFI_ENET_CMD_DUP_ENTRY	= 2,    /* !< Duplicate entry in CAM */
	BFI_ENET_CMD_CAM_FULL	= 3,    /* !< CAM is full */
	BFI_ENET_CMD_NOT_OWNER	= 4,    /* !< Not permitted, b'cos not owner */
	BFI_ENET_CMD_NOT_EXEC	= 5,    /* !< Was not sent to f/w at all */
	BFI_ENET_CMD_WAITING	= 6,    /* !< Waiting for completion (VMware) */
	BFI_ENET_CMD_PORT_DISABLED    = 7,    /* !< port in disabled state */
} bfi_enet_err_t;


/**
 *
 * Generic Request
 *
 * @brief bfi_enet_req is used by:
 *	BFI_ENET_H2I_RX_CFG_CLR_REQ
 *	BFI_ENET_H2I_TX_CFG_CLR_REQ
 */
struct bfi_enet_req {
	struct bfi_msgq_mhdr_s mh;
};


/**
 *
 * Enable/Disable Request
 *
 * @brief bfi_enet_enable_req is used by:
 *	BFI_ENET_H2I_RSS_ENABLE_REQ 	(enet_id must be zero)
 *	BFI_ENET_H2I_RX_PROMISCUOUS_REQ (enet_id must be zero)
 *	BFI_ENET_H2I_RX_DEFAULT_REQ 	(enet_id must be zero)
 *	BFI_ENET_H2I_RX_MAC_MCAST_FILTER_REQ
 *	BFI_ENET_H2I_PORT_ADMIN_UP_REQ	(enet_id must be zero)
 */
struct bfi_enet_enable_req {
	struct bfi_msgq_mhdr_s mh;
	uint8_t enable;			/* 1 = enable;  0 = disable */
	uint8_t	rsvd[3];
};


/**
 *
 * Generic Response
 */
struct bfi_enet_rsp {
	struct 		bfi_msgq_mhdr_s mh;
	uint8_t		error;		/*!< if error see cmd_offset */
	uint8_t		rsvd;
	uint16_t 	cmd_offset;	/*!< offset to invalid parameter */
	uint32_t	bw;
};


/**
 *
 * ENET AENs
 *
 * @brief bfi_enet_aen is used by:
 *	BFI_ENET_I2H_LINK_DOWN_AEN
 *	BFI_ENET_I2H_LINK_UP_AEN
 */
struct bfi_enet_aen {
	struct bfi_msgq_mhdr_s  mh;
	uint32_t	reason;
	uint8_t		cee_linkup;
	uint8_t		prio_map;	/*!< enet priority bit-map */
	uint8_t		iscsi_prio_map;
	uint8_t		rsvd[1];
	struct bfa_cee_dcbx_cfg_s dcb_cfg;
};

/**
 *
 * GLOBAL CONFIGURATION
 *
 */

/**
 * @brief bfi_enet_attr_req is used by:
 *	BFI_ENET_H2I_GET_ATTR_REQ
 */
struct bfi_enet_attr_req_s {
	struct bfi_msgq_mhdr_s  mh;
};

/**
 * @brief bfi_enet_attr_rsp is used by:
 *	BFI_ENET_I2H_GET_ATTR_RSP
 */
struct bfi_enet_attr_rsp {
	struct bfi_msgq_mhdr_s  mh;
	uint8_t		error;		/*!< if error see cmd_offset */
	uint8_t		rsvd;
	uint16_t 	cmd_offset;	/*!< offset to invalid parameter */
	uint32_t	max_cfg;
	uint32_t	max_ucmac;
	uint32_t	rit_size;
	uint32_t	bw;
	uint32_t	max_wol_magic;
	uint32_t	max_wol_frame;
};


/**
 *
 * Tx Configuration
 *
 * @brief bfi_enet_tx_cfg is used by:
 *	BFI_ENET_H2I_TX_CFG_SET_REQ
 */
typedef enum {
	BFI_ENET_TX_VLAN_NOP	= 0,
	BFI_ENET_TX_VLAN_INS	= 1,
	BFI_ENET_TX_VLAN_WI	= 2,
} bfi_enet_tx_vlan_mode_t;


struct bfi_enet_tx_cfg {
	uint8_t		vlan_mode;		/*!< processing mode */
	uint8_t		rsvd;
	uint16_t	vlan_id;
	uint8_t		admit_tagged_frame;
	uint8_t		apply_vlan_filter;
	uint8_t		add_to_vswitch;
	uint8_t		rsvd1[1];
};


struct bfi_enet_tx_cfg_req {
	struct bfi_msgq_mhdr_s mh;
	uint8_t	num_queues; /* # of Tx Queues */
	uint8_t rsvd[3];

	struct {
		struct bfi_enet_txq	q;
		struct bfi_enet_ib	ib;
	} q_cfg[BFI_ENET_TXQ_PRIO_MAX];

	struct bfi_enet_ib_cfg	ib_cfg;

	struct bfi_enet_tx_cfg	tx_cfg;
};


struct bfi_enet_tx_cfg_rsp {
	struct bfi_msgq_mhdr_s mh;
	uint8_t	error;
	uint8_t hw_id;			 /* For debugging */
	uint8_t	rsvd[2];
	struct {
		uint32_t	q_dbell;  /* PCI base address offset */
		uint32_t	i_dbell;  /* PCI base address offset */
		uint8_t		hw_qid;   /* For debugging */
		uint8_t		rsvd[3];
	} q_handles[BFI_ENET_TXQ_PRIO_MAX];
};


/**
 *
 * Rx Configuration
 *
 * @brief bfi_enet_rx_cfg is used by:
 *	BFI_ENET_H2I_RX_CFG_SET_REQ
 */
typedef enum {
	BFI_ENET_RXQ_SINGLE		= 1,
	BFI_ENET_RXQ_LARGE_SMALL	= 2,
	BFI_ENET_RXQ_HDS		= 3,
	BFI_ENET_RXQ_HDS_OPT_BASED	= 4,
} bfi_enet_rxq_type_t;


typedef enum {
	BFI_ENET_HDS_FORCED	= 0x01,
	BFI_ENET_HDS_IPV6_UDP 	= 0x02,
	BFI_ENET_HDS_IPV6_TCP 	= 0x04,
	BFI_ENET_HDS_IPV4_TCP 	= 0x08,
	BFI_ENET_HDS_IPV4_UDP 	= 0x10,
} bfi_enet_hds_type_t;


struct bfi_enet_rx_cfg {
	uint8_t		rxq_type;
	uint8_t		rsvd[1];
	uint16_t	frame_size;

	struct {
		uint8_t			max_header_size;
		uint8_t			force_offset;
		uint8_t			type;
		uint8_t			rsvd1;
	} hds;

	uint8_t		multi_buffer;
	uint8_t 	strip_vlan;
	uint8_t		drop_untagged;
	uint8_t		rsvd2;
};


/*
 * Multicast frames are received on the ql of q-set index zero.
 * On the completion queue.  RxQ ID = even is for large/data buffer queues
 * and RxQ ID = odd is for small/header buffer queues.
 */
struct bfi_enet_rx_cfg_req {
	struct bfi_msgq_mhdr_s mh;
	uint8_t num_queue_sets;  /* # of Rx Queue Sets */
	uint8_t	rsvd[3];

	struct {
		struct bfi_enet_rxq	ql;	/* large/data/single buffers */
		struct bfi_enet_rxq	qs;	/* small/header buffers */
		struct bfi_enet_cq	cq;
		struct bfi_enet_ib	ib;
	} q_cfg[BFI_ENET_RX_QSET_MAX];

	struct bfi_enet_ib_cfg	ib_cfg;

	struct bfi_enet_rx_cfg	rx_cfg;
};


struct bfi_enet_rx_cfg_rsp {
	struct bfi_msgq_mhdr_s mh;
	uint8_t error;
	uint8_t hw_id;			 /* For debugging */
	uint8_t	rsvd[2];
	struct {
		uint32_t	ql_dbell; /* PCI base address offset */
		uint32_t	qs_dbell; /* PCI base address offset */
		uint32_t	i_dbell;  /* PCI base address offset */
		uint8_t		hw_lqid;  /* For debugging */
		uint8_t		hw_sqid;  /* For debugging */
		uint8_t		hw_cqid;  /* For debugging */
		uint8_t		rsvd;
	} q_handles[BFI_ENET_RX_QSET_MAX];
};


/**
 *
 * RIT
 *
 * @brief bfi_enet_rit_req is used by:
 *	BFI_ENET_H2I_RIT_CFG_REQ
 */
struct bfi_enet_rit_req {
	struct bfi_msgq_mhdr_s mh;
	uint16_t size;			/* number of table-entries used */
	uint8_t	rsvd[2];
	uint8_t	table[BFI_ENET_RSS_RIT_MAX];
};


/**
 *
 * RSS
 *
 * @brief bfi_enet_rss_cfg_req is used by:
 *	BFI_ENET_H2I_RSS_CFG_REQ
 */
typedef enum {
	BFI_ENET_RSS_IPV6	= 0x01,
	BFI_ENET_RSS_IPV6_TCP	= 0x02,
	BFI_ENET_RSS_IPV4	= 0x04,
	BFI_ENET_RSS_IPV4_TCP	= 0x08
} bfi_enet_rss_type_t;


struct bfi_enet_rss_cfg {
	uint8_t		type;
	uint8_t 	mask;
	uint8_t 	rsvd[2];
	uint32_t	key[BFI_ENET_RSS_KEY_LEN];
};


struct bfi_enet_rss_cfg_req {
	struct bfi_msgq_mhdr_s mh;
	struct bfi_enet_rss_cfg cfg;
};


/**
 *
 * MAC Unicast
 *
 * @brief bfi_enet_rx_vlan_req is used by:
 *	BFI_ENET_H2I_MAC_UCAST_SET_REQ
 *	BFI_ENET_H2I_MAC_UCAST_CLR_REQ
 *	BFI_ENET_H2I_MAC_UCAST_ADD_REQ
 *	BFI_ENET_H2I_MAC_UCAST_DEL_REQ
 */
struct bfi_enet_ucast_req {
	struct bfi_msgq_mhdr_s	mh;
	mac_t			mac_addr;
	uint8_t			rsvd[2];
};


/**
 *
 * MAC Unicast + VLAN
 *
 */
struct bfi_enet_mac_n_vlan_req {  /* TODO */
	struct bfi_msgq_mhdr_s	mh;
	uint16_t		vlan_id;
	mac_t			mac_addr;
};


/**
 *
 * MAC Multicast
 *
 * @brief bfi_enet_mac_mfilter_add_req is used by:
 *	BFI_ENET_H2I_MAC_MCAST_ADD_REQ
 */
struct bfi_enet_mcast_add_req {
	struct bfi_msgq_mhdr_s	mh;
	mac_t		mac_addr;
	uint8_t		rsvd[2];
};

/**
 * @brief bfi_enet_mac_mfilter_add_rsp is used by:
 * BFI_ENET_I2H_MAC_MCAST_ADD_RSP
 */
struct bfi_enet_mcast_add_rsp {
	struct bfi_msgq_mhdr_s mh;
	uint8_t		error;
	uint8_t		rsvd;
	uint16_t	cmd_offset;
	uint16_t	handle;
	uint8_t		rsvd1[2];
};

/**
 *
 * @brief bfi_enet_mac_mfilter_del_req is used by:
 *	BFI_ENET_H2I_MAC_MCAST_DEL_REQ
 */
struct bfi_enet_mcast_del_req {
	struct bfi_msgq_mhdr_s	mh;
	uint16_t		handle;
	uint8_t			rsvd[2];
};


/**
 *
 * VLAN
 *
 * @brief bfi_enet_rx_vlan_req is used by:
 *	BFI_ENET_H2I_RX_VLAN_SET_REQ
 */
struct bfi_enet_rx_vlan_req {
	struct bfi_msgq_mhdr_s	mh;
	uint8_t		block_idx;
	uint8_t		rsvd[3];
	uint32_t	bit_mask[BFI_ENET_VLAN_WORDS_MAX];
};


/**
 *
 * PAUSE
 *
 * @brief bfi_enet_set_pause_req is used by:
 *	BFI_ENET_H2I_SET_PAUSE_REQ
 */
struct bfi_enet_set_pause_req {
	struct bfi_msgq_mhdr_s mh;
	uint8_t	rsvd[2];
	uint8_t	tx_pause;	/* 1 = enable;  0 = disable */
	uint8_t	rx_pause;	/* 1 = enable;  0 = disable */
};


/**
 *
 * DIAGNOSTICS
 *
 * @brief bfi_enet_diag_lb_req is used by:
 *      BFI_ENET_H2I_DIAG_LOOPBACK
 */
struct bfi_enet_diag_lb_req {
	struct bfi_msgq_mhdr_s mh;
	uint8_t	rsvd[2];
	uint8_t	mode;		/* cable or Serdes */
	uint8_t	enable;		/* 1 = enable;  0 = disable */
};

/**
 * @brief enum for Loopback opmodes
 */
enum {
	BFI_ENET_DIAG_LB_OPMODE_EXT = 0,
	BFI_ENET_DIAG_LB_OPMODE_CBL = 1,
};


/**
 *
 * STATISTICS
 *
 * @brief bfi_enet_stats_req is used by:
 *    BFI_ENET_H2I_STATS_GET_REQ
 *    BFI_ENET_I2H_STATS_CLR_REQ
 */
struct bfi_enet_stats_req {
	struct bfi_msgq_mhdr_s	mh;
	uint16_t	stats_mask;
	uint8_t		rsvd[2];
	uint32_t	rx_enet_mask;
	uint32_t	tx_enet_mask;
	union bfi_addr_u	host_buffer;
};

/**
 * @brief defines for "stats_mask" above.
 */
#define BFI_ENET_STATS_MAC    (1 << 0)    /* !< MAC Statistics */
#define BFI_ENET_STATS_BPC    (1 << 1)    /* !< Pause Stats from BPC */
#define BFI_ENET_STATS_RAD    (1 << 2)    /* !< Rx Admission Statistics */
#define BFI_ENET_STATS_RX_FC  (1 << 3)    /* !< Rx FC Stats from RxA */
#define BFI_ENET_STATS_TX_FC  (1 << 4)    /* !< Tx FC Stats from TxA */

#define BFI_ENET_STATS_ALL    0x1f

/* TxF Frame Statistics */
struct bfi_enet_stats_txf {
	uint64_t ucast_octets;
	uint64_t ucast;
	uint64_t ucast_vlan;

	uint64_t mcast_octets;
	uint64_t mcast;
	uint64_t mcast_vlan;

	uint64_t bcast_octets;
	uint64_t bcast;
	uint64_t bcast_vlan;

	uint64_t errors;
	uint64_t filter_vlan;      /* frames filtered due to VLAN */
	uint64_t filter_mac_sa;    /* frames filtered due to SA check */
};


/* RxF Frame Statistics */
struct bfi_enet_stats_rxf {
	uint64_t ucast_octets;
	uint64_t ucast;
	uint64_t ucast_vlan;

	uint64_t mcast_octets;
	uint64_t mcast;
	uint64_t mcast_vlan;

	uint64_t bcast_octets;
	uint64_t bcast;
	uint64_t bcast_vlan;
	uint64_t frame_drops;
};

/* Fixme combine fc_tx & fc_rx */
/* FC Tx Frame Statistics */
struct bfi_enet_stats_fc_tx {
	uint64_t txf_ucast_octets;
	uint64_t txf_ucast;
	uint64_t txf_ucast_vlan;

	uint64_t txf_mcast_octets;
	uint64_t txf_mcast;
	uint64_t txf_mcast_vlan;

	uint64_t txf_bcast_octets;
	uint64_t txf_bcast;
	uint64_t txf_bcast_vlan;

	uint64_t txf_parity_errors;
	uint64_t txf_timeout;
	uint64_t txf_fid_parity_errors;
};

/* FC Rx Frame Statistics */
struct bfi_enet_stats_fc_rx {
	uint64_t rxf_ucast_octets;
	uint64_t rxf_ucast;
	uint64_t rxf_ucast_vlan;

	uint64_t rxf_mcast_octets;
	uint64_t rxf_mcast;
	uint64_t rxf_mcast_vlan;

	uint64_t rxf_bcast_octets;
	uint64_t rxf_bcast;
	uint64_t rxf_bcast_vlan;
};


/* RAD Frame Statistics */
struct bfi_enet_stats_rad {
	uint64_t rx_frames;
	uint64_t rx_octets;
	uint64_t rx_vlan_frames;

	uint64_t rx_ucast;
	uint64_t rx_ucast_octets;
	uint64_t rx_ucast_vlan;

	uint64_t rx_mcast;
	uint64_t rx_mcast_octets;
	uint64_t rx_mcast_vlan;

	uint64_t rx_bcast;
	uint64_t rx_bcast_octets;
	uint64_t rx_bcast_vlan;

	uint64_t rx_drops;
};


/* BPC Tx Registers */
struct bfi_enet_stats_bpc {
	/* transmit stats */
	uint64_t tx_pause[8];
	uint64_t tx_zero_pause[8];	/*!< Pause cancellation */
	/*!<Pause initiation rather than retention */
	uint64_t tx_first_pause[8];

	/* receive stats */
	uint64_t rx_pause[8];
	uint64_t rx_zero_pause[8];	/*!< Pause cancellation */
	/*!<Pause initiation rather than retention */
	uint64_t rx_first_pause[8];
};


/* MAC Rx Statistics */
struct bfi_enet_stats_mac {
	uint64_t stats_clr_cnt;		/* times this stats cleared */

	uint64_t frame_64;		/* both rx and tx counter */
	uint64_t frame_65_127;		/* both rx and tx counter */
	uint64_t frame_128_255;		/* both rx and tx counter */
	uint64_t frame_256_511;		/* both rx and tx counter */
	uint64_t frame_512_1023;	/* both rx and tx counter */
	uint64_t frame_1024_1518;	/* both rx and tx counter */
	uint64_t frame_1519_1522;	/* both rx and tx counter */

	/* receive stats */
	uint64_t rx_bytes;
	uint64_t rx_packets;
	uint64_t rx_fcs_error;
	uint64_t rx_multicast;
	uint64_t rx_broadcast;
	uint64_t rx_control_frames;
	uint64_t rx_pause;
	uint64_t rx_unknown_opcode;
	uint64_t rx_alignment_error;
	uint64_t rx_frame_length_error;
	uint64_t rx_code_error;
	uint64_t rx_carrier_sense_error;
	uint64_t rx_undersize;
	uint64_t rx_oversize;
	uint64_t rx_fragments;
	uint64_t rx_jabber;
	uint64_t rx_drop;

	/* transmit stats */
	uint64_t tx_bytes;
	uint64_t tx_packets;
	uint64_t tx_multicast;
	uint64_t tx_broadcast;
	uint64_t tx_pause;
	uint64_t tx_deferral;
	uint64_t tx_excessive_deferral;
	uint64_t tx_single_collision;
	uint64_t tx_muliple_collision;
	uint64_t tx_late_collision;
	uint64_t tx_excessive_collision;
	uint64_t tx_total_collision;
	uint64_t tx_pause_honored;
	uint64_t tx_drop;
	uint64_t tx_jabber;
	uint64_t tx_fcs_error;
	uint64_t tx_control_frame;
	uint64_t tx_oversize;
	uint64_t tx_undersize;
	uint64_t tx_fragments;
};

/**
 * Complete statistics, DMAed from fw to host followed by
 * BFI_ENET_I2H_STATS_GET_RSP
 */
struct bfi_enet_stats {
	struct bfi_enet_stats_mac	mac_stats;
	struct bfi_enet_stats_bpc	bpc_stats;
	struct bfi_enet_stats_rad	rad_stats;
	struct bfi_enet_stats_rad	rlb_stats;
	struct bfi_enet_stats_fc_rx	fc_rx_stats;
	struct bfi_enet_stats_fc_tx	fc_tx_stats;
	struct bfi_enet_stats_rxf	rxf_stats[BFI_ENET_CFG_MAX];
	struct bfi_enet_stats_txf	txf_stats[BFI_ENET_CFG_MAX];
};

/**
 *
 * WOL
 *
 */
/* Generic WOL Frame */
struct bfi_enet_wol_frame_gen {
	struct bfi_msgq_mhdr_s mh;
	uint8_t	rsvd[2];
	uint8_t frame_id;
	uint8_t enable;
	uint8_t frame[BFI_ENET_WOL_FRAME_LEN];
	uint8_t mask[BFI_ENET_WOL_FRAME_MASK_LEN];
};


/* WOL Magic Frame */
struct bfi_enet_wol_magic_frame {
	struct bfi_msgq_mhdr_s mh;
	uint8_t	rsvd;
	uint8_t frame_id;
	uint8_t enable;
	uint8_t password_enable;
	mac_t	mac_addr;
	uint8_t password[BFI_ENET_WOL_MAGIC_PASSWD_LEN];
};

/* @brief message structures from firmware to host  */
union bfi_enet_i2h_msg_u {
	struct bfi_enet_rsp		enet_rsp;
	struct bfi_enet_attr_rsp	enet_attr_rsp;
	struct bfi_enet_tx_cfg_rsp	enet_tx_cfg_rsp;
	struct bfi_enet_rx_cfg_rsp	enet_rx_cfg_rsp;
	struct bfi_enet_mcast_add_rsp	enet_mcast_add_rsp;
};
typedef union bfi_enet_i2h_msg_u bfi_enet_i2h_msg_t;

/**
 * NBOOT
 */

typedef enum {
	BFI_NBOOT_LINK_DOWN	= 0,
	BFI_NBOOT_LINK_UP	= 1,
} bfi_nboot_link_state_t;

typedef enum {
	BFI_NBOOT_H2I_ENABLE_REQ		= 1,	/*!< nboot enable    */
	BFI_NBOOT_H2I_DISABLE_REQ		= 2,	/*!< nboot disable    */
	BFI_NBOOT_H2I_MAC_MCAST_ADD_REQ		= 3,	/*!< add mcast addr  */
	BFI_NBOOT_H2I_MAC_MCAST_DEL_REQ		= 4,	/*!< del mcast addr  */
	BFI_NBOOT_H2I_TX_REQ			= 5,	/*!< transmit packet */
	BFI_NBOOT_H2I_RX_REQ			= 6,	/*!< post Rx buffer  */
} bfi_nboot_h2i_t;

typedef enum {
	BFI_NBOOT_I2H_ENABLE_RSP		= BFA_I2HM(1),
	BFI_NBOOT_I2H_DISABLE_RSP		= BFA_I2HM(2),
	BFI_NBOOT_I2H_MAC_MCAST_ADD_RSP		= BFA_I2HM(3),
	BFI_NBOOT_I2H_MAC_MCAST_DEL_RSP		= BFA_I2HM(4),
	BFI_NBOOT_I2H_TX_RSP			= BFA_I2HM(5),
	BFI_NBOOT_I2H_RX_RSP			= BFA_I2HM(6),
	BFI_NBOOT_I2H_RX_AEN			= BFA_I2HM(7),
	BFI_NBOOT_I2H_LINK_AEN			= BFA_I2HM(8),
} bfi_nboot_i2h_t;

/**
 * Network boot enable request.
 */
typedef struct {
	bfi_mhdr_t		mh;	/*!< common msg header		*/
	bfi_alen_t		rxbuf;	/*!< Receive buffer		*/
	bfi_addr_t		txsge;	/*!< Tx SGE buffer address	*/
	mac_t			mac;	/*!< network boot client MAC	*/
	uint16_t		vlan;	/*!< network boot VLAN ID	*/
} bfi_nboot_enable_req_t;

/**
 * Network boot disable request.
 */
typedef struct {
	bfi_mhdr_t		mh;	/*!< common msg header		*/
} bfi_nboot_disable_req_t;

/**
 * Network boot add/del mcast addr request
 */
typedef struct {
	bfi_mhdr_t	mh;		/*!< common msg header		   */
	mac_t		mac;		/*!< network boot client mcast MAC */
	uint8_t		rsvd[2];
} bfi_nboot_mac_mcast_req_t;

/**
* Network boot: transmit request
 */
typedef struct {
	bfi_mhdr_t		mh;		/*!< common msg header	*/
	uint16_t		len;
	uint8_t			rsvd[2];
} bfi_nboot_tx_req_t;

/**
* Network boot: post rx buffer request
 */
typedef struct {
	bfi_mhdr_t		mh;		/*!< common msg header	*/
} bfi_nboot_rx_req_t;

/**
 * Network boot: Host to firmware message.
 */
typedef union {
	bfi_mhdr_t			mhdr;
	bfi_nboot_enable_req_t		enable;
	bfi_nboot_disable_req_t		disable;
	bfi_nboot_mac_mcast_req_t	mac_mcast_add;
	bfi_nboot_mac_mcast_req_t	mac_mcast_del;
	bfi_nboot_tx_req_t		tx;
	bfi_nboot_rx_req_t		rx;
} bfi_nboot_h2i_msg_t;

/**
 * @brief
 *	  Response header format used by all responses
 *	  For both responses and asynchronous notifications
 */
typedef struct  {
	struct bfi_mhdr_s mh;		/*!< common msg header */
	uint8_t		status;
	uint8_t		rsvd[3];
	uint32_t	tx_bytes;
	uint32_t	tx_packets;
	uint32_t	tx_bcast;
} bfi_nboot_rsp_t;

/**
 * @brief bfi_nboot_aen is used by:
 *  	BFI_NBOOT_I2H_LINK_AEN
 */
struct bfi_nboot_link_aen {
	struct bfi_mhdr_s mh;		/*!< common msg header */
	uint8_t		link_status;
	uint8_t		rsvd[3];
};

/**
 * @brief bfi_nboot_aen is used by:
 *  	BFI_NBOOT_I2H_RX_AEN
 */
struct bfi_nboot_rx_aen {
	struct bfi_mhdr_s mh;		/*!< common msg header */
	uint16_t	pktlen;
	uint8_t		rsvd[2];
};

/**
 * Network boot: Firmware to host message.
 */
typedef union {
	bfi_mbmsg_t			*msg;
	bfi_nboot_rsp_t			*rsp;
	struct bfi_nboot_link_aen	*link_aen;
	struct bfi_nboot_rx_aen		*rx_aen;
} bfi_nboot_i2h_msg_t;

/**
 * @brief
 * 	The following error codes can be returned
 *	by the mbox commands
 */
typedef enum {
	BFI_NBOOT_CMD_OK 		= 0,
	BFI_NBOOT_CMD_FAIL 		= 1,
	BFI_NBOOT_CMD_DUP_ENTRY		= 2,	/* !< Duplicate entry in CAM */
	BFI_NBOOT_CMD_NOT_ENABLED	= 3,	/* !< Nboot not enabled */
	BFI_NBOOT_CMD_NOT_EXEC		= 4,   	/* !< Not sent to f/w at all */
	BFI_NBOOT_CMD_WAITING		= 5,	/* !< Waiting for completion */
} bfi_nboot_mbox_err_t;


#pragma pack()

#endif  /* __BFI_ENET_H__ */
