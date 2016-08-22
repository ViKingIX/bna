/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BNA_TYPES_H__
#define __BNA_TYPES_H__

#include "cna_os.h"
#include "bna_hw.h"
#include <cna/bfa_port.h>
#include <cna/bfa_sfp.h>
#include <cna/bfa_flash.h>
#include <cna/bfa_phy.h>
#include <cna/bfa_fru.h>
#include <cna/bfa_msgq.h>
#include <cna/bfa_edma.h>



/**
 *
 * Forward declarations
 *
 */



struct bna_mcam_handle_s;
struct bna_txq_s;
struct bna_tx_s;
struct bna_rxq_s;
struct bna_cq_s;
struct bna_rx_s;
struct bna_rxf_s;
struct bna_enet_s;
struct bna_s;
struct bnad_s;



/**
 *
 * Enums, primitive data types
 *
 */



enum bna_status_e {
	BNA_STATUS_T_DISABLED		= 0,
	BNA_STATUS_T_ENABLED		= 1
};

enum bna_cleanup_type_e {
	BNA_HARD_CLEANUP 		= 0,
	BNA_SOFT_CLEANUP 		= 1
};

enum bna_cb_status_e {
	BNA_CB_SUCCESS 			= 0,
	BNA_CB_FAIL			= 1,
	BNA_CB_INTERRUPT		= 2,
	BNA_CB_BUSY			= 3,
	BNA_CB_INVALID_MAC		= 4,
	BNA_CB_MCAST_LIST_FULL		= 5,
	BNA_CB_UCAST_CAM_FULL		= 6,
	BNA_CB_WAITING			= 7,
	BNA_CB_NOT_EXEC			= 8
};

enum bna_res_type_e {
	BNA_RES_T_MEM			= 1,
	BNA_RES_T_INTR			= 2
};

enum bna_mem_type_e {
	BNA_MEM_T_KVA 			= 1,
	BNA_MEM_T_DMA 			= 2
};

enum bna_intr_type_e {
	BNA_INTR_T_INTX			= 1,
	BNA_INTR_T_MSIX			= 2
};

enum bna_res_req_type_e {
	BNA_RES_MEM_T_COM 		= 0,
	BNA_RES_MEM_T_ATTR 		= 1,
	BNA_RES_MEM_T_FWTRC 		= 2,
	BNA_RES_MEM_T_STATS 		= 3,
	BNA_RES_T_MAX
};

enum bna_mod_res_req_type_e {
	BNA_MOD_RES_MEM_T_TX_ARRAY	= 0,
	BNA_MOD_RES_MEM_T_TXQ_ARRAY	= 1,
	BNA_MOD_RES_MEM_T_RX_ARRAY	= 2,
	BNA_MOD_RES_MEM_T_RXP_ARRAY	= 3,
	BNA_MOD_RES_MEM_T_RXQ_ARRAY	= 4,
	BNA_MOD_RES_MEM_T_UCMAC_ARRAY	= 5,
	BNA_MOD_RES_MEM_T_MCMAC_ARRAY	= 6,
	BNA_MOD_RES_MEM_T_MCHANDLE_ARRAY = 7,
	BNA_MOD_RES_MEM_T_WOLMAGIC_ARRAY = 8,
	BNA_MOD_RES_MEM_T_WOLFRAME_ARRAY = 9,
	BNA_MOD_RES_T_MAX
};

enum bna_tx_res_req_type_e {
	BNA_TX_RES_MEM_T_TCB		= 0,
	BNA_TX_RES_MEM_T_UNMAPQ		= 1,
	BNA_TX_RES_MEM_T_QPT 		= 2,
	BNA_TX_RES_MEM_T_SWQPT		= 3,
	BNA_TX_RES_MEM_T_PAGE 		= 4,
	BNA_TX_RES_MEM_T_IBIDX		= 5,
	BNA_TX_RES_INTR_T_TXCMPL 	= 6,
	BNA_TX_RES_T_MAX,
};

enum bna_rx_mem_type_e {
	BNA_RX_RES_MEM_T_CCB		= 0,
	BNA_RX_RES_MEM_T_RCB		= 1,
	BNA_RX_RES_MEM_T_UNMAPHQ	= 2,
	BNA_RX_RES_MEM_T_UNMAPDQ	= 3,
	BNA_RX_RES_MEM_T_CQPT		= 4,
	BNA_RX_RES_MEM_T_CSWQPT		= 5,
	BNA_RX_RES_MEM_T_CQPT_PAGE	= 6,
	BNA_RX_RES_MEM_T_HQPT		= 7,
	BNA_RX_RES_MEM_T_DQPT		= 8,
	BNA_RX_RES_MEM_T_HSWQPT		= 9,
	BNA_RX_RES_MEM_T_DSWQPT		= 10,
	BNA_RX_RES_MEM_T_DPAGE		= 11,
	BNA_RX_RES_MEM_T_HPAGE		= 12,
	BNA_RX_RES_MEM_T_IBIDX		= 13,
	BNA_RX_RES_MEM_T_RIT		= 14,
	BNA_RX_RES_T_INTR		= 15,
	BNA_RX_RES_T_MAX		= 16
};

enum bna_link_status_e {
	BNA_LINK_DOWN			= 0,
	BNA_LINK_UP			= 1,
	BNA_CEE_UP 			= 2
};

enum bna_ethport_flags_e {
	BNA_ETHPORT_F_ADMIN_UP 		= 1,
	BNA_ETHPORT_F_PORT_ENABLED	= 2,
	BNA_ETHPORT_F_RX_STARTED	= 4,
};

enum bna_enet_type_e {
	BNA_ENET_T_REGULAR		= 0,
	BNA_ENET_T_LOOPBACK_INTERNAL	= 1,
	BNA_ENET_T_LOOPBACK_EXTERNAL	= 2,
};

enum bna_enet_flags_e {
	BNA_ENET_F_IOCETH_READY		= 1,
	BNA_ENET_F_ENABLED		= 2,
	BNA_ENET_F_PAUSE_CHANGED 	= 4,
	BNA_ENET_F_MTU_CHANGED		= 8
};

enum bna_tx_type_e {
	BNA_TX_T_REGULAR		= 0,
	BNA_TX_T_LOOPBACK		= 1,
};

enum bna_tx_flags_e {
	BNA_TX_F_ENET_STARTED		= 1,
	BNA_TX_F_ENABLED		= 2,
	BNA_TX_F_PRIO_CHANGED		= 4,
	BNA_TX_F_BW_UPDATED		= 8,
};

enum bna_tx_mod_flags_e {
	BNA_TX_MOD_F_ENET_STARTED	= 1,
	BNA_TX_MOD_F_ENET_LOOPBACK	= 2,
};

enum bna_rx_type_e {
	BNA_RX_T_REGULAR		= 0,
	BNA_RX_T_LOOPBACK		= 1,
};

enum bna_rxp_type_e {
	BNA_RXP_SINGLE 			= 1,
	BNA_RXP_SLR 			= 2,
	BNA_RXP_HDS 			= 3
};

enum bna_rxmode_e {
	BNA_RXMODE_PROMISC 		= 1,
	BNA_RXMODE_DEFAULT 		= 2,
	BNA_RXMODE_ALLMULTI 		= 4
};

enum bna_rx_flags_e {
	BNA_RX_F_ENET_STARTED		= 1,
	BNA_RX_F_ENABLED		= 2,
};

enum bna_rx_mod_flags_e {
	BNA_RX_MOD_F_ENET_STARTED	= 1,
	BNA_RX_MOD_F_ENET_LOOPBACK	= 2,
};

enum bna_rxf_flags_e {
	BNA_RXF_F_PAUSED		= 1,
};

enum bna_rss_flags_e {
	BNA_RSS_F_RIT_PENDING		= 1,
	BNA_RSS_F_CFG_PENDING		= 2,
	BNA_RSS_F_STATUS_PENDING	= 4,
};

enum bna_mod_flags_e {
	BNA_MOD_F_INIT_DONE		= 1,
};

enum bna_pkt_rates_e {
	BNA_PKT_RATE_10K		= 10000,
	BNA_PKT_RATE_20K		= 20000,
	BNA_PKT_RATE_30K		= 30000,
	BNA_PKT_RATE_40K		= 40000,
	BNA_PKT_RATE_50K		= 50000,
	BNA_PKT_RATE_60K		= 60000,
	BNA_PKT_RATE_70K		= 70000,
	BNA_PKT_RATE_80K		= 80000,
};

enum bna_dim_load_types_e {
	BNA_LOAD_T_HIGH_4		= 0, /* 80K <= r */
	BNA_LOAD_T_HIGH_3		= 1, /* 60K <= r < 80K */
	BNA_LOAD_T_HIGH_2		= 2, /* 50K <= r < 60K */
	BNA_LOAD_T_HIGH_1		= 3, /* 40K <= r < 50K */
	BNA_LOAD_T_LOW_1		= 4, /* 30K <= r < 40K */
	BNA_LOAD_T_LOW_2		= 5, /* 20K <= r < 30K */
	BNA_LOAD_T_LOW_3		= 6, /* 10K <= r < 20K */
	BNA_LOAD_T_LOW_4		= 7, /* r < 10K */
	BNA_LOAD_T_MAX			= 8
};

enum bna_dim_bias_types_e {
	BNA_BIAS_T_SMALL		= 0, /* small pkts > (large pkts * 2) */
	BNA_BIAS_T_LARGE		= 1, /* Not BNA_BIAS_T_SMALL */
	BNA_BIAS_T_MAX			= 2
};

#define BNA_MAX_NAME_SIZE	64
struct bna_ident_s {
	int 			id;
	char			name[BNA_MAX_NAME_SIZE];
};

struct bna_mac_s {
	/* This should be the first one */
	bfa_q_t			qe;
	uint8_t			addr[CNA_ETH_ALEN];
	struct bna_mcam_handle_s *handle;
};

struct bna_mem_descr_s {
	uint32_t		len;
	cna_os_addr_t		kva;
	struct bna_dma_addr_s	dma;
};

struct bna_mem_info_s {
	enum bna_mem_type_e	mem_type;
	uint32_t		len;
	uint32_t 		num;
	uint32_t		align_sz; /* 0/1 = no alignment */
	struct bna_mem_descr_s	*mdl;
	void			*cookie; /* For bnad to unmap dma later */
};

struct bna_intr_descr_s {
	int			vector;
};

struct bna_intr_info_s {
	enum bna_intr_type_e	intr_type;
	int			num;
	struct bna_intr_descr_s	*idl;
};

union bna_res_u {
	struct bna_mem_info_s	mem_info;
	struct bna_intr_info_s	intr_info;
};

struct bna_res_info_s {
	enum bna_res_type_e	res_type;
	union bna_res_u		res_u;
};

/* HW QPT */
struct bna_qpt_s {
	struct bna_dma_addr_s	hw_qpt_ptr;
	cna_os_addr_t		kv_qpt_ptr;
	uint32_t		page_count;
	uint32_t		page_size;
};

struct bna_attr_s {
	bfa_boolean_t		fw_query_complete;
	int			num_txq;
	int			num_rxp;
	int			num_ucmac;
	int			num_mcmac;
	int			max_rit_size;
	int			bw;
	int			num_wol_magic;	/* magic packet */
	int			num_wol_frame;	/* frame pattern */
};

struct bna_wol_s {
	bfa_q_t			qe;
	uint32_t	os_id;
	uint8_t	index;
	uint8_t	rsvd[3];
	uint8_t			pattern[BFI_ENET_WOL_FRAME_LEN];
	uint8_t			mask[BFI_ENET_WOL_FRAME_MASK_LEN];
};


/**
 *
 * Ethport
 *
 */



struct bna_ethport_s {
	bfa_fsm_t		fsm;
	enum bna_ethport_flags_e flags;

	enum bna_link_status_e	link_status;

	int			rx_started_count;

	void (*stop_cbfn)(struct bna_enet_s *);

	void (*adminup_cbfn)(struct bnad_s *, enum bna_cb_status_e);

	void (*link_cbfn)(struct bnad_s *, enum bna_link_status_e);

	struct bfa_msgq_cmd_entry_s msgq_cmd;
	union {
		struct bfi_enet_enable_req admin_req;
		struct bfi_enet_diag_lb_req lpbk_req;
	} bfi_enet_cmd;

	struct bna_s		*bna;
};



/**
 *
 * Interrupt Block
 *
 */



/* Doorbell structure */
struct bna_ib_dbell_s {
	cna_iomem_t		doorbell_addr;
	uint32_t		doorbell_ack;
};

/* IB structure */
struct bna_ib_s {
	struct bna_dma_addr_s	ib_seg_host_addr;
	cna_os_addr_t		ib_seg_host_addr_kva;

	struct bna_ib_dbell_s	door_bell;

	enum bna_intr_type_e	intr_type;
	int			intr_vector;

	uint8_t 		coalescing_timeo;    /* Unit is 5usec. */
	int			interpkt_count;
	int			interpkt_timeo;
};



/**
 *
 * Tx object
 *
 */



/* Tx datapath control structure */
#define BNA_Q_NAME_SIZE		16
struct bna_tcb_s {
	/* Fast path */
	void			**sw_qpt;
	void			*sw_q;
	void			*unmap_q;
	uint32_t		producer_index;
	uint32_t		consumer_index;
	volatile uint32_t	*hw_consumer_index;
	uint32_t		q_depth;
	cna_iomem_t		q_dbell;
	struct bna_ib_dbell_s	*i_dbell;
	/* Control path */
	struct bna_txq_s	*txq;
	struct bnad_s		*bnad;
	void			*priv; /* BNAD's cookie */
	enum bna_intr_type_e	intr_type;
	int			intr_vector;
	uint8_t			priority; /* Current priority */
	unsigned long		flags; /* Used by shim as required */
	int			id;
	char			name[BNA_Q_NAME_SIZE];
};

/* TxQ QPT and configuration */
struct bna_txq_s {
	/* This should be the first one */
	bfa_q_t			qe;

	uint8_t			priority;

	struct bna_qpt_s	qpt;
	struct bna_tcb_s	*tcb;
	struct bna_ib_s		ib;

	struct bna_tx_s		*tx;

	int			hw_id;

	uint64_t 		tx_packets;
	uint64_t 		tx_bytes;
};

/* Tx object */
struct bna_tx_s {
	/* This should be the first one */
	bfa_q_t			qe;
	int			rid;
	int			hw_id;

	bfa_fsm_t		fsm;
	enum bna_tx_flags_e	flags;

	enum bna_tx_type_e	type;
	int			num_txq;

	bfa_q_t			txq_q;
	uint16_t		txf_vlan_id;

	bfa_boolean_t		veb_enable;

	/* Tx event handlers */
	void (*tcb_setup_cbfn)(struct bnad_s *, struct bna_tcb_s *);
	void (*tcb_destroy_cbfn)(struct bnad_s *, struct bna_tcb_s *);
	void (*tx_stall_cbfn)(struct bnad_s *, struct bna_tx_s *);
	void (*tx_resume_cbfn)(struct bnad_s *, struct bna_tx_s *);
	void (*tx_cleanup_cbfn)(struct bnad_s *, struct bna_tx_s *);

	/* callback for bna_tx_disable(), bna_tx_stop() */
	void (*stop_cbfn)(void *arg, struct bna_tx_s *tx);
	void			*stop_cbarg;

	/* callback for bna_tx_prio_set() */
	void (*prio_change_cbfn)(struct bnad_s *bnad, struct bna_tx_s *tx);

	struct bfa_msgq_cmd_entry_s msgq_cmd;
	union {
		struct bfi_enet_tx_cfg_req	cfg_req;
		struct bfi_enet_req		req;
		struct bfi_enet_tx_cfg_rsp	cfg_rsp;
	} bfi_enet_cmd;

	struct bna_s		*bna;
	void			*priv;	/* BNAD's cookie */
};

/* Tx object configuration used during creation */
struct bna_tx_config_s {
	int			num_txq;
	int			txq_depth;
	int			coalescing_timeo;
	enum bna_tx_type_e	tx_type;
};

struct bna_tx_event_cbfn_s {
	/* Optional */
	void (*tcb_setup_cbfn)(struct bnad_s *, struct bna_tcb_s *);
	void (*tcb_destroy_cbfn)(struct bnad_s *, struct bna_tcb_s *);
	/* Mandatory */
	void (*tx_stall_cbfn)(struct bnad_s *, struct bna_tx_s *);
	void (*tx_resume_cbfn)(struct bnad_s *, struct bna_tx_s *);
	void (*tx_cleanup_cbfn)(struct bnad_s *, struct bna_tx_s *);
};

/* Tx module - keeps track of free, active tx objects */
struct bna_tx_mod_s {
	struct bna_tx_s 	*tx;		/* BFI_MAX_TXQ entries */
	struct bna_txq_s	*txq;		/* BFI_MAX_TXQ entries */

	bfa_q_t			tx_free_q;
	bfa_q_t			tx_active_q;

	bfa_q_t			txq_free_q;

	/* callback for bna_tx_mod_stop() */
	void (*stop_cbfn)(struct bna_enet_s *enet);

	bfa_wc_t		tx_stop_wc;

	enum bna_tx_mod_flags_e	flags;

	uint8_t			prio_map;
	int			default_prio;
	int			iscsi_over_cee;
	int			iscsi_prio;
	int			prio_reconfigured;
	struct bfa_cee_dcbx_cfg_s dcb_cfg;

	uint32_t		rid_mask;

	struct bna_s		*bna;
};



/**
 *
 * Rx object
 *
 */



/* Rx datapath control structure */
struct bna_rcb_s {
	/* Fast path */
	void			**sw_qpt;
	void			*sw_q;
	void			*unmap_q;
	uint32_t		producer_index;
	uint32_t		consumer_index;
	uint32_t		q_depth;
	cna_iomem_t		q_dbell;
	/* Control path */
	struct bna_rxq_s	*rxq;
	struct bna_ccb_s	*ccb;
	struct bnad_s		*bnad;
	void			*priv; /* BNAD's cookie */
	unsigned long		flags;
	int			id;
};

/* RxQ structure - QPT, configuration */
struct bna_rxq_s {
	/* This should be the first one */
	bfa_q_t			qe;

	int			buffer_size;
	int			q_depth;
	uint32_t		num_vecs;
	enum bna_status_e	multi_buffer;

	struct bna_qpt_s	qpt;
	struct bna_rcb_s	*rcb;

	struct bna_rxp_s	*rxp;
	struct bna_rx_s		*rx;

	int			hw_id;

	uint64_t 		rx_packets;
	uint64_t		rx_bytes;
	uint64_t 		rx_packets_with_error;
	uint64_t 		rxbuf_alloc_failed;
};

/* RxQ pair */
union bna_rxq_u {
	struct {
		struct bna_rxq_s	*hdr;
		struct bna_rxq_s	*data;
	} hds;
	struct {
		struct bna_rxq_s	*small;
		struct bna_rxq_s	*large;
	} slr;
	struct {
		struct bna_rxq_s	*only;
		struct bna_rxq_s	*reserved;
	} single;
};

/* Packet rate for Dynamic Interrupt Moderation */
struct bna_pkt_rate_s {
	uint32_t		small_pkt_cnt;
	uint32_t		large_pkt_cnt;
};

/* Completion control structure */
struct bna_ccb_s {
	/* Fast path */
	void			**sw_qpt;
	void			*sw_q;
	uint32_t		producer_index;
	volatile uint32_t	*hw_producer_index;
	uint32_t		q_depth;
	struct bna_ib_dbell_s	*i_dbell;
	struct bna_rcb_s	*rcb[2];
	void			*ctrl; /* For shim */
	struct bna_pkt_rate_s	pkt_rate;
	uint32_t		pkts_una;
	uint32_t		bytes_per_intr;

	/* Control path */
	struct bna_cq_s		*cq;
	struct bnad_s		*bnad;
	void			*priv; /* BNAD's cookie */
	enum bna_intr_type_e	intr_type;
	int			intr_vector;
	uint8_t			rx_coalescing_timeo; /* For NAPI */
	int			id;
	char			name[BNA_Q_NAME_SIZE];
};

/* CQ QPT, configuration  */
struct bna_cq_s {
	struct bna_qpt_s	qpt;
	struct bna_ccb_s	*ccb;

	struct bna_ib_s		ib;

	struct bna_rx_s		*rx;
};

/* Rx Path structure - one per MSIX vector/CPU */
struct bna_rxp_s {
	/* This should be the first one */
	bfa_q_t			qe;

	enum bna_rxp_type_e	type;
	union	bna_rxq_u	rxq;
	struct bna_cq_s		cq;

	struct bna_rx_s		*rx;

	/* MSI-x vector number for configuring RSS */
	int			vector;
	int			hw_id;
};

struct bna_rss_config_s {
	bfi_enet_rss_type_t	hash_type;
	uint8_t			hash_mask;
	uint32_t		toeplitz_hash_key[BFI_ENET_RSS_KEY_LEN];
};

struct bna_hds_config_s {
	bfi_enet_hds_type_t	hdr_type;
	int			forced_offset;
};

/* RxF structure (hardware Rx Function) */
struct bna_rxf_s {
	bfa_fsm_t		fsm;
	enum bna_rxf_flags_e	flags;

	struct bfa_msgq_cmd_entry_s msgq_cmd;
	union {
		struct bfi_enet_enable_req req;
		struct bfi_enet_rss_cfg_req rss_req;
		struct bfi_enet_rit_req rit_req;
		struct bfi_enet_rx_vlan_req vlan_req;
		struct bfi_enet_mcast_add_req mcast_add_req;
		struct bfi_enet_mcast_del_req mcast_del_req;
		struct bfi_enet_ucast_req ucast_req;
		struct bfi_enet_wol_magic_frame wol_magic;
		struct bfi_enet_wol_frame_gen wol_frame;
	} bfi_enet_cmd;

	/* callback for bna_rxf_start() */
	void (*start_cbfn) (struct bna_rx_s *rx);
	struct bna_rx_s		*start_cbarg;

	/* callback for bna_rxf_stop() */
	void (*stop_cbfn) (struct bna_rx_s *rx);
	struct bna_rx_s 	*stop_cbarg;

	/* callback for bna_rx_receive_pause() / bna_rx_receive_resume() */
	void (*oper_state_cbfn) (struct bnad_s *bnad, struct bna_rx_s *rx);
	struct bnad_s		*oper_state_cbarg;

	/**
	 * callback for:
	 *	bna_rxf_ucast_set()
	 *	bna_rxf_{ucast/mcast}_add(),
	 * 	bna_rxf_{ucast/mcast}_del(),
	 *	bna_rxf_mode_set()
	 */
	void (*cam_fltr_cbfn)(struct bnad_s *bnad, struct bna_rx_s *rx);
	struct bnad_s		*cam_fltr_cbarg;

	/* List of unicast addresses yet to be applied to h/w */
	bfa_q_t			ucast_pending_add_q;
	bfa_q_t			ucast_pending_del_q;
	struct bna_mac_s	*ucast_pending_mac;
	int			ucast_pending_set;
	/* ucast addresses applied to the h/w */
	bfa_q_t			ucast_active_q;
	struct bna_mac_s	ucast_active_mac;
	int			ucast_active_set;

	/* List of multicast addresses yet to be applied to h/w */
	bfa_q_t			mcast_pending_add_q;
	bfa_q_t			mcast_pending_del_q;
	/* multicast addresses applied to the h/w */
	bfa_q_t			mcast_active_q;
	bfa_q_t			mcast_handle_q;

	/* Rx modes yet to be applied to h/w */
	enum bna_rxmode_e	rxmode_pending;
	enum bna_rxmode_e	rxmode_pending_bitmask;
	/* Rx modes applied to h/w */
	enum bna_rxmode_e	rxmode_active;

	uint8_t			vlan_pending_bitmask;
	enum bna_status_e	vlan_filter_status;
	uint32_t	vlan_filter_table[(BFI_ENET_VLAN_ID_MAX + 1) / 32];
	bfa_boolean_t		vlan_strip_pending;
	enum bna_status_e	vlan_strip_status;

	enum bna_rss_flags_e	rss_pending;
	enum bna_status_e	rss_status;
	struct bna_rss_config_s	rss_cfg;
	uint8_t			*rit;
	int			rit_size;

	/* List of magic packet data yet to be applied to h/w */
	bfa_q_t			wol_m_pending_add_q;
	bfa_q_t			wol_m_pending_del_q;
	/* magic packet data applied to the h/w */
	bfa_q_t			wol_m_active_q;

	/* List of frame pattern data yet to be applied to h/w */
	bfa_q_t			wol_f_pending_add_q;
	bfa_q_t			wol_f_pending_del_q;
	/* frame pattern data applied to the h/w */
	bfa_q_t			wol_f_active_q;

	struct bna_rx_s		*rx;
};

/* Rx object */
struct bna_rx_s {
	/* This should be the first one */
	bfa_q_t			qe;
	int			rid;
	int			hw_id;

	bfa_fsm_t		fsm;

	enum bna_rx_type_e	type;

	int			num_paths;
	bfa_q_t			rxp_q;

	struct bna_hds_config_s	hds_cfg;

	struct bna_rxf_s	rxf;

	enum bna_rx_flags_e	rx_flags;

	struct bfa_msgq_cmd_entry_s msgq_cmd;
	union {
		struct bfi_enet_rx_cfg_req	cfg_req;
		struct bfi_enet_req		req;
		struct bfi_enet_rx_cfg_rsp	cfg_rsp;
	} bfi_enet_cmd;

	/* Rx event handlers */
	void (*rcb_setup_cbfn)(struct bnad_s *, struct bna_rcb_s *);
	void (*rcb_destroy_cbfn)(struct bnad_s *, struct bna_rcb_s *);
	void (*ccb_setup_cbfn)(struct bnad_s *, struct bna_ccb_s *);
	void (*ccb_destroy_cbfn)(struct bnad_s *, struct bna_ccb_s *);
	void (*rx_stall_cbfn)(struct bnad_s *, struct bna_rx_s *);
	void (*rx_cleanup_cbfn)(struct bnad_s *, struct bna_rx_s *);
	void (*rx_post_cbfn)(struct bnad_s *, struct bna_rx_s *);

	/* callback for bna_rx_disable(), bna_rx_stop() */
	void (*stop_cbfn)(void *arg, struct bna_rx_s *rx);
	void			*stop_cbarg;

	struct bna_s		*bna;
	void			*priv; /* Shim's cookie */
};

/* Rx object configuration used during creation */
struct bna_rx_config_s {
	enum bna_rx_type_e	rx_type;
	int			num_paths;
	enum bna_rxp_type_e	rxp_type;
	int			paused;
	int			coalescing_timeo;

	/*
	 * Small/Large (or Header/Data) buffer size to be configured
	 * for SLR and HDS queue type.
	 */
	uint32_t		frame_size;

	/* header or small queue */
	uint32_t		q1_depth;
	uint32_t		q1_buf_size;

	/* data or large queue */
	uint32_t		q0_depth;
	uint32_t		q0_buf_size;
	uint32_t		q0_num_vecs;
	enum bna_status_e	q0_multi_buf;

	enum bna_status_e	rss_status;
	struct bna_rss_config_s	rss_config;

	struct bna_hds_config_s	hds_config;

	enum bna_status_e	vlan_strip_status;
};

struct bna_rx_event_cbfn_s {
	/* Optional */
	void (*rcb_setup_cbfn)(struct bnad_s *, struct bna_rcb_s *);
	void (*rcb_destroy_cbfn)(struct bnad_s *, struct bna_rcb_s *);
	void (*ccb_setup_cbfn)(struct bnad_s *, struct bna_ccb_s *);
	void (*ccb_destroy_cbfn)(struct bnad_s *, struct bna_ccb_s *);
	void (*rx_stall_cbfn)(struct bnad_s *, struct bna_rx_s *);
	/* Mandatory */
	void (*rx_cleanup_cbfn)(struct bnad_s *, struct bna_rx_s *);
	void (*rx_post_cbfn)(struct bnad_s *, struct bna_rx_s *);
};

/* Rx module - keeps track of free, active rx objects */
struct bna_rx_mod_s {
	struct bna_s		*bna;		/* back pointer to parent */
	struct bna_rx_s 	*rx;		/* BFI_MAX_RXQ entries */
	struct bna_rxp_s 	*rxp;		/* BFI_MAX_RXQ entries */
	struct bna_rxq_s 	*rxq;		/* BFI_MAX_RXQ entries */

	bfa_q_t			rx_free_q;
	bfa_q_t			rx_active_q;
	int			rx_free_count;

	bfa_q_t			rxp_free_q;
	int			rxp_free_count;

	bfa_q_t			rxq_free_q;
	int			rxq_free_count;

	enum bna_rx_mod_flags_e	flags;

	/* callback for bna_rx_mod_stop() */
	void (*stop_cbfn)(struct bna_enet_s *enet);

	bfa_wc_t		rx_stop_wc;
	uint32_t		dim_vector[BNA_LOAD_T_MAX][BNA_BIAS_T_MAX];
	uint32_t		rid_mask;
};



/**
 *
 * Enet
 *
 */



/* Pause configuration */
struct bna_pause_config_s {
	enum bna_status_e	tx_pause;
	enum bna_status_e	rx_pause;
};

struct bna_enet_s {
	bfa_fsm_t		fsm;
	enum bna_enet_flags_e	flags;

	enum bna_enet_type_e	type;

	struct bna_pause_config_s pause_config;
	int			mtu;

	/* Callback for bna_enet_disable(), enet_stop() */
	void (*stop_cbfn)(void *);
	void			*stop_cbarg;

	/* Callback for bna_enet_pause_config() */
	void (*pause_cbfn)(struct bnad_s *);

	/* Callback for bna_enet_mtu_set() */
	void (*mtu_cbfn)(struct bnad_s *);

	bfa_wc_t		chld_stop_wc;

	struct bfa_msgq_cmd_entry_s msgq_cmd;
	struct bfi_enet_set_pause_req pause_req;

	struct bna_s		*bna;
};



/**
 *
 * IOCEth
 *
 */



struct bna_ioceth_s {
	bfa_fsm_t		fsm;
	struct bfa_ioc_s	ioc;
	struct bfa_timer_mod_s	timer_mod;

	struct bna_attr_s	attr;
	struct bfa_msgq_cmd_entry_s msgq_cmd;
	struct bfi_enet_attr_req_s attr_req;

	bfa_wc_t		chld_stop_wc;

	void (*stop_cbfn)(struct bnad_s *bnad);
	struct bnad_s		*stop_cbarg;

	struct bna_s		*bna;
};



/**
 *
 * CAM
 *
 */



struct bna_ucam_mod_s {
	struct bna_mac_s	*ucmac;		/* num_ucmac * 2 entries */
	bfa_q_t			free_q;
	bfa_q_t			del_q;

	struct bna_s		*bna;
};

struct bna_mcam_handle_s {
	/* This should be the first one */
	bfa_q_t			qe;
	int			handle;
	int			refcnt;
};

struct bna_mcam_mod_s {
	struct bna_mac_s	*mcmac;		/* num_mcmac * 2 entries */
	struct bna_mcam_handle_s *mchandle;	/* num_mcmac entries */
	bfa_q_t			free_q;
	bfa_q_t			del_q;
	bfa_q_t			free_handle_q;

	struct bna_s		*bna;
};



/**
 *
 * Statistics
 *
 */



struct bna_stats_s {
	struct bna_dma_addr_s	hw_stats_dma;
	struct bfi_enet_stats	*hw_stats_kva;
	struct bfi_enet_stats	hw_stats;
};

struct bna_stats_mod_s {
	bfa_boolean_t		ioc_ready;
	bfa_boolean_t		stats_get_busy;
	bfa_boolean_t		stats_clr_busy;
	struct bfa_msgq_cmd_entry_s stats_get_cmd;
	struct bfa_msgq_cmd_entry_s stats_clr_cmd;
	struct bfi_enet_stats_req stats_get;
	struct bfi_enet_stats_req stats_clr;
	int			pending_reqs;
	void (*stop_cbfn)(struct bna_ioceth_s *ioceth);
	struct bna_s		*bna;
};

struct bna_wol_mod_s {
	bfa_q_t		free_magic_q;
	bfa_q_t		free_frame_q;
	struct bna_wol_s *wol_magic;
	struct bna_wol_s *wol_frame;
	struct bna_s *bna;
};


/**
 *
 * BNA
 *
 */



struct bna_s {
	struct bna_ident_s	ident;
	struct bfa_pcidev_s	pcidev;

	struct bna_reg_s	regs;
	struct bna_bit_defn_s	bits;

	struct bna_stats_s	stats;

	struct bfa_trc_mod_s	*trcmod;
	struct bfa_log_mod_s	*logm;
	struct bfa_aen_s	*aen;

	struct bna_ioceth_s	ioceth;
	struct bfa_diag_s	diag;
	struct bfa_cee_s	cee;
	struct bfa_sfp_s	sfp;
	struct bfa_port_s	phy_port;
	struct bfa_flash_s	flash;
	struct bfa_phy_s	phy;
	struct bfa_fru_s	fru;
	struct bfa_msgq_s	msgq;
	struct bfa_ablk_s	ablk;
	struct bfa_edma_s	edma;

	struct bna_ethport_s	ethport;
	struct bna_enet_s 	enet;
	struct bna_stats_mod_s	stats_mod;

	struct bna_tx_mod_s	tx_mod;
	struct bna_rx_mod_s	rx_mod;
	struct bna_ucam_mod_s	ucam_mod;
	struct bna_mcam_mod_s	mcam_mod;

	struct bna_wol_mod_s	wol_mod;

	enum bna_mod_flags_e	mod_flags;

	int			default_mode_rid;
	int			promisc_rid;

	struct bnad_s		*bnad;
};
#endif	/* __BNA_TYPES_H__ */
