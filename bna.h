/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BNA_H__
#define __BNA_H__

#include "cna_os.h"
#include "defs/bfa_defs.h"
#include "defs/bfa_defs_vnic.h"
#include "cna/bfa_ioc.h"
#include "cna/bfa_diag.h"
#include "aen/bfa_aen.h"
#include "cs/bfa_log.h"
#include "log/bfa_log_hal.h"
#include "bfi/bfi_enet.h"
#include "bna_types.h"
#include "bna_trcmod.h"


extern uint32_t bna_dim_vector[][BNA_BIAS_T_MAX];
extern uint32_t bna_napi_dim_vector[][BNA_BIAS_T_MAX];


/**
 *
 *  Macros and constants
 *
 */

#define vlan_tx_tag_get(__skb)          ((__skb)->vlan_tci & ~VLAN_TAG_PRESENT)
#define vlan_tx_tag_present(__skb)      ((__skb)->vlan_tci & VLAN_TAG_PRESENT)
#define smp_mb__before_clear_bit()      barrier()
#define SET_ETHTOOL_OPS(netdev,ops) \
	( (netdev)->ethtool_ops = (ops) )


#define BNA_IOC_TIMER_FREQ		BFA_TIMER_FREQ

/* Log string size */
#define BNA_MESSAGE_SIZE		256

#define bna_ioceth_timer(_dev)		bfa_timer_beat(&((_dev)->timer_mod))

#define bna_is_small_rxq(_id) ((_id) & 0x1)

#define bna_is_header_rxq(_id) bna_is_small_rxq(_id)

#define BNA_MIN(x, y) (((x) < (y)) ? (x) : (y))

#define BNA_MAC_IS_EQUAL(_mac1, _mac2)					\
	(!cna_os_memcmp((_mac1), (_mac2), sizeof(mac_t)))

#define BNA_POWER_OF_2(x) (((x) & ((x) - 1)) == 0)

#define BNA_TO_POWER_OF_2(x)						\
do {									\
	int _shift = 0;							\
	while ((x) && (x) != 1) {					\
		(x) >>= 1;						\
		_shift++;						\
	}								\
	(x) <<= _shift;							\
} while (0)

#define BNA_TO_POWER_OF_2_HIGH(x)					\
do {									\
	int n = 1;							\
	while (n < (x))							\
		n <<= 1;						\
	(x) = n;							\
} while (0)

/*
 * input : _addr-> os dma addr in host endian format,
 * output : _bna_dma_addr-> pointer to hw dma addr
 */
#define BNA_SET_DMA_ADDR(_addr, _bna_dma_addr)				\
do {									\
	uint64_t tmp_addr =						\
	cna_os_dma_addr64((uint64_t)(_addr));				\
	(_bna_dma_addr)->msb = ((struct bna_dma_addr_s *)&tmp_addr)->msb; \
	(_bna_dma_addr)->lsb = ((struct bna_dma_addr_s *)&tmp_addr)->lsb; \
} while (0)

/*
 * input : _bna_dma_addr-> pointer to hw dma addr
 * output : _addr-> os dma addr in host endian format
 */
#define BNA_GET_DMA_ADDR(_bna_dma_addr, _addr)			\
do {								\
	(_addr) = (cna_os_u64((_bna_dma_addr)->msb))		\
	| ((cna_os_ntohl((_bna_dma_addr)->lsb) & 0xffffffff));	\
} while (0)


#define BNA_TRC_MOD(__mod)		((BNA_TRC_ ## __mod) << BFA_TRC_MOD_SH)

#define BNA_TRC_FILE(__mod, __submod)					\
	static int __trc_fileno = 					\
		((BNA_TRC_ ## __mod ## _ ## __submod) | BNA_TRC_MOD(__mod))

#define	containing_rec(addr, type, field)				\
	((type *)((unsigned char *)(addr) - 				\
	(unsigned char *)(&((type *)0)->field)))

/* Datapath */
#define BNA_TXQ_WI_NEEDED(_vectors)	(((_vectors) + 3) >> 2)

/* TxQ element is 64 bytes */
#define BNA_TXQ_PAGE_INDEX_MAX		(CNA_PAGE_SIZE >> 6)
#define BNA_TXQ_PAGE_INDEX_MAX_SHIFT	(CNA_PAGE_SHIFT - 6)

#define BNA_TXQ_QPGE_PTR_GET(_qe_idx, _qpt_ptr, _qe_ptr, _qe_ptr_range) \
{									\
	unsigned int page_index;	/* index within a page */	\
	void *page_addr;						\
	page_index = (_qe_idx) & (BNA_TXQ_PAGE_INDEX_MAX - 1); 		\
	(_qe_ptr_range) = (BNA_TXQ_PAGE_INDEX_MAX - page_index); 	\
	page_addr = (_qpt_ptr)[((_qe_idx) >>  BNA_TXQ_PAGE_INDEX_MAX_SHIFT)];\
	(_qe_ptr) = &((struct bna_txq_entry_s *)(page_addr))[page_index]; \
}

/* RxQ element is 8 bytes */
#define BNA_RXQ_PAGE_INDEX_MAX		(CNA_PAGE_SIZE >> 3)
#define BNA_RXQ_PAGE_INDEX_MAX_SHIFT	(CNA_PAGE_SHIFT - 3)

#define BNA_RXQ_QPGE_PTR_GET(_qe_idx, _qpt_ptr, _qe_ptr, _qe_ptr_range) \
{									\
	unsigned int page_index;	/* index within a page */	\
	void *page_addr;						\
	page_index = (_qe_idx) & (BNA_RXQ_PAGE_INDEX_MAX - 1);		\
	(_qe_ptr_range) = (BNA_RXQ_PAGE_INDEX_MAX - page_index);	\
	page_addr = (_qpt_ptr)[((_qe_idx) >>				\
				BNA_RXQ_PAGE_INDEX_MAX_SHIFT)];		\
	(_qe_ptr) = &((struct bna_rxq_entry_s *)(page_addr))[page_index]; \
}

/* CQ element is 16 bytes */
#define BNA_CQ_PAGE_INDEX_MAX		(CNA_PAGE_SIZE >> 4)
#define BNA_CQ_PAGE_INDEX_MAX_SHIFT	(CNA_PAGE_SHIFT - 4)

#define BNA_CQ_QPGE_PTR_GET(_qe_idx, _qpt_ptr, _qe_ptr, _qe_ptr_range)	\
{									\
	unsigned int page_index;	  /* index within a page */	\
	void *page_addr;						\
									\
	page_index = (_qe_idx) & (BNA_CQ_PAGE_INDEX_MAX - 1);		\
	(_qe_ptr_range) = (BNA_CQ_PAGE_INDEX_MAX - page_index);		\
	page_addr = (_qpt_ptr)[((_qe_idx) >>				\
				    BNA_CQ_PAGE_INDEX_MAX_SHIFT)];	\
	(_qe_ptr) = &((struct bna_cq_entry_s *)(page_addr))[page_index];\
}

#define BNA_QE_INDX_2_PTR(_cast, _qe_idx, _q_base)			\
	(&((_cast *)(_q_base))[(_qe_idx)])

#define BNA_QE_INDX_RANGE(_qe_idx, _q_depth) ((_q_depth) - (_qe_idx))

#define BNA_QE_INDX_ADD(_qe_idx, _qe_num, _q_depth)			\
	((_qe_idx) = ((_qe_idx) + (_qe_num)) & ((_q_depth) - 1))

#define BNA_QE_INDX_INC(_idx, _q_depth)					\
do {									\
	(_idx)++;							\
	if ((_idx) == (_q_depth))					\
		(_idx) = 0;						\
} while (0)

#define BNA_Q_INDEX_CHANGE(_old_idx, _updated_idx, _q_depth)		\
	(((_updated_idx) - (_old_idx)) & ((_q_depth) - 1))

#define BNA_QE_FREE_CNT(_q_ptr, _q_depth)				\
	(((_q_ptr)->consumer_index - (_q_ptr)->producer_index - 1) &	\
	 ((_q_depth) - 1))

#define BNA_QE_IN_USE_CNT(_q_ptr, _q_depth)				\
	((((_q_ptr)->producer_index - (_q_ptr)->consumer_index)) &	\
	 (_q_depth - 1))

#define BNA_Q_GET_CI(_q_ptr)		((_q_ptr)->q.consumer_index)

#define BNA_Q_GET_PI(_q_ptr)		((_q_ptr)->q.producer_index)

#define BNA_Q_PI_ADD(_q_ptr, _num)					\
	(_q_ptr)->q.producer_index =					\
		(((_q_ptr)->q.producer_index + (_num)) &		\
		((_q_ptr)->q.q_depth - 1))

#define BNA_Q_CI_ADD(_q_ptr, _num) 					\
	(_q_ptr)->q.consumer_index =					\
		(((_q_ptr)->q.consumer_index + (_num))  		\
		& ((_q_ptr)->q.q_depth - 1))

#define BNA_Q_FREE_COUNT(_q_ptr)					\
	(BNA_QE_FREE_CNT(&((_q_ptr)->q), (_q_ptr)->q.q_depth))

#define BNA_Q_IN_USE_COUNT(_q_ptr)  					\
	(BNA_QE_IN_USE_CNT(&(_q_ptr)->q, (_q_ptr)->q.q_depth))

#define BNA_LARGE_PKT_SIZE		1000

#define BNA_UPDATE_PKT_CNT(_pkt, _len)					\
do {									\
	if ((_len) > BNA_LARGE_PKT_SIZE) {				\
		(_pkt)->large_pkt_cnt++;				\
	} else {							\
		(_pkt)->small_pkt_cnt++;				\
	}								\
} while (0)

#define is_xxx_enable(mode, bitmask, xxx) ((bitmask & xxx) && (mode & xxx))

#define is_xxx_disable(mode, bitmask, xxx) ((bitmask & xxx) && !(mode & xxx))

#define xxx_enable(mode, bitmask, xxx)					\
do {									\
	bitmask |= xxx;							\
	mode |= xxx;							\
} while (0)

#define xxx_disable(mode, bitmask, xxx)					\
do {									\
	bitmask |= xxx;							\
	mode &= ~xxx;							\
} while (0)

#define xxx_inactive(mode, bitmask, xxx)				\
do {									\
	bitmask &= ~xxx;						\
	mode &= ~xxx;							\
} while (0)

#define is_promisc_enable(mode, bitmask)				\
	is_xxx_enable(mode, bitmask, BNA_RXMODE_PROMISC)

#define is_promisc_disable(mode, bitmask)				\
	is_xxx_disable(mode, bitmask, BNA_RXMODE_PROMISC)

#define promisc_enable(mode, bitmask)					\
	xxx_enable(mode, bitmask, BNA_RXMODE_PROMISC)

#define promisc_disable(mode, bitmask)					\
	xxx_disable(mode, bitmask, BNA_RXMODE_PROMISC)

#define promisc_inactive(mode, bitmask)					\
	xxx_inactive(mode, bitmask, BNA_RXMODE_PROMISC)

#define is_default_enable(mode, bitmask)				\
	is_xxx_enable(mode, bitmask, BNA_RXMODE_DEFAULT)

#define is_default_disable(mode, bitmask)				\
	is_xxx_disable(mode, bitmask, BNA_RXMODE_DEFAULT)

#define default_enable(mode, bitmask)					\
	xxx_enable(mode, bitmask, BNA_RXMODE_DEFAULT)

#define default_disable(mode, bitmask)					\
	xxx_disable(mode, bitmask, BNA_RXMODE_DEFAULT)

#define default_inactive(mode, bitmask)					\
	xxx_inactive(mode, bitmask, BNA_RXMODE_DEFAULT)

#define is_allmulti_enable(mode, bitmask)				\
	is_xxx_enable(mode, bitmask, BNA_RXMODE_ALLMULTI)

#define is_allmulti_disable(mode, bitmask)				\
	is_xxx_disable(mode, bitmask, BNA_RXMODE_ALLMULTI)

#define allmulti_enable(mode, bitmask)					\
	xxx_enable(mode, bitmask, BNA_RXMODE_ALLMULTI)

#define allmulti_disable(mode, bitmask)					\
	xxx_disable(mode, bitmask, BNA_RXMODE_ALLMULTI)

#define allmulti_inactive(mode, bitmask)				\
	xxx_inactive(mode, bitmask, BNA_RXMODE_ALLMULTI)

#define	GET_RXQS(rxp, q0, q1)	do {					\
	switch ((rxp)->type) {						\
	case BNA_RXP_SINGLE:						\
		(q0) = rxp->rxq.single.only;				\
		(q1) = NULL;						\
		break;							\
	case BNA_RXP_SLR:						\
		(q0) = rxp->rxq.slr.large;				\
		(q1) = rxp->rxq.slr.small;				\
		break;							\
	case BNA_RXP_HDS:						\
		(q0) = rxp->rxq.hds.data;				\
		(q1) = rxp->rxq.hds.hdr;				\
		break;							\
	}								\
} while (0)

#define bna_tx_rid_mask(_bna) (_bna)->tx_mod.rid_mask

#define bna_rx_rid_mask(_bna) (_bna)->rx_mod.rid_mask

#define bna_tx_from_rid(_bna, _rid, _tx)				\
do {									\
	struct bna_tx_mod_s *__tx_mod = &(_bna)->tx_mod;		\
	struct bna_tx_s *__tx;						\
	bfa_q_t *qe;							\
	_tx = NULL;							\
	bfa_q_iter(&__tx_mod->tx_active_q, qe) {			\
		__tx = (struct bna_tx_s *)qe;				\
		if (__tx->rid == (_rid)) {				\
			(_tx) = __tx;					\
			break;						\
		}							\
	}								\
} while (0)

#define bna_rx_from_rid(_bna, _rid, _rx)				\
do {									\
	struct bna_rx_mod_s *__rx_mod = &(_bna)->rx_mod;		\
	struct bna_rx_s *__rx;						\
	bfa_q_t *qe;							\
	_rx = NULL;							\
	bfa_q_iter(&__rx_mod->rx_active_q, qe) {			\
		__rx = (struct bna_rx_s *)qe;				\
		if (__rx->rid == (_rid)) {				\
			(_rx) = __rx;					\
			break;						\
		}							\
	}								\
} while (0)

#define bna_prio_allowed(_bna, _prio) ((_bna)->tx_mod.prio_map & (1 << _prio))

#define bna_iscsi_prio(_bna) ((_bna)->tx_mod.iscsi_prio)

#define bna_default_prio(_bna) ((_bna)->tx_mod.default_prio)

#define bna_is_iscsi_over_cee(_bna) ((_bna)->tx_mod.iscsi_over_cee)

#define bna_is_prio_reconfig(_bna) ((_bna)->tx_mod.prio_reconfigured)

#define bna_max_mcmac(_bna) ((_bna)->ioceth.attr.num_mcmac)

#define bna_is_rx_enabled(_rx) ((_rx)->rx_flags & BNA_RX_F_ENABLED)

#define bna_attr(_bna) (&(_bna)->ioceth.attr)

#define bna_mcam_mod_free_q(_bna) (&(_bna)->mcam_mod.free_q)

#define bna_mcam_mod_del_q(_bna) (&(_bna)->mcam_mod.del_q)

#define bna_ucam_mod_free_q(_bna) (&(_bna)->ucam_mod.free_q)

#define bna_ucam_mod_del_q(_bna) (&(_bna)->ucam_mod.del_q)

/**
 *
 *  Inline functions
 *
 */


static inline struct bna_mac_s *bna_mac_find(bfa_q_t *q, uint8_t *addr)
{
	struct bna_mac_s *mac = NULL;
	bfa_q_t *qe;
	bfa_q_iter(q, qe) {
		if (BNA_MAC_IS_EQUAL(((struct bna_mac_s *)qe)->addr, addr)) {
			mac = (struct bna_mac_s *)qe;
			break;
		}
	}
	return mac;
}


/**
 *
 * Function prototypes
 *
 */


/**
 * MBOX
 */


/* API for BNAD */
void bna_mbox_handler(struct bna_s *bna, uint32_t intr_status);


/**
 * ETHPORT
 */


/* FW response handlers */
void bna_bfi_ethport_admin_rsp(struct bna_ethport_s *ethport,
			       struct bfi_msgq_mhdr_s *msghdr);
void bna_bfi_ethport_lpbk_rsp(struct bna_ethport_s *ethport,
			      struct bfi_msgq_mhdr_s *msghdr);
void bna_bfi_ethport_linkup_aen(struct bna_ethport_s *ethport,
				struct bfi_msgq_mhdr_s *msghdr);
void bna_bfi_ethport_linkdown_aen(struct bna_ethport_s *ethport,
				  struct bfi_msgq_mhdr_s *msghdr);
void bna_bfi_ethport_enable_aen(struct bna_ethport_s *ethport,
				struct bfi_msgq_mhdr_s *msghdr);
void bna_bfi_ethport_disable_aen(struct bna_ethport_s *ethport,
				 struct bfi_msgq_mhdr_s *msghdr);

/* Callbacks for RX */
void bna_ethport_cb_rx_started(struct bna_ethport_s *ethport);
void bna_ethport_cb_rx_stopped(struct bna_ethport_s *ethport);

/* APIs for ENET */
void bna_ethport_start(struct bna_ethport_s *ethport);
void bna_ethport_stop(struct bna_ethport_s *ethport);
void bna_ethport_fail(struct bna_ethport_s *ethport);

/* APIs for BNA */
void bna_ethport_init(struct bna_ethport_s *ethport, struct bna_s *bna);
void bna_ethport_uninit(struct bna_ethport_s *ethport);

/* APIs for BNAD */
void bna_ethport_admin_up(struct bna_ethport_s *ethport,
			void (*cbfn)(struct bnad_s *, enum bna_cb_status_e));
void bna_ethport_admin_down(struct bna_ethport_s *ethport);
void bna_ethport_linkcbfn_set(struct bna_ethport_s *ethport,
			   void (*linkcbfn)(struct bnad_s *,
					    enum bna_link_status_e));
int bna_ethport_is_disabled(struct bna_ethport_s *ethport);


/**
 * TX MODULE AND TX
 */


/* FW response handelrs */
void bna_bfi_tx_enet_start_rsp(struct bna_tx_s *tx,
			       struct bfi_msgq_mhdr_s *msghdr);
void bna_bfi_tx_enet_stop_rsp(struct bna_tx_s *tx,
			      struct bfi_msgq_mhdr_s *msghdr);
void bna_bfi_tx_bw_update(struct bna_tx_mod_s *tx_mod);

/* APIs for ENET */
void bna_tx_mod_start(struct bna_tx_mod_s *tx_mod, enum bna_tx_type_e type);
void bna_tx_mod_stop(struct bna_tx_mod_s *tx_mod, enum bna_tx_type_e type);
void bna_tx_mod_fail(struct bna_tx_mod_s *tx_mod);
void bna_tx_mod_prio_reconfig(struct bna_tx_mod_s *tx_mod, int cee_linkup,
			      uint8_t prio_map, uint8_t iscsi_prio_map);

/* APIs for BNA */
void bna_tx_mod_init(struct bna_tx_mod_s *tx_mod, struct bna_s *bna,
		     struct bna_res_info_s *res_info);
void bna_tx_mod_uninit(struct bna_tx_mod_s *tx_mod);

/* APIs for BNAD */
void bna_tx_res_req(int num_txq, int txq_depth,
		    struct bna_res_info_s *res_info);
struct bna_tx_s *bna_tx_create(struct bna_s *bna, struct bnad_s *bnad,
			       struct bna_tx_config_s *tx_cfg,
			       struct bna_tx_event_cbfn_s *tx_cbfn,
			       struct bna_res_info_s *res_info, void *priv);
void bna_tx_destroy(struct bna_tx_s *tx);
void bna_tx_enable(struct bna_tx_s *tx);
void bna_tx_disable(struct bna_tx_s *tx, enum bna_cleanup_type_e type,
		    void (*cbfn)(void *, struct bna_tx_s *));
void bna_tx_cleanup_complete(struct bna_tx_s *tx);
void bna_tx_prio_set(struct bna_tx_s *tx, int prio,
		     void (*cbfn)(struct bnad_s *, struct bna_tx_s *));
void bna_tx_coalescing_timeo_set(struct bna_tx_s *tx, int coalescing_timeo);
void bna_tx_stats_clr(struct bna_tx_s *tx);


/**
 * RX MODULE, RX, RXF
 */


/* FW response handlers */
void bna_bfi_rx_enet_start_rsp(struct bna_rx_s *rx,
			       struct bfi_msgq_mhdr_s *msghdr);
void bna_bfi_rx_enet_stop_rsp(struct bna_rx_s *rx,
			      struct bfi_msgq_mhdr_s *msghdr);
void bna_bfi_rxf_cfg_rsp(struct bna_rxf_s *rxf, struct bfi_msgq_mhdr_s *msghdr);
void bna_bfi_rxf_mcast_add_rsp(struct bna_rxf_s *rxf,
			       struct bfi_msgq_mhdr_s *msghdr);
void bna_bfi_rxf_ucast_set_rsp(struct bna_rxf_s *rxf,
			       struct bfi_msgq_mhdr_s *msghdr);

/* RxF APIs for RX */
void bna_rxf_start(struct bna_rxf_s *rxf);
void bna_rxf_stop(struct bna_rxf_s *rxf);
void bna_rxf_fail(struct bna_rxf_s *rxf);
void bna_rxf_init(struct bna_rxf_s *rxf, struct bna_rx_s *rx,
		  struct bna_rx_config_s *q_config,
		  struct bna_res_info_s *res_info);
void bna_rxf_uninit(struct bna_rxf_s *rxf);

/* RX APIs for RxF */
void bna_rx_cb_rxf_stopped(struct bna_rx_s *rx);
void bna_rx_cb_rxf_started(struct bna_rx_s *rx);

/* APIs for ENET */
void bna_rx_mod_start(struct bna_rx_mod_s *rx_mod, enum bna_rx_type_e type);
void bna_rx_mod_stop(struct bna_rx_mod_s *rx_mod, enum bna_rx_type_e type);
void bna_rx_mod_fail(struct bna_rx_mod_s *rx_mod);

/* APIs for BNA */
void bna_rx_mod_init(struct bna_rx_mod_s *rx_mod, struct bna_s *bna,
		     struct bna_res_info_s *res_info);
void bna_rx_mod_uninit(struct bna_rx_mod_s *rx_mod);

/* APIs for BNAD */
void bna_rx_res_req(struct bna_rx_config_s *rx_config,
		    struct bna_res_info_s *res_info);
struct bna_rx_s *bna_rx_create(struct bna_s *bna, struct bnad_s *bnad,
			       struct bna_rx_config_s *rx_cfg,
			       struct bna_rx_event_cbfn_s *rx_cbfn,
			       struct bna_res_info_s *res_info, void *priv);
void bna_rx_destroy(struct bna_rx_s *rx);
void bna_rx_enable(struct bna_rx_s *rx);
void bna_rx_disable(struct bna_rx_s *rx, enum bna_cleanup_type_e type,
		    void (*cbfn)(void *, struct bna_rx_s *));
void bna_rx_cleanup_complete(struct bna_rx_s *rx);
void bna_rx_coalescing_timeo_set(struct bna_rx_s *rx, int coalescing_timeo);
void bna_rx_dim_reconfig(struct bna_s *bna, uint32_t vector[][BNA_BIAS_T_MAX]);
void bna_rx_dim_update(struct bna_ccb_s *ccb);
enum bna_cb_status_e
bna_rx_ucast_set(struct bna_rx_s *rx, uint8_t *ucmac,
		 void (*cbfn)(struct bnad_s *, struct bna_rx_s *));
enum bna_cb_status_e
bna_rx_ucast_clr(struct bna_rx_s *rx,
		 void (*cbfn)(struct bnad_s *, struct bna_rx_s *));
enum bna_cb_status_e
bna_rx_ucast_add(struct bna_rx_s *rx, uint8_t* ucmac,
		 void (*cbfn)(struct bnad_s *, struct bna_rx_s *));
enum bna_cb_status_e
bna_rx_ucast_del(struct bna_rx_s *rx, uint8_t *ucmac,
		 void (*cbfn)(struct bnad_s *, struct bna_rx_s *));
enum bna_cb_status_e
bna_rx_ucast_listset(struct bna_rx_s *rx, int count, uint8_t *uclist,
		     void (*cbfn)(struct bnad_s *, struct bna_rx_s *));
enum bna_cb_status_e
bna_rx_mcast_add(struct bna_rx_s *rx, uint8_t *mcmac,
		 void (*cbfn)(struct bnad_s *, struct bna_rx_s *));
enum bna_cb_status_e
bna_rx_mcast_del(struct bna_rx_s *rx, uint8_t *mcmac,
		 void (*cbfn)(struct bnad_s *, struct bna_rx_s *));
enum bna_cb_status_e
bna_rx_mcast_listset(struct bna_rx_s *rx, int count, uint8_t *mcmac,
		     void (*cbfn)(struct bnad_s *, struct bna_rx_s *));
void bna_rx_mcast_delall(struct bna_rx_s *rx,
			 void (*cbfn)(struct bnad_s *, struct bna_rx_s *));
enum bna_cb_status_e
bna_rx_mode_set(struct bna_rx_s *rx, enum bna_rxmode_e rxmode,
		enum bna_rxmode_e bitmask,
		void (*cbfn)(struct bnad_s *, struct bna_rx_s *));
void bna_rx_vlan_add(struct bna_rx_s *rx, int vlan_id);
void bna_rx_vlan_del(struct bna_rx_s *rx, int vlan_id);
void bna_rx_vlanfilter_enable(struct bna_rx_s *rx);
void bna_rx_vlanfilter_disable(struct bna_rx_s *rx);
void bna_rx_vlan_strip_enable(struct bna_rx_s *rx);
void bna_rx_vlan_strip_disable(struct bna_rx_s *rx);
void bna_rx_rss_enable(struct bna_rx_s *rx);
void bna_rx_rss_disable(struct bna_rx_s *rx);
void bna_rx_rss_reconfig(struct bna_rx_s *rx,
			 struct bna_rss_config_s *rss_config);
void bna_rx_rss_rit_set(struct bna_rx_s *rx, unsigned int *vectors,
			int nvectors);
void bna_rx_hds_enable(struct bna_rx_s *rx, struct bna_hds_config_s *hds_config,
		       void (*cbfn)(struct bnad_s *, struct bna_rx_s *));
void bna_rx_hds_disable(struct bna_rx_s *rx,
			void (*cbfn)(struct bnad_s *, struct bna_rx_s *));
void bna_rx_receive_pause(struct bna_rx_s *rx,
			  void (*cbfn)(struct bnad_s *, struct bna_rx_s *));
void bna_rx_receive_resume(struct bna_rx_s *rx,
			   void (*cbfn)(struct bnad_s *, struct bna_rx_s *));
void bna_rx_stats_clr(struct bna_rx_s *rx);
enum bna_cb_status_e
bna_rx_wol_add_magic(struct bna_rx_s *rx, uint32_t id, mac_t *mac,
			   void (*cbfn)(struct bnad_s *, struct bna_rx_s *));
enum bna_cb_status_e
bna_rx_wol_add_frame(struct bna_rx_s *rx, uint32_t id,
				uint8_t *pattern, uint8_t *mask,
			   void (*cbfn)(struct bnad_s *, struct bna_rx_s *));
enum bna_cb_status_e
bna_rx_wol_del_magic(struct bna_rx_s *rx, uint32_t id,
			   void (*cbfn)(struct bnad_s *, struct bna_rx_s *));
enum bna_cb_status_e
bna_rx_wol_del_frame(struct bna_rx_s *rx, uint32_t id,
			   void (*cbfn)(struct bnad_s *, struct bna_rx_s *));


/**
 * ENET
 */


/* FW response handlers */
void bna_bfi_pause_set_rsp(struct bna_enet_s *enet,
			   struct bfi_msgq_mhdr_s *msghdr);
void bna_bfi_bw_update_aen(struct bna_enet_s *enet,
			   struct bfi_msgq_mhdr_s *msghdr);

/* API for RX */
int bna_enet_mtu_get(struct bna_enet_s *enet);

/* Callbacks for ETHPORT, TX, RX */
void bna_enet_cb_ethport_stopped(struct bna_enet_s *enet);
void bna_enet_cb_tx_stopped(struct bna_enet_s *enet);
void bna_enet_cb_rx_stopped(struct bna_enet_s *enet);

/* APIs for IOCETH */
void bna_enet_start(struct bna_enet_s *enet);
void bna_enet_stop(struct bna_enet_s *enet);
void bna_enet_fail(struct bna_enet_s *enet);

/* APIs for BNA */
void bna_enet_init(struct bna_enet_s *enet, struct bna_s *bna);
void bna_enet_uninit(struct bna_enet_s *enet);

/* API for BNAD */
void bna_enet_enable(struct bna_enet_s *enet);
void bna_enet_disable(struct bna_enet_s *enet, enum bna_cleanup_type_e type,
		      void (*cbfn)(void *));
void bna_enet_pause_config(struct bna_enet_s *enet,
			   struct bna_pause_config_s *pause_config,
			   void (*cbfn)(struct bnad_s *));
void bna_enet_mtu_set(struct bna_enet_s *enet, int mtu,
		      void (*cbfn)(struct bnad_s *));
void bna_enet_perm_mac_get(struct bna_enet_s *enet, mac_t *mac);
void bna_enet_type_set(struct bna_enet_s *enet, enum bna_enet_type_e type);
enum bna_enet_type_e bna_enet_type_get(struct bna_enet_s *enet);


/**
 * IOCETH
 */


/* FW response handlers */
void bna_bfi_attr_get_rsp(struct bna_ioceth_s *ioceth,
			  struct bfi_msgq_mhdr_s *msghdr);

/* APIs for ENET, STATS_MOD etc */
void bna_ioceth_cb_stats_mod_stopped(struct bna_ioceth_s *ioceth);
void bna_ioceth_cb_enet_stopped(void *arg);

/* APIs for BNA */
void bna_ioceth_init(struct bna_ioceth_s *ioceth, struct bna_s *bna,
		     struct bna_res_info_s *res_info);
void bna_ioceth_uninit(struct bna_ioceth_s *ioceth);
bfa_boolean_t bna_ioceth_state_is_failed(struct bna_ioceth_s *ioceth);

/* APIs for BNAD */
void bna_ioceth_enable(struct bna_ioceth_s *ioceth);
void bna_ioceth_disable(struct bna_ioceth_s *ioceth,
			enum bna_cleanup_type_e type);


/**
 * BNA
 */


/* FW response handlers */
void bna_bfi_stats_get_rsp(struct bna_s *bna, struct bfi_msgq_mhdr_s *msghdr);
void bna_bfi_stats_clr_rsp(struct bna_s *bna, struct bfi_msgq_mhdr_s *msghdr);

/* APIs for RxF */
struct bna_mac_s * bna_cam_mod_mac_get(bfa_q_t *head);
void bna_cam_mod_mac_put(bfa_q_t *tail, struct bna_mac_s *mac);
struct bna_mcam_handle_s *bna_mcam_mod_handle_get(struct bna_mcam_mod_s *mod);
void bna_mcam_mod_handle_put(struct bna_mcam_mod_s *mcam_mod,
			  struct bna_mcam_handle_s *handle);
struct bna_wol_s *bna_wol_mod_magic_get(struct bna_wol_mod_s *ucam_mod);
void bna_wol_mod_magic_put(struct bna_wol_mod_s *wol_mod,
			  struct bna_wol_s *wol);
struct bna_wol_s *bna_wol_mod_frame_get(struct bna_wol_mod_s *ucam_mod);
void bna_wol_mod_frame_put(struct bna_wol_mod_s *wol_mod,
			  struct bna_wol_s *wol);

/* APIs for BNAD */
void bna_res_req(struct bna_res_info_s *res_info);
void bna_mod_res_req(struct bna_s *bna, struct bna_res_info_s *res_info);
void bna_init(struct bna_s *bna, struct bnad_s *bnad,
	      struct bna_ident_s *ident, struct bfa_pcidev_s *pcidev,
	      struct bna_res_info_s *res_info);
void bna_mod_init(struct bna_s *bna, struct bna_res_info_s *res_info);
void bna_uninit(struct bna_s *bna);
int bna_num_txq_set(struct bna_s *bna, int num_txq);
int bna_num_rxp_set(struct bna_s *bna, int num_rxp);
void bna_hw_stats_get(struct bna_s *bna);
void bna_hw_stats_clr(struct bna_s *bna);
void bna_trc_log_aen_set(struct bna_s *bna, struct bfa_trc_mod_s *trcmod,
			 struct bfa_log_mod_s *logm, struct bfa_aen_s *aen);
void bna_vnic_attr_get(struct bna_s *bna, struct bfa_vnic_attr_s *attr);
void bna_vnic_stats_get(struct bna_s *bna,
				struct bfa_vnic_stats_s *stats);
void bna_vnic_stats_clr(struct bna_s *bna);


/**
 * BNAD
 */


/* AEN callback */
void bnad_cb_aen_notify(void *bnad);

/* Callbacks for ENET */
void bnad_cb_ethport_link_status(struct bnad_s *bnad,
			      enum bna_link_status_e status);
void bnad_cb_bw_update(struct bnad_s *bnad, int bw);

/* Callbacks for IOCETH */
void bnad_cb_ioceth_ready(struct bnad_s *bnad);
void bnad_cb_ioceth_failed(struct bnad_s *bnad);
void bnad_cb_ioceth_disabled(struct bnad_s *bnad);
void bnad_cb_mbox_intr_enable(struct bnad_s *bnad);
void bnad_cb_mbox_intr_disable(struct bnad_s *bnad);

/* Callbacks for BNA */
void bnad_cb_stats_get(struct bnad_s *bnad, enum bna_cb_status_e status,
		       struct bna_stats_s *stats);

/* VNIC related */
void bnad_vnic_attr_get(struct bnad_s *bnad, struct bfa_vnic_attr_s *attr);
void bnad_vnic_stats_get(struct bnad_s *bnad, struct bfa_vnic_stats_s *stats);
void bnad_vnic_stats_clr(struct bnad_s *bnad);



/**
 * MISC
 */


/* Utility required by AEN */
void bnad_log_printf(struct bfa_log_mod_s *log_mod,
		     uint32_t msg_id, va_list params, const char *fmt, ...);

#endif  /* __BNA_H__ */
