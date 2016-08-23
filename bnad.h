/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corp.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BNAD_H__
#define __BNAD_H__

#include <bna.h>
#include <log/bfa_log_linux.h>
#include "bnad_compat.h"
#include "bfa_version.h"

/*
 * GLOBAL #defines (CONSTANTS)
 */
#define BNAD_NAME			"bna"
#define BNAD_NAME_LEN			64

#define BNAD_VERSION			BFA_VERSION

#define BNAD_MAILBOX_MSIX_INDEX		0
#define BNAD_MAILBOX_MSIX_VECTORS	1
#define BNAD_INTX_TX_IB_BITMASK		0x1
#define BNAD_INTX_RX_IB_BITMASK		0x2

#define BNAD_STATS_TIMER_FREQ		1000 	/* in msecs */

#define BNAD_IOCETH_TIMEOUT		10000

#define BNAD_MIN_Q_DEPTH		512
#define BNAD_MAX_RXQ_DEPTH		32768
#define BNAD_MAX_TXQ_DEPTH		2048

#define BNAD_RXQS_PER_CQ		2

#define BNAD_JUMBO_MTU			9000

#define BNAD_NETIF_WAKE_THRESHOLD	8

#define BNAD_RXQ_REFILL_THRESHOLD_SHIFT	3

/* Bit positions for tcb->flags */
#define BNAD_TXQ_FREE_SENT		0
#define BNAD_TXQ_TX_STARTED		1

/* Bit positions for rcb->flags */
#define BNAD_RXQ_STARTED		0
#define BNAD_RXQ_POST_OK		1

/* Resource limits */
#define BNAD_MAX_TXQ			(BNAD_MAX_TX * BNAD_MAX_TXQ_PER_TX)
#define BNAD_MAX_RXP			(BNAD_MAX_RX * BNAD_MAX_RXP_PER_RX)

#define BNAD_NUM_TXQ			(bnad->num_tx * bnad->num_txq_per_tx)
#define BNAD_NUM_RXP			(bnad->num_rx * bnad->num_rxp_per_rx)

#define BNAD_FRAME_SIZE(_mtu) \
	(ETH_HLEN + VLAN_HLEN + (_mtu) + ETH_FCS_LEN)

/*
 * DATA STRUCTURES
 */

/* enums */
enum bnad_intr_source_e {
	BNAD_INTR_TX		= 1,
	BNAD_INTR_RX		= 2
};

enum bnad_link_state_e {
	BNAD_LS_DOWN		= 0,
	BNAD_LS_UP 		= 1
};

struct bnad_aen_s {
	struct bfa_aen_s *aen;
	void *file_map[BFA_AEN_MAX_APP];
};

struct bnad_s;
struct bnad_diag_s;
struct bnad_ioctl_comp_s {
	struct bnad_s 		*bnad;
	struct completion 	comp;
	int 			comp_status;
};

struct bnad_completion_s {
	struct completion 	ioc_comp;
	struct completion 	ucast_comp;
	struct completion	mcast_comp;
	struct completion	tx_comp;
	struct completion	rx_comp;
	struct completion	stats_comp;
	struct completion	enet_comp;
	struct completion	mtu_comp;

	uint8_t			ioc_comp_status;
	uint8_t			ucast_comp_status;
	uint8_t			mcast_comp_status;
	uint8_t			tx_comp_status;
	uint8_t			rx_comp_status;
	uint8_t			stats_comp_status;
	uint8_t			port_comp_status;
	uint8_t			mtu_comp_status;
};

/* Tx Rx Control Stats */
struct bnad_drv_stats_s {
	uint64_t 		netif_queue_stop;
	uint64_t		netif_queue_wakeup;
	uint64_t 		netif_queue_stopped;
	uint64_t		tso4;
	uint64_t		tso6;
	uint64_t		tso_err;
	uint64_t		tcpcsum_offload;
	uint64_t		udpcsum_offload;
	uint64_t		csum_help;
	uint64_t		tx_skb_too_short;
	uint64_t		tx_skb_stopping;
	uint64_t		tx_skb_max_vectors;
	uint64_t		tx_skb_mss_too_long;
	uint64_t		tx_skb_tso_too_short;
	uint64_t		tx_skb_tso_prepare;
	uint64_t		tx_skb_non_tso_too_long;
	uint64_t		tx_skb_tcp_hdr;
	uint64_t		tx_skb_udp_hdr;
	uint64_t		tx_skb_csum_err;
	uint64_t		tx_skb_headlen_too_long;
	uint64_t		tx_skb_headlen_zero;
	uint64_t		tx_skb_frag_zero;
	uint64_t		tx_skb_len_mismatch;

	uint64_t		hw_stats_updates;
	uint64_t		netif_rx_dropped;
	uint64_t		lro_flushed;
	uint64_t		lro_aggregated;

	uint64_t		link_toggle;
	uint64_t		cee_toggle;

	uint64_t		rxp_info_alloc_failed;
	uint64_t		mbox_intr_disabled;
	uint64_t		mbox_intr_enabled;
	uint64_t		tx_unmap_q_alloc_failed;
	uint64_t		rx_unmap_q_alloc_failed;

	uint64_t		rxbuf_alloc_failed;
};

/* Complete driver stats */
struct bnad_stats_s {
	struct bnad_drv_stats_s	drv_stats;
	struct bna_stats_s 	*bna_stats;
};

/* Tx / Rx Resources */
struct bnad_tx_res_info_s {
	struct bna_res_info_s 	res_info[BNA_TX_RES_T_MAX];
};

struct bnad_rx_res_info_s {
	struct bna_res_info_s 	res_info[BNA_RX_RES_T_MAX];
};

struct bnad_tx_info_s {
	struct bna_tx_s		*tx; /* 1:1 between tx_info & tx */
	struct bna_tcb_s	*tcb[BNAD_MAX_TXQ_PER_TX];
	uint32_t		tx_id;
	struct delayed_work	tx_cleanup_work;
} ____cacheline_aligned;

struct bnad_rx_info_s {
	struct bna_rx_s 	*rx; /* 1:1 between rx_info & rx */

	struct bnad_rx_ctrl_s	rx_ctrl[BNAD_MAX_RXP_PER_RX];
	uint32_t		rx_id;
	struct work_struct	rx_cleanup_work;
} ____cacheline_aligned;

struct bnad_tx_vector_s {
	BNAD_DMA_UNMAP_ADDR()
	BNAD_DMA_UNMAP_LEN
};

struct bnad_tx_unmap_s {
	struct sk_buff		*skb;
	uint32_t		nvecs;
	struct bnad_tx_vector_s	vectors[BFI_TX_MAX_VECTORS_PER_WI];
};

struct bnad_rx_vector_s {
	BNAD_DMA_UNMAP_ADDR()
	uint32_t		len;
} __attribute__((packed));

struct bnad_rx_unmap_s {
	struct page		*page;
	struct sk_buff		*skb;
	struct bnad_rx_vector_s	vector;
	uint32_t		page_offset;
};

enum bnad_rxbuf_type {
	BNAD_RXBUF_NONE		= 0,
	BNAD_RXBUF_SK_BUFF 	= 1,
	BNAD_RXBUF_PAGE		= 2,
	BNAD_RXBUF_MULTI_BUFF	= 3
};

#define BNAD_RXBUF_IS_SK_BUFF(_type)	((_type) == BNAD_RXBUF_SK_BUFF)
#define BNAD_RXBUF_IS_MULTI_BUFF(_type)	((_type) == BNAD_RXBUF_MULTI_BUFF)

struct bnad_rx_unmap_q_s {
	int32_t			reuse_pi;
	int32_t			alloc_order;
	uint32_t		map_size;
	enum bnad_rxbuf_type	type;
	struct bnad_rx_unmap_s	unmap[0] ____cacheline_aligned;
};

#define BNAD_PCI_DEV_IS_CAT2(_bnad) \
	((_bnad)->pcidev->device == PCI_DEVICE_ID_BROCADE_CATAPULT2)

/* Defines for run_flags bit-mask */
/* Set, tested & cleared using xxx_bit() functions */
/* Values indicated bit positions */
#define	BNAD_RF_CEE_RUNNING		0
#define BNAD_RF_MTU_SET 		1
#define BNAD_RF_MBOX_IRQ_DISABLED	2
#define BNAD_RF_NETDEV_REGISTERED	3
#define BNAD_RF_STATS_TIMER_RUNNING	4
#define BNAD_RF_TX_PRIO_INV_NETQ	5
#define	BNAD_RF_TX_PRIO_SET		6
#define	BNAD_RF_TX_TIMEOUT		7
#define BNAD_RF_IOCTL_ENABLED		8
#define BNAD_RF_RBL_TIMER_RUNNING	9


/* Define for Fast Path flags */
/* Defined as bit positions */
#define BNAD_FP_LRO			0

struct bnad_s {
	struct net_device 	*netdev;

	/* Data path */
	struct bnad_tx_info_s	tx_info[BNAD_MAX_TX];
	struct bnad_rx_info_s	rx_info[BNAD_MAX_RX];

	BNAD_DECLARE_RBL()

	BNAD_ACTIVE_VLANS()
	/*
	 * These q numbers are global only because
	 * they are used to calculate MSIx vectors.
	 * Actually the exact # of queues are per Tx/Rx
	 * object.
	 */
	uint32_t		num_tx;
	uint32_t		num_rx;
	uint32_t		num_txq_per_tx;
	uint32_t		num_rxp_per_rx;

	BNAD_DECLARE_NETQ()

	uint32_t		txq_depth;
	uint32_t		rxq_depth;

	uint8_t			tx_coalescing_timeo;
	uint8_t			rx_coalescing_timeo;

	struct bna_rx_config_s	rx_config[BNAD_MAX_RX] ____cacheline_aligned;
	struct bna_tx_config_s	tx_config[BNAD_MAX_TX] ____cacheline_aligned;

	uint32_t		rx_csum;

	void __iomem		*bar0;	/* BAR0 address */

	struct bna_s		bna;

	uint32_t		cfg_flags;
	unsigned long		run_flags;

	struct pci_dev 		*pcidev;
	uint64_t		mmio_start;
	uint64_t		mmio_len;

	uint32_t		msix_num;
	struct msix_entry	*msix_table;

	struct mutex		conf_mutex;
	spinlock_t		bna_lock ____cacheline_aligned;

	/* Timers */
	struct timer_list	ioc_timer;
	struct timer_list	stats_timer;

	/* Control path resources, memory & irq */
	struct bna_res_info_s	res_info[BNA_RES_T_MAX];
	struct bna_res_info_s	mod_res_info[BNA_MOD_RES_T_MAX];
	struct bnad_tx_res_info_s	tx_res_info[BNAD_MAX_TX];
	struct bnad_rx_res_info_s	rx_res_info[BNAD_MAX_RX];

	struct bnad_completion_s bnad_completions;

	/* Burnt in MAC address */
	mac_t			perm_addr;

	struct workqueue_struct *work_q;
	BNAD_DECLARE_NETQ_WORK();

	/* Log, Trace, AEN */
	struct bfa_trc_mod_s 	*trcmod;
	struct bfa_log_mod_s 	*logmod;
	struct bnad_aen_s 	aen;
	uint8_t			ref_count;

	/* Statistics */
	struct bnad_stats_s	stats;
	struct net_device_stats net_stats;

	struct bnad_diag_s	*diag;

	char			adapter_name[BNAD_NAME_LEN];
	char 			port_name[BNAD_NAME_LEN];
	char			mbox_irq_name[BNAD_NAME_LEN];
	char			wq_name[BNAD_NAME_LEN];

	struct bna_ident_s	ident;
	struct list_head	list_entry;

	/* debugfs specific data */
	char		*regdata;
	uint32_t	reglen;
	struct dentry *bnad_dentry_files[5];
	struct dentry *port_debugfs_root;

	/* ethtool set_eeprom specific */
	void    *flash_buffer;
	uint32_t total_flashsz;
	uint32_t cur_flash_off;
	uint32_t cur_flash_part;
};

struct bnad_drvinfo {
	struct bfa_ioc_attr_s	ioc_attr;
	struct bfa_cee_attr_s	cee_attr;
	bfa_flash_attr_t	flash_attr;
	uint32_t	cee_status;
	uint32_t	flash_status;
};

/*
 * EXTERN VARIABLES
 */
extern struct mutex		bnad_list_mutex;
extern struct list_head 	bnad_list;

/*
 * EXTERN PROTOTYPES
 */
/* Netdev entry point prototypes */
extern int bnad_open(struct net_device *netdev);
extern int bnad_stop(struct net_device *netdev);
extern int bnad_start_xmit(struct sk_buff *skb, struct net_device *netdev);
extern void bnad_set_rx_mode(struct net_device *netdev);
extern struct net_device_stats *bnad_get_netdev_stats(
				struct net_device *netdev);
extern int bnad_set_mac_address(struct net_device *netdev, void *mac_addr);
extern int bnad_mac_addr_set_locked(struct bnad_s *bnad, uint8_t *mac_addr);
extern int bnad_enable_default_bcast(struct bnad_s *bnad);

extern int bnad_change_mtu(struct net_device *netdev, int new_mtu);
extern int bnad_poll(struct net_device *netdev, int *budget);
extern void bnad_netpoll(struct net_device *netdev);
extern void bnad_set_ethtool_ops(struct net_device *netdev);
extern uint32_t bnad_reinit_rx(struct bnad_s *bnad);

/* Configuration & setup */
extern struct bnad_s *bnad_get_bnadev(int bna_id);

extern void bnad_enable_mbox_irq(struct bnad_s *bnad);
extern void bnad_disable_mbox_irq(struct bnad_s *bnad);

extern void bnad_tx_coalescing_timeo_set(struct bnad_s *bnad);
extern void bnad_rx_coalescing_timeo_set(struct bnad_s *bnad);

extern void bnad_q_num_init(struct bnad_s *bnad);
extern void bnad_q_num_adjust(struct bnad_s *bnad, uint32_t max_txq,
		uint32_t max_rxq);

extern int bnad_mem_alloc(struct bnad_s *bnad,
				struct bna_mem_info_s *mem_info);
extern void bnad_mem_free(struct bnad_s *bnad,
				struct bna_mem_info_s *mem_info);

extern int bnad_setup_rx(struct bnad_s *bnad, uint32_t rx_id);
extern int bnad_setup_tx(struct bnad_s *bnad, uint32_t tx_id);
extern void bnad_destroy_tx(struct bnad_s *bnad, uint32_t tx_id);
extern void bnad_destroy_rx(struct bnad_s *bnad, uint32_t rx_id);

extern int bnad_add_mac_address_locked(struct bnad_s *bnad,
					uint8_t *mac_addr, uint32_t rxq_id);
extern int bnad_del_mac_address_locked(struct bnad_s *bnad,
					uint8_t *mac_addr, uint32_t rxq_id);
extern int bnad_mtu_set(struct bnad_s *bnad, int mtu);

extern int bnad_tx_prio_set(struct bnad_s *bnad, uint32_t tx_id,
			    uint8_t priority);

/* Timer start/stop protos */
extern void bnad_stats_timer_start(struct bnad_s *bnad);
extern void bnad_stats_timer_stop(struct bnad_s *bnad);

/* Datapath */
extern void bnad_disable_txrx_irqs(struct bnad_s *bnad);
extern void bnad_enable_txrx_irqs(struct bnad_s *bnad);
extern uint32_t bnad_cq_process(struct bnad_s *bnad, struct bna_ccb_s *ccb,
				int budget);
extern uint32_t bnad_tx_complete(struct bnad_s *bnad, struct bna_tcb_s *tcb);
extern irqreturn_t bnad_isr(int irq, void *data);

/* Used by IOCTL */
extern void bnad_ioctl_init(void);
extern void bnad_ioctl_exit(void);
extern void bnad_cb_port_disabled(void *arg, enum bna_cb_status_e status);
extern int bnad_ioceth_disable(struct bnad_s *bnad);
extern int bnad_ioceth_enable(struct bnad_s *bnad);

/* Statistics */
extern void bnad_netdev_qstats_fill(struct bnad_s *bnad);
extern void bnad_netdev_hwstats_fill(struct bnad_s *bnad);
extern int bnad_hw_stats_get(struct bnad_s *bnad);
extern void bnad_drv_stats_get(struct bnad_s *bnad,
			struct bnad_ethport_stats_s *stats);
extern int bnad_stats_clr(struct bnad_s *bnad);

/* Logging */
void bnad_log_printf(struct bfa_log_mod_s *log_mod,
		     uint32_t msg_id, va_list params, const char *fmt, ...);
extern uint32_t bnad_get_loglevel(struct bnad_s *bnad, int16_t mod_id);
extern void bnad_set_loglevel(struct bnad_s *bnad, u32 msglevel,
				int16_t mod_id);

/* Debugfs */
void	bnad_debugfs_init(struct bnad_s *bnad);
void	bnad_debugfs_uninit(struct bnad_s *bnad);

/**
 * MACROS
 */
/* To set & get the stats counters */
#define BNAD_UPDATE_CTR(_bnad, _ctr)				\
				(((_bnad)->stats.drv_stats._ctr)++)

#define BNAD_GET_CTR(_bnad, _ctr) ((_bnad)->stats.drv_stats._ctr)

#define bnad_spin_lock(_flags)		\
	spin_lock_irqsave(&bnad->bna_lock, (_flags))
#define bnad_spin_unlock(_flags)	\
	spin_unlock_irqrestore(&bnad->bna_lock, (_flags))

#define bnad_list_lock()	mutex_lock(&bnad_list_mutex)
#define bnad_list_unlock()	mutex_unlock(&bnad_list_mutex)

#define bnad_pci_resource_len(_bnad)	\
			pci_resource_len((_bnad)->pcidev, 0)

#define bnad_get_stack_name(_bnad)	((_bnad)->netdev->name)

static inline char *
bnad_get_stack_mac(struct bnad_s *bnad)
{
	if (is_zero_ether_addr(&bnad->perm_addr.mac[0])) {
		bna_enet_perm_mac_get(&bnad->bna.enet, &bnad->perm_addr);
		bnad_set_netdev_perm_addr(bnad);
	}
	return bnad->netdev->dev_addr;
}

#define bnad_enable_rx_irq_unsafe(_ccb)			\
{							\
	if (likely(test_bit(BNAD_RXQ_STARTED, &(_ccb)->rcb[0]->flags))) {\
		bna_ib_coalescing_timer_set((_ccb)->i_dbell,	\
			(_ccb)->rx_coalescing_timeo);		\
		bna_ib_ack((_ccb)->i_dbell, (_ccb)->pkts_una);	\
		(_ccb)->pkts_una = 0;				\
	}							\
}

#define bnad_init_completion(_bnad, _comp)		\
{							\
	(_comp)->bnad = (_bnad);			\
	(_comp)->comp_status = 0;			\
	init_completion(&((_comp)->comp));		\
}

#define bnad_wait_for_completion(_bnad, _comp, _flags)	\
{							\
	spin_unlock_irqrestore(&((_bnad)->bna_lock), (_flags));\
	wait_for_completion(&((_comp)->comp));		\
	spin_lock_irqsave(&((_bnad)->bna_lock), (_flags));\
}

/**
 * INLINE FUNCTIONS
 */

/*
 * Deep Inspection : Checks if packet is ISCSI based on
 * standard iSCSI port
 */
#define BNAD_TCP_ISCSI_PORT	3260
#define BNAD_IS_ISCSI_PKT(_tch)					\
(((_tch)->source == ntohs(BNAD_TCP_ISCSI_PORT)) ||		\
		((_tch)->dest == ntohs(BNAD_TCP_ISCSI_PORT)))

static inline bool bnad_is_iscsi(struct sk_buff *skb)
{
	uint16_t		proto = 0;
	struct tcphdr		*th;

	if (skb->protocol == htons(ETH_P_IP))
		proto = ip_hdr(skb)->protocol;
#ifdef NETIF_F_IPV6_CSUM
	else if (skb->protocol == htons(ETH_P_IPV6))
		/* nexthdr may not be TCP immediately. */
		proto = ipv6_hdr(skb)->nexthdr;
#endif
	if (proto == IPPROTO_TCP) {
		th = tcp_hdr(skb);
		if (BNAD_IS_ISCSI_PKT(th))
			return true;
	}

	return false;
}

/*
 * Ethdiag loopback
 */

#define BNAD_DIAG_LB_TX_INTR			0xe
#define BNAD_DIAG_LB_RX_INTR			0xf

#define bnad_diag_lb_intr_type_get(_bnad)			\
	(((_bnad)->cfg_flags & BNAD_CF_MSIX) ?			\
		BNA_INTR_T_MSIX : BNA_INTR_T_INTX)
#define bnad_diag_lb_tx_intr_get(_bnad)			\
	(((_bnad)->cfg_flags & BNAD_CF_MSIX) ? 0 : BNAD_DIAG_LB_TX_INTR)
#define bnad_diag_lb_rx_intr_get(_bnad)			\
	(((_bnad)->cfg_flags & BNAD_CF_MSIX) ? 0 : BNAD_DIAG_LB_RX_INTR)

#define bnad_diag_lb_msix_vector_get(_bnad, _idx)	\
	((_bnad)->msix_table[(_idx)].vector)

#define bnad_diag_lb_mem_alloc(_bnad, _mem_info)	\
	bnad_mem_alloc((_bnad), (_mem_info))

#define bnad_diag_lb_mem_free(_bnad, _mem_info)		\
	bnad_mem_free((_bnad), (_mem_info))

static inline void
bnad_diag_lb_txrx_irq_free(struct bnad_s *bnad,
			   struct bna_intr_info_s *intr_info)
{
	cna_os_kfree(intr_info->idl, sizeof(*(intr_info->idl)));
	intr_info->idl = NULL;
}

#define bnad_diag_lb_tx_irq_free(_bnad, _intr_info)	\
	bnad_diag_lb_txrx_irq_free((_bnad), (_intr_info))

#define bnad_diag_lb_rx_irq_free(_bnad, _intr_info)	\
	bnad_diag_lb_txrx_irq_free((_bnad), (_intr_info))

#define BNAD_HIGH_THROUGHPUT_THRESH	100		/* Bytes per us */
#define BNAD_RX_COALESCE_TIMEO_MAX	12		/* 12 * 5 = 60us */
#define BNAD_RX_COALESCE_TIMEO_MIN	1		/* 1 * 5 = 5us */
static inline void
bnad_rx_intr_rate_adjust(struct bnad_rx_ctrl_s *rx_ctrl)
{
	struct bna_ccb_s *ccb = rx_ctrl->ccb;
	int bytes_per_us;

	bytes_per_us = (ccb->bytes_per_intr /
		(ccb->rx_coalescing_timeo * BFI_COALESCING_TIMER_UNIT));
	ccb->bytes_per_intr = 0;
	if (bytes_per_us < BNAD_HIGH_THROUGHPUT_THRESH) {
		if (ccb->rx_coalescing_timeo > BNAD_RX_COALESCE_TIMEO_MIN)
			ccb->rx_coalescing_timeo--;
	} else {
		if (ccb->rx_coalescing_timeo < BNAD_RX_COALESCE_TIMEO_MAX)
			ccb->rx_coalescing_timeo++;
	}
}

#endif /* __BNAD_H__ */
