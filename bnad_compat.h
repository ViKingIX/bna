/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corp.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BNAD_COMPAT_H__
#define __BNAD_COMPAT_H__

#include <linux/rtnetlink.h>
#include <linux/workqueue.h>
#include <linux/ipv6.h>
#include <linux/etherdevice.h>
#include <linux/mutex.h>

/* Fix for IA64 */
#include <asm/checksum.h>
#include <net/ip6_checksum.h>

#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 1, 0)
#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/if_vlan.h>

#define BNAD_NEW_VLAN_INTERFACE
#endif

/*
 * NETIF_F_GRO & Kernel Version > 2.6.18 ensures kernel GRO support
 * BNAD_GRO_ENABLED ensures we want to support this in the driver
 */
#if defined(NETIF_F_GRO) && LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 18) && \
					defined(BNAD_GRO_ENABLED)
#define BNAD_GRO 1
#elif defined(BNAD_MAKE_LRO) && defined(NETIF_F_LRO) && defined(CONFIG_INET_LRO)
#define BNAD_LRO 1
#else
#endif

#if defined(BNAD_GRO)
extern uint bnad_gro_disable;
extern uint bnad_multi_buffer_rx;
#elif defined(BNAD_LRO)
#include <linux/inet_lro.h>

extern uint bnad_lro_disable;
#else
#include <net/ip.h>
#include <net/tcp.h>
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 24)
#define BNAD_NEW_NAPI
#endif

#define BNAD_TXQ_DEPTH		2048
#define BNAD_RXQ_DEPTH		2048

#define BNAD_MAX_TX		1
#define BNAD_MAX_TXQ_PER_TX	8

#define BNAD_MAX_RX		1
#define BNAD_MAX_RXP_PER_RX	16
#define BNAD_MAX_RXQ_PER_RXP	2

/* Bit mask values for bnad->cfg_flags */
#define	BNAD_CF_DIM_ENABLED		0x01	/* DIM */
#define	BNAD_CF_PROMISC			0x02
#define BNAD_CF_ALLMULTI		0x04
#define BNAD_CF_DEFAULT			0x08
#define	BNAD_CF_MSIX			0x10	/* If in MSIx mode */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 18)
#define bool	uint
enum {
	false = 0,
	true
};
#endif

#ifndef IRQF_SHARED
#define IRQF_SHARED SA_SHIRQ
#endif

#ifndef ETH_FCS_LEN
#define ETH_FCS_LEN 4
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 14)
#ifndef mutex_destroy
#define mutex_destroy(_mutex)		do { } while (0)
#endif

static inline void
pci_intx(struct pci_dev *pdev, int enable)
{
	u16 pci_command, new;

	pci_read_config_word(pdev, PCI_COMMAND, &pci_command);

	if (enable) {
		new = pci_command & ~PCI_COMMAND_INTX_DISABLE;
	} else {
		new = pci_command | PCI_COMMAND_INTX_DISABLE;
	}

	if (new != pci_command) {

		pci_write_config_word(pdev, PCI_COMMAND, new);
	}
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 19)
typedef irqreturn_t (*bnad_isr_t)(int, void *, struct pt_regs *);
#else
typedef irq_handler_t bnad_isr_t;
#endif

/*
 * Control structure pointed to ccb->ctrl, which
 * determines the NAPI / LRO behavior CCB
 * There is 1:1 corres. between ccb & ctrl
 */
struct bnad_rx_ctrl_s {
	struct bna_ccb_s	*ccb;
	struct bnad_s		*bnad;
	unsigned long		flags;
#ifdef BNAD_LRO
	struct net_lro_mgr	lro;
#endif
#if defined(BNAD_NEW_NAPI) || defined(BNAD_GRO)
	struct napi_struct	napi;
#endif
	uint64_t		rx_intr_ctr;
	uint64_t		rx_poll_ctr;
	uint64_t		rx_schedule;
	uint64_t		rx_keep_poll;
	uint64_t		rx_complete;
};

#define BNAD_DECLARE_RBL()
#define BNAD_CHECK_RBL(BNAD) 0

#ifndef BNAD_NEW_VLAN_INTERFACE
#define BNAD_ACTIVE_VLANS() \
	struct vlan_group *vlan_grp;
#else
#define BNAD_ACTIVE_VLANS() \
	unsigned long active_vlans[BITS_TO_LONGS(VLAN_N_VID)];
#endif

/*
 * External variables & functions
 */

#define BNAD_RBL_INIT(bnad)
#define BNAD_RBL_UNINIT(bnad)
#define BNAD_RBL_BW_UPDATE(bnad, bw)
#define BNAD_PKT_DROP(bnad, pktsz)	(0)

#ifdef BNAD_NEW_NAPI

#define BNAD_NAPI_POLL_QUOTA			64
extern int
bnad_napi_poll_rx(struct napi_struct *napi, int budget);

#define BNAD_NAPI_ADD(_bnad, _rx_id)			\
do {							\
	int i;						\
	for (i = 0; i < (_bnad)->num_rxp_per_rx; i++)	\
		netif_napi_add((_bnad)->netdev, 	\
			&(_bnad)->rx_info[(_rx_id)].rx_ctrl[i].napi, \
			bnad_napi_poll_rx, BNAD_NAPI_POLL_QUOTA);\
} while (0)

#define BNAD_NAPI_ENABLE(_bnad, _napi)	\
	napi_enable((_napi))

#define BNAD_NAPI_DISABLE(_bnad, _napi)	\
	napi_disable((_napi))

#define BNAD_NAPI_DEL(_bnad, _rx_id)			\
do {							\
	int i;						\
	for (i = 0; i < (_bnad)->num_rxp_per_rx; i++)	\
		netif_napi_del(&(_bnad)->rx_info[(_rx_id)].rx_ctrl[i].napi);\
} while (0)

#define BNAD_COMPAT_NAPI_ENABLE(_bnad, _napi) {}

#else /* ! BNAD_NEW_NAPI */

#if defined(BNAD_GRO)
#define BNAD_NAPI_ADD(_bnad, _rx_id)			\
do {							\
	int i;						\
	for (i = 0; i < (_bnad)->num_rxp_per_rx; i++)	\
		(_bnad)->rx_info[(_rx_id)].rx_ctrl[i].napi.dev = \
						(_bnad)->netdev; \
} while (0)
#else
#define BNAD_NAPI_ADD(_bnad, _rx_id)
#endif

#define BNAD_NAPI_ENABLE(_bnad, _napi)		\
	netif_poll_enable((_bnad->netdev));

#define BNAD_NAPI_DISABLE(_bnad, _napi)	\
	netif_poll_disable((_bnad->netdev));

#define BNAD_NAPI_DEL(_bnad, _rx_id)

#define BNAD_COMPAT_NAPI_ENABLE(_bnad, _napi) BNAD_NAPI_ENABLE(_bnad, _napi)

#endif

extern int bnad_rxq_gro_init(struct bnad_s *bnad, struct bna_rcb_s *rcb);
extern void bnad_vlan_bit_set(unsigned short vid, struct bnad_s *bnad);
extern void bnad_vlan_bit_clear(unsigned short vid, struct bnad_s *bnad);
#ifndef BNAD_NEW_VLAN_INTERFACE
extern void bnad_vlan_rx_register(struct net_device *netdev,
	struct vlan_group *vlan_grp);
#endif
extern void bnad_restore_vlans(struct bnad_s *bnad, uint32_t rx_id);

#ifdef BNAD_LRO
#define BNAD_LRO_MAX_DESC	8
#define BNAD_LRO_MAX_AGGR	64
#endif
extern int bnad_lro_init(struct bnad_s *bnad, uint32_t rx_id);
extern void bnad_lro_uninit(struct bnad_s *bnad, uint32_t rx_id);

extern void bnad_lro_stats_fill(struct bnad_s *bnad);
extern void bnad_lro_stats_clear(struct bnad_s *bnad);

#define bnad_vlan_rx_config(bnad)
#define bnad_set_vlan_strip_status(_bnad, _rx_config) \
	(_rx_config)->vlan_strip_status = BNA_STATUS_T_ENABLED

#define BNAD_DECLARE_NETQ()
#define BNAD_INIT_NETQ_WORK(_bnad)
#define BNAD_UNINIT_NETQ_WORK(_bnad)

#define bnad_conf_lock()	mutex_lock(&bnad->conf_mutex)
#define bnad_conf_unlock()	mutex_unlock(&bnad->conf_mutex)

extern void bnad_netdev_init(struct bnad_s *bnad, bool using_dac);
extern void bnad_set_netdev_perm_addr(struct bnad_s *bnad);
extern int bnad_tso_prepare(struct bnad_s *bnad, struct sk_buff *skb);
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 14, 0)
extern uint16_t bnad_tx_select_queue(struct net_device *netdev,
				     struct sk_buff *skb);
#else
extern uint16_t bnad_tx_select_queue(struct net_device *netdev,
				     struct sk_buff *skb,
				     void *accel_priv,
				     select_queue_fallback_t fallback);
#endif
extern void bnad_netif_rx_schedule_poll(struct bnad_s *bnad,
					struct bna_ccb_s *ccb);

#if defined(BNAD_GRO) && LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
#define bnad_enable_multi_buffer \
	(!bnad_gro_disable && bnad_multi_buffer_rx)
#else
#define bnad_enable_multi_buffer	(0)
#endif

#define bnad_clear_pdev_netdev(_pdev)

#define BNAD_RXMODE_PROMISC_DEFAULT	BNA_RXMODE_PROMISC

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 20)
#define bnad_work_func_t	work_func_t

#define BNAD_INIT_WORK(_work, _func, _data)	\
		INIT_WORK((_work), (_func))
#else
typedef void (*bnad_work_func_t)(void *);

#define BNAD_INIT_WORK(_work, _func, _data)	\
		INIT_WORK((_work), (_func), (_data))
#endif

#ifdef INIT_DELAYED_WORK
#define BNAD_INIT_DELAYED_WORK(_work, _func)	\
		INIT_DELAYED_WORK((_work), (_func))

#define BNAD_QUEUE_DELAYED_WORK(_work_q, _work, _delay) \
		queue_delayed_work((_work_q), (_work), (_delay))
#else
struct delayed_work {
	struct work_struct work;
};

#define BNAD_INIT_DELAYED_WORK(_work, _func)	\
		INIT_WORK(&(_work)->work, (_func), (_work))

#define BNAD_QUEUE_DELAYED_WORK(_work_q, _work, _delay) \
		queue_delayed_work((_work_q), &(_work)->work, (_delay))
#endif

#define BNAD_FLUSH_WORKQUEUE(_bnad)		\
		flush_workqueue((_bnad)->work_q)

#define BNAD_DECLARE_NETQ_WORK()
#define BNAD_CANCEL_NETQ_WORK_SYNC(_bnad)

#define bnad_work_netq_reinit_tx_prio_set(_bnad)

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 16)
#define setup_timer(_timer, _fn, _data)		\
{						\
	(_timer)->function = (_fn);		\
	(_timer)->data = (_data);		\
	init_timer((_timer));			\
}
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 15, 0)
#define bnad_smp_mb() smp_mb__before_atomic()
#else
#define bnad_smp_mb() smp_mb__before_clear_bit()
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 30)
#define BNAD_DMA_MASK(_x) DMA_BIT_MASK(_x)
#else
#define BNAD_DMA_MASK(_x) DMA_##_x##BIT_MASK
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 29)
#define bnad_rx_schedule_prep(__netdev, _napi)          \
		napi_schedule_prep((_napi))
#define __bnad_rx_schedule(_netdev, _napi)              \
		__napi_schedule((_napi))
#define bnad_rx_complete(_netdev, _napi)                \
		napi_complete((_napi))
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 24)
#define bnad_rx_schedule_prep(_netdev, _napi)           \
		netif_rx_schedule_prep((_netdev), (_napi))
#define __bnad_rx_schedule(_netdev, _napi)              \
		__netif_rx_schedule((_netdev), (_napi))
#define bnad_rx_complete(_netdev, _napi)                \
		netif_rx_complete((_netdev), (_napi))
#else
#define bnad_rx_schedule_prep(_netdev, _napi)          \
		netif_rx_schedule_prep((_netdev))
#define __bnad_rx_schedule(_netdev, _napi)             \
		__netif_rx_schedule((_netdev))
#define bnad_rx_complete(_netdev, _napi)               \
		netif_rx_complete((_netdev))
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 32)
#define netdev_tx_t int
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 31)
#define bnad_netdev_trans_start_set(_netdev)		\
	(_netdev)->trans_start = jiffies
#else
#define bnad_netdev_trans_start_set(_netdev)
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 29)
#define bnad_netdev_last_rx_set(_netdev)		\
	(_netdev)->last_rx = jiffies
#else
#define bnad_netdev_last_rx_set(_netdev)
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 12)
static inline int
is_zero_ether_addr(const u8 *addr)
{
	return !(addr[0] | addr[1] | addr[2] | addr[3] | addr[4] | addr[5]);
}
#endif

#ifdef BNAD_NEW_NAPI
#define BNAD_GET_NUM_RXP_PER_RX()	\
	(min((uint32_t)num_online_cpus(), (uint32_t)(BNAD_MAX_RXP_PER_RX)))
#else
#define BNAD_GET_NUM_RXP_PER_RX()	1
#endif

#ifndef CHECKSUM_PARTIAL
#define CHECKSUM_PARTIAL CHECKSUM_HW
#endif

#ifndef SKB_DATAREF_SHIFT
#define skb_header_cloned(_skb) 0
#endif

#ifndef NETIF_F_GSO
#define gso_size tso_size
#endif

#ifdef NETIF_F_TSO
#define skb_is_gso bnad_skb_is_gso
static inline int
bnad_skb_is_gso(const struct sk_buff *skb)
{
	return skb_shinfo(skb)->gso_size;
}
#else
#define skb_is_gso(skb)	0
#endif

#ifndef VLAN_GROUP_ARRAY_LEN
#define VLAN_GROUP_ARRAY_LEN VLAN_N_VID
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 21)
#define vlan_group_get_device(vg, vlan_id)	(vg->vlan_devices[vlan_id])
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 22)
#define ip_hdr(_skb)		((_skb)->nh.iph)
#define ip_hdrlen(_skb)		((_skb)->nh.iph->ihl * 4)
#define ipv6_hdr(_skb)		((_skb)->nh.ipv6h)
#define tcp_hdr(_skb) 		((_skb)->h.th)
#define tcp_hdrlen(_skb)	((_skb)->h.th->doff << 2)

#define skb_reset_network_header(_skb)	do {	\
	(_skb)->nh.raw = (_skb)->data;		\
} while (0)
#define skb_transport_offset(_skb)	((_skb)->h.raw - (skb)->data)
#define skb_set_transport_header(_skb, _offset) do {	\
	(_skb)->h.raw = (_skb)->data + (_offset);	\
} while (0)
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 35)
#define BNAD_VLAN_PRIO_MASK	0x0e000
#define BNAD_VLAN_PRIO_SHIFT	13
#else
#define BNAD_VLAN_PRIO_MASK	VLAN_PRIO_MASK
#define BNAD_VLAN_PRIO_SHIFT	VLAN_PRIO_SHIFT
#endif

#ifdef BNAD_NEW_VLAN_INTERFACE
#define bnad_vlan_group(_bnad)	(NULL)
#else
#define bnad_vlan_group(_bnad)	((_bnad)->vlan_grp)
#endif

static inline void
bnad_rcv_skb(struct bnad_rx_ctrl_s *rx_ctrl, struct sk_buff *skb,
		uint32_t cmpl_flags, uint32_t cfg_flags,
		void *vlan_grp, uint16_t nalv_tag, void *cmpl)
{
	if ((cmpl_flags & BNA_CQ_EF_VLAN)) {
#ifndef BNAD_NEW_VLAN_INTERFACE
		bool stripped = (!(cfg_flags & BNAD_CF_PROMISC));

		if (stripped && vlan_grp) {
#if defined(BNAD_GRO)
			if (!bnad_gro_disable) {
				vlan_gro_frags(&rx_ctrl->napi,
						vlan_grp, ntohs(nalv_tag));
				return;
			}
#elif defined(BNAD_LRO)
			if (!bnad_lro_disable &&
				skb->ip_summed == CHECKSUM_UNNECESSARY) {
				lro_vlan_hwaccel_receive_skb(&rx_ctrl->lro, skb,
					vlan_grp, ntohs(nalv_tag), cmpl);
				return;
			}
#endif
			vlan_hwaccel_receive_skb(skb, vlan_grp,
					ntohs(nalv_tag));
			return;
		}
#else	/* BNAD_NEW_VLAN_INTERFACE */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
		__vlan_hwaccel_put_tag(skb, htons(ETH_P_8021Q),
				ntohs(nalv_tag));
#else
		__vlan_hwaccel_put_tag(skb, ntohs(nalv_tag));
#endif
#endif	/* !BNAD_NEW_VLAN_INTERFACE */
	}

#if defined(BNAD_GRO)
	if (!bnad_gro_disable) {
		napi_gro_frags(&rx_ctrl->napi);
		return;
	}
#elif defined(BNAD_LRO)
	if (!bnad_lro_disable &&
			skb->ip_summed == CHECKSUM_UNNECESSARY) {
		lro_receive_skb(&rx_ctrl->lro, skb, cmpl);
		return;
	}
#endif
	netif_receive_skb(skb);
}

static inline void
bnad_rcv_flush(struct bnad_rx_ctrl_s *rx_ctrl)
{
#if defined(BNAD_GRO)
	if (!bnad_gro_disable)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 7, 0)
		napi_gro_flush(&rx_ctrl->napi, false);
#else
		napi_gro_flush(&rx_ctrl->napi);
#endif
#elif defined(BNAD_LRO)
	if (!bnad_lro_disable)
		lro_flush_all(&rx_ctrl->lro);
#endif
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35)
#define bnad_netdev_mc_count(_netdev) (netdev_mc_count((_netdev)))

#define bnad_netdev_mc_list_is_empty(_netdev) 		\
		(netdev_mc_empty((_netdev)))

static inline void
bnad_netdev_mc_list_get(struct net_device *netdev, uint8_t *mc_list)
{
	int mc_count = bnad_netdev_mc_count(netdev);
	int i = 1; /* Index 0 has broadcast address */
	struct netdev_hw_addr *mc_addr;

	CNA_ASSERT_DEBUG(mc_count);
	CNA_ASSERT_DEBUG(mc_list);

	netdev_for_each_mc_addr(mc_addr, netdev) {
		CNA_ASSERT_DEBUG(mc_addr);
		memcpy(&mc_list[i * CNA_ETH_ALEN], &mc_addr->addr[0],
							CNA_ETH_ALEN);
		i++;
	}
}
#else /* KERNEL VERSION < 2.6.35 */
#define bnad_netdev_mc_count(_netdev) ((_netdev)->mc_count)

#define bnad_netdev_mc_list_is_empty(_netdev) 		\
			(!((_netdev)->mc_count))

static inline void
bnad_netdev_mc_list_get(struct net_device *netdev, uint8_t *mc_list)
{
	int mc_count = bnad_netdev_mc_count(netdev);
	int i = 1; /* Index 0 has broadcast address */
	struct dev_mc_list *mc;

	CNA_ASSERT_DEBUG(mc_count);
	CNA_ASSERT_DEBUG(mc_list);

	mc = netdev->mc_list;
	for (i = 1; mc && i < mc_count + 1; i++, mc = mc->next)
		memcpy(&mc_list[i * CNA_ETH_ALEN],
			mc->dmi_addr, CNA_ETH_ALEN);
}
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
#define BNAD_MACVLAN
#endif

#ifdef BNAD_MACVLAN

#ifndef netdev_uc_count
#define netdev_uc_count(dev) ((dev)->uc.count)
#endif

#ifndef netdev_uc_empty
#define netdev_uc_empty(dev) ((dev)->uc.count == 0)
#endif

#ifndef netdev_for_each_uc_addr
#define netdev_for_each_uc_addr(ha, dev) \
		list_for_each_entry(ha, &dev->uc.list, list)
#endif

extern void bnad_set_rx_ucast_fltr(struct bnad_s *bnad);

#else /* !BNAD_MACVLAN */

#define bnad_set_rx_ucast_fltr(_bnad)

#endif /* BNAD_MACVLAN */

#define	bnad_set_rx_qid(_skb, _rxq)

#define bnad_tx_tcb_get(_bnad, _txq_id)	\
	(_bnad)->tx_info[0].tcb[_txq_id]

#define BNAD_TCB_2_TXQ_ID(_tcb)	((_tcb)->id)

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
#define BNAD_TXQ_ID_GET(_netdev, _skb)			\
	skb_get_queue_mapping((_skb));
#else
#define BNAD_TXQ_ID_GET(_netdev, _skb)			\
	bnad_tx_select_queue((_netdev), (_skb));
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 23)

#define BNAD_DECLARE_TCB_TXQ_ID(_tcb)	\
	uint32_t txq_id = ((_tcb)->id)

#define bnad_alloc_netdev(_size, _txq_num)	\
		alloc_etherdev_mq((_size), (_txq_num))

#define bnad_stop_txq(_netdev, _txq_id)		\
			netif_stop_subqueue((_netdev), (_txq_id))

#define bnad_wake_txq(_netdev, _txq_id)		\
			netif_wake_subqueue((_netdev), (_txq_id))


#else  /* KERNEL VERSION < 2.6.23 */

#define BNAD_DECLARE_TCB_TXQ_ID(_tcb)
#define bnad_alloc_netdev(_size, _txq_num) alloc_etherdev((_size))
#define bnad_stop_txq(_netdev, _txq_id)		\
			netif_stop_queue(_netdev)

#define bnad_wake_txq(_netdev, _txq_id)		\
			netif_wake_queue(_netdev)

#endif /* KERNEL VERSION < 2.6.23 */

/*
 * Need to treat bnad_txq_stopped separately,
 * since this is different on 2.6.23
 */
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 23)
#define bnad_txq_stopped(_netdev, _txq_id)	\
			__netif_subqueue_stopped((_netdev), (_txq_id))
#elif LINUX_VERSION_CODE == KERNEL_VERSION(2, 6, 23)
#define bnad_txq_stopped(_netdev, _txq_id)	\
			netif_subqueue_stopped((_netdev), (_txq_id))
#else
#define bnad_txq_stopped(_netdev, _txq_id)	\
			netif_queue_stopped(_netdev)
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36)
#define bnad_skb_tx_timestamp(_skb) skb_tx_timestamp((_skb))
#elif defined(RHEL_RELEASE_VERSION)
#if (RHEL_RELEASE_CODE >= RHEL_RELEASE_VERSION(6, 4))
#define bnad_skb_tx_timestamp(_skb) skb_tx_timestamp((_skb))
#else
#define bnad_skb_tx_timestamp(_skb)
#endif
#else
#define bnad_skb_tx_timestamp(_skb)
#endif

#define bnad_alloc_netq_array(_bnad)	(0)
#define bnad_free_netq_array(_bnad)
#define BNAD_NETQ_REGISTER(_bnad, _netdev)
#define bnad_netq_reinit_needed(_bnad, _invalidate_netq)
#define bnad_netq_mtu_reinit_needed(_bnad, _mtu, _new_mtu)  (0)
#define bnad_netq_mtu_reinit(bnad)				(0)
#define bnad_netq_reinit_tx_prio(bnad)
#define bnad_netq_cleanup(bnad)

#define bnad_alloc_skb(_netdevice, _size) alloc_skb(_size, GFP_ATOMIC);

static inline struct sk_buff *
bnad_rxq_get_skb(struct bnad_rx_ctrl_s *rx_ctrl)
{
#ifdef BNAD_GRO
	return napi_get_frags(&rx_ctrl->napi);
#else
	CNA_ASSERT_DEBUG(0);
	return NULL;
#endif
}

#if defined(__ia64__) || defined(__powerpc__)
#define bnad_udelay     udelay
#else
#define bnad_udelay     __udelay
#endif

#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 0, 0)
/*
 * linux kernel versions > 3.3.x introduced netdev_features_t which
 * replaced the u32 return type of ndo_set_features() & ndo_fix_features()
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 3, 0)
typedef u32 bnad_netdev_features_t;
#else
typedef netdev_features_t bnad_netdev_features_t;
#endif
bnad_netdev_features_t bnad_fix_features(struct net_device *netdev,
					bnad_netdev_features_t features);
int bnad_set_features(struct net_device *dev, bnad_netdev_features_t features);
#endif

/*
 * linux kernel 4.0 renames vlan_tx_* helpers since "tx" is misleading there.
 * The same macros are used for rx as well.
 */
#if defined(vlan_tx_tag_present)
#define bnad_vlan_tag_present(_skb) vlan_tx_tag_present(_skb)
#else
#define bnad_vlan_tag_present(_skb) skb_vlan_tag_present(_skb)
#endif
#if defined(vlan_tx_tag_get)
#define bnad_vlan_tag_get(_skb) vlan_tx_tag_get(_skb)
#else
#define bnad_vlan_tag_get(_skb) skb_vlan_tag_get(_skb)
#endif

/*
 * PCI DMA macros
 */
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 1, 0)
#define BNAD_DMA_UNMAP_ADDR() \
		DECLARE_PCI_UNMAP_ADDR(dma_addr);

#define bnad_tx_dma_unmap_addr(_unmap, _vector) \
		pci_unmap_addr(&(_unmap)->vectors[_vector], dma_addr)

#define bnad_rx_dma_unmap_addr(_unmap) \
		pci_unmap_addr(&(_unmap)->vector, dma_addr)

#define bnad_dma_map(_bnad, _frag, _size) \
		pci_map_page((_bnad)->pcidev, (_frag)->page, \
			(_frag)->page_offset, _size, \
			PCI_DMA_TODEVICE)

#define bnad_tx_unmap_addr_set(_unmap, _vector, _dma_addr) \
		pci_unmap_addr_set(&(_unmap)->vectors[_vector], \
			dma_addr, _dma_addr)

#define bnad_rx_unmap_addr_set(_unmap, _dma_addr) \
		pci_unmap_addr_set(&(_unmap)->vector, \
			dma_addr, _dma_addr)
#else /* KERNEL_VERSION > 3.1.0 */
#define BNAD_DMA_UNMAP_ADDR() \
		DEFINE_DMA_UNMAP_ADDR(dma_addr);

#define bnad_tx_dma_unmap_addr(_unmap, _vector) \
		dma_unmap_addr(&(_unmap)->vectors[_vector], dma_addr)

#define bnad_rx_dma_unmap_addr(_unmap) \
		dma_unmap_addr(&(_unmap)->vector, dma_addr)

#define bnad_dma_map(_bnad, _frag, _size) \
		skb_frag_dma_map(&(_bnad)->pcidev->dev, _frag, \
			0, _size, \
			DMA_TO_DEVICE)

#define bnad_tx_unmap_addr_set(_unmap, _vector, _dma_addr) \
		dma_unmap_addr_set(&(_unmap)->vectors[_vector], \
			dma_addr, _dma_addr)

#define bnad_rx_unmap_addr_set(_unmap, _dma_addr) \
		dma_unmap_addr_set(&(_unmap)->vector, \
			dma_addr, _dma_addr)
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
#define BNAD_DMA_UNMAP_LEN \
	 uint32_t len;
#define bnad_tx_unmap_len_set(_unmap, _vect_id, _size) \
	_unmap->vectors[_vect_id].len = _size;
#define bnad_tx_dma_unmap_len(_unmap, _vect_id, _nvecs, _skb) \
	_unmap->vectors[_vect_id].len
#else
#define BNAD_DMA_UNMAP_LEN
#define bnad_tx_unmap_len_set(_unmap, _vect_id, _size)
#define bnad_tx_dma_unmap_len(_unmap, _vect_id, _nvecs, _skb) \
	skb_shinfo(_skb)->frags[_nvecs].size
#endif

/*
 * CONFIG_HOTPLUG is no more available as an option in 3.8 based kernels.
 * As result the __dev* markings are also not available.
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
#define bnad_init_t int
#define bnad_exit_t void
#define BNAD_EXIT bnad_pci_remove
#else
#define bnad_init_t int __devinit
#define bnad_exit_t void __devexit
#define BNAD_EXIT __devexit_p(bnad_pci_remove)
#endif

#endif /* __BNAD_COMPAT_H__ */
