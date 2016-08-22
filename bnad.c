/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details
 *
 * Maintained-by: Rasesh Mody <rasesh.mody@qlogic.com>
 */

#include "cna_os.h"

#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/etherdevice.h>
#include <linux/in.h>
#include <linux/ethtool.h>
#include <linux/ip.h>

#include "bna.h"

#include "bnad_compat.h"
#include "bnad_trcmod.h"
#include "bnad.h"

BNA_TRC_FILE(LDRV, BNAD);

/*
 * Module params
 */
static uint bnad_msix_disable;
module_param(bnad_msix_disable, uint, 0444);
MODULE_PARM_DESC(bnad_msix_disable, "Disable MSIX mode");

static uint bnad_log_level;
module_param(bnad_log_level, uint, 0644);
MODULE_PARM_DESC(bnad_log_level, "Log level");

static uint bnad_ioc_auto_recover = 1;
module_param(bnad_ioc_auto_recover, uint, 0444);
MODULE_PARM_DESC(bnad_ioc_auto_recover, "Enable / Disable auto recovery");

static uint bna_debugfs_enable = 1;
module_param(bna_debugfs_enable, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(bna_debugfs_enable, "Enables debugfs feature, default=1,"
		" Range[false:0|true:1]");

static uint bna_veb_enable = 0;
module_param(bna_veb_enable, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(bna_veb_enable, "Enable VEB, default=0,"
		" Range[false:0|true:1]");

/*
 * Global variables
 */

struct mutex bnad_list_mutex;
LIST_HEAD(bnad_list);

const uint8_t bnad_bcast_addr[] =  {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

/*
 * Local MACROS
 */

#define BNAD_GET_MBOX_IRQ(_bnad)				\
	(((_bnad)->cfg_flags & BNAD_CF_MSIX) ?			\
	 ((_bnad)->msix_table[BNAD_MAILBOX_MSIX_INDEX].vector) : 	\
	 ((_bnad)->pcidev->irq))

#define BNAD_FILL_UNMAPQ_MEM_REQ(_res_info, _num, _size)	\
do {								\
	(_res_info)->res_type = BNA_RES_T_MEM;			\
	(_res_info)->res_u.mem_info.mem_type = BNA_MEM_T_KVA;	\
	(_res_info)->res_u.mem_info.num = (_num);		\
	(_res_info)->res_u.mem_info.len = (_size);		\
} while (0)

#define BNAD_TXRX_SYNC_MDELAY	(20) 		/* 20 msecs */

int
bnad_add_to_list(struct bnad_s *bnad)
{
	static int bna_id = 0;
	int ret;

	mutex_lock(&bnad_list_mutex);
	list_add_tail(&bnad->list_entry, &bnad_list);
	ret = bna_id++;
	mutex_unlock(&bnad_list_mutex);

	return ret;
}

void
bnad_remove_from_list(struct bnad_s *bnad)
{
	mutex_lock(&bnad_list_mutex);
	list_del(&bnad->list_entry);
	mutex_unlock(&bnad_list_mutex);
}

const struct pci_device_id bnad_pci_id_table[] = {
	{
		PCI_DEVICE(PCI_VENDOR_ID_BROCADE,
			PCI_DEVICE_ID_BROCADE_CATAPULT),
		.class = PCI_CLASS_NETWORK_ETHERNET << 8,
		.class_mask =  0xffff00
	},
	{
		PCI_DEVICE(PCI_VENDOR_ID_BROCADE,
			PCI_DEVICE_ID_BROCADE_CATAPULT2),
		.class = PCI_CLASS_NETWORK_ETHERNET << 8,
		.class_mask =  0xffff00
	},
	{0, },
};

MODULE_DEVICE_TABLE(pci, bnad_pci_id_table);

/* To bnad_aen.c */
void
bnad_cb_aen_notify(void *bnad)
{
}

void
bfa_os_gettimeofday(struct bfa_timeval_s *tv)
{
	struct timeval tmp_tv;

	do_gettimeofday(&tmp_tv);
	tv->tv_sec = (uint32_t) tmp_tv.tv_sec;
	tv->tv_usec = (uint32_t) tmp_tv.tv_usec;
}


/* Tx Datapath functions */


/* Caller shoudl ensure that the entry at unmap_q[index] is valid */
static uint32_t bnad_tx_buff_unmap(struct bnad_s *bnad,
			      struct bnad_tx_unmap_s *unmap_q,
			      uint32_t q_depth, uint32_t index)
{
	struct bnad_tx_unmap_s *unmap;
	struct sk_buff *skb;
	int vector, nvecs;

	unmap = &unmap_q[index];
	nvecs = unmap->nvecs;
	CNA_ASSERT_DEBUG(nvecs);

	vector = 0;
	skb = unmap->skb;
	unmap->skb = NULL;
	unmap->nvecs = 0;
	dma_unmap_single(&bnad->pcidev->dev,
			bnad_tx_dma_unmap_addr(unmap, vector),
			skb_headlen(skb), DMA_TO_DEVICE);
	bnad_tx_unmap_addr_set(unmap, vector, 0);
	nvecs--;

	while (nvecs) {
		vector++;
		if (vector == BFI_TX_MAX_VECTORS_PER_WI) {
			vector = 0;
			BNA_QE_INDX_INC(index, q_depth);
			unmap = &unmap_q[index];
			CNA_ASSERT_DEBUG(unmap->skb == NULL);
			CNA_ASSERT_DEBUG(unmap->nvecs == 0);
		}

		dma_unmap_page(&bnad->pcidev->dev,
				bnad_tx_dma_unmap_addr(unmap, vector),
				bnad_tx_dma_unmap_len(unmap, vector, nvecs,
					skb),
				DMA_TO_DEVICE);
		bnad_tx_unmap_addr_set(unmap, vector, 0);
		nvecs--;
	}

	BNA_QE_INDX_INC(index, q_depth);

	return index;
}

/*
 * Frees all pending Tx Bufs per TxQ
 * At this point no activity is expected on the Q,
 * so DMA unmap & freeing is fine.
 * Called from work context.
 */
static void
bnad_txq_cleanup(struct bnad_s *bnad, struct bna_tcb_s *tcb)
{
	struct bnad_tx_unmap_s *unmap_q = tcb->unmap_q;
	struct sk_buff *skb;
	int i;

	for (i = 0; i < tcb->q_depth; i++) {
		skb = unmap_q[i].skb;
		if (!skb)
			continue;
		bnad_tx_buff_unmap(bnad, unmap_q, tcb->q_depth, i);
		dev_kfree_skb_any(skb);
	}
}

/*
 * Free all TxQs buffers and then notify TX_E_CLEANUP_DONE to Tx fsm.
 */
static void
bnad_tx_cleanup(struct delayed_work *work)
{
	struct bnad_tx_info_s *tx_info =
		container_of(work, struct bnad_tx_info_s, tx_cleanup_work);
	struct bnad_s *bnad = NULL;
	struct bna_tcb_s *tcb;
	unsigned long flags;
	uint32_t i, pending = 0;

	for (i = 0; i < BNAD_MAX_TXQ_PER_TX; i++) {
		tcb = tx_info->tcb[i];
		if (!tcb)
			continue;

		bnad = tcb->bnad;

		if (test_and_set_bit(BNAD_TXQ_FREE_SENT, &tcb->flags)) {
			pending++;
			continue;
		}

		bfa_trc(bnad, i);

		bnad_txq_cleanup(bnad, tcb);

		smp_mb__before_clear_bit();
		clear_bit(BNAD_TXQ_FREE_SENT, &tcb->flags);
	}

	CNA_ASSERT_DEBUG(bnad);

	if (pending) {
		BNAD_QUEUE_DELAYED_WORK(bnad->work_q, &tx_info->tx_cleanup_work,
				msecs_to_jiffies(1));
		return;
	}

	spin_lock_irqsave(&bnad->bna_lock, flags);
	bna_tx_cleanup_complete(tx_info->tx);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);
}

/*
 * bnad_free_txbufs : Frees the Tx bufs on Tx completion
 * Can be called in a) Interrupt context
 *		    b) Sending context
 */
static uint32_t
bnad_txcmpl_process(struct bnad_s *bnad, struct bna_tcb_s *tcb)
{
	uint32_t sent_packets = 0, sent_bytes = 0;
	uint32_t wis, unmap_wis, hw_cons, cons, q_depth;
	struct bnad_tx_unmap_s *unmap_q = tcb->unmap_q;
	struct bnad_tx_unmap_s *unmap;
	struct sk_buff *skb;

	/*
	 * Just return if TX is stopped.
	 */
	if (!test_bit(BNAD_TXQ_TX_STARTED, &tcb->flags))
		return 0;

	hw_cons = *(tcb->hw_consumer_index);
	cons = tcb->consumer_index;
	q_depth = tcb->q_depth;

	wis = BNA_Q_INDEX_CHANGE(cons, hw_cons, q_depth);
	CNA_ASSERT(wis <= BNA_QE_IN_USE_CNT(tcb, q_depth));

	while (wis) {
		unmap = &unmap_q[cons];

		skb = unmap->skb;
		CNA_ASSERT_DEBUG(skb);
		sent_packets++;
		sent_bytes += skb->len;

		unmap_wis = BNA_TXQ_WI_NEEDED(unmap->nvecs);
		CNA_ASSERT_DEBUG(wis >= unmap_wis);
		wis -= unmap_wis;

		cons = bnad_tx_buff_unmap(bnad, unmap_q, q_depth, cons);
		dev_kfree_skb_any(skb);
	}

	CNA_ASSERT_DEBUG(hw_cons == cons);

	/* Update consumer pointers. */
	tcb->consumer_index = hw_cons;

	tcb->txq->tx_packets += sent_packets;
	tcb->txq->tx_bytes += sent_bytes;

	return sent_packets;
}

uint32_t
bnad_tx_complete(struct bnad_s *bnad, struct bna_tcb_s *tcb)
{
	struct net_device *netdev = bnad->netdev;
	uint32_t sent = 0;

	if (test_and_set_bit(BNAD_TXQ_FREE_SENT, &tcb->flags))
		return 0;

	sent = bnad_txcmpl_process(bnad, tcb);
	if (sent) {
		BNAD_DECLARE_TCB_TXQ_ID(tcb);

		if (bnad_txq_stopped(netdev, txq_id) &&
		    netif_carrier_ok(netdev) &&
		    BNA_QE_FREE_CNT(tcb, tcb->q_depth) >=
				    BNAD_NETIF_WAKE_THRESHOLD) {
			if (test_bit(BNAD_TXQ_TX_STARTED, &tcb->flags)) {
				bnad_wake_txq(netdev, txq_id);
				BNAD_UPDATE_CTR(bnad, netif_queue_wakeup);
			}
		}
	}

	if (likely(test_bit(BNAD_TXQ_TX_STARTED, &tcb->flags)))
		bna_ib_ack(tcb->i_dbell, sent);

	smp_mb__before_clear_bit();
	clear_bit(BNAD_TXQ_FREE_SENT, &tcb->flags);

	return sent;
}

/* Returns 0 for success */
static int
bnad_txq_wi_prepare(struct bnad_s *bnad, struct bna_tcb_s *tcb,
		    struct sk_buff *skb, struct bna_txq_entry_s *txqent)
{
	bna_txq_wi_ctrl_flag_t flags = 0;
	uint32_t gso_size;
	uint16_t vlan_tag = 0;

	if (vlan_tx_tag_present(skb)) {
		vlan_tag = (uint16_t)vlan_tx_tag_get(skb);
		flags |= (BNA_TXQ_WI_CF_INS_PRIO | BNA_TXQ_WI_CF_INS_VLAN);
	}
	if (test_bit(BNAD_RF_CEE_RUNNING, &bnad->run_flags)) {
		vlan_tag = ((tcb->priority & 0x7) << BNAD_VLAN_PRIO_SHIFT)
				| (vlan_tag & 0x1fff);
		flags |= (BNA_TXQ_WI_CF_INS_PRIO | BNA_TXQ_WI_CF_INS_VLAN);
	}
	txqent->hdr.wi.vlan_tag = htons(vlan_tag);

	gso_size = skb_is_gso(skb);
	if (gso_size) {
		if (unlikely(gso_size > bnad->netdev->mtu)) {
			BNAD_UPDATE_CTR(bnad, tx_skb_mss_too_long);
			return -EINVAL;
		}
		if (unlikely((gso_size + skb_transport_offset(skb) +
			      tcp_hdrlen(skb)) >= skb->len)) {
			txqent->hdr.wi.opcode =
				__constant_htons(BNA_TXQ_WI_SEND);
			txqent->hdr.wi.lso_mss = 0;
			BNAD_UPDATE_CTR(bnad, tx_skb_tso_too_short);
		} else {
			txqent->hdr.wi.opcode =
				__constant_htons(BNA_TXQ_WI_SEND_LSO);
			txqent->hdr.wi.lso_mss = htons(gso_size);
		}

		if (bnad_tso_prepare(bnad, skb)) {
			BNAD_UPDATE_CTR(bnad, tx_skb_tso_prepare);
			return -EINVAL;
		}

		flags |= (BNA_TXQ_WI_CF_IP_CKSUM | BNA_TXQ_WI_CF_TCP_CKSUM);
		txqent->hdr.wi.l4_hdr_size_n_offset =
			htons(BNA_TXQ_WI_L4_HDR_N_OFFSET(
			tcp_hdrlen(skb) >> 2, skb_transport_offset(skb)));
	} else  {

		txqent->hdr.wi.opcode =	__constant_htons(BNA_TXQ_WI_SEND);
		txqent->hdr.wi.lso_mss = 0;

		if (unlikely(skb->len > (bnad->netdev->mtu + ETH_HLEN))) {
			BNAD_UPDATE_CTR(bnad, tx_skb_non_tso_too_long);
			return -EINVAL;
		}

		if (skb->ip_summed == CHECKSUM_PARTIAL) {
			uint8_t proto = 0;

			if (skb->protocol == __constant_htons(ETH_P_IP))
				proto = ip_hdr(skb)->protocol;
#ifdef NETIF_F_IPV6_CSUM
			else if (skb->protocol ==
				 __constant_htons(ETH_P_IPV6)) {
				/* nexthdr may not be TCP immediately. */
				proto = ipv6_hdr(skb)->nexthdr;
			}
#endif
			if (proto == IPPROTO_TCP) {
				flags |= BNA_TXQ_WI_CF_TCP_CKSUM;
				txqent->hdr.wi.l4_hdr_size_n_offset =
					htons(BNA_TXQ_WI_L4_HDR_N_OFFSET
					      (0, skb_transport_offset(skb)));

				BNAD_UPDATE_CTR(bnad, tcpcsum_offload);

				if (unlikely(skb_headlen(skb) <
					    skb_transport_offset(skb) +
				    tcp_hdrlen(skb))) {
					BNAD_UPDATE_CTR(bnad, tx_skb_tcp_hdr);
					return -EINVAL;
				}
			} else if (proto == IPPROTO_UDP) {
				flags |= BNA_TXQ_WI_CF_UDP_CKSUM;
				txqent->hdr.wi.l4_hdr_size_n_offset =
					htons(BNA_TXQ_WI_L4_HDR_N_OFFSET
					      (0, skb_transport_offset(skb)));

				BNAD_UPDATE_CTR(bnad, udpcsum_offload);
				if (unlikely(skb_headlen(skb) <
					    skb_transport_offset(skb) +
				    sizeof(struct udphdr))) {
					BNAD_UPDATE_CTR(bnad, tx_skb_udp_hdr);
					return -EINVAL;
				}
			} else {
				BNAD_UPDATE_CTR(bnad, tx_skb_csum_err);
				return -EINVAL;
			}
		} else
			txqent->hdr.wi.l4_hdr_size_n_offset = 0;
	}

	txqent->hdr.wi.flags = htons(flags);
	txqent->hdr.wi.frame_length = htonl(skb->len);

	return 0;
}

/* bnad_start_xmit : Netdev entry point for Transmit */
/*		     Called under lock held by net_device */
netdev_tx_t
bnad_start_xmit(struct sk_buff *skb, struct net_device *netdev)
{
	struct bnad_s 		*bnad = netdev_priv(netdev);
	struct bna_tcb_s	*tcb = NULL;
	uint32_t		txq_id;
	struct bnad_tx_unmap_s 	*unmap_q, *unmap, *head_unmap;
	uint32_t 		prod, q_depth, vect_id;
	uint32_t 		wis, vectors, len;
	int 			i;
	dma_addr_t 		dma_addr;
	struct bna_txq_entry_s 	*txqent;

	len = skb_headlen(skb);

	/* Sanity checks for the skb */

	if (unlikely(skb->len <= ETH_HLEN)) {
		dev_kfree_skb(skb);
		BNAD_UPDATE_CTR(bnad, tx_skb_too_short);
		return NETDEV_TX_OK;
	}
	if (unlikely(len > BFI_TX_MAX_DATA_PER_VECTOR)) {
		dev_kfree_skb(skb);
		BNAD_UPDATE_CTR(bnad, tx_skb_headlen_zero);
		return NETDEV_TX_OK;
	}
	if (unlikely(len == 0)) {
		dev_kfree_skb(skb);
		BNAD_UPDATE_CTR(bnad, tx_skb_headlen_zero);
		return NETDEV_TX_OK;
	}

	txq_id = BNAD_TXQ_ID_GET(netdev, skb);
	tcb = bnad_tx_tcb_get(bnad, txq_id);

	/*
	 * Takes care of the Tx that is scheduled between clearing the flag
	 * and the netif_tx_stop_all_queues() call.
	 */
	if (unlikely(!tcb || !test_bit(BNAD_TXQ_TX_STARTED, &tcb->flags))) {
		dev_kfree_skb(skb);
		BNAD_UPDATE_CTR(bnad, tx_skb_stopping);
		return NETDEV_TX_OK;
	}

	q_depth = tcb->q_depth;
	prod = tcb->producer_index;
	unmap_q = tcb->unmap_q;
	CNA_ASSERT_PERF(unmap_q);

	vectors = 1 + skb_shinfo(skb)->nr_frags;
	wis = BNA_TXQ_WI_NEEDED(vectors);	/* 4 vectors per work item */

	if (unlikely(vectors > BFI_TX_MAX_VECTORS_PER_PKT)) {
		dev_kfree_skb(skb);
		BNAD_UPDATE_CTR(bnad, tx_skb_max_vectors);
		return NETDEV_TX_OK;
	}

	/* Check for available TxQ resources */
	if (unlikely(wis > BNA_QE_FREE_CNT(tcb, q_depth))) {
		if ((*tcb->hw_consumer_index != tcb->consumer_index) &&
		    !test_and_set_bit(BNAD_TXQ_FREE_SENT, &tcb->flags)) {
			uint32_t sent;
			sent = bnad_txcmpl_process(bnad, tcb);
			if (likely(test_bit(BNAD_TXQ_TX_STARTED, &tcb->flags)))
				bna_ib_ack(tcb->i_dbell, sent);
			smp_mb__before_clear_bit();
			clear_bit(BNAD_TXQ_FREE_SENT, &tcb->flags);
		} else {
			bnad_stop_txq(netdev, txq_id);
			BNAD_UPDATE_CTR(bnad, netif_queue_stop);
		}

		smp_mb();
		/*
		 * Check again to deal with race condition between
		 * bnad_stop_txq here, and bnad_wake_txq in
		 * interrupt handler which is not inside netif tx lock.
		 */
		if (likely(wis > BNA_QE_FREE_CNT(tcb, q_depth))) {
			BNAD_UPDATE_CTR(bnad, netif_queue_stop);
			return NETDEV_TX_BUSY;
		} else {
			bnad_wake_txq(netdev, txq_id);
			BNAD_UPDATE_CTR(bnad, netif_queue_wakeup);
		}
	}

	txqent = &((struct bna_txq_entry_s *)tcb->sw_q)[prod];
	head_unmap = &unmap_q[prod];

	/* Program the opcode, flags, frame_len, num_vectors in WI */
	if (bnad_txq_wi_prepare(bnad, tcb, skb, txqent)) {
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}
	txqent->hdr.wi.reserved = 0;
	txqent->hdr.wi.num_vectors = vectors;

	head_unmap->skb = skb;
	head_unmap->nvecs = 0;

	/* Program the vectors */
	unmap = head_unmap;
	dma_addr = pci_map_single(bnad->pcidev, skb->data, len,
			PCI_DMA_TODEVICE);
	BNA_SET_DMA_ADDR(dma_addr, &txqent->vector[0].host_addr);
	txqent->vector[0].length = htons(len);
	bnad_tx_unmap_addr_set(unmap, 0, dma_addr);
	head_unmap->nvecs++;

	for (i = 0, vect_id = 0; i < vectors - 1; i++) {
		struct skb_frag_struct *frag = &skb_shinfo(skb)->frags[i];
		uint32_t size = frag->size;

		if (unlikely(size == 0)) {
			/* Undo the changes starting at tcb->producer_index */
			bnad_tx_buff_unmap(bnad, unmap_q, q_depth,
				tcb->producer_index);
			dev_kfree_skb(skb);
			BNAD_UPDATE_CTR(bnad, tx_skb_frag_zero);
			return NETDEV_TX_OK;
		}

		len += size;

		vect_id++;
		if (vect_id == BFI_TX_MAX_VECTORS_PER_WI) {
			vect_id = 0;
			BNA_QE_INDX_INC(prod, q_depth);
			txqent = &((struct bna_txq_entry_s *)tcb->sw_q)[prod];
			txqent->hdr.wi_ext.opcode =
				__constant_htons(BNA_TXQ_WI_EXTENSION);
			unmap = &unmap_q[prod];
		}

		dma_addr = bnad_dma_map(bnad, frag, size);
		bnad_tx_unmap_len_set(unmap, vect_id, size);
		BNA_SET_DMA_ADDR(dma_addr, &txqent->vector[vect_id].host_addr);
		txqent->vector[vect_id].length = htons(size);
		bnad_tx_unmap_addr_set(unmap, vect_id, dma_addr);
		head_unmap->nvecs++;
	}

	if (unlikely(len != skb->len)) {
		/* Undo the changes starting at tcb->producer_index */
		bnad_tx_buff_unmap(bnad, unmap_q, q_depth, tcb->producer_index);
		dev_kfree_skb(skb);
		BNAD_UPDATE_CTR(bnad, tx_skb_len_mismatch);
		return NETDEV_TX_OK;
	}

	BNA_QE_INDX_INC(prod, q_depth);
	tcb->producer_index = prod;

	smp_mb();

	if (unlikely(!test_bit(BNAD_TXQ_TX_STARTED, &tcb->flags)))
		return NETDEV_TX_OK;

	bnad_netdev_trans_start_set(netdev);

	bnad_skb_tx_timestamp(skb);

	bna_txq_prod_indx_doorbell(tcb);
	smp_mb();

	return NETDEV_TX_OK;
}


/* Rx datapath functions */
static inline void
bnad_rxq_cleanup_page(struct bnad_s *bnad, struct bnad_rx_unmap_s *unmap)
{
	if (!unmap->page)
		return;

	dma_unmap_page(&bnad->pcidev->dev,
			bnad_rx_dma_unmap_addr(unmap),
			unmap->vector.len, DMA_FROM_DEVICE);
	put_page(unmap->page);
	unmap->page = NULL;
	bnad_rx_unmap_addr_set(unmap, 0);
	unmap->vector.len = 0;
}

static inline void
bnad_rxq_cleanup_skb(struct bnad_s *bnad, struct bnad_rx_unmap_s *unmap)
{
	if (!unmap->skb)
		return;

	dma_unmap_single(&bnad->pcidev->dev,
			bnad_rx_dma_unmap_addr(unmap),
			unmap->vector.len, DMA_FROM_DEVICE);
	dev_kfree_skb_any(unmap->skb);
	unmap->skb = NULL;
	bnad_rx_unmap_addr_set(unmap, 0);
	unmap->vector.len = 0;
}

static void
bnad_rxq_alloc_uninit(struct bnad_s *bnad, struct bna_rcb_s *rcb)
{
	struct bnad_rx_unmap_q_s *unmap_q = rcb->unmap_q;

	unmap_q->reuse_pi = -1;
	unmap_q->alloc_order = -1;
	unmap_q->map_size = 0;
	unmap_q->type = BNAD_RXBUF_NONE;
}

static void
bnad_rxq_alloc_init(struct bnad_s *bnad, struct bna_rcb_s *rcb)
{
	struct bnad_rx_unmap_q_s *unmap_q = rcb->unmap_q;

	bnad_rxq_alloc_uninit(bnad, rcb);

	if (bnad_rxq_gro_init(bnad, rcb) == 0)
		return;

	unmap_q->type = BNAD_RXBUF_SK_BUFF;
}

static void
bnad_rxq_cleanup(struct bnad_s *bnad, struct bna_rcb_s *rcb)
{
	struct bnad_rx_unmap_q_s *unmap_q = rcb->unmap_q;
	int i;

	for (i = 0; i < rcb->q_depth; i++) {
		struct bnad_rx_unmap_s *unmap = &unmap_q->unmap[i];

		if (BNAD_RXBUF_IS_SK_BUFF(unmap_q->type))
			bnad_rxq_cleanup_skb(bnad, unmap);
		else
			bnad_rxq_cleanup_page(bnad, unmap);
	}

	bnad_rxq_alloc_uninit(bnad, rcb);
}

/*
 * Reinitialize completions in CQ, once Rx is taken down
 */
static void
bnad_cq_cleanup(struct bnad_s *bnad, struct bna_ccb_s *ccb)
{
	struct bna_cq_entry_s *cmpl;
	int i;

	for (i = 0; i < ccb->q_depth; i++) {
		cmpl = &((struct bna_cq_entry_s *)ccb->sw_q)[i];
		cmpl->valid = 0;
	}
}

/*
 * Free all RxQs buffers and then notify RX_E_CLEANUP_DONE to Rx fsm.
 */
static void
bnad_rx_cleanup(void *work)
{
	struct bnad_rx_info_s *rx_info =
		container_of(work, struct bnad_rx_info_s, rx_cleanup_work);
	struct bnad_rx_ctrl_s *rx_ctrl;
	struct bnad_s *bnad = NULL;
	unsigned long flags;
	uint32_t i;

	for (i = 0; i < BNAD_MAX_RXP_PER_RX; i++) {
		rx_ctrl = &rx_info->rx_ctrl[i];

		if (!rx_ctrl->ccb)
			continue;

		bnad = rx_ctrl->ccb->bnad;
		bfa_trc(bnad, i);

		/*
		 * Wait till the poll handler has exited
		 * and nothing can be scheduled anymore
		 */
		BNAD_NAPI_DISABLE(bnad, &rx_ctrl->napi);

		bnad_cq_cleanup(bnad, rx_ctrl->ccb);
		bnad_rxq_cleanup(bnad, rx_ctrl->ccb->rcb[0]);
		if (rx_ctrl->ccb->rcb[1])
			bnad_rxq_cleanup(bnad, rx_ctrl->ccb->rcb[1]);
	}

	CNA_ASSERT_DEBUG(bnad);

	spin_lock_irqsave(&bnad->bna_lock, flags);
	bna_rx_cleanup_complete(rx_info->rx);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);
}

static uint32_t
bnad_rxq_refill_page(struct bnad_s *bnad, struct bna_rcb_s *rcb,
		uint32_t nalloc)
{
	uint32_t alloced, prod, q_depth;
	struct bnad_rx_unmap_q_s *unmap_q = rcb->unmap_q;
	struct bnad_rx_unmap_s *unmap, *prev;
	struct bna_rxq_entry_s *rxent;
	struct page *page;
	uint32_t page_offset, alloc_size;
	dma_addr_t dma_addr;

	prod = rcb->producer_index;
	q_depth = rcb->q_depth;

	alloc_size = CNA_PAGE_SIZE << unmap_q->alloc_order;
	alloced = 0;

	while (nalloc--) {
		unmap = &unmap_q->unmap[prod];

		if (unmap_q->reuse_pi < 0) {
			page = alloc_pages(GFP_ATOMIC | __GFP_COMP,
					unmap_q->alloc_order);
			page_offset = 0;
		} else {
			prev = &unmap_q->unmap[unmap_q->reuse_pi];
			page = prev->page;
			page_offset = prev->page_offset + unmap_q->map_size;
			get_page(page);
		}

		if (unlikely(!page)) {
			BNAD_UPDATE_CTR(bnad, rxbuf_alloc_failed);
			rcb->rxq->rxbuf_alloc_failed++;
			goto finishing;
		}

		dma_addr = pci_map_page(bnad->pcidev, page, page_offset,
				unmap_q->map_size, PCI_DMA_FROMDEVICE);

		unmap->page = page;
		unmap->page_offset = page_offset;
		bnad_rx_unmap_addr_set(unmap, dma_addr);
		unmap->vector.len = unmap_q->map_size;
		page_offset += unmap_q->map_size;

		if (page_offset < alloc_size)
			unmap_q->reuse_pi = prod;
		else
			unmap_q->reuse_pi = -1;

		rxent = &((struct bna_rxq_entry_s *)rcb->sw_q)[prod];
		BNA_SET_DMA_ADDR(dma_addr, &rxent->host_addr);
		BNA_QE_INDX_INC(prod, q_depth);
		alloced++;
	}

finishing:
	if (likely(alloced)) {
		rcb->producer_index = prod;
		smp_mb();
		if (likely(test_bit(BNAD_RXQ_POST_OK, &rcb->flags)))
			bna_rxq_prod_indx_doorbell(rcb);
	}

	return alloced;
}

static uint32_t
bnad_rxq_refill_skb(struct bnad_s *bnad, struct bna_rcb_s *rcb,
		uint32_t nalloc)
{
	uint32_t alloced, prod, q_depth, buff_sz;
	struct bnad_rx_unmap_q_s *unmap_q = rcb->unmap_q;
	struct bnad_rx_unmap_s *unmap;
	struct bna_rxq_entry_s *rxent;
	struct sk_buff *skb;
	dma_addr_t dma_addr;

	buff_sz = rcb->rxq->buffer_size;
	prod = rcb->producer_index;
	q_depth = rcb->q_depth;

	alloced = 0;
	while (nalloc--) {
		unmap = &unmap_q->unmap[prod];

		skb = bnad_alloc_skb(bnad->netdev, buff_sz + NET_IP_ALIGN);

		if (unlikely(!skb)) {
			BNAD_UPDATE_CTR(bnad, rxbuf_alloc_failed);
			rcb->rxq->rxbuf_alloc_failed++;
			goto finishing;
		}

		skb->dev = bnad->netdev;
		skb_reserve(skb, NET_IP_ALIGN);

		dma_addr = pci_map_single(bnad->pcidev, skb->data, buff_sz,
				PCI_DMA_FROMDEVICE);

		unmap->skb = skb;
		bnad_rx_unmap_addr_set(unmap, dma_addr);
		unmap->vector.len = buff_sz;

		rxent = &((struct bna_rxq_entry_s *)rcb->sw_q)[prod];
		BNA_SET_DMA_ADDR(dma_addr, &rxent->host_addr);
		BNA_QE_INDX_INC(prod, q_depth);
		alloced++;
	}

finishing:
	if (likely(alloced)) {
		rcb->producer_index = prod;
		smp_mb();
		if (likely(test_bit(BNAD_RXQ_POST_OK, &rcb->flags)))
			bna_rxq_prod_indx_doorbell(rcb);
	}

	return alloced;
}

/* Allocate and post BNAD_RXQ_REFILL_THRESHOLD_SHIFT buffers at a time */
static inline void
bnad_rxq_post(struct bnad_s *bnad, struct bna_rcb_s *rcb)
{
	struct bnad_rx_unmap_q_s *unmap_q = rcb->unmap_q;
	uint32_t to_alloc;

	to_alloc = BNA_QE_FREE_CNT(rcb, rcb->q_depth);
	if (!(to_alloc >> BNAD_RXQ_REFILL_THRESHOLD_SHIFT))
		return;

	if (BNAD_RXBUF_IS_SK_BUFF(unmap_q->type))
		bnad_rxq_refill_skb(bnad, rcb, to_alloc);
	else
		bnad_rxq_refill_page(bnad, rcb, to_alloc);
}

const uint32_t flags_cksum_prot_mask = (BNA_CQ_EF_IPV4 | BNA_CQ_EF_L3_CKSUM_OK |
					BNA_CQ_EF_IPV6 |
					BNA_CQ_EF_TCP | BNA_CQ_EF_UDP |
					BNA_CQ_EF_L4_CKSUM_OK);

const uint32_t flags_tcp4 = (BNA_CQ_EF_IPV4 | BNA_CQ_EF_L3_CKSUM_OK |
				BNA_CQ_EF_TCP | BNA_CQ_EF_L4_CKSUM_OK);
const uint32_t flags_tcp6 = (BNA_CQ_EF_IPV6 |
				BNA_CQ_EF_TCP | BNA_CQ_EF_L4_CKSUM_OK);
const uint32_t flags_udp4 = (BNA_CQ_EF_IPV4 | BNA_CQ_EF_L3_CKSUM_OK |
				BNA_CQ_EF_UDP | BNA_CQ_EF_L4_CKSUM_OK);
const uint32_t flags_udp6 = (BNA_CQ_EF_IPV6 |
				BNA_CQ_EF_UDP | BNA_CQ_EF_L4_CKSUM_OK);

static void
bnad_cq_drop_packet(struct bnad_s *bnad, struct bna_rcb_s *rcb,
		uint32_t sop_ci, uint32_t nvecs)
{
	struct bnad_rx_unmap_q_s *unmap_q;
	struct bnad_rx_unmap_s *unmap;
	uint32_t ci, vec;

	unmap_q = rcb->unmap_q;
	for (vec = 0, ci = sop_ci; vec < nvecs; vec++) {
		unmap = &unmap_q->unmap[ci];
		BNA_QE_INDX_INC(ci, rcb->q_depth);

		if (BNAD_RXBUF_IS_SK_BUFF(unmap_q->type))
			bnad_rxq_cleanup_skb(bnad, unmap);
		else
			bnad_rxq_cleanup_page(bnad, unmap);
	}
}

static void
bnad_cq_setup_skb_frags(struct bna_rcb_s *rcb, struct sk_buff *skb,
		uint32_t sop_ci, uint32_t nvecs, uint32_t last_fraglen)
{
	struct bnad_s *bnad;
	uint32_t ci, vec, len, totlen = 0;
	struct bnad_rx_unmap_q_s *unmap_q;
	struct bnad_rx_unmap_s *unmap;

	unmap_q = rcb->unmap_q;
	bnad = rcb->bnad;

	/* prefetch header */
	prefetch(page_address(unmap_q->unmap[sop_ci].page) +
			unmap_q->unmap[sop_ci].page_offset);

	for (vec = 1, ci = sop_ci; vec <= nvecs; vec++) {
		unmap = &unmap_q->unmap[ci];
		BNA_QE_INDX_INC(ci, rcb->q_depth);

		dma_unmap_page(&bnad->pcidev->dev,
				bnad_rx_dma_unmap_addr(unmap),
				unmap->vector.len, DMA_FROM_DEVICE);

		/* FIXME: uggh... */
		/* each vector is filled to its max except the last */
		len = (vec == nvecs) ?
			last_fraglen : unmap->vector.len;
		totlen += len;

		skb_fill_page_desc(skb, skb_shinfo(skb)->nr_frags,
				unmap->page, unmap->page_offset, len);

		unmap->page = NULL;
		unmap->vector.len = 0;
	}

	skb->len += totlen;
	skb->data_len += totlen;
	skb->truesize += totlen;
}

static inline void
bnad_cq_setup_skb(struct bnad_s *bnad, struct sk_buff *skb,
		struct bnad_rx_unmap_s *unmap, uint32_t len)
{
	prefetch(skb->data);
	dma_unmap_single(&bnad->pcidev->dev,
			bnad_rx_dma_unmap_addr(unmap),
			unmap->vector.len, DMA_FROM_DEVICE);

	skb_put(skb, len);
	skb->protocol = eth_type_trans(skb, bnad->netdev);

	unmap->skb = NULL;
	unmap->vector.len = 0;
}

uint32_t
bnad_cq_process(struct bnad_s *bnad, struct bna_ccb_s *ccb, int budget)
{
	struct bna_cq_entry_s *cq, *cmpl, *next_cmpl;
	uint32_t packets = 0, len = 0, totlen = 0;
	uint32_t pi, vec, sop_ci = 0, nvecs = 0;
	struct bnad_rx_unmap_q_s *unmap_q;
	struct bnad_rx_unmap_s *unmap = NULL;
	struct bna_rcb_s *rcb = NULL;
	struct sk_buff *skb = NULL;
	uint32_t flags, masked_flags;

	cq = ccb->sw_q;
	cmpl = &cq[ccb->producer_index];

	while (packets < budget) {
		if (!cmpl->valid)
			break;
		rmb();

		if (bna_is_small_rxq(cmpl->rxq_id))
			rcb = ccb->rcb[1];
		else
			rcb = ccb->rcb[0];

		unmap_q = rcb->unmap_q;

		/* start of packet ci */
		sop_ci = rcb->consumer_index;

		if (BNAD_RXBUF_IS_SK_BUFF(unmap_q->type)) {
			unmap = &unmap_q->unmap[sop_ci];
			skb = unmap->skb;
			CNA_ASSERT_PERF(skb);
		} else {
			skb = bnad_rxq_get_skb(ccb->ctrl);
			if (unlikely(!skb))
				break;
		}
		prefetch(skb);

		flags = ntohl(cmpl->flags);
		totlen = len = ntohs(cmpl->length);
		nvecs = 1;

		/*
		 * Check all the completions for this frame.
		 * busy-wait doesn't help much, break here.
		 */
		if (BNAD_RXBUF_IS_MULTI_BUFF(unmap_q->type) &&
				(flags & BNA_CQ_EF_EOP) == 0) {
			CNA_ASSERT_PERF(rcb->rxq->multi_buffer);

			pi = ccb->producer_index;
			do {
				BNA_QE_INDX_INC(pi, ccb->q_depth);
				next_cmpl = &cq[pi];

				if (!next_cmpl->valid)
					break;
				rmb();

				len = ntohs(next_cmpl->length);
				flags = ntohl(next_cmpl->flags);

				nvecs++;
				totlen += len;
			} while ((flags & BNA_CQ_EF_EOP) == 0);

			if (!next_cmpl->valid)
				break;
		}

		/* TODO: BNA_CQ_EF_LOCAL ? */
		if (unlikely(flags & (BNA_CQ_EF_MAC_ERROR |
						BNA_CQ_EF_FCS_ERROR |
						BNA_CQ_EF_TOO_LONG))) {
			bnad_cq_drop_packet(bnad, rcb, sop_ci, nvecs);
			rcb->rxq->rx_packets_with_error++;

			goto next;
		}

		if (BNAD_CHECK_RBL(bnad))
		    if (BNAD_PKT_DROP(bnad, totlen)) {
			bnad_cq_drop_packet(bnad, rcb, sop_ci, nvecs);

			goto next;
		    }

		if (BNAD_RXBUF_IS_SK_BUFF(unmap_q->type))
			bnad_cq_setup_skb(bnad, skb, unmap, len);
		else
			bnad_cq_setup_skb_frags(rcb, skb, sop_ci, nvecs, len);

		packets++;
		rcb->rxq->rx_packets++;
		rcb->rxq->rx_bytes += totlen;
		ccb->bytes_per_intr += totlen;

		masked_flags = flags & flags_cksum_prot_mask;

		if (likely(bnad->rx_csum &&
			((masked_flags == flags_tcp4) ||
			 (masked_flags == flags_udp4) ||
			 (masked_flags == flags_tcp6) ||
			 (masked_flags == flags_udp6))))
			skb->ip_summed = CHECKSUM_UNNECESSARY;
		else
			skb->ip_summed = CHECKSUM_NONE;

		bnad_set_rx_qid(skb, rcb->rxq);

		bnad_rcv_skb(ccb->ctrl, skb, flags, bnad->cfg_flags,
				bnad_vlan_group(bnad), cmpl->vlan_tag, cmpl);

		bnad_netdev_last_rx_set(bnad->netdev);

next:
		BNA_QE_INDX_ADD(rcb->consumer_index, nvecs, rcb->q_depth);
		for (vec = 0; vec < nvecs; vec++) {
			cmpl = &cq[ccb->producer_index];
			cmpl->valid = 0;
			BNA_QE_INDX_INC(ccb->producer_index, ccb->q_depth);
		}
		cmpl = &cq[ccb->producer_index];
	}

	bnad_rcv_flush(ccb->ctrl);

	if ((ccb->pkts_una + packets) <= BFI_IBUNA_MAX)
		ccb->pkts_una += packets;
	else {
		if (test_bit(BNAD_RXQ_STARTED, &ccb->rcb[0]->flags)) {
			bna_ib_ack_disable_irq(ccb->i_dbell, ccb->pkts_una);
			ccb->pkts_una = packets;
		}
	}

	bnad_rxq_post(bnad, ccb->rcb[0]);
	if (ccb->rcb[1])
		bnad_rxq_post(bnad, ccb->rcb[1]);

	return packets;
}

/* Interrupt handlers */


/* Mbox Interrupt Handlers */
static irqreturn_t
bnad_msix_mbox_handler(int irq, void *data)
{
	uint32_t intr_status;
	unsigned long  flags;
	struct bnad_s *bnad = (struct bnad_s *)(data);

	spin_lock_irqsave(&bnad->bna_lock, flags);

	if (unlikely(test_bit(BNAD_RF_MBOX_IRQ_DISABLED, &bnad->run_flags))) {
		spin_unlock_irqrestore(&bnad->bna_lock, flags);
		return IRQ_HANDLED;
	}

	bna_intr_status_get(&bnad->bna, intr_status);

	if (BNA_IS_MBOX_ERR_INTR(&bnad->bna, intr_status))
		bna_mbox_handler(&bnad->bna, intr_status);
	else
		bfa_trc(bnad, intr_status);

	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	return IRQ_HANDLED;
}

/* MSIX Tx Completion Handler */
static irqreturn_t
bnad_msix_tx(int irq, void *data)
{
	struct bna_tcb_s *tcb = (struct bna_tcb_s *)data;
	struct bnad_s *bnad = tcb->bnad;

	bnad_tx_complete(bnad, tcb);

	return IRQ_HANDLED;
}

/* MSIX Rx Path Handler */
static irqreturn_t
bnad_msix_rx(int irq, void *data)
{
	struct bna_ccb_s *ccb = (struct bna_ccb_s *)data;

	if (ccb) {
		((struct bnad_rx_ctrl_s *)(ccb->ctrl))->rx_intr_ctr++;
		bnad_netif_rx_schedule_poll(ccb->bnad, ccb);
	}

	return IRQ_HANDLED;
}

/* INTx handler */
irqreturn_t
bnad_isr(int irq, void *data)
{
	int i, j;
	uint32_t intr_status;
	unsigned long flags;
	struct bnad_s *bnad = (struct bnad_s *)(data);
	struct bnad_rx_info_s *rx_info;
	struct bnad_rx_ctrl_s *rx_ctrl;
	struct bna_tcb_s *tcb = NULL;

	spin_lock_irqsave(&bnad->bna_lock, flags);

	if (unlikely(test_bit(BNAD_RF_MBOX_IRQ_DISABLED, &bnad->run_flags))) {
		spin_unlock_irqrestore(&bnad->bna_lock, flags);
		return IRQ_NONE;
	}

	bna_intr_status_get(&bnad->bna, intr_status);
	if (unlikely(!intr_status)) {
		spin_unlock_irqrestore(&bnad->bna_lock, flags);
		return IRQ_NONE;
	}

	if (BNA_IS_MBOX_ERR_INTR(&bnad->bna, intr_status))
		bna_mbox_handler(&bnad->bna, intr_status);

	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	if (!BNA_IS_INTX_DATA_INTR(intr_status))
		return IRQ_HANDLED;

	/* Process data interrupts */
	/* Tx processing */
	for (i = 0; i < bnad->num_tx; i++) {
		for (j = 0; j < bnad->num_txq_per_tx; j++) {
			tcb = bnad->tx_info[i].tcb[j];
			if (tcb && test_bit(BNAD_TXQ_TX_STARTED, &tcb->flags))
				bnad_tx_complete(bnad, bnad->tx_info[i].tcb[j]);
		}
	}

	/* Rx processing */
	for (i = 0; i < bnad->num_rx; i++) {
		rx_info = &bnad->rx_info[i];
		if (!rx_info->rx)
			continue;
		for (j = 0; j < bnad->num_rxp_per_rx; j++) {
			rx_ctrl = &rx_info->rx_ctrl[j];
			if (rx_ctrl->ccb)
				bnad_netif_rx_schedule_poll(bnad,
							    rx_ctrl->ccb);
		}
	}
	return IRQ_HANDLED;
}


/* Interrupt enable / disable */
static void
bnad_enable_msix(struct bnad_s *bnad)
{
	int i, ret;
	unsigned long flags;

	bfa_trc(bnad, bnad->cfg_flags);
	spin_lock_irqsave(&bnad->bna_lock, flags);
	if (!(bnad->cfg_flags & BNAD_CF_MSIX)) {
		spin_unlock_irqrestore(&bnad->bna_lock, flags);
		return;
	}
	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	if (bnad->msix_table)
		return;

	bfa_trc(bnad, bnad->msix_num);

	bnad->msix_table =
		kcalloc(bnad->msix_num, sizeof(struct msix_entry), GFP_KERNEL);

	if (!bnad->msix_table)
		goto intx_mode;

	for (i = 0; i < bnad->msix_num; i++)
		bnad->msix_table[i].entry = i;

	ret = pci_enable_msix(bnad->pcidev, bnad->msix_table, bnad->msix_num);
	if (ret > 0) {
		/* Not enough MSI-X vectors. */
		bfa_trc(bnad, ret);

		printk(KERN_WARNING "BNA[%d]: %d MSI-X vectors allocated < "
		       "%d requested\n", bnad->ident.id, ret, bnad->msix_num);

		spin_lock_irqsave(&bnad->bna_lock, flags);
		/* ret = #of vectors that we got */
		bnad_q_num_adjust(bnad, (ret - BNAD_MAILBOX_MSIX_VECTORS) / 2,
			(ret - BNAD_MAILBOX_MSIX_VECTORS) / 2);
		spin_unlock_irqrestore(&bnad->bna_lock, flags);

		bnad->msix_num = BNAD_NUM_TXQ + BNAD_NUM_RXP +
			 BNAD_MAILBOX_MSIX_VECTORS;

		bfa_trc(bnad, bnad->msix_num);

		if (bnad->msix_num > ret)
			goto intx_mode;

		/* Try once more with adjusted numbers */
		/* If this fails, fall back to INTx */
		ret = pci_enable_msix(bnad->pcidev, bnad->msix_table,
				      bnad->msix_num);
		bfa_trc(bnad, ret);
		if (ret)
			goto intx_mode;

	} else if (ret < 0) {
		bfa_trc(bnad, ret);
		goto intx_mode;
	}

	pci_intx(bnad->pcidev, 0);
	return;

intx_mode:
	printk(KERN_WARNING "BNA[%d]: MSI-X enable failed - "
			    "operating in INTx mode\n", bnad->ident.id);
	bfa_trc(bnad, bnad->cfg_flags);

	kfree(bnad->msix_table);
	bnad->msix_table = NULL;
	bnad->msix_num = 0;
	spin_lock_irqsave(&bnad->bna_lock, flags);
	bnad->cfg_flags &= ~BNAD_CF_MSIX;
	bnad_q_num_init(bnad);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);
}

static void
bnad_disable_msix(struct bnad_s *bnad)
{
	uint32_t cfg_flags;
	unsigned long flags;

	spin_lock_irqsave(&bnad->bna_lock, flags);
	cfg_flags = bnad->cfg_flags;
	if (bnad->cfg_flags & BNAD_CF_MSIX)
		bnad->cfg_flags &= ~BNAD_CF_MSIX;
	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	if (cfg_flags & BNAD_CF_MSIX) {
		pci_disable_msix(bnad->pcidev);
		kfree(bnad->msix_table);
		bnad->msix_table = NULL;
	}
	bfa_trc(bnad, bnad->cfg_flags);
}

/*
 * Called in interrupt / callback context
 * with bna_lock held, so cfg_flags access is OK
 */
void
bnad_enable_mbox_irq(struct bnad_s *bnad)
{
	int irq = BNAD_GET_MBOX_IRQ(bnad);

	bfa_trc(bnad, irq);
	bfa_trc(bnad, bnad->run_flags);

	clear_bit(BNAD_RF_MBOX_IRQ_DISABLED, &bnad->run_flags);

	BNAD_UPDATE_CTR(bnad, mbox_intr_enabled);
}


/*
 * Called with bnad->bna_lock held b'cos of
 * bnad->cfg_flags access.
 */
void
bnad_disable_mbox_irq(struct bnad_s *bnad)
{
	int irq = BNAD_GET_MBOX_IRQ(bnad);

	bfa_trc(bnad, irq);
	bfa_trc(bnad, bnad->run_flags);

	set_bit(BNAD_RF_MBOX_IRQ_DISABLED, &bnad->run_flags);

	BNAD_UPDATE_CTR(bnad, mbox_intr_disabled);
}

/* Control Path Handlers */

/* Callbacks */
void
bnad_cb_mbox_intr_enable(struct bnad_s *bnad)
{
	bnad_enable_mbox_irq(bnad);
}

void
bnad_cb_mbox_intr_disable(struct bnad_s *bnad)
{
	bnad_disable_mbox_irq(bnad);
}

void
bnad_cb_ioceth_ready(struct bnad_s *bnad)
{
	bfa_trc(bnad, BNA_CB_SUCCESS);

	bnad->bnad_completions.ioc_comp_status = BNA_CB_SUCCESS;
	complete(&bnad->bnad_completions.ioc_comp);
}

void
bnad_cb_ioceth_failed(struct bnad_s *bnad)
{
	bfa_trc(bnad, BNA_CB_FAIL);

	bnad->bnad_completions.ioc_comp_status = BNA_CB_FAIL;
	complete(&bnad->bnad_completions.ioc_comp);
}

void
bnad_cb_ioceth_disabled(struct bnad_s *bnad)
{
	bfa_trc(bnad, BNA_CB_SUCCESS);

	bnad->bnad_completions.ioc_comp_status = BNA_CB_SUCCESS;
	complete(&bnad->bnad_completions.ioc_comp);
}

void
bnad_cb_enet_disabled(void *arg)
{
	struct bnad_s *bnad = (struct bnad_s *)arg;

	bfa_trc(bnad, netif_carrier_ok(bnad->netdev));

	netif_carrier_off(bnad->netdev);
	complete(&bnad->bnad_completions.enet_comp);
}

void
bnad_cb_ethport_link_status(struct bnad_s *bnad,
			enum bna_link_status_e link_status)
{
	bool link_up = 0;

	bfa_trc(bnad, link_status);

	link_up = (link_status == BNA_LINK_UP) || (link_status == BNA_CEE_UP);

	if (link_status == BNA_CEE_UP) {
		if (!test_bit(BNAD_RF_CEE_RUNNING, &bnad->run_flags))
			BNAD_UPDATE_CTR(bnad, cee_toggle);
		set_bit(BNAD_RF_CEE_RUNNING, &bnad->run_flags);
	} else {
		if (test_bit(BNAD_RF_CEE_RUNNING, &bnad->run_flags))
			BNAD_UPDATE_CTR(bnad, cee_toggle);
		clear_bit(BNAD_RF_CEE_RUNNING, &bnad->run_flags);
	}

	bnad_netq_reinit_tx_prio(bnad);

	if (link_up) {
		if (!netif_carrier_ok(bnad->netdev)) {
			uint tx_id, tcb_id;
			printk(KERN_WARNING "bna: %s link up\n",
				bnad->netdev->name);
			netif_carrier_on(bnad->netdev);
			BNAD_UPDATE_CTR(bnad, link_toggle);
			bfa_trc(bnad, BNAD_GET_CTR(bnad, link_toggle));
			for (tx_id = 0; tx_id < bnad->num_tx; tx_id++) {
				for (tcb_id = 0; tcb_id < bnad->num_txq_per_tx;
				      tcb_id++) {
					struct bna_tcb_s *tcb =
					bnad->tx_info[tx_id].tcb[tcb_id];
					uint32_t txq_id;
					if (!tcb)
						continue;

					txq_id = BNAD_TCB_2_TXQ_ID(tcb);

					if (test_bit(BNAD_TXQ_TX_STARTED,
						     &tcb->flags)) {
						/*
						 * Force an immediate
						 * Transmit Schedule */
						printk(KERN_INFO "bna: %s %d "
						      "TXQ_STARTED\n",
						       bnad->netdev->name,
						       txq_id);
						bnad_wake_txq(bnad->netdev,
							      txq_id);
						BNAD_UPDATE_CTR(bnad,
							netif_queue_wakeup);
						bfa_trc(bnad, BNAD_GET_CTR(bnad,
							netif_queue_wakeup));
					} else {
						bnad_stop_txq(bnad->netdev,
							      txq_id);
						BNAD_UPDATE_CTR(bnad,
							netif_queue_stop);
						bfa_trc(bnad, BNAD_GET_CTR(bnad,
							netif_queue_stop));
					}
				}
			}
		}
	} else {
		if (netif_carrier_ok(bnad->netdev)) {
			printk(KERN_WARNING "bna: %s link down\n",
				bnad->netdev->name);
			netif_carrier_off(bnad->netdev);
			BNAD_UPDATE_CTR(bnad, link_toggle);
			bfa_trc(bnad, BNAD_GET_CTR(bnad, link_toggle));
		}
	}
}

void
bnad_cb_bw_update(struct bnad_s *bnad, int bw)
{
	BNAD_RBL_BW_UPDATE(bnad, bw);
}

void
bnad_cb_tx_disabled(void *arg, struct bna_tx_s *tx)
{
	struct bnad_s *bnad = (struct bnad_s *)arg;

	complete(&bnad->bnad_completions.tx_comp);
}

void
bnad_cb_tcb_setup(struct bnad_s *bnad, struct bna_tcb_s *tcb)
{
	struct bnad_tx_info_s *tx_info =
			(struct bnad_tx_info_s *)tcb->txq->tx->priv;

	tcb->priv = tcb;
	tx_info->tcb[tcb->id] = tcb;
}

void
bnad_cb_tcb_destroy(struct bnad_s *bnad, struct bna_tcb_s *tcb)
{
	struct bnad_tx_info_s *tx_info =
			(struct bnad_tx_info_s *)tcb->txq->tx->priv;

	tx_info->tcb[tcb->id] = NULL;
	tcb->priv = NULL;
}

void
bnad_cb_ccb_setup(struct bnad_s *bnad, struct bna_ccb_s *ccb)
{
	struct bnad_rx_info_s *rx_info =
			(struct bnad_rx_info_s *)ccb->cq->rx->priv;

	rx_info->rx_ctrl[ccb->id].ccb = ccb;
	ccb->ctrl = &rx_info->rx_ctrl[ccb->id];

}

void
bnad_cb_ccb_destroy(struct bnad_s *bnad, struct bna_ccb_s *ccb)
{
	struct bnad_rx_info_s *rx_info =
			(struct bnad_rx_info_s *)ccb->cq->rx->priv;

	rx_info->rx_ctrl[ccb->id].ccb = NULL;
}

void
bnad_cb_tx_stall(struct bnad_s *bnad, struct bna_tx_s *tx)
{
	struct bnad_tx_info_s *tx_info = (struct bnad_tx_info_s *)tx->priv;
	struct bna_tcb_s *tcb;
	uint32_t txq_id;
	int i;

	for (i = 0; i < BNAD_MAX_TXQ_PER_TX; i++) {
		tcb = tx_info->tcb[i];
		if (!tcb)
			continue;
		txq_id = BNAD_TCB_2_TXQ_ID(tcb);
		bfa_trc(bnad, txq_id);
		clear_bit(BNAD_TXQ_TX_STARTED, &tcb->flags);
		bnad_stop_txq(bnad->netdev, txq_id);
		printk(KERN_INFO "bna: %s %d TXQ_STOPPED\n",
			bnad->netdev->name, txq_id);
	}
}


void
bnad_cb_tx_resume(struct bnad_s *bnad, struct bna_tx_s *tx)
{
	struct bnad_tx_info_s *tx_info = (struct bnad_tx_info_s *)tx->priv;
	struct bna_tcb_s *tcb;
	uint32_t txq_id;
	int i;

	for (i = 0; i < BNAD_MAX_TXQ_PER_TX; i++) {
		tcb = tx_info->tcb[i];
		if (!tcb)
			continue;
		txq_id = BNAD_TCB_2_TXQ_ID(tcb);
		bfa_trc(bnad, txq_id);

		CNA_ASSERT(0 == test_bit(BNAD_TXQ_TX_STARTED, &tcb->flags));

		set_bit(BNAD_TXQ_TX_STARTED, &tcb->flags);

		CNA_ASSERT(*(tcb->hw_consumer_index) == 0);

		if (netif_carrier_ok(bnad->netdev)) {
			printk(KERN_INFO "bna: %s %d TXQ_STARTED\n",
				bnad->netdev->name, txq_id);
			bnad_wake_txq(bnad->netdev, txq_id);
			BNAD_UPDATE_CTR(bnad, netif_queue_wakeup);
			bfa_trc(bnad, BNAD_GET_CTR(bnad, netif_queue_wakeup));
		}
	}

	/*
	 * Workaround for first ioceth enable failure & we
	 * get a 0 MAC address. We try to get the MAC address
	 * again here.
	 */
	if (is_zero_ether_addr(&bnad->perm_addr.mac[0])) {
		bna_enet_perm_mac_get(&bnad->bna.enet, &bnad->perm_addr);
		bnad_set_netdev_perm_addr(bnad);
	}

}

void
bnad_cb_tx_cleanup(struct bnad_s *bnad, struct bna_tx_s *tx)
{
	struct bnad_tx_info_s *tx_info = (struct bnad_tx_info_s *)tx->priv;
	struct bna_tcb_s *tcb;
	int i;

	bfa_trc(bnad, tx->rid);
	for (i = 0; i < BNAD_MAX_TXQ_PER_TX; i++) {
		tcb = tx_info->tcb[i];
		if (!tcb)
			continue;
		bfa_trc(bnad, tcb->flags);
	}

	BNAD_QUEUE_DELAYED_WORK(bnad->work_q, &tx_info->tx_cleanup_work, 0);
}

void
bnad_cb_rx_stall(struct bnad_s *bnad, struct bna_rx_s *rx)
{
	struct bnad_rx_info_s *rx_info = (struct bnad_rx_info_s *)rx->priv;
	struct bna_ccb_s *ccb;
	struct bnad_rx_ctrl_s *rx_ctrl;
	int i;

	bfa_trc(bnad, 0);

	for (i = 0; i < BNAD_MAX_RXP_PER_RX; i++) {
		rx_ctrl = &rx_info->rx_ctrl[i];
		ccb = rx_ctrl->ccb;
		if (!ccb)
			continue;

		clear_bit(BNAD_RXQ_POST_OK, &ccb->rcb[0]->flags);

		if (ccb->rcb[1])
			clear_bit(BNAD_RXQ_POST_OK, &ccb->rcb[1]->flags);
	}
}

void
bnad_cb_rx_cleanup(struct bnad_s *bnad, struct bna_rx_s *rx)
{
	struct bnad_rx_info_s *rx_info = (struct bnad_rx_info_s *)rx->priv;
	struct bna_ccb_s *ccb;
	struct bnad_rx_ctrl_s *rx_ctrl;
	int i;

	bfa_trc(bnad, rx->rid);

	for (i = 0; i < BNAD_MAX_RXP_PER_RX; i++) {
		rx_ctrl = &rx_info->rx_ctrl[i];
		ccb = rx_ctrl->ccb;
		if (!ccb)
			continue;

		clear_bit(BNAD_RXQ_STARTED, &ccb->rcb[0]->flags);

		if (ccb->rcb[1])
			clear_bit(BNAD_RXQ_STARTED, &ccb->rcb[1]->flags);

	}

	queue_work(bnad->work_q, &rx_info->rx_cleanup_work);
}

void
bnad_cb_rx_post(struct bnad_s *bnad, struct bna_rx_s *rx)
{
	struct bnad_rx_info_s *rx_info = (struct bnad_rx_info_s *)rx->priv;
	struct bna_ccb_s *ccb;
	struct bna_rcb_s *rcb;
	struct bnad_rx_ctrl_s *rx_ctrl;
	int i, j;

	bfa_trc(bnad, rx->rid);

	for (i = 0; i < BNAD_MAX_RXP_PER_RX; i++) {
		rx_ctrl = &rx_info->rx_ctrl[i];
		ccb = rx_ctrl->ccb;
		if (!ccb)
			continue;

		BNAD_NAPI_ENABLE(bnad, &rx_ctrl->napi);

		for (j = 0; j < BNAD_MAX_RXQ_PER_RXP; j++) {
			rcb = ccb->rcb[j];
			if (!rcb)
				continue;

			bnad_rxq_alloc_init(bnad, rcb);

			set_bit(BNAD_RXQ_STARTED, &rcb->flags);
			set_bit(BNAD_RXQ_POST_OK, &rcb->flags);
			bnad_rxq_post(bnad, rcb);
		}
	}
}

void
bnad_cb_rx_disabled(void *arg, struct bna_rx_s *rx)
{
	struct bnad_s *bnad = (struct bnad_s *)arg;

	complete(&bnad->bnad_completions.rx_comp);
}

void
bnad_cb_rx_ucast_add(struct bnad_s *bnad, struct bna_rx_s *rx)
{
	bnad->bnad_completions.ucast_comp_status = BNA_CB_SUCCESS;
	complete(&bnad->bnad_completions.ucast_comp);
}

void
bnad_cb_rx_ucast_del(struct bnad_s *bnad, struct bna_rx_s *rx)
{
	bnad->bnad_completions.ucast_comp_status = BNA_CB_SUCCESS;
	complete(&bnad->bnad_completions.ucast_comp);
}

void
bnad_cb_rx_mcast_add(struct bnad_s *bnad, struct bna_rx_s *rx)
{
	bnad->bnad_completions.mcast_comp_status = BNA_CB_SUCCESS;
	complete(&bnad->bnad_completions.mcast_comp);
}

void
bnad_cb_stats_get(struct bnad_s *bnad, enum bna_cb_status_e status,
		       struct bna_stats_s *stats)
{
	if (status == BNA_CB_SUCCESS)
		BNAD_UPDATE_CTR(bnad, hw_stats_updates);
	else
		bfa_trc(bnad, status);

	if (!netif_running(bnad->netdev) ||
		!test_bit(BNAD_RF_STATS_TIMER_RUNNING, &bnad->run_flags))
		return;

	mod_timer(&bnad->stats_timer,
		  jiffies + msecs_to_jiffies(BNAD_STATS_TIMER_FREQ));
}

void
bnad_cb_enet_mtu_set(struct bnad_s *bnad)
{
	bnad->bnad_completions.mtu_comp_status = BNA_CB_SUCCESS;
	complete(&bnad->bnad_completions.mtu_comp);
}

void
bnad_vnic_attr_get(struct bnad_s *bnad, struct bfa_vnic_attr_s *attr)
{
	attr->offloads = 0;

	if (bnad->netdev->features & NETIF_F_IP_CSUM)
		attr->offloads |= (BFA_VNIC_TX_IPCSUM | BFA_VNIC_TX_TCPCSUM |
				BFA_VNIC_TX_UDPCSUM);

	if (bnad->rx_csum)
		attr->offloads |= (BFA_VNIC_RX_IPCSUM | BFA_VNIC_RX_TCPCSUM |
				BFA_VNIC_RX_UDPCSUM);

	if (bnad->netdev->features & NETIF_F_TSO)
		attr->offloads |= BFA_VNIC_LSO;

	/* TODO: Need to fill OS allocated ID for TxF, RxF */
}

static void
bnad_tx_stats_get(struct bnad_tx_info_s *tx_info,
	struct bfa_vnic_stats_txf_s *txf_stats,
	struct bfa_vnic_stats_txq_s *txq_stats)
{
	int i;

	for (i = 0; i < txf_stats->num_txq; i++) {
		txq_stats[i].unmap_prod_idx = 0;
		txq_stats[i].unmap_cons_idx = 0;
	}
}

void
bnad_vnic_stats_get(struct bnad_s *bnad, struct bfa_vnic_stats_s *stats)
{
	struct bnad_tx_info_s *tx_info;
	struct bfa_vnic_stats_txf_s *txf_stats;
	struct bfa_vnic_stats_txq_s *txq_stats;
	int i;
	int j;
	int txq_idx = 0;

	stats->link_toggle = BNAD_GET_CTR(bnad, link_toggle);
	stats->cee_toggle = BNAD_GET_CTR(bnad, cee_toggle);

	for (i = 0; i < stats->num_txf; i++) {
		txf_stats = &stats->txf_stats[i];
		for (j = 0; j < BNAD_MAX_TX; j++) {
			tx_info = &bnad->tx_info[j];
			if (tx_info->tx &&
				(tx_info->tx->rid == txf_stats->id)) {
				txq_stats = &stats->txq_stats[txq_idx];
				bnad_tx_stats_get(tx_info,
						txf_stats,
						txq_stats);
				txq_idx += (int)txf_stats->num_txq;
			}
		}
	}
}

void
bnad_vnic_stats_clr(struct bnad_s *bnad)
{
	bnad->stats.drv_stats.link_toggle = 0;
	bnad->stats.drv_stats.cee_toggle = 0;
}

/* Initialization functions */

static void
bnad_trc_log_aen_uninit(struct bnad_s *bnad)
{
	kfree(bnad->trcmod);
	kfree(bnad->logmod);
	kfree(bnad->aen.aen);
}

static int
bnad_trc_log_aen_init(struct bnad_s *bnad,
		      struct pci_dev *pdev)
{
	bnad->trcmod = kzalloc(sizeof(struct bfa_trc_mod_s), GFP_KERNEL);

	if (!bnad->trcmod) {
		printk(KERN_ERR "BNA[%d] : Failed to allocate Trace Memory\n",
		       bnad->ident.id);
		return -ENOMEM;
	}
	bfa_trc_init(bnad->trcmod);
	bfa_trc(bnad, bnad->ident.id);

	bnad->logmod = kzalloc(sizeof(struct bfa_log_mod_s), GFP_KERNEL);
	if (!bnad->logmod) {
		printk(KERN_ERR "BNA[%d] : Failed to allocate Log Memory\n",
		       bnad->ident.id);
		kfree(bnad->trcmod);
		return -ENOMEM;
	}

	bfa_log_init(bnad->logmod, (char *)(pci_name(pdev)),
		     bnad_log_printf);

	if (bnad_log_level == 0)
		bfa_log_set_level_all(bnad->logmod, BFA_LOG_INFO);
	else {
		if (BFA_STATUS_EINVAL ==
			bfa_log_set_level_all(bnad->logmod, bnad_log_level)) {
			kfree(bnad->trcmod);
			kfree(bnad->logmod);
			printk(KERN_ERR "BNA[%d] : Invalid Log Level\n",
				bnad->ident.id);
			return -EINVAL;
		}
	}

	bnad->aen.aen = kzalloc(sizeof(struct bfa_aen_s), GFP_KERNEL);
	if (!bnad->aen.aen) {
		printk(KERN_ERR "BNA[%d] : Failed to allocate Log Memory\n",
		       bnad->ident.id);
		kfree(bnad->trcmod);
		kfree(bnad->logmod);
		return -ENOMEM;
	}
	bfa_aen_init(bnad->aen.aen, bnad->trcmod, bnad, bnad->ident.id,
			bnad_cb_aen_notify, bfa_os_gettimeofday);
	return 0;
}

/* PCI Initialization */
static int
bnad_pci_init(struct bnad_s *bnad,
	      struct pci_dev *pdev, bool *using_dac)
{
	int err;

	err = pci_enable_device(pdev);
	if (err) {
		bfa_trc(bnad, err);
		return err;
	}
	err = pci_request_regions(pdev, BNAD_NAME);
	if (err) {
		bfa_trc(bnad, err);
		goto disable_device;
	}
	if (!pci_set_dma_mask(pdev, BNAD_DMA_MASK(64)) &&
	    !pci_set_consistent_dma_mask(pdev, BNAD_DMA_MASK(64))) {
		*using_dac = 1;
	} else {
		err = pci_set_dma_mask(pdev, BNAD_DMA_MASK(32));
		if (err) {
			err = pci_set_consistent_dma_mask(pdev,
						BNAD_DMA_MASK(32));
			if (err) {
				bfa_trc(bnad, err);
				goto release_regions;
			}
		}
		*using_dac = 0;
	}
	pci_set_master(pdev);
	return 0;

release_regions:
	pci_release_regions(pdev);
disable_device:
	pci_disable_device(pdev);

	return err;
}

static void
bnad_pci_uninit(struct pci_dev *pdev)
{
	pci_release_regions(pdev);
	pci_disable_device(pdev);
}

static int
bnad_wq_init(struct bnad_s *bnad)
{
	sprintf(bnad->wq_name, "%s_wq_%d", BNAD_NAME, bnad->ident.id);

	bnad->work_q = create_singlethread_workqueue(bnad->wq_name);

	if (!bnad->work_q)
		return -ENOMEM;

	BNAD_INIT_NETQ_WORK(bnad);
	return 0;
}

static void
bnad_wq_uninit(struct bnad_s *bnad)
{
	if (bnad->work_q) {
		flush_workqueue(bnad->work_q);
		destroy_workqueue(bnad->work_q);
		bnad->work_q = NULL;
	}

	BNAD_UNINIT_NETQ_WORK(bnad);
}

/*
 * 1. Initialize the bnad structure
 * 2. Setup netdev pointer in pci_dev
 * 3. Initialize no. of TxQ & CQs & MSIX vectors
 * 4. Initialize work queue.
 */
static int
bnad_init(struct bnad_s *bnad,
	  struct pci_dev *pdev, struct net_device *netdev)
{
	unsigned long flags;

	SET_NETDEV_DEV(netdev, &pdev->dev);
	pci_set_drvdata(pdev, netdev);

	bnad->netdev = netdev;
	bnad->pcidev = pdev;
	bnad->mmio_start = pci_resource_start(pdev, 0);
	bnad->mmio_len = pci_resource_len(pdev, 0);
	bnad->bar0 = ioremap_nocache(bnad->mmio_start, bnad->mmio_len);
	if (!bnad->bar0) {
		dev_err(&pdev->dev, "ioremap for bar0 failed\n");
		pci_set_drvdata(pdev, NULL);
		return -ENOMEM;
	}
	printk(KERN_INFO "bar0 mapped to %p, len %llu\n", bnad->bar0,
	       (unsigned long long) bnad->mmio_len);

	spin_lock_irqsave(&bnad->bna_lock, flags);
	if (!bnad_msix_disable)
		bnad->cfg_flags = BNAD_CF_MSIX;

	bnad_q_num_init(bnad);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	bnad->msix_num = (bnad->num_tx * bnad->num_txq_per_tx) +
		(bnad->num_rx * bnad->num_rxp_per_rx) +
			 BNAD_MAILBOX_MSIX_VECTORS;

	bnad->txq_depth = BNAD_TXQ_DEPTH;
	bnad->rxq_depth = BNAD_RXQ_DEPTH;
	bnad->rx_csum = true;

	bnad->tx_coalescing_timeo = BFI_TX_COALESCING_TIMEO;
	bnad->rx_coalescing_timeo = BFI_RX_COALESCING_TIMEO;

	bnad->cfg_flags |= BNAD_CF_DIM_ENABLED;

	return bnad_wq_init(bnad);
}

/*
 * Must be called after bnad_pci_uninit()
 * so that iounmap() and pci_set_drvdata(NULL)
 * happens only after PCI uninitialization.
 */
static void
bnad_uninit(struct bnad_s *bnad)
{
	bnad_wq_uninit(bnad);

	if (bnad->bar0)
		iounmap(bnad->bar0);
	pci_set_drvdata(bnad->pcidev, NULL);
}

/*
 * Initialize locks
	a) Per device mutes used for serializing configuration
	   changes from OS / ioctl interface
	   Note : VMWare uses a spin lock instead
	b) spin lock used to protect bna state machine
 */
static void
bnad_lock_init(struct bnad_s *bnad)
{
	spin_lock_init(&bnad->bna_lock);
	mutex_init(&bnad->conf_mutex);
}

static void
bnad_lock_uninit(struct bnad_s *bnad)
{
	mutex_destroy(&bnad->conf_mutex);
}

/* Resource allocation, free functions */

void
bnad_mem_free(struct bnad_s *bnad,
	      struct bna_mem_info_s *mem_info)
{
	int i;
	dma_addr_t dma_pa;

	if (mem_info->mdl == NULL)
		return;

	CNA_ASSERT_DEBUG((mem_info->mem_type == BNA_MEM_T_DMA ||
		   mem_info->mem_type == BNA_MEM_T_KVA));

	for (i = 0; i < mem_info->num; i++) {
		if (mem_info->mdl[i].kva != NULL) {
			if (mem_info->mem_type == BNA_MEM_T_DMA) {
				BNA_GET_DMA_ADDR(&(mem_info->mdl[i].dma),
						dma_pa);
				dma_free_coherent(&bnad->pcidev->dev,
						mem_info->mdl[i].len,
						mem_info->mdl[i].kva, dma_pa);
			} else
				kfree(mem_info->mdl[i].kva);
		}
	}
	kfree(mem_info->mdl);
	mem_info->mdl = NULL;
}

int
bnad_mem_alloc(struct bnad_s *bnad,
	       struct bna_mem_info_s *mem_info)
{
	int i;
	dma_addr_t dma_pa;

	if ((mem_info->num == 0) || (mem_info->len == 0)) {
		mem_info->mdl = NULL;
		return 0;
	}

	mem_info->mdl = kcalloc(mem_info->num, sizeof(struct bna_mem_descr_s),
				GFP_KERNEL);
	if (mem_info->mdl == NULL)
		return -ENOMEM;

	if (mem_info->mem_type == BNA_MEM_T_DMA) {
		for (i = 0; i < mem_info->num; i++) {
			mem_info->mdl[i].len = mem_info->len;
			mem_info->mdl[i].kva =
					dma_alloc_coherent(&bnad->pcidev->dev,
							mem_info->len,
							&dma_pa, GFP_KERNEL);

			if (mem_info->mdl[i].kva == NULL)
				goto err_return;

			BNA_SET_DMA_ADDR(dma_pa,
					 &(mem_info->mdl[i].dma));
		}
	} else {
		for (i = 0; i < mem_info->num; i++) {
			CNA_ASSERT_DEBUG((mem_info->mem_type ==
				BNA_MEM_T_KVA) && (mem_info->len != 0));

			mem_info->mdl[i].len = mem_info->len;
			mem_info->mdl[i].kva = kzalloc(mem_info->len,
							GFP_KERNEL);
			if (mem_info->mdl[i].kva == NULL)
				goto err_return;
		}
	}

	return 0;

err_return:
	printk(KERN_ERR "BNA[%d] : Failed memory allocation. Type:%d Size:%d\n",
		       bnad->ident.id, mem_info->mem_type, mem_info->len);
	bnad_mem_free(bnad, mem_info);
	return -ENOMEM;
}

/* Free IRQ for Mailbox */
static void
bnad_mbox_irq_free(struct bnad_s *bnad)
{
	int irq;
	unsigned long flags;

	spin_lock_irqsave(&bnad->bna_lock, flags);
	bnad_disable_mbox_irq(bnad);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	irq = BNAD_GET_MBOX_IRQ(bnad);

	bfa_trc(bnad, irq);
	bfa_trc(bnad, bnad->cfg_flags);

	free_irq(irq, bnad);
}

/*
 * Allocates IRQ for Mailbox, but keep it disabled
 * This will be enabled once we get the mbox enable callback
 * from bna
 */
static int
bnad_mbox_irq_alloc(struct bnad_s *bnad)
{
	int 		err = 0;
	unsigned long 	flags, irq_flags;
	uint32_t	irq;
	bnad_isr_t 	irq_handler;

	spin_lock_irqsave(&bnad->bna_lock, flags);

	bfa_trc(bnad, bnad->cfg_flags);
	if (bnad->cfg_flags & BNAD_CF_MSIX) {
		irq_handler = (bnad_isr_t)bnad_msix_mbox_handler;
		irq = bnad->msix_table[BNAD_MAILBOX_MSIX_INDEX].vector;
		irq_flags = 0;
	} else {
		irq_handler = (bnad_isr_t)bnad_isr;
		irq = bnad->pcidev->irq;
		irq_flags = IRQF_SHARED;
	}

	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	bfa_trc(bnad, irq);
	sprintf(bnad->mbox_irq_name, "%s%d", BNAD_NAME, bnad->ident.id);

        /*
         * Set the Mbox IRQ disabled flag, so that the IRQ handler
         * called from request_irq() for SHARED IRQs do not execute
	 */
	set_bit(BNAD_RF_MBOX_IRQ_DISABLED, &bnad->run_flags);

	BNAD_UPDATE_CTR(bnad, mbox_intr_disabled);

	err = request_irq(irq, irq_handler, irq_flags,
			  bnad->mbox_irq_name, bnad);

	return err;
}

static void
bnad_txrx_irq_free(struct bnad_s *bnad, struct bna_intr_info_s *intr_info)
{
	kfree(intr_info->idl);
	intr_info->idl = NULL;
}

/* Allocates Interrupt Descriptor List for MSIX/INT-X vectors */
static int
bnad_txrx_irq_alloc(struct bnad_s *bnad, enum bnad_intr_source_e src,
		    uint32_t txrx_id, struct bna_intr_info_s *intr_info)
{
	int i, vector_start = 0;
	uint32_t cfg_flags;
	unsigned long flags;

	spin_lock_irqsave(&bnad->bna_lock, flags);
	cfg_flags = bnad->cfg_flags;
	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	if (cfg_flags & BNAD_CF_MSIX) {
		bfa_trc(bnad, intr_info->num);
		intr_info->intr_type = BNA_INTR_T_MSIX;
		intr_info->idl = kcalloc(intr_info->num,
					sizeof(struct bna_intr_descr_s),
					GFP_KERNEL);
		if (!intr_info->idl)
			return -ENOMEM;

		switch (src) {
		case BNAD_INTR_TX:
			CNA_ASSERT_DEBUG(intr_info->num <= (bnad->num_tx *
				bnad->num_txq_per_tx));
			vector_start = BNAD_MAILBOX_MSIX_VECTORS + txrx_id;
			break;

		case BNAD_INTR_RX:
			CNA_ASSERT_DEBUG(intr_info->num <= (bnad->num_rx *
				bnad->num_rxp_per_rx));
			vector_start = BNAD_MAILBOX_MSIX_VECTORS +
				(bnad->num_tx * bnad->num_txq_per_tx) +
				txrx_id;
			break;

		default:
			CNA_ASSERT(0);
		}

		for (i = 0; i < intr_info->num; i++)
			intr_info->idl[i].vector = vector_start + i;
	} else {
		intr_info->intr_type = BNA_INTR_T_INTX;
		intr_info->num = 1;
		intr_info->idl = kcalloc(intr_info->num,
					sizeof(struct bna_intr_descr_s),
					GFP_KERNEL);
		if (!intr_info->idl)
			return -ENOMEM;

		switch (src) {
		case BNAD_INTR_TX:
			intr_info->idl[0].vector = BNAD_INTX_TX_IB_BITMASK;
			break;

		case BNAD_INTR_RX:
			intr_info->idl[0].vector = BNAD_INTX_RX_IB_BITMASK;
			break;
		}
	}
	return 0;
}

/**
 * NOTE: Should be called for MSIX only
 * Unregisters Tx MSIX vector(s) from the kernel
 */
static void
bnad_tx_msix_unregister(struct bnad_s *bnad, struct bnad_tx_info_s *tx_info,
			int num_txqs)
{
	int i;
	int vector_num;

	for (i = 0; i < num_txqs; i++) {
		if (tx_info->tcb[i] == NULL)
			continue;

		CNA_ASSERT_DEBUG(tx_info->tcb[i]->intr_type ==
					BNA_INTR_T_MSIX);

		vector_num = tx_info->tcb[i]->intr_vector;
		/* VMWare : Will need to sync explicitly */
		free_irq(bnad->msix_table[vector_num].vector, tx_info->tcb[i]);
	}
}

/**
 * NOTE: Should be called for MSIX only
 * Registers Tx MSIX vector(s) and ISR(s), cookie with the kernel
 */
static int
bnad_tx_msix_register(struct bnad_s *bnad, struct bnad_tx_info_s *tx_info,
			uint32_t tx_id, int num_txqs)
{
	int i;
	int err;
	int vector_num;

	for (i = 0; i < num_txqs; i++) {
		CNA_ASSERT_DEBUG(tx_info->tcb[i]->intr_type ==
					BNA_INTR_T_MSIX);

		vector_num = tx_info->tcb[i]->intr_vector;
		sprintf(tx_info->tcb[i]->name, "%s TXQ %d", bnad->netdev->name,
				tx_id + tx_info->tcb[i]->id);
		err = request_irq(bnad->msix_table[vector_num].vector,
				  (bnad_isr_t)bnad_msix_tx, 0,
				  tx_info->tcb[i]->name,
				  tx_info->tcb[i]);
		if (err)
			goto err_return;
	}

	return 0;

err_return:
	if (i > 0)
		bnad_tx_msix_unregister(bnad, tx_info, (i - 1));
	return -1;
}

/**
 * NOTE: Should be called for MSIX only
 * Unregisters Rx MSIX vector(s) from the kernel
 */
static void
bnad_rx_msix_unregister(struct bnad_s *bnad, struct bnad_rx_info_s *rx_info,
			int num_rxps)
{
	int i;
	int vector_num;

	for (i = 0; i < num_rxps; i++) {
		if (rx_info->rx_ctrl[i].ccb == NULL)
			continue;
		CNA_ASSERT_DEBUG(rx_info->rx_ctrl[i].ccb->intr_type ==
				BNA_INTR_T_MSIX);

		vector_num = rx_info->rx_ctrl[i].ccb->intr_vector;
		/* VMWare : Will need to sync explicitly */
		free_irq(bnad->msix_table[vector_num].vector,
			 rx_info->rx_ctrl[i].ccb);
	}
}

/**
 * NOTE: Should be called for MSIX only
 * Registers Tx MSIX vector(s) and ISR(s), cookie with the kernel
 */
static int
bnad_rx_msix_register(struct bnad_s *bnad, struct bnad_rx_info_s *rx_info,
			uint32_t rx_id, int num_rxps)
{
	int i;
	int err;
	int vector_num;

	for (i = 0; i < num_rxps; i++) {
		CNA_ASSERT_DEBUG(rx_info->rx_ctrl[i].ccb->intr_type ==
				BNA_INTR_T_MSIX);

		vector_num = rx_info->rx_ctrl[i].ccb->intr_vector;
		sprintf(rx_info->rx_ctrl[i].ccb->name, "%s CQ %d",
			bnad->netdev->name,
			rx_id + rx_info->rx_ctrl[i].ccb->id);
		err = request_irq(bnad->msix_table[vector_num].vector,
				  (bnad_isr_t)bnad_msix_rx, 0,
				  rx_info->rx_ctrl[i].ccb->name,
				  rx_info->rx_ctrl[i].ccb);
		if (err)
			goto err_return;
	}

	return 0;

err_return:
	if (i > 0)
		bnad_rx_msix_unregister(bnad, rx_info, (i - 1));
	return -1;
}

/* Free BNA resources */
static void
bnad_res_free(struct bnad_s *bnad, struct bna_res_info_s *res_info,
		uint32_t res_val_max)
{
	int i;

	for (i = 0; i < res_val_max; i++) {
		bnad_mem_free(bnad, &res_info[i].res_u.mem_info);
	}
}

/* Allocates memory and interrupt resources for BNA */
static int
bnad_res_alloc(struct bnad_s *bnad, struct bna_res_info_s *res_info,
		uint32_t res_val_max)
{
	int i, err;

	for (i = 0; i < res_val_max; i++) {
		CNA_ASSERT_DEBUG(res_info[i].res_type == BNA_RES_T_MEM);
			err = bnad_mem_alloc(bnad, &res_info[i].res_u.mem_info);
		if (err)
			goto err_return;
	}
	return 0;

err_return:
	bnad_res_free(bnad, res_info, res_val_max);
	return err;
}

/* Free Tx object Resources */
static void
bnad_tx_res_free(struct bnad_s *bnad, struct bna_res_info_s *res_info)
{
	int i;

	for (i = 0; i < BNA_TX_RES_T_MAX; i++) {
		if (res_info[i].res_type == BNA_RES_T_MEM)
			bnad_mem_free(bnad, &res_info[i].res_u.mem_info);
		else if (res_info[i].res_type == BNA_RES_T_INTR)
			bnad_txrx_irq_free(bnad, &res_info[i].res_u.intr_info);
	}
}

/* Allocates memory and interrupt resources for Tx object */
static int
bnad_tx_res_alloc(struct bnad_s *bnad, struct bna_res_info_s *res_info,
		  uint32_t tx_id)
{
	int i, err = 0;

	for (i = 0; i < BNA_TX_RES_T_MAX; i++) {

		CNA_ASSERT_DEBUG(res_info[i].res_type == BNA_RES_T_MEM ||
			   res_info[i].res_type == BNA_RES_T_INTR);

		if (res_info[i].res_type == BNA_RES_T_MEM)
			err = bnad_mem_alloc(bnad,
					&res_info[i].res_u.mem_info);
		else if (res_info[i].res_type == BNA_RES_T_INTR)
			err = bnad_txrx_irq_alloc(bnad, BNAD_INTR_TX, tx_id,
					&res_info[i].res_u.intr_info);
		if (err) {
			printk(KERN_ERR "bna: %s Failed Tx alloc %d, rc:%d\n",
				bnad->netdev->name, i, err);
			goto err_return;
		}
	}
	return 0;

err_return:
	bnad_tx_res_free(bnad, res_info);
	return err;
}

/* Free Rx object Resources */
static void
bnad_rx_res_free(struct bnad_s *bnad, struct bna_res_info_s *res_info)
{
	int i;

	for (i = 0; i < BNA_RX_RES_T_MAX; i++) {
		if (res_info[i].res_type == BNA_RES_T_MEM)
			bnad_mem_free(bnad, &res_info[i].res_u.mem_info);
		else if (res_info[i].res_type == BNA_RES_T_INTR)
			bnad_txrx_irq_free(bnad, &res_info[i].res_u.intr_info);
	}
}

/* Allocates memory and interrupt resources for Rx object */
static int
bnad_rx_res_alloc(struct bnad_s *bnad, struct bna_res_info_s *res_info,
		  uint rx_id)
{
	int i, err = 0;

	/* All memory needs to be allocated before setup_ccbs */
	for (i = 0; i < BNA_RX_RES_T_MAX; i++) {

		CNA_ASSERT_DEBUG(res_info[i].res_type == BNA_RES_T_MEM ||
				res_info[i].res_type == BNA_RES_T_INTR);

		if (res_info[i].res_type == BNA_RES_T_MEM)
			err = bnad_mem_alloc(bnad,
					&res_info[i].res_u.mem_info);
		else if (res_info[i].res_type == BNA_RES_T_INTR)
			err = bnad_txrx_irq_alloc(bnad, BNAD_INTR_RX, rx_id,
					&res_info[i].res_u.intr_info);
		if (err) {
			printk(KERN_ERR "bna: %s Failed Rx alloc %d, rc:%d\n",
				bnad->netdev->name, i, err);
			goto err_return;
		}
	}
	return 0;

err_return:
	bnad_rx_res_free(bnad, res_info);
	return err;
}

/* Timer callbacks */
/* a) IOC timer */
static void
bnad_ioc_timeout(unsigned long data)
{
	struct bnad_s *bnad = (struct bnad_s *)data;
	unsigned long flags;

	spin_lock_irqsave(&bnad->bna_lock, flags);
	bna_ioceth_timer(&bnad->bna.ioceth);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	mod_timer(&bnad->ioc_timer,
		  jiffies + msecs_to_jiffies(BNA_IOC_TIMER_FREQ));
}

/* c)  Statistics Timer */
static void
bnad_stats_timeout(unsigned long data)
{
	struct bnad_s 	*bnad = (struct bnad_s *)data;
	unsigned long flags;

	if (!netif_running(bnad->netdev) ||
		!test_bit(BNAD_RF_STATS_TIMER_RUNNING, &bnad->run_flags))
		return;

	spin_lock_irqsave(&bnad->bna_lock, flags);
	bna_hw_stats_get(&bnad->bna);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);
}

/*
 * Set up timer for statistics
 * Called with bnad_conf_lock() held
 */
void
bnad_stats_timer_start(struct bnad_s *bnad)
{
	unsigned long flags;

	spin_lock_irqsave(&bnad->bna_lock, flags);
	if (!test_and_set_bit(BNAD_RF_STATS_TIMER_RUNNING, &bnad->run_flags)) {
		setup_timer(&bnad->stats_timer, bnad_stats_timeout,
			    (unsigned long)bnad);
		mod_timer(&bnad->stats_timer,
			  jiffies + msecs_to_jiffies(BNAD_STATS_TIMER_FREQ));
	}
	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	bfa_trc(bnad, bnad->run_flags);
}

/*
 * Stops the stats timer
 * Called with bnad_conf_lock() held
 */
void
bnad_stats_timer_stop(struct bnad_s *bnad)
{
	int to_del = 0;
	unsigned long flags;

	spin_lock_irqsave(&bnad->bna_lock, flags);
	if (test_and_clear_bit(BNAD_RF_STATS_TIMER_RUNNING, &bnad->run_flags)) {
		to_del = 1;
		bfa_trc(bnad, bnad->run_flags);
	}
	spin_unlock_irqrestore(&bnad->bna_lock, flags);
	if (to_del)
		del_timer_sync(&bnad->stats_timer);
}

/* Enable / disable ioceth */
int
bnad_ioceth_disable(struct bnad_s *bnad)
{
	unsigned long flags;
	int err = 0;

	spin_lock_irqsave(&bnad->bna_lock, flags);
	bnad_netdev_trans_start_set(bnad->netdev);
	init_completion(&bnad->bnad_completions.ioc_comp);
	bna_ioceth_disable(&bnad->bna.ioceth, BNA_HARD_CLEANUP);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	wait_for_completion_timeout(&bnad->bnad_completions.ioc_comp,
		msecs_to_jiffies(BNAD_IOCETH_TIMEOUT));

	err = bnad->bnad_completions.ioc_comp_status;
	bfa_trc(bnad, err);
	return err;
}

int
bnad_ioceth_enable(struct bnad_s *bnad)
{
	unsigned long flags;
	int err = 0;

	spin_lock_irqsave(&bnad->bna_lock, flags);
	init_completion(&bnad->bnad_completions.ioc_comp);
	bnad->bnad_completions.ioc_comp_status = BNA_CB_WAITING;
	bna_ioceth_enable(&bnad->bna.ioceth);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	wait_for_completion_timeout(&bnad->bnad_completions.ioc_comp,
		msecs_to_jiffies(BNAD_IOCETH_TIMEOUT));

	err = bnad->bnad_completions.ioc_comp_status;

	bfa_trc(bnad, err);
	return err;
}

/* BNAD DEBUG */
void
debug_dump_res(struct bnad_s *bnad, struct bna_res_info_s *res_info,
		uint32_t res_val_max)
{
	int j, i;
	struct bna_mem_info_s *mem_info;
	struct bna_intr_info_s *intr_info;
	struct bna_intr_descr_s *idl;
	struct bna_mem_descr_s *mdl;

	printk("BNA [%d] :\n", bnad->ident.id);
	for (j = 0; j < res_val_max; j++) {
		if (res_info[j].res_type == BNA_RES_T_MEM) {
			mem_info = &res_info[j].res_u.mem_info;
			printk("[%d] Mem Type %d Len %d Num %d mdl 0x%p\n", j,
				mem_info->mem_type, mem_info->len,
				mem_info->num, mem_info->mdl);
			/* Just print the firt few, so as not to flood */
			for (i = 0; i < min((uint32_t)3, mem_info->num); i++) {
				mdl = &mem_info->mdl[i];
				if (!mdl) continue;
				printk("[%d] mdl[%d] len %d kva 0x%p "
					"msb 0x%x lsb 0x%x\n",
					bnad->ident.id, i, mdl->len, mdl->kva,
					mdl->dma.msb, mdl->dma.lsb);
			}
		} else if (res_info[j].res_type == BNA_RES_T_INTR) {
			intr_info = &res_info[j].res_u.intr_info;
			printk("[%d] Intr Type %d Num %d idl 0x%p\n", j,
				intr_info->intr_type, intr_info->num,
				intr_info->idl);
			for (i = 0; i < intr_info->num; i++) {
				idl = &intr_info->idl[i];
				if (!idl) continue;
				printk("[%d] vector [%d] = %d\n",
					bnad->ident.id, i, idl->vector);
			}
		} else
			printk("res_info[%d] : ERROR : "
					"Allocation Messed Up\n", j);
	}
}
/* BNAD DEBUG */

static bnad_init_t
bnad_pci_probe(struct pci_dev *pdev,
		const struct pci_device_id *pcidev_id)
{
	bool 	using_dac;
	int 	err;
	unsigned long flags;
	struct bnad_s *bnad;
	struct bna_s *bna;
	struct net_device *netdev;
	struct bfa_pcidev_s pcidev_info;

	printk(KERN_INFO "bnad_pci_probe : (0x%p, 0x%p) PCI Func : (%d)\n",
	       pdev, pcidev_id, PCI_FUNC(pdev->devfn));

	/*
	 * Allocates sizeof(struct net_device + struct bnad_s)
	 * bnad = netdev->priv
	 */
	netdev = bnad_alloc_netdev(sizeof(struct bnad_s), BNAD_MAX_TXQ);
	if (!netdev) {
		dev_err(&pdev->dev, "netdev allocation failed\n");
		err = -ENOMEM;
		return err;
	}
	bnad = netdev_priv(netdev);

	bnad->ident.id = bnad_add_to_list(bnad);

	/* Trace / log / AEN initialization */
	err = bnad_trc_log_aen_init(bnad, pdev);
	if (err) {
		printk(KERN_ERR "BNA[%d]:Trace/Log/AEN initialization failed\n",
			bnad->ident.id);
		goto free_netdev;
	}

	bnad_lock_init(bnad);

	set_bit(BNAD_RF_IOCTL_ENABLED, &bnad->run_flags);

	bnad_conf_lock();
	/*
	 * PCI initialization
	 * 	Output : using_dac = 1 for 64 bit DMA
	 *		           = 0 for 32 bit DMA
	 */
	err = bnad_pci_init(bnad, pdev, &using_dac);
	if (err) {
		bfa_trc(bnad, err);
		goto free_trc_log;
	}

	/*
	 * Initialize bnad structure
	 * Setup relation between pci_dev & netdev
	 */
	err = bnad_init(bnad, pdev, netdev);
	if (err) {
		bfa_trc(bnad, err);
		goto pci_uninit;
	}
	/* Initialize netdev structure, set up ethtool ops */
	bnad_netdev_init(bnad, using_dac);

	/* Set link to down state */
	netif_carrier_off(netdev);

	/* Setup the debugfs node for this bfad */
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 31))
	if (bna_debugfs_enable)
		bnad_debugfs_init(bnad);
#endif

	/* Get resource requirement form bna */
	spin_lock_irqsave(&bnad->bna_lock, flags);
	bna_res_req(&bnad->res_info[0]);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	/* Allocate resources from bna */
	err = bnad_res_alloc(bnad, &bnad->res_info[0], BNA_RES_T_MAX);
	if (err) {
		bfa_trc(bnad, err);
		goto drv_uninit;
	}

	bna = &bnad->bna;

	/* Setup pcidev_info for bna_init() */
	pcidev_info.pci_slot = PCI_SLOT(bnad->pcidev->devfn);
	pcidev_info.pci_func = PCI_FUNC(bnad->pcidev->devfn);
	pcidev_info.device_id = bnad->pcidev->device;
	pcidev_info.pci_bar_kva = bnad->bar0;

	spin_lock_irqsave(&bnad->bna_lock, flags);
	bna_trc_log_aen_set(bna, bnad->trcmod, bnad->logmod, bnad->aen.aen);
	bna_init(bna, bnad, &bnad->ident, &pcidev_info, &bnad->res_info[0]);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	bnad->stats.bna_stats = &bna->stats;

	bnad_enable_msix(bnad);
	err = bnad_mbox_irq_alloc(bnad);
	if (err) {
		bfa_trc(bnad, err);
		goto res_free;
	}

	/* Set up IOC timer */
	setup_timer(&(bnad->ioc_timer), bnad_ioc_timeout,
		    ((unsigned long)bnad));
	/* Now start the timer before calling IOC */
	mod_timer(&bnad->ioc_timer,
		  jiffies + msecs_to_jiffies(BNA_IOC_TIMER_FREQ));

	BNAD_RBL_INIT(bnad);

	/*
	 * Start the chip
	 * If the call back comes with error, we bail out.
	 * This is a catastrophic error.
	 */
	err = bnad_ioceth_enable(bnad);

	if (err) {
		bfa_trc(bnad, err);
		printk("BNA[%d]: Initialization failed err=%d\n",
		       bnad->ident.id, err);
		bnad_clear_pdev_netdev(pdev);
		goto probe_success;
	}

	spin_lock_irqsave(&bnad->bna_lock, flags);
	if (bna_num_txq_set(bna, BNAD_NUM_TXQ + 1) ||
		bna_num_rxp_set(bna, BNAD_NUM_RXP + 1)) {
		bnad_q_num_adjust(bnad, bna_attr(bna)->num_txq - 1,
			bna_attr(bna)->num_rxp - 1);
		if (bna_num_txq_set(bna, BNAD_NUM_TXQ + 1) ||
			bna_num_rxp_set(bna, BNAD_NUM_RXP + 1))
			err = -EIO;
	}
	spin_unlock_irqrestore(&bnad->bna_lock, flags);
	if (err) {
		bfa_trc(bnad, err);
		goto disable_ioceth;
	}

	err = bnad_alloc_netq_array(bnad);
	if (err) {
		err = -EIO;
		bfa_trc(bnad, err);
		goto disable_ioceth;
	}

	spin_lock_irqsave(&bnad->bna_lock, flags);
	bna_mod_res_req(&bnad->bna, &bnad->mod_res_info[0]);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	err = bnad_res_alloc(bnad, &bnad->mod_res_info[0], BNA_MOD_RES_T_MAX);

	if (err) {
		err = -EIO;
		bfa_trc(bnad, err);
		goto netq_free;
	}

	spin_lock_irqsave(&bnad->bna_lock, flags);
	bna_mod_init(&bnad->bna, &bnad->mod_res_info[0]);

	bna_enet_perm_mac_get(&bna->enet, &bnad->perm_addr);
	bnad_set_netdev_perm_addr(bnad);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	bfa_trc(bnad, err);

	BNAD_NETQ_REGISTER(bnad, netdev);

	/*
	 * Unlock before calling register_netdev()
	 */
	bnad_conf_unlock();

	/* Finally, reguister with net_device layer */
	err = register_netdev(netdev);
	if (err) {
		bfa_trc(bnad, err);
		printk(KERN_ERR "BNA[%d] : Registering with netdev failed\n",
		       bnad->ident.id);
		goto probe_uninit;
	}
	set_bit(BNAD_RF_NETDEV_REGISTERED, &bnad->run_flags);

	return 0;

probe_success:
	bnad_conf_unlock();
	return 0;

probe_uninit:
	bnad_conf_lock();
	bnad_res_free(bnad, &bnad->mod_res_info[0], BNA_MOD_RES_T_MAX);
netq_free:
	bnad_free_netq_array(bnad);
disable_ioceth:
	bnad_ioceth_disable(bnad);
	del_timer_sync(&bnad->ioc_timer);
	spin_lock_irqsave(&bnad->bna_lock, flags);
	bna_uninit(bna);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);
	bnad_mbox_irq_free(bnad);
	bnad_disable_msix(bnad);
res_free:
	bnad_res_free(bnad, &bnad->res_info[0], BNA_RES_T_MAX);
drv_uninit:
	/* Remove the debugfs node for this bnad */
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 31))
	kfree(bnad->regdata);
	bnad_debugfs_uninit(bnad);
#endif
	bnad_uninit(bnad);
pci_uninit:
	bnad_pci_uninit(pdev);
free_trc_log:
	bnad_conf_unlock();
	bnad_lock_uninit(bnad);
	bnad_trc_log_aen_uninit(bnad);
free_netdev:
	bnad_remove_from_list(bnad);
	free_netdev(netdev);
	return err;
}


static bnad_exit_t
bnad_pci_remove(struct pci_dev *pdev)
{
	unsigned long flags;
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct bnad_s *bnad;
	struct bna_s *bna;

	if (!netdev)
		return;

	printk(KERN_INFO "%s bnad_pci_remove\n", netdev->name);
	bnad = netdev_priv(netdev);
	bna = &bnad->bna;
	bfa_trc(bnad, bna->ident.id);

	bnad_remove_from_list(bnad);

	if (test_and_clear_bit(BNAD_RF_NETDEV_REGISTERED, &bnad->run_flags))
		unregister_netdev(netdev);
	bnad_conf_lock();
	bnad_ioceth_disable(bnad);
	BNAD_RBL_UNINIT(bnad);
	del_timer_sync(&bnad->ioc_timer);
	spin_lock_irqsave(&bnad->bna_lock, flags);
	bna_uninit(bna);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);
	bnad_free_netq_array(bnad);
	bnad_res_free(bnad, &bnad->mod_res_info[0], BNA_MOD_RES_T_MAX);
	bnad_res_free(bnad, &bnad->res_info[0], BNA_RES_T_MAX);
	bnad_mbox_irq_free(bnad);
	bnad_disable_msix(bnad);
	bnad_pci_uninit(pdev);
	bnad_conf_unlock();
	bnad_lock_uninit(bnad);
	/* Remove the debugfs node for this bnad */
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 31))
	kfree(bnad->regdata);
	bnad_debugfs_uninit(bnad);
	if (bnad->flash_buffer)
		vfree(bnad->flash_buffer);
#endif
	bnad_uninit(bnad);
	bnad_trc_log_aen_uninit(bnad);
	free_netdev(netdev);
}

/* Utilities */

/* Should be held with conf_lock held */
void
bnad_destroy_tx(struct bnad_s *bnad, uint32_t tx_id)
{
	struct bnad_tx_info_s *tx_info = &bnad->tx_info[tx_id];
	struct bna_res_info_s *res_info = &bnad->tx_res_info[tx_id].res_info[0];
	unsigned long flags;

	if (!tx_info->tx)
		return;

	init_completion(&bnad->bnad_completions.tx_comp);
	spin_lock_irqsave(&bnad->bna_lock, flags);
	bna_tx_stats_clr(tx_info->tx);
	bna_tx_disable(tx_info->tx, BNA_HARD_CLEANUP, bnad_cb_tx_disabled);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);
	wait_for_completion(&bnad->bnad_completions.tx_comp);

	if (tx_info->tcb[0]->intr_type == BNA_INTR_T_MSIX)
		bnad_tx_msix_unregister(bnad, tx_info,
			bnad->num_txq_per_tx);

	spin_lock_irqsave(&bnad->bna_lock, flags);
	bna_tx_destroy(tx_info->tx);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	tx_info->tx = NULL;
	tx_info->tx_id = 0;

	bnad_tx_res_free(bnad, res_info);
}

/* Should be held with conf_lock held */
int
bnad_setup_tx(struct bnad_s *bnad, uint32_t tx_id)
{
	int err;
	unsigned long flags;
	struct bnad_tx_info_s *tx_info = &bnad->tx_info[tx_id];
	struct bna_res_info_s *res_info = &bnad->tx_res_info[tx_id].res_info[0];
	struct bna_intr_info_s *intr_info =
			&res_info[BNA_TX_RES_INTR_T_TXCMPL].res_u.intr_info;
	struct bna_tx_config_s *tx_config = &bnad->tx_config[tx_id];
	struct bna_tx_event_cbfn_s tx_cbfn;
	struct bna_tx_s *tx;

	bfa_trc(bnad, bnad->num_txq_per_tx);
	bfa_trc(bnad, bnad->txq_depth);

	tx_info->tx_id = tx_id;

	/* Initialize the Tx object configuration */
	tx_config->num_txq = bnad->num_txq_per_tx;
	tx_config->txq_depth = bnad->txq_depth;
	tx_config->tx_type = BNA_TX_T_REGULAR;
	tx_config->coalescing_timeo = bnad->tx_coalescing_timeo;

	/* Initialize the tx event handlers */
	tx_cbfn.tcb_setup_cbfn = bnad_cb_tcb_setup;
	tx_cbfn.tcb_destroy_cbfn = bnad_cb_tcb_destroy;
	tx_cbfn.tx_stall_cbfn = bnad_cb_tx_stall;
	tx_cbfn.tx_resume_cbfn = bnad_cb_tx_resume;
	tx_cbfn.tx_cleanup_cbfn = bnad_cb_tx_cleanup;

	/* Get BNA's resource requirement for one tx object */
	spin_lock_irqsave(&bnad->bna_lock, flags);
	bna_tx_res_req(bnad->num_txq_per_tx,
		bnad->txq_depth, res_info);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	/* Fill Unmap Q memory requirements */
	BNAD_FILL_UNMAPQ_MEM_REQ(&res_info[BNA_TX_RES_MEM_T_UNMAPQ],
		bnad->num_txq_per_tx, (sizeof(struct bnad_tx_unmap_s) *
		bnad->txq_depth));

	/* Allocate resources */
	err = bnad_tx_res_alloc(bnad, res_info, tx_id);
	if (err)
		return err;

	/* Ask BNA to create one Tx object, supplying required resources */
	spin_lock_irqsave(&bnad->bna_lock, flags);
	tx = bna_tx_create(&bnad->bna, bnad, tx_config, &tx_cbfn, res_info,
			tx_info);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	if (!tx) {
		err = -ENOMEM;
		printk(KERN_ERR "bna: %s Failed to create Tx\n",
			bnad->netdev->name);
		goto err_return;
	}

	tx_info->tx = tx;

	tx->veb_enable =
	(bna_veb_enable)?BNA_STATUS_T_ENABLED:BNA_STATUS_T_DISABLED;

	BNAD_INIT_DELAYED_WORK(&tx_info->tx_cleanup_work,
			(bnad_work_func_t)bnad_tx_cleanup);

	/* Register ISR for the Tx object */
	if (intr_info->intr_type == BNA_INTR_T_MSIX) {
		CNA_ASSERT_DEBUG(bnad->num_txq_per_tx
			== intr_info->num);
		err = bnad_tx_msix_register(bnad, tx_info,
			tx_id, bnad->num_txq_per_tx);
		if (err) {
			printk(KERN_ERR "bna: %s Tx MSIX failed. rc: %d\n",
				bnad->netdev->name, err);
			goto cleanup_tx;
		}
	}

	spin_lock_irqsave(&bnad->bna_lock, flags);
	bna_tx_enable(tx);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	return 0;

cleanup_tx:
	spin_lock_irqsave(&bnad->bna_lock, flags);
	bna_tx_destroy(tx_info->tx);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	tx_info->tx = NULL;
	tx_info->tx_id = 0;

err_return:
	bnad_tx_res_free(bnad, res_info);
	return err;
}


int
bnad_tx_prio_set(struct bnad_s *bnad, uint32_t tx_id, uint8_t priority)
{
	unsigned long flags;
	struct bnad_tx_info_s 	*tx_info = &bnad->tx_info[tx_id];

	if (!tx_info->tx)
		return 1;
	spin_lock_irqsave(&bnad->bna_lock, flags);
	bna_tx_prio_set(tx_info->tx, priority, NULL);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	return 0;
}

/* Setup the rx config for bna_rx_create */
/* Shim decides the configuration */
static void
bnad_init_rx_config(struct bnad_s *bnad, struct bna_rx_config_s *rx_config)
{
	memset(rx_config, 0, sizeof(*rx_config));
	rx_config->rx_type = BNA_RX_T_REGULAR;
	rx_config->num_paths = bnad->num_rxp_per_rx;
	rx_config->coalescing_timeo = bnad->rx_coalescing_timeo;

	if (bnad->num_rxp_per_rx > 1) {
		rx_config->rss_status = BNA_STATUS_T_ENABLED;
		rx_config->rss_config.hash_type =
				(BFI_ENET_RSS_IPV6 |
				 BFI_ENET_RSS_IPV6_TCP |
				 BFI_ENET_RSS_IPV4  |
				 BFI_ENET_RSS_IPV4_TCP);
		rx_config->rss_config.hash_mask =
				bnad->num_rxp_per_rx - 1;
		get_random_bytes(rx_config->rss_config.toeplitz_hash_key,
			sizeof(rx_config->rss_config.toeplitz_hash_key));
	} else {
		rx_config->rss_status = BNA_STATUS_T_DISABLED;
		memset(&rx_config->rss_config, 0,
		       sizeof(rx_config->rss_config));
	}

	rx_config->frame_size = BNAD_FRAME_SIZE(bnad->netdev->mtu);
	rx_config->q0_multi_buf = BNA_STATUS_T_DISABLED;

	/*
	 * BNA_RXP_SINGLE - one data-buffer queue
	 * BNA_RXP_SLR - one small-buffer and one large-buffer queues
	 * BNA_RXP_HDS - one header-buffer and one data-buffer queues
	 */
	/* TODO: configurable param for queue type */
	rx_config->rxp_type = BNA_RXP_SLR;

	if (BNAD_PCI_DEV_IS_CAT2(bnad) &&
			bnad_enable_multi_buffer &&
			rx_config->frame_size > 4096) {
		CNA_ASSERT_DEBUG(CNA_PAGE_SIZE == 4096);
		/*
		 * though size_routing_enable is set in SLR,
		 * small packets may get routed to same rxq.
		 * set buf_size to 2048 instead of CNA_PAGE_SIZE.
		 */
		rx_config->q0_buf_size = 2048;
		/* this should be in multiples of 2 */
		rx_config->q0_num_vecs = 4;
		rx_config->q0_depth = bnad->rxq_depth * rx_config->q0_num_vecs;
		rx_config->q0_multi_buf = BNA_STATUS_T_ENABLED;
	} else {
		rx_config->q0_buf_size = rx_config->frame_size;
		rx_config->q0_num_vecs = 1;
		rx_config->q0_depth = bnad->rxq_depth;
	}

	/* initialize for q1 for BNA_RXP_SLR/BNA_RXP_HDS */
	if (rx_config->rxp_type == BNA_RXP_SLR) {
		rx_config->q1_depth = bnad->rxq_depth;
		rx_config->q1_buf_size = BFI_SMALL_RXBUF_SIZE;
	}

	bnad_set_vlan_strip_status(bnad, rx_config);
}

static void
bnad_rx_ctrl_init(struct bnad_s *bnad, uint32_t rx_id)
{
	struct bnad_rx_info_s *rx_info = &bnad->rx_info[rx_id];
	int i;

	for (i = 0; i < bnad->num_rxp_per_rx; i++)
		rx_info->rx_ctrl[i].bnad = bnad;
}

/* Called with bnad_conf_lock() held */
uint32_t
bnad_reinit_rx(struct bnad_s *bnad)
{
	struct net_device *netdev = bnad->netdev;
	uint32_t err = 0, current_err = 0;
	uint32_t rx_id = 0, count = 0;
	unsigned long flags;

	/* destroy and create new rx objects */
	for (rx_id = 0; rx_id < bnad->num_rx; rx_id++) {
		if (!bnad->rx_info[rx_id].rx)
			continue;
		bnad_destroy_rx(bnad, rx_id);
	}

	spin_lock_irqsave(&bnad->bna_lock, flags);
	bna_enet_mtu_set(&bnad->bna.enet,
			BNAD_FRAME_SIZE(bnad->netdev->mtu), NULL);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	for (rx_id = 0; rx_id < bnad->num_rx; rx_id++) {
		CNA_ASSERT_DEBUG(bnad->rx_info[rx_id].rx == NULL);
		count++;
		current_err = bnad_setup_rx(bnad, rx_id);
		if (current_err && !err) {
			err = current_err;
			printk(KERN_ERR "RXQ:%u setup failed\n", rx_id);
		}

		/*
		 * if not cleared, dev_close gets stuck, in
		 * kernels having older implementation of NAPI.
		 */
		BNAD_COMPAT_NAPI_ENABLE(bnad, rx_id);
	}

	/* restore rx configuration */
	if (bnad->rx_info[0].rx && !err) {
		bnad_restore_vlans(bnad, 0);
		bnad_enable_default_bcast(bnad);
		spin_lock_irqsave(&bnad->bna_lock, flags);
		bnad_mac_addr_set_locked(bnad, netdev->dev_addr);
		spin_unlock_irqrestore(&bnad->bna_lock, flags);
		bnad_set_rx_mode(netdev);
		bnad_vlan_rx_config(bnad);
	}

	bfa_trc(bnad, count);

	return count;
}

/* Called with bnad_conf_lock() held */
void
bnad_destroy_rx(struct bnad_s *bnad, uint32_t rx_id)
{
	unsigned long flags;
	struct bnad_rx_info_s *rx_info = &bnad->rx_info[rx_id];
	struct bna_rx_config_s *rx_config = &bnad->rx_config[rx_id];
	struct bna_res_info_s *res_info = &bnad->rx_res_info[rx_id].res_info[0];

	if (!rx_info->rx)
		return;

	init_completion(&bnad->bnad_completions.rx_comp);
	spin_lock_irqsave(&bnad->bna_lock, flags);
	bna_rx_stats_clr(rx_info->rx);
	bna_rx_disable(rx_info->rx, BNA_HARD_CLEANUP, bnad_cb_rx_disabled);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);
	wait_for_completion(&bnad->bnad_completions.rx_comp);

	if (rx_info->rx_ctrl[0].ccb->intr_type == BNA_INTR_T_MSIX)
		bnad_rx_msix_unregister(bnad, rx_info, rx_config->num_paths);

	BNAD_NAPI_DEL(bnad, rx_id);

	spin_lock_irqsave(&bnad->bna_lock, flags);
	bna_rx_destroy(rx_info->rx);

	rx_info->rx = NULL;
	rx_info->rx_id = 0;
	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	bnad_lro_uninit(bnad, rx_id);
	bnad_rx_res_free(bnad, res_info);
}

/* Called with bnad_conf_lock() held */
int
bnad_setup_rx(struct bnad_s *bnad, uint32_t rx_id)
{
	int err;
	unsigned long flags;
	struct bnad_rx_info_s *rx_info = &bnad->rx_info[rx_id];
	struct bna_res_info_s *res_info = &bnad->rx_res_info[rx_id].res_info[0];
	struct bna_intr_info_s *intr_info =
			&res_info[BNA_RX_RES_T_INTR].res_u.intr_info;
	struct bna_rx_config_s *rx_config = &bnad->rx_config[rx_id];
	struct bna_rx_event_cbfn_s rx_cbfn;
	struct bna_rx_s *rx;

	bfa_trc(bnad, bnad->rxq_depth);
	bfa_trc(bnad, bnad->num_rxp_per_rx);

	rx_info->rx_id = rx_id;

	/* Initialize the Rx object configuration */
	bnad_init_rx_config(bnad, rx_config);

	/* Initialize the Rx event handlers */
	rx_cbfn.rcb_setup_cbfn = NULL;
	rx_cbfn.rcb_destroy_cbfn = NULL;
	rx_cbfn.ccb_setup_cbfn = bnad_cb_ccb_setup;
	rx_cbfn.ccb_destroy_cbfn = bnad_cb_ccb_destroy;
	rx_cbfn.rx_stall_cbfn = bnad_cb_rx_stall;
	rx_cbfn.rx_cleanup_cbfn = bnad_cb_rx_cleanup;
	rx_cbfn.rx_post_cbfn = bnad_cb_rx_post;

	/* Get BNA's resource requirement for one Rx object */
	spin_lock_irqsave(&bnad->bna_lock, flags);
	bna_rx_res_req(rx_config, res_info);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	/* Fill Unmap Q memory requirements */
	BNAD_FILL_UNMAPQ_MEM_REQ(&res_info[BNA_RX_RES_MEM_T_UNMAPDQ],
			rx_config->num_paths,
			(rx_config->q0_depth *
			 sizeof(struct bnad_rx_unmap_s)) +
			 sizeof(struct bnad_rx_unmap_q_s));

	if (rx_config->rxp_type != BNA_RXP_SINGLE) {
		BNAD_FILL_UNMAPQ_MEM_REQ(&res_info[BNA_RX_RES_MEM_T_UNMAPHQ],
				rx_config->num_paths,
				(rx_config->q1_depth *
				 sizeof(struct bnad_rx_unmap_s) +
				 sizeof(struct bnad_rx_unmap_q_s)));
	}

	/* Allocate resource */
	err = bnad_rx_res_alloc(bnad, res_info, rx_id);
	if (err)
		return err;

	bnad_rx_ctrl_init(bnad, rx_id);

	err = bnad_lro_init(bnad, rx_id);
	if (err)
		return err;

	/* Ask BNA to create one Rx object, supplying required resources */
	spin_lock_irqsave(&bnad->bna_lock, flags);
	rx = bna_rx_create(&bnad->bna, bnad, rx_config, &rx_cbfn, res_info,
			rx_info);
	if (!rx) {
		err = -ENOMEM;
		spin_unlock_irqrestore(&bnad->bna_lock, flags);
		printk(KERN_ERR "bna: %s Failed to create Rx\n",
			bnad->netdev->name);
		goto err_return;
	}
	rx_info->rx = rx;
	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	BNAD_INIT_WORK(&rx_info->rx_cleanup_work,
			(bnad_work_func_t)(bnad_rx_cleanup),
			(void *)&rx_info->rx_cleanup_work);

	/*
	 * Init NAPI, so that state is set to NAPI_STATE_SCHED,
	 * so that IRQ handler cannot schedule NAPI at this point.
	 */
	BNAD_NAPI_ADD(bnad, rx_id);

	/* Register ISR for the Rx object */
	if (intr_info->intr_type == BNA_INTR_T_MSIX) {
		CNA_ASSERT_DEBUG(rx_config->num_paths == intr_info->num);
		err = bnad_rx_msix_register(bnad, rx_info, rx_id,
						rx_config->num_paths);
		if (err) {
			printk(KERN_ERR "bna: %s Rx MSIX failed. rc: %d\n",
				bnad->netdev->name, err);
			goto err_return;
		}
	}

	spin_lock_irqsave(&bnad->bna_lock, flags);
	if (0 == rx_id) {
		/* Enable VLAN filtering only on the default Rx */
		bna_rx_vlanfilter_enable(rx);
	}

	bna_rx_enable(rx);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	return 0;

err_return:
	/* FIXME: rather call bna_rx_destroy() & bnad_rx_res_free() here */
	bnad_destroy_rx(bnad, rx_id);
	return err;
}

/* Called with conf_lock & bnad->bna_lock held */
void
bnad_tx_coalescing_timeo_set(struct bnad_s *bnad)
{
	struct bnad_tx_info_s 	*tx_info;
	int 	i;

	for (i = 0; i < bnad->num_tx; i++) {
		tx_info = &bnad->tx_info[i];
		if (!tx_info->tx)
			continue;
		bna_tx_coalescing_timeo_set(tx_info->tx,
				bnad->tx_coalescing_timeo);
	}
}

/* Called with conf_lock & bnad->bna_lock held */
void
bnad_rx_coalescing_timeo_set(struct bnad_s *bnad)
{
	struct bnad_rx_info_s 	*rx_info;
	int 	i;

	for (i = 0; i < bnad->num_rx; i++) {
		rx_info = &bnad->rx_info[i];
		if (!rx_info->rx)
			continue;
		bna_rx_coalescing_timeo_set(rx_info->rx,
				bnad->rx_coalescing_timeo);
	}
}

/*
 * Called with bnad->bna_lock held
 */
int
bnad_mac_addr_set_locked(struct bnad_s *bnad, uint8_t *mac_addr)
{
	int ret;

	if (!is_valid_ether_addr(mac_addr))
		return -EADDRNOTAVAIL;

	/* If datapath is down, pretend everything went through */
	if (!bnad->rx_info[0].rx)
		return 0;

	ret = bna_rx_ucast_set(bnad->rx_info[0].rx, mac_addr, NULL);
	if (ret != BNA_CB_SUCCESS)
		return -EADDRNOTAVAIL;

	return 0;
}

/* Should be called with conf_lock held */
int
bnad_enable_default_bcast(struct bnad_s *bnad)
{
	struct bnad_rx_info_s *rx_info = &bnad->rx_info[0];
	unsigned long flags;
	int ret;

	init_completion(&bnad->bnad_completions.mcast_comp);

	spin_lock_irqsave(&bnad->bna_lock, flags);
	ret = bna_rx_mcast_add(rx_info->rx, (uint8_t *)bnad_bcast_addr,
				bnad_cb_rx_mcast_add);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	if (ret == BNA_CB_SUCCESS)
		wait_for_completion(&bnad->bnad_completions.mcast_comp);
	else
		return -ENODEV;

	if (bnad->bnad_completions.mcast_comp_status != BNA_CB_SUCCESS)
		return -ENODEV;

	return 0;
}


/* Statistics utilities */
void
bnad_netdev_qstats_fill(struct bnad_s *bnad)
{
	struct net_device_stats *net_stats = &bnad->net_stats;
	int i, j;

	for (i = 0; i < bnad->num_rx; i++) {
		for (j = 0; j < bnad->num_rxp_per_rx; j++) {
			if (bnad->rx_info[i].rx_ctrl[j].ccb) {
				net_stats->rx_packets += bnad->rx_info[i].
				rx_ctrl[j].ccb->rcb[0]->rxq->rx_packets;
				net_stats->rx_bytes += bnad->rx_info[i].
					rx_ctrl[j].ccb->rcb[0]->rxq->rx_bytes;
				if (bnad->rx_info[i].rx_ctrl[j].ccb->rcb[1] &&
					bnad->rx_info[i].rx_ctrl[j].ccb->
					rcb[1]->rxq) {
					net_stats->rx_packets +=
						bnad->rx_info[i].rx_ctrl[j].
						ccb->rcb[1]->rxq->rx_packets;
					net_stats->rx_bytes +=
						bnad->rx_info[i].rx_ctrl[j].
						ccb->rcb[1]->rxq->rx_bytes;
				}
			}
		}
	}
	for (i = 0; i < bnad->num_tx; i++) {
		for (j = 0; j < bnad->num_txq_per_tx; j++) {
			if (bnad->tx_info[i].tcb[j]) {
				net_stats->tx_packets +=
				bnad->tx_info[i].tcb[j]->txq->tx_packets;
				net_stats->tx_bytes +=
					bnad->tx_info[i].tcb[j]->txq->tx_bytes;
			}
		}
	}
}

/*
 * Must be called with the bna_lock held.
 */
void
bnad_netdev_hwstats_fill(struct bnad_s *bnad)
{
	struct bfi_enet_stats_mac *mac_stats;
	struct net_device_stats *net_stats = &bnad->net_stats;
	uint32_t bmap;
	int i;

	mac_stats = &bnad->stats.bna_stats->hw_stats.mac_stats;
	net_stats->rx_errors =
		mac_stats->rx_fcs_error + mac_stats->rx_alignment_error +
		mac_stats->rx_frame_length_error + mac_stats->rx_code_error +
		mac_stats->rx_undersize;
	net_stats->tx_errors = mac_stats->tx_fcs_error +
					mac_stats->tx_undersize;
#if 0
	net_stats->rx_errors += mac_stats->rx_oversize;
	net_stats->tx_errors += mac_stats->tx_oversize;
#endif
	net_stats->rx_dropped = mac_stats->rx_drop;
	net_stats->tx_dropped = mac_stats->tx_drop;
	net_stats->multicast = mac_stats->rx_multicast;
	net_stats->collisions = mac_stats->tx_total_collision;

	net_stats->rx_length_errors = mac_stats->rx_frame_length_error;

	/* receive ring buffer overflow  ?? */

	net_stats->rx_crc_errors = mac_stats->rx_fcs_error;
	net_stats->rx_frame_errors = mac_stats->rx_alignment_error;
	/* recv'r fifo overrun */
	bmap = bna_rx_rid_mask(&bnad->bna);
	for (i = 0; bmap; i++) {
		if (bmap & 1) {
			net_stats->rx_fifo_errors +=
				bnad->stats.bna_stats->
					hw_stats.rxf_stats[i].frame_drops;
			break;
		}
		bmap >>= 1;
	}
}

int
bnad_hw_stats_get(struct bnad_s *bnad)
{
	return 0;
}

/*
 * Used spin_lock to synchronize reading of stats structures, which
 * is written by BNA under the same lock.
 */
struct net_device_stats *
bnad_get_netdev_stats(struct net_device *netdev)
{
	struct bnad_s *bnad = netdev_priv(netdev);
	unsigned long flags;

	spin_lock_irqsave(&bnad->bna_lock, flags);

	memset(&bnad->net_stats, 0, sizeof(struct net_device_stats));

	bnad_netdev_qstats_fill(bnad);
	bnad_netdev_hwstats_fill(bnad);

	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	return &bnad->net_stats;
}

int
bnad_stats_clr(struct bnad_s *bnad)
{
	int i, j;
	unsigned long flags;

	memset(&bnad->stats.drv_stats, 0, sizeof(struct bnad_drv_stats_s));
	memset(&bnad->net_stats, 0, sizeof(struct net_device_stats));
	spin_lock_irqsave(&bnad->bna_lock, flags);
	/* Clear lro statistics from stack */
	bnad_lro_stats_clear(bnad);
	for (i = 0; i < bnad->num_rx; i++) {
		if (!bnad->rx_info[i].rx)
			continue;
		for (j = 0; j < bnad->num_rxp_per_rx; j++) {
			if (bnad->rx_info[i].rx_ctrl[j].ccb) {
				bnad->rx_info[i].rx_ctrl[j].rx_intr_ctr = 0;
				bnad->rx_info[i].rx_ctrl[j].rx_poll_ctr = 0;
				bnad->rx_info[i].rx_ctrl[j].rx_schedule = 0;
				bnad->rx_info[i].rx_ctrl[j].rx_keep_poll = 0;
				bnad->rx_info[i].rx_ctrl[j].rx_complete = 0;
				bnad->rx_info[i].rx_ctrl[j].ccb->
					rcb[0]->rxq->rx_packets = 0;
				bnad->rx_info[i].rx_ctrl[j].ccb->
					rcb[0]->rxq->rx_bytes = 0;
				bnad->rx_info[i].rx_ctrl[j].ccb->
					rcb[0]->rxq->rx_packets_with_error = 0;
				bnad->rx_info[i].rx_ctrl[j].ccb->
					rcb[0]->rxq->rxbuf_alloc_failed = 0;
				if (bnad->rx_info[i].rx_ctrl[j].ccb->rcb[1] &&
					bnad->rx_info[i].rx_ctrl[j].ccb->
					rcb[1]->rxq) {
						bnad->rx_info[i].rx_ctrl[j].
						ccb->rcb[1]->rxq->
						rx_packets = 0;
						bnad->rx_info[i].rx_ctrl[j].
						ccb->rcb[1]->rxq->rx_bytes = 0;
						bnad->rx_info[i].rx_ctrl[j].
						ccb->rcb[1]->rxq->
						rx_packets_with_error = 0;
						bnad->rx_info[i].rx_ctrl[j].
						ccb->rcb[1]->rxq->
						rxbuf_alloc_failed = 0;
				}
			}
		}
	}
	for (i = 0; i < bnad->num_tx; i++) {
		if (!bnad->tx_info[i].tx)
			continue;
		for (j = 0; j < bnad->num_txq_per_tx; j++) {
			if (bnad->tx_info[i].tcb[j]) {
				bnad->tx_info[i].tcb[j]->txq->tx_packets = 0;
				bnad->tx_info[i].tcb[j]->txq->tx_bytes = 0;
			}
		}
	}
	spin_unlock_irqrestore(&bnad->bna_lock, flags);
	return 0;
}

void
bnad_drv_stats_get(struct bnad_s *bnad, struct bnad_ethport_stats_s *stats)
{
	memset(stats, 0, sizeof(struct bnad_ethport_stats_s));

	/* Rx */
	stats->rx_drop = bnad->stats.drv_stats.netif_rx_dropped;
	stats->rx_alloc_fail = bnad->stats.drv_stats.rxbuf_alloc_failed;

	/* Tx */
	stats->tx_lso4 = bnad->stats.drv_stats.tso4;
	stats->tx_lso6 = bnad->stats.drv_stats.tso6;
	stats->tx_lso_err = bnad->stats.drv_stats.tso_err;
	stats->tx_tcp_cso = bnad->stats.drv_stats.tcpcsum_offload;
	stats->tx_udp_cso = bnad->stats.drv_stats.udpcsum_offload;
	stats->tx_csum_help = bnad->stats.drv_stats.csum_help;

	stats->tx_skb_too_short = bnad->stats.drv_stats.tx_skb_too_short;
	stats->tx_skb_stopping = bnad->stats.drv_stats.tx_skb_stopping;
	stats->tx_skb_max_vectors = bnad->stats.drv_stats.tx_skb_max_vectors;
	stats->tx_skb_mss_too_long = bnad->stats.drv_stats.tx_skb_mss_too_long;
	stats->tx_skb_tso_too_short =
		bnad->stats.drv_stats.tx_skb_tso_too_short;
	stats->tx_skb_tso_prepare = bnad->stats.drv_stats.tx_skb_tso_prepare;
	stats->tx_skb_non_tso_too_long =
		bnad->stats.drv_stats.tx_skb_non_tso_too_long;
	stats->tx_skb_tcp_hdr = bnad->stats.drv_stats.tx_skb_tcp_hdr;
	stats->tx_skb_udp_hdr = bnad->stats.drv_stats.tx_skb_udp_hdr;
	stats->tx_skb_csum_err = bnad->stats.drv_stats.tx_skb_csum_err;
	stats->tx_skb_headlen_too_long =
		bnad->stats.drv_stats.tx_skb_headlen_too_long;
	stats->tx_skb_headlen_zero = bnad->stats.drv_stats.tx_skb_headlen_zero;
	stats->tx_skb_frag_zero = bnad->stats.drv_stats.tx_skb_frag_zero;
	stats->tx_skb_len_mismatch = bnad->stats.drv_stats.tx_skb_len_mismatch;

	/* Ctrl */
	stats->link_toggle = bnad->stats.drv_stats.link_toggle;
	stats->cee_toggle = bnad->stats.drv_stats.cee_toggle;
	stats->mbox_intr_disable = bnad->stats.drv_stats.mbox_intr_disabled;
	stats->mbox_intr_enable = bnad->stats.drv_stats.mbox_intr_enabled;
	stats->tx_stop = bnad->stats.drv_stats.netif_queue_stop;
	stats->tx_wakeup = bnad->stats.drv_stats.netif_queue_wakeup;
	stats->fw_stats_query = bnad->stats.drv_stats.hw_stats_updates;
}

static void
bnad_mbox_irq_sync(struct bnad_s *bnad)
{
	uint32_t irq;
	unsigned long flags;

	bfa_trc(bnad, bnad->cfg_flags);

	spin_lock_irqsave(&bnad->bna_lock, flags);
	if (bnad->cfg_flags & BNAD_CF_MSIX)
		irq = bnad->msix_table[BNAD_MAILBOX_MSIX_INDEX].vector;
	else
		irq = bnad->pcidev->irq;
	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	bfa_trc(bnad, irq);
	synchronize_irq(irq);
}

/* Netdev entry points */
int
bnad_open(struct net_device *netdev)
{
	int err;
	unsigned long flags;
	struct bnad_s *bnad = netdev_priv(netdev);
	struct bna_pause_config_s pause_config;

	bnad_conf_lock();

	/* Tx */
	err = bnad_setup_tx(bnad, 0);
	if (err)
		goto err_return;

	/* Rx */
	err = bnad_setup_rx(bnad, 0);
	if (err)
		goto cleanup_tx;

	/* Port */
	pause_config.tx_pause = 0;
	pause_config.rx_pause = 0;

	spin_lock_irqsave(&bnad->bna_lock, flags);
	bna_enet_mtu_set(&bnad->bna.enet,
			BNAD_FRAME_SIZE(bnad->netdev->mtu), NULL);
	bna_enet_pause_config(&bnad->bna.enet, &pause_config, NULL);
	bna_enet_enable(&bnad->bna.enet);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	/* Enable broadcast */
	bnad_enable_default_bcast(bnad);

	/* Restore VLANs, if any */
	bnad_restore_vlans(bnad, 0);

	/* Set the UCAST address */
	spin_lock_irqsave(&bnad->bna_lock, flags);
	bnad_mac_addr_set_locked(bnad, netdev->dev_addr);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	/* Start the stats timer */
	bnad_stats_timer_start(bnad);

	bnad_conf_unlock();

	return 0;

cleanup_tx:
	bnad_destroy_tx(bnad, 0);

err_return:
	bnad_conf_unlock();
	return err;
}

int
bnad_stop(struct net_device *netdev)
{
	struct bnad_s *bnad = netdev_priv(netdev);
	unsigned long flags;

	bnad_conf_lock();

	/* Stop the stats timer */
	bnad_stats_timer_stop(bnad);

	init_completion(&bnad->bnad_completions.enet_comp);

	spin_lock_irqsave(&bnad->bna_lock, flags);
	bna_enet_disable(&bnad->bna.enet, BNA_HARD_CLEANUP,
			bnad_cb_enet_disabled);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	wait_for_completion(&bnad->bnad_completions.enet_comp);

	bnad_netq_cleanup(bnad);

	bnad_destroy_tx(bnad, 0);
	bnad_destroy_rx(bnad, 0);

	/* Synchronize mailbox IRQ */
	bnad_mbox_irq_sync(bnad);

	bnad_conf_unlock();

	BNAD_FLUSH_WORKQUEUE(bnad);

	BNAD_CANCEL_NETQ_WORK_SYNC(bnad);
	return 0;
}

static void
bnad_set_rx_mcast_fltr(struct bnad_s *bnad)
{
	struct net_device *netdev = bnad->netdev;
	int mc_count = bnad_netdev_mc_count(netdev);
	enum bna_cb_status_e ret;
	uint8_t *mac_list;

	if (netdev->flags & IFF_ALLMULTI)
		goto mode_allmulti;

	if (bnad_netdev_mc_list_is_empty(netdev))
		return;

	bfa_trc(bnad, mc_count);

	if (mc_count > bna_attr(&bnad->bna)->num_mcmac)
		goto mode_allmulti;

	mac_list = kzalloc((mc_count + 1) * CNA_ETH_ALEN, GFP_ATOMIC);

	if (mac_list == NULL)
		goto mode_allmulti;

	memcpy(&mac_list[0], &bnad_bcast_addr[0], CNA_ETH_ALEN);

	/* copy rest of the MCAST addresses */
	bnad_netdev_mc_list_get(netdev, mac_list);
	ret = bna_rx_mcast_listset(bnad->rx_info[0].rx, mc_count + 1,
			mac_list, NULL);
	kfree(mac_list);

	if (ret != BNA_CB_SUCCESS)
		goto mode_allmulti;

	return;

mode_allmulti:
	bnad->cfg_flags |= BNAD_CF_ALLMULTI;
	bna_rx_mcast_delall(bnad->rx_info[0].rx, NULL);
}

void
bnad_set_rx_mode(struct net_device *netdev)
{
	struct bnad_s *bnad = netdev_priv(netdev);
	enum bna_rxmode_e new_mode, mode_mask;
	unsigned long flags;

	bfa_trc(bnad, netdev->flags);

	spin_lock_irqsave(&bnad->bna_lock, flags);

	if (bnad->rx_info[0].rx == NULL) {
		bfa_trc(bnad, netdev->state);
		spin_unlock_irqrestore(&bnad->bna_lock, flags);
		return;
	}

	/* clear bnad flags to update it with new settings */
	bnad->cfg_flags &= ~(BNAD_CF_PROMISC | BNAD_CF_DEFAULT |
			BNAD_CF_ALLMULTI);

	new_mode = 0;
	if (netdev->flags & IFF_PROMISC) {
		new_mode |= BNAD_RXMODE_PROMISC_DEFAULT;
		bnad->cfg_flags |= BNAD_CF_PROMISC;
	} else {
		bnad_set_rx_mcast_fltr(bnad);

		if (bnad->cfg_flags & BNAD_CF_ALLMULTI)
			new_mode |= BNA_RXMODE_ALLMULTI;

		bnad_set_rx_ucast_fltr(bnad);

		if (bnad->cfg_flags & BNAD_CF_DEFAULT)
			new_mode |= BNA_RXMODE_DEFAULT;
	}

	bfa_trc(bnad, new_mode);
	mode_mask = BNA_RXMODE_PROMISC | BNA_RXMODE_DEFAULT |
			BNA_RXMODE_ALLMULTI;
	bna_rx_mode_set(bnad->rx_info[0].rx, new_mode, mode_mask, NULL);

	if (bnad->cfg_flags & BNAD_CF_PROMISC)
		bna_rx_vlan_strip_disable(bnad->rx_info[0].rx);
	else
		bna_rx_vlan_strip_enable(bnad->rx_info[0].rx);

	spin_unlock_irqrestore(&bnad->bna_lock, flags);
}

/*
 * bna_lock is used to sync writes to netdev->addr
 * conf_lock cannot be used since this call may be made
 * in a non-blocking context.
 */
int
bnad_set_mac_address(struct net_device *netdev, void *mac_addr)
{
	int err;
	unsigned long flags;
	struct bnad_s *bnad = netdev_priv(netdev);
	struct sockaddr *sa = (struct sockaddr *)mac_addr;

	spin_lock_irqsave(&bnad->bna_lock, flags);

	err = bnad_mac_addr_set_locked(bnad, sa->sa_data);

	if (!err)
		memcpy(netdev->dev_addr, sa->sa_data, netdev->addr_len);

	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	return err;
}

int
bnad_add_mac_address_locked(struct bnad_s *bnad, uint8_t *mac_addr,
			    uint32_t rx_id)
{
	int ret;
	unsigned long flags;

	if (!is_valid_ether_addr(mac_addr)) {
		return -EADDRNOTAVAIL;
	}

	init_completion(&bnad->bnad_completions.ucast_comp);

	spin_lock_irqsave(&bnad->bna_lock, flags);
	ret = bna_rx_ucast_add(bnad->rx_info[rx_id].rx, mac_addr,
				bnad_cb_rx_ucast_add);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	if (ret == BNA_CB_SUCCESS)
		wait_for_completion(&bnad->bnad_completions.ucast_comp);
	else
		return -EADDRNOTAVAIL;

	if (bnad->bnad_completions.ucast_comp_status != BNA_CB_SUCCESS) {
		return -EADDRNOTAVAIL;
	}
	return 0;
}

int
bnad_del_mac_address_locked(struct bnad_s *bnad, uint8_t *mac_addr,
			    uint32_t rx_id)
{
	int ret;
	unsigned long flags;

	if (!is_valid_ether_addr(mac_addr))
		return -EADDRNOTAVAIL;

	if (!bnad->rx_info[rx_id].rx)
		return 0;

	init_completion(&bnad->bnad_completions.ucast_comp);
	spin_lock_irqsave(&bnad->bna_lock, flags);
	ret = bna_rx_ucast_del(bnad->rx_info[rx_id].rx, mac_addr,
				bnad_cb_rx_ucast_del);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	if (ret == BNA_CB_SUCCESS)
		wait_for_completion(&bnad->bnad_completions.ucast_comp);
	else
		return -EADDRNOTAVAIL;

	if (bnad->bnad_completions.ucast_comp_status != BNA_CB_SUCCESS) {
		return -EADDRNOTAVAIL;
	}

	return 0;
}

int
bnad_mtu_set(struct bnad_s *bnad, int frame_size)
{
	unsigned long flags;

	init_completion(&bnad->bnad_completions.mtu_comp);

	spin_lock_irqsave(&bnad->bna_lock, flags);
	bna_enet_mtu_set(&bnad->bna.enet, frame_size, bnad_cb_enet_mtu_set);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	wait_for_completion(&bnad->bnad_completions.mtu_comp);

	return bnad->bnad_completions.mtu_comp_status;
}

int
bnad_change_mtu(struct net_device *netdev, int new_mtu)
{
	struct bnad_s *bnad = netdev_priv(netdev);
	uint32_t rx_count = 0, frame, new_frame;
	int err, mtu;


	if (new_mtu + ETH_HLEN < ETH_ZLEN || new_mtu > BNAD_JUMBO_MTU)
		return -EINVAL;

	bnad_conf_lock();

	mtu = netdev->mtu;
	netdev->mtu = new_mtu;

	frame = BNAD_FRAME_SIZE(mtu);
	new_frame = BNAD_FRAME_SIZE(new_mtu);

	/* check if multi-buffer needs to be enabled */
	if (BNAD_PCI_DEV_IS_CAT2(bnad) && bnad_enable_multi_buffer &&
						netif_running(bnad->netdev)) {
		/* only when transition is over 4K */
		if ((frame <= 4096 && new_frame > 4096) ||
				(frame > 4096 && new_frame <= 4096))
			rx_count = bnad_reinit_rx(bnad);
	}

	/*
	 * rx_count > 0 - new rx created
	 *	- VMware reinit NetQ
	 *	- Linux set err = 0 and return
	 */
	if (bnad_netq_mtu_reinit_needed(bnad, mtu, new_mtu) || rx_count)
		err = bnad_netq_mtu_reinit(bnad);
	else
		err = bnad_mtu_set(bnad, new_frame);

	if (err)
		err = -EBUSY;

	bnad_conf_unlock();
	return err;
}

/* Logging function used by bfa_log */
void
bnad_log_printf(struct bfa_log_mod_s *log_mod,
		     uint32_t msg_id, va_list params, const char *fmt, ...)
{
	va_list ap;
	char buf[128];

	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);

	printk(buf);
}

uint32_t
bnad_get_loglevel(struct bnad_s *bnad, int16_t mod_id)
{
	uint32_t msglevel;

	if ((mod_id <= BFA_LOG_UNUSED_ID) ||
		(mod_id > BFA_LOG_MODULE_ID_MAX))
		mod_id = BFA_LOG_LINUX_ID;

	msglevel = bfa_log_get_level(bnad->logmod, mod_id);
	return msglevel;
}

void
bnad_set_loglevel(struct bnad_s *bnad, u32 msglevel, int16_t mod_id)
{
	switch (mod_id) {
		case BFA_LOG_MODULE_ID_ALL:
		case BFA_LOG_UNUSED_ID:
			bfa_log_set_level_all(bnad->logmod, msglevel);
			break;
		case BFA_LOG_AEN_ALL:
			bfa_log_set_level_aen(bnad->logmod, msglevel);
			break;
		case BFA_LOG_DRV_ALL:
			mod_id = BFA_LOG_LINUX_ID;
		default:
			bfa_log_set_level(bnad->logmod, mod_id, msglevel);
			break;
	}
}

/*
 * Called once per module load/unload
 */
static void
bnad_global_uninit(void)
{
	mutex_destroy(&bnad_list_mutex);
}

static void
bnad_global_init(void)
{
	mutex_init(&bnad_list_mutex);
	bfa_ioc_auto_recover(bnad_ioc_auto_recover);
}

static struct pci_driver bnad_pci_driver = {
	.name = BNAD_NAME,
	.id_table = bnad_pci_id_table,
	.probe = bnad_pci_probe,
	.remove = BNAD_EXIT,
};

static int __init
bnad_module_init(void)
{
	int err;

	printk(KERN_INFO "QLogic BR-series 10G Ethernet driver - version: %s\n",
			bfa_version);

	bnad_global_init();

	err = pci_register_driver(&bnad_pci_driver);
	if (err < 0) {
		printk(KERN_ERR "bna : PCI registration failed in module init "
		       "(%d)\n", err);
		return err;
	}

	bnad_ioctl_init();
	return 0;
}

static void __exit
bnad_module_exit(void)
{
	bnad_ioctl_exit();
	pci_unregister_driver(&bnad_pci_driver);
	bnad_global_uninit();
}

module_init(bnad_module_init);
module_exit(bnad_module_exit);

MODULE_AUTHOR("Brocade");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("QLogic BR-series 10G PCIe Ethernet driver");
MODULE_VERSION(BNAD_VERSION);
