/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#include "cna_os.h"
#include "bna.h"

#include "bnad_compat.h"
#include "bnad.h"
#include "bnad_diag_lb_common.h"
#include "bnad_trcmod.h"

#include "bfad_ioctl_cna.h"
#include "bnad_ioctl_common.h"

BNA_TRC_FILE(LDRV, DIAG_LB_CMN);

#define BNAD_DIAG_GEN_INIT_COMPLETION(_comp)\
do {						\
	(_comp)->comp_done = 0;		\
} while (0)

/*
 * Wait for BNAD_DIAG_USLEEP_TIME msecs each time
 * But bail out after BNAD_DIAG_TOTAL_WAIT_LOOP times of loop
 * Currently this time is 1 second
 */
/* Assume that bnad_udelay() does not schedule */
#define BNAD_DIAG_GEN_WAIT_FOR_COMPLETION(_comp)		\
do {								\
	int loop_ctr = 0;					\
	while ((_comp)->comp_done != BNAD_DIAG_STATUS_COMPLETE) {\
		bnad_udelay(BNAD_DIAG_USLEEP_TIME);		\
		if (++loop_ctr >= BNAD_DIAG_TOTAL_WAIT_LOOP) {	\
			(_comp)->comp_done = BNAD_DIAG_STATUS_COMPLETE;\
			(_comp)->comp_status = BNA_CB_FAIL;	\
			break;					\
		}						\
	}							\
} while (0)

#define BNAD_DIAG_GEN_COMPLETE(_comp, _status)	\
do {						\
	(_comp)->comp_status = (_status);	\
	(_comp)->comp_done = BNAD_DIAG_STATUS_COMPLETE;\
} while (0)


#define BNAD_DIAG_INIT_COMPLETION(_diag)	\
	BNAD_DIAG_GEN_INIT_COMPLETION(&((_diag)->comp))

#define BNAD_DIAG_WAIT_FOR_COMPLETION(_diag) 			\
	BNAD_DIAG_GEN_WAIT_FOR_COMPLETION(&((_diag)->comp))

#define BNAD_DIAG_COMPLETE(_diag, _status)	\
	BNAD_DIAG_GEN_COMPLETE(&((_diag)->comp), (_status))

#define BNAD_DIAG_LINK_INIT_COMPLETION(_diag)	\
	BNAD_DIAG_GEN_INIT_COMPLETION(&((_diag)->link_comp))

#define BNAD_DIAG_LINK_WAIT_FOR_COMPLETION(_diag) 			\
	BNAD_DIAG_GEN_WAIT_FOR_COMPLETION(&((_diag)->link_comp))

#define BNAD_DIAG_LINK_COMPLETE(_diag, _status)	\
	BNAD_DIAG_GEN_COMPLETE(&((_diag)->link_comp), (_status))

/* Static Prototypes */
static uint32_t
bnad_diag_lb_txbufs_force_free(struct bnad_s *bnad, struct bna_tcb_s *tcb,
				uint32_t rcvd);
static void
bnad_diag_lb_post_rxbufs(struct bnad_s *bnad, struct bna_rcb_s *rcb);
static void
bnad_diag_lb_rxbufs_free(struct bnad_s *bnad, struct bna_rcb_s *rcb);

/* Callbacks */
static void
bnad_cb_lb_rx_ucast_set(struct bnad_s *bnad, struct bna_rx_s *rx)
{
	BNAD_DIAG_COMPLETE(bnad->diag, BNA_CB_SUCCESS);
}

void
bnad_cb_lb_enet_disabled(void *arg)
{
	struct bnad_s *bnad = (struct bnad_s *)arg;

	BNAD_DIAG_COMPLETE(bnad->diag, BNA_CB_SUCCESS);
}

void
bnad_cb_lb_ethport_admin_up(struct bnad_s *bnad, enum bna_cb_status_e status)
{
	bfa_trc(bnad, status);
	BNAD_DIAG_COMPLETE(bnad->diag, status);
}

void
bnad_cb_lb_ethport_link_status(struct bnad_s *bnad,
			enum bna_link_status_e link_status)
{
	bfa_trc(bnad, link_status);
	bnad->diag->link_status = link_status;
	BNAD_DIAG_LINK_COMPLETE(bnad->diag, BNA_CB_SUCCESS);
}

void
bnad_cb_lb_tx_disabled(void *arg, struct bna_tx_s *tx)
{
	struct bnad_s *bnad = (struct bnad_s *)(arg);

	BNAD_DIAG_COMPLETE(bnad->diag, BNA_CB_SUCCESS);
}

void
bnad_cb_lb_tcb_setup(struct bnad_s *bnad, struct bna_tcb_s *tcb)
{
	struct bnad_diag_tx_info_s *tx_info =
			(struct bnad_diag_tx_info_s *)tcb->txq->tx->priv;
	struct bnad_diag_pkt_s *pkt;
	struct bnad_diag_unmap_q_s *unmap_q = tcb->unmap_q;
	int i;

	tx_info->tcb = tcb;

	CNA_ASSERT(unmap_q);

	unmap_q->producer_index = 0;
	unmap_q->consumer_index = 0;
	unmap_q->q_depth = BNAD_DIAG_UNMAPQ_DEPTH;

	if (!tx_info->mem_info.mdl) {
		bfa_trc(bnad, tx_info->mem_info.num);
		return;
	}
	for (i = 0; i < tx_info->mem_info.num; i++) {
		pkt = &unmap_q->unmap_array[i];
		pkt->data = tx_info->mem_info.mdl[i].kva;
		pkt->hdr.dma.msb = tx_info->mem_info.mdl[i].dma.msb;
		pkt->hdr.dma.lsb = tx_info->mem_info.mdl[i].dma.lsb;
		pkt->hdr.len = tx_info->mem_info.mdl[i].len;
		pkt->hdr.free = 1;
	}
}

void
bnad_cb_lb_tcb_destroy(struct bnad_s *bnad, struct bna_tcb_s *tcb)
{
	struct bnad_diag_tx_info_s *tx_info =
			(struct bnad_diag_tx_info_s *)tcb->txq->tx->priv;
	struct bnad_diag_unmap_q_s *unmap_q = tcb->unmap_q;
	struct bnad_diag_pkt_s *pkt;
	int i;

	if (!tx_info->tcb)
		return;

	CNA_ASSERT(unmap_q);

	for (i = 0; i < tx_info->mem_info.num; i++) {
		pkt = &unmap_q->unmap_array[i];
		pkt->data = NULL;
		pkt->hdr.len = 0;
	}

	tx_info->tcb = NULL;
}


static void
bnad_cb_lb_tx_stall(struct bnad_s *bnad, struct bna_tx_s *tx)
{
	struct bnad_diag_tx_info_s *tx_info =
			(struct bnad_diag_tx_info_s *)tx->priv;

	bfa_trc(bnad, tx->rid);
	CNA_ASSERT(tx_info == &bnad->diag->tx_info);
	bnad->diag->run_flags |= BNAD_DIAG_RF_TX_STALLED;
}

static void
bnad_cb_lb_tx_resume(struct bnad_s *bnad, struct bna_tx_s *tx)
{
	bfa_trc(bnad, tx->rid);
	bnad->diag->run_flags &= ~BNAD_DIAG_RF_TX_STALLED;
}

static void
bnad_cb_lb_tx_cleanup(struct bnad_s *bnad, struct bna_tx_s *tx)
{
	struct bnad_diag_tx_info_s *tx_info =
			(struct bnad_diag_tx_info_s *)tx->priv;
	struct bna_tcb_s *tcb = tx_info->tcb;
	struct bnad_diag_unmap_q_s *unmap_q = tcb->unmap_q;
	struct bnad_diag_pkt_s	*unmap_array, *pkt;
	int i;

	if (!tcb || (!tcb->unmap_q)) {
		bfa_trc(bnad, 0);
		return;
	}

	CNA_ASSERT(unmap_q);

	unmap_array = unmap_q->unmap_array;

	if (!unmap_array) {
		bfa_trc(bnad, unmap_q->producer_index);
		bfa_trc(bnad, unmap_q->consumer_index);
		return;
	}

	for (i = 0; i < unmap_q->q_depth; i++) {
		pkt = &unmap_array[i];
		pkt->hdr.free = 1;
	}

	unmap_q->consumer_index = 0;
	tcb->consumer_index = 0;

	bna_tx_cleanup_complete(tx);
}


static void
bnad_cb_lb_rcb_setup(struct bnad_s *bnad, struct bna_rcb_s *rcb)
{
	struct bnad_diag_pkt_s *pkt;
	struct bnad_diag_rx_info_s *rx_info =
			(struct bnad_diag_rx_info_s *)rcb->rxq->rx->priv;
	struct bnad_diag_unmap_q_s *unmap_q = rcb->unmap_q;
	int i;

	CNA_ASSERT(unmap_q);

	unmap_q->producer_index = 0;
	unmap_q->consumer_index = 0;
	unmap_q->q_depth = BNAD_DIAG_UNMAPQ_DEPTH;

	if (!rx_info->mem_info.mdl) {
		bfa_trc(bnad, rx_info->mem_info.num);
		return;
	}
	for (i = 0; i < rx_info->mem_info.num; i++) {
		pkt = &unmap_q->unmap_array[i];
		pkt->data = rx_info->mem_info.mdl[i].kva;
		pkt->hdr.dma.msb = rx_info->mem_info.mdl[i].dma.msb;
		pkt->hdr.dma.lsb = rx_info->mem_info.mdl[i].dma.lsb;
		pkt->hdr.len = rx_info->mem_info.mdl[i].len;
		pkt->hdr.free = 1;
	}
}

void
bnad_cb_lb_rcb_destroy(struct bnad_s *bnad, struct bna_rcb_s *rcb)
{
	struct bnad_diag_pkt_s *pkt;
	struct bnad_diag_rx_info_s *rx_info =
			(struct bnad_diag_rx_info_s *)rcb->rxq->rx->priv;
	struct bnad_diag_unmap_q_s *unmap_q = rcb->unmap_q;
	int i;

	CNA_ASSERT(unmap_q);

	for (i = 0; i < rx_info->mem_info.num; i++) {
		pkt = &unmap_q->unmap_array[i];
		pkt->data = NULL;
		pkt->hdr.len = 0;
	}

	unmap_q->producer_index = 0;
}

void
bnad_cb_lb_ccb_setup(struct bnad_s *bnad, struct bna_ccb_s *ccb)
{
	struct bnad_diag_rx_info_s *rx_info =
			(struct bnad_diag_rx_info_s *)ccb->cq->rx->priv;

	CNA_ASSERT(rx_info);
	rx_info->ccb = ccb;
	ccb->ctrl = NULL;
}

void
bnad_cb_lb_ccb_destroy(struct bnad_s *bnad, struct bna_ccb_s *ccb)
{
	struct bnad_diag_rx_info_s *rx_info =
			(struct bnad_diag_rx_info_s *)ccb->cq->rx->priv;

	CNA_ASSERT(rx_info);
	rx_info->ccb = NULL;
}

void
bnad_cb_lb_rx_cleanup(struct bnad_s *bnad, struct bna_rx_s *rx)
{
	struct bnad_diag_rx_info_s *rx_info =
			(struct bnad_diag_rx_info_s *)rx->priv;
	struct bna_ccb_s *ccb = rx_info->ccb;

	bfa_trc(bnad, ccb->id);
	bnad->diag->run_flags |= BNAD_DIAG_RF_RX_CLEANED;
	bnad_diag_lb_rxbufs_free(bnad, ccb->rcb[0]);
	bna_rx_cleanup_complete(rx);
}


void
bnad_cb_lb_rx_post(struct bnad_s *bnad, struct bna_rx_s *rx)
{
	struct bnad_diag_rx_info_s *rx_info =
			(struct bnad_diag_rx_info_s *)rx->priv;

	bfa_trc(bnad, rx->rid);
	bnad->diag->run_flags &= ~BNAD_DIAG_RF_RX_CLEANED;

	bnad_diag_lb_post_rxbufs(bnad, rx_info->ccb->rcb[0]);
}

void
bnad_cb_lb_rx_disabled(void *arg, struct bna_rx_s *rx)
{
	struct bnad_s *bnad = (struct bnad_s *)arg;

	BNAD_DIAG_COMPLETE(bnad->diag, BNA_CB_SUCCESS);
}

/* Utils */

static uint32_t
bnad_diag_lb_txbufs_force_free(struct bnad_s *bnad, struct bna_tcb_s *tcb,
				uint32_t rcvd)
{
	uint32_t 		sent_packets = 0, sent_bytes = 0;
	uint16_t 		wis, unmap_cons, updated_hw_cons;
	struct bnad_diag_pkt_s	*unmap_array, *pkt;
	struct bnad_diag_unmap_q_s *unmap_q = tcb->unmap_q;

	updated_hw_cons = tcb->consumer_index;
	BNA_QE_INDX_ADD(updated_hw_cons, rcvd, tcb->q_depth);

	wis = BNA_Q_INDEX_CHANGE(tcb->consumer_index,
				  updated_hw_cons, tcb->q_depth);

	bfa_trc(bnad, tcb->consumer_index);
	bfa_trc(bnad, updated_hw_cons);
	bfa_trc(bnad, wis);

	unmap_array = unmap_q->unmap_array;
	unmap_cons = unmap_q->consumer_index;

	while (wis) {
		pkt = &unmap_array[unmap_cons];

		CNA_ASSERT(((unmap_q->producer_index -
			unmap_cons) & (unmap_q->q_depth - 1)) >= 1);

		sent_packets++;
		sent_bytes += pkt->hdr.len;
		wis -= BNA_TXQ_WI_NEEDED(1);

		pkt->hdr.free = 1;

		BNA_QE_INDX_ADD(unmap_cons, 1, unmap_q->q_depth);
	}

	bna_ib_ack(tcb->i_dbell, wis);

	/* Update consumer pointers. */
	tcb->consumer_index = updated_hw_cons;
	unmap_q->consumer_index = unmap_cons;

	bfa_trc(bnad, tcb->consumer_index);
	bfa_trc(bnad, unmap_q->consumer_index);

	tcb->txq->tx_packets += sent_packets;
	tcb->txq->tx_bytes += sent_bytes;

	CNA_ASSERT(unmap_q->consumer_index == tcb->consumer_index);

	return sent_packets;
}

static void
bnad_diag_lb_rxbufs_free(struct bnad_s *bnad, struct bna_rcb_s *rcb)
{
	struct bnad_diag_unmap_q_s *unmap_q = rcb->unmap_q;
	struct bnad_diag_pkt_s *pkt;

	while (BNA_QE_IN_USE_CNT(unmap_q, unmap_q->q_depth)) {
		pkt = &unmap_q->unmap_array[unmap_q->consumer_index];
		pkt->hdr.free = 1;
		BNA_QE_INDX_ADD(unmap_q->consumer_index, 1, unmap_q->q_depth);
		BNA_QE_INDX_ADD(rcb->consumer_index, 1, rcb->q_depth);
	}

	/* Reset rcb */
	rcb->producer_index = 0;
	rcb->consumer_index = 0;
	unmap_q->producer_index = 0;
	unmap_q->consumer_index = 0;

	*(rcb->ccb->hw_producer_index) = 0;
}


static bfa_status_t
bnad_diag_lb_txrx_irq_alloc(struct bnad_s *bnad, int vector,
				struct bna_intr_info_s *intr_info)
{
	CNA_ASSERT(intr_info->num == 1);

	intr_info->intr_type = bnad_diag_lb_intr_type_get(bnad);

	bfa_trc(bnad, intr_info->intr_type);
	bfa_trc(bnad, intr_info->num);
	bfa_trc(bnad, vector);

	intr_info->idl = cna_os_kcalloc(intr_info->num,
					sizeof(struct bna_intr_descr_s));
	if (!intr_info->idl)
		return BFA_STATUS_ENOMEM;

	intr_info->idl[0].vector = vector;

	return BFA_STATUS_OK;
}

/* Free Tx object Resources */
static void
bnad_diag_lb_tx_res_free(struct bnad_s *bnad, struct bna_res_info_s *res_info)
{
	int i;

	bfa_trc(bnad, 0);

	for (i = 0; i < BNA_TX_RES_T_MAX; i++) {
		if (res_info[i].res_type == BNA_RES_T_MEM)
			bnad_diag_lb_mem_free(bnad,
					      &res_info[i].res_u.mem_info);
		else if (res_info[i].res_type == BNA_RES_T_INTR)
			bnad_diag_lb_tx_irq_free(bnad,
						 &res_info[i].res_u.intr_info);
	}
	bnad_diag_lb_mem_free(bnad, &bnad->diag->tx_info.mem_info);
}

/* Allocates memory and interrupt resources for Tx object */
static bfa_status_t
bnad_diag_lb_tx_res_alloc(struct bnad_s *bnad, struct bna_res_info_s *res_info)
{
	int i, idx;
	bfa_status_t err = BFA_STATUS_OK;

	bfa_trc(bnad, 0);

	for (i = 0; i < BNA_TX_RES_T_MAX; i++) {

		CNA_ASSERT(res_info[i].res_type == BNA_RES_T_MEM ||
			   res_info[i].res_type == BNA_RES_T_INTR);

		if (res_info[i].res_type == BNA_RES_T_MEM) {
			err = bnad_diag_lb_mem_alloc(bnad,
					&res_info[i].res_u.mem_info);
			if (err) {
				bfa_trc(bnad, err);
				bfa_trc(bnad, i);
				bfa_trc(bnad,
					res_info[i].res_u.mem_info.mem_type);
				bfa_trc(bnad, res_info[i].res_u.mem_info.num);
				bfa_trc(bnad, res_info[i].res_u.mem_info.len);
			}
		} else if (res_info[i].res_type == BNA_RES_T_INTR) {
			/*
			 * idx is MSIX index for MSIX case and bit shift
			 * in ISR for INTx case.
			 */
			idx = bnad_diag_lb_tx_intr_get(bnad);
			err = bnad_diag_lb_txrx_irq_alloc(bnad, idx,
					&res_info[i].res_u.intr_info);
		}
		if (err) {
			bfa_trc(bnad, err);
			return BFA_STATUS_ENOMEM;
		}
	}

	/* Setup pkt pool */
	bfa_trc(bnad, bnad->diag->tx_info.mem_info.num);
	bfa_trc(bnad, bnad->diag->tx_info.mem_info.len);

	if (bnad_diag_lb_mem_alloc(bnad, &bnad->diag->tx_info.mem_info))
		return BFA_STATUS_ENOMEM;

	return 0;
}

static void
bnad_diag_lb_rx_res_free(struct bnad_s *bnad, struct bna_res_info_s *res_info)
{
	int i;

	bfa_trc(bnad, 0);

	for (i = 0; i < BNA_RX_RES_T_MAX; i++) {
		if (res_info[i].res_type == BNA_RES_T_MEM)
			bnad_diag_lb_mem_free(bnad,
						&res_info[i].res_u.mem_info);
		else if (res_info[i].res_type == BNA_RES_T_INTR)
			bnad_diag_lb_rx_irq_free(bnad,
						&res_info[i].res_u.intr_info);
	}
	bnad_diag_lb_mem_free(bnad, &bnad->diag->rx_info.mem_info);
}

/* Allocates memory and interrupt resources for Rx object */
static bfa_status_t
bnad_diag_lb_rx_res_alloc(struct bnad_s *bnad, struct bna_res_info_s *res_info)
{
	int i, idx;
	bfa_status_t err = BFA_STATUS_OK;

	bfa_trc(bnad, 0);

	for (i = 0; i < BNA_RX_RES_T_MAX; i++) {

		CNA_ASSERT(res_info[i].res_type == BNA_RES_T_MEM ||
			   res_info[i].res_type == BNA_RES_T_INTR);

		if (res_info[i].res_type == BNA_RES_T_MEM) {
			err = bnad_diag_lb_mem_alloc(bnad,
					&res_info[i].res_u.mem_info);
			if (err) {
				bfa_trc(bnad, err);
				bfa_trc(bnad, i);
				bfa_trc(bnad,
					res_info[i].res_u.mem_info.mem_type);
				bfa_trc(bnad, res_info[i].res_u.mem_info.num);
				bfa_trc(bnad, res_info[i].res_u.mem_info.len);
			}
		} else if (res_info[i].res_type == BNA_RES_T_INTR) {
			/*
			 * idx is MSIX index for MSIX case and bit shift
			 * in ISR for INTx case.
			 */
			idx = bnad_diag_lb_rx_intr_get(bnad);
			err = bnad_diag_lb_txrx_irq_alloc(bnad, idx,
					&res_info[i].res_u.intr_info);
		}
		if (err) {
			bfa_trc(bnad, err);
			return BFA_STATUS_ENOMEM;
		}
	}

	/* Setup unmap array / actual pkt pool */
	bfa_trc(bnad, bnad->diag->rx_info.mem_info.num);
	bfa_trc(bnad, bnad->diag->rx_info.mem_info.len);
	bfa_trc(bnad, bnad->diag->rx_info.mem_info.mem_type);

	if (bnad_diag_lb_mem_alloc(bnad, &bnad->diag->rx_info.mem_info))
		return BFA_STATUS_ENOMEM;

	return err;
}

/* Diag Loopback Setup/Cleanup */
static void
bnad_diag_lb_tx_cleanup(struct bnad_s *bnad)
{
	struct bnad_diag_tx_info_s *tx_info = &bnad->diag->tx_info;
	struct bna_res_info_s *res_info = &bnad->diag->tx_info.res_info[0];
	struct bnad_diag_s *diag = bnad->diag;
	unsigned long flags;

	CNA_ASSERT(diag);

	bfa_trc(bnad, 0);

	if (!tx_info->tx)
		return;

	BNAD_DIAG_INIT_COMPLETION(diag);
	bnad_spin_lock(flags);
	bna_tx_disable(tx_info->tx, BNA_HARD_CLEANUP, bnad_cb_lb_tx_disabled);
	bnad_spin_unlock(flags);
	BNAD_DIAG_WAIT_FOR_COMPLETION(bnad->diag);

	bnad_spin_lock(flags);
	bna_tx_destroy(tx_info->tx);
	bnad_spin_unlock(flags);

	tx_info->tx = NULL;

	bnad_diag_lb_tx_res_free(bnad, res_info);
}

static bfa_status_t
bnad_diag_lb_tx_setup(struct bnad_s *bnad)
{
	struct bnad_diag_tx_info_s *tx_info = &bnad->diag->tx_info;
	struct bna_res_info_s *res_info = &bnad->diag->tx_info.res_info[0];
	struct bna_intr_info_s *intr_info =
			&res_info[BNA_TX_RES_INTR_T_TXCMPL].res_u.intr_info;
	struct bna_tx_config_s *tx_config = &bnad->diag->tx_info.tx_cfg;
	struct bna_tx_s *tx;
	struct bna_tx_event_cbfn_s tx_cbfn;
	unsigned long flags;
	bfa_status_t status;

	/* Setup diag config */
	tx_config->num_txq = 1;
	tx_config->txq_depth = BNAD_DIAG_Q_DEPTH;
	tx_config->tx_type = BNA_TX_T_LOOPBACK;
	tx_config->coalescing_timeo = BFI_TX_COALESCING_TIMEO;

	bfa_trc(bnad, tx_config->num_txq);
	bfa_trc(bnad, tx_config->txq_depth);
	bfa_trc(bnad, tx_config->tx_type);

	/* Setup diag callbacks */
	tx_cbfn.tcb_setup_cbfn = bnad_cb_lb_tcb_setup;
	tx_cbfn.tcb_destroy_cbfn = bnad_cb_lb_tcb_destroy;
	tx_cbfn.tx_stall_cbfn = bnad_cb_lb_tx_stall;
	tx_cbfn.tx_resume_cbfn = bnad_cb_lb_tx_resume;
	tx_cbfn.tx_cleanup_cbfn = bnad_cb_lb_tx_cleanup;

	bnad_spin_lock(flags);
	bna_tx_res_req(tx_config->num_txq, BNAD_DIAG_Q_DEPTH, res_info);
	bnad_spin_unlock(flags);

	/* Fill Unmap Q details for res_info */
	res_info[BNA_TX_RES_MEM_T_UNMAPQ].res_type = BNA_RES_T_MEM;
	res_info[BNA_TX_RES_MEM_T_UNMAPQ].res_u.mem_info.mem_type =
					BNA_MEM_T_KVA;
	res_info[BNA_TX_RES_MEM_T_UNMAPQ].res_u.mem_info.num = 1;
	res_info[BNA_TX_RES_MEM_T_UNMAPQ].res_u.mem_info.len =
		 (sizeof(struct bnad_diag_unmap_q_s) +
		 ((sizeof(struct bnad_diag_pkt_s) *
		 (BNAD_DIAG_UNMAPQ_DEPTH - 1))));

	/* Fill up the mem_info for unmap array */
	tx_info->mem_info.mem_type = BNA_MEM_T_DMA;
	tx_info->mem_info.num = BNAD_DIAG_UNMAPQ_DEPTH;
	tx_info->mem_info.len = BNAD_DIAG_ETH_HLEN + BNAD_DIAG_ETH_MTU;

	bfa_trc(bnad, tx_info->mem_info.mem_type);
	bfa_trc(bnad, tx_info->mem_info.num);
	bfa_trc(bnad, tx_info->mem_info.len);

	if ((status = bnad_diag_lb_tx_res_alloc(bnad, res_info))) {
		bfa_trc(bnad, status);
		return status;
	}

	bnad_spin_lock(flags);
	tx = bna_tx_create(&bnad->bna, bnad, tx_config, &tx_cbfn, res_info,
			    tx_info);
	bnad_spin_unlock(flags);

	if (!tx) {
		bfa_trc(bnad, 0);
		return BFA_STATUS_DEVBUSY;
	}
	bnad->diag->tx_info.tx = tx;

	bfa_trc(bnad, intr_info->intr_type);

	bnad_spin_lock(flags);
	bna_tx_enable(tx);
	bnad_spin_unlock(flags);
	return BFA_STATUS_OK;
}

static void
bnad_diag_lb_rx_cleanup(struct bnad_s *bnad)
{
	struct bnad_diag_rx_info_s *rx_info = &bnad->diag->rx_info;
	struct bna_res_info_s *res_info = &bnad->diag->rx_info.res_info[0];
	unsigned long flags;

	bfa_trc(bnad, 0);

	if (!rx_info->rx)
		return;

	BNAD_DIAG_INIT_COMPLETION(bnad->diag);
	bnad_spin_lock(flags);
	bna_rx_disable(rx_info->rx, BNA_HARD_CLEANUP, bnad_cb_lb_rx_disabled);
	bnad_spin_unlock(flags);
	BNAD_DIAG_WAIT_FOR_COMPLETION(bnad->diag);

	bfa_trc(bnad, bnad->diag->comp.comp_status);

	bnad_spin_lock(flags);
	bna_rx_destroy(rx_info->rx);
	bnad_spin_unlock(flags);

	rx_info->rx = NULL;

	bnad_diag_lb_rx_res_free(bnad, res_info);
}

static bfa_status_t
bnad_diag_lb_rx_setup(struct bnad_s *bnad)
{
	struct bnad_diag_rx_info_s *rx_info = &bnad->diag->rx_info;
	struct bna_res_info_s *res_info = &bnad->diag->rx_info.res_info[0];
	struct bna_rx_config_s *rx_config = &bnad->diag->rx_info.rx_cfg;
	struct bna_rx_event_cbfn_s rx_cbfn;
	struct bna_rx_s *rx;
	unsigned long flags;
	bfa_status_t status;

	/* Initialize the Rx object configuration */
	rx_config->rx_type = BNA_RX_T_LOOPBACK;
	rx_config->num_paths = 1;
	rx_config->rss_status = BNA_STATUS_T_DISABLED;
	rx_config->rxp_type = BNA_RXP_SINGLE;
	rx_config->q0_depth = BNAD_DIAG_Q_DEPTH;
	rx_config->q0_num_vecs = 1;
	rx_config->q0_multi_buf = BNA_STATUS_T_DISABLED;
	rx_config->vlan_strip_status = BNA_STATUS_T_ENABLED;
	rx_config->coalescing_timeo = BFI_RX_COALESCING_TIMEO;

	/* Initialize the Rx event handlers */
	rx_cbfn.rcb_setup_cbfn = bnad_cb_lb_rcb_setup;
	rx_cbfn.rcb_destroy_cbfn = NULL;
	rx_cbfn.ccb_setup_cbfn = bnad_cb_lb_ccb_setup;
	rx_cbfn.ccb_destroy_cbfn = bnad_cb_lb_ccb_destroy;
	rx_cbfn.rx_stall_cbfn = NULL;
	rx_cbfn.rx_cleanup_cbfn = bnad_cb_lb_rx_cleanup;
	rx_cbfn.rx_post_cbfn = bnad_cb_lb_rx_post;

	bfa_trc(bnad, rx_config->rx_type);
	bfa_trc(bnad, rx_config->num_paths);
	bfa_trc(bnad, rx_config->rss_status);
	bfa_trc(bnad, rx_config->rxp_type);
	bfa_trc(bnad, rx_config->q0_depth);
	bfa_trc(bnad, rx_config->vlan_strip_status);

	bnad_spin_lock(flags);
	bna_rx_res_req(rx_config, res_info);
	bnad_spin_unlock(flags);

	/* Fill Unmap Q memory requirements */
	res_info[BNA_RX_RES_MEM_T_UNMAPDQ].res_type = BNA_RES_T_MEM;
	res_info[BNA_RX_RES_MEM_T_UNMAPDQ].res_u.mem_info.mem_type =
								BNA_MEM_T_KVA;
	res_info[BNA_RX_RES_MEM_T_UNMAPDQ].res_u.mem_info.num = 1;
	res_info[BNA_RX_RES_MEM_T_UNMAPDQ].res_u.mem_info.len =
		(sizeof(struct bnad_diag_unmap_q_s) +
		 ((sizeof(struct bnad_diag_pkt_s) *
		  (BNAD_DIAG_UNMAPQ_DEPTH - 1))));

	/* Fill up the mem_info for unmap array */
	rx_info->mem_info.mem_type = BNA_MEM_T_DMA;
	rx_info->mem_info.num = BNAD_DIAG_UNMAPQ_DEPTH;
	rx_info->mem_info.len = BNAD_DIAG_ETH_HLEN + BNAD_DIAG_ETH_MTU
				+ BNAD_DIAG_ETH_FCS_LEN;

	bfa_trc(bnad, rx_info->mem_info.mem_type);
	bfa_trc(bnad, rx_info->mem_info.num);
	bfa_trc(bnad, rx_info->mem_info.len);

	/* Allocate resource */
	if ((status = bnad_diag_lb_rx_res_alloc(bnad, res_info))) {
		bfa_trc(bnad, status);
		return status;
	}

	bnad_spin_lock(flags);
	rx = bna_rx_create(&bnad->bna, bnad, rx_config, &rx_cbfn,
						res_info, rx_info);
	bnad_spin_unlock(flags);
	if (!rx) {
		bfa_trc(bnad, 0);
		return BFA_STATUS_DEVBUSY;
	}
	rx_info->rx = rx;

	BNAD_DIAG_INIT_COMPLETION(bnad->diag);
	bnad_spin_lock(flags);
	bna_rx_ucast_add(rx, (uint8_t *)&bnad->diag->mac_addr,
			 bnad_cb_lb_rx_ucast_set);
	bnad_spin_unlock(flags);

	BNAD_DIAG_WAIT_FOR_COMPLETION(bnad->diag);

	bfa_trc(bnad, bnad->diag->comp.comp_status);

	if (bnad->diag->comp.comp_status)
		return BFA_STATUS_DEVBUSY;

	/* Ask BNA to kickstart the Rx object */
	bnad_spin_lock(flags);
	bna_rx_enable(rx);
	bnad_spin_unlock(flags);

	return BFA_STATUS_OK;
}

static bfa_status_t
bnad_diag_lb_cleanup(struct bnad_s *bnad)
{
	unsigned long flags;

	bfa_trc(bnad, 0);

	BNAD_DIAG_INIT_COMPLETION(bnad->diag);

	bnad_spin_lock(flags);
	bna_enet_disable(&bnad->bna.enet, BNA_HARD_CLEANUP,
			 bnad_cb_lb_enet_disabled);
	bnad_spin_unlock(flags);

	BNAD_DIAG_WAIT_FOR_COMPLETION(bnad->diag);


	bnad_spin_lock(flags);
	bna_enet_type_set(&bnad->bna.enet, BNA_ENET_T_REGULAR);

	bfa_trc(bnad, bnad->bna.enet.type);

	bna_ethport_linkcbfn_set(&bnad->bna.ethport,
			      bnad_cb_ethport_link_status);
	bnad_spin_unlock(flags);

	bnad_diag_lb_tx_cleanup(bnad);
	bnad_diag_lb_rx_cleanup(bnad);

	bfa_trc(bnad, bnad->diag->prior_enet_flags);
	bfa_trc(bnad, bnad->diag->prior_ethport_flags);
	bfa_trc(bnad, bnad->diag->mtu);
	bfa_trc(bnad, bnad->diag->p_cfg.tx_pause);
	bfa_trc(bnad, bnad->diag->p_cfg.rx_pause);

	bnad_spin_lock(flags);

	/* Restore PAUSE & MTU, previous ethport state */
	if (bnad->diag->prior_ethport_flags & BNA_ETHPORT_F_ADMIN_UP)
		bna_ethport_admin_up(&bnad->bna.ethport, NULL);

	if (bnad->diag->prior_enet_flags & BNA_ENET_F_ENABLED) {
		bna_enet_mtu_set(&bnad->bna.enet, bnad->diag->mtu, NULL);
		bna_enet_pause_config(&bnad->bna.enet,
					&bnad->diag->p_cfg, NULL);
		bna_enet_enable(&bnad->bna.enet);
	}

	bnad_spin_unlock(flags);

	return BFA_STATUS_OK;
}


static bfa_status_t
bnad_diag_lb_setup(struct bnad_s *bnad, enum bna_enet_type_e type)
{
	int mtu;
	struct bna_pause_config_s pause_config;
	unsigned long flags;
	bfa_status_t status;

	BNAD_DIAG_INIT_COMPLETION(bnad->diag);

	bnad_spin_lock(flags);
	bna_ethport_admin_down(&bnad->bna.ethport);

	bna_enet_disable(&bnad->bna.enet, BNA_HARD_CLEANUP,
			 bnad_cb_lb_enet_disabled);
	bnad_spin_unlock(flags);

	BNAD_DIAG_WAIT_FOR_COMPLETION(bnad->diag);
	bfa_trc(bnad, bnad->diag->comp.comp_status);

	if (bnad->diag->comp.comp_status != BNA_CB_SUCCESS)
		return BFA_STATUS_DEVBUSY;

	bfa_trc(bnad, type);

	bnad_spin_lock(flags);
	bna_enet_type_set(&bnad->bna.enet, type);

	bna_ethport_linkcbfn_set(&bnad->bna.ethport,
			      bnad_cb_lb_ethport_link_status);
	bnad_spin_unlock(flags);

	if ((status = bnad_diag_lb_tx_setup(bnad))) {
		bfa_trc(bnad, status);
		return status;
	}

	if ((status = bnad_diag_lb_rx_setup(bnad))) {
		bfa_trc(bnad, status);
		return status;
	}

	mtu = BNAD_DIAG_ETH_HLEN + BNAD_DIAG_ETH_MTU + BNAD_DIAG_ETH_FCS_LEN;
	pause_config.tx_pause = 1;
	pause_config.rx_pause = 1;

	bfa_trc(bnad, mtu);
	bfa_trc(bnad, pause_config.tx_pause);
	bfa_trc(bnad, pause_config.rx_pause);

	bnad_spin_lock(flags);

	bna_enet_mtu_set(&bnad->bna.enet, mtu, NULL);
	bna_enet_pause_config(&bnad->bna.enet, &pause_config, NULL);

	bna_enet_enable(&bnad->bna.enet);

	bnad_spin_unlock(flags);

	BNAD_DIAG_INIT_COMPLETION(bnad->diag);
	BNAD_DIAG_LINK_INIT_COMPLETION(bnad->diag);
	bnad_spin_lock(flags);
	bna_ethport_admin_up(&bnad->bna.ethport, bnad_cb_lb_ethport_admin_up);
	bnad_spin_unlock(flags);
	BNAD_DIAG_WAIT_FOR_COMPLETION(bnad->diag);
	if (bnad->diag->comp.comp_status != BNA_CB_SUCCESS)
		return BFA_STATUS_PORT_NOT_DISABLED;
	BNAD_DIAG_LINK_WAIT_FOR_COMPLETION(bnad->diag);

	bfa_trc(bnad, bnad->diag->link_comp.comp_status);
	bfa_trc(bnad, bnad->diag->link_status);

	if (bnad->diag->link_comp.comp_status ||
			(bnad->diag->link_status == BNA_LINK_DOWN))
		return BFA_STATUS_LINKTIMEOUT;

	return BFA_STATUS_OK;
}

static void
bnad_diag_lb_pkt_fill(struct bnad_s *bnad, struct bnad_diag_pkt_s *pkt,
			uint32_t pattern)
{
	struct bnad_diag_eth_hdr_s *ehdr = NULL;
	struct bnad_diag_pkt_pload_s *pload = NULL;
	uint32_t i, *data = NULL;
	uint32_t len;

	CNA_ASSERT(pkt);

	pload = (struct bnad_diag_pkt_pload_s *)(pkt->data);
	ehdr = &pload->eth_hdr;
	memcpy(&ehdr->src, &bnad->diag->mac_addr, BNAD_DIAG_ETH_ALEN);
	memcpy(&ehdr->dst, &ehdr->src, BNAD_DIAG_ETH_ALEN);

	ehdr->proto = cna_os_htons(BNAD_DIAG_ETH_P_LOOPBACK);

	/* The first snd_seq is 1 */
	bnad->diag->snd_seq++;
	pload->seq_num = cna_os_htonl(bnad->diag->snd_seq);

	data = (uint32_t *)((uint8_t *)pkt->data + sizeof(*pload));
	len = pkt->hdr.len - sizeof(*pload);

	if (len > 0)
		len -= 4; /* For FCS length */
	else
		len = 0;

	pkt->hdr.free = 0;

	for (i = 0; i < (len >> 2); i++)
		data[i] = pattern;
}

static bfa_status_t
bnad_diag_lb_snd_one(struct bnad_s *bnad, struct bnad_diag_pkt_s *send_pkt,
		     uint32_t pattern)
{
	uint16_t 		txq_prod;
	uint32_t 		unmap_prod, wis, wis_used, wi_range;
	uint32_t 		vectors, vect_id, acked, r_flags;
	unsigned long		flags;

	struct bnad_diag_tx_info_s 	*tx_info;
	struct bna_tcb_s		*tcb;
	struct bnad_diag_unmap_q_s 	*unmap_q;
	struct bna_txq_entry_s 		*txqent;

	struct bnad_diag_pkt_s		*pkt = send_pkt;

	tx_info = &bnad->diag->tx_info;
	tcb = tx_info->tcb;
	unmap_q = tcb->unmap_q;

	CNA_ASSERT(pkt);

	vectors = 1;
	wis = BNA_TXQ_WI_NEEDED(vectors);	/* 4 vectors per work item */
	acked = 0;

	bnad_spin_lock(flags);

	r_flags = bnad->diag->run_flags;
	bnad_spin_unlock(flags);

	if (r_flags & BNAD_DIAG_RF_TX_STALLED) {
		bfa_trc(bnad, r_flags);
		return BFA_STATUS_DEVBUSY;
	}

	unmap_prod = unmap_q->producer_index;
	pkt = &unmap_q->unmap_array[unmap_prod];

	/* If still there is nothing */
	if (pkt->hdr.free != 1) {
		bfa_trc(bnad, r_flags);
		bfa_trc(bnad, pkt->hdr.free);
		return BFA_STATUS_DEVBUSY;
	}

	CNA_ASSERT(pkt->hdr.len >= BNAD_DIAG_ETH_HLEN &&
		   pkt->hdr.len < BFI_TX_MAX_DATA_PER_VECTOR);

	bnad_diag_lb_pkt_fill(bnad, pkt, pattern);

	wis_used = 1;
	vect_id = 0;

	txq_prod = tcb->producer_index;
	BNA_TXQ_QPGE_PTR_GET(txq_prod, tcb->sw_qpt, txqent, wi_range);
	CNA_ASSERT(wi_range && wi_range <= tcb->q_depth);

	txqent->hdr.wi.reserved = 0;
	txqent->hdr.wi.num_vectors = vectors;
	txqent->hdr.wi.opcode = cna_os_htons(BNA_TXQ_WI_SEND);
	txqent->hdr.wi.vlan_tag = 0;
	txqent->hdr.wi.lso_mss = 0;
	txqent->hdr.wi.l4_hdr_size_n_offset = 0;
	txqent->hdr.wi.flags = 0;
	txqent->hdr.wi.frame_length = cna_os_htonl(pkt->hdr.len);
	txqent->vector[vect_id].length = cna_os_htons(pkt->hdr.len);

	txqent->vector[vect_id].host_addr.msb = pkt->hdr.dma.msb;
	txqent->vector[vect_id].host_addr.lsb = pkt->hdr.dma.lsb;

	BNA_QE_INDX_ADD(unmap_prod, 1, unmap_q->q_depth);

	unmap_q->producer_index = unmap_prod;
	BNA_QE_INDX_ADD(txq_prod, wis_used, tcb->q_depth);
	tcb->producer_index = txq_prod;

	CNA_ASSERT(unmap_q->producer_index == tcb->producer_index);

	bna_txq_prod_indx_doorbell(tcb);

	return BFA_STATUS_OK;
}

static void
bnad_diag_lb_post_rxbufs(struct bnad_s *bnad, struct bna_rcb_s *rcb)
{
	uint16_t to_alloc, alloced, unmap_prod, wi_range;
	struct bna_rxq_entry_s *rxent;
	struct bnad_diag_pkt_s *unmap_array, *pkt;
	struct bnad_diag_unmap_q_s *unmap_q = rcb->unmap_q;

	alloced = 0;
	to_alloc =
		BNA_QE_FREE_CNT(unmap_q, unmap_q->q_depth);

	unmap_array = unmap_q->unmap_array;
	unmap_prod = unmap_q->producer_index;

	BNA_RXQ_QPGE_PTR_GET(unmap_prod, rcb->sw_qpt, rxent, wi_range);
	CNA_ASSERT(wi_range && wi_range <= rcb->q_depth);

	while (to_alloc--) {
		if (!wi_range) {
			BNA_RXQ_QPGE_PTR_GET(unmap_prod, rcb->sw_qpt, rxent,
					     wi_range);
			CNA_ASSERT(wi_range && wi_range <= rcb->q_depth);
		}
		pkt = &unmap_array[unmap_prod];
		if (!pkt->hdr.free)
			goto done;
		else
			pkt->hdr.free = 0;

		rxent->host_addr.msb = pkt->hdr.dma.msb;
		rxent->host_addr.lsb = pkt->hdr.dma.lsb;

		BNA_QE_INDX_ADD(unmap_prod, 1, unmap_q->q_depth);

		rxent++;
		wi_range--;
		alloced++;
	}

done:
	if (alloced) {
		unmap_q->producer_index = unmap_prod;
		rcb->producer_index = unmap_prod;
		bna_rxq_prod_indx_doorbell(rcb);
	}
}

static void
bnad_diag_lb_refill_rxq(struct bnad_s *bnad, struct bna_rcb_s *rcb)
{
	struct bnad_diag_unmap_q_s *unmap_q = rcb->unmap_q;
	unsigned long flags;

	CNA_ASSERT(unmap_q);

	if (BNA_QE_FREE_CNT(unmap_q, unmap_q->q_depth) >> 3) {
		bnad_spin_lock(flags);
		bnad_diag_lb_post_rxbufs(bnad, rcb);
		bnad_spin_unlock(flags);
	}
}

static int
bnad_diag_lb_rx_pkt_check(struct bnad_s *bnad, struct bnad_diag_pkt_s *pkt)
{
	uint32_t 			exp_rcv_seq;
	struct bnad_diag_pkt_pload_s 	*pload;
	struct bnad_diag_eth_hdr_s 	*ehdr;
	unsigned long			flags;

	pload = (struct bnad_diag_pkt_pload_s *)(pkt->data);
	ehdr = &pload->eth_hdr;

	exp_rcv_seq = bnad->diag->rcv_seq + 1;

	if ((!cna_os_memcmp(&ehdr->dst, &ehdr->src, BNAD_DIAG_ETH_ALEN)) &&
		(!cna_os_memcmp(&ehdr->dst,
			&bnad->diag->mac_addr, BNAD_DIAG_ETH_ALEN)) &&
		(cna_os_ntohs(ehdr->proto) ==
					BNAD_DIAG_ETH_P_LOOPBACK) &&
		(exp_rcv_seq == cna_os_ntohl(pload->seq_num))) {
			bnad->diag->rcv_seq++;
			bnad_spin_lock(flags);
			pkt->hdr.free = 1;
			bnad_spin_unlock(flags);
			return 0;
	} else
		bnad->diag->rx_drop_cnt++;

	return (-1);
}

static int
bnad_diag_lb_rcv(struct bnad_s *bnad, uint32_t budget)
{
	struct bna_cq_entry_s *cmpl, *next_cmpl;
	struct bna_rcb_s *rcb = NULL;
	unsigned int wi_range, packets = 0, wis = 0;
	struct bnad_diag_unmap_q_s *unmap_q;
	struct bnad_diag_pkt_s *pkt;
	struct bna_ccb_s *ccb = bnad->diag->rx_info.ccb;
	uint32_t fl;
	unsigned long flags;

	bnad_spin_lock(flags);
	fl = bnad->diag->run_flags;
	bnad_spin_unlock(flags);

	if (fl & BNAD_DIAG_RF_RX_CLEANED)
		return packets;

	BNA_CQ_QPGE_PTR_GET(ccb->producer_index, ccb->sw_qpt, cmpl,
			    wi_range);
	CNA_ASSERT(cmpl);
	CNA_ASSERT(wi_range && wi_range <= ccb->q_depth);

	rcb = ccb->rcb[0];
	while (cmpl->valid && packets < budget) {

		unmap_q = rcb->unmap_q;

		pkt = &unmap_q->unmap_array[unmap_q->consumer_index];

		BNA_QE_INDX_ADD(unmap_q->consumer_index, 1, unmap_q->q_depth);
		BNA_QE_INDX_ADD(rcb->consumer_index, 1, rcb->q_depth);

		wis++;
		if (--wi_range)
			next_cmpl = cmpl + 1;
		else {
			BNA_QE_INDX_ADD(ccb->producer_index, wis, ccb->q_depth);
			wis = 0;
			BNA_CQ_QPGE_PTR_GET(ccb->producer_index, ccb->sw_qpt,
						next_cmpl, wi_range);
			CNA_ASSERT(next_cmpl);
			CNA_ASSERT(wi_range && wi_range <= ccb->q_depth);
		}

		fl = cna_os_ntohl(cmpl->flags);
		if ((fl &
		     (BNA_CQ_EF_MAC_ERROR | BNA_CQ_EF_FCS_ERROR |
		      BNA_CQ_EF_TOO_LONG))) {
			goto next;
		}
		if (!bnad_diag_lb_rx_pkt_check(bnad, pkt))
			packets++;
next:
		cmpl->valid = 0;
		cmpl = next_cmpl;
	}

	BNA_QE_INDX_ADD(ccb->producer_index, wis, ccb->q_depth);

	if (ccb) {
		bna_ib_ack(ccb->i_dbell, packets);
		bnad_diag_lb_refill_rxq(bnad, ccb->rcb[0]);
	}
	return packets;
}

/*
 * ASIC workaround in RxA PE code(7/18/2011) increased the threshold
 * for the RxQ to 144.
 */
#define RXQ_THRESHOLD		145
static bfa_status_t
bnad_diag_lb_snd_rcv(struct bnad_s *bnad, uint32_t loopcnt,
		     uint32_t pattern)
{
	int i;
	bfa_status_t status = BFA_STATUS_OK;
	uint32_t send_burst, retries, sent, rcvd, rx_use_cnt;
	uint32_t pkts_to_send, tot_sent, tot_rcvd;
	unsigned long flags;
	struct bnad_diag_pkt_s *pkt;
	struct bnad_diag_unmap_q_s *tx_umap_q =
				bnad->diag->tx_info.tcb->unmap_q;
	struct bnad_diag_unmap_q_s *rx_umap_q =
				bnad->diag->rx_info.ccb->rcb[0]->unmap_q;

	bnad->diag->snd_seq = 0;
	bnad->diag->rcv_seq = 0;

	sent = rcvd = 0;
	tot_sent = tot_rcvd = 0;

	pkts_to_send = loopcnt;

	while (pkts_to_send) {
		/* Break in case of H/W error */
		if ((bnad->diag->run_flags & BNAD_DIAG_RF_TX_STALLED) ||
			(bnad->diag->run_flags & BNAD_DIAG_RF_RX_CLEANED)) {
			bfa_trc(bnad, bnad->diag->run_flags);
			status = BFA_STATUS_IOC_FAILURE;
			goto done;
		}
		sent = 0;
		rcvd = 0;
		retries = 0;
		status = BFA_STATUS_OK;

		rx_use_cnt = BNA_QE_IN_USE_CNT(rx_umap_q, rx_umap_q->q_depth);

		send_burst =
			BNA_MIN(pkts_to_send, (rx_use_cnt - RXQ_THRESHOLD));

		bfa_trc(bnad, send_burst);
		bfa_trc(bnad, bnad->diag->tx_info.tcb->consumer_index);

		/* Send Loop */
		for (i = 0; i < send_burst; i++) {
			pkt = &tx_umap_q->unmap_array[i];

			CNA_ASSERT(pkt->hdr.len);

			status = bnad_diag_lb_snd_one(bnad, pkt, pattern);
			if (!status) {
				sent++;
				tot_sent++;
			} else
				bfa_trc(bnad, status);
		}

		/* Recv Loop */
		while (1) {
			/* Q depth is the budget */
			rcvd += bnad_diag_lb_rcv(bnad, BNAD_DIAG_Q_DEPTH);
			if ((rcvd >= sent) || (retries++ >=
						BNAD_DIAG_RX_MAX_RETRIES))
				break;
			bnad_udelay(60);
		}

		tot_rcvd += rcvd;

		bfa_trc(bnad, sent);
		bfa_trc(bnad, rcvd);

		if ((rcvd < sent) || (status != 0)) {
			bfa_trc(bnad, sent);
			bfa_trc(bnad, rcvd);
			if (rcvd < sent)
				status = BFA_STATUS_MISSINGFRM;
			goto done;
		}

		/* Force free as many Tx buffers as received */
		bnad_spin_lock(flags);
		bnad_diag_lb_txbufs_force_free(bnad,
						bnad->diag->tx_info.tcb, rcvd);
		bnad_spin_unlock(flags);

		pkts_to_send -= sent;
	}

done:
	bfa_trc(bnad, loopcnt);
	bfa_trc(bnad, tot_sent);
	bfa_trc(bnad, tot_rcvd);
	bfa_trc(bnad,
		cna_mem_readw(bnad->diag->tx_info.tcb->hw_consumer_index));
	return status;
}

static bfa_status_t
bnad_diag_lb_run(struct bnad_s *bnad, bfa_ioctl_diag_ll_loopback_t *iocmd)
{
	struct bnad_diag_s *diag;
	enum bna_enet_type_e type = BNA_ENET_T_LOOPBACK_INTERNAL;
	bfa_status_t status = BFA_STATUS_OK;
	unsigned long flags;

	CNA_ASSERT(bnad);
	CNA_ASSERT(iocmd);

	iocmd->status = BFA_STATUS_OK;
	bnad->diag = diag = cna_os_kzalloc(sizeof(struct bnad_diag_s));

	if (!diag) {
		iocmd->status = BFA_STATUS_ENOMEM;
		bfa_trc(bnad, sizeof(struct bnad_diag_s));
		return BFA_STATUS_ENOMEM;
	}

	bfa_trc(bnad, iocmd->opmode);
	bfa_trc(bnad, iocmd->lpcnt);
	bfa_trc(bnad, iocmd->pat);

	bnad_spin_lock(flags);
	bna_enet_perm_mac_get(&bnad->bna.enet, &bnad->diag->mac_addr);
	bnad_spin_unlock(flags);

	/* Store previous configuration */

	diag->p_cfg.rx_pause = bnad->bna.enet.pause_config.rx_pause;
	diag->p_cfg.tx_pause = bnad->bna.enet.pause_config.tx_pause;
	diag->mtu = bnad->bna.enet.mtu;
	diag->prior_enet_flags = bnad->bna.enet.flags;
	diag->prior_ethport_flags = bnad->bna.ethport.flags;

	bfa_trc(bnad, diag->p_cfg.tx_pause);
	bfa_trc(bnad, diag->p_cfg.rx_pause);
	bfa_trc(bnad, diag->mtu);
	bfa_trc(bnad, diag->prior_enet_flags);
	bfa_trc(bnad, diag->prior_ethport_flags);

	if (iocmd->opmode == BFA_PORT_OPMODE_LB_EXT)
		type = BNA_ENET_T_LOOPBACK_INTERNAL;
	else if (iocmd->opmode == BFA_PORT_OPMODE_LB_CBL)
		type = BNA_ENET_T_LOOPBACK_EXTERNAL;
	else {
		iocmd->status = BFA_STATUS_EINVAL;
		bfa_trc(bnad, iocmd->opmode);
		status = BFA_STATUS_EINVAL;
		goto done;
	}

	bfa_trc(bnad, type);

	if ((status = bnad_diag_lb_setup(bnad, type))) {
		iocmd->status = status;
		bfa_trc(bnad, status);
		bnad_diag_lb_cleanup(bnad);
		goto done;
	}

	if ((status = bnad_diag_lb_snd_rcv(bnad, iocmd->lpcnt, iocmd->pat))) {
		iocmd->result.status = status;
		bfa_trc(bnad, status);
	}

	bnad_diag_lb_cleanup(bnad);
done:
	cna_os_kfree(bnad->diag, sizeof(struct bnad_diag_s));
	bnad->diag = NULL;
	return status;
}

int
bnad_ioctl_diag_loopback(struct bnad_s *bnad, void *arg)
{
	bfa_ioctl_diag_ll_loopback_t *iocmd =
		(bfa_ioctl_diag_ll_loopback_t *)(arg);
	bfa_status_t status;

	cna_os_memset(&iocmd->result, 0, sizeof(iocmd->result));

	bfa_trc(bnad, iocmd->lpcnt);
	bfa_trc(bnad, iocmd->pat);

	status = bnad_diag_lb_run(bnad, iocmd);
	bfa_trc(bnad, status);

	return 0;
}
