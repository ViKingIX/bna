/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

/**
 * @file bfa_msgq.c MSGQ module source file.
 */

#include <cs/bfa_log.h>
#include <bfi/bfi.h>
#include <cna/bfa_msgq.h>
#include <cna/bfa_ioc.h>

BFA_TRC_FILE(CNA, MSGQ);

#define call_cmdq_ent_cbfn(_cmdq_ent, _status)				\
{									\
	bfa_msgq_cmdcbfn_t cbfn;					\
	void *cbarg;							\
	cbfn = (_cmdq_ent)->cbfn;					\
	cbarg = (_cmdq_ent)->cbarg;					\
	(_cmdq_ent)->cbfn = NULL;					\
	(_cmdq_ent)->cbarg = NULL;					\
	if (cbfn) {							\
		cbfn(cbarg, (_status));					\
	}								\
}

static void bfa_msgq_cmdq_dbell(struct bfa_msgq_cmdq_s *cmdq);
static void bfa_msgq_cmdq_copy_rsp(struct bfa_msgq_cmdq_s *cmdq);

enum cmdq_event_e {
	CMDQ_E_START			= 1,
	CMDQ_E_STOP			= 2,
	CMDQ_E_FAIL			= 3,
	CMDQ_E_POST			= 4,
	CMDQ_E_INIT_RESP		= 5,
	CMDQ_E_DB_READY			= 6,
};

bfa_fsm_state_decl(cmdq, stopped, struct bfa_msgq_cmdq_s, enum cmdq_event_e);
bfa_fsm_state_decl(cmdq, init_wait, struct bfa_msgq_cmdq_s, enum cmdq_event_e);
bfa_fsm_state_decl(cmdq, ready, struct bfa_msgq_cmdq_s, enum cmdq_event_e);
bfa_fsm_state_decl(cmdq, dbell_wait, struct bfa_msgq_cmdq_s,
			enum cmdq_event_e);

static void
cmdq_sm_stopped_entry(struct bfa_msgq_cmdq_s *cmdq)
{
	struct bfa_msgq_cmd_entry_s *cmdq_ent;

	bfa_trc(cmdq->msgq, 0);
	cmdq->producer_index = 0;
	cmdq->consumer_index = 0;
	cmdq->flags = 0;
	cmdq->token = 0;
	cmdq->offset = 0;
	cmdq->bytes_to_copy = 0;
	while (!bfa_q_is_empty(&cmdq->pending_q)) {
		bfa_q_deq(&cmdq->pending_q, &cmdq_ent);
		bfa_q_qe_init(&cmdq_ent->qe);
		call_cmdq_ent_cbfn(cmdq_ent, BFA_STATUS_FAILED);
	}
}

static void
cmdq_sm_stopped(struct bfa_msgq_cmdq_s *cmdq, enum cmdq_event_e event)
{
	bfa_trc(cmdq->msgq, event);

	switch (event) {
	case CMDQ_E_START:
		bfa_fsm_set_state(cmdq, cmdq_sm_init_wait);
		break;

	case CMDQ_E_STOP:
	case CMDQ_E_FAIL:
		/* No-op */
		break;

	case CMDQ_E_POST:
		cmdq->flags |= BFA_MSGQ_CMDQ_F_DB_UPDATE;
		break;

	default:
		bfa_sm_fault(cmdq->msgq, event);
	}
}

static void
cmdq_sm_init_wait_entry(struct bfa_msgq_cmdq_s *cmdq)
{
	bfa_trc(cmdq->msgq, 0);
	bfa_wc_down(&cmdq->msgq->init_wc);
}

static void
cmdq_sm_init_wait(struct bfa_msgq_cmdq_s *cmdq, enum cmdq_event_e event)
{
	bfa_trc(cmdq->msgq, event);

	switch (event) {
	case CMDQ_E_STOP:
	case CMDQ_E_FAIL:
		bfa_fsm_set_state(cmdq, cmdq_sm_stopped);
		break;

	case CMDQ_E_POST:
		cmdq->flags |= BFA_MSGQ_CMDQ_F_DB_UPDATE;
		break;

	case CMDQ_E_INIT_RESP:
		if (cmdq->flags & BFA_MSGQ_CMDQ_F_DB_UPDATE) {
			cmdq->flags &= ~BFA_MSGQ_CMDQ_F_DB_UPDATE;
			bfa_fsm_set_state(cmdq, cmdq_sm_dbell_wait);
		} else
			bfa_fsm_set_state(cmdq, cmdq_sm_ready);
		break;

	default:
		bfa_sm_fault(cmdq->msgq, event);
	}
}

static void
cmdq_sm_ready_entry(struct bfa_msgq_cmdq_s *cmdq)
{
	bfa_trc(cmdq->msgq, 0);
}

static void
cmdq_sm_ready(struct bfa_msgq_cmdq_s *cmdq, enum cmdq_event_e event)
{
	bfa_trc(cmdq->msgq, event);

	switch (event) {
	case CMDQ_E_STOP:
	case CMDQ_E_FAIL:
		bfa_fsm_set_state(cmdq, cmdq_sm_stopped);
		break;

	case CMDQ_E_POST:
		bfa_fsm_set_state(cmdq, cmdq_sm_dbell_wait);
		break;

	default:
		bfa_sm_fault(cmdq->msgq, event);
	}
}

static void
cmdq_sm_dbell_wait_entry(struct bfa_msgq_cmdq_s *cmdq)
{
	bfa_trc(cmdq->msgq, 0);
	bfa_assert(!bfa_ioc_is_disabled(cmdq->msgq->ioc));
	bfa_msgq_cmdq_dbell(cmdq);
}

static void
cmdq_sm_dbell_wait(struct bfa_msgq_cmdq_s *cmdq, enum cmdq_event_e event)
{
	bfa_trc(cmdq->msgq, event);

	switch (event) {
	case CMDQ_E_STOP:
	case CMDQ_E_FAIL:
		bfa_fsm_set_state(cmdq, cmdq_sm_stopped);
		break;

	case CMDQ_E_POST:
		cmdq->flags |= BFA_MSGQ_CMDQ_F_DB_UPDATE;
		break;

	case CMDQ_E_DB_READY:
		if (cmdq->flags & BFA_MSGQ_CMDQ_F_DB_UPDATE) {
			cmdq->flags &= ~BFA_MSGQ_CMDQ_F_DB_UPDATE;
			bfa_fsm_set_state(cmdq, cmdq_sm_dbell_wait);
		} else
			bfa_fsm_set_state(cmdq, cmdq_sm_ready);
		break;

	default:
		bfa_sm_fault(cmdq->msgq, event);
	}
}

static void
bfa_msgq_cmdq_dbell_ready(void *arg)
{
	struct bfa_msgq_cmdq_s *cmdq = (struct bfa_msgq_cmdq_s *)arg;
	bfa_fsm_send_event(cmdq, CMDQ_E_DB_READY);
}

static void
bfa_msgq_cmdq_dbell(struct bfa_msgq_cmdq_s *cmdq)
{
	struct bfi_msgq_h2i_db_s *dbell =
		(struct bfi_msgq_h2i_db_s *)(&cmdq->dbell_mb.msg[0]);

	bfa_trc(cmdq->msgq, cmdq->consumer_index);
	bfa_trc(cmdq->msgq, cmdq->producer_index);

	bfa_os_memset(dbell, 0, sizeof(struct bfi_msgq_h2i_db_s));
	bfi_h2i_set(dbell->mh, BFI_MC_MSGQ, BFI_MSGQ_H2I_DOORBELL_PI, 0);
	dbell->mh.mtag.i2htok = 0;
	dbell->idx.cmdq_pi = bfa_os_htons(cmdq->producer_index);

	if (!bfa_ioc_mbox_queue(cmdq->msgq->ioc, &cmdq->dbell_mb,
				bfa_msgq_cmdq_dbell_ready, cmdq)) {
		bfa_msgq_cmdq_dbell_ready(cmdq);
	}
}

static void
__cmd_copy(struct bfa_msgq_cmdq_s *cmdq, struct bfa_msgq_cmd_entry_s *cmd)
{
	size_t len = cmd->msg_size;
	int num_entries = 0;
	size_t to_copy;
	uint8_t *src, *dst;

	src = (uint8_t *)cmd->msg_hdr;
	dst = (uint8_t *)cmdq->addr.kva;
	dst += (cmdq->producer_index * BFI_MSGQ_CMD_ENTRY_SIZE);

	while (len) {
		to_copy = (len < BFI_MSGQ_CMD_ENTRY_SIZE) ?
				len : BFI_MSGQ_CMD_ENTRY_SIZE;
		bfa_os_memcpy(dst, src, to_copy);
		bfa_assert(len >= to_copy);
		len -= to_copy;
		src += BFI_MSGQ_CMD_ENTRY_SIZE;
		BFA_MSGQ_INDX_ADD(cmdq->producer_index, 1, cmdq->depth);
		dst = (uint8_t *)cmdq->addr.kva;
		dst += (cmdq->producer_index * BFI_MSGQ_CMD_ENTRY_SIZE);
		num_entries++;
	}

	bfa_assert(num_entries == bfa_os_htons(cmd->msg_hdr->num_entries));
	bfa_trc(cmdq->msgq, num_entries);
}

static void
bfa_msgq_cmdq_ci_update(struct bfa_msgq_cmdq_s *cmdq, struct bfi_mbmsg_s *mb)
{
	struct bfi_msgq_i2h_db_s *dbell = (struct bfi_msgq_i2h_db_s *)mb;
	struct bfa_msgq_cmd_entry_s *cmd;
	int posted = 0;

	bfa_assert((cmdq->fsm == (bfa_fsm_t)cmdq_sm_ready) ||
		(cmdq->fsm == (bfa_fsm_t)cmdq_sm_dbell_wait));

	cmdq->consumer_index = bfa_os_ntohs(dbell->idx.cmdq_ci);
	bfa_trc(cmdq->msgq, cmdq->consumer_index);
	bfa_trc(cmdq->msgq, cmdq->producer_index);

	/* Walk through pending list to see if the command can be posted */
	while (!bfa_q_is_empty(&cmdq->pending_q)) {
		cmd =
		(struct bfa_msgq_cmd_entry_s *)bfa_q_first(&cmdq->pending_q);
		if (bfa_os_ntohs(cmd->msg_hdr->num_entries) <=
			BFA_MSGQ_FREE_CNT(cmdq)) {
			bfa_q_qe_deq(&cmd->qe);
			__cmd_copy(cmdq, cmd);
			posted = 1;
			call_cmdq_ent_cbfn(cmd, BFA_STATUS_OK);
		} else {
			break;
		}
	}

	if (posted)
		bfa_fsm_send_event(cmdq, CMDQ_E_POST);
}

static void
bfa_msgq_cmdq_copy_next(void *arg)
{
	struct bfa_msgq_cmdq_s *cmdq = (struct bfa_msgq_cmdq_s *)arg;

	if (cmdq->bytes_to_copy)
		bfa_msgq_cmdq_copy_rsp(cmdq);
}

static void
bfa_msgq_cmdq_copy_req(struct bfa_msgq_cmdq_s *cmdq, struct bfi_mbmsg_s *mb)
{
	struct bfi_msgq_i2h_cmdq_copy_req_s *req =
		(struct bfi_msgq_i2h_cmdq_copy_req_s *)mb;

	bfa_assert(cmdq->bytes_to_copy == 0);
	bfa_assert(req->len);

	cmdq->token = 0;
	cmdq->offset = bfa_os_ntohs(req->offset);
	cmdq->bytes_to_copy = bfa_os_ntohs(req->len);
	bfa_msgq_cmdq_copy_rsp(cmdq);
}

static void
bfa_msgq_cmdq_copy_rsp(struct bfa_msgq_cmdq_s *cmdq)
{
	struct bfi_msgq_h2i_cmdq_copy_rsp_s *rsp =
		(struct bfi_msgq_h2i_cmdq_copy_rsp_s *)&cmdq->copy_mb.msg[0];
	int copied;
	uint8_t *addr = (uint8_t *)cmdq->addr.kva;

	bfa_os_memset(rsp, 0, sizeof(struct bfi_msgq_h2i_cmdq_copy_rsp_s));
	bfi_h2i_set(rsp->mh, BFI_MC_MSGQ, BFI_MSGQ_H2I_CMDQ_COPY_RSP, 0);
	rsp->mh.mtag.i2htok = bfa_os_htons(cmdq->token);
	copied = (cmdq->bytes_to_copy >= BFI_CMD_COPY_SZ) ? BFI_CMD_COPY_SZ :
		cmdq->bytes_to_copy;
	addr += cmdq->offset;
	memcpy(rsp->data, addr, copied);

	cmdq->token++;
	cmdq->offset += copied;
	bfa_assert(cmdq->bytes_to_copy >= copied);
	cmdq->bytes_to_copy -= copied;

	if (!bfa_ioc_mbox_queue(cmdq->msgq->ioc, &cmdq->copy_mb,
				bfa_msgq_cmdq_copy_next, cmdq)) {
		bfa_msgq_cmdq_copy_next(cmdq);
	}
}

static void
bfa_msgq_cmdq_attach(struct bfa_msgq_cmdq_s *cmdq, struct bfa_msgq_s *msgq)
{
	cmdq->depth = BFA_MSGQ_CMDQ_NUM_ENTRY;
	bfa_q_init(&cmdq->pending_q);
	cmdq->msgq = msgq;
	bfa_fsm_set_state(cmdq, cmdq_sm_stopped);
}

static void bfa_msgq_rspq_dbell(struct bfa_msgq_rspq_s *rspq);

enum rspq_event_e {
	RSPQ_E_START			= 1,
	RSPQ_E_STOP			= 2,
	RSPQ_E_FAIL			= 3,
	RSPQ_E_RESP			= 4,
	RSPQ_E_INIT_RESP		= 5,
	RSPQ_E_DB_READY			= 6,
};

bfa_fsm_state_decl(rspq, stopped, struct bfa_msgq_rspq_s, enum rspq_event_e);
bfa_fsm_state_decl(rspq, init_wait, struct bfa_msgq_rspq_s,
			enum rspq_event_e);
bfa_fsm_state_decl(rspq, ready, struct bfa_msgq_rspq_s, enum rspq_event_e);
bfa_fsm_state_decl(rspq, dbell_wait, struct bfa_msgq_rspq_s,
			enum rspq_event_e);

static void
rspq_sm_stopped_entry(struct bfa_msgq_rspq_s *rspq)
{
	bfa_trc(rspq->msgq, 0);
	bfa_assert(rspq->producer_index == rspq->consumer_index);
	rspq->producer_index = 0;
	rspq->consumer_index = 0;
	rspq->flags = 0;
}

static void
rspq_sm_stopped(struct bfa_msgq_rspq_s *rspq, enum rspq_event_e event)
{
	bfa_trc(rspq->msgq, event);

	switch (event) {
	case RSPQ_E_START:
		bfa_fsm_set_state(rspq, rspq_sm_init_wait);
		break;

	case RSPQ_E_STOP:
	case RSPQ_E_FAIL:
		/* No-op */
		break;

	default:
		bfa_sm_fault(rspq->msgq, event);
	}
}

static void
rspq_sm_init_wait_entry(struct bfa_msgq_rspq_s *rspq)
{
	bfa_trc(rspq->msgq, 0);
	bfa_wc_down(&rspq->msgq->init_wc);
}

static void
rspq_sm_init_wait(struct bfa_msgq_rspq_s *rspq, enum rspq_event_e event)
{
	bfa_trc(rspq->msgq, event);

	switch (event) {
	case RSPQ_E_FAIL:
	case RSPQ_E_STOP:
		bfa_fsm_set_state(rspq, rspq_sm_stopped);
		break;

	case RSPQ_E_INIT_RESP:
		bfa_fsm_set_state(rspq, rspq_sm_ready);
		break;

	default:
		bfa_sm_fault(rspq->msgq, event);
	}
}

static void
rspq_sm_ready_entry(struct bfa_msgq_rspq_s *rspq)
{
	bfa_trc(rspq->msgq, 0);
}

static void
rspq_sm_ready(struct bfa_msgq_rspq_s *rspq, enum rspq_event_e event)
{
	bfa_trc(rspq->msgq, event);

	switch (event) {
	case RSPQ_E_STOP:
	case RSPQ_E_FAIL:
		bfa_fsm_set_state(rspq, rspq_sm_stopped);
		break;

	case RSPQ_E_RESP:
		bfa_fsm_set_state(rspq, rspq_sm_dbell_wait);
		break;

	default:
		bfa_sm_fault(rspq->msgq, event);
	}
}

static void
rspq_sm_dbell_wait_entry(struct bfa_msgq_rspq_s *rspq)
{
	bfa_trc(rspq->msgq, 0);
	if (!bfa_ioc_is_disabled(rspq->msgq->ioc))
		bfa_msgq_rspq_dbell(rspq);
}

static void
rspq_sm_dbell_wait(struct bfa_msgq_rspq_s *rspq, enum rspq_event_e event)
{
	bfa_trc(rspq->msgq, event);

	switch (event) {
	case RSPQ_E_STOP:
	case RSPQ_E_FAIL:
		bfa_fsm_set_state(rspq, rspq_sm_stopped);
		break;

	case RSPQ_E_RESP:
		rspq->flags |= BFA_MSGQ_RSPQ_F_DB_UPDATE;
		break;

	case RSPQ_E_DB_READY:
		if (rspq->flags & BFA_MSGQ_RSPQ_F_DB_UPDATE) {
			rspq->flags &= ~BFA_MSGQ_RSPQ_F_DB_UPDATE;
			bfa_fsm_set_state(rspq, rspq_sm_dbell_wait);
		} else
			bfa_fsm_set_state(rspq, rspq_sm_ready);
		break;

	default:
		bfa_sm_fault(rspq->msgq, event);
	}
}

static void
bfa_msgq_rspq_dbell_ready(void *arg)
{
	struct bfa_msgq_rspq_s *rspq = (struct bfa_msgq_rspq_s *)arg;
	bfa_fsm_send_event(rspq, RSPQ_E_DB_READY);
}

static void
bfa_msgq_rspq_dbell(struct bfa_msgq_rspq_s *rspq)
{
	struct bfi_msgq_h2i_db_s *dbell =
		(struct bfi_msgq_h2i_db_s *)(&rspq->dbell_mb.msg[0]);

	bfa_trc(rspq->msgq, rspq->consumer_index);
	bfa_trc(rspq->msgq, rspq->producer_index);

	bfa_os_memset(dbell, 0, sizeof(struct bfi_msgq_h2i_db_s));
	bfi_h2i_set(dbell->mh, BFI_MC_MSGQ, BFI_MSGQ_H2I_DOORBELL_CI, 0);
	dbell->mh.mtag.i2htok = 0;
	dbell->idx.rspq_ci = bfa_os_htons(rspq->consumer_index);

	if (!bfa_ioc_mbox_queue(rspq->msgq->ioc, &rspq->dbell_mb,
				bfa_msgq_rspq_dbell_ready, rspq)) {
		bfa_msgq_rspq_dbell_ready(rspq);
	}
}

static void
bfa_msgq_rspq_pi_update(struct bfa_msgq_rspq_s *rspq, struct bfi_mbmsg_s *mb)
{
	struct bfi_msgq_i2h_db_s *dbell = (struct bfi_msgq_i2h_db_s *)mb;
	struct bfi_msgq_mhdr_s *msghdr;
	int num_entries;
	int mc;
	uint8_t *rspq_qe;

	bfa_assert((rspq->fsm == (bfa_fsm_t)rspq_sm_ready) ||
		(rspq->fsm == (bfa_fsm_t)rspq_sm_dbell_wait));

	rspq->producer_index = bfa_os_ntohs(dbell->idx.rspq_pi);
	bfa_trc(rspq->msgq, rspq->consumer_index);
	bfa_trc(rspq->msgq, rspq->producer_index);

	while (rspq->consumer_index != rspq->producer_index) {
		rspq_qe = (uint8_t *)rspq->addr.kva;
		rspq_qe += (rspq->consumer_index * BFI_MSGQ_RSP_ENTRY_SIZE);
		msghdr = (struct bfi_msgq_mhdr_s *)rspq_qe;

		mc = msghdr->msg_class;
		num_entries = bfa_os_ntohs(msghdr->num_entries);

		if ((mc >= BFI_MC_MAX) || (rspq->rsphdlr[mc].cbfn == NULL)) {
			bfa_trc(rspq->msgq, mc);
			bfa_assert(0);
			bfa_trc_stop(rspq->msgq->trcmod);
			break;
		}

		bfa_assert(num_entries);
		bfa_trc(rspq->msgq, num_entries);

		(rspq->rsphdlr[mc].cbfn)(rspq->rsphdlr[mc].cbarg, msghdr);

		BFA_MSGQ_INDX_ADD(rspq->consumer_index, num_entries,
				rspq->depth);
	}

	bfa_fsm_send_event(rspq, RSPQ_E_RESP);
}

static void
bfa_msgq_rspq_attach(struct bfa_msgq_rspq_s *rspq, struct bfa_msgq_s *msgq)
{
	rspq->depth = BFA_MSGQ_RSPQ_NUM_ENTRY;
	rspq->msgq = msgq;
	bfa_fsm_set_state(rspq, rspq_sm_stopped);
}

static void
bfa_msgq_init_rsp(struct bfa_msgq_s *msgq,
		 struct bfi_mbmsg_s *mb)
{
	struct bfi_msgq_cfg_rsp_s *mb_cfg_rsp =
		(struct  bfi_msgq_cfg_rsp_s *)mb;

	bfa_assert(mb_cfg_rsp->cmd_status == 0);
	bfa_fsm_send_event(&msgq->cmdq, CMDQ_E_INIT_RESP);
	bfa_fsm_send_event(&msgq->rspq, RSPQ_E_INIT_RESP);
}

static void
bfa_msgq_init(void *arg)
{
	struct bfa_msgq_s *msgq = (struct bfa_msgq_s *)arg;
	struct bfi_msgq_cfg_req_s *msgq_cfg =
		(struct bfi_msgq_cfg_req_s *)&msgq->init_mb.msg[0];

	bfa_trc(msgq, msgq->cmdq.addr.pa);
	bfa_trc(msgq, bfa_os_u32(msgq->cmdq.addr.pa));
	bfa_trc(msgq, msgq->cmdq.depth);
	bfa_trc(msgq, msgq->cmdq.consumer_index);
	bfa_trc(msgq, msgq->cmdq.producer_index);

	bfa_trc(msgq, msgq->rspq.addr.pa);
	bfa_trc(msgq, bfa_os_u32(msgq->rspq.addr.pa));
	bfa_trc(msgq, msgq->rspq.depth);
	bfa_trc(msgq, msgq->rspq.consumer_index);
	bfa_trc(msgq, msgq->rspq.producer_index);

	bfa_os_memset(msgq_cfg, 0, sizeof(struct bfi_msgq_cfg_req_s));
	bfi_h2i_set(msgq_cfg->mh, BFI_MC_MSGQ, BFI_MSGQ_H2I_INIT_REQ, 0);
	msgq_cfg->mh.mtag.i2htok = 0;

	bfa_dma_be_addr_set(msgq_cfg->cmdq.addr, msgq->cmdq.addr.pa);
	msgq_cfg->cmdq.q_depth = bfa_os_htons(msgq->cmdq.depth);
	bfa_dma_be_addr_set(msgq_cfg->rspq.addr, msgq->rspq.addr.pa);
	msgq_cfg->rspq.q_depth = bfa_os_htons(msgq->rspq.depth);

	bfa_ioc_mbox_queue(msgq->ioc, &msgq->init_mb, NULL, NULL);
}

static void
bfa_msgq_isr(void *cbarg, struct bfi_mbmsg_s *msg)
{
	struct bfa_msgq_s *msgq = (struct bfa_msgq_s *)cbarg;

	bfa_assert(msg->mh.msg_class == BFI_MC_MSGQ);

	bfa_trc(msgq, msg->mh.msg_id);
	switch (msg->mh.msg_id) {
	case BFI_MSGQ_I2H_INIT_RSP:
		bfa_msgq_init_rsp(msgq, msg);
		break;

	case BFI_MSGQ_I2H_DOORBELL_PI:
		bfa_msgq_rspq_pi_update(&msgq->rspq, msg);
		break;

	case BFI_MSGQ_I2H_DOORBELL_CI:
		bfa_msgq_cmdq_ci_update(&msgq->cmdq, msg);
		break;

	case BFI_MSGQ_I2H_CMDQ_COPY_REQ:
		bfa_msgq_cmdq_copy_req(&msgq->cmdq, msg);
		break;

	default:
		bfa_assert(0);
	}
}

static void
bfa_msgq_notify(void *cbarg, enum bfa_ioc_event_e event)
{
	struct bfa_msgq_s *msgq = (struct bfa_msgq_s *)cbarg;

	bfa_trc(msgq, event);

	switch (event) {
	case BFA_IOC_E_ENABLED:
		bfa_wc_init(&msgq->init_wc, bfa_msgq_init, msgq);
		bfa_wc_up(&msgq->init_wc);
		bfa_fsm_send_event(&msgq->cmdq, CMDQ_E_START);
		bfa_wc_up(&msgq->init_wc);
		bfa_fsm_send_event(&msgq->rspq, RSPQ_E_START);
		bfa_wc_wait(&msgq->init_wc);
		break;

	case BFA_IOC_E_DISABLED:
		bfa_fsm_send_event(&msgq->cmdq, CMDQ_E_STOP);
		bfa_fsm_send_event(&msgq->rspq, RSPQ_E_STOP);
		break;

	case BFA_IOC_E_FAILED:
		bfa_fsm_send_event(&msgq->cmdq, CMDQ_E_FAIL);
		bfa_fsm_send_event(&msgq->rspq, RSPQ_E_FAIL);
		break;

	default:
		break;
	}
}

uint32_t
bfa_msgq_meminfo(void)
{
	return (BFA_ROUNDUP(BFA_MSGQ_CMDQ_SIZE, BFA_DMA_ALIGN_SZ) +
		BFA_ROUNDUP(BFA_MSGQ_RSPQ_SIZE, BFA_DMA_ALIGN_SZ));
}

void
bfa_msgq_memclaim(struct bfa_msgq_s *msgq, uint8_t *kva, uint64_t pa)
{
	bfa_assert((pa & (BFA_DMA_ALIGN_SZ - 1)) == 0);
	bfa_trc(msgq, pa);
	msgq->cmdq.addr.kva = kva;
	msgq->cmdq.addr.pa  = pa;

	kva += BFA_ROUNDUP(BFA_MSGQ_CMDQ_SIZE, BFA_DMA_ALIGN_SZ);
	pa += BFA_ROUNDUP(BFA_MSGQ_CMDQ_SIZE, BFA_DMA_ALIGN_SZ);

	bfa_trc(msgq, pa);
	msgq->rspq.addr.kva = kva;
	msgq->rspq.addr.pa = pa;
}

void
bfa_msgq_attach(struct bfa_msgq_s *msgq, struct bfa_ioc_s *ioc,
		struct bfa_trc_mod_s *trcmod,
		struct bfa_log_mod_s *logmod)
{
	msgq->ioc    = ioc;
	msgq->logm = logmod;
	msgq->trcmod = trcmod;

	bfa_msgq_cmdq_attach(&msgq->cmdq, msgq);
	bfa_msgq_rspq_attach(&msgq->rspq, msgq);

	bfa_ioc_mbox_regisr(msgq->ioc, BFI_MC_MSGQ, bfa_msgq_isr, msgq);
	bfa_q_qe_init(&msgq->ioc_notify);
	bfa_ioc_notify_init(&msgq->ioc_notify, bfa_msgq_notify, msgq);
	bfa_ioc_notify_register(msgq->ioc, &msgq->ioc_notify);

	bfa_trc(msgq, 0);
}

void
bfa_msgq_regisr(struct bfa_msgq_s *msgq, enum bfi_mclass mc,
		bfa_msgq_mcfunc_t cbfn, void *cbarg)
{
	msgq->rspq.rsphdlr[mc].cbfn	= cbfn;
	msgq->rspq.rsphdlr[mc].cbarg	= cbarg;
}

void
bfa_msgq_cmd_post(struct bfa_msgq_s *msgq,  struct bfa_msgq_cmd_entry_s *cmd)
{
	bfa_assert(cmd->msg_size > 0);

	if (bfa_os_ntohs(cmd->msg_hdr->num_entries) <=
		BFA_MSGQ_FREE_CNT(&msgq->cmdq)) {
		__cmd_copy(&msgq->cmdq, cmd);
		call_cmdq_ent_cbfn(cmd, BFA_STATUS_OK);
		bfa_fsm_send_event(&msgq->cmdq, CMDQ_E_POST);
	} else {
		bfa_q_enq(&msgq->cmdq.pending_q, &cmd->qe);
	}
}

void
bfa_msgq_rsp_copy(struct bfa_msgq_s *msgq, uint8_t *buf, size_t buf_len)
{
	struct bfa_msgq_rspq_s *rspq = &msgq->rspq;
	size_t len = buf_len;
	size_t to_copy;
	int ci;
	uint8_t *src, *dst;

	ci = rspq->consumer_index;
	src = (uint8_t *)rspq->addr.kva;
	src += (ci * BFI_MSGQ_RSP_ENTRY_SIZE);
	dst = buf;

	while (len) {
		to_copy = (len < BFI_MSGQ_RSP_ENTRY_SIZE) ?
				len : BFI_MSGQ_RSP_ENTRY_SIZE;
		bfa_os_memcpy(dst, src, to_copy);
		bfa_assert(len >= to_copy);
		len -= to_copy;
		dst += BFI_MSGQ_RSP_ENTRY_SIZE;
		BFA_MSGQ_INDX_ADD(ci, 1, rspq->depth);
		src = (uint8_t *)rspq->addr.kva;
		src += (ci * BFI_MSGQ_RSP_ENTRY_SIZE);
	}
}
