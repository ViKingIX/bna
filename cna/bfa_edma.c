/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

/**
 * @file bfa_edma.c EDMA module source file.
 */

#include <bfi/bfi.h>
#include <cna/bfa_edma.h>

BFA_TRC_FILE(CNA, EDMA);

static void bfa_edma_read_rsp(struct bfa_edma_s *edma);

static void
bfa_edma_read_next(void *arg)
{
	struct bfa_edma_s *edma = (struct bfa_edma_s *)arg;

	edma->mbox_busy = BFA_FALSE;
	bfa_edma_read_rsp(edma);
}

static void
bfa_edma_read_rsp(struct bfa_edma_s *edma)
{
	bfi_edma_h2i_read_rsp_t *rsp =
			(bfi_edma_h2i_read_rsp_t *)&edma->read_mb.msg[0];
	int copied;

	while (edma->bytes_to_copy && !edma->mbox_busy) {

		bfa_os_memset(rsp, 0, sizeof(bfi_edma_h2i_read_rsp_t));

		bfi_h2i_set(rsp->mh, BFI_MC_EDMA, BFI_EDMA_H2I_READ_RSP, 0);

		copied = (edma->bytes_to_copy >= BFI_EDMA_READ_SZ) ?
				BFI_EDMA_READ_SZ : edma->bytes_to_copy;

		memcpy(rsp->data, edma->addr, copied);

		edma->addr += copied;
		bfa_assert(edma->bytes_to_copy >= copied);
		edma->bytes_to_copy -= copied;

		edma->mbox_busy = bfa_ioc_mbox_queue(edma->ioc,
				&edma->read_mb, bfa_edma_read_next, edma);
	}
}

static void
bfa_edma_process_read_req(struct bfa_edma_s *edma, struct bfi_mbmsg_s *mb)
{
	bfi_edma_i2h_read_req_t *req = (bfi_edma_i2h_read_req_t *)mb;
	uint64_t addr;

	bfa_assert(edma->bytes_to_copy == 0);
	bfa_assert(req->len);

	addr = (uint64_t)bfa_os_ntohl(req->addr_hi);
	edma->addr = (uint8_t *)((addr << 32) |
			bfa_os_ntohl(req->addr_lo));

	edma->bytes_to_copy = bfa_os_ntohl(req->len);

	bfa_trc(edma, (uint64_t)edma->addr);
	bfa_trc(edma, edma->bytes_to_copy);

	edma->mbox_busy = BFA_FALSE;
	bfa_edma_read_rsp(edma);
}

static void
bfa_edma_isr(void *cbarg, struct bfi_mbmsg_s *msg)
{
	struct bfa_edma_s *edma = (struct bfa_edma_s *)cbarg;

	bfa_assert(msg->mh.msg_class == BFI_MC_EDMA);

	switch (msg->mh.msg_id) {
	case BFI_EDMA_I2H_READ_REQ:
		bfa_edma_process_read_req(edma, msg);
		break;

	default:
		bfa_assert(0);
	}
}

void
bfa_edma_attach(bfa_edma_t *edma, struct bfa_ioc_s *ioc,
				struct bfa_trc_mod_s *trcmod)
{
	edma->ioc	= ioc;
	edma->trcmod	= trcmod;

	bfa_ioc_mbox_regisr(edma->ioc, BFI_MC_EDMA, bfa_edma_isr, edma);
	bfa_trc(edma, 0);
}
