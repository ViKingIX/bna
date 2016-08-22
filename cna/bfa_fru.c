/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#include <bfi/bfi_fru.h>
#include <cna/bfa_fru.h>
#include <cna/bfa_ioc.h>

BFA_TRC_FILE(CNA, FRU);

#define BFA_FRU_DMA_BUF_SZ  	0x0800		/*!< 2k dma buffer */
#define BFA_FRU_CHINOOK_MAX_SIZE 0x10000
#define BFA_FRU_LIGHTNING_MAX_SIZE 0x200
/*
 *****************************************************************************
 *
 * Internal functions
 *
 *****************************************************************************
 */

static bfa_boolean_t
bfa_fru_present(struct bfa_fru_s *fru)
{
	return (fru->ioc->asic_gen == BFI_ASIC_GEN_CT2);
}


static void
bfa_fru_cb(struct bfa_fru_s *fru)
{
	fru->op_busy = 0;
	if (fru->cbfn)
		fru->cbfn(fru->cbarg, fru->status);
}

static void
bfa_fru_notify(void *cbarg, enum bfa_ioc_event_e event)
{
	bfa_fru_t *fru = cbarg;

	bfa_trc(fru, event);

	switch (event) {
	case BFA_IOC_E_DISABLED:
	case BFA_IOC_E_FAILED:
		if (fru->op_busy) {
			fru->status = BFA_STATUS_IOC_FAILURE;
			fru->cbfn(fru->cbarg, fru->status);
			fru->op_busy = 0;
		}
		break;

	default:
		break;
	}
}

/**
 * @brief Send fru write request.
 *
 * @param[in] cbarg - callback argument
 */
static void
bfa_fru_write_send(void *cbarg, bfi_fru_h2i_msgs_t msg_type)
{
	bfa_fru_t *fru = cbarg;
	bfi_fru_write_req_t *msg = (bfi_fru_write_req_t *) fru->mb.msg;
	uint32_t len;

	msg->offset = bfa_os_htonl(fru->addr_off + fru->offset);
	len = (fru->residue < BFA_FRU_DMA_BUF_SZ) ?
				fru->residue : BFA_FRU_DMA_BUF_SZ;
	msg->length = bfa_os_htonl(len);

	/*
	 * indicate if it's the last msg of the whole write operation
	 */
	msg->last = (len == fru->residue) ? 1 : 0;
	msg->trfr_cmpl = (len == fru->residue) ? fru->trfr_cmpl : 0;
	bfi_h2i_set(msg->mh, BFI_MC_FRU, msg_type, bfa_ioc_portid(fru->ioc));
	bfa_dma_alen_set(fru->ioc, &msg->alen, len, fru->dbuf_kva,
				fru->dbuf_pa);

	memcpy(fru->dbuf_kva, fru->ubuf + fru->offset, len);
	bfa_ioc_mbox_queue(fru->ioc, &fru->mb, NULL, NULL);

	fru->residue -= len;
	fru->offset += len;
}

/**
 * @brief Send fru read request.
 *
 * @param[in] cbarg - callback argument
 */
static void
bfa_fru_read_send(void *cbarg, bfi_fru_h2i_msgs_t msg_type)
{
	bfa_fru_t *fru = cbarg;
	bfi_fru_read_req_t *msg = (bfi_fru_read_req_t *) fru->mb.msg;
	uint32_t len;

	msg->offset = bfa_os_htonl(fru->addr_off + fru->offset);
	len = (fru->residue < BFA_FRU_DMA_BUF_SZ) ?
				fru->residue : BFA_FRU_DMA_BUF_SZ;
	msg->length = bfa_os_htonl(len);
	bfi_h2i_set(msg->mh, BFI_MC_FRU, msg_type, bfa_ioc_portid(fru->ioc));
	bfa_alen_set(&msg->alen, len, fru->dbuf_pa);
	bfa_ioc_mbox_queue(fru->ioc, &fru->mb, NULL, NULL);
}

/**
 * @brief Flash memory info API.
 *
 * @param[in] mincfg - minimal cfg variable
 */
uint32_t
bfa_fru_meminfo(bfa_boolean_t mincfg)
{
	/* min driver doesn't need fru */
	if (mincfg)
		return 0;

	return (BFA_ROUNDUP(BFA_FRU_DMA_BUF_SZ, BFA_DMA_ALIGN_SZ));
}

/**
 * @brief Flash attach API.
 *
 * @param[in] fru - fru structure
 * @param[in] ioc  - ioc structure
 * @param[in] dev  - device structure
 * @param[in] trcmod - trace module
 * @param[in] logmod - log module
 */
void
bfa_fru_attach(struct bfa_fru_s *fru, struct bfa_ioc_s *ioc, void *dev,
	bfa_trc_mod_t *trcmod, bfa_boolean_t mincfg)
{
	fru->ioc = ioc;
	fru->trcmod = trcmod;
	fru->cbfn = NULL;
	fru->cbarg = NULL;
	fru->op_busy = 0;

	bfa_ioc_mbox_regisr(fru->ioc, BFI_MC_FRU, bfa_fru_intr, fru);
	bfa_q_qe_init(&fru->ioc_notify);
	bfa_ioc_notify_init(&fru->ioc_notify, bfa_fru_notify, fru);
	bfa_ioc_notify_register(fru->ioc, &fru->ioc_notify);

	/* min driver doesn't need fru */
	if (mincfg) {
		fru->dbuf_kva = NULL;
		fru->dbuf_pa = 0;
	}
}

/**
 * @brief Claim memory for fru
 *
 * @param[in] fru - fru structure
 * @param[in] dm_kva - pointer to virtual memory address
 * @param[in] dm_pa - frusical memory address
 * @param[in] mincfg - minimal cfg variable
 */
void
bfa_fru_memclaim(struct bfa_fru_s *fru, uint8_t *dm_kva, uint64_t dm_pa,
	bfa_boolean_t mincfg)
{
	if (mincfg)
		return;

	fru->dbuf_kva = dm_kva;
	fru->dbuf_pa = dm_pa;
	bfa_os_memset(fru->dbuf_kva, 0, BFA_FRU_DMA_BUF_SZ);
	dm_kva += BFA_ROUNDUP(BFA_FRU_DMA_BUF_SZ, BFA_DMA_ALIGN_SZ);
	dm_pa += BFA_ROUNDUP(BFA_FRU_DMA_BUF_SZ, BFA_DMA_ALIGN_SZ);
}

/*
 *****************************************************************************
 *
 * Externalized functions
 *
 *****************************************************************************
 */


/**
 * @ig hal_fru_api
 * @{
 */

/**
 * @brief Update fru vpd image.
 *
 * @param[in] fru - fru structure
 * @param[in] buf - update data buffer
 * @param[in] len - data buffer length
 * @param[in] offset - offset relative to starting address
 * @param[in] cbfn - callback function
 * @param[in] cbarg - callback argument
 *
 * Return status.
 */
bfa_status_t
bfa_fruvpd_update(struct bfa_fru_s *fru,
	void *buf, uint32_t len, uint32_t offset,
	bfa_cb_fru_t cbfn, void *cbarg, uint8_t trfr_cmpl)
{
	bfa_trc(fru, BFI_FRUVPD_H2I_WRITE_REQ);
	bfa_trc(fru, len);
	bfa_trc(fru, offset);

	if (!bfa_fru_present(fru))
		return BFA_STATUS_FRU_NOT_PRESENT;
	/*
	 * For Lightning2 and Prowler command unsupported
	*/
	if (bfa_ioc_get_card_type(fru->ioc) != BFA_MFG_TYPE_CHINOOK &&
		bfa_ioc_get_card_type(fru->ioc) != BFA_MFG_TYPE_CHINOOK2)
		return BFA_STATUS_CMD_NOTSUPP;

	if (!bfa_ioc_is_operational(fru->ioc))
		return BFA_STATUS_IOC_NON_OP;

	if (fru->op_busy) {
		bfa_trc(fru, fru->op_busy);
		return (BFA_STATUS_DEVBUSY);
	}

	fru->op_busy = 1;

	fru->cbfn = cbfn;
	fru->cbarg = cbarg;
	fru->residue = len;
	fru->offset = 0;
	fru->addr_off = offset;
	fru->ubuf = buf;
	fru->trfr_cmpl = trfr_cmpl;

	bfa_fru_write_send(fru, BFI_FRUVPD_H2I_WRITE_REQ);

	return (BFA_STATUS_OK);
}

/**
 * @brief Read fru vpd image.
 *
 * @param[in] fru - fru structure
 * @param[in] buf - read data buffer
 * @param[in] len - data buffer length
 * @param[in] offset - offset relative to starting address
 * @param[in] cbfn - callback function
 * @param[in] cbarg - callback argument
 *
 * Return status.
 */
bfa_status_t
bfa_fruvpd_read(struct bfa_fru_s *fru,
	void *buf, uint32_t len, uint32_t offset,
	bfa_cb_fru_t cbfn, void *cbarg)
{
	bfa_trc(fru, BFI_FRUVPD_H2I_READ_REQ);
	bfa_trc(fru, len);
	bfa_trc(fru, offset);

	if (!bfa_fru_present(fru))
		return BFA_STATUS_FRU_NOT_PRESENT;

	/*
	 * For Lightning2 and Prowler command unsupported
	*/
	if (bfa_ioc_get_card_type(fru->ioc) != BFA_MFG_TYPE_CHINOOK &&
		bfa_ioc_get_card_type(fru->ioc) != BFA_MFG_TYPE_CHINOOK2)
		return BFA_STATUS_CMD_NOTSUPP;

	if (!bfa_ioc_is_operational(fru->ioc))
		return BFA_STATUS_IOC_NON_OP;

	if (fru->op_busy) {
		bfa_trc(fru, fru->op_busy);
		return (BFA_STATUS_DEVBUSY);
	}

	fru->op_busy = 1;

	fru->cbfn = cbfn;
	fru->cbarg = cbarg;
	fru->residue = len;
	fru->offset = 0;
	fru->addr_off = offset;
	fru->ubuf = buf;
	bfa_fru_read_send(fru, BFI_FRUVPD_H2I_READ_REQ);

	return (BFA_STATUS_OK);
}

/**
 * @brief get maximum size fru vpd image.
 *
 * @param[in] fru - fru structure
 * @param[out] size - maximum size of fru vpd data
 *
 * Return status.
 */
bfa_status_t
bfa_fruvpd_get_max_size(struct bfa_fru_s *fru, uint32_t *max_size)
{
	if (!bfa_fru_present(fru))
		return BFA_STATUS_FRU_NOT_PRESENT;

	if (!bfa_ioc_is_operational(fru->ioc))
		return BFA_STATUS_IOC_NON_OP;

	if (bfa_ioc_get_card_type(fru->ioc) == BFA_MFG_TYPE_CHINOOK ||
		bfa_ioc_get_card_type(fru->ioc) == BFA_MFG_TYPE_CHINOOK2)
		*max_size = BFA_FRU_CHINOOK_MAX_SIZE;
	/*
	 * Lightning2 Specific
	else if (bfa_ioc_get_card_type(fru->ioc) == BFA_MFG_TYPE_LIGHTNING2)
		*max_size = BFA_FRU_LIGHTNING_MAX_SIZE;
	else
		*max_size = 0x200;
	*/
	else
		return BFA_STATUS_CMD_NOTSUPP;
	return BFA_STATUS_OK;
}
/**
 * @brief tfru write.
 *
 * @param[in] fru - fru structure
 * @param[in] buf - update data buffer
 * @param[in] len - data buffer length
 * @param[in] offset - offset relative to starting address
 * @param[in] cbfn - callback function
 * @param[in] cbarg - callback argument
 *
 * Return status.
 */
bfa_status_t
bfa_tfru_write(struct bfa_fru_s *fru,
	void *buf, uint32_t len, uint32_t offset,
	bfa_cb_fru_t cbfn, void *cbarg)
{
	bfa_trc(fru, BFI_TFRU_H2I_WRITE_REQ);
	bfa_trc(fru, len);
	bfa_trc(fru, offset);
	bfa_trc(fru, *((uint8_t *) buf));

	if (!bfa_fru_present(fru))
		return BFA_STATUS_FRU_NOT_PRESENT;

	if (!bfa_ioc_is_operational(fru->ioc))
		return BFA_STATUS_IOC_NON_OP;

	if (fru->op_busy) {
		bfa_trc(fru, fru->op_busy);
		return (BFA_STATUS_DEVBUSY);
	}

	fru->op_busy = 1;

	fru->cbfn = cbfn;
	fru->cbarg = cbarg;
	fru->residue = len;
	fru->offset = 0;
	fru->addr_off = offset;
	fru->ubuf = buf;

	bfa_fru_write_send(fru, BFI_TFRU_H2I_WRITE_REQ);

	return (BFA_STATUS_OK);
}

/**
 * @brief tfru read.
 *
 * @param[in] fru - fru structure
 * @param[in] buf - read data buffer
 * @param[in] len - data buffer length
 * @param[in] offset - offset relative to starting address
 * @param[in] cbfn - callback function
 * @param[in] cbarg - callback argument
 *
 * Return status.
 */
bfa_status_t
bfa_tfru_read(struct bfa_fru_s *fru,
	void *buf, uint32_t len, uint32_t offset,
	bfa_cb_fru_t cbfn, void *cbarg)
{
	bfa_trc(fru, BFI_TFRU_H2I_READ_REQ);
	bfa_trc(fru, len);
	bfa_trc(fru, offset);

	if (!bfa_fru_present(fru))
		return BFA_STATUS_FRU_NOT_PRESENT;

	if (!bfa_ioc_is_operational(fru->ioc))
		return BFA_STATUS_IOC_NON_OP;

	if (fru->op_busy) {
		bfa_trc(fru, fru->op_busy);
		return (BFA_STATUS_DEVBUSY);
	}

	fru->op_busy = 1;

	fru->cbfn = cbfn;
	fru->cbarg = cbarg;
	fru->residue = len;
	fru->offset = 0;
	fru->addr_off = offset;
	fru->ubuf = buf;
	bfa_fru_read_send(fru, BFI_TFRU_H2I_READ_REQ);

	return (BFA_STATUS_OK);
}

/**
 * @brief Process fru response messages upon receiving interrupts.
 *
 * @param[in] fruarg - fru structure
 * @param[in] msg - message structure
 */
void
bfa_fru_intr(void *fruarg, struct bfi_mbmsg_s *msg)
{
	struct bfa_fru_s *fru = fruarg;
	bfi_fru_rsp_t *rsp = (bfi_fru_rsp_t *)msg;
	uint32_t status;

	bfa_trc(fru, msg->mh.msg_id);

	if (!fru->op_busy) {
		/*
		 * receiving response after ioc failure
		 */
		bfa_trc(fru, 0x9999);
		return;
	}

	switch (msg->mh.msg_id) {
	case BFI_FRUVPD_I2H_WRITE_RSP:
	case BFI_TFRU_I2H_WRITE_RSP:
		status = bfa_os_ntohl(rsp->status);
		bfa_trc(fru, status);

		if (status != BFA_STATUS_OK || fru->residue == 0) {
			fru->status = status;
			bfa_fru_cb(fru);
		} else {
			bfa_trc(fru, fru->offset);
			if (msg->mh.msg_id == BFI_FRUVPD_I2H_WRITE_RSP)
				bfa_fru_write_send(fru,
					BFI_FRUVPD_H2I_WRITE_REQ);
			else
				bfa_fru_write_send(fru,
					BFI_TFRU_H2I_WRITE_REQ);
		}
		break;
	case BFI_FRUVPD_I2H_READ_RSP:
	case BFI_TFRU_I2H_READ_RSP:
		status = bfa_os_ntohl(rsp->status);
		bfa_trc(fru, status);

		if (status != BFA_STATUS_OK) {
			fru->status = status;
			bfa_fru_cb(fru);
		} else {
			uint32_t len = bfa_os_ntohl(rsp->length);

			bfa_trc(fru, fru->offset);
			bfa_trc(fru, len);

			memcpy(fru->ubuf + fru->offset, fru->dbuf_kva, len);
			fru->residue -= len;
			fru->offset += len;

			if (fru->residue == 0) {
				fru->status = status;
				bfa_fru_cb(fru);
			} else {
				if (msg->mh.msg_id == BFI_FRUVPD_I2H_READ_RSP)
					bfa_fru_read_send(fru,
						BFI_FRUVPD_H2I_READ_REQ);
				else
					bfa_fru_read_send(fru,
						BFI_TFRU_H2I_READ_REQ);
			}
		}
		break;
	default:
		bfa_assert(0);
	}
}

/**
 * @}
 */
