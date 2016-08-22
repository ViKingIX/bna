/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#include <bfi/bfi_flash.h>
#include <bfi/bfi_mfg.h>
#include <cna/bfa_flash.h>
#include <cna/bfa_ioc.h>

BFA_TRC_FILE(CNA, FLASH);

/**
 * FLASH DMA buffer should be big enough to hold both MFG block and
 * asic block(64k) at the same time and also should be 2k aligned to
 * avoid write segement to cross sector boundary.
 */
#define BFA_FLASH_SEG_SZ		2048
#define BFA_FLASH_DMA_BUF_SZ \
	BFA_ROUNDUP(0x010000 + sizeof(struct bfa_mfg_block_s), BFA_FLASH_SEG_SZ)

/*
 *****************************************************************************
 *
 * Internal functions
 *
 *****************************************************************************
 */

static void
bfa_hal_flash_aen_audit_post(struct bfa_ioc_s *ioc,
				enum bfa_audit_aen_event event,
				int inst,
				int type)
{
	union bfa_aen_data_u  aen_data;

	struct bfa_log_mod_s *logmod = ioc->logm;
	wwn_t	pwwn = bfa_ioc_get_pwwn(ioc);
	char	pwwn_buf[BFA_STRING_32];
	char	*pwwn_ptr;

	pwwn_ptr = wwn2str(pwwn_buf, sizeof(pwwn_buf), pwwn);
	bfa_log(logmod, BFA_LOG_CREATE_ID(BFA_AEN_CAT_AUDIT, event), inst,
			type, pwwn_ptr);

	aen_data.audit.pwwn = pwwn;
	aen_data.audit.partition_inst = inst;
	aen_data.audit.partition_type = type;
	bfa_aen_post(ioc->aen, BFA_AEN_CAT_AUDIT, event, &aen_data);
}

static void
bfa_flash_cb(struct bfa_flash_s *flash)
{
	flash->op_busy = 0;
	if (flash->cbfn)
		flash->cbfn(flash->cbarg, flash->status);
}

static void
bfa_flash_notify(void *cbarg, enum bfa_ioc_event_e event)
{
	bfa_flash_t *flash = cbarg;

	bfa_trc(flash, event);

	switch (event) {
	case BFA_IOC_E_DISABLED:
	case BFA_IOC_E_FAILED:
		if (flash->op_busy) {
			flash->status = BFA_STATUS_IOC_FAILURE;
			flash->cbfn(flash->cbarg, flash->status);
			flash->op_busy = 0;
		}
		break;

	default:
		break;
	}
}

/**
 * @brief Send flash attribute query request.
 *
 * @param[in] cbarg - callback argument
 */
static void
bfa_flash_query_send(void *cbarg)
{
	bfa_flash_t *flash = cbarg;
	bfi_flash_query_req_t *msg = (bfi_flash_query_req_t *) flash->mb.msg;

	bfi_h2i_set(msg->mh, BFI_MC_FLASH, BFI_FLASH_H2I_QUERY_REQ,
		    bfa_ioc_portid(flash->ioc));
	bfa_alen_set(&msg->alen, sizeof(bfa_flash_attr_t), flash->dbuf_pa);
	bfa_ioc_mbox_queue(flash->ioc, &flash->mb, NULL, NULL);
}

/**
 * @brief Send flash write request.
 *
 * @param[in] cbarg - callback argument
 */
static void
bfa_flash_write_send(bfa_flash_t *flash)
{
	bfi_flash_write_req_t *msg = (bfi_flash_write_req_t *) flash->mb.msg;
	uint32_t len;

	msg->type = bfa_os_ntohl(flash->type);
	msg->instance = flash->instance;
	msg->offset = bfa_os_ntohl(flash->addr_off + flash->offset);
	len = (flash->residue < BFA_FLASH_DMA_BUF_SZ) ?
		flash->residue : BFA_FLASH_DMA_BUF_SZ;
	msg->length = bfa_os_ntohl(len);

	/*
	 * indicate if it's the last msg of the whole write operation
	 */
	msg->last = (len == flash->residue) ? 1 : 0;

	bfi_h2i_set(msg->mh, BFI_MC_FLASH, BFI_FLASH_H2I_WRITE_REQ,
		    bfa_ioc_portid(flash->ioc));
	bfa_dma_alen_set(flash->ioc, &msg->alen, len,
			flash->dbuf_kva, flash->dbuf_pa);

	memcpy(flash->dbuf_kva, flash->ubuf + flash->offset, len);
	bfa_ioc_mbox_queue(flash->ioc, &flash->mb, NULL, NULL);

	flash->residue -= len;
	flash->offset += len;
}

/**
 * @brief Send flash read request.
 *
 * @param[in] cbarg - callback argument
 */
static void
bfa_flash_read_send(void *cbarg)
{
	bfa_flash_t *flash = cbarg;
	bfi_flash_read_req_t *msg = (bfi_flash_read_req_t *) flash->mb.msg;
	uint32_t len;

	msg->type = bfa_os_ntohl(flash->type);
	msg->instance = flash->instance;
	msg->offset = bfa_os_ntohl(flash->addr_off + flash->offset);
	len = (flash->residue < BFA_FLASH_DMA_BUF_SZ) ?
				flash->residue : BFA_FLASH_DMA_BUF_SZ;
	msg->length = bfa_os_ntohl(len);
	bfi_h2i_set(msg->mh, BFI_MC_FLASH, BFI_FLASH_H2I_READ_REQ,
		    bfa_ioc_portid(flash->ioc));
	bfa_alen_set(&msg->alen, len, flash->dbuf_pa);
	bfa_ioc_mbox_queue(flash->ioc, &flash->mb, NULL, NULL);
}

/**
 * @brief Send flash erase request.
 *
 * @param[in] cbarg - callback argument
 */
static void
bfa_flash_erase_send(void *cbarg)
{
	bfa_flash_t *flash = cbarg;
	bfi_flash_erase_req_t *msg = (bfi_flash_erase_req_t *) flash->mb.msg;

	msg->type = bfa_os_ntohl(flash->type);
	msg->instance = flash->instance;
	bfi_h2i_set(msg->mh, BFI_MC_FLASH, BFI_FLASH_H2I_ERASE_REQ,
		    bfa_ioc_portid(flash->ioc));
	bfa_ioc_mbox_queue(flash->ioc, &flash->mb, NULL, NULL);
}

static void
bfa_mfg_intr(void *flasharg, struct bfi_mbmsg_s *msg)
{
	struct bfa_flash_s *flash = flasharg;
	struct bfi_mfg_update_reply_s *update;

	if (msg->mh.msg_id != BFI_MFG_I2H_UPDATE_REPLY) {
		bfa_assert(0);
		return;
	}

	update = (struct bfi_mfg_update_reply_s *)msg;

	bfa_trc(flash, update->status);
	flash->status = update->status;
	bfa_flash_cb(flash);
}

/**
 * @brief Send get boot version request.
 *
 * @param[in] cbarg - callback argument
 */
static void
bfa_flash_boot_ver_send(void *cbarg)
{
	bfa_flash_t *flash = cbarg;
	bfi_flash_boot_ver_req_t *msg =
		(bfi_flash_boot_ver_req_t *) flash->mb.msg;

	bfi_h2i_set(msg->mh, BFI_MC_FLASH, BFI_FLASH_H2I_BOOT_VER_REQ,
		    bfa_ioc_portid(flash->ioc));
	bfa_alen_set(&msg->alen, flash->residue, flash->dbuf_pa);
	bfa_ioc_mbox_queue(flash->ioc, &flash->mb, NULL, NULL);
}

/**
 * @brief Get flash boot version.
 *
 * @param[in] flash - flash structure
 * @param[in] buf - read data buffer
 * @param[in] len - data buffer length
 * @param[in] cbfn - callback function
 * @param[in] cbarg - callback argument
 *
 * Return status.
 */
bfa_status_t
bfa_flash_get_boot_version(struct bfa_flash_s *flash,
	void *buf, uint32_t len, bfa_cb_flash_t cbfn, void *cbarg)
{
	bfa_trc(flash, BFI_FLASH_H2I_BOOT_VER_REQ);

	if (!bfa_ioc_is_operational(flash->ioc))
		return BFA_STATUS_IOC_NON_OP;

	if (len < BFA_VERSION_LEN) {
		bfa_trc(flash, len);
		return (BFA_STATUS_FLASH_BAD_LEN);
	}

	if (flash->op_busy) {
		bfa_trc(flash, flash->op_busy);
		return (BFA_STATUS_DEVBUSY);
	}

	flash->op_busy = 1;

	flash->cbfn = cbfn;
	flash->cbarg = cbarg;
	flash->residue = BFA_VERSION_LEN;
	flash->ubuf = buf;

	bfa_flash_boot_ver_send(flash);

	return (BFA_STATUS_OK);
}

/**
 * @brief Process flash response messages upon receiving interrupts.
 *
 * @param[in] flasharg - flash structure
 * @param[in] msg - message structure
 */
static void
bfa_flash_intr(void *flasharg, struct bfi_mbmsg_s *msg)
{
	struct bfa_flash_s *flash = flasharg;
	uint32_t status;

	union {
		bfi_flash_query_rsp_t *query;
		bfi_flash_erase_rsp_t *erase;
		bfi_flash_write_rsp_t *write;
		bfi_flash_read_rsp_t *read;
		bfi_flash_boot_ver_rsp_t *boot_ver;
		bfi_flash_event_t *event;
		struct bfi_mbmsg_s   *msg;
	} m;

	m.msg = msg;
	bfa_trc(flash, msg->mh.msg_id);

	if (!flash->op_busy && msg->mh.msg_id != BFI_FLASH_I2H_EVENT) {
		/*
		 * receiving response after ioc failure
		 */
		bfa_trc(flash, 0x9999);
		return;
	}

	switch (msg->mh.msg_id) {
	case BFI_FLASH_I2H_QUERY_RSP:
		status = bfa_os_ntohl(m.query->status);
		bfa_trc(flash, status);

		if (status == BFA_STATUS_OK) {
			uint32_t i;
			bfa_flash_attr_t *attr, *f;

			attr = (bfa_flash_attr_t *) flash->ubuf;
			f = (bfa_flash_attr_t *) flash->dbuf_kva;
			attr->status = bfa_os_ntohl(f->status);
			attr->npart = bfa_os_ntohl(f->npart);
			bfa_trc(flash, attr->status);
			bfa_trc(flash, attr->npart);
			for (i = 0; i < attr->npart; i++) {
				attr->part[i].part_type =
					bfa_os_ntohl(f->part[i].part_type);
				attr->part[i].part_instance =
					bfa_os_ntohl(f->part[i].part_instance);
				attr->part[i].part_off =
					bfa_os_ntohl(f->part[i].part_off);
				attr->part[i].part_size =
					bfa_os_ntohl(f->part[i].part_size);
				attr->part[i].part_len =
					bfa_os_ntohl(f->part[i].part_len);
				attr->part[i].part_status =
					bfa_os_ntohl(f->part[i].part_status);
			}

		}

		flash->status = status;
		bfa_flash_cb(flash);
		break;
	case BFI_FLASH_I2H_ERASE_RSP:
		status = bfa_os_ntohl(m.erase->status);
		bfa_trc(flash, status);

		flash->status = status;
		bfa_flash_cb(flash);
		break;
	case BFI_FLASH_I2H_WRITE_RSP:
		status = bfa_os_ntohl(m.write->status);
		bfa_trc(flash, status);

		if (status != BFA_STATUS_OK || flash->residue == 0) {
			flash->status = status;
			bfa_flash_cb(flash);
		} else {
			bfa_trc(flash, flash->offset);
			bfa_flash_write_send(flash);
		}
		break;
	case BFI_FLASH_I2H_READ_RSP:
		status = bfa_os_ntohl(m.read->status);
		bfa_trc(flash, status);

		if (status != BFA_STATUS_OK) {
			flash->status = status;
			bfa_flash_cb(flash);
		} else {
			uint32_t len = bfa_os_ntohl(m.read->length);
			bfa_trc(flash, flash->offset);
			bfa_trc(flash, len);

			memcpy(flash->ubuf + flash->offset,
				flash->dbuf_kva, len);
			flash->residue -= len;
			flash->offset += len;

			if (flash->residue == 0) {
				flash->status = status;
				bfa_flash_cb(flash);
			} else {
				bfa_flash_read_send(flash);
			}
		}
		break;
	case BFI_FLASH_I2H_BOOT_VER_RSP:
		status = bfa_os_ntohl(m.boot_ver->status);
		bfa_trc(flash, status);
		if (status == BFA_STATUS_OK) {
			memcpy(flash->ubuf, flash->dbuf_kva, flash->residue);
		}

		flash->status = status;
		bfa_flash_cb(flash);
		break;
	case BFI_FLASH_I2H_EVENT:
		status = bfa_os_ntohl(m.event->status);
		bfa_trc(flash, status);
		if (status == BFA_STATUS_BAD_FWCFG)
			bfa_ioc_aen_post(flash->ioc, BFA_IOC_AEN_FWCFG_ERROR);
		else if (status == BFA_STATUS_INVALID_VENDOR) {
			uint32_t param;
			param = bfa_os_ntohl(m.event->param);
			bfa_trc(flash, param);
			bfa_ioc_aen_post(flash->ioc,
				BFA_IOC_AEN_INVALID_VENDOR);
		}
		break;
	default:
		bfa_assert(0);
	}
}

/**
 * @brief Flash memory info API.
 *
 * @param[in] mincfg - minimal cfg variable
 */
uint32_t
bfa_flash_meminfo(bfa_boolean_t mincfg)
{
	/* min driver doesn't need flash */
	if (mincfg)
		return 0;

	return (BFA_ROUNDUP(BFA_FLASH_DMA_BUF_SZ, BFA_DMA_ALIGN_SZ));
}

/**
 * @brief Flash attach API.
 *
 * @param[in] flash - flash structure
 * @param[in] ioc  - ioc structure
 * @param[in] dev  - device structure
 * @param[in] trcmod - trace module
 * @param[in] logmod - log module
 */
void
bfa_flash_attach(struct bfa_flash_s *flash, struct bfa_ioc_s *ioc, void *dev,
	bfa_trc_mod_t *trcmod, bfa_boolean_t mincfg)
{
	flash->ioc = ioc;
	flash->trcmod = trcmod;
	flash->cbfn = NULL;
	flash->cbarg = NULL;
	flash->op_busy = 0;

	bfa_ioc_mbox_regisr(flash->ioc, BFI_MC_FLASH, bfa_flash_intr, flash);
	bfa_ioc_mbox_regisr(flash->ioc, BFI_MC_MFG, bfa_mfg_intr, flash);
	bfa_q_qe_init(&flash->ioc_notify);
	bfa_ioc_notify_init(&flash->ioc_notify, bfa_flash_notify, flash);
	bfa_ioc_notify_register(flash->ioc, &flash->ioc_notify);

	/* min driver doesn't need flash */
	if (mincfg) {
		flash->dbuf_kva = NULL;
		flash->dbuf_pa = 0;
	}
}

/**
 * @brief Claim memory for flash
 *
 * @param[in] flash - flash structure
 * @param[in] dm_kva - pointer to virtual memory address
 * @param[in] dm_pa - physical memory address
 * @param[in] mincfg - minimal cfg variable
 */
void
bfa_flash_memclaim(struct bfa_flash_s *flash, uint8_t *dm_kva, uint64_t dm_pa,
	bfa_boolean_t mincfg)
{
	if (mincfg)
		return;

	flash->dbuf_kva = dm_kva;
	flash->dbuf_pa = dm_pa;
	bfa_os_memset(flash->dbuf_kva, 0, BFA_FLASH_DMA_BUF_SZ);
	dm_kva += BFA_ROUNDUP(BFA_FLASH_DMA_BUF_SZ, BFA_DMA_ALIGN_SZ);
	dm_pa += BFA_ROUNDUP(BFA_FLASH_DMA_BUF_SZ, BFA_DMA_ALIGN_SZ);
}

/*
 *****************************************************************************
 *
 * Externalized functions
 *
 *****************************************************************************
 */

/**
 * @ig hal_flash_api
 * @{
 */

/**
 * @brief Get flash attribute.
 *
 * @param[in] flash - flash structure
 * @param[in] attr - flash attribute structure
 * @param[in] cbfn - callback function
 * @param[in] cbarg - callback argument
 *
 * Return status.
 */
bfa_status_t
bfa_flash_get_attr(struct bfa_flash_s *flash, bfa_flash_attr_t *attr,
		       bfa_cb_flash_t cbfn, void *cbarg)
{
	bfa_trc(flash, BFI_FLASH_H2I_QUERY_REQ);

	if (!bfa_ioc_is_operational(flash->ioc))
		return BFA_STATUS_IOC_NON_OP;

	if (flash->op_busy) {
		bfa_trc(flash, flash->op_busy);
		return (BFA_STATUS_DEVBUSY);
	}

	flash->op_busy = 1;

	flash->cbfn = cbfn;
	flash->cbarg = cbarg;
	flash->ubuf = (uint8_t *) attr;
	bfa_flash_query_send(flash);

	return (BFA_STATUS_OK);
}

/**
 * @brief Erase flash partition.
 *
 * @param[in] flash - flash structure
 * @param[in] type - flash partition type
 * @param[in] instance - flash partition instance
 * @param[in] cbfn - callback function
 * @param[in] cbarg - callback argument
 *
 * Return status.
 */
bfa_status_t
bfa_flash_erase_part(struct bfa_flash_s *flash, bfa_flash_part_type_t type,
			 uint8_t instance, bfa_cb_flash_t cbfn, void *cbarg)
{
	bfa_trc(flash, BFI_FLASH_H2I_ERASE_REQ);
	bfa_trc(flash, type);
	bfa_trc(flash, instance);

	if (!bfa_ioc_is_operational(flash->ioc))
		return BFA_STATUS_IOC_NON_OP;

	if (flash->op_busy) {
		bfa_trc(flash, flash->op_busy);
		return (BFA_STATUS_DEVBUSY);
	}

	flash->op_busy = 1;

	flash->cbfn = cbfn;
	flash->cbarg = cbarg;
	flash->type = type;
	flash->instance = instance;

	bfa_flash_erase_send(flash);

	bfa_hal_flash_aen_audit_post(flash->ioc, BFA_AUDIT_AEN_FLASH_ERASE,
					instance, type);

	return (BFA_STATUS_OK);
}

/**
 * @brief Update flash partition.
 *
 * @param[in] flash - flash structure
 * @param[in] type - flash partition type
 * @param[in] instance - flash partition instance
 * @param[in] buf - update data buffer
 * @param[in] len - data buffer length
 * @param[in] offset - offset relative to the partition starting address
 * @param[in] cbfn - callback function
 * @param[in] cbarg - callback argument
 *
 * Return status.
 */
bfa_status_t
bfa_flash_update_part(struct bfa_flash_s *flash, bfa_flash_part_type_t type,
			  uint8_t instance, void *buf, uint32_t len,
			  uint32_t offset, bfa_cb_flash_t cbfn,
			  void *cbarg, uint8_t chk_offset)
{
	bfa_trc(flash, BFI_FLASH_H2I_WRITE_REQ);
	bfa_trc(flash, type);
	bfa_trc(flash, instance);
	bfa_trc(flash, len);
	bfa_trc(flash, offset);

	if (!bfa_ioc_is_operational(flash->ioc))
		return BFA_STATUS_IOC_NON_OP;

	/**
	 * 'len' must be in word (4-byte) boundary
	 * 'offset' must be in sector (16kb) boundary
	 */
	if (!len || (len & 0x03) || (chk_offset && (offset & 0x00003FFF)))
		return (BFA_STATUS_FLASH_BAD_LEN);

	if (type == BFA_FLASH_PART_MFG)
		return BFA_STATUS_EINVAL;

	if (flash->op_busy) {
		bfa_trc(flash, flash->op_busy);
		return (BFA_STATUS_DEVBUSY);
	}

	flash->op_busy = 1;

	flash->cbfn = cbfn;
	flash->cbarg = cbarg;
	flash->type = type;
	flash->instance = instance;
	flash->residue = len;
	flash->offset = 0;
	flash->addr_off = offset;
	flash->ubuf = buf;

	bfa_flash_write_send(flash);

	return (BFA_STATUS_OK);
}

/**
 * @brief Read flash partition.
 *
 * @param[in] flash - flash structure
 * @param[in] type - flash partition type
 * @param[in] instance - flash partition instance
 * @param[in] buf - read data buffer
 * @param[in] len - data buffer length
 * @param[in] offset - offset relative to the partition starting address
 * @param[in] cbfn - callback function
 * @param[in] cbarg - callback argument
 *
 * Return status.
 */
bfa_status_t
bfa_flash_read_part(struct bfa_flash_s *flash, bfa_flash_part_type_t type,
			uint8_t instance, void *buf, uint32_t len,
			uint32_t offset, bfa_cb_flash_t cbfn,
			void *cbarg, uint8_t chk_offset)
{
	bfa_trc(flash, BFI_FLASH_H2I_READ_REQ);
	bfa_trc(flash, type);
	bfa_trc(flash, instance);
	bfa_trc(flash, len);
	bfa_trc(flash, offset);

	if (!bfa_ioc_is_operational(flash->ioc))
		return BFA_STATUS_IOC_NON_OP;

	/**
	 * 'len' must be in word (4-byte) boundary
	 * 'offset' must be in sector (16kb) boundary
	 */
	if (!len || (len & 0x03) || (chk_offset && (offset & 0x00003FFF)))
		return (BFA_STATUS_FLASH_BAD_LEN);

	if (flash->op_busy) {
		bfa_trc(flash, flash->op_busy);
		return (BFA_STATUS_DEVBUSY);
	}

	flash->op_busy = 1;

	flash->cbfn = cbfn;
	flash->cbarg = cbarg;
	flash->type = type;
	flash->instance = instance;
	flash->residue = len;
	flash->offset = 0;
	flash->addr_off = offset;
	flash->ubuf = buf;
	bfa_flash_read_send(flash);

	return (BFA_STATUS_OK);
}

bfa_status_t
bfa_mfg_update(struct bfa_flash_s *flash, struct bfa_mfg_block_s *mfgblk,
	void *asic_block, int asic_blen, bfa_cb_flash_t cbfn, void *cbarg)
{
	struct bfi_mfg_update_req_s *msg;

	if (!bfa_ioc_is_operational(flash->ioc))
		return BFA_STATUS_IOC_NON_OP;

	if (flash->op_busy) {
		bfa_trc(flash, flash->op_busy);
		return (BFA_STATUS_DEVBUSY);
	}

	if (asic_blen && (asic_blen & 0xFFFF))
		return (BFA_STATUS_FLASH_BAD_LEN);

	if ((asic_blen + sizeof(struct bfa_mfg_block_s)) > BFA_FLASH_DMA_BUF_SZ)
		return BFA_STATUS_ENOMEM;

	flash->op_busy = 1;
	bfa_hal_flash_aen_audit_post(flash->ioc, BFA_AUDIT_AEN_FLASH_UPDATE,
				0, BFA_FLASH_PART_MFG);
	flash->cbfn = cbfn;
	flash->cbarg = cbarg;

	msg = (struct bfi_mfg_update_req_s *) flash->mb.msg;
	bfi_h2i_set(msg->mh, BFI_MC_MFG, BFI_MFG_H2I_UPDATE_REQ,
		bfa_ioc_portid(flash->ioc));

	bfa_dma_alen_set(flash->ioc, &msg->alen_mfgblk,
			sizeof(struct bfa_mfg_block_s),
			flash->dbuf_kva, flash->dbuf_pa);

	memcpy(flash->dbuf_kva, mfgblk, sizeof(struct bfa_mfg_block_s));

	bfa_dma_alen_set(flash->ioc, &msg->alen_ablk, asic_blen,
			flash->dbuf_kva + sizeof(struct bfa_mfg_block_s),
			flash->dbuf_pa + sizeof(struct bfa_mfg_block_s));
	if (asic_blen > 0)
		memcpy(flash->dbuf_kva + sizeof(struct bfa_mfg_block_s),
			asic_block, asic_blen);
	bfa_ioc_mbox_queue(flash->ioc, &flash->mb, NULL, NULL);

	return BFA_STATUS_OK;
}

/**
 * @}
 */
