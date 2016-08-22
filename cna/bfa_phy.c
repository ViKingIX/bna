/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#include <bfi/bfi_phy.h>
#include <cna/bfa_phy.h>
#include <cna/bfa_ioc.h>

BFA_TRC_FILE(CNA, PHY);

#define BFA_PHY_DMA_BUF_SZ  	0x02000		/*!< 8k dma buffer */
#define BFA_PHY_LOCK_STATUS	0x018878	/*!< phy semaphore status reg */

/*
 *****************************************************************************
 *
 * Internal functions
 *
 *****************************************************************************
 */

static void
bfa_phy_ntoh32(uint32_t *obuf, uint32_t *ibuf, int sz)
{
	int i, m = sz >> 2;

	for (i = 0; i < m; i++)
		obuf[i] = bfa_os_ntohl(ibuf[i]);
}

static void
bfa_phy_ntoh16(uint16_t *obuf, uint16_t *ibuf, int sz)
{
	int i, m = sz >> 1;

	for (i = 0; i < m; i++)
		obuf[i] = bfa_os_ntohs(ibuf[i]);
}

static void
bfa_phy_hton16(uint16_t *obuf, uint16_t *ibuf, int sz)
{
	int i, m = sz >> 1;

	for (i = 0; i < m; i++)
		obuf[i] = bfa_os_htons(ibuf[i]);
}

static bfa_boolean_t
bfa_phy_present(struct bfa_phy_s *phy)
{
	return (phy->ioc->attr->card_type == BFA_MFG_TYPE_LIGHTNING);
}


static void
bfa_phy_cb(struct bfa_phy_s *phy)
{
	phy->op_busy = 0;
	if (phy->cbfn)
		phy->cbfn(phy->cbarg, phy->status);
}

static void
bfa_phy_notify(void *cbarg, enum bfa_ioc_event_e event)
{
	bfa_phy_t *phy = cbarg;

	bfa_trc(phy, event);

	switch (event) {
	case BFA_IOC_E_DISABLED:
	case BFA_IOC_E_FAILED:
		if (phy->op_busy) {
			phy->status = BFA_STATUS_IOC_FAILURE;
			phy->cbfn(phy->cbarg, phy->status);
			phy->op_busy = 0;
		}
		break;

	default:
		break;
	}
}

/**
 * @brief Send phy attribute query request.
 *
 * @param[in] cbarg - callback argument
 */
static void
bfa_phy_query_send(void *cbarg)
{
	bfa_phy_t *phy = cbarg;
	bfi_phy_query_req_t *msg = (bfi_phy_query_req_t *) phy->mb.msg;

	msg->instance = phy->instance;
	bfi_h2i_set(msg->mh, BFI_MC_PHY, BFI_PHY_H2I_QUERY_REQ,
		    bfa_ioc_portid(phy->ioc));
	bfa_dma_alen_set(phy->ioc, &msg->alen, sizeof(bfa_phy_attr_t),
				phy->dbuf_kva, phy->dbuf_pa);
	bfa_ioc_mbox_queue(phy->ioc, &phy->mb, NULL, NULL);
}

/**
 * @brief Send phy write request.
 *
 * @param[in] cbarg - callback argument
 */
static void
bfa_phy_write_send(void *cbarg)
{
	bfa_phy_t *phy = cbarg;
	bfi_phy_write_req_t *msg = (bfi_phy_write_req_t *) phy->mb.msg;
	uint32_t len;
	uint16_t *buf;

	msg->instance = phy->instance;
	msg->offset = bfa_os_htonl(phy->addr_off + phy->offset);
	len = (phy->residue < BFA_PHY_DMA_BUF_SZ) ?
				phy->residue : BFA_PHY_DMA_BUF_SZ;
	msg->length = bfa_os_htonl(len);

	/*
	 * indicate if it's the last msg of the whole write operation
	 */
	msg->last = (len == phy->residue) ? 1 : 0;

	bfi_h2i_set(msg->mh, BFI_MC_PHY, BFI_PHY_H2I_WRITE_REQ,
		    bfa_ioc_portid(phy->ioc));
	bfa_alen_set(&msg->alen, len, phy->dbuf_pa);

	buf = (uint16_t *) (phy->ubuf + phy->offset);
	bfa_phy_hton16((uint16_t *)phy->dbuf_kva, buf, len);
	bfa_ioc_mbox_queue(phy->ioc, &phy->mb, NULL, NULL);

	phy->residue -= len;
	phy->offset += len;
}

/**
 * @brief Send phy read request.
 *
 * @param[in] cbarg - callback argument
 */
static void
bfa_phy_read_send(void *cbarg)
{
	bfa_phy_t *phy = cbarg;
	bfi_phy_read_req_t *msg = (bfi_phy_read_req_t *) phy->mb.msg;
	uint32_t len;

	msg->instance = phy->instance;
	msg->offset = bfa_os_htonl(phy->addr_off + phy->offset);
	len = (phy->residue < BFA_PHY_DMA_BUF_SZ) ?
				phy->residue : BFA_PHY_DMA_BUF_SZ;
	msg->length = bfa_os_htonl(len);
	bfi_h2i_set(msg->mh, BFI_MC_PHY, BFI_PHY_H2I_READ_REQ,
		    bfa_ioc_portid(phy->ioc));
	bfa_alen_set(&msg->alen, len, phy->dbuf_pa);
	bfa_ioc_mbox_queue(phy->ioc, &phy->mb, NULL, NULL);
}

/**
 * @brief Send phy stats request.
 *
 * @param[in] cbarg - callback argument
 */
static void
bfa_phy_stats_send(void *cbarg)
{
	bfa_phy_t *phy = cbarg;
	bfi_phy_stats_req_t *msg = (bfi_phy_stats_req_t *) phy->mb.msg;

	msg->instance = phy->instance;
	bfi_h2i_set(msg->mh, BFI_MC_PHY, BFI_PHY_H2I_STATS_REQ,
		    bfa_ioc_portid(phy->ioc));
	bfa_alen_set(&msg->alen, sizeof(bfa_phy_stats_t), phy->dbuf_pa);
	bfa_ioc_mbox_queue(phy->ioc, &phy->mb, NULL, NULL);
}

/**
 * @brief Flash memory info API.
 *
 * @param[in] mincfg - minimal cfg variable
 */
uint32_t
bfa_phy_meminfo(bfa_boolean_t mincfg)
{
	/* min driver doesn't need phy */
	if (mincfg)
		return 0;

	return (BFA_ROUNDUP(BFA_PHY_DMA_BUF_SZ, BFA_DMA_ALIGN_SZ));
}

/**
 * @brief Flash attach API.
 *
 * @param[in] phy - phy structure
 * @param[in] ioc  - ioc structure
 * @param[in] dev  - device structure
 * @param[in] trcmod - trace module
 * @param[in] logmod - log module
 */
void
bfa_phy_attach(struct bfa_phy_s *phy, struct bfa_ioc_s *ioc, void *dev,
	bfa_trc_mod_t *trcmod, bfa_boolean_t mincfg)
{
	phy->ioc = ioc;
	phy->trcmod = trcmod;
	phy->cbfn = NULL;
	phy->cbarg = NULL;
	phy->op_busy = 0;

	bfa_ioc_mbox_regisr(phy->ioc, BFI_MC_PHY, bfa_phy_intr, phy);
	bfa_q_qe_init(&phy->ioc_notify);
	bfa_ioc_notify_init(&phy->ioc_notify, bfa_phy_notify, phy);
	bfa_ioc_notify_register(phy->ioc, &phy->ioc_notify);

	/* min driver doesn't need phy */
	if (mincfg) {
		phy->dbuf_kva = NULL;
		phy->dbuf_pa = 0;
	}
}

/**
 * @brief Claim memory for phy
 *
 * @param[in] phy - phy structure
 * @param[in] dm_kva - pointer to virtual memory address
 * @param[in] dm_pa - physical memory address
 * @param[in] mincfg - minimal cfg variable
 */
void
bfa_phy_memclaim(struct bfa_phy_s *phy, uint8_t *dm_kva, uint64_t dm_pa,
	bfa_boolean_t mincfg)
{
	if (mincfg)
		return;

	phy->dbuf_kva = dm_kva;
	phy->dbuf_pa = dm_pa;
	bfa_os_memset(phy->dbuf_kva, 0, BFA_PHY_DMA_BUF_SZ);
	dm_kva += BFA_ROUNDUP(BFA_PHY_DMA_BUF_SZ, BFA_DMA_ALIGN_SZ);
	dm_pa += BFA_ROUNDUP(BFA_PHY_DMA_BUF_SZ, BFA_DMA_ALIGN_SZ);
}

/*
 *****************************************************************************
 *
 * Externalized functions
 *
 *****************************************************************************
 */


/**
 * @ig hal_phy_api
 * @{
 */

bfa_boolean_t
bfa_phy_busy(struct bfa_ioc_s *ioc)
{
	bfa_os_addr_t rb;

	rb = bfa_ioc_bar0(ioc);

	return (bfa_reg_read(rb + BFA_PHY_LOCK_STATUS));
}

/**
 * @brief Get phy attribute.
 *
 * @param[in] phy - phy structure
 * @param[in] attr - phy attribute structure
 * @param[in] cbfn - callback function
 * @param[in] cbarg - callback argument
 *
 * Return status.
 */
bfa_status_t
bfa_phy_get_attr(struct bfa_phy_s *phy, uint8_t instance,
	bfa_phy_attr_t *attr,
	bfa_cb_phy_t cbfn, void *cbarg)
{
	bfa_trc(phy, BFI_PHY_H2I_QUERY_REQ);
	bfa_trc(phy, instance);

	if (!bfa_phy_present(phy))
		return BFA_STATUS_PHY_NOT_PRESENT;

	if (!bfa_ioc_is_operational(phy->ioc))
		return BFA_STATUS_IOC_NON_OP;

	if (phy->op_busy || bfa_phy_busy(phy->ioc)) {
		bfa_trc(phy, phy->op_busy);
		return (BFA_STATUS_DEVBUSY);
	}

	phy->op_busy = 1;

	phy->cbfn = cbfn;
	phy->cbarg = cbarg;
	phy->instance = instance;
	phy->ubuf = (uint8_t *) attr;
	bfa_phy_query_send(phy);

	return (BFA_STATUS_OK);
}

/**
 * @brief Get phy stats.
 *
 * @param[in] phy - phy structure
 * @param[in] instance - phy image instance
 * @param[in] stats - pointer to phy stats
 * @param[in] cbfn - callback function
 * @param[in] cbarg - callback argument
 *
 * Return status.
 */
bfa_status_t
bfa_phy_get_stats(struct bfa_phy_s *phy, uint8_t instance,
	bfa_phy_stats_t *stats,
	bfa_cb_phy_t cbfn, void *cbarg)
{
	bfa_trc(phy, BFI_PHY_H2I_STATS_REQ);
	bfa_trc(phy, instance);

	if (!bfa_phy_present(phy))
		return BFA_STATUS_PHY_NOT_PRESENT;

	if (!bfa_ioc_is_operational(phy->ioc))
		return BFA_STATUS_IOC_NON_OP;

	if (phy->op_busy || bfa_phy_busy(phy->ioc)) {
		bfa_trc(phy, phy->op_busy);
		return (BFA_STATUS_DEVBUSY);
	}

	phy->op_busy = 1;

	phy->cbfn = cbfn;
	phy->cbarg = cbarg;
	phy->instance = instance;
	phy->ubuf = (uint8_t *) stats;
	bfa_phy_stats_send(phy);

	return (BFA_STATUS_OK);
}

/**
 * @brief Update phy image.
 *
 * @param[in] phy - phy structure
 * @param[in] instance - phy image instance
 * @param[in] buf - update data buffer
 * @param[in] len - data buffer length
 * @param[in] offset - offset relative to starting address
 * @param[in] cbfn - callback function
 * @param[in] cbarg - callback argument
 *
 * Return status.
 */
bfa_status_t
bfa_phy_update(struct bfa_phy_s *phy, uint8_t instance,
	void *buf, uint32_t len, uint32_t offset,
	bfa_cb_phy_t cbfn, void *cbarg)
{
	bfa_trc(phy, BFI_PHY_H2I_WRITE_REQ);
	bfa_trc(phy, instance);
	bfa_trc(phy, len);
	bfa_trc(phy, offset);

	if (!bfa_phy_present(phy))
		return BFA_STATUS_PHY_NOT_PRESENT;

	if (!bfa_ioc_is_operational(phy->ioc))
		return BFA_STATUS_IOC_NON_OP;

	/**
	 * 'len' must be in word (4-byte) boundary
	 */
	if (!len || (len & 0x03))
		return (BFA_STATUS_FAILED);

	if (phy->op_busy || bfa_phy_busy(phy->ioc)) {
		bfa_trc(phy, phy->op_busy);
		return (BFA_STATUS_DEVBUSY);
	}

	phy->op_busy = 1;

	phy->cbfn = cbfn;
	phy->cbarg = cbarg;
	phy->instance = instance;
	phy->residue = len;
	phy->offset = 0;
	phy->addr_off = offset;
	phy->ubuf = buf;

	bfa_phy_write_send(phy);

	return (BFA_STATUS_OK);
}

/**
 * @brief Read phy image.
 *
 * @param[in] phy - phy structure
 * @param[in] instance - phy image instance
 * @param[in] buf - read data buffer
 * @param[in] len - data buffer length
 * @param[in] offset - offset relative to starting address
 * @param[in] cbfn - callback function
 * @param[in] cbarg - callback argument
 *
 * Return status.
 */
bfa_status_t
bfa_phy_read(struct bfa_phy_s *phy, uint8_t instance,
	void *buf, uint32_t len, uint32_t offset,
	bfa_cb_phy_t cbfn, void *cbarg)
{
	bfa_trc(phy, BFI_PHY_H2I_READ_REQ);
	bfa_trc(phy, instance);
	bfa_trc(phy, len);
	bfa_trc(phy, offset);

	if (!bfa_phy_present(phy))
		return BFA_STATUS_PHY_NOT_PRESENT;

	if (!bfa_ioc_is_operational(phy->ioc))
		return BFA_STATUS_IOC_NON_OP;

	/**
	 * 'len' must be in word (4-byte) boundary
	 */
	if (!len || (len & 0x03))
		return (BFA_STATUS_FAILED);

	if (phy->op_busy || bfa_phy_busy(phy->ioc)) {
		bfa_trc(phy, phy->op_busy);
		return (BFA_STATUS_DEVBUSY);
	}

	phy->op_busy = 1;

	phy->cbfn = cbfn;
	phy->cbarg = cbarg;
	phy->instance = instance;
	phy->residue = len;
	phy->offset = 0;
	phy->addr_off = offset;
	phy->ubuf = buf;
	bfa_phy_read_send(phy);

	return (BFA_STATUS_OK);
}

/**
 * @brief Process phy response messages upon receiving interrupts.
 *
 * @param[in] phyarg - phy structure
 * @param[in] msg - message structure
 */
void
bfa_phy_intr(void *phyarg, struct bfi_mbmsg_s *msg)
{
	struct bfa_phy_s *phy = phyarg;
	uint32_t status;

	union {
		bfi_phy_query_rsp_t *query;
		bfi_phy_stats_rsp_t *stats;
		bfi_phy_write_rsp_t *write;
		bfi_phy_read_rsp_t *read;
		struct bfi_mbmsg_s   *msg;
	} m;

	m.msg = msg;
	bfa_trc(phy, msg->mh.msg_id);

	if (!phy->op_busy) {
		/*
		 * receiving response after ioc failure
		 */
		bfa_trc(phy, 0x9999);
		return;
	}

	switch (msg->mh.msg_id) {
	case BFI_PHY_I2H_QUERY_RSP:
		status = bfa_os_ntohl(m.query->status);
		bfa_trc(phy, status);

		if (status == BFA_STATUS_OK) {
			bfa_phy_attr_t *attr;

			attr = (bfa_phy_attr_t *) phy->ubuf;
			bfa_phy_ntoh32((uint32_t *)attr,
				(uint32_t *)phy->dbuf_kva,
				sizeof(bfa_phy_attr_t));
			bfa_trc(phy, attr->status);
			bfa_trc(phy, attr->length);
		}

		phy->status = status;
		bfa_phy_cb(phy);
		break;
	case BFI_PHY_I2H_STATS_RSP:
		status = bfa_os_ntohl(m.stats->status);
		bfa_trc(phy, status);

		if (status == BFA_STATUS_OK) {
			bfa_phy_stats_t *stats;

			stats = (bfa_phy_stats_t *) phy->ubuf;
			bfa_phy_ntoh32((uint32_t *)stats,
				(uint32_t *)phy->dbuf_kva,
				sizeof(bfa_phy_stats_t));
			bfa_trc(phy, stats->status);
		}

		phy->status = status;
		bfa_phy_cb(phy);
		break;
	case BFI_PHY_I2H_WRITE_RSP:
		status = bfa_os_ntohl(m.write->status);
		bfa_trc(phy, status);

		if (status != BFA_STATUS_OK || phy->residue == 0) {
			phy->status = status;
			bfa_phy_cb(phy);
		} else {
			bfa_trc(phy, phy->offset);
			bfa_phy_write_send(phy);
		}
		break;
	case BFI_PHY_I2H_READ_RSP:
		status = bfa_os_ntohl(m.read->status);
		bfa_trc(phy, status);

		if (status != BFA_STATUS_OK) {
			phy->status = status;
			bfa_phy_cb(phy);
		} else {
			uint32_t len = bfa_os_ntohl(m.read->length);
			uint16_t *buf = (uint16_t *)(phy->ubuf + phy->offset);

			bfa_trc(phy, phy->offset);
			bfa_trc(phy, len);

			bfa_phy_ntoh16(buf, (uint16_t *)phy->dbuf_kva, len);
			phy->residue -= len;
			phy->offset += len;

			if (phy->residue == 0) {
				phy->status = status;
				bfa_phy_cb(phy);
			} else {
				bfa_phy_read_send(phy);
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
