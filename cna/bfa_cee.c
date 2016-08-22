/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

/**
 * @file bfa_cee.c CEE module source file.
 */
#include <cna/bfa_port.h>
#include <bfi/bfi_cna.h>
#include <cna/bfa_ioc.h>

BFA_TRC_FILE(CNA, CEE);

static void bfa_cee_format_lldp_cfg(struct bfa_cee_lldp_cfg_s *lldp_cfg);
static void bfa_cee_format_cee_cfg(void *buffer);

static void
bfa_cee_format_cee_cfg(void *buffer)
{
	struct bfa_cee_attr_s *cee_cfg = buffer;
	bfa_cee_format_lldp_cfg(&cee_cfg->lldp_remote);
}

static void
bfa_cee_stats_swap(struct bfa_cee_stats_s *stats)
{
	uint32_t *buffer = (uint32_t *)stats;
	int i;

	for (i = 0; i < (sizeof(struct bfa_cee_stats_s) / sizeof(uint32_t));
		i++) {
		buffer[i] = bfa_os_ntohl(buffer[i]);
	}
}

static void
bfa_cee_format_lldp_cfg(struct bfa_cee_lldp_cfg_s *lldp_cfg)
{
	lldp_cfg->time_to_live =
			bfa_os_ntohs(lldp_cfg->time_to_live);
	lldp_cfg->enabled_system_cap =
			bfa_os_ntohs(lldp_cfg->enabled_system_cap);
}


/**
 * bfa_cee_attr_meminfo()
 *
 * @brief Returns the size of the DMA memory needed by CEE attributes
 *
 * @param[in] void
 *
 * @return Size of DMA region
 */
static uint32_t
bfa_cee_attr_meminfo(void)
{
	return BFA_ROUNDUP(sizeof(struct bfa_cee_attr_s), BFA_DMA_ALIGN_SZ);
}
/**
 * bfa_cee_stats_meminfo()
 *
 * @brief Returns the size of the DMA memory needed by CEE stats
 *
 * @param[in] void
 *
 * @return Size of DMA region
 */
static uint32_t
bfa_cee_stats_meminfo(void)
{
	return BFA_ROUNDUP(sizeof(struct bfa_cee_stats_s), BFA_DMA_ALIGN_SZ);
}

/**
 * bfa_cee_cfg_meminfo()
 *
 * @brief Returns the size of the DMA memory needed by CEE dcbx configuration
 *
 * @param[in] void
 *
 * @return Size of DMA region
 */
static uint32_t
bfa_cee_cfg_meminfo(void)
{
	return BFA_ROUNDUP(sizeof(struct bfa_cee_dcbx_cfg_s), BFA_DMA_ALIGN_SZ);
}

/**
 * bfa_cee_get_attr_isr()
 *
 * @brief CEE ISR for get-attributes responses from f/w
 *
 * @param[in] cee - Pointer to the CEE module
 *            status - Return status from the f/w
 *
 * @return void
 */
static void
bfa_cee_get_attr_isr(struct bfa_cee_s *cee, bfa_status_t status)
{
	cee->get_attr_status = status;
	bfa_trc(cee, 0);
	if (status == BFA_STATUS_OK) {
		bfa_trc(cee, 0);
		memcpy(cee->attr, cee->attr_dma.kva,
		    sizeof(struct bfa_cee_attr_s));
		bfa_cee_format_cee_cfg(cee->attr);
	}
	cee->get_attr_pending = BFA_FALSE;
	if (cee->cbfn.get_attr_cbfn) {
		bfa_trc(cee, 0);
		cee->cbfn.get_attr_cbfn(cee->cbfn.get_attr_cbarg, status);
	}
	bfa_trc(cee, 0);
}

/**
 * bfa_cee_get_attr_isr()
 *
 * @brief CEE ISR for get-stats responses from f/w
 *
 * @param[in] cee - Pointer to the CEE module
 *            status - Return status from the f/w
 *
 * @return void
 */
static void
bfa_cee_get_stats_isr(struct bfa_cee_s *cee, bfa_status_t status)
{
	cee->get_stats_status = status;
	bfa_trc(cee, 0);
	if (status == BFA_STATUS_OK) {
		bfa_trc(cee, 0);
		memcpy(cee->stats, cee->stats_dma.kva, sizeof(bfa_cee_stats_t));
		bfa_cee_stats_swap(cee->stats);
	}
	cee->get_stats_pending = BFA_FALSE;
	bfa_trc(cee, 0);
	if (cee->cbfn.get_stats_cbfn) {
		bfa_trc(cee, 0);
		cee->cbfn.get_stats_cbfn(cee->cbfn.get_stats_cbarg, status);
	}
	bfa_trc(cee, 0);
}

/**
 * bfa_cee_get_attr_isr()
 *
 * @brief CEE ISR for reset-stats responses from f/w
 *
 * @param[in] cee - Pointer to the CEE module
 *            status - Return status from the f/w
 *
 * @return void
 */
static void
bfa_cee_reset_stats_isr(struct bfa_cee_s *cee, bfa_status_t status)
{
	cee->reset_stats_status = status;
	cee->reset_stats_pending = BFA_FALSE;
	if (cee->cbfn.reset_stats_cbfn) {
		cee->cbfn.reset_stats_cbfn(cee->cbfn.reset_stats_cbarg, status);
	}
}

/**
 * bfa_cee_set_cfg_isr()
 *
 * @brief CEE ISR for set_cfg responses from f/w
 *
 * @param[in] cee - Pointer to the CEE module
 *            status - Return status from the f/w
 *
 * @return void
 */
static void
bfa_cee_set_cfg_isr(struct bfa_cee_s *cee, bfa_status_t status)
{
	// For now, do not care about pending stuff. Just invoke the callback
	// cee->reset_stats_status = status;
	// cee->reset_stats_pending = BFA_FALSE;
	if (cee->cbfn.set_cfg_cbfn) {
		cee->cbfn.set_cfg_cbfn(cee->cbfn.set_cfg_cbarg, status);
	}
}

/**
 * bfa_cee_meminfo()
 *
 * @brief Returns the size of the DMA memory needed by CEE module
 *
 * @param[in] void
 *
 * @return Size of DMA region
 */
uint32_t
bfa_cee_meminfo(void)
{
	return bfa_cee_attr_meminfo() +
		bfa_cee_stats_meminfo() +
		bfa_cee_cfg_meminfo();
}

/**
 * bfa_cee_mem_claim()
 *
 * @brief Initialized CEE DMA Memory
 *
 * @param[in] cee CEE module pointer
 *	      dma_kva Kernel Virtual Address of CEE DMA Memory
 *	      dma_pa  Physical Address of CEE DMA Memory
 *
 * @return void
 */
void
bfa_cee_mem_claim(struct bfa_cee_s *cee, uint8_t *dma_kva, uint64_t dma_pa)
{
	cee->attr_dma.kva = dma_kva;
	cee->attr_dma.pa = dma_pa;
	cee->stats_dma.kva = dma_kva + bfa_cee_attr_meminfo();
	cee->stats_dma.pa = dma_pa + bfa_cee_attr_meminfo();
	cee->cfg_dma.kva = dma_kva + bfa_cee_attr_meminfo() +
					bfa_cee_stats_meminfo();
	cee->cfg_dma.pa = dma_pa + bfa_cee_attr_meminfo() +
					bfa_cee_stats_meminfo();
	cee->attr = (bfa_cee_attr_t *) dma_kva;
	cee->stats = (bfa_cee_stats_t *) (dma_kva + bfa_cee_attr_meminfo());
	cee->cfg = (bfa_cee_dcbx_cfg_t *) (dma_kva + bfa_cee_attr_meminfo() +
					bfa_cee_stats_meminfo());
}

/**
 * bfa_cee_get_attr()
 *
 * @brief
 *   Send the request to the f/w to fetch CEE attributes.
 *
 * @param[in] Pointer to the CEE module data structure.
 *
 * @return Status
 */

bfa_status_t
bfa_cee_get_attr(struct bfa_cee_s *cee, bfa_cee_attr_t *attr,
		     bfa_cee_get_attr_cbfn_t cbfn, void *cbarg)
{
	struct bfi_cee_get_req_s *cmd;

	bfa_assert((cee != NULL) && (cee->ioc != NULL));
	bfa_trc(cee, 0);
	if (!bfa_ioc_is_operational(cee->ioc)) {
		bfa_trc(cee, 0);
		return BFA_STATUS_IOC_FAILURE;
	}
	if (cee->get_attr_pending == BFA_TRUE) {
		bfa_trc(cee, 0);
		return 	BFA_STATUS_DEVBUSY;
	}
	cee->get_attr_pending = BFA_TRUE;
	cmd = (struct bfi_cee_get_req_s *) cee->get_cfg_mb.msg;
	cee->attr = attr;
	cee->cbfn.get_attr_cbfn = cbfn;
	cee->cbfn.get_attr_cbarg = cbarg;
	bfi_h2i_set(cmd->mh, BFI_MC_CEE, BFI_CEE_H2I_GET_CFG_REQ,
	    bfa_ioc_portid(cee->ioc));
	bfa_dma_be_addr_set(cmd->dma_addr, cee->attr_dma.pa);
	bfa_ioc_mbox_queue(cee->ioc, &cee->get_cfg_mb, NULL, NULL);
	bfa_trc(cee, 0);

	return BFA_STATUS_OK;
}

/**
 * bfa_cee_get_attr()
 *
 * @brief
 *   Send the request to the f/w to fetch CEE attributes.
 *
 * @param[in] Pointer to the CEE module data structure.
 *
 * @return Status
 */

bfa_status_t
bfa_cee_set_cfg(struct bfa_cee_s *cee, bfa_cee_dcbx_cfg_t *cfg,
		     bfa_cee_set_cfg_cbfn_t cbfn, void *cbarg)
{
	struct bfi_cee_set_req_s *cmd;

	bfa_assert((cee != NULL) && (cee->ioc != NULL));
	bfa_trc(cee, 0);
	if (!bfa_ioc_is_operational(cee->ioc)) {
		bfa_trc(cee, 0);
		return BFA_STATUS_IOC_FAILURE;
	}

	// FIXME: Should we check if another set_cfg command pending?

	cmd = (struct bfi_cee_set_req_s *) cee->set_cfg_mb.msg;
	cee->cfg = cfg;
	cee->cbfn.set_cfg_cbfn = cbfn;
	// FIXME: We really donot need to provide the args.
	// But we need to copy the contents of the cfg to the cfg_dma.kva?
	cee->cbfn.set_cfg_cbarg = cbarg;
	bfa_os_memcpy(cee->cfg_dma.kva, cfg, sizeof(bfa_cee_dcbx_cfg_t));
	bfi_h2i_set(cmd->mh, BFI_MC_CEE, BFI_CEE_H2I_SET_CFG_REQ,
	    bfa_ioc_portid(cee->ioc));
	bfa_dma_alen_set(cee->ioc, &cmd->alen, sizeof(bfa_cee_dcbx_cfg_t),
		cee->cfg_dma.kva, cee->cfg_dma.pa);
	bfa_ioc_mbox_queue(cee->ioc, &cee->set_cfg_mb, NULL, NULL);
	bfa_trc(cee, 0);

	return BFA_STATUS_OK;
}

/**
 * bfa_cee_get_stats()
 *
 * @brief
 *   Send the request to the f/w to fetch CEE statistics.
 *
 * @param[in] Pointer to the CEE module data structure.
 *
 * @return Status
 */

bfa_status_t
bfa_cee_get_stats(struct bfa_cee_s *cee, bfa_cee_stats_t *stats,
		      bfa_cee_get_stats_cbfn_t cbfn, void *cbarg)
{
	struct bfi_cee_get_req_s *cmd;

	bfa_assert((cee != NULL) && (cee->ioc != NULL));

	if (!bfa_ioc_is_operational(cee->ioc)) {
		bfa_trc(cee, 0);
		return BFA_STATUS_IOC_FAILURE;
	}
	if (cee->get_stats_pending == BFA_TRUE) {
		bfa_trc(cee, 0);
		return 	BFA_STATUS_DEVBUSY;
	}
	cee->get_stats_pending = BFA_TRUE;
	cmd = (struct bfi_cee_get_req_s *) cee->get_stats_mb.msg;
	cee->stats = stats;
	cee->cbfn.get_stats_cbfn = cbfn;
	cee->cbfn.get_stats_cbarg = cbarg;
	bfi_h2i_set(cmd->mh, BFI_MC_CEE, BFI_CEE_H2I_GET_STATS_REQ,
	    bfa_ioc_portid(cee->ioc));
	bfa_dma_be_addr_set(cmd->dma_addr, cee->stats_dma.pa);
	bfa_ioc_mbox_queue(cee->ioc, &cee->get_stats_mb, NULL, NULL);
	bfa_trc(cee, 0);

	return BFA_STATUS_OK;
}

/**
 * bfa_cee_reset_stats()
 *
 * @brief Clears CEE Stats in the f/w.
 *
 * @param[in] Pointer to the CEE module data structure.
 *
 * @return Status
 */

bfa_status_t
bfa_cee_reset_stats(struct bfa_cee_s *cee, bfa_cee_reset_stats_cbfn_t cbfn,
			void *cbarg)
{
	struct bfi_cee_reset_stats_s *cmd;

	bfa_assert((cee != NULL) && (cee->ioc != NULL));
	if (!bfa_ioc_is_operational(cee->ioc)) {
		bfa_trc(cee, 0);
		return BFA_STATUS_IOC_FAILURE;
	}
	if (cee->reset_stats_pending == BFA_TRUE) {
		bfa_trc(cee, 0);
		return 	BFA_STATUS_DEVBUSY;
	}
	cee->reset_stats_pending = BFA_TRUE;
	cmd = (struct bfi_cee_reset_stats_s *) cee->reset_stats_mb.msg;
	cee->cbfn.reset_stats_cbfn = cbfn;
	cee->cbfn.reset_stats_cbarg = cbarg;
	bfi_h2i_set(cmd->mh, BFI_MC_CEE, BFI_CEE_H2I_RESET_STATS,
	    bfa_ioc_portid(cee->ioc));
	bfa_ioc_mbox_queue(cee->ioc, &cee->reset_stats_mb, NULL, NULL);
	bfa_trc(cee, 0);
	return BFA_STATUS_OK;
}


/**
 * bfa_cee_isrs()
 *
 * @brief Handles Mail-box interrupts for CEE module.
 *
 * @param[in] Pointer to the CEE module data structure.
 *
 * @return void
 */

void
bfa_cee_isr(void *cbarg, struct bfi_mbmsg_s *m)
{
	union bfi_cee_i2h_msg_u *msg;
	struct bfi_cee_get_rsp_s *get_rsp;
	struct bfa_cee_s *cee = (struct bfa_cee_s *) cbarg;
	msg = (union bfi_cee_i2h_msg_u *) m;
	get_rsp = (struct bfi_cee_get_rsp_s *) m;
	bfa_trc(cee, msg->mh.msg_id);
	switch (msg->mh.msg_id) {
	case BFI_CEE_I2H_GET_CFG_RSP:
		bfa_trc(cee, get_rsp->cmd_status);
		bfa_cee_get_attr_isr(cee, get_rsp->cmd_status);
		break;
	case BFI_CEE_I2H_GET_STATS_RSP:
		bfa_cee_get_stats_isr(cee, get_rsp->cmd_status);
		break;
	case BFI_CEE_I2H_RESET_STATS_RSP:
		bfa_cee_reset_stats_isr(cee, get_rsp->cmd_status);
		break;
	case BFI_CEE_I2H_SET_CFG_RSP:
		bfa_cee_set_cfg_isr(cee, get_rsp->cmd_status);
		break;
	default:
		bfa_assert(0);
	}
}

/**
 * bfa_cee_notify()
 *
 * @brief CEE module IOC event handler.
 *
 * @param[in] Pointer to the CEE module data structure.
 * @param[in] IOC event type
 *
 * @return void
 */

void
bfa_cee_notify(void *arg, enum bfa_ioc_event_e event)
{
	struct bfa_cee_s *cee;
	cee = (struct bfa_cee_s *) arg;

	bfa_trc(cee, event);

	switch (event) {
	case BFA_IOC_E_DISABLED:
	case BFA_IOC_E_FAILED:
		if (cee->get_attr_pending == BFA_TRUE) {
			cee->get_attr_status = BFA_STATUS_FAILED;
			cee->get_attr_pending  = BFA_FALSE;
			if (cee->cbfn.get_attr_cbfn) {
				cee->cbfn.get_attr_cbfn(
					cee->cbfn.get_attr_cbarg,
					BFA_STATUS_FAILED);
			}
		}
		if (cee->get_stats_pending == BFA_TRUE) {
			cee->get_stats_status = BFA_STATUS_FAILED;
			cee->get_stats_pending  = BFA_FALSE;
			if (cee->cbfn.get_stats_cbfn) {
				cee->cbfn.get_stats_cbfn(
					cee->cbfn.get_stats_cbarg,
					BFA_STATUS_FAILED);
			}
		}
		if (cee->reset_stats_pending == BFA_TRUE) {
			cee->reset_stats_status = BFA_STATUS_FAILED;
			cee->reset_stats_pending  = BFA_FALSE;
			if (cee->cbfn.reset_stats_cbfn) {
				cee->cbfn.reset_stats_cbfn(
					cee->cbfn.reset_stats_cbarg,
					BFA_STATUS_FAILED);
			}
		}
		break;

	default:
		break;
	}
}

/**
 * bfa_cee_log_trc_set()
 *
 * @brief CEE module-log trc set API
 *
 * @param[in] trcmod -
 *            logmod -
 *
 * @return void
 */
void
bfa_cee_log_trc_set(struct bfa_cee_s *cee, bfa_trc_mod_t *trcmod,
	bfa_log_mod_t *logmod)
{
	cee->trcmod = trcmod;
	cee->logmod = logmod;
}

/**
 * bfa_cee_attach()
 *
 * @brief CEE module-attach API
 *
 * @param[in] cee - Pointer to the CEE module data structure
 *            ioc - Pointer to the ioc module data structure
 *            dev - Pointer to the device driver module data structure
 *                  The device driver specific mbox ISR functions have
 *                  this pointer as one of the parameters.
 *
 * @return void
 */
void
bfa_cee_attach(struct bfa_cee_s *cee, struct bfa_ioc_s *ioc,
		void *dev)
{
	bfa_assert(cee != NULL);
	cee->dev = dev;
	cee->ioc = ioc;

	bfa_ioc_mbox_regisr(cee->ioc, BFI_MC_CEE, bfa_cee_isr, cee);
	bfa_q_qe_init(&cee->ioc_notify);
	bfa_ioc_notify_init(&cee->ioc_notify, bfa_cee_notify, cee);
	bfa_ioc_notify_register(cee->ioc, &cee->ioc_notify);
	bfa_trc(cee, 0);
}

/**
 * bfa_cee_detach()
 *
 * @brief CEE module-detach API
 *
 * @param[in] cee - Pointer to the CEE module data structure
 *
 * @return void
 */
void
bfa_cee_detach(struct bfa_cee_s *cee)
{
}
