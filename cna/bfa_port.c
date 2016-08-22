/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

/**
 * @file bfa_port.c Port module source file.
 */
#include <defs/bfa_defs_svc.h>
#include <cs/bfa_log.h>
#include <cna/bfa_port.h>
#include <bfi/bfi_cna.h>
#include <cna/bfa_phy.h>
#include <cna/bfa_ioc.h>


BFA_TRC_FILE(CNA, PORT);

static void
bfa_port_stats_swap(struct bfa_port_s *port, union bfa_port_stats_u *stats)
{
	uint32_t    *dip = (uint32_t *) stats;
	uint32_t    t0, t1;
	int	    i;

	for (i = 0; i < sizeof(union bfa_port_stats_u)/sizeof(uint32_t);
		i += 2) {
		t0 = dip[i];
		t1 = dip[i + 1];
#ifdef __BIGENDIAN
		dip[i] = bfa_os_ntohl(t0);
		dip[i + 1] = bfa_os_ntohl(t1);
#else
		dip[i] = bfa_os_ntohl(t1);
		dip[i + 1] = bfa_os_ntohl(t0);
#endif
	}

	/** @todo 
	 * QoS stats r also swapped as 64bit; that structure also
	 * has to use 64 bit counters
	 */
}

/**
 * bfa_port_enable_isr()
 *
 * @brief Port enable response from f/w ISR
 *
 * @param[in] port - Pointer to the port module
 *            status - Return status from the f/w
 *
 * @return void
 */
static void
bfa_port_enable_isr(struct bfa_port_s *port, bfa_status_t status)
{
	bfa_trc(port, status);
	port->endis_pending = BFA_FALSE;
	if (port->endis_cbfn) {
		port->endis_cbfn(port->endis_cbarg, status);
	}
}

/**
 * bfa_port_disable_isr()
 *
 * @brief Port disable response from f/w ISR
 *
 * @param[in] port - Pointer to the port module
 *            status - Return status from the f/w
 *
 * @return void
 */
static void
bfa_port_disable_isr(struct bfa_port_s *port, bfa_status_t status)
{
	bfa_trc(port, status);
	port->endis_pending = BFA_FALSE;
	if (port->endis_cbfn) {
		port->endis_cbfn(port->endis_cbarg, status);
	}
}

/**
 * bfa_port_get_stats_isr()
 *
 * @brief Port ISR for get-stats responses from f/w
 *
 * @param[in] port - Pointer to the Port module
 *            status - Return status from the f/w
 *
 * @return void
 */
static void
bfa_port_get_stats_isr(struct bfa_port_s *port, bfa_status_t status)
{
	port->stats_status = status;
	port->stats_busy = BFA_FALSE;

	if (status == BFA_STATUS_OK) {
		bfa_timeval_t tv;

		memcpy(port->stats, port->stats_dma.kva,
		       sizeof(union bfa_port_stats_u));
		bfa_port_stats_swap(port, port->stats);

		bfa_os_gettimeofday(&tv);
		port->stats->fc.secs_reset = tv.tv_sec - port->stats_reset_time;
	}

	if (port->stats_cbfn) {
		port->stats_cbfn(port->stats_cbarg, status);
		port->stats_cbfn = NULL;
	}
}

/**
 * bfa_port_clear_stats_isr()
 *
 * @brief Port ISR for clear-stats responses from f/w
 *
 * @param[in] port - Pointer to the Port module
 *            status - Return status from the f/w
 *
 * @return void
 */
static void
bfa_port_clear_stats_isr(struct bfa_port_s *port, bfa_status_t status)
{
	bfa_timeval_t tv;

	port->stats_status = status;
	port->stats_busy   = BFA_FALSE;

	/**
	* re-initialize time stamp for stats reset
	*/
	bfa_os_gettimeofday(&tv);
	port->stats_reset_time = tv.tv_sec;

	if (port->stats_cbfn) {
		port->stats_cbfn(port->stats_cbarg, status);
		port->stats_cbfn = NULL;
	}
}

/**
 * bfa_port_isr()
 *
 * @brief Handles Mail-box interrupts for Port module.
 *
 * @param[in] Pointer to the Port module data structure.
 *
 * @return void
 */
static void
bfa_port_isr(void *cbarg, struct bfi_mbmsg_s *m)
{
	struct bfa_port_s *port = (struct bfa_port_s *) cbarg;
	union bfi_port_i2h_msg_u *i2hmsg;

	i2hmsg = (union bfi_port_i2h_msg_u *) m;
	bfa_trc(port, m->mh.msg_id);

	switch (m->mh.msg_id) {
	case BFI_PORT_I2H_ENABLE_RSP :
		if (port->endis_pending == BFA_FALSE)
			break;
		bfa_port_enable_isr(port, i2hmsg->enable_rsp.status);
		break;

	case BFI_PORT_I2H_DISABLE_RSP :
		if (port->endis_pending == BFA_FALSE)
			break;
		bfa_port_disable_isr(port, i2hmsg->disable_rsp.status);
		break;

	case BFI_PORT_I2H_GET_STATS_RSP:
		/* Stats busy flag is still set? (may be cmd timed out) */
		if (port->stats_busy == BFA_FALSE)
			break;
		bfa_port_get_stats_isr(port, i2hmsg->getstats_rsp.status);
		break;

	case BFI_PORT_I2H_CLEAR_STATS_RSP:
		if (port->stats_busy == BFA_FALSE)
			break;
		bfa_port_clear_stats_isr(port, i2hmsg->clearstats_rsp.status);
		break;

	default:
		bfa_assert(0);
	}
}

/**
 * bfa_port_meminfo()
 *
 * @brief Returns the size of the DMA memory needed by Port module
 *
 * @param[in] void
 *
 * @return Size of DMA region
 */
uint32_t
bfa_port_meminfo(void)
{
	return BFA_ROUNDUP(sizeof(union bfa_port_stats_u), BFA_DMA_ALIGN_SZ);
}

/**
 * bfa_port_mem_claim()
 *
 * @brief Initialized Port DMA Memory
 *
 * @param[in] port Port module pointer
 *	      dma_kva Kernel Virtual Address of Port DMA Memory
 *	      dma_pa  Physical Address of Port DMA Memory
 *
 * @return void
 */
void
bfa_port_mem_claim(struct bfa_port_s *port, uint8_t *dma_kva, uint64_t dma_pa)
{
	port->stats_dma.kva = dma_kva;
	port->stats_dma.pa  = dma_pa;
}

/**
 * bfa_port_enable()
 *
 * @brief
 *   Send the Port enable request to the f/w
 *
 * @param[in] Pointer to the Port module data structure.
 *
 * @return Status
 */
bfa_status_t
bfa_port_enable(struct bfa_port_s *port, bfa_port_endis_cbfn_t cbfn,
		 void *cbarg)
{
	struct bfi_port_generic_req_s *m;

	/* if port is PBC disabled, return error */
	if (port->pbc_disabled) {
		bfa_trc(port, BFA_STATUS_PBC);
		return BFA_STATUS_PBC;
	}

	if (bfa_ioc_is_disabled(port->ioc)) {
		bfa_trc(port, BFA_STATUS_IOC_DISABLED);
		return BFA_STATUS_IOC_DISABLED;
	}

	if (!bfa_ioc_is_operational(port->ioc)) {
		bfa_trc(port, BFA_STATUS_IOC_FAILURE);
		return BFA_STATUS_IOC_FAILURE;
	}

	/* if port is d-port enabled, return error */
	if (port->dport_enabled) {
		bfa_trc(port, BFA_STATUS_DPORT_ERR);
		return BFA_STATUS_DPORT_ERR;
	}

	if (port->endis_pending || bfa_phy_busy(port->ioc)) {
		bfa_trc(port, BFA_STATUS_DEVBUSY);
		return 	BFA_STATUS_DEVBUSY;
	}

	m = (struct bfi_port_generic_req_s *) port->endis_mb.msg;

	port->msgtag++;
	port->endis_cbfn    = cbfn;
	port->endis_cbarg   = cbarg;
	port->endis_pending = BFA_TRUE;

	bfi_h2i_set(m->mh, BFI_MC_PORT, BFI_PORT_H2I_ENABLE_REQ,
		    bfa_ioc_portid(port->ioc));
	bfa_ioc_mbox_queue(port->ioc, &port->endis_mb, NULL, NULL);

	return BFA_STATUS_OK;
}

/**
 * bfa_port_disable()
 *
 * @brief
 *   Send the Port disable request to the f/w
 *
 * @param[in] Pointer to the Port module data structure.
 *
 * @return Status
 */
bfa_status_t
bfa_port_disable(struct bfa_port_s *port, bfa_port_endis_cbfn_t cbfn,
		  void *cbarg)
{
	struct bfi_port_generic_req_s *m;

	/* if port is PBC disabled, return error */
	if (port->pbc_disabled) {
		bfa_trc(port, BFA_STATUS_PBC);
		return BFA_STATUS_PBC;
	}

	if (bfa_ioc_is_disabled(port->ioc)) {
		bfa_trc(port, BFA_STATUS_IOC_DISABLED);
		return BFA_STATUS_IOC_DISABLED;
	}

	if (!bfa_ioc_is_operational(port->ioc)) {
		bfa_trc(port, BFA_STATUS_IOC_FAILURE);
		return BFA_STATUS_IOC_FAILURE;
	}

	/* if port is d-port enabled, return error */
	if (port->dport_enabled) {
		bfa_trc(port, BFA_STATUS_DPORT_ERR);
		return BFA_STATUS_DPORT_ERR;
	}

	if (port->endis_pending || bfa_phy_busy(port->ioc)) {
		bfa_trc(port, BFA_STATUS_DEVBUSY);
		return 	BFA_STATUS_DEVBUSY;
	}

	m = (struct bfi_port_generic_req_s *) port->endis_mb.msg;

	port->msgtag++;
	port->endis_cbfn    = cbfn;
	port->endis_cbarg   = cbarg;
	port->endis_pending = BFA_TRUE;

	bfi_h2i_set(m->mh, BFI_MC_PORT, BFI_PORT_H2I_DISABLE_REQ,
		    bfa_ioc_portid(port->ioc));
	bfa_ioc_mbox_queue(port->ioc, &port->endis_mb, NULL, NULL);

	return BFA_STATUS_OK;
}

/**
 * bfa_port_get_stats()
 *
 * @brief
 *   Send the request to the f/w to fetch Port statistics.
 *
 * @param[in] Pointer to the Port module data structure.
 *
 * @return Status
 */
bfa_status_t
bfa_port_get_stats(struct bfa_port_s *port, union bfa_port_stats_u *stats,
		    bfa_port_stats_cbfn_t cbfn, void *cbarg)
{
	struct bfi_port_get_stats_req_s *m;

	if (!bfa_ioc_is_operational(port->ioc)) {
		bfa_trc(port, BFA_STATUS_IOC_FAILURE);
		return BFA_STATUS_IOC_FAILURE;
	}

	if (port->stats_busy) {
		bfa_trc(port, BFA_STATUS_DEVBUSY);
		return 	BFA_STATUS_DEVBUSY;
	}

	m = (struct bfi_port_get_stats_req_s *) port->stats_mb.msg;

	port->stats	  = stats;
	port->stats_cbfn  = cbfn;
	port->stats_cbarg = cbarg;
	port->stats_busy  = BFA_TRUE;
	bfa_dma_be_addr_set(m->dma_addr, port->stats_dma.pa);

	bfi_h2i_set(m->mh, BFI_MC_PORT, BFI_PORT_H2I_GET_STATS_REQ,
		    bfa_ioc_portid(port->ioc));
	bfa_ioc_mbox_queue(port->ioc, &port->stats_mb, NULL, NULL);

	return BFA_STATUS_OK;
}

/**
 * bfa_port_clear_stats()
 *
 * @brief Clears Port Stats in the f/w.
 *
 * @param[in] Pointer to the Port module data structure.
 *
 * @return Status
 */
bfa_status_t
bfa_port_clear_stats(struct bfa_port_s *port, bfa_port_stats_cbfn_t cbfn,
		      void *cbarg)
{
	struct bfi_port_generic_req_s *m;

	if (!bfa_ioc_is_operational(port->ioc)) {
		bfa_trc(port, BFA_STATUS_IOC_FAILURE);
		return BFA_STATUS_IOC_FAILURE;
	}

	if (port->stats_busy) {
		bfa_trc(port, BFA_STATUS_DEVBUSY);
		return 	BFA_STATUS_DEVBUSY;
	}

	m = (struct bfi_port_generic_req_s *) port->stats_mb.msg;

	port->stats_cbfn  = cbfn;
	port->stats_cbarg = cbarg;
	port->stats_busy  = BFA_TRUE;

	bfi_h2i_set(m->mh, BFI_MC_PORT, BFI_PORT_H2I_CLEAR_STATS_REQ,
		    bfa_ioc_portid(port->ioc));
	bfa_ioc_mbox_queue(port->ioc, &port->stats_mb, NULL, NULL);

	return BFA_STATUS_OK;
}

/**
 * bfa_port_notify()
 *
 * @brief Port module IOC event handler.
 *
 * @param[in] Pointer to the Port module data structure.
 * @param[in] IOC event structure
 *
 * @return void
 */
void
bfa_port_notify(void *arg, enum bfa_ioc_event_e event)
{
	struct bfa_port_s *port = (struct bfa_port_s *) arg;

	bfa_trc(port, event);

	switch (event) {
	case BFA_IOC_E_DISABLED:
	case BFA_IOC_E_FAILED:
		/* Fail any pending get_stats/clear_stats requests */
		if (port->stats_busy) {
			if (port->stats_cbfn)
				port->stats_cbfn(port->stats_cbarg,
						BFA_STATUS_FAILED);
			port->stats_cbfn = NULL;
			port->stats_busy = BFA_FALSE;
		}

		/* Clear any enable/disable is pending */
		if (port->endis_pending) {
			if (port->endis_cbfn)
				port->endis_cbfn(port->endis_cbarg,
						BFA_STATUS_FAILED);
			port->endis_cbfn = NULL;
			port->endis_pending = BFA_FALSE;
		}

		/* clear D-port mode */
		if (port->dport_enabled) {
			bfa_port_set_dportenabled(port, BFA_FALSE);
		}
		break;

	default:
		break;
	}
}

/**
 * bfa_port_attach()
 *
 * @brief Port module-attach API
 *
 * @param[in] port - Pointer to the Port module data structure
 *            ioc  - Pointer to the ioc module data structure
 *            dev  - Pointer to the device driver module data structure
 *                   The device driver specific mbox ISR functions have
 *                   this pointer as one of the parameters.
 *            trcmod -
 *            logmod -
 *
 * @return void
 */
void
bfa_port_attach(struct bfa_port_s *port, struct bfa_ioc_s *ioc,
		 void *dev, bfa_trc_mod_t *trcmod, bfa_log_mod_t *logmod)
{
	bfa_timeval_t tv;

	bfa_assert(port);

	port->dev    = dev;
	port->ioc    = ioc;
	port->trcmod = trcmod;
	port->logmod = logmod;

	port->stats_busy = BFA_FALSE;
	port->endis_pending = BFA_FALSE;
	port->stats_cbfn = NULL;
	port->endis_cbfn = NULL;
	port->pbc_disabled = BFA_FALSE;
	port->dport_enabled = BFA_FALSE;

	bfa_ioc_mbox_regisr(port->ioc, BFI_MC_PORT, bfa_port_isr, port);
	bfa_q_qe_init(&port->ioc_notify);
	bfa_ioc_notify_init(&port->ioc_notify, bfa_port_notify, port);
	bfa_ioc_notify_register(port->ioc, &port->ioc_notify);

	/**
	 * initialize time stamp for stats reset
	 */
	bfa_os_gettimeofday(&tv);
	port->stats_reset_time = tv.tv_sec;

	bfa_trc(port, 0);
}

/**
 * bfa_port_detach()
 *
 * @brief Port module-detach API
 *
 * @param[in] port - Pointer to the Port module data structure
 *
 * @return void
 */
void
bfa_port_detach(struct bfa_port_s *port)
{
	bfa_trc(port, 0);
}

/**
 * bfa_port_set_pbcdisabled();
 *
 * @brief Port module- set pbc disabled flag
 *
 * @param[in] port - Pointer to the Port module data structure
 *
 * @return void
 */
void
bfa_port_set_pbcdisabled(struct bfa_port_s *port)
{
	port->pbc_disabled = BFA_TRUE;
}

/**
 * bfa_port_set_dportenabled();
 *
 * @brief Port module- set pbc disabled flag
 *
 * @param[in] port - Pointer to the Port module data structure
 *
 * @return void
 */
void
bfa_port_set_dportenabled(struct bfa_port_s *port, bfa_boolean_t enabled)
{
	port->dport_enabled = enabled;
}
