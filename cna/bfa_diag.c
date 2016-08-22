/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#include <cna/bfa_ioc.h>
#include <cna/bfa_diag.h>

BFA_TRC_FILE(CNA, DIAG);

#define	BFA_DIAG_MEMTEST_TOV	50000	/* !< memtest timeout in msec */
#define	CT2_BFA_DIAG_MEMTEST_TOV	(9*30*1000)  /* 4.5 min */

/**
 * @dg hal_diag_pvt BFA DIAG private functions
 * @{
 */

static void
bfa_cb_diag(struct bfa_diag_s *diag)
{
		diag->cbfn(diag->cbarg, diag->status);
	diag->block = 0;
}

static void
bfa_cb_diag_fwping(struct bfa_diag_s *diag)
{
		diag->fwping.cbfn(diag->fwping.cbarg, diag->fwping.status);
		diag->fwping.lock = 0;
}

static void
bfa_cb_diag_tsensor(struct bfa_diag_s *diag)
{
		diag->tsensor.cbfn(diag->tsensor.cbarg, diag->tsensor.status);
		diag->tsensor.lock = 0;
}

/**
 * IOC event handler.
 */
static void
bfa_diag_notify(void *diag_arg, enum bfa_ioc_event_e event)
{
	struct bfa_diag_s *diag = diag_arg;

	bfa_trc(diag, event);
	bfa_trc(diag, diag->block);
	bfa_trc(diag, diag->fwping.lock);
	bfa_trc(diag, diag->tsensor.lock);

	switch (event) {
	case BFA_IOC_E_DISABLED:
	case BFA_IOC_E_FAILED:
		if (diag->fwping.lock) {
			diag->fwping.status = BFA_STATUS_IOC_FAILURE;
			bfa_cb_diag_fwping(diag);
		}

		if (diag->tsensor.lock) {
			diag->tsensor.status = BFA_STATUS_IOC_FAILURE;
			bfa_cb_diag_tsensor(diag);
		}

		if (diag->block) {
			if (diag->timer_active) {
				bfa_timer_stop(&diag->timer);
				diag->timer_active = 0;
			}

			diag->status = BFA_STATUS_IOC_FAILURE;
			bfa_cb_diag(diag);
		}
		break;

	default:
		break;
	}
}

static void
bfa_diag_memtest_done(void *cbarg)
{
	bfa_diag_mod_t *diag = cbarg;
	struct bfa_ioc_s  *ioc = diag->ioc;
	bfa_diag_memtest_result_t *res = diag->result;
	uint32_t	loff = BFI_BOOT_MEMTEST_RES_ADDR;
	uint32_t	pgnum, pgoff, i;

	pgnum = bfa_ioc_smem_pgnum(ioc, loff);
	pgoff = bfa_ioc_smem_pgoff(ioc, loff);

	bfa_reg_write(ioc->ioc_regs.host_page_num_fn, pgnum);

	for (i = 0; i < (sizeof(bfa_diag_memtest_result_t) / sizeof(uint32_t));
	     i++) {
		/*
		 * read test result from smem
		 */
		*((uint32_t *) res + i) =
			bfa_mem_read(ioc->ioc_regs.smem_page_start, loff);
		loff += sizeof(uint32_t);
	}

	/* Reset IOC fwstates to BFI_IOC_UNINIT */
	bfa_ioc_reset_fwstate(ioc);

	res->status = bfa_os_swap32(res->status);
	bfa_trc(diag, res->status);

	if (res->status == BFI_BOOT_MEMTEST_RES_SIG) {
		diag->status = BFA_STATUS_OK;
	} else {
		diag->status = BFA_STATUS_MEMTEST_FAILED;

		res->addr = bfa_os_swap32(res->addr);
		res->exp = bfa_os_swap32(res->exp);
		res->act = bfa_os_swap32(res->act);
		res->err_status = bfa_os_swap32(res->err_status);
		res->err_status1 = bfa_os_swap32(res->err_status1);
		res->err_addr = bfa_os_swap32(res->err_addr);

		bfa_trc(diag, res->addr);
		bfa_trc(diag, res->exp);
		bfa_trc(diag, res->act);
		bfa_trc(diag, res->err_status);
		bfa_trc(diag, res->err_status1);
		bfa_trc(diag, res->err_addr);
	}

	diag->timer_active = 0;
	bfa_cb_diag(diag);
}

/*
 * Firmware ping
 */

/**
 * Perform DMA test directly
 */
static void
diag_fwping_send(struct bfa_diag_s *diag)
{
	bfi_diag_fwping_req_t *fwping_req;
	uint32_t	i;

	bfa_trc(diag, diag->fwping.dbuf_pa);

	/* fill DMA area with pattern */
	for (i = 0; i < (BFI_DIAG_DMA_BUF_SZ >> 2); i++) {
		*((uint32_t *)diag->fwping.dbuf_kva + i) = diag->fwping.data;
	}

	/* Fill mbox msg */
	fwping_req = (bfi_diag_fwping_req_t *)diag->fwping.mbcmd.msg;

	/* Setup SG list */
	bfa_alen_set(&fwping_req->alen, BFI_DIAG_DMA_BUF_SZ,
		     diag->fwping.dbuf_pa);
	/* Set up dma count */
	fwping_req->count = bfa_os_htonl(diag->fwping.count);
	/* Set up data pattern */
	fwping_req->data = diag->fwping.data;

	/* build host command */
	bfi_h2i_set(fwping_req->mh, BFI_MC_DIAG, BFI_DIAG_H2I_FWPING,
		    bfa_ioc_portid(diag->ioc));

	/* send mbox cmd */
	bfa_ioc_mbox_queue(diag->ioc, &diag->fwping.mbcmd, NULL, NULL);
}

static void
diag_fwping_comp(struct bfa_diag_s *diag, bfi_diag_fwping_rsp_t *diag_rsp)
{
	uint32_t	rsp_data = diag_rsp->data;
	uint8_t		rsp_dma_status = diag_rsp->dma_status;

	bfa_trc(diag, rsp_data);
	bfa_trc(diag, rsp_dma_status);

	if (rsp_dma_status == BFA_STATUS_OK) {
		uint32_t i, pat;
		pat = (diag->fwping.count & 0x1) ?
			~(diag->fwping.data) :
			diag->fwping.data;
		/* Check mbox data */
		if (diag->fwping.data != rsp_data) {
			bfa_trc(diag, rsp_data);
			diag->fwping.result->dmastatus =
				BFA_STATUS_DATACORRUPTED;
			diag->fwping.status = BFA_STATUS_DATACORRUPTED;
			bfa_cb_diag_fwping(diag);
				return;
		}
		/* Check dma pattern */
		for (i = 0; i < (BFI_DIAG_DMA_BUF_SZ >> 2); i++) {
			if (*((uint32_t *)diag->fwping.dbuf_kva + i) != pat) {
				bfa_trc(diag, i);
				bfa_trc(diag, pat);
				bfa_trc(diag,
					*((uint32_t *)diag->fwping.dbuf_kva +
					  i));

				diag->fwping.result->dmastatus =
					BFA_STATUS_DATACORRUPTED;
				diag->fwping.status = BFA_STATUS_DATACORRUPTED;
				bfa_cb_diag_fwping(diag);
					return;
			}
		}
		diag->fwping.result->dmastatus = BFA_STATUS_OK;
		diag->fwping.status = BFA_STATUS_OK;
		bfa_cb_diag_fwping(diag);
	} else {
		diag->fwping.status = BFA_STATUS_HDMA_FAILED;
		bfa_cb_diag_fwping(diag);
	}
}

/*
 * Temperature Sensor
 */

static void
diag_tempsensor_send(struct bfa_diag_s *diag)
{
	bfi_diag_ts_req_t *msg;

	msg = (bfi_diag_ts_req_t *)diag->tsensor.mbcmd.msg;
	bfa_trc(diag, msg->temp);
	/* build host command */
	bfi_h2i_set(msg->mh, BFI_MC_DIAG, BFI_DIAG_H2I_TEMPSENSOR,
		    bfa_ioc_portid(diag->ioc));
	/* send mbox cmd */
	bfa_ioc_mbox_queue(diag->ioc, &diag->tsensor.mbcmd, NULL, NULL);
}

static void
diag_tempsensor_comp(struct bfa_diag_s *diag, bfi_diag_ts_rsp_t *rsp)
{
	if (!diag->tsensor.lock) {
		/*
		 * receiving response after ioc failure
		 */
		bfa_trc(diag, diag->tsensor.lock);
		return;
	}

	/*
	 * ASIC junction tempsensor is a reg read operation
	 * it will always return OK
	 */
	diag->tsensor.temp->temp = bfa_os_ntohs(rsp->temp);
	diag->tsensor.temp->ts_junc = rsp->ts_junc;
	diag->tsensor.temp->ts_brd = rsp->ts_brd;

	if (rsp->ts_brd) {
		/* tsensor.temp->status is brd_temp status */
		diag->tsensor.temp->status = rsp->status;
		if (rsp->status == BFA_STATUS_OK) {
			diag->tsensor.temp->brd_temp =
				bfa_os_ntohs(rsp->brd_temp);
		} else {
			diag->tsensor.temp->brd_temp = 0;
		}
	}

	bfa_trc(diag, rsp->status);
	bfa_trc(diag, rsp->ts_junc);
	bfa_trc(diag, rsp->temp);
	bfa_trc(diag, rsp->ts_brd);
	bfa_trc(diag, rsp->brd_temp);

	/* tsensor status is always good bcos we always have junction temp */
	diag->tsensor.status = BFA_STATUS_OK;
	bfa_cb_diag_tsensor(diag);
}

/*
 *      LED Test command
 */
static void
diag_ledtest_send(struct bfa_diag_s *diag, bfa_diag_ledtest_t *ledtest)
{
	bfi_diag_ledtest_req_t	*msg;

	msg = (bfi_diag_ledtest_req_t *)diag->ledtest.mbcmd.msg;
	/* build host command */
	bfi_h2i_set(msg->mh, BFI_MC_DIAG, BFI_DIAG_H2I_LEDTEST,
		    bfa_ioc_portid(diag->ioc));

	/*
	 * convert the freq from N blinks per 10 sec to
	 * crossbow ontime value. We do it here because division is need
	 */
	if (ledtest->freq)
		ledtest->freq = 500 / ledtest->freq;

	if (ledtest->freq == 0)
		ledtest->freq = 1;

	bfa_trc(diag, ledtest->freq);
	/* mcpy(&ledtest_req->req, ledtest, sizeof(bfa_diag_ledtest_t)); */
	msg->cmd = (uint8_t) ledtest->cmd;
	msg->color = (uint8_t) ledtest->color;
	msg->portid = bfa_ioc_portid(diag->ioc);
	msg->led = ledtest->led;
	msg->freq = bfa_os_htons(ledtest->freq);

	/* send mbox cmd */
	bfa_ioc_mbox_queue(diag->ioc, &diag->ledtest.mbcmd, NULL, NULL);
}

static void
diag_ledtest_comp(struct bfa_diag_s *diag, bfi_diag_ledtest_rsp_t * msg)
{
	bfa_trc(diag, diag->ledtest.lock);
	diag->ledtest.lock = BFA_FALSE;
	/* no bfa_cb_queue is needed because driver is not waiting */
}

/*
 * Port beaconing
 */
static void
diag_portbeacon_send(struct bfa_diag_s *diag, bfa_boolean_t beacon,
		     uint32_t sec)
{
	bfi_diag_portbeacon_req_t *msg;

	msg = (bfi_diag_portbeacon_req_t *)diag->beacon.mbcmd.msg;
	/* build host command */
	bfi_h2i_set(msg->mh, BFI_MC_DIAG, BFI_DIAG_H2I_PORTBEACON,
		    bfa_ioc_portid(diag->ioc));

	msg->beacon = beacon;
	msg->period = bfa_os_htonl(sec);
	/* send mbox cmd */
	bfa_ioc_mbox_queue(diag->ioc, &diag->beacon.mbcmd, NULL, NULL);
}

static void
diag_portbeacon_comp(struct bfa_diag_s *diag)
{
	bfa_trc(diag, diag->beacon.state);
	diag->beacon.state = BFA_FALSE;

	if (diag->cbfn_beacon)
		diag->cbfn_beacon(diag->dev, BFA_FALSE,
				  diag->beacon.link_e2e);
}

/**
 * @}
 */

/**
 * @dg hal_diag_public
 * @{
 */
/**
 * @ Diag hmbox handler
 */
void
bfa_diag_intr(void *diagarg, struct bfi_mbmsg_s *msg)
{
	struct bfa_diag_s *diag = diagarg;

	switch (msg->mh.msg_id) {
	case BFI_DIAG_I2H_PORTBEACON:
		diag_portbeacon_comp(diag);
		break;

	case BFI_DIAG_I2H_FWPING:
		diag_fwping_comp(diag, (bfi_diag_fwping_rsp_t *) msg);
		break;

	case BFI_DIAG_I2H_TEMPSENSOR:
		diag_tempsensor_comp(diag, (bfi_diag_ts_rsp_t *) msg);
		break;

	case BFI_DIAG_I2H_LEDTEST:
		diag_ledtest_comp(diag, (bfi_diag_ledtest_rsp_t *) msg);
		break;

	default:
		bfa_trc(diag, msg->mh.msg_id);
		bfa_assert(0);
	}
}

/**
 * @}
 */

/**
 * @ig hal_diag_api
 * @{
 */

/*
 * Register access
 */

/**
 * @brief
 *   DIAG register read
 */
bfa_status_t
bfa_diag_reg_read(struct bfa_diag_s *diag, uint32_t offset, uint32_t len,
		      uint32_t *buf, uint32_t force)
{
	bfa_status_t    status;
	struct bfa_ioc_s  *ioc = diag->ioc;
	bfa_os_addr_t   rb;
	bfa_os_addr_t   addr;
	uint32_t	i;

	if (!force && !bfa_ioc_is_initialized(ioc))
		return BFA_STATUS_IOC_NON_OP;

	rb = bfa_ioc_bar0(ioc);

	offset &= BFA_REG_ADDRMSK(ioc);	/* offset only 17 bit and word align */

	/*
	 * offset and len sanity check
	 */
	status = bfa_ioc_reg_offset_check(ioc, offset, len);
	if (status)
		return status;

	addr = rb + offset;
	for (i = 0; i < len; i++) {
		*buf = bfa_reg_read(addr);
		buf++;
		addr += sizeof(uint32_t);
	}
	return BFA_STATUS_OK;
}

/**
 * @brief
 *   DIAG register write
 */
bfa_status_t
bfa_diag_reg_write(struct bfa_diag_s *diag, uint32_t offset, uint32_t len,
		       uint32_t value, uint32_t force)
{
	bfa_status_t    status;
	struct bfa_ioc_s  *ioc = diag->ioc;
	bfa_os_addr_t	addr;
	uint32_t i;

	if (!force && !bfa_ioc_is_initialized(ioc))
		return BFA_STATUS_IOC_NON_OP;

	offset &= BFA_REG_ADDRMSK(ioc);	/* offset only 17 bit and word align */

	/*
	 * offset and len sanity check
	 */
	status = bfa_ioc_reg_offset_check(ioc, offset, len);
	if (status)
		return status;

	addr = ((uint8_t *) bfa_ioc_bar0(ioc) + offset);
	for (i = 0; i < len; i++) {
		bfa_reg_write(addr, value);
		addr++;
	}
	return BFA_STATUS_OK;
}

/**
 * @brief
 * Gen RAM Test
 *
 *   @param[in] *diag		- diag data struct
 *   @param[in] *memtest	- mem test params input from upper layer,
 *   @param[in] pattern		- mem test pattern
 *   @param[in] *result		- mem test result
 *   @param[in] cbfn		- mem test callback functioin
 *   @param[in] cbarg		- callback functioin arg
 *
 *   @param[out]
 */
bfa_status_t
bfa_diag_memtest(struct bfa_diag_s *diag, bfa_diag_memtest_t *memtest,
		     uint32_t pattern, bfa_diag_memtest_result_t *result,
		     bfa_cb_diag_t cbfn, void *cbarg)
{
	uint32_t	memtest_tov;

	bfa_trc(diag, pattern);

	if (!bfa_ioc_adapter_is_disabled(diag->ioc))
		return BFA_STATUS_ADAPTER_ENABLED;

	/*
	 * check to see if there is another destructive diag cmd running
	 */
	if (diag->block) {
		bfa_trc(diag, diag->block);
		return (BFA_STATUS_DEVBUSY);
	} else {
		diag->block = 1;
	}

	diag->result = result;
	diag->cbfn = cbfn;
	diag->cbarg = cbarg;

	/*
	 * download memtest code and take LPU0 out of reset
	 */
	bfa_ioc_boot(diag->ioc, BFI_FWBOOT_TYPE_MEMTEST, BFI_FWBOOT_ENV_OS);

	memtest_tov = (bfa_ioc_asic_gen(diag->ioc) == BFI_ASIC_GEN_CT2) ?
		CT2_BFA_DIAG_MEMTEST_TOV : BFA_DIAG_MEMTEST_TOV;
	bfa_timer_begin(diag->ioc->timer_mod, &diag->timer,
			bfa_diag_memtest_done, diag, memtest_tov);
	diag->timer_active = 1;
	return (BFA_STATUS_OK);
}

/**
 * @brief
 * DIAG firmware ping command
 *
 *   @param[in] *diag		- diag data struct
 *   @param[in] cnt		- dma loop count for testing PCIE
 *   @param[in] data		- data pattern to pass in fw
 *   @param[in] *result		- pt to bfa_diag_fwping_result_t data struct
 *   @param[in] cbfn		- callback function
 *   @param[in] *cbarg		- callback functioin arg
 *
 *   @param[out]
 */
bfa_status_t
bfa_diag_fwping(struct bfa_diag_s *diag, uint32_t cnt, uint32_t data,
		    bfa_diag_fwping_result_t *result, bfa_cb_diag_t cbfn,
		    void *cbarg)
{
	bfa_trc(diag, cnt);
	bfa_trc(diag, data);

	if (!bfa_ioc_is_operational(diag->ioc))
		return BFA_STATUS_IOC_NON_OP;

	if (bfa_asic_id_ct2(bfa_ioc_devid((diag->ioc))) &&
	    ((diag->ioc)->clscode == BFI_PCIFN_CLASS_ETH))
		return BFA_STATUS_CMD_NOTSUPP;

	/*
	 * check to see if there is another destructive diag cmd running
	 */
	if (diag->block || diag->fwping.lock) {
		bfa_trc(diag, diag->block);
		bfa_trc(diag, diag->fwping.lock);
		return (BFA_STATUS_DEVBUSY);
	}

	/* Initialization */
	diag->fwping.lock = 1;
	diag->fwping.cbfn = cbfn;
	diag->fwping.cbarg = cbarg;
	diag->fwping.result = result;
	diag->fwping.data = data;
	diag->fwping.count = cnt;

	/* Init test results */
	diag->fwping.result->data = 0;
	diag->fwping.result->status = BFA_STATUS_OK;

	/* kick off the first ping */
	diag_fwping_send(diag);
	return (BFA_STATUS_OK);
}

/**
 * @brief
 * Read Temperature Sensor
 *
 *   @param[in] *diag		- diag data struct
 *   @param[in] *result		- pt to bfa_diag_temp_t data struct
 *   @param[in] cbfn		- callback function
 *   @param[in] *cbarg		- callback functioin arg
 *
 *   @param[out]
 */
bfa_status_t
bfa_diag_tsensor_query(struct bfa_diag_s *diag, bfa_diag_temp_t *result,
		       bfa_cb_diag_t cbfn, void *cbarg)
{
	/*
	 * check to see if there is a destructive diag cmd running
	 */
	if (diag->block || diag->tsensor.lock) {
		bfa_trc(diag, diag->block);
		bfa_trc(diag, diag->tsensor.lock);
		return (BFA_STATUS_DEVBUSY);
	}

	if (!bfa_ioc_is_operational(diag->ioc))
		return BFA_STATUS_IOC_NON_OP;

	/* Init diag mod params */
	diag->tsensor.lock = 1;
	diag->tsensor.temp = result;
	diag->tsensor.cbfn = cbfn;
	diag->tsensor.cbarg = cbarg;
	diag->tsensor.status = BFA_STATUS_OK;

	/* Send msg to fw */
	diag_tempsensor_send(diag);

	return (BFA_STATUS_OK);
}

/**
 * @brief
 * LED Test command
 *
 *   @param[in] *diag		- diag data struct
 *   @param[in] *ledtest	- pt to ledtest data structure
 *
 *   @param[out]
 */
bfa_status_t
bfa_diag_ledtest(struct bfa_diag_s *diag, bfa_diag_ledtest_t *ledtest)
{
	bfa_trc(diag, ledtest->cmd);

	if (!bfa_ioc_is_operational(diag->ioc))
		return BFA_STATUS_IOC_NON_OP;

	if (diag->beacon.state)
		return BFA_STATUS_BEACON_ON;

	if (diag->ledtest.lock)
		return BFA_STATUS_LEDTEST_OP;

	/* Send msg to fw */
	diag->ledtest.lock = BFA_TRUE;
	diag_ledtest_send(diag, ledtest);

	return (BFA_STATUS_OK);
}

/**
 * @brief
 * Port beaconing command
 *
 *   @param[in] *diag		- diag data struct
 *   @param[in] beacon		- port beaconing 1:ON	0:OFF
 *   @param[in] link_e2e_beacon	- link beaconing 1:ON	0:OFF
 *   @param[in] sec		- beaconing duration in seconds
 *
 *   @param[out]
 */
bfa_status_t
bfa_diag_beacon_port(struct bfa_diag_s *diag, bfa_boolean_t beacon,
		     bfa_boolean_t link_e2e_beacon, uint32_t sec)
{
	bfa_trc(diag, beacon);
	bfa_trc(diag, link_e2e_beacon);
	bfa_trc(diag, sec);
	if (!bfa_ioc_is_operational(diag->ioc))
		return BFA_STATUS_IOC_NON_OP;

	if (diag->ledtest.lock)
		return BFA_STATUS_LEDTEST_OP;

	if (diag->beacon.state && beacon)	/* beacon alread on */
		return BFA_STATUS_BEACON_ON;

	diag->beacon.state 		= beacon;
	diag->beacon.link_e2e 	= link_e2e_beacon;
	if (diag->cbfn_beacon)
		diag->cbfn_beacon(diag->dev, beacon, link_e2e_beacon);

	/* Send msg to fw */
	diag_portbeacon_send(diag, beacon, sec);

	return (BFA_STATUS_OK);
}


/**
 * Return DMA memory needed by diag module.
 */
uint32_t
bfa_diag_meminfo(void)
{
	return (BFA_ROUNDUP(BFI_DIAG_DMA_BUF_SZ, BFA_DMA_ALIGN_SZ));
}

/**
 * Attach virtual and physical memory for Diag.
 */
void
bfa_diag_attach(struct bfa_diag_s *diag, struct bfa_ioc_s *ioc, void *dev,
	bfa_cb_diag_beacon_t cbfn_beacon, bfa_trc_mod_t *trcmod)
{
	diag->dev = dev;
	diag->ioc = ioc;
	diag->trcmod = trcmod;

	diag->block = 0;
	diag->cbfn = NULL;
	diag->cbarg = NULL;
	diag->result = NULL;
	diag->cbfn_beacon = cbfn_beacon;

	bfa_ioc_mbox_regisr(diag->ioc, BFI_MC_DIAG, bfa_diag_intr, diag);
	bfa_q_qe_init(&diag->ioc_notify);
	bfa_ioc_notify_init(&diag->ioc_notify, bfa_diag_notify, diag);
	bfa_ioc_notify_register(diag->ioc, &diag->ioc_notify);
}

void
bfa_diag_memclaim(struct bfa_diag_s *diag, uint8_t *dm_kva, uint64_t dm_pa)
{
	diag->fwping.dbuf_kva = dm_kva;
	diag->fwping.dbuf_pa = dm_pa;
	bfa_os_memset(diag->fwping.dbuf_kva, 0, BFI_DIAG_DMA_BUF_SZ);
}

/**
 * @}
 */
