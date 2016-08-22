/*
 * Copyright (c) 2009-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#include <defs/bfa_defs_cna.h>
#include <aen/bfa_aen.h>
#include <cs/bfa_log.h>
#include <bfi/bfi_cna.h>
#include <cna/bfa_ioc.h>
#include <cna/bfa_sfp.h>


BFA_TRC_FILE(CNA, SFP);

/*
 * forward declarations
 */
static void bfa_sfp_getdata_send(struct bfa_sfp_s *sfp);
static void bfa_sfp_media_get(struct bfa_sfp_s *sfp);
static bfa_status_t bfa_sfp_speed_valid(struct bfa_sfp_s *sfp,
					bfa_port_speed_t portspeed);


/**
 * @dg hal_sfp_pvt BFA SFP private functions
 * @{
 */

static void
bfa_cb_sfp_show(struct bfa_sfp_s *sfp)
{
	bfa_trc(sfp, sfp->lock);
	if (sfp->cbfn)
		sfp->cbfn(sfp->cbarg, sfp->status);
	sfp->lock = 0;
	sfp->cbfn = NULL;
}

static void
bfa_cb_sfp_state_query(struct bfa_sfp_s *sfp)
{
	bfa_trc(sfp, sfp->portspeed);
	if (sfp->media) {
		bfa_sfp_media_get(sfp);
		if (sfp->state_query_cbfn)
			sfp->state_query_cbfn(sfp->state_query_cbarg,
					      sfp->status);
		sfp->media = NULL;
	}

	if (sfp->portspeed) {
		sfp->status = bfa_sfp_speed_valid(sfp, sfp->portspeed);
		if (sfp->state_query_cbfn)
			sfp->state_query_cbfn(sfp->state_query_cbarg,
					      sfp->status);
		sfp->portspeed = BFA_PORT_SPEED_UNKNOWN;
	}

	sfp->state_query_lock = 0;
	sfp->state_query_cbfn = NULL;
}

/**
 * IOC event handler.
 */
static void
bfa_sfp_notify(void *sfp_arg, enum bfa_ioc_event_e event)
{
	struct bfa_sfp_s *sfp = sfp_arg;

	bfa_trc(sfp, event);
	bfa_trc(sfp, sfp->lock);
	bfa_trc(sfp, sfp->state_query_lock);

	switch (event) {
	case BFA_IOC_E_DISABLED:
	case BFA_IOC_E_FAILED:
		if (sfp->lock) {
			sfp->status = BFA_STATUS_IOC_FAILURE;
			bfa_cb_sfp_show(sfp);
		}

		if (sfp->state_query_lock) {
			sfp->status = BFA_STATUS_IOC_FAILURE;
			bfa_cb_sfp_state_query(sfp);
		}
		break;

	default:
		break;
	}
}

/**
 * Mapping sfp scn event to aen event
 */
static enum bfa_port_aen_event
bfa_sfp_map_aen_evt(struct bfa_sfp_s *sfp, bfi_sfp_scn_t *rsp)
{
	enum bfa_port_aen_event aen_evt = 0;

	switch (rsp->event) {
	case BFA_SFP_SCN_INSERTED:
		aen_evt = BFA_PORT_AEN_SFP_INSERT;
		break;
	case BFA_SFP_SCN_REMOVED:
		aen_evt = BFA_PORT_AEN_SFP_REMOVE;
		break;
	case BFA_SFP_SCN_FAILED:
		aen_evt = BFA_PORT_AEN_SFP_ACCESS_ERROR;
		break;
	case BFA_SFP_SCN_UNSUPPORT:
		aen_evt = BFA_PORT_AEN_SFP_UNSUPPORT;
		break;
	case BFA_SFP_SCN_POM:
		aen_evt = BFA_PORT_AEN_SFP_POM;
		break;
	default:
		bfa_trc(sfp, rsp->event);
		bfa_assert(0);
	}

	return aen_evt;
}

/**
 * SFP's State Change Notification post to AEN
 */
static void
bfa_sfp_scn_aen_post(struct bfa_sfp_s *sfp, bfi_sfp_scn_t *rsp)
{
	union bfa_aen_data_u aen_data;
	struct bfa_log_mod_s *logmod = sfp->ioc->logm;
	enum bfa_port_aen_event aen_evt;

	wwn_t		pwwn;
	char		pwwn_buf[BFA_STRING_32];
	char		*pwwn_ptr = NULL;
	mac_t		mac;
	char		mac_buf[BFA_STRING_32];
	char		*mac_ptr = NULL;
	char		*pomlvl_str[BFA_PORT_AEN_SFP_POM_MAX] =
		{"Green", "Amber", "Red"};
	char		*pomlvl_ptr;
	enum bfa_ioc_type_e ioc_type = bfa_ioc_get_type(sfp->ioc);

	bfa_trc(sfp, (((uint64_t)rsp->pomlvl) << 16) |
		(((uint64_t)rsp->sfpid) << 8) |
		((uint64_t)rsp->event));

	memset(&pwwn, 0, sizeof(pwwn));
	memset(&mac, 0, sizeof(mac));

	aen_evt = bfa_sfp_map_aen_evt(sfp, rsp);
	aen_data.port.ioc_type = ioc_type;

	switch (ioc_type) {
	case BFA_IOC_TYPE_FC:
		pwwn = bfa_ioc_get_pwwn(sfp->ioc);
		pwwn_ptr = wwn2str(pwwn_buf, sizeof(pwwn_buf), pwwn);
		bfa_trc(sfp, pwwn);
		break;
	case BFA_IOC_TYPE_FCoE:
		pwwn = bfa_ioc_get_pwwn(sfp->ioc);
		pwwn_ptr = wwn2str(pwwn_buf, sizeof(pwwn_buf), pwwn);
		mac = bfa_ioc_get_mac(sfp->ioc);
		mac_ptr = mac2str(mac_buf, sizeof(mac_buf), mac);
		bfa_trc(sfp, pwwn);
		break;
	case BFA_IOC_TYPE_LL:
		mac = bfa_ioc_get_mac(sfp->ioc);
		mac_ptr = mac2str(mac_buf, sizeof(mac_buf), mac);
		break;
	default:
		bfa_assert(ioc_type == BFA_IOC_TYPE_FC);
		break;
	}
	aen_data.port.pwwn = pwwn;
	aen_data.port.mac = mac;

	if (aen_evt == BFA_PORT_AEN_SFP_POM) {
		bfa_assert((rsp->pomlvl >= BFA_PORT_AEN_SFP_POM_GREEN) &&
			    (rsp->pomlvl <= BFA_PORT_AEN_SFP_POM_RED));
		aen_data.port.level = rsp->pomlvl;
		pomlvl_ptr = pomlvl_str[rsp->pomlvl - 1];
		switch (ioc_type) {
		case BFA_IOC_TYPE_FC:
			bfa_log(logmod,
				BFA_LOG_CREATE_ID(BFA_AEN_CAT_PORT, aen_evt),
				pomlvl_ptr, pwwn_ptr);
			break;
		case BFA_IOC_TYPE_LL:
			bfa_log(logmod,
				BFA_LOG_CREATE_ID(BFA_AEN_CAT_PORT, aen_evt),
				pomlvl_ptr, mac_ptr);
			break;
		case BFA_IOC_TYPE_FCoE:
			bfa_log(logmod,
				BFA_LOG_CREATE_ID(BFA_AEN_CAT_PORT, aen_evt),
				pomlvl_ptr, pwwn_ptr);
			bfa_log(logmod,
				BFA_LOG_CREATE_ID(BFA_AEN_CAT_PORT, aen_evt),
				pomlvl_ptr, mac_ptr);
			break;
		default:
			bfa_assert(0);
		}
	} else {
		switch (ioc_type) {
		case BFA_IOC_TYPE_FC:
			bfa_log(logmod,
				BFA_LOG_CREATE_ID(BFA_AEN_CAT_PORT, aen_evt),
				pwwn_ptr);
			break;
		case BFA_IOC_TYPE_LL:
			bfa_log(logmod,
				BFA_LOG_CREATE_ID(BFA_AEN_CAT_PORT, aen_evt),
				mac_ptr);
			break;
		case BFA_IOC_TYPE_FCoE:
			bfa_log(logmod,
				BFA_LOG_CREATE_ID(BFA_AEN_CAT_PORT, aen_evt),
				pwwn_ptr);
			bfa_log(logmod,
				BFA_LOG_CREATE_ID(BFA_AEN_CAT_PORT, aen_evt),
				mac_ptr);
			break;
		default:
			bfa_assert(0);
		}
	}

	bfa_aen_post(sfp->ioc->aen, BFA_AEN_CAT_PORT, aen_evt, &aen_data);
}

/**
 * SFP get data send
 */
static void
bfa_sfp_getdata_send(struct bfa_sfp_s *sfp)
{
	bfi_sfp_req_t *req = (bfi_sfp_req_t *)sfp->mbcmd.msg;

	bfa_trc(sfp, req->memtype);

	/* build host command */
	bfi_h2i_set(req->mh, BFI_MC_SFP, BFI_SFP_H2I_SHOW,
		    bfa_ioc_portid(sfp->ioc));

	/* send mbox cmd */
	bfa_ioc_mbox_queue(sfp->ioc, &sfp->mbcmd, NULL, NULL);
}

/**
 * SFP is valid, read sfp data
 */
static void
bfa_sfp_getdata(struct bfa_sfp_s *sfp, bfi_sfp_mem_t memtype)
{
	bfi_sfp_req_t *req = (bfi_sfp_req_t *)sfp->mbcmd.msg;

	bfa_assert(sfp->lock == 0);
	bfa_trc(sfp, sfp->state);

	sfp->lock = 1;
	sfp->memtype = memtype;
	req->memtype = memtype;

	/* Setup SG list */
	bfa_alen_set(&req->alen, sizeof(sfp_mem_t), sfp->dbuf_pa);

	bfa_sfp_getdata_send(sfp);
}

/**
 * SFP scn handler
 */
static void
bfa_sfp_scn(struct bfa_sfp_s *sfp, struct bfi_mbmsg_s *msg)
{
	bfi_sfp_scn_t *rsp = (bfi_sfp_scn_t *) msg;

	switch (rsp->event) {
	case BFA_SFP_SCN_INSERTED:
		sfp->state = BFA_SFP_STATE_INSERTED;
		sfp->data_valid = 0;
		bfa_sfp_scn_aen_post(sfp, rsp);
		break;

	case BFA_SFP_SCN_REMOVED:
		sfp->state = BFA_SFP_STATE_REMOVED;
		sfp->data_valid = 0;
		bfa_sfp_scn_aen_post(sfp, rsp);
		break;

	case BFA_SFP_SCN_FAILED:
		sfp->state = BFA_SFP_STATE_FAILED;
		sfp->data_valid = 0;
		bfa_sfp_scn_aen_post(sfp, rsp);
		break;

	case BFA_SFP_SCN_UNSUPPORT:
		sfp->state = BFA_SFP_STATE_UNSUPPORT;
		bfa_sfp_scn_aen_post(sfp, rsp);
		if (!sfp->lock)
		bfa_sfp_getdata(sfp, BFI_SFP_MEM_ALL);
		break;

	case BFA_SFP_SCN_POM:
		bfa_sfp_scn_aen_post(sfp, rsp);
		break;

	case BFA_SFP_SCN_VALID:
		sfp->state = BFA_SFP_STATE_VALID;
		sfp->is_elb = rsp->is_elb;
		if (!sfp->lock)
		bfa_sfp_getdata(sfp, BFI_SFP_MEM_ALL);
		break;

	default:
		bfa_trc(sfp, rsp->event);
		bfa_assert(0);
	}
}

/**
 * SFP show complete
 */
static void
bfa_sfp_show_comp(struct bfa_sfp_s *sfp, struct bfi_mbmsg_s *msg)
{
	bfi_sfp_rsp_t *rsp = (bfi_sfp_rsp_t *) msg;

	if (!sfp->lock) {
		/*
		 * receiving response after ioc failure
		 */
		bfa_trc(sfp, sfp->lock);
		return;
	}

	bfa_trc(sfp, rsp->status);
	if (rsp->status == BFA_STATUS_OK) {
		sfp->data_valid	= 1;
		if (sfp->state == BFA_SFP_STATE_VALID) {
			sfp->status = BFA_STATUS_OK;
		} else if (sfp->state == BFA_SFP_STATE_UNSUPPORT) {
			sfp->status = BFA_STATUS_SFP_UNSUPP;
		} else {
			bfa_trc(sfp, sfp->state);
		}
	} else {
		sfp->data_valid	= 0;
		sfp->status = rsp->status;
		/*
		 * sfpshow shouldn't change sfp state
		 */
	}

	bfa_trc(sfp, sfp->memtype);
	if (sfp->memtype == BFI_SFP_MEM_DIAGEXT) {
		bfa_trc(sfp, sfp->data_valid);
		if (sfp->data_valid) {
			uint32_t size = sizeof(sfp_mem_t);
			uint8_t *des = (uint8_t *) &(sfp->sfpmem->srlid_base);
			memcpy(des, sfp->dbuf_kva, size);
		}
		/*
		 * Queue completion callback.
		 */
		bfa_cb_sfp_show(sfp);
	} else {
		sfp->lock = 0;
	}

	bfa_trc(sfp, sfp->state_query_lock);
	if (sfp->state_query_lock) {
		sfp->state = rsp->state;
		/* Complete callback */
		bfa_cb_sfp_state_query(sfp);
	}
}

/**
 * SFP query fw sfp state
 */
static void
bfa_sfp_state_query(struct bfa_sfp_s *sfp)
{
	bfi_sfp_req_t *req = (bfi_sfp_req_t *)sfp->mbcmd.msg;

	/* Should not be doing query if not in _INIT state */
	bfa_assert(sfp->state == BFA_SFP_STATE_INIT);
	bfa_assert(sfp->state_query_lock == 0);
	bfa_trc(sfp, sfp->state);

	sfp->state_query_lock = 1;
	req->memtype = 0;

	if (!sfp->lock) {
		bfa_sfp_getdata(sfp, BFI_SFP_MEM_ALL);
	}
	/* sfp show is going on, data will be ready */
}

static void
bfa_sfp_media_get(struct bfa_sfp_s *sfp)
{
	bfa_defs_sfp_media_t *media = sfp->media;
	sfp_mem_t *sfpmem = (sfp_mem_t *)sfp->dbuf_kva;

	*media = BFA_SFP_MEDIA_UNKNOWN;

	if (sfp->state == BFA_SFP_STATE_UNSUPPORT) {
		*media = BFA_SFP_MEDIA_UNSUPPORT;

	} else if (sfp->state == BFA_SFP_STATE_VALID) {
		sfp_xcvr_e10g_code_t e10g;
		uint16_t xmtr_tech = (sfpmem->srlid_base.xcvr[4] & 0x3) << 7 |
			(sfpmem->srlid_base.xcvr[5] >> 1);

		e10g.b = sfpmem->srlid_base.xcvr[0];
		/*
		 * check fc transmitter tech
		 */
		if ((xmtr_tech & SFP_XMTR_TECH_CU) ||
		    (xmtr_tech & SFP_XMTR_TECH_CP) ||
		    (xmtr_tech & SFP_XMTR_TECH_CA)) {
			*media = BFA_SFP_MEDIA_CU;
		} else if ((xmtr_tech & SFP_XMTR_TECH_EL_INTRA) ||
			 (xmtr_tech & SFP_XMTR_TECH_EL_INTER)) {
			*media = BFA_SFP_MEDIA_EL;
		} else if ((xmtr_tech & SFP_XMTR_TECH_LL) ||
			 (xmtr_tech & SFP_XMTR_TECH_LC)) {
			*media = BFA_SFP_MEDIA_LW;
		} else if ((xmtr_tech & SFP_XMTR_TECH_SL) ||
			 (xmtr_tech & SFP_XMTR_TECH_SN) ||
			 (xmtr_tech & SFP_XMTR_TECH_SA)) {
			*media = BFA_SFP_MEDIA_SW;
		/*
		 * Check 10G Ethernet Compilance code
		 */
		} else if (e10g.b & 0x10) {
			*media = BFA_SFP_MEDIA_SW;
		} else if (e10g.b & 0x60) {
			*media = BFA_SFP_MEDIA_LW;
		} else if (e10g.r.e10g_unall & 0x80) {
			*media = BFA_SFP_MEDIA_UNKNOWN;
		} else {
			bfa_trc(sfp, 0);
		}
	} else {
		bfa_trc(sfp, sfp->state);
	}
}

static bfa_status_t
bfa_sfp_speed_valid(struct bfa_sfp_s *sfp, bfa_port_speed_t portspeed)
{
	sfp_mem_t *sfpmem = (sfp_mem_t *)sfp->dbuf_kva;
	sfp_xcvr_t *xcvr = (sfp_xcvr_t *) sfpmem->srlid_base.xcvr;
	sfp_xcvr_fc3_code_t fc3 = xcvr->fc3;
	sfp_xcvr_e10g_code_t e10g = xcvr->e10g;

	if (portspeed == BFA_PORT_SPEED_10GBPS) {
		if (e10g.r.e10g_sr || e10g.r.e10g_lr) {
			return BFA_STATUS_OK;
		} else {
			bfa_trc(sfp, e10g.b);
			return BFA_STATUS_UNSUPP_SFP_SPEED;
		}
	}
	if (((portspeed & BFA_PORT_SPEED_16GBPS) && fc3.r.mb1600) ||
		   ((portspeed & BFA_PORT_SPEED_8GBPS) && fc3.r.mb800) ||
		   ((portspeed & BFA_PORT_SPEED_4GBPS) && fc3.r.mb400) ||
		   ((portspeed & BFA_PORT_SPEED_2GBPS) && fc3.r.mb200) ||
		   ((portspeed & BFA_PORT_SPEED_1GBPS) && fc3.r.mb100)) {
		return BFA_STATUS_OK;
	} else {
		bfa_trc(sfp, portspeed);
		bfa_trc(sfp, fc3.b);
		bfa_trc(sfp, e10g.b);
		return BFA_STATUS_UNSUPP_SFP_SPEED;
	}
}

/**
 * @}
 */

/**
 * @dg hal_sfp_public
 * @{
 */
/**
 * @ SFP hmbox handler
 */
void
bfa_sfp_intr(void *sfparg, struct bfi_mbmsg_s *msg)
{
	struct bfa_sfp_s *sfp = sfparg;

	switch (msg->mh.msg_id) {
	case BFI_SFP_I2H_SHOW:
		bfa_sfp_show_comp(sfp, msg);
		break;

	case BFI_SFP_I2H_SCN:
		bfa_sfp_scn(sfp, msg);
		break;

	default:
		bfa_trc(sfp, msg->mh.msg_id);
		bfa_assert(0);
	}
}


/**
 * @}
 */

/**
 * @ig hal_sfp_api
 *	API for driver layer
 * @{
 */

/**
 * @brief Return DMA memory needed by sfp module.
 */
uint32_t
bfa_sfp_meminfo(void)
{
	return (BFA_ROUNDUP(sizeof(sfp_mem_t), BFA_DMA_ALIGN_SZ));
}

/**
 * @brief Attach virtual and physical memory for SFP.
 */
void
bfa_sfp_attach(struct bfa_sfp_s *sfp, struct bfa_ioc_s *ioc, void *dev,
	       bfa_trc_mod_t *trcmod)
{
	sfp->dev = dev;
	sfp->ioc = ioc;
	sfp->trcmod = trcmod;

	sfp->cbfn = NULL;
	sfp->cbarg = NULL;
	sfp->sfpmem = NULL;
	sfp->lock = 0;
	sfp->data_valid = 0;
	sfp->state = BFA_SFP_STATE_INIT;
	sfp->state_query_lock = 0;
	sfp->state_query_cbfn = NULL;
	sfp->state_query_cbarg = NULL;
	sfp->media = NULL;
	sfp->portspeed = BFA_PORT_SPEED_UNKNOWN;
	sfp->is_elb = BFA_FALSE;

	bfa_ioc_mbox_regisr(sfp->ioc, BFI_MC_SFP, bfa_sfp_intr, sfp);
	bfa_q_qe_init(&sfp->ioc_notify);
	bfa_ioc_notify_init(&sfp->ioc_notify, bfa_sfp_notify, sfp);
	bfa_ioc_notify_register(sfp->ioc, &sfp->ioc_notify);
}

/**
 * @brief Claim Memory for SFP
 */
void
bfa_sfp_memclaim(struct bfa_sfp_s *sfp, uint8_t *dm_kva, uint64_t dm_pa)
{
	sfp->dbuf_kva	= dm_kva;
	sfp->dbuf_pa	= dm_pa;
	bfa_os_memset(sfp->dbuf_kva, 0, sizeof(sfp_mem_t));

	dm_kva += BFA_ROUNDUP(sizeof(sfp_mem_t), BFA_DMA_ALIGN_SZ);
	dm_pa += BFA_ROUNDUP(sizeof(sfp_mem_t), BFA_DMA_ALIGN_SZ);
}

/**
 * @brief Show SFP eeprom content
 *
 * @param[in] sfp   - bfa sfp module
 *
 * @param[out] sfpmem - sfp eeprom data
 *
 */
bfa_status_t
bfa_sfp_show(struct bfa_sfp_s *sfp, sfp_mem_t *sfpmem, bfa_cb_sfp_t cbfn,
	     void *cbarg)
{

	if (!bfa_ioc_is_operational(sfp->ioc)) {
		bfa_trc(sfp, 0);
		return BFA_STATUS_IOC_NON_OP;
	}

	if (sfp->lock) {
		bfa_trc(sfp, 0);
		return BFA_STATUS_DEVBUSY;
	}

	sfp->cbfn = cbfn;
	sfp->cbarg = cbarg;
	sfp->sfpmem = sfpmem;

	bfa_sfp_getdata(sfp, BFI_SFP_MEM_DIAGEXT);
	return BFA_STATUS_OK;
}

/**
 * @brief return SFP Media type
 *
 * @param[in] sfp   - bfa sfp module
 *
 * @param[out] media - port speed from user
 *
 */
bfa_status_t
bfa_sfp_media(struct bfa_sfp_s *sfp, bfa_defs_sfp_media_t *media,
	      bfa_cb_sfp_t cbfn, void *cbarg)
{
	if (!bfa_ioc_is_operational(sfp->ioc)) {
		bfa_trc(sfp, 0);
		return BFA_STATUS_IOC_NON_OP;
	}

	sfp->media = media;
	if (sfp->state == BFA_SFP_STATE_INIT) {
		if (sfp->state_query_lock) {
			bfa_trc(sfp, 0);
			return BFA_STATUS_DEVBUSY;
		} else {
			sfp->state_query_cbfn = cbfn;
			sfp->state_query_cbarg = cbarg;
			bfa_sfp_state_query(sfp);
			return BFA_STATUS_SFP_NOT_READY;
		}
	}

	bfa_sfp_media_get(sfp);
	return BFA_STATUS_OK;
}

/**
 * @brief Check if user set port speed is allowed by the SFP
 *
 * @param[in] sfp	- bfa sfp module
 * @param[in] portspeed - port speed from user
 *
 */
bfa_status_t
bfa_sfp_speed(struct bfa_sfp_s *sfp, bfa_port_speed_t portspeed,
	      bfa_cb_sfp_t cbfn, void *cbarg)
{
	bfa_assert(portspeed != BFA_PORT_SPEED_UNKNOWN);

	if (!bfa_ioc_is_operational(sfp->ioc))
		return BFA_STATUS_IOC_NON_OP;

	/* For Mezz card, all speed is allowed */
	if (bfa_mfg_is_mezz(sfp->ioc->attr->card_type))
		return BFA_STATUS_OK;

	/*
	 * Check SFP state
	 */
	sfp->portspeed = portspeed;
	if (sfp->state == BFA_SFP_STATE_INIT) {
		if (sfp->state_query_lock) {
			bfa_trc(sfp, 0);
			return BFA_STATUS_DEVBUSY;
		} else {
			sfp->state_query_cbfn = cbfn;
			sfp->state_query_cbarg = cbarg;
			bfa_sfp_state_query(sfp);
			return BFA_STATUS_SFP_NOT_READY;
		}
	}

	if (sfp->state == BFA_SFP_STATE_REMOVED ||
	    sfp->state == BFA_SFP_STATE_FAILED) {
		bfa_trc(sfp, sfp->state);
		return BFA_STATUS_NO_SFP_DEV;
	}

	if (sfp->state == BFA_SFP_STATE_INSERTED) {
		bfa_trc(sfp, sfp->state);
		return BFA_STATUS_DEVBUSY;  /* sfp is reading data */
	}

	/* For eloopback, all speed is allowed */
	if (sfp->is_elb)
		return BFA_STATUS_OK;

	return bfa_sfp_speed_valid(sfp, portspeed);
}

/**
 * @}
 */
