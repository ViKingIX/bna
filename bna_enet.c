/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#include "bna.h"
#include "cs/bfa_cs.h"
#include "cna/bfa_edma.h"

BNA_TRC_FILE(HAL, ENET);

static void
bna_err_handler(struct bna_s *bna, uint32_t intr_status)
{
	bfa_trc(bna, intr_status);
	if (BNA_IS_HALT_INTR(bna, intr_status))
		bna_halt_clear(bna);

	bfa_ioc_error_isr(&bna->ioceth.ioc);
}

void
bna_mbox_handler(struct bna_s *bna, uint32_t intr_status)
{
	if (BNA_IS_ERR_INTR(bna, intr_status)) {
		bna_err_handler(bna, intr_status);
		return;
	}
	if (BNA_IS_MBOX_INTR(bna, intr_status))
		bfa_ioc_mbox_isr(&bna->ioceth.ioc);
}

static void
bna_msgq_rsp_handler(void *arg, struct bfi_msgq_mhdr_s *msghdr)
{
	struct bna_s *bna = (struct bna_s *)arg;
	struct bna_tx_s *tx;
	struct bna_rx_s *rx;

	bfa_trc(bna, msghdr->msg_id);
	bfa_trc(bna, msghdr->enet_id);

	switch (msghdr->msg_id) {
	case BFI_ENET_I2H_RX_CFG_SET_RSP:
		bna_rx_from_rid(bna, msghdr->enet_id, rx);
		if (rx)
			bna_bfi_rx_enet_start_rsp(rx, msghdr);
		break;

	case BFI_ENET_I2H_RX_CFG_CLR_RSP:
		bna_rx_from_rid(bna, msghdr->enet_id, rx);
		if (rx)
			bna_bfi_rx_enet_stop_rsp(rx, msghdr);
		break;

	case BFI_ENET_I2H_RIT_CFG_RSP:
	case BFI_ENET_I2H_RSS_CFG_RSP:
	case BFI_ENET_I2H_RSS_ENABLE_RSP:
	case BFI_ENET_I2H_RX_PROMISCUOUS_RSP:
	case BFI_ENET_I2H_RX_DEFAULT_RSP:
	case BFI_ENET_I2H_MAC_UCAST_CLR_RSP:
	case BFI_ENET_I2H_MAC_UCAST_ADD_RSP:
	case BFI_ENET_I2H_MAC_UCAST_DEL_RSP:
	case BFI_ENET_I2H_MAC_MCAST_DEL_RSP:
	case BFI_ENET_I2H_MAC_MCAST_FILTER_RSP:
	case BFI_ENET_I2H_RX_VLAN_SET_RSP:
	case BFI_ENET_I2H_RX_VLAN_STRIP_ENABLE_RSP:
	case BFI_ENET_I2H_WOL_MAGIC_RSP:
	case BFI_ENET_I2H_WOL_FRAME_RSP:
		bna_rx_from_rid(bna, msghdr->enet_id, rx);
		if (rx)
			bna_bfi_rxf_cfg_rsp(&rx->rxf, msghdr);
		break;

	case BFI_ENET_I2H_MAC_UCAST_SET_RSP:
		bna_rx_from_rid(bna, msghdr->enet_id, rx);
		if (rx)
			bna_bfi_rxf_ucast_set_rsp(&rx->rxf, msghdr);
		break;

	case BFI_ENET_I2H_MAC_MCAST_ADD_RSP:
		bna_rx_from_rid(bna, msghdr->enet_id, rx);
		if (rx)
			bna_bfi_rxf_mcast_add_rsp(&rx->rxf, msghdr);
		break;

	case BFI_ENET_I2H_TX_CFG_SET_RSP:
		bna_tx_from_rid(bna, msghdr->enet_id, tx);
		if (tx)
			bna_bfi_tx_enet_start_rsp(tx, msghdr);
		break;

	case BFI_ENET_I2H_TX_CFG_CLR_RSP:
		bna_tx_from_rid(bna, msghdr->enet_id, tx);
		if (tx)
			bna_bfi_tx_enet_stop_rsp(tx, msghdr);
		break;

	case BFI_ENET_I2H_PORT_ADMIN_RSP:
		bna_bfi_ethport_admin_rsp(&bna->ethport, msghdr);
		break;

	case BFI_ENET_I2H_DIAG_LOOPBACK_RSP:
		bna_bfi_ethport_lpbk_rsp(&bna->ethport, msghdr);
		break;

	case BFI_ENET_I2H_SET_PAUSE_RSP:
		bna_bfi_pause_set_rsp(&bna->enet, msghdr);
		break;

	case BFI_ENET_I2H_GET_ATTR_RSP:
		bna_bfi_attr_get_rsp(&bna->ioceth, msghdr);
		break;

	case BFI_ENET_I2H_STATS_GET_RSP:
		bna_bfi_stats_get_rsp(bna, msghdr);
		break;

	case BFI_ENET_I2H_STATS_CLR_RSP:
		bna_bfi_stats_clr_rsp(bna, msghdr);
		break;

	case BFI_ENET_I2H_LINK_UP_AEN:
		bna_bfi_ethport_linkup_aen(&bna->ethport, msghdr);
		break;

	case BFI_ENET_I2H_LINK_DOWN_AEN:
		bna_bfi_ethport_linkdown_aen(&bna->ethport, msghdr);
		break;

	case BFI_ENET_I2H_PORT_ENABLE_AEN:
		bna_bfi_ethport_enable_aen(&bna->ethport, msghdr);
		break;

	case BFI_ENET_I2H_PORT_DISABLE_AEN:
		bna_bfi_ethport_disable_aen(&bna->ethport, msghdr);
		break;

	case BFI_ENET_I2H_BW_UPDATE_AEN:
		bna_bfi_bw_update_aen(&bna->enet, msghdr);
		break;

	default:
		break;
	}
}

/**
 * ETHPORT
 */

#define call_ethport_stop_cbfn(_ethport)				\
do {									\
	if ((_ethport)->stop_cbfn) {					\
		void (*cbfn)(struct bna_enet_s *);			\
		cbfn = (_ethport)->stop_cbfn;				\
		(_ethport)->stop_cbfn = NULL;				\
		cbfn(&(_ethport)->bna->enet);				\
	}								\
} while (0)

#define call_ethport_adminup_cbfn(ethport, status)			\
do {									\
	if ((ethport)->adminup_cbfn) {					\
		void (*cbfn)(struct bnad_s *, enum bna_cb_status_e);	\
		cbfn = (ethport)->adminup_cbfn;				\
		(ethport)->adminup_cbfn = NULL;				\
		cbfn((ethport)->bna->bnad, status);			\
	}								\
} while (0)

static inline int
ethport_can_be_up(struct bna_ethport_s *ethport)
{
	int ready = 0;
	if (ethport->bna->enet.type == BNA_ENET_T_REGULAR)
		ready = ((ethport->flags & BNA_ETHPORT_F_ADMIN_UP) &&
			 (ethport->flags & BNA_ETHPORT_F_RX_STARTED) &&
			 (ethport->flags & BNA_ETHPORT_F_PORT_ENABLED));
	else
		ready = ((ethport->flags & BNA_ETHPORT_F_ADMIN_UP) &&
			 (ethport->flags & BNA_ETHPORT_F_RX_STARTED) &&
			 !(ethport->flags & BNA_ETHPORT_F_PORT_ENABLED));
	return (ready);
}

#define ethport_is_up ethport_can_be_up

static void bna_bfi_ethport_up(struct bna_ethport_s *ethport);
static void bna_bfi_ethport_down(struct bna_ethport_s *ethport);
static void bna_ethport_aen_post(struct bna_s *bna,
			enum bfa_ethport_aen_event event);


enum bna_ethport_event_e {
	ETHPORT_E_START			= 1,
	ETHPORT_E_STOP			= 2,
	ETHPORT_E_FAIL			= 3,
	ETHPORT_E_UP			= 4,
	ETHPORT_E_DOWN			= 5,
	ETHPORT_E_FWRESP_UP_OK		= 6,
	ETHPORT_E_FWRESP_DOWN		= 7,
	ETHPORT_E_FWRESP_UP_FAIL	= 8,
};

bfa_fsm_state_decl(bna_ethport, stopped, struct bna_ethport_s,
			enum bna_ethport_event_e);
bfa_fsm_state_decl(bna_ethport, down, struct bna_ethport_s,
			enum bna_ethport_event_e);
bfa_fsm_state_decl(bna_ethport, up_resp_wait, struct bna_ethport_s,
			enum bna_ethport_event_e);
bfa_fsm_state_decl(bna_ethport, down_resp_wait, struct bna_ethport_s,
			enum bna_ethport_event_e);
bfa_fsm_state_decl(bna_ethport, up, struct bna_ethport_s,
			enum bna_ethport_event_e);
bfa_fsm_state_decl(bna_ethport, last_resp_wait, struct bna_ethport_s,
			enum bna_ethport_event_e);

static void
bna_ethport_sm_stopped_entry(struct bna_ethport_s *ethport)
{
	bfa_trc(ethport->bna, 0);
	call_ethport_stop_cbfn(ethport);
}

static void
bna_ethport_sm_stopped(struct bna_ethport_s *ethport,
			enum bna_ethport_event_e event)
{
	bfa_trc(ethport->bna, event);
	switch (event) {
	case ETHPORT_E_START:
		bfa_fsm_set_state(ethport, bna_ethport_sm_down);
		break;

	case ETHPORT_E_STOP:
		call_ethport_stop_cbfn(ethport);
		break;

	case ETHPORT_E_FAIL:
		/* No-op */
		break;

	case ETHPORT_E_DOWN:
		/* This event is received due to Rx objects failing */
		/* No-op */
		break;

	default:
		bfa_sm_fault(ethport->bna, event);
	}
}

static void
bna_ethport_sm_down_entry(struct bna_ethport_s *ethport)
{
	bfa_trc(ethport->bna, 0);
}

static void
bna_ethport_sm_down(struct bna_ethport_s *ethport,
			enum bna_ethport_event_e event)
{
	bfa_trc(ethport->bna, event);
	switch (event) {
	case ETHPORT_E_STOP:
		bfa_fsm_set_state(ethport, bna_ethport_sm_stopped);
		break;

	case ETHPORT_E_FAIL:
		bfa_fsm_set_state(ethport, bna_ethport_sm_stopped);
		break;

	case ETHPORT_E_UP:
		bfa_fsm_set_state(ethport, bna_ethport_sm_up_resp_wait);
		bna_bfi_ethport_up(ethport);
		break;

	default:
		bfa_sm_fault(ethport->bna, event);
	}
}

static void
bna_ethport_sm_up_resp_wait_entry(struct bna_ethport_s *ethport)
{
	bfa_trc(ethport->bna, 0);
	CNA_ASSERT(ethport_can_be_up(ethport));

	/**
	 * NOTE: Do not call bna_bfi_ethport_up() here. That will over step
	 * mbox due to down_resp_wait -> up_resp_wait transition on event
	 * ETHPORT_E_UP
	 */
}

static void
bna_ethport_sm_up_resp_wait(struct bna_ethport_s *ethport,
			enum bna_ethport_event_e event)
{
	bfa_trc(ethport->bna, event);
	switch (event) {
	case ETHPORT_E_STOP:
		bfa_fsm_set_state(ethport, bna_ethport_sm_last_resp_wait);
		break;

	case ETHPORT_E_FAIL:
		call_ethport_adminup_cbfn(ethport, BNA_CB_FAIL);
		bfa_fsm_set_state(ethport, bna_ethport_sm_stopped);
		break;

	case ETHPORT_E_DOWN:
		call_ethport_adminup_cbfn(ethport, BNA_CB_INTERRUPT);
		bfa_fsm_set_state(ethport, bna_ethport_sm_down_resp_wait);
		break;

	case ETHPORT_E_FWRESP_UP_OK:
		call_ethport_adminup_cbfn(ethport, BNA_CB_SUCCESS);
		bfa_fsm_set_state(ethport, bna_ethport_sm_up);
		break;

	case ETHPORT_E_FWRESP_UP_FAIL:
		call_ethport_adminup_cbfn(ethport, BNA_CB_FAIL);
		bfa_fsm_set_state(ethport, bna_ethport_sm_down);
		break;

	case ETHPORT_E_FWRESP_DOWN:
		/* down_resp_wait -> up_resp_wait transition on ETHPORT_E_UP */
		bna_bfi_ethport_up(ethport);
		break;

	default:
		bfa_sm_fault(ethport->bna, event);
	}
}

static void
bna_ethport_sm_down_resp_wait_entry(struct bna_ethport_s *ethport)
{
	bfa_trc(ethport->bna, 0);
	/**
	 * NOTE: Do not call bna_bfi_ethport_down() here. That will over step
	 * mbox due to up_resp_wait -> down_resp_wait transition on event
	 * ETHPORT_E_DOWN
	 */
}

static void
bna_ethport_sm_down_resp_wait(struct bna_ethport_s *ethport,
			enum bna_ethport_event_e event)
{
	bfa_trc(ethport->bna, event);
	switch (event) {
	case ETHPORT_E_STOP:
		bfa_fsm_set_state(ethport, bna_ethport_sm_last_resp_wait);
		break;

	case ETHPORT_E_FAIL:
		bfa_fsm_set_state(ethport, bna_ethport_sm_stopped);
		break;

	case ETHPORT_E_UP:
		bfa_fsm_set_state(ethport, bna_ethport_sm_up_resp_wait);
		break;

	case ETHPORT_E_FWRESP_UP_OK:
		/* up_resp_wait->down_resp_wait transition on ETHPORT_E_DOWN */
		bna_bfi_ethport_down(ethport);
		break;

	case ETHPORT_E_FWRESP_UP_FAIL:
	case ETHPORT_E_FWRESP_DOWN:
		bfa_fsm_set_state(ethport, bna_ethport_sm_down);
		break;

	default:
		bfa_sm_fault(ethport->bna, event);
	}
}

static void
bna_ethport_sm_up_entry(struct bna_ethport_s *ethport)
{
	bfa_trc(ethport->bna, 0);
}

static void
bna_ethport_sm_up(struct bna_ethport_s *ethport,
			enum bna_ethport_event_e event)
{
	bfa_trc(ethport->bna, event);
	switch (event) {
	case ETHPORT_E_STOP:
		bfa_fsm_set_state(ethport, bna_ethport_sm_last_resp_wait);
		bna_bfi_ethport_down(ethport);
		break;

	case ETHPORT_E_FAIL:
		bfa_fsm_set_state(ethport, bna_ethport_sm_stopped);
		break;

	case ETHPORT_E_DOWN:
		bfa_fsm_set_state(ethport, bna_ethport_sm_down_resp_wait);
		bna_bfi_ethport_down(ethport);
		break;

	default:
		bfa_sm_fault(ethport->bna, event);
	}
}

static void
bna_ethport_sm_last_resp_wait_entry(struct bna_ethport_s *ethport)
{
	bfa_trc(ethport->bna, 0);
}

static void
bna_ethport_sm_last_resp_wait(struct bna_ethport_s *ethport,
			enum bna_ethport_event_e event)
{
	bfa_trc(ethport->bna, event);
	switch (event) {
	case ETHPORT_E_FAIL:
		bfa_fsm_set_state(ethport, bna_ethport_sm_stopped);
		break;

	case ETHPORT_E_DOWN:
		/**
		 * This event is received due to Rx objects stopping in
		 * parallel to ethport
		 */
		/* No-op */
		break;

	case ETHPORT_E_FWRESP_UP_OK:
		/* up_resp_wait->last_resp_wait transition on ETHPORT_T_STOP */
		bna_bfi_ethport_down(ethport);
		break;

	case ETHPORT_E_FWRESP_UP_FAIL:
	case ETHPORT_E_FWRESP_DOWN:
		bfa_fsm_set_state(ethport, bna_ethport_sm_stopped);
		break;

	default:
		bfa_sm_fault(ethport->bna, event);
	}
}

static void
bna_bfi_ethport_admin_up(struct bna_ethport_s *ethport)
{
	struct bfi_enet_enable_req *admin_up_req =
		&ethport->bfi_enet_cmd.admin_req;

	bfi_msgq_mhdr_set(admin_up_req->mh, BFI_MC_ENET,
		BFI_ENET_H2I_PORT_ADMIN_UP_REQ, 0, 0);
	admin_up_req->mh.num_entries = cna_os_htons(
		bfi_msgq_num_cmd_entries(sizeof(struct bfi_enet_enable_req)));
	admin_up_req->enable = BNA_STATUS_T_ENABLED;

	bfa_msgq_cmd_set(&ethport->msgq_cmd, NULL, NULL,
		sizeof(struct bfi_enet_enable_req), &admin_up_req->mh);
	bfa_msgq_cmd_post(&ethport->bna->msgq, &ethport->msgq_cmd);
}

static void
bna_bfi_ethport_admin_down(struct bna_ethport_s *ethport)
{
	struct bfi_enet_enable_req *admin_down_req =
		&ethport->bfi_enet_cmd.admin_req;

	bfi_msgq_mhdr_set(admin_down_req->mh, BFI_MC_ENET,
		BFI_ENET_H2I_PORT_ADMIN_UP_REQ, 0, 0);
	admin_down_req->mh.num_entries = cna_os_htons(
		bfi_msgq_num_cmd_entries(sizeof(struct bfi_enet_enable_req)));
	admin_down_req->enable = BNA_STATUS_T_DISABLED;

	bfa_msgq_cmd_set(&ethport->msgq_cmd, NULL, NULL,
		sizeof(struct bfi_enet_enable_req), &admin_down_req->mh);
	bfa_msgq_cmd_post(&ethport->bna->msgq, &ethport->msgq_cmd);
}

static void
bna_bfi_ethport_lpbk_up(struct bna_ethport_s *ethport)
{
	struct bfi_enet_diag_lb_req *lpbk_up_req =
		&ethport->bfi_enet_cmd.lpbk_req;

	bfi_msgq_mhdr_set(lpbk_up_req->mh, BFI_MC_ENET,
		BFI_ENET_H2I_DIAG_LOOPBACK_REQ, 0, 0);
	lpbk_up_req->mh.num_entries = cna_os_htons(
		bfi_msgq_num_cmd_entries(sizeof(struct bfi_enet_diag_lb_req)));
	lpbk_up_req->mode = (ethport->bna->enet.type ==
				BNA_ENET_T_LOOPBACK_INTERNAL) ?
				BFI_ENET_DIAG_LB_OPMODE_EXT :
				BFI_ENET_DIAG_LB_OPMODE_CBL;
	lpbk_up_req->enable = BNA_STATUS_T_ENABLED;

	bfa_msgq_cmd_set(&ethport->msgq_cmd, NULL, NULL,
		sizeof(struct bfi_enet_diag_lb_req), &lpbk_up_req->mh);
	bfa_msgq_cmd_post(&ethport->bna->msgq, &ethport->msgq_cmd);
}

static void
bna_bfi_ethport_lpbk_down(struct bna_ethport_s *ethport)
{
	struct bfi_enet_diag_lb_req *lpbk_down_req =
		&ethport->bfi_enet_cmd.lpbk_req;

	bfi_msgq_mhdr_set(lpbk_down_req->mh, BFI_MC_ENET,
		BFI_ENET_H2I_DIAG_LOOPBACK_REQ, 0, 0);
	lpbk_down_req->mh.num_entries = cna_os_htons(
		bfi_msgq_num_cmd_entries(sizeof(struct bfi_enet_diag_lb_req)));
	lpbk_down_req->enable = BNA_STATUS_T_DISABLED;

	bfa_msgq_cmd_set(&ethport->msgq_cmd, NULL, NULL,
		sizeof(struct bfi_enet_diag_lb_req), &lpbk_down_req->mh);
	bfa_msgq_cmd_post(&ethport->bna->msgq, &ethport->msgq_cmd);
}

static void
bna_bfi_ethport_up(struct bna_ethport_s *ethport)
{
	if (ethport->bna->enet.type == BNA_ENET_T_REGULAR)
		bna_bfi_ethport_admin_up(ethport);
	else
		bna_bfi_ethport_lpbk_up(ethport);
}

static void
bna_bfi_ethport_down(struct bna_ethport_s *ethport)
{
	if (ethport->bna->enet.type == BNA_ENET_T_REGULAR)
		bna_bfi_ethport_admin_down(ethport);
	else
		bna_bfi_ethport_lpbk_down(ethport);
}

static void
bna_ethport_aen_post(struct bna_s *bna, enum bfa_ethport_aen_event event)
{
	union bfa_aen_data_u aen_data;
	char	buf[BFA_STRING_32];
	char	*buf_ptr = NULL;


	aen_data.ethport.mac = bfa_ioc_get_mac(&bna->ioceth.ioc);

	if (event == BFA_ETHPORT_AEN_LINKUP) {
		if (bna->ethport.link_status == BNA_CEE_UP)
			aen_data.ethport.cee_state = BFA_ETHPORT_CEE_LINKUP;
		else
			aen_data.ethport.cee_state = BFA_ETHPORT_CEE_LINKDOWN;
	} else if ((event == BFA_ETHPORT_AEN_ENABLE) ||
			(event == BFA_ETHPORT_AEN_DISABLE)) {
		buf_ptr = mac2str(buf, sizeof(buf), aen_data.ethport.mac);
		bfa_log(bna->logm,
			BFA_LOG_CREATE_ID(BFA_AEN_CAT_ETHPORT, event),
			buf_ptr);
	}

	bfa_aen_post(bna->ioceth.ioc.aen,
			BFA_AEN_CAT_ETHPORT, event, &aen_data);
}

void
bna_ethport_init(struct bna_ethport_s *ethport, struct bna_s *bna)
{
	ethport->flags |= (BNA_ETHPORT_F_ADMIN_UP | BNA_ETHPORT_F_PORT_ENABLED);
	ethport->bna = bna;

	ethport->link_status = BNA_LINK_DOWN;
	ethport->link_cbfn = bnad_cb_ethport_link_status;

	ethport->rx_started_count = 0;

	ethport->stop_cbfn = NULL;
	ethport->adminup_cbfn = NULL;

	bfa_fsm_set_state(ethport, bna_ethport_sm_stopped);
}

void
bna_ethport_uninit(struct bna_ethport_s *ethport)
{
	ethport->flags &= ~BNA_ETHPORT_F_ADMIN_UP;
	ethport->flags &= ~BNA_ETHPORT_F_PORT_ENABLED;

	CNA_ASSERT(ethport->fsm == (bfa_fsm_t)bna_ethport_sm_stopped);
	CNA_ASSERT(ethport->rx_started_count == 0);
	CNA_ASSERT(ethport->stop_cbfn == NULL);
	CNA_ASSERT(ethport->adminup_cbfn == NULL);
	CNA_ASSERT(ethport->flags == 0);

	ethport->bna = NULL;
}

void
bna_ethport_start(struct bna_ethport_s *ethport)
{
	CNA_ASSERT(ethport->fsm == (bfa_fsm_t)bna_ethport_sm_stopped);
	bfa_trc(ethport->bna, 0);
	bfa_fsm_send_event(ethport, ETHPORT_E_START);
}

void
bna_ethport_stop(struct bna_ethport_s *ethport)
{
	bfa_trc(ethport->bna, 0);
	CNA_ASSERT(ethport->stop_cbfn == NULL);
	ethport->stop_cbfn = bna_enet_cb_ethport_stopped;
	bfa_fsm_send_event(ethport, ETHPORT_E_STOP);
}

void
bna_ethport_fail(struct bna_ethport_s *ethport)
{
	bfa_trc(ethport->bna, 0);
	/* Reset the physical port status to enabled */
	ethport->flags |= BNA_ETHPORT_F_PORT_ENABLED;

	if (ethport->link_status != BNA_LINK_DOWN) {
		ethport->link_status = BNA_LINK_DOWN;
		ethport->link_cbfn(ethport->bna->bnad, BNA_LINK_DOWN);
	}
	bfa_fsm_send_event(ethport, ETHPORT_E_FAIL);
}

void
bna_ethport_admin_up(struct bna_ethport_s *ethport,
		void (*callback)(struct bnad_s *, enum bna_cb_status_e))
{
	CNA_ASSERT(ethport->adminup_cbfn == NULL);
	ethport->adminup_cbfn = callback;

	if (ethport->flags & BNA_ETHPORT_F_ADMIN_UP) {
		call_ethport_adminup_cbfn(ethport, BNA_CB_SUCCESS);
		return;
	}

	if ((ethport->bna->enet.type != BNA_ENET_T_REGULAR) &&
		(ethport->flags & BNA_ETHPORT_F_PORT_ENABLED)) {
		call_ethport_adminup_cbfn(ethport, BNA_CB_FAIL);
		return;
	}

	ethport->flags |= BNA_ETHPORT_F_ADMIN_UP;
	bfa_trc(ethport->bna, ethport->flags);

	if (ethport_can_be_up(ethport)) {
		CNA_ASSERT((ethport->fsm ==
			(bfa_fsm_t)bna_ethport_sm_down_resp_wait) ||
			(ethport->fsm == (bfa_fsm_t)bna_ethport_sm_down));
		bfa_fsm_send_event(ethport, ETHPORT_E_UP);
	}
}

void
bna_ethport_admin_down(struct bna_ethport_s *ethport)
{
	int ethport_up = ethport_is_up(ethport);

	if (!(ethport->flags & BNA_ETHPORT_F_ADMIN_UP))
		return;

	ethport->flags &= ~BNA_ETHPORT_F_ADMIN_UP;
	bfa_trc(ethport->bna, ethport->flags);

	if (ethport_up) {
		CNA_ASSERT((ethport->fsm ==
			(bfa_fsm_t)bna_ethport_sm_up_resp_wait) ||
			(ethport->fsm == (bfa_fsm_t)bna_ethport_sm_up));
		bfa_fsm_send_event(ethport, ETHPORT_E_DOWN);
	}
}

/* Should be called only when ethport is disabled */
void
bna_ethport_linkcbfn_set(struct bna_ethport_s *ethport,
		      void (*linkcbfn)(struct bnad_s *, enum bna_link_status_e))
{
	CNA_ASSERT(ethport->fsm == (bfa_fsm_t)bna_ethport_sm_stopped);
	ethport->link_cbfn = linkcbfn;
}

void
bna_bfi_ethport_enable_aen(struct bna_ethport_s *ethport,
				struct bfi_msgq_mhdr_s *msghdr)
{
	ethport->flags |= BNA_ETHPORT_F_PORT_ENABLED;
	bfa_trc(ethport->bna, ethport->flags);

	if (ethport_can_be_up(ethport))
		bfa_fsm_send_event(ethport, ETHPORT_E_UP);
}

void
bna_bfi_ethport_disable_aen(struct bna_ethport_s *ethport,
				struct bfi_msgq_mhdr_s *msghdr)
{
	int ethport_up = ethport_is_up(ethport);

	ethport->flags &= ~BNA_ETHPORT_F_PORT_ENABLED;
	bfa_trc(ethport->bna, ethport->flags);

	if (ethport_up)
		bfa_fsm_send_event(ethport, ETHPORT_E_DOWN);
}

void
bna_ethport_cb_rx_started(struct bna_ethport_s *ethport)
{
	ethport->rx_started_count++;

	if (ethport->rx_started_count == 1) {

		ethport->flags |= BNA_ETHPORT_F_RX_STARTED;
		bfa_trc(ethport->bna, ethport->flags);

		if (ethport_can_be_up(ethport))
			bfa_fsm_send_event(ethport, ETHPORT_E_UP);
	}
}

void
bna_ethport_cb_rx_stopped(struct bna_ethport_s *ethport)
{
	int ethport_up = ethport_is_up(ethport);

	CNA_ASSERT(ethport->rx_started_count > 0);
	ethport->rx_started_count--;

	if (ethport->rx_started_count == 0) {

		ethport->flags &= ~BNA_ETHPORT_F_RX_STARTED;
		bfa_trc(ethport->bna, ethport->flags);

		if (ethport_up)
			bfa_fsm_send_event(ethport, ETHPORT_E_DOWN);
	}
}

void
bna_bfi_ethport_admin_rsp(struct bna_ethport_s *ethport,
				struct bfi_msgq_mhdr_s *msghdr)
{
	struct bfi_enet_enable_req *admin_req =
		&ethport->bfi_enet_cmd.admin_req;
	struct bfi_enet_rsp *rsp = (struct bfi_enet_rsp *)msghdr;

	switch (admin_req->enable) {
	case BNA_STATUS_T_ENABLED:
		if (rsp->error == BFI_ENET_CMD_OK) {
			bfa_fsm_send_event(ethport, ETHPORT_E_FWRESP_UP_OK);
			bna_ethport_aen_post(ethport->bna,
				BFA_ETHPORT_AEN_ENABLE);
		} else {
			ethport->flags &= ~BNA_ETHPORT_F_PORT_ENABLED;
			bfa_fsm_send_event(ethport, ETHPORT_E_FWRESP_UP_FAIL);
		}
		break;

	case BNA_STATUS_T_DISABLED:
		CNA_ASSERT(rsp->error == BFI_ENET_CMD_OK);
		bfa_fsm_send_event(ethport, ETHPORT_E_FWRESP_DOWN);
		ethport->link_status = BNA_LINK_DOWN;
		ethport->link_cbfn(ethport->bna->bnad, BNA_LINK_DOWN);
		bna_ethport_aen_post(ethport->bna, BFA_ETHPORT_AEN_DISABLE);
		break;
	}
}

void
bna_bfi_ethport_lpbk_rsp(struct bna_ethport_s *ethport,
				struct bfi_msgq_mhdr_s *msghdr)
{
	struct bfi_enet_diag_lb_req *diag_lb_req =
		&ethport->bfi_enet_cmd.lpbk_req;
	struct bfi_enet_rsp *rsp = (struct bfi_enet_rsp *)msghdr;

	switch (diag_lb_req->enable) {
	case BNA_STATUS_T_ENABLED:
		if (rsp->error == BFI_ENET_CMD_OK) {
			bfa_fsm_send_event(ethport, ETHPORT_E_FWRESP_UP_OK);
		} else {
			ethport->flags &= ~BNA_ETHPORT_F_ADMIN_UP;
			bfa_fsm_send_event(ethport, ETHPORT_E_FWRESP_UP_FAIL);
		}
		break;

	case BNA_STATUS_T_DISABLED:
		CNA_ASSERT(rsp->error == BFI_ENET_CMD_OK);
		bfa_fsm_send_event(ethport, ETHPORT_E_FWRESP_DOWN);
		break;
	}
}

void
bna_bfi_ethport_linkup_aen(struct bna_ethport_s *ethport,
			struct bfi_msgq_mhdr_s *msghdr)
{
	struct bfi_enet_aen *aen = (struct bfi_enet_aen *)msghdr;

	bfa_trc(ethport->bna, ethport->link_status);
	ethport->link_status = BNA_LINK_UP;
	if (aen->cee_linkup)
		ethport->link_status = BNA_CEE_UP;

	/* Copy the dcbx values */
	/* Note: This should form the remote QOS params to be reported to OS */
	bfa_os_memcpy(&ethport->bna->tx_mod.dcb_cfg,
				&aen->dcb_cfg,
				sizeof(bfa_cee_dcbx_cfg_t));

	/* Dispatch events */
	bna_tx_mod_prio_reconfig(&ethport->bna->tx_mod, aen->cee_linkup,
		aen->prio_map, aen->iscsi_prio_map);
	ethport->link_cbfn(ethport->bna->bnad, ethport->link_status);
	bna_ethport_aen_post(ethport->bna, BFA_ETHPORT_AEN_LINKUP);
}

void
bna_bfi_ethport_linkdown_aen(struct bna_ethport_s *ethport,
				struct bfi_msgq_mhdr_s *msghdr)
{
	bfa_trc(ethport->bna, ethport->link_status);
	ethport->link_status = BNA_LINK_DOWN;

	/* Dispatch events */
	ethport->link_cbfn(ethport->bna->bnad, BNA_LINK_DOWN);
	bna_ethport_aen_post(ethport->bna, BFA_ETHPORT_AEN_LINKDOWN);
}

int
bna_ethport_is_disabled(struct bna_ethport_s *ethport)
{
	return (!(ethport->flags & BNA_ETHPORT_F_PORT_ENABLED));
}

/**
 * ENET
 */

#define bna_enet_chld_start(enet)					\
do {									\
	enum bna_tx_type_e tx_type =					\
		((enet)->type == BNA_ENET_T_REGULAR) ?			\
		BNA_TX_T_REGULAR : BNA_TX_T_LOOPBACK;			\
	enum bna_rx_type_e rx_type =					\
		((enet)->type == BNA_ENET_T_REGULAR) ?			\
		BNA_RX_T_REGULAR : BNA_RX_T_LOOPBACK;			\
	bna_ethport_start(&(enet)->bna->ethport);			\
	bna_tx_mod_start(&(enet)->bna->tx_mod, tx_type);		\
	bna_rx_mod_start(&(enet)->bna->rx_mod, rx_type);		\
} while (0)

#define bna_enet_chld_stop(enet)					\
do {									\
	enum bna_tx_type_e tx_type =					\
		((enet)->type == BNA_ENET_T_REGULAR) ?			\
		BNA_TX_T_REGULAR : BNA_TX_T_LOOPBACK;			\
	enum bna_rx_type_e rx_type =					\
		((enet)->type == BNA_ENET_T_REGULAR) ?			\
		BNA_RX_T_REGULAR : BNA_RX_T_LOOPBACK;			\
	CNA_ASSERT((enet)->chld_stop_wc.wc_count == 0);			\
	bfa_wc_init(&(enet)->chld_stop_wc, bna_enet_cb_chld_stopped, (enet));\
	bfa_wc_up(&(enet)->chld_stop_wc);				\
	bna_ethport_stop(&(enet)->bna->ethport);			\
	bfa_wc_up(&(enet)->chld_stop_wc);				\
	bna_tx_mod_stop(&(enet)->bna->tx_mod, tx_type);			\
	bfa_wc_up(&(enet)->chld_stop_wc);				\
	bna_rx_mod_stop(&(enet)->bna->rx_mod, rx_type);			\
	bfa_wc_wait(&(enet)->chld_stop_wc);				\
} while (0)

#define bna_enet_chld_fail(enet)					\
do {									\
	bna_ethport_fail(&(enet)->bna->ethport);			\
	bna_tx_mod_fail(&(enet)->bna->tx_mod);				\
	bna_rx_mod_fail(&(enet)->bna->rx_mod);				\
} while (0)

#define bna_enet_rx_start(enet)						\
do {									\
	enum bna_rx_type_e rx_type =					\
		((enet)->type == BNA_ENET_T_REGULAR) ?			\
		BNA_RX_T_REGULAR : BNA_RX_T_LOOPBACK;			\
	bna_rx_mod_start(&(enet)->bna->rx_mod, rx_type);		\
} while (0)

#define bna_enet_rx_stop(enet)						\
do {									\
	enum bna_rx_type_e rx_type =					\
		((enet)->type == BNA_ENET_T_REGULAR) ?			\
		BNA_RX_T_REGULAR : BNA_RX_T_LOOPBACK;			\
	CNA_ASSERT((enet)->chld_stop_wc.wc_count == 0);			\
	bfa_wc_init(&(enet)->chld_stop_wc, bna_enet_cb_chld_stopped, (enet));\
	bfa_wc_up(&(enet)->chld_stop_wc);				\
	bna_rx_mod_stop(&(enet)->bna->rx_mod, rx_type);			\
	bfa_wc_wait(&(enet)->chld_stop_wc);				\
} while (0)

#define call_enet_stop_cbfn(enet)					\
do {									\
	if ((enet)->stop_cbfn) {					\
		void (*cbfn)(void *);					\
		void *cbarg;						\
		cbfn = (enet)->stop_cbfn;				\
		cbarg = (enet)->stop_cbarg;				\
		(enet)->stop_cbfn = NULL;				\
		(enet)->stop_cbarg = NULL;				\
		cbfn(cbarg);						\
	}								\
} while (0)

#define call_enet_pause_cbfn(enet)					\
do {									\
	if ((enet)->pause_cbfn) {					\
		void (*cbfn)(struct bnad_s *);				\
		cbfn = (enet)->pause_cbfn;				\
		(enet)->pause_cbfn = NULL;				\
		cbfn((enet)->bna->bnad);				\
	}								\
} while (0)

#define call_enet_mtu_cbfn(enet)					\
do {									\
	if ((enet)->mtu_cbfn) {						\
		void (*cbfn)(struct bnad_s *);				\
		cbfn = (enet)->mtu_cbfn;				\
		(enet)->mtu_cbfn = NULL;				\
		cbfn((enet)->bna->bnad);				\
	}								\
} while (0)

static void bna_enet_cb_chld_stopped(void *arg);
static void bna_bfi_pause_set(struct bna_enet_s *enet);

enum bna_enet_event_e {
	ENET_E_START			= 1,
	ENET_E_STOP			= 2,
	ENET_E_FAIL			= 3,
	ENET_E_PAUSE_CFG		= 4,
	ENET_E_MTU_CFG			= 5,
	ENET_E_FWRESP_PAUSE		= 6,
	ENET_E_CHLD_STOPPED		= 7,
};

bfa_fsm_state_decl(bna_enet, stopped, struct bna_enet_s,
			enum bna_enet_event_e);
bfa_fsm_state_decl(bna_enet, pause_init_wait, struct bna_enet_s,
			enum bna_enet_event_e);
bfa_fsm_state_decl(bna_enet, last_resp_wait, struct bna_enet_s,
			enum bna_enet_event_e);
bfa_fsm_state_decl(bna_enet, started, struct bna_enet_s,
			enum bna_enet_event_e);
bfa_fsm_state_decl(bna_enet, cfg_wait, struct bna_enet_s,
			enum bna_enet_event_e);
bfa_fsm_state_decl(bna_enet, cfg_stop_wait, struct bna_enet_s,
			enum bna_enet_event_e);
bfa_fsm_state_decl(bna_enet, chld_stop_wait, struct bna_enet_s,
			enum bna_enet_event_e);

static void
bna_enet_sm_stopped_entry(struct bna_enet_s *enet)
{
	bfa_trc(enet->bna, 0);
	call_enet_pause_cbfn(enet);
	call_enet_mtu_cbfn(enet);
	call_enet_stop_cbfn(enet);
	CNA_ASSERT((enet->flags & BNA_ENET_F_PAUSE_CHANGED) == 0);
	CNA_ASSERT((enet->flags & BNA_ENET_F_MTU_CHANGED) == 0);
}

static void
bna_enet_sm_stopped(struct bna_enet_s *enet, enum bna_enet_event_e event)
{
	bfa_trc(enet->bna, event);
	switch (event) {
	case ENET_E_START:
		bfa_fsm_set_state(enet, bna_enet_sm_pause_init_wait);
		break;

	case ENET_E_STOP:
		call_enet_stop_cbfn(enet);
		break;

	case ENET_E_FAIL:
		/* No-op */
		break;

	case ENET_E_PAUSE_CFG:
		call_enet_pause_cbfn(enet);
		break;

	case ENET_E_MTU_CFG:
		call_enet_mtu_cbfn(enet);
		break;

	case ENET_E_CHLD_STOPPED:
		/**
		 * This event is received due to Ethport, Tx and Rx objects
		 * failing
		 */
		/* No-op */
		break;

	default:
		bfa_sm_fault(enet->bna, event);
	}
}

static void
bna_enet_sm_pause_init_wait_entry(struct bna_enet_s *enet)
{
	bfa_trc(enet->bna, 0);
	bna_bfi_pause_set(enet);
}

static void
bna_enet_sm_pause_init_wait(struct bna_enet_s *enet,
				enum bna_enet_event_e event)
{
	bfa_trc(enet->bna, event);
	switch (event) {
	case ENET_E_STOP:
		enet->flags &= ~BNA_ENET_F_PAUSE_CHANGED;
		bfa_fsm_set_state(enet, bna_enet_sm_last_resp_wait);
		break;

	case ENET_E_FAIL:
		enet->flags &= ~BNA_ENET_F_PAUSE_CHANGED;
		bfa_fsm_set_state(enet, bna_enet_sm_stopped);
		break;

	case ENET_E_PAUSE_CFG:
		enet->flags |= BNA_ENET_F_PAUSE_CHANGED;
		break;

	case ENET_E_MTU_CFG:
		/* No-op */
		break;

	case ENET_E_FWRESP_PAUSE:
		if (enet->flags & BNA_ENET_F_PAUSE_CHANGED) {
			enet->flags &= ~BNA_ENET_F_PAUSE_CHANGED;
			bna_bfi_pause_set(enet);
		} else {
			bfa_fsm_set_state(enet, bna_enet_sm_started);
			bna_enet_chld_start(enet);
		}
		break;

	default:
		bfa_sm_fault(enet->bna, event);
	}
}

static void
bna_enet_sm_last_resp_wait_entry(struct bna_enet_s *enet)
{
	bfa_trc(enet->bna, 0);
	enet->flags &= ~BNA_ENET_F_PAUSE_CHANGED;
}

static void
bna_enet_sm_last_resp_wait(struct bna_enet_s *enet,
				enum bna_enet_event_e event)
{
	bfa_trc(enet->bna, event);
	switch (event) {
	case ENET_E_FAIL:
	case ENET_E_FWRESP_PAUSE:
		bfa_fsm_set_state(enet, bna_enet_sm_stopped);
		break;

	default:
		bfa_sm_fault(enet->bna, event);
	}
}

static void
bna_enet_sm_started_entry(struct bna_enet_s *enet)
{
	bfa_trc(enet->bna, 0);
	/**
	 * NOTE: Do not call bna_enet_chld_start() here, since it will be
	 * inadvertently called during cfg_wait->started transition as well
	 */
	call_enet_pause_cbfn(enet);
	call_enet_mtu_cbfn(enet);
}

static void
bna_enet_sm_started(struct bna_enet_s *enet,
			enum bna_enet_event_e event)
{
	bfa_trc(enet->bna, event);
	switch (event) {
	case ENET_E_STOP:
		bfa_fsm_set_state(enet, bna_enet_sm_chld_stop_wait);
		break;

	case ENET_E_FAIL:
		bfa_fsm_set_state(enet, bna_enet_sm_stopped);
		bna_enet_chld_fail(enet);
		break;

	case ENET_E_PAUSE_CFG:
		bfa_fsm_set_state(enet, bna_enet_sm_cfg_wait);
		bna_bfi_pause_set(enet);
		break;

	case ENET_E_MTU_CFG:
		bfa_fsm_set_state(enet, bna_enet_sm_cfg_wait);
		bna_enet_rx_stop(enet);
		break;

	default:
		bfa_sm_fault(enet->bna, event);
	}
}

static void
bna_enet_sm_cfg_wait_entry(struct bna_enet_s *enet)
{
	bfa_trc(enet->bna, 0);
}

static void
bna_enet_sm_cfg_wait(struct bna_enet_s *enet,
			enum bna_enet_event_e event)
{
	bfa_trc(enet->bna, event);
	switch (event) {
	case ENET_E_STOP:
		enet->flags &= ~BNA_ENET_F_PAUSE_CHANGED;
		enet->flags &= ~BNA_ENET_F_MTU_CHANGED;
		bfa_fsm_set_state(enet, bna_enet_sm_cfg_stop_wait);
		break;

	case ENET_E_FAIL:
		enet->flags &= ~BNA_ENET_F_PAUSE_CHANGED;
		enet->flags &= ~BNA_ENET_F_MTU_CHANGED;
		bfa_fsm_set_state(enet, bna_enet_sm_stopped);
		bna_enet_chld_fail(enet);
		break;

	case ENET_E_PAUSE_CFG:
		enet->flags |= BNA_ENET_F_PAUSE_CHANGED;
		break;

	case ENET_E_MTU_CFG:
		enet->flags |= BNA_ENET_F_MTU_CHANGED;
		break;

	case ENET_E_CHLD_STOPPED:
		bna_enet_rx_start(enet);
		/* Fall through */
	case ENET_E_FWRESP_PAUSE:
		if (enet->flags & BNA_ENET_F_PAUSE_CHANGED) {
			enet->flags &= ~BNA_ENET_F_PAUSE_CHANGED;
			bna_bfi_pause_set(enet);
		} else if (enet->flags & BNA_ENET_F_MTU_CHANGED) {
			enet->flags &= ~BNA_ENET_F_MTU_CHANGED;
			bna_enet_rx_stop(enet);
		} else {
			bfa_fsm_set_state(enet, bna_enet_sm_started);
		}
		break;

	default:
		bfa_sm_fault(enet->bna, event);
	}
}

static void
bna_enet_sm_cfg_stop_wait_entry(struct bna_enet_s *enet)
{
	bfa_trc(enet->bna, 0);
	enet->flags &= ~BNA_ENET_F_PAUSE_CHANGED;
	enet->flags &= ~BNA_ENET_F_MTU_CHANGED;
}

static void
bna_enet_sm_cfg_stop_wait(struct bna_enet_s *enet,
				enum bna_enet_event_e event)
{
	bfa_trc(enet->bna, event);
	switch (event) {
	case ENET_E_FAIL:
		bfa_fsm_set_state(enet, bna_enet_sm_stopped);
		bna_enet_chld_fail(enet);
		break;

	case ENET_E_FWRESP_PAUSE:
	case ENET_E_CHLD_STOPPED:
		bfa_fsm_set_state(enet, bna_enet_sm_chld_stop_wait);
		break;

	default:
		bfa_sm_fault(enet->bna, event);
	}
}

static void
bna_enet_sm_chld_stop_wait_entry(struct bna_enet_s *enet)
{
	bfa_trc(enet->bna, 0);
	bna_enet_chld_stop(enet);
}

static void
bna_enet_sm_chld_stop_wait(struct bna_enet_s *enet,
				enum bna_enet_event_e event)
{
	bfa_trc(enet->bna, event);
	switch (event) {
	case ENET_E_FAIL:
		bfa_fsm_set_state(enet, bna_enet_sm_stopped);
		bna_enet_chld_fail(enet);
		break;

	case ENET_E_CHLD_STOPPED:
		bfa_fsm_set_state(enet, bna_enet_sm_stopped);
		break;

	default:
		bfa_sm_fault(enet->bna, event);
	}
}

static void
bna_bfi_pause_set(struct bna_enet_s *enet)
{
	struct bfi_enet_set_pause_req *pause_req = &enet->pause_req;

	bfi_msgq_mhdr_set(pause_req->mh, BFI_MC_ENET,
		BFI_ENET_H2I_SET_PAUSE_REQ, 0, 0);
	pause_req->mh.num_entries = cna_os_htons(
	bfi_msgq_num_cmd_entries(sizeof(struct bfi_enet_set_pause_req)));
	pause_req->tx_pause = enet->pause_config.tx_pause;
	pause_req->rx_pause = enet->pause_config.rx_pause;

	bfa_msgq_cmd_set(&enet->msgq_cmd, NULL, NULL,
		sizeof(struct bfi_enet_set_pause_req), &pause_req->mh);
	bfa_msgq_cmd_post(&enet->bna->msgq, &enet->msgq_cmd);
}

static void
bna_enet_cb_chld_stopped(void *arg)
{
	struct bna_enet_s *enet = (struct bna_enet_s *)arg;

	bfa_fsm_send_event(enet, ENET_E_CHLD_STOPPED);
}

void
bna_enet_init(struct bna_enet_s *enet, struct bna_s *bna)
{
	enet->bna = bna;
	enet->flags = 0;
	enet->mtu = 0;
	enet->type = BNA_ENET_T_REGULAR;

	enet->stop_cbfn = NULL;
	enet->stop_cbarg = NULL;

	enet->pause_cbfn = NULL;

	enet->mtu_cbfn = NULL;

	bfa_fsm_set_state(enet, bna_enet_sm_stopped);
}

void
bna_enet_uninit(struct bna_enet_s *enet)
{
	CNA_ASSERT(enet->fsm == (bfa_fsm_t)bna_enet_sm_stopped);
	CNA_ASSERT(enet->chld_stop_wc.wc_count == 0);

	CNA_ASSERT(enet->stop_cbfn == NULL);
	CNA_ASSERT(enet->pause_cbfn == NULL);
	CNA_ASSERT(enet->mtu_cbfn == NULL);

	enet->flags = 0;

	enet->bna = NULL;
}

void
bna_enet_start(struct bna_enet_s *enet)
{
	CNA_ASSERT(enet->fsm == (bfa_fsm_t)bna_enet_sm_stopped);
	bfa_trc(enet->bna, 0);
	bfa_trc(enet->bna, enet->flags);
	enet->flags |= BNA_ENET_F_IOCETH_READY;
	if (enet->flags & BNA_ENET_F_ENABLED)
		bfa_fsm_send_event(enet, ENET_E_START);
}

void
bna_enet_stop(struct bna_enet_s *enet)
{
	bfa_trc(enet->bna, 0);
	CNA_ASSERT(enet->stop_cbfn == NULL);
	CNA_ASSERT(enet->pause_cbfn == NULL);
	CNA_ASSERT(enet->mtu_cbfn == NULL);
	enet->stop_cbfn = bna_ioceth_cb_enet_stopped;
	enet->stop_cbarg = &enet->bna->ioceth;

	bfa_trc(enet->bna, enet->flags);
	enet->flags &= ~BNA_ENET_F_IOCETH_READY;
	bfa_fsm_send_event(enet, ENET_E_STOP);
}

void
bna_enet_fail(struct bna_enet_s *enet)
{
	bfa_trc(enet->bna, 0);
	bfa_trc(enet->bna, enet->flags);
	enet->flags &= ~BNA_ENET_F_IOCETH_READY;
	bfa_fsm_send_event(enet, ENET_E_FAIL);
}

void
bna_enet_cb_ethport_stopped(struct bna_enet_s *enet)
{
	bfa_wc_down(&enet->chld_stop_wc);
}

void
bna_enet_cb_tx_stopped(struct bna_enet_s *enet)
{
	bfa_wc_down(&enet->chld_stop_wc);
}

void
bna_enet_cb_rx_stopped(struct bna_enet_s *enet)
{
	bfa_wc_down(&enet->chld_stop_wc);
}

void
bna_bfi_pause_set_rsp(struct bna_enet_s *enet, struct bfi_msgq_mhdr_s *msghdr)
{
	struct bfi_enet_rsp *rsp = (struct bfi_enet_rsp *)msghdr;

	CNA_ASSERT(rsp->error == BFI_ENET_CMD_OK);
	bfa_fsm_send_event(enet, ENET_E_FWRESP_PAUSE);
}

void
bna_bfi_bw_update_aen(struct bna_enet_s *enet, struct bfi_msgq_mhdr_s *msghdr)
{
	struct bfi_enet_rsp *aen = (struct bfi_enet_rsp *)msghdr;

	bna_bfi_tx_bw_update(&enet->bna->tx_mod);
	bnad_cb_bw_update(enet->bna->bnad, cna_os_ntohl(aen->bw));
}

int
bna_enet_mtu_get(struct bna_enet_s *enet)
{
	CNA_ASSERT(enet->mtu != 0);
	return enet->mtu;
}

void
bna_enet_enable(struct bna_enet_s *enet)
{
	if (enet->fsm != (bfa_sm_t)bna_enet_sm_stopped)
		return;

	CNA_ASSERT(enet->mtu != 0);

	bfa_trc(enet->bna, enet->flags);
	enet->flags |= BNA_ENET_F_ENABLED;

	if (enet->flags & BNA_ENET_F_IOCETH_READY)
		bfa_fsm_send_event(enet, ENET_E_START);
}

void
bna_enet_disable(struct bna_enet_s *enet, enum bna_cleanup_type_e type,
		 void (*cbfn)(void *))
{
	CNA_ASSERT(cbfn);

	if (type == BNA_SOFT_CLEANUP) {
		(*cbfn)(enet->bna->bnad);
		return;
	}

	CNA_ASSERT(enet->stop_cbfn == NULL);
	CNA_ASSERT(enet->pause_cbfn == NULL);
	CNA_ASSERT(enet->mtu_cbfn == NULL);
	enet->stop_cbfn = cbfn;
	enet->stop_cbarg = enet->bna->bnad;

	bfa_trc(enet->bna, enet->flags);
	enet->flags &= ~BNA_ENET_F_ENABLED;

	bfa_fsm_send_event(enet, ENET_E_STOP);
}

void
bna_enet_pause_config(struct bna_enet_s *enet,
		      struct bna_pause_config_s *pause_config,
		      void (*cbfn)(struct bnad_s *))
{
	bfa_os_assign(enet->pause_config, *pause_config);

	CNA_ASSERT(enet->stop_cbfn == NULL);
	CNA_ASSERT(enet->pause_cbfn == NULL);
	CNA_ASSERT(enet->mtu_cbfn == NULL);
	enet->pause_cbfn = cbfn;

	bfa_fsm_send_event(enet, ENET_E_PAUSE_CFG);
}

void
bna_enet_mtu_set(struct bna_enet_s *enet, int mtu,
		 void (*cbfn)(struct bnad_s *))
{
	enet->mtu = mtu;

	CNA_ASSERT(enet->stop_cbfn == NULL);
	CNA_ASSERT(enet->pause_cbfn == NULL);
	CNA_ASSERT(enet->mtu_cbfn == NULL);
	enet->mtu_cbfn = cbfn;

	bfa_fsm_send_event(enet, ENET_E_MTU_CFG);
}

void
bna_enet_perm_mac_get(struct bna_enet_s *enet, mac_t *mac)
{
	*mac = bfa_ioc_get_mac(&enet->bna->ioceth.ioc);
}

/* Should be called only when enet is disabled */
void
bna_enet_type_set(struct bna_enet_s *enet, enum bna_enet_type_e type)
{
	CNA_ASSERT(enet->fsm == (bfa_fsm_t)bna_enet_sm_stopped);
	enet->type = type;
}

enum bna_enet_type_e
bna_enet_type_get(struct bna_enet_s *enet)
{
	return enet->type;
}

/**
 * STATS
 */

static void
bna_stats_mod_init(struct bna_stats_mod_s *stats_mod, struct bna_s *bna)
{
	stats_mod->bna = bna;
}

static void
bna_stats_mod_uninit(struct bna_stats_mod_s *stats_mod)
{
	CNA_ASSERT(stats_mod->stop_cbfn == NULL);
	CNA_ASSERT(stats_mod->stats_get_busy == BFA_FALSE);
	CNA_ASSERT(stats_mod->stats_clr_busy == BFA_FALSE);
	CNA_ASSERT(stats_mod->pending_reqs == 0);
	stats_mod->bna = NULL;
}

static void
bna_stats_mod_start(struct bna_stats_mod_s *stats_mod)
{
	stats_mod->ioc_ready = BFA_TRUE;
	CNA_ASSERT(stats_mod->stop_cbfn == NULL);
	CNA_ASSERT(stats_mod->stats_get_busy == BFA_FALSE);
	CNA_ASSERT(stats_mod->stats_clr_busy == BFA_FALSE);
	CNA_ASSERT(stats_mod->pending_reqs == 0);
}

static void
bna_stats_mod_stop(struct bna_stats_mod_s *stats_mod)
{
	stats_mod->ioc_ready = BFA_FALSE;
	if (!stats_mod->pending_reqs) {
		bna_ioceth_cb_stats_mod_stopped(&stats_mod->bna->ioceth);
		return;
	}

	stats_mod->stop_cbfn = bna_ioceth_cb_stats_mod_stopped;
}

static void
bna_stats_mod_fail(struct bna_stats_mod_s *stats_mod)
{
	stats_mod->ioc_ready = BFA_FALSE;
	if (stats_mod->stats_get_busy)
		bnad_cb_stats_get(stats_mod->bna->bnad, BNA_CB_FAIL,
			&stats_mod->bna->stats);
	stats_mod->stats_get_busy = BFA_FALSE;
	stats_mod->stats_clr_busy = BFA_FALSE;
	stats_mod->stop_cbfn = NULL;
	stats_mod->pending_reqs = 0;
}

#define call_stats_mod_stop_cbfn(_stats_mod)				\
do {									\
	CNA_ASSERT((_stats_mod)->pending_reqs > 0);			\
	(_stats_mod)->pending_reqs--;					\
	if ((_stats_mod)->stop_cbfn && !(_stats_mod)->pending_reqs) {	\
		void (*cbfn)(struct bna_ioceth_s *);			\
		cbfn = (_stats_mod)->stop_cbfn;				\
		(_stats_mod)->stop_cbfn = NULL;				\
		(*cbfn)(&(_stats_mod)->bna->ioceth);			\
	}								\
} while (0)

static void
bna_bfi_stats_get(struct bna_s *bna)
{
	struct bfi_enet_stats_req *stats_req = &bna->stats_mod.stats_get;

	bna->stats_mod.stats_get_busy = BFA_TRUE;
	bna->stats_mod.pending_reqs++;

	bfi_msgq_mhdr_set(stats_req->mh, BFI_MC_ENET,
		BFI_ENET_H2I_STATS_GET_REQ, 0, 0);
	stats_req->mh.num_entries = cna_os_htons(
		bfi_msgq_num_cmd_entries(sizeof(struct bfi_enet_stats_req)));
	stats_req->stats_mask = cna_os_htons(BFI_ENET_STATS_ALL);
	stats_req->tx_enet_mask = cna_os_htonl(bna->tx_mod.rid_mask);
	stats_req->rx_enet_mask = cna_os_htonl(bna->rx_mod.rid_mask);
	stats_req->host_buffer.a32.addr_hi = bna->stats.hw_stats_dma.msb;
	stats_req->host_buffer.a32.addr_lo = bna->stats.hw_stats_dma.lsb;

	bfa_trc(bna, stats_req->tx_enet_mask);
	bfa_trc(bna, stats_req->rx_enet_mask);

	bfa_msgq_cmd_set(&bna->stats_mod.stats_get_cmd, NULL, NULL,
		sizeof(struct bfi_enet_stats_req), &stats_req->mh);
	bfa_msgq_cmd_post(&bna->msgq, &bna->stats_mod.stats_get_cmd);
}

static void
bna_cb_bfi_stats_clr(void *cbarg, enum bfa_status status)
{
	struct bna_s *bna = cbarg;
	bna->stats_mod.stats_clr_busy = BFA_FALSE;
}

static void
bna_bfi_stats_clr(struct bna_s *bna, uint16_t stats_mask, uint32_t tx_enet_mask,
	uint32_t rx_enet_mask)
{
	struct bfi_enet_stats_req *stats_req = &bna->stats_mod.stats_clr;

	bna->stats_mod.stats_clr_busy = BFA_TRUE;
	bna->stats_mod.pending_reqs++;

	bfi_msgq_mhdr_set(stats_req->mh, BFI_MC_ENET,
		BFI_ENET_H2I_STATS_CLR_REQ, 0, 0);
	stats_req->mh.num_entries = cna_os_htons(
		bfi_msgq_num_cmd_entries(sizeof(struct bfi_enet_stats_req)));
	stats_req->stats_mask = cna_os_htons(stats_mask);
	stats_req->tx_enet_mask = cna_os_htonl(tx_enet_mask);
	stats_req->rx_enet_mask = cna_os_htonl(rx_enet_mask);

	bfa_msgq_cmd_set(&bna->stats_mod.stats_clr_cmd, bna_cb_bfi_stats_clr,
		bna, sizeof(struct bfi_enet_stats_req), &stats_req->mh);
	bfa_msgq_cmd_post(&bna->msgq, &bna->stats_mod.stats_clr_cmd);
}

#define bna_stats_copy(_name, _type)					\
do {									\
	count = sizeof(struct bfi_enet_stats_ ## _type) / sizeof(uint64_t);\
	stats_src = (uint64_t *)&bna->stats.hw_stats_kva->_name ## _stats;\
	stats_dst = (uint64_t *)&bna->stats.hw_stats._name ## _stats;\
	for (i = 0; i < count; i++)					\
		stats_dst[i] = cna_os_ntohll(stats_src[i]);		\
} while (0)								\

void
bna_bfi_stats_get_rsp(struct bna_s *bna, struct bfi_msgq_mhdr_s *msghdr)
{
	struct bfi_enet_stats_req *stats_req = &bna->stats_mod.stats_get;
	uint64_t *stats_src;
	uint64_t *stats_dst;
	uint32_t tx_enet_mask = cna_os_ntohl(stats_req->tx_enet_mask);
	uint32_t rx_enet_mask = cna_os_ntohl(stats_req->rx_enet_mask);
	int count;
	int i;

	bna_stats_copy(mac, mac);
	bna_stats_copy(bpc, bpc);
	bna_stats_copy(rad, rad);
	bna_stats_copy(rlb, rad);
	bna_stats_copy(fc_rx, fc_rx);
	bna_stats_copy(fc_tx, fc_tx);

	stats_src = (uint64_t *)&(bna->stats.hw_stats_kva->rxf_stats[0]);

	/* Copy Rxf stats to SW area, scatter them while copying */
	for (i = 0; i < BFI_ENET_CFG_MAX; i++) {
		stats_dst = (uint64_t *)&(bna->stats.hw_stats.rxf_stats[i]);
		cna_os_memset(stats_dst, 0, sizeof(struct bfi_enet_stats_rxf));
		if (rx_enet_mask & ((uint32_t)(1 << i))) {
			int k;
			count = sizeof(struct bfi_enet_stats_rxf) /
				sizeof(uint64_t);
			for (k = 0; k < count; k++) {
				stats_dst[k] = cna_os_ntohll(*stats_src);
				stats_src++;
			}
		}
	}

	/* Copy Txf stats to SW area, scatter them while copying */
	for (i = 0; i < BFI_ENET_CFG_MAX; i++) {
		stats_dst = (uint64_t *)&(bna->stats.hw_stats.txf_stats[i]);
		cna_os_memset(stats_dst, 0, sizeof(struct bfi_enet_stats_txf));
		if (tx_enet_mask & ((uint32_t)(1 << i))) {
			int k;
			count = sizeof(struct bfi_enet_stats_txf) /
				sizeof(uint64_t);
			for (k = 0; k < count; k++) {
				stats_dst[k] = cna_os_ntohll(*stats_src);
				stats_src++;
			}
		}
	}

	bna->stats_mod.stats_get_busy = BFA_FALSE;
	call_stats_mod_stop_cbfn(&bna->stats_mod);
	bnad_cb_stats_get(bna->bnad, BNA_CB_SUCCESS, &bna->stats);
}

void
bna_bfi_stats_clr_rsp(struct bna_s *bna, struct bfi_msgq_mhdr_s *msghdr)
{
	call_stats_mod_stop_cbfn(&bna->stats_mod);
}

/**
 * IOCETH
 */

#define enable_mbox_intr(_ioceth)					\
do {									\
	uint32_t intr_status;						\
	if (bfa_ioc_boot_env(&(_ioceth)->ioc) == BFI_FWBOOT_ENV_OS) {	\
		bna_intr_status_get((_ioceth)->bna, intr_status);	\
		bnad_cb_mbox_intr_enable((_ioceth)->bna->bnad);		\
		bna_mbox_intr_enable((_ioceth)->bna);			\
	}								\
} while (0)

#define disable_mbox_intr(_ioceth)					\
do {									\
	bna_mbox_intr_disable((_ioceth)->bna);				\
	if (bfa_ioc_boot_env(&(_ioceth)->ioc) == BFI_FWBOOT_ENV_OS)	\
		bnad_cb_mbox_intr_disable((_ioceth)->bna->bnad);	\
} while (0)

#define call_ioceth_stop_cbfn(_ioceth)					\
do {									\
	if ((_ioceth)->stop_cbfn) {					\
		void (*cbfn)(struct bnad_s *);				\
		struct bnad_s *cbarg;					\
		cbfn = (_ioceth)->stop_cbfn;				\
		cbarg = (_ioceth)->stop_cbarg;				\
		(_ioceth)->stop_cbfn = NULL;				\
		(_ioceth)->stop_cbarg = NULL;				\
		cbfn(cbarg);						\
	}								\
} while (0)

#define bna_ioceth_chld_stop(ioceth)					\
do {									\
	CNA_ASSERT((ioceth)->chld_stop_wc.wc_count == 0);		\
	bfa_wc_init(&(ioceth)->chld_stop_wc, bna_ioceth_cb_chld_stopped,\
		(ioceth));\
	bfa_wc_up(&(ioceth)->chld_stop_wc);				\
	bna_stats_mod_stop(&(ioceth)->bna->stats_mod);			\
	bfa_wc_up(&(ioceth)->chld_stop_wc);				\
	bna_enet_stop(&(ioceth)->bna->enet);				\
	bfa_wc_wait(&(ioceth)->chld_stop_wc);				\
} while (0)

static void bna_ioceth_cb_chld_stopped(void *cbarg);
static void bna_bfi_attr_get(struct bna_ioceth_s *ioceth);

enum bna_ioceth_event_e {
	IOCETH_E_ENABLE			= 1,
	IOCETH_E_DISABLE		= 2,
	IOCETH_E_IOC_RESET		= 3,
	IOCETH_E_IOC_FAILED		= 4,
	IOCETH_E_IOC_READY		= 5,
	IOCETH_E_ENET_ATTR_RESP		= 6,
	IOCETH_E_CHLD_STOPPED		= 7,
	IOCETH_E_IOC_DISABLED		= 8,
};

bfa_fsm_state_decl(bna_ioceth, stopped, struct bna_ioceth_s,
			enum bna_ioceth_event_e);
bfa_fsm_state_decl(bna_ioceth, ioc_ready_wait, struct bna_ioceth_s,
			enum bna_ioceth_event_e);
bfa_fsm_state_decl(bna_ioceth, enet_attr_wait, struct bna_ioceth_s,
			enum bna_ioceth_event_e);
bfa_fsm_state_decl(bna_ioceth, ready, struct bna_ioceth_s,
			enum bna_ioceth_event_e);
bfa_fsm_state_decl(bna_ioceth, last_resp_wait, struct bna_ioceth_s,
			enum bna_ioceth_event_e);
bfa_fsm_state_decl(bna_ioceth, chld_stop_wait, struct bna_ioceth_s,
			enum bna_ioceth_event_e);
bfa_fsm_state_decl(bna_ioceth, ioc_disable_wait, struct bna_ioceth_s,
			enum bna_ioceth_event_e);
bfa_fsm_state_decl(bna_ioceth, failed, struct bna_ioceth_s,
			enum bna_ioceth_event_e);

static void
bna_ioceth_sm_stopped_entry(struct bna_ioceth_s *ioceth)
{
	bfa_trc(ioceth->bna, 0);
	call_ioceth_stop_cbfn(ioceth);
}

static void
bna_ioceth_sm_stopped(struct bna_ioceth_s *ioceth,
			enum bna_ioceth_event_e event)
{
	bfa_trc(ioceth->bna, event);
	switch (event) {
	case IOCETH_E_ENABLE:
		bna_msix_idx_clear(ioceth->bna, &ioceth->bna->pcidev);
		bfa_fsm_set_state(ioceth, bna_ioceth_sm_ioc_ready_wait);
		bfa_ioc_enable(&ioceth->ioc);
		break;

	case IOCETH_E_DISABLE:
		bfa_fsm_set_state(ioceth, bna_ioceth_sm_stopped);
		break;

	case IOCETH_E_IOC_RESET:
		enable_mbox_intr(ioceth);
		break;

	case IOCETH_E_IOC_FAILED:
		disable_mbox_intr(ioceth);
		bfa_fsm_set_state(ioceth, bna_ioceth_sm_failed);
		break;

	default:
		bfa_sm_fault(ioceth->bna, event);
	}
}

static void
bna_ioceth_sm_ioc_ready_wait_entry(struct bna_ioceth_s *ioceth)
{
	bfa_trc(ioceth->bna, 0);
	/**
	 * Do not call bfa_ioc_enable() here. It must be called in the
	 * previous state due to failed -> ioc_ready_wait transition.
	 */
}

static void
bna_ioceth_sm_ioc_ready_wait(struct bna_ioceth_s *ioceth,
				enum bna_ioceth_event_e event)
{
	bfa_trc(ioceth->bna, event);
	switch (event) {
	case IOCETH_E_DISABLE:
		bfa_fsm_set_state(ioceth, bna_ioceth_sm_ioc_disable_wait);
		bfa_ioc_disable(&ioceth->ioc);
		break;

	case IOCETH_E_IOC_RESET:
		enable_mbox_intr(ioceth);
		break;

	case IOCETH_E_IOC_FAILED:
		disable_mbox_intr(ioceth);
		bfa_fsm_set_state(ioceth, bna_ioceth_sm_failed);
		break;

	case IOCETH_E_IOC_READY:
		bfa_fsm_set_state(ioceth, bna_ioceth_sm_enet_attr_wait);
		break;

	default:
		bfa_sm_fault(ioceth->bna, event);
	}
}

static void
bna_ioceth_sm_enet_attr_wait_entry(struct bna_ioceth_s *ioceth)
{
	bfa_trc(ioceth->bna, 0);
	bna_bfi_attr_get(ioceth);
}

static void
bna_ioceth_sm_enet_attr_wait(struct bna_ioceth_s *ioceth,
				enum bna_ioceth_event_e event)
{
	bfa_trc(ioceth->bna, event);
	switch (event) {
	case IOCETH_E_DISABLE:
		bfa_fsm_set_state(ioceth, bna_ioceth_sm_last_resp_wait);
		break;

	case IOCETH_E_IOC_FAILED:
		disable_mbox_intr(ioceth);
		bfa_fsm_set_state(ioceth, bna_ioceth_sm_failed);
		break;

	case IOCETH_E_ENET_ATTR_RESP:
		bfa_fsm_set_state(ioceth, bna_ioceth_sm_ready);
		break;

	default:
		bfa_sm_fault(ioceth->bna, event);
	}
}

static void
bna_ioceth_sm_ready_entry(struct bna_ioceth_s *ioceth)
{
	bfa_trc(ioceth->bna, 0);
	bna_enet_start(&ioceth->bna->enet);
	bna_stats_mod_start(&ioceth->bna->stats_mod);
	bnad_cb_ioceth_ready(ioceth->bna->bnad);
}

static void
bna_ioceth_sm_ready(struct bna_ioceth_s *ioceth, enum bna_ioceth_event_e event)
{
	bfa_trc(ioceth->bna, event);
	switch (event) {
	case IOCETH_E_DISABLE:
		bfa_fsm_set_state(ioceth, bna_ioceth_sm_chld_stop_wait);
		break;

	case IOCETH_E_IOC_FAILED:
		disable_mbox_intr(ioceth);
		bna_enet_fail(&ioceth->bna->enet);
		bna_stats_mod_fail(&ioceth->bna->stats_mod);
		bfa_fsm_set_state(ioceth, bna_ioceth_sm_failed);
		break;

	default:
		bfa_sm_fault(ioceth->bna, event);
	}
}

static void
bna_ioceth_sm_last_resp_wait_entry(struct bna_ioceth_s *ioceth)
{
	bfa_trc(ioceth->bna, 0);
}

static void
bna_ioceth_sm_last_resp_wait(struct bna_ioceth_s *ioceth,
				enum bna_ioceth_event_e event)
{
	bfa_trc(ioceth->bna, event);
	switch (event) {
	case IOCETH_E_IOC_FAILED:
		bfa_fsm_set_state(ioceth, bna_ioceth_sm_ioc_disable_wait);
		disable_mbox_intr(ioceth);
		bfa_ioc_disable(&ioceth->ioc);
		break;

	case IOCETH_E_ENET_ATTR_RESP:
		bfa_fsm_set_state(ioceth, bna_ioceth_sm_ioc_disable_wait);
		bfa_ioc_disable(&ioceth->ioc);
		break;

	default:
		bfa_sm_fault(ioceth->bna, event);
	}
}

static void
bna_ioceth_sm_chld_stop_wait_entry(struct bna_ioceth_s *ioceth)
{
	bfa_trc(ioceth->bna, 0);
	bna_ioceth_chld_stop(ioceth);
}

static void
bna_ioceth_sm_chld_stop_wait(struct bna_ioceth_s *ioceth,
				enum bna_ioceth_event_e event)
{
	bfa_trc(ioceth->bna, event);
	switch (event) {
	case IOCETH_E_IOC_FAILED:
		bfa_fsm_set_state(ioceth, bna_ioceth_sm_ioc_disable_wait);
		disable_mbox_intr(ioceth);
		bna_enet_fail(&ioceth->bna->enet);
		bna_stats_mod_fail(&ioceth->bna->stats_mod);
		bfa_ioc_disable(&ioceth->ioc);
		break;

	case IOCETH_E_CHLD_STOPPED:
		bfa_fsm_set_state(ioceth, bna_ioceth_sm_ioc_disable_wait);
		bfa_ioc_disable(&ioceth->ioc);
		break;

	default:
		bfa_sm_fault(ioceth->bna, event);
	}
}

static void
bna_ioceth_sm_ioc_disable_wait_entry(struct bna_ioceth_s *ioceth)
{
	bfa_trc(ioceth->bna, 0);
}

static void
bna_ioceth_sm_ioc_disable_wait(struct bna_ioceth_s *ioceth,
				enum bna_ioceth_event_e event)
{
	bfa_trc(ioceth->bna, event);
	switch (event) {
	case IOCETH_E_IOC_DISABLED:
		disable_mbox_intr(ioceth);
		bfa_fsm_set_state(ioceth, bna_ioceth_sm_stopped);
		break;

	case IOCETH_E_CHLD_STOPPED:
		/* This event is received due to enet failing */
		/* No-op */
		break;

	default:
		bfa_sm_fault(ioceth->bna, event);
	}
}

static void
bna_ioceth_sm_failed_entry(struct bna_ioceth_s *ioceth)
{
	bfa_trc(ioceth->bna, 0);
	bnad_cb_ioceth_failed(ioceth->bna->bnad);
}

static void
bna_ioceth_sm_failed(struct bna_ioceth_s *ioceth,
			enum bna_ioceth_event_e event)
{
	bfa_trc(ioceth->bna, event);
	switch (event) {
	case IOCETH_E_DISABLE:
		bfa_fsm_set_state(ioceth, bna_ioceth_sm_ioc_disable_wait);
		bfa_ioc_disable(&ioceth->ioc);
		break;

	case IOCETH_E_IOC_RESET:
		enable_mbox_intr(ioceth);
		bfa_fsm_set_state(ioceth, bna_ioceth_sm_ioc_ready_wait);
		break;

	case IOCETH_E_IOC_FAILED:
		break;

	default:
		bfa_sm_fault(ioceth->bna, event);
	}
}

static void
bna_bfi_attr_get(struct bna_ioceth_s *ioceth)
{
	struct bfi_enet_attr_req_s *attr_req = &ioceth->attr_req;

	bfi_msgq_mhdr_set(attr_req->mh, BFI_MC_ENET,
		BFI_ENET_H2I_GET_ATTR_REQ, 0, 0);
	attr_req->mh.num_entries = cna_os_htons(
	bfi_msgq_num_cmd_entries(sizeof(struct bfi_enet_attr_req_s)));
	bfa_msgq_cmd_set(&ioceth->msgq_cmd, NULL, NULL,
		sizeof(struct bfi_enet_attr_req_s), &attr_req->mh);
	bfa_msgq_cmd_post(&ioceth->bna->msgq, &ioceth->msgq_cmd);
}


/* IOC callback functions */

static void
bna_cb_ioceth_enable(void *arg, enum bfa_status error)
{
	struct bna_ioceth_s *ioceth = (struct bna_ioceth_s *)arg;

	bfa_trc(ioceth->bna, error);
	if (error)
		bfa_fsm_send_event(ioceth, IOCETH_E_IOC_FAILED);
	else
		bfa_fsm_send_event(ioceth, IOCETH_E_IOC_READY);
}

static void
bna_cb_ioceth_disable(void *arg)
{
	struct bna_ioceth_s *ioceth = (struct bna_ioceth_s *)arg;

	bfa_trc(ioceth->bna, 0);
	bfa_fsm_send_event(ioceth, IOCETH_E_IOC_DISABLED);
}

static void
bna_cb_ioceth_hbfail(void *arg)
{
	struct bna_ioceth_s *ioceth = (struct bna_ioceth_s *)arg;

	bfa_trc(ioceth->bna, 0);
	bfa_fsm_send_event(ioceth, IOCETH_E_IOC_FAILED);
}

static void
bna_cb_ioceth_reset(void *arg)
{
	struct bna_ioceth_s *ioceth = (struct bna_ioceth_s *)arg;

	bfa_trc(ioceth->bna, 0);
	bfa_fsm_send_event(ioceth, IOCETH_E_IOC_RESET);
}

static struct bfa_ioc_cbfn_s bna_ioceth_cbfn = {
	bna_cb_ioceth_enable,
	bna_cb_ioceth_disable,
	bna_cb_ioceth_hbfail,
	bna_cb_ioceth_reset
};

static void bna_attr_init(struct bna_ioceth_s *ioceth)
{
	ioceth->attr.num_txq = BFI_ENET_DEF_TXQ;
	ioceth->attr.num_rxp = BFI_ENET_DEF_RXP;
	ioceth->attr.num_ucmac = BFI_ENET_DEF_UCAM;
	ioceth->attr.num_mcmac = BFI_ENET_MAX_MCAM;
	ioceth->attr.max_rit_size = BFI_ENET_DEF_RITSZ;
	ioceth->attr.num_wol_magic = 0;
	ioceth->attr.num_wol_frame = 0;
	ioceth->attr.fw_query_complete = BFA_FALSE;
}

static void
bna_ioceth_cb_chld_stopped(void *cbarg)
{
	struct bna_ioceth_s *ioceth = (struct bna_ioceth_s *)cbarg;
	bfa_fsm_send_event(ioceth, IOCETH_E_CHLD_STOPPED);
}

void
bna_ioceth_cb_stats_mod_stopped(struct bna_ioceth_s *ioceth)
{
	bfa_wc_down(&ioceth->chld_stop_wc);
}

void
bna_ioceth_cb_enet_stopped(void *arg)
{
	struct bna_ioceth_s *ioceth = (struct bna_ioceth_s *)arg;

	bfa_wc_down(&ioceth->chld_stop_wc);
}

void
bna_ioceth_init(struct bna_ioceth_s *ioceth, struct bna_s *bna,
		struct bna_res_info_s *res_info)
{
	uint64_t dma;
	uint8_t *kva;

	ioceth->bna = bna;

	/**
	 * Attach IOC and claim:
	 *	1. DMA memory for IOC attributes
	 *	2. Kernel memory for FW trace
	 */
	bfa_ioc_trc_aen_log_set(&ioceth->ioc, bna->trcmod, bna->aen,
			bna->logm);
	bfa_ioc_attach(&ioceth->ioc, ioceth, &bna_ioceth_cbfn,
			&ioceth->timer_mod);
	bfa_ioc_pci_init(&ioceth->ioc, &bna->pcidev, BFI_PCIFN_CLASS_ETH);

	BNA_GET_DMA_ADDR(
		&res_info[BNA_RES_MEM_T_ATTR].res_u.mem_info.mdl[0].dma, dma);
	kva = res_info[BNA_RES_MEM_T_ATTR].res_u.mem_info.mdl[0].kva;
	bfa_ioc_mem_claim(&ioceth->ioc, kva, dma);

	kva = res_info[BNA_RES_MEM_T_FWTRC].res_u.mem_info.mdl[0].kva;
	bfa_ioc_debug_memclaim(&ioceth->ioc, kva);

	bfa_edma_attach(&bna->edma, &ioceth->ioc, bna->trcmod);

	/**
	 * Attach common modules (Diag, SFP, CEE, Port) and claim respective
	 * DMA memory.
	 */
	BNA_GET_DMA_ADDR(
		&res_info[BNA_RES_MEM_T_COM].res_u.mem_info.mdl[0].dma, dma);
	kva = res_info[BNA_RES_MEM_T_COM].res_u.mem_info.mdl[0].kva;
	bfa_diag_attach(&bna->diag, &ioceth->ioc, bna, NULL, bna->trcmod);
	bfa_diag_memclaim(&bna->diag, kva, dma);
	kva += bfa_diag_meminfo();
	dma += bfa_diag_meminfo();

	bfa_sfp_attach(&bna->sfp, &ioceth->ioc, bna, bna->trcmod);
	bfa_sfp_memclaim(&bna->sfp, kva, dma);
	kva += bfa_sfp_meminfo();
	dma += bfa_sfp_meminfo();

	bfa_cee_log_trc_set(&bna->cee, bna->trcmod, bna->logm);
	bfa_cee_attach(&bna->cee, &ioceth->ioc, bna);
	bfa_cee_mem_claim(&bna->cee, kva, dma);
	kva += bfa_cee_meminfo();
	dma += bfa_cee_meminfo();

	bfa_port_attach(&bna->phy_port, &ioceth->ioc, bna, bna->trcmod,
			bna->logm);
	bfa_port_mem_claim(&bna->phy_port, kva, dma);
	kva += bfa_port_meminfo();
	dma += bfa_port_meminfo();

	bfa_flash_attach(&bna->flash, &ioceth->ioc, bna, bna->trcmod,
				BFA_FALSE);
	bfa_flash_memclaim(&bna->flash, kva, dma, BFA_FALSE);
	kva += bfa_flash_meminfo(BFA_FALSE);
	dma += bfa_flash_meminfo(BFA_FALSE);

	bfa_phy_attach(&bna->phy, &ioceth->ioc, bna, bna->trcmod, BFA_FALSE);
	bfa_phy_memclaim(&bna->phy, kva, dma, BFA_FALSE);
	kva += bfa_phy_meminfo(BFA_FALSE);
	dma += bfa_phy_meminfo(BFA_FALSE);

	bfa_fru_attach(&bna->fru, &ioceth->ioc, bna, bna->trcmod, BFA_FALSE);
	bfa_fru_memclaim(&bna->fru, kva, dma, BFA_FALSE);
	kva += bfa_fru_meminfo(BFA_FALSE);
	dma += bfa_fru_meminfo(BFA_FALSE);

	bfa_ablk_attach(&bna->ablk, &ioceth->ioc);
	bfa_ablk_memclaim(&bna->ablk, kva, dma);
	kva += bfa_ablk_meminfo();
	dma += bfa_ablk_meminfo();

	bfa_msgq_attach(&bna->msgq, &ioceth->ioc, bna->trcmod, bna->logm);
	bfa_msgq_memclaim(&bna->msgq, kva, dma);
	bfa_msgq_regisr(&bna->msgq, BFI_MC_ENET, bna_msgq_rsp_handler, bna);
	kva += bfa_msgq_meminfo();
	dma += bfa_msgq_meminfo();

	bfa_timer_init(&ioceth->timer_mod);

	ioceth->stop_cbfn = NULL;
	ioceth->stop_cbarg = NULL;

	bna_attr_init(ioceth);

	bfa_fsm_set_state(ioceth, bna_ioceth_sm_stopped);
}

void
bna_ioceth_uninit(struct bna_ioceth_s *ioceth)
{
	CNA_ASSERT(ioceth->fsm == (bfa_sm_t)bna_ioceth_sm_stopped);
	CNA_ASSERT(ioceth->chld_stop_wc.wc_count == 0);

	bfa_cee_detach(&ioceth->bna->cee);

	bfa_ioc_detach(&ioceth->ioc);

	/**
	 * Sleep stress test for Windows issues a SurpriseRemoval in D3.
	 * This results in another call to ioceth_uninit which will require
	 * the bna instance to be sane. Comment out the reset of bna.
	 * This change does not affect any other OS.
	 */

	/* ioceth->bna = NULL; */
}

void
bna_bfi_attr_get_rsp(struct bna_ioceth_s *ioceth,
			struct bfi_msgq_mhdr_s *msghdr)
{
	struct bfi_enet_attr_rsp *rsp = (struct bfi_enet_attr_rsp *)msghdr;

	/**
	 * Store only if not set earlier, since BNAD can override the HW
	 * attributes
	 */
	if (!ioceth->attr.fw_query_complete) {
		ioceth->attr.num_txq = cna_os_ntohl(rsp->max_cfg);
		ioceth->attr.num_rxp = cna_os_ntohl(rsp->max_cfg);
		ioceth->attr.num_ucmac = cna_os_ntohl(rsp->max_ucmac);
		ioceth->attr.num_mcmac = BFI_ENET_MAX_MCAM;
		ioceth->attr.max_rit_size = cna_os_ntohl(rsp->rit_size);
		ioceth->attr.num_wol_magic = cna_os_ntohl(rsp->max_wol_magic);
		ioceth->attr.num_wol_frame = cna_os_ntohl(rsp->max_wol_frame);
		ioceth->attr.fw_query_complete = BFA_TRUE;
	}
	ioceth->attr.bw = cna_os_ntohl(rsp->bw);
	bnad_cb_bw_update(ioceth->bna->bnad, ioceth->attr.bw);

	bfa_fsm_send_event(ioceth, IOCETH_E_ENET_ATTR_RESP);
}

void
bna_ioceth_enable(struct bna_ioceth_s *ioceth)
{
	if (ioceth->fsm == (bfa_fsm_t)bna_ioceth_sm_ready) {
		bnad_cb_ioceth_ready(ioceth->bna->bnad);
		return;
	}

	if (ioceth->fsm == (bfa_fsm_t)bna_ioceth_sm_stopped) {
		bfa_fsm_send_event(ioceth, IOCETH_E_ENABLE);
	}
}

void
bna_ioceth_disable(struct bna_ioceth_s *ioceth, enum bna_cleanup_type_e type)
{
	if (type == BNA_SOFT_CLEANUP) {
		bnad_cb_ioceth_disabled(ioceth->bna->bnad);
		return;
	}

	CNA_ASSERT(ioceth->stop_cbfn == NULL);
	ioceth->stop_cbfn = bnad_cb_ioceth_disabled;
	ioceth->stop_cbarg = ioceth->bna->bnad;

	bfa_fsm_send_event(ioceth, IOCETH_E_DISABLE);
}

bfa_boolean_t
bna_ioceth_state_is_failed(struct bna_ioceth_s *ioceth)
{
	return (ioceth->fsm == (bfa_fsm_t)bna_ioceth_sm_failed) ?
		BFA_TRUE : BFA_FALSE;
}

static void
bna_ucam_mod_init(struct bna_ucam_mod_s *ucam_mod, struct bna_s *bna,
		  struct bna_res_info_s *res_info)
{
	int i;

	ucam_mod->ucmac = (struct bna_mac_s *)
	res_info[BNA_MOD_RES_MEM_T_UCMAC_ARRAY].res_u.mem_info.mdl[0].kva;

	bfa_q_init(&ucam_mod->free_q);
	for (i = 0; i < bna->ioceth.attr.num_ucmac; i++) {
		bfa_q_qe_init(&ucam_mod->ucmac[i].qe);
		bfa_q_enq(&ucam_mod->free_q, &ucam_mod->ucmac[i].qe);
	}

	/* A separate queue to allow synchronous setting of a list of MACs */
	bfa_q_init(&ucam_mod->del_q);
	for (i = i; i < (bna->ioceth.attr.num_ucmac * 2); i++) {
		bfa_q_qe_init(&ucam_mod->ucmac[i].qe);
		bfa_q_enq(&ucam_mod->del_q, &ucam_mod->ucmac[i].qe);
	}

	ucam_mod->bna = bna;
}

static void
bna_ucam_mod_uninit(struct bna_ucam_mod_s *ucam_mod)
{
	bfa_q_t *qe;
	int i;

	i = 0;
	bfa_q_iter(&ucam_mod->free_q, qe)
		i++;

	CNA_ASSERT(i == ucam_mod->bna->ioceth.attr.num_ucmac);

	i = 0;
	bfa_q_iter(&ucam_mod->del_q, qe)
		i++;

	CNA_ASSERT(i == ucam_mod->bna->ioceth.attr.num_ucmac);


	ucam_mod->bna = NULL;
}

static void
bna_mcam_mod_init(struct bna_mcam_mod_s *mcam_mod, struct bna_s *bna,
		  struct bna_res_info_s *res_info)
{
	int i;

	mcam_mod->mcmac = (struct bna_mac_s *)
	res_info[BNA_MOD_RES_MEM_T_MCMAC_ARRAY].res_u.mem_info.mdl[0].kva;

	bfa_q_init(&mcam_mod->free_q);
	for (i = 0; i < bna->ioceth.attr.num_mcmac; i++) {
		bfa_q_qe_init(&mcam_mod->mcmac[i].qe);
		bfa_q_enq(&mcam_mod->free_q, &mcam_mod->mcmac[i].qe);
	}

	/* A separate queue to allow synchronous setting of a list of MACs */
	bfa_q_init(&mcam_mod->del_q);
	for (i = i; i < (bna->ioceth.attr.num_mcmac * 2); i++) {
		bfa_q_qe_init(&mcam_mod->mcmac[i].qe);
		bfa_q_enq(&mcam_mod->del_q, &mcam_mod->mcmac[i].qe);
	}

	mcam_mod->mchandle = (struct bna_mcam_handle_s *)
	res_info[BNA_MOD_RES_MEM_T_MCHANDLE_ARRAY].res_u.mem_info.mdl[0].kva;

	bfa_q_init(&mcam_mod->free_handle_q);
	for (i = 0; i < bna->ioceth.attr.num_mcmac; i++) {
		bfa_q_qe_init(&mcam_mod->mchandle[i].qe);
		bfa_q_enq(&mcam_mod->free_handle_q, &mcam_mod->mchandle[i].qe);
	}

	mcam_mod->bna = bna;
}

static void
bna_mcam_mod_uninit(struct bna_mcam_mod_s *mcam_mod)
{
	bfa_q_t *qe;
	int i;

	i = 0;
	bfa_q_iter(&mcam_mod->free_q, qe) i++;
	CNA_ASSERT(i == mcam_mod->bna->ioceth.attr.num_mcmac);

	i = 0;
	bfa_q_iter(&mcam_mod->del_q, qe) i++;
	CNA_ASSERT(i == mcam_mod->bna->ioceth.attr.num_mcmac);

	i = 0;
	bfa_q_iter(&mcam_mod->free_handle_q, qe) i++;
	CNA_ASSERT(i == mcam_mod->bna->ioceth.attr.num_mcmac);

	mcam_mod->bna = NULL;
}

static void
bna_wol_mod_init(struct bna_wol_mod_s *wol_mod, struct bna_s *bna,
		  struct bna_res_info_s *res_info)
{
	int i;

	bfa_q_init(&wol_mod->free_magic_q);
	if (bna->ioceth.attr.num_wol_magic) {
		wol_mod->wol_magic = (struct bna_wol_s *)
			res_info[BNA_MOD_RES_MEM_T_WOLMAGIC_ARRAY].
			res_u.mem_info.mdl[0].kva;
		for (i = 0; i < bna->ioceth.attr.num_wol_magic; i++) {
			bfa_q_qe_init(&wol_mod->wol_magic[i].qe);
			bfa_q_enq(&wol_mod->free_magic_q,
				&wol_mod->wol_magic[i].qe);
			wol_mod->wol_magic[i].index = i;
		}
	}
	bfa_q_init(&wol_mod->free_frame_q);
	if (bna->ioceth.attr.num_wol_frame) {
		wol_mod->wol_frame = (struct bna_wol_s *)
			res_info[BNA_MOD_RES_MEM_T_WOLFRAME_ARRAY].
			res_u.mem_info.mdl[0].kva;
		for (i = 0; i < bna->ioceth.attr.num_wol_frame; i++) {
			bfa_q_qe_init(&wol_mod->wol_frame[i].qe);
			bfa_q_enq(&wol_mod->free_frame_q,
				&wol_mod->wol_frame[i].qe);
			wol_mod->wol_frame[i].index = i;
		}
	}
	wol_mod->bna = bna;
}

static void
bna_wol_mod_uninit(struct bna_wol_mod_s *wol_mod)
{
	bfa_q_t *qe;
	int i = 0;

	bfa_q_iter(&wol_mod->free_magic_q, qe)
		i++;
	bfa_trc(wol_mod->bna, (uint64_t)i << 16 |
		wol_mod->bna->ioceth.attr.num_wol_magic);
	i = 0;
	bfa_q_iter(&wol_mod->free_frame_q, qe)
		i++;
	bfa_trc(wol_mod->bna, (uint64_t)i << 16 |
		wol_mod->bna->ioceth.attr.num_wol_frame);
	wol_mod->bna = NULL;
}

void
bna_res_req(struct bna_res_info_s *res_info)
{
	/* DMA memory for COMMON_MODULE */
	res_info[BNA_RES_MEM_T_COM].res_type = BNA_RES_T_MEM;
	res_info[BNA_RES_MEM_T_COM].res_u.mem_info.mem_type = BNA_MEM_T_DMA;
	res_info[BNA_RES_MEM_T_COM].res_u.mem_info.num = 1;
	res_info[BNA_RES_MEM_T_COM].res_u.mem_info.len = CNA_ALIGN(
				(bfa_diag_meminfo() +
				bfa_sfp_meminfo() +
				bfa_port_meminfo() +
				bfa_cee_meminfo() +
				bfa_flash_meminfo(BFA_FALSE) +
				bfa_phy_meminfo(BFA_FALSE) +
				bfa_fru_meminfo(BFA_FALSE) +
				bfa_ablk_meminfo() +
				bfa_msgq_meminfo()), CNA_PAGE_SIZE);

	/* DMA memory for retrieving IOC attributes */
	res_info[BNA_RES_MEM_T_ATTR].res_type = BNA_RES_T_MEM;
	res_info[BNA_RES_MEM_T_ATTR].res_u.mem_info.mem_type = BNA_MEM_T_DMA;
	res_info[BNA_RES_MEM_T_ATTR].res_u.mem_info.num = 1;
	res_info[BNA_RES_MEM_T_ATTR].res_u.mem_info.len =
				CNA_ALIGN(bfa_ioc_meminfo(), CNA_PAGE_SIZE);

	/* Virtual memory for retreiving fw_trc */
	res_info[BNA_RES_MEM_T_FWTRC].res_type = BNA_RES_T_MEM;
	res_info[BNA_RES_MEM_T_FWTRC].res_u.mem_info.mem_type = BNA_MEM_T_KVA;
	res_info[BNA_RES_MEM_T_FWTRC].res_u.mem_info.num = 1;
	res_info[BNA_RES_MEM_T_FWTRC].res_u.mem_info.len =
				bfa_ioc_debug_trcsz(BFA_TRUE);

	/* DMA memory for retreiving stats */
	res_info[BNA_RES_MEM_T_STATS].res_type = BNA_RES_T_MEM;
	res_info[BNA_RES_MEM_T_STATS].res_u.mem_info.mem_type = BNA_MEM_T_DMA;
	res_info[BNA_RES_MEM_T_STATS].res_u.mem_info.num = 1;
	res_info[BNA_RES_MEM_T_STATS].res_u.mem_info.len =
				CNA_ALIGN(sizeof(struct bfi_enet_stats),
					CNA_PAGE_SIZE);
}

void
bna_mod_res_req(struct bna_s *bna, struct bna_res_info_s *res_info)
{
	struct bna_attr_s *attr = &bna->ioceth.attr;

	/* Virtual memory for Tx objects - stored by Tx module */
	CNA_ASSERT(attr->num_txq);
	res_info[BNA_MOD_RES_MEM_T_TX_ARRAY].res_type = BNA_RES_T_MEM;
	res_info[BNA_MOD_RES_MEM_T_TX_ARRAY].res_u.mem_info.mem_type =
		BNA_MEM_T_KVA;
	res_info[BNA_MOD_RES_MEM_T_TX_ARRAY].res_u.mem_info.num = 1;
	res_info[BNA_MOD_RES_MEM_T_TX_ARRAY].res_u.mem_info.len =
		attr->num_txq * sizeof(struct bna_tx_s);

	/* Virtual memory for TxQ - stored by Tx module */
	res_info[BNA_MOD_RES_MEM_T_TXQ_ARRAY].res_type = BNA_RES_T_MEM;
	res_info[BNA_MOD_RES_MEM_T_TXQ_ARRAY].res_u.mem_info.mem_type =
		BNA_MEM_T_KVA;
	res_info[BNA_MOD_RES_MEM_T_TXQ_ARRAY].res_u.mem_info.num = 1;
	res_info[BNA_MOD_RES_MEM_T_TXQ_ARRAY].res_u.mem_info.len =
		attr->num_txq * sizeof(struct bna_txq_s);

	/* Virtual memory for Rx objects - stored by Rx module */
	CNA_ASSERT(attr->num_rxp);
	res_info[BNA_MOD_RES_MEM_T_RX_ARRAY].res_type = BNA_RES_T_MEM;
	res_info[BNA_MOD_RES_MEM_T_RX_ARRAY].res_u.mem_info.mem_type =
		BNA_MEM_T_KVA;
	res_info[BNA_MOD_RES_MEM_T_RX_ARRAY].res_u.mem_info.num = 1;
	res_info[BNA_MOD_RES_MEM_T_RX_ARRAY].res_u.mem_info.len =
		attr->num_rxp * sizeof(struct bna_rx_s);

	/* Virtual memory for RxPath - stored by Rx module */
	res_info[BNA_MOD_RES_MEM_T_RXP_ARRAY].res_type = BNA_RES_T_MEM;
	res_info[BNA_MOD_RES_MEM_T_RXP_ARRAY].res_u.mem_info.mem_type =
		BNA_MEM_T_KVA;
	res_info[BNA_MOD_RES_MEM_T_RXP_ARRAY].res_u.mem_info.num = 1;
	res_info[BNA_MOD_RES_MEM_T_RXP_ARRAY].res_u.mem_info.len =
		attr->num_rxp * sizeof(struct bna_rxp_s);

	/* Virtual memory for RxQ - stored by Rx module */
	res_info[BNA_MOD_RES_MEM_T_RXQ_ARRAY].res_type = BNA_RES_T_MEM;
	res_info[BNA_MOD_RES_MEM_T_RXQ_ARRAY].res_u.mem_info.mem_type =
		BNA_MEM_T_KVA;
	res_info[BNA_MOD_RES_MEM_T_RXQ_ARRAY].res_u.mem_info.num = 1;
	res_info[BNA_MOD_RES_MEM_T_RXQ_ARRAY].res_u.mem_info.len =
		(attr->num_rxp * 2) * sizeof(struct bna_rxq_s);

	/* Virtual memory for Unicast MAC address - stored by ucam module */
	CNA_ASSERT(attr->num_ucmac);
	res_info[BNA_MOD_RES_MEM_T_UCMAC_ARRAY].res_type = BNA_RES_T_MEM;
	res_info[BNA_MOD_RES_MEM_T_UCMAC_ARRAY].res_u.mem_info.mem_type =
		BNA_MEM_T_KVA;
	res_info[BNA_MOD_RES_MEM_T_UCMAC_ARRAY].res_u.mem_info.num = 1;
	res_info[BNA_MOD_RES_MEM_T_UCMAC_ARRAY].res_u.mem_info.len =
		(attr->num_ucmac * 2) * sizeof(struct bna_mac_s);

	/* Virtual memory for Multicast MAC address - stored by mcam module */
	res_info[BNA_MOD_RES_MEM_T_MCMAC_ARRAY].res_type = BNA_RES_T_MEM;
	res_info[BNA_MOD_RES_MEM_T_MCMAC_ARRAY].res_u.mem_info.mem_type =
		BNA_MEM_T_KVA;
	res_info[BNA_MOD_RES_MEM_T_MCMAC_ARRAY].res_u.mem_info.num = 1;
	res_info[BNA_MOD_RES_MEM_T_MCMAC_ARRAY].res_u.mem_info.len =
		(attr->num_mcmac * 2) * sizeof(struct bna_mac_s);

	/* Virtual memory for Multicast handle - stored by mcam module */
	res_info[BNA_MOD_RES_MEM_T_MCHANDLE_ARRAY].res_type = BNA_RES_T_MEM;
	res_info[BNA_MOD_RES_MEM_T_MCHANDLE_ARRAY].res_u.mem_info.mem_type =
		BNA_MEM_T_KVA;
	res_info[BNA_MOD_RES_MEM_T_MCHANDLE_ARRAY].res_u.mem_info.num = 1;
	res_info[BNA_MOD_RES_MEM_T_MCHANDLE_ARRAY].res_u.mem_info.len =
		attr->num_mcmac * sizeof(struct bna_mcam_handle_s);

	/* Virtual memory for WOL - stored by wol module */
	res_info[BNA_MOD_RES_MEM_T_WOLMAGIC_ARRAY].res_type = BNA_RES_T_MEM;
	res_info[BNA_MOD_RES_MEM_T_WOLMAGIC_ARRAY].res_u.mem_info.mem_type =
		BNA_MEM_T_KVA;
	res_info[BNA_MOD_RES_MEM_T_WOLMAGIC_ARRAY].res_u.mem_info.num = 1;
	res_info[BNA_MOD_RES_MEM_T_WOLMAGIC_ARRAY].res_u.mem_info.len =
		attr->num_wol_magic * sizeof(struct bna_wol_s);
	res_info[BNA_MOD_RES_MEM_T_WOLFRAME_ARRAY].res_type = BNA_RES_T_MEM;
	res_info[BNA_MOD_RES_MEM_T_WOLFRAME_ARRAY].res_u.mem_info.mem_type =
		BNA_MEM_T_KVA;
	res_info[BNA_MOD_RES_MEM_T_WOLFRAME_ARRAY].res_u.mem_info.num = 1;
	res_info[BNA_MOD_RES_MEM_T_WOLFRAME_ARRAY].res_u.mem_info.len =
		attr->num_wol_frame * sizeof(struct bna_wol_s);
}

void
bna_init(struct bna_s *bna, struct bnad_s *bnad, struct bna_ident_s *ident,
		struct bfa_pcidev_s *pcidev, struct bna_res_info_s *res_info)
{
	bna->bnad = bnad;
	bfa_os_assign(bna->ident, *ident);
	bfa_os_assign(bna->pcidev, *pcidev);

	bna->stats.hw_stats_kva = (struct bfi_enet_stats *)
		res_info[BNA_RES_MEM_T_STATS].res_u.mem_info.mdl[0].kva;
	bna->stats.hw_stats_dma.msb =
		res_info[BNA_RES_MEM_T_STATS].res_u.mem_info.mdl[0].dma.msb;
	bna->stats.hw_stats_dma.lsb =
		res_info[BNA_RES_MEM_T_STATS].res_u.mem_info.mdl[0].dma.lsb;

	bna_reg_addr_init(bna, &bna->pcidev);

	/* Also initializes diag, cee, sfp, phy_port, msgq */
	bna_ioceth_init(&bna->ioceth, bna, res_info);

	bna_stats_mod_init(&bna->stats_mod, bna);
	bna_enet_init(&bna->enet, bna);
	bna_ethport_init(&bna->ethport, bna);
}

void
bna_mod_init(struct bna_s *bna, struct bna_res_info_s *res_info)
{
	bna_tx_mod_init(&bna->tx_mod, bna, res_info);

	bna_rx_mod_init(&bna->rx_mod, bna, res_info);

	bna_ucam_mod_init(&bna->ucam_mod, bna, res_info);

	bna_mcam_mod_init(&bna->mcam_mod, bna, res_info);

	bna_wol_mod_init(&bna->wol_mod, bna, res_info);

	bna->default_mode_rid = BFI_INVALID_RID;
	bna->promisc_rid = BFI_INVALID_RID;

	bna->mod_flags |= BNA_MOD_F_INIT_DONE;
}

void
bna_uninit(struct bna_s *bna)
{
	if (bna->mod_flags & BNA_MOD_F_INIT_DONE) {
		bna_mcam_mod_uninit(&bna->mcam_mod);
		bna_ucam_mod_uninit(&bna->ucam_mod);
		bna_wol_mod_uninit(&bna->wol_mod);
		bna_rx_mod_uninit(&bna->rx_mod);
		bna_tx_mod_uninit(&bna->tx_mod);
		bna->mod_flags &= ~BNA_MOD_F_INIT_DONE;
	}

	bna_stats_mod_uninit(&bna->stats_mod);
	bna_ethport_uninit(&bna->ethport);
	bna_enet_uninit(&bna->enet);

	bna_ioceth_uninit(&bna->ioceth);

	bna->bnad = NULL;
}

int
bna_num_txq_set(struct bna_s *bna, int num_txq)
{
	if (bna->ioceth.attr.fw_query_complete &&
		(num_txq <= bna->ioceth.attr.num_txq)) {
		bna->ioceth.attr.num_txq = num_txq;
		return BNA_CB_SUCCESS;
	}

	return BNA_CB_FAIL;
}

int
bna_num_rxp_set(struct bna_s *bna, int num_rxp)
{
	if (bna->ioceth.attr.fw_query_complete &&
		(num_rxp <= bna->ioceth.attr.num_rxp)) {
		bna->ioceth.attr.num_rxp = num_rxp;
		return BNA_CB_SUCCESS;
	}

	return BNA_CB_FAIL;
}

struct bna_mac_s *
bna_cam_mod_mac_get(bfa_q_t *head)
{
	bfa_q_t *qe;

	if (bfa_q_is_empty(head))
		return NULL;

	bfa_q_deq(head, &qe);
	return (struct bna_mac_s *)qe;
}

void
bna_cam_mod_mac_put(bfa_q_t *tail, struct bna_mac_s *mac)
{
	bfa_q_enq(tail, &mac->qe);
}

struct bna_mcam_handle_s *
bna_mcam_mod_handle_get(struct bna_mcam_mod_s *mcam_mod)
{
	bfa_q_t *qe;

	if (bfa_q_is_empty(&mcam_mod->free_handle_q))
		return NULL;

	bfa_q_deq(&mcam_mod->free_handle_q, &qe);

	return (struct bna_mcam_handle_s *)qe;
}

void
bna_mcam_mod_handle_put(struct bna_mcam_mod_s *mcam_mod,
			struct bna_mcam_handle_s *handle)
{
	CNA_ASSERT(handle->refcnt == 0);
	bfa_q_enq(&mcam_mod->free_handle_q, &handle->qe);
}

struct bna_wol_s *
bna_wol_mod_magic_get(struct bna_wol_mod_s *wol_mod)
{
	bfa_q_t *qe;
	struct bna_wol_s *wol;

	if (bfa_q_is_empty(&wol_mod->free_magic_q))
		return NULL;

	bfa_q_deq(&wol_mod->free_magic_q, &qe);

	wol = (struct bna_wol_s *)qe;
	return wol;
}

void
bna_wol_mod_magic_put(struct bna_wol_mod_s *wol_mod, struct bna_wol_s *wol)
{
	wol->os_id = 0;
	bfa_q_enq(&wol_mod->free_magic_q, &wol->qe);
}

struct bna_wol_s *
bna_wol_mod_frame_get(struct bna_wol_mod_s *wol_mod)
{
	bfa_q_t *qe;
	struct bna_wol_s *wol;

	if (bfa_q_is_empty(&wol_mod->free_frame_q))
		return NULL;

	bfa_q_deq(&wol_mod->free_frame_q, &qe);

	wol = (struct bna_wol_s *)qe;
	return wol;
}

void
bna_wol_mod_frame_put(struct bna_wol_mod_s *wol_mod, struct bna_wol_s *wol)
{
	wol->os_id = 0;
	bfa_q_enq(&wol_mod->free_frame_q, &wol->qe);
}

void
bna_trc_log_aen_set(struct bna_s *bna, struct bfa_trc_mod_s *trcmod,
		struct bfa_log_mod_s *logm, struct bfa_aen_s *aen)
{
	bna->trcmod = trcmod;
	bfa_trc(bna, bna->ident.id);
	bfa_trc(bna, bna->pcidev.pci_func);

	bna->logm = logm;
	bna->aen = aen;
}

void
bna_hw_stats_get(struct bna_s *bna)
{
	if (!bna->stats_mod.ioc_ready) {
		bnad_cb_stats_get(bna->bnad, BNA_CB_FAIL, &bna->stats);
		return;
	}
	if (bna->stats_mod.stats_get_busy) {
		bnad_cb_stats_get(bna->bnad, BNA_CB_BUSY, &bna->stats);
		return;
	}

	bna_bfi_stats_get(bna);
}

void
bna_hw_stats_clr(struct bna_s *bna)
{
	cna_os_memset(&bna->stats.hw_stats, 0, sizeof(struct bfi_enet_stats));
	if (bna->stats_mod.ioc_ready && !bna->stats_mod.stats_clr_busy)
		bna_bfi_stats_clr(bna, BFI_ENET_STATS_ALL, bna->tx_mod.rid_mask,
			bna->rx_mod.rid_mask);
}

static void
bna_hw_tx_stats_clr(struct bna_tx_s *tx)
{
	struct bfi_enet_stats *stats = &tx->bna->stats.hw_stats;

	cna_os_memset(&stats->txf_stats[tx->rid], 0,
		sizeof(struct bfi_enet_stats_txf));
	if (tx->bna->stats_mod.ioc_ready && !tx->bna->stats_mod.stats_clr_busy)
		bna_bfi_stats_clr(tx->bna, 0, (1 << tx->rid), 0);
}

static void
bna_hw_rx_stats_clr(struct bna_rx_s *rx)
{
	struct bfi_enet_stats *stats = &rx->bna->stats.hw_stats;

	cna_os_memset(&stats->rxf_stats[rx->rid], 0,
		sizeof(struct bfi_enet_stats_rxf));
	if (rx->bna->stats_mod.ioc_ready && !rx->bna->stats_mod.stats_clr_busy)
		bna_bfi_stats_clr(rx->bna, 0, 0, (1 << rx->rid));
}

static void
bna_rxp_info_get(struct bna_rxp_s *rxp, struct bfa_vnic_rxp_info_s *rxp_info)
{
	struct bfa_vnic_rxq_info_s *rxq_info;

	rxp_info->cq_id = rxp->cq.ccb->id;
	rxp_info->hw_cq_id = rxp->hw_id;
	rxp_info->msix_vector =
		(rxp->cq.ccb->intr_type == BNA_INTR_T_INTX) ?
		-1 : rxp->cq.ccb->intr_vector;
	if (rxp->type == BNA_RXP_SINGLE) {
		rxp_info->num_rxq = 1;
		rxq_info = &rxp_info->rxq_info[0];
		rxq_info->id = rxp->cq.ccb->rcb[0]->id;
		rxq_info->hw_id = rxp->cq.ccb->rcb[0]->rxq->hw_id;
		rxq_info->buf_size = rxp->rxq.single.only->buffer_size;
		rxq_info->q_depth = rxp->cq.ccb->rcb[0]->q_depth;
	} else {
		rxp_info->num_rxq = 2;

		rxq_info = &rxp_info->rxq_info[0];
		rxq_info->id = rxp->cq.ccb->rcb[0]->id;
		rxq_info->hw_id = rxp->cq.ccb->rcb[0]->rxq->hw_id;
		rxq_info->buf_size = rxp->rxq.slr.large->buffer_size;
		rxq_info->q_depth = rxp->cq.ccb->rcb[0]->q_depth;

		rxq_info = &rxp_info->rxq_info[1];
		rxq_info->id = rxp->cq.ccb->rcb[1]->id;
		rxq_info->hw_id = rxp->cq.ccb->rcb[1]->rxq->hw_id;
		rxq_info->buf_size = rxp->rxq.slr.small->buffer_size;
		rxq_info->q_depth = rxp->cq.ccb->rcb[1]->q_depth;
	}
}

static void
bna_rxf_ucmac_list_get(struct bna_rx_s *rx,
	struct bfa_vnic_rxf_info_s *rxf_info)
{
	struct bna_mac_s *mac;
	bfa_q_t *qe;

	rxf_info->num_ucmac = 0;
	if (rx->rxf.ucast_pending_mac) {
		cna_os_memcpy(&rxf_info->ucmac_list[rxf_info->num_ucmac],
			rx->rxf.ucast_pending_mac->addr, sizeof(mac_t));
		rxf_info->num_ucmac++;
	}
	bfa_q_iter(&rx->rxf.ucast_pending_add_q, qe) {
		if (rxf_info->num_ucmac >= BFA_VNIC_MAX_UCMAC)
			break;
		mac = (struct bna_mac_s *)qe;
		cna_os_memcpy(&rxf_info->ucmac_list[rxf_info->num_ucmac],
			mac->addr, sizeof(mac_t));
		rxf_info->num_ucmac++;
	}
	bfa_q_iter(&rx->rxf.ucast_active_q, qe) {
		if (rxf_info->num_ucmac >= BFA_VNIC_MAX_UCMAC)
			break;
		mac = (struct bna_mac_s *)qe;
		cna_os_memcpy(&rxf_info->ucmac_list[rxf_info->num_ucmac],
			mac->addr, sizeof(mac_t));
		rxf_info->num_ucmac++;
	}
}

static void
bna_rxf_mcmac_list_get(struct bna_rx_s *rx,
	struct bfa_vnic_rxf_info_s *rxf_info)
{
	struct bna_mac_s *mac;
	bfa_q_t *qe;

	rxf_info->num_mcmac = 0;
	bfa_q_iter(&rx->rxf.mcast_pending_add_q, qe) {
		if (rxf_info->num_mcmac >= BFA_VNIC_MAX_MCMAC)
			break;
		mac = (struct bna_mac_s *)qe;
		cna_os_memcpy(&rxf_info->mcmac_list[rxf_info->num_mcmac],
			mac->addr, sizeof(mac_t));
		rxf_info->num_mcmac++;
	}
	bfa_q_iter(&rx->rxf.mcast_active_q, qe) {
		if (rxf_info->num_mcmac >= BFA_VNIC_MAX_MCMAC)
			break;
		mac = (struct bna_mac_s *)qe;
		cna_os_memcpy(&rxf_info->mcmac_list[rxf_info->num_mcmac],
			mac->addr, sizeof(mac_t));
		rxf_info->num_mcmac++;
	}
}

static void
bna_rxf_info_get(struct bna_rx_s *rx, struct bfa_vnic_rxf_info_s *rxf_info)
{
	struct bfa_vnic_rxp_info_s *rxp_info;
	struct bna_rxp_s *rxp;
	bfa_q_t *qe;
	int i;

	rxf_info->id = rx->rid;
	rxf_info->hw_id = rx->hw_id;

	bna_rxf_ucmac_list_get(rx, rxf_info);

	bna_rxf_mcmac_list_get(rx, rxf_info);

	rxf_info->promisc_status =
		(rx->rxf.rxmode_active & BNA_RXMODE_PROMISC) ? 1 : 0;
	rxf_info->default_status =
		(rx->rxf.rxmode_active & BNA_RXMODE_DEFAULT) ? 1 : 0;
	rxf_info->vlan_filter_status = (uint8_t)rx->rxf.vlan_filter_status;
	rxf_info->rss_status = (uint8_t)rx->rxf.rss_status;
	rxf_info->rit_size = rx->rxf.rit_size;
	memcpy(rxf_info->rit, rx->rxf.rit, BFI_ENET_RSS_RIT_MAX);
	rxf_info->num_rxp = rx->num_paths;
	i = 0;
	bfa_q_iter(&rx->rxp_q, qe) {
		rxp = (struct bna_rxp_s *)qe;
		rxp_info = &rxf_info->rxp_info[i];
		bna_rxp_info_get(rxp, rxp_info);
		i++;
	}
}

static void
bna_txf_info_get(struct bna_tx_s *tx, struct bfa_vnic_txf_info_s *txf_info)
{
	struct bfa_vnic_txq_info_s *txq_info;
	struct bna_txq_s *txq;
	bfa_q_t *qe;
	int i;

	txf_info->id = tx->rid;
	txf_info->hw_id = tx->hw_id;
	txf_info->num_txq = tx->num_txq;
	i = 0;
	bfa_q_iter(&tx->txq_q, qe) {
		txq = (struct bna_txq_s *)qe;
		txq_info = &txf_info->txq_info[i];
		txq_info->id = txq->tcb->id;
		txq_info->hw_id = txq->hw_id;
		txq_info->msix_vector =
			(txq->tcb->intr_type == BNA_INTR_T_INTX) ? -1 :
			txq->tcb->intr_vector;
		txq_info->priority = txq->tcb->priority;
		txq_info->q_depth = txq->tcb->q_depth;
		i++;
	}
}

void
bna_vnic_attr_get(struct bna_s *bna, struct bfa_vnic_attr_s *attr)
{
	struct bna_tx_s *tx;
	struct bfa_vnic_txf_info_s *txf_info;
	struct bna_rx_s *rx;
	struct bfa_vnic_rxf_info_s *rxf_info;
	bfa_q_t *qe;
	int i;

	attr->mtu = bna->enet.mtu;
	attr->iscsi_prio = bna->tx_mod.iscsi_prio;
	attr->def_nw_prio = bna->tx_mod.default_prio;

	/* Txf */
	i = 0;
	if (bna->mod_flags & BNA_MOD_F_INIT_DONE) {
		bfa_q_iter(&bna->tx_mod.tx_active_q, qe) {
			tx = (struct bna_tx_s *)qe;
			txf_info = &attr->txf_info[i];
			bna_txf_info_get(tx, txf_info);
			i++;
		}
	}
	attr->num_txf = i;

	/* Rxf */
	i = 0;
	if (bna->mod_flags & BNA_MOD_F_INIT_DONE) {
		bfa_q_iter(&bna->rx_mod.rx_active_q, qe) {
			rx = (struct bna_rx_s *)qe;
			rxf_info = &attr->rxf_info[i];
			bna_rxf_info_get(rx, rxf_info);
			i++;
		}
	}
	attr->num_rxf = i;

	bnad_vnic_attr_get(bna->bnad, attr);
}

static void
bna_txq_stats_get(struct bna_txq_s *txq, struct bfa_vnic_stats_txq_s *txq_stats)
{
	struct bna_tcb_s *tcb = txq->tcb;

	txq_stats->id = tcb->id;
	txq_stats->tx_stop = 0;
	txq_stats->tx_resume = 0;
	txq_stats->lso4 = 0;
	txq_stats->lso6 = 0;
	txq_stats->lso_errors = 0;
	txq_stats->tx_bytes = txq->tx_bytes;
	txq_stats->tx_packets = txq->tx_packets;
	txq_stats->ip_cso = 0;
	txq_stats->tcp_cso = 0;
	txq_stats->udp_cso = 0;
	txq_stats->cso_errors = 0;
	txq_stats->out_of_wi = 0;
	txq_stats->dma_map_errors = 0;
	txq_stats->prod_idx = tcb->producer_index;
	txq_stats->cons_idx = tcb->consumer_index;
	txq_stats->unmap_prod_idx = 0;
	txq_stats->unmap_cons_idx = 0;
	txq_stats->hw_cons_idx = cna_mem_readw(tcb->hw_consumer_index);
}

static bfa_boolean_t
bna_txf_stats_get(struct bna_tx_s *tx, struct bfa_vnic_stats_txf_s *txf_stats)
{
	struct bna_s *bna = tx->bna;
	struct bfi_enet_stats_txf *hw_txf_stats;

	memset(txf_stats, 0, sizeof(struct bfa_vnic_stats_txf_s));
	txf_stats->id = tx->rid;

	if (!(bna->tx_mod.rid_mask & (1 << tx->rid)))
		return BFA_FALSE;

	hw_txf_stats = &bna->stats.hw_stats.txf_stats[tx->rid];
	txf_stats->ucast_octets 	= hw_txf_stats->ucast_octets;
	txf_stats->ucast 		= hw_txf_stats->ucast;
	txf_stats->ucast_vlan 		= hw_txf_stats->ucast_vlan;
	txf_stats->mcast_octets 	= hw_txf_stats->mcast_octets;
	txf_stats->mcast 		= hw_txf_stats->mcast;
	txf_stats->mcast_vlan 		= hw_txf_stats->mcast_vlan;
	txf_stats->bcast_octets 	= hw_txf_stats->bcast_octets;
	txf_stats->bcast 		= hw_txf_stats->bcast;
	txf_stats->bcast_vlan 		= hw_txf_stats->bcast_vlan;
	txf_stats->errors 		= hw_txf_stats->errors;
	txf_stats->filter_vlan 		= hw_txf_stats->filter_vlan;
	txf_stats->filter_mac_sa 	= hw_txf_stats->filter_mac_sa;

	return BFA_TRUE;
}

static void
bna_rxp_stats_get(struct bna_rxp_s *rxp, struct bfa_vnic_stats_rxp_s *rxp_stats)
{
	struct bna_ccb_s *ccb = rxp->cq.ccb;
	struct bna_rxq_s *rxq;
	struct bna_rcb_s *rcb;
	struct bfa_vnic_stats_rxq_s *rxq_stats;
	int i;

	rxp_stats->id = ccb->id;
	rxp_stats->cq_prod_idx = ccb->producer_index;
	rxp_stats->cq_hw_idx = cna_mem_readw(ccb->hw_producer_index);
	rxp_stats->num_rxq = (ccb->rcb[1]) ? 2 : 1;

	for (i = 0; i < rxp_stats->num_rxq; i++) {
		rxq_stats = &rxp_stats->rxq_stats[i];
		rcb = ccb->rcb[i];
		rxq = rcb->rxq;
		rxq_stats->id = rcb->id;
		rxq_stats->rx_cleanup = 0;
		rxq_stats->rx_post = 0;
		rxq_stats->rx_schedule = 0;
		rxq_stats->low_buf = 0;
		rxq_stats->buf_alloc_fail = rxq->rxbuf_alloc_failed;
		rxq_stats->rx_bytes = rxq->rx_bytes;
		rxq_stats->rx_packets = rxq->rx_packets;
		rxq_stats->rx_mac_err = 0;
		rxq_stats->rx_csum_err = 0;
		rxq_stats->lro = 0;
		rxq_stats->lro_flush = 0;
		rxq_stats->prod_idx = rcb->producer_index;
		rxq_stats->cons_idx = rcb->consumer_index;
	}
}

static bfa_boolean_t
bna_rxf_stats_get(struct bna_rx_s *rx, struct bfa_vnic_stats_rxf_s *rxf_stats)
{
	struct bna_s *bna = rx->bna;
	struct bfi_enet_stats_rxf *hw_rxf_stats;

	memset(rxf_stats, 0, sizeof(struct bfa_vnic_stats_rxf_s));
	rxf_stats->id = rx->rid;

	if (!(bna->rx_mod.rid_mask & (1 << rx->rid)))
		return BFA_FALSE;

	hw_rxf_stats = &bna->stats.hw_stats.rxf_stats[rx->rid];
	rxf_stats->ucast_octets 	= hw_rxf_stats->ucast_octets;
	rxf_stats->ucast 		= hw_rxf_stats->ucast;
	rxf_stats->ucast_vlan 		= hw_rxf_stats->ucast_vlan;
	rxf_stats->mcast_octets 	= hw_rxf_stats->mcast_octets;
	rxf_stats->mcast 		= hw_rxf_stats->mcast;
	rxf_stats->mcast_vlan 		= hw_rxf_stats->mcast_vlan;
	rxf_stats->bcast_octets 	= hw_rxf_stats->bcast_octets;
	rxf_stats->bcast 		= hw_rxf_stats->bcast;
	rxf_stats->bcast_vlan 		= hw_rxf_stats->bcast_vlan;
	rxf_stats->frame_drops 		= hw_rxf_stats->frame_drops;

	return BFA_TRUE;
}

static void
bna_rad_stats_copy(struct bfa_vnic_stats_rad_s *dst,
	struct bfi_enet_stats_rad *src)
{
	dst->rx_frames = src->rx_frames;
	dst->rx_octets = src->rx_octets;
	dst->rx_vlan_frames = src->rx_vlan_frames;
	dst->rx_ucast = src->rx_ucast;
	dst->rx_ucast_octets = src->rx_ucast_octets;
	dst->rx_ucast_vlan = src->rx_ucast_vlan;
	dst->rx_mcast = src->rx_mcast;
	dst->rx_mcast_octets = src->rx_mcast_octets;
	dst->rx_mcast_vlan = src->rx_mcast_vlan;
	dst->rx_bcast = src->rx_bcast;
	dst->rx_bcast_octets = src->rx_bcast_octets;
	dst->rx_bcast_vlan = src->rx_bcast_vlan;
	dst->rx_drops = src->rx_drops;
}

void
bna_vnic_stats_get(struct bna_s *bna, struct bfa_vnic_stats_s *stats)
{
	struct bna_tx_s *tx;
	struct bna_rx_s *rx;
	struct bfi_enet_stats_bpc *hw_bpc_stats;
	struct bfi_enet_stats_rad *hw_rad_stats;
	struct bfi_enet_stats_rad *hw_rlb_stats;
	struct bna_rxp_s *rxp;
	struct bfa_vnic_stats_rxp_s *rxp_stats;
	struct bna_txq_s *txq;
	struct bfa_vnic_stats_txq_s *txq_stats;
	bfa_q_t *qe, *rxp_qe, *txq_qe;
	int i, j, k;
	bfa_boolean_t valid_stats;

	hw_bpc_stats = &bna->stats.hw_stats.bpc_stats;
	hw_rad_stats = &bna->stats.hw_stats.rad_stats;
	hw_rlb_stats = &bna->stats.hw_stats.rlb_stats;

	memcpy(stats->bpc_stats.tx_pause, hw_bpc_stats->tx_pause,
		sizeof(uint64_t) * 8);
	memcpy(stats->bpc_stats.tx_zero_pause, hw_bpc_stats->tx_zero_pause,
		sizeof(uint64_t) * 8);
	memcpy(stats->bpc_stats.tx_first_pause, hw_bpc_stats->tx_first_pause,
		sizeof(uint64_t) * 8);
	memcpy(stats->bpc_stats.rx_pause, hw_bpc_stats->rx_pause,
		sizeof(uint64_t) * 8);
	memcpy(stats->bpc_stats.rx_zero_pause, hw_bpc_stats->rx_zero_pause,
		sizeof(uint64_t) * 8);
	memcpy(stats->bpc_stats.rx_first_pause, hw_bpc_stats->rx_first_pause,
		sizeof(uint64_t) * 8);

	bna_rad_stats_copy(&stats->rad_stats, hw_rad_stats);
	bna_rad_stats_copy(&stats->rlb_stats, hw_rlb_stats);

	/**
	 * Assumption: txf:txq is either 1:1 or 1:n where n < 8
	 */
	i = 0;
	j = 0;
	if (bna->mod_flags & BNA_MOD_F_INIT_DONE) {
		bfa_q_iter(&bna->tx_mod.tx_active_q, qe) {
			tx = (struct bna_tx_s *)qe;
			valid_stats = bna_txf_stats_get(tx,
						&stats->txf_stats[i]);

			k = 0;
			if (valid_stats) {
				bfa_q_iter(&tx->txq_q, txq_qe) {
					txq = (struct bna_txq_s *)txq_qe;
					txq_stats = &stats->txq_stats[j + k];
					bna_txq_stats_get(txq, txq_stats);
					k++;
				}
			}
			stats->txf_stats[i].num_txq = k;
			j += k;
			i++;
		}
	}
	stats->num_txf = i;

	/**
	 * Assumption: rxf:rxp is either 1:1 or 1:n where n < 16
	 */
	i = 0;
	j = 0;
	if (bna->mod_flags & BNA_MOD_F_INIT_DONE) {
		bfa_q_iter(&bna->rx_mod.rx_active_q, qe) {
			rx = (struct bna_rx_s *)qe;
			valid_stats = bna_rxf_stats_get(rx,
						&stats->rxf_stats[i]);

			k = 0;
			if (valid_stats == BFA_TRUE) {
				bfa_q_iter(&rx->rxp_q, rxp_qe) {
					rxp = (struct bna_rxp_s *)rxp_qe;
					rxp_stats = &stats->rxp_stats[j + k];
					bna_rxp_stats_get(rxp, rxp_stats);
					k++;
				}
			}
			stats->rxf_stats[i].num_rxp = k;
			j += k;

			i++;
		}
	}
	stats->num_rxf = i;

	bnad_vnic_stats_get(bna->bnad, stats);
}

void
bna_tx_stats_clr(struct bna_tx_s *tx)
{
	struct bna_txq_s *txq;
	bfa_q_t *qe;

	bfa_q_iter(&tx->txq_q, qe) {
		txq = (struct bna_txq_s *)qe;
		txq->tx_bytes = 0;
		txq->tx_packets = 0;
	}
	bna_hw_tx_stats_clr(tx);
}

void
bna_rx_stats_clr(struct bna_rx_s *rx)
{
	struct bna_rxp_s *rxp;
	struct bna_ccb_s *ccb;
	struct bna_rxq_s *rxq;
	bfa_q_t *qe;
	int num_rxq;
	int i;

	bfa_q_iter(&rx->rxp_q, qe) {
		rxp = (struct bna_rxp_s *)qe;
		ccb = rxp->cq.ccb;
		num_rxq = (ccb->rcb[1]) ? 2 : 1;
		for (i = 0; i < num_rxq; i++) {
			rxq = ccb->rcb[i]->rxq;
			rxq->rxbuf_alloc_failed = 0;
			rxq->rx_bytes = 0;
			rxq->rx_packets = 0;
		}
	}
	bna_hw_rx_stats_clr(rx);
}

void
bna_vnic_stats_clr(struct bna_s *bna)
{
	struct bna_tx_s *tx;
	struct bna_rx_s *rx;
	bfa_q_t *qe;

	bna_hw_stats_clr(bna);

	if (bna->mod_flags & BNA_MOD_F_INIT_DONE) {
		bfa_q_iter(&bna->tx_mod.tx_active_q, qe) {
			tx = (struct bna_tx_s *)qe;
			bna_tx_stats_clr(tx);
		}
	}

	if (bna->mod_flags & BNA_MOD_F_INIT_DONE) {
		bfa_q_iter(&bna->rx_mod.rx_active_q, qe) {
			rx = (struct bna_rx_s *)qe;
			bna_rx_stats_clr(rx);
		}
	}

	bnad_vnic_stats_clr(bna->bnad);
}
