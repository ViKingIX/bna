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
#include "bfi/bfi.h"

BNA_TRC_FILE(HAL, TXRX);

/**
 * IB
 */

void
bna_ib_coalescing_timeo_set(struct bna_ib_s *ib, uint8_t coalescing_timeo)
{
	ib->coalescing_timeo = coalescing_timeo;
	ib->door_bell.doorbell_ack = BNA_DOORBELL_IB_INT_ACK(
				(uint32_t)ib->coalescing_timeo, 0);
}

/**
 * RXF
 */

#define	call_rxf_start_cbfn(rxf)					\
do {									\
	if ((rxf)->start_cbfn) {					\
		void (*cbfn)(struct bna_rx_s *);			\
		struct bna_rx_s *cbarg;					\
		cbfn = (rxf)->start_cbfn;				\
		cbarg = (rxf)->start_cbarg;				\
		(rxf)->start_cbfn = NULL;				\
		(rxf)->start_cbarg = NULL;				\
		cbfn(cbarg);						\
	}								\
} while (0)

#define	call_rxf_stop_cbfn(rxf)						\
do {									\
	if ((rxf)->stop_cbfn) {						\
		void (*cbfn)(struct bna_rx_s *);			\
		struct bna_rx_s *cbarg;					\
		cbfn = (rxf)->stop_cbfn;				\
		cbarg = (rxf)->stop_cbarg;				\
		(rxf)->stop_cbfn = NULL;				\
		(rxf)->stop_cbarg = NULL;				\
		cbfn(cbarg);						\
	}								\
} while (0)

#define	call_rxf_cam_fltr_cbfn(rxf)					\
do {									\
	if ((rxf)->cam_fltr_cbfn) {					\
		void (*cbfn)(struct bnad_s *, struct bna_rx_s *);	\
		struct bnad_s *cbarg;					\
		cbfn = (rxf)->cam_fltr_cbfn;				\
		cbarg = (rxf)->cam_fltr_cbarg;				\
		(rxf)->cam_fltr_cbfn = NULL;				\
		(rxf)->cam_fltr_cbarg = NULL;				\
		cbfn(cbarg, rxf->rx);					\
	}								\
} while (0)

#define	call_rxf_pause_cbfn(rxf)					\
do {									\
	if ((rxf)->oper_state_cbfn) {					\
		void (*cbfn)(struct bnad_s *, struct bna_rx_s *);	\
		struct bnad_s *cbarg;					\
		cbfn = (rxf)->oper_state_cbfn;				\
		cbarg = (rxf)->oper_state_cbarg;			\
		(rxf)->oper_state_cbfn = NULL;				\
		(rxf)->oper_state_cbarg = NULL;				\
		cbfn(cbarg, rxf->rx);					\
	}								\
} while (0)

#define	call_rxf_resume_cbfn(rxf) call_rxf_pause_cbfn(rxf)

#define bna_rxf_vlan_cfg_soft_reset(rxf)				\
do {									\
	(rxf)->vlan_pending_bitmask = (uint8_t)BFI_VLAN_BMASK_ALL;	\
	(rxf)->vlan_strip_pending = BFA_TRUE;	\
} while (0)

#define bna_rxf_rss_cfg_soft_reset(rxf)					\
do {									\
	if ((rxf)->rss_status == BNA_STATUS_T_ENABLED)			\
		(rxf)->rss_pending = (BNA_RSS_F_RIT_PENDING | 		\
				BNA_RSS_F_CFG_PENDING |			\
				BNA_RSS_F_STATUS_PENDING);		\
} while (0)

static int bna_rxf_cfg_apply(struct bna_rxf_s *rxf);
static void bna_rxf_cfg_reset(struct bna_rxf_s *rxf);
static int bna_rxf_fltr_clear(struct bna_rxf_s *rxf);
static struct bna_rxp_s *bna_rx_get_rxp(struct bna_rx_s *rx, int vector);

enum bna_rxf_event_e {
	RXF_E_START			= 1,
	RXF_E_STOP			= 2,
	RXF_E_FAIL			= 3,
	RXF_E_CONFIG			= 4,
	RXF_E_PAUSE			= 5,
	RXF_E_RESUME			= 6,
	RXF_E_FW_RESP			= 7,
};

bfa_fsm_state_decl(bna_rxf, stopped, struct bna_rxf_s,
			enum bna_rxf_event_e);
bfa_fsm_state_decl(bna_rxf, paused, struct bna_rxf_s,
			enum bna_rxf_event_e);
bfa_fsm_state_decl(bna_rxf, cfg_wait, struct bna_rxf_s,
			enum bna_rxf_event_e);
bfa_fsm_state_decl(bna_rxf, started, struct bna_rxf_s,
			enum bna_rxf_event_e);
bfa_fsm_state_decl(bna_rxf, fltr_clr_wait, struct bna_rxf_s,
			enum bna_rxf_event_e);
bfa_fsm_state_decl(bna_rxf, last_resp_wait, struct bna_rxf_s,
			enum bna_rxf_event_e);

static void
bna_rxf_sm_stopped_entry(struct bna_rxf_s *rxf)
{
	bfa_trc(rxf->rx->bna, rxf->rx->rid);
	call_rxf_stop_cbfn(rxf);
	CNA_ASSERT(rxf->start_cbfn == NULL);
	CNA_ASSERT(rxf->cam_fltr_cbfn == NULL);
	CNA_ASSERT(rxf->oper_state_cbfn == NULL);
}

static void
bna_rxf_sm_stopped(struct bna_rxf_s *rxf, enum bna_rxf_event_e event)
{
	bfa_trc(rxf->rx->bna, event);
	switch (event) {
	case RXF_E_START:
		if (rxf->flags & BNA_RXF_F_PAUSED) {
			bfa_fsm_set_state(rxf, bna_rxf_sm_paused);
			call_rxf_start_cbfn(rxf);
		} else
			bfa_fsm_set_state(rxf, bna_rxf_sm_cfg_wait);
		break;

	case RXF_E_STOP:
		call_rxf_stop_cbfn(rxf);
		break;

	case RXF_E_FAIL:
		/* No-op */
		break;

	case RXF_E_CONFIG:
		call_rxf_cam_fltr_cbfn(rxf);
		break;

	case RXF_E_PAUSE:
		rxf->flags |= BNA_RXF_F_PAUSED;
		call_rxf_pause_cbfn(rxf);
		break;

	case RXF_E_RESUME:
		rxf->flags &= ~BNA_RXF_F_PAUSED;
		call_rxf_resume_cbfn(rxf);
		break;

	default:
		bfa_sm_fault(rxf->rx->bna, event);
	}
}

static void
bna_rxf_sm_paused_entry(struct bna_rxf_s *rxf)
{
	bfa_trc(rxf->rx->bna, rxf->rx->rid);
	call_rxf_pause_cbfn(rxf);
}

static void
bna_rxf_sm_paused(struct bna_rxf_s *rxf, enum bna_rxf_event_e event)
{
	bfa_trc(rxf->rx->bna, event);
	switch (event) {
	case RXF_E_STOP:
	case RXF_E_FAIL:
		bfa_fsm_set_state(rxf, bna_rxf_sm_stopped);
		break;

	case RXF_E_CONFIG:
		call_rxf_cam_fltr_cbfn(rxf);
		break;

	case RXF_E_RESUME:
		rxf->flags &= ~BNA_RXF_F_PAUSED;
		bfa_fsm_set_state(rxf, bna_rxf_sm_cfg_wait);
		break;

	default:
		bfa_sm_fault(rxf->rx->bna, event);
	}
}

static void
bna_rxf_sm_cfg_wait_entry(struct bna_rxf_s *rxf)
{
	bfa_trc(rxf->rx->bna, rxf->rx->rid);
	if (!bna_rxf_cfg_apply(rxf)) {
		/* No more pending config updates */
		bfa_fsm_set_state(rxf, bna_rxf_sm_started);
	}
}

static void
bna_rxf_sm_cfg_wait(struct bna_rxf_s *rxf, enum bna_rxf_event_e event)
{
	bfa_trc(rxf->rx->bna, event);
	switch (event) {
	case RXF_E_STOP:
		CNA_ASSERT(rxf->start_cbfn == NULL);
		CNA_ASSERT(rxf->cam_fltr_cbfn == NULL);
		CNA_ASSERT(rxf->oper_state_cbfn == NULL);
		bfa_fsm_set_state(rxf, bna_rxf_sm_last_resp_wait);
		break;

	case RXF_E_FAIL:
		bna_rxf_cfg_reset(rxf);
		call_rxf_start_cbfn(rxf);
		call_rxf_cam_fltr_cbfn(rxf);
		call_rxf_resume_cbfn(rxf);
		bfa_fsm_set_state(rxf, bna_rxf_sm_stopped);
		break;

	case RXF_E_CONFIG:
		/* No-op */
		break;

	case RXF_E_PAUSE:
		rxf->flags |= BNA_RXF_F_PAUSED;
		call_rxf_start_cbfn(rxf);
		bfa_fsm_set_state(rxf, bna_rxf_sm_fltr_clr_wait);
		break;

	case RXF_E_FW_RESP:
		if (!bna_rxf_cfg_apply(rxf)) {
			/* No more pending config updates */
			bfa_fsm_set_state(rxf, bna_rxf_sm_started);
		}
		break;

	default:
		bfa_sm_fault(rxf->rx->bna, event);
	}
}

static void
bna_rxf_sm_started_entry(struct bna_rxf_s *rxf)
{
	bfa_trc(rxf->rx->bna, rxf->rx->rid);
	call_rxf_start_cbfn(rxf);
	call_rxf_cam_fltr_cbfn(rxf);
	call_rxf_resume_cbfn(rxf);
}

static void
bna_rxf_sm_started(struct bna_rxf_s *rxf, enum bna_rxf_event_e event)
{
	bfa_trc(rxf->rx->bna, event);
	switch (event) {
	case RXF_E_STOP:
	case RXF_E_FAIL:
		bna_rxf_cfg_reset(rxf);
		bfa_fsm_set_state(rxf, bna_rxf_sm_stopped);
		break;

	case RXF_E_CONFIG:
		bfa_fsm_set_state(rxf, bna_rxf_sm_cfg_wait);
		break;

	case RXF_E_PAUSE:
		rxf->flags |= BNA_RXF_F_PAUSED;
		if (!bna_rxf_fltr_clear(rxf))
			bfa_fsm_set_state(rxf, bna_rxf_sm_paused);
		else
			bfa_fsm_set_state(rxf, bna_rxf_sm_fltr_clr_wait);
		break;

	default:
		bfa_sm_fault(rxf->rx->bna, event);
	}
}

static void
bna_rxf_sm_fltr_clr_wait_entry(struct bna_rxf_s *rxf)
{
	bfa_trc(rxf->rx->bna, rxf->rx->rid);
}

static void
bna_rxf_sm_fltr_clr_wait(struct bna_rxf_s *rxf, enum bna_rxf_event_e event)
{
	bfa_trc(rxf->rx->bna, event);
	switch (event) {
	case RXF_E_FAIL:
		CNA_ASSERT(rxf->start_cbfn == NULL);
		CNA_ASSERT(rxf->cam_fltr_cbfn == NULL);
		bna_rxf_cfg_reset(rxf);
		call_rxf_pause_cbfn(rxf);
		bfa_fsm_set_state(rxf, bna_rxf_sm_stopped);
		break;

	case RXF_E_FW_RESP:
		if (!bna_rxf_fltr_clear(rxf)) {
			/* No more pending CAM entries to clear */
			bfa_fsm_set_state(rxf, bna_rxf_sm_paused);
		}
		break;

	default:
		bfa_sm_fault(rxf->rx->bna, event);
	}
}

static void
bna_rxf_sm_last_resp_wait_entry(struct bna_rxf_s *rxf)
{
	bfa_trc(rxf->rx->bna, rxf->rx->rid);
}

static void
bna_rxf_sm_last_resp_wait(struct bna_rxf_s *rxf, enum bna_rxf_event_e event)
{
	bfa_trc(rxf->rx->bna, event);

	switch (event) {
	case RXF_E_FAIL:
	case RXF_E_FW_RESP:
		CNA_ASSERT(rxf->start_cbfn == NULL);
		CNA_ASSERT(rxf->cam_fltr_cbfn == NULL);
		CNA_ASSERT(rxf->oper_state_cbfn == NULL);
		bna_rxf_cfg_reset(rxf);
		bfa_fsm_set_state(rxf, bna_rxf_sm_stopped);
		break;

	default:
		bfa_sm_fault(rxf->rx->bna, event);
	}
}

static void
bna_bfi_ucast_req(struct bna_rxf_s *rxf, struct bna_mac_s *mac,
		bfi_enet_h2i_msgs_t req_type)
{
	struct bfi_enet_ucast_req *req = &rxf->bfi_enet_cmd.ucast_req;

	bfi_msgq_mhdr_set(req->mh, BFI_MC_ENET, req_type, 0, rxf->rx->rid);
	req->mh.num_entries = cna_os_htons(
	bfi_msgq_num_cmd_entries(sizeof(struct bfi_enet_ucast_req)));
	cna_os_memcpy(&req->mac_addr, &mac->addr, sizeof(mac_t));
	bfa_msgq_cmd_set(&rxf->msgq_cmd, NULL, NULL,
		sizeof(struct bfi_enet_ucast_req), &req->mh);
	bfa_msgq_cmd_post(&rxf->rx->bna->msgq, &rxf->msgq_cmd);
}

static void
bna_bfi_mcast_add_req(struct bna_rxf_s *rxf, struct bna_mac_s *mac)
{
	struct bfi_enet_mcast_add_req *req =
		&rxf->bfi_enet_cmd.mcast_add_req;

	bfi_msgq_mhdr_set(req->mh, BFI_MC_ENET, BFI_ENET_H2I_MAC_MCAST_ADD_REQ,
		0, rxf->rx->rid);
	req->mh.num_entries = cna_os_htons(
	bfi_msgq_num_cmd_entries(sizeof(struct bfi_enet_mcast_add_req)));
	cna_os_memcpy(&req->mac_addr, &mac->addr, sizeof(mac_t));
	bfa_msgq_cmd_set(&rxf->msgq_cmd, NULL, NULL,
		sizeof(struct bfi_enet_mcast_add_req), &req->mh);
	bfa_msgq_cmd_post(&rxf->rx->bna->msgq, &rxf->msgq_cmd);
}

static void
bna_bfi_mcast_del_req(struct bna_rxf_s *rxf, uint16_t handle)
{
	struct bfi_enet_mcast_del_req *req =
		&rxf->bfi_enet_cmd.mcast_del_req;

	bfi_msgq_mhdr_set(req->mh, BFI_MC_ENET, BFI_ENET_H2I_MAC_MCAST_DEL_REQ,
		0, rxf->rx->rid);
	req->mh.num_entries = cna_os_htons(
	bfi_msgq_num_cmd_entries(sizeof(struct bfi_enet_mcast_del_req)));
	req->handle = cna_os_htons(handle);
	bfa_msgq_cmd_set(&rxf->msgq_cmd, NULL, NULL,
		sizeof(struct bfi_enet_mcast_del_req), &req->mh);
	bfa_msgq_cmd_post(&rxf->rx->bna->msgq, &rxf->msgq_cmd);
}

static void
bna_bfi_mcast_filter_req(struct bna_rxf_s *rxf, enum bna_status_e status)
{
	struct bfi_enet_enable_req *req = &rxf->bfi_enet_cmd.req;

	bfi_msgq_mhdr_set(req->mh, BFI_MC_ENET,
		BFI_ENET_H2I_MAC_MCAST_FILTER_REQ, 0, rxf->rx->rid);
	req->mh.num_entries = cna_os_htons(
		bfi_msgq_num_cmd_entries(sizeof(struct bfi_enet_enable_req)));
	req->enable = status;
	bfa_msgq_cmd_set(&rxf->msgq_cmd, NULL, NULL,
		sizeof(struct bfi_enet_enable_req), &req->mh);
	bfa_msgq_cmd_post(&rxf->rx->bna->msgq, &rxf->msgq_cmd);
}

static void
bna_bfi_rx_promisc_req(struct bna_rxf_s *rxf, enum bna_status_e status)
{
	struct bfi_enet_enable_req *req = &rxf->bfi_enet_cmd.req;

	CNA_ASSERT(rxf->rx->rid == 0);
	bfi_msgq_mhdr_set(req->mh, BFI_MC_ENET,
		BFI_ENET_H2I_RX_PROMISCUOUS_REQ, 0, rxf->rx->rid);
	req->mh.num_entries = cna_os_htons(
		bfi_msgq_num_cmd_entries(sizeof(struct bfi_enet_enable_req)));
	req->enable = status;
	bfa_msgq_cmd_set(&rxf->msgq_cmd, NULL, NULL,
		sizeof(struct bfi_enet_enable_req), &req->mh);
	bfa_msgq_cmd_post(&rxf->rx->bna->msgq, &rxf->msgq_cmd);
}

static void
bna_bfi_rx_default_mode_req(struct bna_rxf_s *rxf, enum bna_status_e status)
{
	struct bfi_enet_enable_req *req = &rxf->bfi_enet_cmd.req;

	CNA_ASSERT(rxf->rx->rid == 0);
	bfi_msgq_mhdr_set(req->mh, BFI_MC_ENET,
		BFI_ENET_H2I_RX_DEFAULT_REQ, 0, rxf->rx->rid);
	req->mh.num_entries = cna_os_htons(
		bfi_msgq_num_cmd_entries(sizeof(struct bfi_enet_enable_req)));
	req->enable = status;
	bfa_msgq_cmd_set(&rxf->msgq_cmd, NULL, NULL,
		sizeof(struct bfi_enet_enable_req), &req->mh);
	bfa_msgq_cmd_post(&rxf->rx->bna->msgq, &rxf->msgq_cmd);
}

static void
bna_bfi_rx_vlan_filter_set(struct bna_rxf_s *rxf, uint8_t block_idx)
{
	struct bfi_enet_rx_vlan_req *req = &rxf->bfi_enet_cmd.vlan_req;
	int i;
	int j;

	bfi_msgq_mhdr_set(req->mh, BFI_MC_ENET,
		BFI_ENET_H2I_RX_VLAN_SET_REQ, 0, rxf->rx->rid);
	req->mh.num_entries = cna_os_htons(
		bfi_msgq_num_cmd_entries(sizeof(struct bfi_enet_rx_vlan_req)));
	req->block_idx = block_idx;
	for (i = 0; i < (BFI_ENET_VLAN_BLOCK_SIZE / 32); i++) {
		j = (block_idx * (BFI_ENET_VLAN_BLOCK_SIZE / 32)) + i;
		if (rxf->vlan_filter_status == BNA_STATUS_T_ENABLED)
			req->bit_mask[i] =
				cna_os_htonl(rxf->vlan_filter_table[j]);
		else
			req->bit_mask[i] = 0xFFFFFFFF;
	}
	bfa_msgq_cmd_set(&rxf->msgq_cmd, NULL, NULL,
		sizeof(struct bfi_enet_rx_vlan_req), &req->mh);
	bfa_msgq_cmd_post(&rxf->rx->bna->msgq, &rxf->msgq_cmd);
}

static void
bna_bfi_vlan_strip_enable(struct bna_rxf_s *rxf)
{
	struct bfi_enet_enable_req *req = &rxf->bfi_enet_cmd.req;

	bfi_msgq_mhdr_set(req->mh, BFI_MC_ENET,
		BFI_ENET_H2I_RX_VLAN_STRIP_ENABLE_REQ, 0, rxf->rx->rid);
	req->mh.num_entries = cna_os_htons(
		bfi_msgq_num_cmd_entries(sizeof(struct bfi_enet_enable_req)));
	req->enable = rxf->vlan_strip_status;
	bfa_msgq_cmd_set(&rxf->msgq_cmd, NULL, NULL,
		sizeof(struct bfi_enet_enable_req), &req->mh);
	bfa_msgq_cmd_post(&rxf->rx->bna->msgq, &rxf->msgq_cmd);
}

static void
bna_bfi_rit_cfg(struct bna_rxf_s *rxf)
{
	struct bfi_enet_rit_req *req = &rxf->bfi_enet_cmd.rit_req;

	bfi_msgq_mhdr_set(req->mh, BFI_MC_ENET,
		BFI_ENET_H2I_RIT_CFG_REQ, 0, rxf->rx->rid);
	req->mh.num_entries = cna_os_htons(
		bfi_msgq_num_cmd_entries(sizeof(struct bfi_enet_rit_req)));
	req->size = cna_os_htons(rxf->rit_size);
	cna_os_memcpy(&req->table[0], rxf->rit, rxf->rit_size);
	bfa_msgq_cmd_set(&rxf->msgq_cmd, NULL, NULL,
		sizeof(struct bfi_enet_rit_req), &req->mh);
	bfa_msgq_cmd_post(&rxf->rx->bna->msgq, &rxf->msgq_cmd);
}

static void
bna_bfi_rss_cfg(struct bna_rxf_s *rxf)
{
	struct bfi_enet_rss_cfg_req *req = &rxf->bfi_enet_cmd.rss_req;
	int i;

	bfi_msgq_mhdr_set(req->mh, BFI_MC_ENET,
		BFI_ENET_H2I_RSS_CFG_REQ, 0, rxf->rx->rid);
	req->mh.num_entries = cna_os_htons(
		bfi_msgq_num_cmd_entries(sizeof(struct bfi_enet_rss_cfg_req)));
	req->cfg.type = rxf->rss_cfg.hash_type;
	req->cfg.mask = rxf->rss_cfg.hash_mask;
	for (i = 0; i < BFI_ENET_RSS_KEY_LEN; i++)
		req->cfg.key[i] =
			cna_os_htonl(rxf->rss_cfg.toeplitz_hash_key[i]);
	bfa_msgq_cmd_set(&rxf->msgq_cmd, NULL, NULL,
		sizeof(struct bfi_enet_rss_cfg_req), &req->mh);
	bfa_msgq_cmd_post(&rxf->rx->bna->msgq, &rxf->msgq_cmd);
}

static void
bna_bfi_rss_enable(struct bna_rxf_s *rxf)
{
	struct bfi_enet_enable_req *req = &rxf->bfi_enet_cmd.req;

	bfi_msgq_mhdr_set(req->mh, BFI_MC_ENET,
		BFI_ENET_H2I_RSS_ENABLE_REQ, 0, rxf->rx->rid);
	req->mh.num_entries = cna_os_htons(
		bfi_msgq_num_cmd_entries(sizeof(struct bfi_enet_enable_req)));
	CNA_ASSERT(rxf->rx->rid == 0);
	req->enable = rxf->rss_status;
	bfa_msgq_cmd_set(&rxf->msgq_cmd, NULL, NULL,
		sizeof(struct bfi_enet_enable_req), &req->mh);
	bfa_msgq_cmd_post(&rxf->rx->bna->msgq, &rxf->msgq_cmd);
}

static void
bna_bfi_wol_magic_add_req(struct bna_rxf_s *rxf, struct bna_wol_s *wol)
{
	struct bfi_enet_wol_magic_frame *req =
		&rxf->bfi_enet_cmd.wol_magic;

	bfi_msgq_mhdr_set(req->mh, BFI_MC_ENET, BFI_ENET_H2I_WOL_MAGIC_REQ,
		0, rxf->rx->rid);
	req->mh.num_entries = cna_os_htons(
	bfi_msgq_num_cmd_entries(sizeof(struct bfi_enet_wol_magic_frame)));
	req->frame_id = wol->index;
	req->enable = BFA_TRUE;
	req->password_enable = BFA_FALSE;
	cna_os_memcpy(&req->mac_addr, &wol->pattern, sizeof(mac_t));
	bfa_msgq_cmd_set(&rxf->msgq_cmd, NULL, NULL,
		sizeof(struct bfi_enet_wol_magic_frame), &req->mh);
	bfa_msgq_cmd_post(&rxf->rx->bna->msgq, &rxf->msgq_cmd);
}

static void
bna_bfi_wol_frame_add_req(struct bna_rxf_s *rxf, struct bna_wol_s *wol)
{
	struct bfi_enet_wol_frame_gen *req =
		&rxf->bfi_enet_cmd.wol_frame;

	bfi_msgq_mhdr_set(req->mh, BFI_MC_ENET, BFI_ENET_H2I_WOL_FRAME_REQ,
		0, rxf->rx->rid);
	req->mh.num_entries = cna_os_htons(
	bfi_msgq_num_cmd_entries(sizeof(struct bfi_enet_wol_frame_gen)));
	req->frame_id = wol->index;
	req->enable = BFA_TRUE;
	cna_os_memcpy(&req->frame, &wol->pattern, BFI_ENET_WOL_FRAME_LEN);
	cna_os_memcpy(&req->mask, &wol->mask, BFI_ENET_WOL_FRAME_MASK_LEN);
	bfa_msgq_cmd_set(&rxf->msgq_cmd, NULL, NULL,
		sizeof(struct bfi_enet_wol_frame_gen), &req->mh);
	bfa_msgq_cmd_post(&rxf->rx->bna->msgq, &rxf->msgq_cmd);
}

static void
bna_bfi_wol_magic_del_req(struct bna_rxf_s *rxf, struct bna_wol_s *wol)
{
	struct bfi_enet_wol_magic_frame *req =
		&rxf->bfi_enet_cmd.wol_magic;

	bfi_msgq_mhdr_set(req->mh, BFI_MC_ENET, BFI_ENET_H2I_WOL_MAGIC_REQ,
		0, rxf->rx->rid);
	req->mh.num_entries = cna_os_htons(
	bfi_msgq_num_cmd_entries(sizeof(struct bfi_enet_wol_magic_frame)));
	req->frame_id = wol->index;
	req->enable = BFA_FALSE;
	req->password_enable = BFA_FALSE;
	cna_os_memcpy(&req->mac_addr, &wol->pattern, sizeof(mac_t));
	bfa_msgq_cmd_set(&rxf->msgq_cmd, NULL, NULL,
		sizeof(struct bfi_enet_wol_magic_frame), &req->mh);
	bfa_msgq_cmd_post(&rxf->rx->bna->msgq, &rxf->msgq_cmd);
}

static void
bna_bfi_wol_frame_del_req(struct bna_rxf_s *rxf, struct bna_wol_s *wol)
{
	struct bfi_enet_wol_frame_gen *req =
		&rxf->bfi_enet_cmd.wol_frame;

	bfi_msgq_mhdr_set(req->mh, BFI_MC_ENET, BFI_ENET_H2I_WOL_FRAME_REQ,
		0, rxf->rx->rid);
	req->mh.num_entries = cna_os_htons(
	bfi_msgq_num_cmd_entries(sizeof(struct bfi_enet_wol_frame_gen)));
	req->frame_id = wol->index;
	req->enable = BFA_FALSE;
	cna_os_memcpy(&req->frame, &wol->pattern, BFI_ENET_WOL_FRAME_LEN);
	cna_os_memcpy(&req->mask, &wol->mask, BFI_ENET_WOL_FRAME_MASK_LEN);
	bfa_msgq_cmd_set(&rxf->msgq_cmd, NULL, NULL,
		sizeof(struct bfi_enet_wol_frame_gen), &req->mh);
	bfa_msgq_cmd_post(&rxf->rx->bna->msgq, &rxf->msgq_cmd);
}

/* This function gets the multicast MAC that has already been added to CAM */
static struct bna_mac_s *
bna_rxf_mcmac_get(struct bna_rxf_s *rxf, uint8_t *mac_addr)
{
	struct bna_mac_s *mac;
	bfa_q_t *qe;

	bfa_q_iter(&rxf->mcast_active_q, qe) {
		mac = (struct bna_mac_s *)qe;
		if (BNA_MAC_IS_EQUAL(&mac->addr, mac_addr))
			return mac;
	}

	bfa_q_iter(&rxf->mcast_pending_del_q, qe) {
		mac = (struct bna_mac_s *)qe;
		if (BNA_MAC_IS_EQUAL(&mac->addr, mac_addr))
			return mac;
	}

	return NULL;
}

static struct bna_mcam_handle_s *
bna_rxf_mchandle_get(struct bna_rxf_s *rxf, int handle)
{
	struct bna_mcam_handle_s *mchandle;
	bfa_q_t *qe;

	bfa_q_iter(&rxf->mcast_handle_q, qe) {
		mchandle = (struct bna_mcam_handle_s *)qe;
		if (mchandle->handle == handle)
			return mchandle;
	}

	return NULL;
}

static void
bna_rxf_mchandle_attach(struct bna_rxf_s *rxf, uint8_t *mac_addr, int handle)
{
	struct bna_mac_s *mcmac;
	struct bna_mcam_handle_s *mchandle;

	mcmac = bna_rxf_mcmac_get(rxf, mac_addr);
	CNA_ASSERT(mcmac->handle == NULL);
	mchandle = bna_rxf_mchandle_get(rxf, handle);
	if (mchandle == NULL) {
		mchandle = bna_mcam_mod_handle_get(&rxf->rx->bna->mcam_mod);
		mchandle->handle = handle;
		mchandle->refcnt = 0;
		bfa_q_enq(&rxf->mcast_handle_q, &mchandle->qe);
	}
	mchandle->refcnt++;
	mcmac->handle = mchandle;
}

static int
bna_rxf_mcast_del(struct bna_rxf_s *rxf, struct bna_mac_s *mac,
		enum bna_cleanup_type_e cleanup)
{
	struct bna_mcam_handle_s *mchandle;
	int ret = 0;

	mchandle = mac->handle;
	if (mchandle == NULL)
		return ret;

	CNA_ASSERT(mchandle->refcnt > 0);
	mchandle->refcnt--;
	if (mchandle->refcnt == 0) {
		if (cleanup == BNA_HARD_CLEANUP) {
			bna_bfi_mcast_del_req(rxf, mchandle->handle);
			ret = 1;
		}
		bfa_q_qe_deq(&mchandle->qe);
		bfa_q_qe_init(&mchandle->qe);
		bna_mcam_mod_handle_put(&rxf->rx->bna->mcam_mod, mchandle);
	}
	mac->handle = NULL;

	return ret;
}

static int
bna_rxf_ucast_cfg_apply(struct bna_rxf_s *rxf)
{
	struct bna_mac_s *mac = NULL;
	bfa_q_t *qe;

	/* Delete MAC addresses previousely added */
	if (!bfa_q_is_empty(&rxf->ucast_pending_del_q)) {
		bfa_q_deq(&rxf->ucast_pending_del_q, &qe);
		bfa_q_qe_init(qe);
		mac = (struct bna_mac_s *)qe;
		bna_bfi_ucast_req(rxf, mac, BFI_ENET_H2I_MAC_UCAST_DEL_REQ);
		bna_cam_mod_mac_put(bna_ucam_mod_del_q(rxf->rx->bna), mac);
		return 1;
	}

	/* Set default unicast MAC */
	if (rxf->ucast_pending_set) {
		CNA_ASSERT(rxf->ucast_pending_mac != NULL);
		rxf->ucast_pending_set = 0;
		cna_os_memcpy(rxf->ucast_active_mac.addr,
			rxf->ucast_pending_mac->addr, CNA_ETH_ALEN);
		rxf->ucast_active_set = 1;
		bna_bfi_ucast_req(rxf, &rxf->ucast_active_mac,
			BFI_ENET_H2I_MAC_UCAST_SET_REQ);
		return 1;
	}

	/* Add additional MAC entries */
	if (!bfa_q_is_empty(&rxf->ucast_pending_add_q)) {
		bfa_q_deq(&rxf->ucast_pending_add_q, &qe);
		bfa_q_qe_init(qe);
		mac = (struct bna_mac_s *)qe;
		bfa_q_enq(&rxf->ucast_active_q, &mac->qe);
		bna_bfi_ucast_req(rxf, mac, BFI_ENET_H2I_MAC_UCAST_ADD_REQ);
		return 1;
	}

	return 0;
}

static int
bna_rxf_ucast_cfg_reset(struct bna_rxf_s *rxf, enum bna_cleanup_type_e cleanup)
{
	bfa_q_t *qe;
	struct bna_mac_s *mac;

	/* Throw away delete pending ucast entries */
	while (!bfa_q_is_empty(&rxf->ucast_pending_del_q)) {
		bfa_q_deq(&rxf->ucast_pending_del_q, &qe);
		bfa_q_qe_init(qe);
		mac = (struct bna_mac_s *)qe;
		if (cleanup == BNA_SOFT_CLEANUP)
			bna_cam_mod_mac_put(bna_ucam_mod_del_q(rxf->rx->bna),
					mac);
		else {
			bna_bfi_ucast_req(rxf, mac,
				BFI_ENET_H2I_MAC_UCAST_DEL_REQ);
			bna_cam_mod_mac_put(bna_ucam_mod_del_q(rxf->rx->bna),
					mac);
			return 1;
		}
	}

	/* Move active ucast entries to pending_add_q */
	while (!bfa_q_is_empty(&rxf->ucast_active_q)) {
		bfa_q_deq(&rxf->ucast_active_q, &qe);
		bfa_q_qe_init(qe);
		bfa_q_enq(&rxf->ucast_pending_add_q, qe);
		if (cleanup == BNA_HARD_CLEANUP) {
			mac = (struct bna_mac_s *)qe;
			bna_bfi_ucast_req(rxf, mac,
				BFI_ENET_H2I_MAC_UCAST_DEL_REQ);
			return 1;
		}
	}

	if (rxf->ucast_active_set) {
		rxf->ucast_pending_set = 1;
		CNA_ASSERT(rxf->ucast_pending_mac);
		rxf->ucast_active_set = 0;
		if (cleanup == BNA_HARD_CLEANUP) {
			bna_bfi_ucast_req(rxf, &rxf->ucast_active_mac,
				BFI_ENET_H2I_MAC_UCAST_CLR_REQ);
			return 1;
		}
	}

	return 0;
}

static int
bna_rxf_mcast_cfg_apply(struct bna_rxf_s *rxf)
{
	struct bna_mac_s *mac = NULL;
	bfa_q_t *qe;
	int ret;

	/* First delete multicast entries to maintain the count */
	while (!bfa_q_is_empty(&rxf->mcast_pending_del_q)) {
		bfa_q_deq(&rxf->mcast_pending_del_q, &qe);
		bfa_q_qe_init(qe);
		mac = (struct bna_mac_s *)qe;
		ret = bna_rxf_mcast_del(rxf, mac, BNA_HARD_CLEANUP);
		bna_cam_mod_mac_put(bna_mcam_mod_del_q(rxf->rx->bna), mac);
		if (ret)
			return ret;
	}

	/* Add multicast entries */
	if (!bfa_q_is_empty(&rxf->mcast_pending_add_q)) {
		bfa_q_deq(&rxf->mcast_pending_add_q, &qe);
		bfa_q_qe_init(qe);
		mac = (struct bna_mac_s *)qe;
		bfa_q_enq(&rxf->mcast_active_q, &mac->qe);
		bna_bfi_mcast_add_req(rxf, mac);
		return 1;
	}

	return 0;
}

static int
bna_rxf_mcast_cfg_reset(struct bna_rxf_s *rxf, enum bna_cleanup_type_e cleanup)
{
	bfa_q_t *qe;
	struct bna_mac_s *mac;
	int ret;

	/* Throw away delete pending mcast entries */
	while (!bfa_q_is_empty(&rxf->mcast_pending_del_q)) {
		bfa_q_deq(&rxf->mcast_pending_del_q, &qe);
		bfa_q_qe_init(qe);
		mac = (struct bna_mac_s *)qe;
		ret = bna_rxf_mcast_del(rxf, mac, cleanup);
		bna_cam_mod_mac_put(bna_mcam_mod_del_q(rxf->rx->bna), mac);
		if (ret)
			return ret;
	}

	/* Move active mcast entries to pending_add_q */
	while (!bfa_q_is_empty(&rxf->mcast_active_q)) {
		bfa_q_deq(&rxf->mcast_active_q, &qe);
		bfa_q_qe_init(qe);
		bfa_q_enq(&rxf->mcast_pending_add_q, qe);
		mac = (struct bna_mac_s *)qe;
		if (bna_rxf_mcast_del(rxf, mac, cleanup))
			return 1;
	}

	return 0;
}

static int
bna_rxf_promisc_cfg_apply(struct bna_rxf_s *rxf)
{
	struct bna_s *bna = rxf->rx->bna;

	/* Enable/disable promiscuous mode */
	if (is_promisc_enable(rxf->rxmode_pending,
				rxf->rxmode_pending_bitmask)) {
		CNA_ASSERT(!(rxf->rxmode_active & BNA_RXMODE_PROMISC));
		CNA_ASSERT(bna->promisc_rid == rxf->rx->rid);
		/* move promisc configuration from pending -> active */
		promisc_inactive(rxf->rxmode_pending,
				rxf->rxmode_pending_bitmask);
		rxf->rxmode_active |= BNA_RXMODE_PROMISC;
		bna_bfi_rx_promisc_req(rxf, BNA_STATUS_T_ENABLED);
		return 1;
	} else if (is_promisc_disable(rxf->rxmode_pending,
				rxf->rxmode_pending_bitmask)) {
		CNA_ASSERT(rxf->rxmode_active & BNA_RXMODE_PROMISC);
		CNA_ASSERT(bna->promisc_rid == rxf->rx->rid);
		/* move promisc configuration from pending -> active */
		promisc_inactive(rxf->rxmode_pending,
				rxf->rxmode_pending_bitmask);
		rxf->rxmode_active &= ~BNA_RXMODE_PROMISC;
		bna->promisc_rid = BFI_INVALID_RID;
		bna_bfi_rx_promisc_req(rxf, BNA_STATUS_T_DISABLED);
		return 1;
	}

	return 0;
}

static int
bna_rxf_promisc_cfg_reset(struct bna_rxf_s *rxf,
			enum bna_cleanup_type_e cleanup)
{
	struct bna_s *bna = rxf->rx->bna;

	/* Clear pending promisc mode disable */
	if (is_promisc_disable(rxf->rxmode_pending,
				rxf->rxmode_pending_bitmask)) {
		CNA_ASSERT(rxf->rxmode_active & BNA_RXMODE_PROMISC);
		CNA_ASSERT(bna->promisc_rid == rxf->rx->rid);
		promisc_inactive(rxf->rxmode_pending,
				rxf->rxmode_pending_bitmask);
		rxf->rxmode_active &= ~BNA_RXMODE_PROMISC;
		bna->promisc_rid = BFI_INVALID_RID;
		if (cleanup == BNA_HARD_CLEANUP) {
			bna_bfi_rx_promisc_req(rxf, BNA_STATUS_T_DISABLED);
			return 1;
		}
	}

	/* Move promisc mode config from active -> pending */
	if (rxf->rxmode_active & BNA_RXMODE_PROMISC) {
		CNA_ASSERT(bna->promisc_rid == rxf->rx->rid);
		promisc_enable(rxf->rxmode_pending,
				rxf->rxmode_pending_bitmask);
		rxf->rxmode_active &= ~BNA_RXMODE_PROMISC;
		if (cleanup == BNA_HARD_CLEANUP) {
			bna_bfi_rx_promisc_req(rxf, BNA_STATUS_T_DISABLED);
			return 1;
		}
	}

	return 0;
}

static int
bna_rxf_default_mode_cfg_apply(struct bna_rxf_s *rxf)
{
	struct bna_s *bna = rxf->rx->bna;

	/* Enable/disable default mode */
	if (is_default_enable(rxf->rxmode_pending,
				rxf->rxmode_pending_bitmask)) {
		CNA_ASSERT(!(rxf->rxmode_active & BNA_RXMODE_DEFAULT));
		CNA_ASSERT(bna->default_mode_rid == rxf->rx->rid);
		/* move default configuration from pending -> active */
		default_inactive(rxf->rxmode_pending,
				rxf->rxmode_pending_bitmask);
		rxf->rxmode_active |= BNA_RXMODE_DEFAULT;
		bna_bfi_rx_default_mode_req(rxf, BNA_STATUS_T_ENABLED);
		return 1;
	} else if (is_default_disable(rxf->rxmode_pending,
				rxf->rxmode_pending_bitmask)) {
		CNA_ASSERT(rxf->rxmode_active & BNA_RXMODE_DEFAULT);
		CNA_ASSERT(bna->default_mode_rid == rxf->rx->rid);
		/* move default configuration from pending -> active */
		default_inactive(rxf->rxmode_pending,
				rxf->rxmode_pending_bitmask);
		rxf->rxmode_active &= ~BNA_RXMODE_DEFAULT;
		bna->default_mode_rid = BFI_INVALID_RID;
		bna_bfi_rx_default_mode_req(rxf, BNA_STATUS_T_DISABLED);
		return 1;
	}

	return 0;
}

static int
bna_rxf_default_mode_cfg_reset(struct bna_rxf_s *rxf,
				enum bna_cleanup_type_e cleanup)
{
	struct bna_s *bna = rxf->rx->bna;

	/* Clear pending default mode disable */
	if (is_default_disable(rxf->rxmode_pending,
				rxf->rxmode_pending_bitmask)) {
		CNA_ASSERT(rxf->rxmode_active & BNA_RXMODE_DEFAULT);
		CNA_ASSERT(bna->default_mode_rid == rxf->rx->rid);
		default_inactive(rxf->rxmode_pending,
				rxf->rxmode_pending_bitmask);
		rxf->rxmode_active &= ~BNA_RXMODE_DEFAULT;
		bna->default_mode_rid = BFI_INVALID_RID;
		if (cleanup == BNA_HARD_CLEANUP) {
			bna_bfi_rx_default_mode_req(rxf, BNA_STATUS_T_DISABLED);
			return 1;
		}
	}

	/* Move default mode config from active -> pending */
	if (rxf->rxmode_active & BNA_RXMODE_DEFAULT) {
		CNA_ASSERT(bna->default_mode_rid == rxf->rx->rid);
		default_enable(rxf->rxmode_pending,
				rxf->rxmode_pending_bitmask);
		rxf->rxmode_active &= ~BNA_RXMODE_DEFAULT;
		if (cleanup == BNA_HARD_CLEANUP) {
			bna_bfi_rx_default_mode_req(rxf, BNA_STATUS_T_DISABLED);
			return 1;
		}
	}

	return 0;
}

static int
bna_rxf_allmulti_cfg_apply(struct bna_rxf_s *rxf)
{
	/* Enable/disable allmulti mode */
	if (is_allmulti_enable(rxf->rxmode_pending,
				rxf->rxmode_pending_bitmask)) {
		CNA_ASSERT(!(rxf->rxmode_active & BNA_RXMODE_ALLMULTI));
		/* move allmulti configuration from pending -> active */
		allmulti_inactive(rxf->rxmode_pending,
				rxf->rxmode_pending_bitmask);
		rxf->rxmode_active |= BNA_RXMODE_ALLMULTI;
		bna_bfi_mcast_filter_req(rxf, BNA_STATUS_T_DISABLED);
		return 1;
	} else if (is_allmulti_disable(rxf->rxmode_pending,
					rxf->rxmode_pending_bitmask)) {
		CNA_ASSERT(rxf->rxmode_active & BNA_RXMODE_ALLMULTI);
		/* move allmulti configuration from pending -> active */
		allmulti_inactive(rxf->rxmode_pending,
				rxf->rxmode_pending_bitmask);
		rxf->rxmode_active &= ~BNA_RXMODE_ALLMULTI;
		bna_bfi_mcast_filter_req(rxf, BNA_STATUS_T_ENABLED);
		return 1;
	}

	return 0;
}

static int
bna_rxf_allmulti_cfg_reset(struct bna_rxf_s *rxf,
			enum bna_cleanup_type_e cleanup)
{
	/* Clear pending allmulti mode disable */
	if (is_allmulti_disable(rxf->rxmode_pending,
				rxf->rxmode_pending_bitmask)) {
		CNA_ASSERT(rxf->rxmode_active & BNA_RXMODE_ALLMULTI);
		allmulti_inactive(rxf->rxmode_pending,
				rxf->rxmode_pending_bitmask);
		rxf->rxmode_active &= ~BNA_RXMODE_ALLMULTI;
		if (cleanup == BNA_HARD_CLEANUP) {
			bna_bfi_mcast_filter_req(rxf, BNA_STATUS_T_ENABLED);
			return 1;
		}
	}

	/* Move allmulti mode config from active -> pending */
	if (rxf->rxmode_active & BNA_RXMODE_ALLMULTI) {
		allmulti_enable(rxf->rxmode_pending,
				rxf->rxmode_pending_bitmask);
		rxf->rxmode_active &= ~BNA_RXMODE_ALLMULTI;
		if (cleanup == BNA_HARD_CLEANUP) {
			bna_bfi_mcast_filter_req(rxf, BNA_STATUS_T_ENABLED);
			return 1;
		}
	}

	return 0;
}

static int
bna_rxf_vlan_cfg_apply(struct bna_rxf_s *rxf)
{
	uint8_t vlan_pending_bitmask;
	int block_idx = 0;

	if (rxf->vlan_pending_bitmask) {
		vlan_pending_bitmask = rxf->vlan_pending_bitmask;
		while (!(vlan_pending_bitmask & 0x1)) {
			block_idx++;
			vlan_pending_bitmask >>= 1;
		}
		rxf->vlan_pending_bitmask &= ~(1 << block_idx);
		bna_bfi_rx_vlan_filter_set(rxf, block_idx);
		return 1;
	}

	return 0;
}

static int
bna_rxf_vlan_strip_cfg_apply(struct bna_rxf_s *rxf)
{
	if (rxf->vlan_strip_pending) {
			rxf->vlan_strip_pending = BFA_FALSE;
			bna_bfi_vlan_strip_enable(rxf);
			return 1;
	}

	return 0;
}

static int
bna_rxf_rss_cfg_apply(struct bna_rxf_s *rxf)
{
	if (rxf->rss_pending) {
		if (rxf->rss_pending & BNA_RSS_F_RIT_PENDING) {
			rxf->rss_pending &= ~BNA_RSS_F_RIT_PENDING;
			bna_bfi_rit_cfg(rxf);
			return 1;
		}

		if (rxf->rss_pending & BNA_RSS_F_CFG_PENDING) {
			rxf->rss_pending &= ~BNA_RSS_F_CFG_PENDING;
			bna_bfi_rss_cfg(rxf);
			return 1;
		}

		if (rxf->rss_pending & BNA_RSS_F_STATUS_PENDING) {
			rxf->rss_pending &= ~BNA_RSS_F_STATUS_PENDING;
			bna_bfi_rss_enable(rxf);
			return 1;
		}
	}

	return 0;
}

static int
bna_rxf_wol_cfg_apply(struct bna_rxf_s *rxf)
{
	struct bna_wol_s *wol = NULL;
	bfa_q_t *qe;

	if (!bfa_q_is_empty(&rxf->wol_m_pending_del_q)) {
		bfa_q_deq(&rxf->wol_m_pending_del_q, &qe);
		bfa_q_qe_init(qe);
		wol = (struct bna_wol_s *)qe;
		bna_bfi_wol_magic_del_req(rxf, wol);
		bna_wol_mod_magic_put(&rxf->rx->bna->wol_mod, wol);
		return 1;
	}
	if (!bfa_q_is_empty(&rxf->wol_f_pending_del_q)) {
		bfa_q_deq(&rxf->wol_f_pending_del_q, &qe);
		bfa_q_qe_init(qe);
		wol = (struct bna_wol_s *)qe;
		bna_bfi_wol_frame_del_req(rxf, wol);
		bna_wol_mod_frame_put(&rxf->rx->bna->wol_mod, wol);
		return 1;
	}
	if (!bfa_q_is_empty(&rxf->wol_m_pending_add_q)) {
		bfa_q_deq(&rxf->wol_m_pending_add_q, &qe);
		bfa_q_qe_init(qe);
		wol = (struct bna_wol_s *)qe;
		bfa_q_enq(&rxf->wol_m_active_q, &wol->qe);
		bna_bfi_wol_magic_add_req(rxf, wol);
		return 1;
	}
	if (!bfa_q_is_empty(&rxf->wol_f_pending_add_q)) {
		bfa_q_deq(&rxf->wol_f_pending_add_q, &qe);
		bfa_q_qe_init(qe);
		wol = (struct bna_wol_s *)qe;
		bfa_q_enq(&rxf->wol_f_active_q, &wol->qe);
		bna_bfi_wol_frame_add_req(rxf, wol);
		return 1;
	}

	return 0;
}

static int
bna_rxf_cfg_apply(struct bna_rxf_s *rxf)
{
	if (bna_rxf_ucast_cfg_apply(rxf))
		return 1;
	if (bna_rxf_mcast_cfg_apply(rxf))
		return 1;
	if (bna_rxf_promisc_cfg_apply(rxf))
		return 1;
	if (bna_rxf_default_mode_cfg_apply(rxf))
		return 1;
	if (bna_rxf_allmulti_cfg_apply(rxf))
		return 1;
	if (bna_rxf_vlan_cfg_apply(rxf))
		return 1;
	if (bna_rxf_vlan_strip_cfg_apply(rxf))
		return 1;
	if (bna_rxf_rss_cfg_apply(rxf))
		return 1;
	if (bna_rxf_wol_cfg_apply(rxf))
		return 1;

	return 0;
}

/* Only software reset */
static void
bna_rxf_cfg_reset(struct bna_rxf_s *rxf)
{
	bna_rxf_ucast_cfg_reset(rxf, BNA_SOFT_CLEANUP);
	bna_rxf_mcast_cfg_reset(rxf, BNA_SOFT_CLEANUP);
	bna_rxf_promisc_cfg_reset(rxf, BNA_SOFT_CLEANUP);
	bna_rxf_default_mode_cfg_reset(rxf, BNA_SOFT_CLEANUP);
	bna_rxf_allmulti_cfg_reset(rxf, BNA_SOFT_CLEANUP);
	bna_rxf_vlan_cfg_soft_reset(rxf);
	bna_rxf_rss_cfg_soft_reset(rxf);
}

static int
bna_rxf_fltr_clear(struct bna_rxf_s *rxf)
{
	if (bna_rxf_ucast_cfg_reset(rxf, BNA_HARD_CLEANUP))
		return 1;
	if (bna_rxf_mcast_cfg_reset(rxf, BNA_HARD_CLEANUP))
		return 1;
	if (bna_rxf_promisc_cfg_reset(rxf, BNA_HARD_CLEANUP))
		return 1;
	if (bna_rxf_default_mode_cfg_reset(rxf, BNA_HARD_CLEANUP))
		return 1;
	if (bna_rxf_allmulti_cfg_reset(rxf, BNA_HARD_CLEANUP))
		return 1;

	return 0;
}

static int
bna_rxf_promisc_enable(struct bna_rxf_s *rxf)
{
	struct bna_s *bna = rxf->rx->bna;
	int ret = 0;

	if (is_promisc_enable(rxf->rxmode_pending,
				rxf->rxmode_pending_bitmask) ||
		(rxf->rxmode_active & BNA_RXMODE_PROMISC)) {
		/* Do nothing if pending enable or already enabled */
		CNA_ASSERT(bna->promisc_rid == rxf->rx->rid);
	} else if (is_promisc_disable(rxf->rxmode_pending,
					rxf->rxmode_pending_bitmask)) {
		/* Turn off pending disable command */
		CNA_ASSERT(bna->promisc_rid == rxf->rx->rid);
		CNA_ASSERT(rxf->rxmode_active & BNA_RXMODE_PROMISC);
		promisc_inactive(rxf->rxmode_pending,
			rxf->rxmode_pending_bitmask);
	} else {
		/* Schedule enable */
		CNA_ASSERT(bna->promisc_rid == BFI_INVALID_RID);
		CNA_ASSERT(!(rxf->rxmode_active & BNA_RXMODE_PROMISC));
		promisc_enable(rxf->rxmode_pending,
				rxf->rxmode_pending_bitmask);
		bna->promisc_rid = rxf->rx->rid;
		ret = 1;
	}

	return ret;
}

static int
bna_rxf_promisc_disable(struct bna_rxf_s *rxf)
{
	struct bna_s *bna = rxf->rx->bna;
	int ret = 0;

	if (is_promisc_disable(rxf->rxmode_pending,
				rxf->rxmode_pending_bitmask) ||
		(!(rxf->rxmode_active & BNA_RXMODE_PROMISC))) {
		/* Do nothing if pending disable or already disabled */
	} else if (is_promisc_enable(rxf->rxmode_pending,
					rxf->rxmode_pending_bitmask)) {
		/* Turn off pending enable command */
		CNA_ASSERT(!(rxf->rxmode_active & BNA_RXMODE_PROMISC));
		CNA_ASSERT(bna->promisc_rid == rxf->rx->rid);
		promisc_inactive(rxf->rxmode_pending,
				rxf->rxmode_pending_bitmask);
		bna->promisc_rid = BFI_INVALID_RID;
	} else if (rxf->rxmode_active & BNA_RXMODE_PROMISC) {
		/* Schedule disable */
		CNA_ASSERT(bna->promisc_rid == rxf->rx->rid);
		promisc_disable(rxf->rxmode_pending,
				rxf->rxmode_pending_bitmask);
		ret = 1;
	}

	return ret;
}

static int
bna_rxf_default_enable(struct bna_rxf_s *rxf)
{
	struct bna_s *bna = rxf->rx->bna;
	int ret = 0;

	if (is_default_enable(rxf->rxmode_pending,
				rxf->rxmode_pending_bitmask) ||
		(rxf->rxmode_active & BNA_RXMODE_DEFAULT)) {
		/* Do nothing if pending enable or already enabled */
		CNA_ASSERT(bna->default_mode_rid == rxf->rx->rid);
	} else if (is_default_disable(rxf->rxmode_pending,
					rxf->rxmode_pending_bitmask)) {
		/* Turn off pending disable command */
		CNA_ASSERT(bna->default_mode_rid == rxf->rx->rid);
		CNA_ASSERT(rxf->rxmode_active & BNA_RXMODE_DEFAULT);
		default_inactive(rxf->rxmode_pending,
			rxf->rxmode_pending_bitmask);
	} else {
		/* Schedule enable */
		CNA_ASSERT(bna->default_mode_rid == BFI_INVALID_RID);
		CNA_ASSERT(!(rxf->rxmode_active & BNA_RXMODE_DEFAULT));
		default_enable(rxf->rxmode_pending,
				rxf->rxmode_pending_bitmask);
		bna->default_mode_rid = rxf->rx->rid;
		ret = 1;
	}

	return ret;
}

static int
bna_rxf_default_disable(struct bna_rxf_s *rxf)
{
	struct bna_s *bna = rxf->rx->bna;
	int ret = 0;

	if (is_default_disable(rxf->rxmode_pending,
				rxf->rxmode_pending_bitmask) ||
		(!(rxf->rxmode_active & BNA_RXMODE_DEFAULT))) {
		/* Do nothing if pending disable or already disabled */
	} else if (is_default_enable(rxf->rxmode_pending,
					rxf->rxmode_pending_bitmask)) {
		/* Turn off pending enable command */
		CNA_ASSERT(!(rxf->rxmode_active & BNA_RXMODE_DEFAULT));
		CNA_ASSERT(bna->default_mode_rid == rxf->rx->rid);
		default_inactive(rxf->rxmode_pending,
				rxf->rxmode_pending_bitmask);
		bna->default_mode_rid = BFI_INVALID_RID;
	} else if (rxf->rxmode_active & BNA_RXMODE_DEFAULT) {
		/* Schedule disable */
		CNA_ASSERT(bna->default_mode_rid == rxf->rx->rid);
		default_disable(rxf->rxmode_pending,
				rxf->rxmode_pending_bitmask);
		ret = 1;
	}

	return ret;
}

static int
bna_rxf_allmulti_enable(struct bna_rxf_s *rxf)
{
	int ret = 0;

	if (is_allmulti_enable(rxf->rxmode_pending,
			rxf->rxmode_pending_bitmask) ||
			(rxf->rxmode_active & BNA_RXMODE_ALLMULTI)) {
		/* Do nothing if pending enable or already enabled */
	} else if (is_allmulti_disable(rxf->rxmode_pending,
					rxf->rxmode_pending_bitmask)) {
		/* Turn off pending disable command */
		CNA_ASSERT(rxf->rxmode_active & BNA_RXMODE_ALLMULTI);
		allmulti_inactive(rxf->rxmode_pending,
			rxf->rxmode_pending_bitmask);
	} else {
		/* Schedule enable */
		CNA_ASSERT(!(rxf->rxmode_active & BNA_RXMODE_ALLMULTI));
		allmulti_enable(rxf->rxmode_pending,
				rxf->rxmode_pending_bitmask);
		ret = 1;
	}

	return ret;
}

static int
bna_rxf_allmulti_disable(struct bna_rxf_s *rxf)
{
	int ret = 0;

	if (is_allmulti_disable(rxf->rxmode_pending,
				rxf->rxmode_pending_bitmask) ||
		(!(rxf->rxmode_active & BNA_RXMODE_ALLMULTI))) {
		/* Do nothing if pending disable or already disabled */
	} else if (is_allmulti_enable(rxf->rxmode_pending,
					rxf->rxmode_pending_bitmask)) {
		/* Turn off pending enable command */
		CNA_ASSERT(!(rxf->rxmode_active & BNA_RXMODE_ALLMULTI));
		allmulti_inactive(rxf->rxmode_pending,
				rxf->rxmode_pending_bitmask);
	} else if (rxf->rxmode_active & BNA_RXMODE_ALLMULTI) {
		/* Schedule disable */
		allmulti_disable(rxf->rxmode_pending,
				rxf->rxmode_pending_bitmask);
		ret = 1;
	}

	return ret;
}

static void
bna_rit_init(struct bna_rxf_s *rxf, int rit_size)
{
	struct bna_rx_s *rx = rxf->rx;
	struct bna_s *bna = rx->bna;
	struct bna_rxp_s *rxp;
	bfa_q_t *qe;
	int offset = 0;

	rxf->rit_size = rit_size;
	bfa_q_iter(&rx->rxp_q, qe) {
		rxp = (struct bna_rxp_s *)qe;
		rxf->rit[offset] = rxp->cq.ccb->id;
		offset++;
	}

	CNA_ASSERT(offset <= bna->ioceth.attr.max_rit_size);
}

void
bna_bfi_rxf_cfg_rsp(struct bna_rxf_s *rxf, struct bfi_msgq_mhdr_s *msghdr)
{
	struct bfi_enet_rsp *rsp = (struct bfi_enet_rsp *)msghdr;

	CNA_ASSERT(rsp->error == 0);
	bfa_fsm_send_event(rxf, RXF_E_FW_RESP);
}

/**
 * @brief UCAST set is called for the base mac address
 *           Notify shim of any failure.
 */
void
bna_bfi_rxf_ucast_set_rsp(struct bna_rxf_s *rxf,
			struct bfi_msgq_mhdr_s *msghdr)
{
	struct bfi_enet_rsp *rsp =
		(struct bfi_enet_rsp *)msghdr;

	bfa_trc(rxf->rx->bna, rsp->error);
	if (rsp->error) {
		/* Post AEN */
		bfa_ioc_aen_post(&rxf->rx->bna->ioceth.ioc,
			BFA_IOC_AEN_INVALID_MAC);
		/* Clear ucast from cache */
		rxf->ucast_active_set = 0;
		// @TODO: Clear the MAC entry?
	}

	bfa_fsm_send_event(rxf, RXF_E_FW_RESP);
}

void
bna_bfi_rxf_mcast_add_rsp(struct bna_rxf_s *rxf,
			struct bfi_msgq_mhdr_s *msghdr)
{
	struct bfi_enet_mcast_add_req *req =
		&rxf->bfi_enet_cmd.mcast_add_req;
	struct bfi_enet_mcast_add_rsp *rsp =
		(struct bfi_enet_mcast_add_rsp *)msghdr;

	CNA_ASSERT(rsp->error == 0);
	bna_rxf_mchandle_attach(rxf, (uint8_t *)&req->mac_addr,
		cna_os_ntohs(rsp->handle));
	bfa_fsm_send_event(rxf, RXF_E_FW_RESP);
}

void
bna_rxf_init(struct bna_rxf_s *rxf,
		struct bna_rx_s *rx,
		struct bna_rx_config_s *q_config,
		struct bna_res_info_s *res_info)
{
	rxf->rx = rx;

	bfa_q_init(&rxf->ucast_pending_add_q);
	bfa_q_init(&rxf->ucast_pending_del_q);
	rxf->ucast_pending_set = 0;
	rxf->ucast_active_set = 0;
	bfa_q_init(&rxf->ucast_active_q);
	rxf->ucast_pending_mac = NULL;

	bfa_q_init(&rxf->mcast_pending_add_q);
	bfa_q_init(&rxf->mcast_pending_del_q);
	bfa_q_init(&rxf->mcast_active_q);
	bfa_q_init(&rxf->mcast_handle_q);

	if (q_config->paused)
		rxf->flags |= BNA_RXF_F_PAUSED;

	rxf->rit = (uint8_t *)
		res_info[BNA_RX_RES_MEM_T_RIT].res_u.mem_info.mdl[0].kva;
	bna_rit_init(rxf, q_config->num_paths);

	rxf->rss_status = q_config->rss_status;
	if (rxf->rss_status == BNA_STATUS_T_ENABLED) {
		bfa_os_assign(rxf->rss_cfg, q_config->rss_config);
		rxf->rss_pending |= BNA_RSS_F_CFG_PENDING;
		rxf->rss_pending |= BNA_RSS_F_RIT_PENDING;
		rxf->rss_pending |= BNA_RSS_F_STATUS_PENDING;
	}

	rxf->vlan_filter_status = BNA_STATUS_T_DISABLED;
	cna_os_memset(rxf->vlan_filter_table, 0,
			(sizeof(uint32_t) * ((BFI_ENET_VLAN_ID_MAX + 1) / 32)));
	rxf->vlan_filter_table[0] |= 1; /* for pure priority tagged frames */
	rxf->vlan_pending_bitmask = (uint8_t)BFI_VLAN_BMASK_ALL;

	rxf->vlan_strip_status = q_config->vlan_strip_status;

	bfa_q_init(&rxf->wol_m_pending_add_q);
	bfa_q_init(&rxf->wol_m_pending_del_q);
	bfa_q_init(&rxf->wol_m_active_q);
	bfa_q_init(&rxf->wol_f_pending_add_q);
	bfa_q_init(&rxf->wol_f_pending_del_q);
	bfa_q_init(&rxf->wol_f_active_q);

	bfa_fsm_set_state(rxf, bna_rxf_sm_stopped);
}

void
bna_rxf_uninit(struct bna_rxf_s *rxf)
{
	struct bna_mac_s *mac;

	rxf->ucast_pending_set = 0;
	rxf->ucast_active_set = 0;

	while (!bfa_q_is_empty(&rxf->ucast_pending_add_q)) {
		bfa_q_deq(&rxf->ucast_pending_add_q, &mac);
		bfa_q_qe_init(&mac->qe);
		bna_cam_mod_mac_put(bna_ucam_mod_free_q(rxf->rx->bna), mac);
	}

	CNA_ASSERT(bfa_q_is_empty(&rxf->ucast_pending_del_q));
	CNA_ASSERT(bfa_q_is_empty(&rxf->ucast_active_q));

	if (rxf->ucast_pending_mac) {
		bfa_q_qe_init(&rxf->ucast_pending_mac->qe);
		bna_cam_mod_mac_put(bna_ucam_mod_free_q(rxf->rx->bna),
				rxf->ucast_pending_mac);
		rxf->ucast_pending_mac = NULL;
	}

	while (!bfa_q_is_empty(&rxf->mcast_pending_add_q)) {
		bfa_q_deq(&rxf->mcast_pending_add_q, &mac);
		bfa_q_qe_init(&mac->qe);
		bna_cam_mod_mac_put(bna_mcam_mod_free_q(rxf->rx->bna), mac);
	}

	CNA_ASSERT(bfa_q_is_empty(&rxf->mcast_pending_del_q));
	CNA_ASSERT(bfa_q_is_empty(&rxf->mcast_active_q));
	CNA_ASSERT(bfa_q_is_empty(&rxf->mcast_handle_q));

	rxf->rxmode_pending = 0;
	rxf->rxmode_pending_bitmask = 0;
	if (rxf->rx->bna->promisc_rid == rxf->rx->rid)
		rxf->rx->bna->promisc_rid = BFI_INVALID_RID;
	if (rxf->rx->bna->default_mode_rid == rxf->rx->rid)
		rxf->rx->bna->default_mode_rid = BFI_INVALID_RID;
	CNA_ASSERT(!(rxf->rxmode_active & BNA_RXMODE_PROMISC));
	CNA_ASSERT(!(rxf->rxmode_active & BNA_RXMODE_DEFAULT));
	CNA_ASSERT(!(rxf->rxmode_active & BNA_RXMODE_ALLMULTI));

	rxf->rss_pending = 0;
	rxf->vlan_strip_pending = BFA_FALSE;

	rxf->flags = 0;

	rxf->rx = NULL;
}

void
bna_rxf_start(struct bna_rxf_s *rxf)
{
	CNA_ASSERT(rxf->start_cbfn == NULL);
	CNA_ASSERT(rxf->stop_cbfn == NULL);
	CNA_ASSERT(rxf->cam_fltr_cbfn == NULL);
	CNA_ASSERT(rxf->oper_state_cbfn == NULL);
	rxf->start_cbfn = bna_rx_cb_rxf_started;
	rxf->start_cbarg = rxf->rx;
	bfa_fsm_send_event(rxf, RXF_E_START);
}

void
bna_rxf_stop(struct bna_rxf_s *rxf)
{
	CNA_ASSERT(rxf->start_cbfn == NULL);
	CNA_ASSERT(rxf->stop_cbfn == NULL);
	CNA_ASSERT(rxf->cam_fltr_cbfn == NULL);
	CNA_ASSERT(rxf->oper_state_cbfn == NULL);
	rxf->stop_cbfn = bna_rx_cb_rxf_stopped;
	rxf->stop_cbarg = rxf->rx;
	bfa_fsm_send_event(rxf, RXF_E_STOP);
}

void
bna_rxf_fail(struct bna_rxf_s *rxf)
{
	bfa_fsm_send_event(rxf, RXF_E_FAIL);
}

enum bna_cb_status_e
bna_rx_ucast_set(struct bna_rx_s *rx, uint8_t *ucmac,
		 void (*cbfn)(struct bnad_s *, struct bna_rx_s *))
{
	struct bna_rxf_s *rxf = &rx->rxf;

	CNA_ASSERT(rxf->stop_cbfn == NULL);
	CNA_ASSERT(rxf->cam_fltr_cbfn == NULL);
	CNA_ASSERT(rxf->oper_state_cbfn == NULL);

	if (rxf->ucast_pending_mac == NULL) {
		rxf->ucast_pending_mac =
			bna_cam_mod_mac_get(bna_ucam_mod_free_q(rxf->rx->bna));
		if (rxf->ucast_pending_mac == NULL)
			return BNA_CB_UCAST_CAM_FULL;
		bfa_q_qe_init(&rxf->ucast_pending_mac->qe);
	}

	cna_os_memcpy(rxf->ucast_pending_mac->addr, ucmac, CNA_ETH_ALEN);
	rxf->ucast_pending_set = 1;
	rxf->cam_fltr_cbfn = cbfn;
	rxf->cam_fltr_cbarg = rx->bna->bnad;

	bfa_fsm_send_event(rxf, RXF_E_CONFIG);

	return BNA_CB_SUCCESS;
}
/**
 * Explicitly removes the default MAC address. The caller is responsible for
 * setting the default MAC address by calling bna_rx_ucast_set
 */
enum bna_cb_status_e
bna_rx_ucast_clr(struct bna_rx_s *rx,
		 void (*cbfn)(struct bnad_s *, struct bna_rx_s *))
{
	struct bna_rxf_s *rxf = &rx->rxf;
	struct bna_mac_s *mac, *del_mac;

	CNA_ASSERT(rxf->stop_cbfn == NULL);
	CNA_ASSERT(rxf->cam_fltr_cbfn == NULL);
	CNA_ASSERT(rxf->oper_state_cbfn == NULL);

	if (rxf->ucast_pending_mac != NULL) {
		mac = rxf->ucast_pending_mac;

		// Reset all the default MAC address related flags/fields
		rxf->ucast_pending_mac = NULL;
		rxf->ucast_pending_set = 0;
		rxf->ucast_active_set = 0;

		del_mac = bna_cam_mod_mac_get(bna_ucam_mod_del_q(rx->bna));
		CNA_ASSERT(del_mac);
		cna_os_memcpy(del_mac, mac, sizeof(*del_mac));
		bfa_q_enq(&rxf->ucast_pending_del_q, &del_mac->qe);
		rxf->cam_fltr_cbfn = cbfn;
		rxf->cam_fltr_cbarg = rx->bna->bnad;

		bna_cam_mod_mac_put(bna_ucam_mod_free_q(rx->bna), mac);

		bfa_fsm_send_event(rxf, RXF_E_CONFIG);
	}

	return BNA_CB_SUCCESS;
}

enum bna_cb_status_e
bna_rx_ucast_add(struct bna_rx_s *rx, uint8_t *addr,
		 void (*cbfn)(struct bnad_s *, struct bna_rx_s *))
{
	struct bna_rxf_s *rxf = &rx->rxf;
	struct bna_mac_s *mac;

	CNA_ASSERT(rxf->stop_cbfn == NULL);
	CNA_ASSERT(rxf->cam_fltr_cbfn == NULL);
	CNA_ASSERT(rxf->oper_state_cbfn == NULL);

	/* Check if already added or pending addition */
	if (bna_mac_find(&rxf->ucast_active_q, addr) ||
		bna_mac_find(&rxf->ucast_pending_add_q, addr)) {
		if (cbfn)
			cbfn(rx->bna->bnad, rx);
		return BNA_CB_SUCCESS;
	}

	mac = bna_cam_mod_mac_get(bna_ucam_mod_free_q(rxf->rx->bna));
	if (mac == NULL)
		return BNA_CB_UCAST_CAM_FULL;
	bfa_q_qe_init(&mac->qe);
	cna_os_memcpy(mac->addr, addr, CNA_ETH_ALEN);
	bfa_q_enq(&rxf->ucast_pending_add_q, &mac->qe);

	rxf->cam_fltr_cbfn = cbfn;
	rxf->cam_fltr_cbarg = rx->bna->bnad;

	bfa_fsm_send_event(rxf, RXF_E_CONFIG);

	return BNA_CB_SUCCESS;
}

enum bna_cb_status_e
bna_rx_ucast_del(struct bna_rx_s *rx, uint8_t *addr,
		 void (*cbfn)(struct bnad_s *, struct bna_rx_s *))
{
	struct bna_rxf_s *rxf = &rx->rxf;
	struct bna_mac_s *mac, *del_mac;

	CNA_ASSERT(rxf->stop_cbfn == NULL);
	CNA_ASSERT(rxf->cam_fltr_cbfn == NULL);
	CNA_ASSERT(rxf->oper_state_cbfn == NULL);

	mac = bna_mac_find(&rxf->ucast_pending_add_q, addr);
	if (mac) {
		bfa_q_qe_deq(&mac->qe);
		bfa_q_qe_init(&mac->qe);
		bna_cam_mod_mac_put(bna_ucam_mod_free_q(rxf->rx->bna), mac);
		if (cbfn)
			(*cbfn)(rx->bna->bnad, rx);
		return BNA_CB_SUCCESS;
	}

	mac = bna_mac_find(&rxf->ucast_active_q, addr);
	if (mac) {
		bfa_q_qe_deq(&mac->qe);
		bfa_q_qe_init(&mac->qe);

		del_mac = bna_cam_mod_mac_get(bna_ucam_mod_del_q(rxf->rx->bna));
		CNA_ASSERT(del_mac);

		cna_os_memcpy(del_mac, mac, sizeof(*del_mac));
		bfa_q_enq(&rxf->ucast_pending_del_q, &del_mac->qe);
		bna_cam_mod_mac_put(bna_ucam_mod_free_q(rxf->rx->bna), mac);
		rxf->cam_fltr_cbfn = cbfn;
		rxf->cam_fltr_cbarg = rx->bna->bnad;
		bfa_fsm_send_event(rxf, RXF_E_CONFIG);
		return BNA_CB_SUCCESS;
	}

	return BNA_CB_INVALID_MAC;
}

enum bna_cb_status_e
bna_rx_mcast_add(struct bna_rx_s *rx, uint8_t *addr,
		 void (*cbfn)(struct bnad_s *, struct bna_rx_s *))
{
	struct bna_rxf_s *rxf = &rx->rxf;
	struct bna_mac_s *mac;

	CNA_ASSERT(rxf->stop_cbfn == NULL);
	CNA_ASSERT(rxf->cam_fltr_cbfn == NULL);
	CNA_ASSERT(rxf->oper_state_cbfn == NULL);

	/* Check if already added or pending addition */
	if (bna_mac_find(&rxf->mcast_active_q, addr) ||
		bna_mac_find(&rxf->mcast_pending_add_q, addr)) {
		if (cbfn)
			cbfn(rx->bna->bnad, rx);
		return BNA_CB_SUCCESS;
	}

	mac = bna_cam_mod_mac_get(bna_mcam_mod_free_q(rxf->rx->bna));
	if (mac == NULL)
		return BNA_CB_MCAST_LIST_FULL;
	bfa_q_qe_init(&mac->qe);
	cna_os_memcpy(mac->addr, addr, CNA_ETH_ALEN);
	bfa_q_enq(&rxf->mcast_pending_add_q, &mac->qe);

	rxf->cam_fltr_cbfn = cbfn;
	rxf->cam_fltr_cbarg = rx->bna->bnad;

	bfa_fsm_send_event(rxf, RXF_E_CONFIG);

	return BNA_CB_SUCCESS;
}

enum bna_cb_status_e
bna_rx_mcast_del(struct bna_rx_s *rx, uint8_t *addr,
		 void (*cbfn)(struct bnad_s *, struct bna_rx_s *))
{
	struct bna_rxf_s *rxf = &rx->rxf;
	struct bna_mac_s *mac, *del_mac;

	CNA_ASSERT(rxf->stop_cbfn == NULL);
	CNA_ASSERT(rxf->cam_fltr_cbfn == NULL);
	CNA_ASSERT(rxf->oper_state_cbfn == NULL);

	mac = bna_mac_find(&rxf->mcast_pending_add_q, addr);
	if (mac) {
		bfa_q_qe_deq(&mac->qe);
		bfa_q_qe_init(&mac->qe);
		bna_cam_mod_mac_put(bna_mcam_mod_free_q(rxf->rx->bna), mac);
		if (cbfn)
			(*cbfn)(rx->bna->bnad, rx);
		return BNA_CB_SUCCESS;
	}

	mac = bna_mac_find(&rxf->mcast_active_q, addr);
	if (mac) {
		bfa_q_qe_deq(&mac->qe);
		bfa_q_qe_init(&mac->qe);

		del_mac = bna_cam_mod_mac_get(bna_mcam_mod_del_q(rxf->rx->bna));
		CNA_ASSERT(del_mac);

		cna_os_memcpy(del_mac, mac, sizeof(*del_mac));
		bfa_q_enq(&rxf->mcast_pending_del_q, &del_mac->qe);
		mac->handle = NULL;
		bna_cam_mod_mac_put(bna_mcam_mod_free_q(rxf->rx->bna), mac);
		rxf->cam_fltr_cbfn = cbfn;
		rxf->cam_fltr_cbarg = rx->bna->bnad;
		bfa_fsm_send_event(rxf, RXF_E_CONFIG);
		return BNA_CB_SUCCESS;
	}

	return BNA_CB_INVALID_MAC;
}

enum bna_cb_status_e
bna_rx_ucast_listset(struct bna_rx_s *rx, int count, uint8_t *uclist,
		     void (*cbfn)(struct bnad_s *, struct bna_rx_s *))
{
	struct bna_ucam_mod_s *ucam_mod = &rx->bna->ucam_mod;
	struct bna_rxf_s *rxf = &rx->rxf;
	bfa_q_t list_head;
	bfa_q_t *qe;
	uint8_t *mcaddr;
	struct bna_mac_s *mac, *del_mac;
	int i;

	CNA_ASSERT(rxf->stop_cbfn == NULL);
	CNA_ASSERT(rxf->cam_fltr_cbfn == NULL);
	CNA_ASSERT(rxf->oper_state_cbfn == NULL);

	/* Purge the pending_add_q */
	while (!bfa_q_is_empty(&rxf->ucast_pending_add_q)) {
		bfa_q_deq(&rxf->ucast_pending_add_q, &qe);
		bfa_q_qe_init(qe);
		mac = (struct bna_mac_s *)qe;
		bna_cam_mod_mac_put(&ucam_mod->free_q, mac);
	}

	/* Schedule active_q entries for deletion */
	while (!bfa_q_is_empty(&rxf->ucast_active_q)) {
		bfa_q_deq(&rxf->ucast_active_q, &qe);
		mac = (struct bna_mac_s *)qe;
		bfa_q_qe_init(&mac->qe);

		del_mac = bna_cam_mod_mac_get(&ucam_mod->del_q);
		CNA_ASSERT(del_mac);
		cna_os_memcpy(del_mac, mac, sizeof(*del_mac));
		bfa_q_enq(&rxf->ucast_pending_del_q, &del_mac->qe);
		bna_cam_mod_mac_put(&ucam_mod->free_q, mac);
	}

	/* Allocate nodes */
	bfa_q_init(&list_head);
	for (i = 0, mcaddr = uclist; i < count; i++) {
		mac = bna_cam_mod_mac_get(&ucam_mod->free_q);
		if (mac == NULL)
			goto err_return;
		bfa_q_qe_init(&mac->qe);
		cna_os_memcpy(mac->addr, mcaddr, CNA_ETH_ALEN);
		bfa_q_enq(&list_head, &mac->qe);
		mcaddr += CNA_ETH_ALEN;
	}

	/* Add the new entries */
	while (!bfa_q_is_empty(&list_head)) {
		bfa_q_deq(&list_head, &qe);
		mac = (struct bna_mac_s *)qe;
		bfa_q_qe_init(&mac->qe);
		bfa_q_enq(&rxf->ucast_pending_add_q, &mac->qe);
	}

	rxf->cam_fltr_cbfn = cbfn;
	rxf->cam_fltr_cbarg = rx->bna->bnad;
	bfa_fsm_send_event(rxf, RXF_E_CONFIG);

	return BNA_CB_SUCCESS;

err_return:
	while (!bfa_q_is_empty(&list_head)) {
		bfa_q_deq(&list_head, &qe);
		mac = (struct bna_mac_s *)qe;
		bfa_q_qe_init(&mac->qe);
		bna_cam_mod_mac_put(&ucam_mod->free_q, mac);
	}

	return BNA_CB_UCAST_CAM_FULL;
}

enum bna_cb_status_e
bna_rx_mcast_listset(struct bna_rx_s *rx, int count, uint8_t *mclist,
		     void (*cbfn)(struct bnad_s *, struct bna_rx_s *))
{
	struct bna_mcam_mod_s *mcam_mod = &rx->bna->mcam_mod;
	struct bna_rxf_s *rxf = &rx->rxf;
	bfa_q_t list_head;
	bfa_q_t *qe;
	uint8_t *mcaddr;
	struct bna_mac_s *mac, *del_mac;
	int i;

	CNA_ASSERT(rxf->stop_cbfn == NULL);
	CNA_ASSERT(rxf->cam_fltr_cbfn == NULL);
	CNA_ASSERT(rxf->oper_state_cbfn == NULL);

	/* Purge the pending_add_q */
	while (!bfa_q_is_empty(&rxf->mcast_pending_add_q)) {
		bfa_q_deq(&rxf->mcast_pending_add_q, &qe);
		bfa_q_qe_init(qe);
		mac = (struct bna_mac_s *)qe;
		bna_cam_mod_mac_put(&mcam_mod->free_q, mac);
	}

	/* Schedule active_q entries for deletion */
	while (!bfa_q_is_empty(&rxf->mcast_active_q)) {
		bfa_q_deq(&rxf->mcast_active_q, &qe);
		mac = (struct bna_mac_s *)qe;
		bfa_q_qe_init(&mac->qe);

		del_mac = bna_cam_mod_mac_get(&mcam_mod->del_q);
		CNA_ASSERT(del_mac);

		cna_os_memcpy(del_mac, mac, sizeof(*del_mac));
		bfa_q_enq(&rxf->mcast_pending_del_q, &del_mac->qe);
		mac->handle = NULL;
		bna_cam_mod_mac_put(&mcam_mod->free_q, mac);
	}

	/* Allocate nodes */
	bfa_q_init(&list_head);
	for (i = 0, mcaddr = mclist; i < count; i++) {
		mac = bna_cam_mod_mac_get(&mcam_mod->free_q);
		if (mac == NULL)
			goto err_return;
		bfa_q_qe_init(&mac->qe);
		cna_os_memcpy(mac->addr, mcaddr, CNA_ETH_ALEN);
		bfa_q_enq(&list_head, &mac->qe);

		mcaddr += CNA_ETH_ALEN;
	}

	/* Add the new entries */
	while (!bfa_q_is_empty(&list_head)) {
		bfa_q_deq(&list_head, &qe);
		mac = (struct bna_mac_s *)qe;
		bfa_q_qe_init(&mac->qe);
		bfa_q_enq(&rxf->mcast_pending_add_q, &mac->qe);
	}

	rxf->cam_fltr_cbfn = cbfn;
	rxf->cam_fltr_cbarg = rx->bna->bnad;
	bfa_fsm_send_event(rxf, RXF_E_CONFIG);

	return BNA_CB_SUCCESS;

err_return:
	while (!bfa_q_is_empty(&list_head)) {
		bfa_q_deq(&list_head, &qe);
		mac = (struct bna_mac_s *)qe;
		bfa_q_qe_init(&mac->qe);
		bna_cam_mod_mac_put(&mcam_mod->free_q, mac);
	}

	return BNA_CB_MCAST_LIST_FULL;
}

void
bna_rx_mcast_delall(struct bna_rx_s *rx,
		    void (*cbfn)(struct bnad_s *, struct bna_rx_s *))
{
	struct bna_rxf_s *rxf = &rx->rxf;
	bfa_q_t *qe;
	struct bna_mac_s *mac, *del_mac;
	int need_hw_config = 0;

	CNA_ASSERT(rxf->stop_cbfn == NULL);
	CNA_ASSERT(rxf->cam_fltr_cbfn == NULL);
	CNA_ASSERT(rxf->oper_state_cbfn == NULL);

	/* Purge all entries from pending_add_q */
	while (!bfa_q_is_empty(&rxf->mcast_pending_add_q)) {
		bfa_q_deq(&rxf->mcast_pending_add_q, &qe);
		mac = (struct bna_mac_s *)qe;
		bfa_q_qe_init(&mac->qe);
		bna_cam_mod_mac_put(bna_mcam_mod_free_q(rxf->rx->bna), mac);
	}

	/* Schedule all entries in active_q for deletion */
	while (!bfa_q_is_empty(&rxf->mcast_active_q)) {
		bfa_q_deq(&rxf->mcast_active_q, &qe);
		mac = (struct bna_mac_s *)qe;
		bfa_q_qe_init(&mac->qe);

		del_mac = bna_cam_mod_mac_get(bna_mcam_mod_del_q(rxf->rx->bna));
		CNA_ASSERT(del_mac);

		cna_os_memcpy(del_mac, mac, sizeof(*del_mac));
		bfa_q_enq(&rxf->mcast_pending_del_q, &del_mac->qe);
		mac->handle = NULL;
		bna_cam_mod_mac_put(bna_mcam_mod_free_q(rxf->rx->bna), mac);
		need_hw_config = 1;
	}

	if (need_hw_config) {
		rxf->cam_fltr_cbfn = cbfn;
		rxf->cam_fltr_cbarg = rx->bna->bnad;
		bfa_fsm_send_event(rxf, RXF_E_CONFIG);
		return;
	}

	if (cbfn)
		(*cbfn)(rx->bna->bnad, rx);
}

static inline struct bna_wol_s *
bna_wol_find(bfa_q_t *q, uint32_t id)
{
	struct bna_wol_s *wol = NULL;
	bfa_q_t *qe;
	bfa_q_iter(q, qe) {
		if (((struct bna_wol_s *)qe)->os_id == id) {
			wol = (struct bna_wol_s *)qe;
			break;
		}
	}
	return wol;
}


enum bna_cb_status_e
bna_rx_wol_add_magic(struct bna_rx_s *rx, uint32_t id, mac_t *addr,
	void (*cbfn)(struct bnad_s *, struct bna_rx_s *))
{
	struct bna_rxf_s *rxf = &rx->rxf;
	struct bna_wol_s *wol;

	CNA_ASSERT(rxf->stop_cbfn == NULL);
	CNA_ASSERT(rxf->cam_fltr_cbfn == NULL);
	CNA_ASSERT(rxf->oper_state_cbfn == NULL);

	/* Check if already added or pending addition */
	if (bna_wol_find(&rxf->wol_m_active_q, id) ||
		bna_wol_find(&rxf->wol_m_pending_add_q, id)) {
		if (cbfn)
			cbfn(rx->bna->bnad, rx);
		return BNA_CB_SUCCESS;
	}

	wol = bna_wol_mod_magic_get(&rxf->rx->bna->wol_mod);
	if (wol == NULL)
		return BNA_CB_UCAST_CAM_FULL;
	bfa_q_qe_init(&wol->qe);
	wol->os_id = id;
	cna_os_memcpy(wol->pattern, addr, CNA_ETH_ALEN);
	bfa_q_enq(&rxf->wol_m_pending_add_q, &wol->qe);

	rxf->cam_fltr_cbfn = cbfn;
	rxf->cam_fltr_cbarg = rx->bna->bnad;

	bfa_fsm_send_event(rxf, RXF_E_CONFIG);

	return BNA_CB_SUCCESS;
}

enum bna_cb_status_e
bna_rx_wol_add_frame(struct bna_rx_s *rx, uint32_t id,
	uint8_t *pattern, uint8_t *mask,
	void (*cbfn)(struct bnad_s *, struct bna_rx_s *))
{
	struct bna_rxf_s *rxf = &rx->rxf;
	struct bna_wol_s *wol;

	CNA_ASSERT(rxf->stop_cbfn == NULL);
	CNA_ASSERT(rxf->cam_fltr_cbfn == NULL);
	CNA_ASSERT(rxf->oper_state_cbfn == NULL);

	/* Check if already added or pending addition */
	if (bna_wol_find(&rxf->wol_f_active_q, id) ||
		bna_wol_find(&rxf->wol_f_pending_add_q, id)) {
		if (cbfn)
			cbfn(rx->bna->bnad, rx);
		return BNA_CB_SUCCESS;
	}

	wol = bna_wol_mod_frame_get(&rxf->rx->bna->wol_mod);
	if (wol == NULL)
		return BNA_CB_UCAST_CAM_FULL;
	bfa_q_qe_init(&wol->qe);
	wol->os_id = id;
	cna_os_memcpy(wol->pattern, pattern, BFI_ENET_WOL_FRAME_LEN);
	cna_os_memcpy(wol->mask, mask, BFI_ENET_WOL_FRAME_MASK_LEN);
	bfa_q_enq(&rxf->wol_f_pending_add_q, &wol->qe);

	rxf->cam_fltr_cbfn = cbfn;
	rxf->cam_fltr_cbarg = rx->bna->bnad;

	bfa_fsm_send_event(rxf, RXF_E_CONFIG);

	return BNA_CB_SUCCESS;
}


enum bna_cb_status_e
bna_rx_wol_del_magic(struct bna_rx_s *rx, uint32_t id,
	void (*cbfn)(struct bnad_s *, struct bna_rx_s *))
{
	struct bna_rxf_s *rxf = &rx->rxf;
	struct bna_wol_s *wol;

	CNA_ASSERT(rxf->stop_cbfn == NULL);
	CNA_ASSERT(rxf->cam_fltr_cbfn == NULL);
	CNA_ASSERT(rxf->oper_state_cbfn == NULL);

	wol = bna_wol_find(&rxf->wol_m_pending_add_q, id);
	if (wol) {
		bfa_q_qe_deq(&wol->qe);
		bfa_q_qe_init(&wol->qe);
		bna_wol_mod_magic_put(&rxf->rx->bna->wol_mod, wol);
		if (cbfn)
			(*cbfn)(rx->bna->bnad, rx);
		return BNA_CB_SUCCESS;
	}
	wol = bna_wol_find(&rxf->wol_m_active_q, id);
	if (wol) {
		bfa_q_qe_deq(&wol->qe);
		bfa_q_qe_init(&wol->qe);
		bfa_q_enq(&rxf->wol_m_pending_del_q, &wol->qe);
		rxf->cam_fltr_cbfn = cbfn;
		rxf->cam_fltr_cbarg = rx->bna->bnad;
		bfa_fsm_send_event(rxf, RXF_E_CONFIG);
		return BNA_CB_SUCCESS;
	}
	return BNA_CB_INVALID_MAC;
}

enum bna_cb_status_e
bna_rx_wol_del_frame(struct bna_rx_s *rx, uint32_t id,
	void (*cbfn)(struct bnad_s *, struct bna_rx_s *))
{
	struct bna_rxf_s *rxf = &rx->rxf;
	struct bna_wol_s *wol;

	CNA_ASSERT(rxf->stop_cbfn == NULL);
	CNA_ASSERT(rxf->cam_fltr_cbfn == NULL);
	CNA_ASSERT(rxf->oper_state_cbfn == NULL);

	wol = bna_wol_find(&rxf->wol_f_pending_add_q, id);
	if (wol) {
		bfa_q_qe_deq(&wol->qe);
		bfa_q_qe_init(&wol->qe);
		bna_wol_mod_frame_put(&rxf->rx->bna->wol_mod, wol);
		if (cbfn)
			(*cbfn)(rx->bna->bnad, rx);
		return BNA_CB_SUCCESS;
	}

	wol = bna_wol_find(&rxf->wol_f_active_q, id);
	if (wol) {
		bfa_q_qe_deq(&wol->qe);
		bfa_q_qe_init(&wol->qe);
		bfa_q_enq(&rxf->wol_f_pending_del_q, &wol->qe);
		rxf->cam_fltr_cbfn = cbfn;
		rxf->cam_fltr_cbarg = rx->bna->bnad;
		bfa_fsm_send_event(rxf, RXF_E_CONFIG);
		return BNA_CB_SUCCESS;
	}

	return BNA_CB_INVALID_MAC;
}

enum bna_cb_status_e
bna_rx_mode_set(struct bna_rx_s *rx, enum bna_rxmode_e new_mode,
		enum bna_rxmode_e bitmask,
		void (*cbfn)(struct bnad_s *, struct bna_rx_s *))
{
	struct bna_rxf_s *rxf = &rx->rxf;
	int need_hw_config = 0;

	CNA_ASSERT(rxf->stop_cbfn == NULL);
	CNA_ASSERT(rxf->cam_fltr_cbfn == NULL);
	CNA_ASSERT(rxf->oper_state_cbfn == NULL);

	/* Error checks */

	if (is_promisc_enable(new_mode, bitmask)) {
		/* If promisc mode is already enabled elsewhere in the system */
		if ((rx->bna->promisc_rid != BFI_INVALID_RID) &&
			(rx->bna->promisc_rid != rxf->rx->rid))
			goto err_return;

		/* If default mode is already enabled in the system */
		if (rx->bna->default_mode_rid != BFI_INVALID_RID)
			goto err_return;

		/* Trying to enable promiscuous and default mode together */
		if (is_default_enable(new_mode, bitmask))
			goto err_return;
	}

	if (is_default_enable(new_mode, bitmask)) {
		/* If default mode is already enabled elsewhere in the system */
		if ((rx->bna->default_mode_rid != BFI_INVALID_RID) &&
			(rx->bna->default_mode_rid != rxf->rx->rid)) {
				goto err_return;
		}

		/* If promiscuous mode is already enabled in the system */
		if (rx->bna->promisc_rid != BFI_INVALID_RID)
			goto err_return;
	}

	/* Process the commands */

	if (is_promisc_enable(new_mode, bitmask)) {
		if (bna_rxf_promisc_enable(rxf))
			need_hw_config = 1;
	} else if (is_promisc_disable(new_mode, bitmask)) {
		if (bna_rxf_promisc_disable(rxf))
			need_hw_config = 1;
	}

	if (is_default_enable(new_mode, bitmask)) {
		if (bna_rxf_default_enable(rxf))
			need_hw_config = 1;
	} else if (is_default_disable(new_mode, bitmask)) {
		if (bna_rxf_default_disable(rxf))
			need_hw_config = 1;
	}

	if (is_allmulti_enable(new_mode, bitmask)) {
		if (bna_rxf_allmulti_enable(rxf))
			need_hw_config = 1;
	} else if (is_allmulti_disable(new_mode, bitmask)) {
		if (bna_rxf_allmulti_disable(rxf))
			need_hw_config = 1;
	}

	/* Trigger h/w if needed */

	if (need_hw_config) {
		rxf->cam_fltr_cbfn = cbfn;
		rxf->cam_fltr_cbarg = rx->bna->bnad;
		bfa_fsm_send_event(rxf, RXF_E_CONFIG);
	} else if (cbfn)
		(*cbfn)(rx->bna->bnad, rx);

	return BNA_CB_SUCCESS;

err_return:
	return BNA_CB_FAIL;
}

void
bna_rx_vlan_add(struct bna_rx_s *rx, int vlan_id)
{
	struct bna_rxf_s *rxf = &rx->rxf;
	int index = (vlan_id >> BFI_VLAN_WORD_SHIFT);
	int bit = (1 << (vlan_id & BFI_VLAN_WORD_MASK));
	int group_id = (vlan_id >> BFI_VLAN_BLOCK_SHIFT);

	rxf->vlan_filter_table[index] |= bit;
	if (rxf->vlan_filter_status == BNA_STATUS_T_ENABLED) {
		rxf->vlan_pending_bitmask |= (1 << group_id);
		bfa_fsm_send_event(rxf, RXF_E_CONFIG);
	}
}

void
bna_rx_vlan_del(struct bna_rx_s *rx, int vlan_id)
{
	struct bna_rxf_s *rxf = &rx->rxf;
	int index = (vlan_id >> BFI_VLAN_WORD_SHIFT);
	int bit = (1 << (vlan_id & BFI_VLAN_WORD_MASK));
	int group_id = (vlan_id >> BFI_VLAN_BLOCK_SHIFT);

	rxf->vlan_filter_table[index] &= ~bit;
	if (rxf->vlan_filter_status == BNA_STATUS_T_ENABLED) {
		rxf->vlan_pending_bitmask |= (1 << group_id);
		bfa_fsm_send_event(rxf, RXF_E_CONFIG);
	}
}

void
bna_rx_vlanfilter_enable(struct bna_rx_s *rx)
{
	struct bna_rxf_s *rxf = &rx->rxf;

	if (rxf->vlan_filter_status == BNA_STATUS_T_DISABLED) {
		rxf->vlan_filter_status = BNA_STATUS_T_ENABLED;
		rxf->vlan_pending_bitmask = (uint8_t)BFI_VLAN_BMASK_ALL;
		bfa_fsm_send_event(rxf, RXF_E_CONFIG);
	}
}

void
bna_rx_vlanfilter_disable(struct bna_rx_s *rx)
{
	struct bna_rxf_s *rxf = &rx->rxf;

	if (rxf->vlan_filter_status == BNA_STATUS_T_ENABLED) {
		rxf->vlan_filter_status = BNA_STATUS_T_DISABLED;
		rxf->vlan_pending_bitmask = (uint8_t)BFI_VLAN_BMASK_ALL;
		bfa_fsm_send_event(rxf, RXF_E_CONFIG);
	}
}

void
bna_rx_vlan_strip_enable(struct bna_rx_s *rx)
{
	struct bna_rxf_s *rxf = &rx->rxf;

	if (rxf->vlan_strip_status == BNA_STATUS_T_DISABLED) {
		rxf->vlan_strip_status = BNA_STATUS_T_ENABLED;
		rxf->vlan_strip_pending = BFA_TRUE;
		bfa_fsm_send_event(rxf, RXF_E_CONFIG);
	}
}

void
bna_rx_vlan_strip_disable(struct bna_rx_s *rx)
{
	struct bna_rxf_s *rxf = &rx->rxf;

	if (rxf->vlan_strip_status != BNA_STATUS_T_DISABLED) {
		rxf->vlan_strip_status = BNA_STATUS_T_DISABLED;
		rxf->vlan_strip_pending = BFA_TRUE;
		bfa_fsm_send_event(rxf, RXF_E_CONFIG);
	}
}

void
bna_rx_rss_enable(struct bna_rx_s *rx)
{
	struct bna_rxf_s *rxf = &rx->rxf;

	if (rxf->rss_status == BNA_STATUS_T_DISABLED) {
		rxf->rss_status = BNA_STATUS_T_ENABLED;
		rxf->rss_pending |= BNA_RSS_F_STATUS_PENDING;
		bfa_fsm_send_event(rxf, RXF_E_CONFIG);
	}
}

void
bna_rx_rss_disable(struct bna_rx_s *rx)
{
	struct bna_rxf_s *rxf = &rx->rxf;

	if (rxf->rss_status != BNA_STATUS_T_DISABLED) {
		rxf->rss_status = BNA_STATUS_T_DISABLED;
		rxf->rss_pending |= BNA_RSS_F_STATUS_PENDING;
		bfa_fsm_send_event(rxf, RXF_E_CONFIG);
	}
}

void
bna_rx_rss_reconfig(struct bna_rx_s *rx, struct bna_rss_config_s *rss_config)
{
	struct bna_rxf_s *rxf = &rx->rxf;
	bfa_os_assign(rxf->rss_cfg, *rss_config);
	rxf->rss_pending |= BNA_RSS_F_CFG_PENDING;
	bfa_fsm_send_event(rxf, RXF_E_CONFIG);
}

void
bna_rx_rss_rit_set(struct bna_rx_s *rx, unsigned int *vectors, int nvectors)
{
	int i;
	struct bna_rxp_s *rxp;
	struct bna_s *bna;
	struct bna_rxf_s *rxf;

	bna = rx->bna;

	CNA_ASSERT(nvectors <= bna->ioceth.attr.max_rit_size);

	/* Build the RIT contents for this RX */
	rxf = &rx->rxf;
	for (i = 0; i < nvectors; i++) {
		rxp = bna_rx_get_rxp(rx, vectors[i]);
		rxf->rit[i] = rxp->cq.ccb->id;
	}
	rxf->rit_size = nvectors;

	rxf->rss_pending |= BNA_RSS_F_RIT_PENDING;
	bfa_fsm_send_event(rxf, RXF_E_CONFIG);
}

void
bna_rx_receive_resume(struct bna_rx_s *rx,
		      void (*cbfn)(struct bnad_s *, struct bna_rx_s *))
{
	struct bna_rxf_s *rxf = &rx->rxf;

	CNA_ASSERT(cbfn);

	CNA_ASSERT(rxf->stop_cbfn == NULL);
	CNA_ASSERT(rxf->cam_fltr_cbfn == NULL);
	CNA_ASSERT(rxf->oper_state_cbfn == NULL);

	rxf->oper_state_cbfn = cbfn;
	rxf->oper_state_cbarg = rx->bna->bnad;
	bfa_fsm_send_event(rxf, RXF_E_RESUME);
}

void
bna_rx_receive_pause(struct bna_rx_s *rx,
		     void (*cbfn)(struct bnad_s *, struct bna_rx_s *))
{
	struct bna_rxf_s *rxf = &rx->rxf;

	CNA_ASSERT(cbfn);

	CNA_ASSERT(rxf->stop_cbfn == NULL);
	CNA_ASSERT(rxf->cam_fltr_cbfn == NULL);
	CNA_ASSERT(rxf->oper_state_cbfn == NULL);

	rxf->oper_state_cbfn = cbfn;
	rxf->oper_state_cbarg = rx->bna->bnad;
	bfa_fsm_send_event(rxf, RXF_E_PAUSE);
}

/**
 * RX
 */

#define	BNA_GET_RXQS(qcfg)	(((qcfg)->rxp_type == BNA_RXP_SINGLE) ?	\
	(qcfg)->num_paths : ((qcfg)->num_paths * 2))

#define	SIZE_TO_PAGES(size)	(((size) >> CNA_PAGE_SHIFT) + ((((size) &\
	(CNA_PAGE_SIZE - 1)) + (CNA_PAGE_SIZE - 1)) >> CNA_PAGE_SHIFT))

#define bfi_enet_datapath_q_init(bfi_q, bna_qpt)			\
do {									\
	struct bna_dma_addr_s cur_q_addr =				\
		*((struct bna_dma_addr_s *)((bna_qpt)->kv_qpt_ptr));	\
	(bfi_q)->pg_tbl.a32.addr_lo = (bna_qpt)->hw_qpt_ptr.lsb;	\
	(bfi_q)->pg_tbl.a32.addr_hi = (bna_qpt)->hw_qpt_ptr.msb;	\
	(bfi_q)->first_entry.a32.addr_lo = cur_q_addr.lsb;		\
	(bfi_q)->first_entry.a32.addr_hi = cur_q_addr.msb;		\
	(bfi_q)->pages = cna_os_htons((uint16_t)(bna_qpt)->page_count);	\
	(bfi_q)->page_sz = cna_os_htons((uint16_t)(bna_qpt)->page_size);\
} while (0)

#define	call_rx_stop_cbfn(rx)						\
do {									\
	if ((rx)->stop_cbfn) {						\
		void (*cbfn)(void *, struct bna_rx_s *);		\
		void *cbarg;						\
		cbfn = (rx)->stop_cbfn;					\
		cbarg = (rx)->stop_cbarg;				\
		(rx)->stop_cbfn = NULL;					\
		(rx)->stop_cbarg = NULL;				\
		cbfn(cbarg, rx);					\
	}								\
} while (0)

#define call_rx_stall_cbfn(rx)						\
do {									\
	if ((rx)->rx_stall_cbfn)					\
		(rx)->rx_stall_cbfn((rx)->bna->bnad, (rx));		\
} while (0)

static void bna_bfi_rx_enet_start(struct bna_rx_s *rx);
static void bna_rx_enet_stop(struct bna_rx_s *rx);
void bna_rx_mod_cb_rx_stopped(void *arg, struct bna_rx_s *rx);

uint32_t bna_dim_vector[BNA_LOAD_T_MAX][BNA_BIAS_T_MAX] = {
	{12, 20},
	{10, 18},
	{8, 16},
	{6, 12},
	{4, 8},
	{3, 6},
	{2, 4},
	{1, 2},
};

uint32_t bna_napi_dim_vector[BNA_LOAD_T_MAX][BNA_BIAS_T_MAX] = {
	{12, 12},
	{6, 10},
	{5, 10},
	{4, 8},
	{3, 6},
	{3, 6},
	{2, 4},
	{1, 2},
};

enum bna_rx_event_e {
	RX_E_START			= 1,
	RX_E_STOP			= 2,
	RX_E_FAIL			= 3,
	RX_E_STARTED			= 4,
	RX_E_STOPPED			= 5,
	RX_E_RXF_STARTED		= 6,
	RX_E_RXF_STOPPED		= 7,
	RX_E_CLEANUP_DONE		= 8,
};

bfa_fsm_state_decl(bna_rx, stopped, struct bna_rx_s, enum bna_rx_event_e);
bfa_fsm_state_decl(bna_rx, start_wait, struct bna_rx_s, enum bna_rx_event_e);
bfa_fsm_state_decl(bna_rx, start_stop_wait, struct bna_rx_s,
		enum bna_rx_event_e);
bfa_fsm_state_decl(bna_rx, rxf_start_wait, struct bna_rx_s,
	enum bna_rx_event_e);
bfa_fsm_state_decl(bna_rx, started, struct bna_rx_s, enum bna_rx_event_e);
bfa_fsm_state_decl(bna_rx, rxf_stop_wait, struct bna_rx_s,
	enum bna_rx_event_e);
bfa_fsm_state_decl(bna_rx, stop_wait, struct bna_rx_s, enum bna_rx_event_e);
bfa_fsm_state_decl(bna_rx, cleanup_wait, struct bna_rx_s,
	enum bna_rx_event_e);
bfa_fsm_state_decl(bna_rx, failed, struct bna_rx_s, enum bna_rx_event_e);
bfa_fsm_state_decl(bna_rx, quiesce_wait, struct bna_rx_s, enum bna_rx_event_e);

static void bna_rx_sm_stopped_entry(struct bna_rx_s *rx)
{
	bfa_trc(rx->bna, rx->rid);
	call_rx_stop_cbfn(rx);
}

static void bna_rx_sm_stopped(struct bna_rx_s *rx,
				enum bna_rx_event_e event)
{
	bfa_trc(rx->bna, event);
	switch (event) {
	case RX_E_START:
		bfa_fsm_set_state(rx, bna_rx_sm_start_wait);
		break;

	case RX_E_STOP:
		call_rx_stop_cbfn(rx);
		break;

	case RX_E_FAIL:
		/* no-op */
		break;

	default:
		bfa_sm_fault(rx->bna, event);
		break;
	}
}

static void bna_rx_sm_start_wait_entry(struct bna_rx_s *rx)
{
	bfa_trc(rx->bna, rx->rid);
	bna_bfi_rx_enet_start(rx);
}

static void bna_rx_sm_start_wait(struct bna_rx_s *rx,
				enum bna_rx_event_e event)
{
	bfa_trc(rx->bna, event);
	switch (event) {
	case RX_E_STOP:
		bfa_fsm_set_state(rx, bna_rx_sm_start_stop_wait);
		break;

	case RX_E_FAIL:
		bfa_fsm_set_state(rx, bna_rx_sm_stopped);
		break;

	case RX_E_STARTED:
		bfa_fsm_set_state(rx, bna_rx_sm_rxf_start_wait);
		break;

	default:
		bfa_sm_fault(rx->bna, event);
		break;
	}
}

static void bna_rx_sm_rxf_start_wait_entry(struct bna_rx_s *rx)
{
	bfa_trc(rx->bna, rx->rid);
	rx->rx_post_cbfn(rx->bna->bnad, rx);
	bna_rxf_start(&rx->rxf);
}

static void bna_rx_sm_rxf_start_wait(struct bna_rx_s *rx,
				enum bna_rx_event_e event)
{
	bfa_trc(rx->bna, event);
	switch (event) {
	case RX_E_STOP:
		bfa_fsm_set_state(rx, bna_rx_sm_rxf_stop_wait);
		break;

	case RX_E_FAIL:
		bfa_fsm_set_state(rx, bna_rx_sm_failed);
		bna_rxf_fail(&rx->rxf);
		call_rx_stall_cbfn(rx);
		rx->rx_cleanup_cbfn(rx->bna->bnad, rx);
		break;

	case RX_E_RXF_STARTED:
		bfa_fsm_set_state(rx, bna_rx_sm_started);
		break;

	default:
		bfa_sm_fault(rx->bna, event);
		break;
	}
}

void
bna_rx_sm_started_entry(struct bna_rx_s *rx)
{
	struct bna_rxp_s *rxp;
	bfa_q_t *qe_rxp;
	enum bfi_fwboot_env boot_env = bfa_ioc_boot_env(&rx->bna->ioceth.ioc);
	boolean_t is_regular = (rx->type == BNA_RX_T_REGULAR);

	bfa_trc(rx->bna, rx->rid);
	/* Start IB */
	bfa_q_iter(&rx->rxp_q, qe_rxp) {
		rxp = (struct bna_rxp_s *)qe_rxp;
		bna_ib_start(rx->bna, &rxp->cq.ib, boot_env, is_regular);
	}

	bna_ethport_cb_rx_started(&rx->bna->ethport);
}

void
bna_rx_sm_started(struct bna_rx_s *rx, enum bna_rx_event_e event)
{
	bfa_trc(rx->bna, event);
	switch (event) {
	case RX_E_STOP:
		bfa_fsm_set_state(rx, bna_rx_sm_rxf_stop_wait);
		bna_ethport_cb_rx_stopped(&rx->bna->ethport);
		bna_rxf_stop(&rx->rxf);
		break;

	case RX_E_FAIL:
		bfa_fsm_set_state(rx, bna_rx_sm_failed);
		bna_ethport_cb_rx_stopped(&rx->bna->ethport);
		bna_rxf_fail(&rx->rxf);
		call_rx_stall_cbfn(rx);
		rx->rx_cleanup_cbfn(rx->bna->bnad, rx);
		break;

	default:
		bfa_sm_fault(rx->bna, event);
		break;
	}
}

void
bna_rx_sm_rxf_stop_wait_entry(struct bna_rx_s *rx)
{
	bfa_trc(rx->bna, rx->rid);
}

void
bna_rx_sm_rxf_stop_wait(struct bna_rx_s *rx, enum bna_rx_event_e event)
{
	bfa_trc(rx->bna, event);
	switch (event) {
	case RX_E_FAIL:
		bfa_fsm_set_state(rx, bna_rx_sm_cleanup_wait);
		bna_rxf_fail(&rx->rxf);
		call_rx_stall_cbfn(rx);
		rx->rx_cleanup_cbfn(rx->bna->bnad, rx);
		break;

	case RX_E_RXF_STARTED:
		bna_rxf_stop(&rx->rxf);
		break;

	case RX_E_RXF_STOPPED:
		bfa_fsm_set_state(rx, bna_rx_sm_stop_wait);
		call_rx_stall_cbfn(rx);
		bna_rx_enet_stop(rx);
		break;

	default:
		bfa_sm_fault(rx->bna, event);
		break;
	}

}

static void
bna_rx_sm_start_stop_wait_entry(struct bna_rx_s *rx)
{
	bfa_trc(rx->bna, rx->rid);
}

static void
bna_rx_sm_start_stop_wait(struct bna_rx_s *rx, enum bna_rx_event_e event)
{
	bfa_trc(rx->bna, event);
	switch (event) {
	case RX_E_FAIL:
	case RX_E_STOPPED:
		bfa_fsm_set_state(rx, bna_rx_sm_stopped);
		break;

	case RX_E_STARTED:
		bna_rx_enet_stop(rx);
		break;

	default:
		bfa_sm_fault(rx->bna, event);
		break;
	}
}

void
bna_rx_sm_stop_wait_entry(struct bna_rx_s *rx)
{
	bfa_trc(rx->bna, rx->rid);
}

void
bna_rx_sm_stop_wait(struct bna_rx_s *rx, enum bna_rx_event_e event)
{
	bfa_trc(rx->bna, event);
	switch (event) {
	case RX_E_FAIL:
	case RX_E_STOPPED:
		bfa_fsm_set_state(rx, bna_rx_sm_cleanup_wait);
		rx->rx_cleanup_cbfn(rx->bna->bnad, rx);
		break;

	case RX_E_STARTED:
		bna_rx_enet_stop(rx);
		break;

	default:
		bfa_sm_fault(rx->bna, event);
		break;
	}
}

void
bna_rx_sm_cleanup_wait_entry(struct bna_rx_s *rx)
{
	bfa_trc(rx->bna, rx->rid);
}

void
bna_rx_sm_cleanup_wait(struct bna_rx_s *rx, enum bna_rx_event_e event)
{
	bfa_trc(rx->bna, event);
	switch (event) {
	case RX_E_FAIL:
	case RX_E_RXF_STOPPED:
		/* No-op */
		break;

	case RX_E_CLEANUP_DONE:
		bfa_fsm_set_state(rx, bna_rx_sm_stopped);
		break;

	default:
		bfa_sm_fault(rx->bna, event);
		break;
	}
}

static void
bna_rx_sm_failed_entry(struct bna_rx_s *rx)
{
	bfa_trc(rx->bna, rx->rid);
}

static void
bna_rx_sm_failed(struct bna_rx_s *rx, enum bna_rx_event_e event)
{
	bfa_trc(rx->bna, event);
	switch (event) {
	case RX_E_START:
		bfa_fsm_set_state(rx, bna_rx_sm_quiesce_wait);
		break;

	case RX_E_STOP:
		bfa_fsm_set_state(rx, bna_rx_sm_cleanup_wait);
		break;

	case RX_E_FAIL:
	case RX_E_RXF_STARTED:
	case RX_E_RXF_STOPPED:
		/* No-op */
		break;

	case RX_E_CLEANUP_DONE:
		bfa_fsm_set_state(rx, bna_rx_sm_stopped);
		break;

	default:
		bfa_sm_fault(rx->bna, event);
		break;
}	}

static void
bna_rx_sm_quiesce_wait_entry(struct bna_rx_s *rx)
{
	bfa_trc(rx->bna, rx->rid);
}

static void
bna_rx_sm_quiesce_wait(struct bna_rx_s *rx, enum bna_rx_event_e event)
{
	bfa_trc(rx->bna, event);
	switch (event) {
	case RX_E_STOP:
		bfa_fsm_set_state(rx, bna_rx_sm_cleanup_wait);
		break;

	case RX_E_FAIL:
		bfa_fsm_set_state(rx, bna_rx_sm_failed);
		break;

	case RX_E_CLEANUP_DONE:
		bfa_fsm_set_state(rx, bna_rx_sm_start_wait);
		break;

	default:
		bfa_sm_fault(rx->bna, event);
		break;
	}
}

static void
bna_bfi_rx_enet_start(struct bna_rx_s *rx)
{
	struct bfi_enet_rx_cfg_req *cfg_req = &rx->bfi_enet_cmd.cfg_req;
	struct bna_rxp_s *rxp = NULL;
	struct bna_rxq_s *q0 = NULL, *q1 = NULL;
	bfa_q_t *rxp_qe;
	int i;

	bfi_msgq_mhdr_set(cfg_req->mh, BFI_MC_ENET,
		BFI_ENET_H2I_RX_CFG_SET_REQ, 0, rx->rid);
	cfg_req->mh.num_entries = cna_os_htons(
		bfi_msgq_num_cmd_entries(sizeof(struct bfi_enet_rx_cfg_req)));

	cfg_req->rx_cfg.frame_size = bna_enet_mtu_get(&rx->bna->enet);
	cfg_req->num_queue_sets = rx->num_paths;
	for (i = 0, rxp_qe = bfa_q_first(&rx->rxp_q);
		i < rx->num_paths;
		i++, rxp_qe = bfa_q_next(rxp_qe)) {

		rxp = (struct bna_rxp_s *)rxp_qe;

		GET_RXQS(rxp, q0, q1);
		switch (rxp->type) {
		case BNA_RXP_SLR:
		case BNA_RXP_HDS:
			/* Small RxQ */
			bfi_enet_datapath_q_init(&cfg_req->q_cfg[i].qs.q,
						&q1->qpt);
			cfg_req->q_cfg[i].qs.rx_buffer_size =
				cna_os_htons((uint16_t)q1->buffer_size);
			/* Fall through */

		case BNA_RXP_SINGLE:
			/* Large/Single RxQ */
			bfi_enet_datapath_q_init(&cfg_req->q_cfg[i].ql.q,
						&q0->qpt);

			if (q0->multi_buffer) {
				/*
				 * multi-buffer is enabled by allocating
				 * a new rx with new set of resources.
				 * q0->buffer_size should be initialized to
				 * fragment size.
				 */
				cfg_req->rx_cfg.multi_buffer =
					BNA_STATUS_T_ENABLED;
			} else {
				q0->buffer_size =
					bna_enet_mtu_get(&rx->bna->enet);
			}

			cfg_req->q_cfg[i].ql.rx_buffer_size =
				cna_os_htons((uint16_t)q0->buffer_size);
			break;

		default:
			CNA_ASSERT(0);
		}

		bfi_enet_datapath_q_init(&cfg_req->q_cfg[i].cq.q,
					&rxp->cq.qpt);

		cfg_req->q_cfg[i].ib.index_addr.a32.addr_lo =
			rxp->cq.ib.ib_seg_host_addr.lsb;
		cfg_req->q_cfg[i].ib.index_addr.a32.addr_hi =
			rxp->cq.ib.ib_seg_host_addr.msb;
		cfg_req->q_cfg[i].ib.intr.msix_index =
			cna_os_htons((uint16_t)rxp->cq.ib.intr_vector);
	}

	cfg_req->ib_cfg.int_pkt_dma = BNA_STATUS_T_DISABLED;
	cfg_req->ib_cfg.int_enabled = BNA_STATUS_T_ENABLED;
	cfg_req->ib_cfg.int_pkt_enabled = BNA_STATUS_T_DISABLED;
	cfg_req->ib_cfg.continuous_coalescing = BNA_STATUS_T_DISABLED;
	cfg_req->ib_cfg.msix = (rxp->cq.ib.intr_type == BNA_INTR_T_MSIX)
				? BNA_STATUS_T_ENABLED :
				BNA_STATUS_T_DISABLED;
	cfg_req->ib_cfg.coalescing_timeout =
			cna_os_htonl((uint32_t)rxp->cq.ib.coalescing_timeo);
	cfg_req->ib_cfg.inter_pkt_timeout =
			cna_os_htonl((uint32_t)rxp->cq.ib.interpkt_timeo);
	cfg_req->ib_cfg.inter_pkt_count = (uint8_t)rxp->cq.ib.interpkt_count;

	switch (rxp->type) {
	case BNA_RXP_SLR:
		cfg_req->rx_cfg.rxq_type = BFI_ENET_RXQ_LARGE_SMALL;
		break;

	case BNA_RXP_HDS:
		cfg_req->rx_cfg.rxq_type = BFI_ENET_RXQ_HDS;
		cfg_req->rx_cfg.hds.type = rx->hds_cfg.hdr_type;
		cfg_req->rx_cfg.hds.force_offset = rx->hds_cfg.forced_offset;
		cfg_req->rx_cfg.hds.max_header_size = rx->hds_cfg.forced_offset;
		break;

	case BNA_RXP_SINGLE:
		cfg_req->rx_cfg.rxq_type = BFI_ENET_RXQ_SINGLE;
		break;

	default:
		CNA_ASSERT(0);
	}
	cfg_req->rx_cfg.strip_vlan = rx->rxf.vlan_strip_status;

	bfa_msgq_cmd_set(&rx->msgq_cmd, NULL, NULL,
		sizeof(struct bfi_enet_rx_cfg_req), &cfg_req->mh);
	bfa_msgq_cmd_post(&rx->bna->msgq, &rx->msgq_cmd);
}

static void
bna_bfi_rx_enet_stop(struct bna_rx_s *rx)
{
	struct bfi_enet_req *req = &rx->bfi_enet_cmd.req;

	bfi_msgq_mhdr_set(req->mh, BFI_MC_ENET,
		BFI_ENET_H2I_RX_CFG_CLR_REQ, 0, rx->rid);
	req->mh.num_entries = cna_os_htons(
		bfi_msgq_num_cmd_entries(sizeof(struct bfi_enet_req)));
	bfa_msgq_cmd_set(&rx->msgq_cmd, NULL, NULL, sizeof(struct bfi_enet_req),
		&req->mh);
	bfa_msgq_cmd_post(&rx->bna->msgq, &rx->msgq_cmd);
}

static void
bna_rx_enet_stop(struct bna_rx_s *rx)
{
	struct bna_rxp_s *rxp;
	bfa_q_t		 *qe_rxp;

	/* Stop IB */
	bfa_q_iter(&rx->rxp_q, qe_rxp) {
		rxp = (struct bna_rxp_s *)qe_rxp;
		bna_ib_stop(rx->bna, &rxp->cq.ib);
	}

	bna_bfi_rx_enet_stop(rx);
}

static int
bna_rx_res_check(struct bna_rx_mod_s *rx_mod, struct bna_rx_config_s *rx_cfg)
{
	if ((rx_mod->rx_free_count == 0) ||
		(rx_mod->rxp_free_count == 0) ||
		(rx_mod->rxq_free_count == 0))
		return 0;

	if (rx_cfg->rxp_type == BNA_RXP_SINGLE) {
		if ((rx_mod->rxp_free_count < rx_cfg->num_paths) ||
			(rx_mod->rxq_free_count < rx_cfg->num_paths))
				return 0;
	} else {
		if ((rx_mod->rxp_free_count < rx_cfg->num_paths) ||
			(rx_mod->rxq_free_count < (2 * rx_cfg->num_paths)))
			return 0;
	}

	return 1;
}

static struct bna_rxq_s *
bna_rxq_get(struct bna_rx_mod_s *rx_mod)
{
	struct bna_rxq_s *rxq = NULL;
	bfa_q_t	*qe = NULL;

	bfa_q_deq(&rx_mod->rxq_free_q, &qe);
	rx_mod->rxq_free_count--;
	rxq = (struct bna_rxq_s *)qe;
	bfa_q_qe_init(&rxq->qe);

	return rxq;
}

static void
bna_rxq_put(struct bna_rx_mod_s *rx_mod, struct bna_rxq_s *rxq)
{
	bfa_q_qe_init(&rxq->qe);
	bfa_q_enq(&rx_mod->rxq_free_q, &rxq->qe);
	rx_mod->rxq_free_count++;
}

static struct bna_rxp_s *
bna_rxp_get(struct bna_rx_mod_s *rx_mod)
{
	bfa_q_t	*qe = NULL;
	struct bna_rxp_s *rxp = NULL;

	bfa_q_deq(&rx_mod->rxp_free_q, &qe);
	rx_mod->rxp_free_count--;
	rxp = (struct bna_rxp_s *)qe;
	bfa_q_qe_init(&rxp->qe);

	return rxp;
}

static void
bna_rxp_put(struct bna_rx_mod_s *rx_mod, struct bna_rxp_s *rxp)
{
	bfa_q_qe_init(&rxp->qe);
	bfa_q_enq(&rx_mod->rxp_free_q, &rxp->qe);
	rx_mod->rxp_free_count++;
}

static struct bna_rx_s *
bna_rx_get(struct bna_rx_mod_s *rx_mod, enum bna_rx_type_e type)
{
	bfa_q_t	*qe = NULL;
	struct bna_rx_s *rx = NULL;

	CNA_ASSERT((type == BNA_RX_T_REGULAR) || (type == BNA_RX_T_LOOPBACK));
	if (type == BNA_RX_T_REGULAR) {
		bfa_q_deq(&rx_mod->rx_free_q, &qe);
	} else {
		bfa_q_deq_tail(&rx_mod->rx_free_q, &qe);
	}
	rx_mod->rx_free_count--;
	rx = (struct bna_rx_s *)qe;
	bfa_q_qe_init(&rx->qe);
	bfa_q_enq(&rx_mod->rx_active_q, &rx->qe);
	rx->type = type;

	return rx;
}

static void
bna_rx_put(struct bna_rx_mod_s *rx_mod, struct bna_rx_s *rx)
{
	bfa_q_t *prev_qe = NULL;
	bfa_q_t *qe;

	bfa_trc(rx_mod->bna, rx->rid);
	bfa_q_qe_init(&rx->qe);

	bfa_q_iter(&rx_mod->rx_free_q, qe) {
		if (((struct bna_rx_s *)qe)->rid < rx->rid)
			prev_qe = qe;
		else {
			CNA_ASSERT(((struct bna_rx_s *)qe)->rid > rx->rid);
			break;
		}
	}

	if (prev_qe == NULL) {
		/* This is the first entry */
		bfa_q_enq_head(&rx_mod->rx_free_q, &rx->qe);
	} else if (bfa_q_next(prev_qe) == &rx_mod->rx_free_q) {
		/* This is the last entry */
		bfa_q_enq(&rx_mod->rx_free_q, &rx->qe);
	} else {
		/* Somewhere in the middle */
		bfa_q_next(&rx->qe) = bfa_q_next(prev_qe);
		bfa_q_prev(&rx->qe) = prev_qe;
		bfa_q_next(prev_qe) = &rx->qe;
		bfa_q_prev(bfa_q_next(&rx->qe)) = &rx->qe;
	}

	rx_mod->rx_free_count++;
}

static void
bna_rxp_add_rxqs(struct bna_rxp_s *rxp, struct bna_rxq_s *q0,
		struct bna_rxq_s *q1)
{
	switch (rxp->type) {
	case BNA_RXP_SINGLE:
		CNA_ASSERT(q1 == NULL);
		rxp->rxq.single.only = q0;
		rxp->rxq.single.reserved = NULL;
		break;
	case BNA_RXP_SLR:
		rxp->rxq.slr.large = q0;
		rxp->rxq.slr.small = q1;
		break;
	case BNA_RXP_HDS:
		rxp->rxq.hds.data = q0;
		rxp->rxq.hds.hdr = q1;
		break;
	default:
		CNA_ASSERT(0);
		break;
	}
}

static struct bna_rxp_s *
bna_rx_get_rxp(struct bna_rx_s *rx, int vector)
{
	struct bna_rxp_s *rxp;
	bfa_q_t *qe;

	bfa_q_iter(&rx->rxp_q, qe) {
		rxp = (struct bna_rxp_s *)qe;
		if (rxp->vector == vector)
			return rxp;
	}
	return NULL;
}

static void
bna_rxq_qpt_setup(struct bna_rxq_s *rxq,
		struct bna_rxp_s *rxp,
		uint32_t page_count,
		uint32_t page_size,
		struct bna_mem_descr_s *qpt_mem,
		struct bna_mem_descr_s *swqpt_mem,
		struct bna_mem_descr_s *page_mem)
{
	uint8_t *kva;
	uint64_t dma;
	struct bna_dma_addr_s bna_dma;
	int	i;

	rxq->qpt.hw_qpt_ptr.lsb = qpt_mem->dma.lsb;
	rxq->qpt.hw_qpt_ptr.msb = qpt_mem->dma.msb;
	rxq->qpt.kv_qpt_ptr = qpt_mem->kva;
	rxq->qpt.page_count = page_count;
	rxq->qpt.page_size = page_size;

	bfa_trc(rxp->rx->bna, rxq->qpt.hw_qpt_ptr.lsb);
	bfa_trc(rxp->rx->bna, rxq->qpt.hw_qpt_ptr.msb);
	bfa_trc(rxp->rx->bna, rxq->qpt.page_count);
	bfa_trc(rxp->rx->bna, rxq->qpt.page_size);

	rxq->rcb->sw_qpt = (void **) swqpt_mem->kva;
	rxq->rcb->sw_q = page_mem->kva;

	kva = page_mem->kva;
	BNA_GET_DMA_ADDR(&page_mem->dma, dma);

	for (i = 0; i < rxq->qpt.page_count; i++) {
		rxq->rcb->sw_qpt[i] = kva;
		kva += CNA_PAGE_SIZE;

		BNA_SET_DMA_ADDR(dma, &bna_dma);
		((struct bna_dma_addr_s *)rxq->qpt.kv_qpt_ptr)[i].lsb =
			bna_dma.lsb;
		((struct bna_dma_addr_s *)rxq->qpt.kv_qpt_ptr)[i].msb =
			bna_dma.msb;
		dma += CNA_PAGE_SIZE;

		bfa_trc(rxp->rx->bna,
			((struct bna_dma_addr_s *)rxq->qpt.kv_qpt_ptr)[i].lsb);
		bfa_trc(rxp->rx->bna,
			((struct bna_dma_addr_s *)rxq->qpt.kv_qpt_ptr)[i].msb);
	}
}

static void
bna_rxp_cqpt_setup(struct bna_rxp_s *rxp,
		uint32_t page_count,
		uint32_t page_size,
		struct bna_mem_descr_s *qpt_mem,
		struct bna_mem_descr_s *swqpt_mem,
		struct bna_mem_descr_s *page_mem)
{
	uint8_t *kva;
	uint64_t dma;
	struct bna_dma_addr_s bna_dma;
	int	i;

	rxp->cq.qpt.hw_qpt_ptr.lsb = qpt_mem->dma.lsb;
	rxp->cq.qpt.hw_qpt_ptr.msb = qpt_mem->dma.msb;
	rxp->cq.qpt.kv_qpt_ptr = qpt_mem->kva;
	rxp->cq.qpt.page_count = page_count;
	rxp->cq.qpt.page_size = page_size;

	bfa_trc(rxp->rx->bna, rxp->cq.qpt.hw_qpt_ptr.lsb);
	bfa_trc(rxp->rx->bna, rxp->cq.qpt.hw_qpt_ptr.msb);
	bfa_trc(rxp->rx->bna, rxp->cq.qpt.page_count);
	bfa_trc(rxp->rx->bna, rxp->cq.qpt.page_size);

	rxp->cq.ccb->sw_qpt = (void **) swqpt_mem->kva;
	rxp->cq.ccb->sw_q = page_mem->kva;

	kva = page_mem->kva;
	BNA_GET_DMA_ADDR(&page_mem->dma, dma);

	for (i = 0; i < rxp->cq.qpt.page_count; i++) {
		rxp->cq.ccb->sw_qpt[i] = kva;
		kva += CNA_PAGE_SIZE;

		BNA_SET_DMA_ADDR(dma, &bna_dma);
		((struct bna_dma_addr_s *)rxp->cq.qpt.kv_qpt_ptr)[i].lsb =
			bna_dma.lsb;
		((struct bna_dma_addr_s *)rxp->cq.qpt.kv_qpt_ptr)[i].msb =
			bna_dma.msb;
		dma += CNA_PAGE_SIZE;

		bfa_trc(rxp->rx->bna,
			((struct bna_dma_addr_s *)rxp->cq.qpt.kv_qpt_ptr)
				[i].lsb);
		bfa_trc(rxp->rx->bna,
			((struct bna_dma_addr_s *)rxp->cq.qpt.kv_qpt_ptr)
				[i].msb);
	}
}

void
bna_bfi_rx_enet_start_rsp(struct bna_rx_s *rx, struct bfi_msgq_mhdr_s *msghdr)
{
	struct bfi_enet_rx_cfg_rsp *cfg_rsp = &rx->bfi_enet_cmd.cfg_rsp;
	struct bna_rxp_s *rxp = NULL;
	struct bna_rxq_s *q0 = NULL, *q1 = NULL;
	bfa_q_t *rxp_qe;
	int i;

	bfa_msgq_rsp_copy(&rx->bna->msgq, (uint8_t *)cfg_rsp,
		sizeof(struct bfi_enet_rx_cfg_rsp));

	CNA_ASSERT(cfg_rsp->error == 0);

	rx->hw_id = cfg_rsp->hw_id;

	for (i = 0, rxp_qe = bfa_q_first(&rx->rxp_q);
		i < rx->num_paths;
		i++, rxp_qe = bfa_q_next(rxp_qe)) {

		rxp = (struct bna_rxp_s *)rxp_qe;
		GET_RXQS(rxp, q0, q1);

		/* Setup doorbells */
		rxp->cq.ccb->i_dbell->doorbell_addr =
			rx->bna->pcidev.pci_bar_kva
			+ cna_os_ntohl(cfg_rsp->q_handles[i].i_dbell);
		bfa_trc(rx->bna, cna_os_ntohl(cfg_rsp->q_handles[i].i_dbell));
		rxp->hw_id = cfg_rsp->q_handles[i].hw_cqid;
		q0->rcb->q_dbell =
			rx->bna->pcidev.pci_bar_kva
			+ cna_os_ntohl(cfg_rsp->q_handles[i].ql_dbell);
		bfa_trc(rx->bna, cna_os_ntohl(cfg_rsp->q_handles[i].ql_dbell));
		q0->hw_id = cfg_rsp->q_handles[i].hw_lqid;
		if (q1) {
			q1->rcb->q_dbell =
			rx->bna->pcidev.pci_bar_kva
			+ cna_os_ntohl(cfg_rsp->q_handles[i].qs_dbell);
			bfa_trc(rx->bna,
				cna_os_ntohl(cfg_rsp->q_handles[i].qs_dbell));
			q1->hw_id = cfg_rsp->q_handles[i].hw_sqid;
		}

		/* Initialize producer/consumer indexes */
		(*rxp->cq.ccb->hw_producer_index) = 0;
		rxp->cq.ccb->producer_index = 0;
		q0->rcb->producer_index = q0->rcb->consumer_index = 0;
		if (q1)
			q1->rcb->producer_index = q1->rcb->consumer_index = 0;

		rxp->cq.ccb->pkts_una = 0;
		rxp->cq.ccb->bytes_per_intr = 0;
	}

	bfa_fsm_send_event(rx, RX_E_STARTED);
}

void
bna_bfi_rx_enet_stop_rsp(struct bna_rx_s *rx, struct bfi_msgq_mhdr_s *msghdr)
{
	struct bfi_enet_rsp *rsp = (struct bfi_enet_rsp *)msghdr;

	CNA_ASSERT(rsp->error == 0);
	bfa_fsm_send_event(rx, RX_E_STOPPED);
}

void
bna_rx_start(struct bna_rx_s *rx)
{
	rx->rx_flags |= BNA_RX_F_ENET_STARTED;
	if (rx->rx_flags & BNA_RX_F_ENABLED)
		bfa_fsm_send_event(rx, RX_E_START);
}

void
bna_rx_stop(struct bna_rx_s *rx)
{
	rx->rx_flags &= ~BNA_RX_F_ENET_STARTED;
	if (rx->fsm == (bfa_fsm_t) bna_rx_sm_stopped) {
		bna_rx_mod_cb_rx_stopped(&rx->bna->rx_mod, rx);
	} else {
		rx->stop_cbfn = bna_rx_mod_cb_rx_stopped;
		rx->stop_cbarg = &rx->bna->rx_mod;
		bfa_fsm_send_event(rx, RX_E_STOP);
	}
}

void
bna_rx_fail(struct bna_rx_s *rx)
{
	/* Indicate Enet is not enabled, and failed */
	rx->rx_flags &= ~BNA_RX_F_ENET_STARTED;
	bfa_fsm_send_event(rx, RX_E_FAIL);
}

void
bna_rx_cb_rxf_started(struct bna_rx_s *rx)
{
	bfa_fsm_send_event(rx, RX_E_RXF_STARTED);
}

void
bna_rx_cb_rxf_stopped(struct bna_rx_s *rx)
{
	bfa_fsm_send_event(rx, RX_E_RXF_STOPPED);
}

void
bna_rx_res_req(struct bna_rx_config_s *q_cfg, struct bna_res_info_s *res_info)
{
	uint32_t cq_size, hq_size, dq_size;
	uint32_t cpage_count, hpage_count, dpage_count;
	struct bna_mem_info_s *mem_info;
	uint32_t cq_depth;
	uint32_t hq_depth;
	uint32_t dq_depth;

	dq_depth = q_cfg->q0_depth;
	hq_depth = ((q_cfg->rxp_type == BNA_RXP_SINGLE) ? 0 : q_cfg->q1_depth);
	cq_depth = dq_depth + hq_depth;

	BNA_TO_POWER_OF_2_HIGH(cq_depth);
	cq_size = cq_depth * BFI_CQ_WI_SIZE;
	cq_size = CNA_ALIGN(cq_size, CNA_PAGE_SIZE);
	cpage_count = SIZE_TO_PAGES(cq_size);

	BNA_TO_POWER_OF_2_HIGH(dq_depth);
	dq_size = dq_depth * BFI_RXQ_WI_SIZE;
	dq_size = CNA_ALIGN(dq_size, CNA_PAGE_SIZE);
	dpage_count = SIZE_TO_PAGES(dq_size);

	if (BNA_RXP_SINGLE != q_cfg->rxp_type) {
		BNA_TO_POWER_OF_2_HIGH(hq_depth);
		hq_size = hq_depth * BFI_RXQ_WI_SIZE;
		hq_size = CNA_ALIGN(hq_size, CNA_PAGE_SIZE);
		hpage_count = SIZE_TO_PAGES(hq_size);
	} else {
		hpage_count = 0;
	}

	res_info[BNA_RX_RES_MEM_T_CCB].res_type = BNA_RES_T_MEM;
	mem_info = &res_info[BNA_RX_RES_MEM_T_CCB].res_u.mem_info;
	mem_info->mem_type = BNA_MEM_T_KVA;
	mem_info->len = sizeof(struct bna_ccb_s);
	mem_info->num = q_cfg->num_paths;

	res_info[BNA_RX_RES_MEM_T_RCB].res_type = BNA_RES_T_MEM;
	mem_info = &res_info[BNA_RX_RES_MEM_T_RCB].res_u.mem_info;
	mem_info->mem_type = BNA_MEM_T_KVA;
	mem_info->len = sizeof(struct bna_rcb_s);
	mem_info->num = BNA_GET_RXQS(q_cfg);

	res_info[BNA_RX_RES_MEM_T_CQPT].res_type = BNA_RES_T_MEM;
	mem_info = &res_info[BNA_RX_RES_MEM_T_CQPT].res_u.mem_info;
	mem_info->mem_type = BNA_MEM_T_DMA;
	mem_info->len = cpage_count * sizeof(struct bna_dma_addr_s);
	mem_info->num = q_cfg->num_paths;

	res_info[BNA_RX_RES_MEM_T_CSWQPT].res_type = BNA_RES_T_MEM;
	mem_info = &res_info[BNA_RX_RES_MEM_T_CSWQPT].res_u.mem_info;
	mem_info->mem_type = BNA_MEM_T_KVA;
	mem_info->len = cpage_count * sizeof(void *);
	mem_info->num = q_cfg->num_paths;

	res_info[BNA_RX_RES_MEM_T_CQPT_PAGE].res_type = BNA_RES_T_MEM;
	mem_info = &res_info[BNA_RX_RES_MEM_T_CQPT_PAGE].res_u.mem_info;
	mem_info->mem_type = BNA_MEM_T_DMA;
	mem_info->len = CNA_PAGE_SIZE * cpage_count;
	mem_info->num = q_cfg->num_paths;

	res_info[BNA_RX_RES_MEM_T_DQPT].res_type = BNA_RES_T_MEM;
	mem_info = &res_info[BNA_RX_RES_MEM_T_DQPT].res_u.mem_info;
	mem_info->mem_type = BNA_MEM_T_DMA;
	mem_info->len = dpage_count * sizeof(struct bna_dma_addr_s);
	mem_info->num = q_cfg->num_paths;

	res_info[BNA_RX_RES_MEM_T_DSWQPT].res_type = BNA_RES_T_MEM;
	mem_info = &res_info[BNA_RX_RES_MEM_T_DSWQPT].res_u.mem_info;
	mem_info->mem_type = BNA_MEM_T_KVA;
	mem_info->len = dpage_count * sizeof(void *);
	mem_info->num = q_cfg->num_paths;

	res_info[BNA_RX_RES_MEM_T_DPAGE].res_type = BNA_RES_T_MEM;
	mem_info = &res_info[BNA_RX_RES_MEM_T_DPAGE].res_u.mem_info;
	mem_info->mem_type = BNA_MEM_T_DMA;
	mem_info->len = CNA_PAGE_SIZE * dpage_count;
	mem_info->num = q_cfg->num_paths;

	res_info[BNA_RX_RES_MEM_T_HQPT].res_type = BNA_RES_T_MEM;
	mem_info = &res_info[BNA_RX_RES_MEM_T_HQPT].res_u.mem_info;
	mem_info->mem_type = BNA_MEM_T_DMA;
	mem_info->len = hpage_count * sizeof(struct bna_dma_addr_s);
	mem_info->num = (hpage_count ? q_cfg->num_paths : 0);

	res_info[BNA_RX_RES_MEM_T_HSWQPT].res_type = BNA_RES_T_MEM;
	mem_info = &res_info[BNA_RX_RES_MEM_T_HSWQPT].res_u.mem_info;
	mem_info->mem_type = BNA_MEM_T_KVA;
	mem_info->len = hpage_count * sizeof(void *);
	mem_info->num = (hpage_count ? q_cfg->num_paths : 0);

	res_info[BNA_RX_RES_MEM_T_HPAGE].res_type = BNA_RES_T_MEM;
	mem_info = &res_info[BNA_RX_RES_MEM_T_HPAGE].res_u.mem_info;
	mem_info->mem_type = BNA_MEM_T_DMA;
	mem_info->len = CNA_PAGE_SIZE * hpage_count;
	mem_info->num = (hpage_count ? q_cfg->num_paths : 0);

	res_info[BNA_RX_RES_MEM_T_IBIDX].res_type = BNA_RES_T_MEM;
	mem_info = &res_info[BNA_RX_RES_MEM_T_IBIDX].res_u.mem_info;
	mem_info->mem_type = BNA_MEM_T_DMA;
	mem_info->len = BFI_IBIDX_SIZE;
	mem_info->num = q_cfg->num_paths;

	res_info[BNA_RX_RES_MEM_T_RIT].res_type = BNA_RES_T_MEM;
	mem_info = &res_info[BNA_RX_RES_MEM_T_RIT].res_u.mem_info;
	mem_info->mem_type = BNA_MEM_T_KVA;
	mem_info->len = BFI_ENET_RSS_RIT_MAX;
	mem_info->num = 1;

	res_info[BNA_RX_RES_T_INTR].res_type = BNA_RES_T_INTR;
	res_info[BNA_RX_RES_T_INTR].res_u.intr_info.intr_type = BNA_INTR_T_MSIX;
	res_info[BNA_RX_RES_T_INTR].res_u.intr_info.num = q_cfg->num_paths;
}

struct bna_rx_s *
bna_rx_create(struct bna_s *bna, struct bnad_s *bnad,
		struct bna_rx_config_s *rx_cfg,
		struct bna_rx_event_cbfn_s *rx_cbfn,
		struct bna_res_info_s *res_info,
		void *priv)
{
	struct bna_rx_mod_s *rx_mod = &bna->rx_mod;
	struct bna_rx_s *rx;
	struct bna_rxp_s *rxp;
	struct bna_rxq_s *q0;
	struct bna_rxq_s *q1;
	struct bna_intr_info_s *intr_info;
	struct bna_mem_descr_s *hqunmap_mem;
	struct bna_mem_descr_s *dqunmap_mem;
	struct bna_mem_descr_s *ccb_mem;
	struct bna_mem_descr_s *rcb_mem;
	struct bna_mem_descr_s *cqpt_mem;
	struct bna_mem_descr_s *cswqpt_mem;
	struct bna_mem_descr_s *cpage_mem;
	struct bna_mem_descr_s *hqpt_mem;
	struct bna_mem_descr_s *dqpt_mem;
	struct bna_mem_descr_s *hsqpt_mem;
	struct bna_mem_descr_s *dsqpt_mem;
	struct bna_mem_descr_s *hpage_mem;
	struct bna_mem_descr_s *dpage_mem;
	uint32_t dpage_count, hpage_count;
	uint32_t hq_idx, dq_idx, rcb_idx;
	uint32_t cq_depth, i;
	uint32_t page_count;

	if (!bna_rx_res_check(rx_mod, rx_cfg))
		return NULL;

	intr_info = &res_info[BNA_RX_RES_T_INTR].res_u.intr_info;
	ccb_mem = &res_info[BNA_RX_RES_MEM_T_CCB].res_u.mem_info.mdl[0];
	rcb_mem = &res_info[BNA_RX_RES_MEM_T_RCB].res_u.mem_info.mdl[0];
	dqunmap_mem = &res_info[BNA_RX_RES_MEM_T_UNMAPDQ].res_u.mem_info.mdl[0];
	hqunmap_mem = &res_info[BNA_RX_RES_MEM_T_UNMAPHQ].res_u.mem_info.mdl[0];
	cqpt_mem = &res_info[BNA_RX_RES_MEM_T_CQPT].res_u.mem_info.mdl[0];
	cswqpt_mem = &res_info[BNA_RX_RES_MEM_T_CSWQPT].res_u.mem_info.mdl[0];
	cpage_mem = &res_info[BNA_RX_RES_MEM_T_CQPT_PAGE].res_u.mem_info.mdl[0];
	hqpt_mem = &res_info[BNA_RX_RES_MEM_T_HQPT].res_u.mem_info.mdl[0];
	dqpt_mem = &res_info[BNA_RX_RES_MEM_T_DQPT].res_u.mem_info.mdl[0];
	hsqpt_mem = &res_info[BNA_RX_RES_MEM_T_HSWQPT].res_u.mem_info.mdl[0];
	dsqpt_mem = &res_info[BNA_RX_RES_MEM_T_DSWQPT].res_u.mem_info.mdl[0];
	hpage_mem = &res_info[BNA_RX_RES_MEM_T_HPAGE].res_u.mem_info.mdl[0];
	dpage_mem = &res_info[BNA_RX_RES_MEM_T_DPAGE].res_u.mem_info.mdl[0];

	page_count = res_info[BNA_RX_RES_MEM_T_CQPT_PAGE].res_u.mem_info.len /
			CNA_PAGE_SIZE;

	dpage_count = res_info[BNA_RX_RES_MEM_T_DPAGE].res_u.mem_info.len /
			CNA_PAGE_SIZE;

	hpage_count = res_info[BNA_RX_RES_MEM_T_HPAGE].res_u.mem_info.len /
			CNA_PAGE_SIZE;

	rx = bna_rx_get(rx_mod, rx_cfg->rx_type);
	rx->bna = bna;
	rx->rx_flags = 0;
	bfa_q_init(&rx->rxp_q);
	rx->stop_cbfn = NULL;
	rx->stop_cbarg = NULL;
	rx->priv = priv;

	rx->rcb_setup_cbfn = rx_cbfn->rcb_setup_cbfn;
	rx->rcb_destroy_cbfn = rx_cbfn->rcb_destroy_cbfn;
	rx->ccb_setup_cbfn = rx_cbfn->ccb_setup_cbfn;
	rx->ccb_destroy_cbfn = rx_cbfn->ccb_destroy_cbfn;
	rx->rx_stall_cbfn = rx_cbfn->rx_stall_cbfn;
	/* Following callbacks are mandatory */
	rx->rx_cleanup_cbfn = rx_cbfn->rx_cleanup_cbfn;
	rx->rx_post_cbfn = rx_cbfn->rx_post_cbfn;
	CNA_ASSERT(rx->rx_cleanup_cbfn);
	CNA_ASSERT(rx->rx_post_cbfn);

	if (rx->bna->rx_mod.flags & BNA_RX_MOD_F_ENET_STARTED) {
		switch (rx->type) {
		case BNA_RX_T_REGULAR:
			if (!(rx->bna->rx_mod.flags &
				BNA_RX_MOD_F_ENET_LOOPBACK))
				rx->rx_flags |= BNA_RX_F_ENET_STARTED;
			break;
		case BNA_RX_T_LOOPBACK:
			if (rx->bna->rx_mod.flags & BNA_RX_MOD_F_ENET_LOOPBACK)
				rx->rx_flags |= BNA_RX_F_ENET_STARTED;
			break;
		}
	}

	rx->num_paths = rx_cfg->num_paths;
	for (i = 0, hq_idx = 0, dq_idx = 0, rcb_idx = 0;
			i < rx->num_paths; i++) {
		rxp = bna_rxp_get(rx_mod);
		bfa_q_enq(&rx->rxp_q, &rxp->qe);
		rxp->type = rx_cfg->rxp_type;
		rxp->rx = rx;
		rxp->cq.rx = rx;

		q0 = bna_rxq_get(rx_mod);
		if (BNA_RXP_SINGLE == rx_cfg->rxp_type)
			q1 = NULL;
		else
			q1 = bna_rxq_get(rx_mod);

		if (1 == intr_info->num)
			rxp->vector = intr_info->idl[0].vector;
		else
			rxp->vector = intr_info->idl[i].vector;

		/* Setup IB */

		rxp->cq.ib.ib_seg_host_addr.lsb =
		res_info[BNA_RX_RES_MEM_T_IBIDX].res_u.mem_info.mdl[i].dma.lsb;
		rxp->cq.ib.ib_seg_host_addr.msb =
		res_info[BNA_RX_RES_MEM_T_IBIDX].res_u.mem_info.mdl[i].dma.msb;
		rxp->cq.ib.ib_seg_host_addr_kva =
		res_info[BNA_RX_RES_MEM_T_IBIDX].res_u.mem_info.mdl[i].kva;
		rxp->cq.ib.intr_type = intr_info->intr_type;
		if (intr_info->intr_type == BNA_INTR_T_MSIX)
			rxp->cq.ib.intr_vector = rxp->vector;
		else
			rxp->cq.ib.intr_vector = (1 << rxp->vector);
		rxp->cq.ib.coalescing_timeo = rx_cfg->coalescing_timeo;
		rxp->cq.ib.interpkt_count = BFI_RX_INTERPKT_COUNT;
		rxp->cq.ib.interpkt_timeo = BFI_RX_INTERPKT_TIMEO;

		bna_rxp_add_rxqs(rxp, q0, q1);

		/* Setup large Q */

		q0->rx = rx;
		q0->rxp = rxp;

		q0->rcb = (struct bna_rcb_s *) rcb_mem[rcb_idx].kva;
		q0->rcb->unmap_q = (void *)dqunmap_mem[dq_idx].kva;
		rcb_idx++; dq_idx++;
		q0->rcb->q_depth = rx_cfg->q0_depth;
		q0->q_depth = rx_cfg->q0_depth;
		q0->multi_buffer = rx_cfg->q0_multi_buf;
		q0->buffer_size = rx_cfg->q0_buf_size;
		q0->num_vecs = rx_cfg->q0_num_vecs;
		q0->rcb->rxq = q0;
		q0->rcb->bnad = bna->bnad;
		q0->rcb->id = 0;
		q0->rx_packets = q0->rx_bytes = 0;
		q0->rx_packets_with_error = q0->rxbuf_alloc_failed = 0;

		bna_rxq_qpt_setup(q0, rxp, dpage_count, CNA_PAGE_SIZE,
			&dqpt_mem[i], &dsqpt_mem[i], &dpage_mem[i]);

		if (rx->rcb_setup_cbfn)
			rx->rcb_setup_cbfn(bnad, q0->rcb);

		/* Setup small Q */

		if (q1) {
			q1->rx = rx;
			q1->rxp = rxp;

			q1->rcb = (struct bna_rcb_s *) rcb_mem[rcb_idx].kva;
			q1->rcb->unmap_q = (void *)hqunmap_mem[hq_idx].kva;
			rcb_idx++; hq_idx++;
			q1->rcb->q_depth = rx_cfg->q1_depth;
			q1->q_depth = rx_cfg->q1_depth;
			q1->multi_buffer = BNA_STATUS_T_DISABLED;
			q1->num_vecs = 1;
			q1->rcb->rxq = q1;
			q1->rcb->bnad = bna->bnad;
			q1->rcb->id = 1;
			q1->buffer_size = (rx_cfg->rxp_type == BNA_RXP_HDS) ?
					rx_cfg->hds_config.forced_offset
					: rx_cfg->q1_buf_size;
			q1->rx_packets = q1->rx_bytes = 0;
			q1->rx_packets_with_error = q1->rxbuf_alloc_failed = 0;

			bna_rxq_qpt_setup(q1, rxp, hpage_count, CNA_PAGE_SIZE,
				&hqpt_mem[i], &hsqpt_mem[i],
				&hpage_mem[i]);

			if (rx->rcb_setup_cbfn)
				rx->rcb_setup_cbfn(bnad, q1->rcb);
		}

		/* Setup CQ */

		rxp->cq.ccb = (struct bna_ccb_s *) ccb_mem[i].kva;
		cq_depth = rx_cfg->q0_depth +
			((rx_cfg->rxp_type == BNA_RXP_SINGLE) ?
			 0 : rx_cfg->q1_depth);
		/*
		 * if multi-buffer is enabled sum of q0_depth
		 * and q1_depth need not be a power of 2
		 */
		BNA_TO_POWER_OF_2_HIGH(cq_depth);
		rxp->cq.ccb->q_depth = cq_depth;
		rxp->cq.ccb->cq = &rxp->cq;
		rxp->cq.ccb->rcb[0] = q0->rcb;
		q0->rcb->ccb = rxp->cq.ccb;
		if (q1) {
			rxp->cq.ccb->rcb[1] = q1->rcb;
			q1->rcb->ccb = rxp->cq.ccb;
		}
		rxp->cq.ccb->hw_producer_index =
			(uint32_t *)rxp->cq.ib.ib_seg_host_addr_kva;
		rxp->cq.ccb->i_dbell = &rxp->cq.ib.door_bell;
		rxp->cq.ccb->intr_type = rxp->cq.ib.intr_type;
		rxp->cq.ccb->intr_vector = rxp->cq.ib.intr_vector;
		rxp->cq.ccb->rx_coalescing_timeo =
			rxp->cq.ib.coalescing_timeo;
		rxp->cq.ccb->pkt_rate.small_pkt_cnt = 0;
		rxp->cq.ccb->pkt_rate.large_pkt_cnt = 0;
		rxp->cq.ccb->bnad = bna->bnad;
		rxp->cq.ccb->id = i;

		bna_rxp_cqpt_setup(rxp, page_count, CNA_PAGE_SIZE,
			&cqpt_mem[i], &cswqpt_mem[i], &cpage_mem[i]);

		if (rx->ccb_setup_cbfn)
			rx->ccb_setup_cbfn(bnad, rxp->cq.ccb);
	}

	bfa_os_assign(rx->hds_cfg, rx_cfg->hds_config);

	bna_rxf_init(&rx->rxf, rx, rx_cfg, res_info);

	bfa_fsm_set_state(rx, bna_rx_sm_stopped);

	rx_mod->rid_mask |= (1 << rx->rid);

	return rx;
}

void
bna_rx_destroy(struct bna_rx_s *rx)
{
	struct bna_rx_mod_s *rx_mod = &rx->bna->rx_mod;
	struct bna_rxq_s *q0 = NULL;
	struct bna_rxq_s *q1 = NULL;
	struct bna_rxp_s *rxp;
	bfa_q_t *qe;

	bna_rxf_uninit(&rx->rxf);

	while (!bfa_q_is_empty(&rx->rxp_q)) {
		bfa_q_deq(&rx->rxp_q, &rxp);
		GET_RXQS(rxp, q0, q1);
		if (rx->rcb_destroy_cbfn)
			rx->rcb_destroy_cbfn(rx->bna->bnad, q0->rcb);
		q0->rcb = NULL;
		q0->rxp = NULL;
		q0->rx = NULL;
		bna_rxq_put(rx_mod, q0);

		if (q1) {
			if (rx->rcb_destroy_cbfn)
				rx->rcb_destroy_cbfn(rx->bna->bnad, q1->rcb);
			q1->rcb = NULL;
			q1->rxp = NULL;
			q1->rx = NULL;
			bna_rxq_put(rx_mod, q1);
		}
		rxp->rxq.slr.large = NULL;
		rxp->rxq.slr.small = NULL;

		if (rx->ccb_destroy_cbfn)
			rx->ccb_destroy_cbfn(rx->bna->bnad, rxp->cq.ccb);
		rxp->cq.ccb = NULL;
		rxp->rx = NULL;
		bna_rxp_put(rx_mod, rxp);
	}

	bfa_q_iter(&rx_mod->rx_active_q, qe) {
		if (qe == &rx->qe) {
			bfa_q_qe_deq(&rx->qe);
			bfa_q_qe_init(&rx->qe);
			break;
		}
	}

	rx_mod->rid_mask &= ~(1 << rx->rid);

	rx->bna = NULL;
	rx->priv = NULL;
	bna_rx_put(rx_mod, rx);
}

void
bna_rx_enable(struct bna_rx_s *rx)
{
	bfa_trc(rx->bna, rx->rx_flags);

	if (rx->fsm != (bfa_sm_t)bna_rx_sm_stopped)
		return;

	rx->rx_flags |= BNA_RX_F_ENABLED;
	if (rx->rx_flags & BNA_RX_F_ENET_STARTED)
		bfa_fsm_send_event(rx, RX_E_START);
}

void
bna_rx_disable(struct bna_rx_s *rx, enum bna_cleanup_type_e type,
		void (*cbfn)(void *, struct bna_rx_s *))
{
	CNA_ASSERT(cbfn);

	if (type == BNA_SOFT_CLEANUP) {
		/* h/w should not be accessed. Treat we're stopped */
		(*cbfn)(rx->bna->bnad, rx);
	} else {
		CNA_ASSERT(rx->stop_cbfn == NULL);
		rx->stop_cbfn = cbfn;
		rx->stop_cbarg = rx->bna->bnad;

		rx->rx_flags &= ~BNA_RX_F_ENABLED;

		bfa_fsm_send_event(rx, RX_E_STOP);
	}
}

void
bna_rx_cleanup_complete(struct bna_rx_s *rx)
{
	bfa_fsm_send_event(rx, RX_E_CLEANUP_DONE);
}

void
bna_rx_coalescing_timeo_set(struct bna_rx_s *rx, int coalescing_timeo)
{
	struct bna_rxp_s *rxp;
	bfa_q_t *qe;

	bfa_q_iter(&rx->rxp_q, qe) {
		rxp = (struct bna_rxp_s *)qe;
		rxp->cq.ccb->rx_coalescing_timeo = coalescing_timeo;
		bna_ib_coalescing_timeo_set(&rxp->cq.ib, coalescing_timeo);
	}
}

void
bna_rx_dim_reconfig(struct bna_s *bna, uint32_t vector[][BNA_BIAS_T_MAX])
{
	int i, j;

	for (i = 0; i < BNA_LOAD_T_MAX; i++)
		for (j = 0; j < BNA_BIAS_T_MAX; j++)
			bna->rx_mod.dim_vector[i][j] = vector[i][j];
}

void
bna_rx_dim_update(struct bna_ccb_s *ccb)
{
	struct bna_s *bna = ccb->cq->rx->bna;
	uint32_t load, bias;
	uint32_t pkt_rt, small_rt, large_rt;
	uint8_t coalescing_timeo;

	if ((ccb->pkt_rate.small_pkt_cnt == 0) &&
		(ccb->pkt_rate.large_pkt_cnt == 0))
		return;

	/* Arrive at preconfigured coalescing timeo value based on pkt rate */

	small_rt = ccb->pkt_rate.small_pkt_cnt;
	large_rt = ccb->pkt_rate.large_pkt_cnt;

	pkt_rt = small_rt + large_rt;

	if (pkt_rt < BNA_PKT_RATE_10K)
		load = BNA_LOAD_T_LOW_4;
	else if (pkt_rt < BNA_PKT_RATE_20K)
		load = BNA_LOAD_T_LOW_3;
	else if (pkt_rt < BNA_PKT_RATE_30K)
		load = BNA_LOAD_T_LOW_2;
	else if (pkt_rt < BNA_PKT_RATE_40K)
		load = BNA_LOAD_T_LOW_1;
	else if (pkt_rt < BNA_PKT_RATE_50K)
		load = BNA_LOAD_T_HIGH_1;
	else if (pkt_rt < BNA_PKT_RATE_60K)
		load = BNA_LOAD_T_HIGH_2;
	else if (pkt_rt < BNA_PKT_RATE_80K)
		load = BNA_LOAD_T_HIGH_3;
	else
		load = BNA_LOAD_T_HIGH_4;

	if (small_rt > (large_rt << 1))
		bias = 0;
	else
		bias = 1;

	ccb->pkt_rate.small_pkt_cnt = 0;
	ccb->pkt_rate.large_pkt_cnt = 0;

	coalescing_timeo = bna->rx_mod.dim_vector[load][bias];
	ccb->rx_coalescing_timeo = coalescing_timeo;

	/* Set it to IB */
	bna_ib_coalescing_timeo_set(&ccb->cq->ib, coalescing_timeo);
}

void
bna_rx_mod_cb_rx_stopped(void *arg, struct bna_rx_s *rx)
{
	struct bna_rx_mod_s *rx_mod = (struct bna_rx_mod_s *)arg;

	bfa_wc_down(&rx_mod->rx_stop_wc);
}

void
bna_rx_mod_cb_rx_stopped_all(void *arg)
{
	struct bna_rx_mod_s *rx_mod = (struct bna_rx_mod_s *)arg;

	if (rx_mod->stop_cbfn)
		rx_mod->stop_cbfn(&rx_mod->bna->enet);
	rx_mod->stop_cbfn = NULL;
}

void
bna_rx_mod_start(struct bna_rx_mod_s *rx_mod, enum bna_rx_type_e type)
{
	struct bna_rx_s *rx;
	bfa_q_t *qe;

	CNA_ASSERT(!(rx_mod->flags & BNA_RX_MOD_F_ENET_STARTED));
	CNA_ASSERT(!(rx_mod->flags & BNA_RX_MOD_F_ENET_LOOPBACK));

	rx_mod->flags |= BNA_RX_MOD_F_ENET_STARTED;
	if (type == BNA_RX_T_LOOPBACK)
		rx_mod->flags |= BNA_RX_MOD_F_ENET_LOOPBACK;

	bfa_q_iter(&rx_mod->rx_active_q, qe) {
		rx = (struct bna_rx_s *)qe;
		if (rx->type == type)
			bna_rx_start(rx);
	}
}

void
bna_rx_mod_stop(struct bna_rx_mod_s *rx_mod, enum bna_rx_type_e type)
{
	struct bna_rx_s *rx;
	bfa_q_t *qe;

	rx_mod->flags &= ~BNA_RX_MOD_F_ENET_STARTED;
	rx_mod->flags &= ~BNA_RX_MOD_F_ENET_LOOPBACK;

	CNA_ASSERT(rx_mod->stop_cbfn == NULL);
	rx_mod->stop_cbfn = bna_enet_cb_rx_stopped;

	CNA_ASSERT(rx_mod->rx_stop_wc.wc_count == 0);
	bfa_wc_init(&rx_mod->rx_stop_wc, bna_rx_mod_cb_rx_stopped_all, rx_mod);

	bfa_q_iter(&rx_mod->rx_active_q, qe) {
		rx = (struct bna_rx_s *)qe;
		if (rx->type == type) {
			bfa_wc_up(&rx_mod->rx_stop_wc);
			bna_rx_stop(rx);
		}
	}

	bfa_wc_wait(&rx_mod->rx_stop_wc);
}

void
bna_rx_mod_fail(struct bna_rx_mod_s *rx_mod)
{
	struct bna_rx_s *rx;
	bfa_q_t *qe;

	rx_mod->flags &= ~BNA_RX_MOD_F_ENET_STARTED;
	rx_mod->flags &= ~BNA_RX_MOD_F_ENET_LOOPBACK;

	bfa_q_iter(&rx_mod->rx_active_q, qe) {
		rx = (struct bna_rx_s *)qe;
		bna_rx_fail(rx);
	}
}

void bna_rx_mod_init(struct bna_rx_mod_s *rx_mod, struct bna_s *bna,
			struct bna_res_info_s *res_info)
{
	int	index;
	struct bna_rx_s *rx_ptr;
	struct bna_rxp_s *rxp_ptr;
	struct bna_rxq_s *rxq_ptr;

	rx_mod->bna = bna;
	rx_mod->flags = 0;

	rx_mod->rx = (struct bna_rx_s *)
		res_info[BNA_MOD_RES_MEM_T_RX_ARRAY].res_u.mem_info.mdl[0].kva;
	rx_mod->rxp = (struct bna_rxp_s *)
		res_info[BNA_MOD_RES_MEM_T_RXP_ARRAY].res_u.mem_info.mdl[0].kva;
	rx_mod->rxq = (struct bna_rxq_s *)
		res_info[BNA_MOD_RES_MEM_T_RXQ_ARRAY].res_u.mem_info.mdl[0].kva;

	/* Initialize the queues */
	bfa_q_init(&rx_mod->rx_free_q);
	rx_mod->rx_free_count = 0;
	bfa_q_init(&rx_mod->rxq_free_q);
	rx_mod->rxq_free_count = 0;
	bfa_q_init(&rx_mod->rxp_free_q);
	rx_mod->rxp_free_count = 0;
	bfa_q_init(&rx_mod->rx_active_q);

	/* Build RX queues */
	for (index = 0; index < bna->ioceth.attr.num_rxp; index++) {
		rx_ptr = &rx_mod->rx[index];

		bfa_q_qe_init(&rx_ptr->qe);
		bfa_q_init(&rx_ptr->rxp_q);
		rx_ptr->bna = NULL;
		rx_ptr->rid = index;
		rx_ptr->stop_cbfn = NULL;
		rx_ptr->stop_cbarg = NULL;

		bfa_q_enq(&rx_mod->rx_free_q, &rx_ptr->qe);
		rx_mod->rx_free_count++;
	}

	/* build RX-path queue */
	for (index = 0; index < bna->ioceth.attr.num_rxp; index++) {
		rxp_ptr = &rx_mod->rxp[index];
		bfa_q_qe_init(&rxp_ptr->qe);
		bfa_q_enq(&rx_mod->rxp_free_q, &rxp_ptr->qe);
		rx_mod->rxp_free_count++;
	}

	/* build RXQ queue */
	for (index = 0; index < (bna->ioceth.attr.num_rxp * 2); index++) {
		rxq_ptr = &rx_mod->rxq[index];
		bfa_q_qe_init(&rxq_ptr->qe);
		bfa_q_enq(&rx_mod->rxq_free_q, &rxq_ptr->qe);
		rx_mod->rxq_free_count++;
	}
}

void
bna_rx_mod_uninit(struct bna_rx_mod_s *rx_mod)
{
	bfa_q_t		*qe;
	int i;

	CNA_ASSERT(rx_mod->flags == 0);
	CNA_ASSERT(rx_mod->rx_stop_wc.wc_count == 0);
	CNA_ASSERT(rx_mod->stop_cbfn == NULL);

	CNA_ASSERT(bfa_q_is_empty(&rx_mod->rx_active_q));

	i = 0;
	bfa_q_iter(&rx_mod->rx_free_q, qe)
		i++;
	CNA_ASSERT(i == rx_mod->bna->ioceth.attr.num_rxp);

	i = 0;
	bfa_q_iter(&rx_mod->rxp_free_q, qe)
		i++;
	CNA_ASSERT(i == rx_mod->bna->ioceth.attr.num_rxp);

	i = 0;
	bfa_q_iter(&rx_mod->rxq_free_q, qe)
		i++;
	CNA_ASSERT(i == (rx_mod->bna->ioceth.attr.num_rxp * 2));

	rx_mod->bna = NULL;
}

/**
 * TX
 */

#define call_tx_stop_cbfn(tx)						\
do {									\
	if ((tx)->stop_cbfn) {						\
		void (*cbfn)(void *, struct bna_tx_s *);		\
		void *cbarg;						\
		cbfn = (tx)->stop_cbfn;					\
		cbarg = (tx)->stop_cbarg;				\
		(tx)->stop_cbfn = NULL;					\
		(tx)->stop_cbarg = NULL;				\
		cbfn(cbarg, (tx));					\
	}								\
} while (0)

#define call_tx_prio_change_cbfn(tx)					\
do {									\
	if ((tx)->prio_change_cbfn) {					\
		void (*cbfn)(struct bnad_s *, struct bna_tx_s *);	\
		cbfn = (tx)->prio_change_cbfn;				\
		(tx)->prio_change_cbfn = NULL;				\
		cbfn((tx)->bna->bnad, (tx));				\
	}								\
} while (0)

static void bna_tx_mod_cb_tx_stopped(void *tx_mod, struct bna_tx_s *tx);
static void bna_bfi_tx_enet_start(struct bna_tx_s *tx);
static void bna_tx_enet_stop(struct bna_tx_s *tx);

enum bna_tx_event_e {
	TX_E_START			= 1,
	TX_E_STOP			= 2,
	TX_E_FAIL			= 3,
	TX_E_STARTED			= 4,
	TX_E_STOPPED			= 5,
	TX_E_PRIO_CHANGE		= 6,
	TX_E_CLEANUP_DONE		= 7,
	TX_E_BW_UPDATE			= 8,
};

bfa_fsm_state_decl(bna_tx, stopped, struct bna_tx_s, enum bna_tx_event_e);
bfa_fsm_state_decl(bna_tx, start_wait, struct bna_tx_s, enum bna_tx_event_e);
bfa_fsm_state_decl(bna_tx, started, struct bna_tx_s, enum bna_tx_event_e);
bfa_fsm_state_decl(bna_tx, stop_wait, struct bna_tx_s, enum bna_tx_event_e);
bfa_fsm_state_decl(bna_tx, cleanup_wait, struct bna_tx_s,
			enum bna_tx_event_e);
bfa_fsm_state_decl(bna_tx, prio_stop_wait, struct bna_tx_s,
			enum bna_tx_event_e);
bfa_fsm_state_decl(bna_tx, prio_cleanup_wait, struct bna_tx_s,
			enum bna_tx_event_e);
bfa_fsm_state_decl(bna_tx, failed, struct bna_tx_s, enum bna_tx_event_e);
bfa_fsm_state_decl(bna_tx, quiesce_wait, struct bna_tx_s,
			enum bna_tx_event_e);

static void
bna_tx_sm_stopped_entry(struct bna_tx_s *tx)
{
	bfa_trc(tx->bna, tx->rid);
	CNA_ASSERT((tx->flags & BNA_TX_F_PRIO_CHANGED) == 0);
	CNA_ASSERT((tx->flags & BNA_TX_F_BW_UPDATED) == 0);
	call_tx_stop_cbfn(tx);
}

static void
bna_tx_sm_stopped(struct bna_tx_s *tx, enum bna_tx_event_e event)
{
	bfa_trc(tx->bna, event);
	switch (event) {
	case TX_E_START:
		bfa_fsm_set_state(tx, bna_tx_sm_start_wait);
		break;

	case TX_E_STOP:
		call_tx_stop_cbfn(tx);
		break;

	case TX_E_FAIL:
		/* No-op */
		break;

	case TX_E_PRIO_CHANGE:
		call_tx_prio_change_cbfn(tx);
		break;

	case TX_E_BW_UPDATE:
		/* No-op */
		break;

	default:
		bfa_sm_fault(tx->bna, event);
	}
}

static void
bna_tx_sm_start_wait_entry(struct bna_tx_s *tx)
{
	bfa_trc(tx->bna, tx->rid);
	bna_bfi_tx_enet_start(tx);
}

static void
bna_tx_sm_start_wait(struct bna_tx_s *tx, enum bna_tx_event_e event)
{
	bfa_trc(tx->bna, event);
	switch (event) {
	case TX_E_STOP:
		tx->flags &= ~(BNA_TX_F_PRIO_CHANGED | BNA_TX_F_BW_UPDATED);
		bfa_fsm_set_state(tx, bna_tx_sm_stop_wait);
		break;

	case TX_E_FAIL:
		tx->flags &= ~(BNA_TX_F_PRIO_CHANGED | BNA_TX_F_BW_UPDATED);
		bfa_fsm_set_state(tx, bna_tx_sm_stopped);
		break;

	case TX_E_STARTED:
		if (tx->flags & (BNA_TX_F_PRIO_CHANGED | BNA_TX_F_BW_UPDATED)) {
			tx->flags &= ~(BNA_TX_F_PRIO_CHANGED |
				BNA_TX_F_BW_UPDATED);
			bfa_fsm_set_state(tx, bna_tx_sm_prio_stop_wait);
		} else
			bfa_fsm_set_state(tx, bna_tx_sm_started);
		break;

	case TX_E_PRIO_CHANGE:
		tx->flags |=  BNA_TX_F_PRIO_CHANGED;
		break;

	case TX_E_BW_UPDATE:
		tx->flags |= BNA_TX_F_BW_UPDATED;
		break;

	default:
		bfa_sm_fault(tx->bna, event);
	}
}

static void
bna_tx_sm_started_entry(struct bna_tx_s *tx)
{
	struct bna_txq_s *txq;
	bfa_q_t		 *qe;
	enum bfi_fwboot_env boot_env = bfa_ioc_boot_env(&tx->bna->ioceth.ioc);
	boolean_t is_regular = (tx->type == BNA_TX_T_REGULAR);

	bfa_trc(tx->bna, tx->rid);
	bfa_q_iter(&tx->txq_q, qe) {
		txq = (struct bna_txq_s *)qe;
		txq->tcb->priority = txq->priority;
		/* Start IB */
		bna_ib_start(tx->bna, &txq->ib, boot_env, is_regular);
	}
	tx->tx_resume_cbfn(tx->bna->bnad, tx);
}

static void
bna_tx_sm_started(struct bna_tx_s *tx, enum bna_tx_event_e event)
{
	bfa_trc(tx->bna, event);
	switch (event) {
	case TX_E_STOP:
		bfa_fsm_set_state(tx, bna_tx_sm_stop_wait);
		tx->tx_stall_cbfn(tx->bna->bnad, tx);
		bna_tx_enet_stop(tx);
		break;

	case TX_E_FAIL:
		bfa_fsm_set_state(tx, bna_tx_sm_failed);
		tx->tx_stall_cbfn(tx->bna->bnad, tx);
		tx->tx_cleanup_cbfn(tx->bna->bnad, tx);
		break;

	case TX_E_PRIO_CHANGE:
	case TX_E_BW_UPDATE:
		bfa_fsm_set_state(tx, bna_tx_sm_prio_stop_wait);
		break;

	default:
		bfa_sm_fault(tx->bna, event);
	}
}

static void
bna_tx_sm_stop_wait_entry(struct bna_tx_s *tx)
{
	bfa_trc(tx->bna, tx->rid);
}

static void
bna_tx_sm_stop_wait(struct bna_tx_s *tx, enum bna_tx_event_e event)
{
	bfa_trc(tx->bna, event);
	switch (event) {
	case TX_E_FAIL:
	case TX_E_STOPPED:
		bfa_fsm_set_state(tx, bna_tx_sm_cleanup_wait);
		tx->tx_cleanup_cbfn(tx->bna->bnad, tx);
		break;

	case TX_E_STARTED:
		/**
		 * We are here due to start_wait -> stop_wait transition on
		 * TX_E_STOP event
		 */
		bna_tx_enet_stop(tx);
		break;

	case TX_E_PRIO_CHANGE:
	case TX_E_BW_UPDATE:
		/* No-op */
		break;

	default:
		bfa_sm_fault(tx->bna, event);
	}
}

static void
bna_tx_sm_cleanup_wait_entry(struct bna_tx_s *tx)
{
	bfa_trc(tx->bna, tx->rid);
}

static void
bna_tx_sm_cleanup_wait(struct bna_tx_s *tx, enum bna_tx_event_e event)
{
	bfa_trc(tx->bna, event);
	switch (event) {
	case TX_E_FAIL:
	case TX_E_PRIO_CHANGE:
	case TX_E_BW_UPDATE:
		/* No-op */
		break;

	case TX_E_CLEANUP_DONE:
		bfa_fsm_set_state(tx, bna_tx_sm_stopped);
		break;

	default:
		bfa_sm_fault(tx->bna, event);
	}
}

static void
bna_tx_sm_prio_stop_wait_entry(struct bna_tx_s *tx)
{
	bfa_trc(tx->bna, tx->rid);
	tx->tx_stall_cbfn(tx->bna->bnad, tx);
	bna_tx_enet_stop(tx);
}

static void
bna_tx_sm_prio_stop_wait(struct bna_tx_s *tx, enum bna_tx_event_e event)
{
	bfa_trc(tx->bna, event);

	switch (event) {
	case TX_E_STOP:
		bfa_fsm_set_state(tx, bna_tx_sm_stop_wait);
		break;

	case TX_E_FAIL:
		bfa_fsm_set_state(tx, bna_tx_sm_failed);
		call_tx_prio_change_cbfn(tx);
		tx->tx_cleanup_cbfn(tx->bna->bnad, tx);
		break;

	case TX_E_STOPPED:
		bfa_fsm_set_state(tx, bna_tx_sm_prio_cleanup_wait);
		break;

	case TX_E_PRIO_CHANGE:
	case TX_E_BW_UPDATE:
		/* No-op */
		break;

	default:
		bfa_sm_fault(tx->bna, event);
	}
}

static void
bna_tx_sm_prio_cleanup_wait_entry(struct bna_tx_s *tx)
{
	bfa_trc(tx->bna, tx->rid);

	call_tx_prio_change_cbfn(tx);
	tx->tx_cleanup_cbfn(tx->bna->bnad, tx);
}

static void
bna_tx_sm_prio_cleanup_wait(struct bna_tx_s *tx, enum bna_tx_event_e event)
{
	bfa_trc(tx->bna, event);

	switch (event) {
	case TX_E_STOP:
		bfa_fsm_set_state(tx, bna_tx_sm_cleanup_wait);
		break;

	case TX_E_FAIL:
		bfa_fsm_set_state(tx, bna_tx_sm_failed);
		break;

	case TX_E_PRIO_CHANGE:
	case TX_E_BW_UPDATE:
		/* No-op */
		break;

	case TX_E_CLEANUP_DONE:
		bfa_fsm_set_state(tx, bna_tx_sm_start_wait);
		break;

	default:
		bfa_sm_fault(tx->bna, event);
	}
}

static void
bna_tx_sm_failed_entry(struct bna_tx_s *tx)
{
	bfa_trc(tx->bna, tx->rid);
}

static void
bna_tx_sm_failed(struct bna_tx_s *tx, enum bna_tx_event_e event)
{
	bfa_trc(tx->bna, event);

	switch (event) {
	case TX_E_START:
		bfa_fsm_set_state(tx, bna_tx_sm_quiesce_wait);
		break;

	case TX_E_STOP:
		bfa_fsm_set_state(tx, bna_tx_sm_cleanup_wait);
		break;

	case TX_E_FAIL:
		/* No-op */
		break;

	case TX_E_CLEANUP_DONE:
		bfa_fsm_set_state(tx, bna_tx_sm_stopped);
		break;

	default:
		bfa_sm_fault(tx->bna, event);
	}
}

static void
bna_tx_sm_quiesce_wait_entry(struct bna_tx_s *tx)
{
	bfa_trc(tx->bna, tx->rid);
}

static void
bna_tx_sm_quiesce_wait(struct bna_tx_s *tx, enum bna_tx_event_e event)
{
	bfa_trc(tx->bna, event);

	switch (event) {
	case TX_E_STOP:
		bfa_fsm_set_state(tx, bna_tx_sm_cleanup_wait);
		break;

	case TX_E_FAIL:
		bfa_fsm_set_state(tx, bna_tx_sm_failed);
		break;

	case TX_E_CLEANUP_DONE:
		bfa_fsm_set_state(tx, bna_tx_sm_start_wait);
		break;

	case TX_E_BW_UPDATE:
		/* No-op */
		break;

	default:
		bfa_sm_fault(tx->bna, event);
	}
}

static void
bna_bfi_tx_enet_start(struct bna_tx_s *tx)
{
	struct bfi_enet_tx_cfg_req *cfg_req = &tx->bfi_enet_cmd.cfg_req;
	struct bna_txq_s *txq = NULL;
	bfa_q_t *qe;
	int i;

	bfi_msgq_mhdr_set(cfg_req->mh, BFI_MC_ENET,
		BFI_ENET_H2I_TX_CFG_SET_REQ, 0, tx->rid);
	cfg_req->mh.num_entries = cna_os_htons(
		bfi_msgq_num_cmd_entries(sizeof(struct bfi_enet_tx_cfg_req)));

	cfg_req->num_queues = tx->num_txq;
	for (i = 0, qe = bfa_q_first(&tx->txq_q);
		i < tx->num_txq;
		i++, qe = bfa_q_next(qe)) {
		txq = (struct bna_txq_s *)qe;

		bfi_enet_datapath_q_init(&cfg_req->q_cfg[i].q.q, &txq->qpt);
		cfg_req->q_cfg[i].q.priority = txq->priority;

		cfg_req->q_cfg[i].ib.index_addr.a32.addr_lo =
			txq->ib.ib_seg_host_addr.lsb;
		cfg_req->q_cfg[i].ib.index_addr.a32.addr_hi =
			txq->ib.ib_seg_host_addr.msb;
		cfg_req->q_cfg[i].ib.intr.msix_index =
			cna_os_htons((uint16_t)txq->ib.intr_vector);
	}

	cfg_req->ib_cfg.int_pkt_dma = BNA_STATUS_T_ENABLED;
	cfg_req->ib_cfg.int_enabled = BNA_STATUS_T_ENABLED;
	cfg_req->ib_cfg.int_pkt_enabled = BNA_STATUS_T_DISABLED;
	cfg_req->ib_cfg.continuous_coalescing = BNA_STATUS_T_ENABLED;
	cfg_req->ib_cfg.msix = (txq->ib.intr_type == BNA_INTR_T_MSIX)
				? BNA_STATUS_T_ENABLED : BNA_STATUS_T_DISABLED;
	cfg_req->ib_cfg.coalescing_timeout =
			cna_os_htonl((uint32_t)txq->ib.coalescing_timeo);
	cfg_req->ib_cfg.inter_pkt_timeout =
			cna_os_htonl((uint32_t)txq->ib.interpkt_timeo);
	cfg_req->ib_cfg.inter_pkt_count = (uint8_t)txq->ib.interpkt_count;

	cfg_req->tx_cfg.vlan_mode = BFI_ENET_TX_VLAN_WI;
	cfg_req->tx_cfg.vlan_id = cna_os_htons((uint16_t)tx->txf_vlan_id);
	cfg_req->tx_cfg.admit_tagged_frame = BNA_STATUS_T_ENABLED;
	cfg_req->tx_cfg.apply_vlan_filter = BNA_STATUS_T_DISABLED;

	if (tx->type == BNA_TX_T_REGULAR)
		cfg_req->tx_cfg.add_to_vswitch = tx->veb_enable;

	bfa_msgq_cmd_set(&tx->msgq_cmd, NULL, NULL,
		sizeof(struct bfi_enet_tx_cfg_req), &cfg_req->mh);
	bfa_msgq_cmd_post(&tx->bna->msgq, &tx->msgq_cmd);
}

static void
bna_bfi_tx_enet_stop(struct bna_tx_s *tx)
{
	struct bfi_enet_req *req = &tx->bfi_enet_cmd.req;

	bfi_msgq_mhdr_set(req->mh, BFI_MC_ENET,
		BFI_ENET_H2I_TX_CFG_CLR_REQ, 0, tx->rid);
	req->mh.num_entries = cna_os_htons(
		bfi_msgq_num_cmd_entries(sizeof(struct bfi_enet_req)));
	bfa_msgq_cmd_set(&tx->msgq_cmd, NULL, NULL, sizeof(struct bfi_enet_req),
		&req->mh);
	bfa_msgq_cmd_post(&tx->bna->msgq, &tx->msgq_cmd);
}

static void
bna_tx_enet_stop(struct bna_tx_s *tx)
{
	struct bna_txq_s *txq;
	bfa_q_t		 *qe;

	/* Stop IB */
	bfa_q_iter(&tx->txq_q, qe) {
		txq = (struct bna_txq_s *)qe;
		bna_ib_stop(tx->bna, &txq->ib);
	}

	bna_bfi_tx_enet_stop(tx);
}

static void
bna_txq_qpt_setup(struct bna_txq_s *txq, int page_count, int page_size,
		struct bna_mem_descr_s *qpt_mem,
		struct bna_mem_descr_s *swqpt_mem,
		struct bna_mem_descr_s *page_mem)
{
	uint8_t *kva;
	uint64_t dma;
	struct bna_dma_addr_s bna_dma;
	int i;

	txq->qpt.hw_qpt_ptr.lsb = qpt_mem->dma.lsb;
	txq->qpt.hw_qpt_ptr.msb = qpt_mem->dma.msb;
	txq->qpt.kv_qpt_ptr = qpt_mem->kva;
	txq->qpt.page_count = page_count;
	txq->qpt.page_size = page_size;

	bfa_trc(txq->tx->bna, txq->qpt.hw_qpt_ptr.lsb);
	bfa_trc(txq->tx->bna, txq->qpt.hw_qpt_ptr.msb);
	bfa_trc(txq->tx->bna, txq->qpt.page_count);
	bfa_trc(txq->tx->bna, txq->qpt.page_size);

	txq->tcb->sw_qpt = (void **) swqpt_mem->kva;
	txq->tcb->sw_q = page_mem->kva;

	kva = page_mem->kva;
	BNA_GET_DMA_ADDR(&page_mem->dma, dma);

	for (i = 0; i < page_count; i++) {
		txq->tcb->sw_qpt[i] = kva;
		kva += CNA_PAGE_SIZE;

		BNA_SET_DMA_ADDR(dma, &bna_dma);
		((struct bna_dma_addr_s *)txq->qpt.kv_qpt_ptr)[i].lsb =
			bna_dma.lsb;
		((struct bna_dma_addr_s *)txq->qpt.kv_qpt_ptr)[i].msb =
			bna_dma.msb;
		dma += CNA_PAGE_SIZE;

		bfa_trc(txq->tx->bna,
			((struct bna_dma_addr_s *)txq->qpt.kv_qpt_ptr)[i].lsb);
		bfa_trc(txq->tx->bna,
			((struct bna_dma_addr_s *)txq->qpt.kv_qpt_ptr)[i].msb);
	}
}

static struct bna_tx_s *
bna_tx_get(struct bna_tx_mod_s *tx_mod, enum bna_tx_type_e type)
{
	bfa_q_t	*qe = NULL;
	struct bna_tx_s *tx = NULL;

	CNA_ASSERT((type == BNA_TX_T_REGULAR) || (type == BNA_TX_T_LOOPBACK));

	if (bfa_q_is_empty(&tx_mod->tx_free_q))
		return NULL;
	if (type == BNA_TX_T_REGULAR) {
		bfa_q_deq(&tx_mod->tx_free_q, &qe);
	} else {
		bfa_q_deq_tail(&tx_mod->tx_free_q, &qe);
	}
	tx = (struct bna_tx_s *)qe;
	bfa_q_qe_init(&tx->qe);
	tx->type = type;

	return tx;
}

static void
bna_tx_free(struct bna_tx_s *tx)
{
	struct bna_tx_mod_s *tx_mod = &tx->bna->tx_mod;
	struct bna_txq_s *txq;
	bfa_q_t *prev_qe;
	bfa_q_t *qe;

	bfa_trc(tx->bna, tx->rid);

	while (!bfa_q_is_empty(&tx->txq_q)) {
		bfa_q_deq(&tx->txq_q, &txq);
		bfa_q_qe_init(&txq->qe);
		txq->tcb = NULL;
		txq->tx = NULL;
		bfa_q_enq(&tx_mod->txq_free_q, &txq->qe);
	}

	bfa_q_iter(&tx_mod->tx_active_q, qe) {
		if (qe == &tx->qe) {
			bfa_q_qe_deq(&tx->qe);
			bfa_q_qe_init(&tx->qe);
			break;
		}
	}

	tx->bna = NULL;
	tx->priv = NULL;

	prev_qe = NULL;
	bfa_q_iter(&tx_mod->tx_free_q, qe) {
		if (((struct bna_tx_s *)qe)->rid < tx->rid)
			prev_qe = qe;
		else {
			CNA_ASSERT(((struct bna_tx_s *)qe)->rid > tx->rid);
			break;
		}
	}

	if (prev_qe == NULL) {
		/* This is the first entry */
		bfa_q_enq_head(&tx_mod->tx_free_q, &tx->qe);
	} else if (bfa_q_next(prev_qe) == &tx_mod->tx_free_q) {
		/* This is the last entry */
		bfa_q_enq(&tx_mod->tx_free_q, &tx->qe);
	} else {
		/* Somewhere in the middle */
		bfa_q_next(&tx->qe) = bfa_q_next(prev_qe);
		bfa_q_prev(&tx->qe) = prev_qe;
		bfa_q_next(prev_qe) = &tx->qe;
		bfa_q_prev(bfa_q_next(&tx->qe)) = &tx->qe;
	}
}

static void
bna_tx_start(struct bna_tx_s *tx)
{
	CNA_ASSERT((tx->fsm == (bfa_sm_t)bna_tx_sm_stopped) ||
			(tx->fsm == (bfa_sm_t)bna_tx_sm_failed));

	bfa_trc(tx->bna, tx->flags);
	tx->flags |= BNA_TX_F_ENET_STARTED;
	if (tx->flags & BNA_TX_F_ENABLED)
		bfa_fsm_send_event(tx, TX_E_START);
}

static void
bna_tx_stop(struct bna_tx_s *tx)
{
	CNA_ASSERT(tx->stop_cbfn == NULL);
	CNA_ASSERT(tx->prio_change_cbfn == NULL);
	tx->stop_cbfn = bna_tx_mod_cb_tx_stopped;
	tx->stop_cbarg = &tx->bna->tx_mod;

	bfa_trc(tx->bna, tx->flags);
	tx->flags &= ~BNA_TX_F_ENET_STARTED;
	bfa_fsm_send_event(tx, TX_E_STOP);
}

static void
bna_tx_fail(struct bna_tx_s *tx)
{
	bfa_trc(tx->bna, tx->flags);
	tx->flags &= ~BNA_TX_F_ENET_STARTED;
	bfa_fsm_send_event(tx, TX_E_FAIL);
}

static void
bna_tx_prio_changed(struct bna_tx_s *tx)
{
	struct bna_tx_mod_s *tx_mod = &tx->bna->tx_mod;
	struct bna_txq_s *txq;
	bfa_q_t		 *qe;

	/* No need of priority reconfiguration for loopback Tx */
	if (tx->type != BNA_TX_T_REGULAR)
		return;

	/**
	 * If there are exactly 8 TxQs, each one occupies one priority.
	 * In such case, there is nothing to reconfigure, since their
	 * priorities remain the same.
	 */
	if (tx->num_txq != BFI_TX_MAX_PRIO) {
		bfa_q_iter(&tx->txq_q, qe) {
			txq = (struct bna_txq_s *)qe;
			txq->priority = (uint8_t)tx_mod->default_prio;
		}

		bfa_fsm_send_event(tx, TX_E_PRIO_CHANGE);
	}
}

void
bna_bfi_tx_enet_start_rsp(struct bna_tx_s *tx, struct bfi_msgq_mhdr_s *msghdr)
{
	struct bfi_enet_tx_cfg_rsp *cfg_rsp = &tx->bfi_enet_cmd.cfg_rsp;
	struct bna_txq_s *txq = NULL;
	bfa_q_t *qe;
	int i;

	bfa_msgq_rsp_copy(&tx->bna->msgq, (uint8_t *)cfg_rsp,
		sizeof(struct bfi_enet_tx_cfg_rsp));

	CNA_ASSERT(cfg_rsp->error == 0);
	tx->hw_id = cfg_rsp->hw_id;

	for (i = 0, qe = bfa_q_first(&tx->txq_q);
		i < tx->num_txq; i++, qe = bfa_q_next(qe)) {

		txq = (struct bna_txq_s *)qe;

		/* Setup doorbells */
		txq->tcb->i_dbell->doorbell_addr =
			tx->bna->pcidev.pci_bar_kva
			+ cna_os_ntohl(cfg_rsp->q_handles[i].i_dbell);
		bfa_trc(tx->bna, cna_os_ntohl(cfg_rsp->q_handles[i].i_dbell));
		txq->tcb->q_dbell =
			tx->bna->pcidev.pci_bar_kva
			+ cna_os_ntohl(cfg_rsp->q_handles[i].q_dbell);
		bfa_trc(tx->bna, cna_os_ntohl(cfg_rsp->q_handles[i].q_dbell));
		txq->hw_id = cfg_rsp->q_handles[i].hw_qid;

		/* Initialize producer/consumer indexes */
		(*txq->tcb->hw_consumer_index) = 0;
		txq->tcb->producer_index = txq->tcb->consumer_index = 0;
	}

	bfa_fsm_send_event(tx, TX_E_STARTED);
}

void
bna_bfi_tx_enet_stop_rsp(struct bna_tx_s *tx, struct bfi_msgq_mhdr_s *msghdr)
{
	struct bfi_enet_rsp *rsp = (struct bfi_enet_rsp *)msghdr;

	CNA_ASSERT(rsp->error == 0);
	bfa_fsm_send_event(tx, TX_E_STOPPED);
}

void
bna_bfi_tx_bw_update(struct bna_tx_mod_s *tx_mod)
{
	struct bna_tx_s *tx;
	bfa_q_t		*qe;

	bfa_q_iter(&tx_mod->tx_active_q, qe) {
		tx = (struct bna_tx_s *)qe;
		bfa_fsm_send_event(tx, TX_E_BW_UPDATE);
	}
}

void
bna_tx_res_req(int num_txq, int txq_depth, struct bna_res_info_s *res_info)
{
	uint32_t q_size;
	uint32_t page_count;
	struct bna_mem_info_s *mem_info;

	res_info[BNA_TX_RES_MEM_T_TCB].res_type = BNA_RES_T_MEM;
	mem_info = &res_info[BNA_TX_RES_MEM_T_TCB].res_u.mem_info;
	mem_info->mem_type = BNA_MEM_T_KVA;
	mem_info->len = sizeof(struct bna_tcb_s);
	mem_info->num = num_txq;

	CNA_ASSERT(BNA_POWER_OF_2(txq_depth));
	q_size = txq_depth * BFI_TXQ_WI_SIZE;
	q_size = CNA_ALIGN(q_size, CNA_PAGE_SIZE);
	page_count = q_size >> CNA_PAGE_SHIFT;

	res_info[BNA_TX_RES_MEM_T_QPT].res_type = BNA_RES_T_MEM;
	mem_info = &res_info[BNA_TX_RES_MEM_T_QPT].res_u.mem_info;
	mem_info->mem_type = BNA_MEM_T_DMA;
	mem_info->len = page_count * sizeof(struct bna_dma_addr_s);
	mem_info->num = num_txq;

	res_info[BNA_TX_RES_MEM_T_SWQPT].res_type = BNA_RES_T_MEM;
	mem_info = &res_info[BNA_TX_RES_MEM_T_SWQPT].res_u.mem_info;
	mem_info->mem_type = BNA_MEM_T_KVA;
	mem_info->len = page_count * sizeof(void *);
	mem_info->num = num_txq;

	res_info[BNA_TX_RES_MEM_T_PAGE].res_type = BNA_RES_T_MEM;
	mem_info = &res_info[BNA_TX_RES_MEM_T_PAGE].res_u.mem_info;
	mem_info->mem_type = BNA_MEM_T_DMA;
	mem_info->len = CNA_PAGE_SIZE * page_count;
	mem_info->num = num_txq;

	res_info[BNA_TX_RES_MEM_T_IBIDX].res_type = BNA_RES_T_MEM;
	mem_info = &res_info[BNA_TX_RES_MEM_T_IBIDX].res_u.mem_info;
	mem_info->mem_type = BNA_MEM_T_DMA;
	mem_info->len = BFI_IBIDX_SIZE;
	mem_info->num = num_txq;

	res_info[BNA_TX_RES_INTR_T_TXCMPL].res_type = BNA_RES_T_INTR;
	res_info[BNA_TX_RES_INTR_T_TXCMPL].res_u.intr_info.intr_type =
			BNA_INTR_T_MSIX;
	res_info[BNA_TX_RES_INTR_T_TXCMPL].res_u.intr_info.num = num_txq;
}

struct bna_tx_s *
bna_tx_create(struct bna_s *bna, struct bnad_s *bnad,
		struct bna_tx_config_s *tx_cfg,
		struct bna_tx_event_cbfn_s *tx_cbfn,
		struct bna_res_info_s *res_info, void *priv)
{
	struct bna_intr_info_s *intr_info;
	struct bna_tx_mod_s *tx_mod = &bna->tx_mod;
	struct bna_tx_s *tx;
	struct bna_txq_s *txq;
	bfa_q_t *qe;
	int page_count;
	int i;

	CNA_ASSERT(BNA_POWER_OF_2(tx_cfg->txq_depth));

	intr_info = &res_info[BNA_TX_RES_INTR_T_TXCMPL].res_u.intr_info;
	page_count = (res_info[BNA_TX_RES_MEM_T_PAGE].res_u.mem_info.len) /
			CNA_PAGE_SIZE;

	/**
	 * Get resources
	 */

	if ((intr_info->num != 1) && (intr_info->num != tx_cfg->num_txq))
		return NULL;

	/* Tx */

	tx = bna_tx_get(tx_mod, tx_cfg->tx_type);
	if (!tx)
		return NULL;
	tx->bna = bna;
	tx->priv = priv;

	/* TxQs */

	bfa_q_init(&tx->txq_q);
	for (i = 0; i < tx_cfg->num_txq; i++) {
		if (bfa_q_is_empty(&tx_mod->txq_free_q))
			goto err_return;

		bfa_q_deq(&tx_mod->txq_free_q, &txq);
		bfa_q_qe_init(&txq->qe);
		bfa_q_enq(&tx->txq_q, &txq->qe);
		txq->tx = tx;
	}

	/*
	 * Initialize
	 */

	/* Tx */

	tx->tcb_setup_cbfn = tx_cbfn->tcb_setup_cbfn;
	tx->tcb_destroy_cbfn = tx_cbfn->tcb_destroy_cbfn;
	/* Following callbacks are mandatory */
	tx->tx_stall_cbfn = tx_cbfn->tx_stall_cbfn;
	tx->tx_resume_cbfn = tx_cbfn->tx_resume_cbfn;
	tx->tx_cleanup_cbfn = tx_cbfn->tx_cleanup_cbfn;
	CNA_ASSERT(tx->tx_stall_cbfn);
	CNA_ASSERT(tx->tx_resume_cbfn);
	CNA_ASSERT(tx->tx_cleanup_cbfn);

	bfa_q_enq(&tx_mod->tx_active_q, &tx->qe);
	CNA_ASSERT(tx->stop_cbfn == NULL);
	CNA_ASSERT(tx->prio_change_cbfn == NULL);

	tx->num_txq = tx_cfg->num_txq;

	tx->flags = 0;
	if (tx->bna->tx_mod.flags & BNA_TX_MOD_F_ENET_STARTED) {
		switch (tx->type) {
		case BNA_TX_T_REGULAR:
			if (!(tx->bna->tx_mod.flags &
				BNA_TX_MOD_F_ENET_LOOPBACK))
				tx->flags |= BNA_TX_F_ENET_STARTED;
			break;
		case BNA_TX_T_LOOPBACK:
			if (tx->bna->tx_mod.flags & BNA_TX_MOD_F_ENET_LOOPBACK)
				tx->flags |= BNA_TX_F_ENET_STARTED;
			break;
		}
	}

	/* TxQ */

	i = 0;
	bfa_q_iter(&tx->txq_q, qe) {
		txq = (struct bna_txq_s *)qe;
		txq->tcb = (struct bna_tcb_s *)
		res_info[BNA_TX_RES_MEM_T_TCB].res_u.mem_info.mdl[i].kva;
		txq->tx_packets = 0;
		txq->tx_bytes = 0;

		/* IB */
		txq->ib.ib_seg_host_addr.lsb =
		res_info[BNA_TX_RES_MEM_T_IBIDX].res_u.mem_info.mdl[i].dma.lsb;
		txq->ib.ib_seg_host_addr.msb =
		res_info[BNA_TX_RES_MEM_T_IBIDX].res_u.mem_info.mdl[i].dma.msb;
		txq->ib.ib_seg_host_addr_kva =
		res_info[BNA_TX_RES_MEM_T_IBIDX].res_u.mem_info.mdl[i].kva;
		txq->ib.intr_type = intr_info->intr_type;
		txq->ib.intr_vector = (intr_info->num == 1) ?
					intr_info->idl[0].vector :
					intr_info->idl[i].vector;
		if (intr_info->intr_type == BNA_INTR_T_INTX)
			txq->ib.intr_vector = (1 <<  txq->ib.intr_vector);
		txq->ib.coalescing_timeo = tx_cfg->coalescing_timeo;
		txq->ib.interpkt_timeo = BFI_TX_INTERPKT_TIMEO;
		txq->ib.interpkt_count = BFI_TX_INTERPKT_COUNT;

		/* TCB */

		txq->tcb->q_depth = tx_cfg->txq_depth;
		txq->tcb->unmap_q = (void *)
		res_info[BNA_TX_RES_MEM_T_UNMAPQ].res_u.mem_info.mdl[i].kva;
		txq->tcb->hw_consumer_index =
			(uint32_t *)txq->ib.ib_seg_host_addr_kva;
		txq->tcb->i_dbell = &txq->ib.door_bell;
		txq->tcb->intr_type = txq->ib.intr_type;
		txq->tcb->intr_vector = txq->ib.intr_vector;
		txq->tcb->txq = txq;
		txq->tcb->bnad = bnad;
		txq->tcb->id = i;

		/* QPT, SWQPT, Pages */
		bna_txq_qpt_setup(txq, page_count, CNA_PAGE_SIZE,
			&res_info[BNA_TX_RES_MEM_T_QPT].res_u.mem_info.mdl[i],
			&res_info[BNA_TX_RES_MEM_T_SWQPT].res_u.mem_info.mdl[i],
			&res_info[BNA_TX_RES_MEM_T_PAGE].
				  res_u.mem_info.mdl[i]);

		/* Callback to shim for setting up TCB */
		if (tx->tcb_setup_cbfn)
			(tx->tcb_setup_cbfn)(bna->bnad, txq->tcb);

		if (tx_cfg->num_txq == BFI_TX_MAX_PRIO)
			txq->priority = txq->tcb->id;
		else
			txq->priority = tx_mod->default_prio;

		i++;
	}

	tx->txf_vlan_id = 0;

	bfa_fsm_set_state(tx, bna_tx_sm_stopped);

	tx_mod->rid_mask |= (1 << tx->rid);

	return tx;

err_return:
	bna_tx_free(tx);
	return NULL;
}

void
bna_tx_destroy(struct bna_tx_s *tx)
{
	struct bna_txq_s *txq;
	bfa_q_t *qe;

	CNA_ASSERT(tx->fsm == (bfa_sm_t)bna_tx_sm_stopped);

	bfa_q_iter(&tx->txq_q, qe) {
		txq = (struct bna_txq_s *)qe;
		if (tx->tcb_destroy_cbfn)
			(tx->tcb_destroy_cbfn)(tx->bna->bnad, txq->tcb);
	}

	tx->bna->tx_mod.rid_mask &= ~(1 << tx->rid);
	bna_tx_free(tx);
}

void
bna_tx_enable(struct bna_tx_s *tx)
{
	if (tx->fsm != (bfa_sm_t)bna_tx_sm_stopped)
		return;

	bfa_trc(tx->bna, tx->flags);
	tx->flags |= BNA_TX_F_ENABLED;

	if (tx->flags & BNA_TX_F_ENET_STARTED)
		bfa_fsm_send_event(tx, TX_E_START);
}

void
bna_tx_disable(struct bna_tx_s *tx, enum bna_cleanup_type_e type,
		void (*cbfn)(void *, struct bna_tx_s *))
{
	CNA_ASSERT(cbfn);

	if (type == BNA_SOFT_CLEANUP) {
		(*cbfn)(tx->bna->bnad, tx);
		return;
	}

	CNA_ASSERT(tx->stop_cbfn == NULL);
	CNA_ASSERT(tx->prio_change_cbfn == NULL);
	tx->stop_cbfn = cbfn;
	tx->stop_cbarg = tx->bna->bnad;

	bfa_trc(tx->bna, tx->flags);
	tx->flags &= ~BNA_TX_F_ENABLED;

	bfa_fsm_send_event(tx, TX_E_STOP);
}

void
bna_tx_cleanup_complete(struct bna_tx_s *tx)
{
	bfa_fsm_send_event(tx, TX_E_CLEANUP_DONE);
}

void
bna_tx_prio_set(struct bna_tx_s *tx, int prio,
		void (*cbfn)(struct bnad_s *, struct bna_tx_s *))
{
	struct bna_txq_s *txq;
	bfa_q_t		 *qe;

	CNA_ASSERT(tx->stop_cbfn == NULL);
	CNA_ASSERT(tx->prio_change_cbfn == NULL);
	tx->prio_change_cbfn = cbfn;

	bfa_q_iter(&tx->txq_q, qe) {
		txq = (struct bna_txq_s *)qe;
		txq->priority = (uint8_t)prio;
	}

	bfa_fsm_send_event(tx, TX_E_PRIO_CHANGE);
}

void
bna_tx_coalescing_timeo_set(struct bna_tx_s *tx, int coalescing_timeo)
{
	struct bna_txq_s *txq;
	bfa_q_t *qe;

	bfa_q_iter(&tx->txq_q, qe) {
		txq = (struct bna_txq_s *)qe;
		bna_ib_coalescing_timeo_set(&txq->ib, coalescing_timeo);
	}
}

static void
bna_tx_mod_cb_tx_stopped(void *arg, struct bna_tx_s *tx)
{
	struct bna_tx_mod_s *tx_mod = (struct bna_tx_mod_s *)arg;

	bfa_wc_down(&tx_mod->tx_stop_wc);
}

static void
bna_tx_mod_cb_tx_stopped_all(void *arg)
{
	struct bna_tx_mod_s *tx_mod = (struct bna_tx_mod_s *)arg;

	if (tx_mod->stop_cbfn)
		tx_mod->stop_cbfn(&tx_mod->bna->enet);
	tx_mod->stop_cbfn = NULL;
}

void
bna_tx_mod_init(struct bna_tx_mod_s *tx_mod, struct bna_s *bna,
		struct bna_res_info_s *res_info)
{
	int i;

	tx_mod->bna = bna;
	tx_mod->flags = 0;

	tx_mod->tx = (struct bna_tx_s *)
		res_info[BNA_MOD_RES_MEM_T_TX_ARRAY].res_u.mem_info.mdl[0].kva;
	tx_mod->txq = (struct bna_txq_s *)
		res_info[BNA_MOD_RES_MEM_T_TXQ_ARRAY].res_u.mem_info.mdl[0].kva;

	bfa_q_init(&tx_mod->tx_free_q);
	bfa_q_init(&tx_mod->tx_active_q);

	bfa_q_init(&tx_mod->txq_free_q);

	for (i = 0; i < bna->ioceth.attr.num_txq; i++) {
		tx_mod->tx[i].rid = i;
		bfa_q_qe_init(&tx_mod->tx[i].qe);
		bfa_q_enq(&tx_mod->tx_free_q, &tx_mod->tx[i].qe);
		bfa_q_qe_init(&tx_mod->txq[i].qe);
		bfa_q_enq(&tx_mod->txq_free_q, &tx_mod->txq[i].qe);
	}

	tx_mod->prio_map = BFI_TX_PRIO_MAP_ALL;
	tx_mod->default_prio = 0;
	tx_mod->iscsi_over_cee = BNA_STATUS_T_DISABLED;
	tx_mod->iscsi_prio = -1;
}

void
bna_tx_mod_uninit(struct bna_tx_mod_s *tx_mod)
{
	bfa_q_t		*qe;
	int i;

	CNA_ASSERT(tx_mod->flags == 0);
	CNA_ASSERT(tx_mod->tx_stop_wc.wc_count == 0);
	CNA_ASSERT(tx_mod->stop_cbfn == NULL);

	CNA_ASSERT(bfa_q_is_empty(&tx_mod->tx_active_q));

	i = 0;
	bfa_q_iter(&tx_mod->tx_free_q, qe)
		i++;
	CNA_ASSERT(i == tx_mod->bna->ioceth.attr.num_txq);

	i = 0;
	bfa_q_iter(&tx_mod->txq_free_q, qe)
		i++;
	CNA_ASSERT(i == tx_mod->bna->ioceth.attr.num_txq);

	tx_mod->bna = NULL;
}

void
bna_tx_mod_start(struct bna_tx_mod_s *tx_mod, enum bna_tx_type_e type)
{
	struct bna_tx_s *tx;
	bfa_q_t		*qe;

	CNA_ASSERT(!(tx_mod->flags & BNA_TX_MOD_F_ENET_STARTED));
	CNA_ASSERT(!(tx_mod->flags & BNA_TX_MOD_F_ENET_LOOPBACK));

	tx_mod->flags |= BNA_TX_MOD_F_ENET_STARTED;
	if (type == BNA_TX_T_LOOPBACK)
		tx_mod->flags |= BNA_TX_MOD_F_ENET_LOOPBACK;

	bfa_q_iter(&tx_mod->tx_active_q, qe) {
		tx = (struct bna_tx_s *)qe;
		if (tx->type == type)
			bna_tx_start(tx);
	}
}

void
bna_tx_mod_stop(struct bna_tx_mod_s *tx_mod, enum bna_tx_type_e type)
{
	struct bna_tx_s *tx;
	bfa_q_t		*qe;

	tx_mod->flags &= ~BNA_TX_MOD_F_ENET_STARTED;
	tx_mod->flags &= ~BNA_TX_MOD_F_ENET_LOOPBACK;

	CNA_ASSERT(tx_mod->stop_cbfn == NULL);
	tx_mod->stop_cbfn = bna_enet_cb_tx_stopped;

	CNA_ASSERT(tx_mod->tx_stop_wc.wc_count == 0);
	bfa_wc_init(&tx_mod->tx_stop_wc, bna_tx_mod_cb_tx_stopped_all, tx_mod);

	bfa_q_iter(&tx_mod->tx_active_q, qe) {
		tx = (struct bna_tx_s *)qe;
		if (tx->type == type) {
			bfa_wc_up(&tx_mod->tx_stop_wc);
			bna_tx_stop(tx);
		}
	}

	bfa_wc_wait(&tx_mod->tx_stop_wc);
}

void
bna_tx_mod_fail(struct bna_tx_mod_s *tx_mod)
{
	struct bna_tx_s *tx;
	bfa_q_t		*qe;

	tx_mod->flags &= ~BNA_TX_MOD_F_ENET_STARTED;
	tx_mod->flags &= ~BNA_TX_MOD_F_ENET_LOOPBACK;

	bfa_q_iter(&tx_mod->tx_active_q, qe) {
		tx = (struct bna_tx_s *)qe;
		bna_tx_fail(tx);
	}
}

void
bna_tx_mod_prio_reconfig(struct bna_tx_mod_s *tx_mod, int cee_linkup,
			uint8_t prio_map, uint8_t iscsi_prio_map)
{
	struct bna_tx_s *tx;
	bfa_q_t		*qe;
	int need_txq_reconfig = 0;
	int iscsi_prio = -1;
	int default_prio = -1;
	int i;

	bfa_trc(tx_mod->bna, cee_linkup);
	bfa_trc(tx_mod->bna, prio_map);
	bfa_trc(tx_mod->bna, iscsi_prio_map);

	/* Select the priority map */
	if (!cee_linkup) {
		/* No CEE. Use all priorities */
		prio_map = BFI_TX_PRIO_MAP_ALL;
		iscsi_prio_map = 0;
		default_prio = 0;
	} else {
		/* Select default priority */
		for (i = 0; i < BFI_TX_MAX_PRIO; i++) {
			if ((prio_map >> i) & 0x1) {
				default_prio = i;
				break;
			}
		}

		/* Derive iscsi priority */
		if (iscsi_prio_map) {
			for (i = 0; i < BFI_TX_MAX_PRIO; i++) {
				if ((iscsi_prio_map >> i) & 0x1) {
					iscsi_prio = i;
					break;
				}
			}
		}

		/**
		 * Network traffic priority map is a superset of iSCSI and
		 * non iSCSI traffic. We are only giving a 'lift' to iSCSI
		 * traffic by redirecting it to iSCSI priority. Other NW
		 * traffic is still allowed to use iSCSI priority
		 */
		prio_map |= iscsi_prio_map;
	}

	if ((prio_map != tx_mod->prio_map) ||
	    (default_prio != tx_mod->default_prio) ||
	    (iscsi_prio != tx_mod->iscsi_prio))
		need_txq_reconfig = 1;

	tx_mod->prio_map = prio_map;
	tx_mod->default_prio = default_prio;
	tx_mod->iscsi_over_cee = (iscsi_prio_map ? BNA_STATUS_T_ENABLED :
				BNA_STATUS_T_DISABLED);
	tx_mod->iscsi_prio = iscsi_prio;
	tx_mod->prio_reconfigured = need_txq_reconfig;

	bfa_trc(tx_mod->bna, tx_mod->prio_map);
	bfa_trc(tx_mod->bna, tx_mod->default_prio);
	bfa_trc(tx_mod->bna, tx_mod->iscsi_over_cee);
	bfa_trc(tx_mod->bna, tx_mod->iscsi_prio);
	bfa_trc(tx_mod->bna, tx_mod->prio_reconfigured);

	/* Reconfigure the TxQs */
	if (need_txq_reconfig) {
		bfa_q_iter(&tx_mod->tx_active_q, qe) {
			tx = (struct bna_tx_s *)qe;
			bna_tx_prio_changed(tx);
		}
	}
}
