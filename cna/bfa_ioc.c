/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#include <cna/bfa_ioc.h>
#include <bfi/bfi_reg.h>
#include <aen/bfa_aen.h>
#include <log/bfa_log_hal.h>
#include <defs/bfa_defs.h>
#include <cna/bfa_flash_raw.h>
#include <defs/bfa_defs_flash.h>

BFA_TRC_FILE(CNA, IOC);

/**
 * IOC local definitions
 */
#define BFA_IOC_TOV		4000	/* msecs */
#define BFA_IOC_HWSEM_TOV	500	/* msecs */
#define BFA_IOC_HB_TOV		500	/* msecs */
#define BFA_IOC_TOV_RECOVER	BFA_IOC_HB_TOV
#define BFA_IOC_POLL_TOV	BFA_TIMER_FREQ

#define bfa_ioc_timer_start(__ioc)					\
	bfa_timer_begin((__ioc)->timer_mod, &(__ioc)->ioc_timer,	\
			bfa_ioc_timeout, (__ioc), BFA_IOC_TOV)
#define bfa_ioc_timer_stop(__ioc)   bfa_timer_stop(&(__ioc)->ioc_timer)

#define bfa_hb_timer_start(__ioc)					\
	bfa_timer_begin((__ioc)->timer_mod, &(__ioc)->hb_timer,		\
			bfa_ioc_hb_check, (__ioc), BFA_IOC_HB_TOV)
#define bfa_hb_timer_stop(__ioc)	bfa_timer_stop(&(__ioc)->hb_timer)

#define BFA_DBG_FWTRC_ENTS	(BFI_IOC_TRC_ENTS)
#define BFA_DBG_FWTRC_LEN					\
	(BFA_DBG_FWTRC_ENTS * sizeof(struct bfa_trc_s) +	\
	 (sizeof(struct bfa_trc_mod_s) -			\
	  BFA_TRC_MAX * sizeof(struct bfa_trc_s)))
#define BFA_DBG_FWTRC_OFF(_fn)	(BFI_IOC_TRC_OFF + BFA_DBG_FWTRC_LEN * (_fn))

#define bfa_ioc_state_disabled(__sm)		\
	(((__sm) == BFI_IOC_UNINIT) ||		\
	 ((__sm) == BFI_IOC_INITING) ||		\
	 ((__sm) == BFI_IOC_HWINIT) ||		\
	 ((__sm) == BFI_IOC_DISABLED) ||	\
	 ((__sm) == BFI_IOC_FAIL) ||		\
	 ((__sm) == BFI_IOC_CFG_DISABLED))

/**
 * Asic specific macros : see bfa_hw_cb.c and bfa_hw_ct.c for details.
 */

#define bfa_ioc_firmware_lock(__ioc)		\
			((__ioc)->ioc_hwif->ioc_firmware_lock(__ioc))
#define bfa_ioc_firmware_unlock(__ioc)		\
			((__ioc)->ioc_hwif->ioc_firmware_unlock(__ioc))
#define bfa_ioc_reg_init(__ioc) ((__ioc)->ioc_hwif->ioc_reg_init(__ioc))
#define bfa_ioc_map_port(__ioc) ((__ioc)->ioc_hwif->ioc_map_port(__ioc))
#define bfa_ioc_notify_fail(__ioc)		\
			((__ioc)->ioc_hwif->ioc_notify_fail(__ioc))
#define bfa_ioc_sync_start(__ioc)		\
			((__ioc)->ioc_hwif->ioc_sync_start(__ioc))
#define bfa_ioc_sync_join(__ioc)		\
			((__ioc)->ioc_hwif->ioc_sync_join(__ioc))
#define bfa_ioc_sync_leave(__ioc)		\
			((__ioc)->ioc_hwif->ioc_sync_leave(__ioc))
#define bfa_ioc_sync_ack(__ioc)			\
			((__ioc)->ioc_hwif->ioc_sync_ack(__ioc))
#define bfa_ioc_sync_complete(__ioc)		\
			((__ioc)->ioc_hwif->ioc_sync_complete(__ioc))
#define bfa_ioc_set_cur_ioc_fwstate(__ioc, __fwstate)		\
			((__ioc)->ioc_hwif->ioc_set_fwstate(__ioc, __fwstate))
#define bfa_ioc_get_cur_ioc_fwstate(__ioc)		\
			((__ioc)->ioc_hwif->ioc_get_fwstate(__ioc))
#define bfa_ioc_set_alt_ioc_fwstate(__ioc, __fwstate)		\
		((__ioc)->ioc_hwif->ioc_set_alt_fwstate(__ioc, __fwstate))
#define bfa_ioc_get_alt_ioc_fwstate(__ioc)		\
			((__ioc)->ioc_hwif->ioc_get_alt_fwstate(__ioc))

#define bfa_ioc_mbox_cmd_pending(__ioc)		\
			(!bfa_q_is_empty(&((__ioc)->mbox_mod.cmd_q)) || \
			bfa_reg_read((__ioc)->ioc_regs.hfn_mbox_cmd))

bfa_boolean_t bfa_auto_recover = BFA_TRUE;

/*
 * forward declarations
 */
static void bfa_ioc_hw_sem_init(struct bfa_ioc_s *ioc);
static void bfa_ioc_hw_sem_get(struct bfa_ioc_s *ioc);
static void bfa_ioc_hw_sem_get_cancel(struct bfa_ioc_s *ioc);
static void bfa_ioc_hwinit(struct bfa_ioc_s *ioc, bfa_boolean_t force);
static void bfa_ioc_timeout(void *ioc);
static void bfa_ioc_poll_fwinit(struct bfa_ioc_s *ioc);
static void bfa_ioc_send_enable(struct bfa_ioc_s *ioc);
static void bfa_ioc_send_disable(struct bfa_ioc_s *ioc);
static void bfa_ioc_send_getattr(struct bfa_ioc_s *ioc);
static void bfa_ioc_hb_monitor(struct bfa_ioc_s *ioc);
static void bfa_ioc_hb_stop(struct bfa_ioc_s *ioc);
static void bfa_ioc_reset(struct bfa_ioc_s *ioc, bfa_boolean_t force);
static void bfa_ioc_mbox_poll(struct bfa_ioc_s *ioc);
static void bfa_ioc_mbox_flush(struct bfa_ioc_s *ioc);
static void bfa_ioc_recover(struct bfa_ioc_s *ioc);
static void bfa_ioc_event_notify(struct bfa_ioc_s *, enum bfa_ioc_event_e);
static void bfa_ioc_disable_comp(struct bfa_ioc_s *ioc);
static void bfa_ioc_lpu_stop(struct bfa_ioc_s *ioc);
static void bfa_ioc_fail_notify(struct bfa_ioc_s *ioc);
static void bfa_ioc_pf_enabled(struct bfa_ioc_s *ioc);
static void bfa_ioc_pf_disabled(struct bfa_ioc_s *ioc);
static void bfa_ioc_pf_failed(struct bfa_ioc_s *ioc);
static void bfa_ioc_pf_hwfailed(struct bfa_ioc_s *ioc);
static void bfa_ioc_pf_fwmismatch(struct bfa_ioc_s *ioc);
static void bfa_ioc_fwver_clear(struct bfa_ioc_s *ioc);

/**
 * @ig hal_ioc_sm
 * @{
 * @img ioc_sm.jpg
 */

/**
 * IOC state machine definitions/declarations
 */
enum ioc_event {
	IOC_E_RESET		= 1,	/*!< IOC reset request		*/
	IOC_E_ENABLE		= 2,	/*!< IOC enable request		*/
	IOC_E_DISABLE		= 3,	/*!< IOC disable request	*/
	IOC_E_DETACH		= 4,	/*!< driver detach cleanup	*/
	IOC_E_ENABLED		= 5,	/*!< f/w enabled		*/
	IOC_E_FWRSP_GETATTR	= 6,	/*!< IOC get attribute response	*/
	IOC_E_DISABLED		= 7,	/*!< f/w disabled		*/
	IOC_E_PFFAILED		= 8,	/*!< failure notice by iocpf sm	*/
	IOC_E_HBFAIL		= 9,	/*!< heartbeat failure		*/
	IOC_E_HWERROR		= 10,	/*!< hardware error interrupt	*/
	IOC_E_TIMEOUT		= 11,	/*!< timeout 			*/
	IOC_E_HWFAILED		= 12,	/*!< PCI mapping failure notice	*/
};

bfa_fsm_state_decl(bfa_ioc, uninit, struct bfa_ioc_s, enum ioc_event);
bfa_fsm_state_decl(bfa_ioc, reset, struct bfa_ioc_s, enum ioc_event);
bfa_fsm_state_decl(bfa_ioc, enabling, struct bfa_ioc_s, enum ioc_event);
bfa_fsm_state_decl(bfa_ioc, getattr, struct bfa_ioc_s, enum ioc_event);
bfa_fsm_state_decl(bfa_ioc, op, struct bfa_ioc_s, enum ioc_event);
bfa_fsm_state_decl(bfa_ioc, fail_retry, struct bfa_ioc_s, enum ioc_event);
bfa_fsm_state_decl(bfa_ioc, fail, struct bfa_ioc_s, enum ioc_event);
bfa_fsm_state_decl(bfa_ioc, disabling, struct bfa_ioc_s, enum ioc_event);
bfa_fsm_state_decl(bfa_ioc, disabled, struct bfa_ioc_s, enum ioc_event);
bfa_fsm_state_decl(bfa_ioc, hwfail, struct bfa_ioc_s, enum ioc_event);

static struct bfa_sm_table_s ioc_sm_table[] = {
	{BFA_SM(bfa_ioc_sm_uninit), BFA_IOC_UNINIT},
	{BFA_SM(bfa_ioc_sm_reset), BFA_IOC_RESET},
	{BFA_SM(bfa_ioc_sm_enabling), BFA_IOC_ENABLING},
	{BFA_SM(bfa_ioc_sm_getattr), BFA_IOC_GETATTR},
	{BFA_SM(bfa_ioc_sm_op), BFA_IOC_OPERATIONAL},
	{BFA_SM(bfa_ioc_sm_fail_retry), BFA_IOC_INITFAIL},
	{BFA_SM(bfa_ioc_sm_fail), BFA_IOC_FAIL},
	{BFA_SM(bfa_ioc_sm_disabling), BFA_IOC_DISABLING},
	{BFA_SM(bfa_ioc_sm_disabled), BFA_IOC_DISABLED},
	{BFA_SM(bfa_ioc_sm_hwfail), BFA_IOC_HWFAIL},
};

/**
 * IOCPF state machine definitions/declarations
 */

#define bfa_iocpf_timer_start(__ioc)					\
	bfa_timer_begin((__ioc)->timer_mod, &(__ioc)->ioc_timer,	\
			bfa_iocpf_timeout, (__ioc), BFA_IOC_TOV)
#define bfa_iocpf_timer_stop(__ioc)	bfa_timer_stop(&(__ioc)->ioc_timer)

#define bfa_iocpf_poll_timer_start(__ioc)				\
	bfa_timer_begin((__ioc)->timer_mod, &(__ioc)->ioc_timer,	\
			bfa_iocpf_poll_timeout, (__ioc), BFA_IOC_POLL_TOV)

#define bfa_sem_timer_start(__ioc)					\
	bfa_timer_begin((__ioc)->timer_mod, &(__ioc)->sem_timer,	\
			bfa_iocpf_sem_timeout, (__ioc), BFA_IOC_HWSEM_TOV)
#define bfa_sem_timer_stop(__ioc)	bfa_timer_stop(&(__ioc)->sem_timer)

/*
 * Forward declareations for iocpf state machine
 */
static void bfa_iocpf_enable(struct bfa_ioc_s *ioc);
static void bfa_iocpf_disable(struct bfa_ioc_s *ioc);
static void bfa_iocpf_fail(struct bfa_ioc_s *ioc);
static void bfa_iocpf_initfail(struct bfa_ioc_s *ioc);
static void bfa_iocpf_getattrfail(struct bfa_ioc_s *ioc);
static void bfa_iocpf_stop(struct bfa_ioc_s *ioc);
static void bfa_iocpf_timeout(void *ioc_arg);
static void bfa_iocpf_sem_timeout(void *ioc_arg);
static void bfa_iocpf_poll_timeout(void *ioc_arg);

/**
 * IOCPF state machine events
 */
enum iocpf_event {
	IOCPF_E_ENABLE		= 1,	/*!< IOCPF enable request	*/
	IOCPF_E_DISABLE		= 2,	/*!< IOCPF disable request	*/
	IOCPF_E_STOP		= 3,	/*!< stop on driver detach	*/
	IOCPF_E_FWREADY		= 4,	/*!< f/w initialization done	*/
	IOCPF_E_FWRSP_ENABLE	= 5,	/*!< enable f/w response	*/
	IOCPF_E_FWRSP_DISABLE	= 6,	/*!< disable f/w response	*/
	IOCPF_E_FAIL		= 7,	/*!< failure notice by ioc sm	*/
	IOCPF_E_INITFAIL	= 8,	/*!< init fail notice by ioc sm	*/
	IOCPF_E_GETATTRFAIL	= 9,	/*!< init fail notice by ioc sm	*/
	IOCPF_E_SEMLOCKED	= 10,	/*!< h/w semaphore is locked	*/
	IOCPF_E_TIMEOUT		= 11,	/*!< f/w response timeout	*/
	IOCPF_E_SEM_ERROR	= 12,	/*!< h/w sem mapping error	*/
};

/**
 * IOCPF states
 */
enum bfa_iocpf_state {
	BFA_IOCPF_RESET		= 1,	/*!< IOC is in reset state */
	BFA_IOCPF_SEMWAIT	= 2,	/*!< Waiting for IOC h/w semaphore */
	BFA_IOCPF_HWINIT	= 3,	/*!< IOC h/w is being initialized */
	BFA_IOCPF_READY		= 4,	/*!< IOCPF is initialized */
	BFA_IOCPF_INITFAIL	= 5,	/*!< IOCPF failed */
	BFA_IOCPF_FAIL		= 6,	/*!< IOCPF failed */
	BFA_IOCPF_DISABLING	= 7,	/*!< IOCPF is being disabled */
	BFA_IOCPF_DISABLED	= 8,	/*!< IOCPF is disabled */
	BFA_IOCPF_FWMISMATCH	= 9,	/*!< IOC f/w different from drivers */
};
typedef enum bfa_iocpf_state bfa_iocpf_state_t;

bfa_fsm_state_decl(bfa_iocpf, reset, struct bfa_iocpf_s, enum iocpf_event);
bfa_fsm_state_decl(bfa_iocpf, fwcheck, struct bfa_iocpf_s, enum iocpf_event);
bfa_fsm_state_decl(bfa_iocpf, mismatch, struct bfa_iocpf_s, enum iocpf_event);
bfa_fsm_state_decl(bfa_iocpf, semwait, struct bfa_iocpf_s, enum iocpf_event);
bfa_fsm_state_decl(bfa_iocpf, hwinit, struct bfa_iocpf_s, enum iocpf_event);
bfa_fsm_state_decl(bfa_iocpf, enabling, struct bfa_iocpf_s, enum iocpf_event);
bfa_fsm_state_decl(bfa_iocpf, ready, struct bfa_iocpf_s, enum iocpf_event);
bfa_fsm_state_decl(bfa_iocpf, initfail_sync, struct bfa_iocpf_s,
						enum iocpf_event);
bfa_fsm_state_decl(bfa_iocpf, initfail, struct bfa_iocpf_s, enum iocpf_event);
bfa_fsm_state_decl(bfa_iocpf, fail_sync, struct bfa_iocpf_s, enum iocpf_event);
bfa_fsm_state_decl(bfa_iocpf, fail, struct bfa_iocpf_s, enum iocpf_event);
bfa_fsm_state_decl(bfa_iocpf, disabling, struct bfa_iocpf_s, enum iocpf_event);
bfa_fsm_state_decl(bfa_iocpf, disabling_sync, struct bfa_iocpf_s,
						enum iocpf_event);
bfa_fsm_state_decl(bfa_iocpf, disabled, struct bfa_iocpf_s, enum iocpf_event);

static struct bfa_sm_table_s iocpf_sm_table[] = {
	{BFA_SM(bfa_iocpf_sm_reset), BFA_IOCPF_RESET},
	{BFA_SM(bfa_iocpf_sm_fwcheck), BFA_IOCPF_FWMISMATCH},
	{BFA_SM(bfa_iocpf_sm_mismatch), BFA_IOCPF_FWMISMATCH},
	{BFA_SM(bfa_iocpf_sm_semwait), BFA_IOCPF_SEMWAIT},
	{BFA_SM(bfa_iocpf_sm_hwinit), BFA_IOCPF_HWINIT},
	{BFA_SM(bfa_iocpf_sm_enabling), BFA_IOCPF_HWINIT},
	{BFA_SM(bfa_iocpf_sm_ready), BFA_IOCPF_READY},
	{BFA_SM(bfa_iocpf_sm_initfail_sync), BFA_IOCPF_INITFAIL},
	{BFA_SM(bfa_iocpf_sm_initfail), BFA_IOCPF_INITFAIL},
	{BFA_SM(bfa_iocpf_sm_fail_sync), BFA_IOCPF_FAIL},
	{BFA_SM(bfa_iocpf_sm_fail), BFA_IOCPF_FAIL},
	{BFA_SM(bfa_iocpf_sm_disabling), BFA_IOCPF_DISABLING},
	{BFA_SM(bfa_iocpf_sm_disabling_sync), BFA_IOCPF_DISABLING},
	{BFA_SM(bfa_iocpf_sm_disabled), BFA_IOCPF_DISABLED},
};

/**
 * IOC State Machine
 */

/**
 * Beginning state. IOC uninit state.
 */

static void
bfa_ioc_sm_uninit_entry(struct bfa_ioc_s *ioc)
{
}

/**
 * IOC is in uninit state.
 */
static void
bfa_ioc_sm_uninit(struct bfa_ioc_s *ioc, enum ioc_event event)
{
	bfa_trc(ioc, event);

	switch (event) {
	case IOC_E_RESET:
		bfa_fsm_set_state(ioc, bfa_ioc_sm_reset);
		break;

	default:
		bfa_sm_fault(ioc, event);
	}
}
/**
 * Reset entry actions -- initialize state machine
 */
static void
bfa_ioc_sm_reset_entry(struct bfa_ioc_s *ioc)
{
	bfa_fsm_set_state(&ioc->iocpf, bfa_iocpf_sm_reset);
}

/**
 * IOC is in reset state.
 */
static void
bfa_ioc_sm_reset(struct bfa_ioc_s *ioc, enum ioc_event event)
{
	bfa_trc(ioc, event);

	switch (event) {
	case IOC_E_ENABLE:
		bfa_fsm_set_state(ioc, bfa_ioc_sm_enabling);
		break;

	case IOC_E_DISABLE:
		bfa_ioc_disable_comp(ioc);
		break;

	case IOC_E_DETACH:
		bfa_fsm_set_state(ioc, bfa_ioc_sm_uninit);
		break;

	default:
		bfa_sm_fault(ioc, event);
	}
}

/**
 *
 */
static void
bfa_ioc_sm_enabling_entry(struct bfa_ioc_s *ioc)
{
	bfa_iocpf_enable(ioc);
}

/**
 * Host IOC function is being enabled, awaiting response from firmware.
 * Semaphore is acquired.
 */
static void
bfa_ioc_sm_enabling(struct bfa_ioc_s *ioc, enum ioc_event event)
{
	bfa_trc(ioc, event);

	switch (event) {
	case IOC_E_ENABLED:
		bfa_fsm_set_state(ioc, bfa_ioc_sm_getattr);
		break;

	case IOC_E_PFFAILED:
		/* !!! fall through !!! */
	case IOC_E_HWERROR:
		ioc->cbfn->enable_cbfn(ioc->bfa, BFA_STATUS_IOC_FAILURE);
		bfa_fsm_set_state(ioc, bfa_ioc_sm_fail);
		if (event != IOC_E_PFFAILED)
			bfa_iocpf_initfail(ioc);
		break;

	case IOC_E_HWFAILED:
		ioc->cbfn->enable_cbfn(ioc->bfa, BFA_STATUS_IOC_FAILURE);
		bfa_fsm_set_state(ioc, bfa_ioc_sm_hwfail);
		break;

	case IOC_E_DISABLE:
		bfa_fsm_set_state(ioc, bfa_ioc_sm_disabling);
		break;

	case IOC_E_DETACH:
		bfa_fsm_set_state(ioc, bfa_ioc_sm_uninit);
		bfa_iocpf_stop(ioc);
		break;

	case IOC_E_ENABLE:
		break;

	default:
		bfa_sm_fault(ioc, event);
	}
}

/**
 *
 */
static void
bfa_ioc_sm_getattr_entry(struct bfa_ioc_s *ioc)
{
	bfa_ioc_timer_start(ioc);
	bfa_ioc_send_getattr(ioc);
}

/**
 * @brief
 * IOC configuration in progress. Timer is active.
 */
static void
bfa_ioc_sm_getattr(struct bfa_ioc_s *ioc, enum ioc_event event)
{
	bfa_trc(ioc, event);

	switch (event) {
	case IOC_E_FWRSP_GETATTR:
		bfa_ioc_timer_stop(ioc);
		bfa_fsm_set_state(ioc, bfa_ioc_sm_op);
		break;

	case IOC_E_PFFAILED:
	case IOC_E_HWERROR:
		bfa_ioc_timer_stop(ioc);
		/* !!! fall through !!! */
	case IOC_E_TIMEOUT:
		ioc->cbfn->enable_cbfn(ioc->bfa, BFA_STATUS_IOC_FAILURE);
		bfa_fsm_set_state(ioc, bfa_ioc_sm_fail);
		if (event != IOC_E_PFFAILED)
			bfa_iocpf_getattrfail(ioc);
		break;

	case IOC_E_DISABLE:
		bfa_ioc_timer_stop(ioc);
		bfa_fsm_set_state(ioc, bfa_ioc_sm_disabling);
		break;

	case IOC_E_ENABLE:
		break;

	default:
		bfa_sm_fault(ioc, event);
	}
}

/**
 *
 */
static void
bfa_ioc_sm_op_entry(struct bfa_ioc_s *ioc)
{
	ioc->cbfn->enable_cbfn(ioc->bfa, BFA_STATUS_OK);
	bfa_ioc_event_notify(ioc, BFA_IOC_E_ENABLED);
	bfa_ioc_hb_monitor(ioc);
	bfa_ioc_aen_post(ioc, BFA_IOC_AEN_ENABLE);
}

static void
bfa_ioc_sm_op(struct bfa_ioc_s *ioc, enum ioc_event event)
{
	bfa_trc(ioc, event);

	switch (event) {
	case IOC_E_ENABLE:
		break;

	case IOC_E_DISABLE:
		bfa_ioc_hb_stop(ioc);
		bfa_fsm_set_state(ioc, bfa_ioc_sm_disabling);
		break;

	case IOC_E_PFFAILED:
	case IOC_E_HWERROR:
		bfa_ioc_hb_stop(ioc);
		/* !!! fall through !!! */
	case IOC_E_HBFAIL:
		if (ioc->iocpf.auto_recover)
			bfa_fsm_set_state(ioc, bfa_ioc_sm_fail_retry);
		else
			bfa_fsm_set_state(ioc, bfa_ioc_sm_fail);

		bfa_ioc_fail_notify(ioc);

		if (event != IOC_E_PFFAILED)
			bfa_iocpf_fail(ioc);
		break;

	default:
		bfa_sm_fault(ioc, event);
	}
}

/**
 *
 */
static void
bfa_ioc_sm_disabling_entry(struct bfa_ioc_s *ioc)
{
	bfa_ioc_aen_post(ioc, BFA_IOC_AEN_DISABLE);
	bfa_iocpf_disable(ioc);
}

/**
 * IOC is being disabled
 */
static void
bfa_ioc_sm_disabling(struct bfa_ioc_s *ioc, enum ioc_event event)
{
	bfa_trc(ioc, event);

	switch (event) {
	case IOC_E_DISABLED:
		bfa_fsm_set_state(ioc, bfa_ioc_sm_disabled);
		break;

	case IOC_E_HWERROR:
		/*
		 * No state change.  Will move to disabled state
		 * after iocpf sm completes failure processing and
		 * moves to disabled state.
		 */
		bfa_iocpf_fail(ioc);
		break;

	case IOC_E_HWFAILED:
		bfa_fsm_set_state(ioc, bfa_ioc_sm_hwfail);
		bfa_ioc_disable_comp(ioc);
		break;

	default:
		bfa_sm_fault(ioc, event);
	}
}

/**
 * IOC disable completion entry.
 */
static void
bfa_ioc_sm_disabled_entry(struct bfa_ioc_s *ioc)
{
	bfa_ioc_disable_comp(ioc);
}

static void
bfa_ioc_sm_disabled(struct bfa_ioc_s *ioc, enum ioc_event event)
{
	bfa_trc(ioc, event);

	switch (event) {
	case IOC_E_ENABLE:
		bfa_fsm_set_state(ioc, bfa_ioc_sm_enabling);
		break;

	case IOC_E_DISABLE:
		ioc->cbfn->disable_cbfn(ioc->bfa);
		break;

	case IOC_E_DETACH:
		bfa_fsm_set_state(ioc, bfa_ioc_sm_uninit);
		bfa_iocpf_stop(ioc);
		break;

	default:
		bfa_sm_fault(ioc, event);
	}
}

/**
 *
 */
static void
bfa_ioc_sm_fail_retry_entry(struct bfa_ioc_s *ioc)
{
	bfa_trc(ioc, 0);
}

/**
 * @brief
 * Hardware initialization retry.
 */
static void
bfa_ioc_sm_fail_retry(struct bfa_ioc_s *ioc, enum ioc_event event)
{
	bfa_trc(ioc, event);

	switch (event) {
	case IOC_E_ENABLED:
		bfa_fsm_set_state(ioc, bfa_ioc_sm_getattr);
		break;

	case IOC_E_PFFAILED:
	case IOC_E_HWERROR:
		/**
		 * Initialization retry failed.
		 */
		ioc->cbfn->enable_cbfn(ioc->bfa, BFA_STATUS_IOC_FAILURE);
		bfa_fsm_set_state(ioc, bfa_ioc_sm_fail);
		if (event != IOC_E_PFFAILED)
			bfa_iocpf_initfail(ioc);
		break;

	case IOC_E_HWFAILED:
		ioc->cbfn->enable_cbfn(ioc->bfa, BFA_STATUS_IOC_FAILURE);
		bfa_fsm_set_state(ioc, bfa_ioc_sm_hwfail);
		break;

	case IOC_E_ENABLE:
		break;

	case IOC_E_DISABLE:
		bfa_fsm_set_state(ioc, bfa_ioc_sm_disabling);
		break;

	case IOC_E_DETACH:
		bfa_fsm_set_state(ioc, bfa_ioc_sm_uninit);
		bfa_iocpf_stop(ioc);
		break;

	default:
		bfa_sm_fault(ioc, event);
	}
}

/**
 *
 */
static void
bfa_ioc_sm_fail_entry(struct bfa_ioc_s *ioc)
{
	bfa_trc(ioc, 0);
}

/**
 * @brief
 * IOC failure.
 */
static void
bfa_ioc_sm_fail(struct bfa_ioc_s *ioc, enum ioc_event event)
{
	bfa_trc(ioc, event);

	switch (event) {

	case IOC_E_ENABLE:
		ioc->cbfn->enable_cbfn(ioc->bfa, BFA_STATUS_IOC_FAILURE);
		break;

	case IOC_E_DISABLE:
		bfa_fsm_set_state(ioc, bfa_ioc_sm_disabling);
		break;

	case IOC_E_DETACH:
		bfa_fsm_set_state(ioc, bfa_ioc_sm_uninit);
		bfa_iocpf_stop(ioc);
		break;

	case IOC_E_HWERROR:
		/*
		 * HB failure notification, ignore.
		 */
		break;

	case IOC_E_HWFAILED:
		break;

	default:
		bfa_sm_fault(ioc, event);
	}
}

/**
 *
 */
static void
bfa_ioc_sm_hwfail_entry(struct bfa_ioc_s *ioc)
{
	bfa_trc(ioc, 0);
}

/**
 * @brief
 * IOC failure.
 */
static void
bfa_ioc_sm_hwfail(struct bfa_ioc_s *ioc, enum ioc_event event)
{
	bfa_trc(ioc, event);

	switch (event) {

	case IOC_E_ENABLE:
		ioc->cbfn->enable_cbfn(ioc->bfa, BFA_STATUS_IOC_FAILURE);
		break;

	case IOC_E_DISABLE:
		ioc->cbfn->disable_cbfn(ioc->bfa);
		break;

	case IOC_E_DETACH:
		bfa_fsm_set_state(ioc, bfa_ioc_sm_uninit);
		break;

	case IOC_E_HWERROR:
		/* Ignore - already in hwfail state */
		break;

	default:
		bfa_sm_fault(ioc, event);
	}
}

/**
 * @}
 */

/**
 * IOCPF State Machine
 */


/**
 * Reset entry actions -- initialize state machine
 */
static void
bfa_iocpf_sm_reset_entry(struct bfa_iocpf_s *iocpf)
{
	iocpf->fw_mismatch_notified = BFA_FALSE;
	iocpf->auto_recover = bfa_auto_recover;
}

/**
 * Beginning state. IOC is in reset state.
 */
static void
bfa_iocpf_sm_reset(struct bfa_iocpf_s *iocpf, enum iocpf_event event)
{
	struct bfa_ioc_s *ioc = iocpf->ioc;

	bfa_trc(ioc, event);

	switch (event) {
	case IOCPF_E_ENABLE:
		bfa_fsm_set_state(iocpf, bfa_iocpf_sm_fwcheck);
		break;

	case IOCPF_E_STOP:
		break;

	default:
		bfa_sm_fault(ioc, event);
	}
}

/**
 * Semaphore should be acquired for version check.
 */
static void
bfa_iocpf_sm_fwcheck_entry(struct bfa_iocpf_s *iocpf)
{
	bfa_ioc_hw_sem_init(iocpf->ioc);
	bfa_ioc_hw_sem_get(iocpf->ioc);
}

/**
 * Awaiting h/w semaphore to continue with version check.
 */
static void
bfa_iocpf_sm_fwcheck(struct bfa_iocpf_s *iocpf, enum iocpf_event event)
{
	struct bfa_ioc_s *ioc = iocpf->ioc;

	bfa_trc(ioc, event);

	switch (event) {
	case IOCPF_E_SEMLOCKED:
		if (bfa_ioc_firmware_lock(ioc)) {
			if (bfa_ioc_sync_start(ioc)) {
				bfa_ioc_sync_join(ioc);
				bfa_fsm_set_state(iocpf, bfa_iocpf_sm_hwinit);
			} else {
				bfa_ioc_firmware_unlock(ioc);
				bfa_ioc_hw_sem_release(ioc);
				bfa_sem_timer_start(ioc);
			}
		} else {
			bfa_ioc_hw_sem_release(ioc);
			bfa_fsm_set_state(iocpf, bfa_iocpf_sm_mismatch);
		}
		break;

	case IOCPF_E_SEM_ERROR:
		bfa_fsm_set_state(iocpf, bfa_iocpf_sm_fail);
		bfa_ioc_pf_hwfailed(ioc);
		break;

	case IOCPF_E_DISABLE:
		bfa_ioc_hw_sem_get_cancel(ioc);
		bfa_fsm_set_state(iocpf, bfa_iocpf_sm_reset);
		bfa_ioc_pf_disabled(ioc);
		break;

	case IOCPF_E_STOP:
		bfa_ioc_hw_sem_get_cancel(ioc);
		bfa_fsm_set_state(iocpf, bfa_iocpf_sm_reset);
		break;

	default:
		bfa_sm_fault(ioc, event);
	}
}

/**
 * Notify enable completion callback and generate mismatch AEN.
 */
static void
bfa_iocpf_sm_mismatch_entry(struct bfa_iocpf_s *iocpf)
{
	/*
	 * Call only the first time sm enters fwmismatch state.
	 */
	if (iocpf->fw_mismatch_notified == BFA_FALSE)
		bfa_ioc_pf_fwmismatch(iocpf->ioc);

	iocpf->fw_mismatch_notified = BFA_TRUE;
	bfa_iocpf_timer_start(iocpf->ioc);
}

/**
 * Awaiting firmware version match.
 */
static void
bfa_iocpf_sm_mismatch(struct bfa_iocpf_s *iocpf, enum iocpf_event event)
{
	struct bfa_ioc_s *ioc = iocpf->ioc;

	bfa_trc(ioc, event);

	switch (event) {
	case IOCPF_E_TIMEOUT:
		bfa_fsm_set_state(iocpf, bfa_iocpf_sm_fwcheck);
		break;

	case IOCPF_E_DISABLE:
		bfa_iocpf_timer_stop(ioc);
		bfa_fsm_set_state(iocpf, bfa_iocpf_sm_reset);
		bfa_ioc_pf_disabled(ioc);
		break;

	case IOCPF_E_STOP:
		bfa_iocpf_timer_stop(ioc);
		bfa_fsm_set_state(iocpf, bfa_iocpf_sm_reset);
		break;

	default:
		bfa_sm_fault(ioc, event);
	}
}

/**
 * Request for semaphore.
 */
static void
bfa_iocpf_sm_semwait_entry(struct bfa_iocpf_s *iocpf)
{
	bfa_ioc_hw_sem_get(iocpf->ioc);
}

/**
 * Awaiting semaphore for h/w initialzation.
 */
static void
bfa_iocpf_sm_semwait(struct bfa_iocpf_s *iocpf, enum iocpf_event event)
{
	struct bfa_ioc_s *ioc = iocpf->ioc;

	bfa_trc(ioc, event);

	switch (event) {
	case IOCPF_E_SEMLOCKED:
		if (bfa_ioc_sync_complete(ioc)) {
			bfa_ioc_sync_join(ioc);
			bfa_fsm_set_state(iocpf, bfa_iocpf_sm_hwinit);
		} else {
			bfa_ioc_hw_sem_release(ioc);
			bfa_sem_timer_start(ioc);
		}
		break;

	case IOCPF_E_SEM_ERROR:
		bfa_fsm_set_state(iocpf, bfa_iocpf_sm_fail);
		bfa_ioc_pf_hwfailed(ioc);
		break;

	case IOCPF_E_DISABLE:
		bfa_ioc_hw_sem_get_cancel(ioc);
		bfa_fsm_set_state(iocpf, bfa_iocpf_sm_disabling_sync);
		break;

	default:
		bfa_sm_fault(ioc, event);
	}
}

/**
 *
 */
static void
bfa_iocpf_sm_hwinit_entry(struct bfa_iocpf_s *iocpf)
{
	iocpf->poll_time = 0;
	bfa_ioc_reset(iocpf->ioc, BFA_FALSE);
}

/**
 * @brief
 * Hardware is being initialized. Interrupts are enabled.
 * Holding hardware semaphore lock.
 */
static void
bfa_iocpf_sm_hwinit(struct bfa_iocpf_s *iocpf, enum iocpf_event event)
{
	struct bfa_ioc_s *ioc = iocpf->ioc;

	bfa_trc(ioc, event);

	switch (event) {
	case IOCPF_E_FWREADY:
		bfa_fsm_set_state(iocpf, bfa_iocpf_sm_enabling);
		break;

	case IOCPF_E_TIMEOUT:
		bfa_ioc_hw_sem_release(ioc);
		bfa_ioc_pf_failed(ioc);
		bfa_fsm_set_state(iocpf, bfa_iocpf_sm_initfail_sync);
		break;

	case IOCPF_E_DISABLE:
		bfa_iocpf_timer_stop(ioc);
		bfa_ioc_sync_leave(ioc);
		bfa_ioc_hw_sem_release(ioc);
		bfa_fsm_set_state(iocpf, bfa_iocpf_sm_disabled);
		break;

	default:
		bfa_sm_fault(ioc, event);
	}
}

/**
 *
 */
static void
bfa_iocpf_sm_enabling_entry(struct bfa_iocpf_s *iocpf)
{
	bfa_iocpf_timer_start(iocpf->ioc);
	/**
	 * Enable Interrupts before sending fw IOC ENABLE cmd.
	 */
	iocpf->ioc->cbfn->reset_cbfn(iocpf->ioc->bfa);
	bfa_ioc_send_enable(iocpf->ioc);
}

/**
 * Host IOC function is being enabled, awaiting response from firmware.
 * Semaphore is acquired.
 */
static void
bfa_iocpf_sm_enabling(struct bfa_iocpf_s *iocpf, enum iocpf_event event)
{
	struct bfa_ioc_s *ioc = iocpf->ioc;

	bfa_trc(ioc, event);

	switch (event) {
	case IOCPF_E_FWRSP_ENABLE:
		bfa_iocpf_timer_stop(ioc);
		bfa_ioc_hw_sem_release(ioc);
		bfa_fsm_set_state(iocpf, bfa_iocpf_sm_ready);
		break;

	case IOCPF_E_INITFAIL:
		bfa_iocpf_timer_stop(ioc);
		/*
		 * !!! fall through !!!
		 */

	case IOCPF_E_TIMEOUT:
		bfa_ioc_hw_sem_release(ioc);
		if (event == IOCPF_E_TIMEOUT)
			bfa_ioc_pf_failed(ioc);
		bfa_fsm_set_state(iocpf, bfa_iocpf_sm_initfail_sync);
		break;

	case IOCPF_E_DISABLE:
		bfa_iocpf_timer_stop(ioc);
		bfa_ioc_hw_sem_release(ioc);
		bfa_fsm_set_state(iocpf, bfa_iocpf_sm_disabling);
		break;

	default:
		bfa_sm_fault(ioc, event);
	}
}


/**
 *
 */
static void
bfa_iocpf_sm_ready_entry(struct bfa_iocpf_s *iocpf)
{
	bfa_ioc_pf_enabled(iocpf->ioc);
}

static void
bfa_iocpf_sm_ready(struct bfa_iocpf_s *iocpf, enum iocpf_event event)
{
	struct bfa_ioc_s *ioc = iocpf->ioc;

	bfa_trc(ioc, event);

	switch (event) {
	case IOCPF_E_DISABLE:
		bfa_fsm_set_state(iocpf, bfa_iocpf_sm_disabling);
		break;

	case IOCPF_E_GETATTRFAIL:
		bfa_fsm_set_state(iocpf, bfa_iocpf_sm_initfail_sync);
		break;

	case IOCPF_E_FAIL:
		bfa_fsm_set_state(iocpf, bfa_iocpf_sm_fail_sync);
		break;

	default:
		bfa_sm_fault(ioc, event);
	}
}

/**
 *
 */
static void
bfa_iocpf_sm_disabling_entry(struct bfa_iocpf_s *iocpf)
{
	bfa_iocpf_timer_start(iocpf->ioc);
	bfa_ioc_send_disable(iocpf->ioc);
}

/**
 * IOC is being disabled
 */
static void
bfa_iocpf_sm_disabling(struct bfa_iocpf_s *iocpf, enum iocpf_event event)
{
	struct bfa_ioc_s *ioc = iocpf->ioc;

	bfa_trc(ioc, event);

	switch (event) {
	case IOCPF_E_FWRSP_DISABLE:
		bfa_iocpf_timer_stop(ioc);
		bfa_fsm_set_state(iocpf, bfa_iocpf_sm_disabling_sync);
		break;

	case IOCPF_E_FAIL:
		bfa_iocpf_timer_stop(ioc);
		/*
		 * !!! fall through !!!
		 */

	case IOCPF_E_TIMEOUT:
		bfa_ioc_set_cur_ioc_fwstate(ioc, BFI_IOC_FAIL);
		bfa_fsm_set_state(iocpf, bfa_iocpf_sm_disabling_sync);
		break;

	case IOCPF_E_FWRSP_ENABLE:
		break;

	default:
		bfa_sm_fault(ioc, event);
	}
}

/**
 *
 */
static void
bfa_iocpf_sm_disabling_sync_entry(struct bfa_iocpf_s *iocpf)
{
	bfa_ioc_hw_sem_get(iocpf->ioc);
}

/**
 * IOC hb ack request is being removed.
 */
static void
bfa_iocpf_sm_disabling_sync(struct bfa_iocpf_s *iocpf, enum iocpf_event event)
{
	struct bfa_ioc_s *ioc = iocpf->ioc;

	bfa_trc(ioc, event);

	switch (event) {
	case IOCPF_E_SEMLOCKED:
		bfa_ioc_sync_leave(ioc);
		bfa_ioc_hw_sem_release(ioc);
		bfa_fsm_set_state(iocpf, bfa_iocpf_sm_disabled);
		break;

	case IOCPF_E_SEM_ERROR:
		bfa_fsm_set_state(iocpf, bfa_iocpf_sm_fail);
		bfa_ioc_pf_hwfailed(ioc);
		break;

	case IOCPF_E_FAIL:
		break;

	default:
		bfa_sm_fault(ioc, event);
	}
}

/**
 * IOC disable completion entry.
 */
static void
bfa_iocpf_sm_disabled_entry(struct bfa_iocpf_s *iocpf)
{
	bfa_ioc_mbox_flush(iocpf->ioc);
	bfa_ioc_pf_disabled(iocpf->ioc);
}

static void
bfa_iocpf_sm_disabled(struct bfa_iocpf_s *iocpf, enum iocpf_event event)
{
	struct bfa_ioc_s *ioc = iocpf->ioc;

	bfa_trc(ioc, event);

	switch (event) {
	case IOCPF_E_ENABLE:
		bfa_fsm_set_state(iocpf, bfa_iocpf_sm_semwait);
		break;

	case IOCPF_E_STOP:
		bfa_ioc_firmware_unlock(ioc);
		bfa_fsm_set_state(iocpf, bfa_iocpf_sm_reset);
		break;

	default:
		bfa_sm_fault(ioc, event);
	}
}

/**
 *
 */
static void
bfa_iocpf_sm_initfail_sync_entry(struct bfa_iocpf_s *iocpf)
{
	bfa_ioc_debug_save_ftrc(iocpf->ioc);
	bfa_ioc_hw_sem_get(iocpf->ioc);
}

/**
 * @brief
 * Hardware initialization failed.
 */
static void
bfa_iocpf_sm_initfail_sync(struct bfa_iocpf_s *iocpf, enum iocpf_event event)
{
	struct bfa_ioc_s *ioc = iocpf->ioc;

	bfa_trc(ioc, event);

	switch (event) {
	case IOCPF_E_SEMLOCKED:
		bfa_ioc_notify_fail(ioc);
		bfa_ioc_sync_leave(ioc);
		bfa_ioc_set_cur_ioc_fwstate(ioc, BFI_IOC_FAIL);
		bfa_ioc_hw_sem_release(ioc);
		bfa_fsm_set_state(iocpf, bfa_iocpf_sm_initfail);
		break;

	case IOCPF_E_SEM_ERROR:
		bfa_fsm_set_state(iocpf, bfa_iocpf_sm_fail);
		bfa_ioc_pf_hwfailed(ioc);
		break;

	case IOCPF_E_DISABLE:
		bfa_ioc_hw_sem_get_cancel(ioc);
		bfa_fsm_set_state(iocpf, bfa_iocpf_sm_disabling_sync);
		break;

	case IOCPF_E_STOP:
		bfa_ioc_hw_sem_get_cancel(ioc);
		bfa_ioc_firmware_unlock(ioc);
		bfa_fsm_set_state(iocpf, bfa_iocpf_sm_reset);
		break;

	case IOCPF_E_FAIL:
		break;

	default:
		bfa_sm_fault(ioc, event);
	}
}

/**
 *
 */
static void
bfa_iocpf_sm_initfail_entry(struct bfa_iocpf_s *iocpf)
{
	bfa_trc(iocpf->ioc, 0);
}

/**
 * @brief
 * Hardware initialization failed.
 */
static void
bfa_iocpf_sm_initfail(struct bfa_iocpf_s *iocpf, enum iocpf_event event)
{
	struct bfa_ioc_s *ioc = iocpf->ioc;

	bfa_trc(ioc, event);

	switch (event) {
	case IOCPF_E_DISABLE:
		bfa_fsm_set_state(iocpf, bfa_iocpf_sm_disabled);
		break;

	case IOCPF_E_STOP:
		bfa_ioc_firmware_unlock(ioc);
		bfa_fsm_set_state(iocpf, bfa_iocpf_sm_reset);
		break;

	default:
		bfa_sm_fault(ioc, event);
	}
}

/**
 *
 */
static void
bfa_iocpf_sm_fail_sync_entry(struct bfa_iocpf_s *iocpf)
{
	/**
	 * Mark IOC as failed in hardware and stop firmware.
	 */
	bfa_ioc_lpu_stop(iocpf->ioc);

	/**
	 * Flush any queued up mailbox requests.
	 */
	bfa_ioc_mbox_flush(iocpf->ioc);

	bfa_ioc_hw_sem_get(iocpf->ioc);
}

/**
 * @brief
 * IOC is in failed state.
 */
static void
bfa_iocpf_sm_fail_sync(struct bfa_iocpf_s *iocpf, enum iocpf_event event)
{
	struct bfa_ioc_s *ioc = iocpf->ioc;

	bfa_trc(ioc, event);

	switch (event) {
	case IOCPF_E_SEMLOCKED:
		bfa_ioc_sync_ack(ioc);
		bfa_ioc_notify_fail(ioc);
		if (!iocpf->auto_recover) {
			bfa_ioc_sync_leave(ioc);
			bfa_ioc_set_cur_ioc_fwstate(ioc, BFI_IOC_FAIL);
			bfa_ioc_hw_sem_release(ioc);
			bfa_fsm_set_state(iocpf, bfa_iocpf_sm_fail);
		} else {
			if (bfa_ioc_sync_complete(ioc))
				bfa_fsm_set_state(iocpf, bfa_iocpf_sm_hwinit);
			else {
				bfa_ioc_hw_sem_release(ioc);
				bfa_fsm_set_state(iocpf, bfa_iocpf_sm_semwait);
			}
		}
		break;

	case IOCPF_E_SEM_ERROR:
		bfa_fsm_set_state(iocpf, bfa_iocpf_sm_fail);
		bfa_ioc_pf_hwfailed(ioc);
		break;

	case IOCPF_E_DISABLE:
		bfa_ioc_hw_sem_get_cancel(ioc);
		bfa_fsm_set_state(iocpf, bfa_iocpf_sm_disabling_sync);
		break;

	case IOCPF_E_FAIL:
		break;

	default:
		bfa_sm_fault(ioc, event);
	}
}

/**
 *
 */
static void
bfa_iocpf_sm_fail_entry(struct bfa_iocpf_s *iocpf)
{
	bfa_trc(iocpf->ioc, 0);
}

/**
 * @brief
 * IOC is in failed state.
 */
static void
bfa_iocpf_sm_fail(struct bfa_iocpf_s *iocpf, enum iocpf_event event)
{
	struct bfa_ioc_s *ioc = iocpf->ioc;

	bfa_trc(ioc, event);

	switch (event) {
	case IOCPF_E_DISABLE:
		bfa_fsm_set_state(iocpf, bfa_iocpf_sm_disabled);
		break;

	default:
		bfa_sm_fault(ioc, event);
	}
}

/**
 * @}
 */

/**
 * @dg hal_ioc_pvt BFA IOC private functions
 * @{
 */

/**
 * Notify common modules registered for notification.
 */
static void
bfa_ioc_event_notify(struct bfa_ioc_s *ioc, enum bfa_ioc_event_e event)
{
	struct bfa_ioc_notify_s		*notify;
	struct bfa_q_s			*qe;

	bfa_q_iter(&ioc->notify_q, qe) {
		notify = (struct bfa_ioc_notify_s *)qe;
		notify->cbfn(notify->cbarg, event);
	}
}

static void
bfa_ioc_disable_comp(struct bfa_ioc_s *ioc)
{
	ioc->cbfn->disable_cbfn(ioc->bfa);
	bfa_ioc_event_notify(ioc, BFA_IOC_E_DISABLED);
}

bfa_boolean_t
bfa_ioc_sem_get(bfa_os_addr_t sem_reg)
{
	volatile uint32_t r32;
	int cnt = 0;
#define BFA_SEM_SPINCNT	3000

	if (bfa_ioc_is_uefi(ioc))
		return BFA_TRUE;

	r32 = bfa_reg_read(sem_reg);

	while ((r32 & 1) && (cnt < BFA_SEM_SPINCNT)) {
		cnt++;
		bfa_os_udelay(2);
		r32 = bfa_reg_read(sem_reg);
	}

	if (!(r32 & 1))
		return BFA_TRUE;

	return BFA_FALSE;
}

void
bfa_ioc_sem_release(bfa_os_addr_t sem_reg)
{
	if (bfa_ioc_is_uefi(ioc))
		return;

	bfa_reg_read(sem_reg);
	bfa_reg_write(sem_reg, 1);
}

/**
 * Called during driver load or hibernate/resume to unlock hw sem
 * in case if it was previously held by uEFI and left locked.
 */
static void
bfa_ioc_hw_sem_init(struct bfa_ioc_s *ioc)
{
	struct bfi_ioc_image_hdr_s fwhdr;
	volatile uint32_t fwstate, r32;

	if (bfa_ioc_is_uefi(ioc))
		return;

	/**
	 * Spin on init semaphore to serialize.
	 */
	r32 = bfa_reg_read(ioc->ioc_regs.ioc_init_sem_reg);
	while (r32 & 0x1) {
		bfa_os_udelay(20);
		r32 = bfa_reg_read(ioc->ioc_regs.ioc_init_sem_reg);
	}

	fwstate = bfa_ioc_get_cur_ioc_fwstate(ioc);

	if (fwstate == BFI_IOC_UNINIT) {
		bfa_reg_write(ioc->ioc_regs.ioc_init_sem_reg, 1);
		return;
	}

	bfa_ioc_fwver_get(ioc, &fwhdr);

	if (bfa_os_swap32(fwhdr.exec) == BFI_FWBOOT_TYPE_NORMAL) {
		bfa_reg_write(ioc->ioc_regs.ioc_init_sem_reg, 1);
		return;
	}

	bfa_ioc_fwver_clear(ioc);

	bfa_trc(ioc, fwstate);
	bfa_trc(ioc, bfa_os_swap32(fwhdr.exec));

	bfa_ioc_set_cur_ioc_fwstate(ioc, BFI_IOC_UNINIT);
	bfa_ioc_set_alt_ioc_fwstate(ioc, BFI_IOC_UNINIT);

	/*
	 * Unlock the hw semaphore. Should be here only once per boot.
	 * Also, clear the use_count and fail_sync registers.
	 */
	bfa_ioc_ownership_reset(ioc);
	/*
	 * unlock init semaphore.
	 */
	bfa_reg_write(ioc->ioc_regs.ioc_init_sem_reg, 1);
}

static void
bfa_ioc_hw_sem_get(struct bfa_ioc_s *ioc)
{
	volatile uint32_t	r32;

	/**
	 * If uEFI, no need to lock semaphore.
	 */
	if (bfa_ioc_is_uefi(ioc)) {
		bfa_fsm_send_event(&ioc->iocpf, IOCPF_E_SEMLOCKED);
		return;
	}

	/**
	 * First read to the semaphore register will return 0, subsequent reads
	 * will return 1. Semaphore is released by writing 1 to the register
	 */
	r32 = bfa_reg_read(ioc->ioc_regs.ioc_sem_reg);
	if (r32 == ~0) {
		bfa_assert(r32 != ~0);
		bfa_fsm_send_event(&ioc->iocpf, IOCPF_E_SEM_ERROR);
		return;
	}
	if (!(r32 & 1)) {
		bfa_fsm_send_event(&ioc->iocpf, IOCPF_E_SEMLOCKED);
		return;
	}

	bfa_sem_timer_start(ioc);
}

void
bfa_ioc_hw_sem_release(struct bfa_ioc_s *ioc)
{
	if (bfa_ioc_is_uefi(ioc))
		return;

	bfa_reg_write(ioc->ioc_regs.ioc_sem_reg, 1);
}

static void
bfa_ioc_hw_sem_get_cancel(struct bfa_ioc_s *ioc)
{
	bfa_sem_timer_stop(ioc);
}

/**
 * @brief
 * Initialize LPU local memory (aka secondary memory / SRAM)
 */
static void
bfa_ioc_lmem_init(struct bfa_ioc_s *ioc)
{
	uint32_t	pss_ctl;
	int		i;
#define PSS_LMEM_INIT_TIME  10000

	pss_ctl = bfa_reg_read(ioc->ioc_regs.pss_ctl_reg);
	pss_ctl &= ~__PSS_LMEM_RESET;
	pss_ctl |= __PSS_LMEM_INIT_EN;

	/*
	 * i2c workaround 12.5khz clock
	 */
	pss_ctl |= __PSS_I2C_CLK_DIV(3UL);
	bfa_reg_write(ioc->ioc_regs.pss_ctl_reg, pss_ctl);

	/**
	 * wait for memory initialization to be complete
	 */
	i = 0;
	do {
		pss_ctl = bfa_reg_read(ioc->ioc_regs.pss_ctl_reg);
		i++;
	} while (!(pss_ctl & __PSS_LMEM_INIT_DONE) && (i < PSS_LMEM_INIT_TIME));

	/**
	 * If memory initialization is not successful, IOC timeout will catch
	 * such failures.
	 */
	bfa_assert(pss_ctl & __PSS_LMEM_INIT_DONE);
	bfa_trc(ioc, pss_ctl);

	pss_ctl &= ~(__PSS_LMEM_INIT_DONE | __PSS_LMEM_INIT_EN);
	bfa_reg_write(ioc->ioc_regs.pss_ctl_reg, pss_ctl);
}

static void
bfa_ioc_lpu_start(struct bfa_ioc_s *ioc)
{
	uint32_t	pss_ctl;

	/**
	 * Take processor out of reset.
	 */
	pss_ctl = bfa_reg_read(ioc->ioc_regs.pss_ctl_reg);
	pss_ctl &= ~__PSS_LPU0_RESET;

	bfa_reg_write(ioc->ioc_regs.pss_ctl_reg, pss_ctl);
}

static void
bfa_ioc_lpu_stop(struct bfa_ioc_s *ioc)
{
	uint32_t	pss_ctl;

	/**
	 * Put processors in reset.
	 */
	pss_ctl = bfa_reg_read(ioc->ioc_regs.pss_ctl_reg);
	pss_ctl |= (__PSS_LPU0_RESET | __PSS_LPU1_RESET);

	bfa_reg_write(ioc->ioc_regs.pss_ctl_reg, pss_ctl);
}

/**
 * Get driver and firmware versions.
 */
void
bfa_ioc_fwver_get(struct bfa_ioc_s *ioc, struct bfi_ioc_image_hdr_s *fwhdr)
{
	uint32_t	pgnum, pgoff;
	uint32_t	loff = 0;
	int		i;
	uint32_t	*fwsig = (uint32_t *) fwhdr;

	pgnum = bfa_ioc_smem_pgnum(ioc, loff);
	pgoff = bfa_ioc_smem_pgoff(ioc, loff);
	bfa_reg_write(ioc->ioc_regs.host_page_num_fn, pgnum);

	for (i = 0; i < (sizeof(struct bfi_ioc_image_hdr_s) / sizeof(uint32_t));
	     i++) {
		fwsig[i] =
			bfa_mem_read(ioc->ioc_regs.smem_page_start, loff);
		loff += sizeof(uint32_t);
	}
}

static bfa_boolean_t
bfa_ioc_fwver_md5_check(struct bfi_ioc_image_hdr_s *fwhdr_1,
				struct bfi_ioc_image_hdr_s *fwhdr_2)
{
	int i;

	for (i = 0; i < BFI_IOC_MD5SUM_SZ; i++) {
		if (fwhdr_1->md5sum[i] != fwhdr_2->md5sum[i]) {
			return BFA_FALSE;
		}
	}
	return BFA_TRUE;
}

/**
 * Returns TRUE if major minor and maintainence are same.
 * If patch versions are same, check for MD5 Checksum to be same.
 */
static bfa_boolean_t
bfa_ioc_fw_ver_compatible(struct bfi_ioc_image_hdr_s *drv_fwhdr,
				struct bfi_ioc_image_hdr_s *fwhdr_to_cmp)
{
	if (drv_fwhdr->signature != fwhdr_to_cmp->signature)
		return BFA_FALSE;
	if (drv_fwhdr->fwver.major != fwhdr_to_cmp->fwver.major)
		return BFA_FALSE;
	if (drv_fwhdr->fwver.minor != fwhdr_to_cmp->fwver.minor)
		return BFA_FALSE;
	if (drv_fwhdr->fwver.maint != fwhdr_to_cmp->fwver.maint)
		return BFA_FALSE;
	if (drv_fwhdr->fwver.patch == fwhdr_to_cmp->fwver.patch &&
		drv_fwhdr->fwver.phase == fwhdr_to_cmp->fwver.phase &&
		drv_fwhdr->fwver.build == fwhdr_to_cmp->fwver.build) {
		return bfa_ioc_fwver_md5_check(drv_fwhdr, fwhdr_to_cmp);
	}
	return BFA_TRUE;
}

static bfa_boolean_t
bfa_ioc_flash_fwver_valid(struct bfi_ioc_image_hdr_s *flash_fwhdr)
{
	if (flash_fwhdr->fwver.major == 0 || flash_fwhdr->fwver.major == 0xFF)
		return BFA_FALSE;
	return BFA_TRUE;
}

static bfa_boolean_t fwhdr_is_ga(struct bfi_ioc_image_hdr_s *fwhdr)
{
	if (fwhdr->fwver.phase == 0 &&
		fwhdr->fwver.build == 0)
		return BFA_TRUE;
	return BFA_FALSE;
}

/**
 * Returns TRUE if both are compatible and patch of fwhdr_to_cmp is better.
 */
static bfi_ioc_img_ver_cmp_t
bfa_ioc_fw_ver_patch_cmp(struct bfi_ioc_image_hdr_s *base_fwhdr,
				struct bfi_ioc_image_hdr_s *fwhdr_to_cmp)
{
	if (bfa_ioc_fw_ver_compatible(base_fwhdr, fwhdr_to_cmp) == BFA_FALSE)
		return BFI_IOC_IMG_VER_INCOMP;

	if (fwhdr_to_cmp->fwver.patch > base_fwhdr->fwver.patch)
		return BFI_IOC_IMG_VER_BETTER;
	else if (fwhdr_to_cmp->fwver.patch < base_fwhdr->fwver.patch)
		return BFI_IOC_IMG_VER_OLD;

	/*
	 * GA takes priority over internal builds of the same patch stream.
	 * At this point major minor maint and patch numbers are same.
	 */

	if (fwhdr_is_ga(base_fwhdr) == BFA_TRUE) {
		if (fwhdr_is_ga(fwhdr_to_cmp))
			return BFI_IOC_IMG_VER_SAME;
		else
			return BFI_IOC_IMG_VER_OLD;
	} else {
		if (fwhdr_is_ga(fwhdr_to_cmp))
			return BFI_IOC_IMG_VER_BETTER;
	}

	if (fwhdr_to_cmp->fwver.phase > base_fwhdr->fwver.phase)
		return BFI_IOC_IMG_VER_BETTER;
	else if (fwhdr_to_cmp->fwver.phase < base_fwhdr->fwver.phase)
		return BFI_IOC_IMG_VER_OLD;

	if (fwhdr_to_cmp->fwver.build > base_fwhdr->fwver.build)
		return BFI_IOC_IMG_VER_BETTER;
	else if (fwhdr_to_cmp->fwver.build < base_fwhdr->fwver.build)
		return BFI_IOC_IMG_VER_OLD;

	/*
	 * All Version Numbers are equal.
	 * Md5 check to be done as a part of compatibility check.
	 */
	return BFI_IOC_IMG_VER_SAME;
}

uint32_t
bfa_ioc_flash_img_get_size(struct bfa_ioc_s *ioc)
{
	return BFI_FLASH_IMAGE_SZ/sizeof(uint32_t);
}

bfa_status_t
bfa_ioc_flash_img_get_chnk(struct bfa_ioc_s *ioc, uint32_t off,
				uint32_t *fwimg)
{
	return bfa_flash_raw_read(ioc->pcidev.pci_bar_kva,
			BFA_FLASH_PART_FWIMG_ADDR + (off * sizeof(uint32_t)),
			(char *)fwimg, BFI_FLASH_CHUNK_SZ);
}

static bfi_ioc_img_ver_cmp_t
bfa_ioc_flash_fwver_cmp(struct bfa_ioc_s *ioc,
			struct bfi_ioc_image_hdr_s *base_fwhdr)
{
	struct bfi_ioc_image_hdr_s *flash_fwhdr;
	bfa_status_t status;
	uint32_t fwimg[BFI_FLASH_CHUNK_SZ_WORDS];

	status = bfa_ioc_flash_img_get_chnk(ioc, 0, fwimg);
	if (status != BFA_STATUS_OK) {
		return BFI_IOC_IMG_VER_INCOMP;
	}

	flash_fwhdr = (struct bfi_ioc_image_hdr_s *) fwimg;
	if (bfa_ioc_flash_fwver_valid(flash_fwhdr) == BFA_TRUE)
		return bfa_ioc_fw_ver_patch_cmp(base_fwhdr, flash_fwhdr);
	else
		return BFI_IOC_IMG_VER_INCOMP;
}

/**
 * Returns TRUE if driver is willing to work with current smem f/w version.
 */
bfa_boolean_t
bfa_ioc_fwver_cmp(struct bfa_ioc_s *ioc,
		struct bfi_ioc_image_hdr_s *smem_fwhdr)
{
	struct bfi_ioc_image_hdr_s *drv_fwhdr;
	bfi_ioc_img_ver_cmp_t smem_flash_cmp, drv_smem_cmp;

	drv_fwhdr = (struct bfi_ioc_image_hdr_s *)
		bfa_cb_image_get_chunk(bfa_ioc_asic_gen(ioc), 0);

	/*
	 * If smem is incompatible or old, driver should not work with it.
	*/
	drv_smem_cmp = bfa_ioc_fw_ver_patch_cmp(drv_fwhdr, smem_fwhdr);
	if (drv_smem_cmp == BFI_IOC_IMG_VER_INCOMP ||
		drv_smem_cmp == BFI_IOC_IMG_VER_OLD) {
		return BFA_FALSE;
	}

	/*
	 * IF Flash has a better F/W than smem do not work with smem.
	 * If smem f/w == flash f/w, as smem f/w not old | incmp, work with it.
	 * If Flash is old or incomp work with smem iff smem f/w == drv f/w.
	*/
	smem_flash_cmp = bfa_ioc_flash_fwver_cmp(ioc, smem_fwhdr);

	if (smem_flash_cmp == BFI_IOC_IMG_VER_BETTER) {
		return BFA_FALSE;
	} else if (smem_flash_cmp == BFI_IOC_IMG_VER_SAME) {
		return BFA_TRUE;
	} else {
		return (drv_smem_cmp == BFI_IOC_IMG_VER_SAME)?
			BFA_TRUE : BFA_FALSE;
	}
}

/**
 * Return true if current running version is valid. Firmware signature and
 * execution context (driver/bios) must match.
 */
static bfa_boolean_t
bfa_ioc_fwver_valid(struct bfa_ioc_s *ioc, uint32_t boot_env)
{
	struct bfi_ioc_image_hdr_s smem_fwhdr;

	bfa_ioc_fwver_get(ioc, &smem_fwhdr);
	if (bfa_os_swap32(smem_fwhdr.bootenv) != boot_env) {
		bfa_trc(ioc, smem_fwhdr.bootenv);
		bfa_trc(ioc, boot_env);
		return BFA_FALSE;
	}

	return bfa_ioc_fwver_cmp(ioc, &smem_fwhdr);
}


/**
 * Invalidate fwver signature
 */
bfa_status_t
bfa_ioc_fwsig_invalidate(struct bfa_ioc_s *ioc)
{

	uint32_t	pgnum, pgoff;
	uint32_t	loff = 0;
	enum bfi_ioc_state ioc_fwstate;

	ioc_fwstate = bfa_ioc_get_cur_ioc_fwstate(ioc);
	if (!bfa_ioc_state_disabled(ioc_fwstate))
		return BFA_STATUS_ADAPTER_ENABLED;

	pgnum = bfa_ioc_smem_pgnum(ioc, loff);
	pgoff = bfa_ioc_smem_pgoff(ioc, loff);
	bfa_reg_write(ioc->ioc_regs.host_page_num_fn, pgnum);
	bfa_mem_write(ioc->ioc_regs.smem_page_start, loff, BFA_IOC_FW_INV_SIGN);
	return BFA_STATUS_OK;
}

/**
 * Clear fwver hdr
 */
static void
bfa_ioc_fwver_clear(struct bfa_ioc_s *ioc)
{
	uint32_t	pgnum, pgoff;
	uint32_t	loff = 0;
	int		i;

	pgnum = bfa_ioc_smem_pgnum(ioc, loff);
	pgoff = bfa_ioc_smem_pgoff(ioc, loff);
	bfa_reg_write(ioc->ioc_regs.host_page_num_fn, pgnum);

	for (i = 0; i < (sizeof(struct bfi_ioc_image_hdr_s) / sizeof(uint32_t));
	     i++) {
		bfa_mem_write(ioc->ioc_regs.smem_page_start, loff, 0);
		loff += sizeof(uint32_t);
	}
}

/**
 * Conditionally flush any pending message from firmware at start.
 */
static void
bfa_ioc_msgflush(struct bfa_ioc_s *ioc)
{
	uint32_t	r32;

	r32 = bfa_reg_read(ioc->ioc_regs.lpu_mbox_cmd);
	if (r32)
		bfa_reg_write(ioc->ioc_regs.lpu_mbox_cmd, 1);
}

/**
 * @img ioc_init_logic.jpg
 */
static void
bfa_ioc_hwinit(struct bfa_ioc_s *ioc, bfa_boolean_t force)
{
	enum bfi_ioc_state ioc_fwstate;
	bfa_boolean_t fwvalid;
	enum bfi_fwboot_type boot_type;
	enum bfi_fwboot_env boot_env;

	ioc_fwstate = bfa_ioc_get_cur_ioc_fwstate(ioc);

	if (force)
		ioc_fwstate = BFI_IOC_UNINIT;

	bfa_trc(ioc, ioc_fwstate);

	boot_type = BFI_FWBOOT_TYPE_NORMAL;
	boot_env = BFI_FWBOOT_ENV_OS;

	/**
	 * Flash based firmware boot UEFI env.
	 */
	if (bfa_ioc_is_uefi(ioc)) {
		boot_type = BFI_FWBOOT_TYPE_FLASH;
		boot_env = BFI_FWBOOT_ENV_UEFI;
	}
	ioc->boot_env = boot_env;

	/**
	 * check if firmware is valid
	 */
	fwvalid = (ioc_fwstate == BFI_IOC_UNINIT) ?
		BFA_FALSE : bfa_ioc_fwver_valid(ioc, boot_env);

	if (!fwvalid) {
		if (bfa_ioc_boot(ioc, boot_type, boot_env) == BFA_STATUS_OK)
			bfa_ioc_poll_fwinit(ioc);
		return;
	}

	/**
	 * If hardware initialization is in progress (initialized by other IOC),
	 * just wait for an initialization completion interrupt.
	 */
	if (ioc_fwstate == BFI_IOC_INITING) {
		bfa_ioc_poll_fwinit(ioc);
		return;
	}

	/**
	 * If IOC function is disabled and firmware version is same,
	 * just re-enable IOC.
	 *
	 * If option rom, IOC must not be in operational state. With
	 * convergence, IOC will be in operational state when 2nd driver
	 * is loaded.
	 */
	if (ioc_fwstate == BFI_IOC_DISABLED || ioc_fwstate == BFI_IOC_OP) {

		/**
		 * When using MSI-X any pending firmware ready event should
		 * be flushed. Otherwise MSI-X interrupts are not delivered.
		 */
		bfa_ioc_msgflush(ioc);
		bfa_fsm_send_event(&ioc->iocpf, IOCPF_E_FWREADY);
		return;
	}

	/**
	 * Initialize the h/w for any other states.
	 */
	if (bfa_ioc_boot(ioc, boot_type, boot_env) == BFA_STATUS_OK)
		bfa_ioc_poll_fwinit(ioc);
}

static void
bfa_ioc_timeout(void *ioc_arg)
{
	struct bfa_ioc_s  *ioc = (struct bfa_ioc_s *) ioc_arg;

	bfa_trc(ioc, 0);
	bfa_fsm_send_event(ioc, IOC_E_TIMEOUT);
}

void
bfa_ioc_mbox_send(struct bfa_ioc_s *ioc, void *ioc_msg, int len)
{
	uint32_t *msgp = (uint32_t *) ioc_msg;
	struct bfi_mhdr_s *mh = (struct bfi_mhdr_s *)msgp;
	uint32_t i;

	/*
	 * Do not trace edma response.
	 */
	if (mh->msg_class != BFI_MC_EDMA) {
		bfa_trc(ioc, msgp[0]);
		bfa_trc(ioc, len);
	}

	bfa_assert(len <= BFI_IOC_MSGLEN_MAX);

	/*
	 * first write msg to mailbox registers
	 */
	for (i = 0; i < len / sizeof(uint32_t); i++)
		bfa_reg_write(ioc->ioc_regs.hfn_mbox + i * sizeof(uint32_t),
			      bfa_os_wtole(msgp[i]));

	for (; i < BFI_IOC_MSGLEN_MAX / sizeof(uint32_t); i++)
		bfa_reg_write(ioc->ioc_regs.hfn_mbox + i * sizeof(uint32_t), 0);

	/*
	 * write 1 to mailbox CMD to trigger LPU event
	 */
	bfa_reg_write(ioc->ioc_regs.hfn_mbox_cmd, 1);
	(void) bfa_reg_read(ioc->ioc_regs.hfn_mbox_cmd);
}

static void
bfa_ioc_send_enable(struct bfa_ioc_s *ioc)
{
	struct bfi_ioc_ctrl_req_s enable_req;
	bfa_timeval_t tv;

	bfi_h2i_set(enable_req.mh, BFI_MC_IOC, BFI_IOC_H2I_ENABLE_REQ,
		    bfa_ioc_portid(ioc));

	enable_req.clscode = bfa_os_htons(ioc->clscode);
	bfa_os_gettimeofday(&tv);
	enable_req.tv_sec = bfa_os_ntohl(tv.tv_sec);
	bfa_ioc_mbox_send(ioc, &enable_req, sizeof(struct bfi_ioc_ctrl_req_s));
}

static void
bfa_ioc_send_disable(struct bfa_ioc_s *ioc)
{
	struct bfi_ioc_ctrl_req_s disable_req;

	bfi_h2i_set(disable_req.mh, BFI_MC_IOC, BFI_IOC_H2I_DISABLE_REQ,
		    bfa_ioc_portid(ioc));
	bfa_ioc_mbox_send(ioc, &disable_req, sizeof(struct bfi_ioc_ctrl_req_s));
}

static void
bfa_ioc_send_getattr(struct bfa_ioc_s *ioc)
{
	struct bfi_ioc_getattr_req_s	attr_req;

	bfi_h2i_set(attr_req.mh, BFI_MC_IOC, BFI_IOC_H2I_GETATTR_REQ,
		    bfa_ioc_portid(ioc));
	bfa_dma_be_addr_set(attr_req.attr_addr, ioc->attr_dma.pa);
	bfa_ioc_mbox_send(ioc, &attr_req, sizeof(attr_req));
}

static void
bfa_ioc_hb_check(void *cbarg)
{
	struct bfa_ioc_s  *ioc = cbarg;
	uint32_t	hb_count;

	hb_count = bfa_reg_read(ioc->ioc_regs.heartbeat);
	if (ioc->hb_count == hb_count) {
		bfa_ioc_recover(ioc);
		return;
	} else {
		ioc->hb_count = hb_count;
	}

	bfa_ioc_mbox_poll(ioc);
	bfa_hb_timer_start(ioc);
}

static void
bfa_ioc_hb_monitor(struct bfa_ioc_s *ioc)
{
	ioc->hb_count = bfa_reg_read(ioc->ioc_regs.heartbeat);
	bfa_hb_timer_start(ioc);
}

static void
bfa_ioc_hb_stop(struct bfa_ioc_s *ioc)
{
	bfa_hb_timer_stop(ioc);
}

/**
 * @brief
 *	Initiate a full firmware download.
 */
static bfa_status_t
bfa_ioc_download_fw(struct bfa_ioc_s *ioc, uint32_t boot_type,
		    uint32_t boot_env)
{
	uint32_t *fwimg;
	uint32_t pgnum, pgoff;
	uint32_t loff = 0;
	uint32_t chunkno = 0;
	uint32_t i;
	uint32_t asicmode;
	uint32_t fwimg_size;
	uint32_t fwimg_buf[BFI_FLASH_CHUNK_SZ_WORDS];
	bfa_status_t status;

	if (boot_env == BFI_FWBOOT_ENV_OS &&
		boot_type == BFI_FWBOOT_TYPE_FLASH) {
		fwimg_size = bfa_ioc_flash_img_get_size(ioc);

		status = bfa_ioc_flash_img_get_chnk(ioc,
			BFA_IOC_FLASH_CHUNK_ADDR(chunkno), fwimg_buf);
		if (status != BFA_STATUS_OK)
			return status;

		fwimg = fwimg_buf;
	} else {
		fwimg_size = bfa_cb_image_get_size(bfa_ioc_asic_gen(ioc));
		fwimg = bfa_cb_image_get_chunk(bfa_ioc_asic_gen(ioc),
					BFA_IOC_FLASH_CHUNK_ADDR(chunkno));
	}

	bfa_trc(ioc, fwimg_size);


	pgnum = bfa_ioc_smem_pgnum(ioc, loff);
	pgoff = bfa_ioc_smem_pgoff(ioc, loff);

	bfa_reg_write(ioc->ioc_regs.host_page_num_fn, pgnum);

	for (i = 0; i < fwimg_size; i++) {

		if (BFA_IOC_FLASH_CHUNK_NO(i) != chunkno) {
			chunkno = BFA_IOC_FLASH_CHUNK_NO(i);

			if (boot_env == BFI_FWBOOT_ENV_OS &&
				boot_type == BFI_FWBOOT_TYPE_FLASH) {
				status = bfa_ioc_flash_img_get_chnk(ioc,
					BFA_IOC_FLASH_CHUNK_ADDR(chunkno),
					fwimg_buf);
				if (status != BFA_STATUS_OK) {
					return status;
				}
				fwimg = fwimg_buf;
			} else {
				fwimg = bfa_cb_image_get_chunk(
					bfa_ioc_asic_gen(ioc),
					BFA_IOC_FLASH_CHUNK_ADDR(chunkno));
			}
		}

		/**
		 * write smem
		 */
		bfa_mem_write(ioc->ioc_regs.smem_page_start, loff,
			      fwimg[BFA_IOC_FLASH_OFFSET_IN_CHUNK(i)]);

		loff += sizeof(uint32_t);

		/**
		 * handle page offset wrap around
		 */
		loff = PSS_SMEM_PGOFF(loff);
		if (loff == 0) {
			pgnum++;
			bfa_reg_write(ioc->ioc_regs.host_page_num_fn,
				      pgnum);
		}
	}

	bfa_reg_write(ioc->ioc_regs.host_page_num_fn,
		      bfa_ioc_smem_pgnum(ioc, 0));

	/*
	 * Set boot type, env and device mode at the end.
	*/
	if (boot_env == BFI_FWBOOT_ENV_OS &&
		boot_type == BFI_FWBOOT_TYPE_FLASH) {
		boot_type = BFI_FWBOOT_TYPE_NORMAL;
	}
	asicmode = BFI_FWBOOT_DEVMODE(ioc->asic_gen, ioc->asic_mode,
				      ioc->port0_mode, ioc->port1_mode);
	bfa_mem_write(ioc->ioc_regs.smem_page_start, BFI_FWBOOT_DEVMODE_OFF,
			bfa_os_swap32(asicmode));
	bfa_mem_write(ioc->ioc_regs.smem_page_start, BFI_FWBOOT_TYPE_OFF,
			bfa_os_swap32(boot_type));
	bfa_mem_write(ioc->ioc_regs.smem_page_start, BFI_FWBOOT_ENV_OFF,
			bfa_os_swap32(boot_env));
	return BFA_STATUS_OK;
}

static void
bfa_ioc_reset(struct bfa_ioc_s *ioc, bfa_boolean_t force)
{
	bfa_ioc_hwinit(ioc, force);
}

/**
 * @brief
 * BFA ioc enable reply by firmware
 */
static void
bfa_ioc_enable_reply(struct bfa_ioc_s *ioc, bfa_mode_t port_mode,
		     uint8_t cap_bm)
{
	struct bfa_iocpf_s *iocpf = &ioc->iocpf;

	ioc->port_mode = ioc->port_mode_cfg = port_mode;
	ioc->ad_cap_bm = cap_bm;
	bfa_fsm_send_event(iocpf, IOCPF_E_FWRSP_ENABLE);
}

/**
 * @brief
 * Update BFA configuration from firmware configuration.
 */
static void
bfa_ioc_getattr_reply(struct bfa_ioc_s *ioc)
{
	struct bfi_ioc_attr_s	*attr = ioc->attr;

	attr->adapter_prop  = bfa_os_ntohl(attr->adapter_prop);
	attr->card_type     = bfa_os_ntohl(attr->card_type);
	attr->maxfrsize	    = bfa_os_ntohs(attr->maxfrsize);
	ioc->fcmode	    = (attr->port_mode == BFI_PORT_MODE_FC);
	attr->mfg_year	    = bfa_os_ntohs(attr->mfg_year);

	bfa_fsm_send_event(ioc, IOC_E_FWRSP_GETATTR);
}

/**
 * Attach time initialization of mbox logic.
 */
static void
bfa_ioc_mbox_attach(struct bfa_ioc_s *ioc)
{
	struct bfa_ioc_mbox_mod_s	*mod = &ioc->mbox_mod;
	int	mc;

	bfa_q_init(&mod->cmd_q);
	for (mc = 0; mc < BFI_MC_MAX; mc++) {
		mod->mbhdlr[mc].cbfn = NULL;
		mod->mbhdlr[mc].cbarg = ioc->bfa;
	}
}

/**
 * Mbox poll timer -- restarts any pending mailbox requests.
 */
static void
bfa_ioc_mbox_poll(struct bfa_ioc_s *ioc)
{
	struct bfa_ioc_mbox_mod_s	*mod = &ioc->mbox_mod;
	struct bfa_mbox_cmd_s		*cmd;
	bfa_mbox_cmd_cbfn_t		cbfn;
	void				*cbarg;
	uint32_t			stat;

	/**
	 * If no command pending, do nothing
	 */
	if (bfa_q_is_empty(&mod->cmd_q))
		return;

	/**
	 * If previous command is not yet fetched by firmware, do nothing
	 */
	stat = bfa_reg_read(ioc->ioc_regs.hfn_mbox_cmd);
	if (stat)
		return;

	/**
	 * Enqueue command to firmware.
	 */
	bfa_q_deq(&mod->cmd_q, &cmd);
	bfa_ioc_mbox_send(ioc, cmd->msg, sizeof(cmd->msg));

	/**
	 * Give a callback to the client, indicating that the command is sent
	 */
	if (cmd->cbfn) {
		cbfn = cmd->cbfn;
		cbarg = cmd->cbarg;
		cmd->cbfn = NULL;
		cbfn(cbarg);
	}
}

/**
 * Cleanup any pending requests.
 */
static void
bfa_ioc_mbox_flush(struct bfa_ioc_s *ioc)
{
	struct bfa_ioc_mbox_mod_s	*mod = &ioc->mbox_mod;
	struct bfa_mbox_cmd_s		*cmd;

	while (!bfa_q_is_empty(&mod->cmd_q))
		bfa_q_deq(&mod->cmd_q, &cmd);
}

/**
 * Read data from SMEM to host through PCI memmap
 *
 * @param[in]	ioc	memory for IOC
 * @param[in]	tbuf	app memory to store data from smem
 * @param[in]	soff	smem offset
 * @param[in]	sz	size of smem in bytes
 */
static bfa_status_t
bfa_ioc_smem_read(struct bfa_ioc_s *ioc, void *tbuf, uint32_t soff, uint32_t sz)
{
	uint32_t pgnum, loff, r32;
	int i, len;
	uint32_t *buf = tbuf;

	pgnum = bfa_ioc_smem_pgnum(ioc, soff);
	loff = bfa_ioc_smem_pgoff(ioc, soff);
	bfa_trc(ioc, pgnum);
	bfa_trc(ioc, loff);
	bfa_trc(ioc, sz);

	/*
	 *  Hold semaphore to serialize pll init and fwtrc.
	 */
	if (BFA_FALSE == bfa_ioc_sem_get(ioc->ioc_regs.ioc_init_sem_reg)) {
		bfa_trc(ioc, 0);
		return BFA_STATUS_FAILED;
	}

	bfa_reg_write(ioc->ioc_regs.host_page_num_fn, pgnum);

	len = sz/sizeof(uint32_t);
	bfa_trc(ioc, len);
	for (i = 0; i < len; i++) {
		r32 = bfa_mem_read(ioc->ioc_regs.smem_page_start, loff);
		buf[i] = bfa_os_swap32(r32);
		loff += sizeof(uint32_t);

		/**
		 * handle page offset wrap around
		 */
		loff = PSS_SMEM_PGOFF(loff);
		if (loff == 0) {
			pgnum++;
			bfa_reg_write(ioc->ioc_regs.host_page_num_fn, pgnum);
		}
	}
	bfa_reg_write(ioc->ioc_regs.host_page_num_fn,
		      bfa_ioc_smem_pgnum(ioc, 0));
	/*
	 *  release semaphore.
	 */
	bfa_ioc_sem_release(ioc->ioc_regs.ioc_init_sem_reg);

	bfa_trc(ioc, pgnum);
	return BFA_STATUS_OK;
}

/**
 * Clear SMEM data from host through PCI memmap
 *
 * @param[in]	ioc	memory for IOC
 * @param[in]	soff	smem offset
 * @param[in]	sz	size of smem in bytes
 */
static bfa_status_t
bfa_ioc_smem_clr(struct bfa_ioc_s *ioc, uint32_t soff, uint32_t sz)
{
	int i, len;
	uint32_t pgnum, loff;

	pgnum = bfa_ioc_smem_pgnum(ioc, soff);
	loff = bfa_ioc_smem_pgoff(ioc, soff);
	bfa_trc(ioc, pgnum);
	bfa_trc(ioc, loff);
	bfa_trc(ioc, sz);

	/*
	 *  Hold semaphore to serialize pll init and fwtrc.
	 */
	if (BFA_FALSE == bfa_ioc_sem_get(ioc->ioc_regs.ioc_init_sem_reg)) {
		bfa_trc(ioc, 0);
		return BFA_STATUS_FAILED;
	}

	bfa_reg_write(ioc->ioc_regs.host_page_num_fn, pgnum);

	len = sz/sizeof(uint32_t); /* len in words */
	bfa_trc(ioc, len);
	for (i = 0; i < len; i++) {
		bfa_mem_write(ioc->ioc_regs.smem_page_start, loff, 0);
		loff += sizeof(uint32_t);

		/**
		 * handle page offset wrap around
		 */
		loff = PSS_SMEM_PGOFF(loff);
		if (loff == 0) {
			pgnum++;
			bfa_reg_write(ioc->ioc_regs.host_page_num_fn, pgnum);
		}
	}
	bfa_reg_write(ioc->ioc_regs.host_page_num_fn,
		      bfa_ioc_smem_pgnum(ioc, 0));

	/*
	 *  release semaphore.
	 */
	bfa_ioc_sem_release(ioc->ioc_regs.ioc_init_sem_reg);
	bfa_trc(ioc, pgnum);
	return BFA_STATUS_OK;
}

/**
 * hal iocpf to ioc interface
 */
static void
bfa_ioc_pf_enabled(struct bfa_ioc_s *ioc)
{
	bfa_fsm_send_event(ioc, IOC_E_ENABLED);
}

static void
bfa_ioc_pf_disabled(struct bfa_ioc_s *ioc)
{
	bfa_fsm_send_event(ioc, IOC_E_DISABLED);
}

static void
bfa_ioc_pf_failed(struct bfa_ioc_s *ioc)
{
	bfa_fsm_send_event(ioc, IOC_E_PFFAILED);
}

static void
bfa_ioc_pf_hwfailed(struct bfa_ioc_s *ioc)
{
	bfa_fsm_send_event(ioc, IOC_E_HWFAILED);
}

static void
bfa_ioc_pf_fwmismatch(struct bfa_ioc_s *ioc)
{
	/**
	 * Provide enable completion callback and AEN notification.
	 */
	ioc->cbfn->enable_cbfn(ioc->bfa, BFA_STATUS_IOC_FAILURE);
	bfa_ioc_aen_post(ioc, BFA_IOC_AEN_FWMISMATCH);
}

/**
 * @}
 */

/**
 * @ig hal_ioc_public
 * @{
 */

bfa_status_t
bfa_ioc_pll_init(struct bfa_ioc_s *ioc)
{

	/*
	 *  Hold semaphore so that nobody can access the chip during init.
	 */
	bfa_ioc_sem_get(ioc->ioc_regs.ioc_init_sem_reg);

	bfa_ioc_pll_init_asic(ioc);

	ioc->pllinit = BFA_TRUE;
	/**
	 * Initialize LMEM
	 */
	bfa_ioc_lmem_init(ioc);

	/*
	 *  release semaphore.
	 */
	bfa_ioc_sem_release(ioc->ioc_regs.ioc_init_sem_reg);

	return BFA_STATUS_OK;
}

/**
 * Interface used by diag module to do firmware boot with memory test
 * as the entry vector.
 */
bfa_status_t
bfa_ioc_boot(struct bfa_ioc_s *ioc, enum bfi_fwboot_type boot_type,
	     enum bfi_fwboot_env boot_env)
{
	struct bfi_ioc_image_hdr_s *drv_fwhdr;
	bfa_status_t status;
	bfa_ioc_stats(ioc, ioc_boots);

	if (bfa_ioc_pll_init(ioc) != BFA_STATUS_OK)
		return BFA_STATUS_FAILED;
	if (boot_env == BFI_FWBOOT_ENV_OS &&
		boot_type == BFI_FWBOOT_TYPE_NORMAL) {

		drv_fwhdr = (struct bfi_ioc_image_hdr_s *)
			bfa_cb_image_get_chunk(bfa_ioc_asic_gen(ioc), 0);

		/**
		 * Work with Flash iff flash f/w is better than driver f/w.
		 * Otherwise push drivers firmware.
		 */
		if (bfa_ioc_flash_fwver_cmp(ioc, drv_fwhdr) ==
						BFI_IOC_IMG_VER_BETTER)
			boot_type = BFI_FWBOOT_TYPE_FLASH;
	}
	/**
	 * Initialize IOC state of all functions on a chip reset.
	 */
	if (boot_type == BFI_FWBOOT_TYPE_MEMTEST) {
		bfa_ioc_set_cur_ioc_fwstate(ioc, BFI_IOC_MEMTEST);
		bfa_ioc_set_alt_ioc_fwstate(ioc, BFI_IOC_MEMTEST);
	} else {
		bfa_ioc_set_cur_ioc_fwstate(ioc, BFI_IOC_INITING);
		bfa_ioc_set_alt_ioc_fwstate(ioc, BFI_IOC_INITING);
	}

	bfa_ioc_msgflush(ioc);
	status = bfa_ioc_download_fw(ioc, boot_type, boot_env);
	if (status == BFA_STATUS_OK)
		bfa_ioc_lpu_start(ioc);
	else {
		bfa_assert(boot_type != BFI_FWBOOT_TYPE_MEMTEST);
		bfa_iocpf_timeout(ioc);
	}
	return status;
}

/**
 * Enable/disable IOC failure auto recovery.
 */
void
bfa_ioc_auto_recover(bfa_boolean_t auto_recover)
{
	bfa_auto_recover = auto_recover;
}


/**
 * @brief
 */
bfa_boolean_t
bfa_ioc_is_operational(struct bfa_ioc_s *ioc)
{
	return bfa_fsm_cmp_state(ioc, bfa_ioc_sm_op);
}

bfa_boolean_t
bfa_ioc_is_initialized(struct bfa_ioc_s *ioc)
{
	uint32_t r32 = bfa_ioc_get_cur_ioc_fwstate(ioc);

	return ((r32 != BFI_IOC_UNINIT) &&
		(r32 != BFI_IOC_INITING) &&
		(r32 != BFI_IOC_MEMTEST));
}

bfa_boolean_t
bfa_ioc_msgget(struct bfa_ioc_s *ioc, void *mbmsg)
{
	uint32_t	*msgp = mbmsg;
	uint32_t	r32;
	int		i;

	r32 = bfa_reg_read(ioc->ioc_regs.lpu_mbox_cmd);
	if ((r32 & 1) == 0)
		return BFA_FALSE;

	/**
	 * read the MBOX msg
	 */
	for (i = 0; i < (sizeof(union bfi_ioc_i2h_msg_u) / sizeof(uint32_t));
	     i++) {
		r32 = bfa_reg_read(ioc->ioc_regs.lpu_mbox +
				   i * sizeof(uint32_t));
		msgp[i] = bfa_os_htonl(r32);
	}

	/**
	 * turn off mailbox interrupt by clearing mailbox status
	 */
	bfa_reg_write(ioc->ioc_regs.lpu_mbox_cmd, 1);
	bfa_reg_read(ioc->ioc_regs.lpu_mbox_cmd);

	return BFA_TRUE;
}

void
bfa_ioc_isr(struct bfa_ioc_s *ioc, struct bfi_mbmsg_s *m)
{
	union bfi_ioc_i2h_msg_u	*msg;
	struct bfa_iocpf_s *iocpf = &ioc->iocpf;

	msg = (union bfi_ioc_i2h_msg_u *) m;

	bfa_ioc_stats(ioc, ioc_isrs);

	switch (msg->mh.msg_id) {
	case BFI_IOC_I2H_HBEAT:
		break;

	case BFI_IOC_I2H_ENABLE_REPLY:
		bfa_ioc_enable_reply(ioc, (bfa_mode_t)msg->fw_event.port_mode,
				     msg->fw_event.cap_bm);
		break;

	case BFI_IOC_I2H_DISABLE_REPLY:
		bfa_fsm_send_event(iocpf, IOCPF_E_FWRSP_DISABLE);
		break;

	case BFI_IOC_I2H_GETATTR_REPLY:
		bfa_ioc_getattr_reply(ioc);
		break;

	default:
		bfa_trc(ioc, msg->mh.msg_id);
		bfa_assert(0);
	}
}

/**
 * IOC attach setup.
 *
 * @param[in]	trcmod	kernel trace module
 * @param[in]	aen	kernel aen event module
 * @param[in]	logm	kernel logging module
 */
void
bfa_ioc_trc_aen_log_set(struct bfa_ioc_s *ioc, struct bfa_trc_mod_s *trcmod,
	       struct bfa_aen_s *aen, struct bfa_log_mod_s *logm)
{
	ioc->trcmod	= trcmod;
	ioc->aen	= aen;
	ioc->logm	= logm;
}

/**
 * IOC attach time initialization and setup.
 *
 * @param[in]	ioc	memory for IOC
 * @param[in]	bfa	driver instance structure
 */
void
bfa_ioc_attach(struct bfa_ioc_s *ioc, void *bfa, struct bfa_ioc_cbfn_s *cbfn,
	       struct bfa_timer_mod_s *timer_mod)
{
	ioc->bfa	= bfa;
	ioc->cbfn	= cbfn;
	ioc->timer_mod	= timer_mod;
	ioc->fcmode	= BFA_FALSE;
	ioc->pllinit	= BFA_FALSE;
	ioc->dbg_fwsave_once = BFA_TRUE;
	ioc->iocpf.ioc	= ioc;

	bfa_ioc_mbox_attach(ioc);
	bfa_q_init(&ioc->notify_q);

	bfa_fsm_set_state(ioc, bfa_ioc_sm_uninit);
	bfa_fsm_send_event(ioc, IOC_E_RESET);
}

/**
 * Driver detach time IOC cleanup.
 */
void
bfa_ioc_detach(struct bfa_ioc_s *ioc)
{
	bfa_fsm_send_event(ioc, IOC_E_DETACH);

	/* Done with detach, empty the notify_q. */
	bfa_q_init(&ioc->notify_q);
}

/**
 * Setup IOC PCI properties.
 *
 * @param[in]	pcidev	PCI device information for this IOC
 */
void
bfa_ioc_pci_init(struct bfa_ioc_s *ioc, struct bfa_pcidev_s *pcidev,
		 enum bfi_pcifn_class clscode)
{
	ioc->clscode	= clscode;
	ioc->pcidev	= *pcidev;

	/**
	 * Initialize IOC and device personality
	 */
	ioc->port0_mode = ioc->port1_mode = BFI_PORT_MODE_FC;
	ioc->asic_mode  = BFI_ASIC_MODE_FC;

	switch (pcidev->device_id) {
	case BFA_PCI_DEVICE_ID_FC_8G1P:
	case BFA_PCI_DEVICE_ID_FC_8G2P:
		ioc->asic_gen = BFI_ASIC_GEN_CB;
		ioc->fcmode = BFA_TRUE;
		ioc->port_mode = ioc->port_mode_cfg = BFA_MODE_HBA;
		ioc->ad_cap_bm = BFA_CM_HBA;
		break;

	case BFA_PCI_DEVICE_ID_CT:
		ioc->asic_gen = BFI_ASIC_GEN_CT;
		ioc->port0_mode = ioc->port1_mode = BFI_PORT_MODE_ETH;
		ioc->asic_mode  = BFI_ASIC_MODE_ETH;
		ioc->port_mode = ioc->port_mode_cfg = BFA_MODE_CNA;
		ioc->ad_cap_bm = BFA_CM_CNA;
		break;

	case BFA_PCI_DEVICE_ID_CT_FC:
		ioc->asic_gen = BFI_ASIC_GEN_CT;
		ioc->fcmode = BFA_TRUE;
		ioc->port_mode = ioc->port_mode_cfg = BFA_MODE_HBA;
		ioc->ad_cap_bm = BFA_CM_HBA;
		break;

	case BFA_PCI_DEVICE_ID_CT2:
	case BFA_PCI_DEVICE_ID_CT2_QUAD:
		ioc->asic_gen = BFI_ASIC_GEN_CT2;
		if (clscode == BFI_PCIFN_CLASS_FC &&
		    pcidev->ssid == BFA_PCI_CT2_SSID_FC) {
			ioc->asic_mode  = BFI_ASIC_MODE_FC16;
			ioc->fcmode = BFA_TRUE;
			ioc->port_mode = ioc->port_mode_cfg = BFA_MODE_HBA;
			ioc->ad_cap_bm = BFA_CM_HBA;
		} else {
			ioc->port0_mode = ioc->port1_mode = BFI_PORT_MODE_ETH;
			ioc->asic_mode  = BFI_ASIC_MODE_ETH;
			if (pcidev->ssid == BFA_PCI_CT2_SSID_FCoE) {
				ioc->port_mode =
				ioc->port_mode_cfg = BFA_MODE_CNA;
				ioc->ad_cap_bm = BFA_CM_CNA;
			} else {
				ioc->port_mode =
				ioc->port_mode_cfg = BFA_MODE_NIC;
				ioc->ad_cap_bm = BFA_CM_NIC;
			}
		}
		break;

	default:
		bfa_assert(0);
	}

	/**
	 * Set asic specific interfaces. See bfa_ioc_cb.c and bfa_ioc_ct.c
	 */
	if (ioc->asic_gen == BFI_ASIC_GEN_CB)
		bfa_ioc_set_cb_hwif(ioc);
	else if (ioc->asic_gen == BFI_ASIC_GEN_CT)
		bfa_ioc_set_ct_hwif(ioc);
	else {
		bfa_assert(ioc->asic_gen == BFI_ASIC_GEN_CT2);
		bfa_ioc_set_ct2_hwif(ioc);
		bfa_ioc_ct2_poweron(ioc);
	}

	bfa_ioc_map_port(ioc);
	bfa_ioc_reg_init(ioc);
}

/**
 * Initialize IOC dma memory
 *
 * @param[in]	dm_kva	kernel virtual address of IOC dma memory
 * @param[in]	dm_pa	physical address of IOC dma memory
 */
void
bfa_ioc_mem_claim(struct bfa_ioc_s *ioc,  uint8_t *dm_kva, uint64_t dm_pa)
{
	/**
	 * dma memory for firmware attribute
	 */
	ioc->attr_dma.kva = dm_kva;
	ioc->attr_dma.pa = dm_pa;
	ioc->attr = (struct bfi_ioc_attr_s *) dm_kva;
}

/**
 * Return size of dma memory required.
 */
uint32_t
bfa_ioc_meminfo(void)
{
	return BFA_ROUNDUP(sizeof(struct bfi_ioc_attr_s), BFA_DMA_ALIGN_SZ);
}

void
bfa_ioc_enable(struct bfa_ioc_s *ioc)
{
	bfa_ioc_stats(ioc, ioc_enables);
	ioc->dbg_fwsave_once = BFA_TRUE;

	bfa_fsm_send_event(ioc, IOC_E_ENABLE);
}

void
bfa_ioc_disable(struct bfa_ioc_s *ioc)
{
	bfa_ioc_stats(ioc, ioc_disables);
	bfa_fsm_send_event(ioc, IOC_E_DISABLE);
}


void
bfa_ioc_suspend(struct bfa_ioc_s *ioc)
{
	ioc->dbg_fwsave_once = BFA_TRUE;
	bfa_fsm_send_event(ioc, IOC_E_HWERROR);
}

/**
 * Returns memory required for saving firmware trace in case of crash.
 * Driver must call this interface to allocate memory required for
 * automatic saving of firmware trace. Driver should call
 * bfa_ioc_debug_memclaim() right after bfa_ioc_attach() to setup this
 * trace memory.
 */
int
bfa_ioc_debug_trcsz(bfa_boolean_t auto_recover)
{
	return BFA_DBG_FWTRC_LEN;
}

/**
 * Initialize memory for saving firmware trace. Driver must initialize
 * trace memory before call bfa_ioc_enable().
 */
void
bfa_ioc_debug_memclaim(struct bfa_ioc_s *ioc, void *dbg_fwsave)
{
	ioc->dbg_fwsave	    = dbg_fwsave;
	ioc->dbg_fwsave_len = bfa_ioc_debug_trcsz(ioc->iocpf.auto_recover);
}

uint32_t
bfa_ioc_smem_pgnum(struct bfa_ioc_s *ioc, uint32_t fmaddr)
{
	return PSS_SMEM_PGNUM(ioc->ioc_regs.smem_pg0, fmaddr);
}

uint32_t
bfa_ioc_smem_pgoff(struct bfa_ioc_s *ioc, uint32_t fmaddr)
{
	return PSS_SMEM_PGOFF(fmaddr);
}

/**
 * Register mailbox message handler functions
 *
 * @param[in]	ioc		IOC instance
 * @param[in]	mcfuncs		message class handler functions
 */
void
bfa_ioc_mbox_register(struct bfa_ioc_s *ioc, bfa_ioc_mbox_mcfunc_t *mcfuncs)
{
	struct bfa_ioc_mbox_mod_s	*mod = &ioc->mbox_mod;
	int				mc;

	for (mc = 0; mc < BFI_MC_MAX; mc++)
		mod->mbhdlr[mc].cbfn = mcfuncs[mc];
}

/**
 * Register mailbox message handler function, to be called by common modules
 */
void
bfa_ioc_mbox_regisr(struct bfa_ioc_s *ioc, enum bfi_mclass mc,
		    bfa_ioc_mbox_mcfunc_t cbfn, void *cbarg)
{
	struct bfa_ioc_mbox_mod_s	*mod = &ioc->mbox_mod;

	mod->mbhdlr[mc].cbfn	= cbfn;
	mod->mbhdlr[mc].cbarg	= cbarg;
}

/**
 * Queue a mailbox command request to firmware. Waits if mailbox is busy.
 * Responsibility of caller to serialize
 *
 * @param[in]	ioc	IOC instance
 * @param[i]	cmd	Mailbox command
 */
bfa_boolean_t
bfa_ioc_mbox_queue(struct bfa_ioc_s *ioc, struct bfa_mbox_cmd_s *cmd,
		   bfa_mbox_cmd_cbfn_t cbfn, void *cbarg)
{
	struct bfa_ioc_mbox_mod_s	*mod = &ioc->mbox_mod;
	uint32_t			stat;

	cmd->cbfn = cbfn;
	cmd->cbarg = cbarg;

	/**
	 * If a previous command is pending, queue new command
	 */
	if (!bfa_q_is_empty(&mod->cmd_q)) {
		bfa_q_enq(&mod->cmd_q, &cmd->qe);
		return BFA_TRUE;
	}

	/**
	 * If mailbox is busy, queue command for poll timer
	 */
	stat = bfa_reg_read(ioc->ioc_regs.hfn_mbox_cmd);
	if (stat) {
		bfa_q_enq(&mod->cmd_q, &cmd->qe);
		return BFA_TRUE;
	}

	/**
	 * mailbox is free -- queue command to firmware
	 */
	bfa_ioc_mbox_send(ioc, cmd->msg, sizeof(cmd->msg));

	return BFA_FALSE;
}

/**
 * Handle mailbox interrupts
 */
void
bfa_ioc_mbox_isr(struct bfa_ioc_s *ioc)
{
	struct bfa_ioc_mbox_mod_s	*mod = &ioc->mbox_mod;
	struct bfi_mbmsg_s		m;
	int				mc;

	if (bfa_ioc_msgget(ioc, &m)) {

		/**
		 * Treat IOC message class as special.
		 */
		mc = m.mh.msg_class;
		if (mc == BFI_MC_IOC) {
			bfa_ioc_isr(ioc, &m);
			return;
		}

		if ((mc >= BFI_MC_MAX) || (mod->mbhdlr[mc].cbfn == NULL)) {
			bfa_trc(ioc, mc);
			return;
		}

		mod->mbhdlr[mc].cbfn(mod->mbhdlr[mc].cbarg, &m);
	}

	bfa_ioc_lpu_read_stat(ioc);

	/**
	 * Try to send pending mailbox commands
	 */
	bfa_ioc_mbox_poll(ioc);
}

void
bfa_ioc_error_isr(struct bfa_ioc_s *ioc)
{
	bfa_ioc_stats(ioc, ioc_hbfails);
	bfa_ioc_stats_hb_count(ioc, ioc->hb_count);

	bfa_fsm_send_event(ioc, IOC_E_HWERROR);
}

/**
 * return true if IOC is disabled
 */
bfa_boolean_t
bfa_ioc_is_disabled(struct bfa_ioc_s *ioc)
{
	return bfa_fsm_cmp_state(ioc, bfa_ioc_sm_disabling) ||
		bfa_fsm_cmp_state(ioc, bfa_ioc_sm_disabled);
}

/**
 * return true if IOC firmware is different.
 */
bfa_boolean_t
bfa_ioc_fw_mismatch(struct bfa_ioc_s *ioc)
{
	return bfa_fsm_cmp_state(ioc, bfa_ioc_sm_reset) ||
		bfa_fsm_cmp_state(&ioc->iocpf, bfa_iocpf_sm_fwcheck) ||
		bfa_fsm_cmp_state(&ioc->iocpf, bfa_iocpf_sm_mismatch);
}

/**
 * Check if adapter is disabled -- both IOCs should be in a disabled
 * state.
 */
bfa_boolean_t
bfa_ioc_adapter_is_disabled(struct bfa_ioc_s *ioc)
{
	uint32_t	ioc_state;

	if (!bfa_fsm_cmp_state(ioc, bfa_ioc_sm_disabled))
		return BFA_FALSE;

	ioc_state = bfa_ioc_get_cur_ioc_fwstate(ioc);
	if (!bfa_ioc_state_disabled(ioc_state))
		return BFA_FALSE;

	if (ioc->pcidev.device_id != BFA_PCI_DEVICE_ID_FC_8G1P) {
		ioc_state = bfa_ioc_get_cur_ioc_fwstate(ioc);
		if (!bfa_ioc_state_disabled(ioc_state))
			return BFA_FALSE;
	}

	return BFA_TRUE;
}

/**
 * Reset IOC fwstate registers.
 */
void
bfa_ioc_reset_fwstate(struct bfa_ioc_s *ioc)
{
	bfa_ioc_set_cur_ioc_fwstate(ioc, BFI_IOC_UNINIT);
	bfa_ioc_set_alt_ioc_fwstate(ioc, BFI_IOC_UNINIT);
}

/**
 * Add to IOC event notification queue. To be used by common
 * modules such as cee, port, diag.
 */
void
bfa_ioc_notify_register(struct bfa_ioc_s *ioc,
			struct bfa_ioc_notify_s *notify)
{
	bfa_q_enq(&ioc->notify_q, &notify->qe);
}

#define BFA_MFG_NAME "QLogic"
void
bfa_ioc_get_adapter_attr(struct bfa_ioc_s *ioc,
			 struct bfa_adapter_attr_s *ad_attr)
{
	struct bfi_ioc_attr_s	*ioc_attr;

	ioc_attr = ioc->attr;

	bfa_ioc_get_adapter_serial_num(ioc, ad_attr->serial_num);
	bfa_ioc_get_adapter_fw_ver(ioc, ad_attr->fw_ver);
	bfa_ioc_get_adapter_optrom_ver(ioc, ad_attr->optrom_ver);
	bfa_ioc_get_adapter_manufacturer(ioc, ad_attr->manufacturer);
	bfa_os_memcpy(&ad_attr->vpd, &ioc_attr->vpd,
		      sizeof(struct bfa_mfg_vpd_s));

	ad_attr->nports = bfa_ioc_get_nports(ioc);
	ad_attr->max_speed = bfa_ioc_speed_sup(ioc);

	bfa_ioc_get_adapter_model(ioc, ad_attr->model);
	/* For now, model descr uses same model string */
	bfa_ioc_get_adapter_model(ioc, ad_attr->model_descr);

	ad_attr->card_type = ioc_attr->card_type;
	ad_attr->is_mezz = bfa_mfg_is_mezz(ioc_attr->card_type);

	if (BFI_ADAPTER_IS_SPECIAL(ioc_attr->adapter_prop))
		ad_attr->prototype = 1;
	else
		ad_attr->prototype = 0;

	ad_attr->pwwn = bfa_ioc_get_pwwn(ioc);
	ad_attr->mac  = bfa_ioc_get_mac(ioc);

	ad_attr->pcie_gen = ioc_attr->pcie_gen;
	ad_attr->pcie_lanes = ioc_attr->pcie_lanes;
	ad_attr->pcie_lanes_orig = ioc_attr->pcie_lanes_orig;
	ad_attr->asic_rev = ioc_attr->asic_rev;

	bfa_ioc_get_pci_chip_rev(ioc, ad_attr->hw_ver);

	ad_attr->cna_capable = bfa_ioc_is_cna(ioc);
	ad_attr->trunk_capable = (ad_attr->nports > 1) &&
				 !bfa_ioc_is_cna(ioc) && !ad_attr->is_mezz;

	ad_attr->mfg_day = ioc_attr->mfg_day;
	ad_attr->mfg_month = ioc_attr->mfg_month;
	ad_attr->mfg_year = ioc_attr->mfg_year;
	bfa_os_memcpy(ad_attr->uuid, ioc_attr->uuid, BFA_ADAPTER_UUID_LEN);
}

enum bfa_ioc_type_e
bfa_ioc_get_type(struct bfa_ioc_s *ioc)
{
	if (ioc->clscode == BFI_PCIFN_CLASS_ETH)
		return BFA_IOC_TYPE_LL;

	bfa_assert(ioc->clscode == BFI_PCIFN_CLASS_FC);

	return (ioc->attr->port_mode == BFI_PORT_MODE_FC)
		? BFA_IOC_TYPE_FC : BFA_IOC_TYPE_FCoE;
}

void
bfa_ioc_get_adapter_serial_num(struct bfa_ioc_s *ioc, char *serial_num)
{
	bfa_os_memset((void *)serial_num, 0, BFA_ADAPTER_SERIAL_NUM_LEN);
	bfa_os_memcpy((void *)serial_num,
			(void *)ioc->attr->brcd_serialnum,
			BFA_ADAPTER_SERIAL_NUM_LEN);
}

void
bfa_ioc_get_adapter_fw_ver(struct bfa_ioc_s *ioc, char *fw_ver)
{
	bfa_os_memset((void *)fw_ver, 0, BFA_VERSION_LEN);
	bfa_os_memcpy(fw_ver, ioc->attr->fw_version, BFA_VERSION_LEN);
}

void
bfa_ioc_get_pci_chip_rev(struct bfa_ioc_s *ioc, char *chip_rev)
{
	bfa_assert(chip_rev);

	bfa_os_memset((void *)chip_rev, 0, BFA_IOC_CHIP_REV_LEN);

	chip_rev[0] = 'R';
	chip_rev[1] = 'e';
	chip_rev[2] = 'v';
	chip_rev[3] = '-';
	chip_rev[4] = ioc->attr->asic_rev;
	chip_rev[5] = '\0';
}

void
bfa_ioc_get_adapter_optrom_ver(struct bfa_ioc_s *ioc, char *optrom_ver)
{
	bfa_os_memset((void *)optrom_ver, 0, BFA_VERSION_LEN);
	bfa_os_memcpy(optrom_ver, ioc->attr->optrom_version,
		      BFA_VERSION_LEN);
}

void
bfa_ioc_get_adapter_manufacturer(struct bfa_ioc_s *ioc, char *manufacturer)
{
	bfa_os_memset((void *)manufacturer, 0, BFA_ADAPTER_MFG_NAME_LEN);
	bfa_os_memcpy(manufacturer, BFA_MFG_NAME, BFA_ADAPTER_MFG_NAME_LEN);
}

void
bfa_ioc_get_adapter_model(struct bfa_ioc_s *ioc, char *model)
{
	struct bfi_ioc_attr_s	*ioc_attr;
	uint8_t     nports = bfa_ioc_get_nports(ioc);

	bfa_assert(model);
	bfa_os_memset((void *)model, 0, BFA_ADAPTER_MODEL_NAME_LEN);

	ioc_attr = ioc->attr;

	if (bfa_asic_id_ct2(ioc->pcidev.device_id) &&
		(!bfa_mfg_is_mezz(ioc_attr->card_type)))
		bfa_os_snprintf(model, BFA_ADAPTER_MODEL_NAME_LEN, "%s-%u-%u%s",
				BFA_MFG_NAME, ioc_attr->card_type, nports, "p");
	else
		bfa_os_snprintf(model, BFA_ADAPTER_MODEL_NAME_LEN, "%s-%u",
				BFA_MFG_NAME, ioc_attr->card_type);
}

enum bfa_ioc_state
bfa_ioc_get_state(struct bfa_ioc_s *ioc)
{
	enum bfa_iocpf_state iocpf_st;
	enum bfa_ioc_state ioc_st = bfa_sm_to_state(ioc_sm_table, ioc->fsm);

	if (ioc_st == BFA_IOC_ENABLING ||
		ioc_st == BFA_IOC_FAIL || ioc_st == BFA_IOC_INITFAIL) {

		iocpf_st = bfa_sm_to_state(iocpf_sm_table, ioc->iocpf.fsm);

		switch (iocpf_st) {
		case BFA_IOCPF_SEMWAIT:
			ioc_st = BFA_IOC_SEMWAIT;
			break;

		case BFA_IOCPF_HWINIT:
			ioc_st = BFA_IOC_HWINIT;
			break;

		case BFA_IOCPF_FWMISMATCH:
			ioc_st = BFA_IOC_FWMISMATCH;
			break;

		case BFA_IOCPF_FAIL:
			ioc_st = BFA_IOC_FAIL;
			break;

		case BFA_IOCPF_INITFAIL:
			ioc_st = BFA_IOC_INITFAIL;
			break;

		default:
			break;
		}
	}

	return ioc_st;
}

void
bfa_ioc_get_attr(struct bfa_ioc_s *ioc, struct bfa_ioc_attr_s *ioc_attr)
{
	bfa_os_memset((void *)ioc_attr, 0, sizeof(struct bfa_ioc_attr_s));

	ioc_attr->state = bfa_ioc_get_state(ioc);
	ioc_attr->port_id = bfa_ioc_portid(ioc);
	ioc_attr->port_mode = ioc->port_mode;
	ioc_attr->port_mode_cfg = ioc->port_mode_cfg;
	ioc_attr->cap_bm = ioc->ad_cap_bm;

	ioc_attr->ioc_type = bfa_ioc_get_type(ioc);

	bfa_ioc_get_adapter_attr(ioc, &ioc_attr->adapter_attr);

	ioc_attr->pci_attr.device_id = bfa_ioc_devid(ioc);
	ioc_attr->pci_attr.pcifn = bfa_ioc_pcifn(ioc);
	ioc_attr->def_fn = bfa_ioc_is_default(ioc);
	bfa_ioc_get_pci_chip_rev(ioc, ioc_attr->pci_attr.chip_rev);
}

/**
 * @ig hal_wwn_public
 * @{
 */
uint32_t
bfa_ioc_get_card_type(struct bfa_ioc_s *ioc)
{
	return ioc->attr->card_type;
}

wwn_t
bfa_ioc_get_pwwn(struct bfa_ioc_s *ioc)
{
	return ioc->attr->pwwn;
}

wwn_t
bfa_ioc_get_nwwn(struct bfa_ioc_s *ioc)
{
	return ioc->attr->nwwn;
}

uint64_t
bfa_ioc_get_adid(struct bfa_ioc_s *ioc)
{
	return ioc->attr->mfg_pwwn;
}

mac_t
bfa_ioc_get_mac(struct bfa_ioc_s *ioc)
{
	/*
	 * Check the IOC type and return the appropriate MAC
	 */
	if (bfa_ioc_get_type(ioc) == BFA_IOC_TYPE_FCoE)
		return ioc->attr->fcoe_mac;
	else
		return ioc->attr->mac;
}

wwn_t
bfa_ioc_get_mfg_pwwn(struct bfa_ioc_s *ioc)
{
	return ioc->attr->mfg_pwwn;
}

wwn_t
bfa_ioc_get_mfg_nwwn(struct bfa_ioc_s *ioc)
{
	return ioc->attr->mfg_nwwn;
}

mac_t
bfa_ioc_get_mfg_mac(struct bfa_ioc_s *ioc)
{
	mac_t	m;

	m = ioc->attr->mfg_mac;
	if (bfa_mfg_is_old_wwn_mac_model(ioc->attr->card_type))
		m.mac[MAC_ADDRLEN - 1] += bfa_ioc_pcifn(ioc);
	else
		bfa_mfg_increment_wwn_mac(&(m.mac[MAC_ADDRLEN-3]),
			bfa_ioc_pcifn(ioc));

	return m;
}

void
bfa_ioc_set_attr_wwn(struct bfa_ioc_s *ioc, wwn_t pwwn, wwn_t nwwn)
{
	ioc->attr->pwwn = pwwn;
	ioc->attr->nwwn = nwwn;
}

/**
 * Send AEN notification
 */
void
bfa_ioc_aen_post(struct bfa_ioc_s *ioc, enum bfa_ioc_aen_event event)
{
	union bfa_aen_data_u  aen_data;
	struct bfa_log_mod_s *logmod = ioc->logm;
	enum bfa_ioc_type_e ioc_type;
	char	buf[BFA_STRING_32];
	char	*buf_ptr = NULL;

	memset(&aen_data.ioc.pwwn, 0, sizeof(aen_data.ioc.pwwn));
	memset(&aen_data.ioc.mac, 0, sizeof(aen_data.ioc.mac));

	ioc_type = bfa_ioc_get_type(ioc);
	switch (ioc_type) {
	case BFA_IOC_TYPE_FC:
		aen_data.ioc.pwwn = bfa_ioc_get_pwwn(ioc);
		buf_ptr = wwn2str(buf, sizeof(buf), aen_data.ioc.pwwn);
		break;
	case BFA_IOC_TYPE_FCoE:
		aen_data.ioc.pwwn = bfa_ioc_get_pwwn(ioc);
		aen_data.ioc.mac = bfa_ioc_get_mac(ioc);
		buf_ptr = wwn2str(buf, sizeof(buf), aen_data.ioc.pwwn);
		break;
	case BFA_IOC_TYPE_LL:
		aen_data.ioc.mac = bfa_ioc_get_mac(ioc);
		buf_ptr = mac2str(buf, sizeof(buf), aen_data.ioc.mac);
		break;
	default:
		bfa_assert(ioc_type == BFA_IOC_TYPE_FC);
		break;
	}

	bfa_log(logmod, BFA_LOG_CREATE_ID(BFA_AEN_CAT_IOC, event), buf_ptr);

	aen_data.ioc.ioc_type = ioc_type;
	bfa_aen_post(ioc->aen, BFA_AEN_CAT_IOC, event, &aen_data);
}

/**
 * Retrieve saved firmware trace from a prior IOC failure.
 */
bfa_status_t
bfa_ioc_debug_fwsave(struct bfa_ioc_s *ioc, void *trcdata, int *trclen)
{
	int	tlen;

	if (ioc->dbg_fwsave_len == 0)
		return BFA_STATUS_ENOFSAVE;

	tlen = *trclen;
	if (tlen > ioc->dbg_fwsave_len)
		tlen = ioc->dbg_fwsave_len;

	bfa_os_memcpy(trcdata, ioc->dbg_fwsave, tlen);
	*trclen = tlen;
	return BFA_STATUS_OK;
}

/**
 * Clear saved firmware trace
 */
void
bfa_ioc_debug_fwsave_clear(struct bfa_ioc_s *ioc)
{
	ioc->dbg_fwsave_once = BFA_TRUE;
}

/**
 * Retrieve saved firmware trace from a prior IOC failure.
 */
bfa_status_t
bfa_ioc_debug_fwtrc(struct bfa_ioc_s *ioc, void *trcdata, int *trclen)
{
	uint32_t loff = BFA_DBG_FWTRC_OFF(bfa_ioc_portid(ioc));
	int tlen;
	bfa_status_t status;

	bfa_trc(ioc, *trclen);

	tlen = *trclen;
	if (tlen > BFA_DBG_FWTRC_LEN)
		tlen = BFA_DBG_FWTRC_LEN;

	status = bfa_ioc_smem_read(ioc, trcdata, loff, tlen);
	*trclen = tlen;
	return status;
}

static void
bfa_ioc_send_fwsync(struct bfa_ioc_s *ioc)
{
	struct bfa_mbox_cmd_s cmd;
	struct bfi_ioc_ctrl_req_s *req = (struct bfi_ioc_ctrl_req_s *) cmd.msg;

	bfi_h2i_set(req->mh, BFI_MC_IOC, BFI_IOC_H2I_DBG_SYNC,
		    bfa_ioc_portid(ioc));
	req->clscode = bfa_os_htons(ioc->clscode);
	bfa_ioc_mbox_queue(ioc, &cmd, NULL, NULL);
}

static void
bfa_ioc_fwsync(struct bfa_ioc_s *ioc)
{
	uint32_t fwsync_iter = 1000;

	bfa_ioc_send_fwsync(ioc);

	/**
	 * After sending a fw sync mbox command wait for it to
	 * take effect.  We will not wait for a response because
	 *    1. fw_sync mbox cmd doesn't have a response.
	 *    2. Even if we implement that,  interrupts might not
	 *	 be enabled when we call this function.
	 * So, just keep checking if any mbox cmd is pending, and
	 * after waiting for a reasonable amount of time, go ahead.
	 * It is possible that fw has crashed and the mbox command
	 * is never acknowledged.
	 */
	while (bfa_ioc_mbox_cmd_pending(ioc) && fwsync_iter > 0)
		fwsync_iter--;
}

/**
 * Dump firmware smem
 */
bfa_status_t
bfa_ioc_debug_fwcore(struct bfa_ioc_s *ioc, void *buf,
				uint32_t *offset, int *buflen)
{
	uint32_t loff;
	int dlen;
	bfa_status_t status;
	uint32_t smem_len = BFA_IOC_FW_SMEM_SIZE(ioc);

	if (*offset >= smem_len) {
		*offset = *buflen = 0;
		return BFA_STATUS_EINVAL;
	}

	loff = *offset;
	dlen = *buflen;

	/**
	 * First smem read, sync smem before proceeding
	 * No need to sync before reading every chunk.
	 */
	if (loff == 0)
		bfa_ioc_fwsync(ioc);

	if ((loff + dlen) >= smem_len)
		dlen = smem_len - loff;

	status = bfa_ioc_smem_read(ioc, buf, loff, dlen);

	if (status != BFA_STATUS_OK) {
		*offset = *buflen = 0;
		return status;
	}

	*offset += dlen;

	if (*offset >= smem_len)
		*offset = 0;

	*buflen = dlen;

	return status;
}

/**
 * Firmware statistics
 */
bfa_status_t
bfa_ioc_fw_stats_get(struct bfa_ioc_s *ioc, void *stats)
{
	uint32_t loff = BFI_IOC_FWSTATS_OFF + \
		BFI_IOC_FWSTATS_SZ * (bfa_ioc_portid(ioc));
	int tlen;
	bfa_status_t status;

	if (ioc->stats_busy) {
		bfa_trc(ioc, ioc->stats_busy);
		return BFA_STATUS_DEVBUSY;
	}
	ioc->stats_busy = BFA_TRUE;

	tlen = sizeof(bfa_fw_stats_t);
	status = bfa_ioc_smem_read(ioc, stats, loff, tlen);

	ioc->stats_busy = BFA_FALSE;
	return status;
}

bfa_status_t
bfa_ioc_fw_stats_clear(struct bfa_ioc_s *ioc)
{
	uint32_t loff = BFI_IOC_FWSTATS_OFF + \
		BFI_IOC_FWSTATS_SZ * (bfa_ioc_portid(ioc));
	int tlen;
	bfa_status_t status;

	if (ioc->stats_busy) {
		bfa_trc(ioc, ioc->stats_busy);
		return BFA_STATUS_DEVBUSY;
	}
	ioc->stats_busy = BFA_TRUE;

	tlen = sizeof(bfa_fw_stats_t);
	status = bfa_ioc_smem_clr(ioc, loff, tlen);

	ioc->stats_busy = BFA_FALSE;
	return status;
}

/**
 * Save firmware trace if configured.
 */
void
bfa_ioc_debug_save_ftrc(struct bfa_ioc_s *ioc)
{
	int		tlen;

	if (ioc->dbg_fwsave_once) {
		ioc->dbg_fwsave_once = BFA_FALSE;
		if (ioc->dbg_fwsave_len) {
			tlen = ioc->dbg_fwsave_len;
			bfa_ioc_debug_fwtrc(ioc, ioc->dbg_fwsave, &tlen);
		}
	}
}

static void
bfa_ioc_fail_notify(struct bfa_ioc_s *ioc)
{
	/**
	 * Notify driver and common modules registered for notification.
	 */
	ioc->cbfn->hbfail_cbfn(ioc->bfa);
	bfa_ioc_event_notify(ioc, BFA_IOC_E_FAILED);

	bfa_ioc_debug_save_ftrc(ioc);

	bfa_ioc_aen_post(ioc, BFA_IOC_AEN_HBFAIL);
}

/**
 * Firmware failure detected. Start recovery actions.
 */
static void
bfa_ioc_recover(struct bfa_ioc_s *ioc)
{
	bfa_ioc_stats(ioc, ioc_hbfails);
	bfa_ioc_stats_hb_count(ioc, ioc->hb_count);

	bfa_fsm_send_event(ioc, IOC_E_HBFAIL);
}

/**
 * @dg hal_iocpf_pvt BFA IOC PF private functions
 * @{
 */

static void
bfa_iocpf_enable(struct bfa_ioc_s *ioc)
{
	bfa_fsm_send_event(&ioc->iocpf, IOCPF_E_ENABLE);
}

static void
bfa_iocpf_disable(struct bfa_ioc_s *ioc)
{
	bfa_fsm_send_event(&ioc->iocpf, IOCPF_E_DISABLE);
}

static void
bfa_iocpf_fail(struct bfa_ioc_s *ioc)
{
	bfa_fsm_send_event(&ioc->iocpf, IOCPF_E_FAIL);
}

static void
bfa_iocpf_initfail(struct bfa_ioc_s *ioc)
{
	bfa_fsm_send_event(&ioc->iocpf, IOCPF_E_INITFAIL);
}

static void
bfa_iocpf_getattrfail(struct bfa_ioc_s *ioc)
{
	bfa_fsm_send_event(&ioc->iocpf, IOCPF_E_GETATTRFAIL);
}

static void
bfa_iocpf_stop(struct bfa_ioc_s *ioc)
{
	bfa_fsm_send_event(&ioc->iocpf, IOCPF_E_STOP);
}

static void
bfa_iocpf_timeout(void *ioc_arg)
{
	struct bfa_ioc_s  *ioc = (struct bfa_ioc_s *) ioc_arg;

	bfa_trc(ioc, 0);
	bfa_fsm_send_event(&ioc->iocpf, IOCPF_E_TIMEOUT);
}

static void
bfa_iocpf_sem_timeout(void *ioc_arg)
{
	struct bfa_ioc_s  *ioc = (struct bfa_ioc_s *) ioc_arg;

	bfa_ioc_hw_sem_get(ioc);
}

static void
bfa_ioc_poll_fwinit(struct bfa_ioc_s *ioc)
{
	uint32_t fwstate = bfa_ioc_get_cur_ioc_fwstate(ioc);

	bfa_trc(ioc, fwstate);

	if (fwstate == BFI_IOC_DISABLED) {
		bfa_fsm_send_event(&ioc->iocpf, IOCPF_E_FWREADY);
		return;
	}

	if (ioc->iocpf.poll_time >= (3 * BFA_IOC_TOV)) {
		bfa_iocpf_timeout(ioc);
	} else {
		ioc->iocpf.poll_time += BFA_IOC_POLL_TOV;
		bfa_iocpf_poll_timer_start(ioc);
	}
}

static void
bfa_iocpf_poll_timeout(void *ioc_arg)
{
	struct bfa_ioc_s  *ioc = (struct bfa_ioc_s *) ioc_arg;

	bfa_ioc_poll_fwinit(ioc);
}

/**
 * @}
 */


/**
 * @dg bfa timer function
 * @{
 */
void
bfa_timer_init(struct bfa_timer_mod_s *mod)
{
	bfa_q_init(&mod->timer_q);
}

void
bfa_timer_beat(struct bfa_timer_mod_s *mod)
{
	struct bfa_q_s *qh = &mod->timer_q;
	struct bfa_q_s *qe, *qe_next;
	struct bfa_timer_s *elem;
	struct bfa_q_s timedout_q;

	bfa_q_init(&timedout_q);

	qe = bfa_q_next(qh);

	while (qe != qh) {
		qe_next = bfa_q_next(qe);

		elem = (struct bfa_timer_s *) qe;
		if (elem->timeout <= BFA_TIMER_FREQ) {
			elem->timeout = 0;
			bfa_q_qe_deq(&elem->qe);
			bfa_q_enq(&timedout_q, &elem->qe);
		} else {
			elem->timeout -= BFA_TIMER_FREQ;
		}

		qe = qe_next;	/* go to next elem */
	}

	/*
	 * Pop all the timeout entries
	 */
	while (!bfa_q_is_empty(&timedout_q)) {
		bfa_q_deq(&timedout_q, &elem);
		elem->timercb(elem->arg);
	}
}

/**
 * Should be called with lock protection
 */
void
bfa_timer_begin(struct bfa_timer_mod_s *mod, struct bfa_timer_s *timer,
		    void (*timercb) (void *), void *arg, unsigned int timeout)
{

	bfa_assert(timercb != NULL);
	bfa_assert(!bfa_q_is_on_q(&mod->timer_q, timer));

	timer->timeout = timeout;
	timer->timercb = timercb;
	timer->arg = arg;

	bfa_q_enq(&mod->timer_q, &timer->qe);
}

/**
 * Should be called with lock protection
 */
void
bfa_timer_stop(struct bfa_timer_s *timer)
{
	bfa_assert(!bfa_q_is_empty(&timer->qe));

	bfa_q_qe_deq(&timer->qe);
}

/**
 * @}
 */

/**
 * @dg bfa asic block configuration
 * @{
 */
static void
bfa_ioc_upd_port_mode_cfg(struct bfa_ioc_s *ioc, uint8_t port_mode)
{
	ioc->port_mode_cfg = port_mode;
}

/**
 * @brief
 * Converts multi-byte BIG Endian fields in bfa_ablk_cfg_s to Little Endian
 */
static void
bfa_ablk_config_le(struct bfa_ablk_cfg_s *cfg)
{
	struct bfa_ablk_cfg_inst_s *cfg_inst;
	int i, j;
	uint16_t be16;

	for (i = 0; i < BFA_ABLK_MAX; i++) {
		cfg_inst = &cfg->inst[i];
		for (j = 0; j < BFA_ABLK_MAX_PFS; j++) {
			be16 = cfg_inst->pf_cfg[j].pers;
			cfg_inst->pf_cfg[j].pers = bfa_os_ntohs(be16);
			be16 = cfg_inst->pf_cfg[j].num_qpairs;
			cfg_inst->pf_cfg[j].num_qpairs = bfa_os_ntohs(be16);
			be16 = cfg_inst->pf_cfg[j].num_vectors;
			cfg_inst->pf_cfg[j].num_vectors = bfa_os_ntohs(be16);
			be16 = cfg_inst->pf_cfg[j].bw_min;
			cfg_inst->pf_cfg[j].bw_min = bfa_os_ntohs(be16);
			be16 = cfg_inst->pf_cfg[j].bw_max;
			cfg_inst->pf_cfg[j].bw_max = bfa_os_ntohs(be16);
		}
	}
}

/**
 * @brief
 * Converts multi-byte BIG Endian fields in bfa_nwpart_cfg_t to Little Endian
 */
static void
bfa_ablk_nw_config_le(bfa_nwpart_cfg_t *nw_cfg)
{
	int i, j;
	uint16_t be16;

	for (i = 0; i < BFA_ABLK_MAX_PORTS; ++i) {
		for (j = 0; j < BFA_NWPART_MAX_PFS_PER_PORT; ++j) {
			be16 = nw_cfg->port[i].vnics[j].pers;
			nw_cfg->port[i].vnics[j].pers = bfa_os_ntohs(be16);
		}
	}
}

static void
bfa_ablk_isr(void *cbarg, struct bfi_mbmsg_s *msg)
{
	struct bfa_ablk_s *ablk = (struct bfa_ablk_s *)cbarg;
	struct bfi_ablk_i2h_rsp_s *rsp = (struct bfi_ablk_i2h_rsp_s *)msg;
	bfa_ablk_cbfn_t cbfn;

	bfa_assert(msg->mh.msg_class == BFI_MC_ABLK);
	bfa_trc(ablk->ioc, msg->mh.msg_id);

	switch (msg->mh.msg_id) {
	case BFI_ABLK_I2H_QUERY:
		if (rsp->status == BFA_STATUS_OK) {
			memcpy(ablk->cfg, ablk->dma_addr.kva,
				sizeof(struct bfa_ablk_cfg_s));
			bfa_ablk_config_le(ablk->cfg);
			ablk->cfg = NULL;
		}
		break;

	case BFI_ABLK_I2H_NWPAR_QUERY:
		if (rsp->status == BFA_STATUS_OK) {
			memcpy(ablk->nwpart_cfg, ablk->dma_addr.kva,
				sizeof(bfa_nwpart_cfg_t));
			bfa_ablk_nw_config_le(ablk->nwpart_cfg);
			ablk->nwpart_cfg = NULL;
		}
		break;

	case BFI_ABLK_I2H_ADPT_CONFIG:
	case BFI_ABLK_I2H_PORT_CONFIG:
		/* update config port mode */
		bfa_ioc_upd_port_mode_cfg(ablk->ioc, rsp->port_mode);
		break;

	case BFI_ABLK_I2H_PF_DELETE:
	case BFI_ABLK_I2H_PF_UPDATE:
	case BFI_ABLK_I2H_OPTROM_ENABLE:
	case BFI_ABLK_I2H_OPTROM_DISABLE:
	case BFI_ABLK_I2H_NWPAR_ENABLE:
	case BFI_ABLK_I2H_NWPAR_DISABLE:
	case BFI_ABLK_I2H_NWPAR_PF_ENBL:
	case BFI_ABLK_I2H_NWPAR_PF_DSBL:
	case BFI_ABLK_I2H_NWPAR_PF_PERS:
		/* No-op */
		break;

	case BFI_ABLK_I2H_PF_CREATE:
		*(ablk->pcifn) = rsp->pcifn;
		ablk->pcifn = NULL;
		break;

	default:
		bfa_assert(0);
	}

	ablk->busy = BFA_FALSE;
	if (ablk->cbfn) {
		cbfn = ablk->cbfn;
		ablk->cbfn = NULL;
		cbfn(ablk->cbarg, rsp->status);
	}
}

static void
bfa_ablk_notify(void *cbarg, enum bfa_ioc_event_e event)
{
	struct bfa_ablk_s *ablk = (struct bfa_ablk_s *)cbarg;

	bfa_trc(ablk->ioc, event);

	switch (event) {
	case BFA_IOC_E_ENABLED:
		bfa_assert(ablk->busy == BFA_FALSE);
		break;

	case BFA_IOC_E_DISABLED:
	case BFA_IOC_E_FAILED:
		/* Fail any pending requests */
		ablk->pcifn = NULL;
		if (ablk->busy) {
			if (ablk->cbfn)
				ablk->cbfn(ablk->cbarg, BFA_STATUS_FAILED);
			ablk->cbfn = NULL;
			ablk->busy = BFA_FALSE;
		}
		break;

	default:
		bfa_assert(0);
		break;
	}
}

uint32_t
bfa_ablk_meminfo(void)
{
	return BFA_ROUNDUP(sizeof(struct bfa_ablk_cfg_s), BFA_DMA_ALIGN_SZ);
}

void
bfa_ablk_memclaim(struct bfa_ablk_s *ablk, uint8_t *dma_kva, uint64_t dma_pa)
{
	ablk->dma_addr.kva = dma_kva;
	ablk->dma_addr.pa  = dma_pa;
}

void
bfa_ablk_attach(struct bfa_ablk_s *ablk, struct bfa_ioc_s *ioc)
{
	ablk->ioc = ioc;

	bfa_ioc_mbox_regisr(ablk->ioc, BFI_MC_ABLK, bfa_ablk_isr, ablk);
	bfa_q_qe_init(&ablk->ioc_notify);
	bfa_ioc_notify_init(&ablk->ioc_notify, bfa_ablk_notify, ablk);
	bfa_ioc_notify_register(ablk->ioc, &ablk->ioc_notify);
}

bfa_status_t
bfa_ablk_query(struct bfa_ablk_s *ablk, struct bfa_ablk_cfg_s *ablk_cfg,
		bfa_ablk_cbfn_t cbfn, void *cbarg)
{
	struct bfi_ablk_h2i_query_s *m;

	bfa_assert(ablk_cfg);

	if (!bfa_ioc_is_operational(ablk->ioc)) {
		bfa_trc(ablk->ioc, BFA_STATUS_IOC_FAILURE);
		return BFA_STATUS_IOC_FAILURE;
	}

	if (ablk->busy) {
		bfa_trc(ablk->ioc, BFA_STATUS_DEVBUSY);
		return 	BFA_STATUS_DEVBUSY;
	}

	ablk->cfg = ablk_cfg;
	ablk->cbfn  = cbfn;
	ablk->cbarg = cbarg;
	ablk->busy  = BFA_TRUE;

	m = (struct bfi_ablk_h2i_query_s *)ablk->mb.msg;
	bfi_h2i_set(m->mh, BFI_MC_ABLK, BFI_ABLK_H2I_QUERY,
		    bfa_ioc_portid(ablk->ioc));
	bfa_dma_be_addr_set(m->addr, ablk->dma_addr.pa);
	bfa_ioc_mbox_queue(ablk->ioc, &ablk->mb, NULL, NULL);

	return BFA_STATUS_OK;
}

bfa_status_t
bfa_nwpart_cfg_query(struct bfa_ablk_s *ablk,
		bfa_nwpart_cfg_t *nwpar_cfg,
		bfa_ablk_cbfn_t cbfn, void *cbarg)
{
	struct bfi_ablk_h2i_query_s *m;

	bfa_assert(nwpar_cfg);

	if (!bfa_ioc_is_operational(ablk->ioc)) {
		bfa_trc(ablk->ioc, BFA_STATUS_IOC_FAILURE);
		return BFA_STATUS_IOC_FAILURE;
	}

	if (ablk->busy) {
		bfa_trc(ablk->ioc, BFA_STATUS_DEVBUSY);
		return 	BFA_STATUS_DEVBUSY;
	}

	ablk->nwpart_cfg = nwpar_cfg;
	ablk->cbfn  = cbfn;
	ablk->cbarg = cbarg;
	ablk->busy  = BFA_TRUE;

	m = (struct bfi_ablk_h2i_query_s *)ablk->mb.msg;
	bfi_h2i_set(m->mh, BFI_MC_ABLK, BFI_ABLK_H2I_NWPAR_QUERY,
		    bfa_ioc_portid(ablk->ioc));
	bfa_dma_be_addr_set(m->addr, ablk->dma_addr.pa);
	bfa_ioc_mbox_queue(ablk->ioc, &ablk->mb, NULL, NULL);

	return BFA_STATUS_OK;
}

bfa_status_t
bfa_ablk_adapter_config(struct bfa_ablk_s *ablk, bfa_mode_t mode,
		int max_pf, int max_vf, bfa_ablk_cbfn_t cbfn, void *cbarg)
{
	struct bfi_ablk_h2i_cfg_req_s *m;

	if (!bfa_ioc_is_operational(ablk->ioc)) {
		bfa_trc(ablk->ioc, BFA_STATUS_IOC_FAILURE);
		return BFA_STATUS_IOC_FAILURE;
	}

	if (ablk->busy) {
		bfa_trc(ablk->ioc, BFA_STATUS_DEVBUSY);
		return 	BFA_STATUS_DEVBUSY;
	}

	ablk->cbfn  = cbfn;
	ablk->cbarg = cbarg;
	ablk->busy  = BFA_TRUE;

	m = (struct bfi_ablk_h2i_cfg_req_s *)ablk->mb.msg;
	bfi_h2i_set(m->mh, BFI_MC_ABLK, BFI_ABLK_H2I_ADPT_CONFIG,
		    bfa_ioc_portid(ablk->ioc));
	m->mode = (uint8_t)mode;
	m->max_pf = (uint8_t)max_pf;
	m->max_vf = (uint8_t)max_vf;
	bfa_ioc_mbox_queue(ablk->ioc, &ablk->mb, NULL, NULL);

	return BFA_STATUS_OK;
}

bfa_status_t
bfa_ablk_port_config(struct bfa_ablk_s *ablk, int port, bfa_mode_t mode,
		int max_pf, int max_vf, bfa_ablk_cbfn_t cbfn, void *cbarg)
{
	struct bfi_ablk_h2i_cfg_req_s *m;

	if (!bfa_ioc_is_operational(ablk->ioc)) {
		bfa_trc(ablk->ioc, BFA_STATUS_IOC_FAILURE);
		return BFA_STATUS_IOC_FAILURE;
	}

	if (ablk->busy) {
		bfa_trc(ablk->ioc, BFA_STATUS_DEVBUSY);
		return 	BFA_STATUS_DEVBUSY;
	}

	ablk->cbfn  = cbfn;
	ablk->cbarg = cbarg;
	ablk->busy  = BFA_TRUE;

	m = (struct bfi_ablk_h2i_cfg_req_s *)ablk->mb.msg;
	bfi_h2i_set(m->mh, BFI_MC_ABLK, BFI_ABLK_H2I_PORT_CONFIG,
		    bfa_ioc_portid(ablk->ioc));
	m->port = (uint8_t)port;
	m->mode = (uint8_t)mode;
	m->max_pf = (uint8_t)max_pf;
	m->max_vf = (uint8_t)max_vf;
	bfa_ioc_mbox_queue(ablk->ioc, &ablk->mb, NULL, NULL);

	return BFA_STATUS_OK;
}

bfa_status_t
bfa_ablk_pf_create(struct bfa_ablk_s *ablk, uint16_t *pcifn,
		uint8_t port, enum bfi_pcifn_class personality,
		uint16_t bw_min, uint16_t bw_max,
		bfa_ablk_cbfn_t cbfn, void *cbarg)
{
	struct bfi_ablk_h2i_pf_req_s *m;

	if (!bfa_ioc_is_operational(ablk->ioc)) {
		bfa_trc(ablk->ioc, BFA_STATUS_IOC_FAILURE);
		return BFA_STATUS_IOC_FAILURE;
	}

	if (ablk->busy) {
		bfa_trc(ablk->ioc, BFA_STATUS_DEVBUSY);
		return 	BFA_STATUS_DEVBUSY;
	}

	ablk->pcifn = pcifn;
	ablk->cbfn = cbfn;
	ablk->cbarg = cbarg;
	ablk->busy  = BFA_TRUE;

	m = (struct bfi_ablk_h2i_pf_req_s *)ablk->mb.msg;
	bfi_h2i_set(m->mh, BFI_MC_ABLK, BFI_ABLK_H2I_PF_CREATE,
		    bfa_ioc_portid(ablk->ioc));
	m->pers = bfa_os_htons((uint16_t)personality);
	m->bw_min = bfa_os_htons(bw_min);
	m->bw_max = bfa_os_htons(bw_max);
	m->port = port;
	bfa_ioc_mbox_queue(ablk->ioc, &ablk->mb, NULL, NULL);

	return BFA_STATUS_OK;
}

bfa_status_t
bfa_ablk_pf_delete(struct bfa_ablk_s *ablk, int pcifn, bfa_ablk_cbfn_t cbfn,
		void *cbarg)
{
	struct bfi_ablk_h2i_pf_req_s *m;

	if (!bfa_ioc_is_operational(ablk->ioc)) {
		bfa_trc(ablk->ioc, BFA_STATUS_IOC_FAILURE);
		return BFA_STATUS_IOC_FAILURE;
	}

	if (ablk->busy) {
		bfa_trc(ablk->ioc, BFA_STATUS_DEVBUSY);
		return 	BFA_STATUS_DEVBUSY;
	}

	ablk->cbfn  = cbfn;
	ablk->cbarg = cbarg;
	ablk->busy  = BFA_TRUE;

	m = (struct bfi_ablk_h2i_pf_req_s *)ablk->mb.msg;
	bfi_h2i_set(m->mh, BFI_MC_ABLK, BFI_ABLK_H2I_PF_DELETE,
		    bfa_ioc_portid(ablk->ioc));
	m->pcifn = (uint8_t)pcifn;
	bfa_ioc_mbox_queue(ablk->ioc, &ablk->mb, NULL, NULL);

	return BFA_STATUS_OK;
}

bfa_status_t
bfa_ablk_pf_update(struct bfa_ablk_s *ablk, int pcifn, uint16_t bw_min,
		uint16_t bw_max, bfa_ablk_cbfn_t cbfn, void *cbarg)
{
	struct bfi_ablk_h2i_pf_req_s *m;

	if (!bfa_ioc_is_operational(ablk->ioc)) {
		bfa_trc(ablk->ioc, BFA_STATUS_IOC_FAILURE);
		return BFA_STATUS_IOC_FAILURE;
	}

	if (ablk->busy) {
		bfa_trc(ablk->ioc, BFA_STATUS_DEVBUSY);
		return 	BFA_STATUS_DEVBUSY;
	}

	ablk->cbfn  = cbfn;
	ablk->cbarg = cbarg;
	ablk->busy  = BFA_TRUE;

	m = (struct bfi_ablk_h2i_pf_req_s *)ablk->mb.msg;
	bfi_h2i_set(m->mh, BFI_MC_ABLK, BFI_ABLK_H2I_PF_UPDATE,
		    bfa_ioc_portid(ablk->ioc));
	m->pcifn = (uint8_t)pcifn;
	m->bw_min = bfa_os_htons(bw_min);
	m->bw_max = bfa_os_htons(bw_max);
	bfa_ioc_mbox_queue(ablk->ioc, &ablk->mb, NULL, NULL);

	return BFA_STATUS_OK;
}

bfa_status_t
bfa_ablk_optrom_en(struct bfa_ablk_s *ablk, bfa_ablk_cbfn_t cbfn, void *cbarg)
{
	struct bfi_ablk_h2i_optrom_s *m;

	if (!bfa_ioc_is_operational(ablk->ioc)) {
		bfa_trc(ablk->ioc, BFA_STATUS_IOC_FAILURE);
		return BFA_STATUS_IOC_FAILURE;
	}

	if (ablk->busy) {
		bfa_trc(ablk->ioc, BFA_STATUS_DEVBUSY);
		return 	BFA_STATUS_DEVBUSY;
	}

	ablk->cbfn  = cbfn;
	ablk->cbarg = cbarg;
	ablk->busy  = BFA_TRUE;

	m = (struct bfi_ablk_h2i_optrom_s *)ablk->mb.msg;
	bfi_h2i_set(m->mh, BFI_MC_ABLK, BFI_ABLK_H2I_OPTROM_ENABLE,
		    bfa_ioc_portid(ablk->ioc));
	bfa_ioc_mbox_queue(ablk->ioc, &ablk->mb, NULL, NULL);

	return BFA_STATUS_OK;
}

bfa_status_t
bfa_ablk_optrom_dis(struct bfa_ablk_s *ablk, bfa_ablk_cbfn_t cbfn, void *cbarg)
{
	struct bfi_ablk_h2i_optrom_s *m;

	if (!bfa_ioc_is_operational(ablk->ioc)) {
		bfa_trc(ablk->ioc, BFA_STATUS_IOC_FAILURE);
		return BFA_STATUS_IOC_FAILURE;
	}

	if (ablk->busy) {
		bfa_trc(ablk->ioc, BFA_STATUS_DEVBUSY);
		return 	BFA_STATUS_DEVBUSY;
	}

	ablk->cbfn  = cbfn;
	ablk->cbarg = cbarg;
	ablk->busy  = BFA_TRUE;

	m = (struct bfi_ablk_h2i_optrom_s *)ablk->mb.msg;
	bfi_h2i_set(m->mh, BFI_MC_ABLK, BFI_ABLK_H2I_OPTROM_DISABLE,
		    bfa_ioc_portid(ablk->ioc));
	bfa_ioc_mbox_queue(ablk->ioc, &ablk->mb, NULL, NULL);

	return BFA_STATUS_OK;
}

bfa_status_t bfa_ablk_nwpar_enable(struct bfa_ablk_s *ablk,
			bfa_ablk_cbfn_t cbfn, void *cbarg)
{
	bfi_ablk_h2i_nwpar_req_t *m;
	if (!bfa_ioc_is_operational(ablk->ioc)) {
		bfa_trc(ablk->ioc, BFA_STATUS_IOC_FAILURE);
		return BFA_STATUS_IOC_FAILURE;
	}

	if (ablk->busy) {
		bfa_trc(ablk->ioc, BFA_STATUS_DEVBUSY);
		return 	BFA_STATUS_DEVBUSY;
	}

	ablk->cbfn  = cbfn;
	ablk->cbarg = cbarg;
	ablk->busy  = BFA_TRUE;

	m = (bfi_ablk_h2i_nwpar_req_t *)ablk->mb.msg;
	bfi_h2i_set(m->mh, BFI_MC_ABLK, BFI_ABLK_H2I_NWPAR_ENABLE,
		    bfa_ioc_portid(ablk->ioc));
	bfa_ioc_mbox_queue(ablk->ioc, &ablk->mb, NULL, NULL);

	return BFA_STATUS_OK;
}

bfa_status_t bfa_ablk_nwpar_disable(struct bfa_ablk_s *ablk,
			bfa_ablk_cbfn_t cbfn, void *cbarg)
{
	bfi_ablk_h2i_nwpar_req_t *m;
	if (!bfa_ioc_is_operational(ablk->ioc)) {
		bfa_trc(ablk->ioc, BFA_STATUS_IOC_FAILURE);
		return BFA_STATUS_IOC_FAILURE;
	}

	if (ablk->busy) {
		bfa_trc(ablk->ioc, BFA_STATUS_DEVBUSY);
		return 	BFA_STATUS_DEVBUSY;
	}

	ablk->cbfn  = cbfn;
	ablk->cbarg = cbarg;
	ablk->busy  = BFA_TRUE;

	m = (bfi_ablk_h2i_nwpar_req_t *)ablk->mb.msg;
	bfi_h2i_set(m->mh, BFI_MC_ABLK, BFI_ABLK_H2I_NWPAR_DISABLE,
		    bfa_ioc_portid(ablk->ioc));
	bfa_ioc_mbox_queue(ablk->ioc, &ablk->mb, NULL, NULL);

	return BFA_STATUS_OK;
}

bfa_status_t
bfa_ablk_nwpar_pf_enable(struct bfa_ablk_s *ablk, int pfn,
			bfa_ablk_cbfn_t cbfn, void *cbarg)
{
	struct bfi_ablk_h2i_pf_req_s *m;

	if (!bfa_ioc_is_operational(ablk->ioc)) {
		bfa_trc(ablk->ioc, BFA_STATUS_IOC_FAILURE);
		return BFA_STATUS_IOC_FAILURE;
	}

	if (ablk->busy) {
		bfa_trc(ablk->ioc, BFA_STATUS_DEVBUSY);
		return 	BFA_STATUS_DEVBUSY;
	}

	ablk->cbfn = cbfn;
	ablk->cbarg = cbarg;
	ablk->busy  = BFA_TRUE;

	m = (struct bfi_ablk_h2i_pf_req_s *)ablk->mb.msg;
	bfi_h2i_set(m->mh, BFI_MC_ABLK, BFI_ABLK_H2I_NWPAR_PF_ENBL,
		    bfa_ioc_portid(ablk->ioc));
	m->pcifn = (uint8_t)pfn;
	bfa_ioc_mbox_queue(ablk->ioc, &ablk->mb, NULL, NULL);

	return BFA_STATUS_OK;
}

bfa_status_t
bfa_ablk_nwpar_pf_disable(struct bfa_ablk_s *ablk, int pfn,
			bfa_ablk_cbfn_t cbfn, void *cbarg)
{
	struct bfi_ablk_h2i_pf_req_s *m;

	if (!bfa_ioc_is_operational(ablk->ioc)) {
		bfa_trc(ablk->ioc, BFA_STATUS_IOC_FAILURE);
		return BFA_STATUS_IOC_FAILURE;
	}

	if (ablk->busy) {
		bfa_trc(ablk->ioc, BFA_STATUS_DEVBUSY);
		return 	BFA_STATUS_DEVBUSY;
	}

	ablk->cbfn = cbfn;
	ablk->cbarg = cbarg;
	ablk->busy  = BFA_TRUE;

	m = (struct bfi_ablk_h2i_pf_req_s *)ablk->mb.msg;
	bfi_h2i_set(m->mh, BFI_MC_ABLK, BFI_ABLK_H2I_NWPAR_PF_DSBL,
		    bfa_ioc_portid(ablk->ioc));
	m->pcifn = (uint8_t)pfn;
	bfa_ioc_mbox_queue(ablk->ioc, &ablk->mb, NULL, NULL);

	return BFA_STATUS_OK;
}

bfa_status_t
bfa_ablk_nwpar_pf_pers_change(struct bfa_ablk_s *ablk, int pfn,
	bfi_pcifn_class_t personality, bfa_ablk_cbfn_t cbfn, void *cbarg)
{
	struct bfi_ablk_h2i_pf_req_s *m;

	if (!bfa_ioc_is_operational(ablk->ioc)) {
		bfa_trc(ablk->ioc, BFA_STATUS_IOC_FAILURE);
		return BFA_STATUS_IOC_FAILURE;
	}

	if (ablk->busy) {
		bfa_trc(ablk->ioc, BFA_STATUS_DEVBUSY);
		return 	BFA_STATUS_DEVBUSY;
	}

	ablk->cbfn = cbfn;
	ablk->cbarg = cbarg;
	ablk->busy  = BFA_TRUE;

	m = (struct bfi_ablk_h2i_pf_req_s *)ablk->mb.msg;
	bfi_h2i_set(m->mh, BFI_MC_ABLK, BFI_ABLK_H2I_NWPAR_PF_PERS,
		    bfa_ioc_portid(ablk->ioc));
	m->pcifn = (uint8_t)pfn;
	m->pers = bfa_os_htons((uint16_t)personality);
	bfa_ioc_mbox_queue(ablk->ioc, &ablk->mb, NULL, NULL);

	return BFA_STATUS_OK;

}

bfa_status_t
bfa_ioc_reg_offset_check(struct bfa_ioc_s *ioc, uint32_t offset, uint32_t len)
{
	uint8_t area;

	len = len << 2;	/* len in dwords, converted to bytes */
	/*
	 * check [16:15]
	 */
	area = (offset >> 15) & 0x7;
	if (area == 0) {
		/*
		 * PCIe core register
		 */
		if ((offset + len) > 0x8000)    /* 8k dwords or 32KB */
			return BFA_STATUS_EINVAL;
	} else if (area == 0x1) {
		/*
		 * CB 32 KB memory page
		 */
		if ((offset + len) > 0x10000)   /* 8k dwords or 32KB */
			return BFA_STATUS_EINVAL;
	} else {
		/*
		 * CB register space 64KB
		 */
		if ((offset + len) > BFA_REG_ADDRSZ(ioc))
			return BFA_STATUS_EINVAL;
	}
	return BFA_STATUS_OK;
}

/**
 * @}
 */
