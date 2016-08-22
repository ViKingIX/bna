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
#include <log/bfa_log_hal.h>
#include <defs/bfa_defs.h>

BFA_TRC_FILE(CNA, IOC_CB);

#define bfa_ioc_cb_join_pos(__ioc) ((uint32_t) (1 << BFA_IOC_CB_JOIN_SH))

/*
 * forward declarations
 */
static bfa_boolean_t bfa_ioc_cb_firmware_lock(struct bfa_ioc_s *ioc);
static void bfa_ioc_cb_firmware_unlock(struct bfa_ioc_s *ioc);
static void bfa_ioc_cb_reg_init(struct bfa_ioc_s *ioc);
static void bfa_ioc_cb_map_port(struct bfa_ioc_s *ioc);
static void bfa_ioc_cb_isr_mode_set(struct bfa_ioc_s *ioc, bfa_boolean_t msix);
static void bfa_ioc_cb_notify_fail(struct bfa_ioc_s *ioc);
static void bfa_ioc_cb_ownership_reset(struct bfa_ioc_s *ioc);
static bfa_boolean_t bfa_ioc_cb_sync_start(struct bfa_ioc_s *ioc);
static void bfa_ioc_cb_sync_join(struct bfa_ioc_s *ioc);
static void bfa_ioc_cb_sync_leave(struct bfa_ioc_s *ioc);
static void bfa_ioc_cb_sync_ack(struct bfa_ioc_s *ioc);
static bfa_boolean_t bfa_ioc_cb_sync_complete(struct bfa_ioc_s *ioc);
static void bfa_ioc_cb_set_cur_ioc_fwstate(
			struct bfa_ioc_s *ioc, bfi_ioc_state_t fwstate);
static bfi_ioc_state_t bfa_ioc_cb_get_cur_ioc_fwstate(struct bfa_ioc_s *ioc);
static void bfa_ioc_cb_set_alt_ioc_fwstate(
			struct bfa_ioc_s *ioc, bfi_ioc_state_t fwstate);
static bfi_ioc_state_t bfa_ioc_cb_get_alt_ioc_fwstate(struct bfa_ioc_s *ioc);

static struct bfa_ioc_hwif_s hwif_cb;

/**
 * Called from bfa_ioc_attach() to map asic specific calls.
 */
void
bfa_ioc_set_cb_hwif(struct bfa_ioc_s *ioc)
{
	hwif_cb.ioc_pll_init = bfa_ioc_cb_pll_init;
	hwif_cb.ioc_firmware_lock = bfa_ioc_cb_firmware_lock;
	hwif_cb.ioc_firmware_unlock = bfa_ioc_cb_firmware_unlock;
	hwif_cb.ioc_reg_init = bfa_ioc_cb_reg_init;
	hwif_cb.ioc_map_port = bfa_ioc_cb_map_port;
	hwif_cb.ioc_isr_mode_set = bfa_ioc_cb_isr_mode_set;
	hwif_cb.ioc_notify_fail = bfa_ioc_cb_notify_fail;
	hwif_cb.ioc_ownership_reset = bfa_ioc_cb_ownership_reset;
	hwif_cb.ioc_sync_start = bfa_ioc_cb_sync_start;
	hwif_cb.ioc_sync_join = bfa_ioc_cb_sync_join;
	hwif_cb.ioc_sync_leave = bfa_ioc_cb_sync_leave;
	hwif_cb.ioc_sync_ack = bfa_ioc_cb_sync_ack;
	hwif_cb.ioc_sync_complete = bfa_ioc_cb_sync_complete;
	hwif_cb.ioc_set_fwstate = bfa_ioc_cb_set_cur_ioc_fwstate;
	hwif_cb.ioc_get_fwstate = bfa_ioc_cb_get_cur_ioc_fwstate;
	hwif_cb.ioc_set_alt_fwstate = bfa_ioc_cb_set_alt_ioc_fwstate;
	hwif_cb.ioc_get_alt_fwstate = bfa_ioc_cb_get_alt_ioc_fwstate;

	ioc->ioc_hwif = &hwif_cb;
}

static void
bfa_ioc_cb_firmware_unlock(struct bfa_ioc_s *ioc)
{
}

/**
 * Host to LPU mailbox message addresses
 */
static struct { uint32_t hfn_mbox, lpu_mbox, hfn_pgn; } iocreg_fnreg[] = {
	{ HOSTFN0_LPU_MBOX0_0, LPU_HOSTFN0_MBOX0_0, HOST_PAGE_NUM_FN0 },
	{ HOSTFN1_LPU_MBOX0_8, LPU_HOSTFN1_MBOX0_8, HOST_PAGE_NUM_FN1 }
};

/**
 * Host <-> LPU mailbox command/status registers
 */
static struct { uint32_t hfn, lpu; } iocreg_mbcmd[] = {

	{ HOSTFN0_LPU0_CMD_STAT, LPU0_HOSTFN0_CMD_STAT },
	{ HOSTFN1_LPU1_CMD_STAT, LPU1_HOSTFN1_CMD_STAT }
};

static void
bfa_ioc_cb_reg_init(struct bfa_ioc_s *ioc)
{
	bfa_os_addr_t	rb;
	int		pcifn = bfa_ioc_pcifn(ioc);

	rb = bfa_ioc_bar0(ioc);

	ioc->ioc_regs.hfn_mbox = rb + iocreg_fnreg[pcifn].hfn_mbox;
	ioc->ioc_regs.lpu_mbox = rb + iocreg_fnreg[pcifn].lpu_mbox;
	ioc->ioc_regs.host_page_num_fn = rb + iocreg_fnreg[pcifn].hfn_pgn;

	if (ioc->port_id == 0) {
		ioc->ioc_regs.heartbeat = rb + BFA_IOC0_HBEAT_REG;
		ioc->ioc_regs.ioc_fwstate = rb + BFA_IOC0_STATE_REG;
		ioc->ioc_regs.alt_ioc_fwstate = rb + BFA_IOC1_STATE_REG;
	} else {
		ioc->ioc_regs.heartbeat = (rb + BFA_IOC1_HBEAT_REG);
		ioc->ioc_regs.ioc_fwstate = (rb + BFA_IOC1_STATE_REG);
		ioc->ioc_regs.alt_ioc_fwstate = (rb + BFA_IOC0_STATE_REG);
	}

	/**
	 * Host <-> LPU mailbox command/status registers
	 */
	ioc->ioc_regs.hfn_mbox_cmd = rb + iocreg_mbcmd[pcifn].hfn;
	ioc->ioc_regs.lpu_mbox_cmd = rb + iocreg_mbcmd[pcifn].lpu;

	/*
	 * PSS control registers
	 */
	ioc->ioc_regs.pss_ctl_reg = (rb + PSS_CTL_REG);
	ioc->ioc_regs.pss_err_status_reg = (rb + PSS_ERR_STATUS_REG);
	ioc->ioc_regs.app_pll_fast_ctl_reg = (rb + APP_PLL_LCLK_CTL_REG);
	ioc->ioc_regs.app_pll_slow_ctl_reg = (rb + APP_PLL_SCLK_CTL_REG);

	/*
	 * IOC semaphore registers and serialization
	 */
	ioc->ioc_regs.ioc_sem_reg = (rb + HOST_SEM0_REG);
	ioc->ioc_regs.ioc_init_sem_reg = (rb + HOST_SEM2_REG);

	/**
	 * sram memory access
	 */
	ioc->ioc_regs.smem_page_start = (rb + PSS_SMEM_PAGE_START);
	ioc->ioc_regs.smem_pg0 = BFI_IOC_SMEM_PG0_CB;

	/*
	 * err set reg : for notification of hb failure
	 */
	ioc->ioc_regs.err_set = (rb + ERR_SET_REG);
}

/**
 * Initialize IOC to port mapping.
 */

static void
bfa_ioc_cb_map_port(struct bfa_ioc_s *ioc)
{
	/**
	 * For crossbow, port id is same as pci function.
	 */
	ioc->port_id = bfa_ioc_pcifn(ioc);

	bfa_trc(ioc, ioc->port_id);
}

/**
 * Set interrupt mode for a function: INTX or MSIX
 */
static void
bfa_ioc_cb_isr_mode_set(struct bfa_ioc_s *ioc, bfa_boolean_t msix)
{
}

/**
 * Synchronized IOC failure processing routines
 */
static bfa_boolean_t
bfa_ioc_cb_sync_start(struct bfa_ioc_s *ioc)
{
	uint32_t ioc_fwstate = bfa_reg_read(ioc->ioc_regs.ioc_fwstate);

	if (bfa_ioc_is_uefi(ioc))
		return BFA_TRUE;

	/**
	 * Driver load time.  If the join bit is set,
	 * it is due to an unclean exit by the driver for this
	 * PCI fn in the previous incarnation. Whoever comes here first
	 * should clean it up, no matter which PCI fn.
	 */
	if (ioc_fwstate & BFA_IOC_CB_JOIN_MASK) {
		bfa_reg_write(ioc->ioc_regs.ioc_fwstate, BFI_IOC_UNINIT);
		bfa_reg_write(ioc->ioc_regs.alt_ioc_fwstate, BFI_IOC_UNINIT);
		return BFA_TRUE;
	}

	return bfa_ioc_cb_sync_complete(ioc);
}

static void
bfa_ioc_cb_sync_join(struct bfa_ioc_s *ioc)
{
	uint32_t r32 = bfa_reg_read(ioc->ioc_regs.ioc_fwstate);
	uint32_t join_pos = bfa_ioc_cb_join_pos(ioc);

	if (bfa_ioc_is_uefi(ioc))
		return;

	bfa_reg_write(ioc->ioc_regs.ioc_fwstate, (r32 | join_pos));
}

static void
bfa_ioc_cb_sync_leave(struct bfa_ioc_s *ioc)
{
	uint32_t r32 = bfa_reg_read(ioc->ioc_regs.ioc_fwstate);
	uint32_t join_pos = bfa_ioc_cb_join_pos(ioc);

	if (bfa_ioc_is_uefi(ioc))
		return;

	bfa_reg_write(ioc->ioc_regs.ioc_fwstate, (r32 & ~join_pos));
}

static void
bfa_ioc_cb_set_cur_ioc_fwstate(struct bfa_ioc_s *ioc, bfi_ioc_state_t fwstate)
{
	uint32_t r32 = bfa_reg_read(ioc->ioc_regs.ioc_fwstate);

	bfa_reg_write(ioc->ioc_regs.ioc_fwstate,
			(fwstate | (r32 & BFA_IOC_CB_JOIN_MASK)));
}

static bfi_ioc_state_t
bfa_ioc_cb_get_cur_ioc_fwstate(struct bfa_ioc_s *ioc)
{
	return ((bfi_ioc_state_t)(bfa_reg_read(ioc->ioc_regs.ioc_fwstate) &
			BFA_IOC_CB_FWSTATE_MASK));
}

static void
bfa_ioc_cb_set_alt_ioc_fwstate(struct bfa_ioc_s *ioc, bfi_ioc_state_t fwstate)
{
	uint32_t r32 = bfa_reg_read(ioc->ioc_regs.alt_ioc_fwstate);

	bfa_reg_write(ioc->ioc_regs.alt_ioc_fwstate,
			(fwstate | (r32 & BFA_IOC_CB_JOIN_MASK)));
}

static bfi_ioc_state_t
bfa_ioc_cb_get_alt_ioc_fwstate(struct bfa_ioc_s *ioc)
{
	return ((bfi_ioc_state_t)(bfa_reg_read(ioc->ioc_regs.alt_ioc_fwstate) &
			BFA_IOC_CB_FWSTATE_MASK));
}

/**
 * Return true if firmware of current driver matches the running firmware.
 */
static bfa_boolean_t
bfa_ioc_cb_firmware_lock(struct bfa_ioc_s *ioc)
{
	enum bfi_ioc_state alt_fwstate, cur_fwstate;
	struct bfi_ioc_image_hdr_s fwhdr;
	/**
	 * If UEFI boot -- no possibility of f/w mismatch on cb.
	 */
	if (bfa_ioc_is_uefi(ioc))
		return BFA_TRUE;

	cur_fwstate = bfa_ioc_cb_get_cur_ioc_fwstate(ioc);
	bfa_trc(ioc, cur_fwstate);
	alt_fwstate = bfa_ioc_cb_get_alt_ioc_fwstate(ioc);
	bfa_trc(ioc, alt_fwstate);

	/**
	 * Uninit implies this is the only driver as of now.
	 */
	if (cur_fwstate == BFI_IOC_UNINIT)
		return BFA_TRUE;
	/**
	 * Check if another driver with a different firmware is active
	 */
	bfa_ioc_fwver_get(ioc, &fwhdr);
	if (!bfa_ioc_fwver_cmp(ioc, &fwhdr) &&
		alt_fwstate != BFI_IOC_DISABLED) {
		bfa_trc(ioc, alt_fwstate);
		return BFA_FALSE;
	}

	return BFA_TRUE;
}

/**
 * Notify other functions on HB failure.
 */
static void
bfa_ioc_cb_notify_fail(struct bfa_ioc_s *ioc)
{
	bfa_reg_write(ioc->ioc_regs.err_set, ~0U);
	bfa_reg_read(ioc->ioc_regs.err_set);
}

/**
 * Cleanup hw semaphore and usecnt registers
 */
static void
bfa_ioc_cb_ownership_reset(struct bfa_ioc_s *ioc)
{

	/*
	 * Read the hw sem reg to make sure that it is locked
	 * before we clear it. If it is not locked, writing 1
	 * will lock it instead of clearing it.
	 */
	bfa_reg_read(ioc->ioc_regs.ioc_sem_reg);
	bfa_ioc_hw_sem_release(ioc);
}

static void
bfa_ioc_cb_sync_ack(struct bfa_ioc_s *ioc)
{
	bfa_ioc_cb_set_cur_ioc_fwstate(ioc, BFI_IOC_FAIL);
}

static bfa_boolean_t
bfa_ioc_cb_sync_complete(struct bfa_ioc_s *ioc)
{
	uint32_t fwstate, alt_fwstate;
	fwstate = bfa_ioc_cb_get_cur_ioc_fwstate(ioc);

	if (bfa_ioc_is_uefi(ioc))
		return BFA_TRUE;

	/**
	 * At this point, this IOC is hoding the hw sem in the
	 * start path (fwcheck) OR in the disable/enable path
	 * OR to check if the other IOC has acknowledged failure.
	 *
	 * So, this IOC can be in UNINIT, INITING, DISABLED, FAIL
	 * or in MEMTEST states. In a normal scenario, this IOC
	 * can not be in OP state when this function is called.
	 *
	 * However, this IOC could still be in OP state when
	 * the OS driver is starting up, if the OptROM code has
	 * left it in that state.
	 *
	 * If we had marked this IOC's fwstate as BFI_IOC_FAIL
	 * in the failure case and now, if the fwstate is not
	 * BFI_IOC_FAIL it implies that the other PCI fn have
	 * reinitialized the ASIC or this IOC got disabled, so
	 * return TRUE.
	 */
	if (fwstate == BFI_IOC_UNINIT ||
		fwstate == BFI_IOC_INITING ||
		fwstate == BFI_IOC_DISABLED ||
		fwstate == BFI_IOC_MEMTEST ||
		fwstate == BFI_IOC_OP)
		return BFA_TRUE;
	else {
		alt_fwstate = bfa_ioc_cb_get_alt_ioc_fwstate(ioc);
		if (alt_fwstate == BFI_IOC_FAIL ||
			alt_fwstate == BFI_IOC_DISABLED ||
			alt_fwstate == BFI_IOC_UNINIT ||
			alt_fwstate == BFI_IOC_INITING ||
			alt_fwstate == BFI_IOC_MEMTEST)
			return BFA_TRUE;

		else
			return BFA_FALSE;
	}

}

/**
 * @}
 */
