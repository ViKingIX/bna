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

BFA_TRC_FILE(CNA, IOC_CT);

#define bfa_ioc_ct_sync_pos(__ioc)	\
		((uint32_t) (1 << bfa_ioc_pcifn(__ioc)))
#define BFA_IOC_SYNC_REQD_SH	16
#define bfa_ioc_ct_get_sync_ackd(__val)	(__val & 0x0000ffff)
#define bfa_ioc_ct_clear_sync_ackd(__val)	(__val & 0xffff0000)
#define bfa_ioc_ct_get_sync_reqd(__val)	(__val >> BFA_IOC_SYNC_REQD_SH)
#define bfa_ioc_ct_sync_reqd_pos(__ioc)	\
			(bfa_ioc_ct_sync_pos(__ioc) << BFA_IOC_SYNC_REQD_SH)

static bfa_boolean_t bfa_ioc_ct_firmware_lock(struct bfa_ioc_s *ioc);
static void bfa_ioc_ct_firmware_unlock(struct bfa_ioc_s *ioc);
static void bfa_ioc_ct_notify_fail(struct bfa_ioc_s *ioc);
static void bfa_ioc_ct_ownership_reset(struct bfa_ioc_s *ioc);
static bfa_boolean_t bfa_ioc_ct_sync_start(struct bfa_ioc_s *ioc);
static void bfa_ioc_ct_sync_join(struct bfa_ioc_s *ioc);
static void bfa_ioc_ct_sync_leave(struct bfa_ioc_s *ioc);
static void bfa_ioc_ct_sync_ack(struct bfa_ioc_s *ioc);
static bfa_boolean_t bfa_ioc_ct_sync_complete(struct bfa_ioc_s *ioc);
static void bfa_ioc_ct_set_cur_ioc_fwstate(
			struct bfa_ioc_s *ioc, bfi_ioc_state_t fwstate);
static bfi_ioc_state_t bfa_ioc_ct_get_cur_ioc_fwstate(struct bfa_ioc_s *ioc);
static void bfa_ioc_ct_set_alt_ioc_fwstate(
			struct bfa_ioc_s *ioc, bfi_ioc_state_t fwstate);
static bfi_ioc_state_t bfa_ioc_ct_get_alt_ioc_fwstate(struct bfa_ioc_s *ioc);

static struct bfa_ioc_hwif_s hwif_ct;
static struct bfa_ioc_hwif_s hwif_ct2;

/**
 * Host to LPU mailbox message addresses
 */
static struct { uint32_t hfn_mbox, lpu_mbox, hfn_pgn; } ct_fnreg[] = {
	{ HOSTFN0_LPU_MBOX0_0, LPU_HOSTFN0_MBOX0_0, HOST_PAGE_NUM_FN0 },
	{ HOSTFN1_LPU_MBOX0_8, LPU_HOSTFN1_MBOX0_8, HOST_PAGE_NUM_FN1 },
	{ HOSTFN2_LPU_MBOX0_0, LPU_HOSTFN2_MBOX0_0, HOST_PAGE_NUM_FN2 },
	{ HOSTFN3_LPU_MBOX0_8, LPU_HOSTFN3_MBOX0_8, HOST_PAGE_NUM_FN3 }
};

/**
 * Host <-> LPU mailbox command/status registers - port 0
 */
static struct { uint32_t hfn, lpu; } ct_p0reg[] = {
	{ HOSTFN0_LPU0_CMD_STAT, LPU0_HOSTFN0_CMD_STAT },
	{ HOSTFN1_LPU0_CMD_STAT, LPU0_HOSTFN1_CMD_STAT },
	{ HOSTFN2_LPU0_CMD_STAT, LPU0_HOSTFN2_CMD_STAT },
	{ HOSTFN3_LPU0_CMD_STAT, LPU0_HOSTFN3_CMD_STAT }
};

/**
 * Host <-> LPU mailbox command/status registers - port 1
 */
static struct { uint32_t hfn, lpu; } ct_p1reg[] = {
	{ HOSTFN0_LPU1_CMD_STAT, LPU1_HOSTFN0_CMD_STAT },
	{ HOSTFN1_LPU1_CMD_STAT, LPU1_HOSTFN1_CMD_STAT },
	{ HOSTFN2_LPU1_CMD_STAT, LPU1_HOSTFN2_CMD_STAT },
	{ HOSTFN3_LPU1_CMD_STAT, LPU1_HOSTFN3_CMD_STAT }
};

static struct { uint32_t hfn_mbox, lpu_mbox, hfn_pgn, hfn, lpu, lpu_read; }
	ct2_reg[] = {
	{ CT2_HOSTFN_LPU0_MBOX0, CT2_LPU0_HOSTFN_MBOX0, CT2_HOSTFN_PAGE_NUM,
	  CT2_HOSTFN_LPU0_CMD_STAT, CT2_LPU0_HOSTFN_CMD_STAT,
	  CT2_HOSTFN_LPU0_READ_STAT},
	{ CT2_HOSTFN_LPU1_MBOX0, CT2_LPU1_HOSTFN_MBOX0, CT2_HOSTFN_PAGE_NUM,
	  CT2_HOSTFN_LPU1_CMD_STAT, CT2_LPU1_HOSTFN_CMD_STAT,
	  CT2_HOSTFN_LPU1_READ_STAT},
};

static void
bfa_ioc_ct_reg_init(struct bfa_ioc_s *ioc)
{
	bfa_os_addr_t	rb;
	int		pcifn = bfa_ioc_pcifn(ioc);

	rb = bfa_ioc_bar0(ioc);

	ioc->ioc_regs.hfn_mbox = rb + ct_fnreg[pcifn].hfn_mbox;
	ioc->ioc_regs.lpu_mbox = rb + ct_fnreg[pcifn].lpu_mbox;
	ioc->ioc_regs.host_page_num_fn = rb + ct_fnreg[pcifn].hfn_pgn;

	if (ioc->port_id == 0) {
		ioc->ioc_regs.heartbeat = rb + BFA_IOC0_HBEAT_REG;
		ioc->ioc_regs.ioc_fwstate = rb + BFA_IOC0_STATE_REG;
		ioc->ioc_regs.alt_ioc_fwstate = rb + BFA_IOC1_STATE_REG;
		ioc->ioc_regs.hfn_mbox_cmd = rb + ct_p0reg[pcifn].hfn;
		ioc->ioc_regs.lpu_mbox_cmd = rb + ct_p0reg[pcifn].lpu;
		ioc->ioc_regs.ll_halt = rb + FW_INIT_HALT_P0;
		ioc->ioc_regs.alt_ll_halt = rb + FW_INIT_HALT_P1;
	} else {
		ioc->ioc_regs.heartbeat = (rb + BFA_IOC1_HBEAT_REG);
		ioc->ioc_regs.ioc_fwstate = (rb + BFA_IOC1_STATE_REG);
		ioc->ioc_regs.alt_ioc_fwstate = rb + BFA_IOC0_STATE_REG;
		ioc->ioc_regs.hfn_mbox_cmd = rb + ct_p1reg[pcifn].hfn;
		ioc->ioc_regs.lpu_mbox_cmd = rb + ct_p1reg[pcifn].lpu;
		ioc->ioc_regs.ll_halt = rb + FW_INIT_HALT_P1;
		ioc->ioc_regs.alt_ll_halt = rb + FW_INIT_HALT_P0;
	}

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
	ioc->ioc_regs.ioc_usage_sem_reg = (rb + HOST_SEM1_REG);
	ioc->ioc_regs.ioc_init_sem_reg = (rb + HOST_SEM2_REG);
	ioc->ioc_regs.ioc_usage_reg = (rb + BFA_FW_USE_COUNT);
	ioc->ioc_regs.ioc_fail_sync = (rb + BFA_IOC_FAIL_SYNC);

	/**
	 * sram memory access
	 */
	ioc->ioc_regs.smem_page_start = (rb + PSS_SMEM_PAGE_START);
	ioc->ioc_regs.smem_pg0 = BFI_IOC_SMEM_PG0_CT;

	/*
	 * err set reg : for notification of hb failure in fcmode
	 */
	ioc->ioc_regs.err_set = (rb + ERR_SET_REG);
}

static void
bfa_ioc_ct2_reg_init(struct bfa_ioc_s *ioc)
{
	bfa_os_addr_t	rb;
	int		port = bfa_ioc_portid(ioc);

	rb = bfa_ioc_bar0(ioc);

	ioc->ioc_regs.hfn_mbox = rb + ct2_reg[port].hfn_mbox;
	ioc->ioc_regs.lpu_mbox = rb + ct2_reg[port].lpu_mbox;
	ioc->ioc_regs.host_page_num_fn = rb + ct2_reg[port].hfn_pgn;
	ioc->ioc_regs.hfn_mbox_cmd = rb + ct2_reg[port].hfn;
	ioc->ioc_regs.lpu_mbox_cmd = rb + ct2_reg[port].lpu;
	ioc->ioc_regs.lpu_read_stat = rb + ct2_reg[port].lpu_read;

	if (port == 0) {
		ioc->ioc_regs.heartbeat = rb + CT2_BFA_IOC0_HBEAT_REG;
		ioc->ioc_regs.ioc_fwstate = rb + CT2_BFA_IOC0_STATE_REG;
		ioc->ioc_regs.alt_ioc_fwstate = rb + CT2_BFA_IOC1_STATE_REG;
		ioc->ioc_regs.ll_halt = rb + FW_INIT_HALT_P0;
		ioc->ioc_regs.alt_ll_halt = rb + FW_INIT_HALT_P1;
	} else {
		ioc->ioc_regs.heartbeat = (rb + CT2_BFA_IOC1_HBEAT_REG);
		ioc->ioc_regs.ioc_fwstate = (rb + CT2_BFA_IOC1_STATE_REG);
		ioc->ioc_regs.alt_ioc_fwstate = rb + CT2_BFA_IOC0_STATE_REG;
		ioc->ioc_regs.ll_halt = rb + FW_INIT_HALT_P1;
		ioc->ioc_regs.alt_ll_halt = rb + FW_INIT_HALT_P0;
	}

	/*
	 * PSS control registers
	 */
	ioc->ioc_regs.pss_ctl_reg = (rb + PSS_CTL_REG);
	ioc->ioc_regs.pss_err_status_reg = (rb + PSS_ERR_STATUS_REG);
	ioc->ioc_regs.app_pll_fast_ctl_reg = (rb + CT2_APP_PLL_LCLK_CTL_REG);
	ioc->ioc_regs.app_pll_slow_ctl_reg = (rb + CT2_APP_PLL_SCLK_CTL_REG);

	/*
	 * IOC semaphore registers and serialization
	 */
	ioc->ioc_regs.ioc_sem_reg = (rb + CT2_HOST_SEM0_REG);
	ioc->ioc_regs.ioc_usage_sem_reg = (rb + CT2_HOST_SEM1_REG);
	ioc->ioc_regs.ioc_init_sem_reg = (rb + CT2_HOST_SEM2_REG);
	ioc->ioc_regs.ioc_usage_reg = (rb + CT2_BFA_FW_USE_COUNT);
	ioc->ioc_regs.ioc_fail_sync = (rb + CT2_BFA_IOC_FAIL_SYNC);

	/**
	 * sram memory access
	 */
	ioc->ioc_regs.smem_page_start = (rb + PSS_SMEM_PAGE_START);
	ioc->ioc_regs.smem_pg0 = BFI_IOC_SMEM_PG0_CT;

	/*
	 * err set reg : for notification of hb failure in fcmode
	 */
	ioc->ioc_regs.err_set = (rb + ERR_SET_REG);
}

/**
 * Initialize IOC to port mapping.
 */

#define FNC_PERS_FN_SHIFT(__fn)	((__fn) * 8)
static void
bfa_ioc_ct_map_port(struct bfa_ioc_s *ioc)
{
	bfa_os_addr_t	rb = ioc->pcidev.pci_bar_kva;
	uint32_t	r32;

	/**
	 * For catapult, base port id on personality register and IOC type
	 */
	r32 = bfa_reg_read(rb + FNC_PERS_REG);
	r32 >>= FNC_PERS_FN_SHIFT(bfa_ioc_pcifn(ioc));
	ioc->port_id = (r32 & __F0_PORT_MAP_MK) >> __F0_PORT_MAP_SH;

	bfa_trc(ioc, bfa_ioc_pcifn(ioc));
	bfa_trc(ioc, ioc->port_id);
}

static void
bfa_ioc_ct2_map_port(struct bfa_ioc_s *ioc)
{
	bfa_os_addr_t	rb = ioc->pcidev.pci_bar_kva;
	uint32_t	r32;

	r32 = bfa_reg_read(rb + CT2_HOSTFN_PERSONALITY0);
	ioc->port_id = ((r32 & __FC_LL_PORT_MAP__MK) >> __FC_LL_PORT_MAP__SH);

	bfa_trc(ioc, bfa_ioc_pcifn(ioc));
	bfa_trc(ioc, ioc->port_id);
}

/**
 * Set interrupt mode for a function: INTX or MSIX
 */
static void
bfa_ioc_ct_isr_mode_set(struct bfa_ioc_s *ioc, bfa_boolean_t msix)
{
	bfa_os_addr_t	rb = ioc->pcidev.pci_bar_kva;
	uint32_t	r32, mode;

	r32 = bfa_reg_read(rb + FNC_PERS_REG);
	bfa_trc(ioc, r32);

	mode = (r32 >> FNC_PERS_FN_SHIFT(bfa_ioc_pcifn(ioc))) &
		__F0_INTX_STATUS;

	/**
	 * If already in desired mode, do not change anything
	 */
	if ((!msix && mode) || (msix && !mode))
		return;

	if (msix)
		mode = __F0_INTX_STATUS_MSIX;
	else
		mode = __F0_INTX_STATUS_INTA;

	r32 &= ~(__F0_INTX_STATUS << FNC_PERS_FN_SHIFT(bfa_ioc_pcifn(ioc)));
	r32 |= (mode << FNC_PERS_FN_SHIFT(bfa_ioc_pcifn(ioc)));
	bfa_trc(ioc, r32);

	bfa_reg_write(rb + FNC_PERS_REG, r32);
}

bfa_boolean_t
bfa_ioc_ct2_lpu_read_stat(struct bfa_ioc_s *ioc)
{
	uint32_t r32;

	r32 = bfa_reg_read(ioc->ioc_regs.lpu_read_stat);
	if (r32) {
		bfa_reg_write(ioc->ioc_regs.lpu_read_stat, 1);
		return BFA_TRUE;
	}

	return BFA_FALSE;
}

/**
 * Called from bfa_ioc_attach() to map asic specific calls.
 */
static void
bfa_ioc_set_ctx_hwif(struct bfa_ioc_s *ioc, struct bfa_ioc_hwif_s *hwif)
{
	hwif->ioc_firmware_lock = bfa_ioc_ct_firmware_lock;
	hwif->ioc_firmware_unlock = bfa_ioc_ct_firmware_unlock;
	hwif->ioc_notify_fail = bfa_ioc_ct_notify_fail;
	hwif->ioc_ownership_reset = bfa_ioc_ct_ownership_reset;
	hwif->ioc_sync_start = bfa_ioc_ct_sync_start;
	hwif->ioc_sync_join = bfa_ioc_ct_sync_join;
	hwif->ioc_sync_leave = bfa_ioc_ct_sync_leave;
	hwif->ioc_sync_ack = bfa_ioc_ct_sync_ack;
	hwif->ioc_sync_complete = bfa_ioc_ct_sync_complete;
	hwif->ioc_set_fwstate = bfa_ioc_ct_set_cur_ioc_fwstate;
	hwif->ioc_get_fwstate = bfa_ioc_ct_get_cur_ioc_fwstate;
	hwif->ioc_set_alt_fwstate = bfa_ioc_ct_set_alt_ioc_fwstate;
	hwif->ioc_get_alt_fwstate = bfa_ioc_ct_get_alt_ioc_fwstate;
}

/**
 * Called from bfa_ioc_attach() to map asic specific calls.
 */
void
bfa_ioc_set_ct_hwif(struct bfa_ioc_s *ioc)
{
	bfa_ioc_set_ctx_hwif(ioc, &hwif_ct);

	hwif_ct.ioc_pll_init = bfa_ioc_ct_pll_init;
	hwif_ct.ioc_reg_init = bfa_ioc_ct_reg_init;
	hwif_ct.ioc_map_port = bfa_ioc_ct_map_port;
	hwif_ct.ioc_isr_mode_set = bfa_ioc_ct_isr_mode_set;
	ioc->ioc_hwif = &hwif_ct;
}

/**
 * Called from bfa_ioc_attach() to map asic specific calls.
 */
void
bfa_ioc_set_ct2_hwif(struct bfa_ioc_s *ioc)
{
	bfa_ioc_set_ctx_hwif(ioc, &hwif_ct2);

	hwif_ct2.ioc_pll_init = bfa_ioc_ct2_pll_init;
	hwif_ct2.ioc_reg_init = bfa_ioc_ct2_reg_init;
	hwif_ct2.ioc_map_port = bfa_ioc_ct2_map_port;
	hwif_ct2.ioc_lpu_read_stat = bfa_ioc_ct2_lpu_read_stat;
	hwif_ct2.ioc_isr_mode_set = NULL;
	ioc->ioc_hwif = &hwif_ct2;
}

/**
 * Workaround for MSI-X resource allocation for catapult-2 with no asic block
 */
#define HOSTFN_MSIX_DEFAULT		64
#define HOSTFN_MSIX_VT_INDEX_MBOX_ERR	0x30138
#define HOSTFN_MSIX_VT_OFST_NUMVT	0x3013c
#define __MSIX_VT_NUMVT__MK		0x003ff800
#define __MSIX_VT_NUMVT__SH		11
#define __MSIX_VT_NUMVT_(_v)		((_v) << __MSIX_VT_NUMVT__SH)
#define __MSIX_VT_OFST_			0x000007ff
void
bfa_ioc_ct2_poweron(struct bfa_ioc_s *ioc)
{
	bfa_os_addr_t rb = ioc->pcidev.pci_bar_kva;
	uint32_t r32;

	r32 = bfa_reg_read(rb + HOSTFN_MSIX_VT_OFST_NUMVT);
	if (r32 & __MSIX_VT_NUMVT__MK) {
		bfa_reg_write(rb + HOSTFN_MSIX_VT_INDEX_MBOX_ERR,
			r32 & __MSIX_VT_OFST_);
		return;
	}

	bfa_reg_write(rb + HOSTFN_MSIX_VT_OFST_NUMVT,
			__MSIX_VT_NUMVT_(HOSTFN_MSIX_DEFAULT - 1) |
			HOSTFN_MSIX_DEFAULT * bfa_ioc_pcifn(ioc));
	bfa_reg_write(rb + HOSTFN_MSIX_VT_INDEX_MBOX_ERR,
			HOSTFN_MSIX_DEFAULT * bfa_ioc_pcifn(ioc));
}

/**
 * Return true if firmware of current driver matches the running firmware.
 */
static bfa_boolean_t
bfa_ioc_ct_firmware_lock(struct bfa_ioc_s *ioc)
{
	enum bfi_ioc_state ioc_fwstate;
	uint32_t usecnt;
	struct bfi_ioc_image_hdr_s fwhdr;

	/**
	 * If UEFI boot -- do not increment usage count
	 */
	if (bfa_ioc_is_uefi(ioc))
		return BFA_TRUE;

	bfa_ioc_sem_get(ioc->ioc_regs.ioc_usage_sem_reg);
	usecnt = bfa_reg_read(ioc->ioc_regs.ioc_usage_reg);

	/**
	 * If usage count is 0, always return TRUE.
	 */
	if (usecnt == 0) {
		bfa_reg_write(ioc->ioc_regs.ioc_usage_reg, 1);
		bfa_ioc_sem_release(ioc->ioc_regs.ioc_usage_sem_reg);
		bfa_reg_write(ioc->ioc_regs.ioc_fail_sync, 0);
		bfa_trc(ioc, usecnt);
		return BFA_TRUE;
	}

	ioc_fwstate = bfa_reg_read(ioc->ioc_regs.ioc_fwstate);
	bfa_trc(ioc, ioc_fwstate);

	/**
	 * Use count cannot be non-zero and chip in uninitialized state.
	 */
	bfa_assert(ioc_fwstate != BFI_IOC_UNINIT);

	/**
	 * Check if another driver with a different firmware is active
	 */
	bfa_ioc_fwver_get(ioc, &fwhdr);
	if (!bfa_ioc_fwver_cmp(ioc, &fwhdr)) {
		bfa_ioc_sem_release(ioc->ioc_regs.ioc_usage_sem_reg);
		bfa_trc(ioc, usecnt);
		return BFA_FALSE;
	}

	/**
	 * Same firmware version. Increment the reference count.
	 */
	usecnt++;
	bfa_reg_write(ioc->ioc_regs.ioc_usage_reg, usecnt);
	bfa_ioc_sem_release(ioc->ioc_regs.ioc_usage_sem_reg);
	bfa_trc(ioc, usecnt);
	return BFA_TRUE;
}

static void
bfa_ioc_ct_firmware_unlock(struct bfa_ioc_s *ioc)
{
	uint32_t usecnt;

	/**
	 * If UEFI boot -- do not decrement usage count
	 */
	if (bfa_ioc_is_uefi(ioc))
		return;

	/**
	 * decrement usage count
	 */
	bfa_ioc_sem_get(ioc->ioc_regs.ioc_usage_sem_reg);
	usecnt = bfa_reg_read(ioc->ioc_regs.ioc_usage_reg);
	bfa_assert(usecnt > 0);

	usecnt--;
	bfa_reg_write(ioc->ioc_regs.ioc_usage_reg, usecnt);
	bfa_trc(ioc, usecnt);

	bfa_ioc_sem_release(ioc->ioc_regs.ioc_usage_sem_reg);
}

/**
 * Notify other functions on HB failure.
 */
static void
bfa_ioc_ct_notify_fail(struct bfa_ioc_s *ioc)
{
	if (bfa_ioc_is_cna(ioc)) {
		bfa_reg_write(ioc->ioc_regs.ll_halt, __FW_INIT_HALT_P);
		bfa_reg_write(ioc->ioc_regs.alt_ll_halt, __FW_INIT_HALT_P);
		/* Wait for halt to take effect */
		bfa_reg_read(ioc->ioc_regs.ll_halt);
		bfa_reg_read(ioc->ioc_regs.alt_ll_halt);
	} else {
		bfa_reg_write(ioc->ioc_regs.err_set, ~0U);
		bfa_reg_read(ioc->ioc_regs.err_set);
	}
}

/**
 * Cleanup hw semaphore and usecnt registers
 */
static void
bfa_ioc_ct_ownership_reset(struct bfa_ioc_s *ioc)
{

	bfa_ioc_sem_get(ioc->ioc_regs.ioc_usage_sem_reg);
	bfa_reg_write(ioc->ioc_regs.ioc_usage_reg, 0);
	bfa_ioc_sem_release(ioc->ioc_regs.ioc_usage_sem_reg);

	bfa_reg_write(ioc->ioc_regs.ioc_fail_sync, 0);
	/*
	 * Read the hw sem reg to make sure that it is locked
	 * before we clear it. If it is not locked, writing 1
	 * will lock it instead of clearing it.
	 */
	bfa_reg_read(ioc->ioc_regs.ioc_sem_reg);
	bfa_ioc_hw_sem_release(ioc);
}

/**
 * Synchronized IOC failure processing routines
 */
static bfa_boolean_t
bfa_ioc_ct_sync_start(struct bfa_ioc_s *ioc)
{
	uint32_t r32 = bfa_reg_read(ioc->ioc_regs.ioc_fail_sync);
	uint32_t sync_reqd = bfa_ioc_ct_get_sync_reqd(r32);

	if (bfa_ioc_is_uefi(ioc))
		return BFA_TRUE;
	/**
	 * Driver load time.  If the sync required bit for this PCI fn
	 * is set, it is due to an unclean exit by the driver for this
	 * PCI fn in the previous incarnation. Whoever comes here first
	 * should clean it up, no matter which PCI fn.
	 */

	if (sync_reqd & bfa_ioc_ct_sync_pos(ioc)) {
		bfa_reg_write(ioc->ioc_regs.ioc_fail_sync, 0);
		bfa_reg_write(ioc->ioc_regs.ioc_usage_reg, 1);
		bfa_reg_write(ioc->ioc_regs.ioc_fwstate, BFI_IOC_UNINIT);
		bfa_reg_write(ioc->ioc_regs.alt_ioc_fwstate, BFI_IOC_UNINIT);
		return BFA_TRUE;
	}

	return bfa_ioc_ct_sync_complete(ioc);
}

static void
bfa_ioc_ct_sync_join(struct bfa_ioc_s *ioc)
{
	uint32_t r32 = bfa_reg_read(ioc->ioc_regs.ioc_fail_sync);
	uint32_t sync_pos = bfa_ioc_ct_sync_reqd_pos(ioc);

	if (bfa_ioc_is_uefi(ioc))
		return;

	bfa_reg_write(ioc->ioc_regs.ioc_fail_sync, (r32 | sync_pos));
}

static void
bfa_ioc_ct_sync_leave(struct bfa_ioc_s *ioc)
{
	uint32_t r32 = bfa_reg_read(ioc->ioc_regs.ioc_fail_sync);
	uint32_t sync_msk = bfa_ioc_ct_sync_reqd_pos(ioc) |
					bfa_ioc_ct_sync_pos(ioc);

	if (bfa_ioc_is_uefi(ioc))
		return;

	bfa_reg_write(ioc->ioc_regs.ioc_fail_sync, (r32 & ~sync_msk));
}

static void
bfa_ioc_ct_sync_ack(struct bfa_ioc_s *ioc)
{
	uint32_t r32 = bfa_reg_read(ioc->ioc_regs.ioc_fail_sync);

	if (bfa_ioc_is_uefi(ioc))
		return;

	bfa_reg_write(ioc->ioc_regs.ioc_fail_sync,
				(r32 | bfa_ioc_ct_sync_pos(ioc)));
}

static bfa_boolean_t
bfa_ioc_ct_sync_complete(struct bfa_ioc_s *ioc)
{
	uint32_t r32 = bfa_reg_read(ioc->ioc_regs.ioc_fail_sync);
	uint32_t sync_reqd = bfa_ioc_ct_get_sync_reqd(r32);
	uint32_t sync_ackd = bfa_ioc_ct_get_sync_ackd(r32);
	uint32_t tmp_ackd;

	if (bfa_ioc_is_uefi(ioc))
		return BFA_TRUE;

	if (sync_ackd == 0)
		return BFA_TRUE;

	/**
	 * The check below is to see whether any other PCI fn
	 * has reinitialized the ASIC (reset sync_ackd bits)
	 * and failed again while this IOC was waiting for hw
	 * semaphore (in bfa_iocpf_sm_semwait()).
	 */
	tmp_ackd = sync_ackd;
	if ((sync_reqd &  bfa_ioc_ct_sync_pos(ioc)) &&
			!(sync_ackd & bfa_ioc_ct_sync_pos(ioc)))
		sync_ackd |= bfa_ioc_ct_sync_pos(ioc);

	if (sync_reqd == sync_ackd) {
		bfa_reg_write(ioc->ioc_regs.ioc_fail_sync,
				bfa_ioc_ct_clear_sync_ackd(r32));
		bfa_reg_write(ioc->ioc_regs.ioc_fwstate, BFI_IOC_FAIL);
		bfa_reg_write(ioc->ioc_regs.alt_ioc_fwstate, BFI_IOC_FAIL);
		return BFA_TRUE;
	}

	/**
	 * If another PCI fn reinitialized and failed again while
	 * this IOC was waiting for hw sem, the sync_ackd bit for
	 * this IOC need to be set again to allow reinitialization.
	 */
	if (tmp_ackd != sync_ackd)
		bfa_reg_write(ioc->ioc_regs.ioc_fail_sync,
					(r32 | sync_ackd));
	return BFA_FALSE;
}

static void
bfa_ioc_ct_set_cur_ioc_fwstate(struct bfa_ioc_s *ioc,
		bfi_ioc_state_t fwstate)
{
	bfa_reg_write(ioc->ioc_regs.ioc_fwstate, fwstate);
}

static bfi_ioc_state_t
bfa_ioc_ct_get_cur_ioc_fwstate(struct bfa_ioc_s *ioc)
{
	return ((bfi_ioc_state_t)bfa_reg_read(ioc->ioc_regs.ioc_fwstate));
}

static void
bfa_ioc_ct_set_alt_ioc_fwstate(struct bfa_ioc_s *ioc,
		bfi_ioc_state_t fwstate)
{
	bfa_reg_write(ioc->ioc_regs.alt_ioc_fwstate, fwstate);
}

static bfi_ioc_state_t
bfa_ioc_ct_get_alt_ioc_fwstate(struct bfa_ioc_s *ioc)
{
	return ((bfi_ioc_state_t) bfa_reg_read(ioc->ioc_regs.alt_ioc_fwstate));
}

/**
 * @}
 */
