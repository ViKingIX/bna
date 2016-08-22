/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#include <defs/bfa_defs.h>
#include <bfi/bfi_reg.h>
#include <cna/bfa_ioc.h>

/*
 * Check the firmware state to know if pll_init has been completed already
 */
bfa_boolean_t
bfa_ioc_cb_pll_init_complete(bfa_os_addr_t rb)
{
	uint32_t ioc0_state, ioc1_state;

	ioc0_state = bfa_reg_read(rb + BFA_IOC0_STATE_REG);
	ioc0_state &= BFA_IOC_CB_FWSTATE_MASK;
	ioc1_state = bfa_reg_read(rb + BFA_IOC1_STATE_REG);
	ioc1_state &= BFA_IOC_CB_FWSTATE_MASK;

	if ((ioc0_state == BFI_IOC_OP) || (ioc0_state == BFI_IOC_DISABLED) ||
	  (ioc1_state == BFI_IOC_OP) || (ioc1_state == BFI_IOC_DISABLED))
		return BFA_TRUE;

	return BFA_FALSE;
}

bfa_status_t
bfa_ioc_cb_pll_init(bfa_os_addr_t rb, enum bfi_asic_mode mode)
{
	uint32_t	pll_sclk, pll_fclk;
	uint32_t	join_bits;

	pll_sclk = __APP_PLL_SCLK_ENABLE | __APP_PLL_SCLK_LRESETN |
		__APP_PLL_SCLK_P0_1(3U) |
		__APP_PLL_SCLK_JITLMT0_1(3U) |
		__APP_PLL_SCLK_CNTLMT0_1(3U);
	pll_fclk = __APP_PLL_LCLK_ENABLE | __APP_PLL_LCLK_LRESETN |
		__APP_PLL_LCLK_RSEL200500 | __APP_PLL_LCLK_P0_1(3U) |
		__APP_PLL_LCLK_JITLMT0_1(3U) |
		__APP_PLL_LCLK_CNTLMT0_1(3U);

	join_bits = bfa_reg_read(rb + BFA_IOC0_STATE_REG) &
				BFA_IOC_CB_JOIN_MASK;
	bfa_reg_write((rb + BFA_IOC0_STATE_REG), (BFI_IOC_UNINIT | join_bits));
	join_bits = bfa_reg_read(rb + BFA_IOC1_STATE_REG) &
				BFA_IOC_CB_JOIN_MASK;
	bfa_reg_write((rb + BFA_IOC1_STATE_REG), (BFI_IOC_UNINIT | join_bits));
	bfa_reg_write((rb + HOSTFN0_INT_MSK), 0xffffffffU);
	bfa_reg_write((rb + HOSTFN1_INT_MSK), 0xffffffffU);
	bfa_reg_write((rb + HOSTFN0_INT_STATUS), 0xffffffffU);
	bfa_reg_write((rb + HOSTFN1_INT_STATUS), 0xffffffffU);
	bfa_reg_write((rb + HOSTFN0_INT_MSK), 0xffffffffU);
	bfa_reg_write((rb + HOSTFN1_INT_MSK), 0xffffffffU);
	bfa_reg_write(rb + APP_PLL_SCLK_CTL_REG,
			  __APP_PLL_SCLK_LOGIC_SOFT_RESET);
	bfa_reg_write(rb + APP_PLL_SCLK_CTL_REG,
			  __APP_PLL_SCLK_BYPASS |
			  __APP_PLL_SCLK_LOGIC_SOFT_RESET);
	bfa_reg_write(rb + APP_PLL_LCLK_CTL_REG,
			  __APP_PLL_LCLK_LOGIC_SOFT_RESET);
	bfa_reg_write(rb + APP_PLL_LCLK_CTL_REG,
			  __APP_PLL_LCLK_BYPASS |
			  __APP_PLL_LCLK_LOGIC_SOFT_RESET);
	bfa_os_udelay(2);
	bfa_reg_write(rb + APP_PLL_SCLK_CTL_REG,
			  __APP_PLL_SCLK_LOGIC_SOFT_RESET);
	bfa_reg_write(rb + APP_PLL_LCLK_CTL_REG,
			  __APP_PLL_LCLK_LOGIC_SOFT_RESET);
	bfa_reg_write(rb + APP_PLL_SCLK_CTL_REG,
			  pll_sclk | __APP_PLL_SCLK_LOGIC_SOFT_RESET);
	bfa_reg_write(rb + APP_PLL_LCLK_CTL_REG,
			  pll_fclk | __APP_PLL_LCLK_LOGIC_SOFT_RESET);
	bfa_os_udelay(2000);
	bfa_reg_write((rb + HOSTFN0_INT_STATUS), 0xffffffffU);
	bfa_reg_write((rb + HOSTFN1_INT_STATUS), 0xffffffffU);
	bfa_reg_write((rb + APP_PLL_SCLK_CTL_REG), pll_sclk);
	bfa_reg_write((rb + APP_PLL_LCLK_CTL_REG), pll_fclk);

	return BFA_STATUS_OK;
}

/*
 * Check the firmware state to know if pll_init has been completed already
 */
bfa_boolean_t
bfa_ioc_ct_pll_init_complete(bfa_os_addr_t rb)
{
	uint32_t ioc0_state, ioc1_state;

	ioc0_state = bfa_reg_read(rb + BFA_IOC0_STATE_REG);
	ioc1_state = bfa_reg_read(rb + BFA_IOC1_STATE_REG);

	if ((ioc0_state == BFI_IOC_OP) || (ioc0_state == BFI_IOC_DISABLED) ||
	  (ioc1_state == BFI_IOC_OP) || (ioc1_state == BFI_IOC_DISABLED))
		return BFA_TRUE;

	return BFA_FALSE;
}

bfa_status_t
bfa_ioc_ct_pll_init(bfa_os_addr_t rb, enum bfi_asic_mode mode)
{
	uint32_t pll_sclk, pll_fclk, r32;
	bfa_boolean_t fcmode = (mode == BFI_ASIC_MODE_FC);

	pll_sclk = __APP_PLL_SCLK_LRESETN | __APP_PLL_SCLK_ENARST |
		__APP_PLL_SCLK_RSEL200500 | __APP_PLL_SCLK_P0_1(3U) |
		__APP_PLL_SCLK_JITLMT0_1(3U) |
		__APP_PLL_SCLK_CNTLMT0_1(1U);
	pll_fclk = __APP_PLL_LCLK_LRESETN | __APP_PLL_LCLK_ENARST |
		__APP_PLL_LCLK_RSEL200500 | __APP_PLL_LCLK_P0_1(3U) |
		__APP_PLL_LCLK_JITLMT0_1(3U) |
		__APP_PLL_LCLK_CNTLMT0_1(1U);

	if (fcmode) {
		bfa_reg_write((rb + OP_MODE), 0);
		bfa_reg_write((rb + ETH_MAC_SER_REG),
				__APP_EMS_CMLCKSEL |
				__APP_EMS_REFCKBUFEN2 |
				__APP_EMS_CHANNEL_SEL);
	} else {
		bfa_reg_write((rb + OP_MODE), __GLOBAL_FCOE_MODE);
		bfa_reg_write((rb + ETH_MAC_SER_REG),
				__APP_EMS_REFCKBUFEN1);
	}
	bfa_reg_write((rb + BFA_IOC0_STATE_REG), BFI_IOC_UNINIT);
	bfa_reg_write((rb + BFA_IOC1_STATE_REG), BFI_IOC_UNINIT);
	bfa_reg_write((rb + HOSTFN0_INT_MSK), 0xffffffffU);
	bfa_reg_write((rb + HOSTFN1_INT_MSK), 0xffffffffU);
	bfa_reg_write((rb + HOSTFN0_INT_STATUS), 0xffffffffU);
	bfa_reg_write((rb + HOSTFN1_INT_STATUS), 0xffffffffU);
	bfa_reg_write((rb + HOSTFN0_INT_MSK), 0xffffffffU);
	bfa_reg_write((rb + HOSTFN1_INT_MSK), 0xffffffffU);
	bfa_reg_write(rb + APP_PLL_SCLK_CTL_REG, pll_sclk |
		__APP_PLL_SCLK_LOGIC_SOFT_RESET);
	bfa_reg_write(rb + APP_PLL_LCLK_CTL_REG, pll_fclk |
		__APP_PLL_LCLK_LOGIC_SOFT_RESET);
	bfa_reg_write(rb + APP_PLL_SCLK_CTL_REG, pll_sclk |
		__APP_PLL_SCLK_LOGIC_SOFT_RESET | __APP_PLL_SCLK_ENABLE);
	bfa_reg_write(rb + APP_PLL_LCLK_CTL_REG, pll_fclk |
		__APP_PLL_LCLK_LOGIC_SOFT_RESET | __APP_PLL_LCLK_ENABLE);
	bfa_reg_read(rb + HOSTFN0_INT_MSK);
	bfa_os_udelay(2000);
	bfa_reg_write((rb + HOSTFN0_INT_STATUS), 0xffffffffU);
	bfa_reg_write((rb + HOSTFN1_INT_STATUS), 0xffffffffU);
	bfa_reg_write(rb + APP_PLL_SCLK_CTL_REG, pll_sclk |
		__APP_PLL_SCLK_ENABLE);
	bfa_reg_write(rb + APP_PLL_LCLK_CTL_REG, pll_fclk |
		__APP_PLL_LCLK_ENABLE);

	if (!fcmode) {
		bfa_reg_write((rb + PMM_1T_RESET_REG_P0), __PMM_1T_RESET_P);
		bfa_reg_write((rb + PMM_1T_RESET_REG_P1), __PMM_1T_RESET_P);
	}
	r32 = bfa_reg_read((rb + PSS_CTL_REG));
	r32 &= ~__PSS_LMEM_RESET;
	bfa_reg_write((rb + PSS_CTL_REG), r32);
	bfa_os_udelay(1000);
	if (!fcmode) {
		bfa_reg_write((rb + PMM_1T_RESET_REG_P0), 0);
		bfa_reg_write((rb + PMM_1T_RESET_REG_P1), 0);
	}

	bfa_reg_write((rb + MBIST_CTL_REG), __EDRAM_BISTR_START);
	bfa_os_udelay(1000);
	r32 = bfa_reg_read((rb + MBIST_STAT_REG));
	bfa_reg_write((rb + MBIST_CTL_REG), 0);
	return BFA_STATUS_OK;
}

static void
bfa_ioc_ct2_sclk_init(bfa_os_addr_t rb)
{
	uint32_t r32;

	/*
	 * put s_clk PLL and PLL FSM in reset
	 */
	r32 = bfa_reg_read((rb + CT2_APP_PLL_SCLK_CTL_REG));
	r32 &= ~(__APP_PLL_SCLK_ENABLE | __APP_PLL_SCLK_LRESETN);
	r32 |= (__APP_PLL_SCLK_ENARST | __APP_PLL_SCLK_BYPASS |
		__APP_PLL_SCLK_LOGIC_SOFT_RESET);
	bfa_reg_write((rb + CT2_APP_PLL_SCLK_CTL_REG), r32);

	/*
	 * Ignore mode and program for the max clock (which is FC16)
	 * Firmware/NFC will do the PLL init appropiately
	 */
	r32 = bfa_reg_read((rb + CT2_APP_PLL_SCLK_CTL_REG));
	r32 &= ~(__APP_PLL_SCLK_REFCLK_SEL | __APP_PLL_SCLK_CLK_DIV2);
	bfa_reg_write((rb + CT2_APP_PLL_SCLK_CTL_REG), r32);

	/*
	 * while doing PLL init dont clock gate ethernet subsystem
	 */
	r32 = bfa_reg_read((rb + CT2_CHIP_MISC_PRG));
	bfa_reg_write((rb + CT2_CHIP_MISC_PRG),
		      r32 | __ETH_CLK_ENABLE_PORT0);

	r32 = bfa_reg_read((rb + CT2_PCIE_MISC_REG));
	bfa_reg_write((rb + CT2_PCIE_MISC_REG),
		      r32 | __ETH_CLK_ENABLE_PORT1);

	/*
	 * set sclk value
	 */
	r32 = bfa_reg_read((rb + CT2_APP_PLL_SCLK_CTL_REG));
	r32 &= (__P_SCLK_PLL_LOCK | __APP_PLL_SCLK_REFCLK_SEL |
		__APP_PLL_SCLK_CLK_DIV2);
	bfa_reg_write((rb + CT2_APP_PLL_SCLK_CTL_REG), r32 | 0x1061731b);

	/*
	 * poll for s_clk lock or delay 1ms
	 */
	bfa_os_udelay(1000);

	/*
	 * Dont do clock gating for ethernet subsystem, firmware/NFC will
	 * do this appropriately
	 */
}

static void
bfa_ioc_ct2_lclk_init(bfa_os_addr_t rb)
{
	uint32_t r32;

	/*
	 * put l_clk PLL and PLL FSM in reset
	 */
	r32 = bfa_reg_read((rb + CT2_APP_PLL_LCLK_CTL_REG));
	r32 &= ~(__APP_PLL_LCLK_ENABLE | __APP_PLL_LCLK_LRESETN);
	r32 |= (__APP_PLL_LCLK_ENARST | __APP_PLL_LCLK_BYPASS |
		__APP_PLL_LCLK_LOGIC_SOFT_RESET);
	bfa_reg_write((rb + CT2_APP_PLL_LCLK_CTL_REG), r32);

	/*
	 * set LPU speed (set for FC16 which will work for other modes)
	 */
	r32 = bfa_reg_read((rb + CT2_CHIP_MISC_PRG));
	bfa_reg_write((rb + CT2_CHIP_MISC_PRG), r32);

	/*
	 * set LPU half speed (set for FC16 which will work for other modes)
	 */
	r32 = bfa_reg_read((rb + CT2_APP_PLL_LCLK_CTL_REG));
	bfa_reg_write((rb + CT2_APP_PLL_LCLK_CTL_REG), r32);

	/*
	 * set lclk for mode (set for FC16)
	 */
	r32 = bfa_reg_read((rb + CT2_APP_PLL_LCLK_CTL_REG));
	r32 &= (__P_LCLK_PLL_LOCK | __APP_LPUCLK_HALFSPEED);
	r32 |= 0x20c1731b;
	bfa_reg_write((rb + CT2_APP_PLL_LCLK_CTL_REG), r32);

	/*
	 * poll for s_clk lock or delay 1ms
	 */
	bfa_os_udelay(1000);
}

static void
bfa_ioc_ct2_mem_init(bfa_os_addr_t rb)
{
	uint32_t r32;

	r32 = bfa_reg_read((rb + PSS_CTL_REG));
	r32 &= ~__PSS_LMEM_RESET;
	bfa_reg_write((rb + PSS_CTL_REG), r32);
	bfa_os_udelay(1000);

	bfa_reg_write((rb + CT2_MBIST_CTL_REG), __EDRAM_BISTR_START);
	bfa_os_udelay(1000);
	bfa_reg_write((rb + CT2_MBIST_CTL_REG), 0);
}

bfa_boolean_t
bfa_ioc_ct2_pll_init_complete(bfa_os_addr_t rb)
{
	uint32_t ioc0_state, ioc1_state;

	ioc0_state = bfa_reg_read(rb + CT2_BFA_IOC0_STATE_REG);
	ioc1_state = bfa_reg_read(rb + CT2_BFA_IOC1_STATE_REG);

	if ((ioc0_state == BFI_IOC_OP) || (ioc0_state == BFI_IOC_DISABLED) ||
	  (ioc1_state == BFI_IOC_OP) || (ioc1_state == BFI_IOC_DISABLED))
		return BFA_TRUE;

	return BFA_FALSE;
}

void
bfa_ioc_ct2_mac_reset(bfa_os_addr_t rb)
{
	/* put port0, port1 MAC & AHB in reset */
	bfa_reg_write(rb + CT2_CSI_MAC_CONTROL_REG(0),
		(__CSI_MAC_RESET | __CSI_MAC_AHB_RESET));
	bfa_reg_write(rb + CT2_CSI_MAC_CONTROL_REG(1),
		(__CSI_MAC_RESET | __CSI_MAC_AHB_RESET));
}

static void
bfa_ioc_ct2_enable_flash(bfa_os_addr_t rb)
{
	volatile uint32_t r32;

	r32 = bfa_reg_read((rb + PSS_GPIO_OUT_REG));
	bfa_reg_write((rb + PSS_GPIO_OUT_REG), r32 & ~1);
	r32 = bfa_reg_read((rb + PSS_GPIO_OE_REG));
	bfa_reg_write((rb + PSS_GPIO_OE_REG), r32 | 1);
}

#define CT2_NFC_MAX_DELAY	1000
#define CT2_NFC_PAUSE_MAX_DELAY	4000
#define CT2_NFC_VER_VALID	0x147
#define CT2_NFC_STATE_RUNNING	0x20000001
#define BFA_IOC_PLL_POLL	1000000


static bfa_boolean_t
bfa_ioc_ct2_nfc_halted(bfa_os_addr_t rb)
{
	volatile uint32_t r32;

	r32 = bfa_reg_read(rb + CT2_NFC_CSR_SET_REG);
	if (r32 & __NFC_CONTROLLER_HALTED)
		return BFA_TRUE;

	return BFA_FALSE;
}

static void
bfa_ioc_ct2_nfc_halt(bfa_os_addr_t rb)
{
	int	i;

	bfa_reg_write(rb + CT2_NFC_CSR_SET_REG, __HALT_NFC_CONTROLLER);
	for (i = 0; i < CT2_NFC_MAX_DELAY; i++) {
		if (bfa_ioc_ct2_nfc_halted(rb))
			break;
		bfa_os_udelay(1000);
	}

	bfa_assert(bfa_ioc_ct2_nfc_halted(rb));
}

static void
bfa_ioc_ct2_nfc_resume(bfa_os_addr_t rb)
{
	volatile uint32_t r32;
	int i;

	bfa_reg_write(rb + CT2_NFC_CSR_CLR_REG, __HALT_NFC_CONTROLLER);
	for (i = 0; i < CT2_NFC_MAX_DELAY; i++) {
		r32 = bfa_reg_read(rb + CT2_NFC_CSR_SET_REG);
		if (!(r32 & __NFC_CONTROLLER_HALTED))
			return;
		bfa_os_udelay(1000);
	}
	bfa_assert(0);
}

static void
bfa_ioc_ct2_clk_reset(bfa_os_addr_t rb)
{
	volatile uint32_t r32;

	bfa_ioc_ct2_sclk_init(rb);
	bfa_ioc_ct2_lclk_init(rb);

	/*
	 * release soft reset on s_clk & l_clk
	 */
	r32 = bfa_reg_read((rb + CT2_APP_PLL_SCLK_CTL_REG));
	bfa_reg_write((rb + CT2_APP_PLL_SCLK_CTL_REG),
		      r32 & ~__APP_PLL_SCLK_LOGIC_SOFT_RESET);

	r32 = bfa_reg_read((rb + CT2_APP_PLL_LCLK_CTL_REG));
	bfa_reg_write((rb + CT2_APP_PLL_LCLK_CTL_REG),
		      r32 & ~__APP_PLL_LCLK_LOGIC_SOFT_RESET);

}

static void
bfa_ioc_ct2_nfc_clk_reset(bfa_os_addr_t rb)
{
	volatile uint32_t r32;
	uint32_t i;

	r32 = bfa_reg_read((rb + PSS_CTL_REG));
	r32 |= (__PSS_LPU0_RESET | __PSS_LPU1_RESET);
	bfa_reg_write((rb + PSS_CTL_REG), r32);

	bfa_reg_write(rb + CT2_CSI_FW_CTL_SET_REG,
		      __RESET_AND_START_SCLK_LCLK_PLLS);

	for (i = 0; i < BFA_IOC_PLL_POLL; i++) {
		r32 = bfa_reg_read(rb + CT2_NFC_FLASH_STS_REG);

		if ((r32 & __FLASH_PLL_INIT_AND_RESET_IN_PROGRESS))
			break;
	}
	bfa_assert((r32 & __FLASH_PLL_INIT_AND_RESET_IN_PROGRESS));

	for (i = 0; i < BFA_IOC_PLL_POLL; i++) {
		r32 = bfa_reg_read(rb + CT2_NFC_FLASH_STS_REG);

		if (!(r32 & __FLASH_PLL_INIT_AND_RESET_IN_PROGRESS))
			break;
	}
	bfa_assert(!(r32 & __FLASH_PLL_INIT_AND_RESET_IN_PROGRESS));

	r32 = bfa_reg_read(rb + CT2_CSI_FW_CTL_REG);
	bfa_assert(!(r32 & __RESET_AND_START_SCLK_LCLK_PLLS));
}

static void
bfa_ioc_ct2_wait_till_nfc_running(bfa_os_addr_t rb)
{
	volatile uint32_t r32;
	int i;

	if (bfa_ioc_ct2_nfc_halted(rb))
		bfa_ioc_ct2_nfc_resume(rb);
	for (i = 0; i < CT2_NFC_PAUSE_MAX_DELAY; i++) {
		r32 = bfa_reg_read(rb + CT2_NFC_STS_REG);
		if (r32 == CT2_NFC_STATE_RUNNING)
			return;
		bfa_os_udelay(1000);
	}

	r32 = bfa_reg_read(rb + CT2_NFC_STS_REG);
	bfa_assert(r32 == CT2_NFC_STATE_RUNNING);
}

bfa_status_t
bfa_ioc_ct2_pll_init(bfa_os_addr_t rb, enum bfi_asic_mode mode)
{
	volatile uint32_t wgn, r32;
	uint32_t nfc_ver;

	wgn = bfa_reg_read(rb + CT2_WGN_STATUS);

	if (wgn == (__WGN_READY | __GLBL_PF_VF_CFG_RDY)) {
		/*
		 * If flash is corrupted, enable flash explicitly
		 */
		bfa_ioc_ct2_clk_reset(rb);
		bfa_ioc_ct2_enable_flash(rb);

		bfa_ioc_ct2_mac_reset(rb);

		bfa_ioc_ct2_clk_reset(rb);
		bfa_ioc_ct2_enable_flash(rb);

	} else {
		nfc_ver = bfa_reg_read(rb + CT2_RSC_GPR15_REG);

		if ((nfc_ver >= CT2_NFC_VER_VALID) &&
		    (wgn == (__A2T_AHB_LOAD | __WGN_READY))) {

			bfa_ioc_ct2_wait_till_nfc_running(rb);

			bfa_ioc_ct2_nfc_clk_reset(rb);
		} else {
			bfa_ioc_ct2_nfc_halt(rb);

			bfa_ioc_ct2_clk_reset(rb);
			bfa_ioc_ct2_mac_reset(rb);
			bfa_ioc_ct2_clk_reset(rb);

		}
	}

	/*
	 * ASIC bug:
	 * The very first PCIe DMA Read done by LPU fails with a fatal error,
	 * when Address Translation Cache (ATC) has been enabled by system BIOS.
	 *
	 * Workaround:
	 * Disable Invalidated Tag Match Enable capability by setting the bit 26
	 * of CHIP_MISC_PRG to 0, by default it is set to 1.
	 */
	r32 = bfa_reg_read((rb + CT2_CHIP_MISC_PRG));
	bfa_reg_write((rb + CT2_CHIP_MISC_PRG), (r32 & 0xfbffffff));

	/*
	 * Mask the interrupts and clear any
	 * pending interrupts left by BIOS/EFI
	 */

	bfa_reg_write((rb + CT2_LPU0_HOSTFN_MBOX0_MSK), 1);
	bfa_reg_write((rb + CT2_LPU1_HOSTFN_MBOX0_MSK), 1);

	/* For first time initialization, no need to clear interrupts */
	r32 = bfa_reg_read(rb + HOST_SEM5_REG);
	if (r32 & 0x1) {
		r32 = bfa_reg_read((rb + CT2_LPU0_HOSTFN_CMD_STAT));
		if (r32 == 1) {
			bfa_reg_write((rb + CT2_LPU0_HOSTFN_CMD_STAT), 1);
			bfa_reg_read((rb + CT2_LPU0_HOSTFN_CMD_STAT));
		}
		r32 = bfa_reg_read((rb + CT2_LPU1_HOSTFN_CMD_STAT));
		if (r32 == 1) {
			bfa_reg_write((rb + CT2_LPU1_HOSTFN_CMD_STAT), 1);
			bfa_reg_read((rb + CT2_LPU1_HOSTFN_CMD_STAT));
		}
	}

	bfa_ioc_ct2_mem_init(rb);

	bfa_reg_write((rb + CT2_BFA_IOC0_STATE_REG), BFI_IOC_UNINIT);
	bfa_reg_write((rb + CT2_BFA_IOC1_STATE_REG), BFI_IOC_UNINIT);
	return BFA_STATUS_OK;
}
