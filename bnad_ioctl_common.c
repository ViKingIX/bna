/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#include "bna.h"

#include "bnad_compat.h"
#include "bnad.h"
#include "bfad_ioctl_cna.h"
#include "bnad_ioctl_common.h"
#include "bnad_trcmod.h"
#include "cna_os.h"
#include "cna/bfa_flash.h"
#include "cna/bfa_phy.h"
#include "cna/bfa_fru.h"
#include <defs/bfa_defs_vnic.h>
#include <ioctl/bfa_ioctl_vnic.h>
#include <ioctl/bfa_ioctl_pcifn.h>
#include <ioctl/bfa_ioctl_nwpart.h>


BNA_TRC_FILE(LDRV, IOCTL_CMN);

int
bnad_ioctl_ioc_get_version(unsigned long arg)
{
	bfa_ioctl_ioc_get_version_t iocmd;
	iocmd.version = BFA_IOCTL_VERSION;
	iocmd.status = BFA_STATUS_OK;

	if (cna_os_cp_to_user
		((void *)arg, (uint8_t *) &iocmd, sizeof(iocmd), 0))
		return EIO;

	return 0;
}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_ioc_get_info(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_ioc_info_t *iocmd = (bfa_ioctl_ioc_info_t *)cmd;
	unsigned long flags = 0;

	bnad_spin_lock(flags);
	bfa_ioc_get_adapter_serial_num(&bnad->bna.ioceth.ioc, iocmd->serialnum);
	iocmd->mac = bfa_ioc_get_mac(&bnad->bna.ioceth.ioc);
	iocmd->factory_mac = bfa_ioc_get_mfg_mac(&bnad->bna.ioceth.ioc);
	cna_os_memcpy(&iocmd->current_mac, (void *)bnad_get_stack_mac(bnad),
				sizeof(mac_t));
	iocmd->bfad_num = bnad->ident.id;
	bnad_spin_unlock(flags);
	bnad_hwpath_get(bnad, iocmd->hwpath, iocmd->adapter_hwpath);
	strncpy(iocmd->eth_name, bnad_get_stack_name(bnad),
			sizeof(bnad_get_stack_name(bnad)) - 1);
	strncpy(iocmd->name, bnad->adapter_name,
		sizeof(iocmd->name) - 1);
	strncpy(iocmd->port_name, bnad->port_name,
		sizeof(iocmd->port_name) - 1);
	iocmd->ioc_type = BFA_IOC_TYPE_LL;
	iocmd->status = BFA_STATUS_OK;

	return 0;
}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_ioc_get_attr(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_ioc_attr_t *iocmd = (bfa_ioctl_ioc_attr_t *)cmd;
	unsigned long flags = 0;

	cna_os_memset(&iocmd->ioc_attr, 0, sizeof(iocmd->ioc_attr));
	bnad_spin_lock(flags);
	bfa_ioc_get_attr(&bnad->bna.ioceth.ioc, &iocmd->ioc_attr);
	bnad_spin_unlock(flags);
	iocmd->ioc_attr.ioc_type = BFA_IOC_TYPE_LL;
	strncpy(iocmd->ioc_attr.driver_attr.driver, "bna", sizeof("bna"));
	strncpy(iocmd->ioc_attr.driver_attr.driver_ver,
		bfa_version, BFA_VERSION_LEN);
	strncpy(iocmd->ioc_attr.driver_attr.fw_ver,
		iocmd->ioc_attr.adapter_attr.fw_ver,
		BFA_VERSION_LEN);
	strncpy(iocmd->ioc_attr.driver_attr.bios_ver,
		iocmd->ioc_attr.adapter_attr.optrom_ver,
		BFA_VERSION_LEN);
	bnad_get_pci_attr(bnad, &iocmd->ioc_attr.pci_attr);
	iocmd->status = BFA_STATUS_OK;
	return 0;
}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_ioc_enable(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_gen_t *iocmd = (bfa_ioctl_gen_t *)cmd;

	bnad_ioceth_enable(bnad);

	iocmd->status = BFA_STATUS_OK;
	return 0;
}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_ioc_disable(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_gen_t *iocmd = (bfa_ioctl_gen_t *)cmd;

	bnad_ioceth_disable(bnad);

	iocmd->status = BFA_STATUS_OK;
	return 0;
}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_ioc_get_stats(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_ioc_stats_t *iocmd = (bfa_ioctl_ioc_stats_t *)cmd;
	unsigned long flags = 0;

	bnad_spin_lock(flags);
	bfa_ioc_fetch_stats(&bnad->bna.ioceth.ioc, &iocmd->ioc_stats);
	bnad_spin_unlock(flags);
	iocmd->status = BFA_STATUS_OK;

	return 0;
}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_ioc_reset_stats(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_gen_t *iocmd = (bfa_ioctl_gen_t *)cmd;
	unsigned long flags = 0;

	bnad_spin_lock(flags);
	bfa_ioc_clr_stats(&bnad->bna.ioceth.ioc);
	bnad_spin_unlock(flags);
	iocmd->status = BFA_STATUS_OK;

	return 0;
}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_ioc_set_adapter_name(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_ioc_set_adapter_name_t *iocmd =
			(bfa_ioctl_ioc_set_adapter_name_t *)cmd;

	strncpy(bnad->adapter_name, iocmd->name,
				sizeof(bnad->adapter_name) - 1);
	iocmd->status = BFA_STATUS_OK;
	return 0;
}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_ioc_set_port_name(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_ioc_set_adapter_name_t *iocmd =
			(bfa_ioctl_ioc_set_adapter_name_t *)cmd;

	strncpy(bnad->port_name, iocmd->name, sizeof(bnad->port_name) - 1);
	iocmd->status = BFA_STATUS_OK;
	return 0;
}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_debug_drv_trace(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_debug_t *iocmd = (bfa_ioctl_debug_t *)cmd;

	if (iocmd->bufsz < sizeof(bfa_trc_mod_t)) {
		iocmd->status = BFA_STATUS_EINVAL;
		goto out;
	}
	if (cna_os_cp_to_user
	    ((void *)(ulong) iocmd->buf_ptr,
	     bnad->trcmod, sizeof(bfa_trc_mod_t), 0)) {
		return EIO;
	}
	iocmd->bufsz = sizeof(bfa_trc_mod_t);
	iocmd->status = BFA_STATUS_OK;
out:
	return 0;
}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_debug_fw_trace(struct bnad_s *bnad, bfa_boolean_t saved, void *cmd)
{
	bfa_ioctl_debug_t *iocmd = (bfa_ioctl_debug_t *)cmd;
	char *buf;
	unsigned long flags = 0;
	uint32_t size = 0;

	if (iocmd->bufsz < sizeof(bfa_trc_mod_t)) {
		iocmd->status = BFA_STATUS_EINVAL;
		goto out;
	}

	size = iocmd->bufsz;
	buf = cna_os_vmalloc(size);
	if (!buf) {
		bfa_trc(bnad, 0x4444);
		return ENOMEM;
	}

	bnad_spin_lock(flags);
	if (!saved)
		iocmd->status = bfa_ioc_debug_fwtrc(&bnad->bna.ioceth.ioc,
					buf, (int *)&iocmd->bufsz);
	else
		iocmd->status = bfa_ioc_debug_fwsave(&bnad->bna.ioceth.ioc,
				buf, (int *)&iocmd->bufsz);
	bnad_spin_unlock(flags);

	if (iocmd->status != BFA_STATUS_OK) {
		cna_os_vfree(buf, size);
		bfa_trc(bnad, 0x5555);
		goto out;
	}

	if (cna_os_cp_to_user
	    ((void *)(ulong) iocmd->buf_ptr, buf, iocmd->bufsz, 0)) {
		cna_os_vfree(buf, size);
		bfa_trc(bnad, 0x55551);
		return EIO;
	}

	cna_os_vfree(buf, size);
out:
	return 0;

}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_debug_fw_trace_clear(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_gen_t *iocmd = (bfa_ioctl_gen_t *)cmd;
	unsigned long flags = 0;

	bnad_spin_lock(flags);
	bfa_ioc_debug_fwsave_clear(&bnad->bna.ioceth.ioc);
	bnad_spin_unlock(flags);

	iocmd->status = BFA_STATUS_OK;
	return 0;
}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_debug_fw_core(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_debug_t *iocmd = (bfa_ioctl_debug_t *)cmd;
	char *buf;
	uint32_t size = 0;
	unsigned long flags = 0;

	if (iocmd->bufsz < BFA_DEBUG_FW_CORE_CHUNK_SZ ||
			!is_aligned(iocmd->bufsz, 2) ||
			!is_aligned(iocmd->offset, 4)) {

		iocmd->status = BFA_STATUS_EINVAL;
		goto out;
	}

	size = iocmd->bufsz;
	buf = cna_os_vmalloc(size);
	if (!buf) {
		bfa_trc(bnad, 0x4444);
		return ENOMEM;
	}

	bnad_spin_lock(flags);

	iocmd->status = bfa_ioc_debug_fwcore(&bnad->bna.ioceth.ioc, buf,
			(uint32_t *)&iocmd->offset, (int *)&iocmd->bufsz);
	bnad_spin_unlock(flags);

	if (iocmd->status != BFA_STATUS_OK) {
		cna_os_vfree(buf, size);
		bfa_trc(bnad, 0x5555);
		goto out;
	}

	if (cna_os_cp_to_user
	    ((void *)(ulong) iocmd->buf_ptr, buf, iocmd->bufsz, 0)) {
		cna_os_vfree(buf, size);
		bfa_trc(bnad, 0x55551);
		return EIO;
	}

	cna_os_vfree(buf, size);
out:
	return 0;

}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_start_dtrc(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_gen_t *iocmd = (bfa_ioctl_gen_t *)cmd;
	unsigned long flags = 0;

	bnad_spin_lock(flags);
	bfa_trc_init(bnad->trcmod);
	bnad_spin_unlock(flags);

	iocmd->status = BFA_STATUS_OK;
	return 0;
}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_stop_dtrc(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_gen_t *iocmd = (bfa_ioctl_gen_t *)cmd;
	unsigned long flags = 0;

	bnad_spin_lock(flags);
	bfa_trc_stop(bnad->trcmod);
	bnad_spin_unlock(flags);

	iocmd->status = BFA_STATUS_OK;
	return 0;
}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_diag_regrd(struct bnad_s *bnad, void *cmd)
{
#ifndef BNAD_GLDV3
	bfa_ioctl_diag_regread_t *iocmd = (bfa_ioctl_diag_regread_t *)cmd;
#else
	bfa_ioctl_diag_regread_bufio_t *iocmd =
					(bfa_ioctl_diag_regread_bufio_t *)cmd;
#endif
	void *buf;
	unsigned long flags = 0;

	if (iocmd->off % 4)
		return EINVAL;

	if (iocmd->off + (iocmd->nwords << 2) > bnad_pci_resource_len(bnad))
		return EINVAL;

	buf = cna_os_kzalloc(iocmd->nwords << 2);
	if (!buf)
		return ENOMEM;

	bnad_spin_lock(flags);
	iocmd->status = bfa_diag_reg_read(&bnad->bna.diag, iocmd->off,
				iocmd->nwords, buf, iocmd->force);
	bnad_spin_unlock(flags);

#ifndef BNAD_GLDV3
	if (cna_os_cp_to_user
	    ((void *)(ulong) iocmd->buf_ptr, buf,
	     iocmd->nwords << 2, 0)) {
		cna_os_kfree(buf, iocmd->nwords << 2);
		return EIO;
	}
#else
	cna_os_memcpy((void *)(ulong)iocmd->buf_ptr, (void *)buf,
				iocmd->nwords << 2);
#endif
	cna_os_kfree(buf, iocmd->nwords << 2);
	return 0;
}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_diag_regwr(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_diag_regwrite_t *iocmd = (bfa_ioctl_diag_regwrite_t *)cmd;
	unsigned long flags = 0;

	if (iocmd->off % 4)
		return EINVAL;

	if (iocmd->off + sizeof(iocmd->word) > bnad_pci_resource_len(bnad))
		return EINVAL;

	bnad_spin_lock(flags);
	iocmd->status = bfa_diag_reg_write(&bnad->bna.diag, iocmd->off, 1,
					iocmd->word, iocmd->force);
	bnad_spin_unlock(flags);

	return 0;
}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_diag_memrd(struct bnad_s *bnad, void *cmd)
{
#ifndef BNAD_GLDV3
	bfa_ioctl_diag_kva_t *iocmd = (bfa_ioctl_diag_kva_t *)cmd;
#else
	bfa_ioctl_diag_kva_bufio_t *iocmd = (bfa_ioctl_diag_kva_bufio_t *)cmd;
#endif
	uint32_t to_copy;
	uint64_t *ptr;

	ptr = (uint64_t *)bnad;
	iocmd->kva_address = (uint64_t)(size_t)ptr;

	to_copy =
		((sizeof(struct bnad_s)) <=
		 (iocmd->uLength *
		  sizeof(uint32_t))) ? (sizeof(struct bnad_s))
		: (iocmd->uLength * sizeof(uint32_t));
	if (to_copy) {
#ifndef BNAD_GLDV3
		if (cna_os_cp_to_user
		    ((void *)(size_t)iocmd->buf_ptr,
		     (void *)(size_t)(iocmd->uAddress),
		     to_copy, 0)) {
			return EIO;
		}
#else
	cna_os_memcpy((void *)(size_t)iocmd->buf_ptr,
				  (void *)(size_t)(iocmd->uAddress),
				  to_copy);
#endif
	}
	iocmd->status = BFA_STATUS_OK;
	return 0;
}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_diag_lb_stat(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_diag_lb_stat_t *iocmd = (bfa_ioctl_diag_lb_stat_t *)cmd;

	/* TODO: Need BNAD support */
	iocmd->status = BFA_STATUS_OK;
	return 0;
}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_diag_memtest(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_diag_memtest_t *iocmd = (bfa_ioctl_diag_memtest_t *)cmd;
	struct bnad_ioctl_comp_s diag_comp;
	unsigned long flags = 0;

	bnad_init_completion(bnad, &diag_comp);
	bnad_spin_lock(flags);
	iocmd->status = bfa_diag_memtest(&bnad->bna.diag, &iocmd->memtest,
				iocmd->pat, &iocmd->result,
				bnad_cb_completion, &diag_comp);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		return 0;
	}
	bnad_wait_for_completion(bnad, &diag_comp, flags);
	bnad_spin_unlock(flags);
	iocmd->status = diag_comp.comp_status;
	return 0;
}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_diag_beacon_lport(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_diag_beacon_t *iocmd = (bfa_ioctl_diag_beacon_t *)cmd;
	unsigned long flags = 0;

	bnad_spin_lock(flags);
	iocmd->status = bfa_diag_beacon_port(&bnad->bna.diag,
				iocmd->beacon, iocmd->link_e2e_beacon,
				iocmd->second);
	bnad_spin_unlock(flags);
	return 0;
}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_diag_led(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_diag_led_t *iocmd = (bfa_ioctl_diag_led_t *)cmd;
	unsigned long flags = 0;

	bnad_spin_lock(flags);
	iocmd->status = bfa_diag_ledtest(&bnad->bna.diag, &iocmd->ledtest);
	bnad_spin_unlock(flags);
	return 0;
}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_diag_fwping(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_diag_fwping_t *iocmd = (bfa_ioctl_diag_fwping_t *)cmd;
	struct bnad_ioctl_comp_s diag_comp;
	unsigned long flags = 0;

	bnad_init_completion(bnad, &diag_comp);
	bnad_spin_lock(flags);
	iocmd->status = bfa_diag_fwping(&bnad->bna.diag, iocmd->cnt,
				iocmd->pattern, &iocmd->result,
				bnad_cb_completion, &diag_comp);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		return 0;
	}
	bnad_wait_for_completion(bnad, &diag_comp, flags);
	bnad_spin_unlock(flags);
	iocmd->status = diag_comp.comp_status;
	return 0;
}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_diag_temp(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_diag_get_temp_t *iocmd = (bfa_ioctl_diag_get_temp_t *)cmd;
	struct bnad_ioctl_comp_s diag_comp;
	unsigned long flags = 0;

	bnad_init_completion(bnad, &diag_comp);
	bnad_spin_lock(flags);
	iocmd->status = bfa_diag_tsensor_query(&bnad->bna.diag,
				&iocmd->result, bnad_cb_completion, &diag_comp);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		return 0;
	}
	bnad_wait_for_completion(bnad, &diag_comp, flags);
	bnad_spin_unlock(flags);
	iocmd->status = diag_comp.comp_status;
	return 0;
}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_diag_sfp(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_sfp_show_t *iocmd = (bfa_ioctl_sfp_show_t *)cmd;
	struct bnad_ioctl_comp_s diag_comp;
	unsigned long flags = 0;

	bnad_init_completion(bnad, &diag_comp);
	bnad_spin_lock(flags);
	iocmd->status = bfa_sfp_show(&bnad->bna.sfp, &iocmd->sfp,
					bnad_cb_completion, &diag_comp);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		return 0;
	}
	bnad_wait_for_completion(bnad, &diag_comp, flags);
	bnad_spin_unlock(flags);
	iocmd->status = diag_comp.comp_status;
	return 0;
}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_log_set_level(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_log_t *iocmd = (bfa_ioctl_log_t *)cmd;

	bnad_set_loglevel(bnad, iocmd->log_level, iocmd->mod_id);
	iocmd->status = BFA_STATUS_OK;
	return 0;
}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_log_get_level(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_log_t *iocmd = (bfa_ioctl_log_t *)cmd;

	iocmd->log_level = bnad_get_loglevel(bnad, iocmd->mod_id);
	iocmd->status = BFA_STATUS_OK;
	return 0;
}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_cee_attr(struct bnad_s *bnad, void *cmd)
{
	void *buffer = NULL;
	int ret_val = 0;
#ifndef BNAD_GLDV3
	bfa_ioctl_cee_attr_t *iocmd = (bfa_ioctl_cee_attr_t *)cmd;
#else
	bfa_ioctl_cee_bufio_t *iocmd = (bfa_ioctl_cee_bufio_t *)cmd;
#endif
	struct bnad_ioctl_comp_s cee_comp;
	unsigned long flags = 0;

	if (iocmd->buf_size != sizeof(bfa_cee_attr_t)) {
		iocmd->status = BFA_STATUS_VERSION_FAIL;
		return 0;
	}
	buffer = cna_os_kzalloc(iocmd->buf_size);
	if (!buffer)
		return ENOMEM;

	bnad_init_completion(bnad, &cee_comp);

	bnad_spin_lock(flags);
	iocmd->status = bfa_cee_get_attr(&bnad->bna.cee, buffer,
					 bnad_cb_completion, &cee_comp);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		goto out;
	}
	bnad_wait_for_completion(bnad, &cee_comp, flags);
	bnad_spin_unlock(flags);
	iocmd->status = cee_comp.comp_status;
	if (cee_comp.comp_status)
		goto out;

#ifndef BNAD_GLDV3
	if (cna_os_cp_to_user((void *)(ulong)iocmd->buf_ptr, buffer,
			iocmd->buf_size, 0))
		ret_val = EIO;
#else
	cna_os_memcpy((void *)(ulong)iocmd->buf_ptr, (void *)buffer,
				iocmd->buf_size);
#endif

out:
	cna_os_kfree(buffer, iocmd->buf_size);
	return ret_val;
}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_cee_get_stats(struct bnad_s *bnad, void *cmd)
{
	void *buffer = NULL;
	int ret_val = 0;
#ifndef BNAD_GLDV3
	bfa_ioctl_cee_stats_t *iocmd = (bfa_ioctl_cee_stats_t *)cmd;
#else
	bfa_ioctl_cee_bufio_t *iocmd = (bfa_ioctl_cee_bufio_t *)cmd;
#endif
	struct bnad_ioctl_comp_s cee_comp;
	unsigned long flags = 0;

	if (iocmd->buf_size != sizeof(bfa_cee_stats_t)) {
		iocmd->status = BFA_STATUS_VERSION_FAIL;
		return 0;
	}
	buffer = cna_os_kzalloc(iocmd->buf_size);
	if (!buffer)
		return ENOMEM;

	bnad_init_completion(bnad, &cee_comp);

	bnad_spin_lock(flags);
	iocmd->status = bfa_cee_get_stats(&bnad->bna.cee, buffer,
					 bnad_cb_completion, &cee_comp);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		goto out;
	}
	bnad_wait_for_completion(bnad, &cee_comp, flags);
	bnad_spin_unlock(flags);
	iocmd->status = cee_comp.comp_status;
	if (cee_comp.comp_status)
		goto out;

#ifndef BNAD_GLDV3
	if (cna_os_cp_to_user((void *)(ulong)iocmd->buf_ptr, buffer,
			iocmd->buf_size, 0))
		ret_val = EIO;
#else
	cna_os_memcpy((void *)(ulong)iocmd->buf_ptr, (void *)buffer,
				iocmd->buf_size);
#endif

out:
	cna_os_kfree(buffer, iocmd->buf_size);
	return ret_val;
}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_cee_reset_stats(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_gen_t *iocmd = (bfa_ioctl_gen_t *)cmd;
	unsigned long flags = 0;

	bnad_spin_lock(flags);
	iocmd->status = bfa_cee_reset_stats(&bnad->bna.cee, NULL, NULL);
	bnad_spin_unlock(flags);
	return 0;
}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_port_enable(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_gen_t *iocmd = (bfa_ioctl_gen_t *)cmd;
	struct bnad_ioctl_comp_s port_comp;
	unsigned long flags = 0;

	bnad_init_completion(bnad, &port_comp);

	bnad_spin_lock(flags);
	iocmd->status = bfa_port_enable(&bnad->bna.phy_port,
			bnad_cb_completion, &port_comp);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		return 0;
	}
	bnad_wait_for_completion(bnad, &port_comp, flags);
	bnad_spin_unlock(flags);

	iocmd->status = port_comp.comp_status;
	return 0;
}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_port_disable(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_gen_t *iocmd = (bfa_ioctl_gen_t *)cmd;
	struct bnad_ioctl_comp_s port_comp;
	unsigned long flags = 0;

	bnad_init_completion(bnad, &port_comp);

	bnad_spin_lock(flags);
	iocmd->status = bfa_port_disable(&bnad->bna.phy_port,
			bnad_cb_completion, &port_comp);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		return 0;
	}
	bnad_wait_for_completion(bnad, &port_comp, flags);
	bnad_spin_unlock(flags);

	iocmd->status = port_comp.comp_status;
	return 0;
}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_port_get_attr(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_port_attr_t *iocmd = (bfa_ioctl_port_attr_t *)cmd;

	cna_os_memset(&iocmd->attr, 0, sizeof (iocmd->attr));
	iocmd->attr.beacon = bnad->bna.diag.beacon.state;
	iocmd->attr.link_e2e_beacon = bnad->bna.diag.beacon.link_e2e;
	iocmd->status = BFA_STATUS_OK;
	return 0;
}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_port_get_stats(struct bnad_s *bnad, void *cmd)
{
#ifndef BNAD_GLDV3
	bfa_ioctl_port_stats_t *iocmd = (bfa_ioctl_port_stats_t *)cmd;
#else
	bfa_ioctl_port_stats_bufio_t *iocmd =
				(bfa_ioctl_port_stats_bufio_t *)cmd;
#endif
	struct bnad_ioctl_comp_s port_comp;
	void *buffer = NULL;
	int ret_val = 0;
	unsigned long flags = 0;

	if (iocmd->buf_size != sizeof(bfa_port_stats_t)) {
		iocmd->status = BFA_STATUS_VERSION_FAIL;
		return 0;
	}
	buffer = cna_os_kzalloc(iocmd->buf_size);
	if (!buffer)
		return ENOMEM;
	bnad_init_completion(bnad, &port_comp);
	bnad_spin_lock(flags);
	iocmd->status = bfa_port_get_stats(&bnad->bna.phy_port, buffer,
				bnad_cb_completion, &port_comp);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		goto out;
	}
	bnad_wait_for_completion(bnad, &port_comp, flags);
	bnad_spin_unlock(flags);
	iocmd->status = port_comp.comp_status;
	if (iocmd->status != BFA_STATUS_OK)
		goto out;
#ifndef BNAD_GLDV3
	if (cna_os_cp_to_user((void *)(ulong)iocmd->buf_ptr, buffer,
			iocmd->buf_size, 0))
		ret_val = EIO;
#else
	cna_os_memcpy((void *)(ulong)iocmd->buf_ptr, (void *)buffer,
				iocmd->buf_size);
#endif
out:
	cna_os_kfree(buffer, iocmd->buf_size);
	return ret_val;

}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_port_reset_stats(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_gen_t *iocmd = (bfa_ioctl_gen_t *)cmd;
	struct bnad_ioctl_comp_s port_comp;
	unsigned long flags = 0;

	/* Clear driver/bna stats */
	bnad_stats_clr(bnad);

	/* Clear hardware stats */
	bnad_init_completion(bnad, &port_comp);
	bnad_spin_lock(flags);
	iocmd->status = bfa_port_clear_stats(&bnad->bna.phy_port,
				bnad_cb_completion, &port_comp);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		return 0;
	}
	bnad_wait_for_completion(bnad, &port_comp, flags);
	bnad_spin_unlock(flags);
	iocmd->status = port_comp.comp_status;
	return 0;
}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_ethport_get_attr(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_ethport_attr_t *iocmd = (bfa_ioctl_ethport_attr_t *)cmd;
	unsigned long flags = 0;

	memset(&iocmd->attr, 0, sizeof(iocmd->attr));
	iocmd->attr.speed_supported = BFA_PORT_SPEED_10GBPS;
	iocmd->attr.speed = BFA_PORT_SPEED_UNKNOWN;

	bnad_spin_lock(flags);
	if (bfa_ioc_is_disabled(&bnad->bna.ioceth.ioc))
		iocmd->attr.port_state = BFA_PORT_ST_IOCDIS;
	else if (bfa_ioc_fw_mismatch(&bnad->bna.ioceth.ioc))
		iocmd->attr.port_state = BFA_PORT_ST_FWMISMATCH;
	else if (bna_ioceth_state_is_failed(&bnad->bna.ioceth))
		iocmd->attr.port_state = BFA_PORT_ST_IOCDOWN;
	else
		bnad_get_ethport_attr(bnad, &iocmd->attr);
	bnad_spin_unlock(flags);

	iocmd->status = BFA_STATUS_OK;
	return 0;
}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_ethport_get_cfg(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_ethport_cfg_t *iocmd = (bfa_ioctl_ethport_cfg_t *)cmd;

	memset(&iocmd->cfg, 0, sizeof(iocmd->cfg));

	iocmd->cfg.opmode = BFA_PORT_OPMODE_NORMAL;

	if (bna_enet_type_get(&bnad->bna.enet) == BNA_ENET_T_LOOPBACK_INTERNAL)
		iocmd->cfg.opmode = BFA_PORT_OPMODE_LB_EXT;

	if (bna_enet_type_get(&bnad->bna.enet) == BNA_ENET_T_LOOPBACK_EXTERNAL)
		iocmd->cfg.opmode = BFA_PORT_OPMODE_LB_CBL;

	bnad_get_ethport_cfg(bnad, &iocmd->cfg);

	iocmd->status = BFA_STATUS_OK;
	return 0;
}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_ethport_get_stats(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_ethport_stats_t *iocmd = (bfa_ioctl_ethport_stats_t *)cmd;
	uint64_t bmap;
	int i;

	bnad_hw_stats_get(bnad);
	memset(&(iocmd->stats.txf_stats[0]), 0,
				sizeof(struct bfi_enet_stats_txf));
	memset(&(iocmd->stats.rxf_stats[0]), 0,
				sizeof(struct bfi_enet_stats_rxf));
	bmap = bna_tx_rid_mask(&bnad->bna);
	i = 0;
	while (bmap) {
		if (bmap & 1) {
			memcpy(&(iocmd->stats.txf_stats[0]),
			&(bnad->stats.bna_stats->hw_stats.txf_stats[i]),
			sizeof(struct bfi_enet_stats_txf));
			break;
		}
		bmap >>= 1;
		i++;
	}
	bmap = bna_rx_rid_mask(&bnad->bna);
	i = 0;
	while (bmap) {
		if (bmap & 1) {
			memcpy(&(iocmd->stats.rxf_stats[0]),
				&(bnad->stats.bna_stats->hw_stats.rxf_stats[i]),
			sizeof(struct bfi_enet_stats_rxf));
			break;
		}
		bmap >>= 1;
		i++;
	}
	bnad_drv_stats_get(bnad, &iocmd->stats.drv_stats);
	iocmd->status = BFA_STATUS_OK;

	return 0;
}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_sfp_media(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_sfp_media_t *iocmd = (bfa_ioctl_sfp_media_t *)cmd;
	struct bnad_ioctl_comp_s sfp_comp;
	unsigned long flags = 0;

	bnad_init_completion(bnad, &sfp_comp);
	bnad_spin_lock(flags);
	iocmd->status = bfa_sfp_media(&bnad->bna.sfp, &iocmd->media,
				bnad_cb_completion, &sfp_comp);
	if (iocmd->status != BFA_STATUS_SFP_NOT_READY) {
		bnad_spin_unlock(flags);
		return 0;
	}
	bnad_wait_for_completion(bnad, &sfp_comp, flags);
	bnad_spin_unlock(flags);
	iocmd->status = sfp_comp.comp_status;
	return 0;
}

/* Note: Should be called holding bnad_conf_lock */
int
bnad_ioctl_sfp_speed(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_sfp_speed_t *iocmd = (bfa_ioctl_sfp_speed_t *)cmd;
	struct bnad_ioctl_comp_s sfp_comp;
	unsigned long flags = 0;

	bnad_init_completion(bnad, &sfp_comp);
	bnad_spin_lock(flags);
	iocmd->status = bfa_sfp_speed(&bnad->bna.sfp, iocmd->speed,
				bnad_cb_completion, &sfp_comp);
	if (iocmd->status != BFA_STATUS_SFP_NOT_READY) {
		bnad_spin_unlock(flags);
		return 0;
	}
	bnad_wait_for_completion(bnad, &sfp_comp, flags);
	bnad_spin_unlock(flags);
	iocmd->status = sfp_comp.comp_status;
	return 0;
}

int
bnad_ioctl_ethboot_cfg(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_ethboot_t *iocmd = (bfa_ioctl_ethboot_t *)cmd;
	struct bnad_ioctl_comp_s fcomp;
	unsigned long flags = 0;

	bnad_init_completion(bnad, &fcomp);
	bnad_spin_lock(flags);
	iocmd->status = bfa_flash_update_part(&bnad->bna.flash,
				BFA_FLASH_PART_PXECFG,
				bna_port_id_get(&bnad->bna), &iocmd->cfg,
				sizeof(struct bfa_ethboot_cfg_s), 0,
				bnad_cb_completion, &fcomp, 1);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		return 0;
	}

	bnad_wait_for_completion(bnad, &fcomp, flags);
	bnad_spin_unlock(flags);
	iocmd->status = fcomp.comp_status;
	return 0;
}

int
bnad_ioctl_ethboot_query(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_ethboot_t *iocmd = (bfa_ioctl_ethboot_t *)cmd;
	struct bnad_ioctl_comp_s fcomp;
	unsigned long flags = 0;

	bnad_init_completion(bnad, &fcomp);
	bnad_spin_lock(flags);
	iocmd->status = bfa_flash_read_part(&bnad->bna.flash,
				BFA_FLASH_PART_PXECFG,
				bna_port_id_get(&bnad->bna),
				&iocmd->cfg, sizeof(struct bfa_ethboot_cfg_s),
				0, bnad_cb_completion, &fcomp, 1);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		return 0;
	}

	bnad_wait_for_completion(bnad, &fcomp, flags);
	bnad_spin_unlock(flags);
	iocmd->status = fcomp.comp_status;
	return 0;
}

int
bnad_ioctl_iscsiboot_cfg(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_iscsiboot_t *iocmd = (bfa_ioctl_iscsiboot_t *)cmd;
	struct bnad_ioctl_comp_s fcomp;
	unsigned long flags = 0;

	bnad_init_completion(bnad, &fcomp);
	bnad_spin_lock(flags);
	iocmd->status = bfa_flash_update_part(&bnad->bna.flash,
				BFA_FLASH_PART_BOOT,
				bna_port_id_get(&bnad->bna), &iocmd->cfg,
				sizeof(struct bfa_iscsiboot_cfg_s), 0,
				bnad_cb_completion, &fcomp, 1);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		return 0;
	}

	bnad_wait_for_completion(bnad, &fcomp, flags);
	bnad_spin_unlock(flags);
	iocmd->status = fcomp.comp_status;
	return 0;
}

int
bnad_ioctl_iscsiboot_query(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_iscsiboot_t *iocmd = (bfa_ioctl_iscsiboot_t *)cmd;
	struct bnad_ioctl_comp_s fcomp;
	unsigned long flags = 0;

	bnad_init_completion(bnad, &fcomp);
	bnad_spin_lock(flags);
	iocmd->status = bfa_flash_read_part(&bnad->bna.flash,
				BFA_FLASH_PART_BOOT,
				bna_port_id_get(&bnad->bna),
				&iocmd->cfg, sizeof(struct bfa_iscsiboot_cfg_s),
				0, bnad_cb_completion, &fcomp, 1);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		return 0;
	}

	bnad_wait_for_completion(bnad, &fcomp, flags);
	bnad_spin_unlock(flags);
	iocmd->status = fcomp.comp_status;
	return 0;
}

int
bnad_ioctl_flash_get_attr(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_flash_attr_t *iocmd = (bfa_ioctl_flash_attr_t *)cmd;
	struct bnad_ioctl_comp_s fcomp;
	unsigned long flags = 0;

	bnad_init_completion(bnad, &fcomp);
	bnad_spin_lock(flags);
	iocmd->status = bfa_flash_get_attr(&bnad->bna.flash, &iocmd->attr,
				       bnad_cb_completion, &fcomp);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		return 0;
	}
	bnad_wait_for_completion(bnad, &fcomp, flags);
	bnad_spin_unlock(flags);
	iocmd->status = fcomp.comp_status;
	return 0;
}

int
bnad_ioctl_flash_read_part(struct bnad_s *bnad, void *cmd, uint32_t offset)
{
#ifndef BNAD_SOLARIS11
	bfa_ioctl_flash_t *iocmd = (bfa_ioctl_flash_t *)cmd;
#else
	bfa_ioctl_flash_bufio_t *iocmd = (bfa_ioctl_flash_bufio_t *)cmd;
#endif
	struct bnad_ioctl_comp_s fcomp;
	void *buf;
	unsigned long flags = 0;

	if (iocmd->bufsz <= 0) {
		bfa_trc(bnad, iocmd->type);
		return EINVAL;
	}

	buf = cna_os_vmalloc(iocmd->bufsz);
	if (!buf) {
		bfa_trc(bnad, 0x4444);
		return ENOMEM;
	}

	bnad_init_completion(bnad, &fcomp);
	bnad_spin_lock(flags);
	iocmd->status =
		bfa_flash_read_part(&bnad->bna.flash, iocmd->type,
					iocmd->instance, buf, iocmd->bufsz,
					offset, bnad_cb_completion, &fcomp, 1);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		cna_os_vfree(buf, iocmd->bufsz);
		return 0;
	}

	bnad_wait_for_completion(bnad, &fcomp, flags);
	bnad_spin_unlock(flags);
	iocmd->status = fcomp.comp_status;

#ifndef BNAD_SOLARIS11
	if (cna_os_cp_to_user((void *)(ulong)iocmd->buf_ptr,
					(void *)buf, iocmd->bufsz, 0)) {
		cna_os_vfree(buf, iocmd->bufsz);
		return EIO;
	}
#else
	cna_os_memcpy((void *)(ulong)iocmd->buf_ptr, (void *)buf,
				iocmd->bufsz);
#endif
	cna_os_vfree(buf, iocmd->bufsz);
	return 0;
}

int
bnad_ioctl_flash_erase_part(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_flash_t *iocmd = (bfa_ioctl_flash_t *)cmd;
	struct bnad_ioctl_comp_s fcomp;
	unsigned long flags = 0;

	bnad_init_completion(bnad, &fcomp);
	bnad_spin_lock(flags);
	iocmd->status = bfa_flash_erase_part(&bnad->bna.flash, iocmd->type,
				iocmd->instance, bnad_cb_completion, &fcomp);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		return 0;
	}

	bnad_wait_for_completion(bnad, &fcomp, flags);
	bnad_spin_unlock(flags);
	iocmd->status = fcomp.comp_status;
	return 0;
}

int
bnad_ioctl_flash_update_part(struct bnad_s *bnad, void *cmd, uint32_t offset)
{
	bfa_ioctl_flash_t *iocmd = (bfa_ioctl_flash_t *)cmd;
	void *buffer;
	struct bnad_ioctl_comp_s fcomp;
	unsigned long flags = 0;

	buffer = cna_os_vmalloc(iocmd->bufsz);
	if (buffer == NULL)
		return ENOMEM;

	if (cna_os_cp_from_user((uint8_t *)buffer,
			(void *)(ulong)iocmd->buf_ptr, iocmd->bufsz, 0)) {
		cna_os_vfree(buffer, iocmd->bufsz);
		return EIO;
	}

	bnad_init_completion(bnad, &fcomp);
	bnad_spin_lock(flags);
	iocmd->status = bfa_flash_update_part(&bnad->bna.flash, iocmd->type,
				  iocmd->instance, buffer, iocmd->bufsz,
				  offset, bnad_cb_completion, &fcomp, 1);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		cna_os_vfree(buffer, iocmd->bufsz);
		return 0;
	}

	bnad_wait_for_completion(bnad, &fcomp, flags);
	bnad_spin_unlock(flags);
	iocmd->status = fcomp.comp_status;

	cna_os_vfree(buffer, iocmd->bufsz);
	return 0;
}

int
bnad_ioctl_flash_optrom(struct bnad_s *bnad, unsigned int cmd, void *pcmd)
{
	bfa_ioctl_gen_t *iocmd = (bfa_ioctl_gen_t *)pcmd;
	struct bnad_ioctl_comp_s fcomp;
	unsigned long flags = 0;

	bnad_init_completion(bnad, &fcomp);
	bnad_spin_lock(flags);
	if (cmd == IOCMD_FLASH_ENABLE_OPTROM)
		iocmd->status = bfa_ablk_optrom_en(&bnad->bna.ablk,
						bnad_cb_completion, &fcomp);
	else
		iocmd->status = bfa_ablk_optrom_dis(&bnad->bna.ablk,
						bnad_cb_completion, &fcomp);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		return 0;
	}
	bnad_wait_for_completion(bnad, &fcomp, flags);
	bnad_spin_unlock(flags);
	iocmd->status = fcomp.comp_status;
	return 0;
}

int
bnad_ioctl_flash_mfg_info(struct bnad_s *bnad, void *cmd)
{
       return EINVAL;
}

int
bnad_ioctl_flash_get_cfg_bootver(struct bnad_s *bnad, void *cmd)
{
#ifndef BNAD_GLDV3
	bfa_ioctl_flash_t *iocmd = (bfa_ioctl_flash_t *)cmd;
#else
	bfa_ioctl_flash_bufio_t *iocmd = (bfa_ioctl_flash_bufio_t *)cmd;
#endif
	struct bnad_ioctl_comp_s fcomp;
	void *buf;
	unsigned long flags = 0;

	buf = cna_os_vmalloc(iocmd->bufsz);
	if (!buf) {
		bfa_trc(bnad, 0x4444);
		return ENOMEM;
	}

	memset(buf, 0, iocmd->bufsz);
	bnad_init_completion(bnad, &fcomp);
	bnad_spin_lock(flags);
	iocmd->status = bfa_flash_get_boot_version(&bnad->bna.flash, buf,
				iocmd->bufsz, bnad_cb_completion, &fcomp);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		cna_os_vfree(buf, iocmd->bufsz);
		return 0;
	}

	bnad_wait_for_completion(bnad, &fcomp, flags);
	bnad_spin_unlock(flags);
	iocmd->status = fcomp.comp_status;

#ifndef BNAD_GLDV3
	if (cna_os_cp_to_user((void *)(ulong)iocmd->buf_ptr,
					(void *)buf, iocmd->bufsz, 0)) {
		cna_os_vfree(buf, iocmd->bufsz);
		return EIO;
	}
#else
	cna_os_memcpy((void *)(ulong)iocmd->buf_ptr, (void *)buf,
				iocmd->bufsz);
#endif

	cna_os_vfree(buf, iocmd->bufsz);
	return 0;
}

int
bnad_ioctl_phy_get_attr(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_phy_attr_t *iocmd = (bfa_ioctl_phy_attr_t *)cmd;
	struct bnad_ioctl_comp_s fcomp;
	unsigned long flags = 0;

	bnad_init_completion(bnad, &fcomp);
	bnad_spin_lock(flags);
	iocmd->status = bfa_phy_get_attr(&bnad->bna.phy, iocmd->instance,
				&iocmd->attr, bnad_cb_completion, &fcomp);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		return 0;
	}
	bnad_wait_for_completion(bnad, &fcomp, flags);
	bnad_spin_unlock(flags);
	iocmd->status = fcomp.comp_status;
	return 0;
}

int
bnad_ioctl_phy_get_stats(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_phy_stats_t *iocmd = (bfa_ioctl_phy_stats_t *)cmd;
	struct bnad_ioctl_comp_s fcomp;
	unsigned long flags = 0;

	bnad_init_completion(bnad, &fcomp);
	bnad_spin_lock(flags);
	iocmd->status = bfa_phy_get_stats(&bnad->bna.phy, iocmd->instance,
				&iocmd->stats, bnad_cb_completion, &fcomp);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		return 0;
	}
	bnad_wait_for_completion(bnad, &fcomp, flags);
	bnad_spin_unlock(flags);
	iocmd->status = fcomp.comp_status;
	return 0;
}

int
bnad_ioctl_phy_update(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_phy_t *iocmd = (bfa_ioctl_phy_t *)cmd;
	void *buffer;
	struct bnad_ioctl_comp_s fcomp;
	unsigned long flags = 0;

	buffer = cna_os_vmalloc(iocmd->bufsz);
	if (buffer == NULL)
		return ENOMEM;

	if (cna_os_cp_from_user((uint8_t *)buffer,
			(void *)(ulong)iocmd->buf_ptr, iocmd->bufsz, 0)) {
		cna_os_vfree(buffer, iocmd->bufsz);
		return EIO;
	}

	bnad_init_completion(bnad, &fcomp);
	bnad_spin_lock(flags);
	iocmd->status = bfa_phy_update(&bnad->bna.phy, iocmd->instance,
				buffer, iocmd->bufsz,
				0, bnad_cb_completion, &fcomp);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		cna_os_vfree(buffer, iocmd->bufsz);
		return 0;
	}

	bnad_wait_for_completion(bnad, &fcomp, flags);
	bnad_spin_unlock(flags);
	iocmd->status = fcomp.comp_status;

	cna_os_vfree(buffer, iocmd->bufsz);
	return 0;
}

int
bnad_ioctl_phy_read(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_phy_t *iocmd = (bfa_ioctl_phy_t *)cmd;
	struct bnad_ioctl_comp_s fcomp;
	void *buf;
	unsigned long flags = 0;

	buf = cna_os_vmalloc(iocmd->bufsz);
	if (!buf) {
		bfa_trc(bnad, 0x4444);
		return ENOMEM;
	}

	bnad_init_completion(bnad, &fcomp);
	bnad_spin_lock(flags);
	iocmd->status =
		bfa_phy_read(&bnad->bna.phy, iocmd->instance,
					buf, iocmd->bufsz, 0,
					bnad_cb_completion, &fcomp);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		cna_os_vfree(buf, iocmd->bufsz);
		return 0;
	}

	bnad_wait_for_completion(bnad, &fcomp, flags);
	bnad_spin_unlock(flags);
	iocmd->status = fcomp.comp_status;

	if (cna_os_cp_to_user((void *)(ulong)iocmd->buf_ptr,
					(void *)buf, iocmd->bufsz, 0)) {
		cna_os_vfree(buf, iocmd->bufsz);
		return EIO;
	}
	cna_os_vfree(buf, iocmd->bufsz);
	return 0;
}

int
bnad_ioctl_pcifn_create(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_pcifn_t *iocmd = (bfa_ioctl_pcifn_t *)cmd;
	struct bnad_ioctl_comp_s fcomp;
	unsigned long flags = 0;

	bnad_init_completion(bnad, &fcomp);

	bnad_spin_lock(flags);
	iocmd->status = bfa_ablk_pf_create(&bnad->bna.ablk,
			&iocmd->pcifn_id,
			iocmd->port,
			iocmd->pcifn_class,
			iocmd->bw_min,
			iocmd->bw_max,
			bnad_cb_completion, &fcomp);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		return 0;
	}
	bnad_wait_for_completion(bnad, &fcomp, flags);
	bnad_spin_unlock(flags);

	iocmd->status = fcomp.comp_status;
	return 0;
}

int
bnad_ioctl_pcifn_delete(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_pcifn_t *iocmd = (bfa_ioctl_pcifn_t *)cmd;
	struct bnad_ioctl_comp_s fcomp;
	unsigned long flags = 0;

	bnad_init_completion(bnad, &fcomp);
	bnad_spin_lock(flags);
	iocmd->status = bfa_ablk_pf_delete(&bnad->bna.ablk,
			iocmd->pcifn_id,
			bnad_cb_completion, &fcomp);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		return 0;
	}
	bnad_wait_for_completion(bnad, &fcomp, flags);
	bnad_spin_unlock(flags);

	iocmd->status = fcomp.comp_status;
	return 0;
}

int
bnad_ioctl_port_nwpar_enable(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_nwpar_t *iocmd = (bfa_ioctl_nwpar_t *)cmd;
	struct bnad_ioctl_comp_s fcomp;
	unsigned long flags = 0;

	bnad_init_completion(bnad, &fcomp);
	bnad_spin_lock(flags);
	iocmd->status = bfa_ablk_nwpar_enable(&bnad->bna.ablk,
			bnad_cb_completion, &fcomp);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		return 0;
	}
	bnad_wait_for_completion(bnad, &fcomp, flags);
	bnad_spin_unlock(flags);

	iocmd->status = fcomp.comp_status;
	return 0;
}

int
bnad_ioctl_port_nwpar_disable(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_nwpar_t *iocmd = (bfa_ioctl_nwpar_t *)cmd;
	struct bnad_ioctl_comp_s fcomp;
	unsigned long flags = 0;

	bnad_init_completion(bnad, &fcomp);
	bnad_spin_lock(flags);
	iocmd->status = bfa_ablk_nwpar_disable(&bnad->bna.ablk,
			bnad_cb_completion, &fcomp);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		return 0;
	}
	bnad_wait_for_completion(bnad, &fcomp, flags);
	bnad_spin_unlock(flags);

	iocmd->status = fcomp.comp_status;
	return 0;
}

int
bnad_ioctl_pcifn_enable(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_pcifn_t *iocmd = (bfa_ioctl_pcifn_t *)cmd;
	struct bnad_ioctl_comp_s fcomp;
	unsigned long flags = 0;

	bnad_init_completion(bnad, &fcomp);
	bnad_spin_lock(flags);
	iocmd->status = bfa_ablk_nwpar_pf_enable(&bnad->bna.ablk,
			iocmd->pcifn_id,
			bnad_cb_completion, &fcomp);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		return 0;
	}
	bnad_wait_for_completion(bnad, &fcomp, flags);
	bnad_spin_unlock(flags);

	iocmd->status = fcomp.comp_status;
	return 0;
}

int
bnad_ioctl_pcifn_disable(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_pcifn_t *iocmd = (bfa_ioctl_pcifn_t *)cmd;
	struct bnad_ioctl_comp_s fcomp;
	unsigned long flags = 0;

	bnad_init_completion(bnad, &fcomp);
	bnad_spin_lock(flags);
	iocmd->status = bfa_ablk_nwpar_pf_disable(&bnad->bna.ablk,
			iocmd->pcifn_id,
			bnad_cb_completion, &fcomp);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		return 0;
	}
	bnad_wait_for_completion(bnad, &fcomp, flags);
	bnad_spin_unlock(flags);

	iocmd->status = fcomp.comp_status;
	return 0;
}

int
bnad_ioctl_pcifn_pers_change(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_pcifn_t *iocmd = (bfa_ioctl_pcifn_t *)cmd;
	struct bnad_ioctl_comp_s fcomp;
	unsigned long flags = 0;

	bnad_init_completion(bnad, &fcomp);
	bnad_spin_lock(flags);
	iocmd->status = bfa_ablk_nwpar_pf_pers_change
			(&bnad->bna.ablk,
			iocmd->pcifn_id, iocmd->pcifn_class,
			bnad_cb_completion, &fcomp);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		return 0;
	}
	bnad_wait_for_completion(bnad, &fcomp, flags);
	bnad_spin_unlock(flags);

	iocmd->status = fcomp.comp_status;
	return 0;
}

int
bnad_ioctl_vnic_query(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_vnic_attr_t *iocmd = (bfa_ioctl_vnic_attr_t *)cmd;
	unsigned long flags = 0;
	void *buf;

	buf = cna_os_vmalloc(iocmd->bufsz);
	if (!buf) {
		bfa_trc(bnad, 0x4444);
		return ENOMEM;
	}

	cna_os_memset(buf, 0, iocmd->bufsz);

	bnad_spin_lock(flags);
	bna_vnic_attr_get(&bnad->bna, (struct bfa_vnic_attr_s*)buf);
	iocmd->status = BFA_STATUS_OK;
	bnad_spin_unlock(flags);

	if (cna_os_cp_to_user((void *)(ulong)iocmd->buf_ptr,
					(void *)buf, iocmd->bufsz, 0)) {
		cna_os_vfree(buf, iocmd->bufsz);
		return EIO;
	}
	cna_os_vfree(buf, iocmd->bufsz);
	iocmd->status = BFA_STATUS_OK;
	return 0;
}

int
bnad_ioctl_vnic_enable(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_gen_t *iocmd = (bfa_ioctl_gen_t *)cmd;

	iocmd->status = bnad_ioceth_enable(bnad);

	return 0;
}

int
bnad_ioctl_vnic_disable(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_gen_t *iocmd = (bfa_ioctl_gen_t *)cmd;

	iocmd->status = bnad_ioceth_disable(bnad);

	return 0;
}

int
bnad_ioctl_pcifn_bw(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_pcifn_t *iocmd = (bfa_ioctl_pcifn_t *)cmd;
	struct bnad_ioctl_comp_s fcomp;
	unsigned long flags = 0;

	bnad_init_completion(bnad, &fcomp);
	bnad_spin_lock(flags);

	iocmd->status = bfa_ablk_pf_update(&bnad->bna.ablk,
			iocmd->pcifn_id,
			iocmd->bw_min,
			iocmd->bw_max,
			bnad_cb_completion, &fcomp);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		return 0;
	}
	bnad_wait_for_completion(bnad, &fcomp, flags);
	bnad_spin_unlock(flags);
	iocmd->status = fcomp.comp_status;
	return 0;
}


int
bnad_ioctl_vnic_stats(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_vnic_stats_t *iocmd = (bfa_ioctl_vnic_stats_t *)cmd;
	void *buf;
	unsigned long flags = 0;

	buf = cna_os_vmalloc(iocmd->bufsz);
	if (!buf) {
		bfa_trc(bnad, 0x4444);
		return ENOMEM;
	}

	bnad_spin_lock(flags);
	bna_vnic_stats_get(&bnad->bna, (struct bfa_vnic_stats_s*)buf);
	bnad_spin_unlock(flags);

	if (cna_os_cp_to_user((void *)(ulong)iocmd->buf_ptr,
					(void *)buf, iocmd->bufsz, 0)) {
		cna_os_vfree(buf, iocmd->bufsz);
		return EIO;
	}
	cna_os_vfree(buf, iocmd->bufsz);
	iocmd->status = BFA_STATUS_OK;
	return 0;
}

int
bnad_ioctl_vnic_reset_stats(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_gen_t *iocmd = (bfa_ioctl_gen_t *)cmd;
	unsigned long flags = 0;

	bnad_spin_lock(flags);
	bna_vnic_stats_clr(&bnad->bna);
	iocmd->status = BFA_STATUS_OK;
	bnad_spin_unlock(flags);

	return 0;
}

int
bnad_ioctl_ioc_get_pcifn_cfg(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_pcifn_cfg_t *iocmd = (bfa_ioctl_pcifn_cfg_t *)cmd;
	struct bnad_ioctl_comp_s fcomp;
	unsigned long flags = 0;
	bnad_init_completion(bnad, &fcomp);

	bnad_spin_lock(flags);
	iocmd->status = bfa_ablk_query(&bnad->bna.ablk,
			&iocmd->pcifn_cfg,
			bnad_cb_completion,
			&fcomp);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		return 0;
	}
	bnad_wait_for_completion(bnad, &fcomp, flags);
	bnad_spin_unlock(flags);

	iocmd->status = fcomp.comp_status;
	return 0;
}


int
bnad_ioctl_adapter_cfg_mode(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_adapter_cfg_mode_t *iocmd =
			(bfa_ioctl_adapter_cfg_mode_t *)cmd;
	struct bnad_ioctl_comp_s fcomp;
	unsigned long flags = 0;

	bnad_init_completion(bnad, &fcomp);
	bnad_spin_lock(flags);
	iocmd->status = bfa_ablk_adapter_config(&bnad->bna.ablk,
			iocmd->cfg.mode,
			iocmd->cfg.max_pf,
			iocmd->cfg.max_vf,
			bnad_cb_completion,
			&fcomp);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		return 0;
	}
	bnad_wait_for_completion(bnad, &fcomp, flags);
	bnad_spin_unlock(flags);

	iocmd->status = fcomp.comp_status;
	return 0;
}

int
bnad_ioctl_port_cfg_mode(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_port_cfg_mode_t *iocmd =
			(bfa_ioctl_port_cfg_mode_t *)cmd;
	struct bnad_ioctl_comp_s fcomp;
	unsigned long flags = 0;

	bnad_init_completion(bnad, &fcomp);
	bnad_spin_lock(flags);
	iocmd->status = bfa_ablk_port_config(&bnad->bna.ablk,
			iocmd->instance,
			iocmd->cfg.mode,
			iocmd->cfg.max_pf,
			iocmd->cfg.max_vf,
			bnad_cb_completion,
			&fcomp);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		return 0;
	}
	bnad_wait_for_completion(bnad, &fcomp, flags);
	bnad_spin_unlock(flags);

	iocmd->status = fcomp.comp_status;
	return 0;

}

int
bnad_ioctl_ioc_get_fwstats(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_ioc_fwstats_t *iocmd =
			(bfa_ioctl_ioc_fwstats_t *)cmd;
	void *buf;
	unsigned long flags = 0;

	buf = cna_os_vmalloc(iocmd->buf_size);
	if (!buf) {
		bfa_trc(bnad, 0x4444);
		return ENOMEM;
	}

	bnad_spin_lock(flags);
	iocmd->status =	bfa_ioc_fw_stats_get(&bnad->bna.ioceth.ioc, buf);
	bnad_spin_unlock(flags);
	if (cna_os_cp_to_user((void *)(ulong)iocmd->buf_ptr, buf,
			iocmd->buf_size, 0)) {
		cna_os_vfree(buf, iocmd->buf_size);
		return EIO;
	}
	cna_os_vfree(buf, iocmd->buf_size);
	return 0;
}

int
bnad_ioctl_ioc_reset_fwstats(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_gen_t *iocmd = (bfa_ioctl_gen_t *)cmd;
	unsigned long flags = 0;

	bnad_spin_lock(flags);
	iocmd->status =	bfa_ioc_fw_stats_clear(&bnad->bna.ioceth.ioc);
	bnad_spin_unlock(flags);

	return 0;
}

int
bnad_ioctl_ioc_fw_sig_inv(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_gen_t *iocmd = (bfa_ioctl_gen_t *)cmd;
	unsigned long flags = 0;

	bnad_spin_lock(flags);
	iocmd->status =	bfa_ioc_fwsig_invalidate(&bnad->bna.ioceth.ioc);
	bnad_spin_unlock(flags);

	return 0;
}

int
bnad_ioctl_tfru_read(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_tfru_t *iocmd = (bfa_ioctl_tfru_t *)cmd;
	struct bnad_ioctl_comp_s fcomp;
	unsigned long flags = 0;

	bnad_init_completion(bnad, &fcomp);
	bnad_spin_lock(flags);
	iocmd->status = bfa_tfru_read(&bnad->bna.fru,
					&iocmd->data,
					iocmd->len,
					iocmd->offset,
					bnad_cb_completion,
					&fcomp);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		return 0;
	}
	bnad_wait_for_completion(bnad, &fcomp, flags);
	bnad_spin_unlock(flags);

	iocmd->status = fcomp.comp_status;
	return 0;

}

int
bnad_ioctl_tfru_write(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_tfru_t *iocmd = (bfa_ioctl_tfru_t *)cmd;
	struct bnad_ioctl_comp_s fcomp;
	unsigned long flags = 0;

	bnad_init_completion(bnad, &fcomp);
	bnad_spin_lock(flags);
	iocmd->status = bfa_tfru_write(&bnad->bna.fru,
					&iocmd->data,
					iocmd->len,
					iocmd->offset,
					bnad_cb_completion,
					&fcomp);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		return 0;
	}
	bnad_wait_for_completion(bnad, &fcomp, flags);
	bnad_spin_unlock(flags);

	iocmd->status = fcomp.comp_status;
	return 0;

}

int
bnad_ioctl_fruvpd_read(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_fruvpd_t *iocmd = (bfa_ioctl_fruvpd_t *)cmd;
	struct bnad_ioctl_comp_s fcomp;
	unsigned long flags = 0;

	bnad_init_completion(bnad, &fcomp);
	bnad_spin_lock(flags);
	iocmd->status = bfa_fruvpd_read(&bnad->bna.fru,
					&iocmd->data,
					iocmd->len,
					iocmd->offset,
					bnad_cb_completion,
					&fcomp);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		return 0;
	}
	bnad_wait_for_completion(bnad, &fcomp, flags);
	bnad_spin_unlock(flags);

	iocmd->status = fcomp.comp_status;
	return 0;

}

int
bnad_ioctl_fruvpd_update(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_fruvpd_t *iocmd = (bfa_ioctl_fruvpd_t *)cmd;
	struct bnad_ioctl_comp_s fcomp;
	unsigned long flags = 0;

	bnad_init_completion(bnad, &fcomp);
	bnad_spin_lock(flags);
	iocmd->status = bfa_fruvpd_update(&bnad->bna.fru,
					&iocmd->data,
					iocmd->len,
					iocmd->offset,
					bnad_cb_completion,
					&fcomp,
					iocmd->trfr_cmpl);
	if (iocmd->status != BFA_STATUS_OK) {
		bnad_spin_unlock(flags);
		return 0;
	}
	bnad_wait_for_completion(bnad, &fcomp, flags);
	bnad_spin_unlock(flags);

	iocmd->status = fcomp.comp_status;
	return 0;

}

int
bnad_ioctl_fruvpd_get_max_size(struct bnad_s *bnad, void *cmd)
{
	bfa_ioctl_fruvpd_max_size_t *iocmd =
				(bfa_ioctl_fruvpd_max_size_t *)cmd;
	unsigned long flags = 0;

	bnad_spin_lock(flags);
	iocmd->status = bfa_fruvpd_get_max_size(&bnad->bna.fru,
					&iocmd->max_size);
	bnad_spin_unlock(flags);
	return 0;
}
