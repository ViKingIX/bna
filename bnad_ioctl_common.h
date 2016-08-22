/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BNAD_IOCTL_COMMON_H__
#define __BNAD_IOCTL_COMMON_H__

void bnad_get_pci_attr(struct bnad_s *bnad,
			struct bfa_ioc_pci_attr_s *pci_attr);
void bnad_cb_completion(void *arg, bfa_status_t status);
void bnad_hwpath_get(struct bnad_s *bnad, char *hwpath, char *adapter_hwpath);
void bnad_get_ethport_attr(struct bnad_s *bnad, bfa_ethport_attr_t *attr);
void bnad_get_ethport_cfg(struct bnad_s *bnad, bfa_ethport_cfg_t *pcfg);

int bnad_ioctl_ioc_get_version(unsigned long arg);
int bnad_ioctl_ioc_get_inst(unsigned long arg);
int bnad_ioctl_ioc_get_info(struct bnad_s *bnad, void *iocmd);
int bnad_ioctl_ioc_get_attr(struct bnad_s *bnad, void *iocmd);
int bnad_ioctl_ioc_enable(struct bnad_s *bnad, void *iocmd);
int bnad_ioctl_ioc_disable(struct bnad_s *bnad, void *iocmd);
int bnad_ioctl_ioc_get_stats(struct bnad_s *bnad, void *iocmd);
int bnad_ioctl_ioc_reset_stats(struct bnad_s *bnad, void *iocmd);
int bnad_ioctl_ioc_set_adapter_name(struct bnad_s *bnad, void *iocmd);
int bnad_ioctl_ioc_set_port_name(struct bnad_s *bnad, void *iocmd);
int bnad_ioctl_ioc_get_pcifn_cfg(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_ioc_get_fwstats(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_ioc_reset_fwstats(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_ioc_fw_sig_inv(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_debug_drv_trace(struct bnad_s *bnad, void *iocmd);
int bnad_ioctl_debug_fw_trace(struct bnad_s *bnad, bfa_boolean_t saved,
					void *iocmd);
int bnad_ioctl_debug_fw_trace_clear(struct bnad_s *bnad, void *iocmd);
int bnad_ioctl_debug_fw_core(struct bnad_s *bnad, void *iocmd);
int bnad_ioctl_start_dtrc(struct bnad_s *bnad, void *iocmd);
int bnad_ioctl_stop_dtrc(struct bnad_s *bnad, void *iocmd);
int bnad_ioctl_diag_regrd(struct bnad_s *bnad, void *iocmd);
int bnad_ioctl_diag_regwr(struct bnad_s *bnad, void *iocmd);
int bnad_ioctl_diag_memrd(struct bnad_s *bnad, void *iocmd);
int bnad_ioctl_diag_lb_stat(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_diag_memtest(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_diag_beacon_lport(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_diag_led(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_diag_fwping(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_diag_temp(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_diag_sfp(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_log_get_level(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_log_set_level(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_cee_reset_stats(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_cee_get_stats(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_cee_attr(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_port_enable(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_port_disable(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_port_get_attr(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_port_get_cfg(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_port_get_stats(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_port_reset_stats(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_ethport_get_attr(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_ethport_get_cfg(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_ethport_get_stats(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_sfp_media(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_sfp_speed(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_ethboot_cfg(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_ethboot_query(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_iscsiboot_cfg(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_iscsiboot_query(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_diag_loopback(struct bnad_s *bnad, void *arg);
int bnad_ioctl_flash_get_attr(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_flash_read_part(struct bnad_s *bnad, void *cmd, uint32_t offset);
int bnad_ioctl_flash_erase_part(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_flash_update_part(struct bnad_s *bnad, void *cmd,
				uint32_t offset);
int bnad_ioctl_flash_get_cfg_bootver(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_flash_optrom(struct bnad_s *bnad, unsigned int cmd, void *pcmd);
int bnad_ioctl_flash_mfg_info(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_phy_get_attr(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_phy_get_stats(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_phy_update(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_phy_read(struct bnad_s *bnad, void *cmd);

int bnad_ioctl_adapter_cfg_mode(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_port_cfg_mode(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_port_nwpar_enable(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_port_nwpar_disable(struct bnad_s *bnad, void *cmd);

int bnad_ioctl_pcifn_create(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_pcifn_delete(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_pcifn_bw(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_pcifn_enable(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_pcifn_disable(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_pcifn_pers_change(struct bnad_s *bnad, void *cmd);

// vnic apis
int bnad_ioctl_vnic_query(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_vnic_enable(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_vnic_disable(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_vnic_stats(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_vnic_reset_stats(struct bnad_s *bnad, void *cmd);
// FRU and TFRU
int bnad_ioctl_tfru_read(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_tfru_write(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_fruvpd_read(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_fruvpd_update(struct bnad_s *bnad, void *cmd);
int bnad_ioctl_fruvpd_get_max_size(struct bnad_s *bnad, void *cmd);

#endif /* __BNAD_IOCTL_COMMON_H__ */
