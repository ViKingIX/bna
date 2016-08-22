/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corp.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

/**
 * @file bnad_ioctl.c  QLogic BR-Series 10G PCIe Ethernet driver.
 */

#include "cna_os.h"
#include "bna.h"

#include "bnad_compat.h"
#include "bnad.h"
#include "bfad_ioctl_cna.h"
#include "bnad_ioctl.h"
#include "bnad_ioctl_common.h"
#include "bnad_trcmod.h"
#include "bnad_aen.h"

BNA_TRC_FILE(LDRV, IOCTL);

#define BNAD_GET(bna_id)			\
do {						\
	bnad = bnad_get_bnadev(bna_id);		\
	if (!bnad) {				\
		return -EINVAL;			\
	}					\
} while (0)

static int bnad_fopen(struct inode *inode, struct file *file);
static int bnad_frelease(struct inode *inode, struct file *file);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 11)
int bnad_fioctl(struct inode *inode, struct file *file, unsigned int cmd,
		unsigned long arg);
#else /* unlocked ioctl */
long bnad_fioctl(struct file *file, unsigned int cmd,
		unsigned long arg);
#endif

static unsigned int bnad_fpoll(struct file *file,
			       struct poll_table_struct *wait);
static int bnad_fasync(int fd, struct file *file, int on);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 11)
long
bnad_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return bnad_fioctl(file, cmd, arg);
}
#endif

struct file_operations bnad_fops = {
	.owner = THIS_MODULE,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 11)
	.ioctl = bnad_fioctl,
#else /* unlocked ioctl */
	.unlocked_ioctl = bnad_fioctl,
	.compat_ioctl = bnad_compat_ioctl,
#endif
	.open = bnad_fopen,
	.release = bnad_frelease,
	.poll = bnad_fpoll,
	.fasync = bnad_fasync
};

static int
bnad_fopen(struct inode *inode, struct file *file)
{
	struct bnad_s *bnad;
	int bnad_minor;

	bnad_minor = MINOR(inode->i_rdev);
	bnad = bnad_get_bnadev(bnad_minor);	/* bnad_minor is always 0 */
	if (bnad == NULL)
		return -EINVAL;

	if (!test_bit(BNAD_RF_IOCTL_ENABLED, &bnad->run_flags)) {
		printk(KERN_WARNING
		       "%s:%d: IOCTL path not yet ready for minor device=%d\n",
		       __func__, __LINE__, bnad_minor);
		return -EINVAL;
	}

	bnad_conf_lock();
	bnad->ref_count++;
	bnad_conf_unlock();

	return 0;
}

static int
bnad_frelease(struct inode *inode, struct file *file)
{
	struct bnad_s *bnad;
	int bnad_minor;

	bnad_minor = MINOR(inode->i_rdev);
	bnad = bnad_get_bnadev(bnad_minor);	/* bnad_minor is always 0 */
	if (bnad == NULL) {
		printk(KERN_WARNING
		       "%s:%d: Invalid minor device=%d for file=%p\n",
		       __func__, __LINE__, bnad_minor, file);
		return -EINVAL;
	}

	bnad_conf_lock();
	if (--bnad->ref_count) {
		bnad_conf_unlock();
		return 0;
	}
	bnad_conf_unlock();

	/*
	 * No more apps using this file handle
	 * Remove this file from the asynchronously notified file's
	 * fasync_helper() may sleep, so release the lock
	 */
	bnad_aen_fasync(-1, file, 0);

	return 0;
}

static unsigned int
bnad_fpoll(struct file *file, struct poll_table_struct *wait)
{
	return bnad_aen_poll(file, wait);
}

static int
bnad_fasync(int fd, struct file *file, int on)
{
	return bnad_aen_fasync(fd, file, on);
}

struct bnad_s *
bnad_get_bnadev(int bna_id)
{
	struct bnad_s *bnad;

	bnad_list_lock();
	list_for_each_entry(bnad, &bnad_list, list_entry) {
		if (bnad->ident.id == bna_id) {
			bnad_list_unlock();
			return bnad;
		}
	}
	bnad_list_unlock();
	return NULL;
}

void *
bnad_cp_from_user(int size, unsigned long arg)
{
	void *iocmd;

	iocmd = (uint8_t *)kmalloc(size, GFP_KERNEL);
	if (iocmd)
		memset(iocmd, 0, size);
	else
		return NULL;
	if (copy_from_user((uint8_t *)iocmd, (void __user *)arg, size)) {
		kfree(iocmd);
		return NULL;
	}
	return iocmd;
}

int
bnad_cp_to_user(int size, unsigned long arg, uint8_t *iocmd)
{
	if (copy_to_user((void __user *)arg, (uint8_t *)iocmd, size)) {
		kfree(iocmd);
		return EIO;
	}
	kfree(iocmd);
	return 0;
}

void
bnad_cb_completion(void *arg, bfa_status_t status)
{
	struct bnad_ioctl_comp_s *ioctl_comp =
			(struct bnad_ioctl_comp_s *)arg;

	ioctl_comp->comp_status = (uint32_t) status;
	complete(&ioctl_comp->comp);
}


int
bnad_ioctl_ioc_get_inst(unsigned long arg)
{
	struct bnad_s *bnad;
	bfa_ioctl_ioc_get_inst_t iocmd;

	cna_os_memset(&iocmd, 0, sizeof(iocmd));

	bnad_list_lock();
	list_for_each_entry(bnad, &bnad_list, list_entry) {
		CNA_ASSERT(iocmd.cnt < 256 && bnad->ident.id < 256);
		iocmd.bm[bnad->ident.id / 8] |=
				(1 << (bnad->ident.id % 8));
		iocmd.cnt++;
	}
	bnad_list_unlock();

	if (cna_os_cp_to_user
	    ((void *)arg, &iocmd, sizeof(iocmd), 0))
			return EIO;
	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 11)
int
bnad_fioctl(struct inode *inode, struct file *file, unsigned int cmd,
	    unsigned long arg)
#else /* unlocked ioctl */
long
bnad_fioctl(struct file *file, unsigned int cmd, unsigned long arg)
#endif
{
	void *iocmd = NULL;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 11)
	int rc = EINVAL;
#else /* unlocked ioctl */
	long rc = EINVAL;
#endif
	int size = _IOC_SIZE(cmd);
	struct bnad_s *bnad = NULL;

	if (!arg)
		return -rc;

	if (BFAD_IOCMD(cmd) == IOCMD_IOC_GET_VERSION) {
		rc = bnad_ioctl_ioc_get_version(arg);
		goto ext;
	}

	if (BFAD_IOCMD(cmd) == IOCMD_IOC_GET_INSTS) {
		rc = bnad_ioctl_ioc_get_inst(arg);
		goto ext;
	}

	if (BFAD_IOCMD(cmd) == IOCMD_AEN_GET) {
		rc = bnad_ioctl_aen_get(file, arg);
		goto ext;
	}
	if (BFAD_IOCMD(cmd) == IOCMD_AEN_UPDATE) {
		rc = bnad_ioctl_aen_update(file, arg);
		goto ext;
	}

	iocmd = bnad_cp_from_user(size, arg);
	if (!iocmd)
		return -EIO;

	switch (BFAD_IOCMD(cmd)) {
	case IOCMD_IOC_GET_INFO:
		BNAD_GET(((bfa_ioctl_ioc_info_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_ioc_get_info(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_IOC_GET_ATTR:
		BNAD_GET(((bfa_ioctl_ioc_attr_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_ioc_get_attr(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_IOC_ENABLE:
		BNAD_GET(((bfa_ioctl_gen_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_ioc_enable(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_IOC_DISABLE:
		BNAD_GET(((bfa_ioctl_gen_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_ioc_disable(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_IOC_GET_STATS:
		BNAD_GET(((bfa_ioctl_ioc_stats_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_ioc_get_stats(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_IOC_RESET_STATS:
		BNAD_GET(((bfa_ioctl_gen_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_ioc_reset_stats(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_IOC_SET_ADAPTER_NAME:
		BNAD_GET(((bfa_ioctl_ioc_set_adapter_name_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_ioc_set_adapter_name(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_IOC_SET_PORT_NAME:
		BNAD_GET(((bfa_ioctl_ioc_set_adapter_name_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_ioc_set_port_name(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_IOC_GET_FWSTATS:
		BNAD_GET(((bfa_ioctl_ioc_fwstats_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_ioc_get_fwstats(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_IOC_RESET_FWSTATS:
		BNAD_GET(((bfa_ioctl_gen_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_ioc_reset_fwstats(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_IOC_FW_SIG_INV:
		BNAD_GET(((bfa_ioctl_gen_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_ioc_fw_sig_inv(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_DEBUG_DRV_TRACE:
		BNAD_GET(((bfa_ioctl_debug_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_debug_drv_trace(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_DEBUG_FW_TRACE:
		BNAD_GET(((bfa_ioctl_debug_t *)iocmd)->bfad_num);
		rc = bnad_ioctl_debug_fw_trace(bnad, BFA_FALSE, iocmd);
		break;
	case IOCMD_DEBUG_FW_STATE:
		BNAD_GET(((bfa_ioctl_debug_t *)iocmd)->bfad_num);
		rc = bnad_ioctl_debug_fw_trace(bnad, BFA_TRUE, iocmd);
		break;
	case IOCMD_DEBUG_FW_STATE_CLR:
		BNAD_GET(((bfa_ioctl_gen_t *)iocmd)->bfad_num);
		rc = bnad_ioctl_debug_fw_trace_clear(bnad, iocmd);
		break;
	case IOCMD_DEBUG_FW_CORE:
		BNAD_GET(((bfa_ioctl_debug_t *)iocmd)->bfad_num);
		rc = bnad_ioctl_debug_fw_core(bnad, iocmd);
		break;
	case IOCMD_DEBUG_START_DTRC:
		BNAD_GET(((bfa_ioctl_gen_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_start_dtrc(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_DEBUG_STOP_DTRC:
		BNAD_GET(((bfa_ioctl_gen_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_stop_dtrc(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_DIAG_REGRD:
		BNAD_GET(((bfa_ioctl_diag_regread_t *)iocmd)->bfad_num);
		rc = bnad_ioctl_diag_regrd(bnad, iocmd);
		break;
	case IOCMD_DIAG_REGWR:
		BNAD_GET(((bfa_ioctl_diag_regwrite_t *)iocmd)->bfad_num);
		rc = bnad_ioctl_diag_regwr(bnad, iocmd);
		break;
	case IOCMD_DIAG_MEMRD:
		BNAD_GET(((bfa_ioctl_diag_kva_t *)iocmd)->bfad_num);
		rc = bnad_ioctl_diag_memrd(bnad, iocmd);
		break;
	case IOCMD_DIAG_TEMP:
		BNAD_GET(((bfa_ioctl_diag_get_temp_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_diag_temp(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_DIAG_SFP:
		BNAD_GET(((bfa_ioctl_sfp_show_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_diag_sfp(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_DIAG_FWPING:
		BNAD_GET(((bfa_ioctl_diag_fwping_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_diag_fwping(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_DIAG_LED:
		BNAD_GET(((bfa_ioctl_diag_led_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_diag_led(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_DIAG_BEACON_LPORT:
		BNAD_GET(((bfa_ioctl_diag_beacon_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_diag_beacon_lport(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_DIAG_MEMTEST:
		BNAD_GET(((bfa_ioctl_diag_memtest_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_diag_memtest(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_DIAG_LB_STAT:
		BNAD_GET(((bfa_ioctl_diag_lb_stat_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_diag_lb_stat(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_LOG_GET_LEVEL:
		BNAD_GET(((bfa_ioctl_log_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_log_get_level(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_LOG_SET_LEVEL:
		BNAD_GET(((bfa_ioctl_log_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_log_set_level(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_CEE_GET_ATTR:
		BNAD_GET(((bfa_ioctl_cee_attr_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_cee_attr(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_CEE_GET_STATS:
		BNAD_GET(((bfa_ioctl_cee_stats_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_cee_get_stats(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_CEE_RESET_STATS:
		BNAD_GET(((bfa_ioctl_gen_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_cee_reset_stats(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_PORT_ENABLE:
		BNAD_GET(((bfa_ioctl_gen_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_port_enable(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_PORT_DISABLE:
		BNAD_GET(((bfa_ioctl_gen_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_port_disable(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_PORT_GET_ATTR:
		BNAD_GET(((bfa_ioctl_port_attr_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_port_get_attr(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_PORT_GET_STATS:
		BNAD_GET(((bfa_ioctl_port_stats_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_port_get_stats(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_PORT_RESET_STATS:
		BNAD_GET(((bfa_ioctl_gen_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_port_reset_stats(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_ETHPORT_GET_ATTR:
		BNAD_GET(((bfa_ioctl_ethport_attr_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_ethport_get_attr(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_ETHPORT_GET_CFG:
		BNAD_GET(((bfa_ioctl_ethport_cfg_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_ethport_get_cfg(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_ETHPORT_GET_STATS:
		BNAD_GET(((bfa_ioctl_ethport_stats_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		spin_lock_irq(&bnad->bna_lock);
		rc = bnad_ioctl_ethport_get_stats(bnad, iocmd);
		spin_unlock_irq(&bnad->bna_lock);
		bnad_conf_unlock();
		break;
	case IOCMD_SFP_MEDIA:
		BNAD_GET(((bfa_ioctl_sfp_media_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_sfp_media(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_SFP_SPEED:
		BNAD_GET(((bfa_ioctl_sfp_speed_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_sfp_speed(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_ETHBOOT_CFG:
		BNAD_GET(((bfa_ioctl_ethboot_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_ethboot_cfg(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_ETHBOOT_QUERY:
		BNAD_GET(((bfa_ioctl_ethboot_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_ethboot_query(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_ISCSIBOOT_CFG:
		BNAD_GET(((bfa_ioctl_iscsiboot_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_iscsiboot_cfg(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_ISCSIBOOT_QUERY:
		BNAD_GET(((bfa_ioctl_iscsiboot_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_iscsiboot_query(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_DIAG_LL_LOOPBACK:
		BNAD_GET(((bfa_ioctl_diag_ll_loopback_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_diag_loopback(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_FLASH_GET_ATTR:
		BNAD_GET(((bfa_ioctl_flash_attr_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_flash_get_attr(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_FLASH_ERASE_PART:
		BNAD_GET(((bfa_ioctl_flash_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_flash_erase_part(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_FLASH_UPDATE_PART:
		BNAD_GET(((bfa_ioctl_flash_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_flash_update_part(bnad, iocmd, 0);
		bnad_conf_unlock();
		break;
	case IOCMD_FLASH_MFG_INFO:
		BNAD_GET(((bfa_ioctl_flash_mfg_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_flash_mfg_info(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_FLASH_READ_PART:
		BNAD_GET(((bfa_ioctl_flash_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_flash_read_part(bnad, iocmd, 0);
		bnad_conf_unlock();
		break;
	case IOCMD_FLASH_ENABLE_OPTROM:
	case IOCMD_FLASH_DISABLE_OPTROM:
		BNAD_GET(((bfa_ioctl_gen_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_flash_optrom(bnad, BFAD_IOCMD(cmd), iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_FLASH_GET_CFG_BOOTVER:
		BNAD_GET(((bfa_ioctl_flash_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_flash_get_cfg_bootver(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_PHY_GET_ATTR:
		BNAD_GET(((bfa_ioctl_phy_attr_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_phy_get_attr(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_PHY_GET_STATS:
		BNAD_GET(((bfa_ioctl_phy_stats_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_phy_get_stats(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_PHY_UPDATE_FW:
		BNAD_GET(((bfa_ioctl_phy_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_phy_update(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_PHY_READ_FW:
		BNAD_GET(((bfa_ioctl_phy_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_phy_read(bnad, iocmd);
		bnad_conf_unlock();
		break;

	case IOCMD_PCIFN_CREATE:
		BNAD_GET(((bfa_ioctl_pcifn_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_pcifn_create(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_PCIFN_DELETE:
		BNAD_GET(((bfa_ioctl_pcifn_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_pcifn_delete(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_PCIFN_BW:
		BNAD_GET(((bfa_ioctl_pcifn_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_pcifn_bw(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_PCIFN_ENABLE:
		BNAD_GET(((bfa_ioctl_pcifn_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_pcifn_enable(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_PCIFN_DISABLE:
		BNAD_GET(((bfa_ioctl_pcifn_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_pcifn_disable(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_PCIFN_PERS_CHANGE:
		BNAD_GET(((bfa_ioctl_pcifn_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_pcifn_pers_change(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_VNIC_QUERY:
		BNAD_GET(((bfa_ioctl_vnic_attr_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_vnic_query(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_VNIC_ENABLE:
		BNAD_GET(((bfa_ioctl_gen_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_vnic_enable(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_VNIC_DISABLE:
		BNAD_GET(((bfa_ioctl_gen_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_vnic_disable(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_VNIC_STATS:
		BNAD_GET(((bfa_ioctl_vnic_stats_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_vnic_stats(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_VNIC_RESET_STATS:
		BNAD_GET(((bfa_ioctl_gen_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_vnic_reset_stats(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_IOC_PCIFN_CFG:
		BNAD_GET(((bfa_ioctl_pcifn_cfg_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_ioc_get_pcifn_cfg(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_ADAPTER_CFG_MODE:
		BNAD_GET(((bfa_ioctl_adapter_cfg_mode_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_adapter_cfg_mode(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_PORT_CFG_MODE:
		BNAD_GET(((bfa_ioctl_port_cfg_mode_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_port_cfg_mode(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_PORT_NWPAR_ENABLE:
		BNAD_GET(((bfa_ioctl_nwpar_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_port_nwpar_enable(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_PORT_NWPAR_DISABLE:
		BNAD_GET(((bfa_ioctl_nwpar_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_port_nwpar_disable(bnad, iocmd);
		bnad_conf_unlock();
		break;

	/* TFRU */
	case IOCMD_TFRU_READ:
		BNAD_GET(((bfa_ioctl_tfru_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_tfru_read(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_TFRU_WRITE:
		BNAD_GET(((bfa_ioctl_tfru_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_tfru_write(bnad, iocmd);
		bnad_conf_unlock();
		break;

	/* FRU */
	case IOCMD_FRUVPD_READ:
		BNAD_GET(((bfa_ioctl_fruvpd_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_fruvpd_read(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_FRUVPD_UPDATE:
		BNAD_GET(((bfa_ioctl_fruvpd_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_fruvpd_update(bnad, iocmd);
		bnad_conf_unlock();
		break;
	case IOCMD_FRUVPD_GET_MAX_SIZE:
		BNAD_GET(((bfa_ioctl_fruvpd_max_size_t *)iocmd)->bfad_num);
		bnad_conf_lock();
		rc = bnad_ioctl_fruvpd_get_max_size(bnad, iocmd);
		bnad_conf_unlock();
		break;

	default:
		rc = EINVAL;
		break;
	}
	if (bnad_cp_to_user(size, arg, iocmd)) {
		if (bnad) {
			bfa_trc(bnad, BFAD_IOCMD(cmd));
			bfa_trc(bnad, size);
		}
		rc = EIO;
	}
ext:
	return -rc;
}

void
bnad_get_pci_attr(struct bnad_s *bnad, struct bfa_ioc_pci_attr_s *pci_attr)
{
	pci_attr->vendor_id = bnad->pcidev->vendor;
	pci_attr->device_id = bnad->pcidev->device;
	pci_attr->ssid = bnad->pcidev->subsystem_device;
	pci_attr->ssvid = bnad->pcidev->subsystem_vendor;
	pci_attr->pcifn = PCI_FUNC(bnad->pcidev->devfn);
}

void
bnad_hwpath_get(struct bnad_s *bnad, char *hwpath, char *adapter_hwpath)
{
	uint8_t i;

	strcpy(hwpath, pci_name(bnad->pcidev));
	strcpy(adapter_hwpath, pci_name(bnad->pcidev));
	for (i = 0; adapter_hwpath[i] != ':' && i < BFA_STRING_32; i++);
	for (; adapter_hwpath[++i] != ':' && i < BFA_STRING_32; );
	adapter_hwpath[i] = '\0';
}

void
bnad_get_ethport_attr(struct bnad_s *bnad, bfa_ethport_attr_t *attr)
{
	attr->mtu = bnad->netdev->mtu;

	if (netif_carrier_ok(bnad->netdev)) {
		attr->port_state = BFA_PORT_ST_LINKUP;
		attr->speed = BFA_PORT_SPEED_10GBPS;
	} else if (bna_ethport_is_disabled(&bnad->bna.ethport))
		attr->port_state = BFA_PORT_ST_DISABLED;
	else
		attr->port_state = BFA_PORT_ST_LINKDOWN;
}

void
bnad_get_ethport_cfg(struct bnad_s *bnad, bfa_ethport_cfg_t *pcfg)
{

#if defined(BNAD_LRO)
#ifndef __VMKLNX__
	if (bnad->netdev->features & NETIF_F_LRO)
#endif
		pcfg->lro = true;
#endif

#ifdef NETIF_F_TSO
	if (bnad->netdev->features & NETIF_F_TSO)
		pcfg->lso_v4 = true;
#endif

#ifdef NETIF_F_TSO6
	if (bnad->netdev->features & NETIF_F_TSO6)
		pcfg->lso_v6 = true;
#endif

	if (bnad->netdev->features & NETIF_F_IP_CSUM)
		pcfg->ipv4_csum = BFA_ETHPORT_TX_CSUM_ENABLED;

#ifdef NETIF_F_IPv6_CSUM
	if (bnad->netdev->features & NETIF_F_IPv6_CSUM)
		pcfg->ipv6_csum = BFA_ETHPORT_TX_CSUM_ENABLED;
#endif

	if (bnad->rx_csum) {
		pcfg->ipv4_csum =
			(pcfg->ipv4_csum == BFA_ETHPORT_TX_CSUM_ENABLED) ?
			BFA_ETHPORT_TXRX_CSUM_ENABLED:
			BFA_ETHPORT_RX_CSUM_ENABLED;

		pcfg->ipv6_csum =
			(pcfg->ipv6_csum == BFA_ETHPORT_TX_CSUM_ENABLED) ?
			BFA_ETHPORT_TXRX_CSUM_ENABLED:
			BFA_ETHPORT_RX_CSUM_ENABLED;
	}

	if (bnad->netdev->flags & IFF_PROMISC)
		pcfg->promisc = true;
}
