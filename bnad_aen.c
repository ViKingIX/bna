/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corp.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#include <linux/netdevice.h>
#include <linux/poll.h>		/* poll_wait() */
#include <linux/spinlock.h>	/* spin_(un)lock_irq* */
#include <asm/uaccess.h>	/* copy_from_user(), copy_to_user() */
#include <linux/fs.h>		/* struct file_operations */
#include <linux/string.h>	/* memset() */

#include "bna.h"
#include "bnad_compat.h"
#include "bnad.h"
#include "bnad_aen.h"
#include "bfad_ioctl_cna.h"
#include "bnad_trcmod.h"

#ifndef __VMKERNEL_MODULE__
#include <linux/rtnetlink.h>
#endif

BNA_TRC_FILE(LDRV, AEN);

DECLARE_WAIT_QUEUE_HEAD(bnad_aen_readq);
struct fasync_struct *bnad_aen_asyncq;
int fasync_enabled;

/**
 * @dg bnad2aen_api Base Linux ==> AEN module interface to support file
 * 			operations
 *     aen2bnad_api AEN module ==> Base Linux interface
 * @{
 */

/**
 * @brief IOCTL function to copy AEN module data onto user space.
 * @param[in,out] arg   - Ptr to user bfa_ioctl_aen_t where data will be
 * 			copied from/to
 * @retval 0            Success
 * @retval -EINVAL      Invalid app_id passed in user command
 * @retval -EFAULT      Error copying data onto user buffer
 */
int
bnad_ioctl_aen_get(struct file *file, unsigned long arg)
{
	bfa_ioctl_aen_t ioc_aen;
	unsigned int req_cnt, cnt = 0;
	unsigned long flags;
	int size = sizeof(bfa_ioctl_aen_t);
	int rc = 0;
	bfa_status_t status = BFA_STATUS_OK;
	struct bnad_s *bnad;
	bfa_aen_entry_t __user *usr_aen_list =
		(bfa_aen_entry_t __user *) (((bfa_ioctl_aen_t __user *) arg)->
					    aen_list);

	if (copy_from_user((void *)&ioc_aen, (void __user *)arg, size))
		return -EFAULT;
	bnad = bnad_get_bnadev(ioc_aen.bfad_num);
	CNA_ASSERT(bnad);

	/* Get events from AEN module */
	req_cnt = ioc_aen.aen_count;
	while (req_cnt--) {
		spin_lock_irqsave(&bnad->bna_lock, flags);
		status = bfa_aen_fetch(bnad->aen.aen, ioc_aen.aen_list, 1,
				       ioc_aen.app_id, &rc);
		spin_unlock_irqrestore(&bnad->bna_lock, flags);
		if ((status != BFA_STATUS_OK) || !rc)
			break;
		if (copy_to_user
		    ((void __user *)&usr_aen_list[cnt],
		     (void *)ioc_aen.aen_list, sizeof(bfa_aen_entry_t)))
			return -EFAULT;
		cnt++;
	}

	ioc_aen.aen_count = cnt;

	if (status != BFA_STATUS_OK)
		ioc_aen.status = status;
	else
		ioc_aen.status = (cnt > 0) ? BFA_STATUS_OK : BFA_STATUS_FAILED;

	bfa_trc(bnad, ioc_aen.aen_count);
	if (copy_to_user
	    ((void __user *)arg, (void *)&ioc_aen,
	     size - sizeof(bfa_aen_entry_t)))
		return -EFAULT;

	return 0;
}

int
bnad_ioctl_aen_update(struct file *file, unsigned long arg)
{
	bfa_ioctl_aen_t ioc_aen;
	struct bnad_s *bnad;
	unsigned long flags;

	if (copy_from_user
	    ((void *)&ioc_aen, (void __user *)arg, sizeof(bfa_ioctl_aen_t)))
		return -EFAULT;

	/* Update all instances to have same file map in Linux */
	bnad_list_lock();
	list_for_each_entry(bnad, &bnad_list, list_entry) {
		spin_lock_irqsave(&bnad->bna_lock, flags);

		/* Update file handle corresponding to this app */
		bnad->aen.file_map[ioc_aen.app_id] = file;
		spin_unlock_irqrestore(&bnad->bna_lock, flags);
	}
	bnad_list_unlock();

	return 0;
}

static bfa_aen_app_t
bnad_aen_get_appid(struct file *file)
{
	int i;
	bfa_aen_app_t app_id = BFA_AEN_MAX_APP;
	struct bnad_s *bnad;
	unsigned long flags;

	/*
	 * Just find the app_id in any bnad instance
	 * Assuming that all other instances will have same file map in Linux
	 */
	bnad_list_lock();
	list_for_each_entry(bnad, &bnad_list, list_entry) {
		spin_lock_irqsave(&bnad->bna_lock, flags);
		for (i = 0; i < BFA_AEN_MAX_APP; i++) {
			if (bnad->aen.file_map[i] == file) {
				app_id = i;
				break;
			}
		}
		spin_unlock_irqrestore(&bnad->bna_lock, flags);
	}
	bnad_list_unlock();

	return app_id;
}

/**
 * @brief Poll function for AEN module file operation through Base Linux driver.
 * @note  To support poll/select interface for non-blocking read/write file
 * 		operations
 * @param[in] file  - File ptr
 * @param[in] wait  - Ptr to fd for which data availability will be checked
 * @return Mask to inform the current data availability status
 */
unsigned int
bnad_aen_poll(struct file *file, struct poll_table_struct *wait)
{
	struct bnad_s *bnad = NULL;
	unsigned int mask = 0;
	unsigned long flags;
	bfa_aen_app_t app_id;

	app_id = bnad_aen_get_appid(file);
	if (app_id == BFA_AEN_MAX_APP)
		return POLLNVAL;

	poll_wait(file, &bnad_aen_readq, wait);

	/* See if there are any events posted */
	bnad_list_lock();
	list_for_each_entry(bnad, &bnad_list, list_entry) {
		spin_lock_irqsave(&bnad->bna_lock, flags);
		CNA_ASSERT(bnad->aen.file_map[app_id] == file);
		if (bfa_aen_fetch_count(bnad->aen.aen, app_id) > 0) {
			mask |= POLLIN | POLLRDNORM;	/* Readable */
			spin_unlock_irqrestore(&bnad->bna_lock, flags);
			break;
		}
		spin_unlock_irqrestore(&bnad->bna_lock, flags);
	}
	bnad_list_unlock();

	return mask;
}

int
bnad_aen_fasync(int fd, struct file *file, int on)
{
	fasync_enabled = on;
	return fasync_helper(fd, file, on, &bnad_aen_asyncq);
}

/**
 * @brief BNAD callback function provided for AEN to inform arrival of
 *  a new event
 * @param[in] bnad  - Linux driver IOC instance ptr
 * @return          None
 */
void
bnad_aen_cb_notify(void *bnad)
{
	wake_up(&bnad_aen_readq);

	/*
	 * Global data fasync_enabled is not protected. Presently no other
	 *  process is using (changing) it.
	 */
	if (fasync_enabled)
		kill_fasync(&bnad_aen_asyncq, SIGIO, POLL_IN);
}

/**
 * @}
 */
