/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corp.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

/*
 * This header file is used by BNAD DRV to extract info from BNA AEN
 */
#ifndef __BNAD_AEN_H__
#define __BNAD_AEN_H__

#include "aen/bfa_aen.h"

int	bnad_ioctl_aen_get(struct file *file, unsigned long arg);
int	bnad_ioctl_aen_update(struct file *file, unsigned long arg);
unsigned int bnad_aen_poll(struct file *file, struct poll_table_struct *wait);
int	bnad_aen_fasync(int fd, struct file *file, int on);
void	bnad_aen_cb_notify(void *bnad);

#endif  /* __BNAD_AEN_H__ */
