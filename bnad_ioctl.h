/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corp.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BNAD_IOCTL_H__
#define __BNAD_IOCTL_H__

#include <linux/ioctl.h>
#include <ioctl/bfa_ioctl_ethport.h>
#include <ioctl/bfa_ioctl_vnic.h>
#include <ioctl/bfa_ioctl_pcifn.h>

/* ETHPORT */
enum {
	BNAD_ETHPORT_GET_STATS = BFAD_CMD_MAX+1,
	BNAD_ETHPORT_GET_ATTR,
	BNAD_ETHPORT_GET_CFG
};

/* ETHPORT */
#define IOCMD_ETHPORT_GET_STATS		\
	_IOWR(BFA_MAGIC, BNAD_ETHPORT_GET_STATS, bfa_ioctl_ethport_stats_t)
#define IOCMD_ETHPORT_GET_ATTR		\
	_IOWR(BFA_MAGIC, BNAD_ETHPORT_GET_ATTR, bfa_ioctl_ethport_attr_t)
#define IOCMD_ETHPORT_GET_CFG		\
	_IOWR(BFA_MAGIC, BNAD_ETHPORT_GET_CFG, bfa_ioctl_ethport_cfg_t)

#endif /* __BNAD_IOCTL_H__ */
