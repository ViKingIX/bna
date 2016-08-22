/*
 * Copyright (c) 2010-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved.
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BFA_IOCTL_PHY_H__
#define __BFA_IOCTL_PHY_H__

#include <defs/bfa_defs_phy.h>
#include <defs/bfa_defs.h>

typedef struct {
	bfa_status_t    status;
	uint16_t	bfad_num;
	uint16_t	instance;
	bfa_phy_attr_t attr;
} bfa_ioctl_phy_attr_t;

/*
 * For PHY_UPDATE, and PHY_READ IOCTLs
 */
typedef struct {
	bfa_status_t    status;
	uint16_t	bfad_num;
	uint16_t	instance;
	uint64_t	bufsz;
#ifdef _WINDOWS
	uint32_t	buf_ptr[1];
#else
	uint64_t	buf_ptr;
#endif
} bfa_ioctl_phy_t;

typedef struct {
	bfa_status_t	status;
	uint16_t	bfad_num;
	uint16_t	instance;
	bfa_phy_stats_t stats;
} bfa_ioctl_phy_stats_t;

#endif
