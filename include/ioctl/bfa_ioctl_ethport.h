/*
 * Copyright (c) 2006-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved.
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BFA_IOCTL_ETHPORT_H__
#define __BFA_IOCTL_ETHPORT_H__

#include <defs/bfa_defs_ethport.h>
#include <cna/pstats/phyport_defs.h>
#include <cna/pstats/ethport_defs.h>

typedef struct {
	bfa_status_t	status;
	uint16_t	bfad_num;
	uint16_t	rsvd;
	struct bfa_ethport_stats_s stats;
} bfa_ioctl_ethport_stats_t;

typedef struct {
	bfa_status_t	status;
	uint16_t	bfad_num;
	uint16_t	rsvd;
	struct bfa_ethport_attr_s attr;
} bfa_ioctl_ethport_attr_t;

typedef struct {
	bfa_status_t	status;
	uint16_t	bfad_num;
	uint16_t	rsvd;
	struct bfa_ethport_cfg_s cfg;
} bfa_ioctl_ethport_cfg_t;

typedef struct _bfa_ioctl_ethport_vmq_s {
	bfa_status_t	status;
	uint16_t	bfad_num;
	uint16_t	rsvd;
	struct bna_vmq_s vmq[BNA_MAX_VMQ];
} bfa_ioctl_ethport_vmq_t;

#endif
