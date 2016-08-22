/*
 * Copyright (c) 2006-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved.
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BFA_IOCTL_PCIFN_H__
#define __BFA_IOCTL_PCIFN_H__

#include <defs/bfa_defs.h>
#include <bfi/bfi.h>

typedef struct {
	bfa_status_t	status;
	uint16_t	bfad_num;
	uint16_t	rsvd;
	bfa_ablk_cfg_t	pcifn_cfg;
} bfa_ioctl_pcifn_cfg_t;

typedef struct {
	bfa_status_t		status;
	uint16_t		bfad_num;
	uint16_t		pcifn_id;
	uint16_t		bw_min;
	uint16_t		bw_max;
	uint8_t			port;
	bfi_pcifn_class_t	pcifn_class;
	uint8_t			rsvd[1];
} bfa_ioctl_pcifn_t;

#endif
