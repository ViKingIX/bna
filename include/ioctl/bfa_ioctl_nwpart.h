/*
 * Copyright (c) 2006-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved.
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BFA_IOCTL_NWPAR_H__
#define __BFA_IOCTL_NWPAR_H__

#include <defs/bfa_defs.h>
#include <bfi/bfi.h>

typedef struct {
	bfa_status_t		status;
	uint16_t		bfad_num;
	uint8_t			rsvd[2];
} bfa_ioctl_nwpar_t;

enum {
	BFA_NWPAR_PF_ENABLE		= 1,
	BFA_NWPAR_PF_DISABLE		= 2,
	BFA_NWPAR_PF_PERS_TOGGLE	= 3
};

typedef struct {
	bfa_status_t	status;
	uint16_t	bfad_num;
	uint8_t		pfn;
	uint8_t		req;
} bfa_ioctl_nwpar_pf_t;
#endif
