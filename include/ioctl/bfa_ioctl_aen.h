/*
 * QLogic Fibre Channel HBA Driver
 * Copyright (c) 2006-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved.
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BFA_IOCTL_AEN_H__
#define __BFA_IOCTL_AEN_H__

#include "defs/bfa_defs_aen.h"

/**
 * This struct is between User space(BFAL) and Kernel space
 * @Param[out]      status      Return status from ioctl call
 * @Param[in]       bfad_num    BFA device instance number
 * @Param[in,out]   aen_list    User allocated aen_list empty array.
 *                              aen_list array with event data, if any.
 * @Param[in,out]   aen_count   aen_list array max size.
 *                              Number of events copied to aen_list.
 *                              (return value == passed value) ==>
 *					  call again for more events.
 *                              (return value < passed value) ==>
 *					 no more events present.
 */
typedef struct bfa_ioctl_aen_s {
	int32_t		bfad_num;
	int32_t		aen_count;
	bfa_aen_app_t	app_id;
	bfa_status_t    status;
	bfa_boolean_t	block;
	uint32_t	rsvd;
	struct bfa_aen_entry_s aen_list[1];
} bfa_ioctl_aen_t;

#endif /* __BFA_IOCTL_AEN_H__ */
