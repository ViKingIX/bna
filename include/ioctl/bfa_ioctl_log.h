/*
 * Copyright (c) 2006-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved.
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BFA_IOCTL_LOG_H__
#define __BFA_IOCTL_LOG_H__

#include <cs/bfa_log.h>

typedef struct {
	bfa_status_t    status;
	enum bfa_log_severity log_level;
	uint16_t        bfad_num;
	int16_t        	mod_id;
	uint8_t         rsvd[4];
} bfa_ioctl_log_t;

#endif
