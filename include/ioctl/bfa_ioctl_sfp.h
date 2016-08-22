/*
 * Copyright (c) 2009-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved.
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BFA_IOCTL_SFP_H__
#define __BFA_IOCTL_SFP_H__


typedef struct {
	bfa_status_t	status;
	uint16_t	bfad_num;
	uint16_t	rsvd;
	sfp_mem_t	sfp;
} bfa_ioctl_sfp_show_t;

typedef struct {
	bfa_status_t	status;
	uint16_t	bfad_num;
	uint16_t	rsvd;
	bfa_defs_sfp_media_t	media;
} bfa_ioctl_sfp_media_t;

typedef struct {
	bfa_status_t	status;
	uint16_t	bfad_num;
	uint16_t	rsvd;
	bfa_port_speed_t	speed;
} bfa_ioctl_sfp_speed_t;

#endif /* __BFA_IOCTL_SFP_H__ */
