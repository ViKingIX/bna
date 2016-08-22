/*
 * Copyright (c) 2006-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved.
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BFA_IOCTL_GENERIC_H__
#define __BFA_IOCTL_GENERIC_H__

typedef struct {
	bfa_status_t	status;
	uint16_t	bfad_num;
	uint16_t	rsvd;
} bfa_ioctl_gen_t;

typedef struct {
	uint16_t		bfad_num;
	int			opcode;
	char			*in_buf;
	char			*out_buf;
	uint32_t		status;
}bfa_tgt_mode_ioctl_t;

typedef struct {
	bfa_status_t	status;
	uint16_t	bfad_num;
	uint16_t	rsvd;
	uint32_t	cfg_val;
} bfa_ioctl_cfgval_t;

#endif
