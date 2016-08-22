/*
 * Copyright (c) 2006-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved.
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BFA_IOCTL_VNIC_H__
#define __BFA_IOCTL_VNIC_H__

#include <defs/bfa_defs_vnic.h>

typedef struct {
	bfa_status_t	status;
	uint16_t	bfad_num;
	uint16_t	pcifn_id;
	uint64_t	bufsz;
#ifdef _WINDOWS
	uint64_t	buf_ptr[1];
#else
	uint64_t	buf_ptr;
#endif
} bfa_ioctl_vnic_attr_t;

typedef struct {
	bfa_status_t	status;
	uint16_t	bfad_num;
	uint16_t	pcifn_id;
	uint64_t	bufsz;
#define SOLARIS_VNIC_BUFFER_SIZE 0xffe8 /* 0xffe8 65512 */
	uint8_t		buf_ptr[SOLARIS_VNIC_BUFFER_SIZE];
	uint64_t	offset;
} bfa_ioctl_vnic_bufio_t;

typedef struct {
	bfa_status_t	status;
	uint16_t	bfad_num;
	uint16_t	pcifn_id;
	uint64_t	bufsz;
#if defined(_WINDOWS)
	uint64_t	buf_ptr[1];
#else
	uint64_t	buf_ptr;
#endif
} bfa_ioctl_vnic_stats_t;

#endif
