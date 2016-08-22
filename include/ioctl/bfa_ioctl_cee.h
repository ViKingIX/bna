/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BFA_IOCTL_CEE_H__
#define __BFA_IOCTL_CEE_H__

typedef struct {
	bfa_status_t	status;
	uint16_t		bfad_num;
	uint16_t		rsvd;
	uint32_t		buf_size;
	uint32_t		rsvd1;
#ifdef _WINDOWS
	uint32_t		buf_ptr[1];
#else
	uint64_t		buf_ptr;
#endif
} bfa_ioctl_cee_attr_t;

typedef struct {
	bfa_status_t	status;
	uint16_t		bfad_num;
	uint16_t		rsvd;
	uint32_t		buf_size;
	uint32_t		rsvd1;
#ifdef _WINDOWS
	uint32_t		buf_ptr[1];
#else
	uint64_t		buf_ptr;
#endif
} bfa_ioctl_cee_stats_t;

typedef struct {
	bfa_status_t	status;
	uint16_t		bfad_num;
	uint16_t		rsvd;
	uint32_t		buf_size;
	uint32_t		rsvd1;
	uint32_t		buf_ptr[1];
} bfa_ioctl_cee_bufio_t;

#endif
