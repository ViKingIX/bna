/*
 * Copyright (c) 2006-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved.
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BFA_IOCTL_DEBUG_H__
#define __BFA_IOCTL_DEBUG_H__

#define BFA_DEBUG_FW_CORE_CHUNK_SZ	0x4000U /* 16K chunks for FW dump */

#define is_aligned(__val, __bytes) (((__val) & ((8 * (__bytes)) - 1)) == 0)

typedef struct {
	bfa_status_t	status;
	uint16_t	bfad_num;
	uint16_t	rsvd;
	uint32_t	bufsz;
	int		inst_no;
#ifdef _WINDOWS
#define WINDOWS_BUFFER_SIZE 0x4000
	char		buf_ptr[WINDOWS_BUFFER_SIZE];
#else
	uint64_t	buf_ptr;
#endif
	uint64_t	offset;
} bfa_ioctl_debug_t;

typedef struct {
	bfa_status_t	status;
	uint16_t	bfad_num;
	uint16_t	rsvd;
	uint32_t	bufsz;
	int		inst_no;
#define SOLARIS_BUFFER_SIZE 0xffe8 /* 65512 */
	uint8_t	buf_ptr[SOLARIS_BUFFER_SIZE];
	uint64_t	offset;
} bfa_ioctl_bufio_t;

typedef struct {
	bfa_status_t	status;
	uint16_t	bfad_num;
	uint16_t	rsvd;
	bfa_boolean_t	ctl;
	int		inst_no;
} bfa_ioctl_portlogctl_t;

#endif
