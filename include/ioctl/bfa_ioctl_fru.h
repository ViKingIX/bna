/*
 * Copyright (c) 2006-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved.
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BFA_IOCTL_FRU_H__
#define __BFA_IOCTL_FRU_H__

#include <defs/bfa_defs.h>
#include <bfi/bfi.h>

#define BFA_TFRU_DATA_SIZE 64
#define BFA_MAX_FRUVPD_TRANSFER_SIZE 0x1000
#define BFA_FRUVPD_ALIGN 256
#define BFA_FRUVPD_TFR_CMPL 1
#define BFA_FRUVPD_TFR_PENDING 0
typedef struct {
	bfa_status_t	status;
	uint16_t	bfad_num;
	uint16_t	rsvd;
	uint32_t	offset;
	uint32_t	len;
	uint8_t		data[BFA_TFRU_DATA_SIZE];
} bfa_ioctl_tfru_t;

typedef struct {
	bfa_status_t	status;
	uint16_t	bfad_num;
	uint16_t	rsvd;
	uint32_t	offset;
	uint32_t	len;
	uint8_t		data[BFA_MAX_FRUVPD_TRANSFER_SIZE];
	uint8_t		trfr_cmpl;
	uint8_t		rsvd_1[3];
} bfa_ioctl_fruvpd_t;

typedef struct {
	bfa_status_t	status;
	uint16_t	bfad_num;
	uint16_t	rsvd;
	uint32_t	max_size;
} bfa_ioctl_fruvpd_max_size_t;
#endif
