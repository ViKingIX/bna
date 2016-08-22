/*
 * Copyright (c) 2010-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BFI_FRU_H__
#define __BFI_FRU_H__

#include <bfi/bfi.h>

#pragma pack(1)

typedef enum {
	BFI_FRUVPD_H2I_WRITE_REQ = 1,
	BFI_FRUVPD_H2I_READ_REQ = 2,
	BFI_TFRU_H2I_WRITE_REQ = 3,
	BFI_TFRU_H2I_READ_REQ = 4,
} bfi_fru_h2i_msgs_t;

typedef enum {
	BFI_FRUVPD_I2H_WRITE_RSP = BFA_I2HM(1),
	BFI_FRUVPD_I2H_READ_RSP = BFA_I2HM(2),
	BFI_TFRU_I2H_WRITE_RSP = BFA_I2HM(3),
	BFI_TFRU_I2H_READ_RSP = BFA_I2HM(4),
} bfi_fru_i2h_msgs_t;

/**
 * FRU write request
 */
typedef struct {
	struct bfi_mhdr_s	mh;		/*!< Common msg header */
	uint8_t			last;
	uint8_t			rsv[3];
	uint8_t			trfr_cmpl;
	uint8_t			rsv_1[3];
	uint32_t		offset;
	uint32_t		length;
	struct bfi_alen_s	alen;
} bfi_fru_write_req_t;

/**
 * FRU read request
 */
typedef struct {
	struct bfi_mhdr_s	mh;		/*!< Common msg header */
	uint32_t		offset;
	uint32_t		length;
	struct bfi_alen_s	alen;
} bfi_fru_read_req_t;

/**
 * FRU response
 */
typedef struct {
	struct bfi_mhdr_s	mh;		/*!< Common msg header */
	uint32_t		status;
	uint32_t		length;
} bfi_fru_rsp_t;

#pragma pack()

#endif /* __BFI_FRU_H__ */
