/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */
#ifndef __BFI_FLASH_H__
#define __BFI_FLASH_H__

#include <bfi/bfi.h>

#pragma pack(1)

typedef enum {
	BFI_FLASH_H2I_QUERY_REQ = 1,
	BFI_FLASH_H2I_ERASE_REQ = 2,
	BFI_FLASH_H2I_WRITE_REQ = 3,
	BFI_FLASH_H2I_READ_REQ = 4,
	BFI_FLASH_H2I_BOOT_VER_REQ = 5,
} bfi_flash_h2i_msgs_t;

typedef enum {
	BFI_FLASH_I2H_QUERY_RSP = BFA_I2HM(1),
	BFI_FLASH_I2H_ERASE_RSP = BFA_I2HM(2),
	BFI_FLASH_I2H_WRITE_RSP = BFA_I2HM(3),
	BFI_FLASH_I2H_READ_RSP = BFA_I2HM(4),
	BFI_FLASH_I2H_BOOT_VER_RSP = BFA_I2HM(5),
	BFI_FLASH_I2H_EVENT = BFA_I2HM(127),
} bfi_flash_i2h_msgs_t;

/**
 * Flash query request
 */
typedef struct {
	struct bfi_mhdr_s	mh;		/*!< Common msg header */
	struct bfi_alen_s	alen;
} bfi_flash_query_req_t;

/**
 * Flash erase request
 */
typedef struct {
	struct bfi_mhdr_s	mh;		/*!< Common msg header */
	uint32_t		type;		/*!< partition type */
	uint8_t			instance;	/*!< partition instance */
	uint8_t			rsv[3];
} bfi_flash_erase_req_t;

/**
 * Flash write request
 */
typedef struct {
	struct bfi_mhdr_s	mh;		/*!< Common msg header */
	struct bfi_alen_s	alen;
	uint32_t		type;		/*!< partition type */
	uint8_t			instance;	/*!< partition instance */
	uint8_t			last;
	uint8_t			rsv[2];
	uint32_t		offset;
	uint32_t		length;
} bfi_flash_write_req_t;

/**
 * Flash read request
 */
typedef struct {
	struct bfi_mhdr_s	mh;		/*!< Common msg header */
	uint32_t		type;		/*!< partition type */
	uint8_t			instance;	/*!< partition instance */
	uint8_t			rsv[3];
	uint32_t		offset;
	uint32_t		length;
	struct bfi_alen_s	alen;
} bfi_flash_read_req_t;

/**
 * Flash boot version request
 */
typedef struct {
	struct bfi_mhdr_s	mh;		/*!< Common msg header */
	struct bfi_alen_s	alen;
} bfi_flash_boot_ver_req_t;

/**
 * Flash query response
 */
typedef struct {
	struct bfi_mhdr_s	mh;		/*!< Common msg header */
	uint32_t		status;
} bfi_flash_query_rsp_t;

/**
 * Flash read response
 */
typedef struct {
	struct bfi_mhdr_s	mh;		/*!< Common msg header */
	uint32_t		type;		/*!< partition type */
	uint8_t			instance;	/*!< partition instance */
	uint8_t			rsv[3];
	uint32_t		status;
	uint32_t		length;
} bfi_flash_read_rsp_t;

/**
 * Flash write response
 */
typedef struct {
	struct bfi_mhdr_s	mh;		/*!< Common msg header */
	uint32_t		type;		/*!< partition type */
	uint8_t			instance;	/*!< partition instance */
	uint8_t			rsv[3];
	uint32_t		status;
	uint32_t		length;
} bfi_flash_write_rsp_t;

/**
 * Flash erase response
 */
typedef struct {
	struct bfi_mhdr_s	mh;		/*!< Common msg header */
	uint32_t		type;		/*!< partition type */
	uint8_t			instance;	/*!< partition instance */
	uint8_t			rsv[3];
	uint32_t		status;
} bfi_flash_erase_rsp_t;

/**
 * Flash boot version response
 */
typedef struct {
	struct bfi_mhdr_s	mh;		/*!< Common msg header */
	uint32_t		status;
} bfi_flash_boot_ver_rsp_t;

/**
 * Flash event notification
 */
typedef struct {
	struct bfi_mhdr_s	mh;		/*!< Common msg header */
	bfa_status_t		status;
	uint32_t		param;
} bfi_flash_event_t;

#pragma pack()

#endif /* __BFI_FLASH_H__ */
