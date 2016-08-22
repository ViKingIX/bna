/*
 * Copyright (c) 2010-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */
#ifndef __BFI_PHY_H__
#define __BFI_PHY_H__

#include <bfi/bfi.h>

#pragma pack(1)

typedef enum {
	BFI_PHY_H2I_QUERY_REQ = 1,
	BFI_PHY_H2I_STATS_REQ = 2,
	BFI_PHY_H2I_WRITE_REQ = 3,
	BFI_PHY_H2I_READ_REQ = 4,
} bfi_phy_h2i_msgs_t;

typedef enum {
	BFI_PHY_I2H_QUERY_RSP = BFA_I2HM(1),
	BFI_PHY_I2H_STATS_RSP = BFA_I2HM(2),
	BFI_PHY_I2H_WRITE_RSP = BFA_I2HM(3),
	BFI_PHY_I2H_READ_RSP = BFA_I2HM(4),
} bfi_phy_i2h_msgs_t;

/**
 * External PHY query request
 */
typedef struct {
	struct bfi_mhdr_s	mh;		/*!< Common msg header */
	uint8_t			instance;
	uint8_t			rsv[3];
	struct bfi_alen_s	alen;
} bfi_phy_query_req_t;

/**
 * External PHY stats request
 */
typedef struct {
	struct bfi_mhdr_s	mh;		/*!< Common msg header */
	uint8_t			instance;
	uint8_t			rsv[3];
	struct bfi_alen_s	alen;
} bfi_phy_stats_req_t;

/**
 * External PHY write request
 */
typedef struct {
	struct bfi_mhdr_s	mh;		/*!< Common msg header */
	uint8_t			instance;
	uint8_t			last;
	uint8_t			rsv[2];
	uint32_t		offset;
	uint32_t		length;
	struct bfi_alen_s	alen;
} bfi_phy_write_req_t;

/**
 * External PHY read request
 */
typedef struct {
	struct bfi_mhdr_s	mh;		/*!< Common msg header */
	uint8_t			instance;
	uint8_t			rsv[3];
	uint32_t		offset;
	uint32_t		length;
	struct bfi_alen_s	alen;
} bfi_phy_read_req_t;

/**
 * External PHY query response
 */
typedef struct {
	struct bfi_mhdr_s	mh;		/*!< Common msg header */
	uint32_t		status;
} bfi_phy_query_rsp_t;

/**
 * External PHY stats response
 */
typedef struct {
	struct bfi_mhdr_s	mh;		/*!< Common msg header */
	uint32_t		status;
} bfi_phy_stats_rsp_t;

/**
 * External PHY read response
 */
typedef struct {
	struct bfi_mhdr_s	mh;		/*!< Common msg header */
	uint32_t		status;
	uint32_t		length;
} bfi_phy_read_rsp_t;

/**
 * External PHY write response
 */
typedef struct {
	struct bfi_mhdr_s	mh;		/*!< Common msg header */
	uint32_t		status;
	uint32_t		length;
} bfi_phy_write_rsp_t;


#pragma pack()

#endif /* __BFI_PHY_H__ */
