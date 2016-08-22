/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */
#ifndef __BFA_DEFS_PHY_H__
#define __BFA_DEFS_PHY_H__

#include <bfa_os_inc.h>

/**
 * FHY status
 */
typedef enum {
	BFA_PHY_STATUS_GOOD 		= 0, /*!< phy is good */
	BFA_PHY_STATUS_NOT_PRESENT 	= 1, /*!< phy does not exist */
	BFA_PHY_STATUS_BAD 		= 2, /*!< phy is bad */
} bfa_phy_status_t;

/**
 * phy attributes for phy query
 */
typedef struct {
	uint32_t	status;		/*!< phy present/absent status */
	uint32_t	length;		/*!< firmware length */
	uint32_t	fw_ver;		/*!< firmware version */
	uint32_t	an_status;	/*!< AN status */
	uint32_t	pma_pmd_status;	/*!< PMA/PMD link status */
	uint32_t	pma_pmd_signal;	/*!< PMA/PMD signal detect */
	uint32_t	pcs_status;	/*!< PCS link status */
} bfa_phy_attr_t;

/**
 * phy stats
 */
struct bfa_phy_stats_s {
	uint32_t	status;		/*!< phy stats status */
	uint32_t	link_breaks;	/*!< Num of link breaks after linkup */
	uint32_t	pma_pmd_fault;	/*!< PMA/PMD fault register data */
	uint32_t	pcs_fault;	/*!< PCS fault register data */
	uint32_t	speed_neg;	/*!< Num of speed negotiation */
	uint32_t	tx_eq_training;	/*!< Num of TX EQ training */
	uint32_t	tx_eq_timeout;	/*!< Num of TX EQ timeout */
	uint32_t	crc_error;	/*!< Num of CRC errors */
};
typedef struct bfa_phy_stats_s bfa_phy_stats_t;

#endif /* __BFA_DEFS_PHY_H__ */
