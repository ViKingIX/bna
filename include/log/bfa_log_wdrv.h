/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

/*
 * messages define for WDRV Module
 */
#ifndef	__BFA_LOG_WDRV_H__
#define	__BFA_LOG_WDRV_H__
#include  <cs/bfa_log.h>
#define BFA_LOG_WDRV_IOC_INIT_ERROR 	\
	(((uint32_t) BFA_LOG_WDRV_ID << BFA_LOG_MODID_OFFSET) | 1)
#define BFA_LOG_WDRV_IOC_INTERNAL_ERROR \
	(((uint32_t) BFA_LOG_WDRV_ID << BFA_LOG_MODID_OFFSET) | 2)
#define BFA_LOG_WDRV_IOC_START_ERROR 	\
	(((uint32_t) BFA_LOG_WDRV_ID << BFA_LOG_MODID_OFFSET) | 3)
#define BFA_LOG_WDRV_IOC_STOP_ERROR 	\
	(((uint32_t) BFA_LOG_WDRV_ID << BFA_LOG_MODID_OFFSET) | 4)
#define BFA_LOG_WDRV_INSUFFICIENT_RESOURCES \
	(((uint32_t) BFA_LOG_WDRV_ID << BFA_LOG_MODID_OFFSET) | 5)
#define BFA_LOG_WDRV_BASE_ADDRESS_MAP_ERROR \
	(((uint32_t) BFA_LOG_WDRV_ID << BFA_LOG_MODID_OFFSET) | 6)
#define BFA_LOG_WDRV_LUN_RESET \
	(((uint32_t) BFA_LOG_WDRV_ID << BFA_LOG_MODID_OFFSET) | 7)
#define BFA_LOG_WDRV_DEVICE_RESET \
	(((uint32_t) BFA_LOG_WDRV_ID << BFA_LOG_MODID_OFFSET) | 8)
#define BFA_LOG_WDRV_BUS_RESET \
	(((uint32_t) BFA_LOG_WDRV_ID << BFA_LOG_MODID_OFFSET) | 9)
#define BFA_LOG_WDRV_LUN_TIMEOUT \
	(((uint32_t) BFA_LOG_WDRV_ID << BFA_LOG_MODID_OFFSET) | 10)
#define BFA_LOG_WDRV_INITIATOR_TIMEOUT \
	(((uint32_t) BFA_LOG_WDRV_ID << BFA_LOG_MODID_OFFSET) | 11)
#endif
