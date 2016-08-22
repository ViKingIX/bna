/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

/* messages define for HAL Module */
#ifndef	__BFA_LOG_HAL_H__
#define	__BFA_LOG_HAL_H__
#include  <cs/bfa_log.h>
#define BFA_LOG_HAL_ASSERT (((uint32_t) BFA_LOG_HAL_ID << BFA_LOG_MODID_OFFSET) | 1)
#define BFA_LOG_HAL_HEARTBEAT_FAILURE (((uint32_t) BFA_LOG_HAL_ID << BFA_LOG_MODID_OFFSET) | 2)
#define BFA_LOG_HAL_FCPIM_PARM_INVALID (((uint32_t) BFA_LOG_HAL_ID << BFA_LOG_MODID_OFFSET) | 3)
#define BFA_LOG_HAL_SM_ASSERT (((uint32_t) BFA_LOG_HAL_ID << BFA_LOG_MODID_OFFSET) | 4)
#define BFA_LOG_HAL_DRIVER_ERROR (((uint32_t) BFA_LOG_HAL_ID << BFA_LOG_MODID_OFFSET) | 5)
#define BFA_LOG_HAL_DRIVER_CONFIG_ERROR (((uint32_t) BFA_LOG_HAL_ID << BFA_LOG_MODID_OFFSET) | 6)
#define BFA_LOG_HAL_MBOX_ERROR (((uint32_t) BFA_LOG_HAL_ID << BFA_LOG_MODID_OFFSET) | 7)
#endif
