/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

/* messages define for SOLARIS Module */
#ifndef	__BFA_LOG_SOLARIS_H__
#define	__BFA_LOG_SOLARIS_H__
#include  <cs/bfa_log.h>
#define BFA_LOG_SOLARIS_DRIVER_ERROR (((uint32_t) BFA_LOG_SOLARIS_ID << BFA_LOG_MODID_OFFSET) | 1)
#define BFA_LOG_SOLARIS_DRIVER_WARN (((uint32_t) BFA_LOG_SOLARIS_ID << BFA_LOG_MODID_OFFSET) | 2)
#define BFA_LOG_SOLARIS_DRIVER_INFO (((uint32_t) BFA_LOG_SOLARIS_ID << BFA_LOG_MODID_OFFSET) | 3)
#define BFA_LOG_SOLARIS_DRIVER_DIAG (((uint32_t) BFA_LOG_SOLARIS_ID << BFA_LOG_MODID_OFFSET) | 4)
#endif
