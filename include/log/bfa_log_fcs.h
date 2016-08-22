/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

/*
 * messages define for FCS Module
 */
#ifndef	__BFA_LOG_FCS_H__
#define	__BFA_LOG_FCS_H__
#include  <cs/bfa_log.h>
#define BFA_LOG_FCS_FABRIC_NOSWITCH 	\
	(((uint32_t) BFA_LOG_FCS_ID << BFA_LOG_MODID_OFFSET) | 1)
#define BFA_LOG_FCS_FABRIC_ISOLATED 	\
	(((uint32_t) BFA_LOG_FCS_ID << BFA_LOG_MODID_OFFSET) | 2)
#endif
