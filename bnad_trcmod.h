/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corp.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

/**
 * @file bfad_trcmod.h Linux driver trace modules
 */


#ifndef __BNAD_TRCMOD_H__
#define __BNAD_TRCMOD_H__


/*
 * !!! Only append to the enums defined here to avoid any versioning
 * !!! needed between trace utility and driver version
 */
enum {
	BNA_TRC_LDRV_BNAD	= 63,
	BNA_TRC_LDRV_ETHTOOL	= 62,
	BNA_TRC_LDRV_IOCTL	= 61,
	BNA_TRC_LDRV_IOCTL_CMN	= 60,
	BNA_TRC_LDRV_COMPAT	= 59,
	BNA_TRC_LDRV_AEN	= 58,
	BNA_TRC_LDRV_QUEUE	= 57,
	BNA_TRC_LDRV_VMCOMPAT	= 56,
	BNA_TRC_LDRV_DIAG_LB_CMN = 55,
};
#endif /* __BNAD_TRCMOD_H__ */
