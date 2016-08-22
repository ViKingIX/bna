/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

/**
 * @file bna_trcmod.h BNA CAL driver trace modules
 */


#ifndef __BNA_TRCMOD_H__
#define __BNA_TRCMOD_H__

/*
 * !!! Only append to the enums defined here to avoid any versioning
 * !!! needed between trace utility and driver version
 */
enum {
	BNA_TRC_HAL_BNA		= 63,
	BNA_TRC_HAL_ENET	= 62,
	BNA_TRC_HAL_TXRX	= 61,
};
#endif /* __BNA_TRCMOD_H__ */
