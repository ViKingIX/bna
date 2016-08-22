/*
 * Copyright (c) 2011-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */
#ifndef __BFI_MFG_H__
#define __BFI_MFG_H__

#include <bfi/bfi.h>

#pragma pack(1)

enum bfi_mfg_h2i_msgs {
	BFI_MFG_H2I_UPDATE_REQ		= 1,
};
typedef enum bfi_mfg_h2i_msgs bfi_mfg_h2i_msgs_t;

enum bfi_mfg_i2h_msgs {
	BFI_MFG_I2H_UPDATE_REPLY	= BFA_I2HM(1),
};
typedef enum bfi_mfg_i2h_msgs bfi_mfg_i2h_msgs_t;

/**
 * BFI_MSG_H2I_UPDATE_REQ message
 */
struct bfi_mfg_update_req_s {
	struct bfi_mhdr_s	mh;
	struct bfi_alen_s	alen_mfgblk;
	struct bfi_alen_s	alen_ablk;
};
typedef struct bfi_mfg_update_req_s bfi_mfg_update_req_t;


/**
 * BFI_MSG_I2H_UPDATE_REPLY message
 */
struct bfi_mfg_update_reply_s {
	struct bfi_mhdr_s	mh;		/*!< Common msg header	*/
	uint8_t			status;		/*!< Update status	*/
	uint8_t			rsvd[3];
};
typedef struct bfi_mfg_update_reply_s bfi_mfg_update_reply_t;

#pragma pack()

#endif	/* __BFI_MFG_H__ */
