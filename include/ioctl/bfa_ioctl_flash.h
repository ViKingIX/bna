/*
 * Copyright (c) 2006-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved.
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BFA_IOCTL_FLASH_H__
#define __BFA_IOCTL_FLASH_H__

#include <defs/bfa_defs_flash.h>
#include <defs/bfa_defs.h>

typedef struct {
	bfa_status_t    status;
	uint16_t	bfad_num;
	uint16_t	rsvd;
	bfa_flash_attr_t attr;
} bfa_ioctl_flash_attr_t;

/*
 * For FLASH_ERASE_PART, FLASH_UPDATE_PART, and FLASH_READ_PART IOCTLs
 */
typedef struct {
	bfa_status_t    status;
	uint16_t	bfad_num;
	uint8_t		instance;
	uint8_t		rsvd;
	bfa_flash_part_type_t type;
	int		bufsz;
#ifdef _WINDOWS
	uint32_t	offset;
	char		buf_ptr[1];
#else
	uint64_t	buf_ptr;
#endif
} bfa_ioctl_flash_t;

typedef struct {
	bfa_status_t    status;
	uint16_t	bfad_num;
	uint8_t		instance;
	uint8_t		rsvd;
	bfa_flash_part_type_t type;
	int		bufsz;
	uint32_t	offset;
	char		buf_ptr[1];
} bfa_ioctl_flash_bufio_t;

typedef struct {
	bfa_status_t    status;
	uint16_t	bfad_num;
	uint16_t	rsvd;
	struct bfa_mfg_block_s mfgblk;
	uint64_t	asic_block;
	uint32_t	asic_blen;
} bfa_ioctl_flash_mfg_t;

#endif
