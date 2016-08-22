/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

/**
 * @file cs_flash.h common flash module API used by firmware, bios, efi
 */

#ifndef __CS_FLASH_H__
#define __CS_FLASH_H__

typedef struct {
	int port;
	uint64_t pcibar;
	uint16_t devid;
} cs_flash_hdl_t;

bfa_status_t cs_flash_read(cs_flash_hdl_t hdl, bfa_flash_part_type_t part,
	void *buf, int len, uint32_t offset);
bfa_status_t cs_flash_write(cs_flash_hdl_t hdl, bfa_flash_part_type_t part,
	void *buf, int len, uint32_t offset);
bfa_status_t cs_flash_sector_erase(cs_flash_hdl_t hdl,
	bfa_flash_part_type_t part, uint32_t offset);

#endif
