/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */
#include <bfa_os_inc.h>

#ifndef __BFA_FLASH_RAW_H__
#define __BFA_FLASH_RAW_H__

#ifndef __FAR
#define __FAR
#endif

bfa_status_t    bfa_flash_raw_read(bfa_os_addr_t pci_bar_kva,
				       uint32_t offset, char __FAR *buf,
				       uint32_t len);
bfa_status_t    bfa_flash_raw_write(bfa_os_addr_t pci_bar_kva,
					uint32_t offset, char __FAR *buf,
					uint32_t len, uint16_t dev_id);
bfa_status_t    bfa_flash_raw_write_no_erase(bfa_os_addr_t pci_bar_kva,
					uint32_t offset, char __FAR *buf,
					uint32_t len, uint16_t dev_id);
bfa_status_t    bfa_flash_raw_erase(bfa_os_addr_t pci_bar_kva,
					uint32_t offset, uint16_t dev_id);
#endif /* __BFA_FLASH_RAW_H__ */
