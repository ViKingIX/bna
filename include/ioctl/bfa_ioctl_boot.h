/*
 * QLogic Fibre Channel HBA Driver
 * Copyright (c) 2006-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved.
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BFA_IOCTL_BOOT_H__
#define __BFA_IOCTL_BOOT_H__

#include <defs/bfa_defs_boot.h>

typedef struct {
	bfa_status_t    status;
	uint16_t	bfad_num;
	uint16_t	rsvd;
	struct bfa_boot_cfg_s  cfg;
} bfa_ioctl_boot_t;

typedef struct {
	bfa_status_t    status;
	uint16_t	bfad_num;
	uint16_t	rsvd;
	struct bfa_iscsiboot_cfg_s  cfg;
} bfa_ioctl_iscsiboot_t;

typedef struct {
	bfa_status_t    status;
	uint16_t	bfad_num;
	uint16_t	rsvd;
	struct bfa_boot_pbc_s  cfg;
} bfa_ioctl_preboot_t;

typedef struct {
	bfa_status_t	status;
	uint16_t	bfad_num;
	uint16_t	rsvd;
	struct	bfa_ethboot_cfg_s  cfg;
} bfa_ioctl_ethboot_t;

#endif
