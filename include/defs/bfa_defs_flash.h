/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */
#ifndef __BFA_DEFS_FLASH_H__
#define __BFA_DEFS_FLASH_H__

#include <bfa_os_inc.h>

#define BFA_FLASH_PART_ENTRY_SIZE	32	/*!< partition entry size */
#define BFA_FLASH_PART_MAX		32	/*!< maximal # of partitions */
#define BFA_FLASH_PART_MAX_V1		16	/*!< version 1 max partitions */
#define BFA_TOTAL_FLASH_SIZE		0x400000

/**
 * Flash status
 */
typedef enum {
	BFA_FLASH_STATUS_GOOD 		= 0, /*!< flash is good */
	BFA_FLASH_STATUS_NOT_PRESENT 	= 1, /*!< flash does not exist */
	BFA_FLASH_STATUS_UNINIT		= 2, /*!< present but not init */
	BFA_FLASH_STATUS_BAD 		= 3, /*!< flash is bad */
	BFA_FLASH_STATUS_EMPTY 		= 4, /*!< flash is empty */
	BFA_FLASH_STATUS_CKFAIL 	= 5, /*!< checksum failed */
} bfa_flash_status_t;

typedef enum {
	BFA_FLASH_PART_OPTROM 	= 1,	/*!< option rom partition */
	BFA_FLASH_PART_FWIMG 	= 2,	/*!< firmware image partition */
	BFA_FLASH_PART_FWCFG 	= 3,	/*!< firmware tuneable config */
	BFA_FLASH_PART_DRV 	= 4,	/*!< IOC driver config */
	BFA_FLASH_PART_BOOT 	= 5,	/*!< boot config */
	BFA_FLASH_PART_ASIC 	= 6,	/*!< asic bootstrap configuration */
	BFA_FLASH_PART_MFG 	= 7,	/*!< manufacturing block partition */
	BFA_FLASH_PART_OPTROM2 	= 8,	/*!< 2nd option rom partition */
	BFA_FLASH_PART_VPD 	= 9,	/*!< vpd data of OEM info */
	BFA_FLASH_PART_PBC 	= 10,	/*!< pre-boot config */
	BFA_FLASH_PART_BOOTOVL	= 11,	/*!< boot overlay partition */
	BFA_FLASH_PART_LOG	= 12,	/*!< firmware log partition */
	BFA_FLASH_PART_PXECFG	= 13,	/*!< pxe boot config partition */
	BFA_FLASH_PART_PXEOVL	= 14,	/*!< pxe boot overlay partition */
	BFA_FLASH_PART_PORTCFG	= 15,	/*!< port cfg partition */
	BFA_FLASH_PART_ASICBK 	= 16,	/*!< asic backup partition */
	BFA_FLASH_PART_END 	= 17,	/*!< last entry of this table */
} bfa_flash_part_type_t;

typedef enum {
	BFA_FLASH_PART_GOOD	   = 0,	/*!< partition is good */
	BFA_FLASH_PART_NOT_PRESENT = 1,	/*!< partition is not present */
	BFA_FLASH_PART_UNINIT      = 2,	/*!< partition not initialized */
	BFA_FLASH_PART_BAD	   = 3,	/*!< partition is bad */
	BFA_FLASH_PART_EMPTY	   = 4,	/*!< partition content is empty */
	BFA_FLASH_PART_CKFAIL	   = 5,	/*!< partition checksum failed */
} bfa_flash_part_status_t;

/**
 * Partition address definition
 */
#define BFA_FLASH_PART_OPTROM_ADDR  0x000000    /*!< option rom address */
#define BFA_FLASH_PART_OPTROM2_ADDR 0x090000    /*!< 2nd option rom address */
#define BFA_FLASH_PART_ASIC_ADDR    0x0e0000    /*!< asic address */
#define BFA_FLASH_PART_FWIMG_ADDR   0x100000    /*!< fw image address */
#define BFA_FLASH_PART_FWCFG_ADDR0  0x200000    /*!< fw config 0 address */
#define BFA_FLASH_PART_FWCFG_ADDR1  0x210000    /*!< fw config 1 address */
#define BFA_FLASH_PART_PBC_ADDR0    0x220000	/*!< pre-boot config 0 addr */
#define BFA_FLASH_PART_PBC_ADDR1    0x230000	/*!< pre-boot config 1 addr */
#define BFA_FLASH_PART_DRV_ADDR0    0x240000    /*!< driver config 0 addr */
#define BFA_FLASH_PART_DRV_ADDR1    0x250000    /*!< driver config 1 addr */
#define BFA_FLASH_PART_BOOT_ADDR0   0x260000    /*!< boot config 0 addr */
#define BFA_FLASH_PART_BOOT_ADDR1   0x270000    /*!< boot config 1 addr */
#define BFA_FLASH_PART_BOOTOVL_ADDR 0x280000    /*!< boot overlay addr */
#define BFA_FLASH_PART_LOG_ADDR     0x2a0000    /*!< fw log addr */
#define BFA_FLASH_PART_ASICBK_ADDR  0x2c0000    /*!< asic backup address */
#define BFA_FLASH_PART_PXECFG_ADDR0 0x300000    /*!< pxe boot config 0 addr */
#define BFA_FLASH_PART_PXECFG_ADDR1 0x310000    /*!< pxe boot config 1 addr */
#define BFA_FLASH_PART_PXEOVL_ADDR  0x320000    /*!< pxe boot overlay addr */
#define BFA_FLASH_PART_PORTCFG_ADDR0 0x340000   /*!< port cfg addr */
#define BFA_FLASH_PART_VPD_ADDR     0x3e0000	/*!< vpd block addr */
#define BFA_FLASH_PART_MFG_ADDR     0x3f0000    /*!< mfg block addr */

/**
 * flash partition attributes
 */
typedef struct {
	uint32_t	part_type;	/*!< partition type */
	uint32_t	part_instance;	/*!< partition instance */
	uint32_t	part_off;	/*!< partition offset */
	uint32_t	part_size;	/*!< partition size */
	uint32_t	part_len;	/*!< partition content length */
	uint32_t	part_status;	/*!< partition status */
	char		rsv[BFA_FLASH_PART_ENTRY_SIZE - 24];
} bfa_flash_part_attr_t;

/**
 * flash attributes
 */
typedef struct {
	uint32_t	status;	/*!< flash overall status */
	uint32_t	npart;	/*!< # of partitions */
	bfa_flash_part_attr_t part[BFA_FLASH_PART_MAX];	/*!< partition info */
} bfa_flash_attr_t;

#endif /* __BFA_DEFS_FLASH_H__ */
