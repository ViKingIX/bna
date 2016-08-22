/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BFA_DEFS_BOOT_H__
#define __BFA_DEFS_BOOT_H__

#include <defs/bfa_defs.h>
#include <defs/bfa_defs_svc.h>

enum {
	BFA_BOOT_BOOTLUN_MAX = 4,	/*!< maximum boot lun per IOC */
	BFA_PREBOOT_BOOTLUN_MAX = 8,	/*!< maximum preboot lun per IOC */
};

#define BOOT_CFG_REV1			1
#define BFA_BOOT_CFG_DEFAULT_VLAN	0

/** 
 * @brief
 *      Boot options setting. Boot options setting determines from where
 *      to get the boot lun information
 */
typedef enum {
	BFA_BOOT_AUTO_DISCOVER	= 0, /*!< Boot from blun provided by fabric */
	BFA_BOOT_STORED_BLUN = 1, /*!< Boot from bluns stored in flash */
	BFA_BOOT_FIRST_LUN	= 2, /*!< Boot from first discovered blun */
	BFA_BOOT_PBC	= 3, /*!< Boot from pbc configured blun  */
} bfa_boot_bootopt_t;

#pragma pack(1)
/**
 * Boot lun information.
 */
struct bfa_boot_bootlun_s {
	wwn_t	pwwn;	/*!< port wwn of target */
	lun_t	lun;	/*!< 64-bit lun */
};
typedef struct bfa_boot_bootlun_s bfa_boot_bootlun_t;
#pragma pack()

/**
 * BOOT boot configuraton
 */
struct bfa_boot_cfg_s {
	uint8_t		version;
	uint8_t		rsvd1;
	uint16_t	chksum;

	uint8_t		enable;		/*!< enable/disable SAN boot */
	uint8_t		speed;		/*!< boot speed settings */
	uint8_t		topology;	/*!< boot topology setting */
	uint8_t		bootopt;	/*!< bfa_boot_bootopt_t */

	uint32_t	nbluns;		/*!< number of boot luns */
	uint8_t		bootup_delay;	/*!< bootup delay */
	uint8_t		rsvd2[3];

	struct bfa_boot_bootlun_s blun[BFA_BOOT_BOOTLUN_MAX];
	bfa_boot_bootlun_t blun_disc[BFA_BOOT_BOOTLUN_MAX];
};
typedef struct bfa_boot_cfg_s bfa_boot_cfg_t;

struct bfa_boot_pbc_s {
	uint8_t		enable;		/*!< enable/disable SAN boot */
	uint8_t		speed;		/*!< boot speed settings */
	uint8_t		topology;	/*!< boot topology setting */
	uint8_t		rsvd1;
	uint32_t	nbluns;		/*!< number of boot luns */
	struct bfa_boot_bootlun_s pblun[BFA_PREBOOT_BOOTLUN_MAX];
};
typedef struct bfa_boot_pbc_s bfa_boot_pbc_t;

/**
 * @brief
 * 	Bootup delay settings
 */
typedef enum {
	BFA_BOOTUP_DELAY_0   = 0,
	BFA_BOOTUP_DELAY_1   = 1,
	BFA_BOOTUP_DELAY_2   = 2,
	BFA_BOOTUP_DELAY_5   = 5,
	BFA_BOOTUP_DELAY_10  = 10,

	BFA_BOOTUP_DELAY_MIN	= BFA_BOOTUP_DELAY_0,
	BFA_BOOTUP_DELAY_MAX	= BFA_BOOTUP_DELAY_10,
} bfa_bootup_delay_t;

struct bfa_ethboot_cfg_s {
	uint8_t		version;
	uint8_t		rsvd1;
	uint16_t	chksum;

	uint8_t		enable;	/*!< enable/disable Eth/PXE boot */
	uint8_t		rsvd2;
	uint16_t	vlan;	/*!< Vlan id configuration */
};
typedef struct bfa_ethboot_cfg_s bfa_ethboot_cfg_t;

struct bfa_bios_s {
	uint8_t		enable;
	uint8_t		bootup_delay;
	uint16_t	rsvd;
};

typedef struct {

	uint32_t    entry_tick;
	uint32_t    exit_tick;
	uint32_t    cur_time;
	uint32_t    total_time;
	uint32_t    value_tick; /* Per tick value based on asic type */

} bfad_bios_bootup_timer_t;

#define IPLEN	4
#define BFA_IBOOT_PARAM_LEN	16
#define BFA_IBOOT_NAME_LEN	64

typedef enum {
	BFA_AUTH_NIL = 0,
	BFA_AUTH_CHAP = 1,
	BFA_AUTH_REV_CHAP = 2,
} bfa_iscsi_boot_auth_t;

typedef enum {
	BFA_MODE_STATIC = 0,
	BFA_MODE_DHCP = 1,
	BFA_MODE_ISNS = 2,
} bfa_iscsi_boot_ip_mode_t;

struct bfa_tcpip_s {
	uint8_t		mode;
	uint8_t		ipver;
	uint16_t	vlan;
	uint8_t		ip[IPLEN];
	uint8_t		netmask[IPLEN];
	uint8_t		gip[IPLEN];
	char		init_name[BFA_IBOOT_NAME_LEN];
};
typedef struct bfa_tcpip_s bfa_tcpip_t;

struct bfa_auth_s {
	uint8_t		mode;
	uint8_t		rsvd1[3];
	char		chap_name_1[BFA_IBOOT_PARAM_LEN];
	char		chap_secret_1[BFA_IBOOT_PARAM_LEN];
	char		reverse_chap_name_1[BFA_IBOOT_PARAM_LEN];
	char		reverse_chap_secret_1[BFA_IBOOT_PARAM_LEN];
	char		chap_name_2[BFA_IBOOT_PARAM_LEN];
	char		chap_secret_2[BFA_IBOOT_PARAM_LEN];
	char		reverse_chap_name_2[BFA_IBOOT_PARAM_LEN];
	char		reverse_chap_secret_2[BFA_IBOOT_PARAM_LEN];
};
typedef struct bfa_auth_s bfa_auth_t;

struct bfa_iscsi_boot_tgt_s {
	uint16_t	ip_port;
	uint8_t		rsvd1[2];

	uint8_t		ip[IPLEN];
	char		name[BFA_IBOOT_NAME_LEN];

	uint64_t	lun;
};
typedef struct bfa_iscsi_boot_tgt_s bfa_iscsi_boot_tgt_t;

struct bfa_iscsi_boot_dev_s {
	uint8_t			mode;
	uint8_t			rsvd1[3];
	bfa_iscsi_boot_tgt_t	tgt[2];
	uint16_t		isns_port;
	uint8_t			rsvd2[2];
	uint8_t			isns_ip[IPLEN];
};

struct bfa_iscsiboot_cfg_s {
	uint8_t				version;
	uint8_t				rsvd;
	uint16_t			cksum;
	struct bfa_bios_s		bios;
	struct bfa_tcpip_s		tcpip;
	struct bfa_auth_s		auth;
	struct bfa_iscsi_boot_dev_s	bootdev;
};
typedef struct bfa_iscsiboot_cfg_s bfa_iscsiboot_cfg_t;

#endif /* __BFA_DEFS_BOOT_H__ */
