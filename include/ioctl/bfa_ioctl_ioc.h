/*
 * Copyright (c) 2006-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved.
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BFA_IOCTL_IOC_H__
#define __BFA_IOCTL_IOC_H__

#include <defs/bfa_defs.h>
#include <defs/bfa_defs_svc.h>

typedef struct {
	bfa_status_t    status;
	uint16_t	bfad_num;
	uint16_t	rsvd;
	struct bfa_ioc_attr_s  ioc_attr;
} bfa_ioctl_ioc_attr_t;

typedef struct {
	bfa_status_t    status;
	uint16_t	bfad_num;
	uint16_t	rsvd;
	struct bfa_ioc_stats_s ioc_stats;
} bfa_ioctl_ioc_stats_t;

typedef struct {
	bfa_status_t    status;
	uint16_t	bfad_num;
	uint16_t	rsvd;
	uint32_t	buf_size;
	uint32_t	rsvd1;
#ifdef _WINDOWS
	uint32_t	buf_ptr[1];
#else
	uint64_t	buf_ptr;
#endif
} bfa_ioctl_ioc_fwstats_t;

typedef struct {
	bfa_status_t    status;
	uint16_t	bfad_num;
	uint16_t	rsvd;
	char		serialnum[64];
	char		hwpath[BFA_STRING_32];
	char		adapter_hwpath[BFA_STRING_32];
	char		guid[BFA_ADAPTER_SYM_NAME_LEN*2];
	char		name[BFA_ADAPTER_SYM_NAME_LEN];
	char		port_name[BFA_ADAPTER_SYM_NAME_LEN];
	char		eth_name[BFA_ADAPTER_SYM_NAME_LEN];
	wwn_t		pwwn;
	wwn_t		nwwn;
	wwn_t		factorypwwn;
	wwn_t		factorynwwn;
	mac_t		mac;
	mac_t		factory_mac; // Factory mac address
	mac_t		current_mac; // Currently assigned mac address
	enum bfa_ioc_type_e	ioc_type;
	uint16_t	pvid; // Port vlan id
	uint16_t	rsvd1;
	uint32_t	host;
	uint32_t	bandwidth;	/* For PF support */
	uint32_t	rsvd2;
} bfa_ioctl_ioc_info_t;

typedef struct {
	uint32_t	cnt;	/* total number of ioc instances */
	uint32_t	rsvd;
	char		bm[256 / 8];	/* bit mask with instances positions */
} bfa_ioctl_ioc_get_inst_t;

typedef struct {
	bfa_status_t    status;
	uint32_t	version;
} bfa_ioctl_ioc_get_version_t;

typedef struct {
	bfa_status_t    status;
	uint16_t	bfad_num;
	uint16_t	rsvd;
	char		name[BFA_ADAPTER_SYM_NAME_LEN];
} bfa_ioctl_ioc_set_adapter_name_t;

typedef struct {
	bfa_status_t    status;
	uint16_t	bfad_num;
	uint16_t	rsvd;
	bfa_adapter_cfg_mode_t cfg;
} bfa_ioctl_adapter_cfg_mode_t;

#endif
