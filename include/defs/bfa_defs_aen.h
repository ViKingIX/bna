/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BFA_DEFS_AEN_H__
#define __BFA_DEFS_AEN_H__

#include <defs/bfa_defs.h>
#include <defs/bfa_defs_svc.h>
#include <defs/bfa_defs_fcs.h>
#include <defs/bfa_defs_ipfc.h>
#include <defs/bfa_defs_ethport.h>
#include <defs/bfa_defs_bni_aen.h>

#define BFA_AEN_MAX_APP		5
#define BFAD_NL_VENDOR_ID (((uint64_t)0x01 << SCSI_NL_VID_TYPE_SHIFT)	\
				| BFA_PCI_VENDOR_ID_BROCADE)

enum bfa_aen_app {
	bfa_aen_app_bcu = 0,	/* No thread for bcu */
	bfa_aen_app_hcm = 1,
	bfa_aen_app_cim = 2,
	bfa_aen_app_snia = 3,
	bfa_aen_app_test = 4,	/* To be removed after unit test */
};
typedef enum bfa_aen_app bfa_aen_app_t;

enum bfa_aen_category {
	BFA_AEN_CAT_ADAPTER 	= 1,
	BFA_AEN_CAT_PORT 	= 2,
	BFA_AEN_CAT_LPORT 	= 3,
	BFA_AEN_CAT_RPORT 	= 4,
	BFA_AEN_CAT_ITNIM 	= 5,
	BFA_AEN_CAT_TIN 	= 6,
	BFA_AEN_CAT_IPFC 	= 7,
	BFA_AEN_CAT_AUDIT 	= 8,
	BFA_AEN_CAT_IOC 	= 9,
	BFA_AEN_CAT_ETHPORT	= 10,
	BFA_AEN_CAT_TEAM	= 11,
	BFA_AEN_CAT_VLAN	= 12,
	BFA_AEN_MAX_CAT 	= 12
};
typedef enum bfa_aen_category bfa_aen_category_t;

/**
 * BFA audit events
 */
enum bfa_audit_aen_event {
	BFA_AUDIT_AEN_AUTH_ENABLE 	= 1,
	BFA_AUDIT_AEN_AUTH_DISABLE 	= 2,
	BFA_AUDIT_AEN_FLASH_ERASE 	= 3,
	BFA_AUDIT_AEN_FLASH_UPDATE 	= 4,
};
typedef enum bfa_audit_aen_event bfa_audit_aen_event_t;

/**
 * audit event data
 */
struct bfa_audit_aen_data_s {
	wwn_t	pwwn;
	int	partition_inst;
	int	partition_type;
};
typedef struct bfa_audit_aen_data_s bfa_audit_aen_data_t;

union bfa_aen_data_u {
	struct bfa_adapter_aen_data_s 	adapter;
	struct bfa_port_aen_data_s 	port;
	struct bfa_lport_aen_data_s 	lport;
	struct bfa_rport_aen_data_s 	rport;
	struct bfa_itnim_aen_data_s 	itnim;
	struct bfa_audit_aen_data_s 	audit;
	struct bfa_ioc_aen_data_s 	ioc;
	struct bfa_ethport_aen_data_s 	ethport;
	struct bfa_team_aen_data_s 	team;
	struct bfa_vlan_aen_data_s 	vlan;
};
typedef union bfa_aen_data_u bfa_aen_data_t;

struct bfa_aen_entry_s {
	enum bfa_aen_category 	aen_category;
	int32_t			aen_type;
	union bfa_aen_data_u	aen_data;
	struct bfa_timeval_s	aen_tv;
	int32_t			seq_num;
	int32_t			bfad_num;
};
typedef struct bfa_aen_entry_s bfa_aen_entry_t;

typedef int bfa_aen_event_t;

#endif /* __BFA_DEFS_AEN_H__ */
