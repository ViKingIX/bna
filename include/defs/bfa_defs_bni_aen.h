/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BFA_DEFS_BNI_AEN_H__
#define __BFA_DEFS_BNI_AEN_H__

/**
 * VLAN events
 */
enum bfa_vlan_aen_event {
	BFA_VLAN_AEN_ADD = 1,
	BFA_VLAN_AEN_REMOVE = 2,
};
typedef enum bfa_vlan_aen_event bfa_vlan_aen_event_t;

/**
 * VLAN aen data struct
 */
struct bfa_vlan_aen_data_s {
	mac_t		mac;
	uint16_t	vlan_id;	/*!< VLAN Id */
};
typedef struct bfa_vlan_aen_data_s bfa_vlan_aen_data_t;

/**
 * Team events
 */
enum bfa_team_aen_event {
	BFA_TEAM_AEN_VLAN_ADD = 1,
	BFA_TEAM_AEN_VLAN_REMOVE = 2,
	BFA_TEAM_AEN_CREATE = 3,
	BFA_TEAM_AEN_DELETE = 4,
	BFA_TEAM_AEN_PORT_ADD = 5,
	BFA_TEAM_AEN_PORT_REMOVE = 6,
	BFA_TEAM_AEN_ACTIVE = 7
};
typedef enum bfa_team_aen_event bfa_team_aen_event_t;

/**
 * Team aen data struct
 */
struct bfa_team_aen_data_s {
	mac_t		team_mac;	/*!< MAC address of the team */
	uint16_t	vlan_id;	/*!< VLAN Id */
	mac_t		port_mac;	/*!< MAC address of the Port */
	uint16_t	rsvd;		/*!< padding */
};
typedef struct bfa_team_aen_data_s bfa_team_aen_data_t;

#endif /* __BFA_DEFS_BNI_AEN_H__ */
