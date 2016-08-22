/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */
/**
 * PLEASE DO NOT EDIT THIS FILE. THIS FILE IS AUTO GENERATED!!
 */
#include <cs/bfa_log.h>

/* messages define for BFA_AEN_CAT_ADAPTER Module */
#define BFA_AEN_ADAPTER_ADD  BFA_LOG_CREATE_ID(BFA_AEN_CAT_ADAPTER, BFA_ADAPTER_AEN_ADD)
#define BFA_AEN_ADAPTER_REMOVE  BFA_LOG_CREATE_ID(BFA_AEN_CAT_ADAPTER, BFA_ADAPTER_AEN_REMOVE)

/* messages define for BFA_AEN_CAT_AUDIT Module */
#define BFA_AEN_AUDIT_AUTH_ENABLE  BFA_LOG_CREATE_ID(BFA_AEN_CAT_AUDIT, BFA_AUDIT_AEN_AUTH_ENABLE)
#define BFA_AEN_AUDIT_AUTH_DISABLE  BFA_LOG_CREATE_ID(BFA_AEN_CAT_AUDIT, BFA_AUDIT_AEN_AUTH_DISABLE)
#define BFA_AEN_AUDIT_FLASH_ERASE  BFA_LOG_CREATE_ID(BFA_AEN_CAT_AUDIT, BFA_AUDIT_AEN_FLASH_ERASE)
#define BFA_AEN_AUDIT_FLASH_UPDATE  BFA_LOG_CREATE_ID(BFA_AEN_CAT_AUDIT, BFA_AUDIT_AEN_FLASH_UPDATE)

/* messages define for BFA_AEN_CAT_ETHPORT Module */
#define BFA_AEN_ETHPORT_LINKUP  BFA_LOG_CREATE_ID(BFA_AEN_CAT_ETHPORT, BFA_ETHPORT_AEN_LINKUP)
#define BFA_AEN_ETHPORT_LINKDOWN  BFA_LOG_CREATE_ID(BFA_AEN_CAT_ETHPORT, BFA_ETHPORT_AEN_LINKDOWN)
#define BFA_AEN_ETHPORT_ENABLE  BFA_LOG_CREATE_ID(BFA_AEN_CAT_ETHPORT, BFA_ETHPORT_AEN_ENABLE)
#define BFA_AEN_ETHPORT_DISABLE  BFA_LOG_CREATE_ID(BFA_AEN_CAT_ETHPORT, BFA_ETHPORT_AEN_DISABLE)

/* messages define for BFA_AEN_CAT_IOC Module */
#define BFA_AEN_IOC_HBGOOD  BFA_LOG_CREATE_ID(BFA_AEN_CAT_IOC, BFA_IOC_AEN_HBGOOD)
#define BFA_AEN_IOC_HBFAIL  BFA_LOG_CREATE_ID(BFA_AEN_CAT_IOC, BFA_IOC_AEN_HBFAIL)
#define BFA_AEN_IOC_ENABLE  BFA_LOG_CREATE_ID(BFA_AEN_CAT_IOC, BFA_IOC_AEN_ENABLE)
#define BFA_AEN_IOC_DISABLE  BFA_LOG_CREATE_ID(BFA_AEN_CAT_IOC, BFA_IOC_AEN_DISABLE)
#define BFA_AEN_IOC_FWMISMATCH  BFA_LOG_CREATE_ID(BFA_AEN_CAT_IOC, BFA_IOC_AEN_FWMISMATCH)
#define BFA_AEN_IOC_FWCFG_ERROR  BFA_LOG_CREATE_ID(BFA_AEN_CAT_IOC, BFA_IOC_AEN_FWCFG_ERROR)
#define BFA_AEN_IOC_INVALID_VENDOR  BFA_LOG_CREATE_ID(BFA_AEN_CAT_IOC, BFA_IOC_AEN_INVALID_VENDOR)
#define BFA_AEN_IOC_INVALID_NWWN  BFA_LOG_CREATE_ID(BFA_AEN_CAT_IOC, BFA_IOC_AEN_INVALID_NWWN)
#define BFA_AEN_IOC_INVALID_PWWN  BFA_LOG_CREATE_ID(BFA_AEN_CAT_IOC, BFA_IOC_AEN_INVALID_PWWN)
#define BFA_AEN_IOC_INVALID_MAC  BFA_LOG_CREATE_ID(BFA_AEN_CAT_IOC, BFA_IOC_AEN_INVALID_MAC)

/* messages define for BFA_AEN_CAT_ITNIM Module */
#define BFA_AEN_ITNIM_ONLINE  BFA_LOG_CREATE_ID(BFA_AEN_CAT_ITNIM, BFA_ITNIM_AEN_ONLINE)
#define BFA_AEN_ITNIM_OFFLINE  BFA_LOG_CREATE_ID(BFA_AEN_CAT_ITNIM, BFA_ITNIM_AEN_OFFLINE)
#define BFA_AEN_ITNIM_DISCONNECT  BFA_LOG_CREATE_ID(BFA_AEN_CAT_ITNIM, BFA_ITNIM_AEN_DISCONNECT)

/* messages define for BFA_AEN_CAT_LPORT Module */
#define BFA_AEN_LPORT_NEW  BFA_LOG_CREATE_ID(BFA_AEN_CAT_LPORT, BFA_LPORT_AEN_NEW)
#define BFA_AEN_LPORT_DELETE  BFA_LOG_CREATE_ID(BFA_AEN_CAT_LPORT, BFA_LPORT_AEN_DELETE)
#define BFA_AEN_LPORT_ONLINE  BFA_LOG_CREATE_ID(BFA_AEN_CAT_LPORT, BFA_LPORT_AEN_ONLINE)
#define BFA_AEN_LPORT_OFFLINE  BFA_LOG_CREATE_ID(BFA_AEN_CAT_LPORT, BFA_LPORT_AEN_OFFLINE)
#define BFA_AEN_LPORT_DISCONNECT  BFA_LOG_CREATE_ID(BFA_AEN_CAT_LPORT, BFA_LPORT_AEN_DISCONNECT)
#define BFA_AEN_LPORT_NEW_PROP  BFA_LOG_CREATE_ID(BFA_AEN_CAT_LPORT, BFA_LPORT_AEN_NEW_PROP)
#define BFA_AEN_LPORT_DELETE_PROP  BFA_LOG_CREATE_ID(BFA_AEN_CAT_LPORT, BFA_LPORT_AEN_DELETE_PROP)
#define BFA_AEN_LPORT_NEW_STANDARD  BFA_LOG_CREATE_ID(BFA_AEN_CAT_LPORT, BFA_LPORT_AEN_NEW_STANDARD)
#define BFA_AEN_LPORT_DELETE_STANDARD  BFA_LOG_CREATE_ID(BFA_AEN_CAT_LPORT, BFA_LPORT_AEN_DELETE_STANDARD)
#define BFA_AEN_LPORT_NPIV_DUP_WWN  BFA_LOG_CREATE_ID(BFA_AEN_CAT_LPORT, BFA_LPORT_AEN_NPIV_DUP_WWN)
#define BFA_AEN_LPORT_NPIV_FABRIC_MAX  BFA_LOG_CREATE_ID(BFA_AEN_CAT_LPORT, BFA_LPORT_AEN_NPIV_FABRIC_MAX)
#define BFA_AEN_LPORT_NPIV_UNKNOWN  BFA_LOG_CREATE_ID(BFA_AEN_CAT_LPORT, BFA_LPORT_AEN_NPIV_UNKNOWN)
#define BFA_AEN_LPORT_NEW_SYNTH_FC  BFA_LOG_CREATE_ID(BFA_AEN_CAT_LPORT, BFA_LPORT_AEN_NEW_SYNTH_FC)
#define BFA_AEN_LPORT_DELETE_SYNTH_FC  BFA_LOG_CREATE_ID(BFA_AEN_CAT_LPORT, BFA_LPORT_AEN_DELETE_SYNTH_FC)

/* messages define for BFA_AEN_CAT_PORT Module */
#define BFA_AEN_PORT_ONLINE  BFA_LOG_CREATE_ID(BFA_AEN_CAT_PORT, BFA_PORT_AEN_ONLINE)
#define BFA_AEN_PORT_OFFLINE  BFA_LOG_CREATE_ID(BFA_AEN_CAT_PORT, BFA_PORT_AEN_OFFLINE)
#define BFA_AEN_PORT_RLIR  BFA_LOG_CREATE_ID(BFA_AEN_CAT_PORT, BFA_PORT_AEN_RLIR)
#define BFA_AEN_PORT_SFP_INSERT  BFA_LOG_CREATE_ID(BFA_AEN_CAT_PORT, BFA_PORT_AEN_SFP_INSERT)
#define BFA_AEN_PORT_SFP_REMOVE  BFA_LOG_CREATE_ID(BFA_AEN_CAT_PORT, BFA_PORT_AEN_SFP_REMOVE)
#define BFA_AEN_PORT_SFP_POM  BFA_LOG_CREATE_ID(BFA_AEN_CAT_PORT, BFA_PORT_AEN_SFP_POM)
#define BFA_AEN_PORT_ENABLE  BFA_LOG_CREATE_ID(BFA_AEN_CAT_PORT, BFA_PORT_AEN_ENABLE)
#define BFA_AEN_PORT_DISABLE  BFA_LOG_CREATE_ID(BFA_AEN_CAT_PORT, BFA_PORT_AEN_DISABLE)
#define BFA_AEN_PORT_AUTH_ON  BFA_LOG_CREATE_ID(BFA_AEN_CAT_PORT, BFA_PORT_AEN_AUTH_ON)
#define BFA_AEN_PORT_AUTH_OFF  BFA_LOG_CREATE_ID(BFA_AEN_CAT_PORT, BFA_PORT_AEN_AUTH_OFF)
#define BFA_AEN_PORT_DISCONNECT  BFA_LOG_CREATE_ID(BFA_AEN_CAT_PORT, BFA_PORT_AEN_DISCONNECT)
#define BFA_AEN_PORT_QOS_NEG  BFA_LOG_CREATE_ID(BFA_AEN_CAT_PORT, BFA_PORT_AEN_QOS_NEG)
#define BFA_AEN_PORT_FABRIC_NAME_CHANGE  BFA_LOG_CREATE_ID(BFA_AEN_CAT_PORT, BFA_PORT_AEN_FABRIC_NAME_CHANGE)
#define BFA_AEN_PORT_SFP_ACCESS_ERROR  BFA_LOG_CREATE_ID(BFA_AEN_CAT_PORT, BFA_PORT_AEN_SFP_ACCESS_ERROR)
#define BFA_AEN_PORT_SFP_UNSUPPORT  BFA_LOG_CREATE_ID(BFA_AEN_CAT_PORT, BFA_PORT_AEN_SFP_UNSUPPORT)

/* messages define for BFA_AEN_CAT_RPORT Module */
#define BFA_AEN_RPORT_ONLINE  BFA_LOG_CREATE_ID(BFA_AEN_CAT_RPORT, BFA_RPORT_AEN_ONLINE)
#define BFA_AEN_RPORT_OFFLINE  BFA_LOG_CREATE_ID(BFA_AEN_CAT_RPORT, BFA_RPORT_AEN_OFFLINE)
#define BFA_AEN_RPORT_DISCONNECT  BFA_LOG_CREATE_ID(BFA_AEN_CAT_RPORT, BFA_RPORT_AEN_DISCONNECT)
#define BFA_AEN_RPORT_QOS_PRIO  BFA_LOG_CREATE_ID(BFA_AEN_CAT_RPORT, BFA_RPORT_AEN_QOS_PRIO)
#define BFA_AEN_RPORT_QOS_FLOWID  BFA_LOG_CREATE_ID(BFA_AEN_CAT_RPORT, BFA_RPORT_AEN_QOS_FLOWID)

/* messages define for BFA_AEN_CAT_TEAM Module */
#define BFA_AEN_TEAM_VLAN_ADD  BFA_LOG_CREATE_ID(BFA_AEN_CAT_TEAM, BFA_TEAM_AEN_VLAN_ADD)
#define BFA_AEN_TEAM_VLAN_REMOVE  BFA_LOG_CREATE_ID(BFA_AEN_CAT_TEAM, BFA_TEAM_AEN_VLAN_REMOVE)
#define BFA_AEN_TEAM_CREATE  BFA_LOG_CREATE_ID(BFA_AEN_CAT_TEAM, BFA_TEAM_AEN_CREATE)
#define BFA_AEN_TEAM_DELETE  BFA_LOG_CREATE_ID(BFA_AEN_CAT_TEAM, BFA_TEAM_AEN_DELETE)
#define BFA_AEN_TEAM_PORT_ADD  BFA_LOG_CREATE_ID(BFA_AEN_CAT_TEAM, BFA_TEAM_AEN_PORT_ADD)
#define BFA_AEN_TEAM_PORT_REMOVE  BFA_LOG_CREATE_ID(BFA_AEN_CAT_TEAM, BFA_TEAM_AEN_PORT_REMOVE)
#define BFA_AEN_TEAM_ACTIVE  BFA_LOG_CREATE_ID(BFA_AEN_CAT_TEAM, BFA_TEAM_AEN_ACTIVE)

/* messages define for BFA_AEN_CAT_VLAN Module */
#define BFA_AEN_VLAN_ADD  BFA_LOG_CREATE_ID(BFA_AEN_CAT_VLAN, BFA_VLAN_AEN_ADD)
#define BFA_AEN_VLAN_REMOVE  BFA_LOG_CREATE_ID(BFA_AEN_CAT_VLAN, BFA_VLAN_AEN_REMOVE)

#include <log/bfa_log_fcs.h>
#include <log/bfa_log_hal.h>
#include <log/bfa_log_linux.h>
#include <log/bfa_log_solaris.h>
#include <log/bfa_log_wdrv.h>

struct bfa_log_msgdef_s bfa_log_msg_array[] = {


/* messages define for BFA_AEN_CAT_ADAPTER Module */
	{BFA_AEN_ADAPTER_ADD, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_ADAPTER_ADD",
	 "New adapter found: SN = %s base port WWN = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | (BFA_LOG_S << BFA_LOG_ARG1) | 0), 2},

	{BFA_AEN_ADAPTER_REMOVE, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_WARNING, "BFA_AEN_ADAPTER_REMOVE",
	 "Adapter removed: SN = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},




/* messages define for BFA_AEN_CAT_AUDIT Module */
	{BFA_AEN_AUDIT_AUTH_ENABLE, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_AUDIT_AUTH_ENABLE",
	 "Authentication enabled for base port: WWN = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_AEN_AUDIT_AUTH_DISABLE, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_AUDIT_AUTH_DISABLE",
	 "Authentication disabled for base port: WWN = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_AEN_AUDIT_FLASH_ERASE, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_CRITICAL, "BFA_AEN_AUDIT_FLASH_ERASE",
	 "Flash partition instance %d of type %d erased, WWN = %s.",
	 ((BFA_LOG_D << BFA_LOG_ARG0) | (BFA_LOG_D << BFA_LOG_ARG1) |
	  (BFA_LOG_S << BFA_LOG_ARG2) | 0), 3},

	{BFA_AEN_AUDIT_FLASH_UPDATE, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_CRITICAL, "BFA_AEN_AUDIT_FLASH_UPDATE",
	 "Flash partition instance %d of type %d updated, WWN = %s.",
	 ((BFA_LOG_D << BFA_LOG_ARG0) | (BFA_LOG_D << BFA_LOG_ARG1) |
	  (BFA_LOG_S << BFA_LOG_ARG2) | 0), 3},




/* messages define for BFA_AEN_CAT_ETHPORT Module */
	{BFA_AEN_ETHPORT_LINKUP, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_ETHPORT_LINKUP",
	 "Base port ethernet linkup: mac = %s CEE status = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | (BFA_LOG_S << BFA_LOG_ARG1) | 0), 2},

	{BFA_AEN_ETHPORT_LINKDOWN, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_ETHPORT_LINKDOWN",
	 "Base port ethernet linkdown: mac = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_AEN_ETHPORT_ENABLE, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_ETHPORT_ENABLE",
	 "Base port ethernet interface enabled: mac = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_AEN_ETHPORT_DISABLE, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_ETHPORT_DISABLE",
	 "Base port ethernet interface disabled: mac = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},




/* messages define for BFA_AEN_CAT_IOC Module */
	{BFA_AEN_IOC_HBGOOD, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_IOC_HBGOOD",
	 "Heart Beat of IOC for %s is good.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_AEN_IOC_HBFAIL, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_CRITICAL, "BFA_AEN_IOC_HBFAIL",
	 "Heart Beat of IOC for %s has failed.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_AEN_IOC_ENABLE, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_IOC_ENABLE",
	 "IOC for %s is enabled.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_AEN_IOC_DISABLE, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_IOC_DISABLE",
	 "IOC for %s is disabled.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_AEN_IOC_FWMISMATCH, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_WARNING, "BFA_AEN_IOC_FWMISMATCH",
	 "Running firmware version is incompatible with the driver version.",
	 (0), 0},

	{BFA_AEN_IOC_FWCFG_ERROR, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_CRITICAL, "BFA_AEN_IOC_FWCFG_ERROR",
	 "Link initialization failed due to firmware configuration read error: WWN = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_AEN_IOC_INVALID_VENDOR, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_ERROR, "BFA_AEN_IOC_INVALID_VENDOR",
	 "Unsupported switch vendor. Link initialization failed: WWN = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_AEN_IOC_INVALID_NWWN, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_ERROR, "BFA_AEN_IOC_INVALID_NWWN",
	 "Invalid NWWN. Link initialization failed: NWWN = 00:00:00:00:00:00:00:00.",
	 (0), 0},

	{BFA_AEN_IOC_INVALID_PWWN, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_ERROR, "BFA_AEN_IOC_INVALID_PWWN",
	 "Invalid PWWN. Link initialization failed: PWWN = 00:00:00:00:00:00:00:00.",
	 (0), 0},

	{BFA_AEN_IOC_INVALID_MAC, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_WARNING, "BFA_AEN_IOC_INVALID_MAC",
	 "Base MAC Address set failed. MAC = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},




/* messages define for BFA_AEN_CAT_ITNIM Module */
	{BFA_AEN_ITNIM_ONLINE, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_ITNIM_ONLINE",
	 "Target (WWN = %s) is online for initiator (WWN = %s).",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | (BFA_LOG_S << BFA_LOG_ARG1) | 0), 2},

	{BFA_AEN_ITNIM_OFFLINE, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_ITNIM_OFFLINE",
	 "Target (WWN = %s) offlined by initiator (WWN = %s).",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | (BFA_LOG_S << BFA_LOG_ARG1) | 0), 2},

	{BFA_AEN_ITNIM_DISCONNECT, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_ERROR, "BFA_AEN_ITNIM_DISCONNECT",
	 "Target (WWN = %s) connectivity lost for initiator (WWN = %s).",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | (BFA_LOG_S << BFA_LOG_ARG1) | 0), 2},




/* messages define for BFA_AEN_CAT_LPORT Module */
	{BFA_AEN_LPORT_NEW, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_LPORT_NEW",
	 "New logical port created: WWN = %s Role = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | (BFA_LOG_S << BFA_LOG_ARG1) | 0), 2},

	{BFA_AEN_LPORT_DELETE, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_LPORT_DELETE",
	 "Logical port deleted: WWN = %s Role = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | (BFA_LOG_S << BFA_LOG_ARG1) | 0), 2},

	{BFA_AEN_LPORT_ONLINE, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_LPORT_ONLINE",
	 "Logical port online: WWN = %s Role = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | (BFA_LOG_S << BFA_LOG_ARG1) | 0), 2},

	{BFA_AEN_LPORT_OFFLINE, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_LPORT_OFFLINE",
	 "Logical port taken offline: WWN = %s Role = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | (BFA_LOG_S << BFA_LOG_ARG1) | 0), 2},

	{BFA_AEN_LPORT_DISCONNECT, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_ERROR, "BFA_AEN_LPORT_DISCONNECT",
	 "Logical port lost fabric connectivity: WWN = %s Role = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | (BFA_LOG_S << BFA_LOG_ARG1) | 0), 2},

	{BFA_AEN_LPORT_NEW_PROP, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_LPORT_NEW_PROP",
	 "New virtual port created using proprietary interface: WWN = %s Role = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | (BFA_LOG_S << BFA_LOG_ARG1) | 0), 2},

	{BFA_AEN_LPORT_DELETE_PROP, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_LPORT_DELETE_PROP",
	 "Virtual port deleted using proprietary interface: WWN = %s Role = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | (BFA_LOG_S << BFA_LOG_ARG1) | 0), 2},

	{BFA_AEN_LPORT_NEW_STANDARD, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_LPORT_NEW_STANDARD",
	 "New virtual port created using standard interface: WWN = %s Role = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | (BFA_LOG_S << BFA_LOG_ARG1) | 0), 2},

	{BFA_AEN_LPORT_DELETE_STANDARD, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_LPORT_DELETE_STANDARD",
	 "Virtual port deleted using standard interface: WWN = %s Role = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | (BFA_LOG_S << BFA_LOG_ARG1) | 0), 2},

	{BFA_AEN_LPORT_NPIV_DUP_WWN, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_WARNING, "BFA_AEN_LPORT_NPIV_DUP_WWN",
	 "Virtual port login failed. Duplicate WWN = %s reported by fabric.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_AEN_LPORT_NPIV_FABRIC_MAX, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_WARNING, "BFA_AEN_LPORT_NPIV_FABRIC_MAX",
	 "Virtual port (WWN = %s) login failed. Max NPIV ports already exist in fabric/fport.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_AEN_LPORT_NPIV_UNKNOWN, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_WARNING, "BFA_AEN_LPORT_NPIV_UNKNOWN",
	 "Virtual port (WWN = %s) login failed.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_AEN_LPORT_NEW_SYNTH_FC, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_LPORT_NEW_SYNTH_FC",
	 "New virtual port created using synthetic fc interface: WWN = %s Role = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | (BFA_LOG_S << BFA_LOG_ARG1) | 0), 2},

	{BFA_AEN_LPORT_DELETE_SYNTH_FC, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_LPORT_DELETE_SYNTH_FC",
	 "Virtual port deleted using synthetic fc interface: WWN = %s Role = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | (BFA_LOG_S << BFA_LOG_ARG1) | 0), 2},




/* messages define for BFA_AEN_CAT_PORT Module */
	{BFA_AEN_PORT_ONLINE, BFA_LOG_ATTR_NONE, BFA_LOG_INFO,
	 "BFA_AEN_PORT_ONLINE",
	 "Base port online: WWN = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_AEN_PORT_OFFLINE, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_WARNING, "BFA_AEN_PORT_OFFLINE",
	 "Base port offline: WWN = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_AEN_PORT_RLIR, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_PORT_RLIR",
	 "RLIR event not supported.",
	 (0), 0},

	{BFA_AEN_PORT_SFP_INSERT, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_PORT_SFP_INSERT",
	 "New SFP found: WWN/MAC = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_AEN_PORT_SFP_REMOVE, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_WARNING, "BFA_AEN_PORT_SFP_REMOVE",
	 "SFP removed: WWN/MAC = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_AEN_PORT_SFP_POM, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_WARNING, "BFA_AEN_PORT_SFP_POM",
	 "SFP POM level to %s: WWN/MAC = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | (BFA_LOG_S << BFA_LOG_ARG1) | 0), 2},

	{BFA_AEN_PORT_ENABLE, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_PORT_ENABLE",
	 "Base port enabled: WWN = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_AEN_PORT_DISABLE, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_PORT_DISABLE",
	 "Base port disabled: WWN = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_AEN_PORT_AUTH_ON, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_PORT_AUTH_ON",
	 "Authentication successful for base port: WWN = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_AEN_PORT_AUTH_OFF, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_ERROR, "BFA_AEN_PORT_AUTH_OFF",
	 "Authentication unsuccessful for base port: WWN = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_AEN_PORT_DISCONNECT, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_ERROR, "BFA_AEN_PORT_DISCONNECT",
	 "Base port (WWN = %s) lost fabric connectivity.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_AEN_PORT_QOS_NEG, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_WARNING, "BFA_AEN_PORT_QOS_NEG",
	 "QOS negotiation failed for base port: WWN = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_AEN_PORT_FABRIC_NAME_CHANGE,
	 BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG, BFA_LOG_WARNING,
	 "BFA_AEN_PORT_FABRIC_NAME_CHANGE",
	 "Base port WWN = %s Fabric WWN = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | (BFA_LOG_S << BFA_LOG_ARG1) | 0), 2},

	{BFA_AEN_PORT_SFP_ACCESS_ERROR, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_WARNING, "BFA_AEN_PORT_SFP_ACCESS_ERROR",
	 "SFP access error: WWN/MAC = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_AEN_PORT_SFP_UNSUPPORT, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_WARNING, "BFA_AEN_PORT_SFP_UNSUPPORT",
	 "Unsupported SFP found: WWN/MAC = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},




/* messages define for BFA_AEN_CAT_RPORT Module */
	{BFA_AEN_RPORT_ONLINE, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_RPORT_ONLINE",
	 "Remote port (WWN = %s) online for logical port (WWN = %s).",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | (BFA_LOG_S << BFA_LOG_ARG1) | 0), 2},

	{BFA_AEN_RPORT_OFFLINE, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_RPORT_OFFLINE",
	 "Remote port (WWN = %s) offlined by logical port (WWN = %s).",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | (BFA_LOG_S << BFA_LOG_ARG1) | 0), 2},

	{BFA_AEN_RPORT_DISCONNECT, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_ERROR, "BFA_AEN_RPORT_DISCONNECT",
	 "Remote port (WWN = %s) connectivity lost for logical port (WWN = %s).",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | (BFA_LOG_S << BFA_LOG_ARG1) | 0), 2},

	{BFA_AEN_RPORT_QOS_PRIO, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_RPORT_QOS_PRIO",
	 "QOS priority changed to %s: RPWWN = %s and LPWWN = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | (BFA_LOG_S << BFA_LOG_ARG1) |
	  (BFA_LOG_S << BFA_LOG_ARG2) | 0), 3},

	{BFA_AEN_RPORT_QOS_FLOWID, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_RPORT_QOS_FLOWID",
	 "QOS flow ID changed to %d: RPWWN = %s and LPWWN = %s.",
	 ((BFA_LOG_D << BFA_LOG_ARG0) | (BFA_LOG_S << BFA_LOG_ARG1) |
	  (BFA_LOG_S << BFA_LOG_ARG2) | 0), 3},




/* messages define for BFA_AEN_CAT_TEAM Module */
	{BFA_AEN_TEAM_VLAN_ADD, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_TEAM_VLAN_ADD",
	 "New VLAN id = %d added/enabled on mac = %s.",
	 ((BFA_LOG_D << BFA_LOG_ARG0) | (BFA_LOG_S << BFA_LOG_ARG1) | 0), 2},

	{BFA_AEN_TEAM_VLAN_REMOVE, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_TEAM_VLAN_REMOVE",
	 "New VLAN id = %d removed/disabled from mac = %s.",
	 ((BFA_LOG_D << BFA_LOG_ARG0) | (BFA_LOG_S << BFA_LOG_ARG1) | 0), 2},

	{BFA_AEN_TEAM_CREATE, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_TEAM_CREATE",
	 "New team mac = %s created/enabled.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_AEN_TEAM_DELETE, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_TEAM_DELETE",
	 "Team mac = %s deleted/disabled.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_AEN_TEAM_PORT_ADD, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_TEAM_PORT_ADD",
	 "A port with mac = %s is added to team mac = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | (BFA_LOG_S << BFA_LOG_ARG1) | 0), 2},

	{BFA_AEN_TEAM_PORT_REMOVE, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_TEAM_PORT_REMOVE",
	 "A port with mac = %s is removed from team mac = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | (BFA_LOG_S << BFA_LOG_ARG1) | 0), 2},

	{BFA_AEN_TEAM_ACTIVE, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_WARNING, "BFA_AEN_TEAM_ACTIVE",
	 "Active port has changed in team mac = %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},




/* messages define for BFA_AEN_CAT_VLAN Module */
	{BFA_AEN_VLAN_ADD, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG, BFA_LOG_INFO,
	 "BFA_AEN_VLAN_ADD",
	 "New VLAN id = %d added/enabled on port mac = %s.",
	 ((BFA_LOG_D << BFA_LOG_ARG0) | (BFA_LOG_S << BFA_LOG_ARG1) | 0), 2},

	{BFA_AEN_VLAN_REMOVE, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "BFA_AEN_VLAN_REMOVE",
	 "New VLAN id = %d removed/disabled from port mac = %s.",
	 ((BFA_LOG_D << BFA_LOG_ARG0) | (BFA_LOG_S << BFA_LOG_ARG1) | 0), 2},




/* messages define for FCS Module */
	{BFA_LOG_FCS_FABRIC_NOSWITCH, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "FCS_FABRIC_NOSWITCH",
	 "No switched fabric presence is detected.",
	 (0), 0},

	{BFA_LOG_FCS_FABRIC_ISOLATED, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "FCS_FABRIC_ISOLATED",
	 "Port is isolated due to VF_ID mismatch. PWWN: %s Port VF_ID: %04x switch port VF_ID: %04x.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | (BFA_LOG_X << BFA_LOG_ARG1) |
	  (BFA_LOG_X << BFA_LOG_ARG2) | 0), 3},




/* messages define for HAL Module */
	{BFA_LOG_HAL_ASSERT, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_ERROR, "HAL_ASSERT",
	 "Assertion failure: %s:%d: %s",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | (BFA_LOG_D << BFA_LOG_ARG1) |
	  (BFA_LOG_S << BFA_LOG_ARG2) | 0), 3},

	{BFA_LOG_HAL_HEARTBEAT_FAILURE, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_CRITICAL, "HAL_HEARTBEAT_FAILURE",
	 "Firmware heartbeat failure at %d",
	 ((BFA_LOG_D << BFA_LOG_ARG0) | 0), 1},

	{BFA_LOG_HAL_FCPIM_PARM_INVALID, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "HAL_FCPIM_PARM_INVALID",
	 "Driver configuration %s value %d is invalid. Value should be within %d and %d.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | (BFA_LOG_D << BFA_LOG_ARG1) |
	  (BFA_LOG_D << BFA_LOG_ARG2) | (BFA_LOG_D << BFA_LOG_ARG3) | 0), 4},

	{BFA_LOG_HAL_SM_ASSERT, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_ERROR, "HAL_SM_ASSERT",
	 "SM Assertion failure: %s:%d: event = %d",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | (BFA_LOG_D << BFA_LOG_ARG1) |
	  (BFA_LOG_D << BFA_LOG_ARG2) | 0), 3},

	{BFA_LOG_HAL_DRIVER_ERROR, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "HAL_DRIVER_ERROR",
	 "%s",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_LOG_HAL_DRIVER_CONFIG_ERROR,
	 BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG, BFA_LOG_INFO,
	 "HAL_DRIVER_CONFIG_ERROR",
	 "%s",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_LOG_HAL_MBOX_ERROR, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "HAL_MBOX_ERROR",
	 "%s",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},




/* messages define for LINUX Module */
	{BFA_LOG_LINUX_DEVICE_CLAIMED, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "LINUX_DEVICE_CLAIMED",
	 "bfa device at %s claimed.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_LOG_LINUX_HASH_INIT_FAILED, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "LINUX_HASH_INIT_FAILED",
	 "Hash table initialization failure for the port %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_LOG_LINUX_SYSFS_FAILED, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "LINUX_SYSFS_FAILED",
	 "sysfs file creation failure for the port %s.",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_LOG_LINUX_MEM_ALLOC_FAILED, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "LINUX_MEM_ALLOC_FAILED",
	 "Memory allocation failed: %s.  ",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_LOG_LINUX_DRIVER_REGISTRATION_FAILED,
	 BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG, BFA_LOG_INFO,
	 "LINUX_DRIVER_REGISTRATION_FAILED",
	 "%s.  ",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_LOG_LINUX_ITNIM_FREE, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "LINUX_ITNIM_FREE",
	 "scsi%d: FCID: %s WWPN: %s",
	 ((BFA_LOG_D << BFA_LOG_ARG0) | (BFA_LOG_S << BFA_LOG_ARG1) |
	  (BFA_LOG_S << BFA_LOG_ARG2) | 0), 3},

	{BFA_LOG_LINUX_ITNIM_ONLINE, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "LINUX_ITNIM_ONLINE",
	 "Target: %d:0:%d FCID: %s WWPN: %s",
	 ((BFA_LOG_D << BFA_LOG_ARG0) | (BFA_LOG_D << BFA_LOG_ARG1) |
	  (BFA_LOG_S << BFA_LOG_ARG2) | (BFA_LOG_S << BFA_LOG_ARG3) | 0), 4},

	{BFA_LOG_LINUX_ITNIM_OFFLINE, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "LINUX_ITNIM_OFFLINE",
	 "Target: %d:0:%d FCID: %s WWPN: %s",
	 ((BFA_LOG_D << BFA_LOG_ARG0) | (BFA_LOG_D << BFA_LOG_ARG1) |
	  (BFA_LOG_S << BFA_LOG_ARG2) | (BFA_LOG_S << BFA_LOG_ARG3) | 0), 4},

	{BFA_LOG_LINUX_SCSI_HOST_FREE, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "LINUX_SCSI_HOST_FREE",
	 "Free scsi%d",
	 ((BFA_LOG_D << BFA_LOG_ARG0) | 0), 1},

	{BFA_LOG_LINUX_SCSI_ABORT, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "LINUX_SCSI_ABORT",
	 "scsi%d: abort cmnd %p iotag %x",
	 ((BFA_LOG_D << BFA_LOG_ARG0) | (BFA_LOG_P << BFA_LOG_ARG1) |
	  (BFA_LOG_X << BFA_LOG_ARG2) | 0), 3},

	{BFA_LOG_LINUX_SCSI_ABORT_COMP, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "LINUX_SCSI_ABORT_COMP",
	 "scsi%d: complete abort 0x%p iotag 0x%x",
	 ((BFA_LOG_D << BFA_LOG_ARG0) | (BFA_LOG_P << BFA_LOG_ARG1) |
	  (BFA_LOG_X << BFA_LOG_ARG2) | 0), 3},

	{BFA_LOG_LINUX_DRIVER_CONFIG_ERROR,
	 BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG, BFA_LOG_INFO,
	 "LINUX_DRIVER_CONFIG_ERROR",
	 "%s",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_LOG_LINUX_BNA_STATE_MACHINE,
	 BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG, BFA_LOG_INFO,
	 "LINUX_BNA_STATE_MACHINE",
	 "%s",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_LOG_LINUX_IOC_ERROR, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "LINUX_IOC_ERROR",
	 "%s",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_LOG_LINUX_RESOURCE_ALLOC_ERROR,
	 BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG, BFA_LOG_INFO,
	 "LINUX_RESOURCE_ALLOC_ERROR",
	 "%s",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_LOG_LINUX_RING_BUFFER_ERROR,
	 BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG, BFA_LOG_INFO,
	 "LINUX_RING_BUFFER_ERROR",
	 "%s",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_LOG_LINUX_DRIVER_ERROR, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_ERROR, "LINUX_DRIVER_ERROR",
	 "%s",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_LOG_LINUX_DRIVER_INFO, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "LINUX_DRIVER_INFO",
	 "%s",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_LOG_LINUX_DRIVER_DIAG, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "LINUX_DRIVER_DIAG",
	 "%s",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_LOG_LINUX_DRIVER_AEN, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "LINUX_DRIVER_AEN",
	 "%s",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},




/* messages define for SOLARIS Module */
	{BFA_LOG_SOLARIS_DRIVER_ERROR, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_ERROR, "SOLARIS_DRIVER_ERROR",
	 "%s",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_LOG_SOLARIS_DRIVER_WARN, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_WARNING, "SOLARIS_DRIVER_WARN",
	 "%s",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_LOG_SOLARIS_DRIVER_INFO, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "SOLARIS_DRIVER_INFO",
	 "%s",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},

	{BFA_LOG_SOLARIS_DRIVER_DIAG, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "SOLARIS_DRIVER_DIAG",
	 "%s",
	 ((BFA_LOG_S << BFA_LOG_ARG0) | 0), 1},




/* messages define for WDRV Module */
	{BFA_LOG_WDRV_IOC_INIT_ERROR, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "WDRV_IOC_INIT_ERROR",
	 "IOC initialization has failed.",
	 (0), 0},

	{BFA_LOG_WDRV_IOC_INTERNAL_ERROR,
	 BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG, BFA_LOG_INFO,
	 "WDRV_IOC_INTERNAL_ERROR",
	 "IOC internal error.  ",
	 (0), 0},

	{BFA_LOG_WDRV_IOC_START_ERROR, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "WDRV_IOC_START_ERROR",
	 "IOC could not be started.  ",
	 (0), 0},

	{BFA_LOG_WDRV_IOC_STOP_ERROR, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_INFO, "WDRV_IOC_STOP_ERROR",
	 "IOC could not be stopped.  ",
	 (0), 0},

	{BFA_LOG_WDRV_INSUFFICIENT_RESOURCES,
	 BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG, BFA_LOG_INFO,
	 "WDRV_INSUFFICIENT_RESOURCES",
	 "Insufficient memory.  ",
	 (0), 0},

	{BFA_LOG_WDRV_BASE_ADDRESS_MAP_ERROR,
	 BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG, BFA_LOG_INFO,
	 "WDRV_BASE_ADDRESS_MAP_ERROR",
	 "Unable to map the IOC onto the system address space.  ",
	 (0), 0},

	{BFA_LOG_WDRV_LUN_RESET, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_WARNING, "WDRV_LUN_RESET",
	 "Logical unit reset was performed upon request. Dump data contains additional details.  ",
	 (0), 0},

	{BFA_LOG_WDRV_DEVICE_RESET, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_WARNING, "WDRV_DEVICE_RESET",
	 "Target reset was performed upon request. Dump data contains additional details.  ",
	 (0), 0},

	{BFA_LOG_WDRV_BUS_RESET, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_WARNING, "WDRV_BUS_RESET",
	 "Bus reset was performed upon request. Dump data contains additional details.  ",
	 (0), 0},

	{BFA_LOG_WDRV_LUN_TIMEOUT, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_WARNING, "WDRV_LUN_TIMEOUT",
	 "Logical unit %d (target WWN: %s) did not respond within the timeout period of %d seconds. ",
	 ((BFA_LOG_D << BFA_LOG_ARG0) | (BFA_LOG_S << BFA_LOG_ARG1) |
	  (BFA_LOG_D << BFA_LOG_ARG2) | 0), 3},

	{BFA_LOG_WDRV_INITIATOR_TIMEOUT, BFA_LOG_ATTR_NONE | BFA_LOG_ATTR_LOG,
	 BFA_LOG_WARNING, "WDRV_INITIATOR_TIMEOUT",
	 "Failed to submit the request to logical unit %d (target WWN: %s) within the timeout period of %d seconds. ",
	 ((BFA_LOG_D << BFA_LOG_ARG0) | (BFA_LOG_S << BFA_LOG_ARG1) |
	  (BFA_LOG_D << BFA_LOG_ARG2) | 0), 3},


	{0, 0, 0, "", "", 0, 0},
};
