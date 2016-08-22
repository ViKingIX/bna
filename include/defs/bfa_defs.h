/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */
#ifndef __BFA_DEFS_H__
#define __BFA_DEFS_H__

#include <protocol/bfa_fc.h>
#include <protocol/sfp.h>
#include <defs/bfa_defs_mfg_comm.h>

/**
 * API status return values
 *
 * NOTE: The error msgs are auto generated from the comments. Only singe line
 * comments are supported
 */
enum bfa_status {
	BFA_STATUS_OK 		= 0,	/*!< Success */
	BFA_STATUS_FAILED 	= 1,	/*!< Operation failed */
	BFA_STATUS_EINVAL 	= 2,	/*!< Invalid params Check input parameters */
	BFA_STATUS_ENOMEM 	= 3,	/*!< Out of resources */
	BFA_STATUS_ENOSYS 	= 4,	/*!< Function not implemented */
	BFA_STATUS_ETIMER 	= 5,	/*!< Timer expired - Retry, if persists, contact support */
	BFA_STATUS_EPROTOCOL 	= 6,	/*!< Protocol error */
	BFA_STATUS_ENOFCPORTS 	= 7,	/*!< No FC ports resources */
	BFA_STATUS_NOFLASH 	= 8,	/*!< Flash not present */
	BFA_STATUS_BADFLASH 	= 9,	/*!< Flash is bad */
	BFA_STATUS_SFP_UNSUPP 	= 10,	/*!< Unsupported SFP - Replace SFP */
	BFA_STATUS_UNKNOWN_VFID = 11,	/*!< VF_ID not found */
	BFA_STATUS_DATACORRUPTED = 12,	/*!< Diag returned data corrupted contact support */
	BFA_STATUS_DEVBUSY 	= 13,	/*!< Device busy - Retry operation */
	BFA_STATUS_ABORTED 	= 14,	/*!< Operation aborted */
	BFA_STATUS_NODEV 	= 15,	/*!< Dev is not present */
	BFA_STATUS_HDMA_FAILED 	= 16,	/*!< Host dma failed contact support */
	BFA_STATUS_FLASH_BAD_LEN = 17,	/*!< Flash bad length */
	BFA_STATUS_UNKNOWN_LWWN = 18,	/*!< LPORT PWWN not found */
	BFA_STATUS_UNKNOWN_RWWN = 19,	/*!< RPORT PWWN not found */
	BFA_STATUS_FCPT_LS_RJT 	= 20,	/*!< Got LS_RJT for FC Pass through Req */
	BFA_STATUS_VPORT_EXISTS = 21,	/*!< VPORT already exists */
	BFA_STATUS_VPORT_MAX 	= 22,	/*!< Reached max VPORT supported limit */
	BFA_STATUS_UNSUPP_SPEED = 23,	/*!< Invalid Speed Check speed setting */
	BFA_STATUS_INVLD_DFSZ 	= 24,	/*!< Invalid Max data field size */
	BFA_STATUS_CNFG_FAILED 	= 25,	/*!< Setting can not be persisted */
	BFA_STATUS_CMD_NOTSUPP 	= 26,	/*!< Command/API not supported */
	BFA_STATUS_NO_ADAPTER 	= 27,	/*!< No QLogic BR-series Adapter Found */
	BFA_STATUS_LINKDOWN 	= 28,	/*!< Link is down - Check or replace SFP/cable */
	BFA_STATUS_FABRIC_RJT 	= 29,	/*!< Reject from attached fabric */
	BFA_STATUS_UNKNOWN_VWWN = 30,	/*!< VPORT PWWN not found */
	BFA_STATUS_NSLOGIN_FAILED = 31,	/*!< Nameserver login failed */
	BFA_STATUS_NO_RPORTS 	= 32,	/*!< No remote ports found */
	BFA_STATUS_NSQUERY_FAILED = 33,	/*!< Nameserver query failed */
	BFA_STATUS_PORT_OFFLINE = 34,	/*!< Port is not online */
	BFA_STATUS_RPORT_OFFLINE = 35,	/*!< RPORT is not online */
	BFA_STATUS_TGTOPEN_FAILED = 36,	/*!< Remote SCSI target open failed */
	BFA_STATUS_BAD_LUNS 	= 37,	/*!< No valid LUNs found */
	BFA_STATUS_IO_FAILURE 	= 38,	/*!< SCSI target IO failure */
	BFA_STATUS_NO_FABRIC 	= 39,	/*!< No switched fabric present */
	BFA_STATUS_EBADF 	= 40,	/*!< Bad file descriptor */
	BFA_STATUS_EINTR 	= 41,	/*!< A signal was caught during ioctl */
	BFA_STATUS_EIO 		= 42,	/*!< I/O error */
	BFA_STATUS_ENOTTY 	= 43,	/*!< Inappropriate I/O control operation */
	BFA_STATUS_ENXIO 	= 44,	/*!< No such device or address */
	BFA_STATUS_EFOPEN 	= 45,	/*!< Failed to open file */
	BFA_STATUS_VPORT_WWN_BP = 46,	/*!< WWN is same as base port's WWN */
	BFA_STATUS_PORT_NOT_DISABLED = 47, /*!< Port not disabled, disable port first */
	BFA_STATUS_BADFRMHDR 	= 48,	/*!< Bad frame header */
	BFA_STATUS_BADFRMSZ 	= 49,	/*!< Bad frame size check and replace SFP/cable */
	BFA_STATUS_MISSINGFRM 	= 50,	/*!< Missing frame check and replace SFP/cable or for Mezz card check and replace pass through module */
	BFA_STATUS_LINKTIMEOUT 	= 51,	/*!< Link timeout check and replace SFP/cable */
	BFA_STATUS_NO_FCPIM_NEXUS = 52,	/*!< No FCP Nexus exists with the  rport */
	BFA_STATUS_CHECKSUM_FAIL = 53,	/*!< checksum failure */
	BFA_STATUS_GZME_FAILED 	= 54,	/*!< Get zone member query failed */
	BFA_STATUS_SCSISTART_REQD = 55,	/*!< SCSI disk require START command */
	BFA_STATUS_IOC_FAILURE 	= 56,	/*!< IOC failure - Retry, if persists contact support */
	BFA_STATUS_INVALID_WWN 	= 57,	/*!< Invalid WWN */
	BFA_STATUS_MISMATCH 	= 58,	/*!< Version mismatch */
	BFA_STATUS_IOC_ENABLED 	= 59,	/*!< IOC is already enabled */
	BFA_STATUS_ADAPTER_ENABLED = 60, /*!< Adapter is not disabled disable adapter first */
	BFA_STATUS_IOC_NON_OP 	= 61,	/*!< IOC is not operational. Enable IOC and if it still fails, contact support */
	BFA_STATUS_ADDR_MAP_FAILURE = 62, /*!< PCI base address not mapped in OS */
	BFA_STATUS_SAME_NAME 	= 63,	/*!< Name exists! use a different name */
	BFA_STATUS_PENDING      = 64,   /*!< API completes asynchronously */
	BFA_STATUS_8G_SPD	= 65,	/*!< Speed setting not valid for 8G HBA */
	BFA_STATUS_4G_SPD	= 66,	/*!< Speed setting not valid for 4G HBA */
	BFA_STATUS_AD_IS_ENABLE = 67,	/*!< Adapter is already enabled */
	BFA_STATUS_EINVAL_TOV 	= 68,	/*!< Invalid path failover TOV */
	BFA_STATUS_EINVAL_QDEPTH = 69,	/*!< Invalid queue depth value */
	BFA_STATUS_VERSION_FAIL = 70,	/*!< Application/Driver version mismatch */
	BFA_STATUS_DIAG_BUSY    = 71,	/*!< diag busy */
	BFA_STATUS_BEACON_ON	= 72,	/*!< Port Beacon already on */
	BFA_STATUS_BEACON_OFF	= 73,	/*!< Port Beacon already off */
	BFA_STATUS_LBEACON_ON   = 74,	/*!< Link End-to-End Beacon already on */
	BFA_STATUS_LBEACON_OFF	= 75,	/*!< Link End-to-End Beacon already off */
	BFA_STATUS_PORT_NOT_INITED = 76, /*!< Port not initialized */
	BFA_STATUS_RPSC_ENABLED = 77, /*!< Target has a valid speed */
	BFA_STATUS_ENOFSAVE 	= 78,	/*!< No saved firmware trace */
	BFA_STATUS_BAD_FILE		= 79,	/*!< Not a valid QLogic BR-series Boot Code file */
	BFA_STATUS_RLIM_EN		= 80,	/*!< Target rate limiting is already enabled */
	BFA_STATUS_RLIM_DIS		= 81,  /*!< Target rate limiting is already disabled */
	BFA_STATUS_IOC_DISABLED  = 82,   /*!< IOC is already disabled */
	BFA_STATUS_ADAPTER_DISABLED  = 83,   /*!< Adapter is already disabled */
	BFA_STATUS_BIOS_DISABLED  = 84,   /*!< Bios is already disabled */
	BFA_STATUS_AUTH_ENABLED  = 85,   /*!< Authentication is already enabled */
	BFA_STATUS_AUTH_DISABLED  = 86,   /*!< Authentication is already disabled */
	BFA_STATUS_ERROR_TRL_ENABLED  = 87,   /*!< Target rate limiting is enabled */
	BFA_STATUS_ERROR_QOS_ENABLED  = 88,   /*!< QoS is enabled */
	BFA_STATUS_NO_SFP_DEV = 89,     /*!< No SFP device check or replace SFP */
	BFA_STATUS_MEMTEST_FAILED = 90,	/*!< Memory test failed contact support */
	BFA_STATUS_INVALID_DEVID = 91,	/*!< Invalid device id provided */
	BFA_STATUS_QOS_ENABLED = 92, /*!< QOS is already enabled */
	BFA_STATUS_QOS_DISABLED = 93, /*!< QOS is already disabled */
	BFA_STATUS_INCORRECT_DRV_CONFIG = 94, /*!< Check configuration key/value pair */
	BFA_STATUS_REG_FAIL = 95, /*!< Can't read windows registry */
	BFA_STATUS_IM_INV_CODE = 96, /*!< Invalid IOCTL code */
	BFA_STATUS_IM_INV_VLAN = 97, /*!< Invalid VLAN ID */
	BFA_STATUS_IM_INV_ADAPT_NAME = 98, /*!< Invalid port name */
	BFA_STATUS_IM_LOW_RESOURCES = 99, /*!< Memory allocation failure in driver */
	BFA_STATUS_IM_VLANID_IS_PVID = 100, /*!< Given VLAN id same as PVID */
	BFA_STATUS_IM_VLANID_EXISTS = 101, /*!< Given VLAN id already exists */
	BFA_STATUS_IM_FW_UPDATE_FAIL = 102, /*!< Updating firmware with new VLAN ID failed */
	BFA_STATUS_PORTLOG_ENABLED = 103, /*!< Port Log is already enabled */
	BFA_STATUS_PORTLOG_DISABLED = 104, /*!< Port Log is already disabled */
	BFA_STATUS_FILE_NOT_FOUND = 105, /*!< Specified file could not be found */
	BFA_STATUS_QOS_FC_ONLY = 106, /*!< QOS can be enabled for FC mode only */
	BFA_STATUS_RLIM_FC_ONLY = 107, /*!< RATELIM can be enabled for FC mode only */
	BFA_STATUS_CT_SPD = 108, /*!< Invalid speed selection for Catapult. */
	BFA_STATUS_LEDTEST_OP = 109, /*!< LED test is operating */
	BFA_STATUS_ADMIN_NOT_DN = 110, /*!< eth port is not at down state, please bring down first */
	BFA_STATUS_10G_SPD = 111, /*!< Speed setting not valid for 10G CNA */
	BFA_STATUS_IM_INV_TEAM_NAME = 112, /*!< Invalid team name */
	BFA_STATUS_IM_DUP_TEAM_NAME = 113, /*!< Given team name already exists */
	BFA_STATUS_IM_ADAPT_ALREADY_IN_TEAM = 114, /*!< Given port is part of another team or VLANS exist */
	BFA_STATUS_IM_ADAPT_HAS_VLANS = 115, /*!< Port has VLANs configured. Delete all VLANs to become part of team */
	BFA_STATUS_IM_PVID_MISMATCH = 116, /*!< Mismatching PVIDs configured for ports */
	BFA_STATUS_IM_LINK_SPEED_MISMATCH = 117, /*!< Mismatching link speeds configured for ports */
	BFA_STATUS_IM_MTU_MISMATCH = 118, /*!< Mismatching MTUs configured for ports */
	BFA_STATUS_IM_RSS_MISMATCH = 119, /*!< Mismatching RSS parameters configured for ports */
	BFA_STATUS_IM_HDS_MISMATCH = 120, /*!< Mismatching HDS parameters configured for ports */
	BFA_STATUS_IM_OFFLOAD_MISMATCH = 121, /*!< Mismatching offload parameters configured for ports */
	BFA_STATUS_IM_PORT_PARAMS = 122, /*!< Error setting port parameters */
	BFA_STATUS_IM_PORT_NOT_IN_TEAM = 123, /*!< Port is not part of team */
	BFA_STATUS_IM_CANNOT_REM_PRI = 124, /*!< Primary port cannot be removed. Change primary before removing */
	BFA_STATUS_IM_MAX_PORTS_REACHED = 125, /*!< Exceeding maximum ports per team */
	BFA_STATUS_IM_LAST_PORT_DELETE = 126, /*!< Last port in team cannot be removed. Delete team instead */
	BFA_STATUS_IM_NO_DRIVER = 127, /*!< IM driver is not installed */
	BFA_STATUS_IM_MAX_VLANS_REACHED = 128, /*!< Exceeding maximum VLANs per port */
	BFA_STATUS_TOMCAT_SPD_NOT_ALLOWED = 129, /*!< Bios speed config not allowed for CNA */
	BFA_STATUS_NO_MINPORT_DRIVER = 130, /*!< Miniport driver is not loaded */
	BFA_STATUS_CARD_TYPE_MISMATCH = 131, /*!< Card type mismatch */
	BFA_STATUS_BAD_ASICBLK = 132, /*!< Bad ASIC block */
	BFA_STATUS_NO_DRIVER = 133, /*!< QLogic BR-series adapter/driver not installed or loaded */
	BFA_STATUS_INVALID_MAC = 134, /*!< Invalid MAC address */
	BFA_STATUS_IM_NO_VLAN = 135, /*!< No VLANs configured on the port */
	BFA_STATUS_IM_ETH_LB_FAILED = 136, /*!< Ethernet loopback test failed */
	BFA_STATUS_IM_PVID_REMOVE = 137, /*!< Cannot remove port VLAN (PVID) */
	BFA_STATUS_IM_PVID_EDIT = 138, /*!< Cannot edit port VLAN (PVID) */
	BFA_STATUS_CNA_NO_BOOT = 139, /*!< Boot upload not allowed for CNA */
	BFA_STATUS_IM_PVID_NON_ZERO = 140, /*!< Port VLAN ID (PVID) is Set to Non-Zero Value */
	BFA_STATUS_IM_INETCFG_LOCK_FAILED = 141, /*!< Acquiring Network Subsytem Lock Failed. Please try after some time */
	BFA_STATUS_IM_GET_INETCFG_FAILED = 142, /*!< Acquiring Network Subsytem handle Failed. Please try after some time */
	BFA_STATUS_IM_NOT_BOUND = 143, /*!< IM driver is not active */
	BFA_STATUS_INSUFFICIENT_PERMS = 144, /*!< User doesn't have sufficient permissions to execute the BCU application */
	BFA_STATUS_IM_INV_VLAN_NAME = 145, /*!< Invalid/Reserved VLAN name string. The name is not allowed for the normal VLANs */
	BFA_STATUS_CMD_NOTSUPP_CNA = 146, /*!< Command not supported for CNA */
	BFA_STATUS_IM_PASSTHRU_EDIT = 147, /*!< Can not edit pass through VLAN ID */
	BFA_STATUS_IM_BIND_FAILED = 148, /*!< IM Driver bind operation failed */
	BFA_STATUS_IM_UNBIND_FAILED = 149, /*!< IM Driver unbind operation failed */
	BFA_STATUS_IM_PORT_IN_TEAM = 150, /*!< Port is already part of the team */
	BFA_STATUS_IM_VLAN_NOT_FOUND = 151, /*!< VLAN ID doesn't exist */
	BFA_STATUS_IM_TEAM_NOT_FOUND = 152, /*!< Team doesn't exist */
	BFA_STATUS_IM_TEAM_CFG_NOT_ALLOWED = 153, /*!< Given settings are not allowed for the current Teaming mode */
	BFA_STATUS_PBC = 154, /*!< Operation not allowed for pre-boot configuration */
	BFA_STATUS_DEVID_MISSING = 155, /*!< Boot image is not for the adapter(s) installed */
	BFA_STATUS_BAD_FWCFG = 156, /*!< Bad firmware configuration */
	BFA_STATUS_CREATE_FILE = 157, /*!< Failed to create temporary file */
	BFA_STATUS_INVALID_VENDOR = 158, /*!< Invalid switch vendor */
	BFA_STATUS_SFP_NOT_READY = 159, /*!< SFP info is not ready. Retry */
	BFA_STATUS_FLASH_UNINIT = 160, /*!< Flash not initialized */
	BFA_STATUS_FLASH_EMPTY = 161, /*!< Flash is empty */
	BFA_STATUS_FLASH_CKFAIL = 162, /*!< Flash checksum failed */
	BFA_STATUS_TRUNK_UNSUPP_SINGLE_PORT = 163, /*!< Trunking is only supported on dual port FC HBAs */
	BFA_STATUS_TRUNK_ENABLED = 164, /*!< Trunk is already enabled on this adapter */
	BFA_STATUS_TRUNK_DISABLED  = 165, /*!< Trunking is disabled on the adapter */
	BFA_STATUS_TRUNK_ERROR_TRL_ENABLED = 166, /*!< Target rate limiting is enabled on either or both of the ports of adapter */
	BFA_STATUS_BOOT_CODE_DRV_UPDATED = 167, /*!< reboot -- -r is needed after boot code update or driver update */
	BFA_STATUS_BOOT_VERSION = 168, /*!< Boot code version not compatible with the driver installed */
	BFA_STATUS_CARDTYPE_MISSING = 169, /*!< Boot image is not for the adapter(s) installed */
	BFA_STATUS_INVALID_CARDTYPE = 170, /*!< Invalid card type provided */
	BFA_STATUS_NO_TOPOLOGY_FOR_CNA = 171, /*!< Topology command not applicable to CNA */
	BFA_STATUS_IM_VLAN_OVER_TEAM_DELETE_FAILED = 172, /*!< VLAN on Team Delete Operation Failed */
	BFA_STATUS_ETHBOOT_ENABLED  = 173,   /*!< Ethboot is already enabled */
	BFA_STATUS_ETHBOOT_DISABLED  = 174,   /*!< Ethboot is already disabled */
	BFA_STATUS_IOPROFILE_OFF = 175, /*!< IO profile OFF */
	BFA_STATUS_NO_PORT_INSTANCE = 176, /*!< Port instance is unavailable or disabled */
	BFA_STATUS_BOOT_CODE_TIMEDOUT = 177, /*!< Boot update has timed out, please try later */
	BFA_STATUS_NO_VPORT_LOCK = 178, /*!< Can not get the lock, vports will not be persisted */
	BFA_STATUS_VPORT_NO_CNFG = 179, /*!< Trying to delete a vport not persisted by the driver */
	BFA_STATUS_IM_MS_HYPERV_ENABLED = 180, /*!< Microsoft Hyper-V is enabled on this Port/VLAN.\nPlease remove this Port/VLAN from Microsoft Hyper-V configuration\nusing Microsoft Hyper-V tools and then try this operation again. */
	BFA_STATUS_IM_TEAM_PASSTHRU_DELETE = 181, /*!< Reserved VLAN in team cannot be removed. Delete team instead. */
	BFA_STATUS_BAD_BOOT_CODE = 182, /*!< Boot code is corrupted, please retry  with a good boot code image, before rebooting the system. */
	BFA_STATUS_PHY_NOT_PRESENT = 183, /*!< PHY module not present */
	BFA_STATUS_NEED_PORTS_DISABLE = 184, /*!< Trunk is enabled on this adapter. Disable trunking or disable all adapter ports. */
	BFA_STATUS_16G_SPD	= 185,	/*!< Speed setting not valid for 16G HBA */
	BFA_STATUS_IM_AUTO_RECOVERY_MISMATCH = 186, /*!< Mismatching Auto Recovery parameters configured for ports */
	BFA_STATUS_IM_FLOW_CTRL_MISMATCH = 187, /*!< Mismatching Flow Control parameters configured for ports */
	BFA_STATUS_IM_INTR_MODERATION_MISMATCH = 188, /*!< Mismatching Interrupt Moderation parameters configured for ports */
	BFA_STATUS_IM_JUMBO_PACKET_MISMATCH = 189, /*!< Mismatching Jumbo Packet parameters configured for ports */
	BFA_STATUS_IM_NW_ADDR_MISMATCH = 190, /*!< Mismatching Locally Administered Address parameters configured for ports */
	BFA_STATUS_IM_VMQ_MISMATCH = 191, /*!< Mismatching VmQ parameters configured for ports */
	BFA_STATUS_FEATURE_NOT_SUPPORTED = 192, /*!< This feature is not supported on this Model or Mode of the Adapter */
	BFA_STATUS_ENTRY_EXISTS = 193, /*!< This entry already exists */
	BFA_STATUS_ENTRY_NOT_EXISTS = 194, /*!< This entry does not exist */
	BFA_STATUS_NO_CHANGE = 195, /*!< This feature is already in that state */
	BFA_STATUS_DISABLED = 196, /*!< This feature is in disabled state */
	BFA_STATUS_FAA_ENABLED = 197, /*!< FAA is already enabled */
	BFA_STATUS_FAA_DISABLED = 198, /*!< FAA is already disabled */
	BFA_STATUS_FAA_ACQUIRED = 199, /*!< FAA is already acquired */
	BFA_STATUS_FAA_ACQ_ADDR = 200, /*!< Acquiring addr from the fabric. Please check cable connection and configuration on the switch */
	BFA_STATUS_BBCR_FC_ONLY = 201, /*!< BBCredit Recovery is supported for FC mode only */
	BFA_STATUS_ERROR_FAA_OVER_TRUNK = 202, /*!< FAA is enabled on either/both of the adapter ports. Disable FAA reboot and try this operation */
	BFA_STATUS_ERROR_TRUNK_ENABLED = 203, /*!< Trunk is enabled on this adapter */
	BFA_STATUS_ERROR_FAA_OPER = 204, /*!< FAA is operational on either/both of the adapter ports. Disable FAA on\nthe switch port, reboot the host and try this operation */
	BFA_STATUS_SINGLE_BEACON_ALLOWED = 205, /*!< Link beacon is currently ON at a different port on the switch.\nPlease turn it OFF first.\nOnly one switch port is allowed to beacon at a time. */
	BFA_STATUS_BEACON_NOT_SUPP_BY_SWITCH = 206, /*!< Link Beacon is not supported by the attached switch. */
	BFA_STATUS_FN0_DELETE = 207, /*!< Base function cannot be deleted/disabled */
	BFA_STATUS_INV_PCIFN = 208, /*!< Invalid PCI function for required operation */
	BFA_STATUS_PBIND_MAP_EXIST = 209, /*!< Persistent Binding already exist for another target */
	BFA_STATUS_PBIND_BOOT_LUN = 210, /*!< The binding is reserved of boot target */
	BFA_STATUS_PBIND_INVALID_RANGE = 211, /*!< The binding is out of range */
	BFA_STATUS_MAX_ENTRY_REACHED = 212, /*!< MAX entry reached */
	BFA_STATUS_MAXPF = 213,	/*!< Maximum number of PFs have been created */
	BFA_STATUS_TRUNK_UNSUPP_MEZZ = 214, /*!< Trunking is not supported on mezz cards */
	BFA_STATUS_TRUNK_UNSUPP_CNA = 215, /*!< Trunking is not supported on CNAs */
	BFA_STATUS_UNSUPP_MODE = 216, /*!< The intended mode is not supported on the adapter */
	BFA_STATUS_IM_TEAM_MS_HYPERV_ENABLED = 217, /*!< Microsoft Hyper-V is enabled on the Team.\nPlease remove Microsoft Hyper-V configuration\nusing Microsoft Hyper-V tools from the team, and then try this operation again. */
	BFA_STATUS_IM_TEAM_VNIC_FROM_SAME_PORT = 218, /*!< Given ports cannot be from the same Physical port. Please try passing one port from each physical port. */
	BFA_STATUS_LB_INT_HBA_ONLY = 219, /*!< Internal loopback is supported for a port in HBA mode only. */
	BFA_STATUS_UNSUPP_SFP_SPEED = 220, /*!< Speed setting is not supported by SFP */
	BFA_STATUS_BUFFER_SIZE_SMALL = 221, /*!< Buffer size is smaller than the required one. */
	BFA_STATUS_IM_TEAM_LIST_WARNING = 222, /*!< All existing team(s)/vlans, if any, are in disabled state.\nPlease enable any of the teams/vlans to list them. */
	BFA_STATUS_IOC0_DISABLED = 223,	/*!< First pcifn on port 0 of adapter is disabled. Please enable it first. */
	BFA_STATUS_UNSUPP_VENDOR = 224, /*!< Unsupported VPD vendor. */
	BFA_STATUS_CORR_VPD_DATA = 225, /*!< Corrupted pci data. */
	BFA_STATUS_VPD_CHK_SUM_ERR = 226, /*!< Vpd checksum error. */
	BFA_STATUS_LR_STR_TAG_ERR = 227, /*!< Incorrect LR_STR_TAG. */
	BFA_STATUS_LR_VPDR_TAG_ERR = 228, /*!< Incorrect LR_VPDR_TAG. */
	BFA_STATUS_VPD_BRCD_HDR_ERR = 229, /*!< QLogic BR-series VPD header not found. */
	BFA_STATUS_TOPOLOGY_LOOP = 230, /*!< Topology is set to Loop */
	BFA_STATUS_LOOP_UNSUPP_MEZZ = 231, /*!< Loop topology is not supported on mezz cards */
	BFA_STATUS_VPORT_RESERVED = 232, /*!< Cannot perform the operation as vport is reserved */
	BFA_STATUS_INVALID_BW = 233, /*!< Invalid bandwidth value */
	BFA_STATUS_QOS_BW_INVALID = 234, /*!< Invalid QOS bandwidth configuration */
	BFA_STATUS_DPORT_ENABLED = 235, /*!< D-port mode is enabled */
	BFA_STATUS_DPORT_DISABLED = 236, /*!< D-port mode is disabled */
	BFA_STATUS_ISCSIBOOT_ENABLED = 237, /*!< iScsi boot is already enabled */
	BFA_STATUS_ISCSIBOOT_DISABLED = 238, /*!< iScsi boot is already disabled */
	BFA_STATUS_CMD_NOTSUPP_MEZZ = 239, /*!< Command not supported for MEZZ card */
	BFA_STATUS_FRU_NOT_PRESENT = 240, /*!< fru module not present */
	BFA_STATUS_INVALID_FRU_VPD_FILE = 241, /*!< fru vpd file is invalid */
	BFA_STATUS_NWPART_NOT_ALLOWED = 242, /*!< Selected operation is not allowed in network partitioning mode */
	BFA_STATUS_DPORT_NO_SFP	= 243, /*!< SFP is not present.\n D-port will be enabled but it will be operational only after inserting a valid SFP. */
	BFA_STATUS_INVALID_BOOTOPTION	= 244, /*!< Invalid Boot option for selected topology */
	BFA_STATUS_DPORT_ERR = 245, /*!< D-port mode is enabled */
	BFA_STATUS_VNIC_MEM_OF_TEAM = 246, /*!< vNIC is member of a team or has vlans configured */
	BFA_STATUS_NWPART_NOT_SUPPORTED = 247, /*!< Network partition is supported in NIC/CNA mode only */
	BFA_STATUS_NWPART_ENABLED = 248, /*!< Network partition is already enabled */
	BFA_STATUS_NWPART_DISABLED = 249, /*!< Network partition is already disabled */
	BFA_STATUS_QOS_MISMATCH = 250, /*!< Mismatching QoS parameters */
	BFA_STATUS_TRL_MISMATCH = 251, /*!< Mismatching target rate limiting parameters */
	BFA_STATUS_SPEED_MISMATCH = 252, /*!< Mismatching speed parameters */
	BFA_STATUS_IM_PORT_HAS_TEAMS = 253, /*!< One/more vNIC(s) of the port is part of an lacp team or has teams configured */
	BFA_STATUS_DPORT_ENOSYS = 254, /*!< Switch has no D_Port functionality */
	BFA_STATUS_DPORT_CANT_PERF = 255, /*!< Switch port is not D_Port capable or D_Port is disabled */
	BFA_STATUS_DPORT_LOGICALERR = 256, /*!< Switch D_Port fail */
	BFA_STATUS_DPORT_SWBUSY = 257, /*!< Switch port busy */
	BFA_STATUS_ERR_BBCR_SPEED_UNSUPPORT = 258, /*!< BB credit recovery is supported at max port speed alone */
	BFA_STATUS_ERROR_BBCR_ENABLED  = 259, /*!< BB credit recovery is enabled */
	BFA_STATUS_INVALID_BBSCN = 260, /*!< Invalid BBSCN value. Valid range is [1-15] */
	BFA_STATUS_DDPORT_ERR = 261, /*!< Dynamic D_Port mode is active.\n To exit dynamic mode, disable D_Port on the remote port */
	BFA_STATUS_DPORT_SFPWRAP_ERR = 262, /*!< Clear e/o_wrap fail, check or replace SFP */
	BFA_STATUS_FCPIM_LUN_RANGE_INVALID = 263, /*!< LUN Range specifed is invalid */
	BFA_STATUS_BOOT_UPDATE_PORT1 = 264, /*!< Boot code update is not allowed on port 1 of 1867/1869 cards */
	BFA_STATUS_BBCR_CFG_NO_CHANGE = 265, /*!< BBCR is operational. Disable BBCR and try this operation again. */
	BFA_STATUS_TRS_ENABLED = 266, /*!< Target reset support is already enabled */
	BFA_STATUS_TRS_DISABLED = 267, /*!< Target reset support is already disabled */
	BFA_STATUS_DPORT_SW_NOTREADY = 268, /*!< Remote port is not ready to start dport test. Check remote port status. */
	BFA_STATUS_IM_RSS_QUE_MISMATCH = 269, /*!< Mismatching RSS Queue parameters configured for ports */
	BFA_STATUS_IM_RX_BUF_MISMATCH = 270, /*!< Mismatching Receive Buffers parameters configured for ports */
	BFA_STATUS_DPORT_INV_SFP	= 271, /*!< Invalid SFP for D-PORT mode. */
	BFA_STATUS_INVALID_WIN_LUN = 272, /*!< Supported LUN range on Windows is 0-254 */
	BFA_STATUS_DPORT_CMD_NOTSUPP	= 273, /*!< Dport is not supported by remote port */
	BFA_STATUS_IOPF_NOT_EXISTS = 274, /*!< There is no IO profile available */
	BFA_STATUS_NO_FABRIC_LOGIN = 275, /*!< Aport has done VVL/Dummy Login to fabric. */
	BFA_STATUS_PORT0_ONLY = 276, /*!< This configuration is allowed only on port 0 when trunking is enabled. */
	BFA_STATUS_MAX_VAL		/*!< Unknown error code */
};
typedef enum bfa_status bfa_status_t;

enum bfa_eproto_status {
	BFA_EPROTO_BAD_ACCEPT = 0,
	BFA_EPROTO_UNKNOWN_RSP = 1
};
typedef enum bfa_eproto_status bfa_eproto_status_t;

enum bfa_boolean {
	BFA_FALSE = 0,
	BFA_TRUE  = 1
};
typedef enum bfa_boolean bfa_boolean_t;

#define BFA_STRING_32	32
#define BFA_VERSION_LEN 64
#define BFA_DATE_LEN	16

/**
 * ---------------------- adapter definitions ------------
 */

/**
 * BFA adapter level attributes.
 */
enum {
	BFA_ADAPTER_SERIAL_NUM_LEN = STRSZ(BFA_MFG_SERIALNUM_SIZE),
					/*
					 *!< adapter serial num length
					 */
	BFA_ADAPTER_MODEL_NAME_LEN  = 16,  /*!< model name length */
	BFA_ADAPTER_MODEL_DESCR_LEN = 128, /*!< model description length */
	BFA_ADAPTER_MFG_NAME_LEN    = 8,   /*!< manufacturer name length */
	BFA_ADAPTER_SYM_NAME_LEN    = 64,  /*!< adapter symbolic name length */
	BFA_ADAPTER_OS_TYPE_LEN	    = 64,  /*!< adapter os type length */
	BFA_ADAPTER_UUID_LEN		= 16,  /*!< adapter uuid length */
};

struct bfa_adapter_attr_s {
	char		manufacturer[BFA_ADAPTER_MFG_NAME_LEN];
	char		serial_num[BFA_ADAPTER_SERIAL_NUM_LEN];
	uint32_t	card_type;
	char		model[BFA_ADAPTER_MODEL_NAME_LEN];
	char		model_descr[BFA_ADAPTER_MODEL_DESCR_LEN];
	wwn_t		pwwn;
	char		node_symname[FC_SYMNAME_MAX];
	char		hw_ver[BFA_VERSION_LEN];
	char		fw_ver[BFA_VERSION_LEN];
	char		optrom_ver[BFA_VERSION_LEN];
	char		os_type[BFA_ADAPTER_OS_TYPE_LEN];
	struct bfa_mfg_vpd_s	vpd;
	struct mac_s	mac;

	uint8_t		nports;
	uint8_t		max_speed;
	uint8_t		prototype;
	char	        asic_rev;

	uint8_t		pcie_gen;
	uint8_t		pcie_lanes_orig;
	uint8_t		pcie_lanes;
	uint8_t	        cna_capable;

	uint8_t		is_mezz;
	uint8_t		trunk_capable;

	uint8_t		mfg_day;	/*!< manufacturing day */
	uint8_t		mfg_month;	/*!< manufacturing month */
	uint16_t	mfg_year;	/*!< manufacturing year */

	uint16_t	rsvd;
	uint8_t		uuid[BFA_ADAPTER_UUID_LEN];
};
typedef struct bfa_adapter_attr_s bfa_adapter_attr_t;

/**
 * BFA adapter level events
 * Arguments below are in BFAL context from Mgmt
 * BFA_PORT_AEN_ADD:        [in]: None     [out]: serial_num, pwwn, nports
 * BFA_PORT_AEN_REMOVE:     [in]: pwwn     [out]: serial_num, pwwn, nports
 */
enum bfa_adapter_aen_event {
	BFA_ADAPTER_AEN_ADD 	= 1,	/*!< New Adapter found event */
	BFA_ADAPTER_AEN_REMOVE 	= 2,	/*!< Adapter removed event */
};
typedef enum bfa_adapter_aen_event bfa_adapter_aen_event_t;

struct bfa_adapter_aen_data_s {
	char		serial_num[BFA_ADAPTER_SERIAL_NUM_LEN];
	uint32_t	nports;	/*!< Number of NPorts */
	wwn_t		pwwn;	/*!< WWN of one of its physical port */
};
typedef struct bfa_adapter_aen_data_s bfa_adapter_aen_data_t;

/**
 * ---------------------- IOC definitions ------------
 */

enum {
	BFA_IOC_DRIVER_LEN	= 16,
	BFA_IOC_CHIP_REV_LEN 	= 8,
};

/**
 * Driver and firmware versions.
 */
struct bfa_ioc_driver_attr_s {
	char		driver[BFA_IOC_DRIVER_LEN];	/*!< driver name */
	char		driver_ver[BFA_VERSION_LEN];	/*!< driver version */
	char		fw_ver[BFA_VERSION_LEN];	/*!< firmware version */
	char		bios_ver[BFA_VERSION_LEN];	/*!< bios version */
	char		efi_ver[BFA_VERSION_LEN];	/*!< EFI version */
	char		ob_ver[BFA_VERSION_LEN];	/*!< openboot version */
};
typedef struct bfa_ioc_driver_attr_s bfa_ioc_driver_attr_t;

/**
 * IOC PCI device attributes
 */
struct bfa_ioc_pci_attr_s {
	uint16_t	vendor_id;	/*!< PCI vendor ID */
	uint16_t	device_id;	/*!< PCI device ID */
	uint16_t	ssid;		/*!< subsystem ID */
	uint16_t	ssvid;		/*!< subsystem vendor ID */
	uint32_t	pcifn;		/*!< PCI device function */
	uint32_t	rsvd;		/* padding */
	char		chip_rev[BFA_IOC_CHIP_REV_LEN];	 /*!< chip revision */
};
typedef struct bfa_ioc_pci_attr_s bfa_ioc_pci_attr_t;

/**
 * IOC states
 */
enum bfa_ioc_state {
	BFA_IOC_UNINIT		= 1,	/*!< IOC is in uninit state */
	BFA_IOC_RESET		= 2,	/*!< IOC is in reset state */
	BFA_IOC_SEMWAIT		= 3,	/*!< Waiting for IOC h/w semaphore */
	BFA_IOC_HWINIT		= 4,	/*!< IOC h/w is being initialized */
	BFA_IOC_GETATTR		= 5,	/*!< IOC is being configured */
	BFA_IOC_OPERATIONAL	= 6,	/*!< IOC is operational */
	BFA_IOC_INITFAIL	= 7,	/*!< IOC hardware failure */
	BFA_IOC_FAIL		= 8,	/*!< IOC heart-beat failure */
	BFA_IOC_DISABLING	= 9,	/*!< IOC is being disabled */
	BFA_IOC_DISABLED	= 10,	/*!< IOC is disabled */
	BFA_IOC_FWMISMATCH	= 11,	/*!< IOC f/w different from drivers */
	BFA_IOC_ENABLING	= 12,	/*!< IOC is being enabled */
	BFA_IOC_HWFAIL		= 13,	/*!< PCI mapping doesn't exist */
	BFA_IOC_ACQ_ADDR	= 14,	/*!< Acquiring addr from fabric */
};
typedef enum bfa_ioc_state bfa_ioc_state_t;

/**
 * IOC firmware stats
 */
struct bfa_fw_ioc_stats_s {
	uint32_t	enable_reqs;	/* Enable Requests */
	uint32_t	disable_reqs;	/* Disable Requests */
	uint32_t	get_attr_reqs;	/* Get Attr Requests */
	uint32_t	dbg_sync;	/* DBG sync count */
	uint32_t	dbg_dump;	/* DBG dump count */
	uint32_t	unknown_reqs;	/* Unknown Requests */
};
typedef struct bfa_fw_ioc_stats_s bfa_fw_ioc_stats_t;

/**
 * IOC driver stats
 */
struct bfa_ioc_drv_stats_s {
	uint32_t	ioc_isrs;	/* Mailbox interrupts */
	uint32_t	ioc_enables;	/* Enable events */
	uint32_t	ioc_disables;	/* Disable events */
	uint32_t	ioc_hbfails;	/* Heartbeat failures */
	uint32_t	ioc_boots;	/* Firmware boots */
	uint32_t	stats_tmos;	/* Stats timeouts */
	uint32_t	hb_count;	/* Heart-beat count */
	uint32_t	disable_reqs;	/* Disable requests */
	uint32_t	enable_reqs;	/* Enable requests */
	uint32_t	disable_replies; /* Disable replies */
	uint32_t	enable_replies;	/* Enable replies */
	uint32_t	rsvd;
};
typedef struct bfa_ioc_drv_stats_s bfa_ioc_drv_stats_t;

/**
 * IOC statistics
 */
struct bfa_ioc_stats_s {
	struct bfa_ioc_drv_stats_s	drv_stats; /*!< driver IOC stats */
	struct bfa_fw_ioc_stats_s 	fw_stats;  /*!< firmware IOC stats */
};
typedef struct bfa_ioc_stats_s bfa_ioc_stats_t;


enum bfa_ioc_type_e {
	BFA_IOC_TYPE_FC		= 1,
	BFA_IOC_TYPE_FCoE	= 2,
	BFA_IOC_TYPE_LL		= 3,
};

/**
 * IOC attributes returned in queries
 */
struct bfa_ioc_attr_s {
	enum bfa_ioc_type_e		ioc_type;
	enum bfa_ioc_state 		state;		/*!< IOC state      */
	struct bfa_adapter_attr_s	adapter_attr;	/*!< HBA attributes */
	struct bfa_ioc_driver_attr_s 	driver_attr;	/*!< driver attr    */
	struct bfa_ioc_pci_attr_s	pci_attr;
	uint8_t				port_id;	/*!< port number    */
	uint8_t				port_mode;	/*!< bfa_mode_s     */
	uint8_t				cap_bm; 	/*!< capability     */
	uint8_t				port_mode_cfg;	/*!< bfa_mode_s     */
	uint8_t				def_fn;		/*!< 1 if default fn */
	uint8_t				rsvd[3];	/*!< 64bit align    */
};
typedef struct bfa_ioc_attr_s bfa_ioc_attr_t;

/**
 * BFA IOC level events
 */
enum bfa_ioc_aen_event {
	BFA_IOC_AEN_HBGOOD	= 1,	/*!< Heart Beat restore event	*/
	BFA_IOC_AEN_HBFAIL	= 2,	/*!< Heart Beat failure event	*/
	BFA_IOC_AEN_ENABLE	= 3,	/*!< IOC enabled event		*/
	BFA_IOC_AEN_DISABLE	= 4,	/*!< IOC disabled event		*/
	BFA_IOC_AEN_FWMISMATCH	= 5,	/*!< IOC firmware mismatch	*/
	BFA_IOC_AEN_FWCFG_ERROR = 6,	/*!< IOC firmware config error	*/
	BFA_IOC_AEN_INVALID_VENDOR = 7,
	BFA_IOC_AEN_INVALID_NWWN = 8,	/*!< Zero NWWN 			*/
	BFA_IOC_AEN_INVALID_PWWN = 9,	/*!< Zero PWWN 			*/
	BFA_IOC_AEN_INVALID_MAC = 10,	/*!< Base MAC address set failed */
};
typedef enum bfa_ioc_aen_event bfa_ioc_aen_event_t;

/**
 * BFA IOC level event data, now just a place holder
 */
struct bfa_ioc_aen_data_s {
	wwn_t	pwwn;
	int16_t ioc_type;
	mac_t	mac;
};
typedef struct bfa_ioc_aen_data_s bfa_ioc_aen_data_t;

/**
 * Adapter capability mask definition
 */
enum {
	BFA_CM_HBA	=	0x01,
	BFA_CM_CNA	=	0x02,
	BFA_CM_NIC	=	0x04,
	BFA_CM_FC16G	= 	0x08,
	BFA_CM_SRIOV	= 	0x10,
	BFA_CM_MEZZ	= 	0x20,
};

/**
 * ---------------------- mfg definitions ------------
 */

/**
 * Checksum size
 */
#define BFA_MFG_CHKSUM_SIZE			16

#define BFA_MFG_PARTNUM_SIZE			14
#define BFA_MFG_SUPPLIER_ID_SIZE		10
#define BFA_MFG_SUPPLIER_PARTNUM_SIZE		20
#define BFA_MFG_SUPPLIER_SERIALNUM_SIZE		20
#define BFA_MFG_SUPPLIER_REVISION_SIZE		4
#define BFA_MFG_MOD_LEN				16

/**
 * Initial capability definition
 */
enum {
	BFA_MFG_IC_FC		= 0x01,
	BFA_MFG_IC_ETH		= 0x02,
};

#pragma pack(1)

/**
 * @brief BFA adapter manufacturing block definition.
 *
 * All numerical fields are in big-endian format.
 */
struct bfa_mfg_block_s {
};
typedef struct bfa_mfg_block_s bfa_mfg_block_t;

#pragma pack()

/**
 * ---------------------- pci definitions ------------
 */

/**
 * PCI device and vendor ID information
 */
enum {
	BFA_PCI_VENDOR_ID_BROCADE	= 0x1657,
	BFA_PCI_DEVICE_ID_FC_8G2P	= 0x13,
	BFA_PCI_DEVICE_ID_FC_8G1P	= 0x17,
	BFA_PCI_DEVICE_ID_CT		= 0x14,
	BFA_PCI_DEVICE_ID_CT_FC		= 0x21,
	BFA_PCI_DEVICE_ID_CT2		= 0x22,
	BFA_PCI_DEVICE_ID_CT2_QUAD	= 0x23,
};

#define bfa_asic_id_cb(__d) 			\
	((__d) == BFA_PCI_DEVICE_ID_FC_8G2P ||	\
	 (__d) == BFA_PCI_DEVICE_ID_FC_8G1P)
#define bfa_asic_id_ct(__d) 			\
	((__d) == BFA_PCI_DEVICE_ID_CT ||	\
	 (__d) == BFA_PCI_DEVICE_ID_CT_FC)
#define bfa_asic_id_ct2(__d) 			\
	((__d) == BFA_PCI_DEVICE_ID_CT2 ||	\
	(__d) == BFA_PCI_DEVICE_ID_CT2_QUAD)
#define bfa_asic_id_ctc(__d) 		\
	(bfa_asic_id_ct(__d) || bfa_asic_id_ct2(__d))

/**
 * PCI sub-system device and vendor ID information
 */
enum {
	BFA_PCI_FCOE_SSDEVICE_ID	= 0x14,
	BFA_PCI_CT2_SSID_FCoE		= 0x22,
	BFA_PCI_CT2_SSID_ETH		= 0x23,
	BFA_PCI_CT2_SSID_FC		= 0x24,
};

/**
 * Maximum number of device address ranges mapped through different BAR(s)
 */
#define BFA_PCI_ACCESS_RANGES 2

/**
 * @brief
 *	Temperature sensor query results
 */
typedef struct bfa_diag_results_tempsensor_s {
	uint32_t	status;
	uint16_t	temp;		/* 10-bit A/D value */
	uint16_t	brd_temp;	/*!< 9-bit board temp */
	uint8_t		ts_junc;  	/*!< show junction tempsensor	*/
	uint8_t		ts_brd;		/*!< show board tempsensor	*/
	uint8_t		rsvd[6];	/*!< keep 8 bytes alignment	*/
} bfa_diag_temp_t;

/**
 * @brief
 *	Showtemp results
 */
typedef struct {
	uint32_t	status;
	bfa_boolean_t	ts_junc;
	bfa_boolean_t	ts_brd;
	double		junc_temp;
	double		brd_temp;
} bfa_diag_showtemp_t;

/**
 * @brief
 * 		Memory test algorithms.
 */
typedef enum {
	BFA_DIAG_MEMTEST_UNKNOWN 	= 0x00,
	BFA_DIAG_MEMTEST_WALK01 	= 0x01,
	BFA_DIAG_MEMTEST_MARCHY 	= 0x02,
	BFA_DIAG_MEMTEST_MATSPLUS 	= 0x04,
	BFA_DIAG_MEMTEST_MOVI 		= 0x08,
	BFA_DIAG_MEMTEST_TLAPNPSF1T 	= 0x10,
} bfa_diag_memtest_algo_t;

/**
 * RAM test params
 */
typedef struct bfa_diag_memtest {
	uint8_t	algo;
	uint8_t	rsvd[7];
} bfa_diag_memtest_t;

/*
 * User are expected to allocate one result data struct for each alog
 */
typedef struct bfa_diag_memtest_result {
	uint32_t	status;
	uint32_t	addr;
	uint32_t	exp; /* expect value read from reg */
	uint32_t	act; /* actually value read */
	uint32_t	err_status;		/* error status reg */
	uint32_t	err_status1;	/* extra error info reg */
	uint32_t	err_addr; /* error address reg */
	uint8_t		algo;
	uint8_t		rsv[3];
} bfa_diag_memtest_result_t;

/**
 * Post diiagnostics test results
 */
typedef struct {
	uint8_t	imist_pass;	/*!< internal SRAM BIST test result */
	uint8_t	emist_pass;	/*!< external SRAM BIST test result */
	uint8_t	serdes_pass;	/*!< serdes test result */
	uint8_t	rsv;
	uint32_t	rsvd;
} bfa_diag_post_result_t;

#define DPORT_ENABLE_LOOPCNT_MAX 4294967295
#define DPORT_ENABLE_LOOPCNT_DEFAULT (1024 * 1024)
#define LB_LOOPCNT_DEFAULT	(128 * 1024)
#define LB_PATTERN_DEFAULT	0xB5B5B5B5
#define QTEST_CNT_DEFAULT	10
#define QTEST_PAT_DEFAULT	LB_PATTERN_DEFAULT
#define ETH_LB_LOOPCNT_DEFAULT	(64 * 1024)
#define ETH_LB_LOOPCNT_MAX	(128 * 1024)

/**
 * Loopback Test
 */
typedef struct {
	uint32_t	loopcnt;
	uint32_t	pattern;
	uint8_t		lb_mode;	/*!< bfa_port_opmode_t */
	uint8_t		speed;		/*!< bfa_port_speed_t */
	uint8_t		rsvd[2];
} bfa_diag_loopback_t;

#pragma pack(1)
typedef struct {
	uint32_t	numtxmfrm;	/* no. of transmit frame */
	uint32_t	numosffrm;	/* no. of outstanding frame */
	uint32_t	numrcvfrm;	/* no. of received good frame */
	uint32_t	badfrminf;	/* mis-match info */
	uint32_t	badfrmnum;	/* mis-match fram number */
	uint8_t		status;		/* loopback test result */
	uint8_t		rsvd[3];
} bfa_diag_loopback_result_t;
#pragma pack()

/*
 * D_Port test status :
 * This will tell the final status of the test.
 */
typedef enum {
	DPORT_TEST_ST_IDLE	= 0,	/*!< The test has not started yet. */
	DPORT_TEST_ST_FINAL	= 1,	/*!< The test done successfully */
	DPORT_TEST_ST_SKIP	= 2,	/*!< the test skipped */
	DPORT_TEST_ST_FAIL	= 3,	/*!< the test failed */
	DPORT_TEST_ST_INPRG	= 4,	/*!< the testing is in progress */
	DPORT_TEST_ST_RESPONDER	= 5,	/*!< test triggered from remote port */
	DPORT_TEST_ST_STOPPED	= 6,	/*!< The test stopped by user. */
	DPORT_TEST_ST_MAX
} bfa_diag_dport_test_status_t;

/**
 * dport test type enum
 */
typedef enum {
	DPORT_TEST_ELOOP	= 0,
	DPORT_TEST_OLOOP	= 1,
	DPORT_TEST_ROLOOP	= 2,
	DPORT_TEST_LINK		= 3,
	DPORT_TEST_MAX
} bfa_diag_dport_test_type_e;

/**
 * dport test mode enum
 */
typedef enum {
	BFA_DPORT_OPMODE_AUTO	= 0,
	BFA_DPORT_OPMODE_MANU	= 1,
} bfa_diag_dport_test_opmode_t;

/**
 * dport sub-test results
 */
typedef struct {
	uint8_t		status;		/*!< bfa_diag_dport_test_status_e */
	uint8_t		rsvd[7];  	/* 64bit align */
	uint64_t	start_time;	/*!< timestamp  */
} bfa_diag_dport_subtest_result_t;

/**
 * dport test results
 */
struct bfa_diag_dport_result_s {
	wwn_t		rp_pwwn;  	/*!< switch port wwn  */
	wwn_t		rp_nwwn;  	/*!< switch node wwn  */
	uint64_t	start_time;	/*!< user/sw start time */
	uint64_t	end_time;	/*!< timestamp  */
	uint8_t		status;		/*!< bfa_diag_dport_test_status_e */
	uint8_t		mode;		/*!< bfa_diag_dport_test_opmode_e */
	uint8_t		rsvd;		/*!< 64bit align */
	uint8_t		speed;  	/*!< link speed for buf_reqd */
	uint16_t	buffer_required;
	uint16_t	frmsz;		/*!< frame size for buf_reqd */
	uint32_t	lpcnt;		/* Frame count */
	uint32_t	pat;		/* Pattern */
	uint32_t	roundtrip_latency;	/*!< in nano sec */
	uint32_t	est_cable_distance;	/*!< in meter */
	bfa_diag_dport_subtest_result_t	subtest[DPORT_TEST_MAX];
};
typedef struct bfa_diag_dport_result_s bfa_diag_dport_result_t;

/**
 * @brief
 * 		Firmware ping test results
 */
typedef struct bfa_diag_results_fwping {
	uint32_t	data;		/* store the corrupted data */
	uint32_t	status;
	uint32_t	dmastatus;
	uint8_t		rsvd[4];
} bfa_diag_fwping_result_t;

/**
 * LED Test command
 */
typedef struct {
	uint32_t	cmd;	/*!< bfa_led_op_t */
	uint32_t	color;	/*!< bfa_led_color_t */
	uint16_t	freq;	/*!< no. of blinks every 10 secs */
	uint8_t		led;	/*!< bitmap of LEDs to be tested */
	uint8_t		rsvd[5];
} bfa_diag_ledtest_t;


#define	BFA_LED_MAX_NUM		3

enum bfa_led_op {
	BFA_LED_OFF   = 0,
	BFA_LED_ON    = 1,
	BFA_LED_FLICK = 2,
	BFA_LED_BLINK = 3,
};
typedef enum bfa_led_op bfa_led_op_t;

enum bfa_led_color {
	BFA_LED_GREEN = 0,
	BFA_LED_AMBER = 1,
};
typedef enum bfa_led_color bfa_led_color_t;

/**
 * POM health status levels for each attributes.
 */
enum bfa_pom_entry_health {
	BFA_POM_HEALTH_NOINFO  = 1,	/*!< no information */
	BFA_POM_HEALTH_NORMAL  = 2,	/*!< health is normal */
	BFA_POM_HEALTH_WARNING = 3,	/*!< warning level */
	BFA_POM_HEALTH_ALARM   = 4,	/*!< alarming level */
};
typedef enum bfa_pom_entry_health bfa_pom_entry_health_t;

/**
 * Reading of temperature/voltage/current/power
 */
struct bfa_pom_entry_s {
	enum bfa_pom_entry_health health;	/*!< POM entry health */
	uint32_t	curr_value;	/*!< current value */
	uint32_t	thr_warn_high;	/*!< threshold warning high */
	uint32_t	thr_warn_low;	/*!< threshold warning low */
	uint32_t	thr_alarm_low;	/*!< threshold alaram low */
	uint32_t	thr_alarm_high;	/*!< threshold alarm high */
};
typedef struct bfa_pom_entry_s bfa_pom_entry_t;

/**
 * POM attributes
 */
struct bfa_pom_attr_s {
	struct bfa_pom_entry_s temperature;	/*!< centigrade */
	struct bfa_pom_entry_s voltage;	/*!< volts */
	struct bfa_pom_entry_s curr;	/*!< milli amps */
	struct bfa_pom_entry_s txpower;	/*!< micro watts */
	struct bfa_pom_entry_s rxpower;	/*!< micro watts */
};
typedef struct bfa_pom_attr_s bfa_pom_attr_t;

typedef enum {
	BFA_SFP_MEDIA_UNKNOWN	= 0x00,
	BFA_SFP_MEDIA_CU	= 0x01,
	BFA_SFP_MEDIA_LW	= 0x02,
	BFA_SFP_MEDIA_SW	= 0x03,
	BFA_SFP_MEDIA_EL	= 0x04,
	BFA_SFP_MEDIA_UNSUPPORT	= 0x05,
} bfa_defs_sfp_media_t;
#define BFA_SFP_MEDIA_NUM		(6) /* number of media type */

/**
 * SFP state change notification event
 */
#define	BFA_SFP_SCN_REMOVED		0
#define	BFA_SFP_SCN_INSERTED		1
#define	BFA_SFP_SCN_POM			2
#define	BFA_SFP_SCN_FAILED 		3
#define	BFA_SFP_SCN_UNSUPPORT		4
#define	BFA_SFP_SCN_VALID		5

/**
 * Temperature sensor status values
 */
enum bfa_tsensor_status {
	BFA_TSENSOR_STATUS_UNKNOWN   = 1,   /*!< unkown status */
	BFA_TSENSOR_STATUS_FAULTY    = 2,   /*!< sensor is faulty */
	BFA_TSENSOR_STATUS_BELOW_MIN = 3,   /*!< temperature below mininum */
	BFA_TSENSOR_STATUS_NOMINAL   = 4,   /*!< normal temperature */
	BFA_TSENSOR_STATUS_ABOVE_MAX = 5,   /*!< temperature above maximum */
};
typedef enum bfa_tsensor_status bfa_tsensor_status_t;

/**
 * Temperature sensor attribute
 */
struct bfa_tsensor_attr_s {
	enum bfa_tsensor_status status;	/*!< temperature sensor status */
	uint32_t		value;	/*!< current temperature in celsius */
};
typedef struct bfa_tsensor_attr_s bfa_tsensor_attr_t;

/**
 * @brief
 * 	Port speed settings
 */
enum bfa_port_speed {
	BFA_PORT_SPEED_UNKNOWN  = 0,
	BFA_PORT_SPEED_1GBPS 	= 1,
	BFA_PORT_SPEED_2GBPS 	= 2,
	BFA_PORT_SPEED_4GBPS 	= 4,
	BFA_PORT_SPEED_8GBPS 	= 8,
	BFA_PORT_SPEED_10GBPS 	= 10,
	BFA_PORT_SPEED_16GBPS 	= 16,
	BFA_PORT_SPEED_32GBPS 	= 32,
	BFA_PORT_SPEED_AUTO	= 0xf,

	BFA_PORT_SPEED_FC_MIN	= BFA_PORT_SPEED_1GBPS,
	BFA_PORT_SPEED_FC_MAX	= BFA_PORT_SPEED_16GBPS,
	BFA_PORT_SPEED_FC_AUTO	= BFA_PORT_SPEED_AUTO,
	BFA_PORT_SPEED_CNA_MIN	= BFA_PORT_SPEED_10GBPS,
	BFA_PORT_SPEED_CNA_MAX	= BFA_PORT_SPEED_10GBPS,
};
typedef enum bfa_port_speed bfa_port_speed_t;

#define bfa_port_cfg_fc_speed_valid(__s)	\
	 (((__s) == BFA_PORT_SPEED_1GBPS) ||	\
	  ((__s) == BFA_PORT_SPEED_2GBPS) ||	\
	  ((__s) == BFA_PORT_SPEED_4GBPS) ||	\
	  ((__s) == BFA_PORT_SPEED_8GBPS) ||	\
	  ((__s) == BFA_PORT_SPEED_16GBPS) ||	\
	  ((__s) == BFA_PORT_SPEED_AUTO))

#define bfa_port_cfg_cna_speed_valid(__s)	\
	((__s) == BFA_PORT_SPEED_10GBPS)

#define bfa_cfg_trl_def_speed_valid(__s)	\
	(((__s) == BFA_PORT_SPEED_1GBPS) ||	\
	 ((__s) == BFA_PORT_SPEED_2GBPS) ||	\
	 ((__s) == BFA_PORT_SPEED_4GBPS) ||	\
	 ((__s) == BFA_PORT_SPEED_8GBPS) ||	\
	 ((__s) == BFA_PORT_SPEED_16GBPS) ||	\
	 ((__s) == BFA_PORT_SPEED_10GBPS))

/**
 * BFA physical port Level events
 * Arguments below are in BFAL context from Mgmt
 * BFA_PORT_AEN_ONLINE:     [in]: pwwn	[out]: pwwn
 * BFA_PORT_AEN_OFFLINE:    [in]: pwwn	[out]: pwwn
 * BFA_PORT_AEN_RLIR:       [in]: None	[out]: pwwn, rlir_data, rlir_len
 * BFA_PORT_AEN_SFP_INSERT: [in]: pwwn	[out]: port_id, pwwn
 * BFA_PORT_AEN_SFP_REMOVE: [in]: pwwn	[out]: port_id, pwwn
 * BFA_PORT_AEN_SFP_POM:    [in]: pwwn	[out]: level, port_id, pwwn
 * BFA_PORT_AEN_ENABLE:     [in]: pwwn	[out]: pwwn
 * BFA_PORT_AEN_DISABLE:    [in]: pwwn	[out]: pwwn
 * BFA_PORT_AEN_AUTH_ON:    [in]: pwwn	[out]: pwwn
 * BFA_PORT_AEN_AUTH_OFF:   [in]: pwwn	[out]: pwwn
 * BFA_PORT_AEN_DISCONNECT: [in]: pwwn	[out]: pwwn
 * BFA_PORT_AEN_QOS_NEG:    [in]: pwwn	[out]: pwwn
 * BFA_PORT_AEN_FABRIC_NAME_CHANGE: [in]: pwwn, [out]: pwwn, fwwn
 *
 */
enum bfa_port_aen_event {
	BFA_PORT_AEN_ONLINE	= 1,	/*!< Physical Port online event */
	BFA_PORT_AEN_OFFLINE	= 2,	/*!< Physical Port offline event */
	BFA_PORT_AEN_RLIR	= 3,	/*!< RLIR event, not supported */
	BFA_PORT_AEN_SFP_INSERT	= 4,	/*!< SFP inserted event */
	BFA_PORT_AEN_SFP_REMOVE	= 5,	/*!< SFP removed event */
	BFA_PORT_AEN_SFP_POM	= 6,	/*!< SFP POM event */
	BFA_PORT_AEN_ENABLE	= 7,	/*!< Physical Port enable event */
	BFA_PORT_AEN_DISABLE	= 8,	/*!< Physical Port disable event */
	BFA_PORT_AEN_AUTH_ON	= 9,	/*!< Physical Port auth success event */
	BFA_PORT_AEN_AUTH_OFF	= 10,	/*!< Physical Port auth fail event */
	BFA_PORT_AEN_DISCONNECT	= 11,	/*!< Physical Port disconnect event */
	BFA_PORT_AEN_QOS_NEG	= 12,  	/*!< Base Port QOS negotiation event */
	BFA_PORT_AEN_FABRIC_NAME_CHANGE	= 13, /*!< Fabric Name/WWN change */
	BFA_PORT_AEN_SFP_ACCESS_ERROR	= 14, /*!< SFP read error event */
	BFA_PORT_AEN_SFP_UNSUPPORT	= 15, /*!< Unsupported SFP event */
};
typedef enum bfa_port_aen_event bfa_port_aen_event_t;

enum bfa_port_aen_sfp_pom {
	BFA_PORT_AEN_SFP_POM_GREEN = 1,	/*!< Normal */
	BFA_PORT_AEN_SFP_POM_AMBER = 2,	/*!< Warning */
	BFA_PORT_AEN_SFP_POM_RED   = 3,	/*!< Critical */
	BFA_PORT_AEN_SFP_POM_MAX   = BFA_PORT_AEN_SFP_POM_RED
};
typedef enum bfa_port_aen_sfp_pom bfa_port_aen_sfp_pom_t;

struct bfa_port_aen_data_s {
	wwn_t		pwwn;	      /*!< WWN of the physical port */
	wwn_t		fwwn;	      /*!< WWN of the fabric port */
	int32_t		phy_port_num; /*!< For SFP related events */
	int16_t		ioc_type;
	int16_t		level;	      /*!< Only transitions will be informed */
	mac_t		mac;	      /*!< MAC address of the ethernet port, */
				      /*   applicable to CNA port only */
	int16_t		rsvd;
};
typedef struct bfa_port_aen_data_s bfa_port_aen_data_t;

/**
 * ASIC block configuration related structures
 */

#define BFA_ABLK_MAX_PORTS	2
#define BFA_ABLK_MAX_PFS	16
#define BFA_NWPART_MAX_PFS_PER_PORT	4

#pragma pack(1)
enum bfa_mode_s {
	BFA_MODE_HBA		= 1,
	BFA_MODE_CNA		= 2,
	BFA_MODE_NIC		= 3
};
typedef enum bfa_mode_s bfa_mode_t;

struct bfa_adapter_cfg_mode_s {
	uint16_t max_pf;
	uint16_t max_vf;
	bfa_mode_t mode;
};
typedef struct bfa_adapter_cfg_mode_s bfa_adapter_cfg_mode_t;

struct bfa_ablk_cfg_pf_s {
	uint16_t		pers;
	uint8_t			port_id;
	uint8_t			optrom;

	uint8_t			valid;
	uint8_t			sriov;
	uint8_t			max_vfs;
	uint8_t			enabled;

	uint16_t		num_qpairs;
	uint16_t		num_vectors;

	uint16_t		bw_min;
	uint16_t		bw_max;
};
typedef struct bfa_ablk_cfg_pf_s bfa_ablk_cfg_pf_t;

struct bfa_ablk_cfg_port_s {
	uint8_t			mode;		/* bfa_mode_t */
	uint8_t			type;		/* bfa_port_type_t */
	uint8_t			max_pfs;
	uint8_t			nw_part;	/* NW Partition (TRUE/FALSE) */
	uint8_t			rsvd[4];
};
typedef struct bfa_ablk_cfg_port_s bfa_ablk_cfg_port_t;

struct bfa_ablk_cfg_inst_s {
	uint8_t			nports;
	uint8_t			max_pfs;
	uint8_t			rsvd[6];
	bfa_ablk_cfg_pf_t	pf_cfg[BFA_ABLK_MAX_PFS];
	bfa_ablk_cfg_port_t	port_cfg[BFA_ABLK_MAX_PORTS];
};
typedef struct bfa_ablk_cfg_inst_s bfa_ablk_cfg_inst_t;

enum {
	BFA_ABLK_ACTIVE		= 0,
	BFA_ABLK_CONFIG		= 1,
	BFA_ABLK_MAX		= 2,
};

struct bfa_ablk_cfg_s {
	bfa_ablk_cfg_inst_t	inst[BFA_ABLK_MAX];
};
typedef struct bfa_ablk_cfg_s bfa_ablk_cfg_t;

struct bfa_vnic_cfg_s {
	uint8_t		pfn;
	uint8_t		bw_min;
	uint8_t		bw_max;
	uint16_t	pers;
	uint8_t		enabled;
	mac_t		pbc_mac;
	mac_t		mfg_mac;
};
typedef struct bfa_vnic_cfg_s bfa_vnic_cfg_t;

struct bfa_nwpart_port_s {
	uint8_t	nw_part;
	uint8_t rsvd[3];
	bfa_vnic_cfg_t vnics[BFA_NWPART_MAX_PFS_PER_PORT];
};
typedef struct bfa_nwpart_port_s bfa_nwpart_port_t;

struct bfa_nwpart_cfg_s {
	bfa_nwpart_port_t port[BFA_ABLK_MAX_PORTS];
};
typedef struct bfa_nwpart_cfg_s bfa_nwpart_cfg_t;

#pragma pack()

#endif /* __BFA_DEFS_H__ */
