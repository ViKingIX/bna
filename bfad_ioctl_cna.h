/*
 * Copyright (c)  2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c)  2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */
#ifndef __BFAD_IOCTL_CNA_H__
#define __BFAD_IOCTL_CNA_H__

#include <linux/ioctl.h>
#include "bfa_os_inc.h"
#include <cs/bfa_cs.h>
#include <defs/bfa_defs.h>
#include <ioctl/bfa_ioctl_version.h>
#include <ioctl/bfa_ioctl_generic.h>
#include <ioctl/bfa_ioctl_ioc.h>
#include <ioctl/bfa_ioctl_pcifn.h>
#include <ioctl/bfa_ioctl_diag.h>
#include <ioctl/bfa_ioctl_sfp.h>
#include <ioctl/bfa_ioctl_debug.h>
#include <ioctl/bfa_ioctl_log.h>
#include <ioctl/bfa_ioctl_aen.h>
#include <ioctl/bfa_ioctl_port.h>
#include <ioctl/bfa_ioctl_cee.h>
#include <ioctl/bfa_ioctl_ethport.h>
#include <ioctl/bfa_ioctl_boot.h>
#include <ioctl/bfa_ioctl_flash.h>
#include <ioctl/bfa_ioctl_phy.h>
#include <ioctl/bfa_ioctl_nwpart.h>
#include <ioctl/bfa_ioctl_fru.h>

/* IOCTL cmd code */
#define BFA_MAGIC   'C'
#define BNA_MAGIC   'D'

#define BFAD_CLEAR_TYPEBITS(cmd) \
	((cmd)&(~(_IOC_TYPEMASK<<_IOC_TYPESHIFT)))

#define BFAD_IOCMD(cmd) \
	((BFAD_CLEAR_TYPEBITS(cmd))|(BFA_MAGIC<<_IOC_TYPESHIFT))

#define BNAD_IOCMD(cmd) \
	((BFAD_CLEAR_TYPEBITS(cmd))|(BNA_MAGIC<<_IOC_TYPESHIFT))

enum {
	/* IOC */
	BFAD_IOC_GET_VERSION = 0x01,
	BFAD_IOC_GET_INSTS,
	BFAD_IOC_ENABLE,
	BFAD_IOC_DISABLE,
	BFAD_IOC_CFG_PPORT,
	BFAD_IOC_GET_ATTR,
	BFAD_IOC_GET_INFO,
	BFAD_IOC_GET_STATS,
	BFAD_IOC_RESET_STATS,
	BFAD_IOC_SET_ADAPTER_NAME,
	BFAD_IOC_SET_PORT_NAME,
	BFAD_IOC_GET_FWSTATS,
	BFAD_IOC_RESET_FWSTATS,
	BFAD_IOC_PCIFN_CFG,
	BFAD_ADAPTER_CFG_MODE,
	BFAD_FW_SIG_INV,
	/* DIAG */
	BFAD_DIAG_REGRD,
	BFAD_DIAG_MEMRD,
	BFAD_DIAG_REGWR,
	BFAD_DIAG_TEMP,
	BFAD_DIAG_POST,
	BFAD_DIAG_MEMTEST,
	BFAD_DIAG_LOOPBACK,
	BFAD_DIAG_FWPING,
	BFAD_DIAG_QUEUETEST,
	BFAD_DIAG_POM,
	BFAD_DIAG_SFP,
	BFAD_DIAG_LED,
	BFAD_DIAG_BEACON_LPORT,
	BFAD_DIAG_LL_LOOPBACK,
	BFAD_DIAG_LB_STAT,
	BFAD_DIAG_DPORT_ENABLE,
	BFAD_DIAG_DPORT_DISABLE,
	BFAD_DIAG_DPORT_SHOW,
	BFAD_DIAG_DPORT_START,
	BFAD_DIAG_DPORT_STOP,
	/* DEBUG */
	BFAD_DEBUG_DRV_TRACE,
	BFAD_DEBUG_DRV_STATE,
	BFAD_DEBUG_FW_TRACE,
	BFAD_DEBUG_FW_STATE,
	BFAD_DEBUG_FW_STATE_CLR,
	BFAD_DEBUG_FW_CORE,
	BFAD_DEBUG_PORTLOG,
	BFAD_DEBUG_PORTLOG_CLR,
	BFAD_DEBUG_PORTLOG_CTL,
	BFAD_DEBUG_START_DTRC,
	BFAD_DEBUG_STOP_DTRC,
	/* LOG */
	BFAD_LOG_SET_LEVEL,
	BFAD_LOG_GET_LEVEL,
	/* AEN */
	BFAD_AEN_GET,
	BFAD_AEN_UPDATE,
	/* PORT */
	BFAD_PORT_ENABLE,
	BFAD_PORT_DISABLE,
	BFAD_PORT_GET_ATTR,
	BFAD_PORT_GET_STATS,
	BFAD_PORT_GET_FC4STATS,
	BFAD_PORT_GET_FCPMAP,
	BFAD_PORT_RESET_STATS,
	BFAD_PORT_CFG_TOPO,
	BFAD_PORT_CFG_SPEED,
	BFAD_PORT_CFG_ALPA,
	BFAD_PORT_CLR_ALPA,
	BFAD_PORT_CFG_MAXFRSZ,
	BFAD_PORT_TRUNK_ENABLE,
	BFAD_PORT_TRUNK_DISABLE,
	BFAD_PORT_CFG_RNID,
	BFAD_PORT_GET_RNID,
	BFAD_PORT_CFG_MODE,
	BFAD_PORT_BBCR_ENABLE,
	BFAD_PORT_BBCR_DISABLE,
	BFAD_PORT_GET_BBCR_ATTR,
	BFAD_PORT_NWPAR_ENABLE,
	BFAD_PORT_NWPAR_DISABLE,
	/* CEE */
	BFAD_CEE_RESET_STATS,
	BFAD_CEE_GET_ATTR,
	BFAD_CEE_GET_STATS,
	/* SFP */
	BFAD_SFP_SHOW,
	BFAD_SFP_MEDIA,
	BFAD_SFP_SPEED,
	/* ETHBOOT */
	BFAD_ETHBOOT_CFG,
	BFAD_ETHBOOT_QUERY,
	/* FLASH */
	BFAD_FLASH_GET_ATTR,
	BFAD_FLASH_ERASE_PART,
	BFAD_FLASH_UPDATE_PART,
	BFAD_FLASH_READ_PART,
	BFAD_FLASH_MFG_INFO,
	BFAD_FLASH_ENABLE_OPTROM,
	BFAD_FLASH_DISABLE_OPTROM,
	BFAD_FLASH_GET_CFG_BOOTVER,
	/* PHY */
	BFAD_PHY_UPDATE_FW,
	BFAD_PHY_READ_FW,
	BFAD_PHY_GET_ATTR,
	BFAD_PHY_GET_STATS,
	/* PCIFN */
	BFAD_PCIFN_CREATE,
	BFAD_PCIFN_DELETE,
	BFAD_PCIFN_ENABLE,
	BFAD_PCIFN_DISABLE,
	BFAD_PCIFN_PERS_CHANGE,
	BFAD_PCIFN_BW,
	/* VNIC */
	BFAD_VNIC_QUERY,
	BFAD_VNIC_ENABLE,
	BFAD_VNIC_DISABLE,
	BFAD_VNIC_STATS,
	BFAD_VNIC_RESET_STATS,
	/* VHBA */
	BFAD_VHBA_QUERY,
	BFAD_VHBA_ENABLE,
	BFAD_VHBA_DISABLE,
	/* ISCSIBOOT */
	BFAD_ISCSIBOOT_CFG,
	BFAD_ISCSIBOOT_QUERY,
	/* TFRU */
	BFAD_TFRU_READ,
	BFAD_TFRU_WRITE,
	/* FRU */
	BFAD_FRUVPD_READ,
	BFAD_FRUVPD_UPDATE,
	BFAD_FRUVPD_GET_MAX_SIZE,

	BFAD_CMD_MAX,
};


/* IOC */
#define IOCMD_IOC_GET_VERSION   _IOWR(BFA_MAGIC, BFAD_IOC_GET_VERSION, \
				      bfa_ioctl_ioc_get_version_t)
#define IOCMD_IOC_GET_INSTS     _IOWR(BFA_MAGIC, BFAD_IOC_GET_INSTS, \
				      bfa_ioctl_ioc_get_inst_t)
#define IOCMD_IOC_ENABLE	_IOWR(BFA_MAGIC, BFAD_IOC_ENABLE, \
				      bfa_ioctl_gen_t)
#define IOCMD_IOC_DISABLE	_IOWR(BFA_MAGIC, BFAD_IOC_DISABLE, \
				      bfa_ioctl_gen_t)
#define IOCMD_IOC_GET_ATTR	_IOWR(BFA_MAGIC, BFAD_IOC_GET_ATTR, \
				      bfa_ioctl_ioc_attr_t)
#define IOCMD_IOC_GET_INFO	_IOWR(BFA_MAGIC, BFAD_IOC_GET_INFO, \
				      bfa_ioctl_ioc_info_t)
#define IOCMD_IOC_GET_STATS	_IOWR(BFA_MAGIC, BFAD_IOC_GET_STATS, \
				      bfa_ioctl_ioc_stats_t)
#define IOCMD_IOC_RESET_STATS	_IOWR(BFA_MAGIC, BFAD_IOC_RESET_STATS, \
				      bfa_ioctl_gen_t)
#define IOCMD_IOC_SET_ADAPTER_NAME	\
				_IOWR(BFA_MAGIC, BFAD_IOC_SET_ADAPTER_NAME, \
				bfa_ioctl_ioc_set_adapter_name_t)
#define IOCMD_IOC_SET_PORT_NAME	\
				_IOWR(BFA_MAGIC, BFAD_IOC_SET_PORT_NAME, \
				bfa_ioctl_ioc_set_adapter_name_t)
#define IOCMD_IOC_GET_FWSTATS	_IOWR(BFA_MAGIC, BFAD_IOC_GET_FWSTATS, \
				      bfa_ioctl_ioc_fwstats_t)
#define IOCMD_IOC_RESET_FWSTATS	_IOWR(BFA_MAGIC, BFAD_IOC_RESET_FWSTATS, \
				      bfa_ioctl_gen_t)
#define IOCMD_IOC_PCIFN_CFG _IOWR(BFA_MAGIC, BFAD_IOC_PCIFN_CFG, \
					bfa_ioctl_pcifn_cfg_t)
#define IOCMD_ADAPTER_CFG_MODE	_IOWR(BFA_MAGIC, BFAD_ADAPTER_CFG_MODE, \
				bfa_ioctl_adapter_cfg_mode_t)
#define IOCMD_IOC_FW_SIG_INV	_IOWR(BFA_MAGIC, BFAD_FW_SIG_INV, \
				bfa_ioctl_gen_t)

/* DIAG */
#define IOCMD_DIAG_REGRD	_IOWR(BFA_MAGIC, BFAD_DIAG_REGRD, \
				      bfa_ioctl_diag_regread_t)
#define IOCMD_DIAG_MEMRD	_IOWR(BFA_MAGIC, BFAD_DIAG_MEMRD, \
				      bfa_ioctl_diag_kva_t)
#define IOCMD_DIAG_REGWR	_IOWR(BFA_MAGIC, BFAD_DIAG_REGWR, \
				      bfa_ioctl_diag_regwrite_t)
#define IOCMD_DIAG_TEMP		_IOWR(BFA_MAGIC, BFAD_DIAG_TEMP, \
				      bfa_ioctl_diag_get_temp_t)
#define IOCMD_DIAG_POST		_IOWR(BFA_MAGIC, BFAD_DIAG_POST, \
				      bfa_ioctl_diag_post_t)
#define IOCMD_DIAG_MEMTEST	_IOWR(BFA_MAGIC, BFAD_DIAG_MEMTEST, \
				      bfa_ioctl_diag_memtest_t)
#define IOCMD_DIAG_LOOPBACK	_IOWR(BFA_MAGIC, BFAD_DIAG_LOOPBACK, \
				      bfa_ioctl_diag_loopback_t)
#define IOCMD_DIAG_FWPING	_IOWR(BFA_MAGIC, BFAD_DIAG_FWPING, \
				      bfa_ioctl_diag_fwping_t)
#define IOCMD_DIAG_QUEUETEST	_IOWR(BFA_MAGIC, BFAD_DIAG_QUEUETEST, \
				      bfa_ioctl_diag_qtest_t)
#define IOCMD_DIAG_POM		_IOWR(BFA_MAGIC, BFAD_DIAG_POM, \
				      bfa_ioctl_diag_pom_t)
#define IOCMD_DIAG_SFP		_IOWR(BFA_MAGIC, BFAD_DIAG_SFP, \
				      bfa_ioctl_diag_sfp_t)
#define IOCMD_DIAG_LED		_IOWR(BFA_MAGIC, BFAD_DIAG_LED, \
				      bfa_ioctl_diag_led_t)
#define IOCMD_DIAG_BEACON_LPORT	_IOWR(BFA_MAGIC, BFAD_DIAG_BEACON_LPORT, \
				      bfa_ioctl_diag_beacon_t)
#define IOCMD_DIAG_LL_LOOPBACK	_IOWR(BFA_MAGIC, BFAD_DIAG_LL_LOOPBACK, \
				      bfa_ioctl_diag_ll_loopback_t)
#define IOCMD_DIAG_LB_STAT	_IOWR(BFA_MAGIC, BFAD_DIAG_LB_STAT, \
				      bfa_ioctl_diag_lb_stat_t)
#define IOCMD_DIAG_DPORT_ENABLE	_IOWR(BFA_MAGIC, BFAD_DIAG_DPORT_ENABLE, \
				      bfa_ioctl_dport_enable_t)
#define IOCMD_DIAG_DPORT_DISABLE	_IOWR(BFA_MAGIC, \
					BFAD_DIAG_DPORT_DISABLE, \
				      bfa_ioctl_gen_t)
#define IOCMD_DIAG_DPORT_SHOW _IOWR(BFA_MAGIC, BFAD_DIAG_DPORT_SHOW, \
					bfa_ioctl_diag_dport_show_t)
#define IOCMD_DIAG_DPORT_START _IOWR(BFA_MAGIC, BFAD_DIAG_DPORT_START, \
					bfa_ioctl_dport_enable_t)
/* SFP */
#define IOCMD_SFP_SHOW		_IOWR(BFA_MAGIC, BFAD_SFP_SHOW, \
				      bfa_ioctl_sfp_show_t)
#define IOCMD_SFP_MEDIA		_IOWR(BFA_MAGIC, BFAD_SFP_MEDIA, \
				      bfa_ioctl_sfp_media_t)
#define IOCMD_SFP_SPEED		_IOWR(BFA_MAGIC, BFAD_SFP_SPEED, \
				      bfa_ioctl_sfp_speed_t)


/* DEBUG */
#define IOCMD_DEBUG_DRV_TRACE	_IOWR(BFA_MAGIC, BFAD_DEBUG_DRV_TRACE, \
				      bfa_ioctl_debug_t)
#define IOCMD_DEBUG_DRV_STATE	_IOWR(BFA_MAGIC, BFAD_DEBUG_DRV_STATE, \
				      bfa_ioctl_debug_t)
#define IOCMD_DEBUG_FW_TRACE	_IOWR(BFA_MAGIC, BFAD_DEBUG_FW_TRACE, \
				      bfa_ioctl_debug_t)
#define IOCMD_DEBUG_FW_STATE	_IOWR(BFA_MAGIC, BFAD_DEBUG_FW_STATE, \
				      bfa_ioctl_debug_t)
#define IOCMD_DEBUG_FW_STATE_CLR  _IOWR(BFA_MAGIC, BFAD_DEBUG_FW_STATE_CLR, \
					bfa_ioctl_gen_t)
#define IOCMD_DEBUG_FW_CORE	_IOWR(BFA_MAGIC, BFAD_DEBUG_FW_CORE, \
				      bfa_ioctl_debug_t)
#define IOCMD_DEBUG_PORTLOG	_IOWR(BFA_MAGIC, BFAD_DEBUG_PORTLOG, \
				      bfa_ioctl_debug_t)
#define IOCMD_DEBUG_PORTLOG_CLR	_IOWR(BFA_MAGIC, BFAD_DEBUG_PORTLOG_CLR, \
				      bfa_ioctl_gen_t)
#define IOCMD_DEBUG_PORTLOG_CTL	_IOWR(BFA_MAGIC, BFAD_DEBUG_PORTLOG_CTL, \
				      bfa_ioctl_portlogctl_t)
#define IOCMD_DEBUG_START_DTRC	_IOWR(BFA_MAGIC, BFAD_DEBUG_START_DTRC, \
				      bfa_ioctl_gen_t)
#define IOCMD_DEBUG_STOP_DTRC	_IOWR(BFA_MAGIC, BFAD_DEBUG_STOP_DTRC, \
				      bfa_ioctl_gen_t)

/* LOG */
#define IOCMD_LOG_SET_LEVEL	_IOWR(BFA_MAGIC, BFAD_LOG_SET_LEVEL, \
				      bfa_ioctl_log_t)
#define IOCMD_LOG_GET_LEVEL	_IOWR(BFA_MAGIC, BFAD_LOG_GET_LEVEL, \
				      bfa_ioctl_log_t)

/* AEN */
#define IOCMD_AEN_GET		_IOWR(BFA_MAGIC, BFAD_AEN_GET, \
				       bfa_ioctl_aen_t)
#define IOCMD_AEN_UPDATE	_IOWR(BFA_MAGIC, BFAD_AEN_UPDATE, \
				       bfa_ioctl_aen_t)

/* PORT */
#define IOCMD_PORT_ENABLE	_IOWR(BFA_MAGIC, BFAD_PORT_ENABLE, \
				      bfa_ioctl_gen_t)
#define IOCMD_PORT_DISABLE	_IOWR(BFA_MAGIC, BFAD_PORT_DISABLE, \
				      bfa_ioctl_gen_t)
#define IOCMD_PORT_GET_ATTR	_IOWR(BFA_MAGIC, BFAD_PORT_GET_ATTR, \
				      bfa_ioctl_port_attr_t)
#define IOCMD_PORT_GET_STATS	_IOWR(BFA_MAGIC, BFAD_PORT_GET_STATS, \
				      bfa_ioctl_port_stats_t)
#define IOCMD_PORT_GET_FC4STATS	_IOWR(BFA_MAGIC, BFAD_PORT_GET_FC4STATS, \
				      bfa_ioctl_port_fc4stats_t)
#define IOCMD_PORT_GET_FCPMAP	_IOWR(BFA_MAGIC, BFAD_PORT_GET_FCPMAP, \
				      bfa_ioctl_port_fcpmap_t)
#define IOCMD_PORT_RESET_STATS	_IOWR(BFA_MAGIC, BFAD_PORT_RESET_STATS, \
				      bfa_ioctl_gen_t)
#define IOCMD_PORT_CFG_TOPO    	_IOWR(BFA_MAGIC, BFAD_PORT_CFG_TOPO, \
				      bfa_ioctl_port_cfg_topo_t)
#define IOCMD_PORT_CFG_SPEED	_IOWR(BFA_MAGIC, BFAD_PORT_CFG_SPEED, \
				      bfa_ioctl_port_cfg_speed_t)
#define IOCMD_PORT_CFG_ALPA	_IOWR(BFA_MAGIC, BFAD_PORT_CFG_ALPA, \
				      bfa_ioctl_port_alpa_t)
#define IOCMD_PORT_CLR_ALPA	_IOWR(BFA_MAGIC, BFAD_PORT_CLR_ALPA, \
				      bfa_ioctl_port_alpa_t)
#define IOCMD_PORT_CFG_MAXFRSZ	_IOWR(BFA_MAGIC, BFAD_PORT_CFG_MAXFRSZ, \
				      bfa_ioctl_port_cfg_maxfrsize_t)
#define IOCMD_PORT_CFG_RNID    	_IOWR(BFA_MAGIC, BFAD_PORT_CFG_RNID, \
				      bfa_ioctl_port_rnid_t)
#define IOCMD_PORT_GET_RNID    	_IOWR(BFA_MAGIC, BFAD_PORT_GET_RNID, \
				      bfa_ioctl_port_rnid_t)
#define IOCMD_PORT_CFG_MODE	_IOWR(BFA_MAGIC, BFAD_PORT_CFG_MODE, \
				bfa_ioctl_port_cfg_mode_t)
#define IOCMD_PORT_BBCR_ENABLE	_IOWR(BFA_MAGIC, BFAD_PORT_BBCR_ENABLE,	\
					bfa_ioctl_bbcr_enable_t)
#define IOCMD_PORT_BBCR_DISABLE	_IOWR(BFA_MAGIC, BFAD_PORT_BBCR_DISABLE,\
					bfa_ioctl_gen_t)
#define IOCMD_PORT_BBCR_GET_ATTR _IOWR(BFA_MAGIC, BFAD_PORT_GET_BBCR_ATTR,\
					bfa_ioctl_bbcr_attr_t)
#define IOCMD_PORT_NWPAR_ENABLE	_IOWR(BFA_MAGIC, BFAD_PORT_NWPAR_ENABLE, \
					bfa_ioctl_nwpar_t)
#define IOCMD_PORT_NWPAR_DISABLE _IOWR(BFA_MAGIC, BFAD_PORT_NWPAR_DISABLE, \
					bfa_ioctl_nwpar_t)

/* CEE */
#define IOCMD_CEE_GET_ATTR	_IOWR(BFA_MAGIC, BFAD_CEE_GET_ATTR, \
				      bfa_ioctl_cee_attr_t)
#define IOCMD_CEE_GET_STATS	_IOWR(BFA_MAGIC, BFAD_CEE_GET_STATS, \
				      bfa_ioctl_cee_stats_t)
#define IOCMD_CEE_RESET_STATS	_IOWR(BFA_MAGIC, BFAD_CEE_RESET_STATS, \
				      bfa_ioctl_gen_t)

/* ETHBOOT */
#define IOCMD_ETHBOOT_CFG	_IOWR(BFA_MAGIC, BFAD_ETHBOOT_CFG, \
					bfa_ioctl_ethboot_t)
#define IOCMD_ETHBOOT_QUERY	_IOWR(BFA_MAGIC, BFAD_ETHBOOT_QUERY, \
					bfa_ioctl_ethboot_t)

/* ISCSIBOOT */
#define IOCMD_ISCSIBOOT_CFG	_IOWR(BFA_MAGIC, BFAD_ISCSIBOOT_CFG, \
					bfa_ioctl_iscsiboot_t)
#define IOCMD_ISCSIBOOT_QUERY	_IOWR(BFA_MAGIC, BFAD_ISCSIBOOT_QUERY, \
					bfa_ioctl_iscsiboot_t)

/* FLASH */
#define IOCMD_FLASH_GET_ATTR		\
	_IOWR(BFA_MAGIC, BFAD_FLASH_GET_ATTR, bfa_ioctl_flash_attr_t)
#define IOCMD_FLASH_ERASE_PART		\
	_IOWR(BFA_MAGIC, BFAD_FLASH_ERASE_PART, bfa_ioctl_flash_t)
#define IOCMD_FLASH_UPDATE_PART		\
	_IOWR(BFA_MAGIC, BFAD_FLASH_UPDATE_PART, bfa_ioctl_flash_t)
#define IOCMD_FLASH_READ_PART		\
	_IOWR(BFA_MAGIC, BFAD_FLASH_READ_PART, bfa_ioctl_flash_t)
#define IOCMD_FLASH_MFG_INFO		\
	_IOWR(BFA_MAGIC, BFAD_FLASH_MFG_INFO, bfa_ioctl_flash_mfg_t)
#define IOCMD_FLASH_ENABLE_OPTROM	\
	_IOWR(BFA_MAGIC, BFAD_FLASH_ENABLE_OPTROM, bfa_ioctl_gen_t)
#define IOCMD_FLASH_DISABLE_OPTROM	\
	_IOWR(BFA_MAGIC, BFAD_FLASH_DISABLE_OPTROM, bfa_ioctl_gen_t)
#define IOCMD_FLASH_GET_CFG_BOOTVER	\
	_IOWR(BFA_MAGIC, BFAD_FLASH_GET_CFG_BOOTVER, bfa_ioctl_flash_t)


/* PHY */
#define IOCMD_PHY_UPDATE_FW		\
	_IOWR(BFA_MAGIC, BFAD_PHY_UPDATE_FW, bfa_ioctl_phy_t)
#define IOCMD_PHY_READ_FW		\
	_IOWR(BFA_MAGIC, BFAD_PHY_READ_FW, bfa_ioctl_phy_t)
#define IOCMD_PHY_GET_ATTR		\
	_IOWR(BFA_MAGIC, BFAD_PHY_GET_ATTR, bfa_ioctl_phy_attr_t)
#define IOCMD_PHY_GET_STATS		\
	_IOWR(BFA_MAGIC, BFAD_PHY_GET_STATS, bfa_ioctl_phy_stats_t)
/* PCIFN */
#define IOCMD_PCIFN_CREATE		\
	_IOWR(BFA_MAGIC, BFAD_PCIFN_CREATE, bfa_ioctl_pcifn_t)
#define IOCMD_PCIFN_DELETE		\
	_IOWR(BFA_MAGIC, BFAD_PCIFN_DELETE, bfa_ioctl_pcifn_t)
#define IOCMD_PCIFN_ENABLE		\
	_IOWR(BFA_MAGIC, BFAD_PCIFN_ENABLE, bfa_ioctl_pcifn_t)
#define IOCMD_PCIFN_DISABLE		\
	_IOWR(BFA_MAGIC, BFAD_PCIFN_DISABLE, bfa_ioctl_pcifn_t)
#define IOCMD_PCIFN_PERS_CHANGE		\
	_IOWR(BFA_MAGIC, BFAD_PCIFN_PERS_CHANGE, bfa_ioctl_pcifn_t)
#define IOCMD_PCIFN_BW _IOWR(BFA_MAGIC, BFAD_PCIFN_BW, bfa_ioctl_pcifn_t)
/* VNIC */
#define IOCMD_VNIC_QUERY _IOWR(BFA_MAGIC, BFAD_VNIC_QUERY,\
						bfa_ioctl_vnic_attr_t)
#define IOCMD_VNIC_ENABLE _IOWR(BFA_MAGIC, BFAD_VNIC_ENABLE, bfa_ioctl_gen_t)
#define IOCMD_VNIC_DISABLE _IOWR(BFA_MAGIC, BFAD_VNIC_DISABLE, bfa_ioctl_gen_t)
#define IOCMD_VNIC_STATS _IOWR(BFA_MAGIC, BFAD_VNIC_STATS,\
						bfa_ioctl_vnic_stats_t)
#define IOCMD_VNIC_RESET_STATS _IOWR(BFA_MAGIC, BFAD_VNIC_RESET_STATS,\
						bfa_ioctl_gen_t)

/* VHBA */
#define IOCMD_VHBA_QUERY _IOWR(BFA_MAGIC, BFAD_VHBA_QUERY,\
						bfa_ioctl_vhba_attr_t)
#define IOCMD_VHBA_ENABLE _IOWR(BFA_MAGIC, BFAD_VHBA_ENABLE, bfa_ioctl_gen_t)
#define IOCMD_VHBA_DISABLE _IOWR(BFA_MAGIC, BFAD_VHBA_DISABLE, bfa_ioctl_gen_t)

/* TFRU */
#define IOCMD_TFRU_READ			\
	_IOWR(BFA_MAGIC, BFAD_TFRU_READ, bfa_ioctl_tfru_t)
#define IOCMD_TFRU_WRITE			\
	_IOWR(BFA_MAGIC, BFAD_TFRU_WRITE, bfa_ioctl_tfru_t)
/* FRU */
#define IOCMD_FRUVPD_READ			\
	_IOWR(BFA_MAGIC, BFAD_FRUVPD_READ, bfa_ioctl_fruvpd_t)
#define IOCMD_FRUVPD_UPDATE			\
	_IOWR(BFA_MAGIC, BFAD_FRUVPD_UPDATE, bfa_ioctl_fruvpd_t)

#define IOCMD_FRUVPD_GET_MAX_SIZE			\
	_IOWR(BFA_MAGIC, BFAD_FRUVPD_GET_MAX_SIZE,	\
		bfa_ioctl_fruvpd_max_size_t)
#endif /* __BFAD_IOCTL_CNA_H__ */
