/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */
#ifndef __BFI_CNA_H__
#define __BFI_CNA_H__

#include <bfi/bfi.h>
#include <defs/bfa_defs_svc.h>
#include <defs/bfa_defs_cna.h>

#pragma pack(1)

enum bfi_port_h2i {
	BFI_PORT_H2I_ENABLE_REQ		= (1),
	BFI_PORT_H2I_DISABLE_REQ	= (2),
	BFI_PORT_H2I_GET_STATS_REQ	= (3),
	BFI_PORT_H2I_CLEAR_STATS_REQ	= (4),
};
typedef enum bfi_port_h2i bfi_port_h2i_t;

enum bfi_port_i2h {
	BFI_PORT_I2H_ENABLE_RSP		= BFA_I2HM(1),
	BFI_PORT_I2H_DISABLE_RSP	= BFA_I2HM(2),
	BFI_PORT_I2H_GET_STATS_RSP	= BFA_I2HM(3),
	BFI_PORT_I2H_CLEAR_STATS_RSP	= BFA_I2HM(4),
};
typedef enum bfi_port_i2h bfi_port_i2h_t;

/**
 * Generic REQ type
 */
struct bfi_port_generic_req_s {
	struct bfi_mhdr_s  mh;		/*!< msg header			    */
	uint32_t	msgtag;		/*!< msgtag for reply		    */
	uint32_t	rsvd;
};
typedef struct bfi_port_generic_req_s bfi_port_generic_req_t;

/**
 * Generic RSP type
 */
struct bfi_port_generic_rsp_s {
	struct bfi_mhdr_s  mh;		/*!< common msg header		    */
	uint8_t		status;		/*!< port enable status		    */
	uint8_t		rsvd[3];
	uint32_t	msgtag;		/*!< msgtag for reply		    */
};
typedef struct bfi_port_generic_rsp_s bfi_port_generic_rsp_t;

/**
 * @todo
 * BFI_PORT_H2I_ENABLE_REQ
 */
typedef struct bfi_port_generic_req_s bfi_port_enable_req_t;

/**
 * @todo
 * BFI_PORT_I2H_ENABLE_RSP
 */
typedef struct bfi_port_generic_rsp_s bfi_port_enable_rsp_t;

/**
 * BFI_PORT_H2I_DISABLE_REQ
 */
typedef struct bfi_port_generic_req_s bfi_port_disable_req_t;

/**
 * BFI_PORT_I2H_DISABLE_RSP
 */
typedef struct bfi_port_generic_rsp_s bfi_port_disable_rsp_t;

/**
 * BFI_PORT_H2I_GET_STATS_REQ
 */
struct bfi_port_get_stats_req_s {
	struct bfi_mhdr_s  mh;		/*!< common msg header		    */
	union bfi_addr_u   dma_addr;
};
typedef struct bfi_port_get_stats_req_s bfi_port_get_stats_req_t;

/**
 * BFI_PORT_I2H_GET_STATS_RSP
 */
typedef struct bfi_port_generic_rsp_s bfi_port_get_stats_rsp_t;

/**
 * BFI_PORT_H2I_CLEAR_STATS_REQ
 */
typedef struct bfi_port_generic_req_s bfi_port_clear_stats_req_t;

/**
 * BFI_PORT_I2H_CLEAR_STATS_RSP
 */
typedef struct bfi_port_generic_rsp_s bfi_port_clear_stats_rsp_t;

union bfi_port_h2i_msg_u {
	struct bfi_mhdr_s		mh;
	struct bfi_port_generic_req_s	enable_req;
	struct bfi_port_generic_req_s	disable_req;
	struct bfi_port_get_stats_req_s	getstats_req;
	struct bfi_port_generic_req_s	clearstats_req;
};
typedef union bfi_port_h2i_msg_u bfi_port_h2i_msg_t;

union bfi_port_i2h_msg_u {
	struct bfi_mhdr_s		mh;
	struct bfi_port_generic_rsp_s	enable_rsp;
	struct bfi_port_generic_rsp_s	disable_rsp;
	struct bfi_port_generic_rsp_s	getstats_rsp;
	struct bfi_port_generic_rsp_s	clearstats_rsp;
};
typedef union bfi_port_i2h_msg_u bfi_port_i2h_msg_t;

/* @brief Mailbox commands from host to (DCBX/LLDP) firmware */
enum bfi_cee_h2i_msgs_e {
	BFI_CEE_H2I_GET_CFG_REQ = 1,
	BFI_CEE_H2I_RESET_STATS = 2,
	BFI_CEE_H2I_GET_STATS_REQ = 3,
	BFI_CEE_H2I_SET_CFG_REQ = 4,
};
typedef enum bfi_cee_h2i_msgs_e bfi_cee_h2i_msgs_t;


/* @brief Mailbox reply and AEN messages from DCBX/LLDP firmware to host */
enum bfi_cee_i2h_msgs_e {
	BFI_CEE_I2H_GET_CFG_RSP = BFA_I2HM(BFI_CEE_H2I_GET_CFG_REQ),
	BFI_CEE_I2H_RESET_STATS_RSP = BFA_I2HM(BFI_CEE_H2I_RESET_STATS),
	BFI_CEE_I2H_GET_STATS_RSP = BFA_I2HM(BFI_CEE_H2I_GET_STATS_REQ),
	BFI_CEE_I2H_SET_CFG_RSP = BFA_I2HM(BFI_CEE_H2I_SET_CFG_REQ),
};
typedef enum bfi_cee_i2h_msgs_e bfi_cee_i2h_msgs_t;


/* Data structures */

/*
 * @brief H2I command structure for resetting the stats.
 * BFI_CEE_H2I_RESET_STATS
 */
struct bfi_lldp_reset_stats_s {
	struct bfi_mhdr_s  mh;
};
typedef struct bfi_lldp_reset_stats_s bfi_lldp_reset_stats_t;

/*
 * @brief H2I command structure for resetting the stats.
 * BFI_CEE_H2I_RESET_STATS
 */
struct bfi_cee_reset_stats_s {
	struct bfi_mhdr_s  mh;
};
typedef struct bfi_cee_reset_stats_s bfi_cee_reset_stats_t;

/*
 * @brief  get configuration  command from host
 * BFI_CEE_H2I_GET_CFG_REQ
 */
struct bfi_cee_get_req_s {
	struct bfi_mhdr_s  mh;
	union bfi_addr_u   dma_addr;
};
typedef struct bfi_cee_get_req_s bfi_cee_get_req_t;


/*
 * @brief reply message from firmware
 * BFI_CEE_I2H_GET_CFG_RSP
 */
struct bfi_cee_get_rsp_s {
	struct bfi_mhdr_s	mh;
	uint8_t			cmd_status;
	uint8_t			rsvd[3];
};
typedef struct bfi_cee_get_rsp_s bfi_cee_get_rsp_t;

/*
 * @brief  get configuration  command from host
 * BFI_CEE_H2I_GET_STATS_REQ
 */
struct bfi_cee_stats_req_s {
	struct bfi_mhdr_s  mh;
	union bfi_addr_u   dma_addr;
};
typedef struct bfi_cee_stats_req_s bfi_cee_stats_req_t;


/*
 * @brief reply message from firmware
 * BFI_CEE_I2H_GET_STATS_RSP
 */
struct bfi_cee_stats_rsp_s {
	struct bfi_mhdr_s	mh;
	uint8_t			cmd_status;
	uint8_t			rsvd[3];
};
typedef struct bfi_cee_stats_rsp_s bfi_cee_stats_rsp_t;

/*
 * @brief  get configuration  command from host
 * BFI_CEE_H2I_SET_CFG_REQ
 */
struct bfi_cee_set_req_s {
	struct bfi_mhdr_s  mh;
	struct bfi_alen_s   alen;
};
typedef struct bfi_cee_set_req_s bfi_cee_set_req_t;


/*
 * @brief reply message from firmware
 * BFI_CEE_I2H_SET_CFG_RSP
 */
struct bfi_cee_set_rsp_s {
	struct bfi_mhdr_s	mh;
	uint8_t			cmd_status;
	uint8_t			rsvd[3];
};
typedef struct bfi_cee_set_rsp_s bfi_cee_set_rsp_t;


/* @brief mailbox command structures from host to firmware */
union bfi_cee_h2i_msg_u {
	struct bfi_mhdr_s		mh;
	struct bfi_cee_get_req_s	get_req;
	struct bfi_cee_stats_req_s	stats_req;
	struct bfi_cee_set_req_s	set_req;
};
typedef union bfi_cee_h2i_msg_u bfi_cee_h2i_msg_t;


/* @brief mailbox message structures from firmware to host	*/
union bfi_cee_i2h_msg_u {
	struct bfi_mhdr_s		mh;
	struct bfi_cee_get_rsp_s	get_rsp;
	struct bfi_cee_stats_rsp_s	stats_rsp;
	struct bfi_cee_set_rsp_s	set_rsp;
};
typedef union bfi_cee_i2h_msg_u bfi_cee_i2h_msg_t;

#pragma pack()


#endif /* __BFI_CNA_H__ */
