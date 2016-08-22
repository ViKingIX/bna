/*
 * Copyright (c) 2010-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BFA_FC_H__
#define __BFA_FC_H__

#include <bfa_os_inc.h>

typedef uint64_t wwn_t;
typedef uint64_t lun_t;

#include "protocol/scsi.h"

#define WWN_NULL	(0)
#define FC_SYMNAME_MAX	256	/*!< max name server symbolic name size */
#define FC_ALPA_MAX	128

#pragma pack(1)

#define MAC_ADDRLEN	(6)
typedef struct mac_s { uint8_t mac[MAC_ADDRLEN]; } mac_t;

/*
 * Fibre Channel Header Structure (FCHS) definition
 */
typedef struct fchs_s {
#ifdef __BIGENDIAN
	uint32_t	routing:4;	/* routing bits */
	uint32_t	cat_info:4;	/* category info */
#else
	uint32_t	cat_info:4;	/* category info */
	uint32_t	routing:4;	/* routing bits */
#endif
	uint32_t	d_id:24;	/* destination identifier */

	uint32_t	cs_ctl:8;	/* class specific control */
	uint32_t	s_id:24;	/* source identifier */

	uint32_t	type:8;		/* data structure type */
	uint32_t	f_ctl:24;	/* initial frame control */

	uint8_t		seq_id;		/* sequence identifier */
	uint8_t		df_ctl;		/* data field control */
	uint16_t	seq_cnt;	/* sequence count */

	uint16_t	ox_id;		/* originator exchange ID */
	uint16_t	rx_id;		/* responder exchange ID */

	uint32_t	ro;		/* relative offset */
} fchs_t;

#define FC_SOF_LEN		4
#define FC_EOF_LEN		4
#define FC_CRC_LEN		4

/*
 * Fibre Channel BB_E Header Structure
 */
typedef struct fcbbehs_s {
	uint16_t	ver_rsvd;
	uint32_t	rsvd[2];
	uint32_t	rsvd__sof;
} fcbbehs_t;

#define FC_SEQ_ID_MAX		256

/*
 * routing bit definitions
 */
enum {
	FC_RTG_FC4_DEV_DATA	= 0x0,	/* FC-4 Device Data */
	FC_RTG_EXT_LINK		= 0x2,	/* Extended Link Data */
	FC_RTG_FC4_LINK_DATA	= 0x3,	/* FC-4 Link Data */
	FC_RTG_VIDEO_DATA	= 0x4,	/* Video Data */
	FC_RTG_EXT_HDR		= 0x5,	/* VFT, IFR or Encapsuled */
	FC_RTG_BASIC_LINK	= 0x8,	/* Basic Link data */
	FC_RTG_LINK_CTRL	= 0xC,	/* Link Control */
};

/*
 * information category for extended link data and FC-4 Link Data
 */
enum {
	FC_CAT_LD_REQUEST	= 0x2,	/* Request */
	FC_CAT_LD_REPLY		= 0x3,	/* Reply */
	FC_CAT_LD_DIAG		= 0xF,	/* for DIAG use only */
};

/*
 * information category for extended headers (VFT, IFR or encapsulation)
 */
enum {
	FC_CAT_VFT_HDR = 0x0,	/* Virtual fabric tagging header */
	FC_CAT_IFR_HDR = 0x1,	/* Inter-Fabric routing header */
	FC_CAT_ENC_HDR = 0x2,	/* Encapsulation header */
};

/*
 * information category for FC-4 device data
 */
enum {
	FC_CAT_UNCATEG_INFO	= 0x0,	/* Uncategorized information */
	FC_CAT_SOLICIT_DATA	= 0x1,	/* Solicited Data */
	FC_CAT_UNSOLICIT_CTRL	= 0x2,	/* Unsolicited Control */
	FC_CAT_SOLICIT_CTRL	= 0x3,	/* Solicited Control */
	FC_CAT_UNSOLICIT_DATA	= 0x4,	/* Unsolicited Data */
	FC_CAT_DATA_DESC	= 0x5,	/* Data Descriptor */
	FC_CAT_UNSOLICIT_CMD	= 0x6,	/* Unsolicited Command */
	FC_CAT_CMD_STATUS	= 0x7,	/* Command Status */
};

/*
 * information category for Link Control
 */
enum {
	FC_CAT_ACK_1		= 0x00,
	FC_CAT_ACK_0_N		= 0x01,
	FC_CAT_P_RJT		= 0x02,
	FC_CAT_F_RJT		= 0x03,
	FC_CAT_P_BSY		= 0x04,
	FC_CAT_F_BSY_DATA	= 0x05,
	FC_CAT_F_BSY_LINK_CTL	= 0x06,
	FC_CAT_F_LCR		= 0x07,
	FC_CAT_NTY		= 0x08,
	FC_CAT_END		= 0x09,
};

/*
 * Type Field Definitions. FC-PH Section 18.5 pg. 165
 */
enum {
	FC_TYPE_BLS		= 0x0,	/* Basic Link Service */
	FC_TYPE_ELS		= 0x1,	/* Extended Link Service */
	FC_TYPE_IP		= 0x5,	/* IP */
	FC_TYPE_FCP		= 0x8,	/* SCSI-FCP */
	FC_TYPE_GPP		= 0x9,	/* SCSI_GPP */
	FC_TYPE_SERVICES	= 0x20,	/* Fibre Channel Services */
	FC_TYPE_FC_FSS		= 0x22,	/* Fabric Switch Services */
	FC_TYPE_FC_AL		= 0x23,	/* FC-AL */
	FC_TYPE_FC_SNMP		= 0x24,	/* FC-SNMP */
	FC_TYPE_FC_SPINFAB	= 0xEE,	/* SPINFAB */
	FC_TYPE_FC_DIAG		= 0xEF,	/* DIAG */
	FC_TYPE_MAX		= 256,	/* 256 FC-4 types */
};

typedef struct {
	uint8_t		bits[FC_TYPE_MAX / 8];
} fc_fc4types_t;

/*
 * Frame Control Definitions. FC-PH Table-45. pg. 168
 */
enum {
	FCTL_EC_ORIG = 0x000000,	/* exchange originator */
	FCTL_EC_RESP = 0x800000,	/* exchange responder */
	FCTL_SEQ_INI = 0x000000,	/* sequence initiator */
	FCTL_SEQ_REC = 0x400000,	/* sequence recipient */
	FCTL_FS_EXCH = 0x200000,	/* first sequence of xchg */
	FCTL_LS_EXCH = 0x100000,	/* last sequence of xchg */
	FCTL_END_SEQ = 0x080000,	/* last frame of sequence */
	FCTL_SI_XFER = 0x010000,	/* seq initiative transfer */
	FCTL_RO_PRESENT = 0x000008,	/* relative offset present */
	FCTL_FILLBYTE_MASK = 0x000003	/* , fill byte mask */
};

/*
 * Fabric Well Known Addresses
 */
enum {
	FC_MIN_WELL_KNOWN_ADDR		= 0xFFFFF0,
	FC_DOMAIN_CONTROLLER_MASK 	= 0xFFFC00,
	FC_ALIAS_SERVER			= 0xFFFFF8,
	FC_MGMT_SERVER			= 0xFFFFFA,
	FC_TIME_SERVER			= 0xFFFFFB,
	FC_NAME_SERVER			= 0xFFFFFC,
	FC_FABRIC_CONTROLLER		= 0xFFFFFD,
	FC_FABRIC_PORT			= 0xFFFFFE,
	FC_BROADCAST_SERVER		= 0xFFFFFF
};

/*
 * domain/area/port defines
 */
#define FC_DOMAIN_MASK  0xFF0000
#define FC_DOMAIN_SHIFT 16
#define FC_AREA_MASK    0x00FF00
#define FC_AREA_SHIFT   8
#define FC_PORT_MASK    0x0000FF
#define FC_PORT_SHIFT   0

#define FC_GET_DOMAIN(p)	(((p) & FC_DOMAIN_MASK) >> FC_DOMAIN_SHIFT)
#define FC_GET_AREA(p)		(((p) & FC_AREA_MASK) >> FC_AREA_SHIFT)
#define FC_GET_PORT(p)		(((p) & FC_PORT_MASK) >> FC_PORT_SHIFT)

#define FC_DOMAIN_CTRLR(p)	(FC_DOMAIN_CONTROLLER_MASK | (FC_GET_DOMAIN(p)))

enum {
	FC_RXID_ANY = 0xFFFFU,
};

/**
 * p2p topology defines
 */
#define N2N_LOCAL_PID   0x01
#define N2N_REMOTE_PID  0x02

/*
 * generic ELS command
 */
typedef struct {
uint32_t	els_code:8;	/* ELS Command Code */
uint32_t	reserved:24;
} fc_els_cmd_t;

/*
 * ELS Command Codes. FC-PH Table-75. pg. 223
 */
enum {
	FC_ELS_LS_RJT = 0x1,	/* Link Service Reject. */
	FC_ELS_ACC = 0x02,	/* Accept */
	FC_ELS_PLOGI = 0x03,	/* N_Port Login. */
	FC_ELS_FLOGI = 0x04,	/* F_Port Login. */
	FC_ELS_LOGO = 0x05,	/* Logout. */
	FC_ELS_ABTX = 0x06,	/* Abort Exchange */
	FC_ELS_RES = 0x08,	/* Read Exchange status */
	FC_ELS_RSS = 0x09,	/* Read sequence status block */
	FC_ELS_RSI = 0x0A,	/* Request Sequence Initiative */
	FC_ELS_ESTC = 0x0C,	/* Estimate Credit. */
	FC_ELS_RTV = 0x0E,	/* Read Timeout Value. */
	FC_ELS_RLS = 0x0F,	/* Read Link Status. */
	FC_ELS_ECHO = 0x10,	/* Echo */
	FC_ELS_TEST = 0x11,	/* Test */
	FC_ELS_RRQ = 0x12,	/* Reinstate Recovery Qualifier. */
	FC_ELS_REC = 0x13,	/* Add this for TAPE support in FCR */
	FC_ELS_SRR = 0x14,	/* SRR */
	FC_ELS_PRLI = 0x20,	/* Process Login */
	FC_ELS_PRLO = 0x21,	/* Process Logout. */
	FC_ELS_SCN = 0x22,	/* State Change Notification. */
	FC_ELS_TPRLO = 0x24,	/* Third Party Process Logout. */
	FC_ELS_PDISC = 0x50,	/* Discover N_Port Parameters. */
	FC_ELS_FDISC = 0x51,	/* Discover F_Port Parameters. */
	FC_ELS_ADISC = 0x52,	/* Discover Address. */
	FC_ELS_FARP_REQ = 0x54,	/* FARP Request. */
	FC_ELS_FARP_REP = 0x55,	/* FARP Reply. */
	FC_ELS_FAN = 0x60,	/* Fabric Address Notification */
	FC_ELS_RSCN = 0x61,	/* Reg State Change Notification */
	FC_ELS_SCR = 0x62,	/* State Change Registration. */
	FC_ELS_RTIN = 0x77,	/* Mangement server request */
	FC_ELS_RNID = 0x78,	/* Mangement server request */
	FC_ELS_RLIR = 0x79,	/* Registered Link Incident Record */

	FC_ELS_RPSC = 0x7D,	/* Report Port Speed Capabilities */
	FC_ELS_QSA = 0x7E,	/* Query Security Attributes. Ref FC-SP */
	FC_ELS_E2E_LBEACON = 0x81,
				/* End-to-End Link Beacon */
	FC_ELS_AUTH = 0x90,	/* Authentication. Ref FC-SP */
	FC_ELS_RFCN = 0x97,	/* Request Fabric Change Notification. Ref
				 *FC-SP */

};

/*
 *  Version numbers for FC-PH standards,
 *  used in login to indicate what port
 *  supports. See FC-PH-X table 158.
 */
enum {
	FC_PH_VER_4_3 = 0x09,
	FC_PH_VER_PH_3 = 0x20,
};

/*
 * PDU size defines
 */
enum {
	FC_MIN_PDUSZ = 512,
	FC_MAX_PDUSZ = 2112,
};

/*
 * N_Port PLOGI Common Service Parameters.
 * FC-PH-x. Figure-76. pg. 308.
 */
typedef struct {
	uint8_t		verhi;	/* FC-PH high version */
	uint8_t		verlo;	/* FC-PH low version */
	uint16_t	bbcred;	/* BB_Credit */

#ifdef __BIGENDIAN
	uint8_t	ciro:1,		/* continuously increasing RO */
			rro:1,		/* random relative offset */
			npiv_supp:1,	/* NPIV supported */
			port_type:1,	/* N_Port/F_port */
			altbbcred:1,	/* alternate BB_Credit */
			resolution:1,	/* ms/ns ED_TOV resolution */
			vvl_info:1,	/* VVL Info included */
			reserved1:1;

	uint8_t	hg_supp:1,
			query_dbc:1,
			security:1,
			sync_cap:1,
			r_t_tov:1,
			dh_dup_supp:1,
			cisc:1,		/* continuously increasing seq count */
			payload:1;
#else
	uint8_t	reserved2:2,
			resolution:1,	/* ms/ns ED_TOV resolution */
			altbbcred:1,	/* alternate BB_Credit */
			port_type:1,	/* N_Port/F_port */
			npiv_supp:1,	/* NPIV supported */
			rro:1,		/* random relative offset */
			ciro:1;		/* continuously increasing RO */

	uint8_t	payload:1,
			cisc:1,		/* continuously increasing seq count */
			dh_dup_supp:1,
			r_t_tov:1,
			sync_cap:1,
			security:1,
			query_dbc:1,
			hg_supp:1;
#endif

	uint16_t	rxsz;		/* recieve data_field size */

	uint16_t	conseq;
	uint16_t	ro_bitmap;

	uint32_t	e_d_tov;
} fc_plogi_csp_t;

/*
 * N_Port PLOGI Class Specific Parameters.
 * FC-PH-x. Figure 78. pg. 318.
 */
typedef struct {
#ifdef __BIGENDIAN
	uint32_t	class_valid:1;
	uint32_t	intermix:1;	/* class intermix supported if set =1.
					 * valid only for class1. Reserved for
					 * class2 & class3
					 */
	uint32_t	reserved1:2;
	uint32_t	sequential:1;
	uint32_t	reserved2:3;
#else
	uint32_t	reserved2:3;
	uint32_t	sequential:1;
	uint32_t	reserved1:2;
	uint32_t	intermix:1;	/* class intermix supported if set =1.
					 * valid only for class1. Reserved for
					 * class2 & class3
					 */
	uint32_t	class_valid:1;
#endif

	uint32_t	reserved3:24;

	uint32_t	reserved4:16;
	uint32_t	rxsz:16;	/* Receive data_field size */

	uint32_t	reserved5:8;
	uint32_t	conseq:8;
	uint32_t	e2e_credit:16;	/* end to end credit */

	uint32_t	reserved7:8;
	uint32_t	ospx:8;
	uint32_t	reserved8:16;
} fc_plogi_clp_t;

#define FLOGI_VVL_BRCD    0x42524344 /*ASCII value for each character in string "BRCD" */

/*
 * PLOGI els command and reply payload
 */
typedef struct {
	fc_els_cmd_t    els_cmd;	/* ELS command code */
	fc_plogi_csp_t  csp;		/* common service params */
	wwn_t			port_name;
	wwn_t			node_name;
	fc_plogi_clp_t  class1;		/* class 1 service parameters */
	fc_plogi_clp_t  class2;		/* class 2 service parameters */
	fc_plogi_clp_t  class3;		/* class 3 service parameters */
	fc_plogi_clp_t  class4;		/* class 4 service parameters */
	uint8_t			vvl[16];	/* vendor version level */
} fc_plogi_t, fc_flogi_t;

/*
 * LOGO els command payload
 */
typedef struct {
	fc_els_cmd_t    els_cmd;	/* ELS command code */
	uint32_t		res1:8;
	uint32_t		nport_id:24;	/* N_Port identifier of source */
	wwn_t			orig_port_name;	/* Port name of the LOGO originator */
} fc_logo_t;

/*
 * ADISC els command payload
 */
typedef struct {
	fc_els_cmd_t    els_cmd;	/* ELS command code */
	uint32_t		res1:8;
	uint32_t		orig_HA:24;	/* originator hard address */
	wwn_t			orig_port_name;	/* originator port name */
	wwn_t			orig_node_name;	/* originator node name */
	uint32_t		res2:8;
	uint32_t		nport_id:24;	/* originator NPortID */
} fc_adisc_t;

/*
 * Exchange status block
 */
typedef struct {
	uint32_t	oxid:16;
	uint32_t	rxid:16;
	uint32_t	res1:8;
	uint32_t	orig_np:24;	/* originator NPortID */
	uint32_t	res2:8;
	uint32_t	resp_np:24;	/* responder NPortID */
	uint32_t	es_bits;
	uint32_t	res3;
	/*
	 * un modified section of the fields
	 */
} fc_exch_status_blk_t;

/*
 * RES els command payload
 */
typedef struct {
	fc_els_cmd_t    els_cmd;	/* ELS command code */
	uint32_t		res1:8;
	uint32_t		nport_id:24;	/* N_Port identifier of source */
	uint32_t		oxid:16;
	uint32_t		rxid:16;
	uint8_t			assoc_hdr[32];
} fc_res_t;

/*
 * RES els accept payload
 */
typedef struct {
	fc_els_cmd_t		els_cmd;	/* ELS command code */
	fc_exch_status_blk_t	fc_exch_blk;	/* Exchange status block */
} fc_res_acc_t;

/*
 * REC els command payload
 */
typedef struct _fc_rec_t {
	fc_els_cmd_t    els_cmd;	/* ELS command code */
	uint32_t		res1:8;
	uint32_t		nport_id:24;	/* N_Port identifier of source */
	uint32_t		oxid:16;
	uint32_t		rxid:16;
} fc_rec_t;

#define FC_REC_ESB_OWN_RSP	0x80000000	/* responder owns */
#define FC_REC_ESB_SI		0x40000000	/* SI is owned 	*/
#define FC_REC_ESB_COMP		0x20000000	/* exchange is complete	*/
#define FC_REC_ESB_ENDCOND_ABN	0x10000000	/* abnormal ending 	*/
#define FC_REC_ESB_RQACT	0x04000000	/* recovery qual active	*/
#define FC_REC_ESB_ERRP_MSK	0x03000000
#define FC_REC_ESB_OXID_INV	0x00800000	/* invalid OXID		*/
#define FC_REC_ESB_RXID_INV	0x00400000	/* invalid RXID		*/
#define FC_REC_ESB_PRIO_INUSE	0x00200000

/*
 * REC els accept payload
 */
typedef struct _fc_rec_acc_t {
	fc_els_cmd_t    els_cmd;	/* ELS command code */
	uint32_t		oxid:16;
	uint32_t		rxid:16;
	uint32_t		res1:8;
	uint32_t		orig_id:24;	/* N_Port id of exchange originator */
	uint32_t		res2:8;
	uint32_t		resp_id:24;	/* N_Port id of exchange responder */
	uint32_t		count;		/* data transfer count */
	uint32_t		e_stat;		/* exchange status */
} fc_rec_acc_t;

/*
 * RSI els payload
 */
typedef struct _fc_rsi_t {
	fc_els_cmd_t    els_cmd;
	uint32_t		res1:8;
	uint32_t		orig_sid:24;
	uint32_t		oxid:16;
	uint32_t		rxid:16;
} fc_rsi_t;

/*
 * structure for PRLI paramater pages, both request & response
 * see FC-PH-X table 113 & 115 for explanation also FCP table 8
 */
typedef struct {
	uint32_t		reserved: 16;
#ifdef __BIGENDIAN
	uint32_t		reserved1: 5;
	uint32_t		rec_support : 1;
	uint32_t		task_retry_id : 1;
	uint32_t		retry : 1;

	uint32_t		confirm : 1;
	uint32_t		doverlay:1;
	uint32_t		initiator:1;
	uint32_t		target:1;
	uint32_t		cdmix:1;
	uint32_t		drmix:1;
	uint32_t		rxrdisab:1;
	uint32_t		wxrdisab:1;
#else
	uint32_t		retry : 1;
	uint32_t		task_retry_id : 1;
	uint32_t		rec_support : 1;
	uint32_t		reserved1: 5;

	uint32_t		wxrdisab:1;
	uint32_t		rxrdisab:1;
	uint32_t		drmix:1;
	uint32_t		cdmix:1;
	uint32_t		target:1;
	uint32_t		initiator:1;
	uint32_t		doverlay:1;
	uint32_t		confirm : 1;
#endif
} fc_prli_params_t;

/*
 * valid values for rspcode in PRLI ACC payload
 */
enum {
	FC_PRLI_ACC_XQTD = 0x1,		/* request executed */
	FC_PRLI_ACC_PREDEF_IMG = 0x5,	/* predefined image - no prli needed */
};

typedef struct {
	uint32_t	type:8;
	uint32_t	codext:8;
#ifdef __BIGENDIAN
	uint32_t	origprocasv:1;
	uint32_t	rsppav:1;
	uint32_t	imagepair:1;
	uint32_t	reserved1:1;
	uint32_t	rspcode:4;
#else
	uint32_t	rspcode:4;
	uint32_t	reserved1:1;
	uint32_t	imagepair:1;
	uint32_t	rsppav:1;
	uint32_t	origprocasv:1;
#endif
	uint32_t	reserved2:8;

	uint32_t	origprocas;
	uint32_t	rspprocas;
	fc_prli_params_t servparams;
} fc_prli_params_page_t;

/*
 * PRLI request and accept payload, FC-PH-X tables 112 & 114
 */
typedef struct {
	uint32_t	command:8;
	uint32_t	pglen:8;
	uint32_t	pagebytes:16;
	fc_prli_params_page_t parampage;
} fc_prli_t;

/*
 * PRLO logout params page
 */
typedef struct {
	uint32_t	type:8;
	uint32_t	type_ext:8;
#ifdef __BIGENDIAN
	uint32_t	opa_valid:1;	/* originator process associator
					 * valid
					 */
	uint32_t	rpa_valid:1;	/* responder process associator valid */
	uint32_t	res1:14;
#else
	uint32_t	res1:14;
	uint32_t	rpa_valid:1;	/* responder process associator valid */
	uint32_t	opa_valid:1;	/* originator process associator
					 * valid
					 */
#endif
	uint32_t	orig_process_assc;
	uint32_t	resp_process_assc;

	uint32_t	res2;
} fc_prlo_params_page_t;

/*
 * PRLO els command payload
 */
typedef struct {
	uint32_t	command:8;
	uint32_t	page_len:8;
	uint32_t	payload_len:16;
	fc_prlo_params_page_t 	prlo_params[1];
} fc_prlo_t;

/*
 * PRLO Logout response parameter page
 */
typedef struct {
	uint32_t	type:8;
	uint32_t	type_ext:8;

#ifdef __BIGENDIAN
	uint32_t	opa_valid:1;	/* originator process associator
					 * valid
					 */
	uint32_t	rpa_valid:1;	/* responder process associator valid */
	uint32_t	res1:14;
#else
	uint32_t	res1:14;
	uint32_t	rpa_valid:1;	/* responder process associator valid */
	uint32_t	opa_valid:1;	/* originator process associator
					 * valid
					 */
#endif
	uint32_t	orig_process_assc;
	uint32_t	resp_process_assc;

	uint32_t	fc4type_csp;
} fc_prlo_acc_params_page_t;

/*
 * PRLO els command ACC payload
 */
typedef struct {
	uint32_t	command:8;
	uint32_t	page_len:8;
	uint32_t	payload_len:16;
	fc_prlo_acc_params_page_t prlo_acc_params[1];
} fc_prlo_acc_t;

/*
 * SCR els command payload
 */
enum {
	FC_SCR_REG_FUNC_FABRIC_DETECTED = 0x01,
	FC_SCR_REG_FUNC_N_PORT_DETECTED = 0x02,
	FC_SCR_REG_FUNC_FULL = 0x03,
	FC_SCR_REG_FUNC_CLEAR_REG = 0xFF,
};

/* SCR VU registrations */
enum {
	FC_VU_SCR_REG_FUNC_FABRIC_NAME_CHANGE = 0x01
};

typedef struct {
	uint32_t	command:8;
	uint32_t	res:24;
	uint32_t	vu_reg_func:8; /* Vendor Unique Registrations */
	uint32_t	res1:16;
	uint32_t	reg_func:8;
} fc_scr_t;

/*
 * Information category for Basic link data
 */
enum {
	FC_CAT_NOP	= 0x0,
	FC_CAT_ABTS	= 0x1,
	FC_CAT_RMC	= 0x2,
	FC_CAT_BA_ACC	= 0x4,
	FC_CAT_BA_RJT	= 0x5,
	FC_CAT_PRMT	= 0x6,
};

/*
 * LS_RJT els reply payload
 */
typedef struct {
	fc_els_cmd_t	els_cmd;		/* ELS command code */
	uint32_t		res1:8;
	uint32_t		reason_code:8;		/* Reason code for reject */
	uint32_t		reason_code_expl:8;	/* Reason code explanation */
	uint32_t		vendor_unique:8;	/* Vendor specific */
} fc_ls_rjt_t;

/*
 * LS_RJT reason codes
 */
enum {
	FC_LS_RJT_RSN_INV_CMD_CODE	= 0x01,
	FC_LS_RJT_RSN_LOGICAL_ERROR	= 0x03,
	FC_LS_RJT_RSN_LOGICAL_BUSY	= 0x05,
	FC_LS_RJT_RSN_PROTOCOL_ERROR	= 0x07,
	FC_LS_RJT_RSN_UNABLE_TO_PERF_CMD = 0x09,
	FC_LS_RJT_RSN_CMD_NOT_SUPP	= 0x0B,
	FC_LS_RJT_RSN_VENDOR_SPECIFIC	= 0xFF,
};

/*
 * LS_RJT reason code explanation
 */
enum {
	FC_LS_RJT_EXP_NO_ADDL_INFO		= 0x00,
	FC_LS_RJT_EXP_SPARMS_ERR_OPTIONS	= 0x01,
	FC_LS_RJT_EXP_SPARMS_ERR_INI_CTL	= 0x03,
	FC_LS_RJT_EXP_SPARMS_ERR_REC_CTL	= 0x05,
	FC_LS_RJT_EXP_SPARMS_ERR_RXSZ		= 0x07,
	FC_LS_RJT_EXP_SPARMS_ERR_CONSEQ		= 0x09,
	FC_LS_RJT_EXP_SPARMS_ERR_CREDIT		= 0x0B,
	FC_LS_RJT_EXP_INV_PORT_NAME		= 0x0D,
	FC_LS_RJT_EXP_INV_NODE_FABRIC_NAME	= 0x0E,
	FC_LS_RJT_EXP_INV_CSP			= 0x0F,
	FC_LS_RJT_EXP_INV_ASSOC_HDR		= 0x11,
	FC_LS_RJT_EXP_ASSOC_HDR_REQD		= 0x13,
	FC_LS_RJT_EXP_INV_ORIG_S_ID		= 0x15,
	FC_LS_RJT_EXP_INV_OXID_RXID_COMB	= 0x17,
	FC_LS_RJT_EXP_CMD_ALREADY_IN_PROG	= 0x19,
	FC_LS_RJT_EXP_LOGIN_REQUIRED		= 0x1E,
	FC_LS_RJT_EXP_INVALID_NPORT_ID		= 0x1F,
	FC_LS_RJT_EXP_INSUFF_RES		= 0x29,
	FC_LS_RJT_EXP_UNABLE_SUPPLY_DATA	= 0x2A,
	FC_LS_RJT_EXP_CMD_NOT_SUPP		= 0x2C,
	FC_LS_RJT_EXP_INV_PAYLOAD_LEN		= 0x2D,
	FC_LS_RJT_EXP_INV_PORT_NODE_NAME	= 0x44,
};

/*
 * RRQ els command payload
 */
typedef struct {
	fc_els_cmd_t    els_cmd;	/* ELS command code */
	uint32_t        res1:8;
	uint32_t        s_id:24;	/* exchange originator S_ID */

	uint32_t        ox_id:16;	/* originator exchange ID */
	uint32_t        rx_id:16;	/* responder exchange ID */

	uint32_t        res2[8];	/* optional association header */
} fc_rrq_t;

/*
 * ABTS BA_ACC reply payload
 */
typedef struct {
	uint32_t        seq_id_valid:8;	/* set to 0x00 for Abort Exchange */
	uint32_t        seq_id:8;	/* invalid for Abort Exchange */
	uint32_t        res2:16;
	uint32_t        ox_id:16;	/* OX_ID from ABTS frame */
	uint32_t        rx_id:16;	/* RX_ID from ABTS frame */
	uint32_t        low_seq_cnt:16;	/* set to 0x0000 for Abort Exchange */
	uint32_t        high_seq_cnt:16;/* set to 0xFFFF for Abort Exchange */
} fc_ba_acc_t;

/*
 * ABTS BA_RJT reject payload
 */
typedef struct {
	uint32_t        res1:8;		/* Reserved */
	uint32_t        reason_code:8;	/* reason code for reject */
	uint32_t        reason_expl:8;	/* reason code explanation */
	uint32_t        vendor_unique:8;/* vendor unique reason code,set to 0 */
} fc_ba_rjt_t;

/*
 * TPRLO logout parameter page
 */
typedef struct {
uint32_t        type:8;
uint32_t        type_ext:8;

#ifdef __BIGENDIAN
	uint32_t        opa_valid:1;
	uint32_t        rpa_valid:1;
	uint32_t        tpo_nport_valid:1;
	uint32_t        global_process_logout:1;
	uint32_t        res1:12;
#else
	uint32_t        res1:12;
	uint32_t        global_process_logout:1;
	uint32_t        tpo_nport_valid:1;
	uint32_t        rpa_valid:1;
	uint32_t        opa_valid:1;
#endif

	uint32_t        orig_process_assc;
	uint32_t        resp_process_assc;

	uint32_t        res2:8;
	uint32_t        tpo_nport_id;
} fc_tprlo_params_page_t;

/*
 * TPRLO ELS command payload
 */
typedef struct {
	uint32_t        command:8;
	uint32_t        page_len:8;
	uint32_t        payload_len:16;

	fc_tprlo_params_page_t tprlo_params[1];
} fc_tprlo_t;

typedef enum {
	FC_GLOBAL_LOGO = 1,
	FC_TPR_LOGO
} fc_tprlo_type_t;

/*
 * TPRLO els command ACC payload
 */
typedef struct {
	uint32_t	command:8;
	uint32_t	page_len:8;
	uint32_t	payload_len:16;
	fc_prlo_acc_params_page_t tprlo_acc_params[1];
} fc_tprlo_acc_t;

/*
 * RSCN els command req payload
 */
#define FC_RSCN_PGLEN	0x4

typedef enum {
	FC_RSCN_FORMAT_PORTID	= 0x0,
	FC_RSCN_FORMAT_AREA	= 0x1,
	FC_RSCN_FORMAT_DOMAIN	= 0x2,
	FC_RSCN_FORMAT_FABRIC	= 0x3,
} fc_rscn_format_t;

typedef struct {
	uint32_t        format:2;
	uint32_t        qualifier:4;
	uint32_t        resvd:2;
	uint32_t        portid:24;
} fc_rscn_event_t;

typedef struct {
	uint8_t         command;
	uint8_t         pagelen;
	uint16_t        payldlen;
	fc_rscn_event_t event[1];
} fc_rscn_pl_t;

/*
 * ECHO els command req payload
 */
typedef struct _fc_echo_t {
	fc_els_cmd_t    els_cmd;
} fc_echo_t;

/*
 * RNID els command
 */

#define RNID_NODEID_DATA_FORMAT_COMMON    		 0x00
#define RNID_NODEID_DATA_FORMAT_FCP3        		 0x08
#define RNID_NODEID_DATA_FORMAT_DISCOVERY     		0xDF

#define RNID_ASSOCIATED_TYPE_UNKNOWN                    0x00000001
#define RNID_ASSOCIATED_TYPE_OTHER                      0x00000002
#define RNID_ASSOCIATED_TYPE_HUB                        0x00000003
#define RNID_ASSOCIATED_TYPE_SWITCH                     0x00000004
#define RNID_ASSOCIATED_TYPE_GATEWAY                    0x00000005
#define RNID_ASSOCIATED_TYPE_STORAGE_DEVICE             0x00000009
#define RNID_ASSOCIATED_TYPE_HOST                       0x0000000A
#define RNID_ASSOCIATED_TYPE_STORAGE_SUBSYSTEM          0x0000000B
#define RNID_ASSOCIATED_TYPE_STORAGE_ACCESS_DEVICE      0x0000000E
#define RNID_ASSOCIATED_TYPE_NAS_SERVER                 0x00000011
#define RNID_ASSOCIATED_TYPE_BRIDGE                     0x00000002
#define RNID_ASSOCIATED_TYPE_VIRTUALIZATION_DEVICE      0x00000003
#define RNID_ASSOCIATED_TYPE_MULTI_FUNCTION_DEVICE      0x000000FF

/*
 * RNID els command payload
 */
typedef struct {
	fc_els_cmd_t    els_cmd;
	uint32_t        node_id_data_format:8;
	uint32_t        reserved:24;
} fc_rnid_cmd_t;

/*
 * RNID els response payload
 */

typedef struct {
	wwn_t           port_name;
	wwn_t           node_name;
} fc_rnid_common_id_data_t;

typedef struct {
	uint32_t        vendor_unique[4];
	uint32_t        asso_type;
	uint32_t        phy_port_num;
	uint32_t        num_attached_nodes;
	uint32_t        node_mgmt:8;
	uint32_t        ip_version:8;
	uint32_t        udp_tcp_port_num:16;
	uint32_t        ip_address[4];
	uint32_t        reserved:16;
	uint32_t        vendor_specific:16;
} fc_rnid_general_topology_data_t;

typedef struct {
	fc_els_cmd_t    els_cmd;
	uint32_t        node_id_data_format:8;
	uint32_t        common_id_data_length:8;
	uint32_t        reserved:8;
	uint32_t        specific_id_data_length:8;
	fc_rnid_common_id_data_t common_id_data;
	fc_rnid_general_topology_data_t gen_topology_data;
} fc_rnid_acc_t;

#define RNID_ASSOCIATED_TYPE_UNKNOWN                    0x00000001
#define RNID_ASSOCIATED_TYPE_OTHER                      0x00000002
#define RNID_ASSOCIATED_TYPE_HUB                        0x00000003
#define RNID_ASSOCIATED_TYPE_SWITCH                     0x00000004
#define RNID_ASSOCIATED_TYPE_GATEWAY                    0x00000005
#define RNID_ASSOCIATED_TYPE_STORAGE_DEVICE             0x00000009
#define RNID_ASSOCIATED_TYPE_HOST                       0x0000000A
#define RNID_ASSOCIATED_TYPE_STORAGE_SUBSYSTEM          0x0000000B
#define RNID_ASSOCIATED_TYPE_STORAGE_ACCESS_DEVICE      0x0000000E
#define RNID_ASSOCIATED_TYPE_NAS_SERVER                 0x00000011
#define RNID_ASSOCIATED_TYPE_BRIDGE                     0x00000002
#define RNID_ASSOCIATED_TYPE_VIRTUALIZATION_DEVICE      0x00000003
#define RNID_ASSOCIATED_TYPE_MULTI_FUNCTION_DEVICE      0x000000FF

typedef enum {
	RPSC_SPEED_CAP_1G = 0x8000,
	RPSC_SPEED_CAP_2G = 0x4000,
	RPSC_SPEED_CAP_4G = 0x2000,
	RPSC_SPEED_CAP_10G = 0x1000,
	RPSC_SPEED_CAP_8G = 0x0800,
	RPSC_SPEED_CAP_16G = 0x0400,

	RPSC_SPEED_CAP_UNKNOWN = 0x0001,
} fc_rpsc_speed_cap_t;

typedef enum {
	RPSC_OP_SPEED_1G = 0x8000,
	RPSC_OP_SPEED_2G = 0x4000,
	RPSC_OP_SPEED_4G = 0x2000,
	RPSC_OP_SPEED_10G = 0x1000,
	RPSC_OP_SPEED_8G = 0x0800,
	RPSC_OP_SPEED_16G = 0x0400,

	RPSC_OP_SPEED_NOT_EST = 0x0001,	/*! speed not established */
} fc_rpsc_op_speed_t;

typedef struct {
	uint16_t        port_speed_cap;	/*! see fc_rpsc_speed_cap_t */
	uint16_t        port_op_speed;	/*! see fc_rpsc_op_speed_t */
} fc_rpsc_speed_info_t;

typedef enum {
	LINK_E2E_BEACON_ON = 1,
	LINK_E2E_BEACON_OFF = 2
} link_e2e_beacon_subcmd_t;

typedef enum {
	BEACON_TYPE_NORMAL	= 1,	/*! Normal Beaconing. Green */
	BEACON_TYPE_WARN	= 2,	/*! Warning Beaconing. Yellow/Amber */
	BEACON_TYPE_CRITICAL	= 3	/*! Critical Beaconing. Red */
} beacon_type_t;

typedef struct els_link_e2e_beacon_param {
	uint8_t         beacon_type;	/* Beacon Type. See beacon_type_t */
	uint8_t         beacon_frequency;
					/* Beacon frequency. Number of blinks
					 * per 10 seconds
					 */
	uint16_t        beacon_duration;/* Beacon duration (in Seconds). The
					 * command operation should be
					 * terminated at the end of this
					 * timeout value.
					 *
					 * Ignored if diag_sub_cmd is
					 * LINK_E2E_BEACON_OFF.
					 *
					 * If 0, beaconing will continue till a
					 * BEACON OFF request is received
					 */
} link_e2e_beacon_param_t;

/*
 * Link E2E beacon request/good response format. For LS_RJTs use fc_ls_rjt_t
 */
typedef struct {
	uint32_t        ls_code;	/*! FC_ELS_E2E_LBEACON in requests *
					 *or FC_ELS_ACC in good replies */
	uint32_t        ls_sub_cmd;	/*! See link_e2e_beacon_subcmd_t */
	link_e2e_beacon_param_t beacon_parm;
} link_e2e_beacon_req_t;

/**
 * If RPSC request is sent to the Domain Controller, the request is for
 * all the ports within that domain (TODO - I don't think FOS implements
 * this...).
 */
typedef struct {
	fc_els_cmd_t    els_cmd;
} fc_rpsc_cmd_t;

/*
 * RPSC Acc
 */
typedef struct {
	uint32_t        command:8;
	uint32_t        rsvd:8;
	uint32_t        num_entries:16;

	fc_rpsc_speed_info_t speed_info[1];
} fc_rpsc_acc_t;

/**
 * If RPSC2 request is sent to the Domain Controller,
 */
#define FC_BRCD_TOKEN    0x42524344 

typedef struct {
	fc_els_cmd_t    els_cmd;
	uint32_t       	token;     
  	uint16_t     	resvd;
    uint16_t     	num_pids;       /* Number of pids in the request */
    struct  {
		uint32_t	rsvd1:8;
		uint32_t	pid:24;				/* port identifier */
    } pid_list[1];    
} fc_rpsc2_cmd_t;

typedef enum {
	RPSC2_PORT_TYPE_UNKNOWN = 0,
	RPSC2_PORT_TYPE_NPORT   = 1,
	RPSC2_PORT_TYPE_NLPORT  = 2,
	RPSC2_PORT_TYPE_NPIV_PORT  = 0x5f,
	RPSC2_PORT_TYPE_NPORT_TRUNK  = 0x6f,
} fc_rpsc2_port_type_t;
/*
 * RPSC2 portInfo entry structure
 */
typedef struct {
    uint32_t    pid;        /* PID */
    uint16_t    resvd1;
    uint16_t    index;      /* port number / index */
    uint8_t     resvd2;
    uint8_t    	type;        /* port type N/NL/... */
    uint16_t    speed;      /* port Operating Speed */
} fc_rpsc2_port_info_t;

/*
 * RPSC2 Accept payload
 */
typedef struct {
	uint8_t        els_cmd;
	uint8_t        resvd;
    uint16_t       num_pids;  /* Number of pids in the request */
    fc_rpsc2_port_info_t  port_info[1];    /* port information */
}fc_rpsc2_acc_t;

/**
 * bit fields so that multiple classes can be specified
 */
typedef enum {
	FC_CLASS_2	= 0x04,
	FC_CLASS_3	= 0x08,
	FC_CLASS_2_3	= 0x0C,
} fc_cos_t;

/*
 * symbolic name
 */
typedef struct {
	uint8_t         symname[FC_SYMNAME_MAX];
} fc_symname_t;

typedef struct {
	uint8_t         alpa_bm[FC_ALPA_MAX / 8];
} fc_alpabm_t;

/*
 * protocol default timeout values
 */
#define FC_ED_TOV		2
#define FC_REC_TOV		(FC_ED_TOV + 1)
#define FC_RA_TOV		10
#define FC_ELS_TOV		(2 * FC_RA_TOV)
#define FC_FCCT_TOV		(3 * FC_RA_TOV)

/*
 * virtual fabric related defines
 */
#define FC_VF_ID_NULL    0	/*!< must not be used as VF_ID */
#define FC_VF_ID_MIN     1
#define FC_VF_ID_MAX     0xEFF
#define FC_VF_ID_CTL     0xFEF	/*!< control VF_ID */

/**
 * Virtual Fabric Tagging header format
 * @caution This is defined only in BIG ENDIAN format.
 */
typedef struct {
	uint32_t        r_ctl:8;
	uint32_t        ver:2;
	uint32_t        type:4;
	uint32_t        res_a:2;
	uint32_t        priority:3;
	uint32_t        vf_id:12;
	uint32_t        res_b:1;
	uint32_t        hopct:8;
	uint32_t        res_c:24;
} fc_vft_t;

/*
 * FC SP
 */

typedef enum {
	FC_AUTH_ELS_MORE_FRAGS_FLAG 	= 0x80,	/*! bit-7. More Fragments
						 * Follow
						 */
	FC_AUTH_ELS_CONCAT_FLAG 	= 0x40,	/*! bit-6. Concatenation Flag */
	FC_AUTH_ELS_SEQ_NUM_FLAG 	= 0x01 	/*! bit-0. Sequence Number */
} auth_els_flags_t;

typedef enum {
	FC_AUTH_MC_AUTH_RJT		= 0x0A,	/*! Auth Reject */
	FC_AUTH_MC_AUTH_NEG 		= 0x0B, /*! Auth Negotiate */
	FC_AUTH_MC_AUTH_DONE 		= 0x0C, /*! Auth Done */

	FC_AUTH_MC_DHCHAP_CHAL 		= 0x10, /*! DHCHAP Challenge */
	FC_AUTH_MC_DHCHAP_REPLY 	= 0x11, /*! DHCHAP Reply */
	FC_AUTH_MC_DHCHAP_SUCC 		= 0x12, /*! DHCHAP Success */

	FC_AUTH_MC_FCAP_REQ 		= 0x13, /*! FCAP Request */
	FC_AUTH_MC_FCAP_ACK 		= 0x14, /*! FCAP Acknowledge */
	FC_AUTH_MC_FCAP_CONF 		= 0x15, /*! FCAP Confirm */

	FC_AUTH_MC_FCPAP_INIT 		= 0x16, /*! FCPAP Init */
	FC_AUTH_MC_FCPAP_ACC 		= 0x17, /*! FCPAP Accept */
	FC_AUTH_MC_FCPAP_COMP 		= 0x18, /*! FCPAP Complete */

	FC_AUTH_MC_IKE_SA_INIT 		= 0x22, /*! IKE SA INIT */
	FC_AUTH_MC_IKE_SA_AUTH 		= 0x23, /*! IKE SA Auth */
	FC_AUTH_MC_IKE_CREATE_CHILD_SA	= 0x24, /*! IKE Create Child SA */
	FC_AUTH_MC_IKE_INFO 		= 0x25, /*! IKE informational */
} auth_msg_codes_t;

typedef enum {
	FC_AUTH_PROTO_VER_1 	= 1,	/*! Protocol Version 1 */
} auth_proto_version_t;

enum {
	FC_AUTH_ELS_COMMAND_CODE = 0x90,/*! Authentication ELS Command code  */
	FC_AUTH_PROTO_PARAM_LEN_SZ = 4,	/*! Size of Proto Parameter Len Field */
	FC_AUTH_PROTO_PARAM_VAL_SZ = 4,	/*! Size of Proto Parameter Val Field */
	FC_MAX_AUTH_SECRET_LEN     = 256,
					/*! Maximum secret string length */
	FC_AUTH_NUM_USABLE_PROTO_LEN_SZ = 4,
					/*! Size of usable protocols field */
	FC_AUTH_RESP_VALUE_LEN_SZ	= 4,
					/*! Size of response value length */
	FC_MAX_CHAP_KEY_LEN	= 256,	/*! Maximum md5 digest length */
	FC_MAX_AUTH_RETRIES     = 3,	/*! Maximum number of retries */
	FC_MD5_DIGEST_LEN       = 16,	/*! MD5 digest length */
	FC_SHA1_DIGEST_LEN      = 20,	/*! SHA1 digest length */
	FC_MAX_DHG_SUPPORTED    = 1,	/*! Maximum DH Groups supported */
	FC_MAX_ALG_SUPPORTED    = 1,	/*! Maximum algorithms supported */
	FC_MAX_PROTO_SUPPORTED  = 1,	/*! Maximum protocols supported */
	FC_START_TXN_ID         = 2,	/*! Starting transaction ID */
};

typedef enum {
	FC_AUTH_PROTO_DHCHAP		= 0x00000001,
	FC_AUTH_PROTO_FCAP 		= 0x00000002,
	FC_AUTH_PROTO_FCPAP 		= 0x00000003,
	FC_AUTH_PROTO_IKEv2 		= 0x00000004,
	FC_AUTH_PROTO_IKEv2_AUTH 	= 0x00000005,
} auth_proto_id_t;

typedef struct {
	uint16_t	name_tag;	/*! Name Tag = 1 for Authentication */
	uint16_t	name_len;	/*! Name Length = 8 for Authentication
					 */
	wwn_t		name;  		/*! Name. TODO - is this PWWN */
} auth_name_t;


typedef enum {
	FC_AUTH_HASH_FUNC_MD5 		= 0x00000005,
	FC_AUTH_HASH_FUNC_SHA_1 	= 0x00000006,
} auth_hash_func_t;

typedef enum {
	FC_AUTH_DH_GID_0_DHG_NULL	= 0x00000000,
	FC_AUTH_DH_GID_1_DHG_1024	= 0x00000001,
	FC_AUTH_DH_GID_2_DHG_1280	= 0x00000002,
	FC_AUTH_DH_GID_3_DHG_1536	= 0x00000003,
	FC_AUTH_DH_GID_4_DHG_2048	= 0x00000004,
	FC_AUTH_DH_GID_6_DHG_3072	= 0x00000006,
	FC_AUTH_DH_GID_7_DHG_4096	= 0x00000007,
	FC_AUTH_DH_GID_8_DHG_6144	= 0x00000008,
	FC_AUTH_DH_GID_9_DHG_8192	= 0x00000009,
} auth_dh_gid_t;

typedef struct auth_els_msg {
	uint8_t		auth_els_code;	/*!< Authentication ELS Code (0x90) */
	uint8_t 	auth_els_flag; 	/*!< Authentication ELS Flags */
	uint8_t 	auth_msg_code; 	/*!< Authentication Message Code */
	uint8_t 	proto_version; 	/*!< Protocol Version */
	uint32_t	msg_len; 	/*!< Message Length */
	uint32_t	trans_id; 	/*!< Transaction Identifier (T_ID) */

	/* Msg payload follows... */
} auth_els_msg_t;


typedef enum auth_neg_param_tags {
	FC_AUTH_NEG_DHCHAP_HASHLIST 	= 0x0001,
	FC_AUTH_NEG_DHCHAP_DHG_ID_LIST 	= 0x0002,
} auth_neg_param_tags_t;


typedef struct dhchap_param_format {
	uint16_t	tag;		/*! Parameter Tag. See
					 * auth_neg_param_tags_t
					 */
	uint16_t	word_cnt;

	/* followed by variable length parameter value... */
} dhchap_param_format_t;

typedef struct auth_proto_params {
	uint32_t	proto_param_len;
	uint32_t	proto_id;

	/*
	 * Followed by variable length Protocol specific parameters. DH-CHAP
	 * uses dhchap_param_format_t
	 */
} auth_proto_params_t;

typedef struct auth_neg_msg {
	auth_name_t		auth_ini_name;
	uint32_t		usable_auth_protos;
	auth_proto_params_t proto_params[1];	/*! (1..usable_auth_proto)
						 * protocol params
						 */
} auth_neg_msg_t;

typedef struct auth_dh_val {
	uint32_t dh_val_len;
	uint32_t dh_val[1];
} auth_dh_val_t;

typedef struct auth_dhchap_chal_msg {
	auth_els_msg_t	hdr;
	auth_name_t 	auth_responder_name;	/* TODO VRK - is auth_name_t
						 * type OK?
						 */
	uint32_t 	hash_id;
	uint32_t 	dh_grp_id;
	uint32_t 	chal_val_len;
	char		chal_val[1];

	/* ...followed by variable Challenge length/value and DH length/value */
} auth_dhchap_chal_msg_t;


typedef enum auth_rjt_codes_s {
	FC_AUTH_RJT_CODE_AUTH_FAILURE 	= 0x01,
	FC_AUTH_RJT_CODE_LOGICAL_ERR	= 0x02,
} auth_rjt_codes_t;

typedef enum auth_rjt_code_exps_s {
	FC_AUTH_CEXP_AUTH_MECH_NOT_USABLE	= 0x01,
	FC_AUTH_CEXP_DH_GROUP_NOT_USABLE 	= 0x02,
	FC_AUTH_CEXP_HASH_FUNC_NOT_USABLE 	= 0x03,
	FC_AUTH_CEXP_AUTH_XACT_STARTED		= 0x04,
	FC_AUTH_CEXP_AUTH_FAILED 			= 0x05,
	FC_AUTH_CEXP_INCORRECT_PLD 		= 0x06,
	FC_AUTH_CEXP_INCORRECT_PROTO_MSG 	= 0x07,
	FC_AUTH_CEXP_RESTART_AUTH_PROTO 	= 0x08,
	FC_AUTH_CEXP_AUTH_CONCAT_NOT_SUPP 	= 0x09,
	FC_AUTH_CEXP_PROTO_VER_NOT_SUPP 	= 0x0A,
} auth_rjt_code_exps_t;

typedef enum auth_status_s {
	FC_AUTH_STATE_INPROGRESS = 0, 	/*! authentication in progress 	*/
	FC_AUTH_STATE_FAILED	= 1, 	/*! authentication failed */
	FC_AUTH_STATE_SUCCESS	= 2 	/*! authentication successful	*/
} auth_status_t;

typedef struct auth_rjt_msg {
	auth_els_msg_t	hdr;
	uint8_t		reason_code;
	uint8_t		reason_code_exp;
	uint8_t		rsvd[2];
} auth_rjt_msg_t;

typedef auth_els_msg_t auth_done_msg_t; /*! Auth done msg has no payload */

typedef struct auth_dhchap_neg_msg_s {
	auth_els_msg_t hdr;
	auth_neg_msg_t nego;
} auth_dhchap_neg_msg_t;

typedef struct auth_dhchap_reply_msg_s {
	auth_els_msg_t	hdr;

	/*
	 * followed by response value length & Value + DH Value Length & Value
	 */
} auth_dhchap_reply_msg_t;

/*
 * SRR FC-4 LS payload
 */
typedef struct {
	fc_els_cmd_t	els_cmd;	/* ELS command code */
	uint32_t        ox_id:16;	/* ox-id */
	uint32_t        rx_id:16;	/* rx-id */
	uint32_t        ro;		/* relative offset */
	uint32_t        r_ctl:8;		/* R_CTL for I.U. */
	uint32_t        res:24;
} fc_srr_t;

/*
 * FCP_CMND definitions
 */
#define FCP_CMND_CDB_LEN    16
#define FCP_CMND_LUN_LEN    8

typedef struct {
	lun_t           lun;		/* 64-bit LU number */
	uint8_t         crn;		/* command reference number */
#ifdef __BIGENDIAN
	uint8_t         resvd:1,
			priority:4,	/* FCP-3: SAM-3 priority */
			taskattr:3;	/* scsi task attribute */
#else
	uint8_t         taskattr:3,	/* scsi task attribute */
			priority:4,	/* FCP-3: SAM-3 priority */
			resvd:1;
#endif
	uint8_t         tm_flags;	/* task management flags */
#ifdef __BIGENDIAN
	uint8_t         addl_cdb_len:6,	/* additional CDB length words */
			iodir:2;	/* read/write FCP_DATA IUs */
#else
	uint8_t         iodir:2,	/* read/write FCP_DATA IUs */
			addl_cdb_len:6;	/* additional CDB length */
#endif
	scsi_cdb_t      cdb;

	/*
	 * !!! additional cdb bytes follows here!!!
	 */
	uint32_t        fcp_dl;	/* bytes to be transferred */
} fcp_cmnd_t;

#define fcp_cmnd_cdb_len(_cmnd) ((_cmnd)->addl_cdb_len * 4 + FCP_CMND_CDB_LEN)
#define fcp_cmnd_fcpdl(_cmnd)	((&(_cmnd)->fcp_dl)[(_cmnd)->addl_cdb_len])

/*
 * fcp_cmnd_t.iodir field values
 */
typedef enum {
	FCP_IODIR_NONE	= 0,
	FCP_IODIR_WRITE = 1,
	FCP_IODIR_READ	= 2,
	FCP_IODIR_RW	= 3,
} fcp_iodir_t;

/*
 * Task attribute field
 */
enum {
	FCP_TASK_ATTR_SIMPLE	= 0,
	FCP_TASK_ATTR_HOQ	= 1,
	FCP_TASK_ATTR_ORDERED	= 2,
	FCP_TASK_ATTR_ACA	= 4,
	FCP_TASK_ATTR_UNTAGGED	= 5,	/* obsolete in FCP-3 */
};

/*
 * Task management flags field - only one bit shall be set
 */
#ifndef BIT
#define BIT(_x)	(1 << (_x))
#endif
typedef enum {
	FCP_TM_ABORT_TASK_SET	= BIT(1),
	FCP_TM_CLEAR_TASK_SET	= BIT(2),
	FCP_TM_LUN_RESET	= BIT(4),
	FCP_TM_TARGET_RESET	= BIT(5),	/* obsolete in FCP-3 */
	FCP_TM_CLEAR_ACA	= BIT(6),
} fcp_tm_cmnd_t;

/*
 * FCP_XFER_RDY IU defines
 */
typedef struct {
	uint32_t        data_ro;
	uint32_t        burst_len;
	uint32_t        reserved;
} fcp_xfer_rdy_t;

/*
 * FCP_RSP residue flags
 */
typedef enum {
	FCP_NO_RESIDUE = 0,	/* no residue */
	FCP_RESID_OVER = 1,	/* more data left that was not sent */
	FCP_RESID_UNDER = 2,	/* less data than requested */
} fcp_residue_t;

enum {
	FCP_RSPINFO_GOOD = 0,
	FCP_RSPINFO_DATALEN_MISMATCH = 1,
	FCP_RSPINFO_CMND_INVALID = 2,
	FCP_RSPINFO_ROLEN_MISMATCH = 3,
	FCP_RSPINFO_TM_NOT_SUPP = 4,
	FCP_RSPINFO_TM_FAILED = 5,
};

#define	FCP_RSP_INFO_MIN	4

typedef struct {
	uint32_t        res0:24;
	uint32_t        rsp_code:8;	/* response code (as above) */
	uint32_t        res1;
} fcp_rspinfo_t;

typedef struct {
	uint32_t        reserved[2];	/* 2 words reserved */
	uint16_t        reserved2;
#ifdef __BIGENDIAN
	uint8_t         reserved3:3;
	uint8_t         fcp_conf_req:1;	/* FCP_CONF is requested */
	uint8_t         resid_flags:2;	/* underflow/overflow */
	uint8_t         sns_len_valid:1;/* sense len is valid */
	uint8_t         rsp_len_valid:1;/* response len is valid */
#else
	uint8_t         rsp_len_valid:1;/* response len is valid */
	uint8_t         sns_len_valid:1;/* sense len is valid */
	uint8_t         resid_flags:2;	/* underflow/overflow */
	uint8_t         fcp_conf_req:1;	/* FCP_CONF is requested */
	uint8_t         reserved3:3;
#endif
	uint8_t         scsi_status;	/* one byte SCSI status */
	uint32_t        residue;	/* residual data bytes */
	uint32_t        sns_len;	/* length od sense info */
	uint32_t        rsp_len;	/* length of response info */
} fcp_resp_t;

#define fcp_snslen(__fcprsp)	((__fcprsp)->sns_len_valid ? 		\
					(__fcprsp)->sns_len : 0)
#define fcp_rsplen(__fcprsp)	((__fcprsp)->rsp_len_valid ? 		\
					(__fcprsp)->rsp_len : 0)
#define fcp_rspinfo(__fcprsp)	((fcp_rspinfo_t *)((__fcprsp) + 1))
#define fcp_snsinfo(__fcprsp)	(((uint8_t *)fcp_rspinfo(__fcprsp)) + 	\
						fcp_rsplen(__fcprsp))

typedef struct {
	fchs_t          fchs;
	fcp_cmnd_t      fcp;
} fcp_cmnd_fr_t;

/*
 * CT
 */
typedef struct {
	uint32_t	rev_id:8;	/* Revision of the CT */
	uint32_t	in_id:24;	/* Initiator Id */
	uint32_t	gs_type:8;	/* Generic service Type */
	uint32_t	gs_sub_type:8;	/* Generic service sub type */
	uint32_t	options:8;	/* options */
	uint32_t	rsvrd:8;	/* reserved */
	uint32_t	cmd_rsp_code:16;/* ct command/response code */
	uint32_t	max_res_size:16;/* maximum/residual size */
	uint32_t	frag_id:8;	/* fragment ID */
	uint32_t	reason_code:8;	/* reason code */
	uint32_t	exp_code:8;	/* explanation code */
	uint32_t	vendor_unq:8;	/* vendor unique */
} ct_hdr_t;

/*
 * defines for the Revision
 */
enum {
	CT_GS3_REVISION = 0x01,
};

/*
 * defines for gs_type
 */
enum {
	CT_GSTYPE_KEYSERVICE	= 0xF7,
	CT_GSTYPE_ALIASSERVICE	= 0xF8,
	CT_GSTYPE_MGMTSERVICE	= 0xFA,
	CT_GSTYPE_TIMESERVICE	= 0xFB,
	CT_GSTYPE_DIRSERVICE	= 0xFC,
};

/*
 * defines for gs_sub_type for gs type directory service
 */
enum {
	CT_GSSUBTYPE_NAMESERVER = 0x02,
};

/*
 * defines for gs_sub_type for gs type management service
 */
enum {
	CT_GSSUBTYPE_CFGSERVER	= 0x01,
	CT_GSSUBTYPE_UNZONED_NS = 0x02,
	CT_GSSUBTYPE_ZONESERVER = 0x03,
	CT_GSSUBTYPE_LOCKSERVER = 0x04,
	CT_GSSUBTYPE_HBA_MGMTSERVER = 0x10,	/* for FDMI */
};

/*
 * defines for CT response code field
 */
enum {
	CT_RSP_REJECT = 0x8001,
	CT_RSP_ACCEPT = 0x8002,
};

/*
 * defintions for CT reason code
 */
enum {
	CT_RSN_INV_CMD		= 0x01,
	CT_RSN_INV_VER		= 0x02,
	CT_RSN_LOGIC_ERR	= 0x03,
	CT_RSN_INV_SIZE		= 0x04,
	CT_RSN_LOGICAL_BUSY	= 0x05,
	CT_RSN_PROTO_ERR	= 0x07,
	CT_RSN_UNABLE_TO_PERF	= 0x09,
	CT_RSN_NOT_SUPP			= 0x0B,
	CT_RSN_SERVER_NOT_AVBL  = 0x0D,
	CT_RSN_SESSION_COULD_NOT_BE_ESTBD = 0x0E,
	CT_RSN_VENDOR_SPECIFIC  = 0xFF,

};

/*
 * definitions for explanations code for Name server
 */
enum {
	CT_NS_EXP_NOADDITIONAL	= 0x00,
	CT_NS_EXP_ID_NOT_REG	= 0x01,
	CT_NS_EXP_PN_NOT_REG	= 0x02,
	CT_NS_EXP_NN_NOT_REG	= 0x03,
	CT_NS_EXP_CS_NOT_REG	= 0x04,
	CT_NS_EXP_IPN_NOT_REG	= 0x05,
	CT_NS_EXP_IPA_NOT_REG	= 0x06,
	CT_NS_EXP_FT_NOT_REG	= 0x07,
	CT_NS_EXP_SPN_NOT_REG	= 0x08,
	CT_NS_EXP_SNN_NOT_REG	= 0x09,
	CT_NS_EXP_PT_NOT_REG	= 0x0A,
	CT_NS_EXP_IPP_NOT_REG	= 0x0B,
	CT_NS_EXP_FPN_NOT_REG	= 0x0C,
	CT_NS_EXP_HA_NOT_REG	= 0x0D,
	CT_NS_EXP_FD_NOT_REG	= 0x0E,
	CT_NS_EXP_FF_NOT_REG	= 0x0F,
	CT_NS_EXP_ACCESSDENIED	= 0x10,
	CT_NS_EXP_UNACCEPTABLE_ID = 0x11,
	CT_NS_EXP_DATABASEEMPTY			= 0x12,
	CT_NS_EXP_NOT_REG_IN_SCOPE 		= 0x13,
	CT_NS_EXP_DOM_ID_NOT_PRESENT 	= 0x14,
	CT_NS_EXP_PORT_NUM_NOT_PRESENT  = 0x15,
	CT_NS_EXP_NO_DEVICE_ATTACHED 	= 0x16
};

/*
 * defintions for the explanation code for all servers
 */
enum {
	CT_EXP_AUTH_EXCEPTION			= 0xF1,
	CT_EXP_DB_FULL					= 0xF2,
	CT_EXP_DB_EMPTY					= 0xF3,
	CT_EXP_PROCESSING_REQ			= 0xF4,
	CT_EXP_UNABLE_TO_VERIFY_CONN	= 0xF5,
	CT_EXP_DEVICES_NOT_IN_CMN_ZONE  = 0xF6
};

/*
 * Command codes for Name server
 */
enum {
	GS_GID_PN	= 0x0121,	/* Get Id on port name */
	GS_GPN_ID	= 0x0112,	/* Get port name on ID */
	GS_GNN_ID	= 0x0113,	/* Get node name on ID */
	GS_GID_FT	= 0x0171,	/* Get Id on FC4 type */
	GS_GSPN_ID	= 0x0118,	/* Get symbolic PN on ID */
	GS_RFT_ID	= 0x0217,	/* Register fc4type on ID */
	GS_RSPN_ID	= 0x0218,	/* Register symbolic PN on ID */
	GS_RSNN_NN	= 0x0239,	/* Register symbolic NN on NN */
	GS_RPN_ID	= 0x0212,	/* Register port name */
	GS_RNN_ID	= 0x0213,	/* Register node name */
	GS_RCS_ID	= 0x0214,	/* Register class of service */
	GS_RPT_ID	= 0x021A,	/* Register port type */
	GS_GA_NXT	= 0x0100,	/* Get all next */
	GS_RFF_ID	= 0x021F,	/* Register FC4 Feature		*/
};

typedef struct {
	uint32_t	rsvd:8;
	uint32_t	dap:24;	/* port identifier */
} fcgs_gpnid_req_t, fcgs_gnnid_req_t, fcgs_gspnid_req_t;

typedef struct {
	wwn_t	port_name;	/* port wwn */
} fcgs_gidpn_req_t;

typedef struct {
	uint32_t	rsvd:8;
	uint32_t	dap:24;	/* port identifier */
} fcgs_gidpn_resp_t;

/**
 * RFT_ID
 */
typedef struct {
	uint32_t	rsvd:8;
	uint32_t	dap:24;		/* port identifier */
	uint32_t	fc4_type[8];	/* fc4 types */
} fcgs_rftid_req_t;

/**
 * RFF_ID : Register FC4 features.
 */

#define FC_GS_FCP_FC4_FEATURE_INITIATOR  0x02
#define FC_GS_FCP_FC4_FEATURE_TARGET	 0x01

typedef struct {
    uint32_t    rsvd          :8;
    uint32_t    dap        	  :24;		/* port identifier	*/
    uint32_t    rsvd1         :16;
    uint32_t    fc4ftr_bits   :8;		/* fc4 feature bits	*/
    uint32_t    fc4_type      :8;		/* corresponding FC4 Type */
} fcgs_rffid_req_t;

/**
 * GID_FT Request
 */
typedef struct {
	uint8_t	reserved;
	uint8_t	domain_id;	/* domain, 0 - all fabric */
	uint8_t	area_id;	/* area, 0 - whole domain */
	uint8_t	fc4_type;	/* FC_TYPE_FCP for SCSI devices */
} fcgs_gidft_req_t;		/* GID_FT Request */

/**
 * GID_FT Response
 */
typedef struct {
	uint8_t		last:1;	/* last port identifier flag */
	uint8_t		reserved:7;
	uint32_t	pid:24;	/* port identifier */
} fcgs_gidft_resp_t;		/* GID_FT Response */

/**
 * RSPN_ID
 */
typedef struct {
	uint32_t	rsvd:8;
	uint32_t	dap:24;		/* port identifier */
	uint8_t		spn_len;	/* symbolic port name length */
	uint8_t		spn[256];	/* symbolic port name */
} fcgs_rspnid_req_t;

/**
 * RSNN_NN
 */
typedef struct {
	wwn_t		node_name;	/* Node name */
	uint8_t		snn_len;	/* symbolic node name length */
	uint8_t		snn[256];	/* symbolic node name */
} fcgs_rsnn_nn_req_t;

/**
 * RPN_ID
 */
typedef struct {
	uint32_t	rsvd:8;
	uint32_t	port_id:24;
	wwn_t		port_name;
} fcgs_rpnid_req_t;

/**
 * RNN_ID
 */
typedef struct {
	uint32_t	rsvd:8;
	uint32_t	port_id:24;
	wwn_t		node_name;
} fcgs_rnnid_req_t;

/**
 * RCS_ID
 */
typedef struct {
	uint32_t	rsvd:8;
	uint32_t	port_id:24;
	uint32_t	cos;
} fcgs_rcsid_req_t;

/**
 * RPT_ID
 */
typedef struct {
	uint32_t	rsvd:8;
	uint32_t	port_id:24;
	uint32_t	port_type:8;
	uint32_t	rsvd1:24;
} fcgs_rptid_req_t;

/**
 * GA_NXT Request
 */
typedef struct {
	uint32_t	rsvd:8;
	uint32_t	port_id:24;
} fcgs_ganxt_req_t;

/**
 * GA_NXT Response
 */
typedef struct {
	uint32_t	port_type:8;	/* Port Type */
	uint32_t	port_id:24;	/* Port Identifier */
	wwn_t		port_name;	/* Port Name */
	uint8_t		spn_len;	/* Length of Symbolic Port Name */
	char		spn[255];	/* Symbolic Port Name */
	wwn_t		node_name;	/* Node Name */
	uint8_t		snn_len;	/* Length of Symbolic Node Name */
	char		snn[255];	/* Symbolic Node Name */
	uint8_t		ipa[8];		/* Initial Process Associator */
	uint8_t		ip[16];		/* IP Address */
	uint32_t	cos;		/* Class of Service */
	uint32_t	fc4types[8];	/* FC-4 TYPEs */
	wwn_t		fabric_port_name;
					/* Fabric Port Name */
	uint32_t	rsvd:8;		/* Reserved */
	uint32_t	hard_addr:24;	/* Hard Address */
} fcgs_ganxt_rsp_t;

/*
 * Fabric Config Server
 */

/*
 * Command codes for Fabric Configuration Server
 */
enum {
	GS_FC_GFN_CMD	= 0x0114,	/* GS FC Get Fabric Name  */
	GS_FC_GMAL_CMD	= 0x0116,	/* GS FC GMAL  */
	GS_FC_TRACE_CMD	= 0x0400,	/* GS FC Trace Route */
	GS_FC_PING_CMD	= 0x0401,	/* GS FC Ping */
};

/*
 * Source or Destination Port Tags.
 */
enum {
	GS_FTRACE_TAG_NPORT_ID		= 1,
	GS_FTRACE_TAG_NPORT_NAME	= 2,
};

/*
* Port Value : Could be a Port id or wwn
 */
typedef union {
	uint32_t	nport_id;
	wwn_t		nport_wwn;
} fcgs_port_val_t;

#define GS_FTRACE_MAX_HOP_COUNT	20
#define GS_FTRACE_REVISION	1

/*
 * Ftrace Related Structures.
 */

/*
 * STR (Switch Trace) Reject Reason Codes. From FC-SW.
 */
enum {
	GS_FTRACE_STR_CMD_COMPLETED_SUCC	= 0,
	GS_FTRACE_STR_CMD_NOT_SUPP_IN_NEXT_SWITCH,
	GS_FTRACE_STR_NO_RESP_FROM_NEXT_SWITCH,
	GS_FTRACE_STR_MAX_HOP_CNT_REACHED,
	GS_FTRACE_STR_SRC_PORT_NOT_FOUND,
	GS_FTRACE_STR_DST_PORT_NOT_FOUND,
	GS_FTRACE_STR_DEVICES_NOT_IN_COMMON_ZONE,
	GS_FTRACE_STR_NO_ROUTE_BW_PORTS,
	GS_FTRACE_STR_NO_ADDL_EXPLN,
	GS_FTRACE_STR_FABRIC_BUSY,
	GS_FTRACE_STR_FABRIC_BUILD_IN_PROGRESS,
	GS_FTRACE_STR_VENDOR_SPECIFIC_ERR_START = 0xf0,
	GS_FTRACE_STR_VENDOR_SPECIFIC_ERR_END = 0xff,
};

/*
 * Ftrace Request
 */
typedef struct {
	uint32_t	revision;
	uint16_t	src_port_tag;	/* Source Port tag */
	uint16_t	src_port_len;	/* Source Port len */
	fcgs_port_val_t src_port_val;	/* Source Port value */
	uint16_t	dst_port_tag;	/* Destination Port tag */
	uint16_t	dst_port_len;	/* Destination Port len */
	fcgs_port_val_t dst_port_val;	/* Destination Port value */
	uint32_t	token;
	uint8_t		vendor_id[8];	/* T10 Vendor Identifier */
	uint8_t		vendor_info[8];	/* Vendor specific Info */
	uint32_t	max_hop_cnt;	/* Max Hop Count */
} fcgs_ftrace_req_t;

/*
 * Path info structure
 */
typedef struct {
	wwn_t		switch_name;		/* Switch WWN */
	uint32_t	domain_id;
	wwn_t		ingress_port_name;	/* Ingress ports wwn */
	uint32_t	ingress_phys_port_num;	/* Ingress ports physical port
						 * number
						 */
	wwn_t		egress_port_name;	/* Ingress ports wwn */
	uint32_t	egress_phys_port_num;	/* Ingress ports physical port
						 * number
						 */
} fcgs_ftrace_path_info;

/*
 * Ftrace Acc Response
 */
typedef struct {
	uint32_t	revision;
	uint32_t	token;
	uint8_t		vendor_id[8];		/* T10 Vendor Identifier */
	uint8_t		vendor_info[8];		/* Vendor specific Info */
	uint32_t	str_rej_reason_code;	/* STR Reject Reason Code */
	uint32_t	num_path_info_entries;	/* No. of path info entries */
	/*
	 * path info entry/entries.
	 */
	fcgs_ftrace_path_info path_info[1];

} fcgs_ftrace_resp_t;

/*
* Fabric Config Server : FCPing
 */

/*
 * FC Ping Request
 */
typedef struct {
	uint32_t	revision;
	uint16_t	port_tag;
	uint16_t	port_len;	/* Port len */
	fcgs_port_val_t port_val;	/* Port value */
	uint32_t	token;
} fcgs_fcping_req_t;

/*
 * FC Ping Response
 */
typedef struct {
	uint32_t	token;
} fcgs_fcping_resp_t;

/*
 * Command codes for zone server query.
 */
enum {
	ZS_GZME = 0x0124,	/* Get zone member extended */
};

/*
 * ZS GZME request
 */
#define ZS_GZME_ZNAMELEN	32
typedef struct {
	uint8_t	znamelen;
	uint8_t	rsvd[3];
	uint8_t	zname[ZS_GZME_ZNAMELEN];
} zs_gzme_req_t;

typedef enum {
	ZS_MBR_TYPE_PWWN	= 1,
	ZS_MBR_TYPE_DOMPORT	= 2,
	ZS_MBR_TYPE_PORTID	= 3,
	ZS_MBR_TYPE_NWWN	= 4,
} zs_mbr_type_t;

typedef struct {
	uint8_t	mbr_type;
	uint8_t	rsvd[3];
	wwn_t	wwn;
} zs_mbr_wwn_t;

typedef struct {
	uint32_t	nmbrs;	/* !< number of zone members */
	zs_mbr_wwn_t	mbr[1];
} zs_query_resp_t;

/*
 * GMAL Command ( Get ( interconnect Element) Management Address List )
 * To retrieve the IP Address of a Switch.
 */

#define CT_GMAL_RESP_PREFIX_TELNET	 "telnet://"
#define CT_GMAL_RESP_PREFIX_HTTP	 "http://"

/*  GMAL/GFN request */
typedef struct  {
	wwn_t    wwn; 	/* PWWN/NWWN */
} fcgs_gmal_req_t, fcgs_gfn_req_t;

/* Accept Response to GMAL */
typedef struct  {
	uint32_t 		ms_len;   /* Num of entries */
	uint8_t     	ms_ma[256];
} fcgs_gmal_resp_t;

typedef struct fc_gmal_entry {
	uint8_t  len;
	uint8_t  prefix[7]; /* like "http://" */
	uint8_t  ip_addr[248]; 
} fcgs_gmal_entry_t;

/*
 * FDMI
 */
/*
 * FDMI Command Codes
 */
#define	FDMI_GRHL		0x0100
#define	FDMI_GHAT		0x0101
#define	FDMI_GRPL		0x0102
#define	FDMI_GPAT		0x0110
#define	FDMI_RHBA		0x0200
#define	FDMI_RHAT		0x0201
#define	FDMI_RPRT		0x0210
#define	FDMI_RPA		0x0211
#define	FDMI_DHBA		0x0300
#define	FDMI_DPRT		0x0310

/*
 * FDMI reason codes
 */
#define	FDMI_NO_ADDITIONAL_EXP		0x00
#define	FDMI_HBA_ALREADY_REG		0x10
#define	FDMI_HBA_ATTRIB_NOT_REG		0x11
#define	FDMI_HBA_ATTRIB_MULTIPLE	0x12
#define	FDMI_HBA_ATTRIB_LENGTH_INVALID	0x13
#define	FDMI_HBA_ATTRIB_NOT_PRESENT	0x14
#define	FDMI_PORT_ORIG_NOT_IN_LIST	0x15
#define	FDMI_PORT_HBA_NOT_IN_LIST	0x16
#define	FDMI_PORT_ATTRIB_NOT_REG	0x20
#define	FDMI_PORT_NOT_REG		0x21
#define	FDMI_PORT_ATTRIB_MULTIPLE	0x22
#define	FDMI_PORT_ATTRIB_LENGTH_INVALID	0x23
#define	FDMI_PORT_ALREADY_REGISTEREED	0x24

/*
 * FDMI Transmission Speed Mask values
 */
#define	FDMI_TRANS_SPEED_1G		0x00000001
#define	FDMI_TRANS_SPEED_2G		0x00000002
#define	FDMI_TRANS_SPEED_10G		0x00000004
#define	FDMI_TRANS_SPEED_4G		0x00000008
#define	FDMI_TRANS_SPEED_8G		0x00000010
#define	FDMI_TRANS_SPEED_16G		0x00000020
#define	FDMI_TRANS_SPEED_UNKNOWN	0x00008000

/*
 * FDMI HBA attribute types
 */
enum fdmi_hba_attribute_type {
	FDMI_HBA_ATTRIB_NODENAME = 1,	/* 0x0001 */
	FDMI_HBA_ATTRIB_MANUFACTURER,	/* 0x0002 */
	FDMI_HBA_ATTRIB_SERIALNUM,	/* 0x0003 */
	FDMI_HBA_ATTRIB_MODEL,		/* 0x0004 */
	FDMI_HBA_ATTRIB_MODEL_DESC,	/* 0x0005 */
	FDMI_HBA_ATTRIB_HW_VERSION,	/* 0x0006 */
	FDMI_HBA_ATTRIB_DRIVER_VERSION,	/* 0x0007 */
	FDMI_HBA_ATTRIB_ROM_VERSION,	/* 0x0008 */
	FDMI_HBA_ATTRIB_FW_VERSION,	/* 0x0009 */
	FDMI_HBA_ATTRIB_OS_NAME,	/* 0x000A */
	FDMI_HBA_ATTRIB_MAX_CT,		/* 0x000B */
	FDMI_HBA_ATTRIB_NODE_SYM_NAME,	/* 0x000C */
	FDMI_HBA_ATTRIB_VENDOR_INFO,	/* 0x000D */
	FDMI_HBA_ATTRIB_NUM_PORTS,	/* 0x000E */
	FDMI_HBA_ATTRIB_FABRIC_NAME,	/* 0x000F */
	FDMI_HBA_ATTRIB_BIOS_VER,	/* 0x0010 */
	FDMI_HBA_ATTRIB_VENDOR_ID = 0x00E0,

	FDMI_HBA_ATTRIB_MAX_TYPE
};

/*
 * FDMI Port attribute types
 */
enum fdmi_port_attribute_type {
	FDMI_PORT_ATTRIB_FC4_TYPES = 1,	/* 0x0001 */
	FDMI_PORT_ATTRIB_SUPP_SPEED,	/* 0x0002 */
	FDMI_PORT_ATTRIB_PORT_SPEED,	/* 0x0003 */
	FDMI_PORT_ATTRIB_FRAME_SIZE,	/* 0x0004 */
	FDMI_PORT_ATTRIB_DEV_NAME,	/* 0x0005 */
	FDMI_PORT_ATTRIB_HOST_NAME,	/* 0x0006 */
	FDMI_PORT_ATTRIB_NODE_NAME,     /* 0x0007 */
	FDMI_PORT_ATTRIB_PORT_NAME,     /* 0x0008 */
	FDMI_PORT_ATTRIB_PORT_SYM_NAME, /* 0x0009 */
	FDMI_PORT_ATTRIB_PORT_TYPE,     /* 0x000A */
	FDMI_PORT_ATTRIB_SUPP_COS,      /* 0x000B */
	FDMI_PORT_ATTRIB_PORT_FAB_NAME, /* 0x000C */
	FDMI_PORT_ATTRIB_PORT_FC4_TYPE, /* 0x000D */
	FDMI_PORT_ATTRIB_PORT_STATE = 0x101,    /* 0x0101 */
	FDMI_PORT_ATTRIB_PORT_NUM_RPRT = 0x102, /* 0x0102 */

	FDMI_PORT_ATTR_MAX_TYPE
};

/*
 * FDMI attribute
 */
typedef struct fdmi_attr_s {
	uint16_t        type;
	uint16_t        len;
	uint8_t         value[1];
} fdmi_attr_t;

/*
 * HBA Attribute Block
 */
typedef struct fdmi_hba_attr_s {
	uint32_t        attr_count;	/* # of attributes */
	fdmi_attr_t     hba_attr;	/* n attributes */
} fdmi_hba_attr_t;

/*
 * Registered Port List
 */
typedef struct fdmi_port_list_s {
	uint32_t        num_ports;	/* number Of Port Entries */
	wwn_t           port_entry;	/* one or more */
} fdmi_port_list_t;

/*
 * Port Attribute Block
 */
typedef struct fdmi_port_attr_s {
	uint32_t        attr_count;	/* # of attributes */
	fdmi_attr_t     port_attr;	/* n attributes */
} fdmi_port_attr_t;

/*
 * FDMI Register HBA Attributes
 */
typedef struct fdmi_rhba_s {
	wwn_t           hba_id;		/* HBA Identifier */
	fdmi_port_list_t port_list;	/* Registered Port List */
	fdmi_hba_attr_t hba_attr_blk;	/* HBA attribute block */
} fdmi_rhba_t;

/*
 * FDMI Register Port
 */
typedef struct fdmi_rprt_s {
	wwn_t           hba_id;		/* HBA Identifier */
	wwn_t           port_name;	/* Port wwn */
	fdmi_port_attr_t port_attr_blk;	/* Port Attr Block */
} fdmi_rprt_t;

/*
 * FDMI Register Port Attributes
 */
typedef struct fdmi_rpa_s {
	wwn_t           port_name;	/* port wwn */
	fdmi_port_attr_t port_attr_blk;	/* Port Attr Block */
} fdmi_rpa_t;

#pragma pack()

#endif	/* __BFA_FC_H__ */
