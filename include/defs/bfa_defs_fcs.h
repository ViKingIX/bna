/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BFA_DEFS_FCS_H__
#define __BFA_DEFS_FCS_H__

#include <protocol/bfa_fc.h>
#include <defs/bfa_defs_svc.h>

/**
 * VF states
 */
enum bfa_vf_state {
	BFA_VF_UNINIT    = 0,	/*!< fabric is not yet initialized */
	BFA_VF_LINK_DOWN = 1,	/*!< link is down */
	BFA_VF_FLOGI     = 2,	/*!< flogi is in progress */
	BFA_VF_AUTH      = 3,	/*!< authentication in progress */
	BFA_VF_NOFABRIC  = 4,	/*!< fabric is not present */
	BFA_VF_ONLINE    = 5,	/*!< login to fabric is complete */
	BFA_VF_EVFP      = 6,	/*!< EVFP is in progress */
	BFA_VF_ISOLATED  = 7,	/*!< port isolated due to vf_id mismatch */
};
typedef enum bfa_vf_state bfa_vf_state_t;

/**
 * VF statistics
 */
struct bfa_vf_stats_s {
	uint32_t	flogi_sent;	/*!< Num FLOGIs sent */
	uint32_t	flogi_rsp_err;	/*!< FLOGI response errors */
	uint32_t	flogi_acc_err;	/*!< FLOGI accept errors */
	uint32_t	flogi_accepts;	/*!< FLOGI accepts received */
	uint32_t	flogi_rejects;	/*!< FLOGI rejects received */
	uint32_t	flogi_unknown_rsp; /*!< Unknown responses for FLOGI */
	uint32_t	flogi_alloc_wait; /*!< Alloc waits before FLOGI sent */
	uint32_t	flogi_rcvd;	/*!< FLOGIs received */
	uint32_t	flogi_rejected;	/*!< Incoming FLOGIs rejected */
	uint32_t	fabric_onlines;	/*!< Fabric online notifications */
	uint32_t	fabric_offlines; /*!< Fabric offline notifications */
	uint32_t	resvd; /*!< padding for 64 bit alignment */
};
typedef struct bfa_vf_stats_s bfa_vf_stats_t;

/**
 * VF attributes returned in queries
 */
struct bfa_vf_attr_s {
	enum bfa_vf_state	state;		/*!< VF state */
	uint32_t		rsvd;
	wwn_t			fabric_name;	/*!< fabric name */
};
typedef struct bfa_vf_attr_s bfa_vf_attr_t;

#define PUBLIC_KEY			15409
#define PRIVATE_KEY			19009
#define KEY_LEN				32399
#define BFA_AUTH_SECRET_STRING_LEN	256
#define BFA_AUTH_FAIL_NO_PASSWORD	0xFE
#define BFA_AUTH_FAIL_TIMEOUT		0xFF

/**
 * Authentication status
 */
enum bfa_auth_status {
	BFA_AUTH_STATUS_NONE 	= 0,	/*!< no authentication */
	BFA_AUTH_UNINIT 	= 1,	/*!< state - uninit */
	BFA_AUTH_NEG_SEND 	= 2,	/*!< state - negotiate send */
	BFA_AUTH_CHAL_WAIT 	= 3,	/*!< state - challenge wait */
	BFA_AUTH_NEG_RETRY 	= 4,	/*!< state - negotiate retry */
	BFA_AUTH_REPLY_SEND 	= 5,	/*!< state - reply send */
	BFA_AUTH_STATUS_WAIT 	= 6,	/*!< state - status wait */
	BFA_AUTH_SUCCESS 	= 7,	/*!< state - success */
	BFA_AUTH_FAILED 	= 8,	/*!< state - failed */
	BFA_AUTH_STATUS_UNKNOWN = 9,	/*!< authentication status unknown */
};
typedef enum bfa_auth_status bfa_auth_status_t;

enum bfa_auth_rej_code {
	BFA_AUTH_RJT_CODE_AUTH_FAILURE   = 1, /*!<auth failure */
	BFA_AUTH_RJT_CODE_LOGICAL_ERR    = 2, /*!<logical error */
};
typedef enum bfa_auth_rej_code bfa_auth_rej_code_t;

/**
 * Authentication reject codes
 */
enum bfa_auth_rej_code_exp {
	BFA_AUTH_MECH_NOT_USABLE	= 1, /*!<auth. mechanism not usable */
	BFA_AUTH_DH_GROUP_NOT_USABLE	= 2, /*!<DH Group not usable */
	BFA_AUTH_HASH_FUNC_NOT_USABLE	= 3, /*!<hash Function not usable */
	BFA_AUTH_AUTH_XACT_STARTED	= 4, /*!<auth xact started */
	BFA_AUTH_AUTH_FAILED		= 5, /*!<auth failed */
	BFA_AUTH_INCORRECT_PLD		= 6, /*!<incorrect payload */
	BFA_AUTH_INCORRECT_PROTO_MSG	= 7, /*!<incorrect proto msg */
	BFA_AUTH_RESTART_AUTH_PROTO	= 8, /*!<restart auth protocol */
	BFA_AUTH_AUTH_CONCAT_NOT_SUPP	= 9, /*!<auth concat not supported */
	BFA_AUTH_PROTO_VER_NOT_SUPP	= 10, /*!<proto version not supported */
};
typedef enum bfa_auth_rej_code_exp bfa_auth_rej_code_exp_t;

struct auth_proto_stats_s {
	uint32_t	auth_rjts;		/* auth_rjts */
	uint32_t	auth_negs;		/* auth_negs */
	uint32_t	auth_dones;		/* auth_dones */
	uint32_t	dhchap_challenges;	/* dhchap_challenges */
	uint32_t	dhchap_replies;		/* dhchap_replies */
	uint32_t	dhchap_successes;	/* dhchap_successes */
};
typedef struct auth_proto_stats_s auth_proto_stats_t;

/**
 * Authentication related statistics
 */
struct bfa_auth_stats_s {
	uint32_t	auth_failures;	/*!< authentication failures */
	uint32_t	auth_successes;	/*!< authentication successes */
	struct auth_proto_stats_s auth_rx_stats; /*!< Rx protocol stats */
	struct auth_proto_stats_s auth_tx_stats; /*!< Tx protocol stats */
};
typedef struct bfa_auth_stats_s bfa_auth_stats_t;

/**
 * Authentication hash function algorithms
 */
enum bfa_auth_algo {
	BFA_AUTH_ALGO_MD5 	= 1,	/*!< Message-Digest algorithm 5 */
	BFA_AUTH_ALGO_SHA1 	= 2,	/*!< Secure Hash Algorithm 1 */
	BFA_AUTH_ALGO_MS 	= 3,	/*!< MD5, then SHA-1 */
	BFA_AUTH_ALGO_SM 	= 4,	/*!< SHA-1, then MD5 */
};
typedef enum bfa_auth_algo bfa_auth_algo_t;

/**
 * DH Groups
 *
 * Current value could be combination of one or more of the following values
 */
enum bfa_auth_group {
	BFA_AUTH_GROUP_DHNULL 	= 0,	/*!< DH NULL (value == 0) */
	BFA_AUTH_GROUP_DH768 	= 1,	/*!< DH group 768 (value == 1) */
	BFA_AUTH_GROUP_DH1024 	= 2,	/*!< DH group 1024 (value == 2) */
	BFA_AUTH_GROUP_DH1280 	= 4,	/*!< DH group 1280 (value == 3) */
	BFA_AUTH_GROUP_DH1536 	= 8,	/*!< DH group 1536 (value == 4) */

	BFA_AUTH_GROUP_ALL 	= 256	/*!< Use default DH group order */
					 /*    0, 1, 2, 3, 4 */
};
typedef enum bfa_auth_group bfa_auth_group_t;

/**
 * Authentication secret sources
 */
enum bfa_auth_secretsource {
	BFA_AUTH_SECSRC_LOCAL 	= 1,	/*!< locally configured */
	BFA_AUTH_SECSRC_RADIUS 	= 2,	/*!< use radius server */
	BFA_AUTH_SECSRC_TACACS 	= 3,	/*!< TACACS server */
};
typedef enum bfa_auth_secretsource bfa_auth_secretsource_t;

/**
 * Authentication attributes
 */
struct bfa_auth_attr_s {
	enum bfa_auth_status 	status;
	enum bfa_auth_algo 	algo;
	enum bfa_auth_algo	neg_algo;
	uint32_t		rsvd1;
	enum bfa_auth_group 	dh_grp;
	enum bfa_auth_rej_code		rjt_code;
	enum bfa_auth_rej_code_exp		rjt_code_exp;
	uint8_t			secret_set;
	uint8_t			resv[3];
};
typedef struct bfa_auth_attr_s bfa_auth_attr_t;


#define BFA_FCS_MAX_LPORTS 256

/**
 * BFA AEN logical port events.
 * Arguments below are in BFAL context from Mgmt
 * BFA_LPORT_AEN_NEW:       [in]: None	 [out]: vf_id, ppwwn, lpwwn, roles
 * BFA_LPORT_AEN_DELETE:    [in]: lpwwn	[out]: vf_id, ppwwn. lpwwn, roles
 * BFA_LPORT_AEN_ONLINE:    [in]: lpwwn	[out]: vf_id, ppwwn. lpwwn, roles
 * BFA_LPORT_AEN_OFFLINE:   [in]: lpwwn	[out]: vf_id, ppwwn. lpwwn, roles
 * BFA_LPORT_AEN_DISCONNECT:[in]: lpwwn	[out]: vf_id, ppwwn. lpwwn, roles
 * BFA_LPORT_AEN_NEW_PROP:  [in]: None	 [out]: vf_id, ppwwn. lpwwn, roles
 * BFA_LPORT_AEN_DELETE_PROP:     [in]: lpwwn  [out]: vf_id, ppwwn. lpwwn, roles
 * BFA_LPORT_AEN_NEW_STANDARD:    [in]: None   [out]: vf_id, ppwwn. lpwwn, roles
 * BFA_LPORT_AEN_DELETE_STANDARD: [in]: lpwwn  [out]: vf_id, ppwwn. lpwwn, roles
 * BFA_LPORT_AEN_NPIV_DUP_WWN:    [in]: lpwwn  [out]: vf_id, ppwwn. lpwwn, roles
 * BFA_LPORT_AEN_NPIV_FABRIC_MAX: [in]: lpwwn  [out]: vf_id, ppwwn. lpwwn, roles
 * BFA_LPORT_AEN_NPIV_UNKNOWN:    [in]: lpwwn  [out]: vf_id, ppwwn. lpwwn, roles
 */
enum bfa_lport_aen_event {
	BFA_LPORT_AEN_NEW	= 1,	/*!< LPort created event */
	BFA_LPORT_AEN_DELETE	= 2,	/*!< LPort deleted event */
	BFA_LPORT_AEN_ONLINE	= 3,	/*!< LPort online event */
	BFA_LPORT_AEN_OFFLINE	= 4,	/*!< LPort offline event */
	BFA_LPORT_AEN_DISCONNECT = 5,	/*!< LPort disconnect event */
	BFA_LPORT_AEN_NEW_PROP	= 6,	/*!< VPort created event */
	BFA_LPORT_AEN_DELETE_PROP = 7,	/*!< VPort deleted event */
	BFA_LPORT_AEN_NEW_STANDARD = 8,	/*!< VPort created event */
	BFA_LPORT_AEN_DELETE_STANDARD = 9,  /*!< VPort deleted event */
	BFA_LPORT_AEN_NPIV_DUP_WWN = 10,    /*!< VPort configured with */
						/* duplicate WWN event */
	BFA_LPORT_AEN_NPIV_FABRIC_MAX = 11, /*!< Max NPIV in fabric/fport */
	BFA_LPORT_AEN_NPIV_UNKNOWN = 12, /*!< Unknown NPIV Error code event */
	BFA_LPORT_AEN_NEW_SYNTH_FC = 13,	/*!< VPort created event */
	BFA_LPORT_AEN_DELETE_SYNTH_FC = 14,  /*!< VPort deleted event */
};
typedef enum bfa_lport_aen_event bfa_lport_aen_event_t;

/**
 * BFA AEN event data structure
 */
struct bfa_lport_aen_data_s {
	uint16_t	vf_id;	/*!< vf_id of this logical port */
	int16_t	 roles;	/*!< Logical port mode,IM/TM/IP etc */
	uint32_t	rsvd;
	wwn_t	   ppwwn;	/*!< WWN of its physical port */
	wwn_t	   lpwwn;	/*!< WWN of this logical port */
};
typedef struct bfa_lport_aen_data_s bfa_lport_aen_data_t;

#define BFA_FCS_FABRIC_IPADDR_SZ  16

/**
 * symbolic names for base port/virtual port
 */
#define BFA_SYMNAME_MAXLEN	128	/* vmware/windows uses 128 bytes */
struct bfa_lport_symname_s {
	char	    symname[BFA_SYMNAME_MAXLEN];
};
typedef struct bfa_lport_symname_s bfa_lport_symname_t;

/**
* Roles of FCS port:
 *     - FCP IM and FCP TM roles cannot be enabled together for a FCS port
 *     - Create multiple ports if both IM and TM functions required.
 *     - Atleast one role must be specified.
 */
enum bfa_lport_role {
	BFA_LPORT_ROLE_FCP_IM 	= 0x01,	/*!< FCP initiator role */
	BFA_LPORT_ROLE_FCP_TM 	= 0x02,	/*!< FCP target role */
	BFA_LPORT_ROLE_FCP_IPFC	= 0x04,	/*!< IP over FC role */
	BFA_LPORT_ROLE_FCP_MAX = BFA_LPORT_ROLE_FCP_IPFC | BFA_LPORT_ROLE_FCP_IM
};
typedef enum bfa_lport_role bfa_lport_role_t;

enum bfa_lport_mode {
	BFA_PORT_MODE_FC	= 0x01,	/*!< FC mode */
	BFA_PORT_MODE_FCOE	= 0x02,	/*!< FCoE mode */
};
typedef enum bfa_lport_mode bfa_lport_mode_t;

/**
 * Refer to fc_prli_params_t
 */
struct bfa_lport_itn_attr_s {
	uint8_t retry		:1;
	uint8_t task_retry_id	:1;
	uint8_t rec_support	:1;
	uint8_t confirm		:1;
	uint8_t rsvd1		:4;
	uint8_t rsvd2[3];
};
typedef struct bfa_lport_itn_attr_s bfa_lport_itn_attr_t;

/**
 * FCS port configuration.
 */
struct bfa_lport_cfg_s {
	wwn_t		pwwn;		/*!< port wwn */
	wwn_t		nwwn;		/*!< node wwn */
	struct bfa_lport_symname_s sym_name;   /*!< vm port symbolic name */
	struct bfa_lport_symname_s node_sym_name; /*!< Node symbolic name */
	enum bfa_lport_role roles;	/*!< FCS port roles */
	union {
		struct {
			uint32_t	rsvd;
		} im_port_attrib;
		struct bfa_lport_itn_attr_s itn_attr;
	} un;
	bfa_boolean_t	preboot_vp;	/*!< vport created from PBC */
	uint8_t		tag[16];	/*!< opaque tag from application */
	bfa_boolean_t	synth_fc;
};
typedef struct bfa_lport_cfg_s bfa_lport_cfg_t;

/**
 * FCS port states
 */
enum bfa_lport_state {
	BFA_LPORT_UNINIT  = 0,	/*!< PORT is not yet initialized */
	BFA_LPORT_FDISC   = 1,	/*!< FDISC is in progress */
	BFA_LPORT_ONLINE  = 2,	/*!< login to fabric is complete */
	BFA_LPORT_OFFLINE = 3,	/*!< No login to fabric */
};
typedef enum bfa_lport_state bfa_lport_state_t;

/**
 * FCS port type. Required for VmWare.
 */
enum bfa_lport_type {
	BFA_LPORT_TYPE_PHYSICAL = 0,
	BFA_LPORT_TYPE_VIRTUAL,
};
typedef enum bfa_lport_type bfa_lport_type_t;

/**
 * FCS port offline reason. Required for VmWare.
 */
enum bfa_lport_offline_reason {
	BFA_LPORT_OFFLINE_UNKNOWN = 0,
	BFA_LPORT_OFFLINE_LINKDOWN,
	BFA_LPORT_OFFLINE_FAB_UNSUPPORTED, /*!< NPIV not supported by the */
					   /* fabric */
	BFA_LPORT_OFFLINE_FAB_NORESOURCES,
	BFA_LPORT_OFFLINE_FAB_LOGOUT,
};
typedef enum bfa_lport_offline_reason bfa_lport_offline_reason_t;

/**
 * FCS lport info. Required for VmWare.
 */
struct bfa_lport_info_s {
	uint8_t	 port_type;	/* bfa_lport_type_t : physical or */
				/* virtual */
	uint8_t	 port_state;	/* one of bfa_lport_state values */
	uint8_t	 offline_reason; /* one of bfa_lport_offline_reason_t */
				/* values */
	wwn_t	   port_wwn;
	wwn_t	   node_wwn;

	/*
	 * following 4 feilds are valid for Physical Ports only
	 */
	uint32_t	max_vports_supp;	/* Max supported vports */
	uint32_t	num_vports_inuse;	/* Num of in use vports */
	uint32_t	max_rports_supp;	/* Max supported rports */
	uint32_t	num_rports_inuse;	/* Num of doscovered rports */

};
typedef struct bfa_lport_info_s bfa_lport_info_t;

/**
 * FCS port statistics
 */
struct bfa_lport_stats_s {
	uint32_t	ns_plogi_sent;		/* ns_plogi_sent */
	uint32_t	ns_plogi_rsp_err;	/* ns_plogi_rsp_err */
	uint32_t	ns_plogi_acc_err;	/* ns_plogi_acc_err */
	uint32_t	ns_plogi_accepts;	/* ns_plogi_accepts */
	uint32_t	ns_rejects;		/* NS command rejects */
	uint32_t	ns_plogi_unknown_rsp;	/* ns_plogi_unknown_rsp */
	uint32_t	ns_plogi_alloc_wait;	/* ns_plogi_alloc_wait */

	uint32_t	ns_retries;		/* NS command retries */
	uint32_t	ns_timeouts;		/* NS command timeouts */

	uint32_t	ns_rspnid_sent;		/* ns_rspnid_sent */
	uint32_t	ns_rspnid_accepts;	/* ns_rspnid_accepts */
	uint32_t	ns_rspnid_rsp_err;	/* ns_rspnid_rsp_err */
	uint32_t	ns_rspnid_rejects;	/* ns_rspnid_rejects */
	uint32_t	ns_rspnid_alloc_wait;	/* ns_rspnid_alloc_wait */

	uint32_t	ns_rftid_sent;		/* ns_rftid_sent */
	uint32_t	ns_rftid_accepts;	/* ns_rftid_accepts */
	uint32_t	ns_rftid_rsp_err;	/* ns_rftid_rsp_err */
	uint32_t	ns_rftid_rejects;	/* ns_rftid_rejects */
	uint32_t	ns_rftid_alloc_wait;	/* ns_rftid_alloc_wait */

	uint32_t	ns_rffid_sent;		/* ns_rffid_sent */
	uint32_t	ns_rffid_accepts;	/* ns_rffid_accepts */
	uint32_t	ns_rffid_rsp_err;	/* ns_rffid_rsp_err */
	uint32_t	ns_rffid_rejects;	/* ns_rffid_rejects */
	uint32_t	ns_rffid_alloc_wait;	/* ns_rffid_alloc_wait */

	uint32_t	ns_gidft_sent;		/* ns_gidft_sent */
	uint32_t	ns_gidft_accepts;	/* ns_gidft_accepts */
	uint32_t	ns_gidft_rsp_err;	/* ns_gidft_rsp_err */
	uint32_t	ns_gidft_rejects;	/* ns_gidft_rejects */
	uint32_t	ns_gidft_unknown_rsp;	/* ns_gidft_unknown_rsp */
	uint32_t	ns_gidft_alloc_wait;	/* ns_gidft_alloc_wait */

	uint32_t	ns_rnnid_sent;		/* ns_rnnid_sent */
	uint32_t	ns_rnnid_accepts;	/* ns_rnnid_accepts */
	uint32_t	ns_rnnid_rsp_err;	/* ns_rnnid_rsp_err */
	uint32_t	ns_rnnid_rejects;	/* ns_rnnid_rejects */
	uint32_t	ns_rnnid_alloc_wait;	/* ns_rnnid_alloc_wait */

	uint32_t	ns_rsnn_nn_sent;	/* ns_rsnn_nn_sent */
	uint32_t	ns_rsnn_nn_accepts;	/* ns_rsnn_nn_accepts */
	uint32_t	ns_rsnn_nn_rsp_err;	/* ns_rsnn_nn_rsp_err */
	uint32_t	ns_rsnn_nn_rejects;	/* ns_rsnn_nn_rejects */
	uint32_t	ns_rsnn_nn_alloc_wait;	/* ns_rsnn_nn_alloc_wait */

	/*
	 * Mgmt Server stats
	 */
	uint32_t	ms_retries;		/* MS command retries */
	uint32_t	ms_timeouts;		/* MS command timeouts */
	uint32_t	ms_plogi_sent;		/* ms_plogi_sent */
	uint32_t	ms_plogi_rsp_err;	/* ms_plogi_rsp_err */
	uint32_t	ms_plogi_acc_err;	/* ms_plogi_acc_err */
	uint32_t	ms_plogi_accepts;	/* ms_plogi_accepts */
	uint32_t	ms_rejects;		/* MS command rejects */
	uint32_t	ms_plogi_unknown_rsp;	/* ms_plogi_unknown_rsp */
	uint32_t	ms_plogi_alloc_wait;	/* ms_plogi_alloc_wait */

	uint32_t	num_rscn;	/* Num of RSCN received */
	uint32_t	num_portid_rscn; /* Num portid format RSCN rcvd */
	uint32_t	uf_recvs; 	/* Unsolicited recv frames	*/
	uint32_t	uf_recv_drops; 	/* Dropped received frames	*/

	uint32_t	plogi_rcvd;	/* Received plogi	*/
	uint32_t	prli_rcvd;	/* Received prli	*/
	uint32_t	adisc_rcvd;	/* Received adisc	*/
	uint32_t	prlo_rcvd;	/* Received prlo	*/
	uint32_t	logo_rcvd;	/* Received logo	*/
	uint32_t	rpsc_rcvd;	/* Received rpsc	*/
	uint32_t	un_handled_els_rcvd; /* Received unhandled ELS	*/
	uint32_t	rport_plogi_timeouts; /* Rport plogi retry timeout cnt */
	uint32_t	rport_del_max_plogi_retry; /* Rport deletes due to plogi max retries */

};
typedef struct bfa_lport_stats_s bfa_lport_stats_t;

/**
 * BFA port attribute returned in queries
 */
struct bfa_lport_attr_s {
	enum bfa_lport_state state;	/*!< port state */
	uint32_t	 pid;	/*!< port ID */
	struct bfa_lport_cfg_s   port_cfg;	/*!< port configuration */
	enum bfa_port_type port_type;	/*!< current topology */
	uint32_t	 loopback;	/*!< cable is externally looped back */
	wwn_t	fabric_name; /*!< attached switch's nwwn */
	uint8_t	fabric_ip_addr[BFA_FCS_FABRIC_IPADDR_SZ]; /*!< attached */
						/* fabric's ip addr */
	mac_t	fpma_mac;	/*!< Lport's FPMA Mac address */
	uint16_t authfail;	/*!< auth failed state */
};
typedef struct bfa_lport_attr_s bfa_lport_attr_t;


/**
 * VPORT states
 */
enum bfa_vport_state {
	BFA_FCS_VPORT_UNINIT 		= 0,
	BFA_FCS_VPORT_CREATED 		= 1,
	BFA_FCS_VPORT_OFFLINE 		= 1,
	BFA_FCS_VPORT_FDISC_SEND 	= 2,
	BFA_FCS_VPORT_FDISC 		= 3,
	BFA_FCS_VPORT_FDISC_RETRY 	= 4,
	BFA_FCS_VPORT_FDISC_RSP_WAIT	= 5,
	BFA_FCS_VPORT_ONLINE 		= 6,
	BFA_FCS_VPORT_DELETING 		= 7,
	BFA_FCS_VPORT_CLEANUP 		= 8,
	BFA_FCS_VPORT_LOGO_SEND 	= 9,
	BFA_FCS_VPORT_LOGO 		= 10,
	BFA_FCS_VPORT_ERROR		= 11,
	BFA_FCS_VPORT_MAX_STATE,
};
typedef enum bfa_vport_state bfa_vport_state_t;

/**
 * vport statistics
 */
struct bfa_vport_stats_s {
	struct bfa_lport_stats_s port_stats;	/*!< base class (port) stats */
	/*
	 * TODO - remove
	 */

	uint32_t	fdisc_sent;	/*!< num fdisc sent */
	uint32_t	fdisc_accepts;	/*!< fdisc accepts */
	uint32_t	fdisc_retries;	/*!< fdisc retries */
	uint32_t	fdisc_timeouts;	/*!< fdisc timeouts */
	uint32_t	fdisc_rsp_err;	/*!< fdisc response error */
	uint32_t	fdisc_acc_bad;	/*!< bad fdisc accepts */
	uint32_t	fdisc_rejects;	/*!< fdisc rejects */
	uint32_t	fdisc_unknown_rsp;
	/*
	 *!< fdisc rsp unknown error
	 */
	uint32_t	fdisc_alloc_wait; /*!< fdisc req (fcxp) alloc wait */

	uint32_t	logo_alloc_wait; /*!< logo req (fcxp) alloc wait */
	uint32_t	logo_sent;	/*!< logo sent */
	uint32_t	logo_accepts;	/*!< logo accepts */
	uint32_t	logo_rejects;	/*!< logo rejects */
	uint32_t	logo_rsp_err;	/*!< logo rsp errors */
	uint32_t	logo_unknown_rsp; /*!< logo rsp unknown errors */

	uint32_t	fab_no_npiv;	/*!< fabric does not support npiv */

	uint32_t	fab_offline;	/*!< offline events from fab SM */
	uint32_t	fab_online;	/*!< online events from fab SM */
	uint32_t	fab_cleanup;	/*!< cleanup request from fab SM */
	uint32_t	rsvd;
};
typedef struct bfa_vport_stats_s bfa_vport_stats_t;

/**
 * BFA vport attribute returned in queries
 */
struct bfa_vport_attr_s {
	struct bfa_lport_attr_s   port_attr;
				/*!< base class (port) attributes */
	enum bfa_vport_state vport_state; /*!< vport state */
	uint32_t	  rsvd;
};
typedef struct bfa_vport_attr_s bfa_vport_attr_t;

/**
 * FCS remote port states
 */
enum bfa_rport_state {
	BFA_RPORT_UNINIT 	= 0,	/*!< PORT is not yet initialized */
	BFA_RPORT_OFFLINE 	= 1,	/*!< rport is offline */
	BFA_RPORT_PLOGI 	= 2,	/*!< PLOGI to rport is in progress */
	BFA_RPORT_ONLINE 	= 3,	/*!< login to rport is complete */
	BFA_RPORT_PLOGI_RETRY 	= 4,	/*!< retrying login to rport */
	BFA_RPORT_NSQUERY 	= 5,	/*!< nameserver query */
	BFA_RPORT_ADISC 	= 6,	/*!< ADISC authentication */
	BFA_RPORT_LOGO 		= 7,	/*!< logging out with rport */
	BFA_RPORT_LOGORCV 	= 8,	/*!< handling LOGO from rport */
	BFA_RPORT_NSDISC 	= 9,	/*!< re-discover rport */
	BFA_RPORT_FC4_OFF_PLOGI	= 10,	/*!< plogi recv while FC4 offline */
	BFA_RPORT_PLOGI_PENDING	= 11,	/*!< inbound plogi is pending */
	BFA_RPORT_FC4_OFF_DELETE = 12,	/*!< delete recv while FC4 offline */
	BFA_RPORT_DELETE_PENDING = 13,	/*!< delete is pending */
};
typedef enum bfa_rport_state bfa_rport_state_t;

/**
 *  Rport Scsi Function : Initiator/Target.
 */
enum bfa_rport_function {
	BFA_RPORT_INITIATOR 	= 0x01,	/*!< SCSI Initiator	*/
	BFA_RPORT_TARGET 	= 0x02,	/*!< SCSI Target	*/
};
typedef enum bfa_rport_function bfa_rport_function_t;

/**
 * port/node symbolic names for rport
 */
#define BFA_RPORT_SYMNAME_MAXLEN	255
struct bfa_rport_symname_s {
	char	    symname[BFA_RPORT_SYMNAME_MAXLEN];
};
typedef struct bfa_rport_symname_s bfa_rport_symname_t;

/**
 * FCS remote port statistics
 */
struct bfa_rport_stats_s {
	uint32_t	offlines;	/*!< remote port offline count  */
	uint32_t	onlines;	/*!< remote port online count   */
	uint32_t	rscns;		/*!< RSCN affecting rport	*/
	uint32_t	plogis;		/*!< plogis sent		*/
	uint32_t	plogi_accs;	/*!< plogi accepts		*/
	uint32_t	plogi_timeouts;	/*!< plogi timeouts		*/
	uint32_t	plogi_rejects;	/*!< rcvd plogi rejects		*/
	uint32_t	plogi_failed;	/*!< local failure		*/
	uint32_t	plogi_rcvd;	/*!< plogis rcvd		*/
	uint32_t	prli_rcvd;	/*!< inbound PRLIs		*/
	uint32_t	adisc_rcvd;	/*!< ADISCs received		*/
	uint32_t	adisc_rejects;	/*!< recvd  ADISC rejects	*/
	uint32_t	adisc_sent;	/*!< ADISC requests sent	*/
	uint32_t	adisc_accs;	/*!< ADISC accepted by rport	*/
	uint32_t	adisc_failed;	/*!< ADISC failed no response */
	uint32_t	adisc_rejected;	/*!< ADISC rejected by us	*/
	uint32_t	logos;		/*!< logos sent			*/
	uint32_t	logo_accs;	/*!< LOGO accepts from rport	*/
	uint32_t	logo_failed;	/*!< LOGO failures		*/
	uint32_t	logo_rejected;	/*!< LOGO rejects from rport	*/
	uint32_t	logo_rcvd;	/*!< LOGO from remote port	*/

	uint32_t	rpsc_rcvd;	/*!< RPSC received		*/
	uint32_t	rpsc_rejects;	/*!< recvd  RPSC rejects	*/
	uint32_t	rpsc_sent;	/*!< RPSC requests sent		*/
	uint32_t	rpsc_accs;	/*!< RPSC accepted by rport	*/
	uint32_t	rpsc_failed;	/*!< RPSC failed no response	*/
	uint32_t	rpsc_rejected;	/*!< RPSC rejected by us	*/

	uint32_t	rjt_insuff_res;	/*!< LS RJT with insuff resources */
	struct bfa_rport_hal_stats_s	hal_stats;  /*!< BFA rport stats */
};
typedef struct bfa_rport_stats_s bfa_rport_stats_t;

/**
 * FCS remote port attributes returned in queries
 */
struct bfa_rport_attr_s {
	wwn_t		nwwn;	/*!< node wwn */
	wwn_t		pwwn;	/*!< port wwn */
	fc_cos_t	cos_supported;	/*!< supported class of services */
	uint32_t		pid;	/*!< port ID */
	uint32_t		df_sz;	/*!< Max payload size */
	enum bfa_rport_state 	state;	/*!< Rport State machine state */
	fc_cos_t		fc_cos;	/*!< FC classes of services */
	bfa_boolean_t   	cisc;	/*!< CISC capable device */
	struct bfa_rport_symname_s symname; /*!< Symbolic Name */
	enum bfa_rport_function	scsi_function; /*!< Initiator/Target */
	struct bfa_rport_qos_attr_s qos_attr; /*!< qos attributes  */
	enum bfa_port_speed curr_speed;   /*!< operating speed got from */
					    /* RPSC ELS. UNKNOWN, if RPSC */
					    /* is not supported */
	bfa_boolean_t 	trl_enforced;	/*!< TRL enforced ? TRUE/FALSE */
	enum bfa_port_speed assigned_speed; /* Speed assigned by the user. */
					 /* will be used if RPSC is not */
					 /* supported by the rport */
	bfa_boolean_t	iopf_enabled;	/* is IOPF enabled */
	uint32_t rsvd;
};
typedef struct bfa_rport_attr_s bfa_rport_attr_t;

#define bfa_rport_aen_qos_data_t struct bfa_rport_qos_attr_s

/**
 * BFA remote port events
 * Arguments below are in BFAL context from Mgmt
 * BFA_RPORT_AEN_ONLINE:    [in]: lpwwn	[out]: vf_id, lpwwn, rpwwn
 * BFA_RPORT_AEN_OFFLINE:   [in]: lpwwn [out]: vf_id, lpwwn, rpwwn
 * BFA_RPORT_AEN_DISCONNECT:[in]: lpwwn [out]: vf_id, lpwwn, rpwwn
 * BFA_RPORT_AEN_QOS_PRIO:  [in]: lpwwn [out]: vf_id, lpwwn, rpwwn, prio
 * BFA_RPORT_AEN_QOS_FLOWID:[in]: lpwwn [out]: vf_id, lpwwn, rpwwn, flow_id
 */
enum bfa_rport_aen_event {
	BFA_RPORT_AEN_ONLINE      = 1,	/*!< RPort online event */
	BFA_RPORT_AEN_OFFLINE     = 2,	/*!< RPort offline event */
	BFA_RPORT_AEN_DISCONNECT  = 3,	/*!< RPort disconnect event */
	BFA_RPORT_AEN_QOS_PRIO    = 4,	/*!< QOS priority change event */
	BFA_RPORT_AEN_QOS_FLOWID  = 5,	/*!< QOS flow Id change event */
};
typedef enum bfa_rport_aen_event bfa_rport_aen_event_t;

struct bfa_rport_aen_data_s {
	uint16_t	vf_id;	/*!< vf_id of this logical port */
	uint16_t	rsvd[3];
	wwn_t	   ppwwn;	/*!< WWN of its physical port */
	wwn_t	   lpwwn;	/*!< WWN of this logical port */
	wwn_t	   rpwwn;	/*!< WWN of this remote port */
	union {
		bfa_rport_aen_qos_data_t qos;
	} priv;
};
typedef struct bfa_rport_aen_data_s bfa_rport_aen_data_t;

struct bfa_rport_remote_link_stats_s {
	uint32_t lfc; /*!< Link Failure Count */
	uint32_t lsyc; /*!< Loss of Synchronization Count */
	uint32_t lsic; /*!< Loss of Signal Count */
	uint32_t pspec; /*!< Primitive Sequence Protocol Error Count */
	uint32_t itwc; /*!< Invalid Transmission Word Count */
	uint32_t icc; /*!< Invalid CRC Count */
};
typedef struct bfa_rport_remote_link_stats_s bfa_rport_remote_link_stats_t;

typedef struct bfa_rport_qualifier_s {
	wwn_t		pwwn;	/*!< Port WWN */
	uint32_t	pid;	/*!< port ID */
	uint32_t	rsvd;
}bfa_rport_qualifier_t;

#define BFA_MAX_IO_INDEX 7
#define BFA_NO_IO_INDEX 9

/**
 * FCS itnim states
 */
enum bfa_itnim_state {
	BFA_ITNIM_OFFLINE 	= 0,	/*!< offline */
	BFA_ITNIM_PRLI_SEND 	= 1,	/*!< prli send */
	BFA_ITNIM_PRLI_SENT 	= 2,	/*!< prli sent */
	BFA_ITNIM_PRLI_RETRY 	= 3,	/*!< prli retry */
	BFA_ITNIM_HCB_ONLINE 	= 4,	/*!< online callback */
	BFA_ITNIM_ONLINE 	= 5,	/*!< online */
	BFA_ITNIM_HCB_OFFLINE 	= 6,	/*!< offline callback */
	BFA_ITNIM_INITIATIOR 	= 7,	/*!< initiator */
};
typedef enum bfa_itnim_state bfa_itnim_state_t;

/**
 * FCS remote port statistics
 */
struct bfa_itnim_stats_s {
	uint32_t	onlines;	/*!< num rport online */
	uint32_t	offlines;	/*!< num rport offline */
	uint32_t	prli_sent;	/*!< num prli sent out */
	uint32_t	fcxp_alloc_wait; /*!< num fcxp alloc waits */
	uint32_t	prli_rsp_err;	/*!< num prli rsp errors */
	uint32_t	prli_rsp_acc;	/*!< num prli rsp accepts */
	uint32_t	initiator;	/*!< rport is an initiator */
	uint32_t	prli_rsp_parse_err; /*!< prli rsp parsing errors */
	uint32_t	prli_rsp_rjt;	/*!< num prli rsp rejects */
	uint32_t	timeout;	/*!< num timeouts detected */
	uint32_t	sler;		/*!< num sler notification from BFA */
	uint32_t	rsvd;		/* padding for 64 bit alignment */
};
typedef struct bfa_itnim_stats_s bfa_itnim_stats_t;

/**
 * FCS itnim attributes returned in queries
 */
struct bfa_itnim_attr_s {
	enum bfa_itnim_state state; /*!< FCS itnim state	*/
	uint8_t retry;		/*!< data retransmision support */
	uint8_t task_retry_id;  /*!< task retry ident support   */
	uint8_t rec_support;    /*!< REC supported	      */
	uint8_t conf_comp;      /*!< confirmed completion supp  */
	uint8_t trs_blocked; /*!< Target Reset disabled */
	uint8_t rsvd[7]; /* padding for 64 bit alignment */
};
typedef struct bfa_itnim_attr_s bfa_itnim_attr_t;

/**
 * BFA ITNIM events.
 * Arguments below are in BFAL context from Mgmt
 * BFA_ITNIM_AEN_NEW:       [in]: None  [out]: vf_id, lpwwn
 * BFA_ITNIM_AEN_DELETE:    [in]: vf_id, lpwwn, rpwwn (0 = all fcp4 targets),
 *				  [out]: vf_id, ppwwn, lpwwn, rpwwn
 * BFA_ITNIM_AEN_ONLINE:    [in]: vf_id, lpwwn, rpwwn (0 = all fcp4 targets),
 *				  [out]: vf_id, ppwwn, lpwwn, rpwwn
 * BFA_ITNIM_AEN_OFFLINE:   [in]: vf_id, lpwwn, rpwwn (0 = all fcp4 targets),
 *				  [out]: vf_id, ppwwn, lpwwn, rpwwn
 * BFA_ITNIM_AEN_DISCONNECT:[in]: vf_id, lpwwn, rpwwn (0 = all fcp4 targets),
 *				  [out]: vf_id, ppwwn, lpwwn, rpwwn
 */
enum bfa_itnim_aen_event {
	BFA_ITNIM_AEN_ONLINE 	= 1,	/*!< Target online */
	BFA_ITNIM_AEN_OFFLINE 	= 2,	/*!< Target offline */
	BFA_ITNIM_AEN_DISCONNECT = 3,	/*!< Target disconnected */
};
typedef enum bfa_itnim_aen_event bfa_itnim_aen_event_t;

/**
 * BFA ITNIM event data structure.
 */
struct bfa_itnim_aen_data_s {
	uint16_t	vf_id;	/*!< vf_id of the IT nexus */
	uint16_t	rsvd[3];
	wwn_t	   ppwwn;	/*!< WWN of its physical port */
	wwn_t	   lpwwn;	/*!< WWN of logical port */
	wwn_t	   rpwwn;	/*!< WWN of remote(target) port */
};
typedef struct bfa_itnim_aen_data_s bfa_itnim_aen_data_t;

#endif /* __BFA_DEFS_ITNIM_H__ */
