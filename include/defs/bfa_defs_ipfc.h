/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */
#ifndef __BFA_DEFS_IPFC_H__
#define __BFA_DEFS_IPFC_H__

#include <defs/bfa_defs.h>

/**
 * FCS ip remote port states
 */
enum bfa_iprp_state {
	BFA_IPRP_UNINIT  = 0,	/*!< PORT is not yet initialized */
	BFA_IPRP_ONLINE  = 1,	/*!< process login is complete */
	BFA_IPRP_OFFLINE = 2,	/*!< iprp is offline */
};
typedef enum bfa_iprp_state bfa_iprp_state_t;

/**
 * FCS remote port statistics
 */
struct bfa_iprp_stats_s {
	uint32_t	offlines;	/* offlines */
	uint32_t	onlines;	/* onlines */
	uint32_t	rscns;		/* rscns */
	uint32_t	plogis;		/* plogis */
	uint32_t	logos;		/* logos */
	uint32_t	plogi_timeouts;	/* plogi_timeouts */
	uint32_t	plogi_rejects;	/* plogi_rejects */
};
typedef struct bfa_iprp_stats_s bfa_iprp_stats_t;

/**
 * FCS iprp attribute returned in queries
 */
struct bfa_iprp_attr_s {
	enum bfa_iprp_state state;
};
typedef struct bfa_iprp_attr_s bfa_iprp_attr_t;

struct bfa_ipfc_stats_s {
	uint32_t arp_sent;	/*!< ARP sent */
	uint32_t arp_recv;	/*!< ARP rcvd */
	uint32_t arp_reply_sent;	/*!< ARP reply sent */
	uint32_t arp_reply_recv;	/*!< ARP reply rcvd */
	uint32_t farp_sent;	/*!< FARP sent */
	uint32_t farp_recv;	/*!< FARP rcvd */
	uint32_t farp_reply_sent;	/*!< FARP reply sent */
	uint32_t farp_reply_recv;	/*!< FARP reply rcvd */
	uint32_t farp_reject_sent;	/*!< FARP reject sent */
	uint32_t farp_reject_recv;	/*!< FARP reject rcvd */
};
typedef struct bfa_ipfc_stats_s bfa_ipfc_stats_t;

struct bfa_ipfc_attr_s {
	bfa_boolean_t enabled;
	uint32_t mtu;
	char	eth_name[BFA_ADAPTER_SYM_NAME_LEN];
};
typedef struct bfa_ipfc_attr_s bfa_ipfc_attr_t;

#endif /* __BFA_DEFS_IPFC_H__ */
