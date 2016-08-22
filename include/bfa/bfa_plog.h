/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */
#ifndef __BFA_PORTLOG_H__
#define __BFA_PORTLOG_H__

#include "protocol/bfa_fc.h"
#include <defs/bfa_defs.h>

#define BFA_PL_NLOG_ENTS 256
#define BFA_PL_LOG_REC_INCR(_x) ((_x)++, (_x) %= BFA_PL_NLOG_ENTS)

#define BFA_PL_STRING_LOG_SZ   32   /* number of chars in string log */
#define BFA_PL_INT_LOG_SZ      8    /* number of integers in the integer log */

enum bfa_plog_log_type {
	BFA_PL_LOG_TYPE_INVALID = 0,
	BFA_PL_LOG_TYPE_INT 	= 1,
	BFA_PL_LOG_TYPE_STRING 	= 2,
};
typedef enum bfa_plog_log_type bfa_plog_log_type_t;

/*
 * the (fixed size) record format for each entry in the portlog
 */
struct bfa_plog_rec_s {
	uint64_t	tv;	/* timestamp */
	uint8_t	 port;	/* Source port that logged this entry */
	uint8_t	 mid;	/* module id */
	uint8_t	 eid;	/* indicates Rx, Tx, IOCTL, etc.  bfa_plog_eid */
	uint8_t	 log_type; /* string/integer log, bfa_plog_log_type_t */
	uint8_t	 log_num_ints;
	/*
	 * interpreted only if log_type is INT_LOG. indicates number of
	 * integers in the int_log[] (0-PL_INT_LOG_SZ).
	 */
	uint8_t	 rsvd;
	uint16_t	misc;	/* can be used to indicate fc frame length */
	union {
		char	    string_log[BFA_PL_STRING_LOG_SZ];
		uint32_t	int_log[BFA_PL_INT_LOG_SZ];
	} log_entry;

};
typedef struct bfa_plog_rec_s bfa_plog_rec_t;

/*
 * the following #defines will be used by the logging entities to indicate
 * their module id. BFAL will convert the integer value to string format
 *
* process to be used while changing the following #defines:
 *  - Always add new entries at the end
 *  - define corresponding string in BFAL
 *  - Do not remove any entry or rearrange the order.
 */
enum bfa_plog_mid {
	BFA_PL_MID_INVALID 	= 0,
	BFA_PL_MID_DEBUG 	= 1,
	BFA_PL_MID_DRVR 	= 2,
	BFA_PL_MID_HAL 		= 3,
	BFA_PL_MID_HAL_FCXP 	= 4,
	BFA_PL_MID_HAL_UF 	= 5,
	BFA_PL_MID_FCS 		= 6,
	BFA_PL_MID_LPS		= 7,
	BFA_PL_MID_MAX 		= 8
};
typedef enum bfa_plog_mid bfa_plog_mid_t;

#define BFA_PL_MID_STRLEN    8
struct bfa_plog_mid_strings_s {
	char	    m_str[BFA_PL_MID_STRLEN];
};
typedef struct bfa_plog_mid_strings_s bfa_plog_mid_strings_t;

/*
 * the following #defines will be used by the logging entities to indicate
 * their event type. BFAL will convert the integer value to string format
 *
* process to be used while changing the following #defines:
 *  - Always add new entries at the end
 *  - define corresponding string in BFAL
 *  - Do not remove any entry or rearrange the order.
 */
enum bfa_plog_eid {
	BFA_PL_EID_INVALID 		= 0,
	BFA_PL_EID_IOC_DISABLE 		= 1,
	BFA_PL_EID_IOC_ENABLE 		= 2,
	BFA_PL_EID_PORT_DISABLE 	= 3,
	BFA_PL_EID_PORT_ENABLE 		= 4,
	BFA_PL_EID_PORT_ST_CHANGE 	= 5,
	BFA_PL_EID_TX 			= 6,
	BFA_PL_EID_TX_ACK1 		= 7,
	BFA_PL_EID_TX_RJT 		= 8,
	BFA_PL_EID_TX_BSY 		= 9,
	BFA_PL_EID_RX 			= 10,
	BFA_PL_EID_RX_ACK1 		= 11,
	BFA_PL_EID_RX_RJT 		= 12,
	BFA_PL_EID_RX_BSY 		= 13,
	BFA_PL_EID_CT_IN 		= 14,
	BFA_PL_EID_CT_OUT 		= 15,
	BFA_PL_EID_DRIVER_START 	= 16,
	BFA_PL_EID_RSCN 		= 17,
	BFA_PL_EID_DEBUG 		= 18,
	BFA_PL_EID_MISC 		= 19,
	BFA_PL_EID_FIP_FCF_DISC		= 20,
	BFA_PL_EID_FIP_FCF_CVL		= 21,
	BFA_PL_EID_LOGIN		= 22,
	BFA_PL_EID_LOGO			= 23,
	BFA_PL_EID_TRUNK_SCN		= 24,
	BFA_PL_EID_MAX
};
typedef enum bfa_plog_eid bfa_plog_eid_t;

#define BFA_PL_ENAME_STRLEN    	8
struct bfa_plog_eid_strings_s {
	char	    e_str[BFA_PL_ENAME_STRLEN];
};
typedef struct bfa_plog_eid_strings_s bfa_plog_eid_strings_t;

#define BFA_PL_SIG_LEN	8
#define BFA_PL_SIG_STR  "12pl123"

/*
 * per port circular log buffer
 */
struct bfa_plog_s {
	char	    plog_sig[BFA_PL_SIG_LEN];	/* Start signature */
	uint8_t	 plog_enabled;
	uint8_t	 rsvd[7];
	uint32_t	ticks;
	uint16_t	head;
	uint16_t	tail;
	struct bfa_plog_rec_s  plog_recs[BFA_PL_NLOG_ENTS];
};
typedef struct bfa_plog_s bfa_plog_t;

void bfa_plog_init(struct bfa_plog_s *plog);
void bfa_plog_str(struct bfa_plog_s *plog, enum bfa_plog_mid mid,
			enum bfa_plog_eid event, uint16_t misc, char *log_str);
void bfa_plog_intarr(struct bfa_plog_s *plog, enum bfa_plog_mid mid,
			enum bfa_plog_eid event, uint16_t misc,
			uint32_t *intarr, uint32_t num_ints);
void bfa_plog_fchdr(struct bfa_plog_s *plog, enum bfa_plog_mid mid,
			enum bfa_plog_eid event, uint16_t misc, fchs_t *fchdr);
void bfa_plog_fchdr_and_pl(struct bfa_plog_s *plog, enum bfa_plog_mid mid,
			enum bfa_plog_eid event, uint16_t misc,
			fchs_t *fchdr, uint32_t pld_w0);
void bfa_plog_clear(struct bfa_plog_s *plog);
void bfa_plog_enable(struct bfa_plog_s *plog);
void bfa_plog_disable(struct bfa_plog_s *plog);
bfa_boolean_t	bfa_plog_get_setting(struct bfa_plog_s *plog);

#endif /* __BFA_PORTLOG_H__ */
