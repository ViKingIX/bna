/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

/**
 * @file bfa_log.h BFA log library data structure and function definition
 */

#ifndef __BFA_LOG_H__
#define __BFA_LOG_H__

#include <bfa_os_inc.h>
#include <defs/bfa_defs.h>
#include <defs/bfa_defs_aen.h>
#include <stdarg.h>

/*
 * BFA log module definition
 *
 * To create a new module id:
 * Add a #define at the end of the list below. Select a value for your
 * definition so that it is one (1) greater than the previous
 * definition. Modify the definition of BFA_LOG_MODULE_ID_MAX to become
 * your new definition.
 * Should have no gaps in between the values because this is used in arrays.
 * IMPORTANT: AEN_IDs must be at the begining, otherwise update bfa_defs_aen.h
 */

enum bfa_log_module_id {
	BFA_LOG_UNUSED_ID	= 0,

	/* AEN defs begin */
	BFA_LOG_AEN_MIN		= BFA_LOG_UNUSED_ID,

	BFA_LOG_AEN_ID_ADAPTER 	= BFA_LOG_AEN_MIN + BFA_AEN_CAT_ADAPTER, /* 1 */
	BFA_LOG_AEN_ID_PORT 	= BFA_LOG_AEN_MIN + BFA_AEN_CAT_PORT,	/* 2 */
	BFA_LOG_AEN_ID_LPORT 	= BFA_LOG_AEN_MIN + BFA_AEN_CAT_LPORT,	/* 3 */
	BFA_LOG_AEN_ID_RPORT 	= BFA_LOG_AEN_MIN + BFA_AEN_CAT_RPORT,	/* 4 */
	BFA_LOG_AEN_ID_ITNIM 	= BFA_LOG_AEN_MIN + BFA_AEN_CAT_ITNIM,	/* 5 */
	BFA_LOG_AEN_ID_TIN 	= BFA_LOG_AEN_MIN + BFA_AEN_CAT_TIN,	/* 6 */
	BFA_LOG_AEN_ID_IPFC 	= BFA_LOG_AEN_MIN + BFA_AEN_CAT_IPFC,	/* 7 */
	BFA_LOG_AEN_ID_AUDIT 	= BFA_LOG_AEN_MIN + BFA_AEN_CAT_AUDIT,	/* 8 */
	BFA_LOG_AEN_ID_IOC	= BFA_LOG_AEN_MIN + BFA_AEN_CAT_IOC,	/* 9 */
	BFA_LOG_AEN_ID_ETHPORT	= BFA_LOG_AEN_MIN + BFA_AEN_CAT_ETHPORT, /* A */
	BFA_LOG_AEN_ID_TEAM	= BFA_LOG_AEN_MIN + BFA_AEN_CAT_TEAM,	/* 11 */
	BFA_LOG_AEN_ID_VLAN	= BFA_LOG_AEN_MIN + BFA_AEN_CAT_VLAN,	/* 12 */

	BFA_LOG_AEN_MAX		= BFA_LOG_AEN_ID_VLAN,
	/* AEN defs end */

	BFA_LOG_MODULE_ID_MIN	= BFA_LOG_AEN_MAX,

	BFA_LOG_FW_ID		= BFA_LOG_MODULE_ID_MIN + 1,
	BFA_LOG_HAL_ID		= BFA_LOG_MODULE_ID_MIN + 2,
	BFA_LOG_FCS_ID		= BFA_LOG_MODULE_ID_MIN + 3,
	BFA_LOG_WDRV_ID		= BFA_LOG_MODULE_ID_MIN + 4,
	BFA_LOG_LINUX_ID	= BFA_LOG_MODULE_ID_MIN + 5,
	BFA_LOG_SOLARIS_ID	= BFA_LOG_MODULE_ID_MIN + 6,

	BFA_LOG_MODULE_ID_MAX 	= BFA_LOG_SOLARIS_ID,

	/* Not part of any arrays */
	BFA_LOG_MODULE_ID_ALL 	= BFA_LOG_MODULE_ID_MAX + 1,
	BFA_LOG_AEN_ALL 	= BFA_LOG_MODULE_ID_MAX + 2,
	BFA_LOG_DRV_ALL		= BFA_LOG_MODULE_ID_MAX + 3,
};
typedef enum bfa_log_module_id bfa_log_module_id_t;

/*
 * BFA log catalog name
 */
#define BFA_LOG_CAT_NAME	"BFA"

/*
 * bfa log severity values
 */
enum bfa_log_severity {
	BFA_LOG_INVALID = 0,
	BFA_LOG_CRITICAL = 1,
	BFA_LOG_ERROR = 2,
	BFA_LOG_WARNING = 3,
	BFA_LOG_INFO = 4,
	BFA_LOG_NONE = 5,
	BFA_LOG_LEVEL_MAX = BFA_LOG_NONE
};
typedef enum bfa_log_severity bfa_log_severity_t;

#define BFA_LOG_MODID_OFFSET		16

/**
 * @brief log msg definition structure
 */
struct bfa_log_msgdef_s {
	uint32_t	msg_id;		/* !< message id */
	int		attributes;	/* !< attributes */
	int		severity;	/* !< severity level */
	char		*msg_value;
					/* !< msg string */
	char		*message;
					/* !< msg format string */
	int		arg_type;	/* !< argument type */
	int		arg_num;	/* !< number of argument */
};
typedef struct bfa_log_msgdef_s bfa_log_msgdef_t;

/*
 * supported argument type
 */
enum bfa_log_arg_type {
	BFA_LOG_S = 0,		/* !< string */
	BFA_LOG_D,		/* !< decimal */
	BFA_LOG_I,		/* !< integer */
	BFA_LOG_O,		/* !< oct number */
	BFA_LOG_U,		/* !< unsigned integer */
	BFA_LOG_X,		/* !< hex number */
	BFA_LOG_F,		/* !< floating */
	BFA_LOG_C,		/* !< character */
	BFA_LOG_L,		/* !< double */
	BFA_LOG_P		/* !< pointer */
};
typedef enum bfa_log_arg_type bfa_log_arg_type_t;

#define BFA_LOG_ARG_TYPE	4
#define BFA_LOG_ARG0		(0 * BFA_LOG_ARG_TYPE)
#define BFA_LOG_ARG1		(1 * BFA_LOG_ARG_TYPE)
#define BFA_LOG_ARG2		(2 * BFA_LOG_ARG_TYPE)
#define BFA_LOG_ARG3		(3 * BFA_LOG_ARG_TYPE)

#define BFA_LOG_GET_MOD_ID(msgid) ((msgid >> BFA_LOG_MODID_OFFSET) & 0xff)
#define BFA_LOG_GET_MSG_IDX(msgid) (msgid & 0xffff)
#define BFA_LOG_GET_MSG_ID(msgdef) ((msgdef)->msg_id)
#define BFA_LOG_GET_MSG_FMT_STRING(msgdef) ((msgdef)->message)
#define BFA_LOG_GET_SEVERITY(msgdef) ((msgdef)->severity)
#define BFA_LOG_GET_ARG_TYPE(msgdef, index) 				\
	((((msgdef)->arg_type) >> (index * BFA_LOG_ARG_TYPE)) & 0xF);

/*
 * Event attributes
 */
#define BFA_LOG_ATTR_NONE	0
#define BFA_LOG_ATTR_AUDIT	1
#define BFA_LOG_ATTR_LOG	2
#define BFA_LOG_ATTR_FFDC	4

#define BFA_LOG_CREATE_ID(msw, lsw) \
	(((uint32_t)msw << BFA_LOG_MODID_OFFSET) | lsw)

struct bfa_log_mod_s;

/**
 * callback function
 */
typedef void (*bfa_log_cb_t)(struct bfa_log_mod_s *log_mod, uint32_t msg_id,
			va_list params, const char *format, ...);

/**
 * @brief log module info structure
 */
struct bfa_log_mod_s {
	char		instance_info[BFA_STRING_32];	/* !< instance info */
	int		log_level[BFA_LOG_MODULE_ID_MAX + 1];
						/* !< log level for modules */
	bfa_log_cb_t	cbfn; 			/* !< callback function */
};
typedef struct bfa_log_mod_s bfa_log_mod_t;

extern int bfa_log_init(struct bfa_log_mod_s *log_mod,
			const char *instance_name, bfa_log_cb_t cbfn);
extern int bfa_log(struct bfa_log_mod_s *log_mod, uint32_t msg_id, ...);
extern bfa_status_t bfa_log_set_level(struct bfa_log_mod_s *log_mod,
			int mod_id, enum bfa_log_severity log_level);
extern bfa_status_t bfa_log_set_level_all(struct bfa_log_mod_s *log_mod,
			enum bfa_log_severity log_level);
extern bfa_status_t bfa_log_set_level_aen(struct bfa_log_mod_s *log_mod,
			enum bfa_log_severity log_level);
extern enum bfa_log_severity bfa_log_get_level(struct bfa_log_mod_s *log_mod,
			int mod_id);
extern enum bfa_log_severity bfa_log_get_msg_level(
			struct bfa_log_mod_s *log_mod, uint32_t msg_id);
extern bfa_log_msgdef_t* bfa_log_get_msg(
			struct bfa_log_mod_s *log_mod, uint32_t msg_id);
/*
 * array of messages generated from xml files
 */
extern struct bfa_log_msgdef_s bfa_log_msg_array[];

#endif
