/*
 * Copyright (c)  2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c)  2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __SCSI_H__
#define __SCSI_H__

#include <bfa_os_inc.h>

#pragma pack(1)

/*
 * generic SCSI cdb definition
 */
#define SCSI_MAX_CDBLEN     16
typedef struct {
	uint8_t         scsi_cdb[SCSI_MAX_CDBLEN];
} scsi_cdb_t;

/*
 * scsi lun serial number definition
 */
#define SCSI_LUN_SN_LEN     32
typedef struct {
	uint8_t         lun_sn[SCSI_LUN_SN_LEN];
} scsi_lun_sn_t;

/*
 * SCSI Direct Access Commands
 */
enum {
	SCSI_OP_TEST_UNIT_READY		= 0x00,
	SCSI_OP_REQUEST_SENSE		= 0x03,
	SCSI_OP_FORMAT_UNIT		= 0x04,
	SCSI_OP_READ6			= 0x08,
	SCSI_OP_WRITE6			= 0x0A,
	SCSI_OP_WRITE_FILEMARKS		= 0x10,
	SCSI_OP_INQUIRY			= 0x12,
	SCSI_OP_MODE_SELECT6		= 0x15,
	SCSI_OP_RESERVE6		= 0x16,
	SCSI_OP_RELEASE6		= 0x17,
	SCSI_OP_MODE_SENSE6		= 0x1A,
	SCSI_OP_START_STOP_UNIT		= 0x1B,
	SCSI_OP_SEND_DIAGNOSTIC		= 0x1D,
	SCSI_OP_READ_CAPACITY		= 0x25,
	SCSI_OP_READ10			= 0x28,
	SCSI_OP_WRITE10			= 0x2A,
	SCSI_OP_VERIFY10		= 0x2F,
	SCSI_OP_READ_DEFECT_DATA	= 0x37,
	SCSI_OP_LOG_SELECT		= 0x4C,
	SCSI_OP_LOG_SENSE		= 0x4D,
	SCSI_OP_MODE_SELECT10		= 0x55,
	SCSI_OP_RESERVE10		= 0x56,
	SCSI_OP_RELEASE10		= 0x57,
	SCSI_OP_MODE_SENSE10		= 0x5A,
	SCSI_OP_PER_RESERVE_IN		= 0x5E,
	SCSI_OP_PER_RESERVE_OUR		= 0x5E,
	SCSI_OP_READ16			= 0x88,
	SCSI_OP_WRITE16			= 0x8A,
	SCSI_OP_VERIFY16		= 0x8F,
	SCSI_OP_READ_CAPACITY16		= 0x9E,
	SCSI_OP_REPORT_LUNS		= 0xA0,
	SCSI_OP_READ12			= 0xA8,
	SCSI_OP_WRITE12			= 0xAA,
	SCSI_OP_UNDEF			= 0xFF,
};

/*
 * SCSI START_STOP_UNIT command
 */
typedef struct {
	uint8_t         opcode;
#ifdef __BIGENDIAN
	uint8_t         lun:3;
	uint8_t         reserved1:4;
	uint8_t         immed:1;
#else
	uint8_t         immed:1;
	uint8_t         reserved1:4;
	uint8_t         lun:3;
#endif
	uint8_t         reserved2;
	uint8_t         reserved3;
#ifdef __BIGENDIAN
	uint8_t         power_conditions:4;
	uint8_t         reserved4:2;
	uint8_t         loEj:1;
	uint8_t         start:1;
#else
	uint8_t         start:1;
	uint8_t         loEj:1;
	uint8_t         reserved4:2;
	uint8_t         power_conditions:4;
#endif
	uint8_t         control;
} scsi_start_stop_unit_t;

/*
 * SCSI SEND_DIAGNOSTIC command
 */
typedef struct {
	uint8_t         opcode;
#ifdef __BIGENDIAN
	uint8_t         self_test_code:3;
	uint8_t         pf:1;
	uint8_t         reserved1:1;
	uint8_t         self_test:1;
	uint8_t         dev_offl:1;
	uint8_t         unit_offl:1;
#else
	uint8_t         unit_offl:1;
	uint8_t         dev_offl:1;
	uint8_t         self_test:1;
	uint8_t         reserved1:1;
	uint8_t         pf:1;
	uint8_t         self_test_code:3;
#endif
	uint8_t         reserved2;

	uint8_t         param_list_length[2];	/* MSB first */
	uint8_t         control;

} scsi_send_diagnostic_t;

/*
 * SCSI READ10/WRITE10 commands
 */
typedef struct {
	uint8_t         opcode;
#ifdef __BIGENDIAN
	uint8_t         lun:3;
	uint8_t         dpo:1;	/* Disable Page Out */
	uint8_t         fua:1;	/* Force Unit Access */
	uint8_t         reserved1:2;
	uint8_t         rel_adr:1;	/* relative address */
#else
	uint8_t         rel_adr:1;
	uint8_t         reserved1:2;
	uint8_t         fua:1;
	uint8_t         dpo:1;
	uint8_t         lun:3;
#endif
	uint8_t         lba0;	/* logical block address - MSB */
	uint8_t         lba1;
	uint8_t         lba2;
	uint8_t         lba3;	/* LSB */
	uint8_t         reserved3;
	uint8_t         xfer_length0;	/* transfer length in blocks - MSB */
	uint8_t         xfer_length1;	/* LSB */
	uint8_t         control;
} scsi_rw10_t;

#define SCSI_CDB10_GET_LBA(cdb)                     \
    (((cdb)->lba0 << 24) | ((cdb)->lba1 << 16) |    \
     ((cdb)->lba2 << 8) | (cdb)->lba3)

#define SCSI_CDB10_SET_LBA(cdb, lba) {  \
    (cdb)->lba0 = lba >> 24;            \
    (cdb)->lba1 = (lba >> 16) & 0xFF;   \
    (cdb)->lba2 = (lba >> 8) & 0xFF;    \
    (cdb)->lba3 = lba & 0xFF;           \
}

#define SCSI_CDB10_GET_TL(cdb)  \
    ((cdb)->xfer_length0 << 8 | (cdb)->xfer_length1)
#define SCSI_CDB10_SET_TL(cdb, tl) {    \
    (cdb)->xfer_length0 = tl >> 8;       \
    (cdb)->xfer_length1 = tl & 0xFF;     \
}

/*
 * SCSI READ6/WRITE6 commands
 */
typedef struct {
	uint8_t         opcode;
#ifdef __BIGENDIAN
	uint8_t         lun:3;
	uint8_t         lba0:5;		/* MSb */
#else
	uint8_t         lba0:5;		/* MSb */
	uint8_t         lun:3;
#endif
	uint8_t         lba1;
	uint8_t         lba2;		/* LSB */
	uint8_t         xfer_length;
	uint8_t         control;
} scsi_rw6_t;

#define SCSI_TAPE_CDB6_GET_TL(cdb)              \
    (((cdb)->tl0 << 16) | ((cdb)->tl1 << 8) | (cdb)->tl2)

#define SCSI_TAPE_CDB6_SET_TL(cdb, tl) {   \
    (cdb)->tl0 = tl >> 16;            \
    (cdb)->tl1 = (tl >> 8) & 0xFF;    \
    (cdb)->tl2 = tl & 0xFF;           \
}

/*
 * SCSI sequential (TAPE) wrtie command
 */
typedef struct {
	uint8_t         opcode;
#ifdef __BIGENDIAN
	uint8_t         rsvd:7;
	uint8_t         fixed:1;	/* MSb */
#else
	uint8_t         fixed:1;	/* MSb */
	uint8_t         rsvd:7;
#endif
	uint8_t         tl0;		/* Msb */
	uint8_t         tl1;
	uint8_t         tl2;		/* Lsb */

	uint8_t         control;
} scsi_tape_wr_t;

#define SCSI_CDB6_GET_LBA(cdb)              \
    (((cdb)->lba0 << 16) | ((cdb)->lba1 << 8) | (cdb)->lba2)

#define SCSI_CDB6_SET_LBA(cdb, lba) {   \
    (cdb)->lba0 = lba >> 16;            \
    (cdb)->lba1 = (lba >> 8) & 0xFF;    \
    (cdb)->lba2 = lba & 0xFF;           \
}

#define SCSI_CDB6_GET_TL(cdb) ((cdb)->xfer_length)
#define SCSI_CDB6_SET_TL(cdb, tl) { \
    (cdb)->xfer_length = tl;         \
}

/*
 * SCSI sense data format
 */
typedef struct {
#ifdef __BIGENDIAN
	uint8_t         valid:1;
	uint8_t         rsp_code:7;
#else
	uint8_t         rsp_code:7;
	uint8_t         valid:1;
#endif
	uint8_t         seg_num;
#ifdef __BIGENDIAN
	uint8_t         file_mark:1;
	uint8_t         eom:1;		/* end of media */
	uint8_t         ili:1;		/* incorrect length indicator */
	uint8_t         reserved:1;
	uint8_t         sense_key:4;
#else
	uint8_t         sense_key:4;
	uint8_t         reserved:1;
	uint8_t         ili:1;		/* incorrect length indicator */
	uint8_t         eom:1;		/* end of media */
	uint8_t         file_mark:1;
#endif
	uint8_t         information[4];	/* device-type or command specific info
					 */
	uint8_t         add_sense_length;
					/* additional sense length */
	uint8_t         command_info[4];/* command specific information
						 */
	uint8_t         asc;		/* additional sense code */
	uint8_t         ascq;		/* additional sense code qualifier */
	uint8_t         fru_code;	/* field replaceable unit code */
#ifdef __BIGENDIAN
	uint8_t         sksv:1;		/* sense key specific valid */
	uint8_t         c_d:1;		/* command/data bit */
	uint8_t         res1:2;
	uint8_t         bpv:1;		/* bit pointer valid */
	uint8_t         bpointer:3;	/* bit pointer */
#else
	uint8_t         bpointer:3;	/* bit pointer */
	uint8_t         bpv:1;		/* bit pointer valid */
	uint8_t         res1:2;
	uint8_t         c_d:1;		/* command/data bit */
	uint8_t         sksv:1;		/* sense key specific valid */
#endif
	uint8_t         fpointer[2];	/* field pointer */
} scsi_sense_t;

#define SCSI_SENSE_CUR_ERR          0x70
#define SCSI_SENSE_DEF_ERR          0x71

/*
 * SCSI sense key values
 */
#define SCSI_SK_NO_SENSE        0x0
#define SCSI_SK_REC_ERR         0x1	/* recovered error */
#define SCSI_SK_NOT_READY       0x2
#define SCSI_SK_MED_ERR         0x3	/* medium error */
#define SCSI_SK_HW_ERR          0x4	/* hardware error */
#define SCSI_SK_ILLEGAL_REQ     0x5
#define SCSI_SK_UNIT_ATT        0x6	/* unit attention */
#define SCSI_SK_DATA_PROTECT    0x7
#define SCSI_SK_BLANK_CHECK     0x8
#define SCSI_SK_VENDOR_SPEC     0x9
#define SCSI_SK_COPY_ABORTED    0xA
#define SCSI_SK_ABORTED_CMND    0xB
#define SCSI_SK_VOL_OVERFLOW    0xD
#define SCSI_SK_MISCOMPARE      0xE

/*
 * SCSI additional sense codes
 */
#define SCSI_ASC_NO_ADD_SENSE           0x00
#define SCSI_ASC_LUN_NOT_READY          0x04
#define SCSI_ASC_LUN_COMMUNICATION      0x08
#define SCSI_ASC_WRITE_ERROR            0x0C
#define SCSI_ASC_INVALID_CMND_CODE      0x20
#define SCSI_ASC_BAD_LBA                0x21
#define SCSI_ASC_INVALID_FIELD_IN_CDB   0x24
#define SCSI_ASC_LUN_NOT_SUPPORTED      0x25
#define SCSI_ASC_LUN_WRITE_PROTECT      0x27
#define SCSI_ASC_POWERON_BDR            0x29	/* power on reset, bus reset,
						 * bus device reset
						 */
#define SCSI_ASC_PARAMS_CHANGED         0x2A
#define SCSI_ASC_CMND_CLEARED_BY_A_I    0x2F
#define SCSI_ASC_SAVING_PARAM_NOTSUPP   0x39
#define SCSI_ASC_TOCC                   0x3F	/* target operating condtions
						 * changed
						 */
#define SCSI_ASC_PARITY_ERROR           0x47
#define SCSI_ASC_CMND_PHASE_ERROR       0x4A
#define SCSI_ASC_DATA_PHASE_ERROR       0x4B
#define SCSI_ASC_VENDOR_SPEC            0x7F

/*
 * SCSI additional sense code qualifiers
 */
#define SCSI_ASCQ_CAUSE_NOT_REPORT      0x00
#define SCSI_ASCQ_BECOMING_READY        0x01
#define SCSI_ASCQ_INIT_CMD_REQ          0x02
#define SCSI_ASCQ_MAN_INTR_REQ		0x03	/* manual intervention req */
#define SCSI_ASCQ_FORMAT_IN_PROGRESS    0x04
#define SCSI_ASCQ_OPERATION_IN_PROGRESS 0x07
#define SCSI_ASCQ_SELF_TEST_IN_PROGRESS 0x09
#define SCSI_ASCQ_WR_UNEXP_UNSOL_DATA   0x0C
#define SCSI_ASCQ_WR_NOTENG_UNSOL_DATA  0x0D

#define SCSI_ASCQ_LBA_OUT_OF_RANGE      0x00
#define SCSI_ASCQ_INVALID_ELEMENT_ADDR  0x01

#define SCSI_ASCQ_LUN_WRITE_PROTECTED       0x00
#define SCSI_ASCQ_LUN_HW_WRITE_PROTECTED    0x01
#define SCSI_ASCQ_LUN_SW_WRITE_PROTECTED    0x02

#define SCSI_ASCQ_POR   0x01	/* power on reset */
#define SCSI_ASCQ_SBR   0x02	/* scsi bus reset */
#define SCSI_ASCQ_BDR   0x03	/* bus device reset */
#define SCSI_ASCQ_DIR   0x04	/* device internal reset */

#define SCSI_ASCQ_MODE_PARAMS_CHANGED       0x01
#define SCSI_ASCQ_LOG_PARAMS_CHANGED        0x02
#define SCSI_ASCQ_RESERVATIONS_PREEMPTED    0x03
#define SCSI_ASCQ_RESERVATIONS_RELEASED     0x04
#define SCSI_ASCQ_REGISTRATIONS_PREEMPTED   0x05

#define SCSI_ASCQ_MICROCODE_CHANGED 0x01
#define SCSI_ASCQ_CHANGED_OPER_COND 0x02
#define SCSI_ASCQ_INQ_CHANGED       0x03	/* inquiry data changed */
#define SCSI_ASCQ_DI_CHANGED        0x05	/* device id changed */
#define SCSI_ASCQ_RL_DATA_CHANGED   0x0E	/* report luns data changed */

#define SCSI_ASCQ_DP_CRC_ERR            0x01	/* data phase crc error */
#define SCSI_ASCQ_DP_SCSI_PARITY_ERR    0x02	/* data phase scsi parity error
						 */
#define SCSI_ASCQ_IU_CRC_ERR            0x03	/* information unit crc error */
#define SCSI_ASCQ_PROTO_SERV_CRC_ERR    0x05

#define SCSI_ASCQ_LUN_TIME_OUT          0x01

/* ------------------------------------------------------------
 * SCSI INQUIRY
 * ------------------------------------------------------------*/

typedef struct {
	uint8_t         opcode;
#ifdef __BIGENDIAN
	uint8_t         lun:3;
	uint8_t         reserved1:3;
	uint8_t         cmd_dt:1;
	uint8_t         evpd:1;
#else
	uint8_t         evpd:1;
	uint8_t         cmd_dt:1;
	uint8_t         reserved1:3;
	uint8_t         lun:3;
#endif
	uint8_t         page_code;
	uint8_t         reserved2;
	uint8_t         alloc_length;
	uint8_t         control;
} scsi_inquiry_t;

typedef struct {
	uint8_t         vendor_id[8];
} scsi_inquiry_vendor_t;

typedef struct {
	uint8_t         product_id[16];
} scsi_inquiry_prodid_t;

typedef struct {
	uint8_t         product_rev[4];
} scsi_inquiry_prodrev_t;

typedef struct {
#ifdef __BIGENDIAN
	uint8_t         peripheral_qual:3;	/* peripheral qualifier */
	uint8_t         device_type:5;		/* peripheral device type */

	uint8_t         rmb:1;			/* removable medium bit */
	uint8_t         device_type_mod:7;	/* device type modifier */

	uint8_t         version;

	uint8_t         aenc:1;		/* async event notification capability
					 */
	uint8_t         trm_iop:1;	/* terminate I/O process */
	uint8_t         norm_aca:1;	/* normal ACA supported */
	uint8_t         hi_support:1;	/* SCSI-3: supports REPORT LUNS */
	uint8_t         rsp_data_format:4;

	uint8_t         additional_len;
	uint8_t         sccs:1;
	uint8_t         reserved1:7;

	uint8_t         reserved2:1;
	uint8_t         enc_serv:1;	/* enclosure service component */
	uint8_t         reserved3:1;
	uint8_t         multi_port:1;	/* multi-port device */
	uint8_t         m_chngr:1;	/* device in medium transport element */
	uint8_t         ack_req_q:1;	/* SIP specific bit */
	uint8_t         addr32:1;	/* SIP specific bit */
	uint8_t         addr16:1;	/* SIP specific bit */

	uint8_t         rel_adr:1;	/* relative address */
	uint8_t         w_bus32:1;
	uint8_t         w_bus16:1;
	uint8_t         synchronous:1;
	uint8_t         linked_commands:1;
	uint8_t         trans_dis:1;
	uint8_t         cmd_queue:1;	/* command queueing supported */
	uint8_t         soft_reset:1;	/* soft reset alternative (VS) */
#else
	uint8_t         device_type:5;	/* peripheral device type */
	uint8_t         peripheral_qual:3;
					/* peripheral qualifier */

	uint8_t         device_type_mod:7;
					/* device type modifier */
	uint8_t         rmb:1;		/* removable medium bit */

	uint8_t         version;

	uint8_t         rsp_data_format:4;
	uint8_t         hi_support:1;	/* SCSI-3: supports REPORT LUNS */
	uint8_t         norm_aca:1;	/* normal ACA supported */
	uint8_t         terminate_iop:1;/* terminate I/O process */
	uint8_t         aenc:1;		/* async event notification capability
					 */

	uint8_t         additional_len;
	uint8_t         reserved1:7;
	uint8_t         sccs:1;

	uint8_t         addr16:1;	/* SIP specific bit */
	uint8_t         addr32:1;	/* SIP specific bit */
	uint8_t         ack_req_q:1;	/* SIP specific bit */
	uint8_t         m_chngr:1;	/* device in medium transport element */
	uint8_t         multi_port:1;	/* multi-port device */
	uint8_t         reserved3:1;	/* TBD - Vendor Specific */
	uint8_t         enc_serv:1;	/* enclosure service component */
	uint8_t         reserved2:1;

	uint8_t         soft_seset:1;	/* soft reset alternative (VS) */
	uint8_t         cmd_queue:1;	/* command queueing supported */
	uint8_t         trans_dis:1;
	uint8_t         linked_commands:1;
	uint8_t         synchronous:1;
	uint8_t         w_bus16:1;
	uint8_t         w_bus32:1;
	uint8_t         rel_adr:1;	/* relative address */
#endif
	scsi_inquiry_vendor_t vendor_id;
	scsi_inquiry_prodid_t product_id;
	scsi_inquiry_prodrev_t product_rev;
	uint8_t         vendor_specific[20];
	uint8_t         reserved4[40];
} scsi_inquiry_data_t;

/*
 * inquiry.peripheral_qual field values
 */
#define SCSI_DEVQUAL_DEFAULT        0
#define SCSI_DEVQUAL_NOT_CONNECTED  1
#define SCSI_DEVQUAL_NOT_SUPPORTED  3

/*
 * inquiry.device_type field values
 */
#define SCSI_DEVICE_DIRECT_ACCESS       0x00
#define SCSI_DEVICE_SEQ_ACCESS          0x01
#define SCSI_DEVICE_ARRAY_CONTROLLER    0x0C
#define SCSI_DEVICE_UNKNOWN             0x1F

/*
 * inquiry.version
 */
#define SCSI_VERSION_ANSI_X3131     2	/* ANSI X3.131 SCSI-2 */
#define SCSI_VERSION_SPC            3	/* SPC (SCSI-3), ANSI X3.301:1997 */
#define SCSI_VERSION_SPC_2          4	/* SPC-2 */

/*
 * response data format
 */
#define SCSI_RSP_DATA_FORMAT        2	/* SCSI-2 & SPC */

/*
 * SCSI inquiry page codes
 */
#define SCSI_INQ_PAGE_VPD_PAGES     0x00	/* supported vpd pages */
#define SCSI_INQ_PAGE_USN_PAGE      0x80	/* unit serial number page */
#define SCSI_INQ_PAGE_DEV_IDENT     0x83	/* device indentification page
						 */
#define SCSI_INQ_PAGES_MAX          3

/*
 * supported vital product data pages
 */
typedef struct {
#ifdef __BIGENDIAN
	uint8_t         peripheral_qual:3;
	uint8_t         device_type:5;
#else
	uint8_t         device_type:5;
	uint8_t         peripheral_qual:3;
#endif
	uint8_t         page_code;
	uint8_t         reserved;
	uint8_t         page_length;
	uint8_t         pages[SCSI_INQ_PAGES_MAX];
} scsi_inq_page_vpd_pages_t;

/*
 * Unit serial number page
 */
#define SCSI_INQ_USN_LEN 32

typedef struct {
	char            usn[SCSI_INQ_USN_LEN];
} scsi_inq_usn_t;

typedef struct {
#ifdef __BIGENDIAN
	uint8_t         peripheral_qual:3;
	uint8_t         device_type:5;
#else
	uint8_t         device_type:5;
	uint8_t         peripheral_qual:3;
#endif
	uint8_t         page_code;
	uint8_t         reserved1;
	uint8_t         page_length;
	scsi_inq_usn_t  usn;
} scsi_inq_page_usn_t;

enum {
	SCSI_INQ_DIP_CODE_BINARY = 1,	/* identifier has binary value */
	SCSI_INQ_DIP_CODE_ASCII = 2,	/* identifier has ascii value */
};

enum {
	SCSI_INQ_DIP_ASSOC_LUN = 0,	/* id is associated with device */
	SCSI_INQ_DIP_ASSOC_PORT = 1,	/* id is associated with port that
					 * received the request
					 */
};

enum {
	SCSI_INQ_ID_TYPE_VENDOR = 1,
	SCSI_INQ_ID_TYPE_IEEE = 2,
	SCSI_INQ_ID_TYPE_FC_FS = 3,
	SCSI_INQ_ID_TYPE_OTHER = 4,
};

typedef struct {
#ifdef __BIGENDIAN
	uint8_t         res0:4;
	uint8_t         code_set:4;
	uint8_t         res1:2;
	uint8_t         association:2;
	uint8_t         id_type:4;
#else
	uint8_t         code_set:4;
	uint8_t         res0:4;
	uint8_t         id_type:4;
	uint8_t         association:2;
	uint8_t         res1:2;
#endif
	uint8_t         res2;
	uint8_t         id_len;
	scsi_lun_sn_t   id;
} scsi_inq_dip_desc_t;

/*
 * Device indentification page
 */
typedef struct {
#ifdef __BIGENDIAN
	uint8_t         peripheral_qual:3;
	uint8_t         device_type:5;
#else
	uint8_t         device_type:5;
	uint8_t         peripheral_qual:3;
#endif
	uint8_t         page_code;
	uint8_t         reserved1;
	uint8_t         page_length;
	scsi_inq_dip_desc_t desc;
} scsi_inq_page_dev_ident_t;

/* ------------------------------------------------------------
 * READ CAPACITY
 * ------------------------------------------------------------
 */

typedef struct {
	uint8_t         opcode;
#ifdef __BIGENDIAN
	uint8_t         lun:3;
	uint8_t         reserved1:4;
	uint8_t         rel_adr:1;
#else
	uint8_t         rel_adr:1;
	uint8_t         reserved1:4;
	uint8_t         lun:3;
#endif
	uint8_t         lba0;	/* MSB */
	uint8_t         lba1;
	uint8_t         lba2;
	uint8_t         lba3;	/* LSB */
	uint8_t         reserved2;
	uint8_t         reserved3;
#ifdef __BIGENDIAN
	uint8_t         reserved4:7;
	uint8_t         pmi:1;	/* partial medium indicator */
#else
	uint8_t         pmi:1;	/* partial medium indicator */
	uint8_t         reserved4:7;
#endif
	uint8_t         control;
} scsi_read_capacity_t;

typedef struct {
	uint32_t        max_lba;	/* maximum LBA available */
	uint32_t        block_length;	/* in bytes */
} scsi_read_capacity_data_t;

typedef struct {
	uint8_t         opcode;
	uint8_t         service_action;
	uint64_t	lba;
	uint32_t	len;		/* Allocation length */
#ifdef __BIGENDIAN
	uint8_t         reserved1:7;
	uint8_t         pmi:1;	/* partial medium indicator */
#else
	uint8_t         pmi:1;	/* partial medium indicator */
	uint8_t         reserved1:7;
#endif
	uint8_t         control;
} scsi_read_capacity16_t;

typedef struct {
	uint64_t        lba;	/* maximum LBA available */
	uint32_t        block_length;	/* in bytes */
#ifdef __BIGENDIAN
	uint8_t         reserved1:4,
			p_type:3,
			prot_en:1;
	uint8_t		reserved2:4,
			lb_pbe:4;	/* logical blocks per physical block exponent */
	uint16_t	reserved3:2,
			lba_align:14;	/* lowest aligned logical block address */
#else
	uint16_t	lba_align:14,	/* lowest aligned logical block address */
			reserved3:2;
	uint8_t		lb_pbe:4,	/* logical blocks per physical block exponent */
			reserved2:4;
	uint8_t		prot_en:1,
			p_type:3,
			reserved1:4;
#endif
	uint64_t	reserved4;
	uint64_t	reserved5;
} scsi_read_capacity16_data_t;

/* ------------------------------------------------------------
 * REPORT LUNS command
 * ------------------------------------------------------------
 */

typedef struct {
	uint8_t         opcode;		/* A0h - REPORT LUNS opCode */
	uint8_t         reserved1[5];
	uint8_t         alloc_length[4];/* allocation length MSB first */
	uint8_t         reserved2;
	uint8_t         control;
} scsi_report_luns_t;

#define SCSI_REPORT_LUN_ALLOC_LENGTH(rl)                		\
    ((rl->alloc_length[0] << 24) | (rl->alloc_length[1] << 16) | 	\
     (rl->alloc_length[2] << 8) | (rl->alloc_length[3]))

#define SCSI_REPORT_LUNS_SET_ALLOCLEN(rl, alloc_len) {   		\
    (rl)->alloc_length[0] = (alloc_len) >> 24;      			\
    (rl)->alloc_length[1] = ((alloc_len) >> 16) & 0xFF; 		\
    (rl)->alloc_length[2] = ((alloc_len) >> 8) & 0xFF;  		\
    (rl)->alloc_length[3] = (alloc_len) & 0xFF;     			\
}

typedef struct {
	uint32_t        lun_list_length;	/* length of LUN list length */
	uint32_t        reserved;
	lun_t		lun[1];			/* first LUN in lun list */
} scsi_report_luns_data_t;

/* -------------------------------------------------------------
 * SCSI mode  parameters
 * -----------------------------------------------------------
 */
enum {
	SCSI_DA_MEDIUM_DEF = 0,	/* direct access default medium type */
	SCSI_DA_MEDIUM_SS = 1,	/* direct access single sided */
	SCSI_DA_MEDIUM_DS = 2,	/* direct access double sided */
};

/*
 * SCSI Mode Select(6) cdb
 */
typedef struct {
	uint8_t         opcode;
#ifdef __BIGENDIAN
	uint8_t         reserved1:3;
	uint8_t         pf:1;		/* page format */
	uint8_t         reserved2:3;
	uint8_t         sp:1;		/* save pages if set to 1 */
#else
	uint8_t         sp:1;	/* save pages if set to 1 */
	uint8_t         reserved2:3;
	uint8_t         pf:1;	/* page format */
	uint8_t         reserved1:3;
#endif
	uint8_t         reserved3[2];
	uint8_t         alloc_len;
	uint8_t         control;
} scsi_mode_select6_t;

/*
 * SCSI Mode Select(10) cdb
 */
typedef struct {
	uint8_t         opcode;
#ifdef __BIGENDIAN
	uint8_t         reserved1:3;
	uint8_t         pf:1;	/* page format */
	uint8_t         reserved2:3;
	uint8_t         sp:1;	/* save pages if set to 1 */
#else
	uint8_t         sp:1;	/* save pages if set to 1 */
	uint8_t         reserved2:3;
	uint8_t         pf:1;	/* page format */
	uint8_t         reserved1:3;
#endif
	uint8_t         reserved3[5];
	uint8_t         alloc_len_msb;
	uint8_t         alloc_len_lsb;
	uint8_t         control;
} scsi_mode_select10_t;

/*
 * SCSI Mode Sense(6) cdb
 */
typedef struct {
	uint8_t         opcode;
#ifdef __BIGENDIAN
	uint8_t         reserved1:4;
	uint8_t         dbd:1;	/* disable block discriptors if set to 1 */
	uint8_t         reserved2:3;

	uint8_t         pc:2;	/* page control */
	uint8_t         page_code:6;
#else
	uint8_t         reserved2:3;
	uint8_t         dbd:1;	/* disable block descriptors if set to 1 */
	uint8_t         reserved1:4;

	uint8_t         page_code:6;
	uint8_t         pc:2;	/* page control */
#endif
	uint8_t         reserved3;
	uint8_t         alloc_len;
	uint8_t         control;
} scsi_mode_sense6_t;

/*
 * SCSI Mode Sense(10) cdb
 */
typedef struct {
	uint8_t         opcode;
#ifdef __BIGENDIAN
	uint8_t         reserved1:3;
	uint8_t         LLBAA:1;	/* long LBA accepted if set to 1 */
	uint8_t         dbd:1;		/* disable block descriptors if set
					 * to 1
					 */
	uint8_t         reserved2:3;

	uint8_t         pc:2;		/* page control */
	uint8_t         page_code:6;
#else
	uint8_t         reserved2:3;
	uint8_t         dbd:1;		/* disable block descriptors if set to
					 * 1
					 */
	uint8_t         LLBAA:1;	/* long LBA accepted if set to 1 */
	uint8_t         reserved1:3;

	uint8_t         page_code:6;
	uint8_t         pc:2;		/* page control */
#endif
	uint8_t         reserved3[4];
	uint8_t         alloc_len_msb;
	uint8_t         alloc_len_lsb;
	uint8_t         control;
} scsi_mode_sense10_t;

#define SCSI_CDB10_GET_AL(cdb)  					\
    ((cdb)->alloc_len_msb << 8 | (cdb)->alloc_len_lsb)

#define SCSI_CDB10_SET_AL(cdb, al) {    				\
    (cdb)->alloc_len_msb = al >> 8;       				\
    (cdb)->alloc_len_lsb = al & 0xFF;     				\
}

#define SCSI_CDB6_GET_AL(cdb) ((cdb)->alloc_len)

#define SCSI_CDB6_SET_AL(cdb, al) { 					\
    (cdb)->alloc_len = al;         					\
}

/*
 * page control field values
 */
#define SCSI_PC_CURRENT_VALUES       0x0
#define SCSI_PC_CHANGEABLE_VALUES    0x1
#define SCSI_PC_DEFAULT_VALUES       0x2
#define SCSI_PC_SAVED_VALUES         0x3

/*
 * SCSI mode page codes
 */
#define SCSI_MP_VENDOR_SPEC     0x00
#define SCSI_MP_DISC_RECN       0x02	/* disconnect-reconnect page */
#define SCSI_MP_FORMAT_DEVICE   0x03
#define SCSI_MP_RDG             0x04	/* rigid disk geometry page */
#define SCSI_MP_FDP             0x05	/* flexible disk page */
#define SCSI_MP_CACHING         0x08	/* caching page */
#define SCSI_MP_CONTROL         0x0A	/* control mode page */
#define SCSI_MP_MED_TYPES_SUP   0x0B	/* medium types supported page */
#define SCSI_MP_INFO_EXCP_CNTL  0x1C	/* informational exception control */
#define SCSI_MP_ALL             0x3F	/* return all pages - mode sense only */

/*
 * mode parameter header
 */
typedef struct {
	uint8_t         mode_datalen;
	uint8_t         medium_type;

	/*
	 * device specific parameters expanded for direct access devices
	 */
#ifdef __BIGENDIAN
	uint32_t        wp:1;		/* write protected */
	uint32_t        reserved1:2;
	uint32_t        dpofua:1;	/* disable page out + force unit access
					 */
	uint32_t        reserved2:4;
#else
	uint32_t        reserved2:4;
	uint32_t        dpofua:1;	/* disable page out + force unit access
					 */
	uint32_t        reserved1:2;
	uint32_t        wp:1;		/* write protected */
#endif

	uint8_t         block_desclen;
} scsi_mode_param_header6_t;

typedef struct {
uint32_t        mode_datalen:16;
uint32_t        medium_type:8;

	/*
	 * device specific parameters expanded for direct access devices
	 */
#ifdef __BIGENDIAN
	uint32_t        wp:1;		/* write protected */
	uint32_t        reserved1:2;
	uint32_t        dpofua:1;	/* disable page out + force unit access
					 */
	uint32_t        reserved2:4;
#else
	uint32_t        reserved2:4;
	uint32_t        dpofua:1;	/* disable page out + force unit access
					 */
	uint32_t        reserved1:2;
	uint32_t        wp:1;		/* write protected */
#endif

#ifdef __BIGENDIAN
	uint32_t        reserved3:7;
	uint32_t        longlba:1;
#else
	uint32_t        longlba:1;
	uint32_t        reserved3:7;
#endif
	uint32_t        reserved4:8;
	uint32_t        block_desclen:16;
} scsi_mode_param_header10_t;

/*
 * mode parameter block descriptor
 */
typedef struct {
	uint32_t        nblks;
	uint32_t        density_code:8;
	uint32_t        block_length:24;
} scsi_mode_param_desc_t;

/*
 * Disconnect-reconnect mode page format
 */
typedef struct {
#ifdef __BIGENDIAN
	uint8_t         ps:1;
	uint8_t         reserved1:1;
	uint8_t         page_code:6;
#else
	uint8_t         page_code:6;
	uint8_t         reserved1:1;
	uint8_t         ps:1;
#endif
	uint8_t         page_len;
	uint8_t         buf_full_ratio;
	uint8_t         buf_empty_ratio;

	uint8_t         bil_msb;	/* bus inactivity limit -MSB */
	uint8_t         bil_lsb;	/* bus inactivity limit -LSB */

	uint8_t         dtl_msb;	/* disconnect time limit - MSB */
	uint8_t         dtl_lsb;	/* disconnect time limit - LSB */

	uint8_t         ctl_msb;	/* connect time limit - MSB */
	uint8_t         ctl_lsb;	/* connect time limit - LSB */

	uint8_t         max_burst_len_msb;
	uint8_t         max_burst_len_lsb;
#ifdef __BIGENDIAN
	uint8_t         emdp:1;	/* enable modify data pointers */
	uint8_t         fa:3;	/* fair arbitration */
	uint8_t         dimm:1;	/* disconnect immediate */
	uint8_t         dtdc:3;	/* data transfer disconnect control */
#else
	uint8_t         dtdc:3;	/* data transfer disconnect control */
	uint8_t         dimm:1;	/* disconnect immediate */
	uint8_t         fa:3;	/* fair arbitration */
	uint8_t         emdp:1;	/* enable modify data pointers */
#endif

	uint8_t         reserved3;

	uint8_t         first_burst_len_msb;
	uint8_t         first_burst_len_lsb;
} scsi_mp_disc_recn_t;

/*
 * SCSI format device mode page
 */
typedef struct {
#ifdef __BIGENDIAN
	uint32_t        ps:1;
	uint32_t        reserved1:1;
	uint32_t        page_code:6;
#else
	uint32_t        page_code:6;
	uint32_t        reserved1:1;
	uint32_t        ps:1;
#endif
	uint32_t        page_len:8;
	uint32_t        tracks_per_zone:16;

	uint32_t        a_sec_per_zone:16;
	uint32_t        a_tracks_per_zone:16;

	uint32_t        a_tracks_per_lun:16;	/* alternate tracks/lun-MSB */
	uint32_t        sec_per_track:16;	/* sectors/track-MSB */

	uint32_t        bytes_per_sector:16;
	uint32_t        interleave:16;

	uint32_t        tsf:16;			/* track skew factor-MSB */
	uint32_t        csf:16;			/* cylinder skew factor-MSB */

#ifdef __BIGENDIAN
	uint32_t        ssec:1;	/* soft sector formatting */
	uint32_t        hsec:1;	/* hard sector formatting */
	uint32_t        rmb:1;	/* removable media */
	uint32_t        surf:1;	/* surface */
	uint32_t        reserved2:4;
#else
	uint32_t        reserved2:4;
	uint32_t        surf:1;	/* surface */
	uint32_t        rmb:1;	/* removable media */
	uint32_t        hsec:1;	/* hard sector formatting */
	uint32_t        ssec:1;	/* soft sector formatting */
#endif
	uint32_t        reserved3:24;
} scsi_mp_format_device_t;

/*
 * SCSI rigid disk device geometry page
 */
typedef struct {
#ifdef __BIGENDIAN
	uint32_t        ps:1;
	uint32_t        reserved1:1;
	uint32_t        page_code:6;
#else
	uint32_t        page_code:6;
	uint32_t        reserved1:1;
	uint32_t        ps:1;
#endif
	uint32_t        page_len:8;
	uint32_t        num_cylinders0:8;
	uint32_t        num_cylinders1:8;

	uint32_t        num_cylinders2:8;
	uint32_t        num_heads:8;
	uint32_t        scwp0:8;
	uint32_t        scwp1:8;

	uint32_t        scwp2:8;
	uint32_t        scrwc0:8;
	uint32_t        scrwc1:8;
	uint32_t        scrwc2:8;

	uint32_t        dsr:16;
	uint32_t        lscyl0:8;
	uint32_t        lscyl1:8;

	uint32_t        lscyl2:8;
#ifdef __BIGENDIAN
	uint32_t        reserved2:6;
	uint32_t        rpl:2;	/* rotational position locking */
#else
	uint32_t        rpl:2;	/* rotational position locking */
	uint32_t        reserved2:6;
#endif
	uint32_t        rot_off:8;
	uint32_t        reserved3:8;

	uint32_t        med_rot_rate:16;
	uint32_t        reserved4:16;
} scsi_mp_rigid_device_geometry_t;

/*
 * SCSI caching mode page
 */
typedef struct {
#ifdef __BIGENDIAN
	uint8_t         ps:1;
	uint8_t         res1:1;
	uint8_t         page_code:6;
#else
	uint8_t         page_code:6;
	uint8_t         res1:1;
	uint8_t         ps:1;
#endif
	uint8_t         page_len;
#ifdef __BIGENDIAN
	uint8_t         ic:1;	/* initiator control */
	uint8_t         abpf:1;	/* abort pre-fetch */
	uint8_t         cap:1;	/* caching analysis permitted */
	uint8_t         disc:1;	/* discontinuity */
	uint8_t         size:1;	/* size enable */
	uint8_t         wce:1;	/* write cache enable */
	uint8_t         mf:1;	/* multiplication factor */
	uint8_t         rcd:1;	/* read cache disable */

	uint8_t         drrp:4;	/* demand read retention priority */
	uint8_t         wrp:4;	/* write retention priority */
#else
	uint8_t         rcd:1;	/* read cache disable */
	uint8_t         mf:1;	/* multiplication factor */
	uint8_t         wce:1;	/* write cache enable */
	uint8_t         size:1;	/* size enable */
	uint8_t         disc:1;	/* discontinuity */
	uint8_t         cap:1;	/* caching analysis permitted */
	uint8_t         abpf:1;	/* abort pre-fetch */
	uint8_t         ic:1;	/* initiator control */

	uint8_t         wrp:4;	/* write retention priority */
	uint8_t         drrp:4;	/* demand read retention priority */
#endif
	uint8_t         dptl[2];/* disable pre-fetch transfer length */
	uint8_t         min_prefetch[2];
	uint8_t         max_prefetch[2];
	uint8_t         max_prefetch_limit[2];
#ifdef __BIGENDIAN
	uint8_t         fsw:1;	/* force sequential write */
	uint8_t         lbcss:1;/* logical block cache segment size */
	uint8_t         dra:1;	/* disable read ahead */
	uint8_t         vs:2;	/* vendor specific */
	uint8_t         res2:3;
#else
	uint8_t         res2:3;
	uint8_t         vs:2;	/* vendor specific */
	uint8_t         dra:1;	/* disable read ahead */
	uint8_t         lbcss:1;/* logical block cache segment size */
	uint8_t         fsw:1;	/* force sequential write */
#endif
	uint8_t         num_cache_segs;

	uint8_t         cache_seg_size[2];
	uint8_t         res3;
	uint8_t         non_cache_seg_size[3];
} scsi_mp_caching_t;

/*
 * SCSI control mode page
 */
typedef struct {
#ifdef __BIGENDIAN
uint8_t         ps:1;
uint8_t         reserved1:1;
uint8_t         page_code:6;
#else
uint8_t         page_code:6;
uint8_t         reserved1:1;
uint8_t         ps:1;
#endif
	uint8_t         page_len;
#ifdef __BIGENDIAN
	uint8_t         tst:3;		/* task set type */
	uint8_t         reserved3:3;
	uint8_t         gltsd:1;	/* global logging target save disable */
	uint8_t         rlec:1;		/* report log exception condition */

	uint8_t         qalgo_mod:4;	/* queue alogorithm modifier */
	uint8_t         reserved4:1;
	uint8_t         qerr:2;		/* queue error management */
	uint8_t         dque:1;		/* disable queuing */

	uint8_t         reserved5:1;
	uint8_t         rac:1;		/* report a check */
	uint8_t         reserved6:2;
	uint8_t         swp:1;		/* software write protect */
	uint8_t         raerp:1;	/* ready AER permission */
	uint8_t         uaaerp:1;	/* unit attenstion AER permission */
	uint8_t         eaerp:1;	/* error AER permission */

	uint8_t         reserved7:5;
	uint8_t         autoload_mod:3;
#else
	uint8_t         rlec:1;		/* report log exception condition */
	uint8_t         gltsd:1;	/* global logging target save disable */
	uint8_t         reserved3:3;
	uint8_t         tst:3;		/* task set type */

	uint8_t         dque:1;		/* disable queuing */
	uint8_t         qerr:2;		/* queue error management */
	uint8_t         reserved4:1;
	uint8_t         qalgo_mod:4;	/* queue alogorithm modifier */

	uint8_t         eaerp:1;	/* error AER permission */
	uint8_t         uaaerp:1;	/* unit attenstion AER permission */
	uint8_t         raerp:1;	/* ready AER permission */
	uint8_t         swp:1;		/* software write protect */
	uint8_t         reserved6:2;
	uint8_t         rac:1;		/* report a check */
	uint8_t         reserved5:1;

	uint8_t         autoload_mod:3;
	uint8_t         reserved7:5;
#endif
	uint8_t         rahp_msb;	/* ready AER holdoff period - MSB */
	uint8_t         rahp_lsb;	/* ready AER holdoff period - LSB */

	uint8_t         busy_timeout_period_msb;
	uint8_t         busy_timeout_period_lsb;

	uint8_t         ext_selftest_compl_time_msb;
	uint8_t         ext_selftest_compl_time_lsb;
} scsi_mp_control_page_t;

/*
 * SCSI medium types supported mode page
 */
typedef struct {
#ifdef __BIGENDIAN
	uint8_t         ps:1;
	uint8_t         reserved1:1;
	uint8_t         page_code:6;
#else
	uint8_t         page_code:6;
	uint8_t         reserved1:1;
	uint8_t         ps:1;
#endif
	uint8_t         page_len;

	uint8_t         reserved3[2];
	uint8_t         med_type1_sup;	/* medium type one supported */
	uint8_t         med_type2_sup;	/* medium type two supported */
	uint8_t         med_type3_sup;	/* medium type three supported */
	uint8_t         med_type4_sup;	/* medium type four supported */
} scsi_mp_medium_types_sup_t;

/*
 * SCSI informational exception control mode page
 */
typedef struct {
#ifdef __BIGENDIAN
	uint8_t         ps:1;
	uint8_t         reserved1:1;
	uint8_t         page_code:6;
#else
	uint8_t         page_code:6;
	uint8_t         reserved1:1;
	uint8_t         ps:1;
#endif
	uint8_t         page_len;
#ifdef __BIGENDIAN
	uint8_t         perf:1;		/* performance */
	uint8_t         reserved3:1;
	uint8_t         ebf:1;		/* enable background fucntion */
	uint8_t         ewasc:1;	/* enable warning */
	uint8_t         dexcpt:1;	/* disable exception control */
	uint8_t         test:1;		/* enable test device failure
					 * notification
					 */
	uint8_t         reserved4:1;
	uint8_t         log_error:1;

	uint8_t         reserved5:4;
	uint8_t         mrie:4;		/* method of reporting info
					 * exceptions
					 */
#else
	uint8_t         log_error:1;
	uint8_t         reserved4:1;
	uint8_t         test:1;		/* enable test device failure
					 * notification
					 */
	uint8_t         dexcpt:1;	/* disable exception control */
	uint8_t         ewasc:1;	/* enable warning */
	uint8_t         ebf:1;		/* enable background fucntion */
	uint8_t         reserved3:1;
	uint8_t         perf:1;		/* performance */

	uint8_t         mrie:4;		/* method of reporting info
					 * exceptions
					 */
	uint8_t         reserved5:4;
#endif
	uint8_t         interval_timer_msb;
	uint8_t         interval_timer_lsb;

	uint8_t         report_count_msb;
	uint8_t         report_count_lsb;
} scsi_mp_info_excpt_cntl_t;

/*
 * Methods of reporting informational exceptions
 */
#define SCSI_MP_IEC_NO_REPORT       0x0	/* no reporting of exceptions */
#define SCSI_MP_IEC_AER             0x1	/* async event reporting */
#define SCSI_MP_IEC_UNIT_ATTN       0x2	/* generate unit attenstion */
#define SCSI_MO_IEC_COND_REC_ERR    0x3	/* conditionally generate recovered
					 * error
					 */
#define SCSI_MP_IEC_UNCOND_REC_ERR  0x4	/* unconditionally generate recovered
					 * error
					 */
#define SCSI_MP_IEC_NO_SENSE        0x5	/* generate no sense */
#define SCSI_MP_IEC_ON_REQUEST      0x6	/* only report exceptions on request */

/*
 * SCSI flexible disk page
 */
typedef struct {
#ifdef __BIGENDIAN
	uint8_t         ps:1;
	uint8_t         reserved1:1;
	uint8_t         page_code:6;
#else
	uint8_t         page_code:6;
	uint8_t         reserved1:1;
	uint8_t         ps:1;
#endif
	uint8_t         page_len;

	uint8_t         transfer_rate_msb;
	uint8_t         transfer_rate_lsb;

	uint8_t         num_heads;
	uint8_t         num_sectors;

	uint8_t         bytes_per_sector_msb;
	uint8_t         bytes_per_sector_lsb;

	uint8_t         num_cylinders_msb;
	uint8_t         num_cylinders_lsb;

	uint8_t         sc_wpc_msb;	/* starting cylinder-write
					 * precompensation msb
					 */
	uint8_t         sc_wpc_lsb;	/* starting cylinder-write
					 * precompensation lsb
					 */
	uint8_t         sc_rwc_msb;	/* starting cylinder-reduced write
					 * current msb
					 */
	uint8_t         sc_rwc_lsb;	/* starting cylinder-reduced write
					 * current lsb
					 */

	uint8_t         dev_step_rate_msb;
	uint8_t         dev_step_rate_lsb;

	uint8_t         dev_step_pulse_width;

	uint8_t         head_sd_msb;	/* head settle delay msb */
	uint8_t         head_sd_lsb;	/* head settle delay lsb */

	uint8_t         motor_on_delay;
	uint8_t         motor_off_delay;
#ifdef __BIGENDIAN
	uint8_t         trdy:1;		/* true ready bit */
	uint8_t         ssn:1;		/* start sector number bit */
	uint8_t         mo:1;		/* motor on bit */
	uint8_t         reserved3:5;

	uint8_t         reserved4:4;
	uint8_t         spc:4;		/* step pulse per cylinder */
#else
	uint8_t         reserved3:5;
	uint8_t         mo:1;		/* motor on bit */
	uint8_t         ssn:1;		/* start sector number bit */
	uint8_t         trdy:1;		/* true ready bit */

	uint8_t         spc:4;		/* step pulse per cylinder */
	uint8_t         reserved4:4;
#endif
	uint8_t         write_comp;
	uint8_t         head_load_delay;
	uint8_t         head_unload_delay;
#ifdef __BIGENDIAN
	uint8_t         pin34:4;	/* pin34 usage */
	uint8_t         pin2:4;		/* pin2 usage */

	uint8_t         pin4:4;		/* pin4 usage */
	uint8_t         pin1:4;		/* pin1 usage */
#else
	uint8_t         pin2:4;		/* pin2 usage */
	uint8_t         pin34:4;	/* pin34 usage */

	uint8_t         pin1:4;		/* pin1 usage */
	uint8_t         pin4:4;		/* pin4 usage */
#endif
	uint8_t         med_rot_rate_msb;
	uint8_t         med_rot_rate_lsb;

	uint8_t         reserved5[2];
} scsi_mp_flexible_disk_t;

typedef struct {
	scsi_mode_param_header6_t mph;	/* mode page header */
	scsi_mode_param_desc_t desc;	/* block descriptor */
	scsi_mp_format_device_t format;	/* format device data */
} scsi_mode_page_format_data6_t;

typedef struct {
	scsi_mode_param_header10_t mph;	/* mode page header */
	scsi_mode_param_desc_t desc;	/* block descriptor */
	scsi_mp_format_device_t format;	/* format device data */
} scsi_mode_page_format_data10_t;

typedef struct {
	scsi_mode_param_header6_t mph;	/* mode page header */
	scsi_mode_param_desc_t desc;	/* block descriptor */
	scsi_mp_rigid_device_geometry_t rdg;
					/* rigid geometry data */
} scsi_mode_page_rdg_data6_t;

typedef struct {
	scsi_mode_param_header10_t mph;	/* mode page header */
	scsi_mode_param_desc_t desc;	/* block descriptor */
	scsi_mp_rigid_device_geometry_t rdg;
					/* rigid geometry data */
} scsi_mode_page_rdg_data10_t;

typedef struct {
	scsi_mode_param_header6_t mph;	/* mode page header */
	scsi_mode_param_desc_t desc;	/* block descriptor */
	scsi_mp_caching_t cache;	/* cache page data */
} scsi_mode_page_cache6_t;

typedef struct {
	scsi_mode_param_header10_t mph;	/* mode page header */
	scsi_mode_param_desc_t desc;	/* block descriptor */
	scsi_mp_caching_t cache;	/* cache page data */
} scsi_mode_page_cache10_t;

/* --------------------------------------------------------------
 * Format Unit command
 * ------------------------------------------------------------
 */

/*
 * Format Unit CDB
 */
typedef struct {
	uint8_t         opcode;
#ifdef __BIGENDIAN
	uint8_t         res1:3;
	uint8_t         fmtdata:1;	/* if set, data out phase has format
					 * data
					 */
	uint8_t         cmplst:1;	/* if set, defect list is complete */
	uint8_t         def_list:3;	/* format of defect descriptor is
					 * fmtdata =1
					 */
#else
	uint8_t         def_list:3;	/* format of defect descriptor is
					 * fmtdata = 1
					 */
	uint8_t         cmplst:1;	/* if set, defect list is complete */
	uint8_t         fmtdata:1;	/* if set, data out phase has format
					 * data
					 */
	uint8_t         res1:3;
#endif
	uint8_t         interleave_msb;
	uint8_t         interleave_lsb;
	uint8_t         vendor_spec;
	uint8_t         control;
} scsi_format_unit_t;

/*
 * h
 */
typedef struct {
	uint8_t         opcode;
#ifdef __BIGENDIAN
	uint8_t         reserved:3;
	uint8_t         obsolete:4;
	uint8_t         extent:1;
#else
	uint8_t         extent:1;
	uint8_t         obsolete:4;
	uint8_t         reserved:3;
#endif
	uint8_t         reservation_id;
	uint16_t        param_list_len;
	uint8_t         control;
} scsi_reserve6_t;

/*
 * h
 */
typedef struct {
	uint8_t         opcode;
#ifdef __BIGENDIAN
	uint8_t         reserved1:3;
	uint8_t         obsolete:4;
	uint8_t         extent:1;
#else
	uint8_t         extent:1;
	uint8_t         obsolete:4;
	uint8_t         reserved1:3;
#endif
	uint8_t         reservation_id;
	uint16_t        reserved2;
	uint8_t         control;
} scsi_release6_t;

/*
 * h
 */
typedef struct {
	uint8_t         opcode;
#ifdef __BIGENDIAN
	uint8_t         reserved1:3;
	uint8_t         third_party:1;
	uint8_t         reserved2:2;
	uint8_t         long_id:1;
	uint8_t         extent:1;
#else
	uint8_t         extent:1;
	uint8_t         long_id:1;
	uint8_t         reserved2:2;
	uint8_t         third_party:1;
	uint8_t         reserved1:3;
#endif
	uint8_t         reservation_id;
	uint8_t         third_pty_dev_id;
	uint8_t         reserved3;
	uint8_t         reserved4;
	uint8_t         reserved5;
	uint16_t        param_list_len;
	uint8_t         control;
} scsi_reserve10_t;

typedef struct {
	uint8_t         opcode;
#ifdef __BIGENDIAN
	uint8_t         reserved1:3;
	uint8_t         third_party:1;
	uint8_t         reserved2:2;
	uint8_t         long_id:1;
	uint8_t         extent:1;
#else
	uint8_t         extent:1;
	uint8_t         long_id:1;
	uint8_t         reserved2:2;
	uint8_t         third_party:1;
	uint8_t         reserved1:3;
#endif
	uint8_t         reservation_id;
	uint8_t         third_pty_dev_id;
	uint8_t         reserved3;
	uint8_t         reserved4;
	uint8_t         reserved5;
	uint16_t        param_list_len;
	uint8_t         control;
} scsi_release10_t;

typedef struct {
	uint8_t         opcode;
#ifdef __BIGENDIAN
	uint8_t         lun:3;
	uint8_t         dpo:1;
	uint8_t         reserved:2;
	uint8_t         bytchk:1;
	uint8_t         reladdr:1;
#else
	uint8_t         reladdr:1;
	uint8_t         bytchk:1;
	uint8_t         reserved:2;
	uint8_t         dpo:1;
	uint8_t         lun:3;
#endif
	uint8_t         lba0;
	uint8_t         lba1;
	uint8_t         lba2;
	uint8_t         lba3;
	uint8_t         reserved1;
	uint8_t         verification_len0;
	uint8_t         verification_len1;
	uint8_t         control_byte;
} scsi_verify10_t;

typedef struct {
	uint8_t         opcode;
#ifdef __BIGENDIAN
	uint8_t         lun:3;
	uint8_t         reserved:5;
#else
	uint8_t         reserved:5;
	uint8_t         lun:3;
#endif
	uint8_t         reserved0;
	uint8_t         reserved1;
	uint8_t         alloc_len;
	uint8_t         control_byte;
} scsi_request_sense_t;

/* ------------------------------------------------------------
 * SCSI status byte values
 * ------------------------------------------------------------
 */
#define SCSI_STATUS_GOOD                   0x00
#define SCSI_STATUS_CHECK_CONDITION        0x02
#define SCSI_STATUS_CONDITION_MET          0x04
#define SCSI_STATUS_BUSY                   0x08
#define SCSI_STATUS_INTERMEDIATE           0x10
#define SCSI_STATUS_ICM                    0x14	/* intermediate condition met */
#define SCSI_STATUS_RESERVATION_CONFLICT   0x18
#define SCSI_STATUS_COMMAND_TERMINATED     0x22
#define SCSI_STATUS_QUEUE_FULL             0x28
#define SCSI_STATUS_ACA_ACTIVE             0x30

#define SCSI_MAX_ALLOC_LEN		0xFF	/* maximum allocarion length
						 * in CDBs
						 */

#define SCSI_OP_WRITE_VERIFY10      0x2E
#define SCSI_OP_WRITE_VERIFY12      0xAE
#define SCSI_OP_UNDEF               0xFF

/*
 * SCSI WRITE-VERIFY(10) command
 */
typedef struct {
	uint8_t         opcode;
#ifdef __BIGENDIAN
	uint8_t         reserved1:3;
	uint8_t         dpo:1;		/* Disable Page Out */
	uint8_t         reserved2:1;
	uint8_t         ebp:1;		/* erse by-pass */
	uint8_t         bytchk:1;	/* byte check */
	uint8_t         rel_adr:1;	/* relative address */
#else
	uint8_t         rel_adr:1;	/* relative address */
	uint8_t         bytchk:1;	/* byte check */
	uint8_t         ebp:1;		/* erse by-pass */
	uint8_t         reserved2:1;
	uint8_t         dpo:1;		/* Disable Page Out */
	uint8_t         reserved1:3;
#endif
	uint8_t         lba0;		/* logical block address - MSB */
	uint8_t         lba1;
	uint8_t         lba2;
	uint8_t         lba3;		/* LSB */
	uint8_t         reserved3;
	uint8_t         xfer_length0;	/* transfer length in blocks - MSB */
	uint8_t         xfer_length1;	/* LSB */
	uint8_t         control;
} scsi_write_verify10_t;

#pragma pack()

#endif /* __SCSI_H__ */
