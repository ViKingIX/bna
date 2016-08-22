/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BFI_H__
#define __BFI_H__

#include <defs/bfa_defs.h>

#pragma pack(1)

/* Per dma segment max size */
#define	BFI_MEM_DMA_SEG_SZ	(131072U)

/* Get number of dma segments required */
#define BFI_MEM_DMA_NSEGS(_num_reqs, _req_sz)				     \
	((uint16_t)(((((_num_reqs) * (_req_sz)) + BFI_MEM_DMA_SEG_SZ - 1) &  \
	 ~(BFI_MEM_DMA_SEG_SZ - 1)) / BFI_MEM_DMA_SEG_SZ))

/* Get num dma reqs - that fit in a segment */
#define BFI_MEM_NREQS_SEG(_rqsz) (BFI_MEM_DMA_SEG_SZ / (_rqsz))

/* Get segment num from tag */
#define BFI_MEM_SEG_FROM_TAG(_tag, _rqsz) ((_tag) / BFI_MEM_NREQS_SEG(_rqsz))

/* Get dma req offset in a segment */
#define BFI_MEM_SEG_REQ_OFFSET(_tag, _sz)	\
	((_tag) - (BFI_MEM_SEG_FROM_TAG(_tag, _sz) * BFI_MEM_NREQS_SEG(_sz)))

/**
 * BFI FW image type
 */
#define	BFI_FLASH_CHUNK_SZ			256	/*!< Flash chunk size */
#define	BFI_FLASH_CHUNK_SZ_WORDS	(BFI_FLASH_CHUNK_SZ/sizeof(uint32_t))
#define BFI_FLASH_IMAGE_SZ 0x100000

/**
 * Msg header common to all msgs
 */
struct bfi_mhdr_s {
	uint8_t		msg_class;	/*!< @ref bfi_mclass_t		    */
	uint8_t		msg_id;		/*!< msg opcode with in the class   */
	union {
		struct {
			uint8_t	qid;
			uint8_t	fn_lpu;	/*!< msg destination		    */
		} h2i;
		uint16_t	i2htok;	/*!< token in msgs to host	    */
	} mtag;
};
typedef struct bfi_mhdr_s bfi_mhdr_t;

#define bfi_fn_lpu(__fn, __lpu)	((__fn) << 1 | (__lpu))
#define bfi_mhdr_2_fn(_mh)	((_mh)->mtag.h2i.fn_lpu >> 1)
#define bfi_mhdr_2_qid(_mh)	(_mh)->mtag.h2i.qid

#define bfi_h2i_set(_mh, _mc, _op, _fn_lpu) do {	\
	(_mh).msg_class 	= (_mc);		\
	(_mh).msg_id		= (_op);		\
	(_mh).mtag.h2i.fn_lpu	= (_fn_lpu);		\
} while (0)

#define bfi_i2h_set(_mh, _mc, _op, _i2htok) do {		\
	(_mh).msg_class 		= (_mc);		\
	(_mh).msg_id			= (_op);		\
	(_mh).mtag.i2htok		= (_i2htok);		\
} while (0)

/*
 * Message opcodes: 0-127 to firmware, 128-255 to host
 */
#define BFI_I2H_OPCODE_BASE	128
#define BFA_I2HM(_x) 			((_x) + BFI_I2H_OPCODE_BASE)

/**
 ****************************************************************************
 *
 * Scatter Gather Element and Page definition
 *
 ****************************************************************************
 */

#define BFI_SGE_INLINE	1
#define BFI_SGE_INLINE_MAX	(BFI_SGE_INLINE + 1)

/**
 * SG Flags
 */
enum {
	BFI_SGE_DATA		= 0,	/*!< data address, not last	     */
	BFI_SGE_DATA_CPL	= 1,	/*!< data addr, last in current page */
	BFI_SGE_DATA_LAST	= 3,	/*!< data address, last		     */
	BFI_SGE_LINK		= 2,	/*!< link address		     */
	BFI_SGE_PGDLEN		= 2,	/*!< cumulative data length for page */
};

/**
 * DMA addresses
 */
union bfi_addr_u {
	struct {
		uint32_t	addr_lo;
		uint32_t	addr_hi;
	} a32;
};
typedef union bfi_addr_u bfi_addr_t;

/**
 * Scatter Gather Element used for fast-path IO requests
 */
struct bfi_sge_s {
#ifdef __BIGENDIAN
	uint32_t	flags:2,
			rsvd:2,
			sg_len:28;
#else
	uint32_t	sg_len:28,
			rsvd:2,
			flags:2;
#endif
	union bfi_addr_u sga;
};
typedef struct bfi_sge_s bfi_sge_t;

/**
 * Generic DMA addr-len pair.
 */
struct bfi_alen_s {
	union bfi_addr_u	al_addr;	/* DMA addr of buffer	*/
	uint32_t		al_len;		/* length of buffer	*/
};
typedef struct bfi_alen_s bfi_alen_t;

/**
 * Scatter Gather Page
 */
#define BFI_SGPG_DATA_SGES		7
#define BFI_SGPG_SGES_MAX		(BFI_SGPG_DATA_SGES + 1)
#define BFI_SGPG_RSVD_WD_LEN	8
struct bfi_sgpg_s {
	struct bfi_sge_s sges[BFI_SGPG_SGES_MAX];
	uint32_t	rsvd[BFI_SGPG_RSVD_WD_LEN];
};
typedef struct bfi_sgpg_s bfi_sgpg_t;

/* FCP module definitions */
#define BFI_IO_MAX	(2000U)
#define BFI_IOIM_SNSLEN	(256U)
#define BFI_IOIM_SNSBUF_SEGS	\
	BFI_MEM_DMA_NSEGS(BFI_IO_MAX, BFI_IOIM_SNSLEN)

/*
 * Large Message structure - 128 Bytes size Msgs
 */
#define BFI_LMSG_SZ		128
#define BFI_LMSG_PL_WSZ	\
			((BFI_LMSG_SZ - sizeof(struct bfi_mhdr_s)) / 4)

struct bfi_msg_s {
	struct bfi_mhdr_s mhdr;
	uint32_t	pl[BFI_LMSG_PL_WSZ];
};
typedef struct bfi_msg_s bfi_msg_t;

/**
 * Mailbox message structure
 */
#define BFI_MBMSG_SZ		7
struct bfi_mbmsg_s {
	struct bfi_mhdr_s	mh;
	uint32_t		pl[BFI_MBMSG_SZ];
};
typedef struct bfi_mbmsg_s bfi_mbmsg_t;

/**
 * Supported PCI function class codes (personality)
 */
enum bfi_pcifn_class {
	BFI_PCIFN_CLASS_FC	= 0x0c04,
	BFI_PCIFN_CLASS_ETH	= 0x0200,
};
typedef enum bfi_pcifn_class bfi_pcifn_class_t;

/**
 * Message Classes
 */
enum bfi_mclass {
	BFI_MC_IOC		= 1,	/*!< IO Controller (IOC)	    */
	BFI_MC_DIAG		= 2,	/*!< Diagnostic Msgs		    */
	BFI_MC_FLASH		= 3,	/*!< Flash message class	    */
	BFI_MC_CEE		= 4,	/*!< CEE			    */
	BFI_MC_FCPORT		= 5,	/*!< FC port			    */
	BFI_MC_IOCFC		= 6,	/*!< FC - IO Controller (IOC)	    */
	BFI_MC_ABLK		= 7,	/*!< ASIC block configuration	    */
	BFI_MC_UF		= 8,	/*!< Unsolicited frame receive	    */
	BFI_MC_FCXP		= 9,	/*!< FC Transport		    */
	BFI_MC_LPS		= 10,	/*!< lport fc login services	    */
	BFI_MC_RPORT		= 11,	/*!< Remote port		    */
	BFI_MC_ITN		= 12,	/*!< I-T nexus (Initiator mode)	    */
	BFI_MC_IOIM_READ	= 13,	/*!< read IO (Initiator mode)	    */
	BFI_MC_IOIM_WRITE	= 14,	/*!< write IO (Initiator mode)	    */
	BFI_MC_IOIM_IO		= 15,	/*!< IO (Initiator mode)	    */
	BFI_MC_IOIM		= 16,	/*!< IO (Initiator mode)	    */
	BFI_MC_IOIM_IOCOM	= 17,	/*!< good IO completion		    */
	BFI_MC_TSKIM		= 18,	/*!< Initiator Task management	    */
	BFI_MC_SBOOT		= 19,	/*!< SAN boot services		    */
	BFI_MC_IPFC		= 20,	/*!< IP over FC Msgs		    */
	BFI_MC_PORT		= 21,	/*!< Physical port		    */
	BFI_MC_SFP		= 22,	/*!< SFP module			    */
	BFI_MC_MSGQ		= 23,	/*!< MSGQ			    */
	BFI_MC_ENET		= 24,	/*!< ENET commands/responses	    */
	BFI_MC_PHY		= 25,	/*!< External PHY message class	    */
	BFI_MC_NBOOT		= 26,	/*!< Network Boot		    */
	BFI_MC_TIO_READ		= 27,	/*!< read IO (Target mode)	    */
	BFI_MC_TIO_WRITE	= 28,	/*!< write IO (Target mode)	    */
	BFI_MC_TIO_DATA_XFERED	= 29,	/*!< ds transferred (target mode)   */
	BFI_MC_TIO_IO		= 30,	/*!< IO (Target mode)		    */
	BFI_MC_TIO		= 31,	/*!< IO (target mode)		    */
	BFI_MC_MFG		= 32,	/*!< MFG/ASIC block commands	    */
	BFI_MC_EDMA		= 33,	/*!< EDMA copy commands		    */
	BFI_MC_FRU		= 34,
	BFI_MC_MAX		= 35
};
typedef enum bfi_mclass bfi_mclass_t;

#define BFI_IOC_MAX_CQS		4
#define BFI_IOC_MAX_CQS_ASIC	8
#define BFI_IOC_MSGLEN_MAX	32	/* 32 bytes */

#define BFI_BOOT_MEMTEST_RES_ADDR   0x900
#define BFI_BOOT_MEMTEST_RES_SIG    0xA0A1A2A3

/**
 *----------------------------------------------------------------------
 *				IOC
 *----------------------------------------------------------------------
 */

/**
 * Different asic generations
 */
enum bfi_asic_gen {
	BFI_ASIC_GEN_CB		= 1,	/* crossbow 8G FC		*/
	BFI_ASIC_GEN_CT		= 2,	/* catapult 8G FC or 10G CNA	*/
	BFI_ASIC_GEN_CT2	= 3,	/* catapult-2 16G FC or 10G CNA	*/
};

enum bfi_asic_mode {
	BFI_ASIC_MODE_FC	= 1,	/* FC upto 8G speed		*/
	BFI_ASIC_MODE_FC16	= 2,	/* FC upto 16G speed		*/
	BFI_ASIC_MODE_ETH	= 3,	/* Ethernet ports		*/
	BFI_ASIC_MODE_COMBO	= 4,	/* FC 16G and Ethernet 10G port	*/
};
typedef enum bfi_asic_mode bfi_asic_mode_t;

enum bfi_ioc_h2i_msgs {
	BFI_IOC_H2I_ENABLE_REQ		= 1,
	BFI_IOC_H2I_DISABLE_REQ		= 2,
	BFI_IOC_H2I_GETATTR_REQ		= 3,
	BFI_IOC_H2I_DBG_SYNC		= 4,
	BFI_IOC_H2I_DBG_DUMP		= 5,
};
typedef enum bfi_ioc_h2i_msgs bfi_ioc_h2i_msgs_t;

enum bfi_ioc_i2h_msgs {
	BFI_IOC_I2H_ENABLE_REPLY	= BFA_I2HM(1),
	BFI_IOC_I2H_DISABLE_REPLY 	= BFA_I2HM(2),
	BFI_IOC_I2H_GETATTR_REPLY 	= BFA_I2HM(3),
	BFI_IOC_I2H_HBEAT		= BFA_I2HM(4),
	BFI_IOC_I2H_ACQ_ADDR_REPLY 	= BFA_I2HM(5),
};
typedef enum bfi_ioc_i2h_msgs bfi_ioc_i2h_msgs_t;

/**
 * BFI_IOC_H2I_GETATTR_REQ message
 */
struct bfi_ioc_getattr_req_s {
	struct bfi_mhdr_s	mh;
	union bfi_addr_u	attr_addr;
};
typedef struct bfi_ioc_getattr_req_s bfi_ioc_getattr_req_t;

#define BFI_IOC_ATTR_UUID_SZ	16
struct bfi_ioc_attr_s {
	wwn_t		mfg_pwwn;	/*!< Mfg port wwn	   */
	wwn_t		mfg_nwwn;	/*!< Mfg node wwn	   */
	mac_t		mfg_mac;	/*!< Mfg mac		   */
	uint8_t		port_mode;	/* bfi_port_mode_t	   */
	uint8_t		rsvd_a;
	wwn_t		pwwn;
	wwn_t		nwwn;
	mac_t		mac;		/*!< PBC or Mfg mac	   */
	uint16_t	rsvd_b;
	mac_t		fcoe_mac;
	uint16_t	rsvd_c;
	char		brcd_serialnum[STRSZ(BFA_MFG_SERIALNUM_SIZE)];
	uint8_t		pcie_gen;
	uint8_t		pcie_lanes_orig;
	uint8_t		pcie_lanes;
	uint8_t		rx_bbcredit;	/*!< receive buffer credits */
	uint32_t	adapter_prop;	/*!< adapter properties     */
	uint16_t	maxfrsize;	/*!< max receive frame size */
	char		asic_rev;
	uint8_t		rsvd_d;
	char		fw_version[BFA_VERSION_LEN];
	char		optrom_version[BFA_VERSION_LEN];
	struct		bfa_mfg_vpd_s	vpd;
	uint32_t	card_type;	/*!< card type			*/
	uint8_t		mfg_day;	/*!< manufacturing day */
	uint8_t		mfg_month;	/*!< manufacturing month */
	uint16_t	mfg_year;	/*!< manufacturing year */
	uint8_t		uuid[BFI_IOC_ATTR_UUID_SZ];	/*!< chinook uuid */
};
typedef struct bfi_ioc_attr_s bfi_ioc_attr_t;

/**
 * BFI_IOC_I2H_GETATTR_REPLY message
 */
struct bfi_ioc_getattr_reply_s {
	struct	bfi_mhdr_s	mh;	/*!< Common msg header		*/
	uint8_t			status;	/*!< cfg reply status		*/
	uint8_t			rsvd[3];
};
typedef struct bfi_ioc_getattr_reply_s bfi_ioc_getattr_reply_t;

/**
 * Firmware memory page offsets
 */
#define BFI_IOC_SMEM_PG0_CB	(0x40)
#define BFI_IOC_SMEM_PG0_CT	(0x180)

/**
 * Firmware statistic offset
 */
#define BFI_IOC_FWSTATS_OFF	(0x6B40)
#define BFI_IOC_FWSTATS_SZ	(4096)

/**
 * Firmware trace offset
 */
#define BFI_IOC_TRC_OFF		(0x4b00)
#define BFI_IOC_TRC_ENTS	256

#define BFI_IOC_FW_SIGNATURE	(0xbfadbfad)
#define BFA_IOC_FW_INV_SIGN	(0xdeaddead)
#define BFI_IOC_MD5SUM_SZ	4

typedef struct bfi_ioc_fwver_s {
#ifdef __BIGENDIAN
	uint8_t patch;
	uint8_t maint;
	uint8_t minor;
	uint8_t major;
	uint8_t rsvd[2];
	uint8_t build;
	uint8_t phase;
#else
	uint8_t major;
	uint8_t minor;
	uint8_t maint;
	uint8_t patch;
	uint8_t phase;
	uint8_t build;
	uint8_t rsvd[2];
#endif
} bfi_ioc_fwver_t;

struct bfi_ioc_image_hdr_s {
	uint32_t	signature;	/* constant signature		*/
	uint8_t		asic_gen;	/* asic generation		*/
	uint8_t		asic_mode;
	uint8_t		port0_mode;	/* device mode for port 0	*/
	uint8_t		port1_mode;	/* device mode for port 1	*/
	uint32_t	exec;		/* exec vector			*/
	uint32_t	bootenv;	/* fimware boot env		*/
	uint32_t	rsvd_b[2];
	bfi_ioc_fwver_t	fwver;
	uint32_t	md5sum[BFI_IOC_MD5SUM_SZ];
};
typedef struct bfi_ioc_image_hdr_s bfi_ioc_image_hdr_t;

typedef enum bfi_ioc_img_ver_cmp_e {
	BFI_IOC_IMG_VER_INCOMP,
	BFI_IOC_IMG_VER_OLD,
	BFI_IOC_IMG_VER_SAME,
	BFI_IOC_IMG_VER_BETTER
} bfi_ioc_img_ver_cmp_t;

#define BFI_FWBOOT_DEVMODE_OFF		4
#define BFI_FWBOOT_TYPE_OFF		8
#define BFI_FWBOOT_ENV_OFF		12
#define BFI_FWBOOT_DEVMODE(__asic_gen, __asic_mode, __p0_mode, __p1_mode) \
	(((uint32_t)(__asic_gen)) << 24 |	\
	 ((uint32_t)(__asic_mode)) << 16 |	\
	 ((uint32_t)(__p0_mode)) << 8 |	\
	 ((uint32_t)(__p1_mode)))

enum bfi_fwboot_type {
	BFI_FWBOOT_TYPE_NORMAL	= 0,
	BFI_FWBOOT_TYPE_FLASH	= 1,
	BFI_FWBOOT_TYPE_MEMTEST	= 2,
};

enum bfi_fwboot_env {
	BFI_FWBOOT_ENV_OS	= 0,
	BFI_FWBOOT_ENV_BIOS	= 1,
	BFI_FWBOOT_ENV_UEFI	= 2,
	BFI_FWBOOT_ENV_NFC	= 3,
};

enum bfi_port_mode {
	BFI_PORT_MODE_FC	= 1,
	BFI_PORT_MODE_ETH	= 2,
};
typedef enum bfi_port_mode bfi_port_mode_t;

struct bfi_ioc_hbeat_s {
	struct bfi_mhdr_s  mh;		/*!< common msg header		*/
	uint32_t	   hb_count;	/*!< current heart beat count	*/
};
typedef struct bfi_ioc_hbeat_s bfi_ioc_hbeat_t;

/**
 * IOC hardware/firmware state
 */
enum bfi_ioc_state {
	BFI_IOC_UNINIT		= 0,	/*!< not initialized		     */
	BFI_IOC_INITING		= 1,	/*!< h/w is being initialized	     */
	BFI_IOC_HWINIT		= 2,	/*!< h/w is initialized		     */
	BFI_IOC_CFG		= 3,	/*!< IOC configuration in progress   */
	BFI_IOC_OP		= 4,	/*!< IOC is operational		     */
	BFI_IOC_DISABLING	= 5,	/*!< IOC is being disabled	     */
	BFI_IOC_DISABLED	= 6,	/*!< IOC is disabled		     */
	BFI_IOC_CFG_DISABLED	= 7,	/*!< IOC is being disabled;transient */
	BFI_IOC_FAIL		= 8,	/*!< IOC heart-beat failure	     */
	BFI_IOC_MEMTEST		= 9,	/*!< IOC is doing memtest	     */
};
typedef enum bfi_ioc_state bfi_ioc_state_t;

#define BFA_IOC_CB_JOIN_SH	16
#define BFA_IOC_CB_FWSTATE_MASK	0x0000ffff
#define BFA_IOC_CB_JOIN_MASK	0xffff0000

#define BFI_IOC_ENDIAN_SIG  0x12345678

enum {
	BFI_ADAPTER_TYPE_FC	= 0x01,		/*!< FC adapters	   */
	BFI_ADAPTER_TYPE_MK	= 0x0f0000,	/*!< adapter type mask     */
	BFI_ADAPTER_TYPE_SH	= 16,	        /*!< adapter type shift    */
	BFI_ADAPTER_NPORTS_MK	= 0xff00,	/*!< number of ports mask  */
	BFI_ADAPTER_NPORTS_SH	= 8,	        /*!< number of ports shift */
	BFI_ADAPTER_SPEED_MK	= 0xff,		/*!< adapter speed mask    */
	BFI_ADAPTER_SPEED_SH	= 0,	        /*!< adapter speed shift   */
	BFI_ADAPTER_PROTO	= 0x100000,	/*!< prototype adapaters   */
	BFI_ADAPTER_TTV		= 0x200000,	/*!< TTV debug capable     */
	BFI_ADAPTER_UNSUPP	= 0x400000,	/*!< unknown adapter type  */
};

#define BFI_ADAPTER_GETP(__prop, __adap_prop)			\
	(((__adap_prop) & BFI_ADAPTER_ ## __prop ## _MK) >>	\
		BFI_ADAPTER_ ## __prop ## _SH)
#define BFI_ADAPTER_SETP(__prop, __val)				\
	((__val) << BFI_ADAPTER_ ## __prop ## _SH)
#define BFI_ADAPTER_IS_PROTO(__adap_type)			\
	((__adap_type) & BFI_ADAPTER_PROTO)
#define BFI_ADAPTER_IS_TTV(__adap_type)				\
	((__adap_type) & BFI_ADAPTER_TTV)
#define BFI_ADAPTER_IS_UNSUPP(__adap_type)			\
	((__adap_type) & BFI_ADAPTER_UNSUPP)
#define BFI_ADAPTER_IS_SPECIAL(__adap_type)			\
	((__adap_type) & (BFI_ADAPTER_TTV | BFI_ADAPTER_PROTO |	\
			BFI_ADAPTER_UNSUPP))

/**
 * BFI_IOC_H2I_ENABLE_REQ & BFI_IOC_H2I_DISABLE_REQ messages
 */
struct bfi_ioc_ctrl_req_s {
	struct bfi_mhdr_s	mh;
	uint16_t		clscode;
	uint16_t		rsvd;
	uint32_t		tv_sec;
};
typedef struct bfi_ioc_ctrl_req_s bfi_ioc_enable_req_t;
typedef struct bfi_ioc_ctrl_req_s bfi_ioc_disable_req_t;

/**
 * BFI_IOC_I2H_ENABLE_REPLY & BFI_IOC_I2H_DISABLE_REPLY messages
 */
struct bfi_ioc_ctrl_reply_s {
	struct bfi_mhdr_s	mh;		/*!< Common msg header     */
	uint8_t			status;		/*!< enable/disable status */
	uint8_t			port_mode;	/*!< bfa_mode_t		*/
	uint8_t			cap_bm; 	/*!< capability bit mask */
	uint8_t			rsvd;
};
typedef struct bfi_ioc_ctrl_reply_s bfi_ioc_enable_reply_t;
typedef struct bfi_ioc_ctrl_reply_s bfi_ioc_disable_reply_t;

#define BFI_IOC_MSGSZ   8
/**
 * H2I Messages
 */
union bfi_ioc_h2i_msg_u {
	struct bfi_mhdr_s 		mh;
	struct bfi_ioc_ctrl_req_s	enable_req;
	struct bfi_ioc_ctrl_req_s	disable_req;
	struct bfi_ioc_getattr_req_s	getattr_req;
	uint32_t			mboxmsg[BFI_IOC_MSGSZ];
};
typedef union bfi_ioc_h2i_msg_u bfi_ioc_h2i_msg_t;

/**
 * I2H Messages
 */
union bfi_ioc_i2h_msg_u {
	struct bfi_mhdr_s		mh;
	struct bfi_ioc_ctrl_reply_s 	fw_event;
	uint32_t			mboxmsg[BFI_IOC_MSGSZ];
};
typedef union bfi_ioc_i2h_msg_u bfi_ioc_i2h_msg_t;


/**
 *----------------------------------------------------------------------
 *				PBC
 *----------------------------------------------------------------------
 */

#define BFI_PBC_MAX_BLUNS	8
#define BFI_PBC_MAX_VPORTS	16

#define BFI_PBC_PORT_DISABLED	 2

/**
 * PBC boot lun configuration
 */
struct bfi_pbc_blun_s {
	wwn_t		tgt_pwwn;
	lun_t		tgt_lun;
};
typedef struct bfi_pbc_blun_s bfi_pbc_blun_t;

/**
 * PBC virtual port configuration
 */
struct bfi_pbc_vport_s {
	wwn_t		vp_pwwn;
	wwn_t		vp_nwwn;
};
typedef struct bfi_pbc_vport_s bfi_pbc_vport_t;

/**
 * BFI pre-boot configuration information
 */
struct bfi_pbc_s {
	uint8_t		port_enabled;
	uint8_t		boot_enabled;
	uint8_t		nbluns;
	uint8_t		nvports;
	uint8_t		port_speed;
	uint8_t		rsvd_a;
	uint16_t	hss;
	wwn_t		pbc_pwwn;
	wwn_t		pbc_nwwn;
	struct bfi_pbc_blun_s blun[ BFI_PBC_MAX_BLUNS ];
	struct bfi_pbc_vport_s vport[ BFI_PBC_MAX_VPORTS ];
};
typedef struct bfi_pbc_s bfi_pbc_t;

/**
 *----------------------------------------------------------------------
 *				SFP
 *----------------------------------------------------------------------
 */

#define	BFI_SFP_MAX_SGES	2

typedef enum {
	BFI_SFP_H2I_SHOW  = 1,
	BFI_SFP_H2I_SCN	  = 2,
} bfi_sfp_h2i_t;


typedef enum {
	BFI_SFP_I2H_SHOW  = BFA_I2HM(BFI_SFP_H2I_SHOW),
	BFI_SFP_I2H_SCN	  = BFA_I2HM(BFI_SFP_H2I_SCN),
} bfi_sfp_i2h_t;

/**
 *	SFP state change notification
 */
typedef struct {
	struct bfi_mhdr_s mhr;		/*!< host msg header		*/
	uint8_t		event;
	uint8_t		sfpid;
	uint8_t		pomlvl;		/*!< pom level: normal/warning/alarm */
	uint8_t		is_elb;		/*!< e-loopback */
} bfi_sfp_scn_t;

/**
 * @brief
 *	SFP state
 */
enum bfa_sfp_stat_t {
	BFA_SFP_STATE_INIT	= 0,	/*!< SFP state is uninit	*/
	BFA_SFP_STATE_REMOVED	= 1,	/*!< SFP is removed		*/
	BFA_SFP_STATE_INSERTED	= 2,	/*!< SFP is inserted		*/
	BFA_SFP_STATE_VALID	= 3,	/*!< SFP is valid		*/
	BFA_SFP_STATE_UNSUPPORT	= 4,	/*!< SFP is unsupport		*/
	BFA_SFP_STATE_FAILED	= 5,	/*!< SFP i2c read fail		*/
};

/**
 * @brief
 *	SFP memory access type
 */
typedef enum {
	BFI_SFP_MEM_ALL		= 0x1,	/* access all data field */
	BFI_SFP_MEM_DIAGEXT	= 0x2,	/* access diag ext data field only */
} bfi_sfp_mem_t;

typedef struct {
	struct bfi_mhdr_s	mh;
	uint8_t			memtype;	/*!< ref bfi_diag_sfpmem_t */
	uint8_t			rsvd[3];
	struct bfi_alen_s	alen;
} bfi_sfp_req_t;

typedef struct {
	struct bfi_mhdr_s	mh;
	uint8_t		status;
	uint8_t		state;
	uint8_t		rsvd[2];
} bfi_sfp_rsp_t;


/**
 *----------------------------------------------------------------------
 *				DIAG
 *----------------------------------------------------------------------
 */
typedef enum {
	BFI_DIAG_H2I_PORTBEACON = 1,
	BFI_DIAG_H2I_LOOPBACK = 2,
	BFI_DIAG_H2I_FWPING = 3,
	BFI_DIAG_H2I_TEMPSENSOR = 4,
	BFI_DIAG_H2I_LEDTEST = 5,
	BFI_DIAG_H2I_QTEST	= 6,
	BFI_DIAG_H2I_DPORT	= 7,
} bfi_diag_h2i_t;

typedef enum {
	BFI_DIAG_I2H_PORTBEACON = BFA_I2HM(BFI_DIAG_H2I_PORTBEACON),
	BFI_DIAG_I2H_LOOPBACK = BFA_I2HM(BFI_DIAG_H2I_LOOPBACK),
	BFI_DIAG_I2H_FWPING = BFA_I2HM(BFI_DIAG_H2I_FWPING),
	BFI_DIAG_I2H_TEMPSENSOR = BFA_I2HM(BFI_DIAG_H2I_TEMPSENSOR),
	BFI_DIAG_I2H_LEDTEST = BFA_I2HM(BFI_DIAG_H2I_LEDTEST),
	BFI_DIAG_I2H_QTEST	= BFA_I2HM(BFI_DIAG_H2I_QTEST),
	BFI_DIAG_I2H_DPORT	= BFA_I2HM(BFI_DIAG_H2I_DPORT),
	BFI_DIAG_I2H_DPORT_SCN	= BFA_I2HM(8),
} bfi_diag_i2h_t;

#define	BFI_DIAG_MAX_SGES		2
#define	BFI_DIAG_DMA_BUF_SZ		(2 * 1024)

/**
 * Loopback
 */
typedef struct {
	struct bfi_mhdr_s  mh;
	uint32_t	loopcnt;
	uint32_t	pattern;
	uint8_t		lb_mode;	/*!< bfa_port_opmode_t */
	uint8_t		speed;		/*!< bfa_port_speed_t */
	uint8_t		rsvd[2];
} bfi_diag_lb_req_t;

typedef struct {
	struct bfi_mhdr_s  mh;		/* 4 bytes */
	bfa_diag_loopback_result_t res;	/* 16 bytes */
} bfi_diag_lb_rsp_t;

/**
 * FW Ping
 */
typedef struct {
	struct bfi_mhdr_s  mh;		/* 4 bytes */
	struct bfi_alen_s  alen;	/* 12 bytes */
	uint32_t	data;		/*!< user input data pattern */
	uint32_t	count;		/*!< user input dma count */
	uint8_t		qtag;		/*!< track CPE vc */
	uint8_t		rsv[3];
} bfi_diag_fwping_req_t;

typedef struct {
	struct bfi_mhdr_s  mh;		/* 4 bytes */
	uint32_t	data;		/*!< user input data pattern	*/
	uint8_t		qtag;		/*!< track CPE vc		*/
	uint8_t		dma_status;	/*!< dma status			*/
	uint8_t		rsv[2];
} bfi_diag_fwping_rsp_t;

/**
 * Temperature Sensor
 */
typedef struct {
	struct bfi_mhdr_s  mh;		/* 4 bytes */
	uint16_t	temp;		/* 10-bit A/D value */
	uint16_t	brd_temp;	/*!< 9-bit board temp */
	uint8_t		status;
	uint8_t		ts_junc;  	/*!< show junction tempsensor	*/
	uint8_t		ts_brd;		/*!< show board tempsensor	*/
	uint8_t		rsv;
} bfi_diag_ts_req_t, bfi_diag_ts_rsp_t;

/**
 * LED Test
 */
typedef struct {
	struct bfi_mhdr_s  mh;	/* 4 bytes */
	uint8_t		cmd;
	uint8_t		color;
	uint8_t		portid;
	uint8_t		led;	/* bitmap of LEDs to be tested */
	uint16_t	freq;	/* no. of blinks every 10 secs */
	uint8_t		rsv[2];
} bfi_diag_ledtest_req_t;

/* notify host led operation is done */
typedef struct {
	struct bfi_mhdr_s  mh;	/* 4 bytes */
} bfi_diag_ledtest_rsp_t;

/**
 * Port beaconing
 */
typedef struct {
	struct bfi_mhdr_s  mh;	/* 4 bytes */
	uint32_t	period;	/*!< beaconing period */
	uint8_t		beacon;	/*!< 1: beacon on */
	uint8_t		rsvd[3];
} bfi_diag_portbeacon_req_t;

/* notify host the beacon is off */
typedef struct {
	struct bfi_mhdr_s  mh;	/* 4 bytes */
} bfi_diag_portbeacon_rsp_t;

/**
 * Queue test
 */
typedef struct {
	struct bfi_mhdr_s	mh;		/* 4 bytes */
	uint32_t	data[BFI_LMSG_PL_WSZ]; /* fill up tcm prefetch area */
} bfi_diag_qtest_req_t, bfi_diag_qtest_rsp_t;

/**
 * D-port test
 */
enum bfi_dport_req_e {
	BFI_DPORT_DISABLE	= 0,	/*!< disable dport request	*/
	BFI_DPORT_ENABLE	= 1,	/*!< enable dport request	*/
	BFI_DPORT_START		= 2,	/*!< start dport request	*/
	BFI_DPORT_SHOW		= 3,	/*!< show dport request	*/
	BFI_DPORT_DYN_DISABLE	= 4,	/*!< disable dynamic dport request */
};

enum bfi_dport_scn_e {
	BFI_DPORT_SCN_TESTSTART		= 1,
	BFI_DPORT_SCN_TESTCOMP		= 2,
	BFI_DPORT_SCN_SFP_REMOVED	= 3,
	BFI_DPORT_SCN_DDPORT_ENABLE	= 4,
	BFI_DPORT_SCN_DDPORT_DISABLE	= 5,
	BFI_DPORT_SCN_FCPORT_DISABLE	= 6,
	BFI_DPORT_SCN_SUBTESTSTART	= 7,
	BFI_DPORT_SCN_TESTSKIP		= 8,
	BFI_DPORT_SCN_DDPORT_DISABLED	= 9,
};

typedef struct {
	struct bfi_mhdr_s	mh;	/* 4 bytes 			*/
	uint8_t		req;	/*!< request 1: enable 0: disable 	*/
	uint8_t		rsvd[3];
	uint32_t	lpcnt;
	uint32_t	payload;
} bfi_diag_dport_req_t;

typedef struct {
	struct bfi_mhdr_s	mh;	/*!< header 4 bytes		*/
	bfa_status_t		status;	/*!< reply status		*/
	wwn_t			pwwn;	/*!< switch port wwn. 8 bytes */
	wwn_t			nwwn;	/*!< switch node wwn. 8 bytes */
} bfi_diag_dport_rsp_t;

typedef struct {
	wwn_t			pwwn;	/*!< switch port wwn. 8 bytes */
	wwn_t			nwwn;	/*!< switch node wwn. 8 bytes */
	uint8_t			type;	/*!< bfa_diag_dport_test_type_e */
	uint8_t			mode;	/*!< bfa_diag_dport_test_opmode_t */
	uint8_t			rsvd[2];
	uint32_t		numfrm; /*!< from switch uint in 1M */
} bfi_diag_dport_scn_teststart_t;

typedef struct {
	uint8_t			status; /*!< bfa_diag_dport_test_status_e */
	uint8_t			speed;  /*!< bfa_port_speed_t  */
	uint16_t		numbuffer; /*!< from switch  */
	uint8_t			subtest_status[DPORT_TEST_MAX];  /* 4 bytes */
	uint32_t		latency;   /*!< from switch  */
	uint32_t		distance;  /*!< from swtich unit in meters  */
				/*!< Buffers required to saturate the link */
	uint16_t		frm_sz;	/*!< from switch for buf_reqd */
	uint8_t			rsvd[2];
} bfi_diag_dport_scn_testcomp_t;

typedef struct {			/*!< max size == RDS_RMESZ	*/
	struct bfi_mhdr_s	mh;	/*!< header 4 bytes		*/
	uint8_t			state;  /*!< new state			*/
	uint8_t			rsvd[3];
	union {
		bfi_diag_dport_scn_teststart_t	teststart;
		bfi_diag_dport_scn_testcomp_t	testcomp;
	} info;
} bfi_diag_dport_scn_t;

union bfi_diag_dport_msg_u {
	bfi_diag_dport_req_t	req;
	bfi_diag_dport_rsp_t	rsp;
	bfi_diag_dport_scn_t	scn;
};
typedef union bfi_diag_dport_msg_u bfi_diag_dport_msg_t;

/**
 *----------------------------------------------------------------------
 *				MSGQ
 *----------------------------------------------------------------------
 */
#define BFI_MSGQ_FULL(_q)	(((_q->pi + 1) % _q->q_depth) == _q->ci)
#define BFI_MSGQ_EMPTY(_q)	(_q->pi == _q->ci)
#define BFI_MSGQ_UPDATE_CI(_q)	(_q->ci = (_q->ci + 1) % _q->q_depth)
#define BFI_MSGQ_UPDATE_PI(_q)	(_q->pi = (_q->pi + 1) % _q->q_depth)

/* q_depth must be power of 2 */
#define BFI_MSGQ_FREE_CNT(_q)	((_q->ci - _q->pi - 1) & (_q->q_depth - 1))

enum bfi_msgq_h2i_msgs_e {
	BFI_MSGQ_H2I_INIT_REQ		= 1,
	BFI_MSGQ_H2I_DOORBELL_PI	= 2,
	BFI_MSGQ_H2I_DOORBELL_CI	= 3,
	BFI_MSGQ_H2I_CMDQ_COPY_RSP	= 4,
};
typedef enum bfi_msgq_h2i_msgs_e bfi_msgq_h2i_msgs_t;

enum bfi_msgq_i2h_msgs_e {
	BFI_MSGQ_I2H_INIT_RSP		= BFA_I2HM(BFI_MSGQ_H2I_INIT_REQ),
	BFI_MSGQ_I2H_DOORBELL_PI	= BFA_I2HM(BFI_MSGQ_H2I_DOORBELL_PI),
	BFI_MSGQ_I2H_DOORBELL_CI	= BFA_I2HM(BFI_MSGQ_H2I_DOORBELL_CI),
	BFI_MSGQ_I2H_CMDQ_COPY_REQ	= BFA_I2HM(BFI_MSGQ_H2I_CMDQ_COPY_RSP),
};
typedef enum bfi_msgq_i2h_msgs_e bfi_msgq_i2h_msgs_t;


/* Messages(commands/responsed/AENS will have the following header */
struct bfi_msgq_mhdr_s {
	uint8_t		msg_class;
	uint8_t		msg_id;
	uint16_t	msg_token;
	uint16_t	num_entries;
	uint8_t		enet_id;
	uint8_t		rsvd[1];
};
typedef struct bfi_msgq_mhdr_s bfi_msgq_mhdr_t;

#define bfi_msgq_mhdr_set(_mh, _mc, _mid, _tok, _enet_id) do {        \
	(_mh).msg_class 	= (_mc);        \
	(_mh).msg_id		= (_mid);       \
	(_mh).msg_token		= (_tok);       \
	(_mh).enet_id		= (_enet_id);   \
} while (0)

/*
 * Mailbox  for messaging interface
 *
*/
#define BFI_MSGQ_CMD_ENTRY_SIZE		(64)    /* TBD */
#define BFI_MSGQ_RSP_ENTRY_SIZE		(64)    /* TBD */
#define BFI_MSGQ_MSG_SIZE_MAX		(2048)  /* TBD */

#define bfi_msgq_num_cmd_entries(_size)					\
	(((_size) + BFI_MSGQ_CMD_ENTRY_SIZE - 1) / BFI_MSGQ_CMD_ENTRY_SIZE)

struct bfi_msgq_s {
	union bfi_addr_u addr;
	uint16_t q_depth;     /* Total num of entries in the queue */
	uint8_t rsvd[2];
};
typedef struct bfi_msgq_s bfi_msgq_t;

/* BFI_ENET_MSGQ_CFG_REQ TBD init or cfg? */
struct bfi_msgq_cfg_req_s {
	struct bfi_mhdr_s mh;
	struct bfi_msgq_s cmdq;
	struct bfi_msgq_s rspq;
};
typedef struct bfi_msgq_cfg_req_s bfi_msgq_cfg_req_t;

/* BFI_ENET_MSGQ_CFG_RSP */
struct bfi_msgq_cfg_rsp_s {
	struct bfi_mhdr_s mh;
	uint8_t cmd_status;
	uint8_t rsvd[3];
};
typedef struct bfi_msgq_cfg_rsp_s bfi_msgq_cfg_rsp_t;


/* BFI_MSGQ_H2I_DOORBELL */
struct bfi_msgq_h2i_db_s {
	struct bfi_mhdr_s mh;
	union {
		uint16_t cmdq_pi;
		uint16_t rspq_ci;
	} idx;
};
typedef struct bfi_msgq_h2i_db_s bfi_msgq_h2i_db_t;

/* BFI_MSGQ_I2H_DOORBELL */
struct bfi_msgq_i2h_db_s {
	struct bfi_mhdr_s mh;
	union {
		uint16_t rspq_pi;
		uint16_t cmdq_ci;
	} idx;
};
typedef struct bfi_msgq_i2h_db_s bfi_msgq_i2h_db_t;

#define BFI_CMD_COPY_SZ 28

/* BFI_MSGQ_H2I_CMD_COPY_RSP */
struct bfi_msgq_h2i_cmdq_copy_rsp_s {
	struct		bfi_mhdr_s mh;
	uint8_t		data[BFI_CMD_COPY_SZ];
};
typedef struct bfi_msgq_h2i_cmdq_copy_rsp_s bfi_msgq_h2i_cmdq_copy_rsp_t;

/* BFI_MSGQ_I2H_CMD_COPY_REQ */
struct bfi_msgq_i2h_cmdq_copy_req_s {
	struct		bfi_mhdr_s mh;
	uint16_t	offset;
	uint16_t	len;
};
typedef struct bfi_msgq_i2h_cmdq_copy_req_s bfi_msgq_i2h_cmdq_copy_req_t;

/**
 *----------------------------------------------------------------------
 *				ABLK
 *----------------------------------------------------------------------
 */

enum bfi_ablk_h2i_msgs_e {
	BFI_ABLK_H2I_QUERY		= 1,
	BFI_ABLK_H2I_ADPT_CONFIG	= 2,
	BFI_ABLK_H2I_PORT_CONFIG	= 3,
	BFI_ABLK_H2I_PF_CREATE		= 4,
	BFI_ABLK_H2I_PF_DELETE		= 5,
	BFI_ABLK_H2I_PF_UPDATE		= 6,
	BFI_ABLK_H2I_OPTROM_ENABLE	= 7,
	BFI_ABLK_H2I_OPTROM_DISABLE	= 8,
	BFI_ABLK_H2I_NWPAR_ENABLE	= 9,
	BFI_ABLK_H2I_NWPAR_DISABLE	= 10,
	BFI_ABLK_H2I_NWPAR_QUERY	= 11,
	BFI_ABLK_H2I_NWPAR_PF_ENBL	= 12,
	BFI_ABLK_H2I_NWPAR_PF_DSBL	= 13,
	BFI_ABLK_H2I_NWPAR_PF_PERS	= 14,
};
typedef enum bfi_ablk_h2i_msgs_e bfi_ablk_h2i_msgs_t;

enum bfi_ablk_i2h_msgs_e {
	BFI_ABLK_I2H_QUERY		= BFA_I2HM(BFI_ABLK_H2I_QUERY),
	BFI_ABLK_I2H_ADPT_CONFIG	= BFA_I2HM(BFI_ABLK_H2I_ADPT_CONFIG),
	BFI_ABLK_I2H_PORT_CONFIG	= BFA_I2HM(BFI_ABLK_H2I_PORT_CONFIG),
	BFI_ABLK_I2H_PF_CREATE		= BFA_I2HM(BFI_ABLK_H2I_PF_CREATE),
	BFI_ABLK_I2H_PF_DELETE		= BFA_I2HM(BFI_ABLK_H2I_PF_DELETE),
	BFI_ABLK_I2H_PF_UPDATE		= BFA_I2HM(BFI_ABLK_H2I_PF_UPDATE),
	BFI_ABLK_I2H_OPTROM_ENABLE	= BFA_I2HM(BFI_ABLK_H2I_OPTROM_ENABLE),
	BFI_ABLK_I2H_OPTROM_DISABLE	= BFA_I2HM(BFI_ABLK_H2I_OPTROM_DISABLE),
	BFI_ABLK_I2H_NWPAR_ENABLE	= BFA_I2HM(BFI_ABLK_H2I_NWPAR_ENABLE),
	BFI_ABLK_I2H_NWPAR_DISABLE	= BFA_I2HM(BFI_ABLK_H2I_NWPAR_DISABLE),
	BFI_ABLK_I2H_NWPAR_QUERY	= BFA_I2HM(BFI_ABLK_H2I_NWPAR_QUERY),
	BFI_ABLK_I2H_NWPAR_PF_ENBL	= BFA_I2HM(BFI_ABLK_H2I_NWPAR_PF_ENBL),
	BFI_ABLK_I2H_NWPAR_PF_DSBL	= BFA_I2HM(BFI_ABLK_H2I_NWPAR_PF_DSBL),
	BFI_ABLK_I2H_NWPAR_PF_PERS	= BFA_I2HM(BFI_ABLK_H2I_NWPAR_PF_PERS),
};
typedef enum bfi_ablk_i2h_msgs_e bfi_ablk_i2h_msgs_t;

/* BFI_ABLK_H2I_QUERY */
struct bfi_ablk_h2i_query_s {
	struct bfi_mhdr_s	mh;
	bfi_addr_t		addr;
};
typedef struct bfi_ablk_h2i_query_s bfi_ablk_h2i_query_t;

/* BFI_ABL_H2I_ADPT_CONFIG, BFI_ABLK_H2I_PORT_CONFIG */
struct bfi_ablk_h2i_cfg_req_s {
	struct bfi_mhdr_s	mh;
	uint8_t			mode;
	uint8_t			port;
	uint8_t			max_pf;
	uint8_t			max_vf;
};
typedef struct bfi_ablk_h2i_cfg_req_s bfi_ablk_h2i_cfg_req_t;

/**
 * BFI_ABLK_H2I_PF_CREATE, BFI_ABLK_H2I_PF_DELETE,
 * BFI_ABLK_H2I_PF_UPDATE
 */
struct bfi_ablk_h2i_pf_req_s {
	struct bfi_mhdr_s	mh;
	uint8_t			pcifn;
	uint8_t			port;
	uint16_t		pers;
	uint16_t		bw_min; /* percent BW @ max speed */
	uint16_t		bw_max; /* percent BW @ max speed */
};
typedef struct bfi_ablk_h2i_pf_req_s bfi_ablk_h2i_pf_req_t;

/**
 * BFI_ABLK_H2I_NWPAR_ENABLE, BFI_ABLK_H2I_NWPAR_DISABLE
 */
struct bfi_ablk_h2i_nwpar_req_s {
	struct bfi_mhdr_s	mh;
};
typedef struct bfi_ablk_h2i_nwpar_req_s bfi_ablk_h2i_nwpar_req_t;

/* BFI_ABLK_H2I_OPTROM_ENABLE, BFI_ABLK_H2I_OPTROM_DISABLE */
struct bfi_ablk_h2i_optrom_s {
	struct bfi_mhdr_s	mh;
};
typedef struct bfi_ablk_h2i_optrom_s bfi_ablk_h2i_optrom_t;

/**
 * BFI_ABLK_I2H_QUERY
 * BFI_ABLK_I2H_PORT_CONFIG
 * BFI_ABLK_I2H_PF_CREATE
 * BFI_ABLK_I2H_PF_DELETE
 * BFI_ABLK_I2H_PF_UPDATE
 * BFI_ABLK_I2H_OPTROM_ENABLE
 * BFI_ABLK_I2H_OPTROM_DISABLE
 */
struct bfi_ablk_i2h_rsp_s {
	struct bfi_mhdr_s	mh;
	uint8_t			status;
	uint8_t			pcifn;
	uint8_t			port_mode;	/*!< bfa_mode_t		*/
};
typedef struct bfi_ablk_i2h_rsp_s bfi_ablk_i2h_rsp_t;

/**
 *----------------------------------------------------------------------
 *				EDMA
 *----------------------------------------------------------------------
 */

#define BFI_EDMA_READ_SZ 28

typedef enum {
	BFI_EDMA_H2I_READ_RSP = 1,
} bfi_edma_h2i_t;

typedef enum {
	BFI_EDMA_I2H_READ_REQ = BFA_I2HM(BFI_EDMA_H2I_READ_RSP),
} bfi_edma_i2h_t;

/* BFI_EDMA_H2I_READ_RSP */
typedef struct {
	struct	bfi_mhdr_s	mh;
	uint8_t			data[BFI_EDMA_READ_SZ];
} bfi_edma_h2i_read_rsp_t;

/* BFI_EDMA_I2H_READ_REQ */
typedef struct {
	struct	bfi_mhdr_s	mh;
	uint32_t		addr_lo;
	uint32_t		addr_hi;
	uint32_t		len;
} bfi_edma_i2h_read_req_t;

#pragma pack()

#endif /* __BFI_H__ */
