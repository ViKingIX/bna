/*
 * Copyright (c)  2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c)  2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __SFP_H__
#define __SFP_H__

#include <protocol/bfa_fc.h>

#pragma pack(1)

#define SFP_SERIAL_ID_ADDR	0xA0
#define SFP_DIGI_DIAG_ADDR	0xA2
#define	SFP_DEV_CHG_ADDR	0x04

/*
 * Address for Small Form-factor Pluggable (SFP) Transceiver.
 */
#define	SFP_ID			00
#define	SFP_ID_EXT		01
#define	SFP_CONNECTOR		02
#define	SFP_TX			03
#define	SFP_TX_1		07
#define	SFP_ENCODING		11
#define	SFP_BR_NORM		12
#define	SFP_LENGTH_KM		14
#define	SFP_LENGTH_9M		15
#define	SFP_LENGTH_50M		16
#define	SFP_LENGTH_62_5M	17
#define	SFP_LENGTH_COPPER	18
#define	SFP_VENDOR_NAME		20
#define	SFP_VENDOR_NAME_1	24
#define	SFP_VENDOR_NAME_2	28
#define	SFP_VENDOR_NAME_3	32
#define	SFP_VENDOR_OUI		37
#define SFP_VENDOR_PN		40
#define SFP_VENDOR_PN_1		44
#define SFP_VENDOR_PN_2		48
#define SFP_VENDOR_PN_3		52
#define SFP_VENDOR_REV		56
#define SFP_CC_BASE		63
#define	SFP_OPTIONS		64
#define	SFP_BR_MAX		66
#define	SFP_BR_MIN		67
#define	SFP_VENDOR_SN		68
#define	SFP_VENDOR_SN_1		72
#define	SFP_VENDOR_SN_2		76
#define	SFP_VENDOR_SN_3		80
#define	SFP_DATE_CODE		84
#define	SFP_DATE_CODE_1		88
#define	SFP_DIAG_MON_TYPE	92
#define	SFP_ENH_OPT		93
#define	SFP_SFF8472_COMP	94
#define SFP_CC_EXT		95

/*
 * SFF-8472
 * Diagnostics: Data Fields - Address A2h
 */
#define SFP_THRESHOLD_START	00
#define	SFP_TEMP_HIGH_ALARM	00
#define	SFP_TEMP_LOW_ALARM	02
#define	SFP_TEMP_HIGH_WARN	04
#define	SFP_TEMP_LOW_WARN	06
#define	SFP_VOL_HIGH_ALARM	08
#define	SFP_VOL_LOW_ALARM	10
#define	SFP_VOL_HIGH_WARN	12
#define	SFP_VOL_LOW_WARM	14
#define	SFP_BIAS_HIGH_ALARM	16
#define	SFP_BIAS_LOW_ALARM	18
#define	SFP_BIAS_HIGH_WARN	20
#define	SFP_BIAS_LOW_WARN	22
#define	SFP_TX_PWR_HIGH_ALARM	24
#define	SFP_TX_PWR_LOW_ALARM	26
#define	SFP_TX_PWR_HIGH_WARN	28
#define	SFP_TX_PWR_LOW_WARN	30
#define	SFP_RX_PWR_HIGH_ALARM	32
#define	SFP_RX_PWR_LOW_ALARM	34
#define	SFP_RX_PWR_HIGH_WARN	36
#define	SFP_RX_PWR_LOW_WARN	38
#define SFP_THRESHOLD_END	40

#define XFP_AUX1_HIGH_ALARM	42
#define XFP_AUX1_LOW_ALARM	44
#define XFP_AUX1_HIGH_WARN	46
#define XFP_AUX1_LOW_WARN	48
#define XFP_AUX2_HIGH_ALARM	50
#define XFP_AUX2_LOW_ALARM	52
#define XFP_AUX2_HIGH_WARN	54
#define XFP_AUX2_LOW_WARN	56

#define	SFP_RX_PWR_4		56
#define	SFP_RX_PWR_3		60
#define	SFP_RX_PWR_2		64
#define	SFP_RX_PWR_1		68
#define	SFP_RX_PWR_0		72
#define	SFP_TX_I_SLOPE		76
#define	SFP_TX_I_OFFSET		78
#define	SFP_TX_I_PWR_SLOPE	80
#define	SFP_TX_I_PWR_OFFSET	82
#define	SFP_T_SLOPE		84
#define	SFP_T_OFFSET		86
#define	SFP_V_SLOPE		84
#define	SFP_V_OFFSET		86
#define	SFP_CHECKSUM		95

#define	SFP_TEMP_MSB		96
#define	SFP_TEMP_LSB		97
#define	SFP_VCC_MSB		98
#define	SFP_VCC_LSB		99
#define	SFP_TX_BIAS_MSB		100
#define	SFP_TX_BIAS_LSB		101
#define	SFP_TX_PWR_MSB		102
#define	SFP_TX_PWR_LSB		103
#define	SFP_RX_PWR_MSB		104
#define	SFP_RX_PWR_LSB		105
#define XFP_AUX1_MSB		106
#define XFP_AUX1_LSB		107
#define XFP_AUX2_MSB		108
#define	XFP_AUX2_LSB		109
#define	SFP_OPTIONAL_CSR	110
#define	SFP_OPTIONAL_ALARM_1	112
#define	SFP_OPTIONAL_ALARM_2	113
#define	SFP_OPTIONAL_WARN_1	116
#define	SFP_OPTIONAL_WARN_2	117
#define SFP_USER_EEPROM		128

#define SFP_DMT_DIGIDIAGMON			0x40
#define	SFP_EOPTION_AWFLAGS			0x80
#define	SFP_DIAGMON_SIZE	10 /* no of bytes of diag monitor data */
#define SFP_DIAGMON_AWFLAGS_MSK		0xFFC0

/*
 * Serial ID: Data Fields -- Address A0h
 * Basic ID field total 64 bytes
 */
typedef struct {
	uint8_t         id;		/* 00: Identifier */
	uint8_t         extid;		/* 01: Extended Identifier */
	uint8_t         connector;	/* 02: Connector */
	uint8_t         xcvr[8];	/* 03-10: Transceiver */
	uint8_t         encoding;	/* 11: Encoding */
	uint8_t         br_norm;	/* 12: BR, Nominal */
	uint8_t         rate_id;	/* 13: Rate Identifier */
	uint8_t         len_km;		/* 14: Length single mode km */
	uint8_t         len_100m;	/* 15: Length single mode 100m */
	uint8_t         len_om2;	/* 16: Length om2 fiber 10m */
	uint8_t         len_om1;	/* 17: Length om1 fiber 10m */
	uint8_t         len_cu;		/* 18: Length copper 1m */
	uint8_t         len_om3;	/* 19: Length om3 fiber 10m */
	uint8_t         vendor_name[16];/* 20-35 */
	uint8_t         unalloc1;
	uint8_t         vendor_oui[3];	/* 37-39 */
	uint8_t         vendor_pn[16];	/* 40-55 */
	uint8_t         vendor_rev[4];	/* 56-59 */
	uint8_t         wavelen[2];	/* 60-61 */
	uint8_t         unalloc2;
	uint8_t         cc_base;	/* 63: check code for base id field */
} sfp_srlid_base_t;

/*
 * Serial ID: Data Fields -- Address A0h
 * Extended id field total 32 bytes
 */
typedef struct {
	uint8_t         options[2];
	uint8_t         br_max;
	uint8_t         br_min;
	uint8_t         vendor_sn[16];
	uint8_t         date_code[8];
	uint8_t         diag_mon_type;	/* 92: Diagnostic Monitoring type */
	uint8_t         en_options;
	uint8_t         sff_8472;
	uint8_t         cc_ext;
} sfp_srlid_ext_t;

/*
 * Diagnostic: Data Fields -- Address A2h
 * Diagnostic and control/status base field total 96 bytes
 */
typedef struct {
	/*
	 * Alarm and warning Thresholds 40 bytes
	 */
	uint8_t         temp_high_alarm[2];	/* 00-01 */
	uint8_t         temp_low_alarm[2];	/* 02-03 */
	uint8_t         temp_high_warning[2];	/* 04-05 */
	uint8_t         temp_low_warning[2];	/* 06-07 */

	uint8_t         volt_high_alarm[2];	/* 08-09 */
	uint8_t         volt_low_alarm[2];	/* 10-11 */
	uint8_t         volt_high_warning[2];	/* 12-13 */
	uint8_t         volt_low_warning[2];	/* 14-15 */

	uint8_t         bias_high_alarm[2];	/* 16-17 */
	uint8_t         bias_low_alarm[2];	/* 18-19 */
	uint8_t         bias_high_warning[2];	/* 20-21 */
	uint8_t         bias_low_warning[2];	/* 22-23 */

	uint8_t         tx_pwr_high_alarm[2];	/* 24-25 */
	uint8_t         tx_pwr_low_alarm[2];	/* 26-27 */
	uint8_t         tx_pwr_high_warning[2];	/* 28-29 */
	uint8_t         tx_pwr_low_warning[2];	/* 30-31 */

	uint8_t         rx_pwr_high_alarm[2];	/* 32-33 */
	uint8_t         rx_pwr_low_alarm[2];	/* 34-35 */
	uint8_t         rx_pwr_high_warning[2];	/* 36-37 */
	uint8_t         rx_pwr_low_warning[2];	/* 38-39 */

	uint8_t         unallocate_1[16];

	/*
	 * ext_cal_const[36]
	 */

	uint8_t         rx_pwr[20];
	uint8_t         tx_i[4];
	uint8_t         tx_pwr[4];
	uint8_t         temp[4];
	uint8_t         volt[4];
	uint8_t         unallocate_2[3];
	uint8_t         cc_dmi;
} sfp_diag_base_t;

/*
 * Diagnostic: Data Fields -- Address A2h
 * Diagnostic and control/status extended field total 24 bytes
 */
typedef struct {
	uint8_t         diag[SFP_DIAGMON_SIZE];
	uint8_t         unalloc1[4];
	uint8_t         status_ctl;
	uint8_t         rsvd;
	uint8_t         alarm_flags[2];
	uint8_t         unalloc2[2];
	uint8_t         warning_flags[2];
	uint8_t         ext_status_ctl[2];
} sfp_diag_ext_t;

/*
 * Diagnostic: Data Fields -- Address A2h
 * General Use Fields: User Writable Table - Features's Control Registers
 * Total 32 bytes
 */
typedef struct {
	uint8_t         rsvd1[2];	/* 128-129 */
	uint8_t         ewrap;		/* 130 */
	uint8_t         rsvd2[2];	/*  */
	uint8_t         owrap;		/* 133 */
	uint8_t         rsvd3[2];	/*  */
	uint8_t         prbs;		/* 136: PRBS 7 generator */
	uint8_t         rsvd4[2];	/*  */
	uint8_t         tx_eqz_16;	/* 139: TX Equalizer (16xFC) */
	uint8_t         tx_eqz_8;	/* 140: TX Equalizer (8xFC) */
	uint8_t         rsvd5[2];	/*  */
	uint8_t         rx_emp_16;	/* 143: RX Emphasis (16xFC) */
	uint8_t         rx_emp_8;	/* 144: RX Emphasis (8xFC) */
	uint8_t         rsvd6[2];	/*  */
	uint8_t         tx_eye_adj;	/* 147: TX eye Threshold Adjust */
	uint8_t         rsvd7[3];	/*  */
	uint8_t         tx_eye_qctl;	/* 151: TX eye Quality Control */
	uint8_t         tx_eye_qres;	/* 152: TX eye Quality Result */
	uint8_t         rsvd8[2];	/*  */
	uint8_t         poh[3];		/* 155-157: Power On Hours */
	uint8_t         rsvd9[2];	/*  */
} sfp_usr_eeprom_t;

typedef struct {
	sfp_srlid_base_t srlid_base;
	sfp_srlid_ext_t srlid_ext;
	sfp_diag_base_t diag_base;
	sfp_diag_ext_t  diag_ext;
	sfp_usr_eeprom_t usr_eeprom;
} sfp_mem_t;

/*
 * Alarm and Warning Flag Bits (SFF-8472 Rev 10.2 Table 3.18)
 */
typedef union {
	uint8_t         b;
	struct {
		uint8_t		temp_hi:1;		/* temperature exceed high threshold */
		uint8_t		temp_lo:1;		/* temperature below low threshold */
		uint8_t		volt_hi:1;		/* supply voltage exceed high threshold */
		uint8_t		volt_lo:1;		/* supply voltage below low threshold */
		uint8_t		tx_i_hi:1;		/* TX Bias current exceed high threshold */
		uint8_t		tx_i_lo:1;		/* TX Bias current below low threshold */
		uint8_t		tx_pwr_hi:1;	/* TX output power exceed high threshold */
		uint8_t		tx_pwr_lo:1;	/* TX output power below low threshold */
	} r;
} sfp_awf_lsb_t;

typedef union {
	uint8_t         b;
	struct {
		uint8_t		rx_pwr_hi:1;	/* RX output power exceed high threshold */
		uint8_t		rx_pwr_lo:1;	/* RX output power below low threshold */
		uint8_t		rsvd:6;
	} r;
} sfp_awf_msb_t;

/*
 * transceiver codes (SFF-8472 Rev 10.2 Table 3.5)
 */
typedef union {
	uint8_t         b;
	struct {
#ifdef __BIGENDIAN
		uint8_t		e10g_unall:1;	/* 10G Ethernet compliance */
		uint8_t		e10g_lrm:1;
		uint8_t		e10g_lr:1;
		uint8_t		e10g_sr:1;
		uint8_t		ib_sx:1;	/* Infiniband compliance */
		uint8_t		ib_lx:1;
		uint8_t		ib_cu_a:1;
		uint8_t		ib_cu_p:1;
#else
		uint8_t		ib_cu_p:1;
		uint8_t		ib_cu_a:1;
		uint8_t		ib_lx:1;
		uint8_t		ib_sx:1;	/* Infiniband compliance */
		uint8_t		e10g_sr:1;
		uint8_t		e10g_lr:1;
		uint8_t		e10g_lrm:1;
		uint8_t		e10g_unall:1;	/* 10G Ethernet compliance */
#endif
	} r;
} sfp_xcvr_e10g_code_t;

typedef union {
	uint8_t         b;
	struct {
		uint8_t         escon:2;	/* ESCON compliance code */
		uint8_t         oc192_reach:1;	/* SONET compliance code */
		uint8_t         so_reach:2;
		uint8_t         oc48_reach:3;
	} r;
} sfp_xcvr_so1_code_t;

typedef union {
	uint8_t         b;
	struct {
		uint8_t         reserved:1;
		uint8_t         oc12_reach:3;	/* OC12 reach */
		uint8_t         reserved1:1;
		uint8_t         oc3_reach:3;	/* OC3 reach */
	} r;
} sfp_xcvr_so2_code_t;

typedef union {
	uint8_t         b;
	struct {
		uint8_t         base_px:1;
		uint8_t         base_bx10:1;
		uint8_t         e100base_fx:1;
		uint8_t         e100base_lx:1;
		uint8_t         e1000base_t:1;
		uint8_t         e1000base_cx:1;
		uint8_t         e1000base_lx:1;
		uint8_t         e1000base_sx:1;
	} r;
} sfp_xcvr_eth_code_t;

typedef struct {
	uint8_t         link_len:5;	/* FC link length */
	uint8_t         xmtr_tech2:3;
	uint8_t         xmtr_tech1:7;	/* FC transmitter technology */
	uint8_t         reserved1:1;
} sfp_xcvr_fc1_code_t;

/*
 * values for link_len above
 */
enum {
	SFP_FC_LINKLEN_M = (1 << 0),	/* Medium distance */
	SFP_FC_LINKLEN_L = (1 << 1),	/* long distance */
	SFP_FC_LINKLEN_I = (1 << 2),	/* intermediate distance */
	SFP_FC_LINKLEN_S = (1 << 3),	/* short distance */
	SFP_FC_LINKLEN_V = (1 << 4)	/* very long distance */
};

/*
 * values for xmtr_tech above
 */
enum {
	SFP_XMTR_TECH_CU = (1 << 0),	/* copper FC-BaseT */
	SFP_XMTR_TECH_CP = (1 << 1),	/* copper passive */
	SFP_XMTR_TECH_CA = (1 << 2),	/* copper active */
	SFP_XMTR_TECH_LL = (1 << 3),	/* longwave laser */
	SFP_XMTR_TECH_SL = (1 << 4),	/* shortwave laser w/ OFC */
	SFP_XMTR_TECH_SN = (1 << 5),	/* shortwave laser w/o OFC */
	SFP_XMTR_TECH_EL_INTRA = (1 << 6),
					/* elec intra-enclosure */
	SFP_XMTR_TECH_EL_INTER = (1 << 7),
					/* elec inter-enclosure */
	SFP_XMTR_TECH_LC = (1 << 8),	/* longwave laser */
	SFP_XMTR_TECH_SA = (1 << 9)
};

typedef union {
	uint8_t         b;
	struct {
		uint8_t         tw_media:1;	/* twin axial pair (tw) */
		uint8_t         tp_media:1;	/* shielded twisted pair (sp) */
		uint8_t         mi_media:1;	/* miniature coax (mi) */
		uint8_t         tv_media:1;	/* video coax (tv) */
		uint8_t         m6_media:1;	/* multimode, 62.5m (m6) */
		uint8_t         m5_media:1;	/* multimode, 50m (m5) */
		uint8_t         reserved:1;
		uint8_t         sm_media:1;	/* single mode (sm) */
	} r;
} sfp_xcvr_fc2_code_t;

typedef union {
	uint8_t         b;
	struct {
#ifdef __BIGENDIAN
		uint8_t		rsv4:1;
		uint8_t		mb800:1;	/* 800 Mbytes/sec */
		uint8_t		mb1600:1; 	/* 1600 Mbytes/sec */
		uint8_t		mb400:1;	/* 400 Mbytes/sec */
		uint8_t		rsv2:1;
		uint8_t		mb200:1;	/* 200 Mbytes/sec */
		uint8_t		rsv1:1;
		uint8_t		mb100:1;	/* 100 Mbytes/sec */
#else
		uint8_t		mb100:1;	/* 100 Mbytes/sec */
		uint8_t		rsv1:1;
		uint8_t		mb200:1;	/* 200 Mbytes/sec */
		uint8_t		rsv2:1;
		uint8_t		mb400:1;	/* 400 Mbytes/sec */
		uint8_t		mb1600:1; 	/* 1600 Mbytes/sec */
		uint8_t		mb800:1;	/* 800 Mbytes/sec */
		uint8_t		rsv4:1;
#endif
	} r;
} sfp_xcvr_fc3_code_t;

typedef union {
	uint8_t         b;
	struct {
		uint8_t         gbase10_sr:1;	/* 400 Mbytes/sec */
		uint8_t         gbase10_lr:1;
		uint8_t         gbase10_er:1;
		uint8_t         reserved1:1;
		uint8_t         gbase10_sw:1;
		uint8_t         gbase10_lw:1;
		uint8_t         gbase10_ew:1;
		uint8_t         reserved2:1;
	} r;
} sfp_xcvr_10g_eth_code_t;

typedef union {
	uint8_t         b;
	struct {
		uint8_t         mx_sn_i_1200:1;	/* 1200-MX-SN-I compliance */
		uint8_t         sm_ll_l_1200:1;	/* 1200-SM-LL-L compliance */
		uint8_t         er_1550:1;	/* Extended reach 1550 nm */
		uint8_t         ir_1300:1;	/* Intermediate reach 1300 nm
						 * FP
						 */
		uint8_t         reserved:4;
	} r;
} sfp_xcvr_10g_fc_code_t;

typedef union {
	uint8_t         b;
	struct {
		uint8_t         xmtr_tech:4;
		uint8_t         wave_len_ctrl:1;
		uint8_t         tec:1;
		uint8_t         detector_type:1;
		uint8_t         tunable:1;
	} r;
} xfp_xcvr_xmtr_tech_t;

typedef struct {
	sfp_xcvr_e10g_code_t e10g;
	sfp_xcvr_so1_code_t so1;
	sfp_xcvr_so2_code_t so2;
	sfp_xcvr_eth_code_t eth;
	sfp_xcvr_fc1_code_t fc1;
	sfp_xcvr_fc2_code_t fc2;
	sfp_xcvr_fc3_code_t fc3;
} sfp_xcvr_t;

/*
 * Diagnostic Monitoring Type.
 */
typedef union {
	uint8_t         b;
	struct {
		uint8_t         legacy_rev:1;	/* Reserved. */
		uint8_t         diag_mon:1;	/* Diagnostic monitoring
						 * implemented
						 */
		uint8_t         int_calib:1;	/* Internally calibrated */
		uint8_t         ext_calib:1;	/* Externally calibrated */
		uint8_t         recv_pwr:1;	/* Received power measurement
						 * type
						 */
		uint8_t         addr_mode:1;	/* Addressing mode */
		uint8_t         reserved:2;	/* Reserved */
	} r;
} sfp_diag_mon_type_t;

#define	SFP_CU_PIGTAIL		0x21		/* Copper pigtail connector */
#define	SFP_LC				0x07		/* SFP LC connector */

#pragma pack()

#endif /* __SFP_H__ */
