/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */
#ifndef __BFA_DEFS_MFG_COMM_H__
#define __BFA_DEFS_MFG_COMM_H__

#include <bfa_os_inc.h>
#include <defs/bfa_defs.h>

/**
 * Manufacturing block version
 */
#define BFA_MFG_VERSION				3
#define BFA_MFG_VERSION_UNINIT			0xFF

/**
 * Manufacturing block encrypted version
 */
#define BFA_MFG_ENC_VER				2

/**
 * Manufacturing block version 1 length
 */
#define BFA_MFG_VER1_LEN			128

/**
 * Manufacturing block header length
 */
#define BFA_MFG_HDR_LEN				4

#define BFA_MFG_SERIALNUM_SIZE			11
#define STRSZ(_n)				(((_n) + 4) & ~3)

/**
 * Manufacturing card type
 */
enum {
	BFA_MFG_TYPE_CB_MAX  = 825,      /*!< Crossbow card type max	*/
	BFA_MFG_TYPE_FC8P2   = 825,      /*!< 8G 2port FC card		*/
	BFA_MFG_TYPE_FC8P1   = 815,      /*!< 8G 1port FC card		*/
	BFA_MFG_TYPE_FC4P2   = 425,      /*!< 4G 2port FC card		*/
	BFA_MFG_TYPE_FC4P1   = 415,      /*!< 4G 1port FC card		*/
	BFA_MFG_TYPE_CNA10P2 = 1020,     /*!< 10G 2port CNA card	*/
	BFA_MFG_TYPE_CNA10P1 = 1010,     /*!< 10G 1port CNA card	*/
	BFA_MFG_TYPE_JAYHAWK = 804,	 /*!< Jayhawk mezz card		*/
	BFA_MFG_TYPE_WANCHESE = 1007,	 /*!< Wanchese mezz card	*/
	BFA_MFG_TYPE_ASTRA    = 807,	 /*!< Astra mezz card		*/
	BFA_MFG_TYPE_LIGHTNING_P0 = 902, /*!< Lightning mezz card - old	*/
	BFA_MFG_TYPE_LIGHTNING = 1741,	 /*!< Lightning mezz card	*/
	BFA_MFG_TYPE_PROWLER_F = 1560,	 /*!< Prowler FC only cards	*/
	BFA_MFG_TYPE_PROWLER_N = 1410,	 /*!< Prowler NIC only cards	*/
	BFA_MFG_TYPE_PROWLER_C = 1710,	 /*!< Prowler CNA only cards	*/
	BFA_MFG_TYPE_PROWLER_D = 1860,	 /*!< Prowler Dual cards	*/
	BFA_MFG_TYPE_CHINOOK   = 1867,	 /*!< Chinook cards		*/
	BFA_MFG_TYPE_CHINOOK2   = 1869,	 /*!< Chinook2 cards		*/
	BFA_MFG_TYPE_LIGHTNING2 = 1864,	 /*!< Lightning II mezz cards	*/
	BFA_MFG_TYPE_INVALID   = 0,	 /*!< Invalid card type		*/
};

#pragma pack(1)

#define bfa_mfg_get_cardtype(mfgblk)	mfgblk->card_type
#define bfa_mfg_get_wwn(mfgblk)			mfgblk->mfg_wwn
#define bfa_mfg_get_mac(mfgblk)			mfgblk->mfg_mac

/**
 * Check if Mezz card
 */
#define bfa_mfg_is_mezz(type) (( \
	(type) == BFA_MFG_TYPE_JAYHAWK || \
	(type) == BFA_MFG_TYPE_WANCHESE || \
	(type) == BFA_MFG_TYPE_ASTRA || \
	(type) == BFA_MFG_TYPE_LIGHTNING_P0 || \
	(type) == BFA_MFG_TYPE_LIGHTNING || \
	(type) == BFA_MFG_TYPE_CHINOOK || \
	(type) == BFA_MFG_TYPE_CHINOOK2 || \
	(type) == BFA_MFG_TYPE_LIGHTNING2))

/**
 * Check if card type valid
 */
#define bfa_mfg_is_card_type_valid(type) (( \
	(type) == BFA_MFG_TYPE_FC8P2 || \
	(type) == BFA_MFG_TYPE_FC8P1 || \
	(type) == BFA_MFG_TYPE_FC4P2 || \
	(type) == BFA_MFG_TYPE_FC4P1 || \
	(type) == BFA_MFG_TYPE_CNA10P2 || \
	(type) == BFA_MFG_TYPE_CNA10P1 || \
	(type) == BFA_MFG_TYPE_PROWLER_F || \
	(type) == BFA_MFG_TYPE_PROWLER_N || \
	(type) == BFA_MFG_TYPE_PROWLER_C || \
	(type) == BFA_MFG_TYPE_PROWLER_D || \
	bfa_mfg_is_mezz(type)))

/**
 * Check if the card having old wwn/mac handling
 */
#define bfa_mfg_is_old_wwn_mac_model(type) (( \
	(type) == BFA_MFG_TYPE_FC8P2 || \
	(type) == BFA_MFG_TYPE_FC8P1 || \
	(type) == BFA_MFG_TYPE_FC4P2 || \
	(type) == BFA_MFG_TYPE_FC4P1 || \
	(type) == BFA_MFG_TYPE_CNA10P2 || \
	(type) == BFA_MFG_TYPE_CNA10P1 || \
	(type) == BFA_MFG_TYPE_JAYHAWK || \
	(type) == BFA_MFG_TYPE_WANCHESE))

#define bfa_mfg_increment_wwn_mac(m, i)				\
do {								\
	uint32_t t = ((uint32_t)(m)[0] << 16) | ((uint32_t)(m)[1] << 8) | \
	  (uint32_t)(m)[2];	\
	t += (i);		\
	(m)[0] = (t >> 16) & 0xFF;				\
	(m)[1] = (t >> 8) & 0xFF;				\
	(m)[2] = t & 0xFF;					\
} while (0)

#define bfa_mfg_adapter_prop_init_flash_cb(mfgblk, prop)	\
do {								\
	switch ((mfgblk)->card_type) {				\
	case BFA_MFG_TYPE_FC8P2:				\
		(prop) = BFI_ADAPTER_SETP(NPORTS, 2) |		\
			BFI_ADAPTER_SETP(SPEED, 8);		\
		break;						\
	case BFA_MFG_TYPE_FC8P1:				\
		(prop) = BFI_ADAPTER_SETP(NPORTS, 1) |		\
			BFI_ADAPTER_SETP(SPEED, 8);		\
		break;						\
	case BFA_MFG_TYPE_FC4P2:				\
		(prop) = BFI_ADAPTER_SETP(NPORTS, 2) |		\
			BFI_ADAPTER_SETP(SPEED, 4);		\
		break;						\
	case BFA_MFG_TYPE_FC4P1:				\
		(prop) = BFI_ADAPTER_SETP(NPORTS, 1) |		\
			BFI_ADAPTER_SETP(SPEED, 4);		\
		break;						\
	default:						\
		(prop) = BFI_ADAPTER_UNSUPP;			\
	}							\
} while (0)

#define bfa_mfg_adapter_prop_init_flash_ct(mfgblk, prop)	\
do {								\
	switch ((mfgblk)->card_type) {				\
	case BFA_MFG_TYPE_JAYHAWK:				\
	case BFA_MFG_TYPE_ASTRA:				\
		(prop) = BFI_ADAPTER_SETP(NPORTS, 2) |		\
			BFI_ADAPTER_SETP(SPEED, 8);		\
		break;						\
	case BFA_MFG_TYPE_CNA10P2:				\
	case BFA_MFG_TYPE_WANCHESE:				\
	case BFA_MFG_TYPE_LIGHTNING_P0:				\
	case BFA_MFG_TYPE_LIGHTNING:				\
		(prop) = BFI_ADAPTER_SETP(NPORTS, 2) |		\
			BFI_ADAPTER_SETP(SPEED, 10);		\
		break;						\
	case BFA_MFG_TYPE_CNA10P1:				\
		(prop) = BFI_ADAPTER_SETP(NPORTS, 1) |		\
			BFI_ADAPTER_SETP(SPEED, 10);		\
		break;						\
	default:						\
		(prop) = BFI_ADAPTER_UNSUPP;			\
	}							\
} while (0)

#define bfa_mfg_adapter_prop_init_flash_ct2(mfgblk, prop)	\
do {								\
	int sp, np = (mfgblk)->mfg_nports;			\
	switch ((mfgblk)->card_type) {				\
	case BFA_MFG_TYPE_PROWLER_F:				\
	case BFA_MFG_TYPE_CHINOOK:				\
	case BFA_MFG_TYPE_CHINOOK2:				\
		sp = 16;					\
		break;						\
	case BFA_MFG_TYPE_PROWLER_N:				\
	case BFA_MFG_TYPE_PROWLER_C:				\
		sp = 10;					\
		break;						\
	case BFA_MFG_TYPE_LIGHTNING2:				\
	case BFA_MFG_TYPE_PROWLER_D:				\
		sp = ((mfgblk)->cap_hba == 'y' ||		\
			(mfgblk)->cap_fc16g == 'y') ? 16 : 10;	\
		break;						\
	default:						\
		sp = 0;						\
	}							\
	(prop) = BFI_ADAPTER_SETP(NPORTS, np) |			\
		BFI_ADAPTER_SETP(SPEED, sp);			\
} while (0)

enum {
	CB_GPIO_TTV	= (1),		/*!< TTV debug capable cards	*/
	CB_GPIO_FC8P2   = (2),		/*!< 8G 2port FC card		*/
	CB_GPIO_FC8P1   = (3),		/*!< 8G 1port FC card		*/
	CB_GPIO_FC4P2   = (4),		/*!< 4G 2port FC card		*/
	CB_GPIO_FC4P1   = (5),		/*!< 4G 1port FC card		*/
	CB_GPIO_DFLY    = (6),		/*!< 8G 2port FC mezzanine card	*/
	CB_GPIO_PROTO   = (1 << 7)	/*!< 8G 2port FC prototypes	*/
};

#define bfa_mfg_adapter_prop_init_gpio(gpio, card_type, prop)	\
do {								\
	if ((gpio) & CB_GPIO_PROTO) {				\
		(prop) |= BFI_ADAPTER_PROTO;			\
		(gpio) &= ~CB_GPIO_PROTO;			\
	}							\
	switch ((gpio)) {					\
	case CB_GPIO_TTV:					\
		(prop) |= BFI_ADAPTER_TTV;			\
	case CB_GPIO_DFLY:					\
	case CB_GPIO_FC8P2:					\
		(prop) |= BFI_ADAPTER_SETP(NPORTS, 2);		\
		(prop) |= BFI_ADAPTER_SETP(SPEED, 8);		\
		(card_type) = BFA_MFG_TYPE_FC8P2;		\
		break;						\
	case CB_GPIO_FC8P1:					\
		(prop) |= BFI_ADAPTER_SETP(NPORTS, 1);		\
		(prop) |= BFI_ADAPTER_SETP(SPEED, 8);		\
		(card_type) = BFA_MFG_TYPE_FC8P1;		\
		break;						\
	case CB_GPIO_FC4P2:					\
		(prop) |= BFI_ADAPTER_SETP(NPORTS, 2);		\
		(prop) |= BFI_ADAPTER_SETP(SPEED, 4);		\
		(card_type) = BFA_MFG_TYPE_FC4P2;		\
		break;						\
	case CB_GPIO_FC4P1:					\
		(prop) |= BFI_ADAPTER_SETP(NPORTS, 1);		\
		(prop) |= BFI_ADAPTER_SETP(SPEED, 4);		\
		(card_type) = BFA_MFG_TYPE_FC4P1;		\
		break;						\
	default:						\
		(prop) |= BFI_ADAPTER_UNSUPP;			\
		(card_type) = BFA_MFG_TYPE_INVALID;		\
	}							\
} while (0)

/**
 * VPD data length
 */
#define BFA_MFG_VPD_LEN			512
#define BFA_MFG_VPD_LEN_INVALID		0

#define BFA_MFG_VPD_PCI_HDR_OFF		137
#define BFA_MFG_VPD_PCI_VER_MASK	0x07	/*!< version mask 3 bits */
#define BFA_MFG_VPD_PCI_VDR_MASK	0xf8	/*!< vendor mask 5 bits */

/**
 * VPD vendor tag
 */
enum {
	BFA_MFG_VPD_UNKNOWN	= 0,     /*!< vendor unknown 		*/
	BFA_MFG_VPD_IBM 	= 1,     /*!< vendor IBM 		*/
	BFA_MFG_VPD_HP  	= 2,     /*!< vendor HP  		*/
	BFA_MFG_VPD_DELL  	= 3,     /*!< vendor DELL  		*/
	BFA_MFG_VPD_PCI_IBM 	= 0x08,  /*!< PCI VPD IBM     		*/
	BFA_MFG_VPD_PCI_HP  	= 0x10,  /*!< PCI VPD HP		*/
	BFA_MFG_VPD_PCI_DELL  	= 0x20,  /*!< PCI VPD DELL		*/
	BFA_MFG_VPD_PCI_BRCD 	= 0xf8,  /*!< PCI VPD QLogic 		*/
};

/**
 * @brief BFA adapter flash vpd data definition.
 *
 * All numerical fields are in big-endian format.
 */
struct bfa_mfg_vpd_s {
	uint8_t		version;	/*!< vpd data version */
	uint8_t		vpd_sig[3];	/*!< characters 'V', 'P', 'D' */
	uint8_t		chksum;		/*!< u8 checksum */
	uint8_t		vendor;		/*!< vendor */
	uint8_t 	len;		/*!< vpd data length excluding header */
	uint8_t 	rsv;
	uint8_t		data[BFA_MFG_VPD_LEN];	/*!< vpd data */
};

#pragma pack()

#endif /* __BFA_DEFS_MFG_H__ */
