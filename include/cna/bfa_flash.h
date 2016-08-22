/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BFA_FLASH_H__
#define __BFA_FLASH_H__

#include <defs/bfa_defs.h>
#include <defs/bfa_defs_flash.h>
#include <bfa/bfa.h>

typedef void    (*bfa_cb_flash_t) (void *cbarg, bfa_status_t status);

typedef struct bfa_flash_s {
	struct bfa_ioc_s *ioc;		/* !< back pointer to ioc */
	struct bfa_trc_mod_s *trcmod;
	uint32_t	type;		/* !< partition type */
	uint8_t		instance;	/* !< partition instance */
	uint8_t		rsv[3];
	uint32_t	op_busy;	/* !< operation busy flag */
	uint32_t	residue;	/* !< residual length */
	uint32_t	offset;		/* !< offset */
	bfa_status_t    status;		/* !< status */
	uint8_t		*dbuf_kva;	/* !< dma buf virtual address */
	uint64_t	dbuf_pa;	/* !< dma buf physical address */
	struct bfa_reqq_wait_s reqq_wait; /* !< to wait for room in reqq */
	bfa_cb_flash_t	cbfn;		/* !< user callback function */
	void		*cbarg;		/* !< user callback arg */
	uint8_t		*ubuf;		/* !< user supplied buffer */
	struct bfa_cb_qe_s	hcb_qe;	/* !< comp: BFA callback qelem */
	uint32_t	addr_off;	/* !< partition address offset */
	struct bfa_mbox_cmd_s mb;	/* !< mailbox */
	struct bfa_ioc_notify_s ioc_notify; /* !< ioc event notify */
	struct bfa_mem_dma_s	flash_dma;
} bfa_flash_t;

#define BFA_FLASH(__bfa)	(&(__bfa)->modules.flash)
#define BFA_MEM_FLASH_DMA(__bfa)	(&(BFA_FLASH(__bfa)->flash_dma))

bfa_status_t bfa_flash_get_attr(struct bfa_flash_s *flash,
		bfa_flash_attr_t *attr, bfa_cb_flash_t cbfn, void *cbarg);
bfa_status_t bfa_flash_erase_part(struct bfa_flash_s *flash,
		bfa_flash_part_type_t type, uint8_t instance,
		bfa_cb_flash_t cbfn, void *cbarg);
bfa_status_t bfa_flash_update_part(struct bfa_flash_s *flash,
		bfa_flash_part_type_t type, uint8_t instance, void *buf,
		uint32_t len, uint32_t offset,
		bfa_cb_flash_t cbfn, void *cbarg, uint8_t chk_offset);
bfa_status_t bfa_flash_read_part(struct bfa_flash_s *flash,
		bfa_flash_part_type_t type, uint8_t instance, void *buf,
		uint32_t len, uint32_t offset,
		bfa_cb_flash_t cbfn, void *cbarg, uint8_t chk_offset);
bfa_status_t bfa_flash_get_boot_version(struct bfa_flash_s *flash,
		void *buf, uint32_t len,
		bfa_cb_flash_t cbfn, void *cbarg);
bfa_status_t bfa_mfg_update(struct bfa_flash_s *flash,
	struct bfa_mfg_block_s *mfgblk, void *asic_block, int asic_blen,
	bfa_cb_flash_t cbfn, void *cbarg);

uint32_t bfa_flash_meminfo(bfa_boolean_t mincfg);
void bfa_flash_attach(struct bfa_flash_s *flash, struct bfa_ioc_s *ioc,
	void *dev, bfa_trc_mod_t *trcmod, bfa_boolean_t mincfg);
void bfa_flash_memclaim(struct bfa_flash_s *flash,
	uint8_t *dm_kva, uint64_t dm_pa, bfa_boolean_t mincfg);

#endif /* __BFA_FLASH_H__ */
