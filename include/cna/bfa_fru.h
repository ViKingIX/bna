/*
 * Copyright (c) 2010-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BFA_FRU_H__
#define __BFA_FRU_H__

#include <defs/bfa_defs.h>
#include <bfa/bfa.h>

typedef void (*bfa_cb_fru_t) (void *cbarg, bfa_status_t status);

typedef struct bfa_fru_s {
	struct bfa_ioc_s *ioc;		/* !< back pointer to ioc */
	struct bfa_trc_mod_s *trcmod;	/* !< trace module */
	uint8_t		op_busy;	/* !< operation busy flag */
	uint8_t		rsv[3];
	uint32_t	residue;	/* !< residual length */
	uint32_t	offset;		/* !< offset */
	bfa_status_t    status;		/* !< status */
	uint8_t		*dbuf_kva;	/* !< dma buf virtual address */
	uint64_t	dbuf_pa;	/* !< dma buf physical address */
	struct bfa_reqq_wait_s reqq_wait; /* !< to wait for room in reqq */
	bfa_cb_fru_t	cbfn;		/* !< user callback function */
	void		*cbarg;		/* !< user callback arg */
	uint8_t		*ubuf;		/* !< user supplied buffer */
	struct bfa_cb_qe_s	hcb_qe;	/* !< comp: BFA callback qelem */
	uint32_t	addr_off;	/* !< fru address offset */
	struct bfa_mbox_cmd_s mb;	/* !< mailbox */
	struct bfa_ioc_notify_s ioc_notify; /* !< ioc event notify */
	struct bfa_mem_dma_s	fru_dma;
	uint8_t		trfr_cmpl;
} bfa_fru_t;

#define BFA_FRU(__bfa)	(&(__bfa)->modules.fru)
#define BFA_MEM_FRU_DMA(__bfa) (&(BFA_FRU(__bfa)->fru_dma))

bfa_status_t bfa_fruvpd_update(struct bfa_fru_s *fru,
		void *buf, uint32_t len, uint32_t offset,
		bfa_cb_fru_t cbfn, void *cbarg, uint8_t trfr_cmpl);
bfa_status_t bfa_fruvpd_read(struct bfa_fru_s *fru,
		void *buf, uint32_t len, uint32_t offset,
		bfa_cb_fru_t cbfn, void *cbarg);

bfa_status_t bfa_fruvpd_get_max_size(struct bfa_fru_s *fru, uint32_t *max_size);

bfa_status_t bfa_tfru_write(struct bfa_fru_s *fru,
		void *buf, uint32_t len, uint32_t offset,
		bfa_cb_fru_t cbfn, void *cbarg);
bfa_status_t bfa_tfru_read(struct bfa_fru_s *fru,
		void *buf, uint32_t len, uint32_t offset,
		bfa_cb_fru_t cbfn, void *cbarg);

uint32_t bfa_fru_meminfo(bfa_boolean_t mincfg);
void bfa_fru_attach(struct bfa_fru_s *fru, struct bfa_ioc_s *ioc,
	void *dev, bfa_trc_mod_t *trcmod, bfa_boolean_t mincfg);
void bfa_fru_memclaim(struct bfa_fru_s *fru,
	uint8_t *dm_kva, uint64_t dm_pa, bfa_boolean_t mincfg);
void bfa_fru_intr(void *fruarg, struct bfi_mbmsg_s *msg);

#endif /* __BFA_FRU_H__ */
