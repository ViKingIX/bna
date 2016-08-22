/*
 * Copyright (c) 2010-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BFA_PHY_H__
#define __BFA_PHY_H__

#include <defs/bfa_defs.h>
#include <defs/bfa_defs_phy.h>
#include <bfa/bfa.h>

typedef void (*bfa_cb_phy_t) (void *cbarg, bfa_status_t status);

typedef struct bfa_phy_s {
	struct bfa_ioc_s *ioc;		/* !< back pointer to ioc */
	struct bfa_trc_mod_s *trcmod;	/* !< trace module */
	uint8_t		instance;	/* !< port instance */
	uint8_t		op_busy;	/* !< operation busy flag */
	uint8_t		rsv[2];
	uint32_t	residue;	/* !< residual length */
	uint32_t	offset;		/* !< offset */
	bfa_status_t    status;		/* !< status */
	uint8_t		*dbuf_kva;	/* !< dma buf virtual address */
	uint64_t	dbuf_pa;	/* !< dma buf physical address */
	struct bfa_reqq_wait_s reqq_wait; /* !< to wait for room in reqq */
	bfa_cb_phy_t	cbfn;		/* !< user callback function */
	void		*cbarg;		/* !< user callback arg */
	uint8_t		*ubuf;		/* !< user supplied buffer */
	struct bfa_cb_qe_s	hcb_qe;	/* !< comp: BFA callback qelem */
	uint32_t	addr_off;	/* !< phy address offset */
	struct bfa_mbox_cmd_s mb;	/* !< mailbox */
	struct bfa_ioc_notify_s ioc_notify; /* !< ioc event notify */
	struct bfa_mem_dma_s	phy_dma;
} bfa_phy_t;

#define BFA_PHY(__bfa)	(&(__bfa)->modules.phy)
#define BFA_MEM_PHY_DMA(__bfa) (&(BFA_PHY(__bfa)->phy_dma))

bfa_boolean_t bfa_phy_busy(struct bfa_ioc_s *ioc);
bfa_status_t bfa_phy_get_attr(struct bfa_phy_s *phy, uint8_t instance,
		bfa_phy_attr_t *attr,
		bfa_cb_phy_t cbfn, void *cbarg);
bfa_status_t bfa_phy_get_stats(struct bfa_phy_s *phy, uint8_t instance,
		bfa_phy_stats_t *stats,
		bfa_cb_phy_t cbfn, void *cbarg);
bfa_status_t bfa_phy_update(struct bfa_phy_s *phy, uint8_t instance,
		void *buf, uint32_t len, uint32_t offset,
		bfa_cb_phy_t cbfn, void *cbarg);
bfa_status_t bfa_phy_read(struct bfa_phy_s *phy, uint8_t instance,
		void *buf, uint32_t len, uint32_t offset,
		bfa_cb_phy_t cbfn, void *cbarg);

uint32_t bfa_phy_meminfo(bfa_boolean_t mincfg);
void bfa_phy_attach(struct bfa_phy_s *phy, struct bfa_ioc_s *ioc,
	void *dev, bfa_trc_mod_t *trcmod, bfa_boolean_t mincfg);
void bfa_phy_memclaim(struct bfa_phy_s *phy,
	uint8_t *dm_kva, uint64_t dm_pa, bfa_boolean_t mincfg);
void bfa_phy_intr(void *phyarg, struct bfi_mbmsg_s *msg);

#endif /* __BFA_PHY_H__ */
