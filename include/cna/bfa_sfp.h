/*
 * Copyright (c) 2009-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BFA_SFP_H__
#define __BFA_SFP_H__

#include <defs/bfa_defs.h>
#include <bfi/bfi.h>
#include <cna/bfa_ioc.h>
#include <cs/bfa_cs.h>


typedef void    (*bfa_cb_sfp_t) (void *cbarg, bfa_status_t status);

struct bfa_sfp_s {
	void			*dev;
	struct bfa_ioc_s	*ioc;
	struct bfa_trc_mod_s	*trcmod;
	sfp_mem_t		*sfpmem; /*!< store pointer pass from ioctl */
	bfa_cb_sfp_t		cbfn;
	void			*cbarg;
	bfi_sfp_mem_t		memtype; /*!< mem access type   */
	uint32_t		status;
	struct bfa_mbox_cmd_s	mbcmd;
	uint8_t			*dbuf_kva;	/*!< dma buf virtual address */
	uint64_t		dbuf_pa;	/*!< dma buf physical address */
	bfa_ioc_notify_t	ioc_notify;
	bfa_defs_sfp_media_t	*media;
	bfa_port_speed_t	portspeed;
	bfa_cb_sfp_t		state_query_cbfn;
	void			*state_query_cbarg;
	uint8_t			lock;
	uint8_t			data_valid;	/*!< data in dbuf is valid */
	uint8_t			state;		/*!< sfp state  */
	uint8_t			state_query_lock;
	struct bfa_mem_dma_s	sfp_dma;
	uint8_t			is_elb;		/*!< eloopback  */
};

typedef struct bfa_sfp_s	bfa_sfp_t;
#define BFA_SFP_MOD(__bfa)	(&(__bfa)->modules.sfp)
#define BFA_MEM_SFP_DMA(__bfa)	(&(BFA_SFP_MOD(__bfa)->sfp_dma))

uint32_t 	bfa_sfp_meminfo(void);

void 		bfa_sfp_attach(struct bfa_sfp_s *sfp, struct bfa_ioc_s *ioc,
			       void *dev, bfa_trc_mod_t *trcmod);

void 		bfa_sfp_memclaim(struct bfa_sfp_s *diag, uint8_t *dm_kva,
				 uint64_t dm_pa);

void 		bfa_sfp_intr(void *bfaarg, struct bfi_mbmsg_s *msg);

bfa_status_t	bfa_sfp_show(struct bfa_sfp_s *sfp, sfp_mem_t *sfpmem,
			     bfa_cb_sfp_t cbfn, void *cbarg);

bfa_status_t	bfa_sfp_media(struct bfa_sfp_s *sfp,
			      bfa_defs_sfp_media_t *media,
			      bfa_cb_sfp_t cbfn, void *cbarg);

bfa_status_t	bfa_sfp_speed(struct bfa_sfp_s *sfp,
			      bfa_port_speed_t portspeed,
			      bfa_cb_sfp_t cbfn, void *cbarg);

#endif /* __BFA_SFP_H__ */
