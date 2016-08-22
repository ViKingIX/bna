/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

/**
 * @file bfa_port.h Port common module header file.
 */

#ifndef __BFA_PORT_H__
#define __BFA_PORT_H__

#include <defs/bfa_defs_cna.h>
#include <cna/bfa_ioc.h>
#include <cs/bfa_cs.h>
#include <cs/bfa_log.h>

typedef void (*bfa_port_stats_cbfn_t) (void *dev, bfa_status_t status);
typedef void (*bfa_port_endis_cbfn_t) (void *dev, bfa_status_t status);

struct bfa_port_s {
	void				*dev;
	struct bfa_ioc_s		*ioc;
	struct bfa_trc_mod_s 		*trcmod;
	struct bfa_log_mod_s		*logmod;
	uint32_t			msgtag;
	bfa_boolean_t			stats_busy;
	struct bfa_mbox_cmd_s 		stats_mb;
	bfa_port_stats_cbfn_t		stats_cbfn;
	void				*stats_cbarg;
	bfa_status_t			stats_status;
	uint32_t			stats_reset_time;
	union bfa_port_stats_u		*stats;
	struct bfa_dma_s		stats_dma;
	bfa_boolean_t			endis_pending;
	struct bfa_mbox_cmd_s		endis_mb;
	bfa_port_endis_cbfn_t		endis_cbfn;
	void				*endis_cbarg;
	bfa_status_t			endis_status;
	bfa_ioc_notify_t		ioc_notify;
	bfa_boolean_t			pbc_disabled;
	bfa_boolean_t			dport_enabled;
	struct bfa_mem_dma_s	port_dma;
};
typedef struct bfa_port_s bfa_port_t;

#define BFA_MEM_PORT_DMA(__bfa)	(&((__bfa)->modules.port.port_dma))

void	     bfa_port_attach(struct bfa_port_s *port, struct bfa_ioc_s *ioc,
				void *dev, bfa_trc_mod_t *trcmod,
				bfa_log_mod_t *logmod);
void	     bfa_port_detach(struct bfa_port_s *port);
void	     bfa_port_notify(void *arg, enum bfa_ioc_event_e event);

bfa_status_t bfa_port_get_stats(struct bfa_port_s *port,
				 union bfa_port_stats_u *stats,
				 bfa_port_stats_cbfn_t cbfn, void *cbarg);
bfa_status_t bfa_port_clear_stats(struct bfa_port_s *port,
				   bfa_port_stats_cbfn_t cbfn, void *cbarg);
bfa_status_t bfa_port_enable(struct bfa_port_s *port,
			      bfa_port_endis_cbfn_t cbfn, void *cbarg);
bfa_status_t bfa_port_disable(struct bfa_port_s *port,
			       bfa_port_endis_cbfn_t cbfn, void *cbarg);
uint32_t     bfa_port_meminfo(void);
void	     bfa_port_mem_claim(struct bfa_port_s *port,
				 uint8_t *dma_kva, uint64_t dma_pa);
void		bfa_port_set_pbcdisabled(struct bfa_port_s *port);

void bfa_port_set_dportenabled(struct bfa_port_s *port, bfa_boolean_t enabled);

/**
 * CEE declaration
 */
typedef void (*bfa_cee_get_attr_cbfn_t) (void *dev, bfa_status_t status);
typedef void (*bfa_cee_get_stats_cbfn_t) (void *dev, bfa_status_t status);
typedef void (*bfa_cee_reset_stats_cbfn_t) (void *dev, bfa_status_t status);
typedef void (*bfa_cee_set_cfg_cbfn_t) (void *dev, bfa_status_t status);

struct bfa_cee_cbfn_s {
	bfa_cee_get_attr_cbfn_t    get_attr_cbfn;
	void *get_attr_cbarg;
	bfa_cee_get_stats_cbfn_t   get_stats_cbfn;
	void *get_stats_cbarg;
	bfa_cee_reset_stats_cbfn_t reset_stats_cbfn;
	void *reset_stats_cbarg;
	bfa_cee_set_cfg_cbfn_t	set_cfg_cbfn;
	void *set_cfg_cbarg;
};

struct bfa_cee_s {
	void *dev;
	bfa_boolean_t get_attr_pending;
	bfa_boolean_t get_stats_pending;
	bfa_boolean_t reset_stats_pending;
	bfa_status_t get_attr_status;
	bfa_status_t get_stats_status;
	bfa_status_t reset_stats_status;
	struct bfa_cee_cbfn_s cbfn;
	struct bfa_ioc_notify_s ioc_notify;
	struct bfa_trc_mod_s *trcmod;
	struct bfa_log_mod_s *logmod;
	struct bfa_cee_attr_s *attr;
	struct bfa_cee_stats_s *stats;
	struct bfa_cee_dcbx_cfg_s *cfg;
	struct bfa_dma_s attr_dma;
	struct bfa_dma_s stats_dma;
	struct bfa_dma_s cfg_dma;
	struct bfa_ioc_s *ioc;
	struct bfa_mbox_cmd_s get_cfg_mb;
	struct bfa_mbox_cmd_s set_cfg_mb;
	struct bfa_mbox_cmd_s get_stats_mb;
	struct bfa_mbox_cmd_s reset_stats_mb;
	struct bfa_mem_dma_s	cee_dma;
};

#define BFA_MEM_CEE_DMA(__bfa)	(&((__bfa)->modules.cee.cee_dma))

uint32_t bfa_cee_meminfo(void);
void bfa_cee_mem_claim(struct bfa_cee_s *cee, uint8_t *dma_kva,
	uint64_t dma_pa);
void bfa_cee_log_trc_set(struct bfa_cee_s *cee, struct bfa_trc_mod_s *trcmod,
	struct bfa_log_mod_s *logmod);
void bfa_cee_attach(struct bfa_cee_s *cee, struct bfa_ioc_s *ioc, void *dev);
void bfa_cee_detach(struct bfa_cee_s *cee);
bfa_status_t bfa_cee_get_attr(struct bfa_cee_s *cee,
	struct bfa_cee_attr_s *attr, bfa_cee_get_attr_cbfn_t cbfn, void *cbarg);
bfa_status_t bfa_cee_get_stats(struct bfa_cee_s *cee,
	struct bfa_cee_stats_s *stats, bfa_cee_get_stats_cbfn_t cbfn,
	void *cbarg);
bfa_status_t bfa_cee_reset_stats(struct bfa_cee_s *cee,
	bfa_cee_reset_stats_cbfn_t cbfn, void *cbarg);
bfa_status_t bfa_cee_set_cfg(struct bfa_cee_s *cee, bfa_cee_dcbx_cfg_t *cfg,
	bfa_cee_set_cfg_cbfn_t cbfn, void *cbarg);


#endif	/* __BFA_PORT_H__ */
