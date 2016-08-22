/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BFA_DIAG_H__
#define __BFA_DIAG_H__

#include <defs/bfa_defs.h>
#include <cna/bfa_ioc.h>
#include <cs/bfa_cs.h>

typedef void (*bfa_cb_diag_t) (void *cbarg, bfa_status_t status);
typedef void (*bfa_cb_diag_beacon_t) (void *dev, bfa_boolean_t beacon,
		bfa_boolean_t link_e2e_beacon);

/**
 * DIAG data structures
 */

/**
 * Firmware ping test results
 */
typedef struct {
	bfa_diag_fwping_result_t *result;
	bfa_cb_diag_t  cbfn;
	void		*cbarg;
	uint32_t	data;
	uint8_t		lock;
	uint8_t		rsv[3];
	uint32_t	status;
	uint32_t	count;
	struct bfa_mbox_cmd_s	mbcmd;
	uint8_t		*dbuf_kva;	/*!< dma buf virtual address */
	uint64_t	dbuf_pa;	/*!< dma buf physical address */
} bfa_diag_fwping_t;

/*
 * temp sensor has it's own cbfn cause it can co-exist with other diag cmd
 */
typedef struct {
	bfa_cb_diag_t	cbfn;
	void		*cbarg;
	bfa_diag_temp_t *temp;
	uint8_t		lock;
	uint8_t		rsv[3];
	uint32_t	status;
	struct bfa_mbox_cmd_s	mbcmd;
} bfa_diag_tsensor_t;

typedef struct {
	sfp_mem_t	*sfpmem;
	bfa_cb_diag_t	cbfn;
	void		*cbarg;
	uint8_t		lock;
	uint8_t		static_data;
	uint8_t		rsv[2];
	uint32_t	status;
	struct bfa_mbox_cmd_s	mbcmd;
	uint8_t		*dbuf_kva;	/*!< dma buf virtual address */
	uint64_t	dbuf_pa;	/*!< dma buf physical address */
} bfa_diag_sfpshow_t;

typedef struct {
	struct bfa_mbox_cmd_s	mbcmd;
	bfa_boolean_t	lock;		/*!< 1: ledtest is operating */
} bfa_diag_led_t;

typedef struct {
	struct bfa_mbox_cmd_s	mbcmd;
	bfa_boolean_t   state;		/*!< port beacon state */
	bfa_boolean_t   link_e2e;	/*!< link beacon state */
} bfa_diag_beacon_t;

struct bfa_diag_s {
	void *dev;
	struct bfa_ioc_s  *ioc;
	bfa_trc_mod_t	  *trcmod;
	bfa_diag_fwping_t fwping;
	bfa_diag_tsensor_t tsensor;
	bfa_diag_sfpshow_t sfpshow;
	bfa_diag_led_t ledtest;
	bfa_diag_beacon_t beacon;
	void		*result;		/* store result pointer */
	struct bfa_timer_s timer;
	bfa_cb_diag_beacon_t  cbfn_beacon;
	bfa_cb_diag_t  cbfn;
	void		*cbarg;
	uint8_t		block;
	uint8_t		timer_active;
	uint8_t		rsvd[2];
	uint32_t	status;
	bfa_ioc_notify_t	ioc_notify;
	struct bfa_mem_dma_s	diag_dma;
};

typedef struct bfa_diag_s	bfa_diag_mod_t;
#define BFA_DIAG_MOD(__bfa)     (&(__bfa)->modules.diag_mod)
#define BFA_MEM_DIAG_DMA(__bfa)	(&(BFA_DIAG_MOD(__bfa)->diag_dma))

/*
 * bfa hal diag API functions
 */
bfa_status_t    bfa_diag_reg_read(struct bfa_diag_s *diag, uint32_t offset,
				uint32_t len, uint32_t *buf, uint32_t force);

bfa_status_t    bfa_diag_reg_write(struct bfa_diag_s *diag, uint32_t offset,
				uint32_t len, uint32_t value, uint32_t force);

bfa_status_t    bfa_diag_tsensor_query(struct bfa_diag_s *diag,
				       bfa_diag_temp_t *result,
					   bfa_cb_diag_t cbfn, void *cbarg);

bfa_status_t    bfa_diag_fwping(struct bfa_diag_s *diag, uint32_t cnt,
			uint32_t pattern, bfa_diag_fwping_result_t *result,
			bfa_cb_diag_t cbfn, void *cbarg);

bfa_status_t    bfa_diag_sfpshow(struct bfa_diag_s *diag, sfp_mem_t *sfpmem,
				     uint8_t static_data, bfa_cb_diag_t cbfn,
				     void *cbarg);

bfa_status_t    bfa_diag_memtest(struct bfa_diag_s *diag,
				     bfa_diag_memtest_t *memtest,
				     uint32_t pattern,
				     bfa_diag_memtest_result_t *result,
				     bfa_cb_diag_t cbfn, void *cbarg);

bfa_status_t    bfa_diag_ledtest(struct bfa_diag_s *diag,
				     bfa_diag_ledtest_t *ledtest);

bfa_status_t    bfa_diag_beacon_port(struct bfa_diag_s *diag,
			 bfa_boolean_t beacon,
			bfa_boolean_t link_e2e_beacon,
			 uint32_t sec);

/**
 * New diag common interface -- TBD
 */
struct bfa_diag_mod;
uint32_t bfa_diag_meminfo(void);
void bfa_diag_attach(struct bfa_diag_s *diag, struct bfa_ioc_s *ioc, void *dev,
	bfa_cb_diag_beacon_t cbfn_beacon, bfa_trc_mod_t *trcmod);
void bfa_diag_memclaim(struct bfa_diag_s *diag, uint8_t *dm_kva,
	uint64_t dm_pa);
void bfa_diag_intr(void *bfaarg, struct bfi_mbmsg_s *msg);

#endif /* __BFA_DIAG_H__ */
