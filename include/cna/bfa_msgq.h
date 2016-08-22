/*
 * Copyright (c) 2009-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BFA_MSGQ_H__
#define __BFA_MSGQ_H__

#include <defs/bfa_defs.h>
#include <bfi/bfi.h>
#include <cna/bfa_ioc.h>
#include <cs/bfa_cs.h>

#define BFA_MSGQ_FREE_CNT(_q)						\
	(((_q)->consumer_index - (_q)->producer_index - 1) & ((_q)->depth - 1))

#define BFA_MSGQ_INDX_ADD(_q_indx, _qe_num, _q_depth)			\
	((_q_indx) = (((_q_indx) + (_qe_num)) & ((_q_depth) - 1)))


#define BFA_MSGQ_CMDQ_NUM_ENTRY		128
#define BFA_MSGQ_CMDQ_SIZE						\
	(BFI_MSGQ_CMD_ENTRY_SIZE * BFA_MSGQ_CMDQ_NUM_ENTRY)

#define BFA_MSGQ_RSPQ_NUM_ENTRY		128
#define BFA_MSGQ_RSPQ_SIZE						\
	(BFI_MSGQ_RSP_ENTRY_SIZE * BFA_MSGQ_RSPQ_NUM_ENTRY)

#define bfa_msgq_cmd_set(_cmd, _cbfn, _cbarg, _msg_size, _msg_hdr)	\
do {									\
	(_cmd)->cbfn = (_cbfn);						\
	(_cmd)->cbarg = (_cbarg);					\
	(_cmd)->msg_size = (_msg_size);					\
	(_cmd)->msg_hdr = (_msg_hdr);					\
} while (0)

struct bfa_msgq_s;

typedef void (*bfa_msgq_cmdcbfn_t)(void *cbarg, enum bfa_status status);

struct bfa_msgq_cmd_entry_s {
	bfa_q_t				qe;
	bfa_msgq_cmdcbfn_t		cbfn;
	void				*cbarg;
	size_t				msg_size;
	struct bfi_msgq_mhdr_s 		*msg_hdr;
};

enum bfa_msgq_cmdq_flags_e {
	BFA_MSGQ_CMDQ_F_DB_UPDATE	= 1,
};

struct bfa_msgq_cmdq_s {
	bfa_fsm_t			fsm;
	enum bfa_msgq_cmdq_flags_e	flags;

	uint16_t			producer_index;
	uint16_t			consumer_index;
	uint16_t			depth; /* FW Q depth is 16 bits */
	struct bfa_dma_s		addr;
	struct bfa_mbox_cmd_s		dbell_mb;

	uint16_t			token;
	int				offset;
	int				bytes_to_copy;
	struct bfa_mbox_cmd_s		copy_mb;

	bfa_q_t				pending_q; /* pending command queue */

	struct bfa_msgq_s 		*msgq;
};

enum bfa_msgq_rspq_flags_e {
	BFA_MSGQ_RSPQ_F_DB_UPDATE	= 1,
};

typedef void (*bfa_msgq_mcfunc_t)(void *cbarg, struct bfi_msgq_mhdr_s *mhdr);

struct bfa_msgq_rspq_s {
	bfa_fsm_t			fsm;
	enum bfa_msgq_rspq_flags_e	flags;

	uint16_t			producer_index;
	uint16_t			consumer_index;
	uint16_t			depth; /* FW Q depth is 16 bits */
	struct bfa_dma_s		addr;
	struct bfa_mbox_cmd_s		dbell_mb;

	int				nmclass;
	struct {
		bfa_msgq_mcfunc_t	cbfn;
		void			*cbarg;
	} rsphdlr[BFI_MC_MAX];

	struct bfa_msgq_s 		*msgq;
};

struct bfa_msgq_s {
	struct bfa_msgq_cmdq_s		cmdq;
	struct bfa_msgq_rspq_s		rspq;

	bfa_wc_t			init_wc;
	struct bfa_mbox_cmd_s		init_mb;

	struct bfa_ioc_notify_s		ioc_notify;
	struct bfa_ioc_s		*ioc;
	struct bfa_log_mod_s		*logm;
	struct bfa_trc_mod_s 		*trcmod;
};

uint32_t bfa_msgq_meminfo(void);
void bfa_msgq_memclaim(struct bfa_msgq_s *msgq, uint8_t *kva, uint64_t pa);
void bfa_msgq_attach(struct bfa_msgq_s *msgq, struct bfa_ioc_s *ioc,
		     struct bfa_trc_mod_s *trcmod,
		     struct bfa_log_mod_s *logmod);
void bfa_msgq_regisr(struct bfa_msgq_s *msgq, enum bfi_mclass mc,
		     bfa_msgq_mcfunc_t cbfn, void *cbarg);
void bfa_msgq_cmd_post(struct bfa_msgq_s *msgq,
		       struct bfa_msgq_cmd_entry_s *cmd);
void bfa_msgq_rsp_copy(struct bfa_msgq_s *msgq, uint8_t *buf, size_t buf_len);

#endif
