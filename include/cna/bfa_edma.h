/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */
#ifndef __BFA_EDMA_H__
#define __BFA_EDMA_H__

#include <bfi/bfi_enet.h>
#include <cna/bfa_ioc.h>

struct bfa_edma_s {
	struct bfa_ioc_s	*ioc;
	struct bfa_trc_mod_s	*trcmod;
	uint8_t			*addr;
	uint32_t		bytes_to_copy;
	bfa_boolean_t		mbox_busy;
	struct bfa_mbox_cmd_s	read_mb;
};

typedef struct bfa_edma_s bfa_edma_t;

void bfa_edma_attach(bfa_edma_t *edma, struct bfa_ioc_s *ioc,
				struct bfa_trc_mod_s *trcmod);
#endif /* __BFA_EDMA_H__ */
