/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BNAD_DIAG_H__
#define __BNAD_DIAG_H__

#define BNAD_DIAG_MAC_ADDR_LEN	6
#define	BNAD_DIAG_Q_DEPTH	1024
#define	BNAD_DIAG_UNMAPQ_DEPTH	BNAD_DIAG_Q_DEPTH

#define BNAD_DIAG_RX_MAX_RETRIES  16

#define BNAD_DIAG_STATUS_COMPLETE 1
#define BNAD_DIAG_USLEEP_TIME		5000 /* in micro seconds */
#define BNAD_DIAG_TOTAL_WAIT_LOOP	1000 /* Tot wait 1000*5000 usec=5 sec */

#define BNAD_DIAG_ETH_MTU	1500
#define BNAD_DIAG_ETH_HLEN	14
#define BNAD_DIAG_ETH_FCS_LEN 	4
#define BNAD_DIAG_ETH_P_LOOPBACK 0x0060
#define BNAD_DIAG_ETH_ALEN	6 /* MAC Addr Len in bytes */

/* Diag Run Flags : Bit-mask value */
/* More than one bit can be set */
#define BNAD_DIAG_RF_TX_STALLED		0x01
#define BNAD_DIAG_RF_RX_CLEANED		0x02

struct bnad_diag_eth_hdr_s {
	mac_t		dst;
	mac_t		src;
	uint16_t	proto;
};

struct bnad_diag_pkt_pload_s {
	struct bnad_diag_eth_hdr_s	eth_hdr;
	uint32_t			seq_num;
};

struct bnad_diag_pkt_hdr_s {
	struct bna_dma_addr_s	dma;
	uint32_t		len; /* Data Len */
	uint8_t			free;
};

struct bnad_diag_pkt_s {
	struct bnad_diag_pkt_hdr_s	hdr;
	void				*data;
};

struct bnad_diag_unmap_q_s {
	uint32_t		producer_index;
	uint32_t		consumer_index;
	uint32_t 		q_depth;
	struct bnad_diag_pkt_s	unmap_array[1];
};

struct bnad_diag_tx_info_s {
	struct bna_res_info_s	res_info[BNA_TX_RES_T_MAX];
	struct bna_mem_info_s	mem_info; /* For unmap array */
	struct bna_tx_config_s	tx_cfg;
	struct bna_tx_s 	*tx;
	struct bna_tcb_s	*tcb;
};

struct bnad_diag_rx_info_s {
	struct bna_res_info_s	res_info[BNA_RX_RES_T_MAX];
	struct bna_mem_info_s	mem_info; /* For unmap array */
	struct bna_rx_config_s	rx_cfg;
	struct bna_rx_s		*rx;
	struct bna_ccb_s	*ccb;
};

struct bnad_diag_completion_s {
	uint8_t			comp_done;
	uint32_t		comp_status;
};

struct bnad_diag_s {
	struct bnad_diag_tx_info_s	tx_info;
	struct bnad_diag_rx_info_s	rx_info;

	mac_t				mac_addr;

	uint32_t			run_flags;

	enum bna_link_status_e		link_status;

	struct bnad_diag_completion_s	comp;
	struct bnad_diag_completion_s	link_comp;

	uint32_t			snd_seq;
	uint32_t			rcv_seq;

	uint32_t			rx_drop_cnt;
	uint32_t			tx_fail_cnt;

	/* Previous configuration that needs to be restored */
	struct bna_pause_config_s	p_cfg;
	uint32_t			mtu;
	uint32_t			prior_enet_flags;
	uint32_t			prior_ethport_flags;
};

#endif /* __BNAD_DIAG_H__ */
