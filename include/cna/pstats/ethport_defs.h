/*
 * Copyright (c) 2006-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved.
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __ETHPORT_DEFS_H__
#define __ETHPORT_DEFS_H__

struct bnad_ethport_stats_s {
	/* Rx */
	uint64_t	rx_complete;	/* Rx completed */
	uint64_t	rx_drop;	/* Rx dropped */
	uint64_t	rx_alloc_fail;	/* Rx alloc failed */
	uint64_t	rx_cso_err;	/* Rx checksum errors */
	uint64_t	rx_mac_err;	/* Rx mac errors */
	uint64_t	rx_small_pkt;	/* Rx small packets */
	uint64_t	rx_large_pkt;	/* Rx large packets */
	uint64_t	rx_lro;		/* Rx lro */
	uint64_t	rx_lro_flush;	/* Rx lro flush */
	uint64_t	rx_low_rxbuf_count; /* Rx low rxbuf count */
	uint64_t	rx_schedule;	/* Rx Schedule */

	/* Tx */
	uint64_t	tx_lso4;	/* Tx lso4 */
	uint64_t	tx_lso6;	/* Tx lso6 */
	uint64_t	tx_lso_err;	/* Tx lso errors */
	uint64_t	tx_tcp_cso;	/* Tx tcp cso */
	uint64_t	tx_udp_cso;	/* Tx udp cso */
	uint64_t	tx_ip4_cso;	/* Tx ip4 cso */
	uint64_t	tx_csum_help;	/* Tx checksum help */
	uint64_t	tx_skb_too_short;	/* Tx skb too short */
	uint64_t	tx_skb_stopping;	/* Tx skb stopping */
	uint64_t	tx_skb_max_vectors;	/* Tx skb max vectors */
	uint64_t	tx_skb_mss_too_long;	/* Tx skb mss too long */
	uint64_t	tx_skb_tso_too_short;	/* Tx skb tso too short */
	uint64_t	tx_skb_tso_prepare;	/* Tx skb tso prepare */
	uint64_t	tx_skb_non_tso_too_long; /* Tx skb non tso too long */
	uint64_t	tx_skb_tcp_hdr;		/* Tx skb tcp header */
	uint64_t	tx_skb_udp_hdr;		/* Tx skb udp header */
	uint64_t	tx_skb_csum_err;	/* Tx skb cksum err */
	uint64_t	tx_skb_headlen_too_long; /* Tx skb headlen too long */
	uint64_t	tx_skb_headlen_zero;	/* Tx skb headlen zero */
	uint64_t	tx_skb_frag_zero;	/* Tx skb frag zero */
	uint64_t	tx_skb_len_mismatch;	/* Tx skb len mismatch */
	uint64_t	tx_map_err;	/* Tx map errors */
	uint64_t	tx_res_drop;	/* Tx res drops */
	uint64_t	tx_small_pkt;	/* Tx small packets */
	uint64_t	tx_large_pkt;	/* Tx large packets */
	uint64_t	tx_out_of_wis_count; /* Tx out of wis count */
	uint64_t	tx_wi_waitq_count; /* Tx wi waitq count */
	uint64_t	tx_ctxt_waitq_count; /* Tx ctxt waitq count */
	uint64_t	tx_max_nbs_per_nbl; /* Tx max nbs per nbl */

	/* Ctrl */
	uint64_t	link_toggle;	/* Link toggle count */
	uint64_t	cee_toggle;	/* CEE toggle count */
	uint64_t	mbox_intr_disable; /* Mbox intr disables */
	uint64_t	mbox_intr_enable; /* Mbox intr enables */
	uint64_t	tx_stop;	/* Tx stops */
	uint64_t	tx_wakeup;	/* Tx wakeups */
	uint64_t	tx_res_stop;	/* Tx res stops */
	uint64_t	tx_hw_stop;	/* Tx hardware stop */
	uint64_t	rx_hw_stop;	/* Rx hardware stops */
	uint64_t	rx_resume;	/* Rx resumes */
	uint64_t	rx_rss_config_count; /* Rx rss config count */
	uint64_t	fw_stats_query;	/*  FW stats query */
};
typedef struct bnad_ethport_stats_s bnad_ethport_stats_t;
#endif
