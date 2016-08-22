/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BFA_DEFS_ETHPORT_H__
#define __BFA_DEFS_ETHPORT_H__

#include <defs/bfa_defs.h>
#include <defs/bfa_defs_svc.h>
#include <protocol/bfa_fc.h>
#include <cna/pstats/phyport_defs.h>
#include <cna/pstats/ethport_defs.h>

typedef struct {
	uint32_t    miniport_state;
	uint32_t    adapter_state;
	uint64_t    tx_count;
	uint64_t    tx_wi;
	uint64_t    tx_sg;
	uint64_t    tx_tcp_chksum;
	uint64_t    tx_udp_chksum;
	uint64_t    tx_ip_chksum;
	uint64_t    tx_lsov1;
	uint64_t    tx_lsov2;
	uint64_t    tx_max_sg_len;
} bna_tx_info_t;

typedef struct {
	uint16_t    q_id;
	uint16_t    buf_size;
	uint16_t    buf_count;
	uint16_t    rsvd;
	uint64_t    rx_count;
	uint64_t    rx_dropped;
	uint64_t    rx_unsupported;
	uint64_t    rx_internal_err;
	uint64_t    rss_count;
	uint64_t    vlan_count;
	uint64_t    rx_tcp_chksum;
	uint64_t    rx_udp_chksum;
	uint64_t    rx_ip_chksum;
	uint64_t    rx_hds;
} bna_rx_queue_info_t;

typedef struct {
	uint16_t    q_set_type;
	uint32_t    miniport_state;
	uint32_t    adapter_state;
	bna_rx_queue_info_t    rx_queue[2];
} bna_rx_q_set_t;

struct bna_port_stats_s {
	bna_tx_info_t	tx_stats;
	uint16_t	qset_count;
	bna_rx_q_set_t  rx_qset[8];
};

typedef struct bna_port_stats_s bna_port_stats_t;

struct bfa_ethport_stats_s {
	struct bna_stats_txf	txf_stats[1];
	struct bna_stats_rxf	rxf_stats[1];
	struct bnad_ethport_stats_s drv_stats;
};
typedef struct bfa_ethport_stats_s bfa_ethport_stats_t;

/**
 * Ethernet port events
 */
enum bfa_ethport_aen_event {
	BFA_ETHPORT_AEN_LINKUP = 1, /*!< Base Port Ethernet link up */
	BFA_ETHPORT_AEN_LINKDOWN = 2, /*!< Base Port Ethernet link down */
	BFA_ETHPORT_AEN_ENABLE = 3, /*!< Base Port Ethernet link enabled */
	BFA_ETHPORT_AEN_DISABLE = 4, /*!< Base Port Ethernet link disabled */
};
typedef enum bfa_ethport_aen_event bfa_ethport_aen_event_t;

enum bfa_ethport_cee_state {
	BFA_ETHPORT_CEE_LINKDOWN = 1,
	BFA_ETHPORT_CEE_LINKUP = 2
};
typedef enum bfa_ethport_cee_state bfa_ethport_cee_state_t;

/**
 * Ethernet port aen data struct
 */
struct bfa_ethport_aen_data_s {
	mac_t 		mac;	/*!< MAC address of the physical port */
	uint16_t	cee_state;
};
typedef struct bfa_ethport_aen_data_s bfa_ethport_aen_data_t;

/**
 * @brief
 * 		Ethernet port attribute values.
 */
struct bfa_ethport_attr_s {
	uint32_t mtu; /*!< maximum transfer unit */
	enum bfa_port_speed speed_supported; /*!< supported speeds */
	enum bfa_port_states port_state; /*!< current port state */
	enum bfa_port_speed speed; /*!< current speed */
};
typedef struct bfa_ethport_attr_s bfa_ethport_attr_t;

/**
 * Ethernet port checksumming state values
 */
enum bfa_ethport_csum_state_e {
	BFA_ETHPORT_CSUM_DISABLED = 0,
	BFA_ETHPORT_TX_CSUM_ENABLED,
	BFA_ETHPORT_RX_CSUM_ENABLED,
	BFA_ETHPORT_TXRX_CSUM_ENABLED
};
typedef enum bfa_ethport_csum_state_e bfa_ethport_csum_state_t;

/**
 * @brief
 * 		Ethernet port configuration
 */
struct bfa_ethport_cfg_s {
	bfa_boolean_t lro;
	bfa_boolean_t lso_v4;
	bfa_boolean_t lso_v6;
	bfa_ethport_csum_state_t ipv4_csum;
	bfa_ethport_csum_state_t ipv6_csum;
	bfa_port_opmode_t opmode;
	bfa_boolean_t promisc;
};
typedef struct bfa_ethport_cfg_s bfa_ethport_cfg_t;

typedef enum bfa_vmq_state {
	BNA_VMQ_STATE_UNDEFINED = 1,
	BNA_VMQ_STATE_ALLOCATED,
	BNA_VMQ_STATE_SET_FILTER,
	BNA_VMQ_STATE_PAUSED,
	BNA_VMQ_STATE_RUNNING,
	BNA_VMQ_STATE_STOP_DMA,
	BNA_VMQ_STATE_FREEING
} bfa_vmq_state_t;

#define BNA_MAX_VMQ 16
#define	BNA_MAX_VM_NAME_LEN	32
#define	BNA_MAX_VM_FILTERS	64

enum bna_vmq_filter_type {
	BNA_VMQ_FILTER_TYPE_MAC = 1,
	BNA_VMQ_FILTER_TYPE_VLAN
};

typedef struct bna_vmq_filter_s {
	uint32_t id;
	uint32_t type;
	uint8_t mac[6];
	uint16_t vlan_id;
} bna_vmq_filter_t;

typedef struct bna_vmq_s {
	uint32_t id;
	uint32_t state;
	uint32_t lookahead;
	uint32_t num_filters;
	uint16_t name[BNA_MAX_VM_NAME_LEN];
	bna_vmq_filter_t filters[BNA_MAX_VM_FILTERS];
} bna_vmq_t;

#endif /* __BFA_DEFS_ETHPORT_H__ */
