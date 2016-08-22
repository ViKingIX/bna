/*
 * Copyright (c) 2006-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved.
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BFA_IOCTL_PORT_H__
#define __BFA_IOCTL_PORT_H__

#include <defs/bfa_defs_svc.h>
#include <defs/bfa_defs.h>
#include <cna/pstats/phyport_defs.h>

typedef struct {
	bfa_status_t    status;
	uint16_t		bfad_num;
	uint16_t		rsvd;
	struct bfa_port_attr_s attr;
} bfa_ioctl_port_attr_t;

typedef struct {
	bfa_status_t    status;
	uint16_t		bfad_num;
	uint16_t		rsvd;
	bfa_faa_attr_t	ioctl_attr;
} bfa_ioctl_faa_attr_t;

typedef struct {
	bfa_status_t    status;
	uint16_t		bfad_num;
	uint16_t		rsvd;
	enum bfa_port_topology topo;
	uint32_t		rsvd1;
} bfa_ioctl_port_cfg_topo_t;

typedef struct {
	bfa_status_t    status;
	uint16_t		bfad_num;
	uint16_t		rsvd;
	enum bfa_port_speed speed;
	uint32_t		rsvd1;
} bfa_ioctl_port_cfg_speed_t;

typedef struct {
	bfa_status_t    status;
	uint16_t		bfad_num;
	uint16_t		rsvd;
	uint8_t			alpa;
	uint8_t			rsvd1[7];
} bfa_ioctl_port_alpa_t;

typedef struct {
	bfa_status_t    status;
	uint16_t		bfad_num;
	uint16_t		maxfrsize;
} bfa_ioctl_port_cfg_maxfrsize_t;

typedef struct {
	bfa_status_t    status;
	uint16_t		bfad_num;
	uint8_t			pp_bmap;
	uint8_t			rsvd;
} bfa_ioctl_port_trunk_t;

typedef struct {
	bfa_status_t	status;
	uint16_t		bfad_num;
	uint16_t		rsvd;
	struct bfa_port_rnid_s	info;
	uint32_t		rsvd1;
} bfa_ioctl_port_rnid_t;

typedef struct {
	bfa_status_t    status;
	uint16_t		bfad_num;
	uint16_t		rsvd;
	uint32_t		buf_size;
	uint32_t		rsvd1;
#ifdef _WINDOWS
	uint32_t		buf_ptr[1];
#else
	uint64_t		buf_ptr;
#endif
} bfa_ioctl_port_stats_t;

typedef struct {
	bfa_status_t    status;
	uint16_t		bfad_num;
	uint16_t		rsvd;
	uint32_t		buf_size;
	uint32_t		rsvd1;
	uint32_t		buf_ptr[1];
} bfa_ioctl_port_stats_bufio_t;

typedef struct {
	bfa_status_t	status;
	uint16_t	bfad_num;
	uint16_t	rsvd;
	struct bfa_trunk_attr_s attr;
} bfa_ioctl_trunk_attr_t;

typedef struct {
	bfa_status_t	status;
	uint16_t	bfad_num;
	uint16_t	pcifn_id;
	bfa_vhba_attr_t attr;
} bfa_ioctl_vhba_attr_t;

typedef struct {
	bfa_status_t    status;
	uint16_t	bfad_num;
	uint16_t	instance;
	bfa_port_cfg_mode_t cfg;
} bfa_ioctl_port_cfg_mode_t;

typedef struct {
	bfa_status_t    status;
	uint16_t	bfad_num;
	uint8_t		bb_scn;
	uint8_t		rsvd;
} bfa_ioctl_bbcr_enable_t;

typedef struct {
	bfa_status_t    status;
	uint16_t	bfad_num;
	uint16_t	rsvd;
	struct bfa_bbcr_attr_s attr;
} bfa_ioctl_bbcr_attr_t;

#endif
