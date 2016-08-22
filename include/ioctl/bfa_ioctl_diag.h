/*
 * Copyright (c) 2006-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved.
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BFA_IOCTL_DIAG_H__
#define __BFA_IOCTL_DIAG_H__

#include <defs/bfa_defs.h>
#include <defs/bfa_defs_svc.h>

typedef struct {
	bfa_status_t	status;
	uint16_t		bfad_num;
	uint16_t		rsvd;
	uint32_t		off;
	uint32_t		nwords;
	int				inst_no;
	uint32_t		force;
#ifdef _WINDOWS
	uint32_t		buf_ptr[1];
#else
	uint64_t		buf_ptr;
#endif
} bfa_ioctl_diag_regread_t;

typedef struct {
	bfa_status_t	status;
	uint16_t		bfad_num;
	uint16_t		rsvd;
	uint32_t		off;
	uint32_t		nwords;
	int				inst_no;
	uint32_t		force;
	uint32_t		buf_ptr[1];
} bfa_ioctl_diag_regread_bufio_t;

typedef struct {
	bfa_status_t	status;
	uint16_t		bfad_num;
	uint16_t		rsvd;
	uint32_t		off;
	uint32_t		word;
	int				inst_no;
	uint32_t		force;
} bfa_ioctl_diag_regwrite_t;

typedef struct {
	bfa_status_t	status;
	uint16_t		bfad_num;
	uint16_t		rsvd;
	bfa_diag_temp_t result;
} bfa_ioctl_diag_get_temp_t;

typedef struct {
	bfa_status_t	status;
	uint16_t		bfad_num;
	uint16_t		rsvd;
	bfa_diag_post_result_t result;
} bfa_ioctl_diag_post_t;

typedef struct {
	bfa_status_t	status;
	uint16_t		bfad_num;
	uint16_t		rsvd[3];
	uint32_t		pat;
	bfa_diag_memtest_result_t result;
	bfa_diag_memtest_t memtest;
} bfa_ioctl_diag_memtest_t;

typedef struct {
	bfa_status_t	status;
	uint16_t	bfad_num;
	uint16_t	rsvd;
	uint32_t	lpcnt;
	uint32_t	pat;
} bfa_ioctl_dport_enable_t;

typedef struct {
	bfa_status_t	status;
	uint16_t		bfad_num;
	uint16_t		rsvd;
	enum bfa_port_opmode opmode;
	enum bfa_port_speed speed;
	uint32_t		lpcnt;
	uint32_t		pat;
	bfa_diag_loopback_result_t result;
} bfa_ioctl_diag_loopback_t;

typedef struct {
	bfa_status_t	status;
	uint16_t		bfad_num;
	uint16_t		rsvd;
	uint32_t		cnt;
	uint32_t		pattern;
	bfa_diag_fwping_result_t result;
} bfa_ioctl_diag_fwping_t;

typedef struct {
	bfa_status_t	status;
	uint16_t		bfad_num;
	uint16_t		rsvd;
	uint32_t		force;
	uint32_t		queue;
	bfa_diag_qtest_result_t result;
} bfa_ioctl_diag_qtest_t;

typedef struct {
	bfa_status_t	status;
	uint16_t		bfad_num;
	uint16_t		rsvd;
	struct bfa_pom_attr_s  pom;
} bfa_ioctl_diag_pom_t;

typedef struct {
	bfa_status_t	status;
	uint16_t		bfad_num;
	uint8_t			static_data;
	uint8_t			rsvd;
	sfp_mem_t		sfp;
} bfa_ioctl_diag_sfp_t;

typedef struct {
	bfa_status_t	status;
	uint16_t		bfad_num;
	uint16_t		rsvd;
	bfa_diag_ledtest_t ledtest;
} bfa_ioctl_diag_led_t;

typedef struct {
	bfa_status_t	status;
	uint16_t		bfad_num;
	uint16_t		rsvd;
	bfa_boolean_t   beacon;
	bfa_boolean_t   link_e2e_beacon;
	uint32_t		second;
} bfa_ioctl_diag_beacon_t;

typedef struct {
	bfa_status_t	status;
	uint16_t		bfad_num;
	uint16_t		rsvd;
	bfa_boolean_t   beacon;
	uint16_t		vf_id;
	uint16_t		rsvd1;
	wwn_t			lpwwn;
	wwn_t			rpwwn;
} bfa_ioctl_diag_rport_t;

/* LL Ethernet Loopback */
typedef struct {
	bfa_status_t	status;
	uint16_t		bfad_num;
	uint16_t		rsvd;
	enum bfa_port_opmode opmode;
	uint32_t		lpcnt; /* no.of frames to be looped back */
	uint32_t		pat;
	bfa_diag_loopback_result_t result;
} bfa_ioctl_diag_ll_loopback_t;

typedef struct {
	bfa_status_t	status;
	uint16_t		bfad_num;
	uint16_t		rsvd;
} bfa_ioctl_diag_lb_stat_t;

typedef struct {
	bfa_status_t status;
	uint16_t	bfad_num;
	uint16_t	rsvd;
	uint32_t	uLength;
	uint32_t	rsvd1;
	uint64_t	kva_address;
	uint64_t	uAddress;
#ifdef _WINDOWS
	uint32_t	buf_ptr[1];
#else
	uint64_t	buf_ptr;
#endif
} bfa_ioctl_diag_kva_t;

typedef struct {
	bfa_status_t status;
	uint16_t	bfad_num;
	uint16_t	rsvd;
	uint32_t	uLength;
	uint32_t	rsvd1;
	uint64_t	kva_address;
	uint64_t	uAddress;
	uint32_t	buf_ptr[1];
} bfa_ioctl_diag_kva_bufio_t;

typedef struct {
	bfa_status_t	status;
	uint16_t		bfad_num;
	uint16_t		rsvd;
	bfa_diag_dport_result_t	result;
} bfa_ioctl_diag_dport_show_t;
#endif
