/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */
#ifndef __BFA_H__
#define __BFA_H__

#include <bfa_os_inc.h>
#include <cs/bfa_cs.h>
#include <cs/bfa_q.h>
#include <cs/bfa_log.h>
#include <bfa/bfa_plog.h>
#include <defs/bfa_defs_svc.h>
#include <defs/bfa_defs_boot.h>
#include <aen/bfa_aen.h>
#include <bfi/bfi.h>
#include <cna/bfa_diag.h>

struct bfa_s;
typedef struct bfa_s bfa_t;


typedef void (*bfa_isr_func_t) (struct bfa_s *bfa, struct bfi_msg_s *m);
typedef void    (*bfa_cb_cbfn_t) (void *cbarg, bfa_boolean_t complete);
typedef void    (*bfa_cb_cbfn_status_t) (void *cbarg, bfa_status_t status);

/**
 * Interrupt message handlers
 */
void bfa_isr_unhandled(struct bfa_s *bfa, struct bfi_msg_s *m);
void bfa_isr_bind(enum bfi_mclass mc, bfa_isr_func_t isr_func);

/**
 * Request and response queue related defines
 */
#define BFA_REQQ_NELEMS_MIN	(4)
#define BFA_RSPQ_NELEMS_MIN	(4)

#define bfa_reqq_pi(__bfa, __reqq)	(__bfa)->iocfc.req_cq_pi[__reqq]
#define bfa_reqq_ci(__bfa, __reqq)					\
	*(uint32_t *)((__bfa)->iocfc.req_cq_shadow_ci[__reqq].kva)

#define bfa_reqq_full(__bfa, __reqq)				\
	(((bfa_reqq_pi(__bfa, __reqq) + 1) &			\
	  ((__bfa)->iocfc.cfg.drvcfg.num_reqq_elems - 1)) ==	\
	 bfa_reqq_ci(__bfa, __reqq))

#define bfa_reqq_next(__bfa, __reqq)					\
	(bfa_reqq_full(__bfa, __reqq) ? NULL :				\
	 ((void *)((struct bfi_msg_s *)((__bfa)->iocfc.req_cq_ba[__reqq].kva) \
		   + bfa_reqq_pi((__bfa), (__reqq)))))

#define bfa_reqq_produce(__bfa, __reqq, __mh)	do {			\
		(__mh).mtag.h2i.qid	= (__bfa)->iocfc.hw_qid[__reqq];\
		(__bfa)->iocfc.req_cq_pi[__reqq]++;			\
		(__bfa)->iocfc.req_cq_pi[__reqq] &=			\
			((__bfa)->iocfc.cfg.drvcfg.num_reqq_elems - 1);	\
		bfa_reg_write((__bfa)->iocfc.bfa_regs.cpe_q_pi[__reqq],	\
			      (__bfa)->iocfc.req_cq_pi[__reqq]);	\
		bfa_os_mmiowb();					\
	} while (0)

#define bfa_rspq_pi(__bfa, __rspq)					\
	*(uint32_t *)((__bfa)->iocfc.rsp_cq_shadow_pi[__rspq].kva)

#define bfa_rspq_ci(__bfa, __rspq)	(__bfa)->iocfc.rsp_cq_ci[__rspq]
#define bfa_rspq_elem(__bfa, __rspq, __ci)				\
	&((struct bfi_msg_s *)((__bfa)->iocfc.rsp_cq_ba[__rspq].kva))[__ci]

#define CQ_INCR(__index, __size)			\
	(__index)++; (__index) &= ((__size) - 1)

/**
 * @brief
 * Queue element to wait for room in request queue. FIFO order is
 * maintained when fullfilling requests.
 */
struct bfa_reqq_wait_s {
	struct bfa_q_s 	qe;
	void		(*qresume) (void *cbarg);
	void		*cbarg;
};

/**
 * Circular queue usage assignments
 */
enum {
	BFA_REQQ_IOC	= 0,	/* !< all low-priority IOC msgs	*/
	BFA_REQQ_FCXP	= 0,	/* !< all FCXP messages		*/
	BFA_REQQ_LPS	= 0,	/* !< all lport service msgs	*/
	BFA_REQQ_PORT	= 0,	/* !< all port messages		*/
	BFA_REQQ_FLASH	= 0,	/* !< for flash module		*/
	BFA_REQQ_DIAG	= 0,	/* !< for diag module		*/
	BFA_REQQ_RPORT	= 0,	/* !< all port messages		*/
	BFA_REQQ_SBOOT	= 0,	/* !< all san boot messages	*/
	BFA_REQQ_QOS_LO	= 1,	/* !< all low priority IO	*/
	BFA_REQQ_QOS_MD	= 2,	/* !< all medium priority IO	*/
	BFA_REQQ_QOS_HI	= 3,	/* !< all high priority IO	*/
};

static inline void
bfa_reqq_winit(struct bfa_reqq_wait_s *wqe, void (*qresume) (void *cbarg),
	       void *cbarg)
{
	wqe->qresume = qresume;
	wqe->cbarg = cbarg;
}

#define bfa_reqq(__bfa, __reqq)	&(__bfa)->reqq_waitq[__reqq]

/**
 * static inline void
 * bfa_reqq_wait(struct bfa_s *bfa, int reqq, struct bfa_reqq_wait_s *wqe)
 */
#define bfa_reqq_wait(__bfa, __reqq, __wqe) do {			\
									\
		struct bfa_q_s *waitq = bfa_reqq(__bfa, __reqq);	\
									\
		bfa_assert(((__reqq) < BFI_IOC_MAX_CQS));		\
		bfa_assert((__wqe)->qresume && (__wqe)->cbarg);		\
									\
		bfa_q_enq(waitq, (__wqe));				\
	} while (0)

#define bfa_reqq_wcancel(__wqe)	bfa_q_qe_deq(__wqe)


/**
 * Generic BFA callback element.
 */
struct bfa_cb_qe_s {
	struct bfa_q_s	qe;
	bfa_cb_cbfn_t	cbfn;
	bfa_boolean_t	once;
	bfa_boolean_t	pre_rmv;	/* set for stack based qe(s) */
	bfa_status_t	fw_status;	/* to access fw status in comp proc */
	void		*cbarg;
};

#define bfa_cb_queue(__bfa, __hcb_qe, __cbfn, __cbarg) do {	\
		(__hcb_qe)->cbfn  = (__cbfn);			\
		(__hcb_qe)->cbarg = (__cbarg);			\
		(__hcb_qe)->pre_rmv = BFA_FALSE;		\
		bfa_q_enq(&(__bfa)->comp_q, (__hcb_qe));	\
	} while (0)

#define bfa_cb_dequeue(__hcb_qe)	bfa_q_qe_deq(__hcb_qe)

#define bfa_cb_queue_once(__bfa, __hcb_qe, __cbfn, __cbarg) do {	\
		(__hcb_qe)->cbfn  = (__cbfn);				\
		(__hcb_qe)->cbarg = (__cbarg);				\
		if (!(__hcb_qe)->once) {				\
			bfa_q_enq(&(__bfa)->comp_q, (__hcb_qe));	\
			(__hcb_qe)->once = BFA_TRUE;			\
		}							\
	} while (0)

#define bfa_cb_queue_status(__bfa, __hcb_qe, __status) do {	\
		(__hcb_qe)->fw_status = (__status);		\
		bfa_q_enq(&(__bfa)->comp_q, (__hcb_qe));	\
	} while (0)

#define bfa_cb_queue_done(__hcb_qe) do {	\
		(__hcb_qe)->once = BFA_FALSE;	\
	} while (0)


/**
 * PCI devices supported by the current BFA
 */
struct bfa_pciid_s {
	uint16_t	device_id;
	uint16_t	vendor_id;
};
typedef struct bfa_pciid_s bfa_pciid_t;

extern char     bfa_version[];

/**
 * FC DIAG qtest data structure
 */
typedef struct {
	bfa_diag_qtest_result_t *result;
	bfa_cb_diag_t		cbfn;
	void				*cbarg;
	struct bfa_timer_s	timer;
	uint32_t	status;
	uint32_t	count;
	uint8_t		lock;
	uint8_t		queue;
	uint8_t		all;
	uint8_t		timer_active;
} bfa_fcdiag_qtest_t;

/**
 * FC DIAG loopback data structure
 */
typedef struct {
	bfa_cb_diag_t	cbfn;
	void		*cbarg;
	void		*result;
	bfa_boolean_t	lock;
	uint32_t	status;
} bfa_fcdiag_lb_t;

/**
 * D_Port data structures
 */

/**
 * dport module data struct
 */
typedef struct bfa_dport_s {
	struct bfa_s	*bfa;		/*!< Back pointer to BFA */
	bfa_sm_t	sm;		/*!< finite state machine	*/
	struct bfa_reqq_wait_s	reqq_wait;
	bfa_cb_diag_t	cbfn;
	void		*cbarg;
	bfi_diag_dport_msg_t	i2hmsg;
	uint8_t		test_state;  	/*!< enum dport_test_state  */
	uint8_t		dynamic;  	/*!< boolean_t  */
	uint8_t		rsvd[2];
	uint32_t	lpcnt;
	uint32_t	payload;	/*!< user defined payload pattern */
	wwn_t		rp_pwwn;
	wwn_t		rp_nwwn;
	bfa_diag_dport_result_t	result;
} bfa_dport_t;

/**
 * FC DIAG data structure
 */
struct bfa_fcdiag_s {
	struct bfa_s	*bfa;		/* Back pointer to BFA */
	bfa_trc_mod_t	*trcmod;
	bfa_fcdiag_lb_t	lb;
	bfa_fcdiag_qtest_t	qtest;
	bfa_dport_t	dport;
};

typedef struct bfa_fcdiag_s	bfa_fcdiag_t;
#define BFA_FCDIAG_MOD(__bfa)     (&(__bfa)->modules.fcdiag)

void	bfa_fcdiag_intr(struct bfa_s *bfa, struct bfi_msg_s *msg);

bfa_status_t    bfa_fcdiag_loopback(struct bfa_s *bfa,
				    enum bfa_port_opmode opmode,
				    enum bfa_port_speed speed, uint32_t lpcnt,
				    uint32_t pat,
				    bfa_diag_loopback_result_t *result,
				    bfa_cb_diag_t cbfn, void *cbarg);

bfa_status_t    bfa_fcdiag_queuetest(struct bfa_s *bfa, uint32_t ignore,
				     uint32_t queue,
				     bfa_diag_qtest_result_t *result,
				     bfa_cb_diag_t cbfn, void *cbarg);

bfa_status_t    bfa_fcdiag_lb_is_running(struct bfa_s *bfa);
bfa_status_t	bfa_dport_enable(struct bfa_s *bfa, uint32_t lpcnt,
					uint32_t pat, bfa_cb_diag_t cbfn,
					void *cbarg);
bfa_status_t	bfa_dport_disable(struct bfa_s *bfa, bfa_cb_diag_t cbfn,
							void *cbarg);
bfa_status_t	bfa_dport_start(struct bfa_s *bfa, uint32_t lpcnt,
					uint32_t pat, bfa_cb_diag_t cbfn,
					void *cbarg);
bfa_status_t	bfa_dport_show(struct bfa_s *bfa,
				bfa_diag_dport_result_t *result);

struct bfa_iocfc_regs_s {
	bfa_os_addr_t   intr_status;
	bfa_os_addr_t   intr_mask;
	bfa_os_addr_t   cpe_q_pi[BFI_IOC_MAX_CQS];
	bfa_os_addr_t   cpe_q_ci[BFI_IOC_MAX_CQS];
	bfa_os_addr_t   cpe_q_ctrl[BFI_IOC_MAX_CQS];
	bfa_os_addr_t   rme_q_ci[BFI_IOC_MAX_CQS];
	bfa_os_addr_t   rme_q_pi[BFI_IOC_MAX_CQS];
	bfa_os_addr_t   rme_q_ctrl[BFI_IOC_MAX_CQS];
};

/**
 * MSIX vector handlers
 */
#define BFA_MSIX_MAX_VECTORS	22
typedef void (*bfa_msix_handler_t)(struct bfa_s *bfa, int vec);
struct bfa_msix_s {
	int	nvecs;
	bfa_msix_handler_t handler[BFA_MSIX_MAX_VECTORS];
};

/**
 * Chip specific interfaces
 */
struct bfa_hwif_s {
	void (*hw_reginit)(struct bfa_s *bfa);
	void (*hw_reqq_ack)(struct bfa_s *bfa, int reqq);
	void (*hw_rspq_ack)(struct bfa_s *bfa, int rspq, uint32_t ci);
	void (*hw_msix_init)(struct bfa_s *bfa, int nvecs);
	void (*hw_msix_ctrl_install)(struct bfa_s *bfa);
	void (*hw_msix_queue_install)(struct bfa_s *bfa);
	void (*hw_msix_uninstall)(struct bfa_s *bfa);
	void (*hw_isr_mode_set)(struct bfa_s *bfa, bfa_boolean_t msix);
	void (*hw_msix_getvecs)(struct bfa_s *bfa, uint32_t *vecmap,
				uint32_t *nvecs, uint32_t *maxvec);
	void (*hw_msix_get_rme_range) (struct bfa_s *bfa, uint32_t *start,
				       uint32_t *end);
	int  cpe_vec_q0;
	int  rme_vec_q0;
};
typedef void (*bfa_cb_iocfc_t) (void *cbarg, enum bfa_status status);

struct bfa_faa_cbfn_s {
	bfa_cb_iocfc_t	faa_cbfn;
	void		*faa_cbarg;
};

struct bfa_faa_args_s {
	struct bfa_faa_attr_s	*faa_attr;
	struct bfa_faa_cbfn_s	faa_cb;
	bfa_faa_state_t		faa_state;
	bfa_boolean_t		busy;
};

struct bfa_iocfc_s {
	bfa_fsm_t		fsm;
	struct bfa_s 		*bfa;
	struct bfa_iocfc_cfg_s 	cfg;

	uint32_t		req_cq_pi[BFI_IOC_MAX_CQS];
	uint32_t		rsp_cq_ci[BFI_IOC_MAX_CQS];
	uint8_t			hw_qid[BFI_IOC_MAX_CQS];

	struct bfa_cb_qe_s	init_hcb_qe;
	struct bfa_cb_qe_s	stop_hcb_qe;
	struct bfa_cb_qe_s	dis_hcb_qe;
	struct bfa_cb_qe_s      en_hcb_qe;
	struct bfa_cb_qe_s	stats_hcb_qe;
	bfa_boolean_t		submod_enabled;
	bfa_boolean_t		cb_reqd;	/*!< Driver call back reqd    */
	bfa_status_t		op_status;	/*!< Status of bfa iocfc op   */

	struct bfa_dma_s	cfg_info;
	struct bfi_iocfc_cfg_s *cfginfo;
	struct bfa_dma_s	cfgrsp_dma;
	struct bfi_iocfc_cfgrsp_s *cfgrsp;
	struct bfa_dma_s   	req_cq_ba[BFI_IOC_MAX_CQS];
	struct bfa_dma_s   	req_cq_shadow_ci[BFI_IOC_MAX_CQS];
	struct bfa_dma_s   	rsp_cq_ba[BFI_IOC_MAX_CQS];
	struct bfa_dma_s   	rsp_cq_shadow_pi[BFI_IOC_MAX_CQS];
	struct bfa_iocfc_regs_s	bfa_regs;	/*!< BFA device registers */
	struct bfa_hwif_s	hwif;

	bfa_cb_iocfc_t		updateq_cbfn; /*!< bios callback function */
	void			*updateq_cbarg;	/*!< bios callback arg */
	uint32_t	intr_mask;
	struct bfa_faa_args_s	faa_args;
	struct bfa_mem_dma_s	ioc_dma;
	struct bfa_mem_dma_s	iocfc_dma;
	struct bfa_mem_dma_s	reqq_dma[BFI_IOC_MAX_CQS];
	struct bfa_mem_dma_s	rspq_dma[BFI_IOC_MAX_CQS];
	struct bfa_mem_kva_s	kva_seg;
};

#define BFA_MEM_IOC_DMA(_bfa)		(&((_bfa)->iocfc.ioc_dma))
#define BFA_MEM_IOCFC_DMA(_bfa)		(&((_bfa)->iocfc.iocfc_dma))
#define BFA_MEM_REQQ_DMA(_bfa, _qno)	(&((_bfa)->iocfc.reqq_dma[(_qno)]))
#define BFA_MEM_RSPQ_DMA(_bfa, _qno)	(&((_bfa)->iocfc.rspq_dma[(_qno)]))
#define BFA_MEM_IOCFC_KVA(_bfa)		(&((_bfa)->iocfc.kva_seg))

#define bfa_fn_lpu(__bfa)	\
	bfi_fn_lpu(bfa_ioc_pcifn(&(__bfa)->ioc), bfa_ioc_portid(&(__bfa)->ioc))
#define bfa_msix_init(__bfa, __nvecs)					\
	(__bfa)->iocfc.hwif.hw_msix_init(__bfa, __nvecs)
#define bfa_msix_ctrl_install(__bfa)					\
	(__bfa)->iocfc.hwif.hw_msix_ctrl_install(__bfa)
#define bfa_msix_queue_install(__bfa)					\
	(__bfa)->iocfc.hwif.hw_msix_queue_install(__bfa)
#define bfa_msix_uninstall(__bfa)					\
	(__bfa)->iocfc.hwif.hw_msix_uninstall(__bfa)
#define bfa_isr_rspq_ack(__bfa, __queue, __ci)				\
	(__bfa)->iocfc.hwif.hw_rspq_ack(__bfa, __queue, __ci)
#define bfa_isr_reqq_ack(__bfa, __queue) do {				\
	if ((__bfa)->iocfc.hwif.hw_reqq_ack)				\
		(__bfa)->iocfc.hwif.hw_reqq_ack(__bfa, __queue);	\
} while (0)
#define bfa_isr_mode_set(__bfa, __msix) do {				\
	if ((__bfa)->iocfc.hwif.hw_isr_mode_set)			\
		(__bfa)->iocfc.hwif.hw_isr_mode_set(__bfa, __msix);	\
} while (0)
#define bfa_isr_mode_set(__bfa, __msix) do {				\
	if ((__bfa)->iocfc.hwif.hw_isr_mode_set)			\
		(__bfa)->iocfc.hwif.hw_isr_mode_set(__bfa, __msix);	\
} while (0)
#define bfa_msix_getvecs(__bfa, __vecmap, __nvecs, __maxvec)		\
	(__bfa)->iocfc.hwif.hw_msix_getvecs(__bfa, __vecmap, __nvecs, __maxvec)
#define bfa_msix_get_rme_range(__bfa, __start, __end)			\
	(__bfa)->iocfc.hwif.hw_msix_get_rme_range(__bfa, __start, __end)
#define bfa_msix(__bfa, __vec)						\
	(__bfa)->msix.handler[__vec](__bfa, __vec)

/*
 * FC specific IOC functions.
 */
void bfa_iocfc_meminfo(struct bfa_iocfc_cfg_s *cfg,
			struct bfa_meminfo_s *meminfo,
			struct bfa_s *bfa);
void bfa_iocfc_attach(struct bfa_s *bfa, void *bfad,
		      struct bfa_iocfc_cfg_s *cfg,
		      struct bfa_pcidev_s *pcidev);
void bfa_iocfc_detach(struct bfa_s *bfa);
void bfa_iocfc_init(struct bfa_s *bfa);
void bfa_iocfc_start(struct bfa_s *bfa);
void bfa_iocfc_stop(struct bfa_s *bfa);
void bfa_iocfc_isr(void *bfa, struct bfi_mbmsg_s *msg);
void bfa_iocfc_set_snsbase(struct bfa_s *bfa, int seg_no,
			   uint64_t snsbase_pa);
bfa_boolean_t bfa_iocfc_is_operational(struct bfa_s *bfa);
void bfa_iocfc_reset_queues(struct bfa_s *bfa);
void bfa_iocfc_updateq(struct bfa_s *bfa, uint32_t reqq_ba, uint32_t rspq_ba,
		       uint32_t reqq_sci, uint32_t rspq_spi,
		       bfa_cb_iocfc_t cbfn, void *cbarg);

void bfa_msix_all(struct bfa_s *bfa, int vec);
void bfa_msix_reqq(struct bfa_s *bfa, int vec);
void bfa_msix_rspq(struct bfa_s *bfa, int vec);
void bfa_msix_lpu_err(struct bfa_s *bfa, int vec);
bfa_boolean_t bfa_isr_rspq(struct bfa_s *bfa, int qid);

void bfa_hwcb_reginit(struct bfa_s *bfa);
void bfa_hwcb_rspq_ack(struct bfa_s *bfa, int rspq, uint32_t ci);
void bfa_hwcb_msix_init(struct bfa_s *bfa, int nvecs);
void bfa_hwcb_msix_ctrl_install(struct bfa_s *bfa);
void bfa_hwcb_msix_queue_install(struct bfa_s *bfa);
void bfa_hwcb_msix_uninstall(struct bfa_s *bfa);
void bfa_hwcb_isr_mode_set(struct bfa_s *bfa, bfa_boolean_t msix);
void bfa_hwcb_msix_getvecs(struct bfa_s *bfa, uint32_t *vecmap, uint32_t *nvecs,
			   uint32_t *maxvec);
void bfa_hwcb_msix_get_rme_range(struct bfa_s *bfa, uint32_t *start,
				 uint32_t *end);
void bfa_hwct_reginit(struct bfa_s *bfa);
void bfa_hwct2_reginit(struct bfa_s *bfa);
void bfa_hwct_reqq_ack(struct bfa_s *bfa, int rspq);
void bfa_hwct_rspq_ack(struct bfa_s *bfa, int rspq, uint32_t ci);
void bfa_hwct2_rspq_ack(struct bfa_s *bfa, int rspq, uint32_t ci);
void bfa_hwct_msix_init(struct bfa_s *bfa, int nvecs);
void bfa_hwct_msix_ctrl_install(struct bfa_s *bfa);
void bfa_hwct_msix_queue_install(struct bfa_s *bfa);
void bfa_hwct_msix_uninstall(struct bfa_s *bfa);
void bfa_hwct_isr_mode_set(struct bfa_s *bfa, bfa_boolean_t msix);
void bfa_hwct_msix_getvecs(struct bfa_s *bfa, uint32_t *vecmap, uint32_t *nvecs,
			   uint32_t *maxvec);
void bfa_hwct_msix_get_rme_range(struct bfa_s *bfa, uint32_t *start,
				 uint32_t *end);

void bfa_com_meminfo(bfa_boolean_t mincfg, struct bfa_meminfo_s *meminfo,
		struct bfa_s *bfa);
void bfa_com_attach(struct bfa_s *bfa, bfa_boolean_t mincfg);
void bfa_iocfc_get_bootwwns(struct bfa_s *bfa, uint8_t *nwwns, wwn_t *wwns);
wwn_t bfa_iocfc_get_pwwn(struct bfa_s *bfa);
wwn_t bfa_iocfc_get_nwwn(struct bfa_s *bfa);
void bfa_iocfc_get_pbc_boot_cfg(struct bfa_s *bfa,
				struct bfa_boot_pbc_s *pbcfg);
int bfa_iocfc_get_pbc_vports(struct bfa_s *bfa, bfi_pbc_vport_t *pbc_vport);


/**
 *----------------------------------------------------------------------
 *		BFA public interfaces
 *----------------------------------------------------------------------
 */
#define bfa_stats(_mod, _stats)	(_mod)->stats._stats ++
#define bfa_ioc_get_stats(__bfa, __ioc_stats)		\
	bfa_ioc_fetch_stats(&(__bfa)->ioc, __ioc_stats)
#define bfa_ioc_clear_stats(__bfa)		\
	bfa_ioc_clr_stats(&(__bfa)->ioc)
#define bfa_get_nports(__bfa)			\
	bfa_ioc_get_nports(&(__bfa)->ioc)
#define bfa_get_adapter_manufacturer(__bfa, __manufacturer)		\
	bfa_ioc_get_adapter_manufacturer(&(__bfa)->ioc, __manufacturer)
#define bfa_get_adapter_model(__bfa, __model)			\
	bfa_ioc_get_adapter_model(&(__bfa)->ioc, __model)
#define bfa_get_adapter_serial_num(__bfa, __serial_num)			\
	bfa_ioc_get_adapter_serial_num(&(__bfa)->ioc, __serial_num)
#define bfa_get_adapter_fw_ver(__bfa, __fw_ver)			\
	bfa_ioc_get_adapter_fw_ver(&(__bfa)->ioc, __fw_ver)
#define bfa_get_adapter_optrom_ver(__bfa, __optrom_ver)			\
	bfa_ioc_get_adapter_optrom_ver(&(__bfa)->ioc, __optrom_ver)
#define bfa_get_pci_chip_rev(__bfa, __chip_rev)			\
	bfa_ioc_get_pci_chip_rev(&(__bfa)->ioc, __chip_rev)
#define bfa_get_ioc_state(__bfa)		\
	bfa_ioc_get_state(&(__bfa)->ioc)
#define bfa_get_type(__bfa)			\
	bfa_ioc_get_type(&(__bfa)->ioc)
#define bfa_get_mac(__bfa)			\
	bfa_ioc_get_mac(&(__bfa)->ioc)
#define bfa_get_mfg_mac(__bfa)			\
	bfa_ioc_get_mfg_mac(&(__bfa)->ioc)
#define bfa_get_fw_clock_res(__bfa)		\
	__bfa->iocfc.cfgrsp->fwcfg.fw_tick_res


/**
 * lun mask macros return NULL when min cfg is enabled and there is
 * no memory allocated for lunmask.
 */

#define bfa_get_lun_mask(__bfa)			\
	((&(__bfa)->modules.dconf_mod)->min_cfg)	\
	? NULL: (&(BFA_DCONF_MOD(__bfa)->dconf->lun_mask))

#define bfa_get_lun_mask_list(_bfa)		\
	((&(_bfa)->modules.dconf_mod)->min_cfg)	\
	? NULL: (bfa_get_lun_mask(_bfa)->lun_list)

#define bfa_get_lun_mask_status(_bfa)		\
	(((&(_bfa)->modules.dconf_mod)->min_cfg)	\
	? BFA_LUNMASK_MINCFG : ((bfa_get_lun_mask(_bfa))->status))

#define bfa_port_id_get(_bfa) ((_bfa)->ioc.port_id)

void bfa_get_pciids(struct bfa_pciid_s **pciids, int *npciids);
void bfa_cfg_get_default(struct bfa_iocfc_cfg_s *cfg);
void bfa_cfg_get_min(struct bfa_iocfc_cfg_s *cfg);
void bfa_cfg_get_meminfo(struct bfa_iocfc_cfg_s *cfg,
			 struct bfa_meminfo_s *meminfo,
			 struct bfa_s *bfa);
void bfa_attach(struct bfa_s *bfa, void *bfad, struct bfa_iocfc_cfg_s *cfg,
		struct bfa_meminfo_s *meminfo,
		struct bfa_pcidev_s *pcidev);
void bfa_init_trc(struct bfa_s *bfa, struct bfa_trc_mod_s *trcmod);
void bfa_init_log(struct bfa_s *bfa, struct bfa_log_mod_s *logmod);
void bfa_init_aen(struct bfa_s *bfa, struct bfa_aen_s *aen);
void bfa_init_plog(struct bfa_s *bfa, struct bfa_plog_s *plog);
void bfa_detach(struct bfa_s *bfa);
void bfa_init(struct bfa_s *bfa);
void bfa_start(struct bfa_s *bfa);
void bfa_stop(struct bfa_s *bfa);
void bfa_suspend(struct bfa_s *bfa);
void bfa_attach_fcs(struct bfa_s *bfa);
void bfa_cb_init(void *bfad, bfa_status_t status);
void bfa_cb_stop(void *bfad, bfa_status_t status);
void bfa_cb_get_bootwwns(void *bfad, uint8_t *nwwns, wwn_t *wwns,
				bfa_boolean_t *bflash, uint8_t *bus,
				uint8_t *target);
void bfa_cb_updateq(void *bfad, bfa_status_t status);

bfa_boolean_t bfa_intx(struct bfa_s *bfa);
void bfa_intx_disable(struct bfa_s *bfa);
void bfa_intx_enable(struct bfa_s *bfa);
void bfa_isr_enable(struct bfa_s *bfa);
void bfa_isr_disable(struct bfa_s *bfa);
void bfa_isr_poll(struct bfa_s *bfa);

void bfa_comp_deq(struct bfa_s *bfa, struct bfa_q_s *comp_q);
void bfa_comp_process(struct bfa_s *bfa, struct bfa_q_s *comp_q);
void bfa_comp_free(struct bfa_s *bfa, struct bfa_q_s *comp_q);

typedef void (*bfa_cb_ioc_t) (void *cbarg, enum bfa_status status);
void bfa_iocfc_get_attr(struct bfa_s *bfa, struct bfa_iocfc_attr_s *attr);
void bfa_get_attr(struct bfa_s *bfa, struct bfa_ioc_attr_s *ioc_attr);
uint8_t bfa_get_max_speed(struct bfa_s *bfa);
uint32_t bfa_get_card_type(struct bfa_s *bfa);
void bfa_adapter_get_attr(struct bfa_s *bfa,
			  struct bfa_adapter_attr_s *ad_attr);
uint64_t bfa_adapter_get_id(struct bfa_s *bfa);

bfa_status_t bfa_iocfc_israttr_set(struct bfa_s *bfa,
				   struct bfa_iocfc_intr_attr_s *attr);

void bfa_iocfc_enable(struct bfa_s *bfa);
void bfa_iocfc_disable(struct bfa_s *bfa);
void bfa_iocfc_dconf_cb(struct bfa_s *bfa, bfa_status_t status);

void bfa_chip_reset(struct bfa_s *bfa);
void bfa_cb_ioc_enable(void *bfad);
void bfa_cb_ioc_disable(void *bfad);
void bfa_timer_tick(struct bfa_s *bfa);
#define bfa_timer_start(_bfa, _timer, _timercb, _arg, _timeout)		\
	bfa_timer_begin(&(_bfa)->timer_mod, _timer, _timercb, _arg, _timeout)

/*
 * BFA debug API functions
 */
bfa_status_t bfa_debug_fwtrc(struct bfa_s *bfa, void *trcdata, int *trclen);
bfa_status_t bfa_debug_fwsave(struct bfa_s *bfa, void *trcdata, int *trclen);
bfa_status_t bfa_debug_fwcore(struct bfa_s *bfa, void *buf,
			      uint32_t *offset, int *buflen);
void bfa_debug_fwsave_clear(struct bfa_s *bfa);

bfa_status_t bfa_fw_stats_get(struct bfa_s *bfa, void *data);
bfa_status_t bfa_fw_stats_clear(struct bfa_s *bfa);
void bfa_get_debug_info(struct bfa_s *bfa);

/*
 * Only used for get stats, stats clr, attrib etc, where multiple apps are
 * allowed to call
 */
struct bfa_cb_pending_q_s {
	struct bfa_cb_qe_s	hcb_qe;
	void			*data;	/* Driver buffer */
};
typedef struct bfa_cb_pending_q_s bfa_cb_pending_q_t;

/* Common macros to operate on pending stats/attr apis */
#define bfa_pending_q_init(__qe, __cbfn, __cbarg, __data) do {	\
	bfa_q_qe_init(&((__qe)->hcb_qe.qe));		\
	(__qe)->hcb_qe.cbfn = (__cbfn);			\
	(__qe)->hcb_qe.cbarg = (__cbarg);		\
	(__qe)->hcb_qe.pre_rmv = BFA_TRUE;		\
	(__qe)->data = (__data);			\
} while (0)

#endif /* __BFA_H__ */
