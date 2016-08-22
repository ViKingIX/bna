/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BFA_IOC_H__
#define __BFA_IOC_H__

#include <bfa_os_inc.h>
#include <cs/bfa_cs.h>
#include <bfi/bfi.h>
#include <cs/bfa_q.h>

/**
 * BFA timer declarations
 */
typedef void (*bfa_timer_cbfn_t)(void *);

/**
 * BFA timer data structure
 */
struct bfa_timer_s {
	struct bfa_q_s	qe;
	bfa_timer_cbfn_t timercb;
	void		*arg;
	int		timeout;	/**< in millisecs. */
};
typedef struct bfa_timer_s bfa_timer_t;

/**
 * Timer module structure
 */
struct bfa_timer_mod_s {
	struct bfa_q_s timer_q;
};

#define BFA_TIMER_FREQ 200 /**< specified in millisecs */

void bfa_timer_beat(struct bfa_timer_mod_s *mod);
void bfa_timer_init(struct bfa_timer_mod_s *mod);
void bfa_timer_begin(struct bfa_timer_mod_s *mod, struct bfa_timer_s *timer,
			bfa_timer_cbfn_t timercb, void *arg,
			unsigned int timeout);
void bfa_timer_stop(struct bfa_timer_s *timer);

/**
 * Generic Scatter Gather Element used by driver
 */
struct bfa_sge_s {
	uint32_t	sg_len;
	void		*sg_addr;
};
typedef struct bfa_sge_s bfa_sge_t;

#define bfa_sge_word_swap(__sge) do {					     \
	((uint32_t *)(__sge))[0] = bfa_os_swap32(((uint32_t *)(__sge))[0]);  \
	((uint32_t *)(__sge))[1] = bfa_os_swap32(((uint32_t *)(__sge))[1]);  \
	((uint32_t *)(__sge))[2] = bfa_os_swap32(((uint32_t *)(__sge))[2]);  \
} while (0)

#define bfa_swap_words(_x)  (	\
	((_x) << 32) | ((_x) >> 32))

#ifdef __BIGENDIAN
#define bfa_sge_to_be(_x)
#define bfa_sge_to_le(_x)	bfa_sge_word_swap(_x)
#define bfa_sgaddr_le(_x)	bfa_swap_words(_x)
#else
#define	bfa_sge_to_be(_x)	bfa_sge_word_swap(_x)
#define bfa_sge_to_le(_x)
#define bfa_sgaddr_le(_x)	(_x)
#endif

/**
 * BFA memory resources
 */
struct bfa_mem_dma_s {
	struct bfa_q_s	qe;		/* Queue of DMA elements */
	uint32_t	mem_len;	/* Total Length in Bytes */
	uint8_t		*kva;		/* kernel virtual address */
	uint64_t	dma;		/* dma address if DMA memory */
	uint8_t		*kva_curp;	/* kva allocation cursor */
	uint64_t	dma_curp;	/* dma allocation cursor */
};
typedef struct bfa_mem_dma_s bfa_mem_dma_t;

struct bfa_mem_kva_s {
	struct bfa_q_s	qe;		/* Queue of KVA elements */
	uint32_t	mem_len;	/* Total Length in Bytes */
	uint8_t		*kva;		/* kernel virtual address */
	uint8_t		*kva_curp;	/* kva allocation cursor */
};
typedef struct bfa_mem_kva_s bfa_mem_kva_t;

struct bfa_meminfo_s {
	struct bfa_mem_dma_s dma_info;
	struct bfa_mem_kva_s kva_info;
};
typedef struct bfa_meminfo_s bfa_meminfo_t;

/* BFA memory segment setup macros */
#define bfa_mem_dma_setup(_meminfo, _dm_ptr, _seg_sz) do {		\
	if (_dm_ptr) {							\
		((bfa_mem_dma_t *)(_dm_ptr))->mem_len = (_seg_sz);	\
		if (_seg_sz)						\
			bfa_q_enq(&(_meminfo)->dma_info.qe, _dm_ptr);	\
	} else								\
		(_meminfo)->dma_info.mem_len += (_seg_sz);		\
} while (0)

#define bfa_mem_kva_setup(_meminfo, _kva_ptr, _seg_sz) do {		\
	if (_kva_ptr) {							\
		((bfa_mem_kva_t *)(_kva_ptr))->mem_len = (_seg_sz);	\
		if (_seg_sz)						\
			bfa_q_enq(&(_meminfo)->kva_info.qe, _kva_ptr);	\
	} else								\
		(_meminfo)->kva_info.mem_len += (_seg_sz);		\
} while (0)

/* BFA dma memory segments iterator */
#define bfa_mem_dma_sptr(_mod, _i) (&(_mod)->dma_seg[(_i)])
#define bfa_mem_dma_seg_iter(_mod, _sptr, _nr, _i)			\
	for (_i = 0, _sptr = bfa_mem_dma_sptr(_mod, _i); _i < (_nr);	\
	     _i++, _sptr = bfa_mem_dma_sptr(_mod, _i))

#define bfa_mem_kva_curp(_mod)	((_mod)->kva_seg.kva_curp)
#define bfa_mem_dma_virt(_sptr) ((_sptr)->kva_curp)
#define bfa_mem_dma_phys(_sptr) ((_sptr)->dma_curp)
#define bfa_mem_dma_len(_sptr)	((_sptr)->mem_len)

/* Get the corresponding dma buf kva for a req - from the tag */
#define bfa_mem_get_dmabuf_kva(_mod, _tag, _rqsz)			\
	(((uint8_t *)							\
	 (_mod)->dma_seg[BFI_MEM_SEG_FROM_TAG(_tag, _rqsz)].kva_curp) +	\
	 BFI_MEM_SEG_REQ_OFFSET(_tag, _rqsz) * (_rqsz))

/* Get the corresponding dma buf pa for a req - from the tag */
#define bfa_mem_get_dmabuf_pa(_mod, _tag, _rqsz)			\
	((_mod)->dma_seg[BFI_MEM_SEG_FROM_TAG(_tag, _rqsz)].dma_curp +	\
	 BFI_MEM_SEG_REQ_OFFSET(_tag, _rqsz) * (_rqsz))

/**
 * PCI device information required by IOC
 */
struct bfa_pcidev_s {
	int		pci_slot;
	uint8_t		pci_func;
	uint16_t	device_id;
	uint16_t	ssid;
	bfa_os_addr_t   pci_bar_kva;
};

/**
 * Structure used to remember the DMA-able memory block's KVA and Physical
 * Address
 */
struct bfa_dma_s {
	void		*kva;	/* ! Kernel virtual address	*/
	uint64_t	pa;	/* ! Physical address		*/
};

#define BFA_DMA_ALIGN_SZ	256
#define BFA_ROUNDUP(_l, _s)	(((_l) + ((_s) - 1)) & ~((_s) - 1))

/**
 * smem size for Crossbow and Catapult
 */
#define BFI_SMEM_CB_SIZE	0x200000U	/* ! 2MB for crossbow	*/
#define BFI_SMEM_CT_SIZE	0x280000U	/* ! 2.5MB for catapult	*/

/**
 * @brief BFA dma address assignment macro. (big endian format)
 */
#define bfa_dma_be_addr_set(dma_addr, pa)	\
		__bfa_dma_be_addr_set(&dma_addr, (uint64_t)pa)
static inline void
__bfa_dma_be_addr_set(union bfi_addr_u *dma_addr, uint64_t pa)
{
	dma_addr->a32.addr_lo = (uint32_t) bfa_os_htonl(pa);
	dma_addr->a32.addr_hi = (uint32_t) bfa_os_htonl(bfa_os_u32(pa));
}

#define bfa_alen_set(__alen, __len, __pa)	\
	__bfa_alen_set(__alen, __len, (uint64_t)__pa)

#define bfa_dma_alen_set(__ioc, __alen, __len, __kva, __pa)		\
	do {								\
		if (bfa_asic_id_ct2(bfa_ioc_devid((__ioc))) &&		\
			((__ioc)->clscode == BFI_PCIFN_CLASS_ETH))	\
			__bfa_alen_set((__alen), (__len),		\
			(uint64_t)(__kva));		\
		else							\
			__bfa_alen_set((__alen), (__len),		\
			(uint64_t)(__pa));		\
	} while (0)

static inline void
__bfa_alen_set(struct bfi_alen_s *alen, uint32_t len, uint64_t pa)
{
	alen->al_len = bfa_os_htonl(len);
	bfa_dma_be_addr_set(alen->al_addr, pa);
}

struct bfa_ioc_regs_s {
	bfa_os_addr_t   hfn_mbox_cmd;
	bfa_os_addr_t   hfn_mbox;
	bfa_os_addr_t   lpu_mbox_cmd;
	bfa_os_addr_t   lpu_mbox;
	bfa_os_addr_t	lpu_read_stat;
	bfa_os_addr_t   pss_ctl_reg;
	bfa_os_addr_t   pss_err_status_reg;
	bfa_os_addr_t   app_pll_fast_ctl_reg;
	bfa_os_addr_t   app_pll_slow_ctl_reg;
	bfa_os_addr_t   ioc_sem_reg;
	bfa_os_addr_t   ioc_usage_sem_reg;
	bfa_os_addr_t   ioc_init_sem_reg;
	bfa_os_addr_t   ioc_usage_reg;
	bfa_os_addr_t   host_page_num_fn;
	bfa_os_addr_t   heartbeat;
	bfa_os_addr_t   ioc_fwstate;
	bfa_os_addr_t   alt_ioc_fwstate;
	bfa_os_addr_t   ll_halt;
	bfa_os_addr_t   alt_ll_halt;
	bfa_os_addr_t   err_set;
	bfa_os_addr_t   ioc_fail_sync;
	bfa_os_addr_t   shirq_isr_next;
	bfa_os_addr_t   shirq_msk_next;
	bfa_os_addr_t   smem_page_start;
	uint32_t	smem_pg0;
};

#define bfa_reg_read(_raddr)	bfa_os_reg_read(_raddr)
#define bfa_reg_write(_raddr, _val)	bfa_os_reg_write(_raddr, _val)
#define bfa_mem_read(_raddr, _off)	bfa_os_mem_read(_raddr, _off)
#define bfa_mem_write(_raddr, _off, _val)	\
					bfa_os_mem_write(_raddr, _off, _val)
/**
 * IOC Mailbox structures
 */
typedef void (*bfa_mbox_cmd_cbfn_t)(void *cbarg);
struct bfa_mbox_cmd_s {
	bfa_q_t		qe;
	bfa_mbox_cmd_cbfn_t	cbfn;
	void			*cbarg;
	uint32_t	msg[BFI_IOC_MSGSZ];
};

/**
 * IOC mailbox module
 */
typedef void (*bfa_ioc_mbox_mcfunc_t)(void *cbarg, struct bfi_mbmsg_s *m);
struct bfa_ioc_mbox_mod_s {
	struct bfa_q_s		cmd_q;	/*!< pending mbox queue	*/
	int			nmclass;	/*!< number of handlers */
	struct {
		bfa_ioc_mbox_mcfunc_t	cbfn;	/*!< message handlers	*/
		void			*cbarg;
	} mbhdlr[BFI_MC_MAX];
};

/**
 * IOC callback function interfaces
 */
typedef void (*bfa_ioc_enable_cbfn_t)(void *bfa, enum bfa_status status);
typedef void (*bfa_ioc_disable_cbfn_t)(void *bfa);
typedef void (*bfa_ioc_hbfail_cbfn_t)(void *bfa);
typedef void (*bfa_ioc_reset_cbfn_t)(void *bfa);
struct bfa_ioc_cbfn_s {
	bfa_ioc_enable_cbfn_t	enable_cbfn;
	bfa_ioc_disable_cbfn_t	disable_cbfn;
	bfa_ioc_hbfail_cbfn_t	hbfail_cbfn;
	bfa_ioc_reset_cbfn_t	reset_cbfn;
};

/**
 * IOC event notification mechanism.
 */
enum bfa_ioc_event_e {
	BFA_IOC_E_ENABLED	= 1,
	BFA_IOC_E_DISABLED	= 2,
	BFA_IOC_E_FAILED	= 3,
};

typedef void (*bfa_ioc_notify_cbfn_t)(void *, enum bfa_ioc_event_e);

struct bfa_ioc_notify_s {
	struct bfa_q_s		qe;
	bfa_ioc_notify_cbfn_t	cbfn;
	void			*cbarg;
};
typedef struct bfa_ioc_notify_s bfa_ioc_notify_t;

/**
 * Initialize a IOC event notification structure
 */
#define bfa_ioc_notify_init(__notify, __cbfn, __cbarg) do {	\
	(__notify)->cbfn = (__cbfn);				\
	(__notify)->cbarg = (__cbarg);				\
} while (0)

struct bfa_iocpf_s {
	bfa_fsm_t		fsm;
	struct bfa_ioc_s	*ioc;
	bfa_boolean_t		fw_mismatch_notified;
	bfa_boolean_t		auto_recover;
	uint32_t		poll_time;
};

struct bfa_ioc_s {
	bfa_fsm_t		fsm;
	struct bfa_s		*bfa;
	struct bfa_pcidev_s	pcidev;
	struct bfa_timer_mod_s 	*timer_mod;
	struct bfa_timer_s 	ioc_timer;
	struct bfa_timer_s 	sem_timer;
	struct bfa_timer_s	hb_timer;
	uint32_t		hb_count;
	struct bfa_q_s		notify_q;
	void			*dbg_fwsave;
	int			dbg_fwsave_len;
	bfa_boolean_t		dbg_fwsave_once;
	enum bfi_pcifn_class	clscode;
	struct bfa_ioc_regs_s 	ioc_regs;
	struct bfa_trc_mod_s	*trcmod;
	struct bfa_aen_s	*aen;
	struct bfa_log_mod_s	*logm;
	struct bfa_ioc_drv_stats_s	stats;
	bfa_boolean_t		fcmode;
	bfa_boolean_t		pllinit;
	bfa_boolean_t   	stats_busy;	/*!< outstanding stats */
	uint8_t			port_id;

	struct bfa_dma_s	attr_dma;
	struct bfi_ioc_attr_s	*attr;
	struct bfa_ioc_cbfn_s	*cbfn;
	struct bfa_ioc_mbox_mod_s mbox_mod;
	struct bfa_ioc_hwif_s	*ioc_hwif;
	struct bfa_iocpf_s	iocpf;
	enum bfi_fwboot_env	boot_env;
	enum bfi_asic_gen	asic_gen;
	enum bfi_asic_mode	asic_mode;
	enum bfi_port_mode	port0_mode;
	enum bfi_port_mode	port1_mode;
	enum bfa_mode_s		port_mode;
	uint8_t			ad_cap_bm; 	/*!< adapter cap bit mask */
	uint8_t			port_mode_cfg; 	/*!< config port mode */
};

struct bfa_ioc_hwif_s {
	bfa_status_t (*ioc_pll_init) (bfa_os_addr_t rb, enum bfi_asic_mode m);
	bfa_boolean_t	(*ioc_firmware_lock)	(struct bfa_ioc_s *ioc);
	void		(*ioc_firmware_unlock)	(struct bfa_ioc_s *ioc);
	void		(*ioc_reg_init)	(struct bfa_ioc_s *ioc);
	void		(*ioc_map_port)	(struct bfa_ioc_s *ioc);
	void		(*ioc_isr_mode_set)	(struct bfa_ioc_s *ioc,
					bfa_boolean_t msix);
	void		(*ioc_notify_fail)	(struct bfa_ioc_s *ioc);
	void		(*ioc_ownership_reset)	(struct bfa_ioc_s *ioc);
	bfa_boolean_t	(*ioc_sync_start)	(struct bfa_ioc_s *ioc);
	void		(*ioc_sync_join)	(struct bfa_ioc_s *ioc);
	void		(*ioc_sync_leave)	(struct bfa_ioc_s *ioc);
	void		(*ioc_sync_ack)		(struct bfa_ioc_s *ioc);
	bfa_boolean_t	(*ioc_sync_complete)	(struct bfa_ioc_s *ioc);
	bfa_boolean_t	(*ioc_lpu_read_stat)	(struct bfa_ioc_s *ioc);
	void		(*ioc_set_fwstate)	(struct bfa_ioc_s *ioc,
					bfi_ioc_state_t fwstate);
	bfi_ioc_state_t	(*ioc_get_fwstate)	(struct bfa_ioc_s *ioc);
	void		(*ioc_set_alt_fwstate)	(struct bfa_ioc_s *ioc,
					bfi_ioc_state_t fwstate);
	bfi_ioc_state_t	(*ioc_get_alt_fwstate)	(struct bfa_ioc_s *ioc);
};

/**
 * ASIC block configurtion related
 */

typedef void (*bfa_ablk_cbfn_t)(void *, enum bfa_status);

struct bfa_ablk_s {
	struct bfa_ioc_s	*ioc;
	struct bfa_ablk_cfg_s	*cfg;
	struct bfa_nwpart_cfg_s *nwpart_cfg;
	uint16_t		*pcifn;
	struct bfa_dma_s	dma_addr;
	bfa_boolean_t		busy;
	struct bfa_mbox_cmd_s	mb;
	bfa_ablk_cbfn_t		cbfn;
	void			*cbarg;
	bfa_ioc_notify_t	ioc_notify;
	struct bfa_mem_dma_s	ablk_dma;
};

#define BFA_MEM_ABLK_DMA(__bfa)	(&((__bfa)->modules.ablk.ablk_dma))

#define bfa_ioc_pcifn(__ioc)		((__ioc)->pcidev.pci_func)
#define bfa_ioc_devid(__ioc)		((__ioc)->pcidev.device_id)
#define bfa_ioc_bar0(__ioc)		((__ioc)->pcidev.pci_bar_kva)
#define bfa_ioc_portid(__ioc)		((__ioc)->port_id)
#define bfa_ioc_asic_gen(__ioc)		((__ioc)->asic_gen)
#define bfa_ioc_is_cna(__ioc)		\
	((bfa_ioc_get_type(__ioc) == BFA_IOC_TYPE_FCoE) || \
		(bfa_ioc_get_type(__ioc) == BFA_IOC_TYPE_LL))
#define bfa_ioc_is_default(__ioc)	\
	(bfa_ioc_pcifn(__ioc) == bfa_ioc_portid(__ioc))
#define bfa_ioc_boot_env(__ioc)	((__ioc)->boot_env)
#define bfa_ioc_fetch_stats(__ioc, __stats) \
		(((__stats)->drv_stats) = (__ioc)->stats)
#define bfa_ioc_clr_stats(__ioc)	\
		bfa_os_memset(&(__ioc)->stats, 0, sizeof((__ioc)->stats))
#define bfa_ioc_maxfrsize(__ioc)	((__ioc)->attr->maxfrsize)
#define bfa_ioc_rx_bbcredit(__ioc)	((__ioc)->attr->rx_bbcredit)
#define bfa_ioc_speed_sup(__ioc)				\
	(bfa_ioc_is_cna(__ioc)) ? BFA_PORT_SPEED_10GBPS :	\
		BFI_ADAPTER_GETP(SPEED, (__ioc)->attr->adapter_prop)
#define bfa_ioc_get_nports(__ioc)	\
	BFI_ADAPTER_GETP(NPORTS, (__ioc)->attr->adapter_prop)

#define bfa_ioc_stats(_ioc, _stats)	((_ioc)->stats._stats++)
#define bfa_ioc_stats_hb_count(_ioc, _hb_count)	\
	((_ioc)->stats.hb_count = (_hb_count))

#define BFA_IOC_FWIMG_MINSZ	(16 * 1024)
#define BFA_IOC_FW_SMEM_SIZE(__ioc)			\
	((bfa_ioc_asic_gen(__ioc) == BFI_ASIC_GEN_CB)	\
	? BFI_SMEM_CB_SIZE : BFI_SMEM_CT_SIZE)

#define BFA_IOC_FLASH_CHUNK_NO(off)		(off / BFI_FLASH_CHUNK_SZ_WORDS)
#define BFA_IOC_FLASH_OFFSET_IN_CHUNK(off)	(off % BFI_FLASH_CHUNK_SZ_WORDS)
#define BFA_IOC_FLASH_CHUNK_ADDR(chunkno)  (chunkno * BFI_FLASH_CHUNK_SZ_WORDS)

#ifdef BFA_IOC_IS_UEFI
#define bfa_ioc_is_uefi(__ioc) BFA_IOC_IS_UEFI
#else
#define bfa_ioc_is_uefi(__ioc) (0)
#endif

/**
 * IOC mailbox interface
 */
bfa_boolean_t bfa_ioc_mbox_queue(struct bfa_ioc_s *ioc,
			struct bfa_mbox_cmd_s *cmd,
			bfa_mbox_cmd_cbfn_t cbfn, void *cbarg);
void bfa_ioc_mbox_register(struct bfa_ioc_s *ioc,
		bfa_ioc_mbox_mcfunc_t *mcfuncs);
void bfa_ioc_mbox_isr(struct bfa_ioc_s *ioc);
void bfa_ioc_mbox_send(struct bfa_ioc_s *ioc, void *ioc_msg, int len);
bfa_boolean_t bfa_ioc_msgget(struct bfa_ioc_s *ioc, void *mbmsg);
void bfa_ioc_mbox_regisr(struct bfa_ioc_s *ioc, enum bfi_mclass mc,
		bfa_ioc_mbox_mcfunc_t cbfn, void *cbarg);

/**
 * IOC interfaces
 */

#define bfa_ioc_pll_init_asic(__ioc) \
	((__ioc)->ioc_hwif->ioc_pll_init((__ioc)->pcidev.pci_bar_kva, \
			   (__ioc)->asic_mode))

bfa_status_t bfa_ioc_pll_init(struct bfa_ioc_s *ioc);
bfa_status_t bfa_ioc_cb_pll_init(bfa_os_addr_t rb, enum bfi_asic_mode mode);
bfa_status_t bfa_ioc_ct_pll_init(bfa_os_addr_t rb, enum bfi_asic_mode mode);
bfa_status_t bfa_ioc_ct2_pll_init(bfa_os_addr_t rb, enum bfi_asic_mode mode);
bfa_boolean_t bfa_ioc_ct_pll_init_complete(bfa_os_addr_t rb);
bfa_boolean_t bfa_ioc_ct2_pll_init_complete(bfa_os_addr_t rb);
bfa_boolean_t bfa_ioc_cb_pll_init_complete(bfa_os_addr_t rb);

#define	bfa_ioc_isr_mode_set(__ioc, __msix) do {			\
	if ((__ioc)->ioc_hwif->ioc_isr_mode_set)			\
		((__ioc)->ioc_hwif->ioc_isr_mode_set(__ioc, __msix));	\
} while (0)
#define	bfa_ioc_ownership_reset(__ioc)				\
			((__ioc)->ioc_hwif->ioc_ownership_reset(__ioc))
#define bfa_ioc_get_fcmode(__ioc)	(__ioc)->fcmode

#define bfa_ioc_lpu_read_stat(__ioc) do {				\
	if ((__ioc)->ioc_hwif->ioc_lpu_read_stat)			\
		((__ioc)->ioc_hwif->ioc_lpu_read_stat(__ioc));		\
} while (0)

void bfa_ioc_set_cb_hwif(struct bfa_ioc_s *ioc);
void bfa_ioc_set_ct_hwif(struct bfa_ioc_s *ioc);
void bfa_ioc_set_ct2_hwif(struct bfa_ioc_s *ioc);
void bfa_ioc_ct2_poweron(struct bfa_ioc_s *ioc);

void bfa_ioc_trc_aen_log_set(struct bfa_ioc_s *ioc,
		struct bfa_trc_mod_s *trcmod, struct bfa_aen_s *aen,
		struct bfa_log_mod_s *logm);
void bfa_ioc_attach(struct bfa_ioc_s *ioc, void *bfa,
		struct bfa_ioc_cbfn_s *cbfn, struct bfa_timer_mod_s *timer_mod);
void bfa_ioc_auto_recover(bfa_boolean_t auto_recover);
void bfa_ioc_detach(struct bfa_ioc_s *ioc);
void bfa_ioc_suspend(struct bfa_ioc_s *ioc);
void bfa_ioc_pci_init(struct bfa_ioc_s *ioc, struct bfa_pcidev_s *pcidev,
		enum bfi_pcifn_class clscode);
uint32_t bfa_ioc_meminfo(void);
void bfa_ioc_mem_claim(struct bfa_ioc_s *ioc,  uint8_t *dm_kva, uint64_t dm_pa);
void bfa_ioc_enable(struct bfa_ioc_s *ioc);
void bfa_ioc_disable(struct bfa_ioc_s *ioc);
bfa_boolean_t bfa_ioc_intx_claim(struct bfa_ioc_s *ioc);

bfa_status_t bfa_ioc_boot(struct bfa_ioc_s *ioc,
		enum bfi_fwboot_type boot_type, enum bfi_fwboot_env boot_env);
void bfa_ioc_isr(struct bfa_ioc_s *ioc, struct bfi_mbmsg_s *msg);
void bfa_ioc_error_isr(struct bfa_ioc_s *ioc);
bfa_boolean_t bfa_ioc_is_operational(struct bfa_ioc_s *ioc);
bfa_boolean_t bfa_ioc_is_initialized(struct bfa_ioc_s *ioc);
bfa_boolean_t bfa_ioc_is_disabled(struct bfa_ioc_s *ioc);
bfa_boolean_t bfa_ioc_is_acq_addr(struct bfa_ioc_s *ioc);
bfa_boolean_t bfa_ioc_fw_mismatch(struct bfa_ioc_s *ioc);
bfa_boolean_t bfa_ioc_adapter_is_disabled(struct bfa_ioc_s *ioc);
void bfa_ioc_reset_fwstate(struct bfa_ioc_s *ioc);
void bfa_ioc_cfg_complete(struct bfa_ioc_s *ioc);
enum bfa_ioc_type_e bfa_ioc_get_type(struct bfa_ioc_s *ioc);
void bfa_ioc_get_adapter_serial_num(struct bfa_ioc_s *ioc, char *serial_num);
void bfa_ioc_get_adapter_fw_ver(struct bfa_ioc_s *ioc, char *fw_ver);
void bfa_ioc_get_adapter_optrom_ver(struct bfa_ioc_s *ioc, char *optrom_ver);
void bfa_ioc_get_adapter_model(struct bfa_ioc_s *ioc, char *model);
void bfa_ioc_get_adapter_manufacturer(struct bfa_ioc_s *ioc,
		char *manufacturer);
void bfa_ioc_get_pci_chip_rev(struct bfa_ioc_s *ioc, char *chip_rev);
enum bfa_ioc_state bfa_ioc_get_state(struct bfa_ioc_s *ioc);

void bfa_ioc_get_attr(struct bfa_ioc_s *ioc, struct bfa_ioc_attr_s *ioc_attr);
void bfa_ioc_get_adapter_attr(struct bfa_ioc_s *ioc,
		struct bfa_adapter_attr_s *ad_attr);
int bfa_ioc_debug_trcsz(bfa_boolean_t auto_recover);
void bfa_ioc_debug_memclaim(struct bfa_ioc_s *ioc, void *dbg_fwsave);
bfa_status_t bfa_ioc_debug_fwsave(struct bfa_ioc_s *ioc, void *trcdata,
		int *trclen);
void bfa_ioc_debug_fwsave_clear(struct bfa_ioc_s *ioc);
bfa_status_t bfa_ioc_debug_fwtrc(struct bfa_ioc_s *ioc, void *trcdata,
				 int *trclen);
bfa_status_t bfa_ioc_debug_fwcore(struct bfa_ioc_s *ioc, void *buf,
	uint32_t *offset, int *buflen);
uint32_t bfa_ioc_smem_pgnum(struct bfa_ioc_s *ioc, uint32_t fmaddr);
uint32_t bfa_ioc_smem_pgoff(struct bfa_ioc_s *ioc, uint32_t fmaddr);
bfa_status_t bfa_ioc_fwsig_invalidate(struct bfa_ioc_s *ioc);
void bfa_ioc_notify_register(struct bfa_ioc_s *ioc,
			     struct bfa_ioc_notify_s *notify);
bfa_boolean_t bfa_ioc_sem_get(bfa_os_addr_t sem_reg);
void bfa_ioc_sem_release(bfa_os_addr_t sem_reg);
void bfa_ioc_hw_sem_release(struct bfa_ioc_s *ioc);
void bfa_ioc_fwver_get(struct bfa_ioc_s *ioc,
			struct bfi_ioc_image_hdr_s *fwhdr);
bfa_boolean_t bfa_ioc_fwver_cmp(struct bfa_ioc_s *ioc,
			struct bfi_ioc_image_hdr_s *fwhdr);
void bfa_ioc_aen_post(struct bfa_ioc_s *ioc, enum bfa_ioc_aen_event event);

bfa_status_t bfa_ioc_fw_stats_get(struct bfa_ioc_s *ioc, void *stats);
bfa_status_t bfa_ioc_fw_stats_clear(struct bfa_ioc_s *ioc);
void bfa_ioc_debug_save_ftrc(struct bfa_ioc_s *ioc);

uint32_t bfa_ioc_get_card_type(struct bfa_ioc_s *ioc);

/*
 * asic block configuration related APIs
 */
uint32_t bfa_ablk_meminfo(void);
void bfa_ablk_memclaim(struct bfa_ablk_s *ablk, uint8_t *dma_kva,
			uint64_t dma_pa);
void bfa_ablk_attach(struct bfa_ablk_s *ablk, struct bfa_ioc_s *ioc);
bfa_status_t bfa_ablk_query(struct bfa_ablk_s *ablk,
			struct bfa_ablk_cfg_s *ablk_cfg,
			bfa_ablk_cbfn_t cbfn, void *cbarg);
bfa_status_t bfa_ablk_adapter_config(struct bfa_ablk_s *ablk,
			bfa_mode_t mode, int max_pf, int max_vf,
			bfa_ablk_cbfn_t cbfn, void *cbarg);
bfa_status_t bfa_ablk_port_config(struct bfa_ablk_s *ablk, int port,
			bfa_mode_t mode, int max_pf, int max_vf,
			bfa_ablk_cbfn_t cbfn, void *cbarg);
bfa_status_t bfa_ablk_pf_create(struct bfa_ablk_s *ablk, uint16_t *pcifn,
			uint8_t port, enum bfi_pcifn_class personality,
			uint16_t bw_min, uint16_t bw_max,
			bfa_ablk_cbfn_t cbfn, void *cbarg);
bfa_status_t bfa_ablk_pf_delete(struct bfa_ablk_s *ablk, int pcifn,
			bfa_ablk_cbfn_t cbfn, void *cbarg);
bfa_status_t bfa_ablk_pf_update(struct bfa_ablk_s *ablk, int pcifn,
			uint16_t bw_min, uint16_t bw_max,
			bfa_ablk_cbfn_t cbfn, void *cbarg);
bfa_status_t bfa_ablk_optrom_en(struct bfa_ablk_s *ablk,
			bfa_ablk_cbfn_t cbfn, void *cbarg);
bfa_status_t bfa_ablk_optrom_dis(struct bfa_ablk_s *ablk,
			bfa_ablk_cbfn_t cbfn, void *cbarg);
bfa_status_t bfa_ablk_nwpar_enable(struct bfa_ablk_s *ablk,
			bfa_ablk_cbfn_t cbfn, void *cbarg);
bfa_status_t bfa_ablk_nwpar_disable(struct bfa_ablk_s *ablk,
			bfa_ablk_cbfn_t cbfn, void *cbarg);
bfa_status_t bfa_nwpart_cfg_query(struct bfa_ablk_s *ablk,
	bfa_nwpart_cfg_t * nwpar_cfg, bfa_ablk_cbfn_t cbfn, void *cbarg);
bfa_status_t bfa_ablk_nwpar_pf_enable(struct bfa_ablk_s *ablk, int pfn,
			bfa_ablk_cbfn_t cbfn, void *cbarg);
bfa_status_t bfa_ablk_nwpar_pf_disable(struct bfa_ablk_s *ablk, int pfn,
			bfa_ablk_cbfn_t cbfn, void *cbarg);
bfa_status_t bfa_ablk_nwpar_pf_pers_change(struct bfa_ablk_s *ablk, int pfn,
	bfi_pcifn_class_t personality, bfa_ablk_cbfn_t cbfn, void *cbarg);


/*
 * Register access
 */
#define BFA_REG_CT_ADDRSZ	(0x40000)
#define BFA_REG_CB_ADDRSZ	(0x20000)
#define BFA_REG_ADDRSZ(__ioc)   \
	((uint32_t)(bfa_asic_id_ctc(bfa_ioc_devid(__ioc)) ? \
		BFA_REG_CT_ADDRSZ : BFA_REG_CB_ADDRSZ))
#define BFA_REG_ADDRMSK(__ioc)	(BFA_REG_ADDRSZ(__ioc) - 1)

/* Register offset check */
bfa_status_t bfa_ioc_reg_offset_check(struct bfa_ioc_s *ioc,
			uint32_t offset, uint32_t len);

/*
 * bfa mfg wwn API functions
 */
wwn_t bfa_ioc_get_pwwn(struct bfa_ioc_s *ioc);
wwn_t bfa_ioc_get_nwwn(struct bfa_ioc_s *ioc);
mac_t bfa_ioc_get_mac(struct bfa_ioc_s *ioc);
wwn_t bfa_ioc_get_mfg_pwwn(struct bfa_ioc_s *ioc);
wwn_t bfa_ioc_get_mfg_nwwn(struct bfa_ioc_s *ioc);
mac_t bfa_ioc_get_mfg_mac(struct bfa_ioc_s *ioc);
uint64_t bfa_ioc_get_adid(struct bfa_ioc_s *ioc);
void bfa_ioc_set_attr_wwn(struct bfa_ioc_s *ioc, wwn_t pwwn, wwn_t nwwn);

/*
 * F/W Image Size & Chunk
 */
uint32_t *bfa_cb_image_get_chunk(enum bfi_asic_gen asic_gen, uint32_t off);
uint32_t bfa_cb_image_get_size(enum bfi_asic_gen asic_gen);
bfa_status_t bfa_ioc_flash_img_get_chnk(struct bfa_ioc_s *ioc, uint32_t off,
				uint32_t *fwimg);
uint32_t bfa_ioc_flash_img_get_size(struct bfa_ioc_s *ioc);
/**
 * CNA TRCMOD declaration
 */
/*
 * !!! Only append to the enums defined here to avoid any versioning
 * !!! needed between trace utility and driver version
 */
enum {
	BFA_TRC_CNA_CEE		= 1,
	BFA_TRC_CNA_PORT	= 2,
	BFA_TRC_CNA_IOC		= 3,
	BFA_TRC_CNA_DIAG	= 4,
	BFA_TRC_CNA_IOC_CB	= 5,
	BFA_TRC_CNA_IOC_CT	= 6,
	BFA_TRC_CNA_SFP		= 7,
	BFA_TRC_CNA_FLASH	= 8,
	BFA_TRC_CNA_PHY		= 9,
	BFA_TRC_CNA_MSGQ	= 10,
	BFA_TRC_CNA_IOC_CT2	= 11,
	BFA_TRC_CNA_EDMA	= 12,
	BFA_TRC_CNA_FRU		= 13,
};

#endif /* __BFA_IOC_H__ */
