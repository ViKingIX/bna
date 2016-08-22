/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BFA_DEFS_SVC_H__
#define __BFA_DEFS_SVC_H__

#include <cs/bfa_q.h>
#include <defs/bfa_defs.h>
#include <protocol/bfa_fc.h>
#include <bfi/bfi.h>

#define BFA_IOCFC_INTR_DELAY	1125
#define BFA_IOCFC_INTR_LATENCY	225
#define BFA_IOCFCOE_INTR_DELAY	25
#define BFA_IOCFCOE_INTR_LATENCY 5
#define BFA_IOCFC_CB_INTR_LATENCY1	1
#define BFA_IOCFC_CB_INTR_LATENCY2	15
#define BFA_IOCFC_CB_INTR_DELAY1	5
#define BFA_IOCFC_CB_INTR_DELAY2	75

/**
 * Interrupt coalescing configuration.
 */
#pragma pack(1)
struct bfa_iocfc_intr_attr_s {
	uint8_t		coalesce;	/*!< enable/disable coalescing */
	uint8_t		rsvd[3];
	uint16_t	latency;	/*!< latency in microseconds   */
	uint16_t	delay;		/*!< delay in microseconds     */
};
typedef struct bfa_iocfc_intr_attr_s bfa_iocfc_intr_attr_t;

/**
 * IOC firmware configuraton
 */
struct bfa_iocfc_fwcfg_s {
	uint16_t        num_fabrics;	/*!< number of fabrics		*/
	uint16_t        num_lports;	/*!< number of local lports	*/
	uint16_t        num_rports;	/*!< number of remote ports	*/
	uint16_t        num_ioim_reqs;	/*!< number of IO reqs		*/
	uint16_t        num_tskim_reqs;	/*!< task management requests	*/
	uint16_t	num_fwtio_reqs;	/*!< number of TM IO reqs in FW */
	uint16_t        num_fcxp_reqs;	/*!< unassisted FC exchanges	*/
	uint16_t        num_uf_bufs;	/*!< unsolicited recv buffers	*/
	uint8_t		num_cqs;
	uint8_t		fw_tick_res;	/*!< FW clock resolution in ms */
	uint8_t		fw_xfer_rdy_tov; /*!< FW xfer_rdy timeout value */
	uint8_t		rsvd[5];
};
typedef struct bfa_iocfc_fwcfg_s bfa_iocfc_fwcfg_t;
#pragma pack()

struct bfa_iocfc_drvcfg_s {
	uint16_t        num_reqq_elems;	/*!< number of req queue elements */
	uint16_t        num_rspq_elems;	/*!< number of rsp queue elements */
	uint16_t        num_sgpgs;	/*!< number of total SG pages	  */
	uint16_t        num_sboot_tgts;	/*!< number of SAN boot targets	  */
	uint16_t        num_sboot_luns;	/*!< number of SAN boot luns	  */
	uint16_t	ioc_recover;	/*!< IOC recovery mode		  */
	uint16_t	min_cfg;	/*!< minimum configuration	  */
	uint16_t	path_tov;	/*!< device path timeout	  */
	uint16_t	num_tio_reqs;   /*!< number of TM IO reqs       */
	uint8_t		port_mode;
	uint8_t		rsvd_a;
	bfa_boolean_t   delay_comp; /*!< delay completion of
							failed inflight IOs */
	uint16_t	num_ttsk_reqs;  /*!< TM task management requests */
	uint32_t	rsvd;
};
typedef struct bfa_iocfc_drvcfg_s bfa_iocfc_drvcfg_t;
/**
 * IOC configuration
 */
struct bfa_iocfc_cfg_s {
	struct bfa_iocfc_fwcfg_s	fwcfg;	/*!< firmware side config */
	struct bfa_iocfc_drvcfg_s	drvcfg;	/*!< driver side config	  */
};
typedef struct bfa_iocfc_cfg_s bfa_iocfc_cfg_t;

/**
 * IOC firmware IOIM stats
*/
struct bfa_fw_ioim_stats_s {
    uint32_t    host_abort;     /*!< IO aborted by host driver*/
    uint32_t    host_cleanup;       /*!< IO clean up by host driver */

    uint32_t    fw_io_timeout;      /*!< No of IOs timed out */
    uint32_t    fw_frm_parse;       /*!< frame parsed by f/w */
    uint32_t    fw_frm_data;        /*!< fcp_data frame parsed by f/w */
    uint32_t    fw_frm_rsp;     /*!< fcp_rsp frame parsed by f/w */
    uint32_t    fw_frm_xfer_rdy;    /*!< xfer_rdy frame parsed by f/w */
    uint32_t    fw_frm_bls_acc;     /*!< BLS ACC  frame parsed by f/w */
    uint32_t    fw_frm_tgt_abort;   /*!< target ABTS parsed by f/w */
    uint32_t    fw_frm_unknown;     /*!< unknown parsed by f/w */
    uint32_t    fw_data_dma;        /*!< f/w DMA'ed the data frame */
    uint32_t    fw_frm_drop;        /*!< f/w drop the frame */

    uint32_t    rec_timeout;        /*!< No of FW rec timed out */
    uint32_t    error_rec;      /*!< FW sending rec on an error condition */
    uint32_t    wait_for_si;        /*!< FW wait for SI */
    uint32_t    rec_rsp_inval;      /*!< REC rsp invalid */
    uint32_t    rec_rsp_xchg_comp;      /*!< REC rsp xchg complete */
    uint32_t    rec_rsp_rd_si_ownd;      /*!< REC rsp read si owned */
    uint32_t    seqr_io_abort;      /*!< target does not know cmd so abort */
    uint32_t    seqr_io_retry;      /*!< SEQR failed so retry IO */

    uint32_t    itn_cisc_upd_rsp;   /*!< ITN cisc updated on fcp_rsp */
    uint32_t    itn_cisc_upd_data;  /*!< ITN cisc updated on fcp_data */
    uint32_t    itn_cisc_upd_xfer_rdy;  /*!< ITN cisc updated on fcp_xfer_rdy */

    uint32_t    fcp_data_lost;      /*!< fcp data lost */

    uint32_t    ro_set_in_xfer_rdy; /*!< Target set RO in Xfer_rdy frame */
    uint32_t    xfer_rdy_ooo_err;   /*!< Out of order Xfer_rdy received */
    uint32_t    xfer_rdy_unknown_err;   /*!< unknown error in xfer_rdy frame */

    uint32_t    io_abort_timeout;   /*!< No of ABTS timed out  */
    uint32_t    sler_initiated;     /*!< SLER initiated */

	uint32_t    unexp_fcp_rsp;      /*!< fcp response in wrong state */

    uint32_t    fcp_rsp_under_run;  /*!< fcp rsp IO underrun */
    uint32_t        fcp_rsp_under_run_wr;   /*!< fcp rsp IO underrun for write */
    uint32_t    fcp_rsp_under_run_err;  /*!< fcp rsp IO underrun error */
    uint32_t        fcp_rsp_resid_inval;    /*!< invalid residue */
    uint32_t    fcp_rsp_over_run;   /*!< fcp rsp IO overrun */
    uint32_t    fcp_rsp_over_run_err;   /*!< fcp rsp IO overrun error */
    uint32_t    fcp_rsp_proto_err;  /*!< protocol error in fcp rsp */
    uint32_t    fcp_rsp_sense_err;  /*!< error in sense info in fcp rsp */
    uint32_t    fcp_conf_req;       /*!< FCP conf requested */

    uint32_t    tgt_aborted_io;     /*!< target initiated abort */

    uint32_t    ioh_edtov_timeout_event;/*!< IOH edtov timer popped */
    uint32_t    ioh_fcp_rsp_excp_event; /*!< IOH FCP_RSP exception */
    uint32_t    ioh_fcp_conf_event; /*!< IOH FCP_CONF */
    uint32_t    ioh_mult_frm_rsp_event; /*!< IOH multi_frame FCP_RSP */
    uint32_t    ioh_hit_class2_event;   /*!< IOH hit class2 */
    uint32_t    ioh_miss_other_event;   /*!< IOH miss other */
    uint32_t    ioh_seq_cnt_err_event;  /*!< IOH seq cnt error */
    uint32_t    ioh_len_err_event;  /*!< IOH len err - fcp_dl != bytes xfered */
    uint32_t    ioh_seq_len_err_event;  /*!< IOH seq len error */
    uint32_t    ioh_data_oor_event; /*!< Data out of range */
    uint32_t    ioh_ro_ooo_event;   /*!< Relative offset out of range */
    uint32_t    ioh_cpu_owned_event;    /*!< IOH hit -iost owned by f/w */
    uint32_t    ioh_unexp_frame_event;  /*!< unexpected frame received count */
    uint32_t    ioh_err_int; /*!< IOH error interrupt */
};
typedef struct bfa_fw_ioim_stats_s bfa_fw_ioim_stats_t;

/**
 * IOC firmware TIO stats
 */
struct bfa_fw_tio_stats_s {
    uint32_t    tio_conf_proc;      /*!< TIO CONF processed */
    uint32_t    tio_conf_drop;      /*!< TIO CONF dropped */
    uint32_t    tio_cleanup_req;    /*!< TIO cleanup requested */
    uint32_t    tio_cleanup_comp;   /*!< TIO cleanup completed */
    uint32_t    tio_abort_rsp;      /*!< TIO abort response */
    uint32_t    tio_abort_rsp_comp; /*!< TIO abort rsp completed */
    uint32_t    tio_abts_req;       /*!< TIO ABTS requested */
    uint32_t    tio_abts_ack;       /*!< TIO ABTS ack-ed */
    uint32_t    tio_abts_ack_nocomp; /*!< TIO ABTS ack-ed but not completed */
    uint32_t    tio_abts_tmo;       /*!< TIO ABTS timeout */
    uint32_t    tio_snsdata_dma;    /*!< TIO sense data DMA */
    uint32_t    tio_rxwchan_wait; /*!< TIO waiting for RX wait channel */
    uint32_t    tio_rxwchan_avail; /*!< TIO RX wait channel available */
    uint32_t    tio_hit_bls;        /*!< TIO IOH BLS event */
    uint32_t    tio_uf_recv;        /*!< TIO received UF */
    uint32_t    tio_rd_invalid_sm; /*!< TIO read reqst in wrong state machine */
    uint32_t    tio_wr_invalid_sm;/*!< TIO write reqst in wrong state machine */

    uint32_t    ds_rxwchan_wait; /*!< DS waiting for RX wait channel */
    uint32_t    ds_rxwchan_avail; /*!< DS RX wait channel available */
    uint32_t    ds_unaligned_rd;    /*!< DS unaligned read */
    uint32_t    ds_rdcomp_invalid_sm; /*!< DS read completed in wrong state machine */
    uint32_t    ds_wrcomp_invalid_sm; /*!< DS write completed in wrong state machine */
    uint32_t    ds_flush_req;       /*!< DS flush requested */
    uint32_t    ds_flush_comp;      /*!< DS flush completed */
    uint32_t    ds_xfrdy_exp;       /*!< DS XFER_RDY expired */
    uint32_t    ds_seq_cnt_err;     /*!< DS seq cnt error */
    uint32_t    ds_seq_len_err;     /*!< DS seq len error */
    uint32_t    ds_data_oor;        /*!< DS data out of order */
    uint32_t    ds_hit_bls;     /*!< DS hit BLS */
    uint32_t    ds_edtov_timer_exp; /*!< DS edtov expired */
    uint32_t    ds_cpu_owned;       /*!< DS cpu owned */
    uint32_t    ds_hit_class2;      /*!< DS hit class2 */
    uint32_t    ds_length_err;      /*!< DS length error */
    uint32_t    ds_ro_ooo_err;      /*!< DS relative offset out-of-order error */
    uint32_t    ds_rectov_timer_exp;    /*!< DS rectov expired */
    uint32_t    ds_unexp_fr_err;    /*!< DS unexp frame error */
};
typedef struct bfa_fw_tio_stats_s bfa_fw_tio_stats_t;

/**
 * IOC firmware IO stats
 */
struct bfa_fw_io_stats_s {
    struct bfa_fw_ioim_stats_s  ioim_stats;
    struct bfa_fw_tio_stats_s       tio_stats;
};
typedef struct bfa_fw_io_stats_s bfa_fw_io_stats_t;

/**
 * IOC port firmware stats
 */

struct bfa_fw_port_fpg_stats_s {
    uint32_t    intr_evt;	/* FPG interrupts */
    uint32_t    intr;		/* FPG level 0 intrs */
    uint32_t    intr_excess;	/* FPG intr_excess */
    uint32_t    intr_cause0;	/* FPG 0 cause intrs */
    uint32_t    intr_other;	/* FPG intr other */
    uint32_t    intr_other_ign;	/* FPG intr other ignored */
    uint32_t    sig_lost;	/* FPG signal lost */
    uint32_t    sig_regained;	/* FPG signal regained */
    uint32_t    sync_lost;	/* FPG sync lost */
    uint32_t    sync_to;	/* FPG sync timeout */	 
    uint32_t    sync_regained;	/* FPG sync regained */
    uint32_t    div2_overflow;	/* FPG div2 overflow */
    uint32_t    div2_underflow;	/* FPG div2 underflow */
    uint32_t    efifo_overflow;	/* FPG efifo overflow */
    uint32_t    efifo_underflow;/* FPG efifo underflow */
    uint32_t    idle_rx;	/* FPG IDLE primitives */
    uint32_t    lrr_rx;		/* FPG LRR primitives */
    uint32_t    lr_rx;		/* FPG LR primitives */
    uint32_t    ols_rx;		/* FPG OLS primitives */
    uint32_t    nos_rx;		/* FPG NOS primitives */
    uint32_t    lip_rx;		/* FPG LIP primitives */
    uint32_t    arbf0_rx;	/* FPG ARBF0 primitives */
    uint32_t    arb_rx;		/* FPG ARB primitives */
    uint32_t    mrk_rx;		/* FPG MRK primitives */
    uint32_t    const_mrk_rx;	/* FPG const_mrk_rx */
    uint32_t    prim_unknown;	/* FPG unknown primitives */
};
typedef struct bfa_fw_port_fpg_stats_s bfa_fw_port_fpg_stats_t;

/**
 * @brief Link state machine statistics
 */
struct bfa_fw_port_lksm_stats_s {
    uint32_t    hwsm_success;       /*!< LKSM HWSM success */
    uint32_t    hwsm_fails;         /*!< LKSM HWSM failures */
    uint32_t    hwsm_wdtov;         /*!< LKSM HWSM timeouts */
    uint32_t    swsm_success;       /*!< LKSM SWSM success */
    uint32_t    swsm_fails;         /*!< LKSM SWSM failures */
    uint32_t    swsm_wdtov;         /*!< LKSM SWSM timeouts */
    uint32_t    busybufs;           /*!< LKSM Busy buffer failures */
    uint32_t    buf_waits;          /*!< LKSM Bufwait state entries */ 
    uint32_t    link_fails;         /*!< LKSM Link failures */
    uint32_t    psp_errors;         /*!< LKSM prim seq protocol errs */
    uint32_t    lr_unexp;           /*!< LKSM LRs unexpected */
    uint32_t    lrr_unexp;          /*!< LKSM LRRs unexpected */
    uint32_t    lr_tx;              /*!< LKSM LR tx started */
    uint32_t    lrr_tx;             /*!< LKSM LRR tx started */
    uint32_t    ols_tx;             /*!< LKSM OLS tx started */
    uint32_t    nos_tx;             /*!< LKSM NOS tx started */
    uint32_t    hwsm_lrr_rx;        /*!< LKSM LRR rx-ed in HWSM */
    uint32_t    hwsm_lr_rx;         /*!< LKSM LR rx-ed in HWSM */
};
typedef struct bfa_fw_port_lksm_stats_s bfa_fw_port_lksm_stats_t;

/**
 * @brief SNSM statistics
 */
struct bfa_fw_port_snsm_stats_s {
    uint32_t    hwsm_success;       /*!< SNSM HWSM success */
    uint32_t    hwsm_fails;         /*!< SNSM HWSM failures */
    uint32_t    hwsm_wdtov;         /*!< SNSM HWSM timeouts */
    uint32_t    swsm_success;       /*!< SNSM SWSM success */
    uint32_t    swsm_wdtov;         /*!< SNSM SWSM timeouts */
    uint32_t    error_resets;       /*!< SNSM Error resets */
    uint32_t    sync_lost;          /*!< SNSM Sync loss count */
    uint32_t    sig_lost;           /*!< SNSM Signal loss count */
    uint32_t    asn8g_attempts;     /*!< SNSM HWSM at 8Gbps attempts */
    uint32_t    adapt_success;      /*!< SNSM adaptation success */
    uint32_t    adapt_fails;        /*!< SNSM adaptation failures */
    uint32_t    adapt_ign_fails;    /*!< SNSM adaptation failures ignored */
};
typedef struct bfa_fw_port_snsm_stats_s bfa_fw_port_snsm_stats_t;

/**
 * @brief PHYSM statistics
 */
struct bfa_fw_port_physm_stats_s {
    uint32_t    module_inserts;     /*!< Module insert count                 */
    uint32_t    module_xtracts;     /*!< Module extracts count               */
    uint32_t    module_invalids;    /*!< Invalid module inserted count       */
    uint32_t    module_read_ign;    /*!< Module validation status ignored    */
    uint32_t    laser_faults;       /*!< Laser fault count                   */
    uint32_t    rsvd;
};
typedef struct bfa_fw_port_physm_stats_s bfa_fw_port_physm_stats_t;

/**
 * @brief FIP statistics
 */
struct bfa_fw_fip_stats_s {
    uint32_t    vlan_req;           /*!< vlan discovery requests             */
    uint32_t    vlan_notify;        /*!< vlan notifications                  */
    uint32_t    vlan_err;           /*!< vlan response error                 */
    uint32_t    vlan_timeouts;      /*!< vlan discovery timeouts              */
    uint32_t    vlan_invalids;      /*!< invalid vlan in discovery advert.   */
    uint32_t    disc_req;           /*!< Discovery solicit requests          */
    uint32_t    disc_rsp;           /*!< Discovery solicit response          */
    uint32_t    disc_err;           /*!< Discovery advt. parse errors        */
    uint32_t    disc_unsol;         /*!< Discovery unsolicited               */
    uint32_t    disc_timeouts;      /*!< Discovery timeouts                  */
    uint32_t    disc_fcf_unavail;   /*!< Discovery FCF Not Avail.            */ 
    uint32_t    linksvc_unsupp;     /*!< Unsupported link service req        */
    uint32_t    linksvc_err;        /*!< Parse error in link service req     */
    uint32_t    logo_req;           /*!< FIP logos received                  */
    uint32_t    clrvlink_req;       /*!< Clear virtual link req              */
    uint32_t    op_unsupp;          /*!< Unsupported FIP operation           */
    uint32_t    untagged;           /*!< Untagged frames (ignored)           */
    uint32_t    invalid_version;    /*!< Invalid FIP version                 */
};
typedef struct bfa_fw_fip_stats_s bfa_fw_fip_stats_t;

/**
 * @brief LPS statistics
 */
struct bfa_fw_lps_stats_s {
    uint32_t    mac_invalids;       /*!< Invalid mac assigned                */
    uint32_t    rsvd;
};
typedef struct bfa_fw_lps_stats_s bfa_fw_lps_stats_t;

/**
 * @brief FCoE port statistics
 */
struct bfa_fw_fcoe_stats_s {
    uint32_t    cee_linkups;        /*!< CEE link up count	*/
    uint32_t    cee_linkdns;        /*!< CEE link down count	*/
    uint32_t    fip_linkups;        /*!< FIP link up count	*/
    uint32_t    fip_linkdns;        /*!< FIP link down count    */
    uint32_t    fip_fails;          /*!< FIP fail count	*/
    uint32_t    mac_invalids;       /*!< Invalid mac assigned	*/
};
typedef struct bfa_fw_fcoe_stats_s bfa_fw_fcoe_stats_t;

/**
 * IOC firmware FCoE port stats
 */
struct bfa_fw_fcoe_port_stats_s {
    struct bfa_fw_fcoe_stats_s  fcoe_stats; 
    struct bfa_fw_fip_stats_s   fip_stats;
};
typedef struct bfa_fw_fcoe_port_stats_s bfa_fw_fcoe_port_stats_t;

/**
 * @brief LPSM statistics
 */
struct bfa_fw_lpsm_stats_s {
	uint32_t	cls_rx;		/* LPSM cls_rx 			*/
	uint32_t	cls_tx;		/* LPSM cls_tx 			*/
	uint32_t	arbf0_rx;	/* LPSM abrf0 rcvd 		*/
	uint32_t	arbf0_tx;	/* LPSM abrf0 xmit 		*/
	uint32_t	init_rx;	/* LPSM loop init start		*/
	uint32_t	unexp_hwst;	/* LPSM unknown hw state 	*/
	uint32_t	unexp_frame;	/* LPSM unknown_frame		*/
	uint32_t	unexp_prim;	/* LPSM unexpected primitive	*/
	uint32_t	prev_alpa_unavail; /* LPSM prev alpa unavailable */
	uint32_t	alpa_unavail;	/* LPSM alpa not available 	*/
	uint32_t	lip_rx;		/* LPSM lip rcvd  		*/
	uint32_t	lip_f7f7_rx;	/* LPSM lip f7f7 rcvd  		*/
	uint32_t	lip_f8_rx;	/* LPSM lip f8 rcvd  		*/
	uint32_t	lip_f8f7_rx;	/* LPSM lip f8f7 rcvd  		*/
	uint32_t	lip_other_rx;	/* LPSM lip other rcvd 		*/
	uint32_t	lip_tx;		/* LPSM lip xmit  		*/
	uint32_t	retry_tov;	/* LPSM retry TOV		 */
	uint32_t	lip_tov;	/* LPSM LIP wait TOV		 */
	uint32_t	idle_tov;	/* LPSM idle wait TOV		 */
	uint32_t	arbf0_tov;	/* LPSM arbfo wait TOV		 */
	uint32_t	stop_loop_tov;	/* LPSM stop loop wait TOV	 */
	uint32_t	lixa_tov;	/* LPSM lisa wait TOV		 */
	uint32_t	lixx_tov;	/* LPSM lilp/lirp wait TOV	 */
	uint32_t	cls_tov;	/* LPSM cls wait TOV		 */
	uint32_t	sler;		/* LPSM SLER recvd		 */
	uint32_t	failed;		/* LPSM failed 			 */
	uint32_t	success;	/* LPSM online			 */	
};
typedef struct bfa_fw_lpsm_stats_s bfa_fw_lpsm_stats_t;

/**
 * IOC firmware FC uport stats
 */
struct bfa_fw_fc_uport_stats_s {
	struct bfa_fw_port_snsm_stats_s		snsm_stats;
	struct bfa_fw_port_lksm_stats_s		lksm_stats;
	struct bfa_fw_lpsm_stats_s		lpsm_stats;
};
typedef struct bfa_fw_fc_uport_stats_s bfa_fw_fc_uport_stats_t;

/**
 * IOC firmware FC port stats
 */
union bfa_fw_fc_port_stats_s {
	struct bfa_fw_fc_uport_stats_s	fc_stats;
	struct bfa_fw_fcoe_port_stats_s	fcoe_stats;
};
typedef union bfa_fw_fc_port_stats_s bfa_fw_fc_port_stats_t;

/**
 * IOC firmware port stats
 */
struct bfa_fw_port_stats_s {
	struct bfa_fw_port_fpg_stats_s		fpg_stats;
	struct bfa_fw_port_physm_stats_s	physm_stats;
	union  bfa_fw_fc_port_stats_s		fc_port;
};
typedef struct bfa_fw_port_stats_s bfa_fw_port_stats_t;

/**
 * fcxchg module statistics
 */
struct bfa_fw_fcxchg_stats_s {
	uint32_t	ua_tag_inv;	/* ua_tag_inv */
	uint32_t	ua_state_inv;	/* ua_state_inv */
};
typedef struct bfa_fw_fcxchg_stats_s bfa_fw_fcxchg_stats_t;

/**
 * @brief
 *  Trunk statistics
 */
typedef struct bfa_fw_trunk_stats_s {
	uint32_t emt_recvd;		/*!< Trunk EMT received	*/
	uint32_t emt_accepted;		/*!< Trunk EMT Accepted	*/
	uint32_t emt_rejected;		/*!< Trunk EMT rejected	*/
	uint32_t etp_recvd;		/*!< Trunk ETP received	*/
	uint32_t etp_accepted;		/*!< Trunk ETP Accepted	*/
	uint32_t etp_rejected;		/*!< Trunk ETP rejected	*/
	uint32_t lr_recvd;		/*!< Trunk LR received		*/
	uint32_t rsvd;			/*!< padding for 64 bit alignment */
} bfa_fw_trunk_stats_t;

typedef struct bfa_fw_aport_stats_s {
	uint32_t flogi_sent;		/*!< Flogi sent		*/
	uint32_t flogi_acc_recvd;	/*!< Flogi Acc received	*/
	uint32_t flogi_rjt_recvd;	/*!< Flogi rejects received	*/
	uint32_t flogi_retries;		/*!< Flogi retries		*/

	uint32_t elp_recvd;		/*!< ELP received		*/
	uint32_t elp_accepted;		/*!< ELP Accepted		*/
	uint32_t elp_rejected;		/*!< ELP rejected		*/
	uint32_t elp_dropped;		/*!< ELP dropped		*/

	uint32_t bbcr_lr_count;		/*!< BBCR Link Resets */
	uint32_t frame_lost_intrs;	/*!< BBCR Frame loss intrs */
	uint32_t rrdy_lost_intrs;	/*!< BBCR Rrdy loss intrs */
	uint32_t rsvd;
} bfa_fw_aport_stats_t;

/**
 * IOCFC firmware stats
 */
struct bfa_fw_iocfc_stats_s {
	uint32_t	cfg_reqs;	/*!< cfg request */
	uint32_t	updq_reqs;	/*!< update queue request */
	uint32_t	ic_reqs;	/*!< interrupt coalesce reqs */
	uint32_t	unknown_reqs;	/*!< Unknown req */
	uint32_t	set_intr_reqs;	/*!< set interrupt reqs */
};
typedef struct bfa_fw_iocfc_stats_s bfa_fw_iocfc_stats_t;



/**
 * IOC attributes returned in queries
 */
struct bfa_iocfc_attr_s {
	struct bfa_iocfc_cfg_s		config;		/*!< IOCFC config   */
	struct bfa_iocfc_intr_attr_s	intr_attr;	/*!< interrupt attr */
};
typedef struct bfa_iocfc_attr_s bfa_iocfc_attr_t;

/**
 * Eth_sndrcv mod stats
 */
struct bfa_fw_eth_sndrcv_stats_s {
	uint32_t	crc_err;	/* crc_err */
	uint32_t	rsvd;		/*!< 64bit align    */
};
typedef struct bfa_fw_eth_sndrcv_stats_s bfa_fw_eth_sndrcv_stats_t;

/**
 * CT MAC mod stats
 */
struct bfa_fw_mac_mod_stats_s {
	uint32_t	mac_on;		/*!< MAC got turned-on */
	uint32_t	link_up;	/*!< link-up */
	uint32_t	signal_off;	/*!< lost signal */
	uint32_t	dfe_on;		/*!< DFE on */
	uint32_t	mac_reset;	/*!< No of MAC reset to bring lnk up */
	uint32_t	pcs_reset;	/*!< No of PCS reset to bring lnk up */
	uint32_t	loopback;	/*!< MAC got into serdes loopback */
	uint32_t	lb_mac_reset; /*!< Num MAC reset to bring linkup in loopback */
	uint32_t	lb_pcs_reset; /*!< Num PCS reset to bring link up in loopback */
	uint32_t	rsvd;		/*!< 64bit align    */
};
typedef struct bfa_fw_mac_mod_stats_s bfa_fw_mac_mod_stats_t;

/**
 * CT MOD stats
 */
struct bfa_fw_ct_mod_stats_s {
	uint32_t	rxa_rds_undrun;	/*!< RxA RDS underrun */
	uint32_t	rad_bpc_ovfl;	/*!< RAD BPC overflow */
	uint32_t	rad_rlb_bpc_ovfl; /*!< RAD RLB BPC overflow */
	uint32_t	bpc_fcs_err;	/*!< BPC FCS_ERR */
	uint32_t	txa_tso_hdr;	/*!< TxA TSO header too long */
	uint32_t	rsvd;		/*!< 64bit align    */
};
typedef struct bfa_fw_ct_mod_stats_s bfa_fw_ct_mod_stats_t;

/**
 * RDS mod stats
 */
struct bfa_fw_rds_stats_s {
	uint32_t	no_fid_drop_err;	/* RDS no fid drop error */
	uint32_t	rsvd;			/*!< 64bit align    */
};
typedef struct bfa_fw_rds_stats_s bfa_fw_rds_stats_t;

/**
 * IOC firmware stats
 */
struct bfa_fw_stats_s {
	struct bfa_fw_ioc_stats_s	ioc_stats;
	struct bfa_fw_iocfc_stats_s	iocfc_stats;
	struct bfa_fw_io_stats_s	io_stats;
	struct bfa_fw_port_stats_s	port_stats;
	struct bfa_fw_fcxchg_stats_s	fcxchg_stats;
	struct bfa_fw_lps_stats_s	lps_stats;
	struct bfa_fw_trunk_stats_s	trunk_stats;
	struct bfa_fw_aport_stats_s	aport_stats;
	struct bfa_fw_mac_mod_stats_s	macmod_stats;
	struct bfa_fw_ct_mod_stats_s	ctmod_stats;
	struct bfa_fw_eth_sndrcv_stats_s	ethsndrcv_stats;
	struct bfa_fw_rds_stats_s	rds_stats;
};
typedef struct bfa_fw_stats_s bfa_fw_stats_t;

#define BFA_IOCFC_PATHTOV_MAX	60
#define BFA_IOCFC_QDEPTH_MAX	2000

/**
 * QoS states
 */
enum bfa_qos_state {
	BFA_QOS_DISABLED = 0,		/*!< QoS is disabled */
	BFA_QOS_ONLINE = 1,		/*!< QoS is online */
	BFA_QOS_OFFLINE = 2,		/*!< QoS is offline */
};
typedef enum bfa_qos_state bfa_qos_state_t;


/**
 * QoS  Priority levels.
 */
enum bfa_qos_priority {
	BFA_QOS_UNKNOWN = 0,
	BFA_QOS_HIGH  = 1,	/*!< QoS Priority Level High */
	BFA_QOS_MED  =  2,	/*!< QoS Priority Level Medium */
	BFA_QOS_LOW  =  3,	/*!< QoS Priority Level Low */
};
typedef enum bfa_qos_priority bfa_qos_priority_t;


/**
 * QoS  bandwidth allocation for each priority level
 */
enum bfa_qos_bw_alloc {
	BFA_QOS_BW_HIGH  = 60,	/*!< bandwidth allocation for High */
	BFA_QOS_BW_MED  =  30,	/*!< bandwidth allocation for Medium */
	BFA_QOS_BW_LOW  =  10,	/*!< bandwidth allocation for Low */
};
typedef enum bfa_qos_bw_alloc bfa_qos_bw_alloc_t;
#pragma pack(1)
struct bfa_qos_bw_s {
	uint8_t		qos_bw_set;
	uint8_t		high;
	uint8_t		med;
	uint8_t		low;
};
typedef struct bfa_qos_bw_s bfa_qos_bw_t;

/**
 * QoS attribute returned in QoS Query
 */
struct bfa_qos_attr_s {
	uint8_t		state;		/*!< QoS current state */
	uint8_t		rsvd[3];
	uint32_t  total_bb_cr;  	 	/*!< Total BB Credits */
	bfa_qos_bw_t	qos_bw;		/*!< QOS bw cfg */
	bfa_qos_bw_t	qos_bw_op;	/*!< QOS bw operational */
};
typedef struct bfa_qos_attr_s bfa_qos_attr_t;

enum bfa_bbcr_state {
	BFA_BBCR_DISABLED,	/*!< BBCR is disable */
	BFA_BBCR_ONLINE,	/*!< BBCR is online  */
	BFA_BBCR_OFFLINE,	/*!< BBCR is offline */
};
typedef enum bfa_bbcr_state bfa_bbcr_state_t;

enum bfa_bbcr_err_reason {
	BFA_BBCR_ERR_REASON_NONE, /*!< Unknown */
	BFA_BBCR_ERR_REASON_SPEED_UNSUP, /*!< Port speed < max sup_speed */
	BFA_BBCR_ERR_REASON_PEER_UNSUP,	/*!< BBCR is disable on peer port */
	BFA_BBCR_ERR_REASON_NON_BRCD_SW, /*!< Connected to non BRCD switch */
	BFA_BBCR_ERR_REASON_FLOGI_RJT, /*!< Login rejected by the switch */
};
typedef enum bfa_bbcr_err_reason bfa_bbcr_err_reason_t;

struct bfa_bbcr_attr_s {
	uint8_t	state;
	uint8_t	peer_bb_scn;
	uint8_t	reason;
	uint8_t	rsvd;
};
typedef struct bfa_bbcr_attr_s bfa_bbcr_attr_t;

/**
 * These fields should be displayed only from the CLI.
 * There will be a separate BFAL API (get_qos_vc_attr ?)
 * to retrieve this.
 *
 */
#define  BFA_QOS_MAX_VC  16

struct bfa_qos_vc_info_s {
	uint8_t vc_credit;
	uint8_t borrow_credit;
	uint8_t priority;
	uint8_t resvd;
};
typedef struct bfa_qos_vc_info_s bfa_qos_vc_info_t;

struct bfa_qos_vc_attr_s {
	uint16_t  total_vc_count;                    /*!< Total VC Count */
	uint16_t  shared_credit;
	uint32_t  elp_opmode_flags;
	struct bfa_qos_vc_info_s vc_info[BFA_QOS_MAX_VC];  /*!<  as many as
							    * total_vc_count */
};
typedef struct bfa_qos_vc_attr_s bfa_qos_vc_attr_t;

/**
 * QoS statistics
 */
struct bfa_qos_stats_s {
	uint32_t	flogi_sent; 		/*!< QoS Flogi sent */
	uint32_t	flogi_acc_recvd;	/*!< QoS Flogi Acc received */
	uint32_t	flogi_rjt_recvd; /*!< QoS Flogi rejects received */
	uint32_t	flogi_retries;		/*!< QoS Flogi retries */

	uint32_t	elp_recvd; 	   	/*!< QoS ELP received */
	uint32_t	elp_accepted;       /*!< QoS ELP Accepted */
	uint32_t	elp_rejected;       /*!< QoS ELP rejected */
	uint32_t	elp_dropped;        /*!< QoS ELP dropped  */

	uint32_t	qos_rscn_recvd;     /*!< QoS RSCN received */
	uint32_t	rsvd; 		/* padding for 64 bit alignment */
};
typedef struct bfa_qos_stats_s bfa_qos_stats_t;


/**
 * @brief
 * FCoE statistics
 */
struct bfa_fcoe_stats_s {
	uint64_t	secs_reset;	/*!< Seconds since stats reset	     */
	uint64_t	cee_linkups;	/*!< CEE link up		     */
	uint64_t	cee_linkdns;	/*!< CEE link down		     */
	uint64_t	fip_linkups;	/*!< FIP link up		     */
	uint64_t	fip_linkdns;	/*!< FIP link down		     */
	uint64_t	fip_fails;	/*!< FIP failures		     */
	uint64_t	mac_invalids;	/*!< Invalid mac assignments	     */
	uint64_t	vlan_req;	/*!< Vlan requests		     */
	uint64_t	vlan_notify;	/*!< Vlan notifications	*/
	uint64_t	vlan_err;	/*!< Vlan notification errors	     */
	uint64_t	vlan_timeouts;	/*!< Vlan request timeouts	     */
	uint64_t	vlan_invalids;	/*!< Vlan invalids		     */
	uint64_t	disc_req;	/*!< Discovery requests	*/
	uint64_t	disc_rsp;	/*!< Discovery responses	     */
	uint64_t	disc_err;	/*!< Discovery error frames	     */
	uint64_t	disc_unsol;	/*!< Discovery unsolicited	     */
	uint64_t	disc_timeouts;	/*!< Discovery timeouts	*/
	uint64_t	disc_fcf_unavail; /*!< Discovery FCF not avail	     */
	uint64_t	linksvc_unsupp;	/*!< FIP link service req unsupp.    */
	uint64_t	linksvc_err;	/*!< FIP link service req errors     */
	uint64_t	logo_req;	/*!< FIP logos received	*/
	uint64_t	clrvlink_req;	/*!< Clear virtual link requests     */
	uint64_t	op_unsupp;	/*!< FIP operation unsupp.	     */
	uint64_t	untagged;	/*!< FIP untagged frames	     */
	uint64_t	txf_ucast;	/*!< Tx FCoE unicast frames	     */
	uint64_t	txf_ucast_vlan;	/*!< Tx FCoE unicast vlan frames     */
	uint64_t	txf_ucast_octets; /*!< Tx FCoE unicast octets	     */
	uint64_t	txf_mcast;	/*!< Tx FCoE multicast frames	     */
	uint64_t	txf_mcast_vlan;	/*!< Tx FCoE multicast vlan frames   */
	uint64_t	txf_mcast_octets; /*!< Tx FCoE multicast octets  */
	uint64_t	txf_bcast;	/*!< Tx FCoE broadcast frames	     */
	uint64_t	txf_bcast_vlan;	/*!< Tx FCoE broadcast vlan frames   */
	uint64_t	txf_bcast_octets; /*!< Tx FCoE broadcast octets	*/
	uint64_t	txf_timeout;	/*!< Tx timeouts		     */
	uint64_t	txf_parity_errors; /*!< Transmit parity err	     */
	uint64_t	txf_fid_parity_errors; /*!< Transmit FID parity err  */
	uint64_t	rxf_ucast_octets; /*!< Rx FCoE unicast octets	     */
	uint64_t	rxf_ucast;	/*!< Rx FCoE unicast frames	     */
	uint64_t	rxf_ucast_vlan;	/*!< Rx FCoE unicast vlan frames     */
	uint64_t	rxf_mcast_octets; /*!< Rx FCoE multicast octets  */
	uint64_t	rxf_mcast;	/*!< Rx FCoE multicast frames	     */
	uint64_t	rxf_mcast_vlan;	/*!< Rx FCoE multicast vlan frames   */
	uint64_t	rxf_bcast_octets; /*!< Rx FCoE broadcast octets	*/
	uint64_t	rxf_bcast;	/*!< Rx FCoE broadcast frames	     */
	uint64_t	rxf_bcast_vlan;	/*!< Rx FCoE broadcast vlan frames   */
};
typedef struct bfa_fcoe_stats_s bfa_fcoe_stats_t;

/**
 * @brief
 * QoS or FCoE stats (fcport stats excluding physical FC port stats)
 */
union bfa_fcport_stats_u {
	struct bfa_qos_stats_s	fcqos;
	struct bfa_fcoe_stats_s	fcoe;
};
typedef union bfa_fcport_stats_u bfa_fcport_stats_t;

#pragma pack()

struct bfa_fcpim_del_itn_stats_s {
    uint32_t    del_itn_iocomp_aborted;     /*!< Deleted ITN - Aborted IO requests */
    uint32_t    del_itn_iocomp_timedout;    /*!< Deleted ITN - IO timeouts */
    uint32_t    del_itn_iocom_sqer_needed;  /*!< Deleted ITN - IO retry for SQ error recovery   */
    uint32_t    del_itn_iocom_res_free;     /*!< Deleted ITN - Delayed freeing of IO resources  */
    uint32_t    del_itn_iocom_hostabrts;    /*!< Deleted ITN - Host IO abort requests   */
    uint32_t    del_itn_total_ios;      /*!< Deleted ITN - Total IO count   */
    uint32_t    del_io_iocdowns;        /*!< Deleted ITN - IO cleaned-up due to IOC down    */
    uint32_t    del_tm_iocdowns;        /*!< Deleted ITN - TM cleaned-up due to IOC down    */
};
typedef struct bfa_fcpim_del_itn_stats_s bfa_fcpim_del_itn_stats_t;

struct bfa_itnim_iostats_s {

	uint32_t	total_ios;		/*!< Total IO Requests*/
	uint32_t	input_reqs;		/*!< Data in-bound requests	*/
	uint32_t	output_reqs;		/*!< Data out-bound requests	*/
	uint32_t	io_comps;		/*!< Total IO Completions	*/
	uint32_t	wr_throughput;		/*!< Write data transferred in bytes	*/
	uint32_t	rd_throughput;		/*!< Read data transferred in bytes	*/

	uint32_t	iocomp_ok;		/*!< Slowpath IO completions	*/
	uint32_t	iocomp_underrun;	/*!< IO underrun		*/
	uint32_t	iocomp_overrun;		/*!< IO overrun		*/
	uint32_t	qwait;			/*!< IO Request-Q wait	*/
	uint32_t	qresumes;		/*!< IO Request-Q wait done	*/
	uint32_t	no_iotags;		/*!< No free IO tag		*/
	uint32_t	iocomp_timedout;	/*!< IO timeouts		*/
	uint32_t	iocom_nexus_abort;	/*!< IO failure due to target offline	*/
	uint32_t	iocom_proto_err;	/*!< IO protocol errors	*/
	uint32_t	iocom_dif_err;		/*!< IO SBC-3 protection errors	*/

	uint32_t	iocom_sqer_needed;	/*!< fcp-2 error recovery failed	*/
	uint32_t	iocom_res_free;		/*!< Delayed freeing of IO tag */

	uint32_t	io_aborts;		/*!< Host IO abort requests	*/
	uint32_t	iocom_hostabrts;	/*!< Host IO abort completions */
	uint32_t	io_cleanups;		/*!< IO clean-up requests	*/
	uint32_t	path_tov_expired;	/*!< IO path tov expired	*/
	uint32_t	iocomp_aborted;		/*!< IO abort completions	*/
	uint32_t	io_iocdowns;		/*!< IO cleaned-up due to IOC down */
	uint32_t	iocom_utags;		/*!< IO comp with unknown tags */

	uint32_t	io_tmaborts;		/*!< Abort request due to TM command	*/
	uint32_t	tm_io_comps;		/*!< Abort completion due to TM command	*/

	uint32_t	creates;		/*!< IT Nexus create requests	*/
	uint32_t	fw_create;		/*!< IT Nexus FW create requests	*/
	uint32_t	create_comps;		/*!< IT Nexus FW create completions	*/
	uint32_t	onlines;		/*!< IT Nexus onlines		*/
	uint32_t	offlines;		/*!< IT Nexus offlines	*/
	uint32_t	fw_delete;		/*!< IT Nexus FW delete requests	*/
	uint32_t	delete_comps;		/*!< IT Nexus FW delete completions	*/
	uint32_t	deletes;		/*!< IT Nexus delete requests	*/
	uint32_t	sler_events;		/*!< SLER events		*/
	uint32_t	ioc_disabled;		/*!< Num IOC disables		*/
	uint32_t	cleanup_comps;		/*!< IT Nexus cleanup completions	*/

	uint32_t	tm_cmnds;		/*!< TM Requests		*/
	uint32_t	tm_fw_rsps;		/*!< TM Completions		*/
	uint32_t	tm_success;		/*!< TM initiated IO cleanup success	*/
	uint32_t	tm_failures;		/*!< TM initiated IO cleanup failure	*/
	uint32_t	no_tskims;		/*!< No free TM tag		*/
	uint32_t	tm_qwait;		/*!< TM Request-Q wait	*/
	uint32_t	tm_qresumes;		/*!< TM Request-Q wait done	*/

	uint32_t	tm_iocdowns;		/*!< TM cleaned-up due to IOC down */
	uint32_t	tm_cleanups;		/*!< TM cleanup requests	*/
	uint32_t	tm_cleanup_comps;	/*!< TM cleanup completions	*/
	uint32_t	lm_lun_across_sg;	/*!< LM lun is across sg data buf */
	uint32_t	lm_lun_not_sup;		/*!< LM lun not supported */
	uint32_t	lm_rpl_data_changed;	/*!< LM report-lun data changed */
	uint32_t	lm_wire_residue_changed; /*!< LM residue in report-lun response changed */
	uint32_t	lm_small_buf_addresidue; /*!< LM buf is smaller than lun cnt reported by tgt */
	uint32_t	lm_lun_not_rdy;		/*!< LM lun not ready */
	uint32_t	lun_scsi_bad_lba;	/*!< SCSI bad LBA	*/
	uint32_t	lun_scsi_wr_prct;	/*!< SCSI LUN write protected */
	uint32_t	lun_scsi_res_preempt;	/*!< SCSI reservation preempted	*/
	uint32_t	lun_scsi_power_on_rst;	/*!< SCSI Reset	(power on, bus, tgt, lun)*/
	uint32_t	lun_scsi_inq_ch;	/*!< SCSI Inquiry data changed 	*/
	uint32_t	lun_scsi_rpl_ch;	/*!< SCSI Report Lun data changed */
	uint32_t	lun_scsi_lun_n_rdy;	/*!< SCSI Lun not ready 	*/
	uint32_t	lun_scsi_lun_n_sup;	/*!< SCSI Lun not supported 	*/
	uint32_t	lun_scsi_res_conflict;	/*!< SCSI reservation conflict 	*/
	uint32_t	lun_scsi_cmd_term;	/*!< SCSI Command terminated 	*/
	uint32_t	lun_scsi_q_full;	/*!< SCSI queue full 	*/
	uint32_t	lun_scsi_aca_act;	/*!< SCSI ACA active 	*/
	uint32_t	lun_scsi_busy;		/*!< SCSI status busy 	*/
	uint32_t	lun_scsi_check_cond;	/*!< SCSI status check condition */
};
typedef struct bfa_itnim_iostats_s bfa_itnim_iostats_t;

/* Modify char* port_stt[] in bfal_port.c if a new state was added */
enum bfa_port_states {
	BFA_PORT_ST_UNINIT 		= 1,
	BFA_PORT_ST_ENABLING_QWAIT 	= 2,
	BFA_PORT_ST_ENABLING 		= 3,
	BFA_PORT_ST_LINKDOWN 		= 4,
	BFA_PORT_ST_LINKUP 		= 5,
	BFA_PORT_ST_DISABLING_QWAIT 	= 6,
	BFA_PORT_ST_DISABLING		= 7,
	BFA_PORT_ST_DISABLED 		= 8,
	BFA_PORT_ST_STOPPED 		= 9,
	BFA_PORT_ST_IOCDOWN 		= 10,
	BFA_PORT_ST_IOCDIS 		= 11,
	BFA_PORT_ST_FWMISMATCH		= 12,
	BFA_PORT_ST_PREBOOT_DISABLED	= 13,
	BFA_PORT_ST_TOGGLING_QWAIT	= 14,
	BFA_PORT_ST_FAA_MISCONFIG	= 15,
	BFA_PORT_ST_DPORT		= 16,
	BFA_PORT_ST_DDPORT		= 17,
	BFA_PORT_ST_MAX_STATE,
};
typedef enum bfa_port_states bfa_port_states_t;

/**
 * @brief
 * 		Port operational type (in sync with SNIA port type).
 */
enum bfa_port_type {
	BFA_PORT_TYPE_UNKNOWN = 1,	/*!< port type is unknown */
	BFA_PORT_TYPE_NPORT   = 5,	/*!< P2P with switched fabric */
	BFA_PORT_TYPE_NLPORT  = 6,	/*!< public loop */
	BFA_PORT_TYPE_LPORT   = 20,	/*!< private loop */
	BFA_PORT_TYPE_P2P     = 21,	/*!< P2P with no switched fabric */
	BFA_PORT_TYPE_VPORT   = 22,	/*!< NPIV - virtual port */
};
typedef enum bfa_port_type bfa_port_type_t;

/**
 * @brief
 * 		Port topology setting. A port's topology and fabric login status
 * 		determine its operational type.
 */
enum bfa_port_topology {
	BFA_PORT_TOPOLOGY_NONE = 0,	/*!< No valid topology */
	BFA_PORT_TOPOLOGY_P2P_OLD_VER = 1,	/*!< P2P def for older ver */
	BFA_PORT_TOPOLOGY_LOOP = 2,	/*!< LOOP topology */
	BFA_PORT_TOPOLOGY_AUTO_OLD_VER = 3,	/*!< auto def for older ver */
	BFA_PORT_TOPOLOGY_AUTO = 4,	/*!< auto topology selection */
	BFA_PORT_TOPOLOGY_P2P  = 5,	/*!< P2P only */
};
typedef enum bfa_port_topology bfa_port_topology_t;

/**
 * @brief
 * 		Physical port loopback types.
 */
enum bfa_port_opmode {
	BFA_PORT_OPMODE_NORMAL   = 0x00, /*!< normal non-loopback mode */
	BFA_PORT_OPMODE_LB_INT   = 0x01, /*!< internal loop back */
	BFA_PORT_OPMODE_LB_SLW   = 0x02, /*!< serial link wrapback (serdes) */
	BFA_PORT_OPMODE_LB_EXT   = 0x04, /*!< external loop back (serdes) */
	BFA_PORT_OPMODE_LB_CBL   = 0x08, /*!< cabled loop back */
	BFA_PORT_OPMODE_LB_NLINT = 0x20, /*!< NL_Port internal loopback */
};
typedef enum bfa_port_opmode bfa_port_opmode_t;

#define BFA_PORT_OPMODE_LB_HARD(_mode)			\
	((_mode == BFA_PORT_OPMODE_LB_INT) ||		\
	(_mode == BFA_PORT_OPMODE_LB_SLW) ||		\
	(_mode == BFA_PORT_OPMODE_LB_EXT))

/**
 * @brief
 * 		Port link state
 */
enum bfa_port_linkstate {
	BFA_PORT_LINKUP		= 1,	/*!< Physical port/Trunk link up */
	BFA_PORT_LINKDOWN	= 2,	/*!< Physical port/Trunk link down */
};
typedef enum bfa_port_linkstate bfa_port_linkstate_t;

/**
 * @brief
 * 		Port link state reason code
 */
enum bfa_port_linkstate_rsn {
	BFA_PORT_LINKSTATE_RSN_NONE		= 0,
	BFA_PORT_LINKSTATE_RSN_DISABLED 	= 1,
	BFA_PORT_LINKSTATE_RSN_RX_NOS 		= 2,
	BFA_PORT_LINKSTATE_RSN_RX_OLS 		= 3,
	BFA_PORT_LINKSTATE_RSN_RX_LIP 		= 4,
	BFA_PORT_LINKSTATE_RSN_RX_LIPF7 	= 5,
	BFA_PORT_LINKSTATE_RSN_SFP_REMOVED 	= 6,
	BFA_PORT_LINKSTATE_RSN_PORT_FAULT 	= 7,
	BFA_PORT_LINKSTATE_RSN_RX_LOS 		= 8,
	BFA_PORT_LINKSTATE_RSN_LOCAL_FAULT 	= 9,
	BFA_PORT_LINKSTATE_RSN_REMOTE_FAULT 	= 10,
	BFA_PORT_LINKSTATE_RSN_TIMEOUT 		= 11,
	BFA_PORT_LINKSTATE_RSN_FAA_MISCONFIG	= 12,



	/* CEE related reason codes/errors */
	CEE_LLDP_INFO_AGED_OUT			= 20,
	CEE_LLDP_SHUTDOWN_TLV_RCVD		= 21,
	CEE_PEER_NOT_ADVERTISE_DCBX		= 22,
	CEE_PEER_NOT_ADVERTISE_PG		= 23,
	CEE_PEER_NOT_ADVERTISE_PFC		= 24,
	CEE_PEER_NOT_ADVERTISE_FCOE		= 25,
	CEE_PG_NOT_COMPATIBLE			= 26,
	CEE_PFC_NOT_COMPATIBLE			= 27,
	CEE_FCOE_NOT_COMPATIBLE			= 28,
	CEE_BAD_PG_RCVD				= 29,
	CEE_BAD_BW_RCVD				= 30,
	CEE_BAD_PFC_RCVD			= 31,
	CEE_BAD_APP_PRI_RCVD			= 32,
	CEE_FCOE_PRI_PFC_OFF			= 33,
	CEE_DUP_CONTROL_TLV_RCVD		= 34,
	CEE_DUP_FEAT_TLV_RCVD			= 35,
	CEE_APPLY_NEW_CFG			= 36, /* reason, not error */
	CEE_PROTOCOL_INIT			= 37, /* reason, not error */
	CEE_PHY_LINK_DOWN			= 38,
	CEE_LLS_FCOE_ABSENT			= 39,
	CEE_LLS_FCOE_DOWN			= 40,
	CEE_ISCSI_NOT_COMPATIBLE		= 41,
	CEE_ISCSI_PRI_PFC_OFF			= 42,
	CEE_ISCSI_PRI_OVERLAP_FCOE_PRI		= 43
};
typedef enum bfa_port_linkstate_rsn bfa_port_linkstate_rsn_t;

#define MAX_LUN_MASK_CFG 16

/**
 * Initially flash content may be fff. On making LUN mask enable and disable
 * state change.  when report lun command is being processed it goes from
 * BFA_LUN_MASK_ACTIVE to BFA_LUN_MASK_FETCH and comes back to
 * BFA_LUN_MASK_ACTIVE.
 */
enum bfa_ioim_lun_mask_state_s {
    BFA_IOIM_LUN_MASK_INACTIVE = 0,
    BFA_IOIM_LUN_MASK_ACTIVE = 1,
    BFA_IOIM_LUN_MASK_FETCHED = 2,
};

enum bfa_lunmask_state_s {
	BFA_LUNMASK_DISABLED = 0x00,
	BFA_LUNMASK_ENABLED = 0x01,
	BFA_LUNMASK_MINCFG = 0x02,
	BFA_LUNMASK_UNINITIALIZED = 0xff,
};

/**
 * FEC states
 */
enum bfa_fec_state_s {
	BFA_FEC_ONLINE = 1,		/*!< FEC is online */
	BFA_FEC_OFFLINE = 2,		/*!< FEC is offline */
	BFA_FEC_OFFLINE_NOT_16G = 3,	/*!< FEC is offline (speed not 16 Gig) */
};
typedef enum bfa_fec_state_s bfa_fec_state_t;

#pragma pack(1)
struct bfa_lun_mask_s {
        wwn_t           lp_wwn;
        wwn_t           rp_wwn;
        lun_t           lun;
        uint8_t         ua;
	uint8_t		rsvd[3];
        uint16_t        rp_tag;
        uint8_t         lp_tag;
        uint8_t         state;
};
typedef struct bfa_lun_mask_s   bfa_lun_mask_t;

struct bfa_lunmask_cfg_s {
	uint8_t status;
	uint8_t rsvd[7];
	struct bfa_lun_mask_s           lun_list[MAX_LUN_MASK_CFG];
};
typedef struct bfa_lunmask_cfg_s bfa_lunmask_cfg_t;

struct bfa_throttle_cfg_s {
	uint16_t is_valid;
	uint16_t value;
	uint32_t rsvd;
};
typedef struct bfa_throttle_cfg_s bfa_throttle_cfg_t;
#pragma pack()

#define BFA_MAX_TRS_MASK_CFG		16

struct bfa_trs_list_s {
	wwn_t	lpwwn;
	wwn_t	rpwwn;
};
typedef struct bfa_trs_list_s bfa_trs_list_t;

struct bfa_trs_mask_s {
	bfa_q_t		qe;
	bfa_trs_list_t	trs_list;
};
typedef struct bfa_trs_mask_s bfa_trs_mask_t;

struct bfa_trs_cfg_s {
	uint16_t	count;
	uint16_t	rsvd[3];
	bfa_q_t		trs_q;
};
typedef struct bfa_trs_cfg_s bfa_trs_cfg_t;

struct bfa_defs_fcpim_throttle_s {
	uint16_t	max_value;
	uint16_t	cur_value;
	uint16_t	cfg_value;
	uint16_t	rsvd;
};
typedef struct bfa_defs_fcpim_throttle_s bfa_defs_fcpim_throttle_t;

#define BFA_BB_SCN_DEF 3
#define BFA_BB_SCN_MAX 0x0F

#pragma pack(1)
/**
 * @brief
 *      Physical port configuration
 */
struct bfa_port_cfg_s {
	uint8_t	 topology;	/*!< bfa_port_topology		*/
	uint8_t	 speed;		/*!< enum bfa_port_speed	*/
	uint8_t	 trunked;	/*!< trunked or not		*/
	uint8_t	 qos_enabled;	/*!< qos enabled or not		*/
	uint8_t	 cfg_hardalpa;	/*!< is hard alpa configured	*/
	uint8_t	 hardalpa;	/*!< configured hard alpa	*/
	uint16_t maxfrsize;	/*!< maximum frame size		*/
	uint8_t	 rx_bbcredit;	/*!< receive buffer credits	*/
	uint8_t	 tx_bbcredit;	/*!< transmit buffer credits	*/
	uint8_t	 ratelimit;	/*!< ratelimit enabled or not	*/
	uint8_t	 trl_def_speed;	/*!< ratelimit default speed	*/
	uint8_t	 bb_cr_enabled; /*!< Config state of BB_SCN	*/
	uint8_t	 bb_scn;	/*!< BB_SCN value for FLOGI Exchg */
	uint8_t	 faa_state;	/*!< FAA enabled/disabled	*/
	uint8_t  rsvd[1];
	uint16_t path_tov;	/*!< device path timeout	*/
	uint16_t q_depth;	/*!< SCSI Queue depth		*/
	bfa_qos_bw_t	qos_bw;	/*!< QOS bandwidth		*/
};
typedef struct bfa_port_cfg_s bfa_port_cfg_t;
#pragma pack()

/**
 * @brief
 * 		Port attribute values.
 */
struct bfa_port_attr_s {
	/*
	 * Static fields
	 */
	wwn_t	   nwwn;		/*!< node wwn */
	wwn_t	   pwwn;		/*!< port wwn */
	wwn_t	   factorynwwn;	/*!< factory node wwn */
	wwn_t	   factorypwwn;	/*!< factory port wwn */
	fc_cos_t	cos_supported;	/*!< supported class of services */
	uint32_t	rsvd;
	fc_symname_t    port_symname;	/*!< port symbolic name */
	enum bfa_port_speed speed_supported; /*!< supported speeds */
	bfa_boolean_t   pbind_enabled;	/*!< Relevant only in Windows */

	/*
	 * Configured values
	 */
	struct bfa_port_cfg_s pport_cfg;	/*!< pport cfg */

	/*
	 * Dynamic field - info from BFA
	 */
	enum bfa_port_states 	port_state;	/*!< current port state */
	enum bfa_port_speed 	speed;		/*!< current speed */
	enum bfa_port_topology 	topology;	/*!< current topology */
	bfa_boolean_t		beacon;		/*!< current beacon status */
	bfa_boolean_t		link_e2e_beacon; /*!< link beacon is on */
	bfa_boolean_t		bbsc_op_status;	/*!< fc credit recovery oper state*/
	bfa_fec_state_t		fec_state;	/*!< current FEC state */

	/*
	 * Dynamic field - info from FCS
	 */
	uint32_t		pid;		/*!< port ID */
	enum bfa_port_type 	port_type;	/*!< current topology */
	uint32_t		loopback;	/*!< external loopback */
	uint32_t		authfail;	/*!< auth fail state */

	/* FCoE specific  */
	uint16_t		fcoe_vlan;
	uint8_t			rsvd1[2];
};
typedef struct bfa_port_attr_s bfa_port_attr_t;

/*
 * FAA state
 */
enum bfa_faa_state {
	BFA_FAA_ENABLED		= 1,
	BFA_FAA_DISABLED	= 2,
};
typedef enum bfa_faa_state bfa_faa_state_t;

/**
 * FAA source
 */
enum bfa_faa_source {
	BFA_FAA_FACTORY		= 1,
	BFA_FAA_BLADE_SYSTEM	= 2,
	BFA_FAA_FABRIC		= 3,
};
typedef enum bfa_faa_source bfa_faa_source_t;

/**
 * FAA attributes
 */
struct bfa_faa_attr_s {
	wwn_t		faa;
	uint8_t		faa_state;
	uint8_t		pwwn_source;
	uint8_t		rsvd[6];
};
typedef struct bfa_faa_attr_s bfa_faa_attr_t;

/**
 * @brief
 *	      Port FCP mappings.
 */
struct bfa_port_fcpmap_s {
	char		osdevname[256];
	uint32_t	bus;
	uint32_t	target;
	uint32_t	oslun;
	uint32_t	fcid;
	wwn_t	   nwwn;
	wwn_t	   pwwn;
	uint64_t	fcplun;
	char		luid[256];
};
typedef struct bfa_port_fcpmap_s bfa_port_fcpmap_t;

/**
 * @brief
 *	      Port RNID info.
 */
struct bfa_port_rnid_s {
	wwn_t	     wwn;
	uint32_t	  unittype;
	uint32_t	  portid;
	uint32_t	  attached_nodes_num;
	uint16_t	  ip_version;
	uint16_t	  udp_port;
	uint8_t	   ipaddr[16];
	uint16_t	  rsvd;
	uint16_t	  topologydiscoveryflags;
};
typedef struct bfa_port_rnid_s bfa_port_rnid_t;

#pragma pack(1)
struct bfa_fcport_fcf_s {
	wwn_t	   name;	   /*!< FCF name		 */
	wwn_t	   fabric_name;    /*!< Fabric Name	      */
	uint8_t		fipenabled;	/*!< FIP enabled or not */
	uint8_t		fipfailed;	/*!< FIP failed or not	*/
	uint8_t		resv[2];
	uint8_t	 pri;	    /*!< FCF priority	     */
	uint8_t	 version;	/*!< FIP version used	 */
	uint8_t	 available;      /*!< Available  for  login    */
	uint8_t	 fka_disabled;   /*!< FKA is disabled	  */
	uint8_t	 maxsz_verified; /*!< FCoE max size verified   */
	uint8_t	 fc_map[3];      /*!< FC map		   */
	uint16_t	vlan;	   /*!< FCoE vlan tag/priority   */
	uint32_t	fka_adv_per;    /*!< FIP  ka advert. period   */
	mac_t	   mac;	    /*!< FCF mac		  */
};
typedef struct bfa_fcport_fcf_s bfa_fcport_fcf_t;

/**
 * @brief
 *	Trunk states for BCU/BFAL
 */
enum bfa_trunk_state {
	BFA_TRUNK_DISABLED	= 0,	/*!< Trunk is not configured	*/
	BFA_TRUNK_ONLINE	= 1,	/*!< Trunk is online		*/
	BFA_TRUNK_OFFLINE	= 2,	/*!< Trunk is offline		*/
};
typedef enum bfa_trunk_state bfa_trunk_state_t;

/**
 * @brief
 *	VC attributes for trunked link
 */
struct bfa_trunk_vc_attr_s {
	uint32_t bb_credit;
	uint32_t elp_opmode_flags;
	uint32_t req_credit;
	uint16_t vc_credits[8];
};
typedef struct bfa_trunk_vc_attr_s bfa_trunk_vc_attr_t;

struct bfa_fcport_loop_info_s {
	uint8_t         myalpa;    /*!< alpa claimed */
	uint8_t         alpabm_val; /*!< alpa bitmap valid or not (1 or 0) */
	uint8_t		resvd[6];
	fc_alpabm_t     alpabm;    /*!< alpa bitmap */
};
typedef struct bfa_fcport_loop_info_s bfa_fcport_loop_info_t;

/**
 * @brief
 * 		Link state information
 */
struct bfa_port_link_s {
	uint8_t	 linkstate;	/*!< Link state bfa_port_linkstate */
	uint8_t	 linkstate_rsn;	/*!< bfa_port_linkstate_rsn_t */
	uint8_t	 topology;	/*!< P2P/LOOP bfa_port_topology */
	uint8_t	 speed;		/*!< Link speed (1/2/4/8 G) */
	uint32_t	linkstate_opt;  /*!< Linkstate optional data (debug) */
	uint8_t	 trunked;	/*!< Trunked or not (1 or 0) */
	uint8_t  fec_state;	/*!< State of FEC */
	uint8_t	 resvd[6];
	struct bfa_qos_attr_s  qos_attr;   /* QoS Attributes */
	union {
		struct bfa_fcport_loop_info_s loop_info;
		struct bfa_bbcr_attr_s bbcr_attr;
		union {
			struct bfa_qos_vc_attr_s qos_vc_attr;
				  /*!< VC info from ELP */
			struct bfa_trunk_vc_attr_s trunk_vc_attr;
			struct bfa_fcport_fcf_s fcf;
				 /*!< FCF information (for FCoE) */
		} vc_fcf;
	} attr;

};
typedef struct bfa_port_link_s bfa_port_link_t;
#pragma pack()

enum bfa_trunk_link_fctl {
	BFA_TRUNK_LINK_FCTL_NORMAL,
	BFA_TRUNK_LINK_FCTL_VC,
	BFA_TRUNK_LINK_FCTL_VC_QOS,
};
typedef enum bfa_trunk_link_fctl bfa_trunk_link_fctl_t;

enum bfa_trunk_link_state {
	BFA_TRUNK_LINK_STATE_UP = 1,		/* link part of trunk */
	BFA_TRUNK_LINK_STATE_DN_LINKDN = 2,	/* physical link down */
	BFA_TRUNK_LINK_STATE_DN_GRP_MIS = 3,	/* trunk group different */
	BFA_TRUNK_LINK_STATE_DN_SPD_MIS = 4,	/* speed mismatch */
	BFA_TRUNK_LINK_STATE_DN_MODE_MIS = 5,	/* remote port not trunked */
};
typedef enum bfa_trunk_link_state bfa_trunk_link_state_t;

#define BFA_TRUNK_MAX_PORTS	2
struct bfa_trunk_link_attr_s {
	wwn_t    trunk_wwn;
	enum bfa_trunk_link_fctl fctl;
	enum bfa_trunk_link_state link_state;
	enum bfa_port_speed	speed;
	uint32_t deskew;
};
typedef struct bfa_trunk_link_attr_s bfa_trunk_link_attr_t;

struct bfa_trunk_attr_s {
	enum bfa_trunk_state	state;
	enum bfa_port_speed	speed;
	uint32_t		port_id;
	uint32_t		rsvd;
	struct bfa_trunk_link_attr_s link_attr[BFA_TRUNK_MAX_PORTS];
};
typedef struct bfa_trunk_attr_s bfa_trunk_attr_t;

struct bfa_rport_hal_stats_s {
	uint32_t        sm_un_cr;	    /*!< uninit: create events      */
	uint32_t        sm_un_unexp;	    /*!< uninit: exception events   */
	uint32_t        sm_cr_on;	    /*!< created: online events     */
	uint32_t        sm_cr_del;	    /*!< created: delete events     */
	uint32_t        sm_cr_hwf;	    /*!< created: IOC down          */
	uint32_t        sm_cr_unexp;	    /*!< created: exception events  */
	uint32_t        sm_fwc_rsp;	    /*!< fw create: f/w responses   */
	uint32_t        sm_fwc_del;	    /*!< fw create: delete events   */
	uint32_t        sm_fwc_off;	    /*!< fw create: offline events  */
	uint32_t        sm_fwc_hwf;	    /*!< fw create: IOC down        */
	uint32_t        sm_fwc_unexp;	    /*!< fw create: exception events*/
	uint32_t        sm_on_off;	    /*!< online: offline events     */
	uint32_t        sm_on_del;	    /*!< online: delete events      */
	uint32_t        sm_on_hwf;	    /*!< online: IOC down events    */
	uint32_t        sm_on_unexp;	    /*!< online: exception events   */
	uint32_t        sm_fwd_rsp;	    /*!< fw delete: fw responses    */
	uint32_t        sm_fwd_del;	    /*!< fw delete: delete events   */
	uint32_t        sm_fwd_hwf;	    /*!< fw delete: IOC down events */
	uint32_t        sm_fwd_unexp;	    /*!< fw delete: exception events*/
	uint32_t        sm_off_del;	    /*!< offline: delete events     */
	uint32_t        sm_off_on;	    /*!< offline: online events     */
	uint32_t        sm_off_hwf;	    /*!< offline: IOC down events   */
	uint32_t        sm_off_unexp;	    /*!< offline: exception events  */
	uint32_t        sm_del_fwrsp;	    /*!< delete: fw responses       */
	uint32_t        sm_del_hwf;	    /*!< delete: IOC down events    */
	uint32_t        sm_del_unexp;	    /*!< delete: exception events   */
	uint32_t        sm_delp_fwrsp;	    /*!< delete pend: fw responses  */
	uint32_t        sm_delp_hwf;	    /*!< delete pend: IOC downs     */
	uint32_t        sm_delp_unexp;	    /*!< delete pend: exceptions    */
	uint32_t        sm_offp_fwrsp;	    /*!< off-pending: fw responses  */
	uint32_t        sm_offp_del;	    /*!< off-pending: deletes       */
	uint32_t        sm_offp_hwf;	    /*!< off-pending: IOC downs     */
	uint32_t        sm_offp_unexp;	    /*!< off-pending: exceptions    */
	uint32_t        sm_iocd_off;	    /*!< IOC down: offline events   */
	uint32_t        sm_iocd_del;	    /*!< IOC down: delete events    */
	uint32_t        sm_iocd_on;	    /*!< IOC down: online events    */
	uint32_t        sm_iocd_unexp;	    /*!< IOC down: exceptions       */
	uint32_t        rsvd;
};
typedef struct bfa_rport_hal_stats_s bfa_rport_hal_stats_t;
#pragma pack(1)
/**
 *  Rport's QoS attributes
 */
struct bfa_rport_qos_attr_s {
	uint8_t			qos_priority;  /*!< rport's QoS priority   */
	uint8_t			rsvd[3];
	uint32_t	       qos_flow_id;	  /*!< QoS flow Id	 */
};
typedef struct bfa_rport_qos_attr_s bfa_rport_qos_attr_t;
#pragma pack()

#define BFA_IOBUCKET_MAX 5
#define BFA_IOPF_MAX 256
#define BFA_IOPF_DA_MAX 256
#define BFA_IOPF_INVALID_ID 0xffff
#define BFA_FCPIM_LUN_INVALID ~0



struct bfa_itnim_latency_s {
	uint64_t min[BFA_IOBUCKET_MAX];
	uint64_t max[BFA_IOBUCKET_MAX];
	uint64_t count[BFA_IOBUCKET_MAX];
	uint64_t avg[BFA_IOBUCKET_MAX];
	uint32_t iocomps[BFA_IOBUCKET_MAX];	/*!< IO completed	*/
	uint32_t clock_res_mul;
	uint32_t clock_res_div;
	uint32_t iopf_start_time;	/*!< IO profile start time	*/
	uint64_t total_iolat_avg;	/*!< IO latency avarage 	*/
};
typedef struct bfa_itnim_latency_s bfa_itnim_latency_t;

enum bfa_fcpim_iopf_lun_type {
	BFA_FCPIM_IOPF_NONE = 0, /* IT level profile */
	BFA_FCPIM_IOPF_ALL = 2,
	BFA_FCPIM_IOPF_RANGE = 3,
	BFA_FCPIM_IOPF_ONE_LUN = 4,
};

enum bfa_fcpim_iopf_data_type {
	BFA_FCPIM_IOPF_DATA_NONE = 0,
	BFA_FCPIM_IOPF_DATA_ITN = 2,
	BFA_FCPIM_IOPF_DATA_LUN = 3,
};


/**
 * @brief 
 * this is config and data for IO profile stored in fcpim.
 */
struct bfa_fcpim_iopf_s {
	wwn_t rpwwn;
	wwn_t lpwwn;
	lun_t lun;
	uint16_t id;
	uint8_t lun_status;
	uint8_t show_done; /* TRUE means get data FALSE means ignore */
	enum bfa_fcpim_iopf_lun_type iopf_type;
	struct bfa_itnim_latency_s io_lat;
	struct bfa_itnim_iostats_s io_st;
};
typedef struct bfa_fcpim_iopf_s bfa_fcpim_iopf_t;

/**
 * this array keeps config and data for IO profile just used in
 * ioctl context
 */
struct bfa_fcpim_iopf_cfg_s {
	wwn_t rpwwn;
	wwn_t lpwwn;
	lun_t lun_id;
	enum bfa_fcpim_iopf_lun_type iopf_type;
	uint8_t lun_status;
	uint8_t rsvd[3];
};
typedef struct bfa_fcpim_iopf_cfg_s bfa_fcpim_iopf_cfg_t;

/**
 * @brief
 * 		Queue test results
 */
typedef struct bfa_diag_qtest_result_s {
	uint32_t	status;
	uint16_t	count;		/* sucessful queue test count */
	uint8_t		queue;
	uint8_t		rsvd;	/* 64-bit align */
} bfa_diag_qtest_result_t;

/**
 * @brief
 * 		vHBA port attribute values.
 */
struct bfa_vhba_attr_s {
	wwn_t			nwwn;		/*!< node wwn */
	wwn_t			pwwn;		/*!< port wwn */
	uint32_t		pid;		/*!< port ID */
	bfa_boolean_t		plog_enabled;	/*!< portlog is enabled */
	uint16_t		path_tov;
	uint8_t			rsvd[6];
};
typedef struct bfa_vhba_attr_s bfa_vhba_attr_t;

struct bfa_pcifn_attr_s {
	enum bfa_ioc_type_e fn_type;
	uint16_t queue_pairs;
	uint16_t mbox_res; /* Mbox resources */
	uint32_t min_msix;
	uint32_t max_msix;
	bfa_boolean_t optrom_enabled;
	bfa_boolean_t sriov_capable;
	uint16_t max_vfs;
	uint16_t active_vfs;
};
typedef struct bfa_pcifn_attr_s bfa_pcifn_attr_t;

struct bfa_port_cfg_mode_s {
	uint16_t max_pf;
	uint16_t max_vf;
	bfa_mode_t mode;
};
typedef struct bfa_port_cfg_mode_s bfa_port_cfg_mode_t;


#endif /* __BFA_DEFS_SVC_H__ */
