/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

/**
 * @brief
 * Contains declarations all OS Specific files needed for BFA layer
 */

#ifndef __BFA_OS_INC_H__
#define __BFA_OS_INC_H__

#define bfa_swap_8b(_x)                                 \
	((((_x) & 0xff00000000000000ull) >> 56)         \
	| (((_x) & 0x00ff000000000000ull) >> 40)       \
	| (((_x) & 0x0000ff0000000000ull) >> 24)       \
	| (((_x) & 0x000000ff00000000ull) >> 8)        \
	| (((_x) & 0x00000000ff000000ull) << 8)        \
	| (((_x) & 0x0000000000ff0000ull) << 24)       \
	| (((_x) & 0x000000000000ff00ull) << 40)       \
	| (((_x) & 0x00000000000000ffull) << 56))

#define bfa_os_swap32(_x)                       \
	((((_x) & 0xff) << 24)		|       \
	(((_x) & 0x0000ff00) << 8)      |       \
	(((_x) & 0x00ff0000) >> 8)      |       \
	(((_x) & 0xff000000) >> 24))


#ifndef __KERNEL__
#include <stdint.h>


#define bfa_os_swap64(_x) bfa_swap_8b(_x)

#else
#include <linux/types.h>

#include <linux/version.h>
#include <linux/pci.h>

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 0))
typedef void irqreturn_t;
#include <bfad_os_compat.h>
#ifdef __VMKERNEL_MODULE__
#define SET_MODULE_VERSION(VER) 		\
	if (!vmk_set_module_version("%s", VER))	\
		return 1;
#define poll_wait(X, Y, Z)
#else
#define SET_MODULE_VERSION(VER)
#endif
#else
#include <linux/dma-mapping.h>
#define SET_MODULE_VERSION(VER)
#endif

#include <linux/idr.h>

#include <linux/interrupt.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0))
#include <linux/cdev.h>
#endif
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/vmalloc.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0)
#include <linux/workqueue.h>
#else
#include <linux/tqueue.h>
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 0))
#include <linux/blk.h>
#include <sd.h>
#include <scsi/scsi.h>
#include <hosts.h>
#else
#include <scsi/scsi.h>
#include <scsi/scsi_host.h>
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0))
#include <scsi/scsi_tcq.h>
#endif
#if defined(__VMKERNEL_MODULE__) || \
		 (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 0))
#include <scsi/scsi_transport_fc.h>
#include <scsi/scsi_transport.h>
#endif

#if (LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 31))
#include <scsi/scsi_bsg_fc.h>
#include <scsi/scsi_devinfo.h>
#define BFAD_DEF_LUNMASK_SUPPORT	BFA_TRUE
#else
#define BFAD_DEF_LUNMASK_SUPPORT	BFA_FALSE
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0))
#include <linux/export.h>
#endif

#ifdef __BIG_ENDIAN
#define __BIGENDIAN
#endif



static inline uint64_t bfa_os_get_log_time(void)
{
	uint64_t system_time = 0;
	struct timeval tv;
	do_gettimeofday(&tv);

	/* We are interested in seconds only. */
	system_time = tv.tv_sec;
	return system_time;
}

#define bfa_io_lat_clock_res_div 1
#define bfa_io_lat_clock_res_mul 1

#define BFA_ERR			KERN_ERR
#define BFA_WARNING		KERN_WARNING
#define BFA_NOTICE		KERN_NOTICE
#define BFA_INFO		KERN_INFO
#define BFA_DEBUG		KERN_DEBUG

#define LOG_BFAD_INIT		0x00000001
#define LOG_FCP_IO		0x00000002

#ifdef DEBUG
#define BFA_LOG_TRACE(bfad, level, mask, fmt, arg...)			\
		BFA_LOG(bfad, level, mask, fmt, ## arg)
#define BFA_DEV_TRACE(bfad, level, fmt, arg...)				\
		BFA_DEV_PRINTF(bfad, level, fmt, ## arg)
#define BFA_TRACE(level, fmt, arg...)					\
		BFA_PRINTF(level, fmt, ## arg)
#else
#define BFA_LOG_TRACE(bfad, level, mask, fmt, arg...)
#define BFA_DEV_TRACE(bfad, level, fmt, arg...)
#define BFA_TRACE(level, fmt, arg...)
#endif

#define BFA_ASSERT(p) do {						\
	if (!(p)) {							\
		printk(KERN_ERR "assert(%s) failed at %s:%d\n", #p,	\
		__FILE__, __LINE__);					\
		BUG();							\
	}								\
} while (0)


#define BFA_LOG(bfad, level, mask, fmt, arg...)				\
do { 									\
	if (((mask) & (((struct bfad_s *)(bfad))->			\
		cfg_data[cfg_log_mask])) || (level[1] <= '3'))		\
		dev_printk(level, &(((struct bfad_s *)			\
			(bfad))->pcidev->dev), fmt, ##arg);		\
} while (0)

#ifndef BFA_DEV_PRINTF
#define BFA_DEV_PRINTF(bfad, level, fmt, arg...)			\
		dev_printk(level, &(((struct bfad_s *)			\
			(bfad))->pcidev->dev), fmt, ##arg);
#endif

#define BFA_PRINTF(level, fmt, arg...)					\
		printk(level fmt, ##arg);

int bfa_os_MWB(void *);

#define bfa_os_mmiowb()		mmiowb()

#define bfa_swap_3b(_x)				\
	((((_x) & 0xff) << 16) |		\
	((_x) & 0x00ff00) |			\
	(((_x) & 0xff0000) >> 16))


#ifndef __BIGENDIAN
#define bfa_os_htons(_x) ((uint16_t)((((_x) & 0xff00) >> 8) | \
				 (((_x) & 0x00ff) << 8)))

#define bfa_os_htonl(_x)	bfa_os_swap32(_x)
#define bfa_os_htonll(_x)	bfa_swap_8b(_x)
#define bfa_os_hton3b(_x)	bfa_swap_3b(_x)

#define bfa_os_wtole(_x)   (_x)

#else

#define bfa_os_htons(_x)   (_x)
#define bfa_os_htonl(_x)   (_x)
#define bfa_os_hton3b(_x)  (_x)
#define bfa_os_htonll(_x)  (_x)
#define bfa_os_wtole(_x)   bfa_os_swap32(_x)

#endif

#define bfa_os_ntohs(_x)   bfa_os_htons(_x)
#define bfa_os_ntohl(_x)   bfa_os_htonl(_x)
#define bfa_os_ntohll(_x)  bfa_os_htonll(_x)
#define bfa_os_ntoh3b(_x)  bfa_os_hton3b(_x)

#define bfa_os_u32(__pa64) ((__pa64) >> 32)

#define bfa_os_memset	memset
#define bfa_os_memcpy	memcpy
#define bfa_os_udelay	udelay
#define bfa_os_vsnprintf vsnprintf
#define bfa_os_snprintf snprintf

#define bfa_os_assign(__t, __s) __t = __s

#define bfa_os_addr_t void __iomem *
#define bfa_os_panic()

#define bfa_os_reg_read(_raddr) readl(_raddr)
#define bfa_os_reg_write(_raddr, _val) writel(((_val)), (_raddr))
#define bfa_os_mem_read(_raddr, _off)                                   \
	bfa_os_swap32(readl((_raddr) + (_off)))
#define bfa_os_mem_write(_raddr, _off, _val)                            \
	writel(bfa_os_swap32((_val)), ((_raddr) + (_off)))

#define BFA_TRC_TS(_trcm) 				\
({							\
		struct timeval tv;			\
							\
		do_gettimeofday(&tv);			\
		(tv.tv_sec*1000000+tv.tv_usec);		\
})

struct bfa_log_mod_s;
void bfa_os_printf(struct bfa_log_mod_s *log_mod, uint32_t msg_id,
			va_list params, const char *fmt, ...);
#endif
#define bfa_os_memcpy	memcpy
#define boolean_t int

/**
 * For current time stamp, OS API will fill-in
 */
struct bfa_timeval_s {
	uint32_t	tv_sec;		/*!< seconds	*/
	uint32_t	tv_usec;	/*!< microseconds   */
};
typedef struct bfa_timeval_s bfa_timeval_t;

void bfa_os_gettimeofday(struct bfa_timeval_s *tv);

static inline uint64_t bfa_os_get_clock(void *hcb_bfad)
{
	struct bfa_timeval_s tv;
	bfa_os_gettimeofday(&tv);
	return (uint64_t)tv.tv_usec;
}
#endif /* __BFA_OS_INC_H__ */
