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
 * Contains declarations of linux specific calls required for QLogic
 * FCoE HBA driver implementation. Each OS has to provide this file.
 */

/**
 * @file cna_os.h Linux driver OS specific declarations.
 */

#ifndef __CNA_OS_H__
#define __CNA_OS_H__

#ifdef __KERNEL__
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/if_vlan.h>
#include <linux/if_ether.h>
#include <asm/page.h>
#include <asm/io.h>
#include <asm/string.h>
#else
#include <sys/io.h>
#endif

#ifdef __KERNEL__

#define CNA_PAGE_SIZE		PAGE_SIZE
#define CNA_PAGE_SHIFT		PAGE_SHIFT

#define CNA_ETH_ALEN		ETH_ALEN

#define CNA_ASSERT_PRINTK(p) \
	printk(KERN_ERR "assert(%s) failed at %s:%d\n", #p, __FILE__, __LINE__)

#ifdef CNA_ASSERT_PRINTK_ONLY
#define CNA_ASSERT_BUG()
#else
#define CNA_ASSERT_BUG()	BUG()
#endif

#ifndef CNA_PERF_BUILD

#define CNA_ASSERT(p) do {						\
	if (!(p)) {							\
		CNA_ASSERT_PRINTK(p);					\
		CNA_ASSERT_BUG();					\
	}								\
} while (0)

#define CNA_ASSERT_PERF(p) do {						\
	if (unlikely(!(p))) {						\
		CNA_ASSERT_PRINTK(p);					\
	}								\
} while (0)

#else /* CNA_PERF_BUILD */

#define CNA_ASSERT(p)
#define CNA_ASSERT_PERF(p)

#endif /* CNA_PROD_BUILD */

#else /*  __KERNEL__ */

#define CNA_ASSERT(p)
#define CNA_ASSERT_PERF(p)

#endif /*  !__KERNEL__ */

#define CNA_ASSERT_DEBUG		CNA_ASSERT_PERF

#define cna_os_swap_8b(_x)		swab64((_x))

#define cna_os_swap32(_x)		swab32((_x))

#define cna_os_htons			htons
#define cna_os_ntohs			ntohs
#define cna_os_htonl			htonl
#define cna_os_ntohl			ntohl
#define cna_os_htonll			cpu_to_be64
#define cna_os_ntohll			be64_to_cpu

#define cna_os_wtole			cpu_to_le32
#define cna_os_wtobe			cpu_to_be32

#define cna_os_dma_addr64		cpu_to_be64

#define cna_os_memset			memset
#define cna_os_memcpy			memcpy
#define cna_os_memcmp			memcmp
#define cna_os_vsprintf 		vsprintf
#define cna_os_snprintf 		snprintf

#define cna_os_vmalloc(_size)	vmalloc((_size))
#define cna_os_vfree(_buf, _size) vfree((_buf))

#define cna_os_kzalloc(_size)	kzalloc((_size), GFP_KERNEL);
#define cna_os_kcalloc(_num, _sizeof_each)	\
			kcalloc((_num), (_sizeof_each), GFP_KERNEL);
#define cna_os_kfree(_buf, _size) kfree((_buf))

#define CNA_ALIGN(x, a)		(((x) + (a) - 1) & ~((a) - 1))

typedef void *				cna_os_addr_t;
typedef void * __iomem			cna_iomem_t;

typedef unsigned long			cna_off_t;

#define cna_irqreturn_t			irqreturn_t

#define CNA_IRQ_HANDLED			IRQ_HANDLED

#define cna_reg_read(_raddr)		readl(_raddr)
#define cna_reg_write(_raddr, _val)	writel(_val, _raddr)

#define cna_mem_readw(_raddr)		cna_reg_read(_raddr)
#define cna_mem_writew(_raddr, _val)	cna_reg_write(_raddr, _val)

/* Required for DMA address manipulation in IOC */
#define bfa_os_u32(__pa64) ((__pa64) >> 32)
#define cna_os_u64(__a32) (((uint64_t)cna_os_ntohl(__a32)) << 32)



extern char bfa_version [];

static inline int
cna_os_cp_to_user(void *usr_buf, void *drv_buf, int size, uint32_t mode)
{
	if (copy_to_user((void __user *)usr_buf, (uint8_t *)drv_buf, size)) {
		return EIO;
	}
	return 0;
}

static inline int
cna_os_cp_from_user(void *drv_buf, void *usr_buf, int size, uint32_t mode)
{
	if (copy_from_user((void __user *)drv_buf, (uint8_t *)usr_buf, size)) {
		return EIO;
	}
	return 0;
}

#endif /* __CNA_OS_H__ */
