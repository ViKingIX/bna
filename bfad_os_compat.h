/* 
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 * 
 * See LICENSE.bna for copyright and licensing details.
 */

#ifndef __BFAD_OS_COMPAT_H__
#define __BFAD_OS_COMPAT_H__

#include <linux/types.h>
#include <linux/delay.h>
#include <linux/param.h>
#include <linux/time.h>

#define DMA_64BIT_MASK 0xffffffffffffffffULL
#define DMA_32BIT_MASK 0x00000000ffffffffULL

#define ULONG_MAX (~0UL)
#define UINT_MAX (~0U)

#define MODULE_VERSION(ver_str)

#if HZ <= 1000 && !(1000 % HZ)
#  define MAX_MSEC_OFFSET \
	(ULONG_MAX - (1000 / HZ) + 1)
#elif HZ > 1000 && !(HZ % 1000)
#  define MAX_MSEC_OFFSET \
	(ULONG_MAX / (HZ / 1000))
#else
#  define MAX_MSEC_OFFSET \
	((ULONG_MAX - 999) / HZ)
#endif

static inline unsigned long msecs_to_jiffies(const unsigned int m)
{
	if (MAX_MSEC_OFFSET < UINT_MAX && m > (unsigned int)MAX_MSEC_OFFSET)
		return MAX_JIFFY_OFFSET;
#if HZ <= 1000 && !(1000 % HZ)
	return ((unsigned long)m + (1000 / HZ) - 1) / (1000 / HZ);
#elif HZ > 1000 && !(HZ % 1000)
	return (unsigned long)m * (HZ / 1000);
#else
	return ((unsigned long)m * HZ + 999) / 1000;
#endif
}

struct device {
	struct pci_dev pdev;
};

static inline struct pci_dev *to_pci_dev(struct device *dev)
{
	return (struct pci_dev *) dev;
}

/* NOTE: dangerous! we ignore the 'gfp' argument */
#define dma_alloc_coherent(dev,sz,dma,gfp) \
	pci_alloc_consistent(to_pci_dev((struct device *)dev),(sz),(dma))

#define dma_free_coherent(dev,sz,addr,dma_addr) \
	pci_free_consistent(to_pci_dev((struct device *)dev),(sz),(addr),(dma_addr))
#define dma_map_sg(dev, a, b, c) \
	pci_map_sg(to_pci_dev(dev), (a), (b), (c))
#define dma_unmap_sg(dev, a, b, c) \
	pci_unmap_sg(to_pci_dev(dev), (a), (b), (c))
#define dma_map_single(dev, a, b, c) \
	pci_map_single(to_pci_dev(dev), (a), (b), (c))
#define dma_unmap_single(dev, a, b, c) \
	pci_unmap_single(to_pci_dev(dev), (a), (b), (c))

#define dev_printk(level, bus, format, arg...)  \
    printk(level "%s %d: " format, dev_driver_string(bus), (bus)->number, \
			## arg)
#ifdef BFA_DEV_PRINTF
#undef BFA_DEV_PRINTF
#endif

#define BFA_DEV_PRINTF(bfad, level, fmt, arg...)	\
		BFA_PRINTF(level, fmt, ##arg)
//		BFA_PRINTF(level, fmt, ## bfad->pcidev)

#ifndef __user
#define __user
#endif

/* These definitions mirror those in pci.h, so they can be used
 * interchangeably with their PCI_ counterparts */
enum dma_data_direction {
	DMA_BIDIRECTIONAL = 0,
	DMA_TO_DEVICE = 1,
	DMA_FROM_DEVICE = 2,
	DMA_NONE = 3,
};

static inline void
pci_dma_sync_single_for_device(struct pci_dev *hwdev, dma_addr_t dma_handle,
                   size_t size, int direction)
{
    /* No flushing needed to sync cpu writes to the device.  */
    //BUG_ON(direction == PCI_DMA_NONE);
	return;
}

#define IRQ_HANDLED
#define IRQ_NONE
#define scsi_host_template	SHT

#define mmiowb()

/* TBD: CC_KFLAGS += -DCONFIG_PCI_MSI => remove the 2 bellow { */

#if defined(__VMKERNEL_MODULE__) || (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0))
extern int pci_enable_msix(struct pci_dev *dev,	
							struct msix_entry *entries, int nvec);
extern void pci_disable_msix(struct pci_dev *dev);
#endif

/*
static inline int pci_enable_msix(struct pci_dev *dev,					\
								struct msix_entry *entries, int nvec)	\
		{ return -1; }
static inline void pci_disable_msix(struct pci_dev *dev) {}
*/

/* } TBD: CC_KFLAGS += -DCONFIG_PCI_MSI => remove the 2 bellow */

#ifndef VMK_CACHE_SIZE
#define VMK_CACHE_SIZE 128
#endif

#ifndef __iomem 
#define __iomem
#endif

#ifndef __force 
#define __force
#endif

typedef struct spin_lock_s {
	spinlock_t	io_req_lock;
	char		pad[VMK_CACHE_SIZE - sizeof(spinlock_t)];
} spin_lock_t __attribute__ ((__aligned__(VMK_CACHE_SIZE)));

#endif
