/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 Qlogic Corp.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

#include "cna_os.h"
#include "bna.h"

#include "bnad_compat.h"
#include "bnad_trcmod.h"
#include "bnad.h"
#include "bfad_ioctl_cna.h"
#include "bnad_ioctl.h"

BNA_TRC_FILE(LDRV, COMPAT);

#ifdef CONFIG_COMPAT
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 9)
#include <linux/ioctl32.h>
#endif
#endif

#ifdef CONFIG_COMPAT
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 9)
static int reg_ioctl_cmd;
#endif
#endif

#if defined(BNAD_GRO)
uint bnad_gro_disable = 0;
module_param(bnad_gro_disable, uint, 0444);
MODULE_PARM_DESC(bnad_gro_disable, "Disable GRO");

uint bnad_multi_buffer_rx = 1;
module_param(bnad_multi_buffer_rx, uint, 0444);
MODULE_PARM_DESC(bnad_multi_buffer_rx,
				 "Multi-buffer Rx; 1 - en(default), 0 - dis");
#elif defined(BNAD_LRO)
uint bnad_lro_disable = 0;
module_param(bnad_lro_disable, uint, 0444);
MODULE_PARM_DESC(bnad_lro_disable, "Disable LRO");
#endif

static int bna_major;
unsigned int bnad_ioctl_codes[] = {
	/* IOC */
	BNAD_IOCMD(IOCMD_IOC_GET_VERSION),
	BNAD_IOCMD(IOCMD_IOC_GET_INSTS),
	BNAD_IOCMD(IOCMD_IOC_ENABLE),
	BNAD_IOCMD(IOCMD_IOC_DISABLE),
	BNAD_IOCMD(IOCMD_IOC_GET_INFO),
	BNAD_IOCMD(IOCMD_IOC_GET_ATTR),
	BNAD_IOCMD(IOCMD_IOC_GET_STATS),
	BNAD_IOCMD(IOCMD_IOC_RESET_STATS),
	BNAD_IOCMD(IOCMD_IOC_SET_ADAPTER_NAME),
	BNAD_IOCMD(IOCMD_IOC_SET_PORT_NAME),
	BNAD_IOCMD(IOCMD_IOC_FW_SIG_INV),
	/* PORT */
	BNAD_IOCMD(IOCMD_PORT_ENABLE),
	BNAD_IOCMD(IOCMD_PORT_DISABLE),
	BNAD_IOCMD(IOCMD_PORT_GET_STATS),
	BNAD_IOCMD(IOCMD_PORT_RESET_STATS),

	/* ETHPORT */
	BNAD_IOCMD(IOCMD_ETHPORT_GET_ATTR),
	BNAD_IOCMD(IOCMD_ETHPORT_GET_STATS),
	BNAD_IOCMD(IOCMD_ETHPORT_GET_CFG),

	/* DEBUG */
	BNAD_IOCMD(IOCMD_DEBUG_DRV_TRACE),
	BNAD_IOCMD(IOCMD_DEBUG_FW_TRACE),
	BNAD_IOCMD(IOCMD_DEBUG_FW_STATE),
	BNAD_IOCMD(IOCMD_DEBUG_FW_STATE_CLR),
	BNAD_IOCMD(IOCMD_DEBUG_FW_CORE),
	BNAD_IOCMD(IOCMD_DEBUG_START_DTRC),
	BNAD_IOCMD(IOCMD_DEBUG_STOP_DTRC),

	/* DIAG */
	BNAD_IOCMD(IOCMD_DIAG_REGRD),
	BNAD_IOCMD(IOCMD_DIAG_REGWR),
	BNAD_IOCMD(IOCMD_DIAG_MEMRD),
	BNAD_IOCMD(IOCMD_DIAG_TEMP),
	BNAD_IOCMD(IOCMD_DIAG_SFP),
	BNAD_IOCMD(IOCMD_DIAG_FWPING),
	BNAD_IOCMD(IOCMD_DIAG_LED),
	BNAD_IOCMD(IOCMD_DIAG_MEMTEST),
	BNAD_IOCMD(IOCMD_DIAG_LB_STAT),
	BNAD_IOCMD(IOCMD_DIAG_LL_LOOPBACK),

	/* CEE */
	BNAD_IOCMD(IOCMD_CEE_GET_ATTR),
	BNAD_IOCMD(IOCMD_CEE_GET_STATS),
	BNAD_IOCMD(IOCMD_CEE_RESET_STATS),

	/* LOG */
	BNAD_IOCMD(IOCMD_LOG_SET_LEVEL),
	BNAD_IOCMD(IOCMD_LOG_GET_LEVEL),

	/* AEN */
	BNAD_IOCMD(IOCMD_AEN_GET),
	BNAD_IOCMD(IOCMD_AEN_UPDATE),
};


extern struct file_operations bnad_fops;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 11)
extern int bnad_fioctl(struct inode *inode, struct file *file,
		unsigned int cmd, unsigned long arg);
#else /* unlocked ioctl */
extern int bnad_fioctl(struct file *file, unsigned int cmd, unsigned long arg);
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 13)
static struct class *bna_class;
#else
static struct class_simple *bna_class;
#endif

#ifdef CONFIG_COMPAT
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 9)
static inline int
bnad_ioctl32_handler(unsigned int fd, unsigned int cmd, unsigned long arg,
		     struct file *filep)
{
	struct inode *inode = filep->f_dentry->d_inode;
	int ret = 0;

	if (inode == NULL)
		return -EROFS;

	ret = bnad_fioctl(inode, filep, cmd, arg);

	return ret;
}
#endif
#endif

static inline int
bnad_os_ioctl32_register(unsigned int *bnad_ioctl_codes, int ioctl_count)
{
#ifdef CONFIG_COMPAT
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 9)
	int ret = 0, count = 0;

	/*
	 * Mark that none of the IOCTLs are registered
	 */
	reg_ioctl_cmd = -1;

	for (count = 0; count < ioctl_count; count++) {
		ret = register_ioctl32_conversion(bnad_ioctl_codes[count],
						  bnad_ioctl32_handler);
		if (ret != 0) {
			printk(KERN_WARNING
			       "IOCTL32 conversion registration failed \n");
			/*
			 * Mark the IOCTL32 registration failure point
			 */
			reg_ioctl_cmd = bnad_ioctl_codes[count];
			return -1;
		}
	}

	/*
	 * Indicate that all the IOCTL32 registration succeeded
	 * by clearing the flag.
	 */
	reg_ioctl_cmd = 0;
	return 0;
#endif
#endif
	return 0;
}

static inline void
bnad_os_ioctl32_unregister(unsigned int *bnad_ioctl_codes, int ioctl_count)
{
#ifdef CONFIG_COMPAT
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 9)
	int count = 0;

	if (reg_ioctl_cmd == -1) {
		/*
		 * Nothing to unregister for IOCTL32
		 */
		return;
	}

	for (count = 0; count < ioctl_count; count++) {
		/*
		 * If we hit the below condition,
		 * it means that there was a failure
		 * in registering the IOCTL32. So, don't unregister.
		 */
		if (bnad_ioctl_codes[count] == reg_ioctl_cmd) {
			/*
			 * When unregistration is done, mark the flag to
			 * indicate the same.
			 */
			reg_ioctl_cmd = -1;
			return;
		}
		unregister_ioctl32_conversion(bnad_ioctl_codes[count]);
	}

	/*
	 * When unregistration is done, mark the flag to
	 * indicate the same.
	 */
	reg_ioctl_cmd = -1;

	return;
#endif
#endif
}

static void
bnad_class_destroy(void)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 13)
	class_destroy(bna_class);
#else
	class_simple_destroy(bna_class);
#endif
	bna_class = NULL;
}

void
bnad_ioctl_init(void)
{
	int ret = 0;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 26)
	struct device *cdev;
#else
	struct class_device *cdev;
#endif
	if (bna_major > 0)
		return;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 13)
	bna_class = class_create(THIS_MODULE, BNAD_NAME);
#else
	bna_class = class_simple_create(THIS_MODULE, BNAD_NAME);
#endif
	if (IS_ERR(bna_class)) {
		printk(KERN_WARNING "Unable to create class for %s.\n",
		       BNAD_NAME);
		bna_class = NULL;
		return;
	}

	ret = bnad_os_ioctl32_register(bnad_ioctl_codes,
				       sizeof(bnad_ioctl_codes) /
				       sizeof(bnad_ioctl_codes[0]));
	if (ret < 0) {
		/*
		 * The IOCTL32 registration failed. Unregister
		 * all the IOCTL32 registrations.
		 */
		bnad_os_ioctl32_unregister(bnad_ioctl_codes,
					   sizeof(bnad_ioctl_codes) /
					   sizeof(bnad_ioctl_codes[0]));
	}

	bna_major = register_chrdev(0, BNAD_NAME, &bnad_fops);
	if (bna_major <= 0) {
		printk(KERN_WARNING "Unable to register chrdev for %s.\n",
		       BNAD_NAME);
		bnad_os_ioctl32_unregister(bnad_ioctl_codes,
					   sizeof(bnad_ioctl_codes) /
					   sizeof(bnad_ioctl_codes[0]));
		goto destroy_class;
	}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 26)
	cdev = device_create(bna_class, NULL, MKDEV(bna_major, 0), NULL,
			     BNAD_NAME);
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 13)
	cdev = class_simple_device_add(bna_class, MKDEV(bna_major, 0), NULL,
				       BNAD_NAME);
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15)
	cdev = class_device_create(bna_class, MKDEV(bna_major, 0), NULL,
				   BNAD_NAME);
#else
	cdev = class_device_create(bna_class, NULL, MKDEV(bna_major, 0), NULL,
				   BNAD_NAME);
#endif
	if (IS_ERR(cdev)) {
		printk(KERN_WARNING "Unable to create class device %s.\n",
		       BNAD_NAME);
		goto unregister_bna;
	}
	return;
unregister_bna:
	unregister_chrdev(bna_major, BNAD_NAME);

	bnad_os_ioctl32_unregister(bnad_ioctl_codes,
				   sizeof(bnad_ioctl_codes) /
				   sizeof(bnad_ioctl_codes[0]));

destroy_class:
	bnad_class_destroy();
}


void
bnad_ioctl_exit(void)
{
	if (bna_major <= 0)
		return;
	if (!bna_class)
		return;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 26)
	device_destroy(bna_class, MKDEV(bna_major, 0));
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 13)
	class_device_destroy(bna_class, MKDEV(bna_major, 0));
#else
	class_simple_device_remove(MKDEV(bna_major, 0));
#endif
	unregister_chrdev(bna_major, BNAD_NAME);

	bnad_os_ioctl32_unregister(bnad_ioctl_codes,
				   sizeof(bnad_ioctl_codes) /
				   sizeof(bnad_ioctl_codes[0]));

	bnad_class_destroy();
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 0, 0)
bnad_netdev_features_t
bnad_fix_features(struct net_device *netdev, bnad_netdev_features_t features)
{
	return features;
}

int
bnad_set_features(struct net_device *dev, bnad_netdev_features_t features)
{
#ifdef BNAD_LRO
	struct bnad_s *bnad = netdev_priv(dev);
	bnad_netdev_features_t changed = features ^ dev->features;

	/* Check if the the carrier is present and if feature is LRO */
	if ((!(changed & NETIF_F_LRO)) || !netif_running(bnad->netdev))
		return 0;

	bnad_conf_lock();
	/* Turn on the LRO feature - if not enabled already */
	if (!(dev->features & NETIF_F_LRO)) {
		dev->features |= NETIF_F_LRO;
		bnad_lro_disable = 0;
		/* Destroy the rx object and re-initialize to enable lro */
		bnad_reinit_rx(bnad);
	} else if ((dev->features & NETIF_F_LRO) && !bnad_lro_disable) {
	/* Turn off the LRO feature - if enabled already */
		dev->features &= ~NETIF_F_LRO;
		bnad_lro_disable = 1;
		/* Destroy the rx object and re-initialize to disable lro */
		bnad_reinit_rx(bnad);
	}
	bnad_conf_unlock();
#endif
	return 0;
}
#endif

/* Non IOCTL function */

#ifdef BNAD_NEW_VLAN_INTERFACE
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
static int
bnad_vlan_rx_add_vid(struct net_device *netdev, __be16 proto,
				 unsigned short vid)
#else
static int
bnad_vlan_rx_add_vid(struct net_device *netdev, unsigned short vid)
#endif
{
	struct bnad_s *bnad = netdev_priv(netdev);
	unsigned long flags;

	bfa_trc(bnad, vid);

	if (!bnad->rx_info[0].rx)
		return 0;

	bnad_conf_lock();

	spin_lock_irqsave(&bnad->bna_lock, flags);
	bna_rx_vlan_add(bnad->rx_info[0].rx, vid);
	bnad_vlan_bit_set(vid, bnad);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	bnad_conf_unlock();

	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
static int
bnad_vlan_rx_kill_vid(struct net_device *netdev, __be16 proto,
				  unsigned short vid)
#else
static int
bnad_vlan_rx_kill_vid(struct net_device *netdev, unsigned short vid)
#endif
{
	struct bnad_s *bnad = netdev_priv(netdev);
	unsigned long flags;

	bfa_trc(bnad, vid);

	if (!bnad->rx_info[0].rx)
		return 0;

	bnad_conf_lock();

	spin_lock_irqsave(&bnad->bna_lock, flags);
	bna_rx_vlan_del(bnad->rx_info[0].rx, vid);
	bnad_vlan_bit_clear(vid, bnad);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	bnad_conf_unlock();

	return 0;
}
#else /* !BNAD_NEW_VLAN_INTERFACE */
static void
bnad_vlan_rx_add_vid(struct net_device *netdev,
				 unsigned short vid)
{
	struct bnad_s *bnad = netdev_priv(netdev);
	unsigned long flags;

	bfa_trc(bnad, vid);

	if (!bnad->rx_info[0].rx)
		return;

	bnad_conf_lock();

	spin_lock_irqsave(&bnad->bna_lock, flags);
	bna_rx_vlan_add(bnad->rx_info[0].rx, vid);
	bnad_vlan_bit_set(vid, bnad);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	bnad_conf_unlock();
}

static void
bnad_vlan_rx_kill_vid(struct net_device *netdev,
				  unsigned short vid)
{
	struct bnad_s *bnad = netdev_priv(netdev);
	unsigned long flags;

	bfa_trc(bnad, vid);

	if (!bnad->rx_info[0].rx)
		return;

	bnad_conf_lock();

	spin_lock_irqsave(&bnad->bna_lock, flags);
	bna_rx_vlan_del(bnad->rx_info[0].rx, vid);
	bnad_vlan_bit_clear(vid, bnad);
	spin_unlock_irqrestore(&bnad->bna_lock, flags);

	bnad_conf_unlock();
}
#endif

#if defined(HAVE_NET_DEVICE_OPS) || LINUX_VERSION_CODE > KERNEL_VERSION(3, 0, 0)
static const struct net_device_ops bnad_netdev_ops = {
	.ndo_open		= bnad_open,
	.ndo_stop		= bnad_stop,
	.ndo_select_queue	= bnad_tx_select_queue,
	.ndo_start_xmit		= bnad_start_xmit,
	.ndo_get_stats		= bnad_get_netdev_stats,
	.ndo_set_rx_mode	= bnad_set_rx_mode,
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3, 1, 0)
	.ndo_set_multicast_list = bnad_set_rx_mode,
#endif
	.ndo_validate_addr      = eth_validate_addr,
	.ndo_set_mac_address    = bnad_set_mac_address,
	.ndo_change_mtu		= bnad_change_mtu,
#ifndef BNAD_NEW_VLAN_INTERFACE
	.ndo_vlan_rx_register   = bnad_vlan_rx_register,
#endif
	.ndo_vlan_rx_add_vid    = bnad_vlan_rx_add_vid,
	.ndo_vlan_rx_kill_vid   = bnad_vlan_rx_kill_vid,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller    = bnad_netpoll,
#endif
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 0, 0)
	.ndo_fix_features	= bnad_fix_features,
	.ndo_set_features	= bnad_set_features,
#endif
};
#endif

void
bnad_netdev_init(struct bnad_s *bnad, bool using_dac)
{
	struct net_device *netdev = bnad->netdev;

#ifdef NETIF_F_IPV6_CSUM
	netdev->features |= NETIF_F_IPV6_CSUM;
#endif
#ifdef NETIF_F_TSO
	netdev->features |= NETIF_F_TSO;
#endif
#ifdef NETIF_F_TSO6
	netdev->features |= NETIF_F_TSO6;
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 39)
	netdev->hw_features = NETIF_F_SG | NETIF_F_RXCSUM |
			NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM |
			NETIF_F_TSO | NETIF_F_TSO6;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	netdev->hw_features |= NETIF_F_HW_VLAN_CTAG_TX;
#else
	netdev->hw_features |= NETIF_F_HW_VLAN_TX;
#endif
#endif

#if defined(BNAD_GRO)
	if (!bnad_gro_disable) {
		netdev->features |= NETIF_F_GRO;
		printk(KERN_WARNING
				"bna: GRO enabled, using kernel stack GRO\n");
	} else
		printk(KERN_WARNING "bna: GRO is disabled\n");
#elif defined(BNAD_LRO)
	if (!bnad_lro_disable) {
		netdev->features |= NETIF_F_LRO;
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 0, 0)
		/* List of features the driver/hw supports turning on/off */
		netdev->hw_features |= NETIF_F_LRO;
#endif
		printk(KERN_WARNING
			"bna: LRO enabled, using kernel stack LRO\n");
	} else
		printk(KERN_WARNING "bna: LRO is disabled\n");
#else
	printk(KERN_WARNING "bna: GRO and LRO disabled\n");
#endif

	netdev->features |= NETIF_F_SG | NETIF_F_IP_CSUM;

	if (using_dac)
		netdev->features |= NETIF_F_HIGHDMA;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 26)
	netdev->vlan_features = netdev->features;
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0)
	netdev->features |= NETIF_F_HW_VLAN_CTAG_RX |
				NETIF_F_HW_VLAN_CTAG_FILTER;
#else
	netdev->features |= NETIF_F_HW_VLAN_TX | NETIF_F_HW_VLAN_RX |
				NETIF_F_HW_VLAN_FILTER;
#endif

	netdev->mem_start = bnad->mmio_start;
	netdev->mem_end = bnad->mmio_start + bnad->mmio_len - 1;

#if ! defined(HAVE_NET_DEVICE_OPS) && \
		LINUX_VERSION_CODE <= KERNEL_VERSION(3, 0, 0)
	netdev->open = bnad_open;
	netdev->stop = bnad_stop;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
	netdev->select_queue = bnad_tx_select_queue;
#endif
	netdev->hard_start_xmit = bnad_start_xmit;
	netdev->get_stats = bnad_get_netdev_stats;
	netdev->set_multicast_list = bnad_set_rx_mode;
	netdev->set_mac_address = bnad_set_mac_address;
	netdev->change_mtu = bnad_change_mtu;
	netdev->vlan_rx_register = bnad_vlan_rx_register;
	netdev->vlan_rx_add_vid = bnad_vlan_rx_add_vid;
	netdev->vlan_rx_kill_vid = bnad_vlan_rx_kill_vid;
#ifdef HAVE_SET_RX_MODE
	netdev->set_rx_mode = &bnad_set_rx_mode;
#endif
#ifndef BNAD_NEW_NAPI
	netdev->poll = bnad_poll;
	netdev->weight = 64;
#endif
#ifdef CONFIG_NET_POLL_CONTROLLER
	netdev->poll_controller = bnad_netpoll;
#endif
#else /* HAVE_NET_DEVICE_OPS */
	netdev->netdev_ops = &bnad_netdev_ops;
#endif
	bnad_set_ethtool_ops(netdev);
}

void
bnad_set_netdev_perm_addr(struct bnad_s *bnad)
{
	struct net_device *netdev = bnad->netdev;

	CNA_ASSERT_DEBUG(netdev->addr_len == sizeof(bnad->perm_addr));
#ifdef ETHTOOL_GPERMADDR
	memcpy(netdev->perm_addr, &bnad->perm_addr, netdev->addr_len);
#endif
	if (is_zero_ether_addr(netdev->dev_addr))
		memcpy(netdev->dev_addr, &bnad->perm_addr, netdev->addr_len);
}

#ifdef CONFIG_NET_POLL_CONTROLLER
void
bnad_netpoll(struct net_device *netdev)
{
	struct bnad_s *bnad = netdev_priv(netdev);
	struct bnad_rx_info_s *rx_info;
	struct bnad_rx_ctrl_s *rx_ctrl;
	uint32_t curr_mask;
	int i, j;

	if (!(bnad->cfg_flags & BNAD_CF_MSIX)) {
		bna_intx_disable(&bnad->bna, curr_mask);
		bnad_isr(bnad->pcidev->irq, netdev);
		bna_intx_enable(&bnad->bna, curr_mask);
	} else {
		/*
		 * Tx processing may happen in sending context, so no need
		 * to explicitly process completions here
		 */

		/* Rx processing */
		for (i = 0; i < bnad->num_rx; i++) {
			rx_info = &bnad->rx_info[i];
			if (!rx_info->rx)
				continue;
			for (j = 0; j < bnad->num_rxp_per_rx; j++) {
				rx_ctrl = &rx_info->rx_ctrl[j];
				if (rx_ctrl->ccb)
					bnad_netif_rx_schedule_poll(bnad,
							    rx_ctrl->ccb);
			}
		}
	}
}
#endif

/* VLAN APIs */
void
bnad_vlan_bit_set(unsigned short vid, struct bnad_s *bnad)
{
#ifdef BNAD_NEW_VLAN_INTERFACE
	set_bit(vid, bnad->active_vlans);
#endif
}

void
bnad_vlan_bit_clear(unsigned short vid, struct bnad_s *bnad)
{
#ifdef BNAD_NEW_VLAN_INTERFACE
	clear_bit(vid, bnad->active_vlans);
#endif
}

#ifndef BNAD_NEW_VLAN_INTERFACE
void
bnad_vlan_rx_register(struct net_device *netdev,
				  struct vlan_group *vlan_grp)
{
	struct bnad_s *bnad = netdev_priv(netdev);

	bnad_conf_lock();
	bnad->vlan_grp = vlan_grp;
	bnad_vlan_rx_config(bnad);
	bnad_conf_unlock();
}
#endif

/* Called with bnad_conf_lock() held */
void
bnad_restore_vlans(struct bnad_s *bnad, uint32_t rx_id)
{
	uint16_t vlan_id;
	unsigned long flags;

#ifndef BNAD_NEW_VLAN_INTERFACE
	if (!bnad->vlan_grp)
		return;
#endif

	CNA_ASSERT_DEBUG(VLAN_GROUP_ARRAY_LEN == BFI_ENET_VLAN_ID_MAX);

#ifndef BNAD_NEW_VLAN_INTERFACE
	for (vlan_id = 0; vlan_id < VLAN_GROUP_ARRAY_LEN; vlan_id++) {
		if (!vlan_group_get_device(bnad->vlan_grp, vlan_id))
			continue;
#else
	for_each_set_bit(vlan_id, bnad->active_vlans, VLAN_N_VID) {
#endif
		bfa_trc(bnad, vlan_id);
		spin_lock_irqsave(&bnad->bna_lock, flags);
		bna_rx_vlan_add(bnad->rx_info[rx_id].rx, vlan_id);
		spin_unlock_irqrestore(&bnad->bna_lock, flags);
	}
}

#ifdef BNAD_MACVLAN
void
bnad_set_rx_ucast_fltr(struct bnad_s *bnad)
{
	struct net_device *netdev = bnad->netdev;
	int uc_count = netdev_uc_count(netdev);
	enum bna_cb_status_e ret;
	uint8_t *mac_list;
	struct netdev_hw_addr *ha;
	int entry;

	if (netdev_uc_empty(bnad->netdev)) {
		bna_rx_ucast_listset(bnad->rx_info[0].rx, 0, NULL, NULL);
		return;
	}

	bfa_trc(bnad, uc_count);

	if (uc_count > bna_attr(&bnad->bna)->num_ucmac)
		goto mode_default;

	mac_list = kzalloc(uc_count * CNA_ETH_ALEN, GFP_ATOMIC);
	if (mac_list == NULL)
		goto mode_default;

	entry = 0;
	netdev_for_each_uc_addr(ha, netdev) {
		CNA_ASSERT_DEBUG(ha);
		memcpy(&mac_list[entry * CNA_ETH_ALEN],
				&ha->addr[0], CNA_ETH_ALEN);
		entry++;
	}

	ret = bna_rx_ucast_listset(bnad->rx_info[0].rx, entry,
			mac_list, NULL);
	kfree(mac_list);

	if (ret != BNA_CB_SUCCESS)
		goto mode_default;

	return;

	/* ucast packets not in UCAM are routed to default function */
mode_default:
	bnad->cfg_flags |= BNAD_CF_DEFAULT;
	bna_rx_ucast_listset(bnad->rx_info[0].rx, 0, NULL, NULL);
}
#endif

/* Utility used by bnad_start_xmit, for doing TSO */
int
bnad_tso_prepare(struct bnad_s *bnad, struct sk_buff *skb)
{
#ifdef NETIF_F_TSO
	int err;

#ifdef SKB_GSO_TCPV4
	/* SKB_GSO_TCPV4 and SKB_GSO_TCPV6 is defined since 2.6.18. */
	CNA_ASSERT_PERF(skb_shinfo(skb)->gso_type == SKB_GSO_TCPV4 ||
		   skb_shinfo(skb)->gso_type == SKB_GSO_TCPV6);
#endif
	if (skb_header_cloned(skb)) {
		err = pskb_expand_head(skb, 0, 0, GFP_ATOMIC);
		if (err) {
			BNAD_UPDATE_CTR(bnad, tso_err);
			return err;
		}
	}

	/*
	 * For TSO, the TCP checksum field is seeded with pseudo-header sum
	 * excluding the length field.
	 */
	if (skb->protocol == htons(ETH_P_IP)) {
		struct iphdr *iph = ip_hdr(skb);

		/* Do we really need these? */
		iph->tot_len = 0;
		iph->check = 0;

		tcp_hdr(skb)->check =
			~csum_tcpudp_magic(iph->saddr, iph->daddr, 0,
					   IPPROTO_TCP, 0);
		BNAD_UPDATE_CTR(bnad, tso4);
#ifdef NETIF_F_TSO6
	} else {
		struct ipv6hdr *ipv6h = ipv6_hdr(skb);

		CNA_ASSERT_PERF(skb->protocol == htons(ETH_P_IPV6));
		ipv6h->payload_len = 0;
		tcp_hdr(skb)->check =
			~csum_ipv6_magic(&ipv6h->saddr, &ipv6h->daddr, 0,
					 IPPROTO_TCP, 0);
		BNAD_UPDATE_CTR(bnad, tso6);
#endif
	}

	return 0;
#else
	return -EINVAL;
#endif
}

#ifdef BNAD_LRO
static int
bnad_lro_get_skb_header(struct sk_buff *skb, void **iphdr, void **tcphdr,
			u64 *hdr_flags, void *priv)
{
	struct bna_cq_entry_s *cmpl = (struct bna_cq_entry_s *)priv;
	uint32_t flags = ntohl(cmpl->flags);

	if (bnad_lro_disable ||
		((flags & (BNA_CQ_EF_IPV4 | BNA_CQ_EF_TCP)) !=
					 (BNA_CQ_EF_IPV4 | BNA_CQ_EF_TCP)))
		return -1;

	skb_reset_network_header(skb);
	skb_set_transport_header(skb, ip_hdrlen(skb));
	*iphdr = ip_hdr(skb);
	*tcphdr = tcp_hdr(skb);
	*hdr_flags = LRO_IPV4 | LRO_TCP;
	return 0;
}
#endif

int
bnad_lro_init(struct bnad_s *bnad, uint32_t rx_id)
{
#ifdef BNAD_LRO
	if (!bnad_lro_disable) {
		struct bnad_rx_info_s *rx_info = &bnad->rx_info[rx_id];
		struct bnad_rx_ctrl_s *rx_ctrl;
		char message[BNA_MESSAGE_SIZE];
		int i;

		for (i = 0; i < bnad->num_rxp_per_rx; i++) {
			rx_ctrl = &rx_info->rx_ctrl[i];
			rx_ctrl->lro.lro_arr = kcalloc(BNAD_LRO_MAX_DESC,
						sizeof(struct net_lro_desc),
						GFP_KERNEL);
			if (!rx_ctrl->lro.lro_arr) {
				sprintf(message,
				"%s : Failed to alloc LRO memory (%d X %lu)",
					bnad->netdev->name, BNAD_LRO_MAX_DESC,
				(unsigned long)(sizeof(struct net_lro_desc)));
				bfa_log(bnad->logmod,
					BFA_LOG_LINUX_RESOURCE_ALLOC_ERROR,
					message);
				bfa_trc(bnad, i);
				bfa_trc(bnad, BNAD_LRO_MAX_DESC);
				bfa_trc(bnad, sizeof(struct net_lro_desc));
				return -ENOMEM;
			}

			memset(rx_ctrl->lro.lro_arr, 0,
				BNAD_LRO_MAX_DESC *
				sizeof(struct net_lro_desc));

			rx_ctrl->lro.dev = bnad->netdev;
			rx_ctrl->lro.features |= LRO_F_NAPI;
			rx_ctrl->lro.features |= LRO_F_EXTRACT_VLAN_ID;
			rx_ctrl->lro.ip_summed = CHECKSUM_UNNECESSARY;
			rx_ctrl->lro.ip_summed_aggr = CHECKSUM_UNNECESSARY;
			rx_ctrl->lro.max_desc = BNAD_LRO_MAX_DESC;
			rx_ctrl->lro.max_aggr = BNAD_LRO_MAX_AGGR;
			rx_ctrl->lro.frag_align_pad = 0;
			rx_ctrl->lro.get_skb_header = bnad_lro_get_skb_header;

			bfa_trc(bnad, i);
		}
		bfa_trc(bnad, BNAD_LRO_MAX_DESC);
		bfa_trc(bnad, sizeof(struct net_lro_desc));
	}
#endif
	return 0;
}

void
bnad_lro_uninit(struct bnad_s *bnad, uint32_t rx_id)
{
#ifdef BNAD_LRO
	if (!bnad_lro_disable) {
		struct bnad_rx_info_s *rx_info = &bnad->rx_info[rx_id];
		int i;

		for (i = 0; i < bnad->num_rxp_per_rx; i++) {
			kfree(rx_info->rx_ctrl[i].lro.lro_arr);
			rx_info->rx_ctrl[i].lro.lro_arr = NULL;
			bfa_trc(bnad, i);
		}
	}
#endif
}

/*
 * Fill up bnad->stats.drv_stats LRO fields from stack
 */
void
bnad_lro_stats_fill(struct bnad_s *bnad)
{
	unsigned long lro_flushed = 0;
	unsigned long lro_aggregated = 0;
#ifdef BNAD_LRO
	if (!bnad_lro_disable) {
		int i, j;
		struct bnad_rx_ctrl_s *rx_ctrl = NULL;

		for (i = 0; i < bnad->num_rx; i++) {
			if (!bnad->rx_info[i].rx)
				continue;
			for (j = 0; j < bnad->num_rxp_per_rx; j++) {
				rx_ctrl = &bnad->rx_info[i].rx_ctrl[j];
				if (rx_ctrl->ccb) {
				/* Aggregate lro_flushed for all RxP's */
					lro_flushed +=
						rx_ctrl->lro.stats.flushed;
					lro_aggregated +=
						rx_ctrl->lro.stats.aggregated;
				}
			}
		}
	}
#endif
	bnad->stats.drv_stats.lro_flushed = lro_flushed;
	bnad->stats.drv_stats.lro_aggregated = lro_aggregated;
}

/*
 * Clear LRO stats from stack
 */
void
bnad_lro_stats_clear(struct bnad_s *bnad)
{
#ifdef BNAD_LRO
	if (!bnad_lro_disable) {
		int i, j;
		struct bnad_rx_ctrl_s *rx_ctrl = NULL;

		for (i = 0; i < bnad->num_rx; i++) {
			if (!bnad->rx_info[i].rx)
				continue;
			for (j = 0; j < bnad->num_rxp_per_rx; j++) {
				rx_ctrl = &bnad->rx_info[i].rx_ctrl[j];
				if (rx_ctrl->ccb) {
					rx_ctrl->lro.stats.flushed = 0;
					rx_ctrl->lro.stats.aggregated = 0;
				}
			}
		}
	}
#endif
}

#ifdef BNAD_NEW_NAPI
int
bnad_napi_poll_rx(struct napi_struct *napi, int budget)
{
	struct bnad_rx_ctrl_s *rx_ctrl =
		container_of(napi, struct bnad_rx_ctrl_s, napi);
	struct bnad_s *bnad = rx_ctrl->bnad;
	int rcvd = 0;

	rx_ctrl->rx_poll_ctr++;

	CNA_ASSERT_PERF(rx_ctrl->ccb);

	if (!netif_carrier_ok(bnad->netdev))
		goto poll_exit;

	rcvd = bnad_cq_process(bnad, rx_ctrl->ccb, budget);
	if (rcvd >= budget)
		return rcvd;
poll_exit:
	bnad_rx_complete(bnad->netdev, napi);

	rx_ctrl->rx_complete++;

	/*
	 * Check again here to avoid race between,
	 * here & bnad_cb_ccb_destroy()
	 */
	if (rx_ctrl->ccb) {
		if (bnad->cfg_flags & BNAD_CF_DIM_ENABLED)
			bnad_rx_intr_rate_adjust(rx_ctrl);
		bnad_enable_rx_irq_unsafe(rx_ctrl->ccb);
	}

	return rcvd;
}

#else	/* !BNAD_NEW_NAPI */

int
bnad_poll(struct net_device *netdev, int *budget)
{
	struct bnad_s *bnad = netdev_priv(netdev);
	struct bnad_rx_ctrl_s *rx_ctrl =
			&bnad->rx_info[0].rx_ctrl[0];
	int quota = min(netdev->quota, *budget);
	uint32_t rcvd = 0;

	rx_ctrl->rx_poll_ctr++;

	if (!netif_carrier_ok(bnad->netdev))
		goto poll_exit;

	rcvd = bnad_cq_process(bnad, rx_ctrl->ccb, quota);

	*budget -= rcvd;
	netdev->quota -= rcvd;

	if (rcvd >= quota)
		return 1;
poll_exit:
	netif_rx_complete(netdev);
	rx_ctrl->rx_complete++;

	/*
	 * Check again here to avoid race between,
	 * here & bnad_cb_ccb_destroy()
	 */
	if (rx_ctrl->ccb) {
		if (bnad->cfg_flags & BNAD_CF_DIM_ENABLED)
			bnad_rx_intr_rate_adjust(rx_ctrl);
		bnad_enable_rx_irq_unsafe(rx_ctrl->ccb);
	}

	return 0;
}
#endif

void
bnad_netif_rx_schedule_poll(struct bnad_s *bnad, struct bna_ccb_s *ccb)
{
	struct bnad_rx_ctrl_s *rx_ctrl = (struct bnad_rx_ctrl_s *)(ccb->ctrl);
#ifdef BNAD_NEW_NAPI
	struct napi_struct *napi = &rx_ctrl->napi;
#else
	void *napi;

	napi = NULL;
#endif
	if (likely(bnad_rx_schedule_prep(bnad->netdev, napi))) {
		__bnad_rx_schedule(bnad->netdev, napi);
		rx_ctrl->rx_schedule++;
	}
}

uint16_t
bnad_tx_select_queue(struct net_device *netdev, struct sk_buff *skb)
{
	struct bnad_s	*bnad =  netdev_priv(netdev);
	struct bna_s 	*bna = &bnad->bna;
	uint8_t 	prio = 0;

	if (bnad->num_txq_per_tx < BFI_TX_MAX_PRIO)
		prio = 0;
	else if (bna_is_iscsi_over_cee(&bnad->bna) && bnad_is_iscsi(skb))
		prio = bna_iscsi_prio(bna);
	else if (vlan_tx_tag_present(skb)) {
		uint8_t pkt_vlan_prio = 0;
		uint16_t pkt_vlan_tag = 0;
		pkt_vlan_tag = (uint16_t)vlan_tx_tag_get(skb);
		pkt_vlan_prio = (pkt_vlan_tag & BNAD_VLAN_PRIO_MASK)
					>> BNAD_VLAN_PRIO_SHIFT;
		prio = bna_prio_allowed(bna, pkt_vlan_prio) ?
			pkt_vlan_prio : bna_default_prio(bna);
	} else
		prio = bna_default_prio(bna);

	return (uint16_t)prio;
}

/*
 * Initialize Q numbers depending on Rx Paths
 * Called with bnad->bna_lock held, because of cfg_flags
 * access.
 */
void
bnad_q_num_init(struct bnad_s *bnad)
{
	int rxps = BNAD_GET_NUM_RXP_PER_RX();

	BNA_TO_POWER_OF_2(rxps);

	if (!(bnad->cfg_flags & BNAD_CF_MSIX))
		rxps = 1;	/* INTx */

	bnad->num_rx = BNAD_MAX_RX;
	/*
	 * BNAD_NUM_TXQ_PER_TX comes from
	 * compat.h and is 8 or 1
	 * BNAD_NUM_TX is always 1
	 */
	bnad->num_tx = BNAD_MAX_TX;
	bnad->num_rxp_per_rx = rxps;
	bnad->num_txq_per_tx = BNAD_MAX_TXQ_PER_TX;
}

/*
 * Adjusts the Q numbers, given a number of max possible queues.
 * Give preference to RSS as opposed to Tx priority Queues,
 * in such a case, just use 1 Tx Q
 * Called with bnad->bna_lock held b'cos of cfg_flags access
 */
void
bnad_q_num_adjust(struct bnad_s *bnad, uint32_t max_txq, uint32_t max_rxq)
{
	if (!(bnad->cfg_flags & BNAD_CF_MSIX)) {
		bnad->num_tx = bnad->num_txq_per_tx = 1;
		bnad->num_rx = bnad->num_rxp_per_rx = 1;
		bfa_trc(bnad, bnad->num_rxp_per_rx);
		return;
	}

	if (max_txq < BNAD_NUM_TXQ) {
		bnad->num_txq_per_tx = 1;
		bnad->num_tx = 1;
	}

	bnad->num_rx = 1;
	bnad->num_rxp_per_rx = min((uint32_t)BNAD_GET_NUM_RXP_PER_RX(),
					max_rxq);
	BNA_TO_POWER_OF_2(bnad->num_rxp_per_rx);
	bfa_trc(bnad, bnad->num_rxp_per_rx);
}


/*
 * Initialize allocation parameters of RxQ for GRO
 */
int
bnad_rxq_gro_init(struct bnad_s *bnad, struct bna_rcb_s *rcb)
{
#ifdef BNAD_GRO
	struct bnad_rx_unmap_q_s *unmap_q = rcb->unmap_q;
	int order;

	unmap_q->reuse_pi = -1;

	if (bnad_gro_disable)
		return 1;

	order = get_order(rcb->rxq->buffer_size);
	unmap_q->type = BNAD_RXBUF_PAGE;

	if (bna_is_small_rxq(rcb->id)) {
		CNA_ASSERT_DEBUG(order == 0);
		unmap_q->alloc_order = 0;
		unmap_q->map_size = rcb->rxq->buffer_size;
	} else {
		if (rcb->rxq->multi_buffer) {
			CNA_ASSERT_DEBUG(order == 0);
			unmap_q->alloc_order = 0;
			unmap_q->map_size = rcb->rxq->buffer_size;
			unmap_q->type = BNAD_RXBUF_MULTI_BUFF;
		} else {
			unmap_q->alloc_order = order;
			unmap_q->map_size =
				(rcb->rxq->buffer_size > 2048) ?
				CNA_PAGE_SIZE << order : 2048;
		}
	}
	CNA_ASSERT_DEBUG((((CNA_PAGE_SIZE) << order) %
				unmap_q->map_size) == 0);

	return 0;
#else
	return 1;
#endif
}
