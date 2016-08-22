/*
 * Copyright (c) 2005-2014 Brocade Communications Systems, Inc.
 * Copyright (c) 2014 QLogic Corporation.
 * All rights reserved
 * www.qlogic.com
 *
 * See LICENSE.bna for copyright and licensing details.
 */

/**
 * @file bfa_flash_raw.c flash raw operations
 *
 */

#include <bfa_os_inc.h>
#include <bfi/bfi.h>
#include <defs/bfa_defs.h>
#include <cna/bfa_flash_raw.h>
#include <bfi/bfi_reg.h>
#include <cs/bfa_cs.h>

/*
 * register definitions
 */
#define FLI_CMD_REG			0x0001d000
#define FLI_WRDATA_REG			0x0001d00c
#define FLI_RDDATA_REG			0x0001d010
#define FLI_ADDR_REG			0x0001d004
#define FLI_DEV_STATUS_REG		0x0001d014

#define BFA_FLASH_FIFO_SIZE		128	/* fifo size */
#define BFA_FLASH_CHECK_MAX		10000	/* max # of status check */
#define BFA_FLASH_BLOCKING_OP_MAX	1000000	/* max # of blocking op check */
#define BFA_FLASH_WIP_MASK		0x01	/* write in progress bit mask */

#define NFC_STATE_RUNNING		0x20000001
#define NFC_STATE_PAUSED		0x00004560
#define NFC_VER_VALID			0x147

typedef enum {
	BFA_FLASH_FAST_READ	= 0x0b,	/* fast read */
	BFA_FLASH_WRITE_ENABLE	= 0x06,	/* write enable */
	BFA_FLASH_SECTOR_ERASE	= 0xd8,	/* sector erase */
	BFA_FLASH_WRITE		= 0x02,	/* write */
	BFA_FLASH_READ_STATUS	= 0x05,	/* read status */
} bfa_flash_cmd_t;

/**
 * @brief hardware error definition
 */
typedef enum {
	BFA_FLASH_NOT_PRESENT	= -1,	/*!< flash not present */
	BFA_FLASH_UNINIT		= -2,	/*!< flash not initialized */
	BFA_FLASH_BAD		= -3,	/*!< flash bad */
	BFA_FLASH_BUSY		= -4,	/*!< flash busy */
	BFA_FLASH_ERR_CMD_ACT	= -5,	/*!< command active never cleared */
	BFA_FLASH_ERR_FIFO_CNT	= -6, 	/*!< fifo count never cleared */
	BFA_FLASH_ERR_WIP	= -7,	/*!< write-in-progress never cleared */
	BFA_FLASH_ERR_TIMEOUT	= -8,	/*!< fli timeout */
	BFA_FLASH_ERR_LEN	= -9,	/*!< invalid length */
} bfa_flash_err_t;

/**
 * @brief flash command register data structure
 */
typedef union {
	struct {
#ifdef __BIGENDIAN
		uint32_t	act:1;
		uint32_t	rsv:1;
		uint32_t	write_cnt:9;
		uint32_t	read_cnt:9;
		uint32_t	addr_cnt:4;
		uint32_t	cmd:8;
#else
		uint32_t	cmd:8;
		uint32_t	addr_cnt:4;
		uint32_t	read_cnt:9;
		uint32_t	write_cnt:9;
		uint32_t	rsv:1;
		uint32_t	act:1;
#endif
	} r;
	uint32_t	i;
} bfa_flash_cmd_reg_t;

/**
 * @brief flash device status register data structure
 */
typedef union {
	struct {
#ifdef __BIGENDIAN
		uint32_t	rsv:21;
		uint32_t	fifo_cnt:6;
		uint32_t	busy:1;
		uint32_t	init_status:1;
		uint32_t	present:1;
		uint32_t	bad:1;
		uint32_t	good:1;
#else
		uint32_t	good:1;
		uint32_t	bad:1;
		uint32_t	present:1;
		uint32_t	init_status:1;
		uint32_t	busy:1;
		uint32_t	fifo_cnt:6;
		uint32_t	rsv:21;
#endif
	} r;
	uint32_t	i;
} bfa_flash_dev_status_reg_t;

/**
 * @brief flash address register data structure
 */
typedef union {
	struct {
#ifdef __BIGENDIAN
		uint32_t	addr:24;
		uint32_t	dummy:8;
#else
		uint32_t	dummy:8;
		uint32_t	addr:24;
#endif
	} r;
	uint32_t	i;
} bfa_flash_addr_reg_t;

uint32_t nfc_delays[] = {3100, 80, 4500, 18000, };	/* in ms */

/**
 * dg flash_raw_private Flash raw private functions
 * @{
 */
static void
bfa_flash_set_cmd(bfa_os_addr_t pci_bar, uint8_t wr_cnt,
		  uint8_t rd_cnt, uint8_t ad_cnt, uint8_t op)
{
	bfa_flash_cmd_reg_t cmd;

	cmd.i = 0;
	cmd.r.act = 1;
	cmd.r.write_cnt = wr_cnt;
	cmd.r.read_cnt = rd_cnt;
	cmd.r.addr_cnt = ad_cnt;
	cmd.r.cmd = op;
	bfa_os_reg_write((pci_bar + FLI_CMD_REG), cmd.i);
}

static void
bfa_flash_set_addr(bfa_os_addr_t pci_bar, uint32_t address)
{
	bfa_flash_addr_reg_t addr;

	addr.r.addr = address & 0x00ffffff;
	addr.r.dummy = 0;
	bfa_os_reg_write((pci_bar + FLI_ADDR_REG), addr.i);
}

static int
bfa_flash_cmd_act_check(bfa_os_addr_t pci_bar)
{
	volatile bfa_flash_cmd_reg_t cmd;

	cmd.i = bfa_os_reg_read(pci_bar + FLI_CMD_REG);

	if (cmd.r.act)
		return BFA_FLASH_ERR_CMD_ACT;

	return 0;
}

/**
 * @brief
 * Flush FLI data fifo.
 *
 * @param[in] pci_bar - pci bar address
 * @param[in] dev_status - device status
 *
 * Return 0 on success, negative error number on error.
 */
static int32_t
bfa_flash_fifo_flush(bfa_os_addr_t pci_bar)
{
	uint32_t i;
	uint32_t t;
	bfa_flash_dev_status_reg_t dev_status;

	dev_status.i = bfa_os_reg_read(pci_bar + FLI_DEV_STATUS_REG);

	if (!dev_status.r.fifo_cnt)
		return 0;

	/* fifo counter in terms of words */
	for (i = 0; i < dev_status.r.fifo_cnt; i++)
		t = bfa_os_reg_read(pci_bar + FLI_RDDATA_REG);

	/*
	 * Check the device status. It may take some time.
	 */
	for (i = 0; i < BFA_FLASH_CHECK_MAX; i++) {
		dev_status.i = bfa_os_reg_read(pci_bar + FLI_DEV_STATUS_REG);
		if (!dev_status.r.fifo_cnt)
			break;
	}

	if (dev_status.r.fifo_cnt)
		return BFA_FLASH_ERR_FIFO_CNT;

	return 0;
}

/**
 * @brief
 * Enable flash write.
 *
 * @param[in] pci_bar - pci bar address
 *
 * Return 0 on success, negative error number on error.
 */
static int32_t
bfa_flash_write_enable(bfa_os_addr_t pci_bar)
{
	int32_t	status;
	int i;

	bfa_flash_set_cmd(pci_bar, 0, 0, 0, BFA_FLASH_WRITE_ENABLE);

	for (i = 0; i < BFA_FLASH_CHECK_MAX; i++) {
		status = bfa_flash_cmd_act_check(pci_bar);
		if (!status)
			return 0;
	}

	return status;
}

/**
 * @brief
 * Read flash status.
 *
 * @param[in] pci_bar - pci bar address
 *
 * Return 0 on success, negative error number on error.
*/
static int32_t
bfa_flash_status_read(bfa_os_addr_t pci_bar)
{
	bfa_flash_dev_status_reg_t	dev_status;
	int32_t				status;
	uint32_t			ret_status;
	int				i;

	status = bfa_flash_fifo_flush(pci_bar);
	if (status < 0)
		return status;

	bfa_flash_set_cmd(pci_bar, 0, 4, 0, BFA_FLASH_READ_STATUS);

	for (i = 0; i < BFA_FLASH_CHECK_MAX; i++) {
		status = bfa_flash_cmd_act_check(pci_bar);
		if (!status)
			break;
	}

	if (status)
		return status;

	dev_status.i = bfa_os_reg_read(pci_bar + FLI_DEV_STATUS_REG);
	if (!dev_status.r.fifo_cnt)
		return BFA_FLASH_BUSY;

	ret_status = bfa_os_reg_read(pci_bar + FLI_RDDATA_REG);
	ret_status >>= 24;

	status = bfa_flash_fifo_flush(pci_bar);
	if (status < 0)
		return status;

	return ret_status;
}

/**
 * @brief
 * Start flash read operation.
 *
 * @param[in] pci_bar - pci bar address
 * @param[in] offset - flash address offset
 * @param[in] len - read data length
 * @param[in] buf - read data buffer
 *
 * Return 0 on success, negative error number on error.
 */
static int32_t
bfa_flash_read_start(bfa_os_addr_t pci_bar, uint32_t offset, uint32_t len,
			 char __FAR *buf)
{
	int32_t status;

	/*
	 * len must be mutiple of 4 and not exceeding fifo size
	 */
	if (len == 0 || len > BFA_FLASH_FIFO_SIZE || (len & 0x03) != 0)
		return BFA_FLASH_ERR_LEN;

	/*
	 * check status
	 */
	status = bfa_flash_status_read(pci_bar);
	if (status == BFA_FLASH_BUSY)
		status = bfa_flash_status_read(pci_bar);

	if (status < 0)
		return status;

	/*
	 * check if write-in-progress bit is cleared
	 */
	if (status & BFA_FLASH_WIP_MASK)
		return BFA_FLASH_ERR_WIP;

	bfa_flash_set_addr(pci_bar, offset);

	bfa_flash_set_cmd(pci_bar, 0, (uint8_t)len, 4, BFA_FLASH_FAST_READ);

	return 0;
}

/**
 * @brief
 * Check flash read operation.
 *
 * @param[in] pci_bar - pci bar address
 *
 * Return flash device status, 1 if busy, 0 if not.
 */
static int32_t
bfa_flash_read_check(bfa_os_addr_t pci_bar)
{
	if (bfa_flash_cmd_act_check(pci_bar))
		return 1;

	return 0;
}
/**
 * @brief
 * End flash read operation.
 *
 * @param[in] pci_bar - pci bar address
 * @param[in] len - read data length
 * @param[in] buf - read data buffer
 *
 */
static void
bfa_flash_read_end(bfa_os_addr_t pci_bar, uint32_t len, char __FAR *buf)
{

	uint32_t i;

	/*
	 * read data fifo up to 32 words
	 */
	for (i = 0; i < len; i += 4) {
		uint32_t w = bfa_os_reg_read(pci_bar + FLI_RDDATA_REG);
		*((uint32_t *) (buf + i)) = bfa_os_swap32(w);
	}

	bfa_flash_fifo_flush(pci_bar);
}

/**
 * @brief
 * Start flash write operation.
 *
 * @param[in] pci_bar - pci bar address
 * @param[in] offset - flash address offset
 * @param[in] len - write data length
 * @param[in] buf - write data buffer
 *
 * Return 0 on success, negative error number on error.
 */
static int32_t
bfa_flash_write_start(bfa_os_addr_t pci_bar, uint32_t offset, uint32_t len,
			  char __FAR *buf)
{
	int32_t status;
	uint32_t i;

	/*
	 * len must be mutiple of 4 and not exceeding fifo size
	 */
	if (len == 0 || len > BFA_FLASH_FIFO_SIZE || (len & 0x03) != 0)
		return (BFA_FLASH_ERR_LEN);

	/*
	 * check status
	 */
	status = bfa_flash_status_read(pci_bar);
	if (status < 0)
		return (status);

	/*
	 * check if write-in-progress bit is cleared
	 */
	if (status & BFA_FLASH_WIP_MASK)
		return (BFA_FLASH_ERR_WIP);

	/*
	 * write enable
	 */
	status = bfa_flash_write_enable(pci_bar);
	if (status < 0)
		return (status);

	bfa_flash_set_addr(pci_bar, offset);

	/*
	 * set up data fifo up to 32 words
	 */
	for (i = 0; i < len; i += 4) {
		uint32_t data = *((uint32_t *) (buf + i));
		data = bfa_os_ntohl(data);
		bfa_os_reg_write((pci_bar + FLI_WRDATA_REG), data);
	}

	bfa_flash_set_cmd(pci_bar, (uint8_t)len, 0, 3, BFA_FLASH_WRITE);

	return 0;
}

/**
 * @brief
 * Check flash write operation.
 *
 * @param[in] pci_bar - pci bar address
 *
 * Return flash device status, 1 if busy, 0 if not.
 */
static int32_t
bfa_flash_write_check(bfa_os_addr_t pci_bar)
{
	int32_t status;
	bfa_flash_dev_status_reg_t dev_status;

	dev_status.i = bfa_os_reg_read(pci_bar + FLI_DEV_STATUS_REG);

	if (dev_status.r.busy)
		return 1;

	status = bfa_flash_status_read(pci_bar);
	if (status < 0 || (status & BFA_FLASH_WIP_MASK))
		return 1;

	return 0;
}

/**
 * @brief
 * Start flash sector erase operation.
 *
 * @param[in] pci_bar - pci bar address
 * @param[in] offset - flash address offset
 *
 * Return 0 on success, negative error number on error.
 */
static int32_t
bfa_flash_sector_erase_start(bfa_os_addr_t pci_bar, uint32_t offset)
{
	int32_t status;

	/*
	 * check status
	 */
	status = bfa_flash_status_read(pci_bar);
	if (status < 0)
		return (status);

	/*
	 * check if write-in-progress bit is cleared
	 */
	if (status & BFA_FLASH_WIP_MASK)
		return (BFA_FLASH_ERR_WIP);

	/*
	 * write enable
	 */
	status = bfa_flash_write_enable(pci_bar);
	if (status < 0)
		return (status);

	/*
	 * ASIC bug workaround on sector erase setup
	 */
	bfa_flash_set_addr(pci_bar, 0);

	bfa_os_reg_write((pci_bar + FLI_WRDATA_REG), offset >> 16);

	bfa_flash_set_cmd(pci_bar, 3, 0, 0, BFA_FLASH_SECTOR_ERASE);

	return 0;
}

/**
 * @brief
 * Check flash erase operation.
 *
 * @param[in] pci_bar - pci bar address
 *
 * Return flash erase status, 1 if busy, 0 if not.
 */
static int32_t
bfa_flash_erase_check(bfa_os_addr_t pci_bar)
{
	int32_t status;
	bfa_flash_dev_status_reg_t dev_status;

	dev_status.i = bfa_os_reg_read(pci_bar + FLI_DEV_STATUS_REG);

	if (dev_status.r.busy)
		return 1;

	status = bfa_flash_status_read(pci_bar);
	if (status < 0 || (status & BFA_FLASH_WIP_MASK)) {
		return 1;
	}

	return 0;
}

static bfa_boolean_t
bfa_flash_nfc_pause_required(bfa_os_addr_t pci_bar, uint16_t dev_id)
{
	if (!bfa_asic_id_ct2(dev_id))
		return BFA_FALSE;

	if (bfa_os_reg_read(pci_bar + CT2_RSC_GPR15_REG) < NFC_VER_VALID)
		return BFA_FALSE;

	if (bfa_os_reg_read(pci_bar + CT2_NFC_STS_REG) == 0xBADFBADF)
		return BFA_FALSE;

	return BFA_TRUE;
}

static uint32_t
bfa_flash_nfc_calc_delay(uint32_t pause_time)
{
	int i;

	for (i = 0; i < sizeof(nfc_delays); i++) {
		if (pause_time == nfc_delays[i])
			return i;
	}
	bfa_assert(0);
	return 0;
}

static uint32_t
bfa_flash_nfc_pause_time_calc(bfa_flash_cmd_t cmd, uint32_t len)
{
	uint32_t pause_time = 0;

	switch (cmd) {
	case BFA_FLASH_WRITE:
		if (len < 65536)
			pause_time = 80;
		else
			pause_time = 3100;
		break;

	case BFA_FLASH_SECTOR_ERASE:
		pause_time = 4500;
		break;

	default:
		bfa_assert(0);
	}

	return pause_time;
}

static void
bfa_flash_wait_till_nfc_running(bfa_os_addr_t pci_bar)
{
	volatile uint32_t nfc_sts;

	while ((nfc_sts = bfa_os_reg_read(pci_bar + CT2_NFC_STS_REG))
		!= NFC_STATE_RUNNING);
}

static void
bfa_flash_nfc_pause(bfa_os_addr_t pci_bar, uint32_t pause_time)
{
	uint32_t delay_val;
	volatile uint32_t nfc_sts;

	delay_val = bfa_flash_nfc_calc_delay(pause_time);
	bfa_assert(delay_val < sizeof(nfc_delays));

	bfa_flash_wait_till_nfc_running(pci_bar);

	/* Clear NFC delay bits */
	bfa_os_reg_write(pci_bar + CT2_CSI_FW_CTL_CLR_REG,
			 __FLASH_UPDATE_ENABLE_DELAY_INSTR_VALUES_MK);

	bfa_os_reg_write(pci_bar + CT2_CSI_FW_CTL_SET_REG,
			 __FLASH_UPDATE_ENABLE_DELAY_INSTR |
			 __FLASH_UPDATE_PENDING | __FLASH_UPDATE_PAUSE_NFC |
			 __FLASH_UPDATE_ENABLE_DELAY_INSTR_VALUES(delay_val));

	nfc_sts = bfa_os_reg_read(pci_bar + CT2_NFC_STS_REG);

	/* Wait till NFC enters update loop */
	while (nfc_sts != NFC_STATE_PAUSED)
		nfc_sts = bfa_os_reg_read(pci_bar + CT2_NFC_STS_REG);

	bfa_os_reg_write(pci_bar + CT2_CSI_FW_CTL_CLR_REG,
			 __FLASH_UPDATE_PAUSE_NFC);
	bfa_os_udelay(1000);
}

/**
 * @}
 *
 * dg flash_raw_public Flash raw operation public functions
 * @{
 */

/**
 * @brief
 * Perform flash raw read.
 *
 * @param[in] pci_bar - pci bar address
 * @param[in] offset - flash partition address offset
 * @param[in] buf - read data buffer
 * @param[in] len - read data length
 *
 * Return status.
 */


#define FLASH_BLOCKING_OP_MAX   500
#define FLASH_SEM_LOCK_REG	0x18820

static int
bfa_raw_sem_get(bfa_os_addr_t bar)
{
	int	locked;

	locked = bfa_os_reg_read((bar + FLASH_SEM_LOCK_REG));
	return (!locked);

}

bfa_status_t
bfa_flash_sem_get(bfa_os_addr_t bar)
{
	int32_t n = FLASH_BLOCKING_OP_MAX;

	while (!bfa_raw_sem_get(bar)) {
		if (--n <= 0) {
			return (BFA_STATUS_BADFLASH);
		}
		bfa_os_udelay(10000);
	}
	return (BFA_STATUS_OK);
}

void
bfa_flash_sem_put(bfa_os_addr_t bar)
{
	bfa_os_reg_write((bar + FLASH_SEM_LOCK_REG), 0);
}

bfa_status_t
bfa_flash_raw_read(bfa_os_addr_t pci_bar, uint32_t offset, char __FAR *buf,
		       uint32_t len)
{
	int32_t n, status;
	uint32_t off, l, s, residue, fifo_sz;

	residue = len;
	off = 0;
	fifo_sz = BFA_FLASH_FIFO_SIZE;
	status = bfa_flash_sem_get(pci_bar);
	if (status != BFA_STATUS_OK)
		return status;

	while (residue) {
		s = offset + off;
		n = s / fifo_sz;
		l = (n + 1) * fifo_sz - s;
		if (l > residue)
			l = residue;

		status = bfa_flash_read_start(pci_bar, offset + off, l,
								&buf[off]);
		if (status < 0) {
			bfa_flash_sem_put(pci_bar);
			return (BFA_STATUS_FAILED);
		}

		n = BFA_FLASH_BLOCKING_OP_MAX;
		while (bfa_flash_read_check(pci_bar)) {
			if (--n <= 0) {
				bfa_flash_sem_put(pci_bar);
				return (BFA_STATUS_FAILED);
			}
		}

		bfa_flash_read_end(pci_bar, l, &buf[off]);

		residue -= l;
		off += l;
	}
	bfa_flash_sem_put(pci_bar);

	return (BFA_STATUS_OK);
}

/**
 * @brief
 * Perform flash raw write with sector erase first.
 *
 * @param[in] pci_bar - pci bar address
 * @param[in] offset - flash partition address offset
 * @param[in] buf - write data buffer
 * @param[in] len - write data length
 *
 * Return status.
 */
bfa_status_t
bfa_flash_raw_write(bfa_os_addr_t pci_bar, uint32_t offset, char __FAR *buf,
		    uint32_t len, uint16_t dev_id)
{
	bfa_status_t status;

	if ((status = bfa_flash_raw_erase(pci_bar, offset, dev_id))
	     != BFA_STATUS_OK)
		return (status);

	return (bfa_flash_raw_write_no_erase(pci_bar, offset, buf, len,
		dev_id));
}

/**
 * @brief
 * Perform flash raw write without sector erase.
 *
 * @param[in] pci_bar - pci bar address
 * @param[in] offset - flash partition address offset
 * @param[in] buf - write data buffer
 * @param[in] len - write data length
 *
 * Return status.
 */
bfa_status_t
bfa_flash_raw_write_no_erase(bfa_os_addr_t pci_bar, uint32_t offset,
			char __FAR *buf, uint32_t len, uint16_t dev_id)
{
	int32_t n, status;
	uint32_t off, l, s, residue, fifo_sz, pause_time, csi_fw_ctl;

	status = bfa_flash_sem_get(pci_bar);
	if (status != BFA_STATUS_OK)
		return status;

	if (bfa_flash_nfc_pause_required(pci_bar, dev_id)) {
		pause_time = bfa_flash_nfc_pause_time_calc(BFA_FLASH_WRITE,
							   len);
		bfa_flash_nfc_pause(pci_bar, pause_time);
	}

	residue = len;
	off = 0;
	fifo_sz = BFA_FLASH_FIFO_SIZE;
	while (residue) {
		s = offset + off;
		n = s / fifo_sz;
		l = (n + 1) * fifo_sz - s;
		if (l > residue)
			l = residue;

		status = bfa_flash_write_start(pci_bar, offset + off, l,
			&buf[off]);

		if (status < 0) {
			bfa_flash_sem_put(pci_bar);
			return (BFA_STATUS_FAILED);
		}

		n = BFA_FLASH_BLOCKING_OP_MAX;
		while (bfa_flash_write_check(pci_bar)) {
			if (--n <= 0) {
				bfa_flash_sem_put(pci_bar);
				return (BFA_STATUS_FAILED);
			}
		}

		residue -= l;
		off += l;
	}

	if (bfa_flash_nfc_pause_required(pci_bar, dev_id)) {
		csi_fw_ctl = bfa_os_reg_read(pci_bar + CT2_CSI_FW_CTL_REG);
		bfa_assert(csi_fw_ctl & __FLASH_UPDATE_ENABLE_DELAY_INSTR);
		bfa_flash_wait_till_nfc_running(pci_bar);
	}

	bfa_flash_sem_put(pci_bar);
	return (BFA_STATUS_OK);
}

/**
 * @brief
 * Perform flash raw sector erase.
 *
 * @param[in] pci_bar - pci bar address
 * @param[in] offset - flash partition address offset
 *
 * Return status.
 */
bfa_status_t
bfa_flash_raw_erase(bfa_os_addr_t pci_bar, uint32_t offset, uint16_t dev_id)
{
	int32_t n, status;
	uint32_t pause_time;
	uint32_t csi_fw_ctl;

	status = bfa_flash_sem_get(pci_bar);

	if (status != BFA_STATUS_OK)
		return status;

	if (bfa_flash_nfc_pause_required(pci_bar, dev_id)) {
		pause_time = bfa_flash_nfc_pause_time_calc(
				BFA_FLASH_SECTOR_ERASE, 0);
		bfa_flash_nfc_pause(pci_bar, pause_time);
	}

	if (bfa_flash_sector_erase_start(pci_bar, offset) < 0) {
		bfa_flash_sem_put(pci_bar);
		return (BFA_STATUS_FAILED);
	}

	n = BFA_FLASH_BLOCKING_OP_MAX;
	while (bfa_flash_erase_check(pci_bar)) {
		if (--n <= 0) {
			bfa_flash_sem_put(pci_bar);
			return (BFA_STATUS_FAILED);
		}
	}

	if (bfa_flash_nfc_pause_required(pci_bar, dev_id)) {
		csi_fw_ctl = bfa_os_reg_read(pci_bar + CT2_CSI_FW_CTL_REG);
		bfa_assert(csi_fw_ctl & __FLASH_UPDATE_ENABLE_DELAY_INSTR);
	}

	bfa_flash_sem_put(pci_bar);

	return (BFA_STATUS_OK);
}

/**
 * @}
 */
