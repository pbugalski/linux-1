/*
 * Marvell NAND flash controller driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Copyright (C) 2017 Marvell
 * Author: Miquel RAYNAL <miquel.raynal@free-electrons.com>
 */

#include <linux/module.h>
#include <linux/clk.h>
#include <linux/mtd/rawnand.h>
#include <linux/of_platform.h>
#include <linux/iopoll.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <asm/unaligned.h>

/* Data FIFO granularity, FIFO reads/writes must be a multiple of this length */
#define FIFO_DEPTH		8
#define FIFO_REP(x)		(x / sizeof(u32))
#define FIFO_DEPTH_32		FIFO_REP(FIFO_DEPTH)
/* NFCv2 does not support chunks larger than 2kiB */
#define MAX_CHUNK		2048
/* Polling is done at a pace of POLL_PERIOD us until POLL_TIMEOUT is reached */
#define POLL_PERIOD		0
#define POLL_TIMEOUT		1000
/* Interrupt maximum wait period in ms */
#define IRQ_TIMEOUT		1000
/* Latency in clock cycles between SoC pins and NFC logic */
#define MIN_RD_DEL_CNT		3

/* NAND controller data flash control register */
#define NDCR			0x00
/* NAND interface timing parameter 0 register */
#define NDTR0			0x04
/* NAND interface timing parameter 1 register */
#define NDTR1			0x0C
/* NAND controller status register */
#define NDSR			0x14
/* NAND ECC control register */
#define NDECCCTRL		0x28
/* NAND controller data buffer register */
#define NDDB			0x40
/* NAND controller command buffer 0 register */
#define NDCB0			0x48
/* NAND controller command buffer 1 register */
#define NDCB1			0x4C
/* NAND controller command buffer 2 register */
#define NDCB2			0x50
/* NAND controller command buffer 3 register */
#define NDCB3			0x54

/* Data flash control register bitfields */
#define NDCR_ALL_INT		GENMASK(11, 0)
#define NDCR_CS1_CMDDM		BIT(7)
#define NDCR_CS0_CMDDM		BIT(8)
#define NDCR_RDYM		BIT(11)
#define NDCR_ND_ARB_EN		BIT(12)
#define NDCR_RA_START		BIT(15)
#define NDCR_RD_ID_CNT(x)	((x & 0x7) << 16)
#define NDCR_PAGE_SZ(x)		(x >= 2048 ? BIT(24) : 0)
#define NDCR_DWIDTH_M		BIT(26)
#define NDCR_DWIDTH_C		BIT(27)
#define NDCR_ND_RUN		BIT(28)
#define NDCR_ECC_EN		BIT(30)
#define NDCR_SPARE_EN		BIT(31)

/* NAND interface timing parameter registers bitfields */
#define NDTR0_TRP(x)		((min_t(unsigned int, x, 0xF) & 0x7) << 0)
#define NDTR0_TRH(x)		(min_t(unsigned int, x, 0x7) << 3)
#define NDTR0_ETRP(x)		((min_t(unsigned int, x, 0xF) & 0x8) << 3)
#define NDTR0_SEL_NRE_EDGE	BIT(7)
#define NDTR0_TWP(x)		(min_t(unsigned int, x, 0x7) << 8)
#define NDTR0_TWH(x)		(min_t(unsigned int, x, 0x7) << 11)
#define NDTR0_TCS(x)		(min_t(unsigned int, x, 0x7) << 16)
#define NDTR0_TCH(x)		(min_t(unsigned int, x, 0x7) << 19)
#define NDTR0_RD_CNT_DEL(x)	(min_t(unsigned int, x, 0xF) << 22)
#define NDTR0_SELCNTR		BIT(26)
#define NDTR0_TADL(x)		(min_t(unsigned int, x, 0x1F) << 27)

#define NDTR1_TAR(x)		(min_t(unsigned int, x, 0xF) << 0)
#define NDTR1_TWHR(x)		(min_t(unsigned int, x, 0xF) << 4)
#define NDTR1_TRHW(x)		(min_t(unsigned int, x / 16, 0x3) << 8)
#define NDTR1_PRESCALE		BIT(14)
#define NDTR1_WAIT_MODE		BIT(15)
#define NDTR1_TR(x)		(min_t(unsigned int, x, 0xFFFF) << 16)

/* NAND controller status register bitfields */
#define NDSR_WRCMDREQ		BIT(0)
#define NDSR_RDDREQ		BIT(1)
#define NDSR_WRDREQ		BIT(2)
#define NDSR_CORERR		BIT(3)
#define NDSR_UNCERR		BIT(4)
#define NDSR_CMDD(cs)		BIT(8 - cs)
#define NDSR_RDY(rb)		BIT(11 + rb)
#define NDSR_ERRCNT(x)		((x >> 16) & 0x1F)

/* NAND ECC control register bitfields */
#define NDECCTRL_BCH_EN		BIT(0)

/* NAND controller command buffer registers bitfields */
#define NDCB0_CMD1(x)		((x & 0xFF) << 0)
#define NDCB0_CMD2(x)		((x & 0xFF) << 8)
#define NDCB0_ADDR_CYC(x)	((x & 0x7) << 16)
#define NDCB0_DBC		BIT(19)
#define NDCB0_CMD_TYPE(x)	((x & 0x7) << 21)
#define NDCB0_CSEL		BIT(24)
#define NDCB0_RDY_BYP		BIT(27)
#define NDCB0_LEN_OVRD		BIT(28)
#define NDCB0_CMD_XTYPE(x)	((x & 0x7) << 29)

#define NDCB1_COLS(x)		((x & 0xFFFF) << 0)
#define NDCB1_ADDRS(x)		(x << 16)

#define NDCB2_ADDR5(x)		(((x >> 16) & 0xFF) << 0)
#define NDCB2_ADDR5_CS		BIT(7)

/* NAND controller command buffer 0 register 'type' and 'xtype' fields */
#define TYPE_READ		0
#define TYPE_WRITE		1
#define TYPE_READ_ID		3
#define TYPE_RESET		5
#define TYPE_NAKED_CMD		6
#define TYPE_NAKED_ADDR		7
#define XTYPE_MONOLITHIC_READ	0
#define XTYPE_MONOLITHIC_WRITE	0
#define XTYPE_LAST_NAKED_READ	1
#define XTYPE_LAST_NAKED_WRITE	1
#define XTYPE_FINAL_COMMAND	3
#define XTYPE_READ		4
#define XTYPE_WRITE_DISPATCH	4
#define XTYPE_NAKED_READ	5
#define XTYPE_NAKED_WRITE	5
#define XTYPE_COMMAND_DISPATCH	6

/*
 * Marvell ECC engine works differently than the others, in order to limit the
 * size of the IP, hardware engineers choose to set a fixed strength at 16 bits
 * per subpage, and depending on a the desired strength needed by the NAND chip,
 * a particular layout mixing data/spare/ecc is defined, with a possible last
 * chunk smaller that the others.
 *
 * @full_chunk_cnt:	Number of full-sized chunks, which is the number of
 *			repetitions of the pattern:
 *			(data_bytes + spare_bytes + ecc_bytes).
 * @data_bytes:		Number of data bytes per chunk
 * @spare_bytes:	Number of spare bytes per chunk
 * @ecc_bytes:		Number of ecc bytes per chunk
 * @last_chunk_cnt:	If there is a last chunk with a different size than
 *			the first ones, the next fields may not be empty
 * @last_data_bytes:	Number of data bytes in the last chunk
 * @last_spare_bytes:	Number of spare bytes in the last chunk
 * @last_ecc_bytes:	Number of ecc bytes in the last chunk
 */
struct marvell_hw_ecc_layout {
	int full_chunk_cnt;
	int data_bytes;
	int spare_bytes;
	int ecc_bytes;
	int last_chunk_cnt;
	int last_data_bytes;
	int last_spare_bytes;
	int last_ecc_bytes;
};

/*
 * The Nand Flash Controller has up to 4 CE and 2 RB pins. The CE selection
 * is made by a field in NDCB0 register, and in another field in NDCB2 register.
 * The datasheet describes the logic with an error: ADDR5 field is once
 * declared at the beginning of NDCB2, and another time at its end. Because the
 * ADDR5 field of NDCB2 may be used by other bytes, it would be more logical
 * to use the last bit of this field instead of the first ones.
 *
 * @ndcb0_csel:		Value of the NDCB0 register with or without the flag
 *			selecting the wanted CE lane. This is set once when
 *			the Device Tree is probed.
 * @ndcb2_addr5:	Value of the NDCB2 register with or without the flag
 *			selecting the wanted CE lane. This is set once when
 *			the Device Tree is probed.
 * @rb:			Ready/Busy pin for the flash chip
 */
struct marvell_nand_chip_sel {
	u32 ndcb0_csel;
	u32 ndcb2_addr5;
	unsigned int rb;
};

/*
 * NAND chip structure: stores NAND chip device related information
 *
 * @chip:		Base NAND chip structure
 * @node:		Used to store NAND chips into a list
 * @layout		NAND layout when using hardware ECC
 * @ndtr0		Timing registers 0 value for this NAND chip
 * @ndtr1		Timing registers 1 value for this NAND chip
 * @selected:		Current active CS
 * @nsels:		Number of CS lines required by the NAND chip
 * @sels:		Array of CS lines descriptions
 */
struct marvell_nand_chip {
	struct nand_chip chip;
	struct list_head node;
	struct marvell_hw_ecc_layout layout;
	u32 ndtr0;
	u32 ndtr1;
	int addr_cyc;
	int selected;
	unsigned int nsels;
	struct marvell_nand_chip_sel sels[0];
};

static inline struct marvell_nand_chip *to_marvell_nand(struct nand_chip *chip)
{
	return container_of(chip, struct marvell_nand_chip, chip);
}

static inline struct marvell_nand_chip_sel *to_nand_sel(struct marvell_nand_chip
							*nand)
{
	return &nand->sels[nand->selected];
}

enum marvell_nfc_variant {
	MARVELL_NFC_VARIANT_PXA3XX,
	MARVELL_NFC_VARIANT_ARMADA370,
};

/*
 * NAND controller capabilities for distinction between compatible strings
 *
 * @variant:		Board type
 * @max_cs_nb:		Number of Chip Select lines available
 * @max_rb_nb:		Number of Ready/Busy lines available
 * @legacy_of_bindings	Indicates if DT parsing must be done using the old
 *			fashion way
 */
struct marvell_nfc_caps {
	enum marvell_nfc_variant variant;
	unsigned int max_cs_nb;
	unsigned int max_rb_nb;
	bool legacy_of_bindings;
};

/*
 * NAND controller structure: stores Marvell NAND controller information
 *
 * @controller:		Base controller structure
 * @dev:		Parent device (used to print error messages)
 * @regs:		NAND controller registers
 * @ecc_clk:		ECC block clock, two times the NAND controller clock
 * @complete:		Completion object to wait for NAND controller events
 * @assigned_cs:	Bitmask describing already assigned CS lines
 * @chips:		List containing all the NAND chips attached to
 *			this NAND controller
 * @caps:		NAND controller capabilities for each compatible string
 * @buf:		Controller local buffer to store a part of the read
 *			buffer when the read operation was not 8 bytes aligned
 *			as is the FIFO.
 * @buf_pos:		Position in the 'buf' buffer
 */
struct marvell_nfc {
	struct nand_hw_control controller;
	struct device *dev;
	void __iomem *regs;
	struct clk *ecc_clk;
	struct completion complete;
	unsigned long assigned_cs;
	struct list_head chips;
	const struct marvell_nfc_caps *caps;

	/*
	 * Buffer handling: @buf will be accessed byte-per-byter but also
	 * int-per-int when exchanging data with the NAND controller FIFO,
	 * 32-bit alignment is then required.
	 */
	u8 buf[FIFO_DEPTH] __aligned(sizeof(u32));
	int buf_pos;
};

static inline struct marvell_nfc *to_marvell_nfc(struct nand_hw_control *ctrl)
{
	return container_of(ctrl, struct marvell_nfc, controller);
}

/*
 * NAND Controller timings expressed in NAND Controller clock cycles
 *
 * @tRP:		ND_nRE pulse width
 * @tRH:		ND_nRE high duration
 * @tWP:		ND_nWE pulse time
 * @tWH:		ND_nWE high duration
 * @tCS:		Enable signal setup time
 * @tCH:		Enable signal hold time
 * @tADL:		Address to write data delay
 * @tAR:		ND_ALE low to ND_nRE low delay
 * @tWHR:		ND_nWE high to ND_nRE low for status read
 * @tRHW:		ND_nRE high duration, read to write delay
 * @tR:			ND_nWE high to ND_nRE low for read
 */
struct marvell_nfc_timings {
	/* NDTR0 fields */
	unsigned int tRP;
	unsigned int tRH;
	unsigned int tWP;
	unsigned int tWH;
	unsigned int tCS;
	unsigned int tCH;
	unsigned int tADL;
	/* NDTR1 fields */
	unsigned int tAR;
	unsigned int tWHR;
	unsigned int tRHW;
	unsigned int tR;
};

/*
 * Derives a duration in numbers of clock cycles.
 *
 * @ps: Duration in pico-seconds
 * @period_ns:  Clock period in nano-seconds
 *
 * Convert the duration in nano-seconds, then divide by the period and
 * return the number of clock periods.
 */
#define TO_CYCLES(ps, period_ns) (DIV_ROUND_UP(ps / 1000, period_ns))

/*
 * Internal hook to mimic core functions whithout having to distinguish if this
 * is the first read operation on the page or not and hence choose the right
 * function.
 */
int nand_read_page(struct nand_chip *chip, unsigned int page, unsigned int column,
		   void *buf, unsigned int len)
{
	if (!column)
		return nand_read_page_op(chip, page, column, buf, len);
	else
		return nand_change_read_column_op(chip, column, buf, len);
}

/*
 * The controller has many flags that could generate interrupts, most of them
 * are disabled and polling is used. For the very slow signals, using interrupts
 * may relax the CPU charge.
 */
static void marvell_nfc_disable_int(struct marvell_nfc *nfc, u32 int_mask)
{
	u32 reg;

	/* Writing 1 disables the interrupt */
	reg = readl_relaxed(nfc->regs + NDCR);
	writel_relaxed(reg | int_mask, nfc->regs + NDCR);
}

static void marvell_nfc_enable_int(struct marvell_nfc *nfc, u32 int_mask)
{
	u32 reg;

	/* Writing 0 enables the interrupt */
	reg = readl_relaxed(nfc->regs + NDCR);
	writel_relaxed(reg & ~int_mask, nfc->regs + NDCR);
}

static void marvell_nfc_clear_int(struct marvell_nfc *nfc, u32 int_mask)
{
	writel_relaxed(int_mask, nfc->regs + NDSR);
}

/*
 * Any time a command has to be sent to the controller, the following sequence
 * has to be followed:
 * - call marvell_nfc_prepare_cmd()
 *      -> activate the ND_RUN bit that will kind of 'start a job'
 *      -> wait the signal indicating the NFC is waiting for a command
 * - send the command (cmd and address cycles)
 * - enventually send or receive the data
 * - call marvell_nfc_end_cmd() with the corresponding flag
 *      -> wait the flag to be triggered or cancel the job with a timeout
 *
 * The following functions are helpers to do this job and keep in the
 * specialized functions the code that really does the operations.
 */
static int marvell_nfc_prepare_cmd(struct nand_chip *chip)
{
	struct marvell_nfc *nfc = to_marvell_nfc(chip->controller);
	u32 ndcr, val;
	int ret;

	/* Deassert ND_RUN and clear NDSR before issuing any command */
	ndcr = readl_relaxed(nfc->regs + NDCR);
	writel_relaxed(ndcr & ~NDCR_ND_RUN, nfc->regs + NDCR);
	writel_relaxed(readl_relaxed(nfc->regs + NDSR), nfc->regs + NDSR);

	/* Assert ND_RUN bit and wait the NFC to be ready */
	writel_relaxed(ndcr | NDCR_ND_RUN, nfc->regs + NDCR);
	ret = readl_relaxed_poll_timeout(nfc->regs + NDSR, val,
					 val & NDSR_WRCMDREQ,
					 POLL_PERIOD, POLL_TIMEOUT);
	if (ret) {
		dev_err(nfc->dev, "Timeout on WRCMDRE\n");
		return -ETIMEDOUT;
	}

	/* Command may be written, clear WRCMDREQ status bit */
	writel_relaxed(NDSR_WRCMDREQ, nfc->regs + NDSR);

	return 0;
}

static void marvell_nfc_send_cmd(struct nand_chip *chip, u32 ndcb0, u32 ndcb1,
				 u32 ndcb2, u32 ndcb3)
{
	struct marvell_nand_chip *marvell_nand = to_marvell_nand(chip);
	struct marvell_nfc *nfc = to_marvell_nfc(chip->controller);

	writel_relaxed(to_nand_sel(marvell_nand)->ndcb0_csel | ndcb0,
		       nfc->regs + NDCB0);
	writel_relaxed(ndcb1, nfc->regs + NDCB0);
	writel(to_nand_sel(marvell_nand)->ndcb2_addr5 | ndcb2,
	       nfc->regs + NDCB0);

	/* Write NDCB0 four times only if LEN_OVRD is set */
	if (ndcb0 & NDCB0_LEN_OVRD)
		writel(ndcb3, nfc->regs + NDCB0);
}

static int marvell_nfc_wait_ndrun(struct nand_chip *chip)
{
	struct marvell_nfc *nfc = to_marvell_nfc(chip->controller);
	u32 val;
	int ret;

	/*
	 * The command is being processed, wait for the ND_RUN bit to be
	 * cleared by the NFC. If not, we must clear it by hand.
	 */
	ret = readl_relaxed_poll_timeout(nfc->regs + NDCR, val,
					 (val & NDCR_ND_RUN) == 0,
					 POLL_PERIOD, POLL_TIMEOUT);
	if (ret) {
		dev_err(nfc->dev, "Timeout on NAND controller run mode\n");
		writel_relaxed(readl_relaxed(nfc->regs + NDCR) & ~NDCR_ND_RUN,
			       nfc->regs + NDCR);
		return ret;
	}

	return 0;
}

static int marvell_nfc_end_cmd(struct nand_chip *chip, int flag,
			       const char *label)
{
	struct marvell_nfc *nfc = to_marvell_nfc(chip->controller);
	u32 val;
	int ret;

	ret = readl_relaxed_poll_timeout(nfc->regs + NDSR, val,
					 val & flag,
					 POLL_PERIOD, POLL_TIMEOUT);
	if (ret) {
		dev_err(nfc->dev, "Timeout on %s\n", label);
		return ret;
	}

	writel_relaxed(flag, nfc->regs + NDSR);

	return 0;
}

static void marvell_nfc_wait_cmdd(struct nand_chip *chip)
{
	struct marvell_nand_chip *marvell_nand = to_marvell_nand(chip);
	int cs_flag = NDSR_CMDD(to_nand_sel(marvell_nand)->ndcb0_csel);

	marvell_nfc_end_cmd(chip, cs_flag, "CMDD");
}

static int marvell_nfc_dev_ready(struct mtd_info *mtd)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct marvell_nand_chip *marvell_nand = to_marvell_nand(chip);
	struct marvell_nfc *nfc = to_marvell_nfc(chip->controller);
	int rdy_flag = NDSR_RDY(to_nand_sel(marvell_nand)->rb);
	u32 ndsr = readl_relaxed(nfc->regs + NDSR);

	if (ndsr & rdy_flag) {
		writel_relaxed(rdy_flag, nfc->regs + NDSR);
		return 1;
	}

	return 0;
}

static int marvell_nfc_waitfunc(struct mtd_info *mtd, struct nand_chip *chip)
{
	struct marvell_nfc *nfc = to_marvell_nfc(chip->controller);
	int ret;

	init_completion(&nfc->complete);

	marvell_nfc_enable_int(nfc, NDCR_RDYM);
	ret = wait_for_completion_timeout(&nfc->complete,
					  msecs_to_jiffies(IRQ_TIMEOUT));
	marvell_nfc_disable_int(nfc, NDCR_RDYM);
	marvell_nfc_clear_int(nfc, NDSR_RDY(0) | NDSR_RDY(1));

	if (!ret)
		dev_err(nfc->dev, "Timeout waiting for RB signal\n");

	return ret ? 0 : -ETIMEDOUT;
}

static int marvell_nfc_wait_op(struct nand_chip *chip, unsigned int timeout)
{
	struct marvell_nfc *nfc = to_marvell_nfc(chip->controller);
	int ret;

	/* Timeout is expressed in ms */
	if (!timeout)
		timeout = IRQ_TIMEOUT;

	init_completion(&nfc->complete);

	marvell_nfc_enable_int(nfc, NDCR_RDYM);
	ret = wait_for_completion_timeout(&nfc->complete,
					  msecs_to_jiffies(timeout));
	marvell_nfc_disable_int(nfc, NDCR_RDYM);
	marvell_nfc_clear_int(nfc, NDSR_RDY(0) | NDSR_RDY(1));

	if (!ret)
		dev_err(nfc->dev, "Timeout waiting for RB signal\n");

	return ret ? 0 : -ETIMEDOUT;
}

static void marvell_nfc_cmd_ctrl(struct mtd_info *mtd, int data,
				 unsigned int ctrl)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct marvell_nfc *nfc = to_marvell_nfc(chip->controller);

	if (ctrl & (NAND_CLE | NAND_ALE)) {
		/*
		 * Reset the position used in the ->read/write{byte,word,buf}()
		 * helpers to trigger a naked read/write.
		 */
		nfc->buf_pos = 0;

		if (marvell_nfc_prepare_cmd(chip))
			return;

		/*
		 * Marvell NFC may use naked commands and naked addresses.
		 *
		 * NDCB{1-3} registers are RO while NDCB0 is RW.
		 * NFC waits for all these three registers to be written
		 * by the use of NDCB0 only (it acts as a FIFO)
		 */
		if (ctrl & NAND_CLE)
			marvell_nfc_send_cmd(chip, data |
					     NDCB0_CMD_TYPE(TYPE_NAKED_CMD),
					     0, 0, 0);
		else if (ctrl & NAND_ALE)
			marvell_nfc_send_cmd(chip,
					     NDCB0_CMD_TYPE(TYPE_NAKED_ADDR) |
					     NDCB0_ADDR_CYC(1),
					     data, 0, 0);

		marvell_nfc_wait_ndrun(chip);
	}
}

static void marvell_nfc_select_chip(struct mtd_info *mtd, int chip)
{
	struct nand_chip *nand = mtd_to_nand(mtd);
	struct marvell_nand_chip *marvell_nand = to_marvell_nand(nand);
	struct marvell_nfc *nfc = to_marvell_nfc(nand->controller);
	u32 ndcr;

	if (chip < 0 || chip >= marvell_nand->nsels)
		return;

	if (chip == marvell_nand->selected)
		return;

	/*
	 * Do not change the timing registers when using the DT property
	 * marvell,nand-keep-config; in that case ->ndtr0 and ->ndtr1 from the
	 * marvell_nand structure are supposedly empty.
	 */
	if (marvell_nand->ndtr0 && marvell_nand->ndtr1) {
		writel_relaxed(marvell_nand->ndtr0, nfc->regs + NDTR0);
		writel_relaxed(marvell_nand->ndtr1, nfc->regs + NDTR1);
	}

	ndcr = readl_relaxed(nfc->regs + NDCR);

	/* Ensure controller is not blocked in operation */
	ndcr &= ~NDCR_ND_RUN;

	/* Adapt bus width */
	if (nand->options & NAND_BUSWIDTH_16)
		ndcr |= NDCR_DWIDTH_M | NDCR_DWIDTH_C;

	/*
	 * Page size as seen by the controller, either 512B or 2kiB. This size
	 * will be the reference for the controller when using LEN_OVRD.
	 */
	ndcr |= NDCR_PAGE_SZ(mtd->writesize);

	/* Update the control register */
	writel_relaxed(ndcr,  nfc->regs + NDCR);

	/* Also reset the interrupt status register */
	marvell_nfc_clear_int(nfc, NDCR_ALL_INT);

	marvell_nand->selected = chip;
}

static irqreturn_t marvell_nfc_isr(int irq, void *dev_id)
{
	struct marvell_nfc *nfc = dev_id;
	u32 st = readl_relaxed(nfc->regs + NDSR);
	u32 ien = (~readl_relaxed(nfc->regs + NDCR)) & NDCR_ALL_INT;

	/*
	 * RDY interrupt mask is one bit in NDCR while there are two status
	 * bit in NDSR (RDY[cs0/cs2] and RDY[cs1/cs3]).
	 */
	if (st & NDSR_RDY(1))
		st |= NDSR_RDY(0);

	if (!(st & ien))
		return IRQ_NONE;

	marvell_nfc_disable_int(nfc, st & NDCR_ALL_INT);

	complete(&nfc->complete);

	return IRQ_HANDLED;
}

static void marvell_nfc_do_naked_read(struct nand_chip *chip, int len)
{
	if (marvell_nfc_prepare_cmd(chip))
		return;

	marvell_nfc_send_cmd(chip, NDCB0_CMD_TYPE(TYPE_READ) |
			     NDCB0_CMD_XTYPE(XTYPE_LAST_NAKED_READ) |
			     NDCB0_LEN_OVRD, 0, 0, len);

	marvell_nfc_end_cmd(chip, NDSR_RDDREQ, "RDDREQ during naked read");
}

static void marvell_nfc_read_buf(struct mtd_info *mtd, u8 *buf, int len)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct marvell_nfc *nfc = to_marvell_nfc(chip->controller);
	int rounded_len, i = 0, boundary = 0, step_len = 0;
	bool wait_cmdd = false;

	/* If there are valid bytes in the local buffer, copy them */
	if (nfc->buf_pos) {
		i = min_t(int, len, FIFO_DEPTH - nfc->buf_pos);
		memcpy(buf, &nfc->buf[nfc->buf_pos], i);
		nfc->buf_pos = (nfc->buf_pos + i) % FIFO_DEPTH;
		/* If there is no more data to read, operation is finished */
		if (i == len)
			return;
	}

	/* Drain FIFO */
	while (i < len) {
		/* Do a naked read any time boundary is reached */
		if (i >= boundary) {
			if (len - i > MAX_CHUNK)
				rounded_len = MAX_CHUNK;
			else
				rounded_len = round_up(len - i, FIFO_DEPTH);

			/* Wait for CMDD signal between operations */
			if (wait_cmdd) {
				wait_cmdd = false;
				marvell_nfc_wait_cmdd(chip);
			}

			marvell_nfc_do_naked_read(chip, rounded_len);
			wait_cmdd = true;
			boundary += rounded_len;
		}

		/*
		 * Cannot ioread32 directly into &buf[i] (might be unaligned) so
		 * always write to nfc->buf (which is aligned) and copy the
		 * right number of bytes to ->buf and move forward accordingly.
		 */
		step_len = min_t(int, len - i, FIFO_DEPTH);

		ioread32_rep(nfc->regs + NDDB, (u32 *)&nfc->buf, FIFO_DEPTH_32);

		memcpy(buf + i, nfc->buf, step_len);
		i += step_len;
	}

	if (wait_cmdd)
		marvell_nfc_wait_cmdd(chip);

	if (step_len < FIFO_DEPTH)
		nfc->buf_pos = step_len;
}

static u8 marvell_nfc_read_byte(struct mtd_info *mtd)
{
	u8 data;

	marvell_nfc_read_buf(mtd, &data, 1);

	return data;
}

static u16 marvell_nfc_read_word(struct mtd_info *mtd)
{
	u8 data[2];

	marvell_nfc_read_buf(mtd, data, 2);

	return data[0] + (data[1] << 8);
}

static void marvell_nfc_do_naked_write(struct nand_chip *chip, int len)
{
	if (marvell_nfc_prepare_cmd(chip))
		return;

	/* Trigger the naked write operation */
	marvell_nfc_send_cmd(chip, NDCB0_CMD_TYPE(TYPE_WRITE) |
			     NDCB0_CMD_XTYPE(XTYPE_NAKED_WRITE) |
			     NDCB0_LEN_OVRD, 0, 0, len);

	marvell_nfc_end_cmd(chip, NDSR_WRDREQ, "WRDREQ during naked write");
}

static void marvell_nfc_write_buf(struct mtd_info *mtd, const u8 *buf,
				  int len)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct marvell_nfc *nfc = to_marvell_nfc(chip->controller);
	int remaining_bytes = 0, i = 0, j;
	int rounded_len, distance, write_length;
	u8 fifo[FIFO_DEPTH] __aligned(sizeof(u32));

	/* While there are remaining bytes to write */
	while (i < len) {
		/*
		 * Derive the number of remaining bytes, with respect to the
		 * following boundaries:
		 * - The chunk to write cannot be larger than MAX_CHUNK
		 * - The chunk to write cannot be smaller than FIFO_DEPTH
		 * Remaining bytes indicates the number of bytes that will
		 * remain in the last uncomplete write (less than
		 * FIFO_DEPTH bytes to write). They will then be copied in
		 * the fifo array (padded with 0xFF).
		 */
		distance = len - i;
		if (distance >= MAX_CHUNK) {
			rounded_len = MAX_CHUNK;
		} else {
			rounded_len = round_down(distance, FIFO_DEPTH);
			remaining_bytes = distance - rounded_len;
		}

		/* write_length matches the number of data cycles */
		if (remaining_bytes)
			write_length = rounded_len + FIFO_DEPTH;
		else
			write_length = rounded_len;

		/* Trigger the naked wrrite operation */
		marvell_nfc_do_naked_write(chip, write_length);

		/*
		 * Effectively write the data to the FIFO. Take some extra
		 * precautions in case buf[] would not be aligned.
		 */
		for (j = 0; j < rounded_len / FIFO_DEPTH; j++) {
			memcpy(fifo, &buf[i + (j * FIFO_DEPTH)], FIFO_DEPTH);
			iowrite32_rep(nfc->regs + NDDB, fifo, FIFO_DEPTH_32);
		}

		if (remaining_bytes) {
			/*
			 * If the write operation is not aligned on
			 * FIFO_DEPTH, pad with empty bytes: 0xFF.
			 */
			memset(fifo, 0xFF, FIFO_DEPTH);
			memcpy(fifo, &buf[i + rounded_len], remaining_bytes);
			iowrite32_rep(nfc->regs + NDDB, fifo, FIFO_DEPTH_32);
		}

		i += write_length;

		marvell_nfc_wait_cmdd(chip);
	}
}

/* HW ECC related functions */
static void marvell_nfc_hw_ecc_enable(struct nand_chip *chip)
{
	struct marvell_nfc *nfc = to_marvell_nfc(chip->controller);
	u32 ndcr = readl_relaxed(nfc->regs + NDCR);

	if (!(ndcr & NDCR_ECC_EN)) {
		writel_relaxed(ndcr | NDCR_ECC_EN | NDCR_SPARE_EN,
			       nfc->regs + NDCR);

		/*
		 * When enabling BCH, set threshold to 0 to always know the
		 * number of corrected bitflips.
		 */
		if (chip->ecc.algo == NAND_ECC_BCH)
			writel_relaxed(NDECCTRL_BCH_EN, nfc->regs + NDECCCTRL);
	}
}

static void marvell_nfc_hw_ecc_disable(struct nand_chip *chip)
{
	struct marvell_nfc *nfc = to_marvell_nfc(chip->controller);
	u32 ndcr = readl_relaxed(nfc->regs + NDCR);

	if (ndcr & NDCR_ECC_EN) {
		writel_relaxed(ndcr & ~(NDCR_ECC_EN | NDCR_SPARE_EN),
			       nfc->regs + NDCR);
		if (chip->ecc.algo == NAND_ECC_BCH)
			writel_relaxed(0, nfc->regs + NDECCCTRL);
	}
}

static void marvell_nfc_hw_ecc_correct(struct nand_chip *chip,
				       u8 *data, int data_len,
				       u8 *oob, int oob_len,
				       unsigned int *max_bitflips)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	struct marvell_nfc *nfc = to_marvell_nfc(chip->controller);
	int bf = 0;
	u32 ndsr;

	ndsr = readl_relaxed(nfc->regs + NDSR);

	/* Check uncorrectable error flag */
	if (ndsr & NDSR_UNCERR) {
		writel_relaxed(ndsr, nfc->regs + NDSR);

		/*
		 * Blank pages (all 0xFF) with no ECC are recognized as bad
		 * because hardware ECC engine expects non-empty ECC values
		 * in that case, so whenever an uncorrectable error occurs,
		 * check if the page is actually blank or not.
		 *
		 * It is important to check the emptyness only on oob_len,
		 * which only covers the spare bytes because after a read with
		 * ECC enabled, the ECC bytes in the buffer have been set by the
		 * ECC engine, so they are not 0xFF.
		 */
		if (!data)
			data_len = 0;
		if (!oob)
			oob_len = 0;
		bf = nand_check_erased_ecc_chunk(data, data_len, NULL, 0,
						 oob, oob_len,
						 chip->ecc.strength);
		if (bf < 0) {
			mtd->ecc_stats.failed++;
			return;
		}
	}

	/* Check correctable error flag */
	if (ndsr & NDSR_CORERR) {
		writel_relaxed(ndsr, nfc->regs + NDSR);

		if (chip->ecc.algo == NAND_ECC_BCH)
			bf = NDSR_ERRCNT(ndsr);
		else
			bf = 1;
	}

	/*
	 * Derive max_bitflips either from the number of bitflips detected by
	 * the hardware ECC engine or by nand_check_erased_ecc_chunk().
	 */
	mtd->ecc_stats.corrected += bf;
	*max_bitflips = max_t(unsigned int, *max_bitflips, bf);
}

/* Reads with HW ECC */
static int marvell_nfc_hw_ecc_hmg_read_page(struct mtd_info *mtd,
					    struct nand_chip *chip,
					    u8 *buf, int oob_required,
					    int page)
{
	struct marvell_nand_chip *marvell_nand = to_marvell_nand(chip);
	struct marvell_nfc *nfc = to_marvell_nfc(chip->controller);
	struct marvell_hw_ecc_layout *lt = &to_marvell_nand(chip)->layout;
	int max_bitflips = 0, ret = 0, i;
	u8 *data, *oob;

	/*
	 * With Hamming, OOB is not fully used (and thus not read entirely), not
	 * expected bytes could show up at the end of the OOB buffer if not
	 * explicitly erased.
	 */
	if (oob_required)
		memset(chip->oob_poi, 0xFF, mtd->oobsize);

	if ((ret = marvell_nfc_prepare_cmd(chip)))
		return ret;

	marvell_nfc_hw_ecc_enable(chip);

	data = buf;
	oob = chip->oob_poi;

	/*
	 * Reading spare area is mandatory when using HW ECC or read operation
	 * will trigger uncorrectable ECC errors, but do not read ECC here.
	 */
	marvell_nfc_send_cmd(chip,
			     NDCB0_CMD_TYPE(TYPE_READ) |
			     NDCB0_CMD_XTYPE(XTYPE_MONOLITHIC_READ) |
			     NDCB0_ADDR_CYC(marvell_nand->addr_cyc) |
			     NDCB0_DBC |
			     NDCB0_CMD1(NAND_CMD_READ0) |
			     NDCB0_CMD2(NAND_CMD_READSTART),
			     NDCB1_ADDRS(page), NDCB2_ADDR5(page), 0);

	marvell_nfc_end_cmd(chip, NDSR_RDDREQ,
			    "RDDREQ while draining FIFO (data)");

	/* Read the data... */
	if (data)
		ioread32_rep(nfc->regs + NDDB, data, FIFO_REP(lt->data_bytes));
	else
		for (i = 0; i < FIFO_REP(lt->data_bytes); i++)
			ioread32_rep(nfc->regs + NDDB, nfc->buf, FIFO_DEPTH_32);

	/* ...then the spare bytes */
	ioread32_rep(nfc->regs + NDDB, oob, FIFO_REP(lt->spare_bytes));

	marvell_nfc_hw_ecc_correct(chip, data, lt->data_bytes,
				   oob, lt->spare_bytes, &max_bitflips);

	marvell_nfc_hw_ecc_disable(chip);

	if (oob_required) {
		/* Read ECC bytes without ECC enabled */
		nand_read_page_op(chip, page,
				  lt->data_bytes + lt->spare_bytes,
				  oob + lt->spare_bytes, lt->ecc_bytes);
	}

	return max_bitflips;
}

static void marvell_nfc_hw_ecc_bch_read_chunk(struct nand_chip *chip, int chunk,
					      u8 *data, int data_len,
					      u8 *oob, int oob_len,
					      int oob_required, int page)
{
	struct marvell_nand_chip *marvell_nand = to_marvell_nand(chip);
	struct marvell_nfc *nfc = to_marvell_nfc(chip->controller);
	struct marvell_hw_ecc_layout *lt = &to_marvell_nand(chip)->layout;
	int read_len, xtype, i, j;
	u32 command_bytes = 0;

	/*
	 * Reading spare area is mandatory when using HW ECC or read operation
	 * will trigger uncorrectable ECC errors, but do not read ECC here.
	 */
	read_len = data_len + oob_len;

	if (marvell_nfc_prepare_cmd(chip))
		return;

	if (chunk == 0)
		command_bytes =	NDCB0_DBC |
			NDCB0_CMD1(NAND_CMD_READ0) |
			NDCB0_CMD2(NAND_CMD_READSTART);

	/*
	 * Trigger the naked read operation only on the last chunk.
	 * Otherwise, use monolithic read.
	 */
	if ((lt->last_chunk_cnt && chunk == lt->full_chunk_cnt) ||
	    (!lt->last_chunk_cnt && chunk == lt->full_chunk_cnt - 1))
		xtype = XTYPE_LAST_NAKED_READ;
	else
		xtype = XTYPE_MONOLITHIC_READ;

	marvell_nfc_send_cmd(chip,
			     NDCB0_CMD_TYPE(TYPE_READ) |
			     NDCB0_CMD_XTYPE(xtype) |
			     NDCB0_ADDR_CYC(marvell_nand->addr_cyc) |
			     command_bytes |
			     NDCB0_LEN_OVRD,
			     NDCB1_ADDRS(page), NDCB2_ADDR5(page), read_len);

	/*
	 * According to the datasheet, when reading from NDDB
	 * with BCH enabled, after each 32 bytes reads, we
	 * have to make sure that the NDSR.RDDREQ bit is set.
	 *
	 * Drain the FIFO, 8 32-bit reads at a time, and skip
	 * the polling on the last read.
	 *
	 * Length is a multiple of 32 bytes, hence it is a multiple of 8 too.
	 *
	 */
	for (i = 0; i < data_len; i += FIFO_DEPTH * sizeof(u32)) {
		marvell_nfc_end_cmd(chip, NDSR_RDDREQ,
				    "RDDREQ while draining FIFO (data)");
		if (data) {
			ioread32_rep(nfc->regs + NDDB, data,
				     FIFO_DEPTH_32 * sizeof(u32));
			data += FIFO_DEPTH * sizeof(u32);
		} else {
			for (j = 0; j < sizeof(u32); j++)
				ioread32_rep(nfc->regs + NDDB, nfc->buf,
					     FIFO_DEPTH_32);
		}
	}

	for (i = 0; i < oob_len; i += FIFO_DEPTH * 4) {
		marvell_nfc_end_cmd(chip, NDSR_RDDREQ,
				    "RDDREQ while draining FIFO (OOB)");
		ioread32_rep(nfc->regs + NDDB, oob, FIFO_DEPTH_32 * 4);
		oob += FIFO_DEPTH * 4;
	}
}

static int marvell_nfc_hw_ecc_bch_read_page(struct mtd_info *mtd,
					    struct nand_chip *chip,
					    u8 *buf, int oob_required,
					    int page)
{
	struct marvell_hw_ecc_layout *lt = &to_marvell_nand(chip)->layout;
	int nchunks = lt->full_chunk_cnt + lt->last_chunk_cnt;
	int max_bitflips = 0;
	u8 *data, *oob;
	int chunk, data_len, oob_len, ecc_len;
	/* Following sizes are used to read the ECC bytes after ECC operation */
	int fixed_oob_size = lt->spare_bytes + lt->ecc_bytes;
	int fixed_chunk_size = lt->data_bytes + fixed_oob_size;

	/*
	 * With BCH, OOB is not fully used (and thus not read entirely), not
	 * expected bytes could show up at the end of the OOB buffer if not
	 * explicitly erased.
	 */
	if (oob_required)
		memset(chip->oob_poi, 0xFF, mtd->oobsize);

	marvell_nfc_hw_ecc_enable(chip);

	for (chunk = 0; chunk < nchunks; chunk++) {
		if (chunk == 0) {
			/* Init pointers to iterate through the chunks */
			if (buf)
				data = buf;
			else
				data = NULL;
			oob = chip->oob_poi;
		} else {
			/* Update pointers */
			if (data)
				data += lt->data_bytes;
			oob += (lt->spare_bytes + lt->ecc_bytes + 2);
		}

		/* Update length */
		if (chunk < lt->full_chunk_cnt) {
			data_len = lt->data_bytes;
			oob_len = lt->spare_bytes;
			ecc_len = lt->ecc_bytes;
		} else {
			data_len = lt->last_data_bytes;
			oob_len = lt->last_spare_bytes;
			ecc_len = lt->last_ecc_bytes;
		}

		/* Read the chunk and detect number of bitflips */
		marvell_nfc_hw_ecc_bch_read_chunk(chip, chunk, data, data_len,
						  oob, oob_len, oob_required,
						  page);

		marvell_nfc_hw_ecc_correct(chip, data, data_len,
					   oob, oob_len, &max_bitflips);
	}

	marvell_nfc_hw_ecc_disable(chip);

	if (!oob_required)
		return max_bitflips;

	/* Read ECC bytes without ECC enabled */
	for (chunk = 0; chunk < lt->full_chunk_cnt; chunk++)
		nand_read_page(chip, page,
			       ((chunk + 1) * (fixed_chunk_size)) -
			       lt->ecc_bytes,
			       chip->oob_poi + ((chunk + 1) *
						(fixed_oob_size + 2) -
						(lt->ecc_bytes + 2)),
			       lt->ecc_bytes);

	if (lt->last_chunk_cnt)
		nand_read_page(chip, page,
			       (lt->full_chunk_cnt * fixed_chunk_size) +
			       lt->last_data_bytes + lt->last_spare_bytes,
			       chip->oob_poi + (lt->full_chunk_cnt *
						(fixed_oob_size + 2)) +
			       lt->last_spare_bytes,
			       lt->last_ecc_bytes);

	return max_bitflips;
}

static int marvell_nfc_hw_ecc_read_oob(struct mtd_info *mtd,
				       struct nand_chip *chip, int page)
{
	return chip->ecc.read_page(mtd, chip, NULL, true, page);
}

/* Raw reads with HW ECC */
static int marvell_nfc_hw_ecc_read_page_raw(struct mtd_info *mtd,
					    struct nand_chip *chip, u8 *buf,
					    int oob_required, int page)
{
	struct marvell_hw_ecc_layout *lt = &to_marvell_nand(chip)->layout;
	int chunk_size = lt->data_bytes + lt->spare_bytes + lt->ecc_bytes;
	u8 *oob = chip->oob_poi;
	int chunk;

	if (oob_required)
		memset(chip->oob_poi, 0xFF, mtd->oobsize);

	for (chunk = 0; chunk < lt->full_chunk_cnt; chunk++) {
		nand_read_page(chip, page, chunk * chunk_size, buf,
			       lt->data_bytes);
		buf += lt->data_bytes;

		if (oob_required) {
			nand_read_data_op(chip, oob, lt->spare_bytes +
					  lt->ecc_bytes, false);
			/* Pad user data with 2 bytes when using BCH (30B) */
			oob += lt->spare_bytes + lt->ecc_bytes + 2;
		}
	}

	if (!lt->last_chunk_cnt)
		return 0;

	nand_read_page(chip, page, chunk * chunk_size, buf,
		       lt->last_data_bytes);
	if (oob_required)
		nand_read_data_op(chip, oob, lt->last_spare_bytes +
				  lt->last_ecc_bytes, false);

	return 0;
}

static int marvell_nfc_hw_ecc_read_oob_raw(struct mtd_info *mtd,
					   struct nand_chip *chip, int page)
{
	struct marvell_hw_ecc_layout *lt = &to_marvell_nand(chip)->layout;
	u8 *oob = chip->oob_poi;
	int chunk_size = lt->data_bytes + lt->spare_bytes + lt->ecc_bytes;
	int chunk;

	for (chunk = 0; chunk < lt->full_chunk_cnt; chunk++) {
		/* Move NAND pointer to the next chunk of OOB data */
		nand_read_page(chip, page,
			       chunk * chunk_size + lt->data_bytes,
			       oob, lt->spare_bytes + lt->ecc_bytes);
		/* Pad user data with 2 bytes when using BCH (30B) */
		oob += lt->spare_bytes + lt->ecc_bytes + 2;
	}

	if (lt->last_chunk_cnt)
		nand_read_data_op(chip, oob,
				  lt->last_spare_bytes + lt->last_ecc_bytes,
				  false);

	return 0;
}

/* Writes with HW ECC */
static int marvell_nfc_hw_ecc_hmg_write_page(struct mtd_info *mtd,
					     struct nand_chip *chip,
					     const u8 *buf,
					     int oob_required, int page)
{
	struct marvell_nand_chip *marvell_nand = to_marvell_nand(chip);
	struct marvell_nfc *nfc = to_marvell_nfc(chip->controller);
	struct marvell_hw_ecc_layout *lt = &to_marvell_nand(chip)->layout;
	const u8 *data = buf, *oob = chip->oob_poi;
	int status, ret, i;

	if ((ret = marvell_nfc_prepare_cmd(chip)))
		return ret;

	marvell_nfc_hw_ecc_enable(chip);

	marvell_nfc_send_cmd(chip,
			     NDCB0_CMD_TYPE(TYPE_WRITE) |
			     NDCB0_CMD_XTYPE(XTYPE_MONOLITHIC_WRITE) |
			     NDCB0_ADDR_CYC(marvell_nand->addr_cyc) |
			     NDCB0_CMD1(NAND_CMD_SEQIN),
			     NDCB1_ADDRS(page), NDCB2_ADDR5(page), 0);

	marvell_nfc_end_cmd(chip, NDSR_WRDREQ,
			    "WRDREQ while loading FIFO (data)");

	/* Write the data. If buf is empty, write empty bytes (0xFF) */
	if (data) {
		iowrite32_rep(nfc->regs + NDDB, data, FIFO_REP(lt->data_bytes));
	} else {
		data = nfc->buf;
		memset(nfc->buf, 0xFF, FIFO_DEPTH);
		for (i = 0; i < FIFO_REP(lt->data_bytes) / FIFO_DEPTH_32; i++)
			iowrite32_rep(nfc->regs + NDDB, data, FIFO_DEPTH_32);
	}

	/* Then write the OOB data */
	iowrite32_rep(nfc->regs + NDDB, oob, FIFO_REP(lt->spare_bytes));

	status = marvell_nfc_waitfunc(mtd, chip);

	marvell_nfc_hw_ecc_disable(chip);

	if (status & NAND_STATUS_FAIL)
		return -EIO;

	return 0;
}

static void marvell_nfc_hw_ecc_bch_write_chunk(struct nand_chip *chip, int chunk,
					       const u8 *data, u8 *oob,
					       int oob_required, int page)
{
	struct marvell_nand_chip *marvell_nand = to_marvell_nand(chip);
	struct marvell_nfc *nfc = to_marvell_nfc(chip->controller);
	struct marvell_hw_ecc_layout *lt = &to_marvell_nand(chip)->layout;
	int data_len, oob_len, write_len, xtype, i;
	u32 command_bytes, address_bytes_ndcb1, address_bytes_ndcb2;

	/* OOB area to write is only spare area when using HW ECC */
	if (chunk < lt->full_chunk_cnt) {
		data_len = lt->data_bytes;
		oob_len = lt->spare_bytes;
	} else {
		data_len = lt->last_data_bytes;
		oob_len = lt->last_spare_bytes;
	}

	write_len = data_len + oob_len;

	/*
	 * First operation dispatches the CMD_SEQIN command, issue the address
	 * cycles and asks for the first chunk of data.
	 * Last operation dispatches the PAGEPROG command and also asks for the
	 * last chunk of data.
	 * All operations in the middle (if any) will issue a naked write and
	 * also ask for data.
	 */
	if (chunk == 0) {
		xtype = XTYPE_WRITE_DISPATCH;
		command_bytes =
			NDCB0_ADDR_CYC(marvell_nand->addr_cyc) |
			NDCB0_CMD1(NAND_CMD_SEQIN);
		address_bytes_ndcb1 = NDCB1_ADDRS(page);
		address_bytes_ndcb2 = NDCB2_ADDR5(page);
	} else if (
		(lt->last_chunk_cnt && (chunk == lt->full_chunk_cnt)) ||
		(!lt->last_chunk_cnt &&	(chunk == lt->full_chunk_cnt - 1))) {
		xtype = XTYPE_LAST_NAKED_WRITE;
		command_bytes = NDCB0_CMD2(NAND_CMD_PAGEPROG) | NDCB0_DBC;
		address_bytes_ndcb1 = 0;
		address_bytes_ndcb2 = 0;
	} else {
		xtype = XTYPE_NAKED_WRITE;
		command_bytes = 0;
		address_bytes_ndcb1 = 0;
		address_bytes_ndcb2 = 0;
	}

	/*
	 * If this is the first chunk, the previous command also embedded
	 * the write operation, no need to repeat it.
	 */
	if (marvell_nfc_prepare_cmd(chip))
		return;

	marvell_nfc_send_cmd(chip,
			     NDCB0_CMD_TYPE(TYPE_WRITE) |
			     NDCB0_CMD_XTYPE(xtype) |
			     command_bytes |
			     NDCB0_LEN_OVRD,
			     address_bytes_ndcb1, address_bytes_ndcb2,
			     write_len);

	marvell_nfc_end_cmd(chip, NDSR_WRDREQ,
			    "WRDREQ while loading FIFO (data)");

	/* Effectively write the data to the data buffer */
	if (data) {
		data += chunk * lt->data_bytes;
		iowrite32_rep(nfc->regs + NDDB, data, FIFO_REP(data_len));
	} else {
		memset(nfc->buf, 0xFF, FIFO_DEPTH);
		data = nfc->buf;
		for (i = 0; i < FIFO_REP(data_len) / FIFO_DEPTH_32; i++)
			iowrite32_rep(nfc->regs + NDDB, data, FIFO_DEPTH_32);
	}

	/* Pad user data with 2 bytes when using BCH (30B) */
	oob += chunk * lt->spare_bytes;
	iowrite32_rep(nfc->regs + NDDB, oob, FIFO_REP(oob_len));
}

static int marvell_nfc_hw_ecc_bch_write_page(struct mtd_info *mtd,
					     struct nand_chip *chip,
					     const u8 *buf,
					     int oob_required, int page)
{
	struct marvell_hw_ecc_layout *lt = &to_marvell_nand(chip)->layout;
	int nchunks = lt->full_chunk_cnt + lt->last_chunk_cnt;
	int chunk, status;

	marvell_nfc_hw_ecc_enable(chip);

	for (chunk = 0; chunk < nchunks; chunk++) {
		marvell_nfc_hw_ecc_bch_write_chunk(chip, chunk, buf,
						   chip->oob_poi,
						   oob_required, page);
		/*
		 * Waiting only for CMDD or PAGED is not enough, ECC are
		 * partially written. No flag is set once the operation is
		 * really finished but the ND_RUN bit is cleared, so wait for it
		 * before stepping into the next command.
		 */
		marvell_nfc_wait_ndrun(chip);
	}

	/*
	 * Wait the command to be done and handle the RBn pin because standard
	 * accessors are unused.
	 */
	status = marvell_nfc_waitfunc(mtd, chip);

	marvell_nfc_hw_ecc_disable(chip);

	if (status & NAND_STATUS_FAIL)
		return -EIO;

	return 0;
}

static int marvell_nfc_hw_ecc_write_oob(struct mtd_info *mtd,
					struct nand_chip *chip, int page)
{
	return chip->ecc.write_page(mtd, chip, NULL, true, page);
}

/* Raw writes with HW ECC */
static int marvell_nfc_hw_ecc_write_page_raw(struct mtd_info *mtd,
					     struct nand_chip *chip,
					     const u8 *buf,
					     int oob_required, int page)
{
	struct marvell_hw_ecc_layout *lt = &to_marvell_nand(chip)->layout;
	int nchunks = lt->full_chunk_cnt + lt->last_chunk_cnt;
	int oob_size = lt->spare_bytes + lt->ecc_bytes;
	int last_oob_size = lt->last_spare_bytes + lt->last_ecc_bytes;
	int chunk;

	nand_prog_page_begin_op(chip, page, 0, NULL, 0);

	for (chunk = 0; chunk < nchunks; chunk++) {
		/*
		 * OOB are not 8-bytes aligned anyway so change the column
		 * at each cycle
		 */
		nand_change_write_column_op(chip, chunk * (lt->data_bytes +
							   oob_size),
					    NULL, 0);

		if (chunk < lt->full_chunk_cnt)
			nand_write_data_op(chip, buf + (chunk * lt->data_bytes),
					   lt->data_bytes, false);
		else
			nand_write_data_op(chip, buf + (chunk * lt->data_bytes),
					   lt->last_data_bytes, false);

		if (!oob_required)
			continue;

		if (chunk < lt->full_chunk_cnt)
			nand_write_data_op(chip, chip->oob_poi +
					   (chunk * (oob_size + 2)),
					   oob_size, false);
		else
			nand_write_data_op(chip, chip->oob_poi +
					   (chunk * (oob_size + 2)),
					   last_oob_size, false);
	}

	return nand_prog_page_end_op(chip);
}

static int marvell_nfc_hw_ecc_write_oob_raw(struct mtd_info *mtd,
					    struct nand_chip *chip, int page)
{
	struct marvell_hw_ecc_layout *lt = &to_marvell_nand(chip)->layout;
	int chunk_size = lt->data_bytes + lt->spare_bytes + lt->ecc_bytes;
	int nchunks = lt->full_chunk_cnt + lt->last_chunk_cnt;
	int oob_size = lt->spare_bytes + lt->ecc_bytes;
	int last_oob_size = lt->last_spare_bytes + lt->last_ecc_bytes;
	int chunk;

	nand_prog_page_begin_op(chip, page, 0, NULL, 0);

	for (chunk = 0; chunk < nchunks; chunk++) {
		nand_change_write_column_op(chip, lt->data_bytes +
					    (chunk * chunk_size), NULL, 0);

		if (chunk < lt->full_chunk_cnt)
			nand_write_data_op(chip, chip->oob_poi +
					   (chunk * (oob_size + 2)),
					   oob_size, false);
		else
			nand_write_data_op(chip, chip->oob_poi +
					   (chunk * (oob_size + 2)),
					   last_oob_size, false);
	}

	return nand_prog_page_end_op(chip);
}

/*
 * HW ECC layouts, identical to old pxa3xx_nand driver,
 * to be fully backward compatible
 */
static int marvell_nand_ooblayout_ecc(struct mtd_info *mtd, int section,
				      struct mtd_oob_region *oobregion)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct marvell_hw_ecc_layout *lt = &to_marvell_nand(chip)->layout;
	int nchunks = lt->full_chunk_cnt;

	if (section >= nchunks)
		return -ERANGE;

	oobregion->offset = ((lt->spare_bytes + lt->ecc_bytes) * section) +
		lt->spare_bytes;
	oobregion->length = lt->ecc_bytes;

	return 0;
}

static int marvell_nand_ooblayout_free(struct mtd_info *mtd, int section,
				       struct mtd_oob_region *oobregion)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct marvell_hw_ecc_layout *lt = &to_marvell_nand(chip)->layout;
	int nchunks = lt->full_chunk_cnt;

	if (section >= nchunks)
		return -ERANGE;

	if (!lt->spare_bytes)
		return 0;

	oobregion->offset = section * (lt->spare_bytes + lt->ecc_bytes);
	oobregion->length = lt->spare_bytes;
	if (!section) {
		/*
		 * Bootrom looks in bytes 0 & 5 for bad blocks for the
		 * 4KB page / 4bit BCH combination.
		 */
		if (mtd->writesize == 4096 && lt->data_bytes == 2048) {
			oobregion->offset += 6;
			oobregion->length -= 6;
		} else {
			oobregion->offset += 2;
			oobregion->length -= 2;
		}
	}

	return 0;
}

static const struct mtd_ooblayout_ops marvell_nand_ooblayout_ops = {
	.ecc = marvell_nand_ooblayout_ecc,
	.free = marvell_nand_ooblayout_free,
};

static int marvell_nand_hw_ecc_ctrl_init(struct mtd_info *mtd,
					 struct nand_ecc_ctrl *ecc,
					 struct device_node *np)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct marvell_nfc *nfc = to_marvell_nfc(chip->controller);
	struct marvell_hw_ecc_layout *lt = &to_marvell_nand(chip)->layout;

	if (mtd->writesize == 512 && ecc->size == 512 && ecc->strength == 1) {
		chip->ecc.algo = NAND_ECC_HAMMING;
		lt->full_chunk_cnt = 1;
		lt->data_bytes = 512;
		lt->ecc_bytes = 6;
	} else if (mtd->writesize == 2048 && ecc->size == 512 &&
		   ecc->strength == 1) {
		chip->ecc.algo = NAND_ECC_HAMMING;
		lt->full_chunk_cnt = 1;
		lt->data_bytes = 2048;
		lt->spare_bytes = 40;
		lt->ecc_bytes = 24;
	} else if (mtd->writesize == 2048 && ecc->size == 512 &&
		   ecc->strength == 4) {
		chip->ecc.algo = NAND_ECC_BCH;
		lt->full_chunk_cnt = 1;
		lt->data_bytes = 2048;
		lt->spare_bytes = 32;
		lt->ecc_bytes = 30;
	} else if (mtd->writesize == 4096 && ecc->size == 512 &&
		   ecc->strength == 4) {
		chip->ecc.algo = NAND_ECC_BCH;
		lt->full_chunk_cnt = 2;
		lt->data_bytes = 2048;
		lt->spare_bytes = 32;
		lt->ecc_bytes = 30;
	} else if (mtd->writesize == 4096 && ecc->size == 512 &&
		   ecc->strength == 8) {
		chip->ecc.algo = NAND_ECC_BCH;
		lt->full_chunk_cnt = 4;
		lt->data_bytes = 1024;
		lt->spare_bytes = 0;
		lt->ecc_bytes = 30;
		lt->last_chunk_cnt = 1;
		lt->last_data_bytes = 0;
		lt->last_spare_bytes = 64;
		lt->last_ecc_bytes = 30;
	} else {
		dev_err(nfc->dev,
			"ECC strength %d at page size %d is not supported\n",
			ecc->strength, mtd->writesize);
		return -ENODEV;
	}

	mtd_set_ooblayout(mtd, &marvell_nand_ooblayout_ops);
	ecc->steps = lt->full_chunk_cnt + lt->last_chunk_cnt;
	ecc->size = lt->data_bytes;

	switch (chip->ecc.algo) {
	case NAND_ECC_HAMMING:
		ecc->read_page = marvell_nfc_hw_ecc_hmg_read_page;
		ecc->write_page = marvell_nfc_hw_ecc_hmg_write_page;
		ecc->strength = 1;
		break;
	case NAND_ECC_BCH:
		ecc->read_page = marvell_nfc_hw_ecc_bch_read_page;
		ecc->write_page = marvell_nfc_hw_ecc_bch_write_page;
		ecc->strength = 16;
		break;
	default:
		dev_err(nfc->dev, "NAND hardware ECC algorithm missing\n");
		return -ENOTSUPP;
	}

	ecc->read_oob = marvell_nfc_hw_ecc_read_oob;
	ecc->write_oob = marvell_nfc_hw_ecc_write_oob;

	ecc->read_page_raw = marvell_nfc_hw_ecc_read_page_raw;
	ecc->write_page_raw = marvell_nfc_hw_ecc_write_page_raw;
	ecc->read_oob_raw = marvell_nfc_hw_ecc_read_oob_raw;
	ecc->write_oob_raw = marvell_nfc_hw_ecc_write_oob_raw;

	return 0;
}

static int marvell_nand_ecc_init(struct mtd_info *mtd,
				 struct nand_ecc_ctrl *ecc,
				 struct device_node *np)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	int ret;

	if (!ecc->size)
		ecc->size = chip->ecc_step_ds;

	if (!ecc->strength)
		ecc->strength = chip->ecc_strength_ds;

	if (!ecc->size || !ecc->strength)
		return -EINVAL;

	switch (ecc->mode) {
	case NAND_ECC_HW:
		ret = marvell_nand_hw_ecc_ctrl_init(mtd, ecc, np);
		if (ret)
			return ret;
		break;
	case NAND_ECC_NONE:
		chip->ecc.algo = 0;
	case NAND_ECC_SOFT:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static u8 bbt_pattern[] = {'M', 'V', 'B', 'b', 't', '0' };
static u8 bbt_mirror_pattern[] = {'1', 't', 'b', 'B', 'V', 'M' };

static struct nand_bbt_descr bbt_main_descr = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE |
		   NAND_BBT_2BIT | NAND_BBT_VERSION,
	.offs =	8,
	.len = 6,
	.veroffs = 14,
	.maxblocks = 8,	/* Last 8 blocks in each chip */
	.pattern = bbt_pattern
};

static struct nand_bbt_descr bbt_mirror_descr = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE |
		   NAND_BBT_2BIT | NAND_BBT_VERSION,
	.offs =	8,
	.len = 6,
	.veroffs = 14,
	.maxblocks = 8,	/* Last 8 blocks in each chip */
	.pattern = bbt_mirror_pattern
};

static int marvell_nfc_setup_data_interface(struct mtd_info *mtd, int chipnr,
					    const struct nand_data_interface
					    *conf)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct marvell_nand_chip *marvell_nand = to_marvell_nand(chip);
	struct marvell_nfc *nfc = to_marvell_nfc(chip->controller);
	unsigned int period_ns = 1000000000 / clk_get_rate(nfc->ecc_clk) * 2;
	const struct nand_sdr_timings *sdr;
	struct marvell_nfc_timings nfc_tmg;
	int read_delay;

	sdr = nand_get_sdr_timings(conf);
	if (IS_ERR(sdr))
		return PTR_ERR(sdr);

	/*
	 * SDR timings are given in pico-seconds while NFC timings must be
	 * expressed in NAND controller clock cycles, which is half of the
	 * frequency of the accessible ECC clock retrieved by clk_get_rate().
	 * This is not written anywhere in the datasheet but was observed
	 * with an oscilloscope.
	 *
	 * NFC datasheet gives equations from which thoses calculations
	 * are derived, they tend to be slightly more restrictives than the
	 * given core timings and may improve the overall speed.
	 */

	nfc_tmg.tRP = TO_CYCLES(DIV_ROUND_UP(sdr->tRC_min, 2), period_ns) - 1;
	nfc_tmg.tRH = nfc_tmg.tRP;
	nfc_tmg.tWP = TO_CYCLES(DIV_ROUND_UP(sdr->tWC_min, 2), period_ns) - 1;
	nfc_tmg.tWH = nfc_tmg.tWP;
	nfc_tmg.tCS = TO_CYCLES(sdr->tCS_min, period_ns);
	nfc_tmg.tCH = TO_CYCLES(sdr->tCH_min, period_ns) - 1;
	nfc_tmg.tADL = TO_CYCLES(sdr->tADL_min, period_ns);
	/*
	 * Read delay is the time of propagation from SoC pins to NFC internal
	 * logic. With non-EDO timings, this is MIN_RD_DEL_CNT clock cycles. In
	 * EDO mode, an additional delay of tRH must be taken into account so
	 * the data is sampled on the falling edge instead of the rising edge.
	 */
	read_delay = sdr->tRC_min >= 30000 ?
		MIN_RD_DEL_CNT : MIN_RD_DEL_CNT + nfc_tmg.tRH;

	nfc_tmg.tAR = TO_CYCLES(sdr->tAR_min, period_ns);
	nfc_tmg.tWHR = TO_CYCLES(sdr->tWHR_min, period_ns) - 2;
	nfc_tmg.tRHW = TO_CYCLES(sdr->tRHW_min, period_ns);

	/* Use WAIT_MODE (wait for RB line) instead of only relying on delays */
	nfc_tmg.tR = TO_CYCLES(sdr->tWB_max, period_ns);

	if (chipnr < 0)
		return 0;

	marvell_nand->ndtr0 =
		NDTR0_TRP(nfc_tmg.tRP) |
		NDTR0_TRH(nfc_tmg.tRH) |
		NDTR0_ETRP(nfc_tmg.tRP) |
		NDTR0_TWP(nfc_tmg.tWP) |
		NDTR0_TWH(nfc_tmg.tWH) |
		NDTR0_TCS(nfc_tmg.tCS) |
		NDTR0_TCH(nfc_tmg.tCH) |
		NDTR0_RD_CNT_DEL(read_delay) |
		NDTR0_SELCNTR |
		NDTR0_TADL(nfc_tmg.tADL);

	marvell_nand->ndtr1 =
		NDTR1_TAR(nfc_tmg.tAR) |
		NDTR1_TWHR(nfc_tmg.tWHR) |
		NDTR1_TRHW(nfc_tmg.tRHW) |
		NDTR1_WAIT_MODE |
		NDTR1_TR(nfc_tmg.tR);

	writel_relaxed(marvell_nand->ndtr0, nfc->regs + NDTR0);
	writel_relaxed(marvell_nand->ndtr1, nfc->regs + NDTR1);

	return 0;
}

static int marvell_nand_chip_init(struct device *dev, struct marvell_nfc *nfc,
				  struct device_node *np)
{
	struct marvell_nand_chip *marvell_nand;
	struct mtd_info *mtd;
	struct nand_chip *chip;
	int nsels, ret, i;
	u32 cs, rb;

	/*
	 * The legacy "num-cs" property indicates the number of CS on the only
	 * chip connected to the controller (legacy bindings does not support
	 * more than one chip). CS are only incremented one by one while the RB
	 * pin is always the #0.
	 *
	 * When not using legacy bindings, a couple of "reg" and "marvell,rb"
	 * properties must be filled. For each chip, expressed as a subnode,
	 * "reg" points to the CS lines and "marvell,rb" to the RB line.
	 */
	if (nfc->caps->legacy_of_bindings) {
		if (!of_get_property(np, "num-cs", &nsels)) {
			dev_err(dev, "missing num-cs property\n");
			return -EINVAL;
		}
	} else {
		if (!of_get_property(np, "reg", &nsels)) {
			dev_err(dev, "missing reg property\n");
			return -EINVAL;
		}
	}

	nsels /= sizeof(u32);
	if (!nsels) {
		dev_err(dev, "invalid reg property size\n");
		return -EINVAL;
	}

	/* Alloc the nand chip structure */
	marvell_nand = devm_kzalloc(dev, sizeof(*marvell_nand) +
				    (nsels *
				     sizeof(struct marvell_nand_chip_sel)),
				    GFP_KERNEL);
	if (!marvell_nand) {
		dev_err(dev, "could not allocate chip structure\n");
		return -ENOMEM;
	}

	marvell_nand->nsels = nsels;
	marvell_nand->selected = -1;

	for (i = 0; i < nsels; i++) {
		if (nfc->caps->legacy_of_bindings) {
			/*
			 * Legacy bindings use the CS lines in natural
			 * order (0, 1, ...)
			 */
			cs = i;
		} else {
			/* Retrieve CS id */
			ret = of_property_read_u32_index(np, "reg", i, &cs);
			if (ret) {
				dev_err(dev, "could not retrieve reg property: %d\n",
					ret);
				return ret;
			}
		}

		if (cs >= nfc->caps->max_cs_nb) {
			dev_err(dev, "invalid reg value: %u (max CS = %d)\n",
				cs, nfc->caps->max_cs_nb);
			return -EINVAL;
		}

		if (test_and_set_bit(cs, &nfc->assigned_cs)) {
			dev_err(dev, "CS %d already assigned\n", cs);
			return -EINVAL;
		}

		/*
		 * The cs variable represents the chip select id, which must be
		 * converted in bit fields for NDCB0 and NDCB2 to select the
		 * right chip.
		 */
		switch (cs) {
		case 0:
			marvell_nand->sels[i].ndcb0_csel = 0;
			marvell_nand->sels[i].ndcb2_addr5 = 0;
			break;
		case 1:
			marvell_nand->sels[i].ndcb0_csel = NDCB0_CSEL;
			marvell_nand->sels[i].ndcb2_addr5 = 0;
			break;
		case 2:
			marvell_nand->sels[i].ndcb0_csel = 0;
			marvell_nand->sels[i].ndcb2_addr5 = NDCB2_ADDR5_CS;
			break;
		case 3:
			marvell_nand->sels[i].ndcb0_csel = NDCB0_CSEL;
			marvell_nand->sels[i].ndcb2_addr5 = NDCB2_ADDR5_CS;
			break;
		default:
			return -EINVAL;
		}

		/* Retrieve RB id */
		if (nfc->caps->legacy_of_bindings) {
			/* Legacy bindings always use RB #0 */
			rb = 0;
		} else {
			ret = of_property_read_u32_index(np, "marvell,rb", i,
							 &rb);
			if (ret) {
				dev_err(dev,
					"could not retrieve RB property: %d\n",
					ret);
				return ret;
			}
		}

		if (rb >= nfc->caps->max_rb_nb) {
			dev_err(dev, "invalid reg value: %u (max RB = %d)\n",
				rb, nfc->caps->max_rb_nb);
			return -EINVAL;
		}

		marvell_nand->sels[i].rb = rb;
	}

	chip = &marvell_nand->chip;
	chip->controller = &nfc->controller;
	nand_set_flash_node(chip, np);

	chip->exec_op = marvell_nfc_exec_op;
	chip->select_chip = marvell_nfc_select_chip;
	chip->cmd_ctrl = marvell_nfc_cmd_ctrl;
	chip->dev_ready = marvell_nfc_dev_ready;
	chip->waitfunc = marvell_nfc_waitfunc;
	chip->read_byte = marvell_nfc_read_byte;
	chip->read_word = marvell_nfc_read_word;
	chip->read_buf = marvell_nfc_read_buf;
	chip->write_buf = marvell_nfc_write_buf;
	if (!of_get_property(np, "marvell,nand-keep-config", NULL))
		chip->setup_data_interface = marvell_nfc_setup_data_interface;

	mtd = nand_to_mtd(chip);
	mtd->dev.parent = dev;

	/*
	 * Default to HW ECC engine mode. If the nand-ecc-mode property is given
	 * in the DT node, this entry will be overwritten in nand_scan_ident().
	 */
	chip->ecc.mode = NAND_ECC_HW;

	ret = nand_scan_ident(mtd, marvell_nand->nsels, NULL);
	if (ret) {
		dev_err(dev, "could not identify the nand chip\n");
		return ret;
	}

	if (chip->bbt_options & NAND_BBT_USE_FLASH) {
		/*
		 * We'll use a bad block table stored in-flash and don't
		 * allow writing the bad block marker to the flash.
		 */
		chip->bbt_options |= NAND_BBT_NO_OOB_BBM;
		chip->bbt_td = &bbt_main_descr;
		chip->bbt_md = &bbt_mirror_descr;
	}

	/*
	 * With RA_START bit set in NDCR, columns takes two address cycles. This
	 * means addressing a chip with more than 256 pages needs a fifth
	 * address cycle. Addressing a chip using CS 2 or 3 also needs this
	 * additional cycle.
	 */
	marvell_nand->addr_cyc = 4;
	for (i = 0; i < nsels; i++)
		if (marvell_nand->sels[i].ndcb2_addr5)
			marvell_nand->addr_cyc = 5;

	if ((mtd->writesize <= 512 && (chip->chipsize > (32 << 20))) ||
	    (mtd->writesize > 512 && (chip->chipsize > (128 << 20))))
		marvell_nand->addr_cyc = 5;

	ret = marvell_nand_ecc_init(mtd, &chip->ecc, np);
	if (ret) {
		dev_err(dev, "ECC init failed: %d\n", ret);
		return ret;
	}

	if (chip->ecc.mode == NAND_ECC_HW) {
		/*
		 * Subpage write not available with hardware ECC, prohibit also
		 * subpage read as in userspace subpage acces would still be
		 * allowed and subpage write, if used, would lead to numerous
		 * uncorrectable ECC errors.
		 */
		chip->options |= NAND_NO_SUBPAGE_WRITE;
	}

	ret = nand_scan_tail(mtd);
	if (ret) {
		dev_err(dev, "nand_scan_tail failed: %d\n", ret);
		return ret;
	}

	ret = mtd_device_register(mtd, NULL, 0);
	if (ret) {
		dev_err(dev, "failed to register mtd device: %d\n", ret);
		nand_release(mtd);
		return ret;
	}

	list_add_tail(&marvell_nand->node, &nfc->chips);

	return 0;
}

static int marvell_nand_chips_init(struct device *dev, struct marvell_nfc *nfc)
{
	struct device_node *np = dev->of_node;
	struct device_node *nand_np;
	int nchips = of_get_child_count(np);
	int max_cs = nfc->caps->max_cs_nb;
	int ret;

	if (nchips > max_cs) {
		dev_err(dev, "too many NAND chips: %d (max = %d CS)\n", nchips,
			max_cs);
		return -EINVAL;
	}

	/*
	 * Legacy bindings do not use child nodes to exhibit NAND chip
	 * properties and layout. Instead, NAND properties are mixed with the
	 * controller's and a single subnode presents the memory layout.
	 */
	if (nfc->caps->legacy_of_bindings) {
		ret = marvell_nand_chip_init(dev, nfc, np);
		return ret;
	}

	for_each_child_of_node(np, nand_np) {
		ret = marvell_nand_chip_init(dev, nfc, nand_np);
		if (ret) {
			of_node_put(nand_np);
			return ret;
		}
	}

	return 0;
}

static void marvell_nand_chips_cleanup(struct marvell_nfc *nfc)
{
	struct marvell_nand_chip *entry, *temp;

	list_for_each_entry_safe(entry, temp, &nfc->chips, node) {
		nand_release(nand_to_mtd(&entry->chip));
		list_del(&entry->node);
	}
}

static void marvell_nfc_init(struct marvell_nfc *nfc)
{
	struct device_node *np = nfc->dev->of_node;
	u32 enable_arbiter = 0;

	/* For PXA only, but other SoCs have this bit marked reserved */
	if (of_get_property(np, "marvell,nand-enable-arbiter", NULL))
		enable_arbiter = NDCR_ND_ARB_EN;

	/*
	 * ECC operations and interruptions are only enabled when specifically
	 * needed. ECC shall not be activated in the early stages (fails probe)
	 */
	writel_relaxed(NDCR_RA_START | NDCR_ALL_INT | enable_arbiter,
		       nfc->regs + NDCR);
	writel_relaxed(0xFFFFFFFF, nfc->regs + NDSR);
	writel_relaxed(0, nfc->regs + NDECCCTRL);
}

static int marvell_nfc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *r;
	struct marvell_nfc *nfc;
	int ret;
	int irq;

	nfc = devm_kzalloc(&pdev->dev, sizeof(struct marvell_nfc),
			   GFP_KERNEL);
	if (!nfc)
		return -ENOMEM;

	nfc->dev = dev;
	nand_hw_control_init(&nfc->controller);
	INIT_LIST_HEAD(&nfc->chips);

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	nfc->regs = devm_ioremap_resource(dev, r);
	if (IS_ERR(nfc->regs))
		return PTR_ERR(nfc->regs);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "failed to retrieve irq\n");
		return irq;
	}

	nfc->ecc_clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(nfc->ecc_clk))
		return PTR_ERR(nfc->ecc_clk);

	ret = clk_prepare_enable(nfc->ecc_clk);
	if (ret)
		return ret;

	marvell_nfc_disable_int(nfc, NDCR_ALL_INT);
	marvell_nfc_clear_int(nfc, NDCR_ALL_INT);
	ret = devm_request_irq(dev, irq, marvell_nfc_isr,
			       0, "marvell-nfc", nfc);
	if (ret)
		goto out_clk_unprepare;

	/* Get NAND controller capabilities */
	if (pdev->id_entry)
		nfc->caps = (void *)pdev->id_entry->driver_data;
	else
		nfc->caps = of_device_get_match_data(&pdev->dev);

	if (!nfc->caps) {
		dev_err(dev, "Could not retrieve NFC caps\n");
		ret = -EINVAL;
		goto out_clk_unprepare;
	}

	/* Init the driver and then probe the chips */
	marvell_nfc_init(nfc);

	platform_set_drvdata(pdev, nfc);

	ret = marvell_nand_chips_init(dev, nfc);
	if (ret)
		goto out_clk_unprepare;

	return 0;

out_clk_unprepare:
	clk_disable_unprepare(nfc->ecc_clk);

	return ret;
}

static int marvell_nfc_remove(struct platform_device *pdev)
{
	struct marvell_nfc *nfc = platform_get_drvdata(pdev);

	marvell_nand_chips_cleanup(nfc);

	clk_disable_unprepare(nfc->ecc_clk);

	return 0;
}

static const struct marvell_nfc_caps marvell_armada370_nfc_caps = {
	.variant = MARVELL_NFC_VARIANT_ARMADA370,
	.max_cs_nb = 4,
	.max_rb_nb = 2,
};

static const struct marvell_nfc_caps marvell_pxa3xx_nfc_caps = {
	.variant = MARVELL_NFC_VARIANT_PXA3XX,
	.max_cs_nb = 2,
	.max_rb_nb = 1,
};

static const struct marvell_nfc_caps marvell_armada370_nfc_legacy_caps = {
	.variant = MARVELL_NFC_VARIANT_ARMADA370,
	.max_cs_nb = 4,
	.max_rb_nb = 2,
	.legacy_of_bindings = true,
};

static const struct marvell_nfc_caps marvell_pxa3xx_nfc_legacy_caps = {
	.variant = MARVELL_NFC_VARIANT_PXA3XX,
	.max_cs_nb = 2,
	.max_rb_nb = 1,
	.legacy_of_bindings = true,
};

static const struct of_device_id marvell_nfc_of_ids[] = {
	{
		.compatible = "marvell,armada370-nand-controller",
		.data = &marvell_armada370_nfc_caps,
	},
	{
		.compatible = "marvell,pxa3xx-nand-controller",
		.data = &marvell_pxa3xx_nfc_caps,
	},
	/* Support for old/deprecated bindings: */
	{
		.compatible = "marvell,armada370-nand",
		.data = &marvell_armada370_nfc_legacy_caps,
	},
	{
		.compatible = "marvell,pxa3xx-nand",
		.data = &marvell_pxa3xx_nfc_legacy_caps,
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, marvell_nand_match);

static struct platform_driver marvell_nfc_driver = {
	.driver	= {
		.name		= "marvell-nfc",
		.of_match_table = marvell_nfc_of_ids,
	},
	.probe	= marvell_nfc_probe,
	.remove	= marvell_nfc_remove,
};
module_platform_driver(marvell_nfc_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Marvell NAND controller driver");
