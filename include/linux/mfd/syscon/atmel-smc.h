/*
 * Atmel SMC (Static Memory Controller) register offsets and bit definitions.
 *
 * Copyright (C) 2014 Atmel
 * Copyright (C) 2014 Free Electrons
 *
 * Author: Boris Brezillon <boris.brezillon@free-electrons.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _LINUX_MFD_SYSCON_ATMEL_SMC_H_
#define _LINUX_MFD_SYSCON_ATMEL_SMC_H_

#include <linux/kernel.h>
#include <linux/regmap.h>

#define ATMEL_SMC_CSR(cs)			((cs) * 0x4)
#define ATMEL_SMC_CSR_NWS_MASK			GENMASK(6, 0)
#define ATMEL_SMC_CSR_TDF_MASK			GENMASK(11, 8)
#define ATMEL_SMC_CSR_BAT_MASK			BIT(12)
#define ATMEL_SMC_CSR_BAT_SELECT		(1 << 12)
#define ATMEL_SMC_CSR_BAT_WRITE			(0 << 12)
#define ATMEL_SMC_CSR_DBW_MASK			GENMASK(14, 13)
#define ATMEL_SMC_CSR_DBW_16			(1 << 13)
#define ATMEL_SMC_CSR_DBW_8			(2 << 13)
#define ATMEL_SMC_CSR_DRP_EARLY			BIT(15)
#define ATMEL_SMC_CSR_ACSS_MASK			GENMASK(17, 16)
#define ATMEL_SMC_CSR_NRW_SETUP_MASK		GENMASK(26, 24)
#define ATMEL_SMC_CSR_NRW_HOLD_MASK		GENMASK(30, 28)

#define ATMEL_SMC_SETUP(cs)			(((cs) * 0x10))
#define ATMEL_HSMC_SETUP(cs)			(0x600 + ((cs) * 0x14))
#define ATMEL_SMC_PULSE(cs)			(((cs) * 0x10) + 0x4)
#define ATMEL_HSMC_PULSE(cs)			(0x600 + ((cs) * 0x14) + 0x4)
#define ATMEL_SMC_CYCLE(cs)			(((cs) * 0x10) + 0x8)
#define ATMEL_HSMC_CYCLE(cs)			(0x600 + ((cs) * 0x14) + 0x8)
#define ATMEL_SMC_NWE_SHIFT			0
#define ATMEL_SMC_NCS_WR_SHIFT			8
#define ATMEL_SMC_NRD_SHIFT			16
#define ATMEL_SMC_NCS_RD_SHIFT			24

#define ATMEL_SMC_MODE(cs)			(((cs) * 0x10) + 0xc)
#define ATMEL_HSMC_MODE(cs)			(0x600 + ((cs) * 0x14) + 0x10)
#define ATMEL_SMC_MODE_READMODE_MASK		BIT(0)
#define ATMEL_SMC_MODE_READMODE_NCS		(0 << 0)
#define ATMEL_SMC_MODE_READMODE_NRD		(1 << 0)
#define ATMEL_SMC_MODE_WRITEMODE_MASK		BIT(1)
#define ATMEL_SMC_MODE_WRITEMODE_NCS		(0 << 1)
#define ATMEL_SMC_MODE_WRITEMODE_NWE		(1 << 1)
#define ATMEL_SMC_MODE_EXNWMODE_MASK		GENMASK(5, 4)
#define ATMEL_SMC_MODE_EXNWMODE_DISABLE		(0 << 4)
#define ATMEL_SMC_MODE_EXNWMODE_FROZEN		(2 << 4)
#define ATMEL_SMC_MODE_EXNWMODE_READY		(3 << 4)
#define ATMEL_SMC_MODE_BAT_MASK			BIT(8)
#define ATMEL_SMC_MODE_BAT_SELECT		(0 << 8)
#define ATMEL_SMC_MODE_BAT_WRITE		(1 << 8)
#define ATMEL_SMC_MODE_DBW_MASK			GENMASK(13, 12)
#define ATMEL_SMC_MODE_DBW_8			(0 << 12)
#define ATMEL_SMC_MODE_DBW_16			(1 << 12)
#define ATMEL_SMC_MODE_DBW_32			(2 << 12)
#define ATMEL_SMC_MODE_TDF_MASK			GENMASK(19, 16)
#define ATMEL_SMC_MODE_TDF(x)			(((x) - 1) << 16)
#define ATMEL_SMC_MODE_TDF_MAX			16
#define ATMEL_SMC_MODE_TDFMODE_OPTIMIZED	BIT(20)
#define ATMEL_SMC_MODE_PMEN			BIT(24)
#define ATMEL_SMC_MODE_PS_MASK			GENMASK(29, 28)
#define ATMEL_SMC_MODE_PS_4			(0 << 28)
#define ATMEL_SMC_MODE_PS_8			(1 << 28)
#define ATMEL_SMC_MODE_PS_16			(2 << 28)
#define ATMEL_SMC_MODE_PS_32			(3 << 28)

#define ATMEL_HSMC_TIMINGS(cs)			(0x600 + ((cs) * 0x14) + 0xc)
#define ATMEL_HSMC_TIMINGS_OCMS			BIT(12)
#define ATMEL_HSMC_TIMINGS_RBNSEL(x)		((x) << 28)
#define ATMEL_HSMC_TIMINGS_NFSEL		BIT(31)
#define ATMEL_HSMC_TIMINGS_TCLR_SHIFT		0
#define ATMEL_HSMC_TIMINGS_TADL_SHIFT		4
#define ATMEL_HSMC_TIMINGS_TAR_SHIFT		8
#define ATMEL_HSMC_TIMINGS_TRR_SHIFT		16
#define ATMEL_HSMC_TIMINGS_TWB_SHIFT		24

/**
 * struct atmel_smc_cs_conf - SMC CS config as described in the datasheet.
 * @setup: NCS/NWE/NRD setup timings (not applicable to at91rm9200)
 * @pulse: NCS/NWE/NRD pulse timings (not applicable to at91rm9200)
 * @cycle: NWE/NRD cycle timings (not applicable to at91rm9200)
 * @timings: advanced NAND related timings (only applicable to HSMC)
 * @mode: all kind of config parameters (see the fields definition above).
 *	  The mode fields are different on at91rm9200
 * @csr: same as @mode but for the at91rm9200 SMC
 */
struct atmel_smc_cs_conf {
	u32 setup;
	u32 pulse;
	u32 cycle;
	u32 timings;
	union {
		u32 mode;
		u32 csr;
	};
};

/**
 * atmel_smc_cs_conf_init - initialize a SMC CS conf
 * @conf: the SMC CS conf to initialize
 *
 * Set all fields to 0 so that one can start defining a new config.
 */
static inline void atmel_smc_cs_conf_init(struct atmel_smc_cs_conf *conf)
{
	memset(conf, 0, sizeof(*conf));
}

/**
 * atmel_smc_cs_encode_ncycles - encode a number of MCK clk cycles in the
 *				 format expected by the SMC engine
 * @ncycles: number of MCK clk cycles
 * @msbpos: position of the MSB part of the timing field
 * @msbwidth: width of the MSB part of the timing field
 * @msbfactor: factor applied to the MSB
 * @encodedval: param used to store the encoding result
 *
 * This function encodes the @ncycles value as described in the datasheet
 * (section "SMC Setup/Pulse/Cycle/Timings Register"). This is a generic
 * helper which called with different parameter depending on the encoding
 * scheme.
 *
 * If the @ncycles value is too big to be encoded, -ERANGE is returned and
 * the encodedval is contains the maximum val. Otherwise, 0 is returned.
 */
static inline int atmel_smc_cs_encode_ncycles(unsigned int ncycles,
					      unsigned int msbpos,
					      unsigned int msbwidth,
					      unsigned int msbfactor,
					      unsigned int *encodedval)
{
	unsigned int lsbmask = GENMASK(msbpos - 1, 0);
	unsigned int msbmask = GENMASK(msbwidth - 1, 0);
	unsigned int msb, lsb;
	int ret = 0;

	msb = ncycles / msbfactor;
	lsb = ncycles % msbfactor;

	if (lsb > lsbmask) {
		lsb = 0;
		msb++;
	}

	/*
	 * Let's just put the maximum we can if the requested setting does
	 * not fit in the register field.
	 * We still return -ERANGE in case the caller cares.
	 */
	if (msb > msbmask) {
		msb = msbmask;
		lsb = lsbmask;
		ret = -ERANGE;
	}

	*encodedval = (msb << msbpos) | lsb;

	return ret;
}

/**
 * atmel_smc_cs_conf_set_timing - set the SMC CS conf Txx parameter to a
 *				  specific value
 * @conf: SMC CS conf descriptor
 * @shift: the position of the Txx field in the TIMINGS register
 * @ncycles: value (expressed in MCK clk cycles) to assign to this Txx
 *	     parameter
 *
 * This function encodes the @ncycles value as described in the datasheet
 * (section "SMC Timings Register"), and then stores the result in the
 * @conf->timings field at @shift position.
 *
 * Returns -EINVAL if shift is invalid, -ERANGE if ncycles does not fit in
 * the field, and 0 otherwise.
 */
static inline int atmel_smc_cs_conf_set_timing(struct atmel_smc_cs_conf *conf,
					       unsigned int shift,
					       unsigned int ncycles)
{
	unsigned int val;
	int ret;

	if (shift != ATMEL_HSMC_TIMINGS_TCLR_SHIFT &&
	    shift != ATMEL_HSMC_TIMINGS_TADL_SHIFT &&
	    shift != ATMEL_HSMC_TIMINGS_TAR_SHIFT &&
	    shift != ATMEL_HSMC_TIMINGS_TRR_SHIFT &&
	    shift != ATMEL_HSMC_TIMINGS_TWB_SHIFT)
		return -EINVAL;

	/*
	 * The formula described in atmel datasheets (section "HSMC Timings
	 * Register"):
	 *
	 * ncycles = (Txx[3] * 64) + Txx[2:0]
	 */
	ret = atmel_smc_cs_encode_ncycles(ncycles, 3, 1, 64, &val);
	conf->timings &= ~GENMASK(shift + 3, shift);
	conf->timings |= val << shift;

	return ret;
}

/**
 * atmel_smc_cs_conf_set_setup - set the SMC CS conf xx_SETUP parameter to a
 *				 specific value
 * @conf: SMC CS conf descriptor
 * @shift: the position of the xx_SETUP field in the SETUP register
 * @ncycles: value (expressed in MCK clk cycles) to assign to this xx_SETUP
 *	     parameter
 *
 * This function encodes the @ncycles value as described in the datasheet
 * (section "SMC Setup Register"), and then stores the result in the
 * @conf->setup field at @shift position.
 *
 * Returns -EINVAL if @shift is invalid, -ERANGE if @ncycles does not fit in
 * the field, and 0 otherwise.
 */
static inline int atmel_smc_cs_conf_set_setup(struct atmel_smc_cs_conf *conf,
					      unsigned int shift,
					      unsigned int ncycles)
{
	unsigned int val;
	int ret;

	if (shift != ATMEL_SMC_NWE_SHIFT && shift != ATMEL_SMC_NCS_WR_SHIFT &&
	    shift != ATMEL_SMC_NRD_SHIFT && shift != ATMEL_SMC_NCS_RD_SHIFT)
		return -EINVAL;

	/*
	 * The formula described in atmel datasheets (section "SMC Setup
	 * Register"):
	 *
	 * ncycles = (128 * xx_SETUP[5]) + xx_SETUP[4:0]
	 */
	ret = atmel_smc_cs_encode_ncycles(ncycles, 5, 1, 128, &val);
	conf->setup &= ~GENMASK(shift + 7, shift);
	conf->setup |= val << shift;

	return ret;
}

/**
 * atmel_smc_cs_conf_set_pulse - set the SMC CS conf xx_PULSE parameter to a
 *				 specific value
 * @conf: SMC CS conf descriptor
 * @shift: the position of the xx_PULSE field in the PULSE register
 * @ncycles: value (expressed in MCK clk cycles) to assign to this xx_PULSE
 *	     parameter
 *
 * This function encodes the @ncycles value as described in the datasheet
 * (section "SMC Pulse Register"), and then stores the result in the
 * @conf->setup field at @shift position.
 *
 * Returns -EINVAL if @shift is invalid, -ERANGE if @ncycles does not fit in
 * the field, and 0 otherwise.
 */
static inline int atmel_smc_cs_conf_set_pulse(struct atmel_smc_cs_conf *conf,
					      unsigned int shift,
					      unsigned int ncycles)
{
	unsigned int val;
	int ret;

	if (shift != ATMEL_SMC_NWE_SHIFT && shift != ATMEL_SMC_NCS_WR_SHIFT &&
	    shift != ATMEL_SMC_NRD_SHIFT && shift != ATMEL_SMC_NCS_RD_SHIFT)
		return -EINVAL;

	/*
	 * The formula described in atmel datasheets (section "SMC Pulse
	 * Register"):
	 *
	 * ncycles = (256 * xx_PULSE[6]) + xx_PULSE[5:0]
	 */
	ret = atmel_smc_cs_encode_ncycles(ncycles, 6, 1, 256, &val);
	conf->pulse &= ~GENMASK(shift + 7, shift);
	conf->pulse |= val << shift;

	return ret;
}

/**
 * atmel_smc_cs_conf_set_cycle - set the SMC CS conf xx_CYCLE parameter to a
 *				 specific value
 * @conf: SMC CS conf descriptor
 * @shift: the position of the xx_CYCLE field in the CYCLE register
 * @ncycles: value (expressed in MCK clk cycles) to assign to this xx_CYCLE
 *	     parameter
 *
 * This function encodes the @ncycles value as described in the datasheet
 * (section "SMC Pulse Register"), and then stores the result in the
 * @conf->setup field at @shift position.
 *
 * Returns -EINVAL if @shift is invalid, -ERANGE if @ncycles does not fit in
 * the field, and 0 otherwise.
 */
static inline int atmel_smc_cs_conf_set_cycle(struct atmel_smc_cs_conf *conf,
					      unsigned int shift,
					      unsigned int ncycles)
{
	unsigned int val;
	int ret;

	if (shift != ATMEL_SMC_NWE_SHIFT && shift != ATMEL_SMC_NRD_SHIFT)
		return -EINVAL;

	/*
	 * The formula described in atmel datasheets (section "SMC Cycle
	 * Register"):
	 *
	 * ncycles = (xx_CYCLE[8:7] * 256) + xx_CYCLE[6:0]
	 */
	ret = atmel_smc_cs_encode_ncycles(ncycles, 7, 2, 256, &val);
	conf->cycle &= ~GENMASK(shift + 15, shift);
	conf->cycle |= val << shift;

	return ret;
}

/**
 * atmel_smc_cs_conf_apply - apply an SMC CS conf
 * @regmap: the SMC regmap
 * @cs: the CS id
 * @conf the SMC CS conf to apply
 *
 * Applies an SMC CS configuration.
 * Only valid on at91sam9/avr32 SoCs.
 */
static inline void atmel_smc_cs_conf_apply(struct regmap *regmap, int cs,
					   const struct atmel_smc_cs_conf *conf)
{
	regmap_write(regmap, ATMEL_SMC_SETUP(cs), conf->setup);
	regmap_write(regmap, ATMEL_SMC_PULSE(cs), conf->pulse);
	regmap_write(regmap, ATMEL_SMC_CYCLE(cs), conf->cycle);
	regmap_write(regmap, ATMEL_SMC_MODE(cs), conf->mode);
}

/**
 * atmel_hsmc_cs_conf_apply - apply an SMC CS conf
 * @regmap: the HSMC regmap
 * @cs: the CS id
 * @conf the SMC CS conf to apply
 *
 * Applies an SMC CS configuration.
 * Only valid on post-sama5 SoCs.
 */
static inline void atmel_hsmc_cs_conf_apply(struct regmap *regmap, int cs,
					const struct atmel_smc_cs_conf *conf)
{
	regmap_write(regmap, ATMEL_HSMC_SETUP(cs), conf->setup);
	regmap_write(regmap, ATMEL_HSMC_PULSE(cs), conf->pulse);
	regmap_write(regmap, ATMEL_HSMC_CYCLE(cs), conf->cycle);
	regmap_write(regmap, ATMEL_HSMC_TIMINGS(cs), conf->timings);
	regmap_write(regmap, ATMEL_HSMC_MODE(cs), conf->mode);
}

/**
 * atmel_smc_cs_conf_get - retrieve the current SMC CS conf
 * @regmap: the SMC regmap
 * @cs: the CS id
 * @conf: the SMC CS conf object to store the current conf
 *
 * Retrieve the SMC CS configuration.
 * Only valid on at91sam9/avr32 SoCs.
 */
static inline void atmel_smc_cs_conf_get(struct regmap *regmap, int cs,
					 struct atmel_smc_cs_conf *conf)
{
	regmap_read(regmap, ATMEL_SMC_SETUP(cs), &conf->setup);
	regmap_read(regmap, ATMEL_SMC_PULSE(cs), &conf->pulse);
	regmap_read(regmap, ATMEL_SMC_CYCLE(cs), &conf->cycle);
	regmap_read(regmap, ATMEL_SMC_MODE(cs), &conf->mode);
}

/**
 * atmel_hsmc_cs_conf_get - retrieve the current SMC CS conf
 * @regmap: the HSMC regmap
 * @cs: the CS id
 * @conf: the SMC CS conf object to store the current conf
 *
 * Retrieve the SMC CS configuration.
 * Only valid on post-sama5 SoCs.
 */
static inline void atmel_hsmc_cs_conf_get(struct regmap *regmap, int cs,
					  struct atmel_smc_cs_conf *conf)
{
	regmap_read(regmap, ATMEL_HSMC_SETUP(cs), &conf->setup);
	regmap_read(regmap, ATMEL_HSMC_PULSE(cs), &conf->pulse);
	regmap_read(regmap, ATMEL_HSMC_CYCLE(cs), &conf->cycle);
	regmap_read(regmap, ATMEL_HSMC_TIMINGS(cs), &conf->timings);
	regmap_read(regmap, ATMEL_HSMC_MODE(cs), &conf->mode);
}

#endif /* _LINUX_MFD_SYSCON_ATMEL_SMC_H_ */
