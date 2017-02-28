/*
 * Driver for Atmel Pulse Width Modulation Controller
 *
 * Copyright (C) 2013 Atmel Corporation
 *		 Bo Shen <voice.shen@atmel.com>
 *
 * Licensed under GPLv2.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>

/* The following is global registers for PWM controller */
#define PWM_ENA			0x04
#define PWM_DIS			0x08
#define PWM_SR			0x0C
#define PWM_ISR			0x1C
/* Bit field in SR */
#define PWM_SR_ALL_CH_ON	0x0F

/* The following register is PWM channel related registers */
#define PWM_CH_REG_OFFSET	0x200
#define PWM_CH_REG_SIZE		0x20

#define PWM_CMR			0x0
/* Bit field in CMR */
#define PWM_CMR_CPOL		(1 << 9)
#define PWM_CMR_UPD_CDTY	(1 << 10)
#define PWM_CMR_CPRE_MSK	0xF

/* The following registers for PWM v1 */
#define PWMV1_CDTY		0x04
#define PWMV1_CPRD		0x08
#define PWMV1_CUPD		0x10

/* The following registers for PWM v2 */
#define PWMV2_CDTY		0x04
#define PWMV2_CDTYUPD		0x08
#define PWMV2_CPRD		0x0C
#define PWMV2_CPRDUPD		0x10

/*
 * Max value for duty and period
 *
 * Although the duty and period register is 32 bit,
 * however only the LSB 16 bits are significant.
 */
#define PWM_MAX_DTY		0xFFFF
#define PWM_MAX_PRD		0xFFFF
#define PRD_MAX_PRES		10

struct atmel_pwm_data {
	void (*update_cdty)(struct pwm_chip *chip,
			    struct pwm_device *pwm,
			    u32 cdty);
	void (*set_cprd_cdty)(struct pwm_chip *chip,
			      struct pwm_device *pwm,
			      u32 cprd, u32 cdty);
};

struct atmel_pwm_chip {
	struct pwm_chip chip;
	struct clk *clk;
	void __iomem *base;
	const struct atmel_pwm_data *data;
	atomic_t isr;
};

static inline struct atmel_pwm_chip *to_atmel_pwm_chip(struct pwm_chip *chip)
{
	return container_of(chip, struct atmel_pwm_chip, chip);
}

static inline u32 atmel_pwm_readl(struct atmel_pwm_chip *chip,
				  unsigned long offset)
{
	return readl_relaxed(chip->base + offset);
}

static inline void atmel_pwm_writel(struct atmel_pwm_chip *chip,
				    unsigned long offset, unsigned long val)
{
	writel_relaxed(val, chip->base + offset);
}

static inline u32 atmel_pwm_ch_readl(struct atmel_pwm_chip *chip,
				     unsigned int ch, unsigned long offset)
{
	unsigned long base = PWM_CH_REG_OFFSET + ch * PWM_CH_REG_SIZE;

	return readl_relaxed(chip->base + base + offset);
}

static inline void atmel_pwm_ch_writel(struct atmel_pwm_chip *chip,
				       unsigned int ch, unsigned long offset,
				       unsigned long val)
{
	unsigned long base = PWM_CH_REG_OFFSET + ch * PWM_CH_REG_SIZE;

	writel_relaxed(val, chip->base + base + offset);
}

static void atmel_pwm_update_cdty_v1(struct pwm_chip *chip,
				     struct pwm_device *pwm,
				     u32 cdty)
{
	struct atmel_pwm_chip *atmel_pwm = to_atmel_pwm_chip(chip);

	atmel_pwm_ch_writel(atmel_pwm, pwm->hwpwm, PWMV1_CUPD, cdty);
}

static void atmel_pwm_set_cprd_cdty_v1(struct pwm_chip *chip,
				       struct pwm_device *pwm,
				       u32 cprd, u32 cdty)
{
	struct atmel_pwm_chip *atmel_pwm = to_atmel_pwm_chip(chip);

	atmel_pwm_ch_writel(atmel_pwm, pwm->hwpwm, PWMV1_CDTY, cdty);
	atmel_pwm_ch_writel(atmel_pwm, pwm->hwpwm, PWMV1_CPRD, cprd);
}

static void atmel_pwm_update_cdty_v2(struct pwm_chip *chip,
				     struct pwm_device *pwm,
				     u32 cdty)
{
	struct atmel_pwm_chip *atmel_pwm = to_atmel_pwm_chip(chip);

	atmel_pwm_ch_writel(atmel_pwm, pwm->hwpwm, PWMV2_CDTYUPD, cdty);
}

static void atmel_pwm_set_cprd_cdty_v2(struct pwm_chip *chip,
				       struct pwm_device *pwm,
				       u32 cprd, u32 cdty)
{
	struct atmel_pwm_chip *atmel_pwm = to_atmel_pwm_chip(chip);

	atmel_pwm_ch_writel(atmel_pwm, pwm->hwpwm, PWMV2_CDTY, cdty);
	atmel_pwm_ch_writel(atmel_pwm, pwm->hwpwm, PWMV2_CPRD, cprd);
}

static int atmel_pwm_stop(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct atmel_pwm_chip *atmel_pwm = to_atmel_pwm_chip(chip);
	unsigned long timeout;

	atmel_pwm_writel(atmel_pwm, PWM_DIS, BIT(pwm->hwpwm));

	/*
	 * Wait for the PWM channel disable operation to be effective before
	 * stopping the clock.
	 */
	timeout = jiffies + 2 * HZ;
	do {
		if (!(atmel_pwm_readl(atmel_pwm, PWM_SR) & BIT(pwm->hwpwm)))
			break;

		usleep_range(10, 100);
	} while (time_before(jiffies, timeout));

	if (atmel_pwm_readl(atmel_pwm, PWM_SR) & BIT(pwm->hwpwm))
		return -ETIMEDOUT;

	clk_disable(atmel_pwm->clk);

	return 0;
}

static int atmel_pwm_calculate_cprd_and_pres(struct pwm_chip *chip,
					     const struct pwm_state *state,
					     u32 *cprd, u32 *pres)
{
	struct atmel_pwm_chip *atmel_pwm = to_atmel_pwm_chip(chip);
	unsigned long long cycles = state->period;

	/* Calculate the period cycles and prescale value */
	cycles *= clk_get_rate(atmel_pwm->clk);
	do_div(cycles, NSEC_PER_SEC);

	for (*pres = 0; cycles > PWM_MAX_PRD; cycles >>= 1)
		(*pres)++;

	if (*pres > PRD_MAX_PRES) {
		dev_err(chip->dev, "pres exceeds the maximum value\n");
		return -EINVAL;
	}

	*cprd = cycles;

	return 0;
}

static void atmel_pwm_calculate_cdty(const struct pwm_state *state,
				     u32 cprd, u32 *cdty)
{
	unsigned long long cycles = state->duty_cycle;

	cycles *= cprd;
	do_div(cycles, state->period);

	*cdty = cycles;
}

static int atmel_pwm_wait_counter_event(struct pwm_chip *chip,
					struct pwm_device *pwm)
{
	struct atmel_pwm_chip *atmel_pwm = to_atmel_pwm_chip(chip);
	unsigned long timeout = jiffies + 2 * HZ;
	u32 isr;

	/* Flush the current status. */
	atomic_and(~BIT(pwm->hwpwm), &atmel_pwm->isr);

	do {
		isr = atmel_pwm_readl(atmel_pwm, PWM_ISR);
		atomic_or(isr, &atmel_pwm->isr);
		if (atomic_read(&atmel_pwm->isr) & BIT(pwm->hwpwm))
			return 0;

		usleep_range(10, 100);
	} while (time_before(jiffies, timeout));

	return -ETIMEDOUT;
}

static int atmel_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			   struct pwm_state *state)
{
	struct atmel_pwm_chip *atmel_pwm = to_atmel_pwm_chip(chip);
	struct pwm_state cstate;
	u32 cprd, cdty, pres;
	int ret = 0;

	pwm_get_state(pwm, &cstate);

	if (state->enabled) {
		/*
		 * We must stop the PWM before changing the polarity or the
		 * period.
		 */
		if (cstate.enabled &&
		    (state->polarity != cstate.polarity ||
		     state->period != cstate.period)) {
			ret = atmel_pwm_stop(chip, pwm);
			if (ret)
				return ret;

			cstate.enabled = false;
		}

		/*
		 * If the PWM is still enabled we only update CDTY, otherwise,
		 * we update all parameters and start the PWM.
		 */
		if (cstate.enabled) {
			cprd = atmel_pwm_ch_readl(atmel_pwm, pwm->hwpwm,
						  PWMV1_CPRD);
			atmel_pwm_calculate_cdty(state, cprd, &cdty);
			atmel_pwm->data->update_cdty(chip, pwm, cdty);
			ret = atmel_pwm_wait_counter_event(chip, pwm);
		} else {
			u32 cmr;

			ret = atmel_pwm_calculate_cprd_and_pres(chip,
								state, &cprd,
								&pres);
			if (ret)
				return ret;

			cmr = pres;
			if (state->polarity == PWM_POLARITY_NORMAL)
				cmr |= PWM_CMR_CPOL;

			atmel_pwm_calculate_cdty(state, cprd, &cdty);

			/* Enable the clock before accessing registers. */
			ret = clk_enable(atmel_pwm->clk);
			if (ret) {
				dev_err(chip->dev,
					"failed to enable PWM clock\n");
				return ret;
			}

			atmel_pwm->data->set_cprd_cdty(chip, pwm, cprd, cdty);
			atmel_pwm_ch_writel(atmel_pwm, pwm->hwpwm, PWM_CMR,
					    cmr);

			/* Start the PWM. */
			atmel_pwm_writel(atmel_pwm, PWM_ENA, BIT(pwm->hwpwm));
		}
	} else if (cstate.enabled) {
		/* Stop the PWM only if it was enabled. */
		ret = atmel_pwm_stop(chip, pwm);
	}

	return ret;
}

static const struct pwm_ops atmel_pwm_ops = {
	.apply = atmel_pwm_apply,
	.owner = THIS_MODULE,
};

static const struct atmel_pwm_data atmel_pwm_data_v1 = {
	.update_cdty = atmel_pwm_update_cdty_v1,
	.set_cprd_cdty = atmel_pwm_set_cprd_cdty_v1,
};

static const struct atmel_pwm_data atmel_pwm_data_v2 = {
	.update_cdty = atmel_pwm_update_cdty_v2,
	.set_cprd_cdty = atmel_pwm_set_cprd_cdty_v2,
};

static const struct platform_device_id atmel_pwm_devtypes[] = {
	{
		.name = "at91sam9rl-pwm",
		.driver_data = (kernel_ulong_t)&atmel_pwm_data_v1,
	}, {
		/* sentinel */
	},
};
MODULE_DEVICE_TABLE(platform, atmel_pwm_devtypes);

static const struct of_device_id atmel_pwm_dt_ids[] = {
	{
		.compatible = "atmel,at91sam9rl-pwm",
		.data = &atmel_pwm_data_v1,
	}, {
		.compatible = "atmel,sama5d3-pwm",
		.data = &atmel_pwm_data_v2,
	}, {
		/* sentinel */
	},
};
MODULE_DEVICE_TABLE(of, atmel_pwm_dt_ids);

static inline const struct atmel_pwm_data *
atmel_pwm_get_driver_data(struct platform_device *pdev)
{
	const struct platform_device_id *id;

	if (pdev->dev.of_node)
		return of_device_get_match_data(&pdev->dev);

	id = platform_get_device_id(pdev);

	return (struct atmel_pwm_data *)id->driver_data;
}

static int atmel_pwm_probe(struct platform_device *pdev)
{
	const struct atmel_pwm_data *data;
	struct atmel_pwm_chip *atmel_pwm;
	struct resource *res;
	int ret;

	data = atmel_pwm_get_driver_data(pdev);
	if (!data)
		return -ENODEV;

	atmel_pwm = devm_kzalloc(&pdev->dev, sizeof(*atmel_pwm), GFP_KERNEL);
	if (!atmel_pwm)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	atmel_pwm->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(atmel_pwm->base))
		return PTR_ERR(atmel_pwm->base);

	atmel_pwm->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(atmel_pwm->clk))
		return PTR_ERR(atmel_pwm->clk);

	ret = clk_prepare(atmel_pwm->clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to prepare PWM clock\n");
		return ret;
	}

	atmel_pwm->chip.dev = &pdev->dev;
	atmel_pwm->chip.ops = &atmel_pwm_ops;
	atmel_pwm->data = data;

	if (pdev->dev.of_node) {
		atmel_pwm->chip.of_xlate = of_pwm_xlate_with_flags;
		atmel_pwm->chip.of_pwm_n_cells = 3;
	}

	atmel_pwm->chip.base = -1;
	atmel_pwm->chip.npwm = 4;
	atmel_pwm->chip.can_sleep = true;
	atomic_set(&atmel_pwm->isr, 0);

	ret = pwmchip_add(&atmel_pwm->chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to add PWM chip %d\n", ret);
		goto unprepare_clk;
	}

	platform_set_drvdata(pdev, atmel_pwm);

	return ret;

unprepare_clk:
	clk_unprepare(atmel_pwm->clk);
	return ret;
}

static int atmel_pwm_remove(struct platform_device *pdev)
{
	struct atmel_pwm_chip *atmel_pwm = platform_get_drvdata(pdev);

	clk_unprepare(atmel_pwm->clk);

	return pwmchip_remove(&atmel_pwm->chip);
}

static struct platform_driver atmel_pwm_driver = {
	.driver = {
		.name = "atmel-pwm",
		.of_match_table = of_match_ptr(atmel_pwm_dt_ids),
	},
	.id_table = atmel_pwm_devtypes,
	.probe = atmel_pwm_probe,
	.remove = atmel_pwm_remove,
};
module_platform_driver(atmel_pwm_driver);

MODULE_ALIAS("platform:atmel-pwm");
MODULE_AUTHOR("Bo Shen <voice.shen@atmel.com>");
MODULE_DESCRIPTION("Atmel PWM driver");
MODULE_LICENSE("GPL v2");
