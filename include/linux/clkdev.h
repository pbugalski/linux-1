/*
 *  include/linux/clkdev.h
 *
 *  Copyright (C) 2008 Russell King.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Helper for the clk API to assist looking up a struct clk.
 */
#ifndef __CLKDEV_H
#define __CLKDEV_H

#include <asm/clkdev.h>

struct clk;
struct device;

/*
 * To avoid a mass-rename of all non-common clock implementations (spread out
 * in arch-specific code), we let them use struct clk for both the internal and
 * external view.
 */
#ifdef CONFIG_COMMON_CLK
struct clk_core;
#define clk_core_t struct clk_core
#else
#define clk_core_t struct clk
#endif

struct clk_lookup {
	struct list_head	node;
	const char		*dev_id;
	const char		*con_id;
	clk_core_t		*clk;
};

#define CLKDEV_INIT(d, n, c)	\
	{			\
		.dev_id = d,	\
		.con_id = n,	\
		.clk = c,	\
	}

struct clk_lookup *clkdev_alloc(clk_core_t *clk, const char *con_id,
	const char *dev_fmt, ...);

void clkdev_add(struct clk_lookup *cl);
void clkdev_drop(struct clk_lookup *cl);

void clkdev_add_table(struct clk_lookup *, size_t);
int clk_add_alias(const char *, const char *, char *, struct device *);

int clk_register_clkdev(clk_core_t *, const char *, const char *, ...);
int clk_register_clkdevs(clk_core_t *, struct clk_lookup *, size_t);

#ifdef CONFIG_COMMON_CLK
int __clk_get(struct clk_core *clk);
void __clk_put(struct clk_core *clk);
#endif

#endif
