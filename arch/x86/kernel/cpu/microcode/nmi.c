// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2022 Intel Corporation
 *	Ashok Raj <ashok.raj@intel.com>
 *
 * X86 CPU microcode update NMI handler.
 *
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/cpu.h>
#include <linux/nmi.h>

#include <asm/microcode.h>

#define SPINUNIT	100 /* 100 nsec */

DEFINE_PER_CPU(struct core_rendez, core_sync);
DEFINE_PER_CPU(struct core_rendez *, nmi_primary_ptr);

static u64 loops_per_spinunit __read_mostly = 2 * SPINUNIT;

static __init int init_loops(void)
{
	unsigned long lpj;

	lpj = this_cpu_read(cpu_info.loops_per_jiffy) ? : loops_per_jiffy;
	loops_per_spinunit = (lpj * HZ)/(NSEC_PER_SEC / SPINUNIT);

	pr_info("Loops per spinunit = 0x%llu\n", loops_per_spinunit);

	return 0;
}

late_initcall(init_loops);

/*
 * Siblings wait until microcode update is completed by the primary thread.
 */
static int noinstr __wait_for_update(atomic_t *t)
{
	long timeout = NSEC_PER_SEC;

	while (!arch_atomic_read(t)) {
		delay_loop(loops_per_spinunit);

		timeout -= SPINUNIT;
		if (timeout < SPINUNIT)
			return 1;
	}
	return 0;
}

noinstr void hold_sibling_in_nmi(void)
{
	struct	 core_rendez *pcpu_core;
	int ret = 0;

	pcpu_core = this_cpu_read(nmi_primary_ptr);
	if (likely(!pcpu_core))
		return;

	/*
	 * Decrement the callin to inform primary thread that the sibling
	 * has arrived and parked in the NMI handler
	 */
	arch_atomic_dec(&pcpu_core->callin);

	ret = __wait_for_update(&pcpu_core->core_done);
	if (ret)
		arch_atomic_inc(&pcpu_core->failed);

	/*
	 * Clear the nmi_trap, so future NMI's won't be affected
	 */
	this_cpu_write(nmi_primary_ptr, NULL);
}
