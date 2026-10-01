// SPDX-License-Identifier: GPL-2.0
/*
 * Symbols the vendor Hi110x code expects from other parts of the HiSilicon kernel.
 * On a laptop none of them is useful, so they are provided here as no-ops or with a
 * plain implementation.
 */
#include <linux/kernel.h>
#include <linux/cpumask.h>
#include <linux/topology.h>

#include "platform_oneimage_define.h"
#include "chr_user.h"

/* ---- CHR (phone fault reporting to the Huawei cloud): not used ---- */
int32_t __chr_exception_para(uint32_t chr_errno, uint8_t *chr_ptr, uint16_t chr_len)
{
	return 0;
}

int32_t __chr_exception_para_q(uint32_t chr_errno, chr_report_flags_enum_uint16 chr_flag,
			       uint8_t *chr_ptr, uint16_t chr_len)
{
	return 0;
}

void chr_dev_exception_callback(void *buff, uint16_t len)
{
}

void chr_host_callback_register(chr_get_wifi_info pfunc)
{
}

void chr_host_callback_unregister(void)
{
}

/* ---- big/little CPU masks (vendor scheduler helpers) ----
 * Kirin 990: CPUs 0-3 are A55, 4-7 the big cores. Use the CPU capacity instead of
 * hard-coding the numbers.
 */
void hisi_get_fast_cpus(struct cpumask *cpumask)
{
	unsigned long max = 0;
	int cpu;

	for_each_possible_cpu(cpu)
		max = max(max, arch_scale_cpu_capacity(cpu));
	cpumask_clear(cpumask);
	for_each_possible_cpu(cpu)
		if (arch_scale_cpu_capacity(cpu) == max)
			cpumask_set_cpu(cpu, cpumask);
	if (cpumask_empty(cpumask) || cpumask_equal(cpumask, cpu_possible_mask)) {
		/* capacities unknown: assume the upper half */
		cpumask_clear(cpumask);
		for_each_possible_cpu(cpu)
			if (cpu >= nr_cpu_ids / 2)
				cpumask_set_cpu(cpu, cpumask);
	}
}

void hisi_get_slow_cpus(struct cpumask *cpumask)
{
	struct cpumask fast;

	hisi_get_fast_cpus(&fast);
	cpumask_andnot(cpumask, cpu_possible_mask, &fast);
}

/* ---- SDIO host reset flag, owned by the vendor MMC host driver ---- */
int g_sdio_reset_ip;
