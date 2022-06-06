/*
 * Faraday LEO - CPU frequency scaling support
 *
 * (C) Copyright 2020 Faraday Technology
 * Bo-Cun Chen <bcchen@faraday-tech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>

#include <asm/cputype.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>
#ifdef CONFIG_SMP
#include <asm/smp.h>
#endif

#include "faraday-cpufreq.h"

#include "mach/hardware.h"

#define GIC_CPU_CTRL                0x00
#define GIC_DIST_ENABLE_SET         0x100
#define GIC_DIST_ENABLE_CLEAR       0x180
#define GIC_DIST_TARGET             0x800

volatile int cpu_freeze = -1;

static u32 saved_irq[32];

static void set_wfi(void)
{
	asm("wfi");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
}

static void write_cpu_freeze(int val)
{
	cpu_freeze = val;
	smp_wmb();
	sync_cache_w(&cpu_freeze);
}

static void prepare_cpu_suspend(void *arg)
{
	//printk("%s cpu=%d\n", __func__, smp_processor_id());
	// disable per-cpu local timer irq
	writel(0x6000FFFF, (void *)PLAT_GIC_DIST_VA_BASE + GIC_DIST_ENABLE_CLEAR);
	
	while (cpu_freeze){
		dsb();
		wfe();
	}
	
	// enable per-cpu local timer irq
	writel(0x6000FFFF, (void *)PLAT_GIC_DIST_VA_BASE + GIC_DIST_ENABLE_SET);
}

#define UART_LSR                    0x14        /* In:  Line Status Register */ 
#define UART_LSR_TEMT               0x40        /* Transmitter empty */ 

#define DLLFRANG(ns)                ((ns) <= 0x04) ? 0 : (((ns) <= 0x09) ? 1 : (((ns) <= 0x13) ? 2 : 3))

static int leo_calculate_freqs(unsigned int freqs_KHz)
{
	int target;

	switch (freqs_KHz) {
		case 500000:
			target = 0x14;
			break;
		case 525000:
			target = 0x15;
			break;
		case 550000:
			target = 0x16;
			break;
		case 575000:
			target = 0x17;
			break;
		case 600000:
			target = 0x18;
			break;
		case 625000:
			target = 0x19;
			break;
		case 650000:
			target = 0x1a;
			break;
		case 675000:
			target = 0x1b;
			break;
		case 700000:
			target = 0x1c;
			break;
		case 725000:
			target = 0x1d;
			break;
		case 750000:
			target = 0x1e;
			break;
		case 775000:
			target = 0x1f;
			break;
		case 800000:
			target = 0x20;
			break;
		case 825000:
			target = 0x21;
			break;
		case 850000:
			target = 0x22;
			break;
		case 875000:
			target = 0x23;
			break;
		case 900000:
			target = 0x24;
			break;
		case 925000:
			target = 0x25;
			break;
		case 950000:
			target = 0x26;
			break;
		case 975000:
			target = 0x27;
			break;
		case 1000000:
			target = 0x28;
			break;
		case 1025000:
			target = 0x29;
			break;
		case 1050000:
			target = 0x20;
			break;
		case 1075000:
			target = 0x2B;
			break;
		case 1100000:
			target = 0x2C;
			break;
		case 1125000:
			target = 0x2D;
			break;
		case 1150000:
			target = 0x2E;
			break;
		case 1175000:
			target = 0x2F;
			break;
		case 1200000:
			target = 0x30;
			break;
		default:
			target = 0x20;
			break;
	}

	return target;
}

static void leo_set_frequency(struct faraday_cpu_dvfs_info *info, unsigned int freqs_KHz)
{
	unsigned int cpu, val;
	int current_ns, target_ns;
	int i, timeout = 10000;

	cpu = smp_processor_id();
	//printk("%s cpu=%d freqs_KHz=%d\n", __func__, cpu, freqs_KHz);

	// saves gic distributor enable registers, and clear its
	local_irq_disable();
	for (i = 0; i < 32; i++) {
		saved_irq[i] = readl((void *)PLAT_GIC_DIST_VA_BASE + GIC_DIST_ENABLE_SET + i * 4);
		writel(0xFFFFFFFF, (void *)PLAT_GIC_DIST_VA_BASE + GIC_DIST_ENABLE_CLEAR + i * 4);
	}
	local_irq_enable();

	// retarget scu irq to current cpu
	val = readl((void *)PLAT_GIC_DIST_VA_BASE + GIC_DIST_TARGET + 0x20) & ~0x0000000F;
	writel(val | (1 << cpu), (void *)PLAT_GIC_DIST_VA_BASE + GIC_DIST_TARGET + 0x20);

	// enable scu irq
	writel(0x00000001, (void *)PLAT_GIC_DIST_VA_BASE + GIC_DIST_ENABLE_SET + 0x04);

	// make slave cpus go to idle
	write_cpu_freeze(1);
	for (i = 0; i < 4; i++) {
		if (i != cpu) {
			smp_call_function_single(i, prepare_cpu_suspend, NULL, 0);
		}
	}

	/* wait uart tx over */
	while (timeout-- > 0) {
		if (readl(info->uart_base + UART_LSR) & UART_LSR_TEMT ) {
			break;
		}
		udelay(10);
	}

	udelay(100);

	// Check whether DDR is in self-refresh state
	// DDR can't be in self-refresh state before speed change
	val = readl(info->ddrc_base + 0x04);
	if (val & (0x1<<10)) {
		val = readl(info->ddrc_base + 0x04) | (0x1<<3);
		writel(val, info->ddrc_base + 0x04);
	}

	writel(0x00000000, info->sysc_base + 0xB4);

	target_ns = leo_calculate_freqs(freqs_KHz);
	current_ns = (readl(info->sysc_base + 0x30) >> 24) & 0xFF;

	if (target_ns > current_ns) {
		for (i = 1; i <= target_ns - current_ns; i++) {
			val  = ((current_ns + i) << 24);
			val |= (0x01 << 16);
			val |= (DLLFRANG(current_ns + i) << 8);
			val |= (0x03 <<  0);
			writel(val, info->sysc_base + 0x30);
			
			writel(0x70000100, info->sysc_base + 0x20);
			
			set_wfi();
		}
	} else {
		for (i = 1; i <= current_ns - target_ns; i++) {
			val  = ((current_ns - i) << 24);
			val |= (0x01 << 16);
			val |= (DLLFRANG(current_ns - i) << 8);
			val |= (0x03 <<  0);
			writel(val, info->sysc_base + 0x30);
			
			writel(0x70000100, info->sysc_base + 0x20);
			
			set_wfi();
		}
	}

	// wakeup slave cpus from idle
	write_cpu_freeze(0);
	dsb();
	sev();

	// restore gic distributor enable registers
	local_irq_disable();
	for (i = 0; i < 32; i++) {
		writel(saved_irq[i], (void *)PLAT_GIC_DIST_VA_BASE + GIC_DIST_ENABLE_SET + i * 4);
	}
	local_irq_enable();
}

int plat_cpufreq_init(struct faraday_cpu_dvfs_info *info)
{
	unsigned int status; 

	info->set_freq = leo_set_frequency;

	status = readl(info->sysc_base + 0x24);
	writel(status, info->sysc_base + 0x24);    /* clear interrupt */
	writel(0x0100, info->sysc_base + 0x28);    /* enable PLL Update interrupt */

	return 0;
}
EXPORT_SYMBOL(plat_cpufreq_init);
