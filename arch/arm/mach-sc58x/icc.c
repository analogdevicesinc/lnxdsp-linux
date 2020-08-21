#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <mach/hardware.h>
#include <mach/sc58x.h>
#include <mach/icc.h>
#include <asm/io.h>

void platform_send_ipi_cpu(unsigned int cpu, int irq)
{
	BUG_ON(cpu > 2);
	writel(cpu + TRGM_SOFT3, __io_address(REG_TRU0_MTR));
}

void platform_send_ipi(cpumask_t callmap, int irq)
{
	unsigned int cpu;

	for_each_cpu(cpu, callmap) {
		platform_send_ipi_cpu(cpu, irq);
	}
}

void platform_clear_ipi(unsigned int cpu, int irq)
{
}

static irqreturn_t coreb_resource_manage_dummy(int irq, void *dev_id)
{
	return 1;
}

int platform_res_manage_request_irq(uint16_t subid, unsigned int cpu)
{
	return 0;
}

void platform_res_manage_free_irq(uint16_t subid)
{
}

void platform_send_ipi_sharc0(void)
{
	writel(TRGM_SOFT1, __io_address(REG_TRU0_MTR));
}

void platform_ipi_init(void)
{
	writel(TRGM_SOFT0, __io_address(REG_TRU0_SSR86));
	writel(TRGM_SOFT1, __io_address(REG_TRU0_SSR90));
	writel(TRGM_SOFT3, __io_address(REG_TRU0_SSR87));
	writel(TRGM_SOFT4, __io_address(REG_TRU0_SSR91));
	writel(TRGM_SOFT5, __io_address(REG_TRU0_SSR95));
	writel(1, __io_address(REG_TRU0_GCTL));
}
