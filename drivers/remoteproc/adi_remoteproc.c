/*
 * Analog Device SHARC Image Loader for SC5XX processors
 *
 * Copyright (C) 2020 Analog Devices
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/kobject.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <mach/hardware.h>
#include <mach/sec.h>
#include <mach/dma.h>
#include <mach/sc58x.h>
#include <mach/icc.h>
#include <mach/gpio.h>
#include "remoteproc_internal.h"

/* The VERIFY_LDR_DATA macro is used to verify LDR data loaded to
 * the target memory.
 *
 * Disable this macro by default to speed up the whole process
 * To enable it by adding below line:
 * #define VERIFY_LDR_DATA
 */

/* location of bootrom that loops idle */
#define SHARC_IDLE_ADDR			(0x00090004)
/* Bit values for the RCU0_MSG register */
#define RCU0_MSG_C0IDLE			0x00000100		/* Core 0 Idle */
#define RCU0_MSG_C1IDLE			0x00000200		/* Core 1 Idle */
#define RCU0_MSG_C2IDLE			0x00000400		/* Core 2 Idle */
#define RCU0_MSG_CRR0			0x00001000		/* Core 0 reset request */
#define RCU0_MSG_CRR1			0x00002000		/* Core 1 reset request */
#define RCU0_MSG_CRR2			0x00004000		/* Core 2 reset request */
#define RCU0_MSG_C1ACTIVATE		0x00080000		/* Core 1 Activated */
#define RCU0_MSG_C2ACTIVATE		0x00100000		/* Core 2 Activated */

#define SPU_MDMA0_SRC_ID		88
#define SPU_MDMA0_DST_ID		89
#define VALID_CORE_MIN			1
#define VALID_CORE_MAX			2

#if defined(VERIFY_LDR_DATA)
#define MEMORY_COUNT 2
#else
#define MEMORY_COUNT 1
#endif

struct adi_rproc_data {
	struct device *dev;
	struct rproc *rproc;
	struct kobject *kobj;
	const char *firmware_name;
	int core_id;
	int core_irq;
	void *mem_virt;
	dma_addr_t mem_handle;
	size_t fw_size;
	unsigned long ldr_load_addr;
	struct rcu_reg __iomem *rcu_base;
};

typedef struct block_code_flag {
	uint32_t bCode:4,			/* 0-3 */
			 bFlag_save:1,		/* 4 */
			 bFlag_aux:1,		/* 5 */
			 bReserved:1,		/* 6 */
			 bFlag_forward:1,	/* 7 */
			 bFlag_fill:1,		/* 8 */
			 bFlag_quickboot:1, /* 9 */
			 bFlag_callback:1,	/* 10 */
			 bFlag_init:1,		/* 11 */
			 bFlag_ignore:1,	/* 12 */
			 bFlag_indirect:1,	/* 13 */
			 bFlag_first:1,		/* 14 */
			 bFlag_final:1,		/* 15 */
			 bHdrCHK:8,			/* 16-23 */
			 bHdrSIGN:8;		/* 0xAD, 0xAC or 0xAB */
} BCODE_FLAG_t;

typedef struct ldr_hdr {
	BCODE_FLAG_t bcode_flag;
	uint32_t target_addr;
	uint32_t byte_count;
	uint32_t argument;
} LDR_Ehdr_t;

static irqreturn_t sharc_irq_handler(int irq, void *priv)
{
	static int irq_count = 0;

	irq_count++;
	/* For the 4th IRQ Message we should toggle LED12 on the SAM board */
	if (irq_count == 4) {
		int led_gpio = gpio_request(GPIO_PD3, "LED12");
		if (led_gpio != 0)
			printk(KERN_ERR "Failed to open LED12 gpio\n");
		else {
			int res = gpio_direction_input(led_gpio);
			if (res != 0)
				printk(KERN_ERR "Failed to set direction on LED12 gpio\n");
			else {
				// Check the LED is already high from the eth message
				int led_state = gpio_get_value(GPIO_PD3);
				if (led_state == 1)
					// led is high, unexpected
					printk(KERN_ERR "LED gpio is unexpectedly high.\n");
				else
					// LED is low as expected, set it high
					gpio_direction_output(GPIO_PD3, 1);
			}
			gpio_free(GPIO_PD3);
		}
	}
	// printk(KERN_ERR "Receive irq from Sharc Core 1\n");
	return IRQ_HANDLED;
}

static ssize_t att_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	return count;
}

static ssize_t att_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	printk("Sending the gong playback notification to Sharc\n");
	platform_send_ipi_sharc0();
	return 0;
}

static DEVICE_ATTR(test, 0644, att_show, NULL);

static int adi_core_set_svect(struct adi_rproc_data *rproc_data,
					unsigned long svect)
{
	int coreid = rproc_data->core_id;
	struct rcu_reg __iomem *rcu_base = rproc_data->rcu_base;

	if (svect && (coreid == 1))
		writel(svect, &rcu_base->reg_rcu_svect1);
	else if (svect && (coreid == 2))
		writel(svect, &rcu_base->reg_rcu_svect2);
	else {
		dev_err(rproc_data->dev, "%s, invalid svect:0x%lx, cord_id:%d\n",
						__func__, svect, coreid);
		return -EINVAL;
	}

	return 0;
}

/* Active SHARC core */
static int adi_core_start(struct adi_rproc_data *rproc_data)
{
	int coreid = rproc_data->core_id;

	struct rcu_reg __iomem *rcu_base = rproc_data->rcu_base;

	if ((coreid < VALID_CORE_MIN) || (coreid > VALID_CORE_MAX)) {
		printk(" %s: invalid Core ID:%d\n", __func__, coreid);
		return -EINVAL;
	}

	/* Clear the IDLE bit when start the SHARC core */
	writel(RCU0_MSG_C0IDLE << coreid, &rcu_base->reg_rcu_msg_clr);

	/* Notify CCES */
	writel(RCU0_MSG_C1ACTIVATE << (coreid-1), &rcu_base->reg_rcu_msg_set);

	return 0;
}

/* Reset ADI SHARC core */
static int adi_core_reset(struct adi_rproc_data *rproc_data)
{
	int coreid = rproc_data->core_id;
	struct rcu_reg __iomem *rcu_base = rproc_data->rcu_base;

	if ((coreid < VALID_CORE_MIN) || (coreid > VALID_CORE_MAX)) {
		printk(" %s: invalid Core ID:%d\n", __func__, coreid);
		return -1;
	}

	/* First put core in reset.
     * Clear CRSTAT bit for given coreid. */
	writel(1 << coreid, &rcu_base->reg_rcu_crstat);

	/* Set SIDIS to disable the system interface */
	writel(readl(&rcu_base->reg_rcu_sidis) | (1 << (coreid-1)),
				&rcu_base->reg_rcu_sidis);

	/*
	 * Wait for access to coreX have been disabled and all the pending
	 * transactions have completed
	 */
	udelay(50);

	/* Set CRCTL bit to put core in reset */
	writel(readl(&rcu_base->reg_rcu_crctl) | (1 << coreid),
				&rcu_base->reg_rcu_crctl);

	/* Poll until Core is in reset */
	while(!(readl(&rcu_base->reg_rcu_crstat) & (1 << coreid)));

	/* Clear SIDIS to reenable the system interface */
	writel(readl(&rcu_base->reg_rcu_sidis) & ~(1 << (coreid-1)),
				&rcu_base->reg_rcu_sidis);

	udelay(50);

	/* Take Core out of reset */
	writel(readl(&rcu_base->reg_rcu_crctl) & ~(1 << coreid),
				&rcu_base->reg_rcu_crctl);

	/* Wait for done */
	udelay(50);

	return 0;
}

static int adi_core_stop(struct adi_rproc_data *rproc_data)
{
	int coreid = rproc_data->core_id;
	int coreirq = rproc_data->core_irq;
	struct rcu_reg __iomem *rcu_base = rproc_data->rcu_base;
	unsigned long timeout = jiffies + HZ*5;
	bool is_timeout = true;

	if ((coreid < VALID_CORE_MIN) || (coreid > VALID_CORE_MAX)) {
		printk(" %s: invalid Core ID:%d\n", __func__, coreid);
		return -EINVAL;
	}

	if (readl(&rcu_base->reg_rcu_crctl) & (1 << coreid))
		return 0;

	/* Check the IDLE bit in RCU_MSG register */
	if (!(readl(&rcu_base->reg_rcu_msg) & (RCU0_MSG_C0IDLE << coreid))) {
		/* Set core reset request bit in RCU_MSG bit(12:14) */
		writel(readl(&rcu_base->reg_rcu_msg) | (RCU0_MSG_CRR0 << coreid),
					&rcu_base->reg_rcu_msg_set);

		/* Raise SOFT IRQ through SEC
		 * DSP enter into ISR to release interrupts used by DSP program
		 */
		sec_set_ssi_coreid(coreirq, coreid);
		sec_enable_ssi(coreirq, false, true);
		sec_enable_sci(coreid);
		sec_raise_irq(coreirq);
	}

	/* Wait until the specific core enter into IDLE bit(8:10)
	 * DSP should set the IDLE bit to 1 manully in ISR
	 */
	do {
		if (readl(&rcu_base->reg_rcu_msg) & (RCU0_MSG_C0IDLE << coreid)) {
			is_timeout = false;
			break;
		}
	} while(time_before(jiffies, timeout));

	if (is_timeout)
		dev_warn(rproc_data->dev, "Timeout to release remote core%d!\n", coreid);

	/* Clear core reset request bit in RCU_MSG bit(12:14) */
	writel(readl(&rcu_base->reg_rcu_msg) & (RCU0_MSG_CRR0 << coreid),
				&rcu_base->reg_rcu_msg_clr);

	/* Clear Activate bit when stop SHARC core */
	writel(RCU0_MSG_C1ACTIVATE << (coreid-1), &rcu_base->reg_rcu_msg_clr);

	return 0;
}

static void ldr_load(struct adi_rproc_data *rproc_data)
{
	LDR_Ehdr_t* block_hdr = NULL;
	uint8_t* virbuf = (uint8_t*) rproc_data->mem_virt;
	uint8_t* phybuf = (uint8_t*) rproc_data->mem_handle;
	void __iomem *remap_addr;
#if defined(VERIFY_LDR_DATA)
	int i;
	uint32_t verfied = 0;
	uint8_t* pCompareBuffer;
	uint8_t* pVerifyBuffer;
#endif

	do {
		/* read the header */
		block_hdr = (LDR_Ehdr_t*) virbuf;

		/* Overwrite the ldr_load_addr */
		if (block_hdr->bcode_flag.bFlag_first)
				rproc_data->ldr_load_addr =
							(unsigned long)block_hdr->target_addr;

		if (block_hdr->bcode_flag.bFlag_ignore == 0
						&& block_hdr->byte_count != 0) {
			if (block_hdr->bcode_flag.bFlag_fill == 0x1) { /* fill */
				remap_addr = ioremap_nocache(block_hdr->target_addr,
									block_hdr->byte_count);
				memset(remap_addr, block_hdr->argument, block_hdr->byte_count);
				iounmap(remap_addr);
			} else { /* normal */
				dma_memcpy(block_hdr->target_addr,
						phybuf + sizeof(LDR_Ehdr_t), block_hdr->byte_count);

#if defined(VERIFY_LDR_DATA)
				pCompareBuffer = virbuf + sizeof(LDR_Ehdr_t);
				pVerifyBuffer = virbuf + rproc_data->fw_size;

				dma_memcpy(phybuf + rproc_data->fw_size, block_hdr->target_addr,
							block_hdr->byte_count);

				/* check the data */
				for (i = 0; i < block_hdr->byte_count; i++) {
					if (pCompareBuffer[i] != pVerifyBuffer[i]) {
						printk("dirty data, pCompareBuffer[%d]:0x%x,\
							pVerifyBuffer[%d]:0x%x\n",
							i, pCompareBuffer[i], i, pVerifyBuffer[i]);
						verfied++;
						break;
					}
				}
#endif
			}
		}

		if (block_hdr->bcode_flag.bFlag_final == 0x1)
			break;

		int offset = sizeof(LDR_Ehdr_t) + (block_hdr->bcode_flag.bFlag_fill ?
							0 : block_hdr->byte_count);
		virbuf += offset;
		phybuf += offset;
	} while (1);

#if defined(VERIFY_LDR_DATA)
	if (verfied == 0)
		printk("success to verify all the data\n");
	else
		printk("fail to verify all the data %d\n",verfied);
#endif

}

static bool ldr_phdr_valid(const LDR_Ehdr_t *phdr)
{
	if (!phdr->byte_count
				&& (phdr->bcode_flag.bHdrSIGN == 0xAD
					|| phdr->bcode_flag.bHdrSIGN == 0xAC
					|| phdr->bcode_flag.bHdrSIGN == 0xAB))
		return true;
	else
		return false;
}

void set_spu_securep_msec(uint16_t n, bool msec);
static void enable_spu(void)
{
	set_spu_securep_msec(SPU_MDMA0_SRC_ID, true);
	set_spu_securep_msec(SPU_MDMA0_DST_ID, true);
}

static void disable_spu(void)
{
	set_spu_securep_msec(SPU_MDMA0_SRC_ID, false);
	set_spu_securep_msec(SPU_MDMA0_DST_ID, false);
}

static int adi_ldr_load(struct adi_rproc_data *rproc_data,
						const struct firmware *fw)
{
	int ret = 0;

	rproc_data->fw_size = fw->size;
	if (!rproc_data->mem_virt) {
		rproc_data->mem_virt = dma_zalloc_coherent(rproc_data->dev,
								fw->size * MEMORY_COUNT, &rproc_data->mem_handle,
								GFP_KERNEL);
		if (rproc_data->mem_virt == NULL) {
			dev_err(rproc_data->dev, "Unable to allocate memory\n");
			return -ENOMEM;
		}
	}

	memcpy((char*)rproc_data->mem_virt, fw->data, fw->size);

	/* Check if it is a LDR file */
	if (ldr_phdr_valid((LDR_Ehdr_t*)rproc_data->mem_virt)) {
		enable_spu();
		ldr_load(rproc_data);
		disable_spu();
	} else {
		printk("## No ldr image at address 0x%x\n", rproc_data->mem_virt);
		dma_free_coherent(rproc_data->dev, rproc_data->fw_size * MEMORY_COUNT,
						rproc_data->mem_virt, rproc_data->mem_handle);
		rproc_data->mem_virt = NULL;
		ret = -1;
	}

	return ret;
}

/*
 * adi_rproc_load: parse and load ADI SHARC LDR file into memory
 *
 * This function would be called when user run the start command
 * echo start > /sys/class/remoteproc/remoteprocX/state
 */
static int adi_rproc_load(struct rproc *rproc, const struct firmware *fw)
{
	int ret = 0;
	struct adi_rproc_data *rproc_data = (struct adi_rproc_data *)rproc->priv;

	ret = adi_ldr_load(rproc_data, fw);
	if (ret) {
		dev_err(rproc_data->dev, "Failed to load ldr, ret:%d\n", ret);
		return ret;
	}

	return ret;
}

/*
 * adi_rproc_start: to start run the applicaiton which is loaded in memory
 *
 * This function would be called when user run the start command
 * echo start > /sys/class/remoteproc/remoteprocX/state
 */
static int adi_rproc_start(struct rproc *rproc)
{
	struct adi_rproc_data *rproc_data = (struct adi_rproc_data *)rproc->priv;
	int ret;

	ret = adi_core_set_svect(rproc_data, rproc_data->ldr_load_addr);
	if (ret)
		return ret;

	ret = adi_core_reset(rproc_data);
	if (ret)
		return ret;

	return adi_core_start(rproc_data);
}

/*
 * adi_rproc_stop: to stop the running applicaiton in DSP
 * This would be called when user run the stop command
 * echo stop > /sys/class/remoteproc/remoteprocX/state
 */
static int adi_rproc_stop(struct rproc *rproc)
{
	struct adi_rproc_data *rproc_data = (struct adi_rproc_data *)rproc->priv;
	int ret;

	ret = adi_core_set_svect(rproc_data, SHARC_IDLE_ADDR);
	if (ret)
		return ret;

	ret = adi_core_stop(rproc_data);
	if (ret)
		return ret;

	ret = adi_core_reset(rproc_data);
	if (ret)
		return ret;


	if (rproc_data->mem_virt) {
		memset(rproc_data->mem_virt, 0, rproc_data->fw_size * MEMORY_COUNT);
		dma_free_coherent(rproc_data->dev, rproc_data->fw_size * MEMORY_COUNT,
						rproc_data->mem_virt, rproc_data->mem_handle);
		rproc_data->mem_virt = NULL;
		rproc_data->fw_size = 0;
	}

	rproc_data->ldr_load_addr = SHARC_IDLE_ADDR;
	return ret;
}

static const struct rproc_ops adi_rproc_ops = {
	.start = adi_rproc_start,
	.stop = adi_rproc_stop,
	.load = adi_rproc_load,
};

static int adi_remoteproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct adi_rproc_data *rproc_data;
	struct device_node *np = dev->of_node;
	struct rproc *rproc;
	struct resource *res;
	int ret;
	int irq;
	const char *name;

	ret = of_property_read_string(np, "firmware-name",
					&name);
	if (ret) {
		dev_err(dev, "Unable to get firmware-name property\n");
		return ret;
	}

	rproc = rproc_alloc(dev, np->name, &adi_rproc_ops,
					name, sizeof(*rproc_data));
	if (!rproc) {
		dev_err(dev, "Unable to allocate remoteproc\n");
		return -ENOMEM;
	}

	rproc_data = (struct adi_rproc_data *)rproc->priv;
	platform_set_drvdata(pdev, rproc);

	ret = of_property_read_u32(np, "core-id",
					&rproc_data->core_id);
	if (ret) {
		dev_err(dev, "Unable to get core-id property\n");
		goto free_rproc;
	}

	ret = of_property_read_u32(np, "core-irq",
					&rproc_data->core_irq);
	if (ret) {
		dev_err(dev, "Unable to get core-irq property\n");
		goto free_rproc;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(dev, "Cannot get IORESOURCE_MEM\n");
		goto free_rproc;
	}
	rproc_data->rcu_base = devm_ioremap_nocache(dev,
							    res->start, resource_size(res));
	if (IS_ERR((void *)rproc_data->rcu_base)) {
		dev_err(dev, "Cannot map rcu memory\n");
		ret = PTR_ERR((void *)rproc_data->rcu_base);
		goto free_rproc;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		dev_err(dev, "No irq specified\n");
		return -ENOENT;
	}

	ret = devm_request_irq(dev, irq, sharc_irq_handler,
					IRQF_SHARED, "sharc", rproc_data);
	if (ret) {
		dev_err(dev, "Failed to register irq %d, ret:%d\n", irq, ret);
		return -ENOENT;
	}

	rproc_data->dev = &pdev->dev;
	rproc_data->rproc = rproc;
	rproc_data->firmware_name = name;
	rproc_data->mem_virt = NULL;
	rproc_data->fw_size = 0;
	rproc_data->ldr_load_addr = SHARC_IDLE_ADDR;

	ret = rproc_add(rproc);
	if (ret) {
		dev_err(dev, "Failed to add rproc\n");
		goto free_rproc;
	}

	rproc_data->kobj = kobject_create_and_add("remotekobj", NULL);
	if (rproc_data->kobj == NULL) {
		ret = -ENOMEM;
		rproc_del(rproc_data->rproc);
		goto free_rproc;
	}

	ret = sysfs_create_file(rproc_data->kobj, &dev_attr_test.attr);
	if (ret < 0) {
		kobject_del(rproc_data->kobj);
		rproc_del(rproc_data->rproc);
		goto free_rproc;
	}

	return 0;

free_rproc:
	rproc_free(rproc);

	return ret;
}

static int adi_remoteproc_remove(struct platform_device *pdev)
{
	struct adi_rproc_data *rproc_data = platform_get_drvdata(pdev);

	kobject_del(rproc_data->kobj);
	rproc_del(rproc_data->rproc);
	rproc_free(rproc_data->rproc);

	return 0;
}

static const struct of_device_id adi_rproc_of_match[] = {
	{ .compatible = "adi,remoteproc" },
	{ },
};
MODULE_DEVICE_TABLE(of, adi_rproc_of_match);

static struct platform_driver adi_rproc_driver = {
	.probe = adi_remoteproc_probe,
	.remove = adi_remoteproc_remove,
	.driver = {
		.name = "adi_remoteproc",
		.of_match_table = adi_rproc_of_match,
	},
};

static int __init adi_remoteproc_init(void)
{
	return platform_driver_register(&adi_rproc_driver);
}
late_initcall_sync(adi_remoteproc_init);

MODULE_DESCRIPTION("Analog Device sc5xx SHARC Image Loader");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Greg Chen <jian.chen@analog.com>");
