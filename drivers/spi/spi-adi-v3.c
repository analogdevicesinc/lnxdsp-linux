// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Analog Devices SPI3 controller driver
ear*
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/delay.h>

/* SPI_CONTROL */
#define SPI_CTL_EN                  0x00000001    /* Enable */
#define SPI_CTL_MSTR                0x00000002    /* Master/Slave */
#define SPI_CTL_PSSE                0x00000004    /* controls modf error in master mode */
#define SPI_CTL_ODM                 0x00000008    /* Open Drain Mode */
#define SPI_CTL_CPHA                0x00000010    /* Clock Phase */
#define SPI_CTL_CPOL                0x00000020    /* Clock Polarity */
#define SPI_CTL_ASSEL               0x00000040    /* Slave Select Pin Control */
#define SPI_CTL_SELST               0x00000080    /* Slave Select Polarity in-between transfers */
#define SPI_CTL_EMISO               0x00000100    /* Enable MISO */
#define SPI_CTL_SIZE                0x00000600    /* Word Transfer Size */
#define SPI_CTL_SIZE08              0x00000000    /* SIZE: 8 bits */
#define SPI_CTL_SIZE16              0x00000200    /* SIZE: 16 bits */
#define SPI_CTL_SIZE32              0x00000400    /* SIZE: 32 bits */
#define SPI_CTL_LSBF                0x00001000    /* LSB First */
#define SPI_CTL_FCEN                0x00002000    /* Flow-Control Enable */
#define SPI_CTL_FCCH                0x00004000    /* Flow-Control Channel Selection */
#define SPI_CTL_FCPL                0x00008000    /* Flow-Control Polarity */
#define SPI_CTL_FCWM                0x00030000    /* Flow-Control Water-Mark */
#define SPI_CTL_FIFO0               0x00000000    /* FCWM: TFIFO empty or RFIFO Full */
#define SPI_CTL_FIFO1               0x00010000    /* FCWM: TFIFO >= 75% empty, RFIFO >= 75% full */
#define SPI_CTL_FIFO2               0x00020000    /* FCWM: TFIFO >= 50% empty, RFIFO >= 50% full */
#define SPI_CTL_FMODE               0x00040000    /* Fast-mode Enable */
#define SPI_CTL_MIOM                0x00300000    /* Multiple I/O Mode */
#define SPI_CTL_MIO_DIS             0x00000000    /* MIOM: Disable */
#define SPI_CTL_MIO_DUAL            0x00100000    /* MIOM: Enable DIOM (Dual I/O Mode) */
#define SPI_CTL_MIO_QUAD            0x00200000    /* MIOM: Enable QUAD (Quad SPI Mode) */
#define SPI_CTL_SOSI                0x00400000    /* Start on MOSI */
#define SPI_CTL_MMWEM               0x00800000    /* Memory Mapped Write Error Mask */
#define SPI_CTL_MMSE                0x01000000    /* Memory Mapped SPI Enable */
/* SPI_RX_CONTROL */
#define SPI_RXCTL_REN               0x00000001    /* Receive Channel Enable */
#define SPI_RXCTL_RTI               0x00000004    /* Receive Transfer Initiate */
#define SPI_RXCTL_RWCEN             0x00000008    /* Receive Word Counter Enable */
#define SPI_RXCTL_RDR               0x00000070    /* Receive Data Request */
#define SPI_RXCTL_RDR_DIS           0x00000000    /* RDR: Disabled */
#define SPI_RXCTL_RDR_NE            0x00000010    /* RDR: RFIFO not empty */
#define SPI_RXCTL_RDR_25            0x00000020    /* RDR: RFIFO 25% full */
#define SPI_RXCTL_RDR_50            0x00000030    /* RDR: RFIFO 50% full */
#define SPI_RXCTL_RDR_75            0x00000040    /* RDR: RFIFO 75% full */
#define SPI_RXCTL_RDR_FULL          0x00000050    /* RDR: RFIFO full */
#define SPI_RXCTL_RDO               0x00000100    /* Receive Data Over-Run */
#define SPI_RXCTL_RRWM              0x00003000    /* FIFO Regular Water-Mark */
#define SPI_RXCTL_RWM_0             0x00000000    /* RRWM: RFIFO Empty */
#define SPI_RXCTL_RWM_25            0x00001000    /* RRWM: RFIFO 25% full */
#define SPI_RXCTL_RWM_50            0x00002000    /* RRWM: RFIFO 50% full */
#define SPI_RXCTL_RWM_75            0x00003000    /* RRWM: RFIFO 75% full */
#define SPI_RXCTL_RUWM              0x00070000    /* FIFO Urgent Water-Mark */
#define SPI_RXCTL_UWM_DIS           0x00000000    /* RUWM: Disabled */
#define SPI_RXCTL_UWM_25            0x00010000    /* RUWM: RFIFO 25% full */
#define SPI_RXCTL_UWM_50            0x00020000    /* RUWM: RFIFO 50% full */
#define SPI_RXCTL_UWM_75            0x00030000    /* RUWM: RFIFO 75% full */
#define SPI_RXCTL_UWM_FULL          0x00040000    /* RUWM: RFIFO full */
/* SPI_TX_CONTROL */
#define SPI_TXCTL_TEN               0x00000001    /* Transmit Channel Enable */
#define SPI_TXCTL_TTI               0x00000004    /* Transmit Transfer Initiate */
#define SPI_TXCTL_TWCEN             0x00000008    /* Transmit Word Counter Enable */
#define SPI_TXCTL_TDR               0x00000070    /* Transmit Data Request */
#define SPI_TXCTL_TDR_DIS           0x00000000    /* TDR: Disabled */
#define SPI_TXCTL_TDR_NF            0x00000010    /* TDR: TFIFO not full */
#define SPI_TXCTL_TDR_25            0x00000020    /* TDR: TFIFO 25% empty */
#define SPI_TXCTL_TDR_50            0x00000030    /* TDR: TFIFO 50% empty */
#define SPI_TXCTL_TDR_75            0x00000040    /* TDR: TFIFO 75% empty */
#define SPI_TXCTL_TDR_EMPTY         0x00000050    /* TDR: TFIFO empty */
#define SPI_TXCTL_TDU               0x00000100    /* Transmit Data Under-Run */
#define SPI_TXCTL_TRWM              0x00003000    /* FIFO Regular Water-Mark */
#define SPI_TXCTL_RWM_FULL          0x00000000    /* TRWM: TFIFO full */
#define SPI_TXCTL_RWM_25            0x00001000    /* TRWM: TFIFO 25% empty */
#define SPI_TXCTL_RWM_50            0x00002000    /* TRWM: TFIFO 50% empty */
#define SPI_TXCTL_RWM_75            0x00003000    /* TRWM: TFIFO 75% empty */
#define SPI_TXCTL_TUWM              0x00070000    /* FIFO Urgent Water-Mark */
#define SPI_TXCTL_UWM_DIS           0x00000000    /* TUWM: Disabled */
#define SPI_TXCTL_UWM_25            0x00010000    /* TUWM: TFIFO 25% empty */
#define SPI_TXCTL_UWM_50            0x00020000    /* TUWM: TFIFO 50% empty */
#define SPI_TXCTL_UWM_75            0x00030000    /* TUWM: TFIFO 75% empty */
#define SPI_TXCTL_UWM_EMPTY         0x00040000    /* TUWM: TFIFO empty */
/* SPI_CLOCK */
#define SPI_CLK_BAUD                0x0000FFFF    /* Baud Rate */
/* SPI_DELAY */
#define SPI_DLY_STOP                0x000000FF    /* Transfer delay time */
#define SPI_DLY_LEADX               0x00000100    /* Extended (1 SCK) LEAD Control */
#define SPI_DLY_LAGX                0x00000200    /* Extended (1 SCK) LAG control */
/* SPI_SSEL */
#define SPI_SLVSEL_SSE1             0x00000002    /* SPISSEL1 Enable */
#define SPI_SLVSEL_SSE2             0x00000004    /* SPISSEL2 Enable */
#define SPI_SLVSEL_SSE3             0x00000008    /* SPISSEL3 Enable */
#define SPI_SLVSEL_SSE4             0x00000010    /* SPISSEL4 Enable */
#define SPI_SLVSEL_SSE5             0x00000020    /* SPISSEL5 Enable */
#define SPI_SLVSEL_SSE6             0x00000040    /* SPISSEL6 Enable */
#define SPI_SLVSEL_SSE7             0x00000080    /* SPISSEL7 Enable */
#define SPI_SLVSEL_SSEL1            0x00000200    /* SPISSEL1 Value */
#define SPI_SLVSEL_SSEL2            0x00000400    /* SPISSEL2 Value */
#define SPI_SLVSEL_SSEL3            0x00000800    /* SPISSEL3 Value */
#define SPI_SLVSEL_SSEL4            0x00001000    /* SPISSEL4 Value */
#define SPI_SLVSEL_SSEL5            0x00002000    /* SPISSEL5 Value */
#define SPI_SLVSEL_SSEL6            0x00004000    /* SPISSEL6 Value */
#define SPI_SLVSEL_SSEL7            0x00008000    /* SPISSEL7 Value */
/* SPI_RWC */
#define SPI_RWC_VALUE               0x0000FFFF    /* Received Word-Count */
/* SPI_RWCR */
#define SPI_RWCR_VALUE              0x0000FFFF    /* Received Word-Count Reload */
/* SPI_TWC */
#define SPI_TWC_VALUE               0x0000FFFF    /* Transmitted Word-Count */
/* SPI_TWCR */
#define SPI_TWCR_VALUE              0x0000FFFF    /* Transmitted Word-Count Reload */
/* SPI_IMASK */
#define SPI_IMSK_RUWM               0x00000002    /* Receive Urgent Water-Mark Interrupt Mask */
#define SPI_IMSK_TUWM               0x00000004    /* Transmit Urgent Water-Mark Interrupt Mask */
#define SPI_IMSK_ROM                0x00000010    /* Receive Over-Run Error Interrupt Mask */
#define SPI_IMSK_TUM                0x00000020    /* Transmit Under-Run Error Interrupt Mask */
#define SPI_IMSK_TCM                0x00000040    /* Transmit Collision Error Interrupt Mask */
#define SPI_IMSK_MFM                0x00000080    /* Mode Fault Error Interrupt Mask */
#define SPI_IMSK_RSM                0x00000100    /* Receive Start Interrupt Mask */
#define SPI_IMSK_TSM                0x00000200    /* Transmit Start Interrupt Mask */
#define SPI_IMSK_RFM                0x00000400    /* Receive Finish Interrupt Mask */
#define SPI_IMSK_TFM                0x00000800    /* Transmit Finish Interrupt Mask */
/* SPI_IMASKCL */
#define SPI_IMSK_CLR_RUW            0x00000002    /* Receive Urgent Water-Mark Interrupt Mask */
#define SPI_IMSK_CLR_TUWM           0x00000004    /* Transmit Urgent Water-Mark Interrupt Mask */
#define SPI_IMSK_CLR_ROM            0x00000010    /* Receive Over-Run Error Interrupt Mask */
#define SPI_IMSK_CLR_TUM            0x00000020    /* Transmit Under-Run Error Interrupt Mask */
#define SPI_IMSK_CLR_TCM            0x00000040    /* Transmit Collision Error Interrupt Mask */
#define SPI_IMSK_CLR_MFM            0x00000080    /* Mode Fault Error Interrupt Mask */
#define SPI_IMSK_CLR_RSM            0x00000100    /* Receive Start Interrupt Mask */
#define SPI_IMSK_CLR_TSM            0x00000200    /* Transmit Start Interrupt Mask */
#define SPI_IMSK_CLR_RFM            0x00000400    /* Receive Finish Interrupt Mask */
#define SPI_IMSK_CLR_TFM            0x00000800    /* Transmit Finish Interrupt Mask */
/* SPI_IMASKST */
#define SPI_IMSK_SET_RUWM           0x00000002    /* Receive Urgent Water-Mark Interrupt Mask */
#define SPI_IMSK_SET_TUWM           0x00000004    /* Transmit Urgent Water-Mark Interrupt Mask */
#define SPI_IMSK_SET_ROM            0x00000010    /* Receive Over-Run Error Interrupt Mask */
#define SPI_IMSK_SET_TUM            0x00000020    /* Transmit Under-Run Error Interrupt Mask */
#define SPI_IMSK_SET_TCM            0x00000040    /* Transmit Collision Error Interrupt Mask */
#define SPI_IMSK_SET_MFM            0x00000080    /* Mode Fault Error Interrupt Mask */
#define SPI_IMSK_SET_RSM            0x00000100    /* Receive Start Interrupt Mask */
#define SPI_IMSK_SET_TSM            0x00000200    /* Transmit Start Interrupt Mask */
#define SPI_IMSK_SET_RFM            0x00000400    /* Receive Finish Interrupt Mask */
#define SPI_IMSK_SET_TFM            0x00000800    /* Transmit Finish Interrupt Mask */
/* SPI_STATUS */
#define SPI_STAT_SPIF               0x00000001    /* SPI Finished */
#define SPI_STAT_RUWM               0x00000002    /* Receive Urgent Water-Mark Breached */
#define SPI_STAT_TUWM               0x00000004    /* Transmit Urgent Water-Mark Breached */
#define SPI_STAT_ROE                0x00000010    /* Receive Over-Run Error Indication */
#define SPI_STAT_TUE                0x00000020    /* Transmit Under-Run Error Indication */
#define SPI_STAT_TCE                0x00000040    /* Transmit Collision Error Indication */
#define SPI_STAT_MODF               0x00000080    /* Mode Fault Error Indication */
#define SPI_STAT_RS                 0x00000100    /* Receive Start Indication */
#define SPI_STAT_TS                 0x00000200    /* Transmit Start Indication */
#define SPI_STAT_RF                 0x00000400    /* Receive Finish Indication */
#define SPI_STAT_TF                 0x00000800    /* Transmit Finish Indication */
#define SPI_STAT_RFS                0x00007000    /* SPI_RFIFO status */
#define SPI_STAT_RFIFO_EMPTY        0x00000000    /* RFS: RFIFO Empty */
#define SPI_STAT_RFIFO_25           0x00001000    /* RFS: RFIFO 25% Full */
#define SPI_STAT_RFIFO_50           0x00002000    /* RFS: RFIFO 50% Full */
#define SPI_STAT_RFIFO_75           0x00003000    /* RFS: RFIFO 75% Full */
#define SPI_STAT_RFIFO_FULL         0x00004000    /* RFS: RFIFO Full */
#define SPI_STAT_TFS                0x00070000    /* SPI_TFIFO status */
#define SPI_STAT_TFIFO_FULL         0x00000000    /* TFS: TFIFO full */
#define SPI_STAT_TFIFO_25           0x00010000    /* TFS: TFIFO 25% empty */
#define SPI_STAT_TFIFO_50           0x00020000    /* TFS: TFIFO 50% empty */
#define SPI_STAT_TFIFO_75           0x00030000    /* TFS: TFIFO 75% empty */
#define SPI_STAT_TFIFO_EMPTY        0x00040000    /* TFS: TFIFO empty */
#define SPI_STAT_FCS                0x00100000    /* Flow-Control Stall Indication */
#define SPI_STAT_RFE                0x00400000    /* SPI_RFIFO Empty */
#define SPI_STAT_TFF                0x00800000    /* SPI_TFIFO Full */
/* SPI_ILAT */
#define SPI_ILAT_RUWMI              0x00000002    /* Receive Urgent Water Mark Interrupt */
#define SPI_ILAT_TUWMI              0x00000004    /* Transmit Urgent Water Mark Interrupt */
#define SPI_ILAT_ROI                0x00000010    /* Receive Over-Run Error Indication */
#define SPI_ILAT_TUI                0x00000020    /* Transmit Under-Run Error Indication */
#define SPI_ILAT_TCI                0x00000040    /* Transmit Collision Error Indication */
#define SPI_ILAT_MFI                0x00000080    /* Mode Fault Error Indication */
#define SPI_ILAT_RSI                0x00000100    /* Receive Start Indication */
#define SPI_ILAT_TSI                0x00000200    /* Transmit Start Indication */
#define SPI_ILAT_RFI                0x00000400    /* Receive Finish Indication */
#define SPI_ILAT_TFI                0x00000800    /* Transmit Finish Indication */
/* SPI_ILATCL */
#define SPI_ILAT_CLR_RUWMI          0x00000002    /* Receive Urgent Water Mark Interrupt */
#define SPI_ILAT_CLR_TUWMI          0x00000004    /* Transmit Urgent Water Mark Interrupt */
#define SPI_ILAT_CLR_ROI            0x00000010    /* Receive Over-Run Error Indication */
#define SPI_ILAT_CLR_TUI            0x00000020    /* Transmit Under-Run Error Indication */
#define SPI_ILAT_CLR_TCI            0x00000040    /* Transmit Collision Error Indication */
#define SPI_ILAT_CLR_MFI            0x00000080    /* Mode Fault Error Indication */
#define SPI_ILAT_CLR_RSI            0x00000100    /* Receive Start Indication */
#define SPI_ILAT_CLR_TSI            0x00000200    /* Transmit Start Indication */
#define SPI_ILAT_CLR_RFI            0x00000400    /* Receive Finish Indication */
#define SPI_ILAT_CLR_TFI            0x00000800    /* Transmit Finish Indication */

/*
 * adi spi3 registers layout
 */
struct adi_spi_regs {
	u32 revid;
	u32 control;
	u32 rx_control;
	u32 tx_control;
	u32 clock;
	u32 delay;
	u32 ssel;
	u32 rwc;
	u32 rwcr;
	u32 twc;
	u32 twcr;
	u32 reserved0;
	u32 emask;
	u32 emaskcl;
	u32 emaskst;
	u32 reserved1;
	u32 status;
	u32 elat;
	u32 elatcl;
	u32 reserved2;
	u32 rfifo;
	u32 reserved3;
	u32 tfifo;
};

struct adi_spi_controller;

struct adi_spi_transfer_ops {
	void (*write)(struct adi_spi_controller *spi_ctrl, struct spi_transfer *xfer);
	void (*read)(struct adi_spi_controller *spi_ctrl, struct spi_transfer *xfer);
	void (*duplex)(struct adi_spi_controller *spi_ctrl, struct spi_transfer *xfer);
};

/* runtime info for spi master */
struct adi_spi_controller {
	/* SPI framework hookup */
	struct spi_controller *spi_ctrl;
	struct device *dev;

	/* Regs base of SPI controller */
	struct adi_spi_regs __iomem *regs;

	/* Current message transfer state info */
	struct spi_transfer *cur_transfer;
	const struct adi_spi_transfer_ops *ops;
	dma_cookie_t tx_cookie;
	dma_cookie_t rx_cookie;

	/* store register value for suspend/resume */
	u32 control;
	u32 ssel;

	struct clk *sclk;
	unsigned long sclk_rate;
};

struct adi_spi_device {
	bool dma;
	u32 control;
};

/*
 * Check if watermark condition is enabled for a fifo queue
 * */
static bool adi_spi_flow_control_on_rx(struct adi_spi_controller *spi_ctrl) 
{
	u32 flow_control_rx_status = ~(spi_ctrl->regs->control & SPI_CTL_FCCH);

	if ((flow_control_rx_status) \
		&& (spi_ctrl->regs->control & SPI_CTL_FCEN))
	    return true;
	
	return false;

}

static bool adi_spi_flow_control_on_tx(struct adi_spi_controller *spi_ctrl) 
{
	u32 flow_control_tx_status = (spi_ctrl->regs->control & SPI_CTL_FCCH);

	if ((flow_control_tx_status) \
		&& (spi_ctrl->regs->control & SPI_CTL_FCEN))
	    return true;
	
	return false;
}

/*
 * Checks if the appropriate watermark condition is met for a tx fifo.
 * */
static bool adi_spi_check_watermark_condition_tx(struct adi_spi_controller *spi_ctrl) 
{
	u32 watermark_status, tfifo_status;

	watermark_status = (spi_ctrl->regs->control & SPI_CTL_FCWM);
	tfifo_status = (spi_ctrl->regs->status & SPI_STAT_TFS);
	if(!adi_spi_flow_control_on_tx(spi_ctrl))
	    return false; //nothing to do
	
	switch(watermark_status) {
	    case SPI_CTL_FIFO0:
		    dev_info(spi_ctrl->dev, "tfifo_status:%08x\n",tfifo_status);
		    pr_info("%s:%d\n",__func__,__LINE__);
		    if (SPI_STAT_TFIFO_EMPTY == tfifo_status)
			return true;
	    
		    break;
	    case SPI_CTL_FIFO1:
		    pr_info("%s:%d\n",__func__,__LINE__);
		    if (SPI_STAT_TFIFO_75 == tfifo_status)
			return true;
		    
		    break;
	    case SPI_CTL_FIFO2:
		    pr_info("%s:%d\n",__func__,__LINE__);
		    if (SPI_STAT_TFIFO_50 == tfifo_status)
			return true;
		    
		    break;
	    default:
		    dev_err(spi_ctrl->dev, "got unknown SPI watermark value:%08x\n",watermark_status);
		    break;
	}
	return false;
}

/*
 * Checks if the appropriate watermark condition is met for a rx fifo.
 * */
static bool adi_spi_check_watermark_condition_rx(struct adi_spi_controller *spi_ctrl) 
{
	u32 watermark_status, rfifo_status;

	watermark_status = (spi_ctrl->regs->control & SPI_CTL_FCWM);
	rfifo_status = (spi_ctrl->regs->status & SPI_STAT_RFS);
    
	if(!adi_spi_flow_control_on_rx(spi_ctrl))
	    return false; //nothing to do

	switch(watermark_status) {
	    case SPI_CTL_FIFO0:
		    dev_info(spi_ctrl->dev, "rfifo_status:%08x\n",rfifo_status);
		    pr_info("%s:%d\n",__func__,__LINE__);
		    if (SPI_STAT_RFIFO_FULL == rfifo_status)
			return true;

		    break;
	    case SPI_CTL_FIFO1:
		    pr_info("%s:%d\n",__func__,__LINE__);
		    if (SPI_STAT_TFIFO_75 == rfifo_status)
			return true;
		    
		    break;
	    case SPI_CTL_FIFO2:
		    pr_info("%s:%d\n",__func__,__LINE__);
		    if (SPI_STAT_TFIFO_50 == rfifo_status)
			return true;
		    
		    break;
	    default:
		    dev_err(spi_ctrl->dev, "got unknown SPI watermark value:%08x\n",watermark_status);
		    break;
    }
    return false;
}


static void adi_spi_disable(struct adi_spi_controller *drv)
{
	u32 ctl;

	ctl = ioread32(&drv->regs->control);
	ctl &= ~SPI_CTL_EN;
	iowrite32(ctl, &drv->regs->control);
}

static void adi_spi_dma_terminate(struct adi_spi_controller *drv)
{
	dmaengine_terminate_sync(drv->spi_ctrl->dma_tx);
	dmaengine_terminate_sync(drv->spi_ctrl->dma_rx);
}

/* Calculate the SPI_CLOCK register value based on input HZ */
static u32 hz_to_spi_clock(u32 sclk, u32 speed_hz)
{
	u32 spi_clock = sclk / speed_hz;

	if (spi_clock)
		spi_clock--;
	return spi_clock;
}

static void adi_spi_u8_write(struct adi_spi_controller *drv,
	struct spi_transfer *xfer)
{
	size_t i;

	if (!spi_controller_is_slave(drv->spi_ctrl))
		pr_info("[%d]master status:%08x\n",__LINE__,drv->regs->status);
	else
		pr_info("[%d]slave status:%08x\n",__LINE__,drv->regs->status);
	for (i = 0; i < xfer->len; ++i) {
		//wait for tfifo 
		while (ioread32(&drv->regs->status) & SPI_STAT_TFF)
			cpu_relax();
		
		iowrite32(*(u8 *)(xfer->tx_buf + i), &drv->regs->tfifo);
		while (ioread32(&drv->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		
		pr_info("%s:%d\n",__func__,__LINE__);
	}
}

static void adi_spi_u16_write(struct adi_spi_controller *drv,
	struct spi_transfer *xfer)
{
	size_t i;

	if (!spi_controller_is_slave(drv->spi_ctrl))
		pr_info("[%d]master status:%08x\n",__LINE__,drv->regs->status);
	else
		pr_info("[%d]slave status:%08x\n",__LINE__,drv->regs->status);
	for (i = 0; i < xfer->len; ++i) {
		iowrite32(*(u16 *)(xfer->tx_buf + 2*i), &drv->regs->tfifo);
		while (ioread32(&drv->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		pr_info("%s:%d\n",__func__,__LINE__);
	}
}

static void adi_spi_u32_write(struct adi_spi_controller *drv,
	struct spi_transfer *xfer)
{
	size_t i;

	if (!spi_controller_is_slave(drv->spi_ctrl))
		pr_info("[%d]master status:%08x\n",__LINE__,drv->regs->status);
	else
		pr_info("[%d]slave status:%08x\n",__LINE__,drv->regs->status);
	for (i = 0; i < xfer->len; ++i) {
		iowrite32(*(u32 *)(xfer->tx_buf + 4*i), &drv->regs->tfifo);
		while (ioread32(&drv->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		pr_info("%s:%d\n",__func__,__LINE__);
	}
}

static void adi_spi_u8_read(struct adi_spi_controller *drv,
	struct spi_transfer *xfer)
{
	size_t i;

	if (!spi_controller_is_slave(drv->spi_ctrl))
		pr_info("[%d]master status:%08x\n",__LINE__,drv->regs->status);
	else
		pr_info("[%d]slave status:%08x\n",__LINE__,drv->regs->status);
	for (i = 0; i < xfer->len; ++i) {
		while (ioread32(&drv->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		*(u8 *)(xfer->rx_buf + i) = ioread32(&drv->regs->rfifo);
	}
}

static void adi_spi_u16_read(struct adi_spi_controller *drv,
	struct spi_transfer *xfer)
{
	size_t i;

	if (!spi_controller_is_slave(drv->spi_ctrl))
		pr_info("[%d]master status:%08x\n",__LINE__,drv->regs->status);
	else
		pr_info("[%d]slave status:%08x\n",__LINE__,drv->regs->status);
	for (i = 0; i < xfer->len; ++i) {
		while (ioread32(&drv->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		*(u16 *)(xfer->rx_buf + 2*i) = ioread32(&drv->regs->rfifo);
	}
}

static void adi_spi_u32_read(struct adi_spi_controller *drv,
	struct spi_transfer *xfer)
{
	size_t i;

	
	if (!spi_controller_is_slave(drv->spi_ctrl))
		pr_info("[%d]master status:%08x\n",__LINE__,drv->regs->status);
	else
		pr_info("[%d]slave status:%08x\n",__LINE__,drv->regs->status);
	for (i = 0; i < xfer->len; ++i) {
		while (ioread32(&drv->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		*(u32 *)(xfer->rx_buf + 4*i) = ioread32(&drv->regs->rfifo);
	}
}

static void adi_spi_u8_duplex(struct adi_spi_controller *drv,
	struct spi_transfer *xfer)
{
	size_t i;

	if (!spi_controller_is_slave(drv->spi_ctrl))
		pr_info("[%d]master status:%08x\n",__LINE__,drv->regs->status);
	else
		pr_info("[%d]slave status:%08x\n",__LINE__,drv->regs->status);
	for (i = 0; i < xfer->len; ++i) {
		iowrite32(*(u8 *)(xfer->tx_buf + i), &drv->regs->tfifo);
		while (ioread32(&drv->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		*(u8 *)(xfer->rx_buf + i) = ioread32(&drv->regs->rfifo);
	}
}

static void adi_spi_u16_duplex(struct adi_spi_controller *drv,
	struct spi_transfer *xfer)
{
	size_t i;

	if (!spi_controller_is_slave(drv->spi_ctrl))
		pr_info("[%d]master status:%08x\n",__LINE__,drv->regs->status);
	else
		pr_info("[%d]slave status:%08x\n",__LINE__,drv->regs->status);
	for (i = 0; i < xfer->len; ++i) {
		iowrite32(*(u16 *)(xfer->tx_buf + 2*i), &drv->regs->tfifo);
		while (ioread32(&drv->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		*(u16 *)(xfer->rx_buf + 2*i) = ioread32(&drv->regs->rfifo);
	}
}

static void adi_spi_u32_duplex(struct adi_spi_controller *drv,
	struct spi_transfer *xfer)
{
	size_t i;

	if (!spi_controller_is_slave(drv->spi_ctrl))
		pr_info("[%d]master status:%08x\n",__LINE__,drv->regs->status);
	else
		pr_info("[%d]slave status:%08x\n",__LINE__,drv->regs->status);
	for (i = 0; i < xfer->len; ++i) {
		iowrite32(*(u32 *)(xfer->tx_buf + 4*i), &drv->regs->tfifo);
		while (ioread32(&drv->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		*(u32 *)(xfer->rx_buf + 4*i) = ioread32(&drv->regs->rfifo);
	}
}

static const struct adi_spi_transfer_ops adi_spi_transfer_ops_u8 = {
	.write  = adi_spi_u8_write,
	.read   = adi_spi_u8_read,
	.duplex = adi_spi_u8_duplex,
};

static const struct adi_spi_transfer_ops adi_spi_transfer_ops_u16 = {
	.write  = adi_spi_u16_write,
	.read   = adi_spi_u16_read,
	.duplex = adi_spi_u16_duplex,
};

static const struct adi_spi_transfer_ops adi_spi_transfer_ops_u32 = {
	.write  = adi_spi_u32_write,
	.read   = adi_spi_u32_read,
	.duplex = adi_spi_u32_duplex,
};

static int adi_spi_pio_xfer(struct spi_controller *spi_ctrl, struct spi_device *spi,
	struct spi_transfer *xfer)
{
	struct adi_spi_controller *drv = spi_controller_get_devdata(spi_ctrl);

	
	if (!xfer->tx_buf) {
		iowrite32(0, &drv->regs->tx_control);
		if(spi_controller_is_slave(spi_ctrl))
			iowrite32(SPI_RXCTL_REN, &drv->regs->rx_control);
		else
			iowrite32(SPI_RXCTL_REN | SPI_RXCTL_RTI, &drv->regs->rx_control);
		drv->ops->read(drv, xfer);
	} else if (!xfer->rx_buf) {
		iowrite32(SPI_RXCTL_REN, &drv->regs->rx_control);
		if(spi_controller_is_slave(spi_ctrl))
			iowrite32(SPI_TXCTL_TEN, &drv->regs->tx_control);
		else
			iowrite32(SPI_TXCTL_TEN | SPI_TXCTL_TTI, &drv->regs->tx_control);
		drv->ops->write(drv, xfer);
	} else {
		iowrite32(SPI_RXCTL_REN, &drv->regs->rx_control);
		if(spi_controller_is_slave(spi_ctrl))
			iowrite32(SPI_TXCTL_TEN, &drv->regs->tx_control);
		else
			iowrite32(SPI_TXCTL_TEN | SPI_TXCTL_TTI, &drv->regs->tx_control);
		drv->ops->duplex(drv, xfer);
	}


	iowrite32(0, &drv->regs->tx_control);
	iowrite32(0, &drv->regs->rx_control);
	return 0;
}

/*
 * Disable both paths and alert spi core that this transfer is done
 */
static void adi_spi_rx_dma_isr(void *data)
{
	struct adi_spi_controller *drv = data;

	struct dma_tx_state state;
	enum dma_status status;


	if (!spi_controller_is_slave(drv->spi_ctrl))
		pr_info("[%d]master status:%08x\n",__LINE__,drv->regs->status);
	else
		pr_info("[%d]slave status:%08x\n",__LINE__,drv->regs->status);

	while(!(drv->regs->status & SPI_STAT_SPIF));

	status = dmaengine_tx_status(drv->spi_ctrl->dma_rx, drv->rx_cookie, &state);
	if (status == DMA_ERROR)
		dev_err(&drv->spi_ctrl->dev, "spi rx dma error\n");
	else
		dev_info(&drv->spi_ctrl->dev, "dmaengine status:%d\n",status);

	iowrite32(0, &drv->regs->tx_control);
	iowrite32(0, &drv->regs->rx_control);
	spi_finalize_current_transfer(drv->spi_ctrl);
}


/*
 * Disable tx path and enable rx path for dual/quad modes
 */
static void adi_spi_tx_dma_isr(void *data)
{
	struct adi_spi_controller *drv = data;
	struct dma_tx_state state;
	enum dma_status status;

	status = dmaengine_tx_status(drv->spi_ctrl->dma_tx, drv->tx_cookie, &state);
	if (status == DMA_ERROR)
		dev_err(&drv->spi_ctrl->dev, "spi tx dma error\n");

	iowrite32(0, &drv->regs->tx_control);
	
	if (!spi_controller_is_slave(drv->spi_ctrl))
		pr_info("[%d]master status:%08x\n",__LINE__,drv->regs->status);
	else
		pr_info("[%d]slave status:%08x\n",__LINE__,drv->regs->status);

	if (drv->cur_transfer->rx_buf)
		dma_async_issue_pending(drv->spi_ctrl->dma_rx);
	 else
		spi_finalize_current_transfer(drv->spi_ctrl);
}

/*
 * Disable both paths and alert spi core that this transfer is done
 */
static void adi_spi_slave_tx_dma_isr(void *data)
{
	struct adi_spi_controller *drv = data;

	struct dma_tx_state state;
	enum dma_status status;


	dev_info(drv->dev,"[%d] TFIFO:%08x\n",__LINE__,ioread32(&drv->regs->tfifo));
	iowrite32(SPI_TXCTL_TEN | SPI_TXCTL_TDR_NF , &drv->regs->tx_control);
	pr_info("[%d]slave status:%08x\n",__LINE__,drv->regs->status);

	status = dmaengine_tx_status(drv->spi_ctrl->dma_tx, drv->tx_cookie, &state);
	if (status == DMA_ERROR)
		dev_err(&drv->spi_ctrl->dev, "spi tx dma error\n");
	else
		dev_info(&drv->spi_ctrl->dev, "dmaengine status:%d\n",status);

	iowrite32(0, &drv->regs->tx_control);
	iowrite32(0, &drv->regs->rx_control);
	spi_finalize_current_transfer(drv->spi_ctrl);
}

/*
 * Disable tx path and enable rx path for dual/quad modes
 */
static void adi_spi_slave_rx_dma_isr(void *data)
{
	struct adi_spi_controller *drv = data;
	struct dma_tx_state state;
	enum dma_status status;

	status = dmaengine_tx_status(drv->spi_ctrl->dma_rx, drv->rx_cookie, &state);
	if (status == DMA_ERROR)
		dev_err(&drv->spi_ctrl->dev, "spi rx dma error\n");

	iowrite32(0, &drv->regs->rx_control);

	dev_info(drv->dev,"[%d] RFIFO:%08x\n",__LINE__,ioread32(&drv->regs->rfifo));
	pr_info("[%d]slave status:%08x\n",__LINE__,drv->regs->status);

	if (drv->cur_transfer->tx_buf)
		dma_async_issue_pending(drv->spi_ctrl->dma_tx);
	 else
		spi_finalize_current_transfer(drv->spi_ctrl);
}


static int adi_spi_dma_write(struct spi_controller *spi_ctrl, struct spi_transfer *xfer) 
{
	
	struct adi_spi_controller *drv = spi_controller_get_devdata(spi_ctrl);
	struct dma_async_tx_descriptor *tx_desc = dmaengine_prep_slave_sg(spi_ctrl->dma_tx, xfer->tx_sg.sgl,
		xfer->tx_sg.nents, DMA_MEM_TO_DEV, 0);
	
	if (!xfer->tx_buf) 
		return 0;

	if (!tx_desc) {
		dev_err(drv->dev, "Unable to allocate TX DMA descriptor\n");
		return -1;
	}

	// transmission callback disabled in case rx also needs to be done
	// since spi_finalize_current_transfer can only be called once
	//
	// but this is not the case for a spi-sub device (its inversed)
	if (!spi_controller_is_slave(spi_ctrl)) {
		if ((!xfer->rx_buf)) {
			tx_desc->callback = adi_spi_tx_dma_isr;
			tx_desc->callback_param = drv;
		}

	} else {
		tx_desc->callback = adi_spi_slave_tx_dma_isr;
		tx_desc->callback_param = drv;
	}

	drv->tx_cookie = dmaengine_submit(tx_desc);
	
	dev_info(drv->dev,"flushing final tx transaction\n");
	dev_info(drv->dev,"cur_transfer_tx:%08x\n",*(uint32_t *)drv->cur_transfer->tx_buf);
	dma_async_issue_pending(spi_ctrl->dma_tx);
	dev_info(drv->dev,"[%d] status:%08x\n",__LINE__,drv->regs->status);
	dev_info(drv->dev,"[%d] TFIFO:%08x\n",__LINE__,ioread32(&drv->regs->tfifo));

	if(spi_controller_is_slave(spi_ctrl)) 
		iowrite32(SPI_TXCTL_TEN | SPI_TXCTL_TDR_EMPTY , &drv->regs->tx_control);
	else
		iowrite32(SPI_TXCTL_TEN | SPI_TXCTL_TTI | SPI_TXCTL_TDR_NF | SPI_TXCTL_TWCEN,
			&drv->regs->tx_control);
		
	return 0;
}


static int adi_spi_dma_read(struct spi_controller *spi_ctrl, struct spi_transfer *xfer) 
{
	
	struct adi_spi_controller *drv = spi_controller_get_devdata(spi_ctrl);
	struct dma_async_tx_descriptor *rx_desc = dmaengine_prep_slave_sg(spi_ctrl->dma_rx, xfer->rx_sg.sgl,
		xfer->rx_sg.nents, DMA_DEV_TO_MEM, 0);
	
	if (!xfer->rx_buf)
		return 0;

	dev_info(drv->dev,"[%d] status:%08x\n",__LINE__,drv->regs->status);
	if (!rx_desc) {
		dev_err(drv->dev, "Unable to allocate RX DMA descriptor\n");
		return -1;
	}

	if (!spi_controller_is_slave(spi_ctrl)) {
		rx_desc->callback = adi_spi_rx_dma_isr;
		rx_desc->callback_param = drv;
	} else if (!xfer->tx_buf) {
		rx_desc->callback = adi_spi_slave_rx_dma_isr;
		rx_desc->callback_param = drv;
	}

	drv->rx_cookie = dmaengine_submit(rx_desc);
	
	dev_info(drv->dev,"[%d] RFIFO:%08x\n",__LINE__,ioread32(&drv->regs->rfifo));
	if(spi_controller_is_slave(spi_ctrl)) {
		iowrite32(SPI_RXCTL_REN | SPI_RXCTL_RDR_NE | SPI_RXCTL_RDR_DIS,
			&drv->regs->rx_control);
	} else {
		iowrite32(SPI_RXCTL_REN | SPI_RXCTL_RTI | SPI_RXCTL_RDR_NE | SPI_RXCTL_RWCEN | SPI_RXCTL_RDR_DIS,
			&drv->regs->rx_control);
	}
	
	dev_info(drv->dev,"flushing final rx transaction\n");
	dev_info(drv->dev,"cur_transfer_rx:%08x\n",*(uint32_t *)drv->cur_transfer->rx_buf);
	dma_async_issue_pending(spi_ctrl->dma_rx);
	return 0;
} 

static int adi_spi_error_check(struct spi_controller *spi_ctrl, int ret)
{
	struct adi_spi_controller *drv = spi_controller_get_devdata(spi_ctrl);
	
	if(!ret)
		return ret;

	adi_spi_dma_terminate(drv);
	return -ENOENT;
}

static int adi_spi_dma_xfer(struct spi_controller *spi_ctrl, struct spi_device *spi,
	struct spi_transfer *xfer)
{
	
	struct adi_spi_controller *drv = spi_controller_get_devdata(spi_ctrl);
	
	if(spi_controller_is_slave(spi_ctrl)) {
		adi_spi_error_check(spi_ctrl, adi_spi_dma_read(spi_ctrl, xfer));
		adi_spi_error_check(spi_ctrl, adi_spi_dma_write(spi_ctrl, xfer));
			
	} else {
		dev_info(drv->dev, "spi ctrl not a slave! getting ready for duplex\n");
		adi_spi_error_check(spi_ctrl, adi_spi_dma_write(spi_ctrl, xfer));
		adi_spi_error_check(spi_ctrl, adi_spi_dma_read(spi_ctrl, xfer));
	}

	dev_info(drv->dev,"[%d] status:%08x\n",__LINE__,drv->regs->status);
	//let callbacks trigger completion
	return 1;
}

static bool adi_spi_can_dma(struct spi_controller *spi_ctrl, struct spi_device *spi,
	struct spi_transfer *xfer)
{
	struct adi_spi_device *chip = spi_get_ctldata(spi);

	if (chip->dma)
		return true;
	return false;
}

static int adi_spi_transfer_one(struct spi_controller *spi_ctrl, struct spi_device *spi,
	struct spi_transfer *xfer)
{
	struct adi_spi_controller *drv = spi_controller_get_devdata(spi_ctrl);
	u32 cr;

	drv->cur_transfer = xfer;

	cr = ioread32(&drv->regs->control) & ~SPI_CTL_MIOM;

	if (xfer->rx_nbits == SPI_NBITS_QUAD || xfer->tx_nbits == SPI_NBITS_QUAD)
		cr |= SPI_CTL_MIO_QUAD;
	else if (xfer->rx_nbits == SPI_NBITS_DUAL || xfer->tx_nbits == SPI_NBITS_DUAL)
		cr |= SPI_CTL_MIO_DUAL;

	iowrite32(cr, &drv->regs->control);
	dev_info(drv->dev,"%d can dma:%d",__LINE__,adi_spi_can_dma(spi_ctrl,spi,xfer));

	if (adi_spi_can_dma(spi_ctrl, spi, xfer))
		return adi_spi_dma_xfer(spi_ctrl, spi, xfer);
	return adi_spi_pio_xfer(spi_ctrl, spi, xfer);
}

/*
 * Settings like clock speed and bits per word are assumed to be the same for all
 * transfers in a message. tx_nbits and rx_nbits can change, however
 */
static int adi_spi_prepare_message(struct spi_controller *spi_ctrl, struct spi_message *msg)
{
	struct adi_spi_controller *drv = spi_controller_get_devdata(spi_ctrl);
	struct adi_spi_device *chip = spi_get_ctldata(msg->spi);
	struct dma_slave_config dma_config = {0};
	struct spi_transfer *xfer;
	int ret;
	u32 cr, cr_width;
	u32 words;

	xfer = list_first_entry(&msg->transfers, struct spi_transfer, transfer_list);
	words = DIV_ROUND_UP(xfer->bits_per_word, 8);
	iowrite32(hz_to_spi_clock(drv->sclk_rate, xfer->speed_hz), &drv->regs->clock);

	switch (words) {
	case 1:
		cr_width = SPI_CTL_SIZE08;
		drv->ops = &adi_spi_transfer_ops_u8;
		break;
	case 2:
		cr_width = SPI_CTL_SIZE16;
		drv->ops = &adi_spi_transfer_ops_u16;
		break;
	case 4:
		cr_width = SPI_CTL_SIZE32;
		drv->ops = &adi_spi_transfer_ops_u32;
		break;
	default:
		dev_err(&spi_ctrl->dev, "invalid word size in incoming message\n");
		return -EINVAL;
	}

	cr = chip->control;
	cr |= cr_width | SPI_CTL_EN;
	cr &= ~SPI_CTL_SOSI;
	iowrite32(cr, &drv->regs->control);

	dma_config.direction = DMA_MEM_TO_DEV;
	dma_config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	dma_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	dma_config.src_maxburst = words;
	dma_config.dst_maxburst = words;
	ret = dmaengine_slave_config(spi_ctrl->dma_tx, &dma_config);
	if (ret) {
		dev_err(drv->dev, "tx dma slave config failed: %d\n", ret);
		return ret;
	}

	dma_config.direction = DMA_DEV_TO_MEM;
	ret = dmaengine_slave_config(spi_ctrl->dma_rx, &dma_config);
	if (ret) {
		dev_err(drv->dev, "rx dma slave config failed: %d\n", ret);
		return ret;
	}

	return 0;
}

static int adi_spi_unprepare_message(struct spi_controller *spi_ctrl, struct spi_message *msg)
{
	struct adi_spi_controller *drv = spi_controller_get_devdata(spi_ctrl);

	adi_spi_disable(drv);
	return 0;
}

static int adi_spi_setup(struct spi_device *spi)
{
	struct adi_spi_device *chip;
	struct device_node *np = spi->dev.of_node;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	spi_set_ctldata(spi, chip);

	chip->dma = false;
	if (of_property_read_bool(np, "adi,enable-dma"))
		chip->dma = true;

	chip->control = 0;
	if (of_property_read_bool(np, "adi,open-drain-mode"))
		chip->control |= SPI_CTL_ODM;

	if (of_property_read_bool(np, "adi,psse"))
		chip->control |= SPI_CTL_PSSE;

	if (spi->mode & SPI_CPOL)
		chip->control |= SPI_CTL_CPOL;
	if (spi->mode & SPI_CPHA)
		chip->control |= SPI_CTL_CPHA;
	if (spi->mode & SPI_LSB_FIRST)
		chip->control |= SPI_CTL_LSBF;
	
	if (spi_controller_is_slave(spi->controller)) {
		chip->control |= SPI_CTL_EMISO; 
		chip->control &= ~SPI_CTL_MSTR;
		pr_info("control:%08x\n",chip->control);
	} else {
		chip->control |= SPI_CTL_MSTR;
		chip->control &= ~SPI_CTL_ASSEL;
		pr_info("control:%08x\n",chip->control);
	}	

	return 0;
}

static void adi_spi_cleanup(struct spi_device *spi)
{
	struct adi_spi_device *chip = spi_get_ctldata(spi);

	if (!chip)
		return;

	spi_set_ctldata(spi, NULL);
	kfree(chip);
}


/*
 * When full, read a word until watermark condition is no longer met
 * */
static int handle_rx_overrun(struct adi_spi_controller *drv){
	/*
	 * Check RWC value, if > 0; read until it decrements to 0.
	 * This should flush fifo out;
	 * */
	
	u32 ctl, rfifo_count, word_size, buf, i;
	word_size = (drv->regs->control & SPI_CTL_SIZE);
	rfifo_count = drv->regs->rwc;

	word_size = DIV_ROUND_UP(word_size,8);
	
	pr_info("rfifo_count:%d\n",rfifo_count);
	
	if (rfifo_count) {
		for (i = 0; i < rfifo_count; ++i) {
		

			if (ioread32(&drv->regs->status) & SPI_STAT_RFE)
				pr_info("exiting prematurely - rfifo empty!\n");
			
			buf = 0;	
			switch(word_size) {
				case 1: 
					buf = ioread8(&drv->regs->rfifo);
					break;
				case 2: 
					buf = ioread16(&drv->regs->rfifo);
					break;
				case 4: 
					buf = ioread32(&drv->regs->rfifo);
					break;
			}
			pr_info("buf:%08x\n",buf);
		}
	} else {
		pr_info("rfifo count was 0! resetting spi\n");
		
		ctl = ioread32(&drv->regs->control);
		ctl &= ~SPI_CTL_EN;
		iowrite32(ctl, &drv->regs->control);
		pr_info("spi disabled\n");
		
		ctl = ioread32(&drv->regs->control);
		ctl |= SPI_CTL_EN;
		iowrite32(ctl, &drv->regs->control);
		pr_info("spi reset\n");
	}

	pr_info("fifo flushed\n");
	rfifo_count = drv->regs->rwc;
	pr_info("rfifo_count:%d\n",rfifo_count);
	return rfifo_count;
}

static irqreturn_t spi_irq_err(int irq, void *dev_id)
{
	struct adi_spi_controller *drv = dev_id;
	u32 status;

	status = ioread32(&drv->regs->status);
	dev_err(drv->dev, "spi error irq, status = 0x%x\n", status);

	pr_info("%s:%d\n",__func__,__LINE__);
	if (adi_spi_check_watermark_condition_tx(drv))
		pr_info("tx watermark met!\n");
	if (adi_spi_check_watermark_condition_rx(drv)) {
		pr_info("rx watermark met!\n");
		handle_rx_overrun(drv);
		goto irq_handled;	
	}

	pr_info("%s:%d\n",__func__,__LINE__);
	pr_info("tx ctl:%08x\nrx ctl:%08x\n",drv->regs->tx_control,drv->regs->rx_control);

	iowrite32(status, &drv->regs->status);

	iowrite32(0, &drv->regs->tx_control);
	iowrite32(0, &drv->regs->rx_control);
	adi_spi_disable(drv);
	adi_spi_dma_terminate(drv);

irq_handled:
	return IRQ_HANDLED;
}

static const struct of_device_id adi_spi_of_match[] = {
	{
		.compatible = "adi,spi3",
	},
	{},
};
MODULE_DEVICE_TABLE(of, adi_spi_of_match);

static int adi_spi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spi_controller *spi_ctrl;
	struct adi_spi_controller *drv;
	struct resource *mem, *res;
	struct clk *sclk;
	int ret;
	bool slave_mode = false;
	u32 control, delay = 0;

	sclk = devm_clk_get(dev, "spi");
	if (IS_ERR(sclk)) {
		dev_err(dev, "can not get spi clock\n");
		return PTR_ERR(sclk);
	}

	slave_mode = of_property_read_bool(dev->of_node, "spi-slave");

	if(!slave_mode) {
		pr_info("master mode!");
		spi_ctrl = devm_spi_alloc_master(dev, sizeof(*drv));
	} else {
		pr_info("slave mode!");
		spi_ctrl = devm_spi_alloc_slave(dev, sizeof(*drv));
	}

	if (!spi_ctrl) {
		dev_err(dev, "can not allocate spi controller\n");
		return -ENOMEM;
	}
	
	platform_set_drvdata(pdev, spi_ctrl);

	/* the mode bits supported by this driver */
	spi_ctrl->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LSB_FIRST |
				SPI_TX_DUAL | SPI_TX_QUAD |
				SPI_RX_DUAL | SPI_RX_QUAD;

//	if(slave_mode)
//		spi_ctrl->mode_bits &= ~SPI_CPHA; 

	spi_ctrl->dev.of_node = dev->of_node;
	spi_ctrl->bus_num = -1;
	spi_ctrl->num_chipselect = 4;
	spi_ctrl->use_gpio_descriptors = true;
	spi_ctrl->cleanup = adi_spi_cleanup;
	spi_ctrl->setup = adi_spi_setup;
	spi_ctrl->prepare_message = adi_spi_prepare_message;
	spi_ctrl->unprepare_message = adi_spi_unprepare_message;
	spi_ctrl->transfer_one = adi_spi_transfer_one;
	spi_ctrl->can_dma = adi_spi_can_dma;
	spi_ctrl->bits_per_word_mask = BIT(32 - 1) | BIT(16 - 1) | BIT(8 - 1);

	drv = spi_controller_get_devdata(spi_ctrl);
	drv->spi_ctrl = spi_ctrl;
	drv->sclk = sclk;
	drv->sclk_rate = clk_get_rate(sclk);
	drv->dev = dev;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	drv->regs = devm_ioremap_resource(dev, mem);
	if (IS_ERR(drv->regs)) {
		dev_err(dev, "Could not map spiv3 memory, check device tree\n");
		return PTR_ERR(drv->regs);
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(dev, "can not get spi error irq\n");
		return -ENXIO;
	}
	ret = devm_request_irq(dev, res->start, spi_irq_err,
			0, "SPI ERROR", drv);
	if (ret) {
		dev_err(dev, "can not request spi error irq\n");
		return ret;
	}
	
	if(!slave_mode)
		iowrite32(SPI_CTL_MSTR, &drv->regs->control);

	if (slave_mode) {
		control |= SPI_CTL_EMISO; 
		control &= ~SPI_CTL_MSTR;
		control |= SPI_CTL_CPHA; //toggle from start
//		control &= ~SPI_CTL_CPHA; //toggle from middle 
	} else {
		//delay |= SPI_DLY_LAGX;		
		//delay |= SPI_DLY_STOP; //maximum delay

		control |= SPI_CTL_MSTR;
		control |= SPI_CTL_CPHA; //toggle from start
		control &= ~SPI_CTL_ASSEL; 
	}

	iowrite32(control, &drv->regs->control);
	iowrite32(delay, &drv->regs->delay);
	if (!slave_mode)
		iowrite32(0x0000FE00, &drv->regs->ssel);
	//iowrite32(0x0, &drv->regs->delay);
	iowrite32(SPI_IMSK_SET_ROM, &drv->regs->emaskst);

	spi_ctrl->dma_tx = dma_request_chan(dev, "tx");
	if (!spi_ctrl->dma_tx) {
		dev_err(dev, "Could not get TX DMA channel\n");
		return -ENOENT;
	}

	spi_ctrl->dma_rx = dma_request_chan(dev, "rx");
	if (!spi_ctrl->dma_rx) {
		dev_err(dev, "Could not get RX DMA channel\n");
		ret = -ENOENT;
		goto err_free_tx_dma;
	}

	ret = clk_prepare_enable(drv->sclk);
	if (ret) {
		dev_err(dev, "Could not enable SPI clock\n");
		goto err_free_rx_dma;
	}

	ret = devm_spi_register_controller(dev, spi_ctrl);
	if (ret) {
		dev_err(dev, "can not  register spi master\n");
		goto err_free_rx_dma;
	}



	dev_info(dev,"probe control:%08x\n",control);
	dev_info(dev,"control:%08x\n",drv->regs->control);
	dev_info(dev, "registered ADI SPI controller\n");
	return ret;

err_free_rx_dma:
	dma_release_channel(spi_ctrl->dma_rx);

err_free_tx_dma:
	dma_release_channel(spi_ctrl->dma_tx);

	return ret;
}

static int adi_spi_remove(struct platform_device *pdev)
{
	struct spi_controller *spi_ctrl = platform_get_drvdata(pdev);
	struct adi_spi_controller *drv = spi_controller_get_devdata(spi_ctrl);

	adi_spi_disable(drv);
	clk_disable_unprepare(drv->sclk);
	dma_release_channel(spi_ctrl->dma_tx);
	dma_release_channel(spi_ctrl->dma_rx);
	return 0;
}

static int __maybe_unused adi_spi_suspend(struct device *dev)
{
	struct spi_controller *spi_ctrl = dev_get_drvdata(dev);

	return spi_controller_suspend(spi_ctrl);
}

static int __maybe_unused adi_spi_resume(struct device *dev)
{
	struct spi_controller *spi_ctrl = dev_get_drvdata(dev);

	return spi_controller_resume(spi_ctrl);
}

static const struct dev_pm_ops adi_spi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(adi_spi_suspend, adi_spi_resume)
};

MODULE_ALIAS("platform:adi-spi3");
static struct platform_driver adi_spi_driver = {
	.driver	= {
		.name	= "adi-spi3",
		.pm     = &adi_spi_pm_ops,
		.of_match_table = adi_spi_of_match,
	},
	.probe      = adi_spi_probe,
	.remove		= adi_spi_remove,
};

module_platform_driver(adi_spi_driver);

MODULE_DESCRIPTION("Analog Devices SPI3 controller driver");
MODULE_AUTHOR("Scott Jiang <Scott.Jiang.Linux@gmail.com>");
MODULE_LICENSE("GPL v2");
