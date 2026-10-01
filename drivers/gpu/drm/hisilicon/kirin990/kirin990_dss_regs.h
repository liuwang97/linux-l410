/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * HiSilicon Kirin 990 display subsystem (DSS v510) registers.
 * Offsets are relative to the DSS base (0xe8400000) unless noted.
 */
#ifndef __KIRIN990_DSS_REGS_H__
#define __KIRIN990_DSS_REGS_H__

#include <linux/bits.h>

/* ---- MIPI DSI0 host (Synopsys DW MIPI DSI v1.31 + HiSilicon glue) ---- */
#define DSS_DSI0			0x01000
#define DSI_PWR_UP			0x004
#define DSI_CLKMGR_CFG			0x008
#define DSI_DPI_VCID			0x00c
#define DSI_DPI_COLOR_CODING		0x010
#define DSI_DPI_CFG_POL			0x014
#define DSI_DPI_LP_CMD_TIM		0x018
#define DSI_PCKHDL_CFG			0x02c
#define DSI_MODE_CFG			0x034
#define DSI_VID_MODE_CFG		0x038
#define DSI_VID_PKT_SIZE		0x03c
#define DSI_VID_HSA_TIME		0x048
#define DSI_VID_HBP_TIME		0x04c
#define DSI_VID_HLINE_TIME		0x050
#define DSI_VID_VSA_LINES		0x054
#define DSI_VID_VBP_LINES		0x058
#define DSI_VID_VFP_LINES		0x05c
#define DSI_VID_VACTIVE_LINES		0x060
#define DSI_CMD_MODE_CFG		0x068
#define DSI_GEN_HDR			0x06c
#define DSI_GEN_PLD_DATA		0x070
#define DSI_CMD_PKT_STATUS		0x074
#define DSI_TO_CNT_CFG			0x078
#define DSI_LPCLK_CTRL			0x094
#define DSI_PHY_TMR_LPCLK_CFG		0x098
#define DSI_PHY_TMR_CFG			0x09c
#define DSI_PHY_RSTZ			0x0a0
#define DSI_PHY_IF_CFG			0x0a4
#define DSI_PHY_STATUS			0x0b0
#define DSI_PHY_TST_CTRL0		0x0b4
#define DSI_PHY_TST_CTRL1		0x0b8
#define DSI_PHY_TMR_RD_CFG		0x0f4
#define DSI_PHY_MODE			0x0fc
/* LDI embedded in the DSI block on v510 */
#define MIPI_LDI_CTRL			0x1b8
#define   LDI_EN			BIT(0)
#define MIPI_DSI_CMD_MOD_CTRL		0x1c0
#define MIPI_LDI_FRM_MSK		0x1d4
#define MIPI_LDI_DPI0_HRZ_CTRL2		0x1ec
#define MIPI_LDI_VRT_CTRL2		0x1f0
#define MIPI_LDI_DPI0_HRZ_CTRL3		0x1f4
/* one-hot vertical state of the LDI (vendor dpu_init.h) */
#define MIPI_LDI_VSTATE			0x1fc
#define   LDI_VSTATE_MASK		0x7ff
#define   LDI_VSTATE_IDLE		BIT(0)
#define   LDI_VSTATE_VSW		BIT(1)
#define   LDI_VSTATE_VBP		BIT(2)
#define   LDI_VSTATE_VACTIVE0		BIT(3)
#define   LDI_VSTATE_VFP		BIT(6)
#define DSI_DPHYTX_CTRL			0x228
#define DSI_DPHYTX_TRSTOP_FLAG		0x22c
#define MIPI_LDI_CPU_ITF_INTS		0x248
#define MIPI_LDI_CPU_ITF_INT_MSK	0x24c
#define   LDI_INT_FRM_END		BIT(1)
#define   LDI_INT_UNFLOW		BIT(2)
#define   LDI_INT_VSYNC			BIT(4)
#define   LDI_INT_VACTIVE0_START	BIT(7)
#define   LDI_INT_VACTIVE0_END		BIT(8)

/* ---- AIF (AXI interface), one 0x20 slot per read channel ---- */
#define DSS_AIF0			0x07000
#define DSS_AIF1			0x09000
#define AIF_CH(ch)			((ch) * 0x20)
#define AIF_CH_CTL			0x00
#define AIF_CMD_RELOAD			0x0a00

/* ---- MIF (MMU interface), channel n at 0x20 * (n + 1) ---- */
#define DSS_MIF				0x0a000
#define MIF_ENABLE			0x000
#define MIF_CH(ch)			(0x20 * ((ch) + 1))
#define MIF_CTRL0			0x00
#define MIF_CTRL1			0x04
#define   MIF_CTRL1_BYPASS		BIT(5)
#define MIF_CTRL2			0x08
#define MIF_CTRL3			0x0c
#define MIF_CTRL4			0x10
#define MIF_CTRL5			0x14

/* ---- MCTL: system part and per-overlay control (CTL0 drives OVL0) ---- */
#define DSS_MCTL_SYS			0x10000
#define MCTL_RCH_FLUSH_EN(ch)		(0x100 + 4 * (ch))
#define MCTL_OV0_FLUSH_EN		0x128
#define MCTL_RCH_OV_OEN(ch)		(0x160 + 4 * (ch))
#define   MCTL_OV_OEN_OV0		BIT(8)
#define MCTL_RCH_OV0_SEL		0x180
#define MCTL_RCH_OV0_SEL1		0x190
#define MCTL_RCH_STARTY(ch)		(0x1c0 + 4 * (ch))
#define MCTL_MOD17_STATUS		0x2c4

#define DSS_MCTL_CTL0			0x10800
#define MCTL_CTL_EN			0x000
#define MCTL_CTL_MUTEX			0x004
#define MCTL_CTL_MUTEX_STATUS		0x008
#define MCTL_CTL_MUTEX_ITF		0x00c
#define MCTL_CTL_MUTEX_DBUF		0x010
#define MCTL_CTL_MUTEX_OV		0x018
#define MCTL_CTL_MUTEX_RCH(ch)		(0x030 + 4 * (ch))
#define MCTL_CTL_TOP			0x050
#define MCTL_CTL_FLUSH_STATUS		0x054
#define MCTL_CTL_CLEAR			0x058
#define MCTL_CTL_STATUS			0x068
#define MCTL_CTL_DBG			0x0e0

/* ---- global ---- */
#define DSS_GLB				0x12000
#define GLB_CPU_PDP_INTS		0x224
#define GLB_CPU_PDP_INT_MSK		0x228

/* ---- read channel blocks (relative to the channel's DMA base) ---- */
#define DMA_OFT_X0			0x000
#define DMA_OFT_Y0			0x004
#define DMA_OFT_X1			0x008
#define DMA_OFT_Y1			0x00c
#define DMA_MASK0			0x010
#define DMA_MASK1			0x014
#define DMA_STRETCH_SIZE_VRT		0x018
#define DMA_CTRL			0x01c
#define   DMA_CTRL_FMT(f)		((f) << 3)
#define   DMA_CTRL_MMU_EN		BIT(8)
#define DMA_TILE_SCRAM			0x020
#define DMA_DATA_ADDR0			0x060
#define DMA_STRIDE0			0x064
#define DMA_STRETCH_STRIDE0		0x068
#define DMA_DATA_NUM0			0x06c
#define DMA_CH_RD_SHADOW		0x0d0
#define DMA_CH_CTL			0x0d4
#define   CH_CTL_EN			BIT(0)
#define DFC_BASE			0x100
#define DFC_DISP_SIZE			0x000
#define DFC_PIX_IN_NUM			0x004
#define DFC_DISP_FMT			0x00c
#define DFC_CLIP_CTL_HRZ		0x010
#define DFC_CLIP_CTL_VRZ		0x014
#define DFC_CTL_CLIP_EN			0x018
#define DFC_ICG_MODULE			0x01c
#define DFC_DITHER_ENABLE		0x020
#define DFC_PADDING_CTL			0x024
#define DFC_BITEXT_CTL			0x040
#define DMA_BUF_BASE			0x800
#define DMA_BUF_CTRL			0x000
#define CH_REG_DEFAULT			0xa00

/* DMA / DFC pixel format codes */
#define DMA_FMT_RGB565			0
#define DMA_FMT_ARGB8888		5
#define DMA_FMT_XRGB8888		6
#define DFC_FMT_RGB565			0
#define DFC_FMT_XRGB8888		5
#define DFC_FMT_ARGB8888		6
#define DFC_FMT_BGR565			7
#define DFC_FMT_XBGR8888		12
#define DFC_FMT_ABGR8888		13

/* ---- overlay compositor OVL0 (8 layers) ---- */
#define DSS_OVL0			0x60000
#define OV_SIZE				0x000
#define OV_BG_COLOR_RGB			0x004
#define OV_BG_COLOR_A			0x008
#define OV_DST_STARTPOS			0x00c
#define OV_DST_ENDPOS			0x010
#define OV_GCFG				0x014
#define OV_LAYER(n)			(0x030 + 0x60 * (n))
#define OV_LAYER_POS			0x00
#define OV_LAYER_SIZE			0x04
#define OV_LAYER_PATTERN_RGB		0x18
#define OV_LAYER_PATTERN_A		0x1c
#define OV_LAYER_ALPHA_MODE		0x20
#define OV_LAYER_ALPHA_A		0x24
#define OV_LAYER_CFG			0x28
#define OV_LAYER_PSPOS			0x2c
#define OV_LAYER_PEPOS			0x30
#define OV_NUM_LAYERS			8
#define OV8_BLOCK_SIZE			0x350
#define OV8_REG_DEFAULT			0x358

/* ---- display pipe front (DPP input, DISP_CH0) ---- */
#define DSS_DISP_CH0			0x62000

/* ---- DSS-internal SMMU ---- */
#define DSS_SMMU			0x80000
#define SMMU_SCR			0x000
#define   SMMU_SCR_GLB_BYPASS		BIT(0)
#define SMMU_SMRX_NS(n)			(0x020 + 4 * (n))
#define   SMMU_SMR_BYPASS		BIT(0)

#endif /* __KIRIN990_DSS_REGS_H__ */
