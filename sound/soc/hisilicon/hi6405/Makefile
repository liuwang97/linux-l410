# SPDX-License-Identifier: GPL-2.0
ccflags-y += -I$(src) -I$(src)/include -I$(src)/codec -I$(src)/slimbus
# the vendor sources guard their Hi6405 paths with this symbol
ccflags-y += -DCONFIG_SND_SOC_DA_COMBINE_V5=1

snd-soc-hi6405-y := hi6405_main.o hi6405_ctrl.o hi64xx_irq.o asp_pcm.o hi6405_card.o \
	hi64xx_resmgr.o hi64xx_mbhc.o hi64xx_utils.o

snd-soc-hi6405-y += slimbus/slimbus.o slimbus/slimbus_drv.o slimbus/csmi.o \
	slimbus/cps_bm.o slimbus/slimbus_6405.o slimbus/slimbus_64xx.o \
	slimbus/slimbus_utils.o slimbus/slimbus_pm.o

snd-soc-hi6405-y += codec/codec_pm.o codec/codec_probe.o codec/da_combine_v5.o \
	codec/kcontrol.o codec/path_widget.o codec/pga_widget.o \
	codec/resource_widget.o codec/single_drv_widget.o \
	codec/single_pga_widget.o codec/single_switch_route.o \
	codec/single_switch_widget.o codec/switch_widget.o \
	codec/switch_widget_utils.o codec/utils.o codec/codec_debugfs.o

obj-$(CONFIG_SND_SOC_HI6405_L410) += snd-soc-hi6405.o
