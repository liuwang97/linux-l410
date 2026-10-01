# SPDX-License-Identifier: GPL-2.0
#
# HiSilicon Hi110x connectivity host driver (vendor hi1105 code base, ARMPC build).
# The vendor built two images, "plat" (platform/, bfgx/, common/) and "wifi" (wifi/),
# with different feature macros and include orders. Both are linked into one module
# here (_PRE_WLAN_AIO) and keep their own per-object flags.

include $(src)/hi110x-features.mk

# Host-only features of the vendor build that depend on HiSilicon kernel extensions
# (cpufreq/core_ctl requests, the vendor-patched BlueZ core used to close/reopen hci0
# around S3, ...). None of them changes data exchanged with the chip.
hi110x-host-only-off := -D_PRE_FEATURE_PLAT_LOCK_CPUFREQ -D_PRE_CONFIG_S3_HCI_DEV_OPT
HI110X_COMM_FLAGS := $(filter-out $(hi110x-host-only-off),$(HI110X_COMM_FLAGS))

obj-$(CONFIG_HI110X) += hi110x.o

# ---------------------------------------------------------------- platform image
plat-oal := oal_bus_if oal_main oal_mem oal_net oal_hardware oal_schedule oal_aes \
	oal_kernel_file oal_cfg80211 oal_sdio_host oal_hcc_host oal_hcc_bus oal_hcc_test \
	oal_softwdt oal_dft oal_workqueue oal_fsm ftrace \
	pcie/pcie_dbg pcie/pcie_firmware pcie/pcie_host pcie/pcie_linux \
	pcie/edma/pcie_edma_host pcie/edma/pcie_edma_linux \
	pcie/ete/ete_linux pcie/ete/ete_host pcie/ete/ete_comm \
	pcie/chip/comm/pcie_chip pcie/chip/comm/pcie_chiptest pcie/chip/comm/pcie_iatu \
	pcie/chip/comm/pcie_pm \
	pcie/chip/hi1103/pcie_chip_hi1103 pcie/chip/hi1103/pcie_chiptest_hi1103 \
	pcie/chip/hi1103/pcie_iatu_hi1103 pcie/chip/hi1103/pcie_pm_hi1103 \
	pcie/chip/hi1105/pcie_chiptest_hi1105 pcie/chip/hi1105/pcie_iatu_hi1105 \
	pcie/chip/hi1105/pcie_pm_hi1105 \
	pcie/chip/hi1106/pcie_chip_hi1106 pcie/chip/hi1106/pcie_chiptest_hi1106 \
	pcie/chip/hi1106/pcie_iatu_hi1106 pcie/chip/hi1106/pcie_pm_hi1106
plat-frw := frw_event_deploy frw_event_main frw_event_sched frw_ipc_msgqueue frw_main \
	frw_task frw_timer
plat-oam := oam_main oam_alarm oam_event oam_log oam_statistics oam_config \
	oam_linux_netlink oam_rdr
plat-pm := plat_firmware plat_firmware_util plat_pm plat_pm_wlan plat_cali plat_efuse \
	platform_common_clk
plat-board := board board_hi1102 ssi_common ssi_hi1103 ssi_hi1106 board_hi1103
plat-bfgx := bfgx_exception_rst bfgx_data_parse bfgx_platform_msg_handle bfgx_dev \
	bfgx_gnss bfgx_user_ctrl bfgx_uart_cust bfgx_bluez

plat-objs := \
	$(addprefix platform/oal/,$(addsuffix .o,$(plat-oal))) \
	$(addprefix platform/frw/,$(addsuffix .o,$(plat-frw))) \
	$(addprefix platform/oam/,$(addsuffix .o,$(plat-oam))) \
	platform/sdt/sdt_drv.o \
	common/customize/hisi_ini.o common/customize/plat_parse_changid.o \
	common/customize/hisi_conn_nve.o \
	common/external/lpcpu_feature.o \
	common/chr_log/chr_devs.o \
	platform/main/plat_main.o \
	$(addprefix platform/pm/,$(addsuffix .o,$(plat-pm))) \
	platform/driver/plat_sdio.o platform/driver/plat_uart.o \
	$(addprefix bfgx/,$(addsuffix .o,$(plat-bfgx))) \
	$(addprefix platform/board/,$(addsuffix .o,$(plat-board))) \
	common/oneimage/oneimage.o

plat-inc := platform/inc platform/inc/frw platform/inc/oal platform/inc/oal/linux \
	platform/inc/oam platform/inc/pm platform/frw platform/oal platform/oal/pcie \
	platform/oam platform/pm bfgx platform/sdt platform/board platform/driver \
	common/oneimage common/customize common/external common/inc common/chr_log \
	wifi/inc/hd_common wifi/inc wifi/hmac wifi/customize

plat-cflags := $(addprefix -I$(src)/,$(plat-inc)) \
	$(HI110X_COMM_FLAGS) $(HI110X_PLAT_FLAGS) \
	-DPLATFORM_TYPE_FOR_HI110X=0 -DPLATFORM_HI110X_k3v2oem1=1 -DPLATFORM_HI110X_UEDGE=2 \
	-DHI1XX_OS_BUILD_VARIANT_ROOT=1 -DHI1XX_OS_BUILD_VARIANT_USER=2 \
	-DOS_HI1XX_BUILD_VERSION=HI1XX_OS_BUILD_VARIANT_USER \
	-DHAVE_HISI_BT
# User build: the "root" variant turns every chip error into GPIO-SSI register dumps
# (which themselves upset the chip's clocks and the PCIe link), bus-exception memory
# dumps and the factory AT-command ioctl.
# The vendor also set HAVE_HISI_FM/GNSS/IR (/dev/hwfm, /dev/hwgnss, /dev/hwir); the
# laptop has no FM/GNSS antenna or IR blaster, so those host interfaces are left out.

# ---------------------------------------------------------------- wifi image
wifi-hmac := hmac_main hmac_rx_data hmac_tx_amsdu hmac_tx_data hmac_user hmac_vap \
	hmac_mgmt_bss_comm hmac_mgmt_classifier hmac_fsm hmac_mgmt_ap hmac_mgmt_sta_up \
	hmac_mgmt_sta hmac_encap_frame_ap hmac_encap_frame_sta hmac_encap_frame hmac_sme_sta \
	hmac_11i hmac_uapsd hmac_config hmac_protection hmac_frag hmac_crypto_tkip \
	hmac_chan_mgmt hmac_smps hmac_opmode hmac_rx_filter hmac_hcc_adapt hmac_reset \
	hmac_scan hmac_p2p hmac_device hmac_resource hmac_wapi_wpi hmac_wapi_sms4 hmac_wapi \
	hmac_cali_mgmt hmac_cali_mgmt_debug hmac_host_rx hmac_cali_arr_init hmac_degradation \
	hmac_blockack hmac_dfs hmac_psm_ap hmac_dfx hmac_edca_opt hmac_arp_offload \
	hmac_tcp_opt hmac_auto_adjust_freq hmac_roam_alg hmac_roam_connect hmac_roam_main \
	hmac_blacklist hmac_rr_performance hmac_roam_scan \
	hmac_traffic_classify hmac_wmmac hmac_vowifi hmac_11ax hmac_sae hmac_11r \
	hmac_ota_report hmac_11w hmac_11k \
	hmac_btcoex hmac_11v hmac_arp_probe hmac_twt hmac_mbo hmac_hiex hmac_dbac \
	hmac_tx_msdu_dscr hmac_wifi6_self_cure hmac_tx_complete hmac_host_tx_data hmac_hid2d \
	hmac_ht_self_cure hmac_host_al_tx hmac_tcp_ack_buf \
	hmac_package_params hmac_stat hmac_tid hmac_soft_retry hmac_multi_netbuf_amsdu \
	hmac_monitor hmac_nrcoex hmac_mgmt_join hmac_host_performance \
	hmac_csi hmac_tx_switch hmac_ftm \
	mac_device mac_frame mac_user mac_vap mac_regdomain mac_ie mac_resource mac_board \
	mac_data mac_hiex mac_ftm mac_mib
wifi-wal := wal_config wal_config_debug wal_linux_bridge wal_main wal_linux_scan \
	wal_linux_rx_rsp wal_linux_event wal_regdb wal_linux_flowctl wal_linux_ioctl \
	wal_linux_vendor wal_linux_netdev_ops wal_linux_ioctl_debug wal_linux_atcmdsrv \
	wal_config_base wal_linux_cfg80211 wal_dfx wal_linux_cfgvendor wal_cfg_ioctl \
	wal_linux_customize wal_config_alg_cfg

wifi-objs := \
	$(addprefix wifi/wal/,$(addsuffix .o,$(wifi-wal))) \
	wifi/main/main.o wifi/main/wlan_chip.o \
	$(addprefix wifi/hmac/,$(addsuffix .o,$(wifi-hmac))) \
	wifi/hal/host_hal_device.o wifi/hal/host_hal_ring.o wifi/hal/host_hal_main.o \
	$(addprefix wifi/hal/product/hi1106/,host_dscr_1106.o host_mac_1106.o host_irq_1106.o \
		host_main_1106.o host_csi_1106.o host_ftm_1106.o) \
	wifi/hal/product/bisheng/host_main_bisheng.o \
	wifi/chip/1103/wlan_chip_1103.o wifi/chip/1103/hisi_customize_wifi_hi1103.o \
	wifi/chip/1106/wlan_chip_1106.o wifi/chip/1106/hisi_customize_wifi_hi1106.o \
	wifi/chip/1106/hisi_conn_nve_interface.o \
	wifi/customize/hisi_customize_wifi.o

wifi-inc := wifi/inc/hd_common platform/inc platform/board wifi/inc wifi/hmac wifi/wal \
	wifi/hal wifi/hal/product/hi1106 wifi/customize \
	platform/inc/frw platform/inc/oal platform/inc/oal/linux platform/inc/pm \
	platform/inc/oam platform/oam platform/pm platform/oal platform/frw platform/sdt \
	common/customize common/oneimage common/inc common/chr_log bfgx

wifi-cflags := $(addprefix -I$(src)/,$(wifi-inc)) \
	$(HI110X_COMM_FLAGS) $(HI110X_WIFI_FLAGS)

# ---------------------------------------------------------------- module
hi110x-glue := hi110x_main.o hi110x_compat.o hi110x_log.o hi110x_eeprom.o hi110x_stubs.o
hi110x-y := $(hi110x-glue) $(plat-objs) $(wifi-objs)

# CONFIG_ARCH_KIRIN_PCIE was a vendor Kconfig symbol (=y) tested by the C code: the
# chip is attached to the Kirin kport PCIe RC (see hi110x_kport.h).
hi110x-common-cflags := -D_PRE_WLAN_AIO -DCONFIG_ARCH_HI1103_SDIO_DEBUG \
	-DCONFIG_ARCH_KIRIN_PCIE -DCONFIG_HI110X_SDIO_STD_CARD_SUPPORT \
	-DPLATFORM_DEBUG_ENABLE -DPLATFORM_SSI_FULL_LOG \
	-I$(src) -include $(src)/hi110x_compat.h \
	-Wno-missing-prototypes -Wno-missing-declarations -Wno-unused-variable \
	-Wno-unused-function -Wno-unused-but-set-variable -Wno-format -Wno-date-time \
	-Wno-stringop-truncation -Wno-array-bounds -Wno-address -Wno-packed-not-aligned \
	-Wno-missing-field-initializers -Wno-sign-compare -Wno-type-limits \
	-Wno-implicit-fallthrough -Wno-unused-const-variable -Wno-enum-conversion \
	-Wno-stringop-overflow -Wno-restrict -Wno-dangling-pointer -Wno-address-of-packed-member

$(foreach o,$(plat-objs),$(eval CFLAGS_$(o) := $$(hi110x-common-cflags) $$(plat-cflags)))
$(foreach o,$(wifi-objs),$(eval CFLAGS_$(o) := $$(hi110x-common-cflags) $$(wifi-cflags)))
$(foreach o,$(hi110x-glue),$(eval CFLAGS_$(o) := $$(hi110x-common-cflags) $$(plat-cflags)))
