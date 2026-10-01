# Linux 6.18 for the Huawei Qingyun L410 (Kirin 990)

这是 Linux 稳定版 v6.18.54 加上华为擎云 L410（KLVU-WDU0B，麒麟 990，Mali-G76）支持的内核源码。
分支 `l410-6.18` 里，上游 v6.18.54 之后的每个提交都是 L410 的改动，`git log v6.18.54..` 就能看全。

配套的 Debian 安装脚本、系统配置和各硬件的调试记录在
[l410-mainline](https://github.com/liuwang97/l410-mainline)。

This branch is upstream Linux v6.18.54 plus the drivers, device tree fixups and configuration
needed to run it on the Huawei Qingyun L410 laptop. Everything L410-specific is in
`git log v6.18.54..`.

## 能用的硬件

| 部件 | 驱动 | 状态 |
|---|---|---|
| CPU 4×A55 + 2×A76 中核 + 2×A76 大核 | PSCI、`hisi-hwvote` cpufreq、EAS、温控 | 正常，三簇调频，90 °C 降频 |
| 存储 UFS 3.1 | `ufs-kirin`（新写的 glue） | HS-G4 ×2，顺序读约 1.6 GB/s |
| USB 3.1 + 板载 hub、摄像头 | `dwc3-kirin990`、`phy-kirin990-usb3` | 正常 |
| 有线网 RTL8168 | `pcie-kport` + 主线 `r8169` | 正常 |
| WiFi / 蓝牙 Hi1103 | `drivers/staging/hi110x`（厂商驱动移植） | WPA2/WPA3 可用，蓝牙可用 |
| 显示 eDP 2160×1440 | `kirin990-dss`（接管 UEFI 点亮的管线） | 60 Hz，关屏整条链断电，硬件光标 |
| GPU Mali-G76 MP16 | 主线 Panfrost | Mesa panfrost，OpenGL ES 3.1 / OpenGL 3.1 |
| 声卡 Hi6405 + 2×TAS2562 | `sound/soc/hisilicon/hi6405`（厂商驱动移植） | 扬声器、耳机、内置麦克风 |
| 键盘、触控板、电池、合盖 | i2c-hid、`huawei-echub` EC 驱动 | 正常 |
| 系统睡眠 | `kirin990-sr` | s2idle 默认可用；deep 唤醒后会冷启动 |

不支持：DP/HDMI 输出、指纹、硬件视频编解码。

## 编译

在 x86-64 的 Debian/Ubuntu（WSL 2 也行）上交叉编译：

```bash
sudo apt install gcc-aarch64-linux-gnu make bc bison flex libssl-dev libelf-dev \
    device-tree-compiler cpio kmod curl ccache
git clone -b l410-6.18 https://github.com/liuwang97/linux-l410.git
cd linux-l410
l410/build.sh -o ../l410-build
```

产物在 `../l410-build/bundle/`：`Image`、`l410.dtb`、`initrd.img`、`modules.tar.gz`、`boot.cfg`。
装到机器上用 l410-mainline 里的 `boot/install-kernel.sh`，见那边的 `docs/install.md`。

## L410 专有的文件

- `l410/configs/*.config`：按文件名顺序合并到 arm64 defconfig 之上的配置片段。
- `l410/dt/l410-firmware.dts`：L410 固件交给厂商 4.19 内核的设备树（从麒麟的 `/sys/firmware/fdt` 导出）。
  这台机器的设备树沿用厂商的私有绑定，没有改写成主线风格。
- `l410/dt/fixups.d/*.dtsi`：6.18 的驱动需要的改动，按文件名顺序叠加在固件设备树上。
- `l410/initramfs/init`：busybox 写的最小 initramfs，按 `root=` 挂根分区后切换过去。
- `l410/firmware/`：编进内核的 USB 3.1 PHY 固件。
- `l410/build.sh`：上面这些加内核本身，一次编出可以安装的整套文件。
