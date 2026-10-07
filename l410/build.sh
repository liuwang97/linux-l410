#!/bin/bash
# Cross-compile the Huawei Qingyun L410 (Kirin 990) kernel on an x86-64 Debian or Ubuntu host
# (WSL 2 works). Produces everything the L410 boots from /boot/l410/ on its Debian root.
#
#   l410/build.sh [-o DIR] [-j JOBS] [-n NAME] [-f FRAGMENT]... [--no-modules]
#
#   -o DIR       build directory (default: <tree>/../l410-build); objects in DIR/obj,
#                installable result in DIR/bundle
#   -j JOBS      parallel jobs (default: nproc)
#   -n NAME      release suffix: 6.18.54-l410-NAME (default: no suffix, 6.18.54-l410)
#   -f FRAGMENT  extra Kconfig fragment merged after l410/configs/*.config (repeatable)
#   --no-modules build Image only
#
# Inputs, all in this tree:
#   l410/configs/*.config      Kconfig fragments, merged in name order on top of arm64 defconfig
#   l410/dt/l410-firmware.dts  device tree the L410 firmware passes to the vendor 4.19 kernel
#   l410/dt/fixups.d/*.dtsi    changes the 6.18 drivers need on top of it, included in name order
#   l410/initramfs/init        busybox init that mounts root= and switches to it
#
# Host packages (Debian/Ubuntu names):
#   gcc-aarch64-linux-gnu make bc bison flex libssl-dev libelf-dev device-tree-compiler cpio kmod
#   curl ca-certificates (ccache is used when installed)
#
# bundle/: Image l410.dtb initrd.img modules.tar.gz boot.cfg kver config
# Install it with install-kernel.sh from the l410-mainline repository, or by hand: copy Image,
# l410.dtb, initrd.img and boot.cfg to /boot/l410/ and unpack modules.tar.gz in /.
set -e
TREE=$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)
OUT=$TREE/../l410-build
JOBS=$(nproc)
NAME=
MODULES=1
FRAGS=()
while [ $# -gt 0 ]; do
	case $1 in
	-o) OUT=$2; shift 2 ;;
	-j) JOBS=$2; shift 2 ;;
	-n) NAME=$2; shift 2 ;;
	-f) FRAGS+=("$(readlink -f "$2")"); shift 2 ;;
	--no-modules) MODULES=0; shift ;;
	-h|--help) sed -n '2,29p' "$0"; exit 0 ;;
	*) echo "unknown option $1 (see --help)" >&2; exit 2 ;;
	esac
done
mkdir -p "$OUT"
OUT=$(cd "$OUT" && pwd)
OBJ=$OUT/obj
B=$OUT/bundle
mkdir -p "$OBJ" "$B"

for t in aarch64-linux-gnu-gcc make bc bison flex dtc cpio depmod curl; do
	command -v $t > /dev/null || { echo "missing host tool: $t (see --help)" >&2; exit 1; }
done
export ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-
CC=aarch64-linux-gnu-gcc
command -v ccache > /dev/null && CC="ccache aarch64-linux-gnu-gcc"
MK=(make -C "$TREE" O="$OBJ" -j"$JOBS" CC="$CC" LOCALVERSION="${NAME:+-$NAME}")

# configuration: arm64 defconfig + the L410 fragments, regenerated on every build
"${MK[@]}" -s defconfig
"$TREE"/scripts/kconfig/merge_config.sh -m -O "$OBJ" "$OBJ/.config" \
	"$TREE"/l410/configs/*.config "${FRAGS[@]}" > "$OUT/merge.log"
"${MK[@]}" -s olddefconfig

"${MK[@]}" Image $([ $MODULES = 1 ] && echo modules)
KVER=$(cat "$OBJ/include/config/kernel.release")
cp "$OBJ/arch/arm64/boot/Image" "$B/Image"
cp "$OBJ/.config" "$B/config"
echo "$KVER" > "$B/kver"

rm -f "$B/modules.tar.gz"
if [ $MODULES = 1 ]; then
	rm -rf "$OUT/modroot"
	"${MK[@]}" -s INSTALL_MOD_PATH="$OUT/modroot" INSTALL_MOD_STRIP=1 modules_install
	rm -f "$OUT/modroot/lib/modules/$KVER/build" "$OUT/modroot/lib/modules/$KVER/source"
	tar -czf "$B/modules.tar.gz" -C "$OUT/modroot" "lib/modules/$KVER"
fi

# device tree: the firmware's tree plus the fixups
{
	echo '/dts-v1/;'
	echo "/include/ \"$TREE/l410/dt/l410-firmware.dts\""
	for f in "$TREE"/l410/dt/fixups.d/*.dtsi; do echo "/include/ \"$f\""; done
} > "$OUT/l410.dts"
dtc -q -I dts -O dtb -o "$B/l410.dtb" "$OUT/l410.dts"

# initramfs: a static busybox and l410/initramfs/init. The package is Debian's
# busybox-static 1:1.37.0-6+b9 (arm64), fetched from snapshot.debian.org by its SHA-1 and
# checked against its SHA-256; BUSYBOX_DEB=<file> uses a local copy instead.
BB_SHA1=6d31276d7d9ae8fd1fd27b9b368bef89e7677d62
BB_SHA256=c833be48abfa16bc19c4966ec93e289ff1ce5d2f1476cad3a57bd105378cd15c
BB=${BUSYBOX_DEB:-$OUT/busybox-static_1.37.0-6+b9_arm64.deb}
if [ ! -f "$BB" ]; then
	curl -fL --retry 3 -o "$BB.part" "https://snapshot.debian.org/file/$BB_SHA1"
	mv "$BB.part" "$BB"
fi
[ -n "$BUSYBOX_DEB" ] || echo "$BB_SHA256  $BB" | sha256sum -c --quiet -
IR=$OUT/initramfs
rm -rf "$IR"
mkdir -p "$IR/bin"
dpkg_x() { # extract ./usr/bin/busybox from the .deb without dpkg (ar + data.tar.*)
	local d; d=$(mktemp -d)
	(cd "$d" && ar x "$1" && tar -xf data.tar.* ./usr/bin/busybox)
	mv "$d/usr/bin/busybox" "$IR/bin/busybox"
	rm -rf "$d"
}
dpkg_x "$(readlink -f "$BB")"
install -m 755 "$TREE/l410/initramfs/init" "$IR/init"
(cd "$IR" && find . | cpio -o -H newc --quiet | gzip -9) > "$B/initrd.img"

# GRUB snippet: the l410 menu entry in the Kylin GRUB sources this file from the Debian root.
# efi=noruntime: EFI GetTime faults on this firmware. clk/pd/regulator_ignore_unused: not every
# consumer of the firmware's clocks, power domains and supplies has a driver yet.
# l410_deadman=0: the kernel takes over the AP watchdog WDT0 the firmware leaves running and
# stops it (also the default since the takeover fix; l410_deadman=<seconds> arms it as a
# bring-up deadman instead).
CMDLINE="root=UUID=@ROOT_UUID@ ro rootwait l410.mode=root ignore_loglevel printk.devkmsg=on panic=10 nokaslr efi=noruntime log_buf_len=16M clk_ignore_unused pd_ignore_unused regulator_ignore_unused console=tty0 l410_deadman=0"
cat > "$B/boot.cfg" << EOF
echo 'L410 kernel $KVER'
linux /boot/l410/Image $CMDLINE
initrd /boot/l410/initrd.img
devicetree /boot/l410/l410.dtb
EOF
echo "bundle: $B ($KVER)"
ls -l "$B"
