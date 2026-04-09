#!/usr/bin/env bash
set -euo pipefail

# Configuration
DEFCONFIG_DEBUG="l760_defconfig"
DEFCONFIG_RELEASE="l760_release_defconfig"

OUTDIR="kout"
ARCH="arm"
TOOLCHAIN_DIR="$(pwd)/toolchain/arm-eabi-4.8/bin"
CROSS_COMPILE="arm-eabi-"
JOBS="$(nproc)"

# Mode
MODE="${1:-debug}"

if [[ "$MODE" == "user" ]]; then
    echo -e "\n\n===== Compilation du noyau de production =====\n"
    DEFCONFIG="$DEFCONFIG_RELEASE"
elif [[ "$MODE" == "check" ]]; then
    DEFCONFIG="$DEFCONFIG_DEBUG"
else
    echo -e "\n\n===== Compilation du noyau de débogage =====\n"
    DEFCONFIG="$DEFCONFIG_DEBUG"
fi

echo "Initialisation..."

# Environnement
export PATH="$TOOLCHAIN_DIR:$PATH"
export ARCH
export CROSS_COMPILE

mkdir -p "$OUTDIR"

# Configuration kernel
echo "Configuration ($DEFCONFIG)..."
make O="$OUTDIR" "$DEFCONFIG"

# Mode analyse statique
if [[ "$MODE" == "check" ]]; then
    echo "Analyse statique..."
    shift || true
    CHECK_TARGET="$*"
    ./scripts/check_code.sh "$DEFCONFIG" "$CHECK_TARGET"
    exit 0
fi

# Build
echo "Compilation..."
find "$OUTDIR/arch" -name "*.dtb" -delete 2>/dev/null || true

make O="$OUTDIR" \
     -j"$JOBS" \
     HOSTCFLAGS="-fcommon -Wno-error"

# Boot image
echo "Génération boot.img..."
./scripts/mkbootimg.sh "$OUTDIR" "$ARCH"

echo "Compilation terminée."
echo "Image disponible : ./bootimg/boot.img"
