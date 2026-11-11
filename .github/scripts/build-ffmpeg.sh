#!/bin/sh
# build-ffmpeg.sh PREFIX [CROSS_PREFIX]
#
# A static libavcodec and libavutil with only the H.264 and HEVC decoders,
# for the A1 video decoder model (--enable-ingenic-vdec-avcodec). Linked
# statically so the release binaries need no FFmpeg on the user's machine.
# CROSS_PREFIX (e.g. x86_64-w64-mingw32-) cross-builds for Windows.
set -eu

VER=7.1.5
SHA256=de668509caf9e35e3cd162473441fdb29538c6d96ed080292b3cf9e6fc5d558f

PREFIX=$1
CROSS=${2:-}
WORK=$(mktemp -d)

curl -fsSL --retry 5 --retry-all-errors --retry-delay 3 -o "$WORK/ffmpeg.tar.xz" \
    "https://ffmpeg.org/releases/ffmpeg-$VER.tar.xz"
if command -v sha256sum >/dev/null; then
    echo "$SHA256  $WORK/ffmpeg.tar.xz" | sha256sum -c -
else
    echo "$SHA256  $WORK/ffmpeg.tar.xz" | shasum -a 256 -c -
fi
tar -C "$WORK" -xJf "$WORK/ffmpeg.tar.xz"
cd "$WORK/ffmpeg-$VER"

set -- --prefix="$PREFIX" --enable-static --disable-shared --enable-pic \
    --disable-programs --disable-doc --disable-network --disable-autodetect \
    --disable-everything --disable-avdevice --disable-avformat \
    --disable-swresample --disable-swscale --disable-avfilter \
    --enable-decoder=h264 --enable-decoder=hevc
if [ -n "$CROSS" ]; then
    set -- "$@" --enable-cross-compile --cross-prefix="$CROSS" \
        --target-os=mingw32 --arch=x86_64
fi
./configure "$@"
make -j"$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)"
make install

# QEMU's meson asks pkg-config without --static: move the libraries a
# static link needs (libm, pthread, bcrypt on Windows) into Libs
for pc in "$PREFIX"/lib/pkgconfig/libavcodec.pc "$PREFIX"/lib/pkgconfig/libavutil.pc; do
    priv=$(sed -n 's/^Libs.private: *//p' "$pc")
    sed -i.bak -e "s|^\(Libs: .*\)$|\1 $priv|" -e '/^Libs.private:/d' "$pc"
    rm -f "$pc.bak"
done
rm -rf "$WORK"
