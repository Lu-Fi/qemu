#!/bin/sh
# Start the released Linux binaries on a bare distribution image.
# Runs inside the container, with the unpacked bundle at /dist.
set -eu
export DEBIAN_FRONTEND=noninteractive

# glibc 2.38+ distributions renamed this package (time_t transition)
glib=libglib2.0-0
apt-get update -qq
apt-cache show libglib2.0-0t64 >/dev/null 2>&1 && glib=libglib2.0-0t64
apt-get install -y -qq --no-install-recommends \
  "$glib" libpixman-1-0 libslirp0 libncursesw6 libzstd1 libgmp10 libfdt1 \
  zlib1g libatomic1 >/dev/null

. /etc/os-release
echo "== $PRETTY_NAME, $(uname -m), glibc $(ldd --version | sed -n '1s/.* //p')"
for b in qemu-system-mipsel qemu-system-riscv32; do
  "/dist/$b" --version >/tmp/v
  echo "$b: $(head -n 1 /tmp/v)"
done
n=$(/dist/qemu-system-mipsel -machine help | grep -c ingenic)
echo "ingenic machines: $n"
[ "$n" -eq 11 ]
