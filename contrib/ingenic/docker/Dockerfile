# SPDX-License-Identifier: GPL-2.0-or-later
# The Ingenic machines in a container: qemu-system-mipsel and
# qemu-system-riscv32 built from this tree, laid out as the release
# tarball is, with qemu-bsp.sh as the entry point.
#
#   docker build -f contrib/ingenic/docker/Dockerfile -t qemu-ingenic .
#   docker run --rm -it --init -v "$PWD:/work" -p 2230:2230 qemu-ingenic \
#       -i /work/thingino-cam.bin -V t31x -L 0.0.0.0
#
# See "Docker" in contrib/ingenic/README.md.

FROM debian:trixie AS build

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      build-essential ninja-build python3 python3-venv pkg-config \
      libglib2.0-dev libslirp-dev libpixman-1-dev zlib1g-dev libfdt-dev \
      flex bison libgmp-dev libzstd-dev git ca-certificates \
 && rm -rf /var/lib/apt/lists/*

COPY . /src
WORKDIR /src/build
RUN ../configure \
      --target-list=mipsel-softmmu,riscv32-softmmu \
      --disable-docs --disable-werror \
      --enable-slirp \
 && ninja qemu-system-mipsel qemu-system-riscv32 \
      contrib/plugins/libips.so contrib/plugins/libcoherency.so

RUN mkdir -p /dist/plugins \
 && cp qemu-system-mipsel qemu-system-riscv32 /dist/ \
 && strip /dist/qemu-system-mipsel /dist/qemu-system-riscv32 \
 && cp contrib/plugins/libips.so contrib/plugins/libcoherency.so /dist/plugins/ \
 && cp -r /src/contrib/ingenic /dist/ingenic \
 && rm -rf /dist/ingenic/docker

FROM debian:trixie-slim

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      libglib2.0-0t64 libpixman-1-0 libslirp0 libfdt1 libgmp10 libzstd1 \
      python3 \
 && rm -rf /var/lib/apt/lists/*

COPY --from=build /dist /opt/qemu-ingenic
ENV PATH=/opt/qemu-ingenic:/opt/qemu-ingenic/ingenic:$PATH
RUN qemu-system-mipsel --version && qemu-system-riscv32 --version

WORKDIR /work
ENTRYPOINT ["qemu-bsp.sh"]
CMD ["-h"]
