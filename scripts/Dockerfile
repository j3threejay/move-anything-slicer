FROM debian:bookworm

RUN dpkg --add-architecture arm64

RUN apt-get update && apt-get install -y \
    gcc-aarch64-linux-gnu \
    binutils-aarch64-linux-gnu \
    make \
    file \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /build

ENV CROSS_PREFIX=aarch64-linux-gnu-
ENV CC=aarch64-linux-gnu-gcc

CMD ["./scripts/build.sh"]
