#!/bin/sh

# Build the manylinux wheel-build image.
# Usage: build-docker.sh [ARCH]   where ARCH is x86_64 (default) or aarch64
ARCH="${1:-x86_64}"

# the build context is this directory, so run from here regardless of the caller's cwd
cd "$(dirname "$(realpath "$0")")" || exit 1

docker build \
       --build-arg ARCH="${ARCH}" \
       --tag rai-manylinux \
       --network host \
       -f Dockerfile . #&> build.log #--no-cache
