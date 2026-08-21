#!/usr/bin/env bash
# Build mbedTLS 3.6.2 into a local prefix and run the Tesla crypto
# known-answer unit test (tools/test/test_tesla_crypto.c) against it.
#
# Why build mbedTLS from source? Phase 0's whole point is validating the
# mbedTLS 3.x API port (ESP-IDF 5.4.1 vendors 3.6.2). Ubuntu 24.04's
# libmbedtls-dev is still 2.28.x, so we pin the exact 3.6.2 release and build
# it locally. Works on any Linux host (CI ubuntu-latest) and in WSL.
#
# Requires: gcc, make, curl.
set -euo pipefail

MBEDTLS_VERSION=3.6.2
MBEDTLS_URL="https://github.com/Mbed-TLS/mbedtls/archive/refs/tags/v${MBEDTLS_VERSION}.tar.gz"
# SHA-256 of the v3.6.2 source tarball (verify on every run so a partial or
# tampered download is never trusted).
MBEDTLS_SHA256="f4a876b1f6921ad0aefb445f974ef62414d33928640b2c45555c5e64a196a1a8"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WORK="${REPO_ROOT}/build/tesla-host-test"
SRC="${WORK}/mbedtls-${MBEDTLS_VERSION}"
PREFIX="${WORK}/mbedtls-prefix"
TARBALL="${WORK}/mbedtls-${MBEDTLS_VERSION}.tar.gz"

mkdir -p "${WORK}"

if [ ! -f "${TARBALL}" ]; then
    echo "==> Downloading mbedTLS ${MBEDTLS_VERSION}"
    curl -fL --retry 3 -o "${TARBALL}" "${MBEDTLS_URL}"
fi

echo "==> Verifying mbedTLS tarball checksum"
if ! echo "${MBEDTLS_SHA256}  ${TARBALL}" | sha256sum -c - >/dev/null; then
    echo "mbedTLS tarball checksum mismatch; delete ${TARBALL} and re-run" >&2
    exit 1
fi

if [ ! -d "${SRC}" ]; then
    echo "==> Extracting mbedTLS ${MBEDTLS_VERSION}"
    tar xzf "${TARBALL}" -C "${WORK}"
fi

if [ ! -f "${PREFIX}/lib/libmbedcrypto.a" ]; then
    echo "==> Building mbedTLS ${MBEDTLS_VERSION}"
    make -C "${SRC}" lib -j"$(nproc)"
    mkdir -p "${PREFIX}/lib" "${PREFIX}/include"
    cp "${SRC}/library"/libmbedcrypto.a "${SRC}/library"/libmbedtls.a \
       "${SRC}/library"/libmbedx509.a "${PREFIX}/lib/"
    cp -r "${SRC}/include/mbedtls" "${SRC}/include/psa" "${PREFIX}/include/"
fi

echo "==> Compiling test"
PROTOCOMP="${REPO_ROOT}/components/tesla-protocol"
cc -std=c99 -Wall -Wextra \
   -I "${PROTOCOMP}" \
   -I "${PROTOCOMP}/generated" \
   -I "${PROTOCOMP}/nanopb" \
   -I "${PREFIX}/include" \
   "${SCRIPT_DIR}/test_tesla_crypto.c" \
   "${PROTOCOMP}/crypto.c" \
   "${PROTOCOMP}/session.c" \
   "${PROTOCOMP}/protobuf_build.c" \
   "${PROTOCOMP}"/generated/*.pb.c \
   "${PROTOCOMP}/nanopb/pb_common.c" \
   "${PROTOCOMP}/nanopb/pb_decode.c" \
   "${PROTOCOMP}/nanopb/pb_encode.c" \
   -L "${PREFIX}/lib" -lmbedcrypto -lmbedtls -lmbedx509 \
   -o "${WORK}/test_tesla_crypto"

echo "==> Running test"
"${WORK}/test_tesla_crypto"
