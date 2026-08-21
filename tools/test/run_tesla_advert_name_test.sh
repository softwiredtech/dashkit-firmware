#!/usr/bin/env bash
# Build (or reuse) a host mbedTLS 3.6.2 prefix and run the Tesla
# advertisement-name matcher unit test (tools/test/test_tesla_advert_name.c)
# against main/tesla/tesla_advert_name.c.
#
# Reuses the exact mbedTLS prefix that run_tesla_crypto_test.sh builds, so
# running both never rebuilds mbedTLS twice. The matcher is pure C, but the
# test derives a legacy ad name from a VIN using mbedTLS SHA-1 (the same hash
# family the firmware uses to check VIN identity from Phase 2 on).
#
# Requires: gcc, make, curl.
# Optional: a VIN as argv to print the exact names your fake beacon must
# broadcast (also validated against the matcher in-test).
set -euo pipefail

MBEDTLS_VERSION=3.6.2
MBEDTLS_URL="https://github.com/Mbed-TLS/mbedtls/archive/refs/tags/v${MBEDTLS_VERSION}.tar.gz"
# SHA-256 of the v3.6.2 source tarball (must match run_tesla_crypto_test.sh).
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
    cp "${SRC}/library"/libmbedcrypto.a "${PREFIX}/lib/"
    cp -r "${SRC}/include/mbedtls" "${SRC}/include/psa" "${PREFIX}/include/"
fi

echo "==> Compiling test"
cc -std=c99 -Wall -Wextra \
   -I "${REPO_ROOT}/main" \
   -I "${REPO_ROOT}/main/tesla" \
   -I "${PREFIX}/include" \
   "${SCRIPT_DIR}/test_tesla_advert_name.c" \
   "${REPO_ROOT}/main/tesla/tesla_advert_name.c" \
   -L "${PREFIX}/lib" -lmbedcrypto \
   -o "${WORK}/test_tesla_advert_name"

echo "==> Running test"
"${WORK}/test_tesla_advert_name" "$@"