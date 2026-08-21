#!/usr/bin/env bash
# Build (or reuse) a host mbedTLS 3.6.2 prefix and run the Tesla Phase 3
# enrollment test (tools/test/test_tesla_enrollment.c): P-256 keypair
# generation and the ToVCSECMessage present-key enrollment builder round-trip,
# against the committed nanopb bindings + crypto/protobuf_build layers.
#
# Reuses the mbedTLS prefix built by run_tesla_crypto_test.sh so running all
# four test scripts never rebuilds mbedTLS twice.
#
# Requires: gcc, make, curl.
set -euo pipefail

MBEDTLS_VERSION=3.6.2
MBEDTLS_URL="https://github.com/Mbed-TLS/mbedtls/archive/refs/tags/v${MBEDTLS_VERSION}.tar.gz"
# SHA-256 of the v3.6.2 source tarball (must match the other run_*.sh scripts).
MBEDTLS_SHA256="f4a876b1f6921ad0aefb445f974ef62414d33928640b2c45555c5e64a196a1a8"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WORK="${REPO_ROOT}/build/tesla-host-test"
SRC="${WORK}/mbedtls-${MBEDTLS_VERSION}"
PREFIX="${WORK}/mbedtls-prefix"
TARBALL="${WORK}/mbedtls-${MBEDTLS_VERSION}.tar.gz"
PROTOCOMP="${REPO_ROOT}/components/tesla-protocol"

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
cc -std=c99 -Wall -Wextra -Wno-unused-function \
   -I "${PROTOCOMP}" \
   -I "${PROTOCOMP}/generated" \
   -I "${PROTOCOMP}/nanopb" \
   -I "${PREFIX}/include" \
   "${SCRIPT_DIR}/test_tesla_enrollment.c" \
   "${PROTOCOMP}/crypto.c" \
   "${PROTOCOMP}/protobuf_build.c" \
   "${PROTOCOMP}"/generated/*.pb.c \
   "${PROTOCOMP}/nanopb/pb_common.c" \
   "${PROTOCOMP}/nanopb/pb_decode.c" \
   "${PROTOCOMP}/nanopb/pb_encode.c" \
   -L "${PREFIX}/lib" -lmbedcrypto -lmbedtls -lmbedx509 \
   -o "${WORK}/test_tesla_enrollment"

echo "==> Running test"
"${WORK}/test_tesla_enrollment"
