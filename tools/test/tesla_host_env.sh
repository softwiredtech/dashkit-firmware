#!/usr/bin/env bash
# Shared host-test bootstrap, sourced (not executed) by the run_tesla_*_test.sh
# scripts. Downloads (SHA-256-pinned), verifies, extracts, and builds a host
# mbedTLS 3.6.2 into build/tesla-host-test/mbedtls-prefix exactly once, then
# defines MBEDTLS_WORK / MBEDTLS_PREFIX / PROTOCOMP for the caller's compile
# line.
#
# Why build mbedTLS from source? The whole point of these tests is validating
# the mbedTLS 3.x API port (ESP-IDF 5.4.1 vendors 3.6.2). Ubuntu 24.04's
# libmbedtls-dev is still 2.28.x, so we pin the exact 3.6.2 release and build
# it locally. Works on any Linux host (CI ubuntu-latest) and in WSL.
#
# Requires: gcc, make, curl.

MBEDTLS_VERSION=3.6.2
MBEDTLS_URL="https://github.com/Mbed-TLS/mbedtls/archive/refs/tags/v${MBEDTLS_VERSION}.tar.gz"
# SHA-256 of the v3.6.2 source tarball (verify on every run so a partial or
# tampered download is never trusted).
MBEDTLS_SHA256="f4a876b1f6921ad0aefb445f974ef62414d33928640b2c45555c5e64a196a1a8"

# Caller context: the runner scripts set SCRIPT_DIR / REPO_ROOT before sourcing.
MBEDTLS_WORK="${REPO_ROOT}/build/tesla-host-test"
PROTOCOMP="${PROTOCOMP:-${REPO_ROOT}/components/tesla-protocol}"
MBEDTLS_SRC="${MBEDTLS_WORK}/mbedtls-${MBEDTLS_VERSION}"
MBEDTLS_PREFIX="${MBEDTLS_WORK}/mbedtls-prefix"
MBEDTLS_TARBALL="${MBEDTLS_WORK}/mbedtls-${MBEDTLS_VERSION}.tar.gz"

mkdir -p "${MBEDTLS_WORK}"

if [ ! -f "${MBEDTLS_TARBALL}" ]; then
    echo "==> Downloading mbedTLS ${MBEDTLS_VERSION}"
    curl -fL --retry 3 -o "${MBEDTLS_TARBALL}" "${MBEDTLS_URL}"
fi

if [ ! -d "${MBEDTLS_SRC}" ]; then
    echo "==> Verifying mbedTLS tarball checksum"
    if ! echo "${MBEDTLS_SHA256}  ${MBEDTLS_TARBALL}" | sha256sum -c - >/dev/null; then
        echo "mbedTLS tarball checksum mismatch; delete ${MBEDTLS_TARBALL} and re-run" >&2
        exit 1
    fi
    echo "==> Extracting mbedTLS ${MBEDTLS_VERSION}"
    tar xzf "${MBEDTLS_TARBALL}" -C "${MBEDTLS_WORK}"
fi

if [ ! -f "${MBEDTLS_PREFIX}/lib/libmbedcrypto.a" ]; then
    echo "==> Building mbedTLS ${MBEDTLS_VERSION}"
    make -C "${MBEDTLS_SRC}" lib -j"$(nproc)"
    mkdir -p "${MBEDTLS_PREFIX}/lib" "${MBEDTLS_PREFIX}/include"
    cp "${MBEDTLS_SRC}/library"/libmbedcrypto.a "${MBEDTLS_SRC}/library"/libmbedtls.a \
       "${MBEDTLS_SRC}/library"/libmbedx509.a "${MBEDTLS_PREFIX}/lib/"
    cp -r "${MBEDTLS_SRC}/include/mbedtls" "${MBEDTLS_SRC}/include/psa" "${MBEDTLS_PREFIX}/include/"
fi
