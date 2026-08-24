#!/usr/bin/env bash
# Run the Tesla enrollment test (tools/test/test_tesla_enrollment.c: keypair
# generation + present-key message round-trip) against a pinned host mbedTLS
# 3.6.2 (built by the shared tesla_host_env.sh bootstrap).
#
# Requires: gcc, make, curl.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
. "${SCRIPT_DIR}/tesla_host_env.sh"

echo "==> Compiling test"
cc -std=c99 -Wall -Wextra -Wno-unused-function \
   -I "${PROTOCOMP}" \
   -I "${PROTOCOMP}/generated" \
   -I "${PROTOCOMP}/nanopb" \
   -I "${MBEDTLS_PREFIX}/include" \
   "${SCRIPT_DIR}/test_tesla_enrollment.c" \
   "${PROTOCOMP}/crypto.c" \
   "${PROTOCOMP}/protobuf_build.c" \
   "${PROTOCOMP}"/generated/*.pb.c \
   "${PROTOCOMP}/nanopb/pb_common.c" \
   "${PROTOCOMP}/nanopb/pb_decode.c" \
   "${PROTOCOMP}/nanopb/pb_encode.c" \
   -L "${MBEDTLS_PREFIX}/lib" -lmbedcrypto -lmbedtls -lmbedx509 \
   -o "${MBEDTLS_WORK}/test_tesla_enrollment"

echo "==> Running test"
"${MBEDTLS_WORK}/test_tesla_enrollment"
