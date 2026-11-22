#!/usr/bin/env bash
# Host tests for the CAN path (plain gcc, no ESP-IDF):
#   test_dbc         the DBC signal engine
#   test_can_stream  the MCP251xFD receive/transmit core against a model of the chip
#   test_can_filter  the app CAN filter against the generated manifest tables
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
OUT="$(mktemp -d)"
trap 'rm -rf "${OUT}"' EXIT

INC=(-I "${SCRIPT_DIR}/shim" -I "${REPO_ROOT}/main/can" -I "${REPO_ROOT}/main/can/dbc" -I "${REPO_ROOT}/main")
SAN=(-fsanitize=address,undefined -fno-sanitize-recover=all -g)

echo "==> DBC engine"
cc -std=gnu11 "${INC[@]}" \
   "${SCRIPT_DIR}/test_dbc.c" \
   "${REPO_ROOT}/main/can/dbc/dbc.c" \
   "${REPO_ROOT}/main/can/dbc/dbc_generated.c" \
   -lm -o "${OUT}/test_dbc"
# Prints one line per check; show them only if something failed.
if "${OUT}/test_dbc" > "${OUT}/test_dbc.log" 2>&1; then
    tail -1 "${OUT}/test_dbc.log"
else
    cat "${OUT}/test_dbc.log"
    exit 1
fi

echo "==> MCP251xFD core"
cc -std=gnu11 -Wall -Wextra -Werror "${SAN[@]}" "${INC[@]}" \
   "${SCRIPT_DIR}/test_can_stream.c" \
   "${REPO_ROOT}/main/can/mcp251xfd_stream.c" \
   -o "${OUT}/test_can_stream"
"${OUT}/test_can_stream"

echo "==> CAN filter"
# -Wno-unused-variable: the log shim compiles the log calls away, leaving the module's TAG unused.
cc -std=gnu11 -Wall -Wextra -Werror -Wno-unused-variable "${SAN[@]}" "${INC[@]}" \
   "${SCRIPT_DIR}/test_can_filter.c" \
   "${REPO_ROOT}/main/can/can_filter.c" \
   "${REPO_ROOT}/main/can/dbc/dbc_generated.c" \
   -o "${OUT}/test_can_filter"
"${OUT}/test_can_filter"

echo "==> CAN stream to several devices"
cc -std=gnu11 -Wall -Wextra -Werror "${SAN[@]}" "${INC[@]}" -I "${REPO_ROOT}/main/ble" \
   "${SCRIPT_DIR}/test_ble_stream.c" \
   "${REPO_ROOT}/main/ble/ble_stream.c" \
   -o "${OUT}/test_ble_stream"
"${OUT}/test_ble_stream"
