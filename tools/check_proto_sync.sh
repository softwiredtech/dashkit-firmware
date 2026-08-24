#!/usr/bin/env bash
# CI check for the vendored Tesla protocol schemas (PR #58 review):
#   1. The committed protos/ match upstream teslamotors/vehicle-command at the
#      commit pinned in components/tesla-protocol/protos/VERSION.
#   2. The committed nanopb bindings (components/tesla-protocol/generated/) are
#      current with tools/gen_proto.py (no drift).
#
# Requires: git, tar, python3. Step 2 additionally needs the nanopb + protoc
# python toolchain (pip install nanopb grpcio-tools) unless NANOPB_GENERATOR /
# PROTOC are set; if neither is available the regen check is skipped (the
# vendored-proto comparison in step 1 still runs).
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROTO_DIR="${REPO_ROOT}/components/tesla-protocol/protos"
GENERATED_DIR="${REPO_ROOT}/components/tesla-protocol/generated"
UPSTREAM_REPO="https://github.com/teslamotors/vehicle-command.git"
WORK="${REPO_ROOT}/build/proto-sync"

# Prefer python3 (CI), fall back to python (Windows dev shells).
if command -v python3 >/dev/null 2>&1; then PY=python3; else PY=python; fi

PINNED="$(sed -n 's/^Pinned commit: *//p' "${PROTO_DIR}/VERSION" | tr -d '\r')"
if [ -z "${PINNED}" ]; then
    echo "error: could not read 'Pinned commit:' from ${PROTO_DIR}/VERSION" >&2
    exit 1
fi
echo "==> Pinned upstream commit: ${PINNED}"

# --- 1. Compare vendored protos against upstream at the pin ----------------
UPSTREAM_DIR="${WORK}/vehicle-command"
if [ ! -d "${UPSTREAM_DIR}/.git" ]; then
    mkdir -p "${WORK}"
    git init -q "${UPSTREAM_DIR}"
    git -C "${UPSTREAM_DIR}" remote add origin "${UPSTREAM_REPO}"
fi
# Depth-1 fetch of the pinned SHA materialises that commit's full tree (all
# reachable blobs), so `git archive` below can resolve every file.
git -C "${UPSTREAM_DIR}" fetch --depth 1 origin "${PINNED}"

# Compare every vendored *.proto against its upstream counterpart. Iterating
# per-file (rather than diff -r of the whole dir) avoids false failures from
# upstream protos we deliberately do not vendor (e.g. the Phase-4 Infotainment
# schemas) or vice-versa.
UPSTREAM_PROTOS="${WORK}/upstream/pkg/protocol/protobuf"
rm -rf "${WORK}/upstream"
mkdir -p "${UPSTREAM_PROTOS}"
git -C "${UPSTREAM_DIR}" archive "${PINNED}" pkg/protocol/protobuf \
    | tar -x -C "${WORK}/upstream"

: > "${WORK}/proto.diff"
while IFS= read -r f; do
    if [ ! -f "${UPSTREAM_PROTOS}/${f}" ]; then
        echo "only in ${PROTO_DIR}: ${f}" >> "${WORK}/proto.diff"
        continue
    fi
    if ! diff -q --strip-trailing-cr "${PROTO_DIR}/${f}" "${UPSTREAM_PROTOS}/${f}" >> "${WORK}/proto.diff"; then
        echo "${f}: differs from upstream" >> "${WORK}/proto.diff"
    fi
done < <(find "${PROTO_DIR}" -maxdepth 1 -name '*.proto' -printf '%f\n' | sort)

if [ -s "${WORK}/proto.diff" ]; then
    echo "==> MISMATCH: vendored protos differ from upstream pin ${PINNED}" >&2
    cat "${WORK}/proto.diff" >&2
    echo "    Re-copy from upstream and/or bump protos/VERSION, then regen." >&2
    exit 1
fi
echo "==> OK: protos/ matches upstream @ ${PINNED}"

# --- 2. Regenerate nanopb bindings and assert no drift ---------------------
# Resolve NANOPB_GENERATOR from the installed module if not set otherwise,
# so gen_proto.py uses the standalone generator script rather than the
# protoc-plugin entry point (which cannot run in standalone mode).
if [ -z "${NANOPB_GENERATOR:-}" ] && "${PY}" -c "import nanopb" >/dev/null 2>&1; then
    NANOPB_GENERATOR="$("${PY}" -c 'import nanopb,os;print(os.path.join(os.path.dirname(nanopb.__file__),"generator","nanopb_generator.py"))')"
    export NANOPB_GENERATOR
fi
if [ -z "${NANOPB_GENERATOR:-}" ] \
   && ! command -v nanopb_generator.py >/dev/null 2>&1 \
   && ! "${PY}" -c "import nanopb" >/dev/null 2>&1; then
    echo "!!> nanopb toolchain not available; skipping the regen-drift check."
    echo "    In CI: pip install nanopb grpcio-tools"
    exit 0
fi

"${PY}" "${REPO_ROOT}/tools/gen_proto.py"

# Compare only tracked files: the vendored proto set covers every import of
# the generated schemas, so gen_proto emits exactly the committed bindings.
# git diff also normalizes line endings, so a Windows checkout (CRLF) does not
# false-fail.
if ! git -C "${REPO_ROOT}" diff --quiet -- "${GENERATED_DIR}"; then
    echo "==> MISMATCH: regenerated nanopb bindings differ from committed" >&2
    git -C "${REPO_ROOT}" diff -- "${GENERATED_DIR}" >&2
    echo "    Re-run tools/gen_proto.py and commit the results." >&2
    exit 1
fi
echo "==> OK: generated/ bindings are current with the vendored protos"
