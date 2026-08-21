#!/usr/bin/env python3
"""Generate nanopb C bindings from the vendored Tesla vehicle-command protos.

Mirrors tools/gen_dbc.py: the generated *.pb.{c,h} are committed to
components/tesla-protocol/generated/ so a clean checkout builds without any
protobuf toolchain. Run this only when the vendored protos or the nanopb
options file change (then commit the regenerated files).

nanopb_generator.py in standalone mode handles one target per input and shells
out to a `protoc` binary itself. We therefore:
  1. create a `protoc` shim on PATH (either a system protoc or grpcio-tools),
  2. run nanopb_generator.py once with every .proto as a target so every file
     is generated, -f for the shared options file, -I to resolve options.

Requires a python environment with `nanopb` (and `grpcio-tools` unless a real
protoc binary is on PATH).

Usage:
    NANOPB_GENERATOR=... python tools/gen_proto.py
"""
import os
import shlex
import shutil
import subprocess
import sys
import tempfile

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROTO_DIR = os.path.join(REPO_ROOT, "components", "tesla-protocol", "protos")
OUT_DIR = os.path.join(REPO_ROOT, "components", "tesla-protocol", "generated")
OPTIONS = os.path.join(PROTO_DIR, "vehicle-command.options")


def find_generator():
    cand = os.environ.get("NANOPB_GENERATOR")
    if cand:
        return cand
    for name in ("nanopb_generator.py", "protoc-gen-nanopb"):
        p = shutil.which(name)
        if p:
            return p
    try:
        import nanopb
        base = os.path.dirname(os.path.abspath(nanopb.__file__))
        cand = os.path.join(base, "generator", "nanopb_generator.py")
        if os.path.exists(cand):
            return cand
    except ImportError:
        pass
    idf = os.environ.get("IDF_PATH")
    if idf:
        cand = os.path.join(idf, "components", "nanopb", "nanopb", "generator",
                            "nanopb_generator.py")
        if os.path.exists(cand):
            return cand
    sys.exit("nanopb generator not found; set NANOPB_GENERATOR or pip install nanopb")


def protoc_cmd():
    if shutil.which("protoc"):
        return None  # real protoc on PATH: no shim needed
    p = os.environ.get("PROTOC")
    if p:
        return [p]
    try:
        import grpc_tools  # noqa: F401
        return [sys.executable, "-m", "grpc_tools.protoc"]
    except ImportError:
        pass
    sys.exit("protoc not found; set PROTOC or pip install grpcio-tools")


def main():
    # Phase 2 generates the VCSEC + handshake message set only. The Infotainment
    # protos (car_server.proto, vehicle.proto, managed_charging.proto) arrive in
    # Phase 4 together with a vendored google/protobuf/timestamp binding; they
    # are excluded here so this commit builds cleanly without that dependency.
    generate = [
        "errors.proto",
        "keys.proto",
        "signatures.proto",
        "universal_message.proto",
        "vcsec.proto",
    ]
    protos = sorted(os.path.join(PROTO_DIR, p)
                    for p in generate
                    if os.path.exists(os.path.join(PROTO_DIR, p)))
    if not protos:
        sys.exit("no .proto files in %s" % PROTO_DIR)

    gen = find_generator()
    pc = protoc_cmd()
    out_tmp = os.path.join(REPO_ROOT, "build", "gen_proto")
    os.makedirs(out_tmp, exist_ok=True)

    env = dict(os.environ)
    if pc is not None:
        # Provide a `protoc` shim on PATH that nanopb_generator shells out to.
        tmp = tempfile.mkdtemp(prefix="gen_proto_")
        bin_dir = os.path.join(tmp, "bin")
        os.makedirs(bin_dir)
        shim = os.path.join(bin_dir, "protoc")
        with open(shim, "w", encoding="utf-8") as f:
            f.write("#!/bin/sh\nexec %s \"$@\"\n" % " ".join(shlex.quote(a) for a in pc))
        os.chmod(shim, 0o755)
        env["PATH"] = bin_dir + os.pathsep + env.get("PATH", "")

    cmd = [sys.executable, gen,
           "-f", OPTIONS, "-I", PROTO_DIR, "-D", out_tmp]
    cmd += protos
    print("Running:", " ".join(cmd))
    subprocess.check_call(cmd, env=env)

    os.makedirs(OUT_DIR, exist_ok=True)
    files = sorted(f for f in os.listdir(out_tmp)
                   if f.endswith(".pb.h") or f.endswith(".pb.c"))
    if not files:
        sys.exit("nanopb produced no .pb.[ch] files")
    for f in files:
        src = os.path.join(out_tmp, f)
        dst = os.path.join(OUT_DIR, f)
        with open(src, "r", encoding="utf-8") as fh:
            content = fh.read().rstrip() + "\n"
        with open(dst, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(content)   # keep nanpob output diff --check clean and LF
        print("generated", dst)

    shutil.copy(OPTIONS, os.path.join(OUT_DIR, "vehicle-command.options"))
    print("Done. Commit the files under", OUT_DIR)


if __name__ == "__main__":
    main()
