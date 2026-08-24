# nanopb/

Vendored nanopb runtime, version **0.4.9.1** (from
`github.com/nanopb/nanopb`, BSD-3-Clause).

The minimal set needed to build the Tesla protobuf messages, per the
integration plan:

- `pb.h`, `pb_common.{c,h}` — common definitions / helper functions
- `pb_decode.{c,h}` — decode path
- `pb_encode.{c,h}` — encode path

No changes from upstream. To re-vendor, download the `nanopb-0.4.9.1` release
and copy these seven files. The `pb_*` objects are only linked into the
firmware once a message actually uses them (Phase 2+).
