# tesla-protocol: Tesla vehicle-command protocol support.
#
# Vendors the Apache-2.0 .proto schema files (protos/, pinned in
# protos/VERSION), the nanopb 0.4.x runtime (nanopb/), and implements the
# crypto/session layer (crypto.c, session.c) on the ESP-IDF mbedTLS 3.x
# component.
#
# The nanopb bindings generated from protos/ are committed to generated/ (like
# dbc_generated.{c,h}) so a clean checkout builds without protoc/nanopb
# installed. They arrive in Phase 2 of the integration plan, which is when the
# first protobuf messages are built; the vendored protos + runtime are already
# wired into this component.
#
# gen_proto.py (the optional codegen path mirroring tools/gen_dbc.py) lands
# with the generated/ commit.

idf_component_register(
    SRCS
        "crypto.c"
        "session.c"
        "nanopb/pb_common.c"
        "nanopb/pb_decode.c"
        "nanopb/pb_encode.c"
    INCLUDE_DIRS
        "."
        "nanopb"
    REQUIRES
        mbedtls
)
