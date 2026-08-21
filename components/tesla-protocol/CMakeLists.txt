# tesla-protocol: Tesla vehicle-command protocol support.
#
# Vendors the Apache-2.0 .proto schema files (protos/, pinned in
# protos/VERSION), the nanopb 0.4.x runtime (nanopb/), and the generated nanpob
# bindings (generated/, produced by tools/gen_proto.py — committed so a clean
# checkout builds without protoc). Implements the crypto/session layer
# (crypto.c, session.c), the protobuf message builders (protobuf_build.c), and
# the Phase 2 handshake/command/response logic, on the ESP-IDF mbedTLS 3.x
# component.

idf_component_register(
    SRCS
        "crypto.c"
        "session.c"
        "protobuf_build.c"
        # Phase 2 generates the VCSEC + handshake message set only; the
        # Infotainment protos (car_server/vehicle/managed_charging) join in
        # Phase 4 with a vendored google/protobuf/timestamp binding.
        "generated/errors.pb.c"
        "generated/keys.pb.c"
        "generated/signatures.pb.c"
        "generated/universal_message.pb.c"
        "generated/vcsec.pb.c"
        "nanopb/pb_common.c"
        "nanopb/pb_decode.c"
        "nanopb/pb_encode.c"
    INCLUDE_DIRS
        "."
        "generated"
        "nanopb"
    REQUIRES
        mbedtls
    PRIV_REQUIRES
        # The generated bindings are self-contained, but pb_common's assertions
        # pull in nothing extra; no private deps currently.
)
