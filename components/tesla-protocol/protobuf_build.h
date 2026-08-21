/*
 * Thin protobuf message builders over the nanopb bindings generated from the
 * vendored Tesla vehicle-command protos (generated/).
 *
 * This layer owns nothing about cryptography or session state — it only fills
 * and (de)serializes the protocol messages. The session layer (session.c)
 * drives the crypto around these builders; the firmware BLE client
 * (main/tesla/) moves the resulting bytes over the central GATT connection.
 *
 * All functions return 0 on success, -1 on any encode/decode error or a
 * too-small output buffer.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "crypto.h"
#include "universal_message.pb.h"
#include "signatures.pb.h"
#include "vcsec.pb.h"

#ifdef __cplusplus
extern "C" {
#endif

// Runtime payload/response wire bound, matching the RoutableMessage protobuf
// buffer size configured in vehicle-command.options.
#define TESLA_PB_PAYLOAD_MAX 256

// Convenience destination fillers.
void tesla_pb_dest_domain(UniversalMessage_Destination *d, uint32_t domain);
void tesla_pb_dest_route(UniversalMessage_Destination *d,
                         const uint8_t *addr, size_t len);

// Encode a RoutableMessage carrying a SessionInfoRequest for `domain`, with
// the client public key and a caller-chosen 16-byte `challenge` (must equal
// the RoutableMessage.uuid used on the wire so the vehicle can authenticate
// the response). `routing` is a fresh 16-byte connection address.
int tesla_pb_encode_handshake(uint32_t domain,
                              const uint8_t client_pub[TESLA_PUBKEY_LEN],
                              const uint8_t routing[16],
                              const uint8_t challenge[16],
                              uint8_t *out, size_t out_cap, size_t *out_len);

// Encode a fully-populated RoutableMessage (payload + signature_data already
// filled by the session/crypto layer). `m` is left untouched.
int tesla_pb_encode_routable(const UniversalMessage_RoutableMessage *m,
                             uint8_t *out, size_t out_cap, size_t *out_len);

// Decode a RoutableMessage.
int tesla_pb_decode_routable(const uint8_t *data, size_t len,
                             UniversalMessage_RoutableMessage *m);

// Encode VCSEC.UnsignedMessage{InformationRequest{GET_STATUS}}.
int tesla_pb_encode_vcsec_status(uint8_t *out, size_t out_cap, size_t *out_len);

// Encode VCSEC.UnsignedMessage{WhitelistOperation{
//   addKeyToWhitelistAndAddPermissions{key, role, secondsToBeActive}}}.
// Used by the Phase 3 pairing flow.
int tesla_pb_encode_vcsec_whitelist(const uint8_t pubkey[TESLA_PUBKEY_LEN],
                                    uint32_t role, uint32_t seconds_to_be_active,
                                    uint8_t *out, size_t out_cap, size_t *out_len);

// Decode a VCSEC.FromVCSECMessage application payload.
int tesla_pb_decode_vcsec_from(const uint8_t *data, size_t len,
                               VCSEC_FromVCSECMessage *m);

#ifdef __cplusplus
}
#endif
