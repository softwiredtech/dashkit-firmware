// Tesla vehicle-command session layer: metadata TLV serialization, session-info
// authentication, and request hashing, on top of crypto.h.
//
// Ported from the Apache-2.0 vehicle-command reference (internal/authentication
// metadata.go / peer.go / signer.go), validated against the known-answer
// vectors in Tesla's protocol.md.
//
// Counter/epoch state management, full signer state, and protobuf building
// arrive in later phases of the integration plan; this file only contains the
// pieces the Phase 0 crypto tests exercise.

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "crypto.h"

// Signatures.Tag values (signatures.proto).
#define TESLA_META_TAG_SIGNATURE_TYPE  0
#define TESLA_META_TAG_DOMAIN          1
#define TESLA_META_TAG_PERSONALIZATION 2
#define TESLA_META_TAG_EPOCH           3
#define TESLA_META_TAG_EXPIRES_AT      4
#define TESLA_META_TAG_COUNTER         5
#define TESLA_META_TAG_CHALLENGE       6
#define TESLA_META_TAG_FLAGS           7
#define TESLA_META_TAG_REQUEST_HASH    8
#define TESLA_META_TAG_FAULT           9

// Signatures.SignatureType values (signatures.proto).
#define TESLA_SIG_TYPE_AES_GCM_PERSONALIZED 5
#define TESLA_SIG_TYPE_HMAC                 6
#define TESLA_SIG_TYPE_HMAC_PERSONALIZED    8
#define TESLA_SIG_TYPE_AES_GCM_RESPONSE     9

// UniversalMessage.Domain values (universal_message.proto).
#define TESLA_DOMAIN_BROADCAST        0
#define TESLA_DOMAIN_VEHICLE_SECURITY 2
#define TESLA_DOMAIN_INFOTAINMENT     3

// RoutableMessage.flags bit for FLAG_ENCRYPT_RESPONSE (= 1 << 1). Requests set
// this bit so 2024.38+ firmware encrypts the response.
#define TESLA_FLAG_ENCRYPT_RESPONSE (1u << 1u)

// SessionInfo.epoch length.
#define TESLA_EPOCH_LEN 16

// Serialized-metadata bound. Worst realistic metadata (response metadata with a
// full VIN and a 33-byte request hash) is ~80 bytes; 128 leaves headroom while
// keeping the struct stack-allocable.
#define TESLA_METADATA_MAX 128

// Tag-length-value metadata serializer. Tags must be added in strictly
// ascending order. This is deliberately stricter than the reference
// implementation, which permits duplicate tags: the serialization stays
// injective (no two metadata sets collide), which the protocol relies on. A
// value longer than 255 bytes is rejected because the TLV length field is a
// single byte.
typedef struct {
    uint8_t buf[TESLA_METADATA_MAX];
    size_t  len;
    int     last_tag;   // -1 = nothing added yet
    bool    finalized;  // 0xFF terminator appended
} tesla_metadata_t;

void tesla_metadata_init(tesla_metadata_t *m);

// Returns 0 on success, -1 on out-of-order tag / oversized value / overflow.
int tesla_metadata_add(tesla_metadata_t *m, uint8_t tag,
                       const uint8_t *value, size_t value_len);
int tesla_metadata_add_u32(tesla_metadata_t *m, uint8_t tag, uint32_t value);

// Appends the 0xFF end-of-metadata byte (idempotent) and returns the
// serialized bytes; NULL on overflow. The returned pointer is valid until the
// next add/serialize.
const uint8_t *tesla_metadata_serialize(tesla_metadata_t *m, size_t *len_out);

// Session-info authentication tag (handshake), per protocol.md:
//
//   SESSION_INFO_KEY = HMAC-SHA256(K, "session info")
//   M = TLV(sig_type=HMAC, VIN, challenge) || 0xFF
//   tag = HMAC-SHA256(SESSION_INFO_KEY, M || encoded_info)
//
// `encoded_info` is the raw SessionInfo protobuf from the vehicle response.
int tesla_session_info_tag(const uint8_t k[TESLA_SHARED_KEY_LEN],
                           const uint8_t *vin, size_t vin_len,
                           const uint8_t *challenge, size_t challenge_len,
                           const uint8_t *encoded_info, size_t encoded_info_len,
                           uint8_t tag[TESLA_HMAC_LEN]);

// Constant-time wrapper around tesla_session_info_tag: true iff `tag` matches.
// Short tags (tag_len != 32) never match.
bool tesla_verify_session_info(const uint8_t k[TESLA_SHARED_KEY_LEN],
                               const uint8_t *vin, size_t vin_len,
                               const uint8_t *challenge, size_t challenge_len,
                               const uint8_t *encoded_info, size_t encoded_info_len,
                               const uint8_t *tag, size_t tag_len);

// Serialized metadata for an outgoing AES-GCM command. The AAD for encryption
// is SHA256(serialized). flags is only encoded when non-zero (backwards
// compatibility rule in protocol.md); it is a full 32-bit value on the wire.
int tesla_build_request_metadata(tesla_metadata_t *m, uint8_t domain,
                                 const uint8_t *vin, size_t vin_len,
                                 const uint8_t epoch[TESLA_EPOCH_LEN],
                                 uint32_t expires_at, uint32_t counter,
                                 uint32_t flags);

// Serialized metadata for decrypting an encrypted response (AAD = SHA256(serialized)).
// Per protocol.md the Flags field is always included here (unlike requests) and
// the fault is always encoded (0 when none).
int tesla_build_response_metadata(tesla_metadata_t *m, uint8_t domain,
                                  const uint8_t *vin, size_t vin_len,
                                  uint32_t counter, uint32_t flags,
                                  const uint8_t *request_hash, size_t request_hash_len,
                                  uint32_t fault);

// Request hash: [sig_type_byte] || authentication tag. Requests to the Vehicle
// Security domain are truncated to 17 bytes (sig byte + first 16 tag bytes);
// for AES-GCM the tag is already 16 bytes so this is a no-op. `out` must hold
// at least TESLA_HMAC_LEN + 1 bytes. Returns 0 on success (with *out_len set),
// -1 if tag_len exceeds TESLA_HMAC_LEN.
int tesla_request_hash(uint8_t sig_type, const uint8_t *tag, size_t tag_len,
                       bool truncate_to_17, uint8_t *out, size_t *out_len);
