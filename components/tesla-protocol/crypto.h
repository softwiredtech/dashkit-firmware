// Tesla vehicle-command crypto primitives, ported to mbedTLS 3.x.
//
// Implements the shared-secret key agreement and symmetric-key primitives
// from Tesla's Apache-2.0 vehicle-command protocol.md:
//
//   K                 = SHA1(X-coordinate of ECDH(priv, peer_pub))[:16]
//   SESSION_INFO_KEY  = HMAC-SHA256(K, "session info")
//   AUTH_CMD_KEY      = HMAC-SHA256(K, "authenticated command")
//   commands/responses = AES-128-GCM(K, nonce, AAD=SHA256(metadata))
//
// No ESP-IDF dependency: this file compiles both in-tree (against the
// ESP-IDF mbedTLS component) and on a host (against a host mbedTLS build),
// so the host-side unit test in tools/test/ can validate it against Tesla's
// published test vectors.
//
// All byte strings are the raw binary encodings; hex notation in comments is
// only for printability (matching protocol.md).

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// 128-bit AES-GCM session key K.
#define TESLA_SHARED_KEY_LEN 16
// Uncompressed NIST-P256 point: 0x04 || X(32) || Y(32).
#define TESLA_PUBKEY_LEN 65
// NIST-P256 private scalar, big-endian.
#define TESLA_PRIVKEY_LEN 32
// AES-GCM initialization vector length.
#define TESLA_NONCE_LEN 12
// AES-GCM authentication tag length.
#define TESLA_GCM_TAG_LEN 16
// HMAC-SHA256 tag length.
#define TESLA_HMAC_LEN 32
#define TESLA_SHA1_LEN 20
#define TESLA_SHA256_LEN 32

// A client P-256 keypair: big-endian private scalar + uncompressed public
// point (0x04 || X || Y). Persisted by the pairing/storage layer and passed to
// the session layer for ECDH and for signer identity.
typedef struct {
    uint8_t priv[TESLA_PRIVKEY_LEN];
    uint8_t pub[TESLA_PUBKEY_LEN];
} tesla_keypair_t;

// RNG callback using the mbedtls f_rng convention (returns 0 on success).
typedef int (*tesla_rng_fn)(void *ctx, uint8_t *buf, size_t len);

// One-shot hashes and HMAC. Return 0 on success, negative mbedTLS error otherwise.
int tesla_sha1(const uint8_t *data, size_t len, uint8_t out[TESLA_SHA1_LEN]);
int tesla_sha256(const uint8_t *data, size_t len, uint8_t out[TESLA_SHA256_LEN]);
int tesla_hmac_sha256(const uint8_t *key, size_t key_len,
                      const uint8_t *msg, size_t msg_len,
                      uint8_t out[TESLA_HMAC_LEN]);
// HMAC-SHA256 over two non-contiguous parts, equivalent to hashing the
// concatenation. Used for the protocol's "M || payload" patterns.
int tesla_hmac_sha256_2(const uint8_t *key, size_t key_len,
                        const uint8_t *a, size_t a_len,
                        const uint8_t *b, size_t b_len,
                        uint8_t out[TESLA_HMAC_LEN]);

// Constant-time comparison for HMAC/GCM tag validation.
// Returns true iff a and b are identical over len bytes.
bool tesla_ct_equal(const uint8_t *a, const uint8_t *b, size_t len);

// K = SHA1(X-coordinate of ECDH(priv, peer_pub))[:16].
//
// The peer public key is checked to be a valid point on NIST-P256 before the
// scalar multiply (invalid-curve hardening). f_rng feeds mbedTLS' ECDH
// blinding (mandatory in mbedTLS 3.x); correctness of the shared secret does
// not depend on RNG quality. The firmware passes a hardware-RNG-backed
// callback; the host test passes a deterministic one.
int tesla_derive_shared_key(const uint8_t priv[TESLA_PRIVKEY_LEN],
                            const uint8_t peer_pub[TESLA_PUBKEY_LEN],
                            tesla_rng_fn f_rng, void *p_rng,
                            uint8_t out[TESLA_SHARED_KEY_LEN]);

// Subkeys derived from K (protocol.md "Key agreement" / "HMAC-SHA256
// authentication").
int tesla_session_info_key(const uint8_t k[TESLA_SHARED_KEY_LEN],
                           uint8_t out[TESLA_HMAC_LEN]);
int tesla_authenticated_command_key(const uint8_t k[TESLA_SHARED_KEY_LEN],
                                    uint8_t out[TESLA_HMAC_LEN]);

// AES-128-GCM authenticated encryption. The caller supplies a fresh 12-byte
// nonce (one CSPRNG draw per message); ciphertext must hold plaintext_len
// bytes. Returns 0 on success, negative mbedTLS error otherwise.
int tesla_gcm_encrypt(const uint8_t k[TESLA_SHARED_KEY_LEN],
                      const uint8_t *plaintext, size_t plaintext_len,
                      const uint8_t *aad, size_t aad_len,
                      const uint8_t nonce[TESLA_NONCE_LEN],
                      uint8_t *ciphertext,
                      uint8_t tag[TESLA_GCM_TAG_LEN]);

// AES-128-GCM authenticated decryption. plaintext must hold ciphertext_len
// bytes. Returns 0 on success, MBEDTLS_ERR_GCM_AUTH_FAILED on tag mismatch,
// other negative mbedTLS errors otherwise.
int tesla_gcm_decrypt(const uint8_t k[TESLA_SHARED_KEY_LEN],
                      const uint8_t *ciphertext, size_t ciphertext_len,
                      const uint8_t *aad, size_t aad_len,
                      const uint8_t nonce[TESLA_NONCE_LEN],
                      const uint8_t tag[TESLA_GCM_TAG_LEN],
                      uint8_t *plaintext);

// ============================================================================
// Phase 3: key generation for present-key enrollment.
//
// Enrollment sends a VCSEC.ToVCSECMessage whose SignedMessage carries the new
// public key and SIGNATURE_TYPE_PRESENT_KEY (see protobuf_build.c). The message
// is NOT cryptographically signed by the firmware — the car authorizes the
// enrollment physically (owner taps an NFC card + confirms on the touchscreen),
// exactly as the Apache-2.0 vehicle-command reference's SendAddKeyRequestWithRole
// does (it marshals the envelope and sends it with no appended signature; the
// reference's Schnorr/P256 code is for Fleet-API JWT signing, not BLE).
// ============================================================================

// Generate a fresh NIST-P256 keypair into `key` (big-endian priv scalar + 65
// bytes uncompressed pub). `f_rng`/`p_rng` must be a cryptographically strong
// RNG (hardware RNG on-device). Returns 0 on success.
int tesla_keypair_generate(tesla_keypair_t *key, tesla_rng_fn f_rng, void *p_rng);
