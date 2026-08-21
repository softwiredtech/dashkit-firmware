#include "session.h"

#include "protobuf_build.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "signatures.pb.h"

#include <string.h>

// ------- Metadata TLV serialization -------

void tesla_metadata_init(tesla_metadata_t *m)
{
    m->len = 0;
    m->last_tag = -1;
    m->finalized = false;
}

int tesla_metadata_add(tesla_metadata_t *m, uint8_t tag,
                       const uint8_t *value, size_t value_len)
{
    if (m->finalized) {
        return -1;
    }
    if (m->last_tag >= (int)tag) {
        return -1;
    }
    if (value_len > 255) {
        return -1;   // TLV length field is a single byte
    }
    if (value_len + 2 > sizeof(m->buf) - m->len) {
        return -1;   // buffer overflow
    }
    m->buf[m->len++] = tag;
    m->buf[m->len++] = (uint8_t)value_len;
    if (value_len > 0) {
        memcpy(&m->buf[m->len], value, value_len);
        m->len += value_len;
    }
    m->last_tag = tag;
    return 0;
}

int tesla_metadata_add_u32(tesla_metadata_t *m, uint8_t tag, uint32_t value)
{
    uint8_t be[4] = {
        (uint8_t)(value >> 24),
        (uint8_t)(value >> 16),
        (uint8_t)(value >> 8),
        (uint8_t)(value >> 0),
    };
    return tesla_metadata_add(m, tag, be, sizeof(be));
}

const uint8_t *tesla_metadata_serialize(tesla_metadata_t *m, size_t *len_out)
{
    if (!m->finalized) {
        if (m->len >= sizeof(m->buf)) {
            *len_out = 0;
            return NULL;
        }
        m->buf[m->len++] = 0xFF;   // TAG_END
        m->finalized = true;
    }
    *len_out = m->len;
    return m->buf;
}

// ------- Session info authentication (handshake) -------

int tesla_session_info_tag(const uint8_t k[TESLA_SHARED_KEY_LEN],
                           const uint8_t *vin, size_t vin_len,
                           const uint8_t *challenge, size_t challenge_len,
                           const uint8_t *encoded_info, size_t encoded_info_len,
                           uint8_t tag[TESLA_HMAC_LEN])
{
    tesla_metadata_t meta;
    const uint8_t *m;
    size_t m_len;
    uint8_t session_key[TESLA_HMAC_LEN];
    uint8_t sig_type = TESLA_SIG_TYPE_HMAC;
    int rc;

    rc = tesla_session_info_key(k, session_key);
    if (rc != 0) {
        return rc;
    }

    tesla_metadata_init(&meta);
    rc = tesla_metadata_add(&meta, TESLA_META_TAG_SIGNATURE_TYPE, &sig_type, 1);
    if (rc == 0) {
        rc = tesla_metadata_add(&meta, TESLA_META_TAG_PERSONALIZATION, vin, vin_len);
    }
    if (rc == 0) {
        rc = tesla_metadata_add(&meta, TESLA_META_TAG_CHALLENGE, challenge, challenge_len);
    }
    if (rc != 0) {
        return -1;
    }
    m = tesla_metadata_serialize(&meta, &m_len);
    if (m == NULL) {
        return -1;
    }

    // tag = HMAC(SESSION_INFO_KEY, M || encoded_info)
    return tesla_hmac_sha256_2(session_key, sizeof(session_key),
                               m, m_len, encoded_info, encoded_info_len, tag);
}

bool tesla_verify_session_info(const uint8_t k[TESLA_SHARED_KEY_LEN],
                               const uint8_t *vin, size_t vin_len,
                               const uint8_t *challenge, size_t challenge_len,
                               const uint8_t *encoded_info, size_t encoded_info_len,
                               const uint8_t *tag, size_t tag_len)
{
    uint8_t expected[TESLA_HMAC_LEN];

    if (tag_len != TESLA_HMAC_LEN) {
        return false;
    }
    if (tesla_session_info_tag(k, vin, vin_len, challenge, challenge_len,
                               encoded_info, encoded_info_len, expected) != 0) {
        return false;
    }
    return tesla_ct_equal(expected, tag, TESLA_HMAC_LEN);
}

// ------- Command / response metadata -------

int tesla_build_request_metadata(tesla_metadata_t *m, uint8_t domain,
                                 const uint8_t *vin, size_t vin_len,
                                 const uint8_t epoch[TESLA_EPOCH_LEN],
                                 uint32_t expires_at, uint32_t counter,
                                 uint32_t flags)
{
    uint8_t sig_type = TESLA_SIG_TYPE_AES_GCM_PERSONALIZED;
    int rc;

    tesla_metadata_init(m);
    rc = tesla_metadata_add(m, TESLA_META_TAG_SIGNATURE_TYPE, &sig_type, 1);
    if (rc == 0) {
        rc = tesla_metadata_add(m, TESLA_META_TAG_DOMAIN, &domain, 1);
    }
    if (rc == 0) {
        rc = tesla_metadata_add(m, TESLA_META_TAG_PERSONALIZATION, vin, vin_len);
    }
    if (rc == 0) {
        rc = tesla_metadata_add(m, TESLA_META_TAG_EPOCH, epoch, TESLA_EPOCH_LEN);
    }
    if (rc == 0) {
        rc = tesla_metadata_add_u32(m, TESLA_META_TAG_EXPIRES_AT, expires_at);
    }
    if (rc == 0) {
        rc = tesla_metadata_add_u32(m, TESLA_META_TAG_COUNTER, counter);
    }
    // Flags are only encoded when non-zero (protocol.md compatibility rule).
    if (rc == 0 && flags != 0) {
        rc = tesla_metadata_add_u32(m, TESLA_META_TAG_FLAGS, flags);
    }
    return rc;
}

int tesla_build_response_metadata(tesla_metadata_t *m, uint8_t domain,
                                  const uint8_t *vin, size_t vin_len,
                                  uint32_t counter, uint32_t flags,
                                  const uint8_t *request_hash, size_t request_hash_len,
                                  uint32_t fault)
{
    uint8_t sig_type = TESLA_SIG_TYPE_AES_GCM_RESPONSE;
    int rc;

    tesla_metadata_init(m);
    rc = tesla_metadata_add(m, TESLA_META_TAG_SIGNATURE_TYPE, &sig_type, 1);
    if (rc == 0) {
        rc = tesla_metadata_add(m, TESLA_META_TAG_DOMAIN, &domain, 1);
    }
    if (rc == 0) {
        rc = tesla_metadata_add(m, TESLA_META_TAG_PERSONALIZATION, vin, vin_len);
    }
    if (rc == 0) {
        rc = tesla_metadata_add_u32(m, TESLA_META_TAG_COUNTER, counter);
    }
    // Response metadata always encodes flags (unlike requests).
    if (rc == 0) {
        rc = tesla_metadata_add_u32(m, TESLA_META_TAG_FLAGS, flags);
    }
    if (rc == 0) {
        rc = tesla_metadata_add(m, TESLA_META_TAG_REQUEST_HASH, request_hash, request_hash_len);
    }
    if (rc == 0) {
        rc = tesla_metadata_add_u32(m, TESLA_META_TAG_FAULT, fault);
    }
    return rc;
}

int tesla_request_hash(uint8_t sig_type, const uint8_t *tag, size_t tag_len,
                       bool truncate_to_17, uint8_t *out, size_t *out_len)
{
    size_t n = tag_len;

    if (out_len != NULL) {
        *out_len = 0;
    }
    if (n > TESLA_HMAC_LEN) {
        return -1;
    }
    if (truncate_to_17 && n > 16) {
        n = 16;
    }
    out[0] = sig_type;
    if (n > 0) {
        memcpy(out + 1, tag, n);
    }
    if (out_len != NULL) {
        *out_len = n + 1;
    }
    return 0;
}

// ============================================================================
// Phase 2: session state, handshake, command signing, response processing,
// and the VCSEC multi-response classifier.
// ============================================================================

void tesla_session_init(tesla_session_t *s, uint8_t domain, tesla_now_ms_fn now_ms)
{
    memset(s, 0, sizeof(*s));
    s->domain = domain;
    s->now_ms = now_ms;
}

uint32_t tesla_session_now_vehicle_s(const tesla_session_t *s)
{
    // No session (or no local clock): can't compute a vehicle timestamp.
    if (!s->valid || s->now_ms == NULL) {
        return 0;
    }
    int64_t now_ms = (int64_t)s->now_ms();
    int64_t elapsed_ms = now_ms - (int64_t)s->sync_local_ms;
    int64_t vehicle_ms = (int64_t)s->clock_time * 1000 + elapsed_ms;
    if (vehicle_ms < 0) {
        return 0;
    }
    return (uint32_t)(vehicle_ms / 1000);
}

int tesla_build_handshake_request(uint32_t domain,
                                  const uint8_t client_pub[TESLA_PUBKEY_LEN],
                                  const uint8_t routing[16],
                                  const uint8_t challenge[16],
                                  uint8_t *out, size_t out_cap, size_t *out_len)
{
    return tesla_pb_encode_handshake(domain, client_pub, routing, challenge,
                                     out, out_cap, out_len);
}

// Derives K from the client keypair and the vehicle public key, then verifies
// the session-info HMAC tag against the challenge we sent. Shared by the
// initial handshake and (Phase 6) resync updates. On success also returns the
// decoded SessionInfo (epoch/counter/clock/status) so the caller need not
// decode it a second time.
static int verify_session_info(const tesla_keypair_t *key,
                               const uint8_t *vin, size_t vin_len,
                               const uint8_t challenge[16],
                               const uint8_t *encoded_info, size_t encoded_info_len,
                               const uint8_t tag[32],
                               uint8_t k_out[TESLA_SHARED_KEY_LEN],
                               uint8_t vin_pubkey[TESLA_PUBKEY_LEN],
                               Signatures_SessionInfo *info_out,
                               tesla_rng_fn f_rng, void *p_rng)
{
    Signatures_SessionInfo info;
    pb_istream_t stream;
    uint8_t k[TESLA_SHARED_KEY_LEN];
    int rc;

    if (challenge == NULL) {
        return -1;
    }

    // Zero before decode: nanopb does not clear callbacks/optional fields, and
    // this runs on unauthenticated input before HMAC verification.
    memset(&info, 0, sizeof(info));
    stream = pb_istream_from_buffer(encoded_info, encoded_info_len);
    if (!pb_decode(&stream, Signatures_SessionInfo_fields, &info)) {
        return -1;
    }
    if (info.publicKey.size != TESLA_PUBKEY_LEN) {
        return -1;
    }

    // K = SHA1(ECDH(client_priv, vehicle_pub))[:16]
    rc = tesla_derive_shared_key(key->priv, info.publicKey.bytes,
                                 f_rng, p_rng, k);
    if (rc != 0) {
        return -1;
    }

    // tag must equal HMAC(SESSION_INFO_KEY, M(challenge, VIN) || encoded_info)
    if (!tesla_verify_session_info(k, vin, vin_len, challenge, 16,
                                   encoded_info, encoded_info_len,
                                   tag, TESLA_HMAC_LEN)) {
        return -1;
    }

    if (k_out != NULL) {
        memcpy(k_out, k, TESLA_SHARED_KEY_LEN);
    }
    if (vin_pubkey != NULL) {
        memcpy(vin_pubkey, info.publicKey.bytes, TESLA_PUBKEY_LEN);
    }
    if (info_out != NULL) {
        *info_out = info;
    }
    return 0;
}

int tesla_session_handshake(tesla_session_t *s, const tesla_keypair_t *key,
                            const uint8_t *vin, size_t vin_len,
                            const uint8_t challenge[16],
                            const uint8_t *resp, size_t resp_len,
                            tesla_rng_fn f_rng, void *p_rng)
{
    UniversalMessage_RoutableMessage m;
    Signatures_SessionInfo info;
    uint8_t k[TESLA_SHARED_KEY_LEN];
    uint8_t pub[TESLA_PUBKEY_LEN];
    int rc = -1;

    if (s == NULL || key == NULL || challenge == NULL) {
        return -1;
    }
    if (tesla_pb_decode_routable(resp, resp_len, &m) != 0) {
        return -1;
    }
    // The response must carry encoded session info as its payload.
    if (m.which_payload != UniversalMessage_RoutableMessage_session_info_tag) {
        return -1;
    }
    // ...and an HMAC session-info tag in the signature data (unauthenticated
    // session info is discarded, mirroring the reference dispatcher).
    if (m.which_sub_sigData != UniversalMessage_RoutableMessage_signature_data_tag ||
        m.sub_sigData.signature_data.which_sig_type !=
            Signatures_SignatureData_session_info_tag_tag) {
        return -1;
    }

    // Verify HMAC + derive K in one decode; populates `info` for the caller.
    rc = verify_session_info(key, vin, vin_len, challenge,
                             m.payload.session_info.bytes,
                             m.payload.session_info.size,
                             m.sub_sigData.signature_data.sig_type.session_info_tag.tag,
                             k, pub, &info, f_rng, p_rng);
    if (rc != 0) {
        return -1;
    }

    s->valid = true;
    memcpy(s->epoch, info.epoch, TESLA_EPOCH_LEN);
    s->counter = info.counter;                    // next command uses counter+1
    s->clock_time = info.clock_time;
    s->sync_local_ms = (s->now_ms != NULL) ? (uint64_t)s->now_ms() : 0;
    s->handle = info.handle;
    memcpy(s->shared_key, k, TESLA_SHARED_KEY_LEN);
    memcpy(s->client_pubkey, key->pub, TESLA_PUBKEY_LEN);
    memcpy(s->vehicle_pubkey, pub, TESLA_PUBKEY_LEN);
    // status defaults to OK when omitted; only KEY_NOT_ON_WHITELIST (1) means
    // the key isn't enrolled. The adversary can't forge this: it's covered by
    // the session-info HMAC we just verified.
    s->whitelisted =
        (info.status == Signatures_Session_Info_Status_SESSION_INFO_STATUS_OK);
    // Fresh replay window: not primed until the first authenticated response.
    s->replay_init = false;
    s->replay_high = 0;
    return 0;
}

int tesla_session_build_command(tesla_session_t *s,
                                const uint8_t *vin, size_t vin_len,
                                uint8_t domain,
                                const uint8_t *payload, size_t payload_len,
                                const uint8_t routing[16], const uint8_t uuid[16],
                                uint8_t *out, size_t out_cap, size_t *out_len,
                                uint8_t request_hash[TESLA_HMAC_LEN + 1],
                                size_t *request_hash_len,
                                tesla_rng_fn f_rng, void *p_rng)
{
    UniversalMessage_RoutableMessage m;
    Signatures_AES_GCM_Personalized_Signature_Data *gcm;
    Signatures_SignatureData *sig;
    tesla_metadata_t meta;
    const uint8_t *mbytes;
    size_t m_len, req_hash_len;
    uint8_t aad[TESLA_SHA256_LEN];
    uint8_t nonce[TESLA_NONCE_LEN];
    uint8_t gcm_tag[TESLA_GCM_TAG_LEN];
    uint32_t counter, expires_at;
    int rc = -1;

    if (s == NULL || !s->valid || payload == NULL || routing == NULL ||
        uuid == NULL || f_rng == NULL || request_hash == NULL) {
        return -1;
    }
    if (payload_len > sizeof(m.payload.protobuf_message_as_bytes.bytes)) {
        return -1;
    }

    // Counter rollover (uint32 wrap). Matches the reference Encrypt() check.
    if (s->counter == 0xFFFFFFFFu) {
        return -1;
    }
    counter = s->counter + 1;

    // expires_at is the vehicle-clock timestamp stamping this command.
    expires_at = tesla_session_now_vehicle_s(s) + TESLA_SESSION_VALIDITY_S;

    rc = tesla_build_request_metadata(&meta, domain, vin, vin_len,
                                      s->epoch, expires_at, counter,
                                      TESLA_FLAG_ENCRYPT_RESPONSE);
    if (rc != 0) {
        return -1;
    }
    mbytes = tesla_metadata_serialize(&meta, &m_len);
    if (mbytes == NULL) {
        return -1;
    }
    rc = tesla_sha256(mbytes, m_len, aad);
    if (rc != 0) {
        return -1;
    }
    if (f_rng(p_rng, nonce, sizeof(nonce)) != 0) {
        return -1;
    }

    memset(&m, 0, sizeof(m));
    m.has_to_destination = true;
    tesla_pb_dest_domain(&m.to_destination, domain);
    m.has_from_destination = true;
    if (tesla_pb_dest_route(&m.from_destination, routing, 16) != 0) {
        return -1;
    }

    m.which_payload = (pb_size_t)UniversalMessage_RoutableMessage_protobuf_message_as_bytes_tag;
    {
        uint8_t ct[TESLA_PB_PAYLOAD_MAX];
        uint8_t tag[TESLA_GCM_TAG_LEN];

        if (payload_len > sizeof(ct)) {
            return -1;
        }
        rc = tesla_gcm_encrypt(s->shared_key, payload, payload_len,
                               aad, sizeof(aad), nonce, ct, tag);
        if (rc != 0) {
            return -1;
        }
        m.payload.protobuf_message_as_bytes.size = (pb_size_t)payload_len;
        memcpy(m.payload.protobuf_message_as_bytes.bytes, ct, payload_len);
        memcpy(gcm_tag, tag, TESLA_GCM_TAG_LEN);
    }

    m.which_sub_sigData = (pb_size_t)UniversalMessage_RoutableMessage_signature_data_tag;
    sig = &m.sub_sigData.signature_data;
    sig->has_signer_identity = true;
    sig->signer_identity.which_identity_type =
        (pb_size_t)Signatures_KeyIdentity_public_key_tag;
    sig->signer_identity.identity_type.public_key.size = TESLA_PUBKEY_LEN;
    memcpy(sig->signer_identity.identity_type.public_key.bytes,
           s->client_pubkey, TESLA_PUBKEY_LEN);

    sig->which_sig_type = (pb_size_t)Signatures_SignatureData_AES_GCM_Personalized_data_tag;
    gcm = &sig->sig_type.AES_GCM_Personalized_data;
    memcpy(gcm->epoch, s->epoch, TESLA_EPOCH_LEN);
    memcpy(gcm->nonce, nonce, TESLA_NONCE_LEN);
    gcm->counter = counter;
    gcm->expires_at = expires_at;
    memcpy(gcm->tag, gcm_tag, TESLA_GCM_TAG_LEN);

    m.uuid.size = 16;
    memcpy(m.uuid.bytes, uuid, 16);
    m.flags = TESLA_FLAG_ENCRYPT_RESPONSE;

    rc = tesla_request_hash(TESLA_SIG_TYPE_AES_GCM_PERSONALIZED,
                            gcm->tag, TESLA_GCM_TAG_LEN,
                            (domain == TESLA_DOMAIN_VEHICLE_SECURITY),
                            request_hash, &req_hash_len);
    if (rc != 0) {
        return -1;
    }
    if (request_hash_len != NULL) {
        *request_hash_len = req_hash_len;
    }

    if (tesla_pb_encode_routable(&m, out, out_cap, out_len) != 0) {
        return -1;
    }

    // Only commit the counter after a successful sign+encode.
    s->counter = counter;
    // Per-request anti-replay: re-arm the window for this request's responses.
    s->replay_init = false;
    s->replay_high = 0;
    return 0;
}

// Anti-replay (per request): only ever called for a response whose GCM tag has
// verified (C1: an unauthenticated frame cannot advance it). The BLE link is
// reliable and ordered, so only require strictly-newer counters — no sliding
// out-of-order window (YAGNI). The signed comparison is wraparound-safe across
// uint32 counter rollover.
static int replay_check(tesla_session_t *s, uint32_t counter)
{
    if (s->replay_init && (int32_t)(counter - s->replay_high) <= 0) {
        return -3;   // duplicate, or not newer than the highest authenticated
    }
    s->replay_high = counter;
    s->replay_init = true;
    return 0;
}

int tesla_session_process_response(tesla_session_t *s,
                                   const uint8_t *vin, size_t vin_len,
                                   const uint8_t *request_hash, size_t request_hash_len,
                                   const uint8_t *resp, size_t resp_len,
                                   uint8_t *payload_out, size_t payload_cap,
                                   size_t *payload_len, uint32_t *fault_out)
{
    UniversalMessage_RoutableMessage m;
    Signatures_AES_GCM_Response_Signature_Data *gcm;
    tesla_metadata_t meta;
    const uint8_t *mbytes;
    size_t m_len;
    uint8_t aad[TESLA_SHA256_LEN];
    uint8_t plain[TESLA_PB_PAYLOAD_MAX];
    size_t plain_len;
    int rc;

    if (s == NULL || !s->valid || request_hash == NULL) {
        return -1;
    }
    if (tesla_pb_decode_routable(resp, resp_len, &m) != 0) {
        return -1;
    }
    if (fault_out != NULL) {
        // Surface the actual fault code, not just a boolean (review N1/N2).
        *fault_out = m.signedMessageStatus.signed_message_fault;
    }

    // A response that carries proactive session info is a desync hint, not an
    // application reply; the caller should re-sync and retry (Phase 6 wires
    // the full recovery; Phase 2 surfaces it as a distinct return).
    if (m.which_payload == UniversalMessage_RoutableMessage_session_info_tag) {
        return -2;
    }
    if (m.which_payload != UniversalMessage_RoutableMessage_protobuf_message_as_bytes_tag) {
        return -1;
    }

    if (m.which_sub_sigData == UniversalMessage_RoutableMessage_signature_data_tag &&
        m.sub_sigData.signature_data.which_sig_type ==
            Signatures_SignatureData_AES_GCM_Response_data_tag) {
        gcm = &m.sub_sigData.signature_data.sig_type.AES_GCM_Response_data;
        // Response metadata domain is the response's *from* domain (its
        // origin), exactly like the reference responseMetadata(); the sender
        // might have used BROADCAST, so we read it from the message rather
        // than assuming our domain.
        uint8_t resp_domain = s->domain;
        if (m.has_from_destination &&
            m.from_destination.which_sub_destination ==
                UniversalMessage_Destination_domain_tag) {
            resp_domain = (uint8_t)m.from_destination.sub_destination.domain;
        }
        // Response metadata: sig_type=AES_GCM_RESPONSE, domain=from domain,
        // flags always included, fault always encoded.
        rc = tesla_build_response_metadata(&meta, resp_domain, vin, vin_len,
                                           gcm->counter, m.flags,
                                           request_hash, request_hash_len,
                                           m.signedMessageStatus.signed_message_fault);
        if (rc != 0) {
            return -1;
        }
        mbytes = tesla_metadata_serialize(&meta, &m_len);
        if (mbytes == NULL) {
            return -1;
        }
        rc = tesla_sha256(mbytes, m_len, aad);
        if (rc != 0) {
            return -1;
        }
        if (m.payload.protobuf_message_as_bytes.size > sizeof(plain)) {
            return -1;
        }
        // Authenticate FIRST: the GCM tag binds nonce/tag/ciphertext and the
        // AAD (which covers the counter via response metadata). Only a
        // successfully authenticated response may advance the anti-replay
        // window — a forged frame fails here and the window stays untouched
        // (review C1). A victim is never returned on auth failure.
        rc = tesla_gcm_decrypt(s->shared_key,
                               m.payload.protobuf_message_as_bytes.bytes,
                               m.payload.protobuf_message_as_bytes.size,
                               aad, sizeof(aad), gcm->nonce, gcm->tag, plain);
        if (rc != 0) {
            return -1;
        }
        plain_len = m.payload.protobuf_message_as_bytes.size;

        // Anti-replay AFTER authentication: only reject non-newer counters
        // (link is ordered; no out-of-order window needed). An unauthenticated
        // frame never reaches here, so it cannot poison the window (C1).
        if (replay_check(s, gcm->counter) != 0) {
            return -3;
        }
    } else {
        // Older firmware (pre-2024.38): response payload is plaintext — NO GCM
        // tag, NO request-hash binding, and NO anti-replay here. Unauthenticated:
        // a rogue link peer could inject arbitrary bytes. Acceptable only for
        // this read-only status poll against 2024.38+ firmware; revisit if a
        // pre-2024.38 car must be supported for anything but status.
        plain_len = m.payload.protobuf_message_as_bytes.size;
        if (plain_len > sizeof(plain)) {
            return -1;
        }
        memcpy(plain, m.payload.protobuf_message_as_bytes.bytes, plain_len);
    }

    if (plain_len > payload_cap) {
        return -1;
    }
    memcpy(payload_out, plain, plain_len);
    if (payload_len != NULL) {
        *payload_len = plain_len;
    }
    return 0;
}

tesla_vcsec_phase_t tesla_vcsec_ingest(const VCSEC_FromVCSECMessage *m)
{
    if (m == NULL) {
        return TESLA_VCSEC_PENDING;
    }
    switch (m->which_sub_message) {
    case VCSEC_FromVCSECMessage_nominalError_tag:
        return TESLA_VCSEC_ERROR;
    case VCSEC_FromVCSECMessage_vehicleStatus_tag:
        return TESLA_VCSEC_STATUS;
    case VCSEC_FromVCSECMessage_commandStatus_tag: {
        // signedMessageStatus: WAIT/ERROR are non-terminal (protocol.md says
        // discard and wait for the specific result); anything else is success.
        const VCSEC_CommandStatus *cs = &m->sub_message.commandStatus;
        if (cs->operationStatus == VCSEC_OperationStatus_E_OPERATIONSTATUS_WAIT ||
            cs->operationStatus == VCSEC_OperationStatus_E_OPERATIONSTATUS_ERROR) {
            return TESLA_VCSEC_PENDING;
        }
        return TESLA_VCSEC_DONE;
    }
    default:
        // Empty / unrecognized: success (no application payload to report).
        return TESLA_VCSEC_DONE;
    }
}
