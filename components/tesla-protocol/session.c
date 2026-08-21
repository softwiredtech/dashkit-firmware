#include "session.h"

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
        return -1;   // can't grow after the terminator
    }
    if (m->last_tag >= (int)tag) {
        return -1;   // tags must be strictly ascending
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
