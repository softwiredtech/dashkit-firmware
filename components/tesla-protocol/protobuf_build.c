#include "protobuf_build.h"

#include "pb_decode.h"
#include "pb_encode.h"

#include <string.h>

void tesla_pb_dest_domain(UniversalMessage_Destination *d, uint32_t domain)
{
    d->which_sub_destination =
        (pb_size_t)UniversalMessage_Destination_domain_tag;
    d->sub_destination.domain = (UniversalMessage_Domain)domain;
}

int tesla_pb_dest_route(UniversalMessage_Destination *d,
                        const uint8_t *addr, size_t len)
{
    d->which_sub_destination =
        (pb_size_t)UniversalMessage_Destination_routing_address_tag;
    if (len > sizeof(d->sub_destination.routing_address.bytes)) {
        return -1;
    }
    d->sub_destination.routing_address.size = (pb_size_t)len;
    memcpy(d->sub_destination.routing_address.bytes, addr, len);
    return 0;
}

int tesla_pb_encode_routable(const UniversalMessage_RoutableMessage *m,
                             uint8_t *out, size_t out_cap, size_t *out_len)
{
    pb_ostream_t stream = pb_ostream_from_buffer(out, out_cap);

    if (!pb_encode(&stream, UniversalMessage_RoutableMessage_fields, m)) {
        return -1;
    }
    if (out_len != NULL) {
        *out_len = stream.bytes_written;
    }
    return 0;
}

int tesla_pb_encode_handshake(uint32_t domain,
                              const uint8_t client_pub[TESLA_PUBKEY_LEN],
                              const uint8_t routing[16],
                              const uint8_t challenge[16],
                              uint8_t *out, size_t out_cap, size_t *out_len)
{
    UniversalMessage_RoutableMessage m;

    memset(&m, 0, sizeof(m));   // init_zero: callbacks/integers zeroed

    m.has_to_destination = true;
    tesla_pb_dest_domain(&m.to_destination, domain);

    m.has_from_destination = true;
    if (tesla_pb_dest_route(&m.from_destination, routing, 16) != 0) {
        return -1;
    }

    m.which_payload =
        (pb_size_t)UniversalMessage_RoutableMessage_session_info_request_tag;
    m.payload.session_info_request.public_key.size = TESLA_PUBKEY_LEN;
    memcpy(m.payload.session_info_request.public_key.bytes,
           client_pub, TESLA_PUBKEY_LEN);
    // NOTE: SessionInfoRequest.challenge is intentionally NOT set. Per
    // protocol.md the handshake's HMAC challenge is the request's `uuid`
    // (below), and every working implementation (Go vehicle-command,
    // pyteslable, ESPHome) leaves `challenge` empty. Sending a populated
    // `challenge` is the one field that differs from the reference and made
    // the real car silently ignore the handshake (no session_info reply).

    m.uuid.size = 16;
    memcpy(m.uuid.bytes, challenge, 16);

    return tesla_pb_encode_routable(&m, out, out_cap, out_len);
}

int tesla_pb_decode_routable(const uint8_t *data, size_t len,
                             UniversalMessage_RoutableMessage *m)
{
    pb_istream_t stream = pb_istream_from_buffer(data, len);

    if (!pb_decode(&stream, UniversalMessage_RoutableMessage_fields, m)) {
        return -1;
    }
    return 0;
}

int tesla_pb_encode_vcsec_status(uint8_t *out, size_t out_cap, size_t *out_len)
{
    VCSEC_UnsignedMessage msg;
    pb_ostream_t stream;

    memset(&msg, 0, sizeof(msg));
    msg.which_sub_message =
        (pb_size_t)VCSEC_UnsignedMessage_InformationRequest_tag;
    msg.sub_message.InformationRequest.informationRequestType =
        VCSEC_InformationRequestType_INFORMATION_REQUEST_TYPE_GET_STATUS;

    stream = pb_ostream_from_buffer(out, out_cap);
    if (!pb_encode(&stream, VCSEC_UnsignedMessage_fields, &msg)) {
        return -1;
    }
    if (out_len != NULL) {
        *out_len = stream.bytes_written;
    }
    return 0;
}

int tesla_pb_encode_vcsec_whitelist(const uint8_t pubkey[TESLA_PUBKEY_LEN],
                                    uint32_t role, uint32_t form_factor,
                                    uint8_t *out, size_t out_cap, size_t *out_len)
{
    VCSEC_UnsignedMessage msg;
    VCSEC_PermissionChange *perm;
    pb_ostream_t stream;

    memset(&msg, 0, sizeof(msg));
    msg.which_sub_message =
        (pb_size_t)VCSEC_UnsignedMessage_WhitelistOperation_tag;

    msg.sub_message.WhitelistOperation.which_sub_message =
        (pb_size_t)VCSEC_WhitelistOperation_addKeyToWhitelistAndAddPermissions_tag;

    perm = &msg.sub_message.WhitelistOperation.sub_message
               .addKeyToWhitelistAndAddPermissions;
    perm->has_key = true;
    perm->key.PublicKeyRaw.size = TESLA_PUBKEY_LEN;
    memcpy(perm->key.PublicKeyRaw.bytes, pubkey, TESLA_PUBKEY_LEN);
    perm->secondsToBeActive = 0;   // permanent key (reference leaves it unset)
    perm->keyRole = (Keys_Role)role;

    // metadataForKey{keyFormFactor}: the car records how the key was enrolled.
    msg.sub_message.WhitelistOperation.has_metadataForKey = true;
    msg.sub_message.WhitelistOperation.metadataForKey.keyFormFactor =
        (VCSEC_KeyFormFactor)form_factor;

    stream = pb_ostream_from_buffer(out, out_cap);
    if (!pb_encode(&stream, VCSEC_UnsignedMessage_fields, &msg)) {
        return -1;
    }
    if (out_len != NULL) {
        *out_len = stream.bytes_written;
    }
    return 0;
}

int tesla_pb_build_enrollment(const tesla_keypair_t *key, uint32_t role,
                              uint32_t form_factor,
                              uint8_t *out, size_t out_cap, size_t *out_len)
{
    uint8_t inner[256];
    size_t inner_len = 0;
    VCSEC_ToVCSECMessage env;
    pb_ostream_t stream;

    if (key == NULL || out == NULL) {
        return -1;
    }
    // Inner application message: the addKey WhitelistOperation (UnsignedMessage).
    if (tesla_pb_encode_vcsec_whitelist(key->pub, role, form_factor,
                                        inner, sizeof(inner), &inner_len) != 0) {
        return -1;
    }

    // Envelope: ToVCSECMessage{ SignedMessage{ protobufMessageAsBytes = inner,
    //                                          signatureType = PRESENT_KEY } }.
    // No signature is appended — the car authorizes the enrollment physically.
    memset(&env, 0, sizeof(env));
    env.has_signedMessage = true;
    env.signedMessage.protobufMessageAsBytes.size = (pb_size_t)inner_len;
    memcpy(env.signedMessage.protobufMessageAsBytes.bytes, inner, inner_len);
    env.signedMessage.signatureType =
        VCSEC_SignatureType_SIGNATURE_TYPE_PRESENT_KEY;

    stream = pb_ostream_from_buffer(out, out_cap);
    if (!pb_encode(&stream, VCSEC_ToVCSECMessage_fields, &env)) {
        return -1;
    }
    if (out_len != NULL) {
        *out_len = stream.bytes_written;
    }
    return 0;
}

int tesla_pb_decode_vcsec_from(const uint8_t *data, size_t len,
                               VCSEC_FromVCSECMessage *m)
{
    pb_istream_t stream = pb_istream_from_buffer(data, len);

    if (!pb_decode(&stream, VCSEC_FromVCSECMessage_fields, m)) {
        return -1;
    }
    return 0;
}
