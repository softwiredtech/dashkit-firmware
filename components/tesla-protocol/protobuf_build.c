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

void tesla_pb_dest_route(UniversalMessage_Destination *d,
                         const uint8_t *addr, size_t len)
{
    d->which_sub_destination =
        (pb_size_t)UniversalMessage_Destination_routing_address_tag;
    if (len > sizeof(d->sub_destination.routing_address.bytes)) {
        len = sizeof(d->sub_destination.routing_address.bytes);
    }
    d->sub_destination.routing_address.size = (pb_size_t)len;
    memcpy(d->sub_destination.routing_address.bytes, addr, len);
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
    tesla_pb_dest_route(&m.from_destination, routing, 16);

    m.which_payload =
        (pb_size_t)UniversalMessage_RoutableMessage_session_info_request_tag;
    m.payload.session_info_request.public_key.size = TESLA_PUBKEY_LEN;
    memcpy(m.payload.session_info_request.public_key.bytes,
           client_pub, TESLA_PUBKEY_LEN);
    m.payload.session_info_request.challenge.size = 16;
    memcpy(m.payload.session_info_request.challenge.bytes, challenge, 16);

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
                                    uint32_t role, uint32_t seconds_to_be_active,
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
    perm->secondsToBeActive = seconds_to_be_active;
    perm->keyRole = (Keys_Role)role;

    stream = pb_ostream_from_buffer(out, out_cap);
    if (!pb_encode(&stream, VCSEC_UnsignedMessage_fields, &msg)) {
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
