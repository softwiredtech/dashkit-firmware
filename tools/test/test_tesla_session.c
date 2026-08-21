// Host round-trip test for the Phase 2 Tesla BLE protocol layer
// (components/tesla-protocol/{crypto,session,protobuf_build}.c over the
// committed nanopb bindings in generated/).
//
// This builds the whole protocol stack off the car and exercises the exact
// in-car milestone the plan defers to after Phase 3 enrollment:
//   handshake -> derive K -> sign+encrypt a VCSEC GET_STATUS -> simulate the
//   vehicle (using protocol.md's published vehicle private key) decrypting +
//   responding -> client decrypts + validates + runs the VCSEC terminal
//   state machine.
//
// Known-answer checks pin both sides to protocol.md's published keys
// (client c/C, vehicle v/V, derived K = 1b2fce...), so a regression in any
// metadata/crypto/message-building step fails loudly.
//
// Compile against a host mbedTLS 3.6 build (see run_tesla_session_test.sh).
//
//   gcc -std=c99 -Wall -Wextra -I components/tesla-protocol
//       -I components/tesla-protocol/generated -I components/tesla-protocol/nanopb
//       -I <mbedtls>/include tools/test/test_tesla_session.c
//       components/tesla-protocol/{crypto.c,session.c,protobuf_build.c}
//       components/tesla-protocol/generated/*.pb.c
//       components/tesla-protocol/nanopb/*.c
//       -L <mbedtls>/lib -lmbedcrypto -lmbedtls -lmbedx509
//       -o /tmp/test_tesla_session && /tmp/test_tesla_session

#include "crypto.h"
#include "session.h"
#include "protobuf_build.h"

#include "universal_message.pb.h"
#include "signatures.pb.h"
#include "vcsec.pb.h"

#include "pb_decode.h"
#include "pb_encode.h"

#include <stdio.h>
#include <string.h>

// ---- tiny test harness (mirrors test_dbc.c / test_tesla_crypto.c) ----
static int g_fail;
#define CHECK(cond, ...) do { \
    if (!(cond)) { printf("FAIL: " __VA_ARGS__); printf("\n"); g_fail++; } \
    else { printf("ok:   " __VA_ARGS__); printf("\n"); } \
} while (0)

static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static int unhex(const char *hex, uint8_t *out, size_t out_cap)
{
    size_t n = strlen(hex);
    if (n % 2 != 0 || n / 2 > out_cap) return -1;
    for (size_t i = 0; i < n / 2; i++) {
        int hi = hexval(hex[2 * i]);
        int lo = hexval(hex[2 * i + 1]);
        if (hi < 0 || lo < 0) return -1;
        out[i] = (uint8_t)((hi << 4) | lo);
    }
    return (int)(n / 2);
}

static bool bytes_eq(const uint8_t *a, const uint8_t *b, size_t n)
{
    return memcmp(a, b, n) == 0;
}

static bool check_hex(const uint8_t *buf, size_t n, const char *want)
{
    uint8_t exp[160];
    int len = unhex(want, exp, sizeof(exp));
    return len == (int)n && bytes_eq(buf, exp, n);
}

// Deterministic RNG so the test is reproducible (ECDH blinding only; the
// correctness of the shared secret does not depend on RNG quality).
static uint32_t g_rng = 0x13579BDFu;
static int dummy_rng(void *ctx, uint8_t *buf, size_t len)
{
    (void)ctx;
    for (size_t i = 0; i < len; i++) {
        g_rng = g_rng * 1664525u + 1013904223u;
        buf[i] = (uint8_t)(g_rng >> 24);
    }
    return 0;
}

static uint64_t g_now_ms = 1000000u;   // deterministic monotonic clock
static uint64_t test_now_ms(void) { return g_now_ms; }

// ---- protocol.md published test keys ----
static const char *VIN = "5YJ30123456789ABC";
// c  private scalar, v  private scalar
static const char *CLIENT_PRIV_HEX = "2538CDC29A97C19C1E99A637D6CF4F8C970C118B56EDE1E6323E6D162C4B30DB";
static const char *VEHICLE_PRIV_HEX = "344EE5B466A7CF1EEB12B6F50331DB2E5EC5834EF5F4BEFCFD8CBE55C2528D70";
static const char *CLIENT_PUB_HEX = "04b2b6bc68c2da0665ce656815594996c62394edd8bea905fe781a754fe6a845a714330902f225e9269d466e05b349981fda9d85cc23c6fb444aa73b629105dc6e";
static const char *VEHICLE_PUB_HEX = "04c7a1f47138486aa4729971494878d33b1a24e39571f748a6e16c5955b3d877d3a6aaa0e955166474af5d32c410f439a2234137ad1bb085fd4e8813c958f11d97";
static const char *K_HEX = "1b2fce19967b79db696f909cff89ea9a";

static uint8_t g_c_priv[32], g_c_pub[65], g_v_priv[32], g_v_pub[65], g_K[16];
static uint8_t g_epoch[16] = { 'e','p','o','c','h','-', 0,1,2,3,4,5,6,7,8,9 };

// Build a SessionInfo protobuf and HMAC session-info tag the way the vehicle
// (holding v, knowing K) would, then wrap both in a RoutableMessage response.
static int vehicle_session_response(const uint8_t challenge[16],
                                    uint32_t counter, uint32_t clock_time,
                                    uint32_t handle,
                                    Signatures_Session_Info_Status status,
                                    uint8_t *resp, size_t resp_cap, size_t *resp_len)
{
    Signatures_SessionInfo info;
    uint8_t encoded[128];
    uint8_t tag[32];
    size_t enc_len = 0;
    UniversalMessage_RoutableMessage m;
    pb_ostream_t os;

    memset(&info, 0, sizeof(info));
    info.counter = counter;
    info.publicKey.size = 65;
    memcpy(info.publicKey.bytes, g_v_pub, 65);
    memcpy(info.epoch, g_epoch, 16);
    info.clock_time = clock_time;
    info.handle = handle;
    info.status = status;

    os = pb_ostream_from_buffer(encoded, sizeof(encoded));
    if (!pb_encode(&os, Signatures_SessionInfo_fields, &info)) return -1;
    enc_len = os.bytes_written;

    // tag = HMAC-SHA256(SESSION_INFO_KEY, M(sig_type=HMAC, VIN, challenge) || info)
    if (tesla_session_info_tag(g_K, (const uint8_t *)VIN, strlen(VIN),
                               challenge, 16, encoded, enc_len, tag) != 0) {
        return -1;
    }

    memset(&m, 0, sizeof(m));
    m.has_from_destination = true;
    tesla_pb_dest_domain(&m.from_destination, TESLA_DOMAIN_INFOTAINMENT);
    m.which_payload = (pb_size_t)UniversalMessage_RoutableMessage_session_info_tag;
    m.payload.session_info.size = (pb_size_t)enc_len;
    memcpy(m.payload.session_info.bytes, encoded, enc_len);
    m.which_sub_sigData = (pb_size_t)UniversalMessage_RoutableMessage_signature_data_tag;
    m.sub_sigData.signature_data.which_sig_type =
        (pb_size_t)Signatures_SignatureData_session_info_tag_tag;
    memcpy(m.sub_sigData.signature_data.sig_type.session_info_tag.tag, tag, 32);
    m.request_uuid.size = 16;
    memcpy(m.request_uuid.bytes, challenge, 16);

    return tesla_pb_encode_routable(&m, resp, resp_cap, resp_len);
}

// Encrypt a VCSEC.FromVCSECMessage app payload into an AES_GCM_Response
// RoutableMessage. `id` is the client's request hash.
static int vehicle_encrypt_response(const uint8_t *vcsec_plain, size_t vcsec_len,
                                    uint32_t counter, const uint8_t *id, size_t id_len,
                                    uint8_t *resp, size_t resp_cap, size_t *resp_len)
{
    tesla_metadata_t meta;
    const uint8_t *md_bytes;
    size_t md_len;
    uint8_t aad[32], nonce[12], ct[TESLA_PB_PAYLOAD_MAX], tag[16];

    memset(&meta, 0, sizeof(meta));
    if (tesla_build_response_metadata(&meta, TESLA_DOMAIN_VEHICLE_SECURITY,
                                      (const uint8_t *)VIN, strlen(VIN),
                                      counter, TESLA_FLAG_ENCRYPT_RESPONSE,
                                      id, id_len, 0) != 0) {
        return -1;
    }
    md_bytes = tesla_metadata_serialize(&meta, &md_len);
    if (md_bytes == NULL) return -1;
    if (tesla_sha256(md_bytes, md_len, aad) != 0) return -1;
    if (dummy_rng(NULL, nonce, sizeof(nonce)) != 0) return -1;
    if (tesla_gcm_encrypt(g_K, vcsec_plain, vcsec_len, aad, sizeof(aad), nonce,
                          ct, tag) != 0) {
        return -1;
    }

    UniversalMessage_RoutableMessage m;
    memset(&m, 0, sizeof(m));
    m.has_from_destination = true;
    tesla_pb_dest_domain(&m.from_destination, TESLA_DOMAIN_VEHICLE_SECURITY);
    m.which_payload = (pb_size_t)UniversalMessage_RoutableMessage_protobuf_message_as_bytes_tag;
    m.payload.protobuf_message_as_bytes.size = (pb_size_t)vcsec_len;
    memcpy(m.payload.protobuf_message_as_bytes.bytes, ct, vcsec_len);
    m.which_sub_sigData = (pb_size_t)UniversalMessage_RoutableMessage_signature_data_tag;
    m.sub_sigData.signature_data.which_sig_type =
        (pb_size_t)Signatures_SignatureData_AES_GCM_Response_data_tag;
    {
        Signatures_AES_GCM_Response_Signature_Data *g =
            &m.sub_sigData.signature_data.sig_type.AES_GCM_Response_data;
        memcpy(g->nonce, nonce, 12);
        g->counter = counter;
        memcpy(g->tag, tag, 16);
    }
    m.flags = TESLA_FLAG_ENCRYPT_RESPONSE;
    return tesla_pb_encode_routable(&m, resp, resp_cap, resp_len);
}

static void test_handshake(void)
{
    uint8_t routing[16] = { 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16 };
    uint8_t challenge[16] = { 16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1 };
    uint8_t req[256], resp[256];
    size_t req_len = 0, resp_len = 0;
    tesla_session_t s;
    tesla_keypair_t client;

    memcpy(client.priv, g_c_priv, 32);
    memcpy(client.pub, g_c_pub, 65);
    tesla_session_init(&s, TESLA_DOMAIN_VEHICLE_SECURITY, test_now_ms);

    // The client builds a handshake request...
    CHECK(tesla_build_handshake_request(TESLA_DOMAIN_VEHICLE_SECURITY,
                                        g_c_pub, routing, challenge,
                                        req, sizeof(req), &req_len) == 0,
          "build handshake request");

    // ...the vehicle responds (using shared K it derived from ECDH(v, C))...
    CHECK(vehicle_session_response(challenge, 6, 2650, 7,
                                   Signatures_Session_Info_Status_SESSION_INFO_STATUS_OK,
                                   resp, sizeof(resp), &resp_len) == 0,
          "vehicle builds handshake response");

    // ...and the client derives K and authenticates the session info.
    CHECK(tesla_session_handshake(&s, &client, (const uint8_t *)VIN, strlen(VIN),
                                  challenge, resp, resp_len, dummy_rng, NULL) == 0,
          "handshake completes");
    CHECK(check_hex(s.shared_key, 16, K_HEX), "K == protocol.md 1b2fce...");
    CHECK(s.valid, "session marked valid");
    CHECK(s.counter == 6, "session counter initialised from SessionInfo");
    CHECK(s.handle == 7, "session handle from SessionInfo");
    CHECK(bytes_eq(s.epoch, g_epoch, 16), "session epoch from SessionInfo");
    CHECK(bytes_eq(s.vehicle_pubkey, g_v_pub, 65), "vehicle pubkey recorded");
    CHECK(s.whitelisted, "key reported whitelisted (SessionInfo status OK)");
}

static void test_command_roundtrip(void)
{
    uint8_t routing[16] = { 0xAA,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1 };
    uint8_t uuid[16] = { 0x55,0x51,0x4f,0x57,0x61,0x6b,0xcc,0x81,0xa8,0xce,0x0f,0x9d,0x7b,0x48,0x32,0x29 };
    uint8_t status_payload[64], req[512], resp[512], plain[256];
    uint8_t request_hash[33];
    size_t status_len, req_len, resp_len, req_hash_len, plain_len;
    uint32_t fault = 0xFFFFFFFFu;
    tesla_session_t s;
    tesla_keypair_t client;
    UniversalMessage_RoutableMessage cmd;
    Signatures_AES_GCM_Personalized_Signature_Data *gcm;
    VCSEC_FromVCSECMessage from;
    tesla_vcsec_phase_t phase;

    memcpy(client.priv, g_c_priv, 32);
    memcpy(client.pub, g_c_pub, 65);
    tesla_session_init(&s, TESLA_DOMAIN_VEHICLE_SECURITY, test_now_ms);

    // Establish a session first (reuse the helper path via a direct handshake).
    {
        uint8_t challenge[16] = { 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1 };
        uint8_t resp[256]; size_t rl = 0;
        g_now_ms = 2000000000u;
        CHECK(vehicle_session_response(challenge, 100, 12345, 3,
                                       Signatures_Session_Info_Status_SESSION_INFO_STATUS_OK,
                                       resp, sizeof(resp), &rl) == 0,
              "setup: vehicle handshake response");
        CHECK(tesla_session_handshake(&s, &client, (const uint8_t *)VIN, strlen(VIN),
                                      challenge, resp, rl, dummy_rng, NULL) == 0,
              "setup: handshake");
    }

    // Build the VCSEC GET_STATUS payload.
    CHECK(tesla_pb_encode_vcsec_status(status_payload, sizeof(status_payload),
                                       &status_len) == 0,
          "encode VCSEC GET_STATUS payload");

    // Sign + encrypt + encode the command.
    CHECK(tesla_session_build_command(&s, (const uint8_t *)VIN, strlen(VIN),
                                      TESLA_DOMAIN_VEHICLE_SECURITY,
                                      status_payload, status_len,
                                      routing, uuid,
                                      req, sizeof(req), &req_len,
                                      request_hash, &req_hash_len,
                                      dummy_rng, NULL) == 0,
          "build+encrypt VCSEC GET_STATUS command");
    CHECK(req_hash_len == 17, "VCSEC request hash is 17 bytes (got %u)",
          (unsigned)req_hash_len);
    CHECK(request_hash[0] == (uint8_t)TESLA_SIG_TYPE_AES_GCM_PERSONALIZED,
          "request hash starts with sig_type AES_GCM_PERSONALIZED");
    CHECK(s.counter == 101, "session counter advanced on command");
    CHECK(s.replay_init == false, "replay window not primed until a response");

    // The vehicle decodes and decrypts the command.
    CHECK(tesla_pb_decode_routable(req, req_len, &cmd) == 0,
          "vehicle decodes command routable");
    CHECK(cmd.sub_sigData.signature_data.which_sig_type ==
              (pb_size_t)Signatures_SignatureData_AES_GCM_Personalized_data_tag,
          "command carries AES_GCM_Personalized signature");
    gcm = &cmd.sub_sigData.signature_data.sig_type.AES_GCM_Personalized_data;

    // Vehicle reconstructs request metadata + decrypts the payload.
    {
        tesla_metadata_t meta;
        const uint8_t *mb; size_t mlen;
        uint8_t aad[32], dec[TESLA_PB_PAYLOAD_MAX];
        CHECK(tesla_build_request_metadata(&meta, TESLA_DOMAIN_VEHICLE_SECURITY,
                                           (const uint8_t *)VIN, strlen(VIN),
                                           gcm->epoch, gcm->expires_at,
                                           gcm->counter,
                                           TESLA_FLAG_ENCRYPT_RESPONSE) == 0,
              "vehicle rebuilds request metadata");
        mb = tesla_metadata_serialize(&meta, &mlen);
        CHECK(mb != NULL, "vehicle serializes request metadata");
        CHECK(tesla_sha256(mb, mlen, aad) == 0, "vehicle computes AAD");
        CHECK(tesla_gcm_decrypt(g_K, cmd.payload.protobuf_message_as_bytes.bytes,
                                cmd.payload.protobuf_message_as_bytes.size,
                                aad, sizeof(aad), gcm->nonce, gcm->tag,
                                dec) == 0,
              "vehicle decrypts command payload");
        // It must be the GET_STATUS UnsignedMessage we sent.
        CHECK(bytes_eq(dec, status_payload, status_len),
              "vehicle sees the exact GET_STATUS payload we sent");
        CHECK(gcm->counter == 101, "vehicle sees counter=101");
        CHECK(gcm->expires_at >= 12345 && gcm->expires_at <= 12345 + TESLA_SESSION_VALIDITY_S,
              "vehicle sees sane expiry (got %u)", (unsigned)gcm->expires_at);
    }

    // Vehicle encrypts its VCSEC status response (using shared K + our request hash).
    {
        VCSEC_FromVCSECMessage vmsg;
        uint8_t vpayload[128]; size_t vlen;
        memset(&vmsg, 0, sizeof(vmsg));
        vmsg.which_sub_message = (pb_size_t)VCSEC_FromVCSECMessage_vehicleStatus_tag;
        vmsg.sub_message.vehicleStatus.vehicleLockState =
            VCSEC_VehicleLockState_E_VEHICLELOCKSTATE_LOCKED;
        vmsg.sub_message.vehicleStatus.vehicleSleepStatus =
            VCSEC_VehicleSleepStatus_E_VEHICLE_SLEEP_STATUS_AWAKE;
        vmsg.sub_message.vehicleStatus.userPresence =
            VCSEC_UserPresence_E_VEHICLE_USER_PRESENCE_PRESENT;
        {
            pb_ostream_t os = pb_ostream_from_buffer(vpayload, sizeof(vpayload));
            CHECK(pb_encode(&os, VCSEC_FromVCSECMessage_fields, &vmsg),
                  "vehicle encodes status response payload");
            vlen = os.bytes_written;
        }
        CHECK(vehicle_encrypt_response(vpayload, vlen, 9001,
                                       request_hash, req_hash_len,
                                       resp, sizeof(resp), &resp_len) == 0,
              "vehicle encrypts status response");
    }

    // Client processes the response: decrypts, validates, exposes no fault.
    CHECK(tesla_session_process_response(&s, (const uint8_t *)VIN, strlen(VIN),
                                         request_hash, req_hash_len,
                                         resp, resp_len,
                                         plain, sizeof(plain), &plain_len,
                                         &fault) == 0,
          "client decrypts + validates status response");
    CHECK(fault == 0, "no protocol fault");
    CHECK(tesla_pb_decode_vcsec_from(plain, plain_len, &from) == 0,
          "client decodes VCSEC.FromVCSECMessage");
    CHECK(from.which_sub_message == (pb_size_t)VCSEC_FromVCSECMessage_vehicleStatus_tag,
          "response carries vehicleStatus");
    CHECK(from.sub_message.vehicleStatus.vehicleLockState ==
              VCSEC_VehicleLockState_E_VEHICLELOCKSTATE_LOCKED,
          "lock state decoded [LOCKED]");
    phase = tesla_vcsec_ingest(&from);
    CHECK(phase == TESLA_VCSEC_STATUS, "state machine: vehicleStatus -> STATUS");

    // Replay protection: the identical response must now be rejected.
    {
        size_t dummy_len = 0; uint32_t dummy_fault = 0;
        CHECK(tesla_session_process_response(&s, (const uint8_t *)VIN, strlen(VIN),
                                             request_hash, req_hash_len,
                                             resp, resp_len,
                                             plain, sizeof(plain), &dummy_len,
                                             &dummy_fault) == -3,
              "replayed response rejected");
    }

    // A second, fresh vehicle response (new counter) for the negative cases.
    {
        VCSEC_FromVCSECMessage vmsg;
        uint8_t vpayload[128]; size_t vlen = 0;
        uint8_t resp2[512]; size_t resp2_len = 0;

        memset(&vmsg, 0, sizeof(vmsg));
        vmsg.which_sub_message = (pb_size_t)VCSEC_FromVCSECMessage_vehicleStatus_tag;
        vmsg.sub_message.vehicleStatus.vehicleLockState =
            VCSEC_VehicleLockState_E_VEHICLELOCKSTATE_UNLOCKED;
        {
            pb_ostream_t os = pb_ostream_from_buffer(vpayload, sizeof(vpayload));
            if (!pb_encode(&os, VCSEC_FromVCSECMessage_fields, &vmsg)) {
                printf("FAIL: encode status for negative case\n"); g_fail++;
                return;
            }
            vlen = os.bytes_written;
        }
        CHECK(vehicle_encrypt_response(vpayload, vlen, 9002,
                                       request_hash, req_hash_len,
                                       resp2, sizeof(resp2), &resp2_len) == 0 &&
              resp2_len > 0,
              "negative: vehicle builds fresh response (counter 9002)");

        // Tamper detection: re-encode the response with the domain taken from
        // the ciphertext-authenticated metadata (from_destination) changed.
        // The client authenticates that field, so GCM must fail.
        {
            UniversalMessage_RoutableMessage tm;
            uint8_t tampered[512]; size_t tl = 0; size_t dummy_len = 0;
            uint32_t dummy_fault = 0;
            if (tesla_pb_decode_routable(resp2, resp2_len, &tm) == 0) {
                tm.from_destination.which_sub_destination =
                    (pb_size_t)UniversalMessage_Destination_domain_tag;
                tm.from_destination.sub_destination.domain =
                    UniversalMessage_Domain_DOMAIN_INFOTAINMENT;
                if (tesla_pb_encode_routable(&tm, tampered, sizeof(tampered), &tl) == 0) {
                    CHECK(tesla_session_process_response(&s, (const uint8_t *)VIN, strlen(VIN),
                                                         request_hash, req_hash_len,
                                                         tampered, tl,
                                                         plain, sizeof(plain), &dummy_len,
                                                         &dummy_fault) == -1,
                          "tampered domain rejected (GCM auth)");
                } else {
                    printf("FAIL: re-encode tampered response\n"); g_fail++;
                }
            } else {
                printf("FAIL: decode response for tamper test\n"); g_fail++;
            }
        }

        // Wrong request hash -> AAD mismatch -> decryption fails. Uses a fresh
        // response (counter 9003, above the tamper test's 9002) so the replay
        // gate doesn't pre-empt the GCM-auth check.
        {
            uint8_t resp3[512]; size_t resp3_len = 0;
            uint8_t bad_id[17] = {0}; size_t dummy_len = 0; uint32_t dummy_fault = 0;
            CHECK(vehicle_encrypt_response(vpayload, vlen, 9003,
                                           request_hash, req_hash_len,
                                           resp3, sizeof(resp3), &resp3_len) == 0 &&
                  resp3_len > 0,
                  "negative: vehicle builds fresh response (counter 9003)");
            CHECK(tesla_session_process_response(&s, (const uint8_t *)VIN, strlen(VIN),
                                                 bad_id, sizeof(bad_id),
                                                 resp3, resp3_len,
                                                 plain, sizeof(plain), &dummy_len,
                                                 &dummy_fault) == -1,
                  "response with wrong request hash rejected");
        }
    }
}

static void test_vcsec_state_machine(void)
{
    VCSEC_FromVCSECMessage m;

    memset(&m, 0, sizeof(m));
    m.which_sub_message = (pb_size_t)VCSEC_FromVCSECMessage_vehicleStatus_tag;
    CHECK(tesla_vcsec_ingest(&m) == TESLA_VCSEC_STATUS,
          "vehicleStatus -> STATUS");

    memset(&m, 0, sizeof(m));
    m.which_sub_message = (pb_size_t)VCSEC_FromVCSECMessage_commandStatus_tag;
    m.sub_message.commandStatus.operationStatus =
        VCSEC_OperationStatus_E_OPERATIONSTATUS_WAIT;
    CHECK(tesla_vcsec_ingest(&m) == TESLA_VCSEC_PENDING,
          "WAIT -> PENDING (busy, keep collecting)");

    memset(&m, 0, sizeof(m));
    m.which_sub_message = (pb_size_t)VCSEC_FromVCSECMessage_commandStatus_tag;
    m.sub_message.commandStatus.operationStatus =
        VCSEC_OperationStatus_E_OPERATIONSTATUS_ERROR;
    CHECK(tesla_vcsec_ingest(&m) == TESLA_VCSEC_PENDING,
          "OPERATIONSTATUS_ERROR -> PENDING (wait for specific error)");

    memset(&m, 0, sizeof(m));
    m.which_sub_message = (pb_size_t)VCSEC_FromVCSECMessage_commandStatus_tag;
    m.sub_message.commandStatus.which_sub_message =
        (pb_size_t)VCSEC_CommandStatus_whitelistOperationStatus_tag;
    CHECK(tesla_vcsec_ingest(&m) == TESLA_VCSEC_DONE,
          "whitelistOperationStatus -> DONE (terminal)");

    memset(&m, 0, sizeof(m));
    m.which_sub_message = (pb_size_t)VCSEC_FromVCSECMessage_commandStatus_tag;
    m.sub_message.commandStatus.operationStatus =
        VCSEC_OperationStatus_E_OPERATIONSTATUS_OK;
    CHECK(tesla_vcsec_ingest(&m) == TESLA_VCSEC_DONE,
          "signedMessageStatus OK -> DONE");

    memset(&m, 0, sizeof(m));
    m.which_sub_message = (pb_size_t)VCSEC_FromVCSECMessage_nominalError_tag;
    m.sub_message.nominalError.genericError = 1;
    CHECK(tesla_vcsec_ingest(&m) == TESLA_VCSEC_ERROR,
          "nominalError -> ERROR");

    // Empty message: terminal success (no application payload).
    memset(&m, 0, sizeof(m));
    CHECK(tesla_vcsec_ingest(&m) == TESLA_VCSEC_DONE,
          "empty -> DONE (success)");
}

static void test_handshake_negative(void)
{
    uint8_t challenge[16] = {0xAB,0xAB,0xAB,0xAB,0xAB,0xAB,0xAB,0xAB,
                             0xAB,0xAB,0xAB,0xAB,0xAB,0xAB,0xAB,0xAB};
    uint8_t resp[256]; size_t resp_len = 0;
    tesla_session_t s;
    tesla_keypair_t client;

    memcpy(client.priv, g_c_priv, 32);
    memcpy(client.pub, g_c_pub, 65);
    tesla_session_init(&s, TESLA_DOMAIN_VEHICLE_SECURITY, test_now_ms);

    CHECK(vehicle_session_response(challenge, 6, 2650, 7,
                                   Signatures_Session_Info_Status_SESSION_INFO_STATUS_OK,
                                   resp, sizeof(resp), &resp_len) == 0,
          "neg-handshake: build valid handshake response");

    // Corrupt one byte of the HMAC session-info tag: authentication must fail
    // and the session must not be trusted.
    {
        UniversalMessage_RoutableMessage m;
        uint8_t tampered[256]; size_t tl = 0;
        if (tesla_pb_decode_routable(resp, resp_len, &m) == 0) {
            m.sub_sigData.signature_data.sig_type.session_info_tag.tag[0] ^= 0xFF;
            if (tesla_pb_encode_routable(&m, tampered, sizeof(tampered), &tl) == 0) {
                CHECK(tesla_session_handshake(&s, &client,
                                              (const uint8_t *)VIN, strlen(VIN),
                                              challenge, tampered, tl,
                                              dummy_rng, NULL) != 0,
                      "neg-handshake: tampered session-info HMAC rejected");
            } else { printf("FAIL: re-encode tampered handshake\n"); g_fail++; }
        } else { printf("FAIL: decode handshake for tamper test\n"); g_fail++; }
    }
    CHECK(!s.valid, "neg-handshake: session stays invalid after rejected handshake");

    // A KEY_NOT_ON_WHITELIST SessionInfo is still a valid-HMAC session, but it
    // must be surfaced as not-whitelisted so the client fires the enroll
    // canary instead of a misleading "handshake complete" (review S5).
    {
        uint8_t resp2[256]; size_t rl2 = 0;
        tesla_session_t s2;
        tesla_keypair_t c2;
        memcpy(c2.priv, g_c_priv, 32);
        memcpy(c2.pub, g_c_pub, 65);
        CHECK(vehicle_session_response(challenge, 6, 2650, 7,
                                       Signatures_Session_Info_Status_SESSION_INFO_STATUS_KEY_NOT_ON_WHITELIST,
                                       resp2, sizeof(resp2), &rl2) == 0,
              "neg-handshake: build not-whitelisted handshake response");
        tesla_session_init(&s2, TESLA_DOMAIN_VEHICLE_SECURITY, test_now_ms);
        CHECK(tesla_session_handshake(&s2, &c2, (const uint8_t *)VIN, strlen(VIN),
                                      challenge, resp2, rl2, dummy_rng, NULL) == 0,
              "neg-handshake: valid-HMAC handshake completes for non-whitelisted key");
        CHECK(s2.whitelisted == false,
              "neg-handshake: key marked NOT whitelisted");
    }
}

static void test_replay_window(void)
{
    // Fresh VCSEC session; SessionInfo sets vehicle clock + counter.
    tesla_session_t s;
    tesla_keypair_t client;
    uint8_t vpayload[2] = {0xAA, 0xBB};
    uint8_t plain[256];

    memcpy(client.priv, g_c_priv, 32);
    memcpy(client.pub, g_c_pub, 65);
    g_now_ms = 2000000000u;
    tesla_session_init(&s, TESLA_DOMAIN_VEHICLE_SECURITY, test_now_ms);
    {
        uint8_t challenge[16] = {2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2};
        uint8_t resp[256]; size_t rl = 0;
        CHECK(vehicle_session_response(challenge, 500, 9000, 9,
                                       Signatures_Session_Info_Status_SESSION_INFO_STATUS_OK,
                                       resp, sizeof(resp), &rl) == 0,
              "rw: vehicle handshake response");
        CHECK(tesla_session_handshake(&s, &client, (const uint8_t *)VIN, strlen(VIN),
                                      challenge, resp, rl, dummy_rng, NULL) == 0,
              "rw: handshake");
    }

    // Build a command so we have a real request-hash binding for the responses.
    uint8_t routing[16] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
    uint8_t uuid[16]    = {9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9};
    uint8_t req[512]; size_t req_len = 0;
    uint8_t request_hash[33]; size_t req_hash_len = 0;
    CHECK(tesla_session_build_command(&s, (const uint8_t *)VIN, strlen(VIN),
                                      TESLA_DOMAIN_VEHICLE_SECURITY,
                                      vpayload, sizeof(vpayload), routing, uuid,
                                      req, sizeof(req), &req_len,
                                      request_hash, &req_hash_len,
                                      dummy_rng, NULL) == 0,
          "rw: build command (request-hash binding)");

    // Build an encrypted response for counter _c and show it in (*_olen).
    #define RW_BUILD(_c, _out, _olen) do { \
        size_t _rl = 0; \
        if (vehicle_encrypt_response(vpayload, sizeof(vpayload), (_c), \
                                     request_hash, req_hash_len, \
                                     (_out), 512, &_rl) != 0 || _rl == 0) { \
            printf("FAIL: rw: vehicle response build c=" #_c "\n"); g_fail++; \
        } else { *(_olen) = _rl; } \
    } while (0)

    // (1) First legitimate response primes the replay window.
    {
        uint8_t resp[512]; size_t rl = 0, pl = 0; uint32_t fl = 0;
        RW_BUILD(501, resp, &rl);
        CHECK(tesla_session_process_response(&s, (const uint8_t *)VIN, strlen(VIN),
                                             request_hash, req_hash_len,
                                             resp, rl, plain, sizeof(plain), &pl, &fl) == 0,
              "rw: legit first response accepted (primes window)");
    }

    // (2) C1: a forged frame with a sky-high counter that FAILS GCM auth must
    //     NOT advance the window; the next legitimate response still works.
    {
        uint8_t forged[512]; size_t frl = 0;
        uint8_t resp[512]; size_t rl = 0, pl = 0; uint32_t fl = 0;
        RW_BUILD(0xFFFFFFF0u, forged, &frl);
        {
            UniversalMessage_RoutableMessage m;
            uint8_t tmp[512]; size_t tl = 0;
            if (tesla_pb_decode_routable(forged, frl, &m) == 0) {
                m.sub_sigData.signature_data.sig_type.AES_GCM_Response_data.tag[0] ^= 0xFF;
                if (tesla_pb_encode_routable(&m, tmp, sizeof(tmp), &tl) == 0) {
                    CHECK(tesla_session_process_response(&s, (const uint8_t *)VIN, strlen(VIN),
                                                         request_hash, req_hash_len,
                                                         tmp, tl, plain, sizeof(plain),
                                                         &pl, &fl) == -1,
                          "rw: forged high-counter frame rejected by auth");
                }
            }
        }
        RW_BUILD(502, resp, &rl);
        CHECK(tesla_session_process_response(&s, (const uint8_t *)VIN, strlen(VIN),
                                             request_hash, req_hash_len,
                                             resp, rl, plain, sizeof(plain), &pl, &fl) == 0,
              "rw: legit response still accepted after forged frame (C1)");
    }

    #undef RW_BUILD

    // (3) uint32 counter rollover: a response counter of 0xFFFFFFFF (max) is
    //     accepted as the first authenticated response of a fresh session.
    {
        tesla_session_t s2;
        tesla_keypair_t c2;
        uint8_t ch[16] = {3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3};
        uint8_t hs[256]; size_t hsl = 0;
        memcpy(c2.priv, g_c_priv, 32);
        memcpy(c2.pub, g_c_pub, 65);
        tesla_session_init(&s2, TESLA_DOMAIN_VEHICLE_SECURITY, test_now_ms);
        CHECK(vehicle_session_response(ch, 0xFFFFFFFEu, 9000, 1,
                                       Signatures_Session_Info_Status_SESSION_INFO_STATUS_OK,
                                       hs, sizeof(hs), &hsl) == 0 &&
              tesla_session_handshake(&s2, &c2, (const uint8_t *)VIN, strlen(VIN),
                                      ch, hs, hsl, dummy_rng, NULL) == 0,
              "rw-rollover: handshake with session counter near max");

        uint8_t r2[16] = {4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4};
        uint8_t u2[16] = {5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5};
        uint8_t req2[512]; size_t req2_len = 0, rh2_len = 0;
        uint8_t rh2[33];
        CHECK(tesla_session_build_command(&s2, (const uint8_t *)VIN, strlen(VIN),
                                          TESLA_DOMAIN_VEHICLE_SECURITY,
                                          vpayload, sizeof(vpayload), r2, u2,
                                          req2, sizeof(req2), &req2_len,
                                          rh2, &rh2_len, dummy_rng, NULL) == 0,
              "rw-rollover: build command");
        {
            uint8_t rr[512]; size_t rrl = 0, pl = 0; uint32_t fl = 0;
            if (vehicle_encrypt_response(vpayload, sizeof(vpayload), 0xFFFFFFFFu,
                                         rh2, rh2_len, rr, sizeof(rr), &rrl) == 0 &&
                rrl > 0) {
                CHECK(tesla_session_process_response(&s2, (const uint8_t *)VIN, strlen(VIN),
                                                     rh2, rh2_len,
                                                     rr, rrl, plain, sizeof(plain), &pl, &fl) == 0,
                      "rw-rollover: 0xFFFFFFFF first response accepted");
            } else { printf("FAIL: rw-rollover: build response\n"); g_fail++; }
        }
    }
}

static void test_protobuf_primitives(void)
{
    // Encoding a whitelist op and a status payload must round-trip and carry
    // the expected role / request type.
    uint8_t buf[128], out[128]; size_t n, o;
    VCSEC_UnsignedMessage um;
    VCSEC_FromVCSECMessage fm;
    pb_istream_t in;

    CHECK(tesla_pb_encode_vcsec_status(buf, sizeof(buf), &n) == 0 && n > 0,
          "status payload encodes");
    in = pb_istream_from_buffer(buf, n);
    memset(&um, 0, sizeof(um));
    CHECK(pb_decode(&in, VCSEC_UnsignedMessage_fields, &um), "status decodes");
    CHECK(um.which_sub_message == (pb_size_t)VCSEC_UnsignedMessage_InformationRequest_tag &&
          um.sub_message.InformationRequest.informationRequestType ==
              VCSEC_InformationRequestType_INFORMATION_REQUEST_TYPE_GET_STATUS,
          "status is InformationRequest/GET_STATUS");

    // Whitelist builder (Phase 3): role + key + form factor must survive.
    uint8_t pk[65];
    unhex(CLIENT_PUB_HEX, pk, sizeof(pk));
    CHECK(tesla_pb_encode_vcsec_whitelist(pk, Keys_Role_ROLE_CHARGING_MANAGER,
                                          VCSEC_KeyFormFactor_KEY_FORM_FACTOR_ANDROID_DEVICE,
                                          buf, sizeof(buf), &n) == 0 && n > 0,
          "whitelist payload encodes");
    in = pb_istream_from_buffer(buf, n);
    memset(&um, 0, sizeof(um));
    CHECK(pb_decode(&in, VCSEC_UnsignedMessage_fields, &um), "whitelist decodes");
    CHECK(um.which_sub_message == (pb_size_t)VCSEC_UnsignedMessage_WhitelistOperation_tag,
          "whitelist op present");
    CHECK(um.sub_message.WhitelistOperation.which_sub_message ==
              (pb_size_t)VCSEC_WhitelistOperation_addKeyToWhitelistAndAddPermissions_tag &&
          um.sub_message.WhitelistOperation.sub_message
              .addKeyToWhitelistAndAddPermissions.keyRole == Keys_Role_ROLE_CHARGING_MANAGER,
          "addKeyToWhitelistAndAddPermissions carries CHARGING_MANAGER role");
    CHECK(um.sub_message.WhitelistOperation.has_metadataForKey &&
          um.sub_message.WhitelistOperation.metadataForKey.keyFormFactor ==
              VCSEC_KeyFormFactor_KEY_FORM_FACTOR_ANDROID_DEVICE,
          "whitelist op carries ANDROID_DEVICE form factor");
    (void)out; (void)o; (void)fm;
}

int main(void)
{
    printf("Tesla BLE Phase 2 protocol round-trip tests\n");
    printf("-------------------------------------------\n");

    unhex(CLIENT_PRIV_HEX, g_c_priv, sizeof(g_c_priv));
    unhex(CLIENT_PUB_HEX, g_c_pub, sizeof(g_c_pub));
    unhex(VEHICLE_PRIV_HEX, g_v_priv, sizeof(g_v_priv));
    unhex(VEHICLE_PUB_HEX, g_v_pub, sizeof(g_v_pub));
    unhex(K_HEX, g_K, sizeof(g_K));

    // Cross-check the published K against our ECDH (both directions).
    {
        uint8_t k1[16], k2[16];
        CHECK(tesla_derive_shared_key(g_c_priv, g_v_pub, dummy_rng, NULL, k1) == 0 &&
              check_hex(k1, 16, K_HEX),
              "ECDH(c, V) derives protocol.md K");
        CHECK(tesla_derive_shared_key(g_v_priv, g_c_pub, dummy_rng, NULL, k2) == 0 &&
              check_hex(k2, 16, K_HEX),
              "ECDH(v, C) derives protocol.md K");
    }

    test_protobuf_primitives();
    test_handshake();
    test_handshake_negative();
    test_command_roundtrip();
    test_vcsec_state_machine();
    test_replay_window();

    printf("\n%d failure(s)\n", g_fail);
    return g_fail ? 1 : 0;
}
