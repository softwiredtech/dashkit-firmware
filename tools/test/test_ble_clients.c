#include "ble_clients.h"

#define CHECK(expr) do { if (!(expr)) return __LINE__; } while (0)

static int test_ble_clients(void)
{
    ble_app_conn_t conns[2] = {
        {.handle = BLE_APP_NO_CONN}, {.handle = BLE_APP_NO_CONN}
    };
    CHECK(!ble_app_has_receivers(conns));
    CHECK(!ble_app_can_write(&conns[0]));
    // Handle zero is valid, but an unencrypted subscriber cannot write.
    conns[0].handle = 0;
    conns[0].subscribed = true;
    CHECK(!ble_app_can_receive(&conns[0]));
    CHECK(!ble_app_can_write(&conns[0]));
    conns[0].encrypted = true;
    CHECK(ble_app_can_write(&conns[0]));
    conns[1].handle = 7;
    conns[1].encrypted = true;
    CHECK(!ble_app_can_receive(&conns[1]));
    CHECK(ble_app_can_write(&conns[1])); // Commands do not require a CAN subscription.
    conns[1].subscribed = true;
    CHECK(ble_app_can_receive(&conns[0]) && ble_app_can_receive(&conns[1]));
    CHECK(ble_app_can_write(&conns[0]) && ble_app_can_write(&conns[1]));
    conns[0].subscribed = false;
    CHECK(ble_app_can_write(&conns[0]) && ble_app_can_write(&conns[1]));
    conns[0].subscribed = true;
    conns[1].handle = BLE_APP_NO_CONN;
    CHECK(ble_app_can_write(&conns[0]) && !ble_app_can_write(&conns[1]));
    conns[0].encrypted = false;
    CHECK(!ble_app_can_write(&conns[0]));

    // Opting out affects only the sender; both keep receiving.
    conns[0].encrypted = true;
    conns[1].handle = 7;
    conns[0].read_only = true;
    CHECK(ble_app_can_receive(&conns[0]));
    CHECK(!ble_app_can_write(&conns[0]) && ble_app_can_write(&conns[1]));
    conns[1].handle = BLE_APP_NO_CONN;
    CHECK(!ble_app_can_write(&conns[0])); // An observer stays read-only when alone.
    CHECK(ble_app_has_receivers(conns)); // The remaining observer still streams.
    conns[1].handle = 7;
    conns[1].read_only = true;
    CHECK(!ble_app_can_write(&conns[0]) && !ble_app_can_write(&conns[1]));
    CHECK(ble_app_can_receive(&conns[0]) && ble_app_can_receive(&conns[1]));
    conns[0].read_only = false;
    CHECK(ble_app_can_write(&conns[0]) && !ble_app_can_write(&conns[1]));
    conns[1].read_only = false;
    CHECK(ble_app_can_write(&conns[0]) && ble_app_can_write(&conns[1]));

    // Three wire frames, with timestamp/bus/address bytes distinct per frame.
    uint8_t batch[42] = {3};
    const size_t starts[] = {1, 19, 29};
    const uint8_t dlcs[] = {8, 0, 3};
    for (size_t f = 0; f < 3; f++) {
        for (size_t j = 0; j < (size_t)10 + dlcs[f]; j++) {
            batch[starts[f] + j] = (uint8_t)(starts[f] + j);
        }
        batch[starts[f] + 9] = dlcs[f];
    }
    CHECK(ble_app_valid_batch(batch, sizeof(batch)));
    CHECK(!ble_app_valid_batch(NULL, 0));
    CHECK(!ble_app_valid_batch(batch, 41));
    CHECK(!ble_app_valid_batch(batch, 1));
    batch[0] = 2;
    CHECK(!ble_app_valid_batch(batch, sizeof(batch)));
    batch[0] = 3;
    batch[10] = 9;
    CHECK(!ble_app_valid_batch(batch, sizeof(batch)));
    batch[10] = 8;
    batch[0] = 0;
    CHECK(!ble_app_valid_batch(batch, sizeof(batch)));
    batch[0] = 3;

    // Two peers negotiate MTU 23 and 512: same frames, different batching.
    uint8_t packet[512];
    size_t offset = 1;
    for (size_t f = 0; f < 3; f++) {
        size_t size = ble_app_next_packet(batch, sizeof(batch), &offset, packet, 20);
        CHECK(size == (size_t)11 + dlcs[f]);
        CHECK(packet[0] == 1);
        CHECK(offset == starts[f] + 10 + dlcs[f]);
        for (size_t j = 1; j < size; j++) CHECK(packet[j] == batch[starts[f] + j - 1]);
    }
    CHECK(offset == sizeof(batch));
    offset = 1;
    CHECK(ble_app_next_packet(batch, sizeof(batch), &offset, packet, 509) == sizeof(batch));
    CHECK(offset == sizeof(batch));
    for (size_t j = 0; j < sizeof(batch); j++) CHECK(packet[j] == batch[j]);
    offset = 1;
    CHECK(ble_app_next_packet(batch, sizeof(batch), &offset, packet, 29) == 29);
    CHECK(packet[0] == 2 && offset == 29);
    CHECK(ble_app_next_packet(batch, sizeof(batch), &offset, packet, 29) == 14);
    CHECK(packet[0] == 1 && offset == sizeof(batch));
    offset = 1;
    CHECK(ble_app_next_packet(batch, sizeof(batch), &offset, packet, 18) == 0);
    CHECK(offset == 1);
    return 0;
}

int main(void)
{
    return test_ble_clients();
}
