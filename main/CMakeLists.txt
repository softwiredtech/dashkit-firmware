# Tesla BLE vehicle-command client (Phase 2: central connect + VCSEC handshake
# + GET_STATUS). Gated by DASHKIT_TESLA_BLE so a feature-off build drops the
# tesla/ modules, the observer *and* central roles, and the storage dependency.
# tesla_ble_adapter depends on NimBLE's scan/central API; tesla_advert_name is
# pure C (used by the host test suite); tesla_ble_storage uses NVS;
# tesla_ble_client uses the tesla-protocol session/protobuf layers.
set(TESLA_SRCS "")
if(CONFIG_DASHKIT_TESLA_BLE)
    list(APPEND TESLA_SRCS
        "tesla/tesla_advert_name.c"
        "tesla/tesla_ble_adapter.c"
        "tesla/tesla_ble_storage.c"
        "tesla/tesla_ble_client.c"
    )
endif()

idf_component_register(
    SRCS
        "main.c"
        "led.c"
        "can/mcp251xfd.c"
        "can/can_manager.c"
        "can/can_filter.c"
        "can/dbc/dbc.c"
        "can/dbc/dbc_generated.c"
        "controls/vehicle_control.c"
        "controls/battery_preheat.c"
        "automation/automation_manager.c"
        "automation/wiper_off.c"
        "automation/multi_finger.c"
        "ble/ble_server.c"
        "ble/ble_ota.c"
        ${TESLA_SRCS}
    INCLUDE_DIRS
        "."
        "can"
        "can/dbc"
        "controls"
        "automation"
        "ble"
        "tesla"
    REQUIRES
        driver
        esp_adc
        nvs_flash
        bt
        esp_timer
        app_update
        # Tesla vehicle-command protocol component (committed nanopb bindings +
        # crypto/session/protobuf-build). mbedtls is an explicit dependency of
        # the main component because the Tesla client drives the crypto layer.
        mbedtls
        tesla-protocol
)

# DBC codegen: regenerate the committed dbc_generated.{c,h} from the vendored
# .dbc + whitelist when a python venv with cantools is available. The generated
# files are committed, so a clean (python-less) checkout still builds without
# this step. OUTPUT paths match the committed paths so an in-tree rebuild simply
# refreshes them in place.
set(DBC_DIR        "${CMAKE_CURRENT_SOURCE_DIR}/../dbc")
set(DBC_TOOLS_DIR  "${CMAKE_CURRENT_SOURCE_DIR}/../tools")
set(DBC_GEN_C      "${CMAKE_CURRENT_SOURCE_DIR}/can/dbc/dbc_generated.c")
set(DBC_GEN_H      "${CMAKE_CURRENT_SOURCE_DIR}/can/dbc/dbc_generated.h")
find_program(DBC_PY python3 PATHS "${DBC_TOOLS_DIR}/.venv/bin" NO_DEFAULT_PATH)
if(DBC_PY)
    add_custom_command(
        OUTPUT  "${DBC_GEN_C}" "${DBC_GEN_H}"
        COMMAND "${DBC_PY}" "${DBC_TOOLS_DIR}/gen_dbc.py"
                --dbc       "${DBC_DIR}/bus_1_tesla_vehicle.dbc"
                --whitelist "${DBC_DIR}/used_messages.txt"
                --out-c     "${DBC_GEN_C}"
                --out-h     "${DBC_GEN_H}"
        DEPENDS "${DBC_DIR}/bus_1_tesla_vehicle.dbc"
                "${DBC_DIR}/used_messages.txt"
                "${DBC_TOOLS_DIR}/gen_dbc.py"
        COMMENT "Regenerating DBC tables (dbc_generated.{c,h})"
        VERBATIM
    )
    add_custom_target(gen_dbc DEPENDS "${DBC_GEN_C}" "${DBC_GEN_H}")
    add_dependencies(${COMPONENT_LIB} gen_dbc)
endif()
