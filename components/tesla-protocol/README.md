# tesla-protocol

Tesla vehicle-command protocol support for DashKit, built as an auto-discovered
ESP-IDF component. Implements the client side of the vehicle-command BLE
protocol per the Tesla BLE integration plan and ADR 0001 (the plan document
and ADR live in the docs/ directory of the parent workspace that contains this
firmware tree).

## Layout

| Path | Contents |
| --- | --- |
| `crypto.c/.h` | P-256 ECDH, SHA-1 KDF, SHA-256/HMAC, AES-128-GCM (mbedTLS 3.x) |
| `session.c/.h` | metadata TLV sort/build, session-info auth, request hash |
| `protos/` | Apache-2.0 `.proto` schemas from `teslamotors/vehicle-command`, pinned in `protos/VERSION` |
| `nanopb/` | vendored nanopb 0.4.9.1 runtime (BSD-3-Clause) |
| `generated/` | committed nanopb bindings (arrives in Phase 2) |

## Phase status (per the integration plan)

- **Phase 0 (this branch):** crypto + session layers, vendored protos/nanopb,
  and a host-side unit test (`tools/test/test_tesla_crypto.c`) validating
  against Tesla's published known-answer vectors. Nothing here touches BLE.
- **Phase 2+:** nanopb bindings (`generated/`), protobuf builders, and the
  NimBLE central adapter.

## Testing

```
tools/test/run_tesla_crypto_test.sh
```

Builds a pinned mbedTLS 3.6.2 into a local prefix (same major.minor as the
ESP-IDF 5.4.1 vendored copy), compiles `crypto.c`/`session.c` with plain gcc,
and runs the known-answer tests. Requires gcc + make + curl on a Linux host
(or WSL).

## Licensing

- `protos/` — Apache-2.0, `teslamotors/vehicle-command` (pinned in `VERSION`)
- `nanopb/` — BSD-3-Clause, `nanopb/nanopb` 0.4.9.1
- `crypto.c/.h`, `session.c/.h`, this file — project code, Apache-2.0

No code from AGPL-licensed reference implementations (ESPHome fork, pmdroid)
is included; they are treated as test-oracle-only per ADR 0001.
