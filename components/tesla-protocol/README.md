# tesla-protocol

Tesla vehicle-command protocol support for DashKit, built as an auto-discovered
ESP-IDF component. Implements the client side of the vehicle-command BLE
protocol (matching the Apache-2.0 `teslamotors/vehicle-command` reference).

## Layout

| Path | Contents |
| --- | --- |
| `crypto.c/.h` | P-256 ECDH, SHA-1 KDF, SHA-256/HMAC, AES-128-GCM (mbedTLS 3.x) |
| `session.c/.h` | metadata TLV sort/build, session-info auth, request hash |
| `protos/` | Apache-2.0 `.proto` schemas from `teslamotors/vehicle-command`, pinned in `protos/VERSION` |
| `nanopb/` | vendored nanopb 0.4.9.1 runtime (BSD-3-Clause) |
| `generated/` | committed nanopb bindings |

## Status

- Crypto + session layers, vendored protos/nanopb, and a host-side unit test
  (`tools/test/test_tesla_crypto.c`) validating against Tesla's published
  known-answer vectors.
- nanopb bindings (`generated/`), protobuf builders, and the NimBLE central
  adapter (used by `main/tesla/`).

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
is included; they are treated as test-oracles only.
