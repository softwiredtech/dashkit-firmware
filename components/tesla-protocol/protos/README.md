# protos/

Vendored Tesla protocol schemas from `teslamotors/vehicle-command`
(`pkg/protocol/protobuf/*.proto`), Apache-2.0. The upstream commit is pinned in
[`VERSION`](VERSION); update both together when bumping.

These files are the schema source for the nanopb bindings committed under
`../generated/` (Phase 2+). No vehicle-command *Go* code is vendored here;
the reference implementation is Apache-2.0 and is only used to cross-check the
crypto layer (see the Phase 0 unit test in `tools/test/test_tesla_crypto.c`).

## Files

- `universal_message.proto` — `RoutableMessage`, domains, flags, faults
- `signatures.proto` — metadata tags, signature types, session info
- `vcsec.proto`, `vehicle.proto`, `car_server.proto`, `managed_charging.proto`
  — application payloads
- `keys.proto` — key roles (Owner, Driver, Charging Manager, ...)
- `common.proto`, `errors.proto` — shared types
