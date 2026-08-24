# protos/

Vendored Tesla protocol schemas from `teslamotors/vehicle-command`
(`pkg/protocol/protobuf/*.proto`), Apache-2.0. The upstream commit is pinned in
[`VERSION`](VERSION); update both together when bumping.

These files are the schema source for the nanopb bindings committed under
`../generated/`. No vehicle-command *Go* code is vendored here;
the reference implementation is Apache-2.0 and is only used to cross-check the
crypto layer (see the unit test in `tools/test/test_tesla_crypto.c`).

## Files

- `universal_message.proto` - `RoutableMessage`, domains, flags, faults
- `signatures.proto` - metadata tags, signature types, session info
- `vcsec.proto` - application payloads (status, whitelist operations)
- `keys.proto`, `errors.proto` - key roles / error codes

Not vendored: `vehicle.proto`, `car_server.proto`, `managed_charging.proto`,
`common.proto` (the Infotainment domain) and `google/protobuf/timestamp.proto`.
None is imported by the schemas above; re-copy them from the same pinned
commit when the Infotainment phase lands.
