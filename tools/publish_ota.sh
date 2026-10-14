#!/usr/bin/env bash
#
# Publish a DashKit firmware release to Firebase Storage.
#
# Uploads the .bin and a manifest.json to gs://<BUCKET>/<PREFIX>/, assigning a
# predefined Firebase download token to each object (via the
# firebaseStorageDownloadTokens metadata) so the public download URLs are
# deterministic and match what the DashPilot app expects. This works with the
# default Firebase Storage security rules — no need to make the bucket public.
#
# Requires: gsutil (Google Cloud SDK), shasum.
#
# Usage:
#   tools/publish_ota.sh [--track legacy|juniper] <version> [path-to-bin]
#   e.g. tools/publish_ota.sh 0.0.2 build/dashkit-firmware.bin
#        tools/publish_ota.sh --track juniper 0.4.2
#
# Each car has its own release track (storage folder + manifest); the app reads
# the manifest of the car selected in its settings. legacy (default) is the
# original dashkit/ folder that every older app build reads.
#
set -euo pipefail

# Firebase project "dashkit-connect" (default Storage bucket). Releases up to
# 0.1.3 live in the old xreport-f792c.appspot.com bucket, which apps built
# before the move still read.
BUCKET="dashkit-connect.firebasestorage.app"

# These MUST match the tokens baked into the apps + committed manifests (the
# same tokens are used on every track):
#   android: FirmwareUpdateRepository manifest URLs
#   ios:     FirmwareUpdateRepository manifest URLs (FirmwareUpdateManager.swift)
#   firmware: ota/<track manifest> (url field)
MANIFEST_TOKEN="68bcc03a-3951-454d-8f35-f5ab06c0ed0a"
BIN_TOKEN="684a672c-53cd-48a4-a325-50e25b807b0b"

TRACK="legacy"
if [[ "${1:-}" == "--track" ]]; then
  TRACK="${2:-}"
  shift 2
fi
case "$TRACK" in
  legacy)  PREFIX="dashkit";         MANIFEST="ota/manifest.json" ;;
  juniper) PREFIX="dashkit-juniper"; MANIFEST="ota/juniper/manifest.json" ;;
  *) echo "unknown track: $TRACK (legacy|juniper)" >&2; exit 1 ;;
esac

VERSION="${1:-}"
BIN="${2:-build/dashkit-firmware.bin}"

if [[ -z "$VERSION" ]]; then
  echo "usage: $0 [--track legacy|juniper] <version> [path-to-bin]" >&2
  exit 1
fi
if [[ ! -f "$BIN" ]]; then
  echo "firmware binary not found: $BIN" >&2
  echo "(build first with: idf.py build)" >&2
  exit 1
fi

BIN_NAME="dashkit-${VERSION}.bin"
SHA=$(shasum -a 256 "$BIN" | awk '{print $1}')
BIN_URL="https://firebasestorage.googleapis.com/v0/b/${BUCKET}/o/${PREFIX}%2F${BIN_NAME}?alt=media&token=${BIN_TOKEN}"

echo "Uploading ${BIN} -> gs://${BUCKET}/${PREFIX}/${BIN_NAME}"
gcloud storage cp "$BIN" "gs://${BUCKET}/${PREFIX}/${BIN_NAME}" \
    --content-type=application/octet-stream \
    --custom-metadata="firebaseStorageDownloadTokens=${BIN_TOKEN}"

# Regenerate the manifest for this release.
MANIFEST_PATH="$(dirname "$0")/../${MANIFEST}"
mkdir -p "$(dirname "$MANIFEST_PATH")"
cat > "$MANIFEST_PATH" <<EOF
{
  "version": "${VERSION}",
  "url": "${BIN_URL}",
  "sha256": "${SHA}",
  "notes": "OTA update ${VERSION}."
}
EOF

echo "Uploading manifest -> gs://${BUCKET}/${PREFIX}/manifest.json"
gcloud storage cp "$MANIFEST_PATH" "gs://${BUCKET}/${PREFIX}/manifest.json" \
    --content-type=application/json \
    --custom-metadata="firebaseStorageDownloadTokens=${MANIFEST_TOKEN}"

echo
echo "Published ${VERSION} on the ${TRACK} track (sha256=${SHA})."
echo "Manifest URL: https://firebasestorage.googleapis.com/v0/b/${BUCKET}/o/${PREFIX}%2Fmanifest.json?alt=media&token=${MANIFEST_TOKEN}"
