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
#   tools/publish_ota.sh <version> [path-to-bin]
#   e.g. tools/publish_ota.sh 0.0.2 build/dashkit-firmware.bin
#
set -euo pipefail

BUCKET="xreport-f792c.appspot.com"
PREFIX="dashkit"

# These MUST match the tokens baked into the app + committed manifest:
#   android: FirmwareUpdateRepository.MANIFEST_URL
#   firmware: ota/manifest.json (url field)
MANIFEST_TOKEN="68bcc03a-3951-454d-8f35-f5ab06c0ed0a"
BIN_TOKEN="684a672c-53cd-48a4-a325-50e25b807b0b"

VERSION="${1:-}"
BIN="${2:-build/dashkit-firmware.bin}"

if [[ -z "$VERSION" ]]; then
  echo "usage: $0 <version> [path-to-bin]" >&2
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
cat > "$(dirname "$0")/../ota/manifest.json" <<EOF
{
  "version": "${VERSION}",
  "url": "${BIN_URL}",
  "sha256": "${SHA}",
  "notes": "OTA update ${VERSION}."
}
EOF

echo "Uploading manifest -> gs://${BUCKET}/${PREFIX}/manifest.json"
gcloud storage cp "$(dirname "$0")/../ota/manifest.json" "gs://${BUCKET}/${PREFIX}/manifest.json" \
    --content-type=application/json \
    --custom-metadata="firebaseStorageDownloadTokens=${MANIFEST_TOKEN}"

echo
echo "Published ${VERSION} (sha256=${SHA})."
echo "Manifest URL: https://firebasestorage.googleapis.com/v0/b/${BUCKET}/o/${PREFIX}%2Fmanifest.json?alt=media&token=${MANIFEST_TOKEN}"
