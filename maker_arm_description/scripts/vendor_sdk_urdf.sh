#!/usr/bin/env bash
# Vendor the Maker Arm URDF and meshes from maker-arm-sdk at a pinned commit.
# Same SDK and same pin as maker-arm-lab. Re-vendor: bump PIN_SHA and run again.
set -euo pipefail

REPO_URL="https://github.com/makermods-robotics/maker-arm-sdk"
PIN_SHA="b30d05a23d72e8c155a8e00f807aba8e8c705f68"
PKG="$(cd "$(dirname "$0")/.." && pwd)"

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
git clone --quiet --filter=blob:none "$REPO_URL" "$TMP/sdk"
git -C "$TMP/sdk" checkout --quiet "$PIN_SHA"

rm -rf "$PKG/meshes"
mkdir -p "$PKG/urdf" "$PKG/meshes"
cp "$TMP/sdk/urdf/maker_arm/meshes/"*.stl "$PKG/meshes/"
sed 's|filename="meshes/|filename="package://maker_arm_description/meshes/|' \
  "$TMP/sdk/urdf/maker_arm/robot.urdf" > "$PKG/urdf/maker_arm_sdk.urdf"

cat > "$PKG/VENDOR.md" <<EOF
# Vendored: the SDK's URDF and meshes

- Source: $REPO_URL, directory \`urdf/maker_arm\`
- Pinned commit: \`$PIN_SHA\` (the same pin as maker-arm-lab)
- License: Apache-2.0, MakerMods (see \`NOTICE\` in the repository root)
- Vendored on: $(date -u +%Y-%m-%d) with \`scripts/vendor_sdk_urdf.sh\`
- Files: \`urdf/maker_arm/robot.urdf\` is \`urdf/maker_arm_sdk.urdf\`, and \`meshes/*.stl\` are copied as they are.
- One change: every \`filename="meshes/\` in the URDF became \`filename="package://maker_arm_description/meshes/\`.
- The URDF keeps the CAD's Y-up base frame. \`urdf/maker_arm.urdf.xacro\` adds the Z-up \`mount_link\`.
EOF
echo "OK: vendored @ $PIN_SHA"
