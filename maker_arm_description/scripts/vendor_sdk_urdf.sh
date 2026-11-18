#!/usr/bin/env bash
# Vendor the Maker Arm URDF and meshes from maker-arm-sdk at a pinned commit.
# The same SDK pin as maker-arm-rs's maker_arm_v1 profile. Re-vendor: bump PIN_SHA and run again.
set -euo pipefail

REPO_URL="https://github.com/makermods-robotics/maker-arm-sdk"
PIN_SHA="0245316c05cb3e8d900f35e70c08e2e18e36731e"
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

# The SDK's arm limits are the designer's, not the arm's: three upper limits are short of the
# real stops. Widen each arm joint so that it covers the real profile's range (the vendor's stop
# captures, 0.05 rad backed off). A limit wider than the profile stays as it is.
python3 - "$PKG/urdf/maker_arm_sdk.urdf" "$PKG/config/daemon/real_profile_nominal.json" <<'EOF'
import json, re, sys

urdf, profile = sys.argv[1:]
text = open(urdf).read()
for i, joint in enumerate(json.load(open(profile))["joints"][:6]):
    pattern = rf'(<joint name="link_00{i + 2}_joint".*?<limit lower=")([^"]+)(" upper=")([^"]+)"'

    def widen(m):
        lo = m[2] if float(m[2]) <= joint["q_lo"] else f'{joint["q_lo"]:.7f}'
        hi = m[4] if float(m[4]) >= joint["q_hi"] else f'{joint["q_hi"]:.7f}'
        return f'{m[1]}{lo}{m[3]}{hi}"'

    text, n = re.subn(pattern, widen, text, count=1, flags=re.S)
    assert n == 1, f"no limit for link_00{i + 2}_joint"
open(urdf, "w").write(text)
EOF

cat > "$PKG/VENDOR.md" <<EOF
# Vendored: the SDK's URDF and meshes

- Source: $REPO_URL, directory \`urdf/maker_arm\`
- Pinned commit: \`$PIN_SHA\` (the same pin as maker-arm-rs's \`maker_arm_v1\` profile)
- License: Apache-2.0, MakerMods (see \`NOTICE\` in the repository root)
- Vendored on: $(date -u +%Y-%m-%d) with \`scripts/vendor_sdk_urdf.sh\`
- Files: \`urdf/maker_arm/robot.urdf\` is \`urdf/maker_arm_sdk.urdf\`, and \`meshes/*.stl\` are copied as they are.
- Change 1: every \`filename="meshes/\` in the URDF became \`filename="package://maker_arm_description/meshes/\`.
- Change 2: the arm joints' limits widen to cover \`config/daemon/real_profile_nominal.json\`, the vendor's stop captures. The SDK's limits are the designer's, and its README says that the actuator limits still need validation. The real stops are wider at the upper end of \`link_004_joint\`, \`link_005_joint\` and \`link_006_joint\`.
- The URDF keeps the CAD's Y-up base frame. \`urdf/maker_arm.urdf.xacro\` adds the Z-up \`mount_link\`.
EOF
echo "OK: vendored @ $PIN_SHA"
