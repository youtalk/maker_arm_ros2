# Vendored: the SDK's URDF and meshes

- Source: https://github.com/makermods-robotics/maker-arm-sdk, directory `urdf/maker_arm`
- Pinned commit: `b30d05a23d72e8c155a8e00f807aba8e8c705f68` (the same pin as maker-arm-lab)
- License: Apache-2.0, MakerMods (see `NOTICE` in the repository root)
- Vendored on: 2026-10-16 with `scripts/vendor_sdk_urdf.sh`
- Files: `urdf/maker_arm/robot.urdf` is `urdf/maker_arm_sdk.urdf`, and `meshes/*.stl` are copied as they are.
- One change: every `filename="meshes/` in the URDF became `filename="package://maker_arm_description/meshes/`.
- The URDF keeps the CAD's Y-up base frame. `urdf/maker_arm.urdf.xacro` adds the Z-up `mount_link`.
