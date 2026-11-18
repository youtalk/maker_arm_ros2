# Vendored: the SDK's URDF and meshes

- Source: https://github.com/makermods-robotics/maker-arm-sdk, directory `urdf/maker_arm`
- Pinned commit: `0245316c05cb3e8d900f35e70c08e2e18e36731e` (the same pin as maker-arm-rs's `maker_arm_v1` profile)
- License: Apache-2.0, MakerMods (see `NOTICE` in the repository root)
- Vendored on: 2026-11-18 with `scripts/vendor_sdk_urdf.sh`
- Files: `urdf/maker_arm/robot.urdf` is `urdf/maker_arm_sdk.urdf`, and `meshes/*.stl` are copied as they are.
- Change 1: every `filename="meshes/` in the URDF became `filename="package://maker_arm_description/meshes/`.
- Change 2: the arm joints' limits widen to cover `config/daemon/real_profile_nominal.json`, the vendor's stop captures. The SDK's limits are the designer's, and its README says that the actuator limits still need validation. The real stops are wider at the upper end of `link_004_joint`, `link_005_joint` and `link_006_joint`.
- The URDF keeps the CAD's Y-up base frame. `urdf/maker_arm.urdf.xacro` adds the Z-up `mount_link`.
