# Daemon model files

The profiles and `dynamics_urdf.json` come from `maker-arm-cli export` of maker-arm-rs at commit `71e31d8874c0a6dc280fb1954761d91be28008c3`. Run these commands in the root of this repository:

    maker-arm-cli export --sim --urdf maker_arm_description/urdf/maker_arm_sdk.urdf --out_dir maker_arm_description/config/daemon
    maker-arm-cli export --calibration maker_arm_description/config/daemon/vendor_calibration.json --caps 0.5 --out maker_arm_description/config/daemon/real_profile_caps05.json
    maker-arm-cli export --calibration maker_arm_description/config/daemon/vendor_calibration.json --caps 1.0 --out maker_arm_description/config/daemon/real_profile_nominal.json

- `sim_profile.json` is the profile of the daemon's built-in simulated arm.
- `dynamics_urdf.json` holds the arm's own URDF model. Its `armature` is `null` on purpose. `provenance` holds the SHA-256 of the URDF.
- `vendor_calibration.json` is the calibration of an arm in the vendor zero (folded against the base, gripper fully closed). It comes from the vendor's gravity convention: motor 5 turns against `link_006_joint`.
- `real_profile_caps05.json` and `real_profile_nominal.json` are the real-arm profiles at caps 0.5 and 1.0. The MuJoCo plant on `vcan0` runs with the nominal one.

For an arm with another zero, make a profile from its own calibration: `maker-arm-cli export --calibration CAL.json --caps 0.5 --out <path>`. The README in the repository root explains the steps.
