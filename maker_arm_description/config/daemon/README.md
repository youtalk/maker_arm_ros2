# Daemon model files

Both files come from `maker-arm-cli export` of maker-arm-rs at commit `0c44d8b52f0f8beb0d26d04a80c204fb80aab82b`. Run this command in the root of this repository:

    maker-arm-cli export --sim --urdf maker_arm_description/urdf/maker_arm_sdk.urdf --out_dir maker_arm_description/config/daemon

- `sim_profile.json` is the profile of the daemon's built-in simulated arm.
- `dynamics_urdf.json` holds the arm's own URDF model. Its `armature` is `null` on purpose. `provenance` holds the SHA-256 of the URDF.

To make a profile for a real arm, run `maker-arm-cli export --calibration CAL.json --caps 0.5 --out <path>`. The README in the repository root explains the steps.
