# Daemon model files

Both files are exports of maker-arm-lab at commit `c9b9a3c`, copied without change.
They were made with this command, run in that checkout:

    PYTHONPATH=$PWD/source .venv/bin/python scripts/export_daemon_files.py --sim --out_dir <this directory>

- `sim_profile.json` is the lab's `sim_profile()`.
- `dynamics_urdf.json` holds the arguments of the lab's `Dynamics` (the arm's own URDF). Its `armature` is `null` on purpose: the pinned maker-arm-rs takes no armature argument yet.
