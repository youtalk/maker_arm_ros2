import os
import subprocess
from pathlib import Path

RELEASE = Path(__file__).resolve().parents[1] / "scripts" / "release"


def run(tmp_path, *args):
    fake = tmp_path / "maker-arm-cli"
    fake.write_text('#!/usr/bin/env bash\necho "$@"\n')
    fake.chmod(0o755)
    env = {**os.environ, "PATH": f"{tmp_path}:{os.environ['PATH']}"}
    return subprocess.run([str(RELEASE), *args], env=env, capture_output=True, text=True,
                          check=True).stdout.strip()


def test_release_adds_can0_when_no_interface_is_given(tmp_path):
    assert run(tmp_path) == "release --interface can0"


def test_release_keeps_the_callers_interface(tmp_path):
    assert run(tmp_path, "--interface", "can1") == "release --interface can1"
    assert run(tmp_path, "--interface=can1") == "release --interface=can1"
