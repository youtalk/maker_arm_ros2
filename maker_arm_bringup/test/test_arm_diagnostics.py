import importlib.machinery
import importlib.util
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "arm_diagnostics"
OK, WARN, ERROR, STALE = 0, 1, 2, 3
TEMPS = [35.0, 36.0, None, 35.0, 34.0, 33.0, 30.0]


def load():
    loader = importlib.machinery.SourceFileLoader("arm_diagnostics", str(SCRIPT))
    spec = importlib.util.spec_from_loader("arm_diagnostics", loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


def line(**kw):
    return {"mode": "released", "clamped": 0, "refused": 0, "fault": 0, "temp_c": TEMPS, **kw}


def run(status, why=None, warn=55.0, error=70.0):
    return {n: (lvl, msg, v) for n, lvl, msg, v in load().diagnostics(status, why, warn, error)}


def test_there_are_8_statuses_with_the_joint_names_in_motor_order():
    names = list(run(line()))
    assert names == ["maker_arm: daemon"] + [f"maker_arm: link_00{i}_joint" for i in range(2, 8)] \
        + ["maker_arm: gripper_left_joint"]


def test_a_null_temperature_is_error_no_reply_and_the_others_keep_their_values():
    s = run(line())
    assert s["maker_arm: link_004_joint"][:2] == (ERROR, "no reply")
    assert s["maker_arm: link_002_joint"][:2] == (OK, "35 C")
    assert s["maker_arm: link_002_joint"][2] == {"motor id": 1, "temperature C": 35.0}
    assert s["maker_arm: gripper_left_joint"][2]["motor id"] == 7


@pytest.mark.parametrize("t, level", [(54.9, OK), (55.0, WARN), (69.9, WARN), (70.0, ERROR)])
def test_the_thresholds_are_inclusive(t, level):
    assert run(line(temp_c=[t] * 7))["maker_arm: link_002_joint"][0] == level


def test_the_thresholds_are_parameters():
    assert run(line(temp_c=[40.0] * 7), warn=30.0, error=45.0)["maker_arm: link_002_joint"][0] == WARN


@pytest.mark.parametrize("mode, level", [("following", OK), ("holding", OK), ("released", OK),
                                         ("fault", ERROR), ("refused", ERROR)])
def test_each_mode_sets_the_daemon_level(mode, level):
    lvl, msg, values = run(line(mode=mode))["maker_arm: daemon"]
    assert (lvl, msg, values["mode"]) == (level, mode, mode)


def test_a_fault_code_gives_its_name():
    lvl, msg, values = run(line(mode="fault", fault=4))["maker_arm: daemon"]
    assert (lvl, msg) == (ERROR, "fault: over temperature")
    assert values["fault code"] == 4 and values["fault"] == "over temperature"


def test_an_unknown_fault_code_is_still_shown():
    assert run(line(mode="fault", fault=99))["maker_arm: daemon"][1] == "fault: fault code 99"


def test_a_failed_status_is_error_and_7_stale_joints():
    s = run(None, "no daemon answers")
    assert s["maker_arm: daemon"][:2] == (ERROR, "no daemon answers")
    joints = [v for n, v in s.items() if n != "maker_arm: daemon"]
    assert len(joints) == 7 and all(v[:2] == (STALE, "no data from the daemon") for v in joints)


def test_status_of_reads_the_status_line_and_rejects_junk():
    m = load()
    assert m.status_of('log\nSTATUS {"mode": "holding"}\n') == {"mode": "holding"}
    assert m.status_of("STATUS nope") is None and m.status_of("") is None


def test_daemon_cli_takes_the_interface_and_socket_of_the_urdf():
    urdf = ("<robot><ros2_control><hardware><param name='interface'>can0</param>"
            "<param name='socket'>/s</param></hardware></ros2_control></robot>")
    assert load().daemon_cli(urdf) == ["maker-arm-cli", "--interface", "can0", "--socket", "/s"]


@pytest.mark.parametrize("status", [line(mode="rebooting"), line(mode=None), line(mode=["holding"]),
                                    {k: v for k, v in line().items() if k != "mode"}])
def test_an_unknown_or_missing_mode_is_error(status):
    assert run(status)["maker_arm: daemon"][0] == ERROR


@pytest.mark.parametrize("bad", [line(temp_c="hot"), line(temp_c=None), line(temp_c=[35.0, "x"] + [1] * 5),
                                 line(temp_c=[True] * 7), line(fault=[1], mode="fault")])
def test_a_malformed_status_is_error_bad_status_and_7_stale(bad):
    s = run(bad)
    assert s["maker_arm: daemon"][0] == ERROR and s["maker_arm: daemon"][1].startswith("bad STATUS: ")
    assert [v[0] for n, v in s.items() if n != "maker_arm: daemon"] == [STALE] * 7
