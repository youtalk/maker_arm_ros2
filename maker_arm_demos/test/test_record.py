from maker_arm_demos.record import Record

ARM = [f'link_00{i}_joint' for i in range(2, 8)]


def test_a_record_round_trips(tmp_path):
    record = Record(
        joint_names=ARM,
        points=[(0.0, [0.0, -1.79, 2.05, -1.3, -0.22, 0.0]),
                (0.1, [0.01, -1.78, 2.04, -1.29, -0.21, 0.02])],
        grips=[(0.0, 0.050636), (0.1, 0.014)],
    )
    path = tmp_path / 'record.json'
    record.save(path)
    back = Record.load(path)
    assert back == record
    assert back.start() == [0.0, -1.79, 2.05, -1.3, -0.22, 0.0]
