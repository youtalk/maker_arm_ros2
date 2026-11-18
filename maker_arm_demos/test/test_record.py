from maker_arm_demos.record import Record

ARM = [f'link_00{i}_joint' for i in range(2, 8)]


def test_a_record_round_trips(tmp_path):
    record = Record(
        joint_names=ARM,
        points=[(0.0, [0.0, -1.5518, 1.6901, -1.038, 0.0, 0.0]),
                (0.1, [0.01, -1.5418, 1.6801, -1.028, 0.01, 0.02])],
        grips=[(0.0, 0.0398135), (0.1, 0.014)],
    )
    path = tmp_path / 'record.json'
    record.save(path)
    back = Record.load(path)
    assert back == record
    assert back.start() == [0.0, -1.5518, 1.6901, -1.038, 0.0, 0.0]
