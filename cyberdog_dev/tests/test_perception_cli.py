from perception import detectors


def test_main_passes_mode_as_argv_to_detector_runner(monkeypatch):
    captured = {}

    def fake_run(detector_name, args=None):
        captured["detector_name"] = detector_name
        captured["args"] = args

    monkeypatch.setattr(detectors, "run", fake_run)

    detectors.main(["orange_ball", "--mode", "sim"])

    assert captured == {
        "detector_name": "orange_ball",
        "args": ["--mode", "sim"],
    }
