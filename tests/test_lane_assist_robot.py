from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from lane_assist_robot import (  # noqa: E402
    PIDController,
    estimate_lane_center,
    most_important_object,
)


def test_pid_proportional_output() -> None:
    pid = PIDController(kp=7.0, ki=0.0, kd=0.0)
    assert pid.compute(error=2.0, dt=0.1) == 14.0


def test_mio_ignores_out_of_lane_and_selects_closest() -> None:
    detections = [
        (10, 250, 30, 30),   # outside ego lane
        (180, 90, 30, 30),   # inside but farther away
        (220, 200, 40, 40),  # inside and closest to the camera
    ]
    assert most_important_object(detections, left_lane=100, right_lane=380) == 2


def test_lane_center_uses_both_boundaries() -> None:
    left = [np.array([[100, 300, 180, 150]])]
    right = [np.array([[300, 150, 380, 300]])]
    assert estimate_lane_center(left, right, default_center=240) == 240


def test_lane_center_falls_back_when_no_lines_are_visible() -> None:
    assert estimate_lane_center([], [], default_center=240) == 240
