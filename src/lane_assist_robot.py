"""Lane keeping and blue-obstacle stopping for a Raspberry Pi robot.

The program supports two modes:

* Hardware mode controls two DC motors through ``gpiozero.Motor``.
* Preview mode (``--no-motors``) runs the complete vision pipeline without
  accessing Raspberry Pi GPIO, which is useful for calibration on a laptop.

Press ``q`` in the preview window, or Ctrl+C in the terminal, to stop safely.
"""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from typing import Sequence

import cv2 as cv
import numpy as np


Line = np.ndarray
BoundingBox = tuple[int, int, int, int]


@dataclass
class PIDController:
    """Small PID controller used to correct the lane-position error."""

    kp: float
    ki: float
    kd: float
    previous_error: float = 0.0
    integral: float = 0.0

    def compute(self, error: float, dt: float) -> float:
        """Return the PID output for ``error`` over time interval ``dt``."""
        dt = max(dt, 1e-6)
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output


@dataclass(frozen=True)
class RobotConfig:
    """Runtime and calibration values for the robot."""

    frame_width: int = 480
    frame_height: int = 320
    requested_fps: int = 20
    base_speed: float = 0.50
    steering_scale: float = 1000.0
    kp: float = 7.0
    ki: float = 0.0
    kd: float = 0.0
    single_lane_offset_px: int = 50
    obstacle_min_area_px: int = 250
    lane_left_fraction: float = 100 / 480
    lane_right_fraction: float = 380 / 480


def roi_filter(image: np.ndarray, vertices: np.ndarray) -> np.ndarray:
    """Keep only pixels inside the polygon defined by ``vertices``."""
    mask = np.zeros_like(image)
    cv.fillPoly(mask, vertices, 255)
    return cv.bitwise_and(image, mask)


def detect_lane_lines(frame: np.ndarray) -> tuple[list[Line], list[Line]]:
    """Detect and classify left/right lane segments with a Hough transform."""
    height, width = frame.shape[:2]
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    edges = cv.Canny(gray, 100, 200, apertureSize=3)

    roi_vertices = np.array(
        [[(100, height), (width // 2, int(height * 0.15)), (width - 100, height)]],
        dtype=np.int32,
    )
    cropped_edges = roi_filter(edges, roi_vertices)
    lines = cv.HoughLinesP(
        cropped_edges,
        rho=2,
        theta=np.pi / 180,
        threshold=50,
        minLineLength=10,
        maxLineGap=30,
    )

    left_lines: list[Line] = []
    right_lines: list[Line] = []
    if lines is None:
        return left_lines, right_lines

    for line in lines:
        x1, y1, x2, y2 = line[0]
        if x2 == x1:
            continue
        slope = (y2 - y1) / (x2 - x1)
        if abs(slope) <= 0.5:
            continue
        (left_lines if slope < 0 else right_lines).append(line)

    return left_lines, right_lines


def draw_lines(frame: np.ndarray, lines: Sequence[Line]) -> np.ndarray:
    """Overlay detected lane segments on a copy of ``frame``."""
    overlay = np.zeros_like(frame)
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv.line(overlay, (x1, y1), (x2, y2), (0, 255, 0), 2)
    return cv.addWeighted(frame, 0.8, overlay, 1.0, 0.0)


def estimate_lane_center(
    left_lines: Sequence[Line],
    right_lines: Sequence[Line],
    default_center: int,
    single_lane_offset_px: int = 50,
) -> int:
    """Estimate the horizontal lane center from detected line endpoints."""
    left_x = [int(x) for line in left_lines for x in (line[0][0], line[0][2])]
    right_x = [int(x) for line in right_lines for x in (line[0][0], line[0][2])]

    if left_x and right_x:
        return int((np.mean(left_x) + np.mean(right_x)) / 2)
    if left_x:
        return int(np.mean(left_x) + single_lane_offset_px)
    if right_x:
        return int(np.mean(right_x) - single_lane_offset_px)
    return default_center


def detect_blue_obstacles(
    frame: np.ndarray, min_area_px: int = 250
) -> list[BoundingBox]:
    """Return bounding boxes for blue obstacles detected in HSV space."""
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    mask = cv.inRange(
        hsv,
        np.array([100, 150, 50], dtype=np.uint8),
        np.array([140, 255, 255], dtype=np.uint8),
    )
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    return [cv.boundingRect(c) for c in contours if cv.contourArea(c) >= min_area_px]


def most_important_object(
    detections: Sequence[BoundingBox], left_lane: int, right_lane: int
) -> int:
    """Return the index of the closest detected obstacle inside the ego lane.

    In the camera view, the bottom-most bounding box is treated as the closest.
    An object outside the horizontal ego-lane bounds is ignored.
    """
    candidates = [
        (index, y + height)
        for index, (x, y, width, height) in enumerate(detections)
        if left_lane < x + width // 2 < right_lane
    ]
    return max(candidates, key=lambda item: item[1])[0] if candidates else -1


def annotate_frame(
    frame: np.ndarray,
    detections: Sequence[BoundingBox],
    mio_index: int,
    lane_center: int,
    lane_error: float,
    control_output: float,
) -> np.ndarray:
    """Draw lane state and obstacle decisions for debugging and demos."""
    output = frame.copy()
    for index, (x, y, width, height) in enumerate(detections):
        is_mio = index == mio_index
        color = (0, 255, 255) if is_mio else (255, 255, 255)
        cv.rectangle(output, (x, y), (x + width, y + height), color, 2)
        cv.putText(
            output,
            "MIO" if is_mio else "out-of-lane",
            (x, max(20, y - 8)),
            cv.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
        )

    cv.drawMarker(
        output,
        (lane_center, output.shape[0] // 2),
        (255, 255, 255),
        cv.MARKER_CROSS,
        18,
        2,
    )
    cv.putText(
        output,
        f"Lane error: {lane_error:.1f}",
        (20, 35),
        cv.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 0, 0),
        2,
    )
    cv.putText(
        output,
        f"Control: {control_output:.1f}",
        (20, 68),
        cv.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 0),
        2,
    )
    cv.putText(
        output,
        "STOP - in-lane obstacle" if mio_index != -1 else "DRIVE",
        (20, 101),
        cv.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 0, 255) if mio_index != -1 else (0, 255, 0),
        2,
    )
    return output


class DifferentialDrive:
    """GPIO wrapper for the two tracks of the robot."""

    def __init__(self) -> None:
        from gpiozero import Motor

        # Physical wiring used in the university prototype.
        self.right_motor = Motor(forward=23, backward=24)
        self.left_motor = Motor(forward=17, backward=27)

    def forward(self, left_speed: float, right_speed: float) -> None:
        self.left_motor.forward(left_speed)
        self.right_motor.forward(right_speed)

    def stop(self) -> None:
        self.left_motor.stop()
        self.right_motor.stop()

    def close(self) -> None:
        self.stop()
        self.left_motor.close()
        self.right_motor.close()


def run(camera_index: int, no_motors: bool, headless: bool) -> None:
    """Run the real-time perception and control loop."""
    config = RobotConfig()
    pid = PIDController(config.kp, config.ki, config.kd)
    drive = None if no_motors else DifferentialDrive()

    camera = cv.VideoCapture(camera_index)
    camera.set(cv.CAP_PROP_FRAME_WIDTH, config.frame_width)
    camera.set(cv.CAP_PROP_FRAME_HEIGHT, config.frame_height)
    camera.set(cv.CAP_PROP_FPS, config.requested_fps)
    if not camera.isOpened():
        raise RuntimeError(f"Camera {camera_index} could not be opened")

    previous_time = time.monotonic()
    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                raise RuntimeError("Camera frame capture failed")

            now = time.monotonic()
            dt = now - previous_time
            previous_time = now

            frame_center = frame.shape[1] // 2
            left_lines, right_lines = detect_lane_lines(frame)
            lane_center = estimate_lane_center(
                left_lines,
                right_lines,
                frame_center,
                config.single_lane_offset_px,
            )
            lane_error = frame_center - lane_center
            control_output = pid.compute(lane_error, dt)

            left_speed = float(
                np.clip(config.base_speed + control_output / config.steering_scale, 0, 1)
            )
            right_speed = float(
                np.clip(config.base_speed - control_output / config.steering_scale, 0, 1)
            )

            obstacles = detect_blue_obstacles(frame, config.obstacle_min_area_px)
            lane_left = int(frame.shape[1] * config.lane_left_fraction)
            lane_right = int(frame.shape[1] * config.lane_right_fraction)
            mio_index = most_important_object(obstacles, lane_left, lane_right)

            if drive is not None:
                if mio_index != -1:
                    drive.stop()
                else:
                    drive.forward(left_speed, right_speed)

            if not headless:
                display = draw_lines(frame, [*left_lines, *right_lines])
                display = annotate_frame(
                    display,
                    obstacles,
                    mio_index,
                    lane_center,
                    lane_error,
                    control_output,
                )
                cv.imshow("Raspberry Pi Lane Assist", display)
                if cv.waitKey(1) & 0xFF == ord("q"):
                    break
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        if drive is not None:
            drive.close()
        camera.release()
        cv.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", type=int, default=0, help="OpenCV camera index")
    parser.add_argument(
        "--no-motors",
        action="store_true",
        help="Run vision only; do not initialize Raspberry Pi GPIO",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Disable the OpenCV preview window",
    )
    return parser.parse_args()


if __name__ == "__main__":
    arguments = parse_args()
    run(arguments.camera, arguments.no_motors, arguments.headless)
