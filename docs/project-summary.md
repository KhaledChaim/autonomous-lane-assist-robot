# Project summary

This repository documents the university team project **RPi-based Lane-Assist Autonomous Car**, developed for AIRE475 Autonomous Cars and Drones at Abu Dhabi University.

## Team

- Khaled Abdul Rahman
- Abdulnasser Alnoaimi
- Abdel Rahman Nizar Alhendi
- Supervisor: Eng. Maha Yaghi

## Objective

Build a low-cost differential-drive robot that follows the center of a marked lane, identifies blue obstacles, stops only for an obstacle inside its lane, ignores out-of-lane objects, and resumes driving after the in-lane obstacle is removed.

## Implemented approach

1. Capture a 480 x 320 video stream from a USB camera.
2. Convert each frame to grayscale and apply Canny edge detection.
3. Restrict processing to a triangular region of interest.
4. Detect lane segments with the probabilistic Hough transform.
5. Estimate the lane center from left and right line segments.
6. Convert the lane-position error into differential motor speeds with PID control.
7. Segment blue obstacles in HSV color space.
8. Treat the bottom-most in-lane detection as the Most Important Object (MIO).
9. Stop both tracks while the MIO is present and continue after it clears.

## Reported results

During track testing, the robot completed repeated laps while remaining inside the lane. It stopped for blue obstacles placed in its path, ignored obstacles outside the ego lane, and resumed movement after the path was cleared. PID tuning reduced oscillation and produced smoother corrections.

## Scope note

The course brief and written report discuss YOLO and Ackermann steering as possible architectures. The source code in this repository documents the implemented prototype: **differential drive, Hough-line lane detection, PID correction, and HSV-based blue-obstacle detection**.
