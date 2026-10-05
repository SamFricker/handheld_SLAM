#!/usr/bin/env python3
"""Atech live orientation viewer.

Reads Atech USB-serial JSON sensor events and displays a smooth 3D cube.
The visual style is intentionally simple and close to the original viewer,
while pygame provides the smoother rendering loop.

Usage:
    python atech_orientation_viewer.py COM3

Install:
    python -m pip install pyserial pygame numpy
"""

from __future__ import annotations

import json
import math
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any

import numpy as np
import pygame

try:
    import serial
    from serial import SerialException
except ImportError as exc:
    raise SystemExit(
        "pyserial is not installed. Run: python -m pip install pyserial pygame numpy"
    ) from exc

BAUD = 115200
FPS = 120
SMOOTHING = 18.0  # Higher = follows sensor more quickly while remaining smooth.
MAX_GYRO_DT = 0.02


@dataclass
class SensorState:
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    gyro_x_dps: float = 0.0
    gyro_y_dps: float = 0.0
    gyro_z_dps: float = 0.0
    yaw_deg: float = 0.0
    packets: int = 0
    connected: bool = False
    last_packet_time: float = 0.0
    last_orientation_time: float = 0.0
    last_error: str = ""


state = SensorState()
state_lock = threading.Lock()


def parse_number(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def update_sensor(key: str, value: Any) -> None:
    number = parse_number(value)
    now = time.perf_counter()

    with state_lock:
        key = key.lower()
        if key == "pitch" and number is not None:
            state.pitch_deg = number
            state.last_orientation_time = now
        elif key == "roll" and number is not None:
            state.roll_deg = number
            state.last_orientation_time = now
        elif key == "gyro_x" and number is not None:
            state.gyro_x_dps = number
        elif key == "gyro_y" and number is not None:
            state.gyro_y_dps = number
        elif key == "gyro_z" and number is not None:
            state.gyro_z_dps = number


def process_packet(packet: Any) -> None:
    if not isinstance(packet, dict):
        return

    payload = packet.get("payload")
    if not isinstance(payload, dict):
        return

    key = payload.get("key")
    value = payload.get("value")
    if not isinstance(key, str):
        return

    if key.lower() == "orientation" and isinstance(value, str):
        # Atech format: "pitch,roll,tilt,orientation_name"
        parts = [item.strip() for item in value.split(",")]
        if len(parts) >= 2:
            pitch = parse_number(parts[0])
            roll = parse_number(parts[1])
            if pitch is not None:
                update_sensor("pitch", pitch)
            if roll is not None:
                update_sensor("roll", roll)
    else:
        update_sensor(key, value)

    with state_lock:
        state.packets += 1
        state.last_packet_time = time.perf_counter()


def serial_worker(port: str) -> None:
    try:
        with serial.Serial(port, BAUD, timeout=0.01) as ser:
            with state_lock:
                state.connected = True
                state.last_error = ""

            # Opening an ESP32 serial port can reset it. Discard boot text.
            time.sleep(0.15)
            ser.reset_input_buffer()

            while True:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").strip()
                if not line or not line.startswith("{"):
                    continue
                try:
                    process_packet(json.loads(line))
                except json.JSONDecodeError:
                    continue

    except SerialException as exc:
        with state_lock:
            state.connected = False
            state.last_error = str(exc)
    except Exception as exc:
        with state_lock:
            state.connected = False
            state.last_error = f"{type(exc).__name__}: {exc}"


def rotation_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """Return Rz(yaw) * Ry(pitch) * Rx(roll)."""
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)

    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)

    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


# Cube in local scanner coordinates.
VERTICES = np.array(
    [
        [-1, -1, -1],
        [1, -1, -1],
        [1, 1, -1],
        [-1, 1, -1],
        [-1, -1, 1],
        [1, -1, 1],
        [1, 1, 1],
        [-1, 1, 1],
    ],
    dtype=float,
)

EDGES = [
    (0, 1), (1, 2), (2, 3), (3, 0),
    (4, 5), (5, 6), (6, 7), (7, 4),
    (0, 4), (1, 5), (2, 6), (3, 7),
]

# Physical Atech Y is shown as viewer X; physical X is shown as viewer Y;
# Z stays Z, matching the user's requested physical convention.
AXIS_REMAP = np.array([[0, 1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)


def project(points: np.ndarray, width: int, height: int) -> list[tuple[int, int]]:
    """Project 3D points with +Z as vertical screen-up and +Y as depth."""
    camera_y = 7.0
    scale = min(width, height) * 0.34
    pts = points.copy()
    pts[:, 1] += camera_y
    denominator = np.maximum(0.5, pts[:, 1])
    x = width * 0.54 + scale * pts[:, 0] / denominator
    y = height * 0.48 - scale * pts[:, 2] / denominator
    return [(int(round(px)), int(round(py))) for px, py in zip(x, y)]


def draw_text(
    screen: pygame.Surface,
    font: pygame.font.Font,
    text: str,
    xy: tuple[int, int],
    color: tuple[int, int, int],
) -> None:
    surface = font.render(text, True, color)
    screen.blit(surface, xy)


def approach(current: float, target: float, dt: float) -> float:
    # Frame-rate-independent exponential smoothing.
    alpha = 1.0 - math.exp(-SMOOTHING * dt)
    return current + (target - current) * alpha


def main() -> int:
    if len(sys.argv) != 2:
        print("Usage: python atech_orientation_viewer.py COM3")
        return 2

    port = sys.argv[1]

    reader = threading.Thread(target=serial_worker, args=(port,), daemon=True)
    reader.start()

    pygame.init()
    pygame.display.set_caption("Atech Live Orientation")
    screen = pygame.display.set_mode((820, 560), pygame.RESIZABLE)
    clock = pygame.time.Clock()

    title_font = pygame.font.SysFont("Segoe UI", 26, bold=True)
    body_font = pygame.font.SysFont("Consolas", 19)
    small_font = pygame.font.SysFont("Segoe UI", 15)

    # Visual state deliberately follows the measured Atech pitch/roll smoothly,
    # while yaw is gyro-integrated because the current sensor is 6-axis.
    visual_pitch = 0.0
    visual_roll = 0.0
    yaw = 0.0
    last_frame = time.perf_counter()

    running = True
    while running:
        now = time.perf_counter()
        dt = now - last_frame
        last_frame = now
        dt = min(max(dt, 0.0), 0.05)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                running = False

        with state_lock:
            target_pitch = state.pitch_deg
            target_roll = state.roll_deg
            gz = state.gyro_z_dps
            measured_pitch = state.pitch_deg
            measured_roll = state.roll_deg
            packets = state.packets
            connected = state.connected
            last_packet = state.last_packet_time
            last_orientation = state.last_orientation_time
            error = state.last_error

        # Smoothly follow the Atech's pitch/roll values.
        visual_pitch = approach(visual_pitch, target_pitch, dt)
        visual_roll = approach(visual_roll, target_roll, dt)

        # Yaw is estimated from Z gyro because the current IMU has no magnetometer.
        yaw -= gz * min(dt, MAX_GYRO_DT)

        # Apply physical-axis convention requested by the user.
        rotation = rotation_matrix(visual_roll, visual_pitch, yaw)
        rotation_display = AXIS_REMAP @ rotation @ AXIS_REMAP.T

        rotated = VERTICES @ rotation_display.T
        width, height = screen.get_size()
        screen_pts = project(rotated, width, height)

        screen.fill((18, 20, 24))
        draw_text(screen, title_font, "Atech live IMU orientation", (30, 25), (240, 240, 240))

        # Two mutually orthogonal grid planes, like a simple 3D graph.
        # XY plane: horizontal ground plane at Z = 0.
        # XZ plane: vertical plane at Y = 0.
        # This makes X, Y and Z spatial relationships much easier to read.
        grid_size = 5.0
        grid_step = 0.5

        def draw_grid_line(a, b, colour=(55, 60, 68), width_px=1):
            pa = project((rotation_display @ a).reshape(1, 3), width, height)[0]
            pb = project((rotation_display @ b).reshape(1, 3), width, height)[0]
            pygame.draw.line(screen, colour, pa, pb, width_px)

        # XY plane at Z=0.
        for y0 in np.arange(-grid_size, grid_size + grid_step, grid_step):
            draw_grid_line(
                np.array([-grid_size, y0, 0.0]),
                np.array([grid_size, y0, 0.0]),
            )
        for x0 in np.arange(-grid_size, grid_size + grid_step, grid_step):
            draw_grid_line(
                np.array([x0, -grid_size, 0.0]),
                np.array([x0, grid_size, 0.0]),
            )

        # XZ plane at Y=0.
        for z0 in np.arange(-grid_size, grid_size + grid_step, grid_step):
            draw_grid_line(
                np.array([-grid_size, 0.0, z0]),
                np.array([grid_size, 0.0, z0]),
                (48, 53, 61),
            )
        for x0 in np.arange(-grid_size, grid_size + grid_step, grid_step):
            draw_grid_line(
                np.array([x0, 0.0, -grid_size]),
                np.array([x0, 0.0, grid_size]),
                (48, 53, 61),
            )

        # Main cube.
        for i, j in EDGES:
            pygame.draw.line(screen, (105, 155, 210), screen_pts[i], screen_pts[j], 4)

        # XYZ axes. +Z is deliberately vertical in the world when the viewer starts.
        origin = project(np.array([[0.0, 0.0, 0.0]]), width, height)[0]
        axis_len = 1.9
        axes = [
            ("X", np.array([axis_len, 0.0, 0.0]), (235, 70, 70)),
            ("Y", np.array([0.0, axis_len, 0.0]), (70, 210, 100)),
            ("Z", np.array([0.0, 0.0, axis_len]), (70, 130, 240)),
        ]
        axis_font = pygame.font.SysFont("Segoe UI", 22, bold=True)
        for label, axis, colour in axes:
            endpoint = rotation_display @ axis
            endpoint_px = project(np.array([endpoint]), width, height)[0]
            pygame.draw.line(screen, colour, origin, endpoint_px, 7)
            # Put a high-contrast axis label just beyond the end of each axis.
            label_surface = axis_font.render(label, True, colour)
            label_bg = pygame.Surface((label_surface.get_width() + 8, label_surface.get_height() + 4), pygame.SRCALPHA)
            label_bg.fill((18, 20, 24, 220))
            label_bg.blit(label_surface, (4, 2))
            screen.blit(label_bg, (endpoint_px[0] + 8, endpoint_px[1] - label_bg.get_height() // 2))

        age = now - last_packet if last_packet else 999.0
        orientation_age = now - last_orientation if last_orientation else 999.0
        if not connected:
            status = "DISCONNECTED"
            status_colour = (240, 100, 100)
        elif age > 0.5:
            status = "NO RECENT DATA"
            status_colour = (240, 170, 70)
        else:
            status = "CONNECTED"
            status_colour = (80, 220, 110)

        info_x = 35
        info_y = height - 165
        draw_text(screen, body_font, status, (info_x, info_y), status_colour)
        draw_text(screen, body_font, f"Pitch: {visual_pitch:8.2f} deg", (info_x, info_y + 28), (240, 240, 240))
        draw_text(screen, body_font, f"Roll : {visual_roll:8.2f} deg", (info_x, info_y + 54), (240, 240, 240))
        draw_text(screen, body_font, f"Yaw* : {yaw:8.2f} deg", (info_x, info_y + 80), (240, 240, 240))
        draw_text(
            screen,
            small_font,
            f"Atech measured pitch/roll: {measured_pitch:.2f} / {measured_roll:.2f} deg",
            (info_x, info_y + 108),
            (180, 185, 195),
        )
        draw_text(
            screen,
            small_font,
            f"Packets: {packets}   Orientation age: {orientation_age*1000:.0f} ms   ESC = quit",
            (info_x, info_y + 132),
            (180, 185, 195),
        )

        if error:
            draw_text(screen, small_font, error[:100], (35, height - 28), (235, 120, 120))

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
