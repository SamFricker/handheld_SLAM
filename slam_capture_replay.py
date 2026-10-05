"""
SLAM Project: capture camera + Atech IMU/range, reconstruct, then replay.

This script does NOT modify video_to_3d.py.

What it does:
1. Opens Polaroid USB camera #1.
2. Connects to the Atech Gateway over Wi-Fi.
3. Shows live camera + IMU/range values.
4. RECORD records:
      recording/recording.mp4
      recording/frame_sensor_log.csv
      recording/sensor_events.jsonl
5. BUILD + REPLAY calls the existing video_to_3d.py on recording.mp4.
6. Parses the resulting COLMAP sparse model.
7. Replays the recorded camera feed while:
      - the camera moves through the fixed 3D reconstruction,
      - a frustum shows where the camera is looking,
      - the travelled path is drawn,
      - 3D points appear progressively as the mapped environment is built.

Important:
- COLMAP monocular reconstruction still has arbitrary global scale.
- The Atech distance is recorded in real millimetres/metres, but this first
  integrated version does not yet use it to scale the COLMAP map.
- The replayed 3D pose comes from COLMAP visual reconstruction. The Atech
  IMU/range are timestamped and displayed/recorded for later fusion.

Run:
    .\.venv\Scripts\python.exe slam_capture_replay.py

Optional replay-only mode after a successful build:
    .\.venv\Scripts\python.exe slam_capture_replay.py --replay
"""

from __future__ import annotations

import argparse
import asyncio
import bisect
import csv
import importlib.util
import json
import math
import os
import re
import struct
import sys
import threading
import time
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any

import cv2
import numpy as np

try:
    import websockets
except ImportError as exc:
    raise SystemExit(
        "Missing package 'websockets'. Install it with:\n"
        r".\.venv\Scripts\python.exe -m pip install websockets"
    ) from exc

try:
    import open3d as o3d
except ImportError as exc:
    raise SystemExit(
        "Missing package 'open3d'. Install it in the project .venv first."
    ) from exc


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

PROJECT_DIR = Path(__file__).resolve().parent
CAMERA_INDEX = 1

ATECH_GATEWAY_URL = (
    "wss://gateway.atech.dev/ws/live/"
    "75d4783b-4a28-4284-87a6-4fe08afae9ab"
)

RECORDING_DIR = PROJECT_DIR / "recording"
VIDEO_PATH = RECORDING_DIR / "recording.mp4"
FRAME_LOG_PATH = RECORDING_DIR / "frame_sensor_log.csv"
SENSOR_EVENTS_PATH = RECORDING_DIR / "sensor_events.jsonl"

RECON_DIR = PROJECT_DIR / "slam_reconstruction"
VIDEO_TO_3D_PATH = PROJECT_DIR / "video_to_3d.py"

WINDOW_TITLE = "SLAM Capture - Polaroid + Atech"

RECONNECT_DELAY = 2.0


# ---------------------------------------------------------------------------
# Atech sensor stream
# ---------------------------------------------------------------------------

@dataclass
class SensorState:
    accel_x: float = float("nan")
    accel_y: float = float("nan")
    accel_z: float = float("nan")
    gyro_x: float = float("nan")
    gyro_y: float = float("nan")
    gyro_z: float = float("nan")
    pitch_deg: float = float("nan")
    roll_deg: float = float("nan")
    tilt_deg: float = float("nan")
    orientation: str = ""
    distance_mm: float = float("nan")
    connected: bool = False
    packets: int = 0
    last_packet_perf: float = 0.0
    last_error: str = ""


class SensorHub:
    def __init__(self, url: str):
        self.url = url
        self.state = SensorState()
        self.lock = threading.Lock()
        self.raw_packet_callback = None

    @staticmethod
    def _number(value: Any) -> float:
        try:
            number = float(value)
            if math.isfinite(number):
                return number
        except (TypeError, ValueError):
            pass
        return float("nan")

    def snapshot(self) -> SensorState:
        with self.lock:
            return SensorState(**asdict(self.state))

    def process_packet(self, packet: Any) -> None:
        """Parse Atech Gateway events, including both live-dashboard and
        direct-device message shapes.  The gateway normally sends:

            {"type":"sensor","key":"pitch","value":-9.46,...}

        and can also wrap that object in ``payload`` or ``data``.
        """
        now_perf = time.perf_counter()

        if self.raw_packet_callback is not None:
            try:
                self.raw_packet_callback(packet, now_perf)
            except Exception:
                pass

        # Some gateways/bridges may hand us a JSON string or bytes.
        if isinstance(packet, bytes):
            try:
                packet = packet.decode("utf-8", errors="replace")
            except Exception:
                return
        if isinstance(packet, str):
            try:
                packet = json.loads(packet)
            except json.JSONDecodeError:
                return

        # Walk all nested dicts/lists so we do not depend on one particular
        # gateway envelope.  In particular, support both:
        #   {"type":"sensor","key":"pitch","value":...}
        #   {"type":"device_event","payload":{"key":"pitch","value":...}}
        def walk(obj: Any):
            if isinstance(obj, dict):
                yield obj
                for value in obj.values():
                    yield from walk(value)
            elif isinstance(obj, list):
                for value in obj:
                    yield from walk(value)

        candidates = list(walk(packet))
        event_count = 0
        changed = False

        with self.lock:
            for item in candidates:
                key = item.get("key") if isinstance(item, dict) else None
                if not isinstance(key, str) or "value" not in item:
                    continue

                key = key.strip().lower()
                value = item.get("value")
                event_type = str(item.get("type", "")).lower()

                # Ignore command/action envelopes.  Sensor/state/button events
                # are the data stream we want to log into the replay CSV.
                if event_type in ("send_to_device", "action", "command"):
                    continue

                if key in ("accel_x", "accel_y", "accel_z",
                           "gyro_x", "gyro_y", "gyro_z"):
                    setattr(self.state, key, self._number(value))
                    changed = True

                elif key in ("pitch", "pitch_deg"):
                    self.state.pitch_deg = self._number(value)
                    changed = True

                elif key in ("roll", "roll_deg"):
                    self.state.roll_deg = self._number(value)
                    changed = True

                elif key in ("tilt", "tilt_deg"):
                    self.state.tilt_deg = self._number(value)
                    changed = True

                elif key in ("distance", "distance_mm", "min_distance"):
                    number = self._number(value)
                    if math.isfinite(number) and number >= 0:
                        self.state.distance_mm = number
                        changed = True

                elif key in ("orientation", "name"):
                    # The current dashboard may report the coarse orientation
                    # as either key="orientation" or key="name".
                    if isinstance(value, str):
                        self.state.orientation = value
                        changed = True

                if changed:
                    event_count += 1

            # Also support a single object containing several readings at once.
            for item in candidates:
                if not isinstance(item, dict):
                    continue
                aliases = {
                    "accel_x": ("accel_x",),
                    "accel_y": ("accel_y",),
                    "accel_z": ("accel_z",),
                    "gyro_x": ("gyro_x",),
                    "gyro_y": ("gyro_y",),
                    "gyro_z": ("gyro_z",),
                    "pitch_deg": ("pitch_deg", "pitch"),
                    "roll_deg": ("roll_deg", "roll"),
                    "tilt_deg": ("tilt_deg", "tilt"),
                    "distance_mm": ("distance_mm", "distance", "min_distance"),
                }
                for target, keys in aliases.items():
                    for alias in keys:
                        if alias in item and not (isinstance(item.get(alias), (dict, list))):
                            number = self._number(item.get(alias))
                            if math.isfinite(number):
                                if target == "distance_mm" and number < 0:
                                    continue
                                setattr(self.state, target, number)
                                changed = True
                            break
                if isinstance(item.get("orientation"), str):
                    self.state.orientation = item["orientation"]
                    changed = True

            if changed:
                self.state.packets += max(1, event_count)
                self.state.last_packet_perf = now_perf

                # Make debugging the live stream easy without flooding the
                # terminal: print a compact summary whenever an orientation/IMU
                # reading changes.
                if (math.isfinite(self.state.pitch_deg) or
                    math.isfinite(self.state.roll_deg) or
                    math.isfinite(self.state.gyro_x) or
                    math.isfinite(self.state.gyro_y) or
                    math.isfinite(self.state.gyro_z)):
                    print(
                        f"ATECH IMU: pitch={self.state.pitch_deg:.3f} "
                        f"roll={self.state.roll_deg:.3f} "
                        f"tilt={self.state.tilt_deg:.3f} "
                        f"gyro=({self.state.gyro_x:.3f},"
                        f"{self.state.gyro_y:.3f},{self.state.gyro_z:.3f})",
                        flush=True,
                    )

    async def _loop(self) -> None:
        while True:
            try:
                async with websockets.connect(
                    self.url,
                    ping_interval=20,
                    ping_timeout=10,
                    close_timeout=2,
                    max_size=2**20,
                    compression=None,
                ) as ws:
                    with self.lock:
                        self.state.connected = True
                        self.state.last_error = ""

                    async for raw in ws:
                        if isinstance(raw, bytes):
                            raw = raw.decode("utf-8", errors="replace")
                        try:
                            packet = json.loads(raw)
                        except json.JSONDecodeError:
                            continue
                        self.process_packet(packet)

            except Exception as exc:
                with self.lock:
                    self.state.connected = False
                    self.state.last_error = f"{type(exc).__name__}: {exc}"
                await asyncio.sleep(RECONNECT_DELAY)

    def start(self) -> None:
        def runner():
            try:
                asyncio.run(self._loop())
            except Exception as exc:
                with self.lock:
                    self.state.connected = False
                    self.state.last_error = str(exc)

        threading.Thread(target=runner, daemon=True).start()


# ---------------------------------------------------------------------------
# Recording
# ---------------------------------------------------------------------------

class SessionRecorder:
    def __init__(self, sensor_hub: SensorHub):
        self.sensor_hub = sensor_hub
        self.recording = False
        self.writer = None
        self.start_perf = 0.0
        self.start_wall = 0.0
        self.frame_index = 0
        self.frame_rows: list[dict[str, Any]] = []
        self.raw_sensor_events: list[dict[str, Any]] = []
        self.lock = threading.Lock()

        self.sensor_hub.raw_packet_callback = self.record_sensor_packet

    def clean_previous(self) -> None:
        RECORDING_DIR.mkdir(parents=True, exist_ok=True)

        for path in (VIDEO_PATH, FRAME_LOG_PATH, SENSOR_EVENTS_PATH):
            if path.exists():
                try:
                    path.unlink()
                except OSError:
                    pass

    def start(self, frame: np.ndarray, camera_fps: float) -> None:
        if self.recording:
            return

        self.clean_previous()

        h, w = frame.shape[:2]
        fps = camera_fps
        if not math.isfinite(fps) or fps < 1 or fps > 120:
            fps = 30.0

        writer = cv2.VideoWriter(
            str(VIDEO_PATH),
            cv2.VideoWriter_fourcc(*"mp4v"),
            fps,
            (w, h),
        )

        if not writer.isOpened():
            print("ERROR: Could not create recording video.")
            return

        with self.lock:
            self.writer = writer
            self.start_perf = time.perf_counter()
            self.start_wall = time.time()
            self.frame_index = 0
            self.frame_rows = []
            self.raw_sensor_events = []
            self.recording = True

        print(f"Recording started: {VIDEO_PATH}")

    def record_sensor_packet(self, packet: Any, rx_perf: float) -> None:
        with self.lock:
            if not self.recording:
                return

            self.raw_sensor_events.append(
                {
                    "t_seconds": rx_perf - self.start_perf,
                    "wall_time": time.time(),
                    "packet": packet,
                }
            )

    def write_frame(self, frame: np.ndarray) -> None:
        with self.lock:
            if not self.recording or self.writer is None:
                return

            t = time.perf_counter() - self.start_perf
            snap = self.sensor_hub.snapshot()

            self.writer.write(frame)

            age_ms = float("nan")
            if snap.last_packet_perf > 0:
                age_ms = max(
                    0.0,
                    (time.perf_counter() - snap.last_packet_perf) * 1000.0,
                )

            self.frame_rows.append(
                {
                    "frame_index": self.frame_index,
                    "t_seconds": t,
                    "wall_time": time.time(),
                    "accel_x": snap.accel_x,
                    "accel_y": snap.accel_y,
                    "accel_z": snap.accel_z,
                    "gyro_x": snap.gyro_x,
                    "gyro_y": snap.gyro_y,
                    "gyro_z": snap.gyro_z,
                    "pitch_deg": snap.pitch_deg,
                    "roll_deg": snap.roll_deg,
                    "tilt_deg": snap.tilt_deg,
                    "orientation": snap.orientation,
                    "distance_mm": snap.distance_mm,
                    "sensor_connected": int(snap.connected),
                    "sensor_age_ms": age_ms,
                }
            )
            self.frame_index += 1

    def stop(self) -> None:
        with self.lock:
            if not self.recording:
                return

            self.recording = False

            if self.writer is not None:
                self.writer.release()
                self.writer = None

            rows = list(self.frame_rows)
            events = list(self.raw_sensor_events)

        if rows:
            with FRAME_LOG_PATH.open("w", newline="", encoding="utf-8") as f:
                writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                writer.writeheader()
                writer.writerows(rows)

        with SENSOR_EVENTS_PATH.open("w", encoding="utf-8") as f:
            for event in events:
                f.write(json.dumps(event, ensure_ascii=False) + "\n")

        print(f"Recording stopped. Frames: {len(rows)}")
        print(f"Frame/sensor log: {FRAME_LOG_PATH}")
        print(f"Raw sensor log:   {SENSOR_EVENTS_PATH}")


# ---------------------------------------------------------------------------
# COLMAP binary readers
# ---------------------------------------------------------------------------

def qvec_to_rotmat(qvec: np.ndarray) -> np.ndarray:
    qw, qx, qy, qz = qvec
    return np.array(
        [
            [
                1 - 2 * qy * qy - 2 * qz * qz,
                2 * qx * qy - 2 * qw * qz,
                2 * qx * qz + 2 * qw * qy,
            ],
            [
                2 * qx * qy + 2 * qw * qz,
                1 - 2 * qx * qx - 2 * qz * qz,
                2 * qy * qz - 2 * qw * qx,
            ],
            [
                2 * qx * qz - 2 * qw * qy,
                2 * qy * qz + 2 * qw * qx,
                1 - 2 * qx * qx - 2 * qy * qy,
            ],
        ],
        dtype=float,
    )


@dataclass
class ColmapImage:
    image_id: int
    qvec: np.ndarray
    tvec: np.ndarray
    camera_id: int
    name: str

    @property
    def rotation(self) -> np.ndarray:
        return qvec_to_rotmat(self.qvec)

    @property
    def center(self) -> np.ndarray:
        r = self.rotation
        return -r.T @ self.tvec

    @property
    def forward(self) -> np.ndarray:
        # COLMAP camera +Z axis points forward.
        return self.rotation.T @ np.array([0.0, 0.0, 1.0])

    @property
    def right(self) -> np.ndarray:
        return self.rotation.T @ np.array([1.0, 0.0, 0.0])

    @property
    def down(self) -> np.ndarray:
        return self.rotation.T @ np.array([0.0, 1.0, 0.0])


def read_null_terminated_string(f) -> str:
    data = bytearray()
    while True:
        b = f.read(1)
        if not b or b == b"\x00":
            break
        data.extend(b)
    return data.decode("utf-8", errors="replace")


def read_images_bin(path: Path) -> dict[int, ColmapImage]:
    images: dict[int, ColmapImage] = {}

    with path.open("rb") as f:
        num_images = struct.unpack("<Q", f.read(8))[0]

        for _ in range(num_images):
            image_id = struct.unpack("<i", f.read(4))[0]
            qvec = np.array(struct.unpack("<dddd", f.read(32)), dtype=float)
            tvec = np.array(struct.unpack("<ddd", f.read(24)), dtype=float)
            camera_id = struct.unpack("<i", f.read(4))[0]
            name = read_null_terminated_string(f)

            num_points2d = struct.unpack("<Q", f.read(8))[0]
            # x(double), y(double), point3D_id(int64) = 24 bytes each.
            f.seek(num_points2d * 24, os.SEEK_CUR)

            images[image_id] = ColmapImage(
                image_id=image_id,
                qvec=qvec,
                tvec=tvec,
                camera_id=camera_id,
                name=name,
            )

    return images


@dataclass
class ColmapPoint:
    point_id: int
    xyz: np.ndarray
    rgb: np.ndarray
    image_ids: list[int]


def read_points3d_bin(path: Path) -> list[ColmapPoint]:
    points: list[ColmapPoint] = []

    with path.open("rb") as f:
        num_points = struct.unpack("<Q", f.read(8))[0]

        for _ in range(num_points):
            point_id = struct.unpack("<Q", f.read(8))[0]
            xyz = np.array(struct.unpack("<ddd", f.read(24)), dtype=float)
            rgb = np.array(struct.unpack("<BBB", f.read(3)), dtype=float) / 255.0
            _error = struct.unpack("<d", f.read(8))[0]

            track_len = struct.unpack("<Q", f.read(8))[0]
            image_ids: list[int] = []

            for _ in range(track_len):
                image_id, _point2d_idx = struct.unpack("<ii", f.read(8))
                image_ids.append(image_id)

            points.append(
                ColmapPoint(
                    point_id=point_id,
                    xyz=xyz,
                    rgb=rgb,
                    image_ids=image_ids,
                )
            )

    return points


# ---------------------------------------------------------------------------
# Reconstruction build using the user's existing, working video_to_3d.py
# ---------------------------------------------------------------------------

def build_reconstruction() -> tuple[float, float]:
    if not VIDEO_TO_3D_PATH.exists():
        raise RuntimeError(f"Missing: {VIDEO_TO_3D_PATH}")
    if not VIDEO_PATH.exists():
        raise RuntimeError("No recording exists yet.")

    spec = importlib.util.spec_from_file_location(
        "_slam_video_to_3d",
        VIDEO_TO_3D_PATH,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("Could not load video_to_3d.py")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    # The existing reconstruction script normally opens a static Open3D
    # viewer at the end. Suppress only that viewer here; do not modify the file.
    if hasattr(module, "open_point_cloud"):
        module.open_point_cloud = lambda: None

    original_argv = sys.argv[:]
    try:
        sys.argv = [str(VIDEO_TO_3D_PATH), str(VIDEO_PATH)]
        result = module.main()
    finally:
        sys.argv = original_argv

    if result not in (None, 0):
        raise RuntimeError("video_to_3d.py failed.")

    sample_fps = float(getattr(module, "SAMPLE_FPS", 3.0))

    cap = cv2.VideoCapture(str(VIDEO_PATH))
    fps = float(cap.get(cv2.CAP_PROP_FPS))
    cap.release()

    if fps <= 0:
        fps = 30.0

    return sample_fps, fps


# ---------------------------------------------------------------------------
# Replay preparation
# ---------------------------------------------------------------------------

FRAME_NAME_RE = re.compile(r"frame_(\d+)\.(?:jpg|jpeg|png)$", re.I)


def find_model_dir() -> Path:
    sparse_dir = RECON_DIR / "sparse"
    candidates = sorted(p for p in sparse_dir.iterdir() if p.is_dir())
    if not candidates:
        raise RuntimeError("No COLMAP sparse model found.")
    return candidates[0]


def read_frame_log() -> list[dict[str, Any]]:
    if not FRAME_LOG_PATH.exists():
        raise RuntimeError(f"Missing frame log: {FRAME_LOG_PATH}")

    rows: list[dict[str, Any]] = []
    with FRAME_LOG_PATH.open("r", newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            converted: dict[str, Any] = dict(row)
            for key in (
                "frame_index", "t_seconds", "accel_x", "accel_y", "accel_z",
                "gyro_x", "gyro_y", "gyro_z", "pitch_deg", "roll_deg",
                "tilt_deg", "distance_mm", "sensor_age_ms",
            ):
                try:
                    converted[key] = float(row[key])
                except (ValueError, TypeError):
                    converted[key] = float("nan")
            rows.append(converted)
    return rows


@dataclass
class ReplayPose:
    t_seconds: float
    image_id: int
    sample_index: int
    center: np.ndarray
    forward: np.ndarray
    right: np.ndarray
    down: np.ndarray


def make_replay_data(
    sample_fps: float,
    source_video_fps: float,
) -> tuple[list[ReplayPose], np.ndarray, np.ndarray, np.ndarray]:
    model_dir = find_model_dir()

    images = read_images_bin(model_dir / "images.bin")
    points = read_points3d_bin(model_dir / "points3D.bin")
    frame_rows = read_frame_log()

    sample_step = max(1, round(source_video_fps / sample_fps))

    pose_items = []
    for image in images.values():
        match = FRAME_NAME_RE.search(image.name)
        if not match:
            continue

        sample_index = int(match.group(1))
        source_frame_index = sample_index * sample_step

        if 0 <= source_frame_index < len(frame_rows):
            t = float(frame_rows[source_frame_index]["t_seconds"])
        else:
            t = source_frame_index / max(source_video_fps, 1e-6)

        pose_items.append(
            (
                sample_index,
                ReplayPose(
                    t_seconds=t,
                    image_id=image.image_id,
                    sample_index=sample_index,
                    center=image.center,
                    forward=image.forward,
                    right=image.right,
                    down=image.down,
                ),
            )
        )

    pose_items.sort(key=lambda item: item[0])
    poses = [item[1] for item in pose_items]

    if not poses:
        raise RuntimeError("No registered COLMAP poses could be mapped to video frames.")

    image_order = {pose.image_id: i for i, pose in enumerate(poses)}

    xyz = []
    rgb = []
    first_seen = []

    for point in points:
        seen_orders = [
            image_order[image_id]
            for image_id in point.image_ids
            if image_id in image_order
        ]
        if not seen_orders:
            continue

        xyz.append(point.xyz)
        rgb.append(point.rgb)
        first_seen.append(min(seen_orders))

    if not xyz:
        raise RuntimeError("No replayable 3D points found.")

    xyz_array = np.asarray(xyz, dtype=float)
    rgb_array = np.asarray(rgb, dtype=float)
    first_seen_array = np.asarray(first_seen, dtype=int)

    sort_order = np.argsort(first_seen_array)
    xyz_array = xyz_array[sort_order]
    rgb_array = rgb_array[sort_order]
    first_seen_array = first_seen_array[sort_order]

    return poses, xyz_array, rgb_array, first_seen_array


# ---------------------------------------------------------------------------
# Replay visualisation
# ---------------------------------------------------------------------------

def make_frustum_lines(
    pose: ReplayPose,
    scale: float,
) -> tuple[np.ndarray, np.ndarray]:
    c = pose.center
    f = pose.forward / max(np.linalg.norm(pose.forward), 1e-9)
    r = pose.right / max(np.linalg.norm(pose.right), 1e-9)
    d = pose.down / max(np.linalg.norm(pose.down), 1e-9)

    depth = scale
    half_w = scale * 0.70
    half_h = scale * 0.48

    plane_center = c + f * depth

    corners = np.array(
        [
            plane_center - r * half_w - d * half_h,
            plane_center + r * half_w - d * half_h,
            plane_center + r * half_w + d * half_h,
            plane_center - r * half_w + d * half_h,
        ],
        dtype=float,
    )

    pts = np.vstack([c, corners, c + f * (scale * 1.6)])

    lines = np.array(
        [
            [0, 1], [0, 2], [0, 3], [0, 4],
            [1, 2], [2, 3], [3, 4], [4, 1],
            [0, 5],  # long forward-looking ray
        ],
        dtype=np.int32,
    )

    return pts, lines


def nearest_frame_row(frame_rows: list[dict[str, Any]], index: int) -> dict[str, Any]:
    if not frame_rows:
        return {}
    index = max(0, min(index, len(frame_rows) - 1))
    return frame_rows[index]


def draw_replay_overlay(
    frame: np.ndarray,
    row: dict[str, Any],
    pose: ReplayPose,
    current_pose_index: int,
    total_poses: int,
) -> np.ndarray:
    display = frame.copy()

    overlay = display.copy()
    cv2.rectangle(overlay, (15, 15), (660, 175), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.65, display, 0.35, 0, display)

    def fnum(key: str, decimals: int = 2) -> str:
        value = row.get(key, float("nan"))
        try:
            value = float(value)
        except (TypeError, ValueError):
            return "--"
        if not math.isfinite(value):
            return "--"
        return f"{value:.{decimals}f}"

    distance_mm = row.get("distance_mm", float("nan"))
    try:
        distance_m = float(distance_mm) / 1000.0
    except (TypeError, ValueError):
        distance_m = float("nan")

    pos = pose.center

    lines = [
        f"Replay pose {current_pose_index + 1}/{total_poses}",
        f"COLMAP position: x={pos[0]:.3f} y={pos[1]:.3f} z={pos[2]:.3f}  (arbitrary scale)",
        f"IMU pitch={fnum('pitch_deg')} deg   roll={fnum('roll_deg')} deg   "
        f"gyroZ={fnum('gyro_z')} deg/s",
        f"Range: {distance_m:.3f} m" if math.isfinite(distance_m) else "Range: --",
    ]

    y = 42
    for text in lines:
        cv2.putText(
            display,
            text,
            (30, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        y += 33

    return display


def replay_session(sample_fps: float | None = None, source_video_fps: float | None = None) -> None:
    if sample_fps is None or source_video_fps is None:
        # Replay-only mode. Recover the sampling settings from the unchanged
        # video_to_3d.py if possible.
        spec = importlib.util.spec_from_file_location(
            "_slam_video_to_3d_replay",
            VIDEO_TO_3D_PATH,
        )
        if spec is None or spec.loader is None:
            raise RuntimeError("Could not load video_to_3d.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        sample_fps = float(getattr(module, "SAMPLE_FPS", 3.0))

        cap_probe = cv2.VideoCapture(str(VIDEO_PATH))
        source_video_fps = float(cap_probe.get(cv2.CAP_PROP_FPS))
        cap_probe.release()
        if source_video_fps <= 0:
            source_video_fps = 30.0

    poses, xyz, rgb, first_seen = make_replay_data(
        float(sample_fps),
        float(source_video_fps),
    )

    frame_rows = read_frame_log()

    cap = cv2.VideoCapture(str(VIDEO_PATH))
    if not cap.isOpened():
        raise RuntimeError("Could not open recorded video for replay.")

    video_fps = float(cap.get(cv2.CAP_PROP_FPS))
    if video_fps <= 0:
        video_fps = 30.0

    pose_times = [pose.t_seconds for pose in poses]

    bounds = np.ptp(xyz, axis=0)
    map_extent = max(float(np.max(bounds)), 1e-3)
    frustum_scale = map_extent * 0.04

    # Static/dynamic Open3D objects.
    map_cloud = o3d.geometry.PointCloud()
    map_cloud.points = o3d.utility.Vector3dVector(np.empty((0, 3)))
    map_cloud.colors = o3d.utility.Vector3dVector(np.empty((0, 3)))

    trajectory = o3d.geometry.LineSet()
    trajectory.points = o3d.utility.Vector3dVector(np.asarray([poses[0].center]))
    trajectory.lines = o3d.utility.Vector2iVector(np.empty((0, 2), dtype=np.int32))

    frustum = o3d.geometry.LineSet()
    fpts, flines = make_frustum_lines(poses[0], frustum_scale)
    frustum.points = o3d.utility.Vector3dVector(fpts)
    frustum.lines = o3d.utility.Vector2iVector(flines)

    vis = o3d.visualization.Visualizer()
    vis.create_window(
        window_name="SLAM Replay - Building the Map",
        width=1100,
        height=750,
    )
    vis.add_geometry(map_cloud)
    vis.add_geometry(trajectory)
    vis.add_geometry(frustum)

    render = vis.get_render_option()
    render.point_size = 3.0

    current_pose_index = -1
    frame_index = 0
    replay_start = time.perf_counter()

    cv2.namedWindow("Recorded Camera Replay", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Recorded Camera Replay", 900, 650)

    print("\nREPLAY")
    print("3D window: fixed world, moving camera frustum and growing map.")
    print("Video window: recorded camera with synchronized IMU/range overlay.")
    print("Press Q in the video window to stop replay.")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break

            if frame_index < len(frame_rows):
                t = float(frame_rows[frame_index].get("t_seconds", frame_index / video_fps))
            else:
                t = frame_index / video_fps

            new_pose_index = bisect.bisect_right(pose_times, t) - 1
            new_pose_index = max(0, min(new_pose_index, len(poses) - 1))

            if new_pose_index != current_pose_index:
                current_pose_index = new_pose_index
                pose = poses[current_pose_index]

                # Reveal all points whose first COLMAP observation has occurred.
                visible_count = int(
                    np.searchsorted(
                        first_seen,
                        current_pose_index,
                        side="right",
                    )
                )
                map_cloud.points = o3d.utility.Vector3dVector(xyz[:visible_count])
                map_cloud.colors = o3d.utility.Vector3dVector(rgb[:visible_count])

                centers = np.asarray(
                    [p.center for p in poses[: current_pose_index + 1]],
                    dtype=float,
                )
                if len(centers) == 1:
                    centers = np.vstack([centers, centers])

                lines = np.array(
                    [[i, i + 1] for i in range(len(centers) - 1)],
                    dtype=np.int32,
                )
                trajectory.points = o3d.utility.Vector3dVector(centers)
                trajectory.lines = o3d.utility.Vector2iVector(lines)

                fpts, flines = make_frustum_lines(pose, frustum_scale)
                frustum.points = o3d.utility.Vector3dVector(fpts)
                frustum.lines = o3d.utility.Vector2iVector(flines)

                vis.update_geometry(map_cloud)
                vis.update_geometry(trajectory)
                vis.update_geometry(frustum)

            pose = poses[current_pose_index]
            row = nearest_frame_row(frame_rows, frame_index)
            display = draw_replay_overlay(
                frame,
                row,
                pose,
                current_pose_index,
                len(poses),
            )

            cv2.imshow("Recorded Camera Replay", display)

            if not vis.poll_events():
                break
            vis.update_renderer()

            # Replay at approximately the recorded timing.
            target_wall = replay_start + t
            sleep_time = target_wall - time.perf_counter()
            if sleep_time > 0:
                time.sleep(min(sleep_time, 0.05))

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

            frame_index += 1

    finally:
        cap.release()
        cv2.destroyWindow("Recorded Camera Replay")
        vis.destroy_window()


# ---------------------------------------------------------------------------
# Live capture UI
# ---------------------------------------------------------------------------

class LiveApp:
    def __init__(self):
        self.sensor_hub = SensorHub(ATECH_GATEWAY_URL)
        self.sensor_hub.start()
        self.recorder = SessionRecorder(self.sensor_hub)

        self.cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open Polaroid camera #{CAMERA_INDEX}.")

        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        fps = float(self.cap.get(cv2.CAP_PROP_FPS))
        self.camera_fps = fps if 1 <= fps <= 120 else 30.0

        self.last_frame = None
        self.build_requested = False

        self.record_rect = (20, 0, 200, 0)
        self.stop_rect = (220, 0, 400, 0)
        self.build_rect = (420, 0, 700, 0)

    @staticmethod
    def _inside(rect, x, y) -> bool:
        x1, y1, x2, y2 = rect
        return x1 <= x <= x2 and y1 <= y <= y2

    def mouse(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if self._inside(self.record_rect, x, y):
            if not self.recorder.recording and self.last_frame is not None:
                self.recorder.start(self.last_frame, self.camera_fps)

        elif self._inside(self.stop_rect, x, y):
            self.recorder.stop()

        elif self._inside(self.build_rect, x, y):
            if self.recorder.recording:
                self.recorder.stop()

            if VIDEO_PATH.exists() and FRAME_LOG_PATH.exists():
                self.build_requested = True

    def draw_ui(self, frame: np.ndarray) -> np.ndarray:
        display = frame.copy()
        h, w = display.shape[:2]

        snap = self.sensor_hub.snapshot()

        status = "RECORDING" if self.recorder.recording else "READY"
        sensor_status = "ATECH ONLINE" if snap.connected else "ATECH OFFLINE"

        cv2.rectangle(display, (15, 15), (610, 130), (0, 0, 0), -1)

        cv2.putText(
            display,
            status,
            (30, 45),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 0, 255) if self.recorder.recording else (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            display,
            sensor_status,
            (220, 45),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0) if snap.connected else (0, 0, 255),
            2,
            cv2.LINE_AA,
        )

        def num(value, suffix=""):
            return f"{value:.2f}{suffix}" if math.isfinite(value) else "--"

        distance = snap.distance_mm / 1000.0 if math.isfinite(snap.distance_mm) else float("nan")

        cv2.putText(
            display,
            f"Pitch {num(snap.pitch_deg, ' deg')}   Roll {num(snap.roll_deg, ' deg')}   "
            f"Range {num(distance, ' m')}",
            (30, 82),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

        cv2.putText(
            display,
            f"Gyro: {num(snap.gyro_x)}, {num(snap.gyro_y)}, {num(snap.gyro_z)} deg/s",
            (30, 112),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

        by1 = h - 70
        by2 = h - 15

        self.record_rect = (20, by1, 200, by2)
        self.stop_rect = (220, by1, 400, by2)
        self.build_rect = (420, by1, min(730, w - 20), by2)

        cv2.rectangle(
            display,
            (20, by1),
            (200, by2),
            (40, 170, 40) if not self.recorder.recording else (70, 70, 70),
            -1,
        )
        cv2.rectangle(
            display,
            (220, by1),
            (400, by2),
            (40, 40, 200) if self.recorder.recording else (70, 70, 70),
            -1,
        )
        cv2.rectangle(
            display,
            (420, by1),
            (min(730, w - 20), by2),
            (180, 110, 20) if VIDEO_PATH.exists() else (70, 70, 70),
            -1,
        )

        cv2.putText(
            display, "RECORD", (55, by1 + 37),
            cv2.FONT_HERSHEY_SIMPLEX, 0.72, (255, 255, 255), 2, cv2.LINE_AA
        )
        cv2.putText(
            display, "STOP", (275, by1 + 37),
            cv2.FONT_HERSHEY_SIMPLEX, 0.72, (255, 255, 255), 2, cv2.LINE_AA
        )
        cv2.putText(
            display, "BUILD + REPLAY", (445, by1 + 37),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA
        )

        return display

    def run(self) -> bool:
        cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_TITLE, 1100, 760)
        cv2.setMouseCallback(WINDOW_TITLE, self.mouse)

        print("Polaroid camera opened.")
        print("Atech Wi-Fi receiver started.")
        print("Buttons: RECORD, STOP, BUILD + REPLAY")
        print("Keys: R record, S stop, B build+replay, Q quit")

        try:
            while True:
                ok, frame = self.cap.read()
                if not ok:
                    raise RuntimeError("Could not read camera frame.")

                self.last_frame = frame.copy()

                if self.recorder.recording:
                    self.recorder.write_frame(frame)

                display = self.draw_ui(frame)
                cv2.imshow(WINDOW_TITLE, display)

                if self.build_requested:
                    break

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                elif key == ord("r") and not self.recorder.recording:
                    self.recorder.start(self.last_frame, self.camera_fps)
                elif key == ord("s"):
                    self.recorder.stop()
                elif key == ord("b"):
                    if self.recorder.recording:
                        self.recorder.stop()
                    if VIDEO_PATH.exists() and FRAME_LOG_PATH.exists():
                        self.build_requested = True
                        break

        finally:
            self.recorder.stop()
            self.cap.release()
            cv2.destroyAllWindows()

        return self.build_requested


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--replay",
        action="store_true",
        help="Replay the latest already-built recording/reconstruction.",
    )
    args = parser.parse_args()

    try:
        if args.replay:
            replay_session()
            return 0

        app = LiveApp()
        should_build = app.run()

        if not should_build:
            return 0

        print("\n" + "=" * 72)
        print("BUILDING 3D RECONSTRUCTION")
        print("=" * 72)
        print("Using your existing video_to_3d.py unchanged.")

        sample_fps, source_video_fps = build_reconstruction()

        print("\n" + "=" * 72)
        print("REPLAYING RECORDED SESSION")
        print("=" * 72)

        replay_session(sample_fps, source_video_fps)
        return 0

    except Exception as exc:
        print(f"\nERROR: {type(exc).__name__}: {exc}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
