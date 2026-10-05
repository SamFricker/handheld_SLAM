r"""
SLAM Project v2: capture camera + Atech IMU/range, reconstruct, and use an
interactive synchronized 3D/video replay.

This script does NOT modify video_to_3d.py.

Capture:
    .\.venv\Scripts\python.exe slam_capture_replay_v2.py

Replay an already-built session:
    .\.venv\Scripts\python.exe slam_capture_replay_v2.py --replay

Replay controls:
- SPACE: play/pause
- Left / A: 1 second back
- Right / D: 1 second forward
- , and .: single-frame back/forward
- J / L: 5 seconds back/forward
- R: restart
- F: toggle final full map vs map-built-so-far
- Q: quit replay
- Timeline slider: scrub anywhere in the recording
- In the 3D window, use the mouse normally to orbit, zoom and pan.

Map scale:
The script estimates metres-per-COLMAP-unit from synchronized VL53L5CX
readings <= 1.0 m. The sensor is assumed to point straight ahead. Readings
above 1.0 m are treated as unknown. Scale is approximate because the sparse
visual reconstruction and ToF beam do not sample exactly the same geometry.
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
    "012a8d5c-dbb2-4055-8419-9113b3c9fe18"
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
        now_perf = time.perf_counter()

        if self.raw_packet_callback is not None:
            try:
                self.raw_packet_callback(packet, now_perf)
            except Exception:
                pass

        if not isinstance(packet, dict):
            return

        payload = packet.get("payload")
        if not isinstance(payload, dict):
            payload = packet

        key = payload.get("key")
        value = payload.get("value")
        if not isinstance(key, str):
            return

        key = key.lower()

        with self.lock:
            if key == "accel_x":
                self.state.accel_x = self._number(value)
            elif key == "accel_y":
                self.state.accel_y = self._number(value)
            elif key == "accel_z":
                self.state.accel_z = self._number(value)
            elif key == "gyro_x":
                self.state.gyro_x = self._number(value)
            elif key == "gyro_y":
                self.state.gyro_y = self._number(value)
            elif key == "gyro_z":
                self.state.gyro_z = self._number(value)
            elif key in ("distance", "min_distance"):
                # Prefer min_distance if both are being transmitted; either is
                # the calibrated VL53L5CX result in millimetres in this project.
                number = self._number(value)
                if math.isfinite(number) and number >= 0:
                    self.state.distance_mm = number
            elif key == "orientation" and isinstance(value, str):
                # Atech format:
                #   "pitch,roll,tilt,orientation_name"
                parts = [item.strip() for item in value.split(",")]
                if len(parts) >= 1:
                    self.state.pitch_deg = self._number(parts[0])
                if len(parts) >= 2:
                    self.state.roll_deg = self._number(parts[1])
                if len(parts) >= 3:
                    self.state.tilt_deg = self._number(parts[2])
                if len(parts) >= 4:
                    self.state.orientation = parts[3]

            self.state.packets += 1
            self.state.last_packet_perf = now_perf

    async def _loop(self) -> None:
        while True:
            try:
                async with websockets.connect(
                    self.url,
                    ping_interval=20,
                    ping_timeout=10,
                    close_timeout=2,
                    max_size=2**20,
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
    xys: np.ndarray
    point3d_ids: np.ndarray

    @property
    def rotation(self) -> np.ndarray:
        # COLMAP world -> camera rotation.
        return qvec_to_rotmat(self.qvec)

    @property
    def c2w(self) -> np.ndarray:
        return self.rotation.T

    @property
    def center(self) -> np.ndarray:
        return -self.rotation.T @ self.tvec

    @property
    def forward(self) -> np.ndarray:
        return self.c2w[:, 2]

    @property
    def right(self) -> np.ndarray:
        return self.c2w[:, 0]

    @property
    def down(self) -> np.ndarray:
        return self.c2w[:, 1]


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
            xys = np.empty((num_points2d, 2), dtype=float)
            point3d_ids = np.empty(num_points2d, dtype=np.int64)

            for i in range(num_points2d):
                x, y, point3d_id = struct.unpack("<ddq", f.read(24))
                xys[i] = (x, y)
                point3d_ids[i] = point3d_id

            images[image_id] = ColmapImage(
                image_id=image_id,
                qvec=qvec,
                tvec=tvec,
                camera_id=camera_id,
                name=name,
                xys=xys,
                point3d_ids=point3d_ids,
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


def _binary_count(path: Path) -> int:
    try:
        with path.open("rb") as f:
            raw = f.read(8)
            return int(struct.unpack("<Q", raw)[0]) if len(raw) == 8 else 0
    except OSError:
        return 0


def find_model_dir() -> Path:
    """Choose the successful COLMAP component with the most useful content."""
    sparse_dir = RECON_DIR / "sparse"
    if not sparse_dir.exists():
        raise RuntimeError("No COLMAP sparse directory found.")

    candidates = [p for p in sparse_dir.iterdir() if p.is_dir()]
    if not candidates:
        raise RuntimeError("No COLMAP sparse model found.")

    scored = []
    for model in candidates:
        n_images = _binary_count(model / "images.bin")
        n_points = _binary_count(model / "points3D.bin")
        if n_images > 0 and n_points > 0:
            scored.append((n_images, n_points, model))

    if not scored:
        raise RuntimeError("COLMAP model folders exist, but none contains a usable map.")

    scored.sort(key=lambda item: (item[0], item[1]), reverse=True)
    n_images, n_points, model = scored[0]
    print(f"Replay model: {model.name}  ({n_images} registered images, {n_points} 3D points)")
    return model


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
    source_frame_index: int
    center: np.ndarray
    c2w: np.ndarray

    @property
    def forward(self) -> np.ndarray:
        return self.c2w[:, 2]

    @property
    def right(self) -> np.ndarray:
        return self.c2w[:, 0]

    @property
    def down(self) -> np.ndarray:
        return self.c2w[:, 1]


def _valid_distance_m(row: dict[str, Any]) -> float | None:
    try:
        mm = float(row.get("distance_mm", float("nan")))
    except (TypeError, ValueError):
        return None
    if not math.isfinite(mm):
        return None
    metres = mm / 1000.0
    # User's ToF is only trusted to about 1 metre. Higher means unknown.
    if 0.04 <= metres <= 1.0:
        return metres
    return None


def _estimate_metric_scale(
    poses: list[ReplayPose],
    images: dict[int, ColmapImage],
    points_by_id: dict[int, ColmapPoint],
    frame_rows: list[dict[str, Any]],
) -> tuple[float | None, list[float]]:
    """
    Estimate metres per COLMAP unit from the forward ToF measurement.

    Prefer reconstructed features near the centre of each camera image,
    because the ToF is assumed to point straight ahead. Use a forward-ray
    fallback if the central region has too few reconstructed features.
    """
    if not poses or not points_by_id:
        return None, []

    # All extracted reconstruction frames have the same dimensions.
    probe_path = RECON_DIR / "images" / images[poses[0].image_id].name
    probe = cv2.imread(str(probe_path))
    if probe is not None:
        image_h, image_w = probe.shape[:2]
    else:
        cap = cv2.VideoCapture(str(VIDEO_PATH))
        image_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or 1
        image_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 1
        cap.release()

    cx = image_w / 2.0
    cy = image_h / 2.0
    ratios: list[float] = []

    all_xyz = np.asarray([p.xyz for p in points_by_id.values()], dtype=float)

    for pose in poses:
        if not (0 <= pose.source_frame_index < len(frame_rows)):
            continue

        measured_m = _valid_distance_m(frame_rows[pose.source_frame_index])
        if measured_m is None:
            continue

        image = images.get(pose.image_id)
        if image is None:
            continue

        valid = image.point3d_ids >= 0
        ids = image.point3d_ids[valid]
        xys = image.xys[valid]

        candidate_distances: list[float] = []

        if len(ids):
            # Normalized radial distance from image centre. 0.22 means the
            # central ~22% of the half-width/half-height region.
            rr = np.sqrt(
                ((xys[:, 0] - cx) / max(cx, 1.0)) ** 2
                + ((xys[:, 1] - cy) / max(cy, 1.0)) ** 2
            )

            for radius_limit in (0.18, 0.28, 0.40):
                chosen = ids[rr <= radius_limit]
                vals = []
                for point_id in chosen:
                    point = points_by_id.get(int(point_id))
                    if point is None:
                        continue
                    vec = point.xyz - pose.center
                    forward_depth = float(np.dot(vec, pose.forward))
                    if forward_depth > 1e-6:
                        vals.append(float(np.linalg.norm(vec)))
                if len(vals) >= 2:
                    candidate_distances = vals
                    break

        if not candidate_distances:
            # Fallback: use all reconstructed points close to the forward ray.
            vec = all_xyz - pose.center
            forward = vec @ pose.forward
            perp = np.linalg.norm(vec - forward[:, None] * pose.forward[None, :], axis=1)
            angle = np.arctan2(perp, np.maximum(forward, 1e-12))
            mask = (forward > 1e-6) & (angle <= math.radians(15.0))
            if np.count_nonzero(mask) >= 2:
                candidate_distances = np.linalg.norm(vec[mask], axis=1).tolist()

        if not candidate_distances:
            continue

        # The ToF should hit the first surface along the central view. Sparse
        # SfM has holes, so use a low percentile rather than a brittle minimum.
        model_distance = float(np.percentile(candidate_distances, 25.0))
        if model_distance <= 1e-9:
            continue

        ratio = measured_m / model_distance
        if math.isfinite(ratio) and ratio > 0:
            ratios.append(ratio)

    if not ratios:
        return None, []

    arr = np.asarray(ratios, dtype=float)

    # Robust filtering in log space because scale ratios are multiplicative.
    if len(arr) >= 4:
        logv = np.log(arr)
        med = float(np.median(logv))
        mad = float(np.median(np.abs(logv - med)))
        tolerance = max(2.8 * mad, 0.30)
        keep = np.abs(logv - med) <= tolerance
        if np.count_nonzero(keep) >= 2:
            arr = arr[keep]

    scale = float(np.median(arr))
    return scale, arr.tolist()


def make_replay_data(
    sample_fps: float,
    source_video_fps: float,
) -> tuple[
    list[ReplayPose], np.ndarray, np.ndarray, np.ndarray,
    float | None, list[dict[str, Any]],
]:
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
                    source_frame_index=source_frame_index,
                    center=image.center.copy(),
                    c2w=image.c2w.copy(),
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
        raise RuntimeError("The selected COLMAP component contains no replayable 3D points.")

    points_by_id = {point.point_id: point for point in points}
    metric_scale, scale_samples = _estimate_metric_scale(
        poses, images, points_by_id, frame_rows
    )

    xyz_array = np.asarray(xyz, dtype=float)
    rgb_array = np.asarray(rgb, dtype=float)
    first_seen_array = np.asarray(first_seen, dtype=int)

    if metric_scale is not None:
        xyz_array *= metric_scale
        for pose in poses:
            pose.center *= metric_scale

        print(
            f"Metric scale estimated from {len(scale_samples)} valid ToF samples: "
            f"{metric_scale:.6f} metres/COLMAP-unit"
        )
        if len(scale_samples) == 1:
            print("Scale warning: only one usable ToF/map correspondence was available.")
    else:
        print("Metric scale unavailable: no reliable <=1 m ToF/map correspondence was found.")

    sort_order = np.argsort(first_seen_array)
    xyz_array = xyz_array[sort_order]
    rgb_array = rgb_array[sort_order]
    first_seen_array = first_seen_array[sort_order]

    print(f"Replay map points: {len(xyz_array):,}")
    print(f"Replay camera poses: {len(poses):,}")

    return (
        poses,
        xyz_array,
        rgb_array,
        first_seen_array,
        metric_scale,
        frame_rows,
    )


# ---------------------------------------------------------------------------
# Replay visualisation
# ---------------------------------------------------------------------------

def _rotmat_to_quat(r: np.ndarray) -> np.ndarray:
    """Rotation matrix -> [w, x, y, z]."""
    trace = float(np.trace(r))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = np.array([
            0.25 * s,
            (r[2, 1] - r[1, 2]) / s,
            (r[0, 2] - r[2, 0]) / s,
            (r[1, 0] - r[0, 1]) / s,
        ])
    else:
        i = int(np.argmax(np.diag(r)))
        if i == 0:
            s = math.sqrt(max(1.0 + r[0, 0] - r[1, 1] - r[2, 2], 0.0)) * 2.0
            q = np.array([
                (r[2, 1] - r[1, 2]) / max(s, 1e-12),
                0.25 * s,
                (r[0, 1] + r[1, 0]) / max(s, 1e-12),
                (r[0, 2] + r[2, 0]) / max(s, 1e-12),
            ])
        elif i == 1:
            s = math.sqrt(max(1.0 + r[1, 1] - r[0, 0] - r[2, 2], 0.0)) * 2.0
            q = np.array([
                (r[0, 2] - r[2, 0]) / max(s, 1e-12),
                (r[0, 1] + r[1, 0]) / max(s, 1e-12),
                0.25 * s,
                (r[1, 2] + r[2, 1]) / max(s, 1e-12),
            ])
        else:
            s = math.sqrt(max(1.0 + r[2, 2] - r[0, 0] - r[1, 1], 0.0)) * 2.0
            q = np.array([
                (r[1, 0] - r[0, 1]) / max(s, 1e-12),
                (r[0, 2] + r[2, 0]) / max(s, 1e-12),
                (r[1, 2] + r[2, 1]) / max(s, 1e-12),
                0.25 * s,
            ])
    norm = np.linalg.norm(q)
    return q / max(norm, 1e-12)


def _quat_to_rotmat(q: np.ndarray) -> np.ndarray:
    q = q / max(np.linalg.norm(q), 1e-12)
    w, x, y, z = q
    return np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*w*z,     2*x*z + 2*w*y],
        [2*x*y + 2*w*z,     1 - 2*x*x - 2*z*z, 2*y*z - 2*w*x],
        [2*x*z - 2*w*y,     2*y*z + 2*w*x,     1 - 2*x*x - 2*y*y],
    ], dtype=float)


def _slerp(q0: np.ndarray, q1: np.ndarray, alpha: float) -> np.ndarray:
    q0 = q0 / max(np.linalg.norm(q0), 1e-12)
    q1 = q1 / max(np.linalg.norm(q1), 1e-12)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        q = q0 + alpha * (q1 - q0)
        return q / max(np.linalg.norm(q), 1e-12)
    theta0 = math.acos(dot)
    sin0 = math.sin(theta0)
    a = math.sin((1.0 - alpha) * theta0) / sin0
    b = math.sin(alpha * theta0) / sin0
    return a * q0 + b * q1


def interpolated_pose(poses: list[ReplayPose], t: float) -> tuple[ReplayPose, int]:
    times = [p.t_seconds for p in poses]
    right = bisect.bisect_right(times, t)
    if right <= 0:
        return poses[0], 0
    if right >= len(poses):
        return poses[-1], len(poses) - 1

    a = poses[right - 1]
    b = poses[right]
    span = max(b.t_seconds - a.t_seconds, 1e-9)
    u = float(np.clip((t - a.t_seconds) / span, 0.0, 1.0))

    center = (1.0 - u) * a.center + u * b.center
    qa = _rotmat_to_quat(a.c2w)
    qb = _rotmat_to_quat(b.c2w)
    c2w = _quat_to_rotmat(_slerp(qa, qb, u))

    pose = ReplayPose(
        t_seconds=t,
        image_id=a.image_id,
        sample_index=a.sample_index,
        source_frame_index=a.source_frame_index,
        center=center,
        c2w=c2w,
    )
    return pose, right - 1


def make_frustum_lines(pose: ReplayPose, scale: float) -> tuple[np.ndarray, np.ndarray]:
    c = pose.center
    f = pose.forward / max(np.linalg.norm(pose.forward), 1e-9)
    r = pose.right / max(np.linalg.norm(pose.right), 1e-9)
    d = pose.down / max(np.linalg.norm(pose.down), 1e-9)

    depth = scale
    half_w = scale * 0.70
    half_h = scale * 0.48
    plane_center = c + f * depth

    corners = np.array([
        plane_center - r * half_w - d * half_h,
        plane_center + r * half_w - d * half_h,
        plane_center + r * half_w + d * half_h,
        plane_center - r * half_w + d * half_h,
    ])

    pts = np.vstack([c, corners, c + f * (scale * 1.8)])
    lines = np.array([
        [0, 1], [0, 2], [0, 3], [0, 4],
        [1, 2], [2, 3], [3, 4], [4, 1],
        [0, 5],
    ], dtype=np.int32)
    return pts, lines


def make_scale_bar(xyz: np.ndarray, metric: bool) -> o3d.geometry.LineSet:
    lo = xyz.min(axis=0)
    hi = xyz.max(axis=0)
    extent = max(float(np.max(hi - lo)), 1e-6)
    length = 1.0 if metric else extent * 0.20
    tick = max(length * 0.08, extent * 0.005)
    start = lo + np.array([0.03 * extent, 0.03 * extent, 0.03 * extent])
    end = start + np.array([length, 0.0, 0.0])

    points = np.array([
        start, end,
        start + [0, -tick, 0], start + [0, tick, 0],
        end + [0, -tick, 0], end + [0, tick, 0],
    ], dtype=float)
    lines = np.array([[0,1], [2,3], [4,5]], dtype=np.int32)
    bar = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines),
    )
    bar.colors = o3d.utility.Vector3dVector(
        np.tile(np.array([[1.0, 1.0, 1.0]]), (len(lines), 1))
    )
    return bar


def draw_replay_overlay(
    frame: np.ndarray,
    row: dict[str, Any],
    pose: ReplayPose,
    pose_index: int,
    total_poses: int,
    playing: bool,
    current_frame: int,
    total_frames: int,
    metric_scale: float | None,
    full_map: bool,
) -> np.ndarray:
    display = frame.copy()
    overlay = display.copy()
    cv2.rectangle(overlay, (15, 15), (790, 220), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.68, display, 0.32, 0, display)

    def number(key: str, decimals: int = 2) -> str:
        try:
            value = float(row.get(key, float("nan")))
        except (TypeError, ValueError):
            return "--"
        return f"{value:.{decimals}f}" if math.isfinite(value) else "--"

    try:
        raw_distance_m = float(row.get("distance_mm", float("nan"))) / 1000.0
    except (TypeError, ValueError):
        raw_distance_m = float("nan")

    if math.isfinite(raw_distance_m) and 0.04 <= raw_distance_m <= 1.0:
        range_text = f"{raw_distance_m:.3f} m"
    elif math.isfinite(raw_distance_m) and raw_distance_m > 1.0:
        range_text = ">1.0 m / unknown"
    else:
        range_text = "unknown"

    if metric_scale is not None:
        pos_text = f"x={pose.center[0]:.3f}  y={pose.center[1]:.3f}  z={pose.center[2]:.3f} m"
        scale_text = f"METRIC MAP  ({metric_scale:.6f} m/COLMAP unit)"
    else:
        pos_text = f"x={pose.center[0]:.3f}  y={pose.center[1]:.3f}  z={pose.center[2]:.3f} units"
        scale_text = "MAP SCALE UNKNOWN"

    lines = [
        f"{'PLAY' if playing else 'PAUSED'}   frame {current_frame + 1}/{total_frames}   pose {pose_index + 1}/{total_poses}",
        f"Position: {pos_text}",
        f"Looking direction: [{pose.forward[0]:+.2f}, {pose.forward[1]:+.2f}, {pose.forward[2]:+.2f}]",
        f"IMU pitch={number('pitch_deg')} deg   roll={number('roll_deg')} deg   gyroZ={number('gyro_z')} deg/s",
        f"Forward ToF: {range_text}   |   {scale_text}",
        f"3D map: {'FINAL FULL MAP' if full_map else 'BUILT UP TO THIS MOMENT'}",
    ]

    y = 42
    for text in lines:
        cv2.putText(
            display, text, (30, y), cv2.FONT_HERSHEY_SIMPLEX,
            0.54, (255, 255, 255), 1, cv2.LINE_AA,
        )
        y += 31

    cv2.putText(
        display,
        "SPACE pause/play | A/D +/-1s | J/L +/-5s | ,/. frame | F full map | R restart | Q quit",
        (22, display.shape[0] - 20),
        cv2.FONT_HERSHEY_SIMPLEX, 0.46, (255, 255, 255), 1, cv2.LINE_AA,
    )
    return display


class VideoFrameReader:
    def __init__(self, path: Path):
        self.cap = cv2.VideoCapture(str(path))
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open replay video: {path}")
        self.fps = float(self.cap.get(cv2.CAP_PROP_FPS))
        if self.fps <= 0:
            self.fps = 30.0
        self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.last_index = -2

    def read(self, index: int) -> np.ndarray:
        index = int(np.clip(index, 0, max(self.total_frames - 1, 0)))
        if index != self.last_index + 1:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, index)
        ok, frame = self.cap.read()
        if not ok:
            raise RuntimeError(f"Could not decode video frame {index}.")
        self.last_index = index
        return frame

    def close(self):
        self.cap.release()


class InteractiveReplay:
    def __init__(
        self,
        poses: list[ReplayPose],
        xyz: np.ndarray,
        rgb: np.ndarray,
        first_seen: np.ndarray,
        metric_scale: float | None,
        frame_rows: list[dict[str, Any]],
    ):
        self.poses = poses
        self.xyz = xyz
        self.rgb = rgb
        self.first_seen = first_seen
        self.metric_scale = metric_scale
        self.frame_rows = frame_rows

        self.video = VideoFrameReader(VIDEO_PATH)
        self.total_frames = self.video.total_frames
        self.fps = self.video.fps

        self.playhead = 0.0
        self.playing = True
        self.full_map = False
        self.quit = False
        self.last_tick = time.perf_counter()
        self.last_rendered_frame = -1
        self.trackbar_internal = False
        self.user_seek_frame: int | None = None

        bounds = np.ptp(self.xyz, axis=0)
        extent = max(float(np.max(bounds)), 1e-6)
        self.frustum_size = 0.16 if metric_scale is not None else extent * 0.04
        if metric_scale is not None:
            self.frustum_size = max(0.08, min(0.30, extent * 0.04))

        # Initialize the map with the FULL cloud so Open3D frames the complete
        # world correctly. It is immediately replaced with the progressive
        # cloud before the first render. This fixes the previous "empty map"
        # viewer problem caused by adding an empty point cloud at startup.
        self.map_cloud = o3d.geometry.PointCloud()
        self.map_cloud.points = o3d.utility.Vector3dVector(self.xyz.copy())
        self.map_cloud.colors = o3d.utility.Vector3dVector(self.rgb.copy())

        self.trajectory = o3d.geometry.LineSet()
        self.frustum = o3d.geometry.LineSet()
        self.range_ray = o3d.geometry.LineSet()
        self.scale_bar = make_scale_bar(self.xyz, metric_scale is not None)

        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis.create_window(
            window_name="SLAM Replay - Interactive 3D World",
            width=1100,
            height=760,
        )
        self.vis.add_geometry(self.map_cloud)
        self.vis.add_geometry(self.trajectory)
        self.vis.add_geometry(self.frustum)
        self.vis.add_geometry(self.range_ray)
        self.vis.add_geometry(self.scale_bar)

        render = self.vis.get_render_option()
        render.point_size = 3.5

        # Fit camera to the full map once, then never reset it automatically.
        self.vis.reset_view_point(True)
        self._register_3d_keys()

        cv2.namedWindow("Recorded Camera Replay", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Recorded Camera Replay", 900, 650)
        cv2.createTrackbar(
            "Timeline",
            "Recorded Camera Replay",
            0,
            max(self.total_frames - 1, 1),
            self._timeline_changed,
        )

    def _timeline_changed(self, value: int):
        if self.trackbar_internal:
            return
        self.user_seek_frame = int(value)
        self.playing = False

    def seek_frames(self, delta: int):
        self.playhead = float(np.clip(
            round(self.playhead) + delta,
            0,
            max(self.total_frames - 1, 0),
        ))
        self.playing = False
        self.last_tick = time.perf_counter()
        self.last_rendered_frame = -1

    def seek_seconds(self, seconds: float):
        self.seek_frames(int(round(seconds * self.fps)))

    def toggle_play(self):
        self.playing = not self.playing
        self.last_tick = time.perf_counter()

    def _register_3d_keys(self):
        def cb_space(vis):
            self.toggle_play(); return False
        def cb_left(vis):
            self.seek_seconds(-1.0); return False
        def cb_right(vis):
            self.seek_seconds(1.0); return False
        def cb_frame_back(vis):
            self.seek_frames(-1); return False
        def cb_frame_fwd(vis):
            self.seek_frames(1); return False
        def cb_j(vis):
            self.seek_seconds(-5.0); return False
        def cb_l(vis):
            self.seek_seconds(5.0); return False
        def cb_r(vis):
            self.playhead = 0.0; self.playing = False; self.last_rendered_frame = -1; return False
        def cb_f(vis):
            self.full_map = not self.full_map; self.last_rendered_frame = -1; return False
        def cb_q(vis):
            self.quit = True; return False

        self.vis.register_key_callback(32, cb_space)
        self.vis.register_key_callback(ord('A'), cb_left)
        self.vis.register_key_callback(ord('D'), cb_right)
        self.vis.register_key_callback(ord(','), cb_frame_back)
        self.vis.register_key_callback(ord('.'), cb_frame_fwd)
        self.vis.register_key_callback(ord('J'), cb_j)
        self.vis.register_key_callback(ord('L'), cb_l)
        self.vis.register_key_callback(ord('R'), cb_r)
        self.vis.register_key_callback(ord('F'), cb_f)
        self.vis.register_key_callback(ord('Q'), cb_q)
        # GLFW left/right arrow key codes.
        self.vis.register_key_callback(263, cb_left)
        self.vis.register_key_callback(262, cb_right)

    def _sensor_row(self, frame_index: int) -> dict[str, Any]:
        if not self.frame_rows:
            return {}
        return self.frame_rows[min(max(frame_index, 0), len(self.frame_rows) - 1)]

    def _update_3d(self, pose: ReplayPose, pose_index: int, row: dict[str, Any]):
        if self.full_map:
            visible = len(self.xyz)
        else:
            visible = int(np.searchsorted(self.first_seen, pose_index, side="right"))
            # Never let the world be totally blank if COLMAP's first tracked
            # point appears slightly after the first registered image.
            if visible == 0:
                visible = min(1, len(self.xyz))

        self.map_cloud.points = o3d.utility.Vector3dVector(self.xyz[:visible])
        self.map_cloud.colors = o3d.utility.Vector3dVector(self.rgb[:visible])

        centers = np.asarray([p.center for p in self.poses[:pose_index + 1]], dtype=float)
        if len(centers) == 0:
            centers = np.asarray([pose.center], dtype=float)
        if len(centers) == 1:
            centers = np.vstack([centers, centers])
        lines = np.asarray([[i, i + 1] for i in range(len(centers) - 1)], dtype=np.int32)
        self.trajectory.points = o3d.utility.Vector3dVector(centers)
        self.trajectory.lines = o3d.utility.Vector2iVector(lines)
        self.trajectory.colors = o3d.utility.Vector3dVector(
            np.tile(np.array([[1.0, 0.7, 0.1]]), (len(lines), 1))
        ) if len(lines) else o3d.utility.Vector3dVector(np.empty((0, 3)))

        fpts, flines = make_frustum_lines(pose, self.frustum_size)
        self.frustum.points = o3d.utility.Vector3dVector(fpts)
        self.frustum.lines = o3d.utility.Vector2iVector(flines)
        self.frustum.colors = o3d.utility.Vector3dVector(
            np.tile(np.array([[1.0, 0.2, 0.2]]), (len(flines), 1))
        )

        # Draw the synchronized ToF ray only when the map has metric scale and
        # the reading is inside the sensor's <=1 m usable range.
        measured_m = _valid_distance_m(row)
        if self.metric_scale is not None and measured_m is not None:
            p0 = pose.center
            p1 = pose.center + pose.forward * measured_m
            self.range_ray.points = o3d.utility.Vector3dVector(np.vstack([p0, p1]))
            self.range_ray.lines = o3d.utility.Vector2iVector(np.array([[0, 1]], dtype=np.int32))
            self.range_ray.colors = o3d.utility.Vector3dVector(np.array([[0.2, 1.0, 0.2]]))
        else:
            self.range_ray.points = o3d.utility.Vector3dVector(np.vstack([pose.center, pose.center]))
            self.range_ray.lines = o3d.utility.Vector2iVector(np.array([[0, 1]], dtype=np.int32))
            self.range_ray.colors = o3d.utility.Vector3dVector(np.array([[0.2, 1.0, 0.2]]))

        self.vis.update_geometry(self.map_cloud)
        self.vis.update_geometry(self.trajectory)
        self.vis.update_geometry(self.frustum)
        self.vis.update_geometry(self.range_ray)

    def _handle_video_key(self, key: int):
        if key == ord('q'):
            self.quit = True
        elif key == 32:
            self.toggle_play()
        elif key in (ord('a'), 81):
            self.seek_seconds(-1.0)
        elif key in (ord('d'), 83):
            self.seek_seconds(1.0)
        elif key == ord(','):
            self.seek_frames(-1)
        elif key == ord('.'):
            self.seek_frames(1)
        elif key == ord('j'):
            self.seek_seconds(-5.0)
        elif key == ord('l'):
            self.seek_seconds(5.0)
        elif key == ord('r'):
            self.playhead = 0.0
            self.playing = False
            self.last_rendered_frame = -1
        elif key == ord('f'):
            self.full_map = not self.full_map
            self.last_rendered_frame = -1

    def run(self):
        print("\nINTERACTIVE REPLAY")
        print("Move freely around the 3D window with the mouse while playback runs or is paused.")
        print("SPACE play/pause | A/D +/-1s | J/L +/-5s | ,/. frame | F final map | R restart | Q quit")
        if self.metric_scale is not None:
            print("3D coordinates are metres. White scale bar = exactly 1 metre.")
            print("Green line = current valid <=1 m forward ToF reading.")
        else:
            print("No reliable metric scale was found; 3D coordinates remain arbitrary units.")

        try:
            while not self.quit:
                now = time.perf_counter()
                dt = min(now - self.last_tick, 0.25)
                self.last_tick = now

                if self.user_seek_frame is not None:
                    self.playhead = float(np.clip(
                        self.user_seek_frame, 0, max(self.total_frames - 1, 0)
                    ))
                    self.user_seek_frame = None
                    self.last_rendered_frame = -1

                if self.playing:
                    self.playhead += dt * self.fps
                    if self.playhead >= self.total_frames - 1:
                        self.playhead = float(max(self.total_frames - 1, 0))
                        self.playing = False

                frame_index = int(np.clip(round(self.playhead), 0, max(self.total_frames - 1, 0)))

                if frame_index != self.last_rendered_frame:
                    frame = self.video.read(frame_index)
                    t = frame_index / self.fps
                    if frame_index < len(self.frame_rows):
                        try:
                            t = float(self.frame_rows[frame_index].get("t_seconds", t))
                        except (TypeError, ValueError):
                            pass

                    pose, pose_index = interpolated_pose(self.poses, t)
                    row = self._sensor_row(frame_index)
                    self._update_3d(pose, pose_index, row)

                    display = draw_replay_overlay(
                        frame, row, pose, pose_index, len(self.poses),
                        self.playing, frame_index, self.total_frames,
                        self.metric_scale, self.full_map,
                    )
                    cv2.imshow("Recorded Camera Replay", display)

                    self.trackbar_internal = True
                    cv2.setTrackbarPos("Timeline", "Recorded Camera Replay", frame_index)
                    self.trackbar_internal = False
                    self.last_rendered_frame = frame_index

                if not self.vis.poll_events():
                    break
                self.vis.update_renderer()

                key = cv2.waitKey(1) & 0xFF
                if key != 255:
                    self._handle_video_key(key)

                time.sleep(0.002)

        finally:
            self.video.close()
            try:
                cv2.destroyWindow("Recorded Camera Replay")
            except cv2.error:
                pass
            self.vis.destroy_window()


def replay_session(sample_fps: float | None = None, source_video_fps: float | None = None) -> None:
    if sample_fps is None or source_video_fps is None:
        spec = importlib.util.spec_from_file_location(
            "_slam_video_to_3d_replay", VIDEO_TO_3D_PATH
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

    poses, xyz, rgb, first_seen, metric_scale, frame_rows = make_replay_data(
        float(sample_fps), float(source_video_fps)
    )

    replay = InteractiveReplay(
        poses=poses,
        xyz=xyz,
        rgb=rgb,
        first_seen=first_seen,
        metric_scale=metric_scale,
        frame_rows=frame_rows,
    )
    replay.run()


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
        if math.isfinite(distance) and distance > 1.0:
            range_text = ">1.0 m / unknown"
        else:
            range_text = num(distance, " m")

        cv2.putText(
            display,
            f"Pitch {num(snap.pitch_deg, ' deg')}   Roll {num(snap.roll_deg, ' deg')}   "
            f"Range {range_text}",
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
