"""
Build a sparse 3D reconstruction from a specified video.

Usage from the project folder:
    .\\.venv\\Scripts\\python.exe video_to_3d.py ".\\recording\\recording.mp4"
    .\\.venv\\Scripts\\python.exe video_to_3d.py ".\\Media\\test.mov"

The script extracts frames from the supplied video, runs CPU-only COLMAP
feature extraction, sequential matching, sparse mapping, converts the model
to a PLY point cloud, and opens it in Open3D.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

import cv2

PROJECT_DIR = Path(__file__).resolve().parent
COLMAP_BAT = Path(r"C:\COLMAP\COLMAP.bat")
RECON_DIR = PROJECT_DIR / "slam_reconstruction"
IMAGE_DIR = RECON_DIR / "images"
DATABASE_PATH = RECON_DIR / "database.db"
SPARSE_DIR = RECON_DIR / "sparse"
PLY_PATH = RECON_DIR / "sparse_model.ply"

SAMPLE_FPS = 3.0
MAX_FRAMES = 200
MAX_IMAGE_SIZE = 1600
FEATURE_THREADS = 2
MATCH_THREADS = 2
SEQUENTIAL_OVERLAP = 4


def run_colmap(command: str, *args: str) -> None:
    cmd = ["cmd", "/c", str(COLMAP_BAT), command, *args]
    print("\n$ " + " ".join(cmd) + "\n")
    result = subprocess.run(cmd, check=False)
    if result.returncode != 0:
        raise RuntimeError(
            f"COLMAP command failed with exit code {result.returncode}"
        )


def reset_workspace() -> None:
    if RECON_DIR.exists():
        print(f"Removing previous reconstruction: {RECON_DIR}")
        shutil.rmtree(RECON_DIR)
    IMAGE_DIR.mkdir(parents=True, exist_ok=True)
    SPARSE_DIR.mkdir(parents=True, exist_ok=True)


def extract_frames(video_path: Path) -> int:
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise RuntimeError(f"Could not open video: {video_path}")

    fps = float(cap.get(cv2.CAP_PROP_FPS))
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if fps <= 0:
        cap.release()
        raise RuntimeError("Could not determine video FPS.")

    step = max(1, round(fps / SAMPLE_FPS))
    print(f"Video FPS: {fps:.2f}; sampling approximately {fps / step:.2f} FPS")

    saved = 0
    frame_index = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break

        if frame_index % step == 0:
            h, w = frame.shape[:2]
            largest = max(h, w)
            if largest > MAX_IMAGE_SIZE:
                scale = MAX_IMAGE_SIZE / largest
                frame = cv2.resize(
                    frame,
                    (int(round(w * scale)), int(round(h * scale))),
                    interpolation=cv2.INTER_AREA,
                )

            out = IMAGE_DIR / f"frame_{saved:05d}.jpg"
            if not cv2.imwrite(
                str(out), frame, [int(cv2.IMWRITE_JPEG_QUALITY), 92]
            ):
                cap.release()
                raise RuntimeError(f"Could not write frame: {out}")
            saved += 1
            if saved >= MAX_FRAMES:
                break

        frame_index += 1

    cap.release()
    if saved < 2:
        raise RuntimeError("Not enough frames extracted for reconstruction.")

    print(f"Extracted {saved} frames to {IMAGE_DIR}")
    return saved


def open_point_cloud() -> None:
    try:
        import open3d as o3d
    except ImportError:
        print("Open3D is not installed in the active .venv; skipping viewer.")
        return

    cloud = o3d.io.read_point_cloud(str(PLY_PATH))
    if cloud.is_empty():
        print(f"PLY was created but contains no points: {PLY_PATH}")
        return

    print(f"Opening {len(cloud.points):,} points in Open3D...")
    o3d.visualization.draw_geometries(
        [cloud],
        window_name="SLAM Project - Sparse 3D Reconstruction",
        width=1100,
        height=750,
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Build a sparse 3D reconstruction from a specified video."
    )
    parser.add_argument("video", help="Path to the video to reconstruct")
    args = parser.parse_args()

    if not COLMAP_BAT.exists():
        print(f"ERROR: COLMAP not found at {COLMAP_BAT}")
        return 1

    video_path = Path(args.video).expanduser().resolve()
    if not video_path.is_file():
        print(f"ERROR: Video not found: {video_path}")
        return 1

    try:
        reset_workspace()
        extract_frames(video_path)

        print("\n" + "=" * 72)
        print("1/3  Feature extraction")
        print("=" * 72)
        run_colmap(
            "feature_extractor",
            "--database_path", str(DATABASE_PATH),
            "--image_path", str(IMAGE_DIR),
            "--ImageReader.single_camera", "1",
            "--FeatureExtraction.use_gpu", "0",
            "--FeatureExtraction.num_threads", str(FEATURE_THREADS),
            "--FeatureExtraction.max_image_size", str(MAX_IMAGE_SIZE),
        )

        print("\n" + "=" * 72)
        print("2/3  Sequential feature matching")
        print("=" * 72)
        run_colmap(
            "sequential_matcher",
            "--database_path", str(DATABASE_PATH),
            "--SequentialMatching.overlap", str(SEQUENTIAL_OVERLAP),
            "--FeatureMatching.use_gpu", "0",
            "--FeatureMatching.num_threads", str(MATCH_THREADS),
        )

        print("\n" + "=" * 72)
        print("3/3  Sparse 3D reconstruction")
        print("=" * 72)
        run_colmap(
            "mapper",
            "--database_path", str(DATABASE_PATH),
            "--image_path", str(IMAGE_DIR),
            "--output_path", str(SPARSE_DIR),
        )

        model_dirs = sorted(p for p in SPARSE_DIR.iterdir() if p.is_dir())
        if not model_dirs:
            raise RuntimeError(
                "Failed to create any sparse model. "
                "Try a video with slower movement, more texture and overlapping views."
            )

        model_dir = model_dirs[0]

        run_colmap(
            "model_converter",
            "--input_path", str(model_dir),
            "--output_path", str(PLY_PATH),
            "--output_type", "PLY",
        )

        print("\n" + "=" * 72)
        print("RECONSTRUCTION COMPLETE")
        print("=" * 72)
        print(f"Input video:   {video_path}")
        print(f"Images:        {IMAGE_DIR}")
        print(f"Sparse model:  {model_dir}")
        print(f"Point cloud:   {PLY_PATH}")
        print("Scale: arbitrary global scale (monocular reconstruction).")

        open_point_cloud()
        return 0

    except (RuntimeError, OSError) as exc:
        print(f"\nERROR: {exc}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
