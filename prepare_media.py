from pathlib import Path
import sys

try:
    import cv2
except ImportError:
    print("OpenCV is required. Install it with:")
    print("python -m pip install opencv-python")
    sys.exit(1)

ROOT = Path(__file__).resolve().parent
MEDIA = ROOT / "Media"
OUT = MEDIA / "readme"
OUT.mkdir(parents=True, exist_ok=True)


def find_by_stem(stem):
    if not MEDIA.exists():
        return None

    for path in MEDIA.iterdir():
        if path.is_file() and path.stem.lower() == stem.lower():
            return path

    return None


def save_jpg(src, dst):
    image = cv2.imread(str(src))

    if image is None:
        print(f"Could not read {src.name}")
        return

    cv2.imwrite(
        str(dst),
        image,
        [int(cv2.IMWRITE_JPEG_QUALITY), 90],
    )

    print(f"Prepared {dst.relative_to(ROOT)}")


images = {
    "tripod-mount.jpg": "tripod-mount",
    "standby-mode.jpg": "standby-mode",
    "bottle-scene.jpg": "bottle-scene",
    "orientation-viewer.jpg": "first-orientation-viewer",
    "orientation-programme.jpg": "OrientationProgramme",
    "bottle-scan.jpg": "bottle-scan",
}

for output_name, source_stem in images.items():
    source = find_by_stem(source_stem)

    if source:
        save_jpg(source, OUT / output_name)


video = find_by_stem("complete-mapping")

if not video:
    raise SystemExit(
        "Could not find the complete-mapping video in the Media folder."
    )

cap = cv2.VideoCapture(str(video))
ok, frame = cap.read()
cap.release()

if not ok or frame is None:
    raise SystemExit(
        "Could not read the first frame of complete-mapping."
    )

thumbnail = OUT / "complete-mapping-thumbnail.jpg"

cv2.imwrite(
    str(thumbnail),
    frame,
    [int(cv2.IMWRITE_JPEG_QUALITY), 92],
)

print(f"Created {thumbnail.relative_to(ROOT)}")

complete_map = OUT / "complete-map.jpg"

if not complete_map.exists():
    cv2.imwrite(
        str(complete_map),
        frame,
        [int(cv2.IMWRITE_JPEG_QUALITY), 90],
    )

print("README media prepared.")