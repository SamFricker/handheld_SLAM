from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Optional

import cv2
from PIL import Image
from google.auth.transport.requests import Request
from google.oauth2.credentials import Credentials
from google_auth_oauthlib.flow import InstalledAppFlow
from googleapiclient.discovery import build
from googleapiclient.errors import HttpError
from googleapiclient.http import MediaFileUpload

ROOT = Path(__file__).resolve().parent
MEDIA = ROOT / "Media"
README_MEDIA = MEDIA / "readme"
TOKEN_FILE = ROOT / ".youtube_token.json"
SCOPES = ["https://www.googleapis.com/auth/youtube.upload"]

VIDEOS = [
    {
        "path": MEDIA / "wireless-orientation-tracking.mov",
        "title": "Handheld SLAM - Wireless Orientation Tracking",
        "description": "Wireless orientation-tracking test from my low-cost handheld SLAM / 3D mapping prototype.",
    },
    {
        "path": MEDIA / "complete-mapping.mp4",
        "title": "Handheld SLAM - Complete Mapping Demo",
        "description": "Combined mapping and localisation demonstration from my low-cost handheld SLAM / 3D mapping prototype.",
    },
    {
        "path": MEDIA / "test.mov",
        "title": "Handheld SLAM - Monocular Reconstruction Test",
        "description": "Source reconstruction test used while developing the monocular 3D mapping pipeline for my handheld SLAM prototype.",
    },
]

STILL_MEDIA = [
    "tripod-mount",
    "standby-mode",
    "bottle-scene",
    "Atechsetup",
    "camera_working",
    "camera_feed",
    "first-orientation-viewer",
    "OrientationProgramme",
    "bottle-scan",
]


def find_client_secret() -> Path:
    candidates = sorted(ROOT.glob("client_secret*.json")) + sorted(ROOT.glob("client_secrets*.json"))
    if not candidates:
        raise SystemExit(
            "No YouTube OAuth client-secret JSON found in the repo root.\n"
            "Download a Desktop-app OAuth client JSON from Google Cloud after enabling YouTube Data API v3, "
            "then place it next to publish.ps1 and run the command again."
        )
    return candidates[0]


def youtube_service():
    creds: Optional[Credentials] = None
    if TOKEN_FILE.exists():
        creds = Credentials.from_authorized_user_file(str(TOKEN_FILE), SCOPES)
    if creds and creds.expired and creds.refresh_token:
        creds.refresh(Request())
    if not creds or not creds.valid:
        flow = InstalledAppFlow.from_client_secrets_file(str(find_client_secret()), SCOPES)
        creds = flow.run_local_server(port=0, open_browser=True)
        TOKEN_FILE.write_text(creds.to_json(), encoding="utf-8")
    return build("youtube", "v3", credentials=creds)


def extract_first_frame(video: Path, output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    cap = cv2.VideoCapture(str(video))
    ok, frame = cap.read()
    cap.release()
    if not ok or frame is None:
        raise RuntimeError(f"Could not read the first frame of {video.name}")
    if not cv2.imwrite(str(output), frame, [int(cv2.IMWRITE_JPEG_QUALITY), 92]):
        raise RuntimeError(f"Could not write thumbnail {output}")


def normalize_still(stem: str) -> None:
    README_MEDIA.mkdir(parents=True, exist_ok=True)
    matches = [p for p in MEDIA.iterdir() if p.is_file() and p.stem.lower() == stem.lower()]
    if not matches:
        print(f"warning: no still image found for {stem}")
        return
    source = matches[0]
    target = README_MEDIA / f"{stem}.jpg"
    try:
        with Image.open(source) as im:
            im.convert("RGB").save(target, "JPEG", quality=92)
    except Exception as exc:
        print(f"warning: could not prepare {source.name}: {exc}")


def upload_video(youtube, item: dict, thumbnail: Path) -> str:
    path: Path = item["path"]
    if not path.exists():
        raise FileNotFoundError(path)

    print(f"Uploading {path.name} ...")
    request = youtube.videos().insert(
        part="snippet,status",
        body={
            "snippet": {
                "title": item["title"],
                "description": item["description"],
                "categoryId": "28",
                "tags": ["SLAM", "robotics", "3D mapping", "computer vision", "engineering"],
            },
            "status": {"privacyStatus": "unlisted"},
        },
        media_body=MediaFileUpload(str(path), chunksize=8 * 1024 * 1024, resumable=True),
    )

    response = None
    while response is None:
        status, response = request.next_chunk()
        if status:
            print(f"  {int(status.progress() * 100)}%")

    video_id = response["id"]
    url = f"https://youtu.be/{video_id}"
    print(f"Uploaded: {url}")

    try:
        youtube.thumbnails().set(
            videoId=video_id,
            media_body=MediaFileUpload(str(thumbnail), mimetype="image/jpeg"),
        ).execute()
        print("  custom thumbnail set from first frame")
    except HttpError as exc:
        print(f"  warning: YouTube would not accept the custom thumbnail: {exc}")

    return url


def replace_demo_section(results: list[dict]) -> None:
    readme = ROOT / "README.md"
    text = readme.read_text(encoding="utf-8")
    start = "<!-- YOUTUBE_DEMOS_START -->"
    end = "<!-- YOUTUBE_DEMOS_END -->"
    if start not in text or end not in text:
        raise RuntimeError("README YouTube marker block is missing")

    cards = []
    for result in results:
        thumb_rel = result["thumbnail"].relative_to(ROOT).as_posix()
        cards.append(
            f'<p><a href="{result["url"]}"><img src="{thumb_rel}" width="720" '
            f'alt="{result["title"]} - click to watch on YouTube"></a><br>'
            f'<a href="{result["url"]}">{result["title"]}</a></p>'
        )

    block = start + "\n" + "\n".join(cards) + "\n" + end
    before = text.split(start, 1)[0]
    after = text.split(end, 1)[1]
    readme.write_text(before + block + after, encoding="utf-8")


def main() -> None:
    README_MEDIA.mkdir(parents=True, exist_ok=True)

    for stem in STILL_MEDIA:
        normalize_still(stem)

    # Use the first frame of complete-mapping as the static screenshot elsewhere in README too.
    complete_mapping = MEDIA / "complete-mapping.mp4"
    if complete_mapping.exists():
        extract_first_frame(complete_mapping, README_MEDIA / "complete-mapping.jpg")

    youtube = youtube_service()
    results = []

    for item in VIDEOS:
        video: Path = item["path"]
        thumb = README_MEDIA / f"youtube-{video.stem}.jpg"
        extract_first_frame(video, thumb)
        url = upload_video(youtube, item, thumb)
        results.append({"title": item["title"], "url": url, "thumbnail": thumb})

    replace_demo_section(results)
    (ROOT / "youtube_links.json").write_text(
        json.dumps([{k: str(v) for k, v in r.items()} for r in results], indent=2),
        encoding="utf-8",
    )
    print("README updated with clickable YouTube thumbnails.")


if __name__ == "__main__":
    main()
