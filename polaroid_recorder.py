import cv2
import os
import time
from datetime import datetime

WINDOW_TITLE = 'Polaroid Camera Recorder'
CAMERA_INDEX = 1
OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'recording')
FRAMES_DIR = os.path.join(OUTPUT_DIR, 'frames')
OUTPUT_VIDEO = os.path.join(OUTPUT_DIR, 'recording.mp4')

os.makedirs(OUTPUT_DIR, exist_ok=True)

recording = False
writer = None
frame_count = 0
start_time = None
last_frame = None


def clean_previous_recording():
    global frame_count
    frame_count = 0
    os.makedirs(FRAMES_DIR, exist_ok=True)
    for name in os.listdir(FRAMES_DIR):
        path = os.path.join(FRAMES_DIR, name)
        if os.path.isfile(path):
            try:
                os.remove(path)
            except OSError:
                pass
    if os.path.exists(OUTPUT_VIDEO):
        try:
            os.remove(OUTPUT_VIDEO)
        except OSError:
            pass


def start_recording(frame):
    global recording, writer, frame_count, start_time
    clean_previous_recording()

    h, w = frame.shape[:2]
    fps = cap.get(cv2.CAP_PROP_FPS)
    if not fps or fps < 1 or fps > 120:
        fps = 30.0

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(OUTPUT_VIDEO, fourcc, fps, (w, h))
    if not writer.isOpened():
        writer = None
        print('ERROR: Could not create video file.')
        return

    recording = True
    start_time = time.time()
    print(f'Recording started -> {OUTPUT_VIDEO}')


def stop_recording():
    global recording, writer, start_time
    if writer is not None:
        writer.release()
        writer = None
    recording = False
    start_time = None
    print(f'Recording stopped. Frames captured: {frame_count}')
    print(f'Video saved to: {OUTPUT_VIDEO}')


def mouse_callback(event, x, y, flags, param):
    if event != cv2.EVENT_LBUTTONDOWN:
        return

    if start_button[0] <= x <= start_button[2] and start_button[1] <= y <= start_button[3]:
        if not recording and last_frame is not None:
            start_recording(last_frame)

    elif stop_button[0] <= x <= stop_button[2] and stop_button[1] <= y <= stop_button[3]:
        if recording:
            stop_recording()


cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
if not cap.isOpened():
    raise SystemExit(f'ERROR: Could not open camera #{CAMERA_INDEX}.')

# Try to keep the feed responsive.
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WINDOW_TITLE, 1100, 760)

start_button = (30, 680, 220, 735)
stop_button = (240, 680, 430, 735)
cv2.setMouseCallback(WINDOW_TITLE, mouse_callback)

print('Polaroid camera connected.')
print('Click RECORD to start, STOP to finish, or press Q to quit.')
print(f'Current recording will overwrite: {OUTPUT_VIDEO}')

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print('ERROR: Could not read frame from camera.')
            break

        last_frame = frame.copy()

        if recording and writer is not None:
            writer.write(frame)
            frame_count += 1

            # Also save individual frames. These are overwritten on the next recording.
            frame_name = os.path.join(FRAMES_DIR, f'frame_{frame_count:06d}.jpg')
            cv2.imwrite(frame_name, frame, [cv2.IMWRITE_JPEG_QUALITY, 92])

        display = frame.copy()
        h, w = display.shape[:2]

        # Buttons are anchored to the bottom of the current window/frame.
        by1, by2 = h - 70, h - 15
        sx1, sx2 = 30, 220
        tx1, tx2 = 240, 430

        status = 'RECORDING' if recording else 'READY'
        cv2.rectangle(display, (20, 15), (230, 55), (0, 0, 0), -1)
        cv2.putText(display, status, (30, 43), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 0, 255) if recording else (0, 255, 0), 2, cv2.LINE_AA)

        if recording and start_time is not None:
            elapsed = time.time() - start_time
            cv2.putText(display, f'Time: {elapsed:5.1f}s   Frames: {frame_count}',
                        (250, 43), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)

        # Draw UI buttons.
        cv2.rectangle(display, (sx1, by1), (sx2, by2), (40, 170, 40) if not recording else (80, 80, 80), -1)
        cv2.rectangle(display, (tx1, by1), (tx2, by2), (40, 40, 200) if recording else (80, 80, 80), -1)
        cv2.putText(display, 'RECORD', (75, by1 + 37), cv2.FONT_HERSHEY_SIMPLEX, 0.85,
                    (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(display, 'STOP', (300, by1 + 37), cv2.FONT_HERSHEY_SIMPLEX, 0.85,
                    (255, 255, 255), 2, cv2.LINE_AA)

        # Update callback hitboxes to current frame geometry.
        start_button = (sx1, by1, sx2, by2)
        stop_button = (tx1, by1, tx2, by2)

        cv2.imshow(WINDOW_TITLE, display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r') and not recording:
            start_recording(last_frame)
        elif key == ord('s') and recording:
            stop_recording()

finally:
    if recording:
        stop_recording()
    cap.release()
    cv2.destroyAllWindows()
