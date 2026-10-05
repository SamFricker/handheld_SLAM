import cv2

# Try camera index 0 first.
# If this opens your laptop webcam instead, try 1, 2, 3...
camera = cv2.VideoCapture(1, cv2.CAP_DSHOW)

if not camera.isOpened():
    print("ERROR: Could not open camera.")
    exit()

print("Camera connected.")
print("Press Q to quit.")

while True:
    ret, frame = camera.read()

    if not ret:
        print("ERROR: Could not read frame.")
        break

    cv2.imshow("Polaroid Live Feed", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

camera.release()
cv2.destroyAllWindows()