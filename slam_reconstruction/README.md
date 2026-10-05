# Handheld SLAM / 3D Mapping Prototype

A low-cost handheld mapping prototype that combines monocular vision, orientation sensing and short-range distance measurement to reconstruct a sparse 3D representation of its surroundings and estimate the device's motion through that space.

I built the project as a practical investigation into the ideas behind SLAM, sensor fusion and robotic perception. It also forms part of my engineering work for a Peru travel grant. I am using the prototype as a technical baseline before visits including UTEC and the NASA-UNSA collaboration, where I plan to investigate how more advanced sensing, mapping and autonomous systems approach related problems.

<p align="center">
  <img src="Media/readme/tripod-mount.jpg" width="31%" alt="Handheld tripod-mounted SLAM prototype">
  <img src="Media/readme/standby-mode.jpg" width="31%" alt="Sensor hardware mounted on the handheld device">
  <img src="Media/readme/bottle-scene.jpg" width="31%" alt="Water bottle reconstruction test scene">
</p>

## Aim

The aim was to develop a handheld system that could recognise the 3D space around it and determine where it was relative to that space. In practice this meant trying to combine three different types of information:

- a standard camera for visual features and camera motion;
- orientation data from the Atech.dev development board;
- a short-range distance sensor to provide a real-world distance reference.

The current version is a proof of concept rather than a complete production SLAM implementation. It can recover a sparse 3D map, estimate camera trajectory, display orientation and viewing direction, and use distance measurements to help represent the reconstruction at a meaningful scale.

## Hardware

- Atech.dev development kit
- Polaroid iE090 digital camera
- power bank
- Elegoo Uno
- hobby ultrasonic / distance sensor
- LED lighting
- tripod / selfie-stick mounting hardware

The final physical layout keeps the camera, distance sensor and orientation sensor close together on a handheld tripod. The Atech board is powered from a portable charger and streams information wirelessly, which solved the initial problem of needing more USB connections than were available on the laptop.

<p align="center">
  <img src="Media/readme/Atechsetup.jpg" width="45%" alt="Atech board setup">
  <img src="Media/readme/camera_working.jpg" width="45%" alt="Camera and sensor development setup">
</p>

## Software

The project is written mainly in Python. The reconstruction experiments use OpenCV, COLMAP and Open3D, with ROS 2 investigated as a possible route for bringing the sensors into a more conventional robotics stack.

The main programs are:

- `polaroid_live.py` - live camera view
- `polaroid_recorder.py` - camera recording
- `atech_orientation_viewer.py` - local orientation visualisation
- `atech_orientation_viewer_wifi.py` - wireless orientation visualisation
- `video_to_3d.py` - video-to-point-cloud reconstruction experiment
- `clean_model.py` - experiments with cleaning the reconstruction
- `slam_capture_replay.py` - combined capture and replay tool
- `slam_capture_replay_v2.py` - later version of the combined tool

## Development

### Camera feed

The first step was getting a stable live feed from the Polaroid camera and recording video that could be processed separately.

<p align="center">
  <img src="Media/readme/camera_feed.jpg" width="70%" alt="Live camera feed">
</p>

### Wireless orientation tracking

I then built a program to visualise the Atech board's orientation in 3D. It was first tested over a direct connection and then changed to work over Wi-Fi so that the handheld device could run from a portable power bank.

<p align="center">
  <img src="Media/readme/first-orientation-viewer.jpg" width="45%" alt="Early orientation viewer">
  <img src="Media/readme/OrientationProgramme.jpg" width="45%" alt="3D orientation visualisation">
</p>

### Monocular 3D reconstruction

A short test video was sampled into 62 frames. COLMAP was used for SIFT feature detection and sequential feature matching, followed by camera-pose estimation and triangulation. It registered 61 of the 62 frames and produced a recognisable coloured sparse point cloud, which was then exported and viewed with Open3D.

Example command:

```powershell
.\.venv\Scripts\python.exe video_to_3d.py ".\Media\test.mov"
```

<p align="center">
  <img src="Media/readme/bottle-scan.jpg" width="46%" alt="Sparse bottle reconstruction">
  <img src="Media/readme/complete-mapping.jpg" width="46%" alt="Mapping visualisation">
</p>

### Combined capture and replay

The next stage was to bring the camera, orientation and distance data into one program that could record a controlled capture and replay it in a 3D virtual environment.

Capture:

```powershell
.\.venv\Scripts\python.exe slam_capture_replay.py
```

Replay:

```powershell
.\.venv\Scripts\python.exe slam_capture_replay.py --replay
```

The program includes pause and rewind controls, a visualisation of the partially reconstructed map, device trajectory and view direction, and distance/scale information.

## Video demonstrations

<!-- YOUTUBE_DEMOS_START -->
The publication script replaces this section with clickable YouTube thumbnails after the videos have been uploaded.
<!-- YOUTUBE_DEMOS_END -->

## Results

The prototype can currently:

- reconstruct a sparse coloured 3D point cloud from camera motion;
- estimate camera movement through the reconstructed scene;
- display the handheld unit's orientation and viewing direction;
- record camera, orientation and distance information over the same capture;
- use short-range distance measurements as a real-world scale reference;
- replay the capture inside a 3D visualisation.

A simple water-bottle scene was used as an early reconstruction test and produced a recognisable 3D point cloud.

<p align="center">
  <img src="Media/readme/bottle-scene.jpg" width="45%" alt="Water bottle test scene">
  <img src="Media/readme/bottle-scan.jpg" width="45%" alt="Water bottle point-cloud result">
</p>

## Limitations

The largest limitation is depth sensing. The current camera estimates the third dimension from parallax as the device moves, so useful reconstruction depends on motion and visible image features. The resulting map is also sparse rather than a continuous surface.

The short-range distance sensor has a narrow field of view, and the IMU/orientation information is recorded and visualised but is not yet fully fused into the visual pose estimate. The current reconstruction pipeline is also not yet real time.

## Next steps

The main software goal is to make the mapping and pose-estimation pipeline operate in real time. That would make the system more suitable for an autonomous platform such as a rover rather than only for capture and later replay.

Further improvements would include tighter visual/IMU/range sensor fusion, loop closure and drift correction, dense reconstruction, surface or mesh generation from the point cloud, and obstacle-detection/navigation logic.

The most significant hardware improvement would be LiDAR. Unlike the present monocular approach, which infers depth from changes between camera frames, LiDAR would provide direct geometric range measurements and should produce much stronger environmental mapping with less device motion.

## Peru engineering research

This project was developed alongside my Peru engineering travel-grant work. Building the system gives me direct experience of the practical problems involved in mapping, localisation, sensing and synchronising low-cost hardware before visiting engineering and research organisations in Peru.

The planned follow-up includes UTEC and the NASA-UNSA collaboration. I want to use those visits to investigate how more advanced robotics and field systems handle sensing, mapping, autonomy and operation in difficult environments, then compare those approaches with the limitations and design choices encountered here.

The aim is not to suggest that this prototype is equivalent to those systems, but to use a working project to make the investigation more technical and informed.

## Repository structure

```text
handheld_SLAM/
├── atech-wireless-orientation/
├── Media/
│   └── readme/                  # generated GitHub images / video thumbnails
├── slam_reconstruction/
├── atech_orientation_viewer.py
├── atech_orientation_viewer_wifi.py
├── clean_model.py
├── polaroid_live.py
├── polaroid_recorder.py
├── slam_capture_replay.py
├── slam_capture_replay_v2.py
├── video_to_3d.py
├── youtube_publish.py
├── publish.ps1
├── LICENSE
└── README.md
```

The raw large videos remain on the development machine and are published to YouTube rather than stored in the Git repository.

## Status

Proof of concept: working. The project demonstrates low-cost 3D reconstruction and localisation concepts and provides a base for future work on real-time mapping and autonomous navigation.
