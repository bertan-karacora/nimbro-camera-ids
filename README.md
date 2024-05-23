# nimbro_camera_ids

## Links

- [IDS peak manual](https://en.ids-imaging.com/download-details/1009698.html?os=linux&version=&bus=64)
- [Application notes for U3-36P1XLS Rev.1.2](https://www.1stvision.com/cameras/IDS/IDS-manuals/en/application-notes-u3-36px.html)
- [Product website for U3-36P1XLS Rev.1.2](https://en.ids-imaging.com/store/u3-36p1xls-rev-1-2.html)
- [Launcher](https://github.com/bertan-karacora/nimbro-ids-launch)

## Installation

1. Install [ROS 2](https://docs.ros.org/en/humble/index.html), [IDS Peak](https://en.ids-imaging.com/files/downloads/ids-peak/readme/ids-peak-linux-readme-2.9.0_EN.html#first-start) and dependencies or use the [launcher](https://github.com/bertan-karacora/nimbro-ids-launch).

2. Clone this Python package into your ROS 2 workspace, e.g., `/colcon_ws/src/nimbro_camera_ids`.

3. Run:

    ```bash
    source /opt/ros/humble/setup.bash
    colcon build
    ```

## Usage

```bash
source /opt/ros/humble/setup.bash
source /colcon_ws/install/setup.bash
ros2 run nimbro_camera_ids spin
```

<!-- TODO: Camera config (in ids peak cockpit) -->
<!-- TODO: Compare with IDS peak cockpit, somehow worse? (Luis D.) -->
<!-- TODO: Camera lense -->
<!-- TODO: timestamps -->
<!-- TODO: Use Launch instead of run -->
<!-- TODO: Reattach reaction stuff -->

<!-- TODO: Node for registration of Lidar and camera -->
<!-- TODO: Problem: Thread takes up up to 60% of a cpu core. If a notebook is running, framerate drops as the thread is not keeping up with the camera acquisition -->

<!-- TODO: Camera calibration (after it is mounted), e.g., https://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration -->
