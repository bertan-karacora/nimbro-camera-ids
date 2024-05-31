# nimbro_camera_ids

## Links

- [IDS peak manual](https://en.ids-imaging.com/download-details/1009698.html?os=linux&version=&bus=64)
- [IDS peak IPL documentation](https://en.ids-imaging.com/manuals/ids-peak/ids-peak-ipl-documentation/2.9.0/en/index.html)
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

<!-- TODO: Camera lense -->
<!-- TODO: Color correction -->
<!-- TODO: Config package -->
<!-- TODO: White balance -->
<!-- TODO: Calculate focal length (like https://en.ids-imaging.com/lensfinder.html) -->
<!-- TODO: Change to gitlab -->

<!-- TODO: timestamps: Maybe linescan mode and listen to internal events: https://en.ids-imaging.com/manuals/ids-peak/ids-peak-user-manual/2.8.0/en/event-selector.html?q=events Or estimate from outside: Subtract delay between image acquisition and message creation and calibrate. Rolling shutter still a problem
<!-- TODO: Auto features -->
<!-- TODO: Color correction -->
<!-- TODO: Hot pixels -->
<!-- TODO: Transformations -->
<!-- TODO: Gamma correction -->
<!-- TODO: IPL library config (number of threads and stuff) -->
<!-- TODO: Possibly color conversion and correction in parallel on a GPU? -->

<!-- TODO: Camera config (in ids peak cockpit) -->
<!-- TODO: Use Launch instead of run -->
<!-- TODO: Reattach reaction stuff -->
<!-- TODO: Node for registration of Lidar and camera -->
<!-- TODO: Problem: Thread takes up up to 60% of a cpu core. If a notebook is running, framerate drops as the thread is not keeping up with the camera acquisition -->
<!-- TODO: Observation: Even at full CPU usage, framerate does not drop. -> Problem is network related? -->
<!-- TODO: Observation:Artifacts from infrared -->

<!-- TODO: Camera calibration (after it is mounted) -->
<!-- TODO: Gain, gamma, and lighting when calibrating -->
<!-- TODO: How to adjust focal lense exactly when attaching lense -->
