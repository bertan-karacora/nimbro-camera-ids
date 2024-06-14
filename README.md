# nimbro_camera_ids

## Links

- [IDS peak manual](https://en.ids-imaging.com/download-details/1009698.html?os=linux&version=&bus=64)
- [IDS peak IPL documentation](https://en.ids-imaging.com/manuals/ids-peak/ids-peak-ipl-documentation/2.9.0/en/index.html)
- [IDS peak AFL documentation](https://en.ids-imaging.com/manuals/ids-peak/ids-peak-afl-documentation/2.9.0/en/index.html)
- [Application notes for U3-36P1XLS Rev.1.2](https://www.1stvision.com/cameras/IDS/IDS-manuals/en/application-notes-u3-36px.html)
- [Product website for U3-36P1XLS Rev.1.2](https://en.ids-imaging.com/store/u3-36p1xls-rev-1-2.html)
- [Product website for Fisheye E2328KRY lense](https://en.ids-imaging.com/store/u3-36p1xls-rev-1-2.html)
- [Launcher](https://git.ais.uni-bonn.de/athome/nimbro_ids_launch)
- [Kalibr (custom)](https://git.ais.uni-bonn.de/athome/kalibr_ds)

## Installation

1. Install [ROS 2](https://docs.ros.org/en/humble/index.html), [IDS Peak](https://en.ids-imaging.com/files/downloads/ids-peak/readme/ids-peak-linux-readme-2.9.0_EN.html#first-start) and dependencies or use Docker via the launcher.

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

## TODO

- Intrinsics calibration
- Extrinsics calibration? Or hard-code using tf tree
- Timestamps: Maybe linescan mode and listen to internal events: https://en.ids-imaging.com/manuals/ids-peak/ids-peak-user-manual/2.8.0/en/event-selector.html?q=events Or estimate from outside: Subtract delay between image acquisition and message creation and calibrate. Rolling shutter would still be a problem.
- Use launchfile instead of ros2 run
- Hot pixel correction
- IPL library config (number of threads and stuff)
- Gamma correction?
- Publish compressed image via image-transport?
- Color calibration? Possibly color conversion and correction in parallel on a GPU?
- Reattach reaction stuff? Instead of watchdog in launcher?
- Benchmarking

## Calibration

### Intrinsics

1. Get [calibration target](https://github.com/ethz-asl/kalibr/wiki/calibration-targets)
2. Record ros2 bag
3. Convert using [rosbags](https://gitlab.com/ternaris/rosbags)
4. Use [Kalibr](https://git.ais.uni-bonn.de/athome/kalibr_ds)
5. Save result in [intrinsics.yaml](nimbro_camera_ids/resources/intrinsics.yaml)
