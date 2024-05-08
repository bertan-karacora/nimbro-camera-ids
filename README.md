# nimbro-ids-ros2

## Links

- [IDS peak manual](https://en.ids-imaging.com/download-details/1009698.html?os=linux&version=&bus=64)
- [Application notes for U3-36P1XLS Rev.1.2](https://www.1stvision.com/cameras/IDS/IDS-manuals/en/application-notes-u3-36px.html)
- [Product website for U3-36P1XLS Rev.1.2](https://en.ids-imaging.com/store/u3-36p1xls-rev-1-2.html)
- [Launcher](https://github.com/bertan-karacora/nimbro-ids-launch)

## Installation

1. Install [ROS 2](https://docs.ros.org/en/humble/index.html), [IDS Peak](https://en.ids-imaging.com/files/downloads/ids-peak/readme/ids-peak-linux-readme-2.9.0_EN.html#first-start) and dependencies or use the [launcher](https://github.com/bertan-karacora/nimbro-ids-launch).

2. Clone this Python package into your ROS 2 workspace, e.g., `/ros2_ws/src/nimbro-ids-ros2`.

3. Run:

    ```bash
    source /opt/ros/humble/setup.bash
    colcon build
    ```

## Usage

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ros2 run camera_ids publish --config default
```

<!-- TODO: Camera config -->
<!-- TODO: timestamps -->
<!-- TODO: Use Launch instead of run -->
<!-- TODO: Reattach reaction stuff-->

<!-- 2024-05-08 -->
<!-- TODO: Camera message fillen-->
<!-- TODO: Camera info topic-->
<!-- TODO: Node for registration of Lidar and camera-->
