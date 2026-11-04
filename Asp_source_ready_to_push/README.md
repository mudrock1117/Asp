# Asp — ESTUN S3-60 ROS 2 workspace

This repository contains the **source code** extracted from the supplied `estun_s3_60` ROS 2 workspace and the separately provided `target_detection4.py` detection node.

## Layout

- `estun_s3_60/src/codroid_node/`: robot IO, control and motion-planning scripts.
- `estun_s3_60/src/codroid_msgs/`: robot-state ROS interfaces.
- `estun_s3_60/src/s3_60_eco_2024/`: robot URDF, Xacro, meshes and visualization.
- `estun_s3_60/src/estun_moveit2_config/`: MoveIt 2 configuration and launch files.
- `estun_s3_60/src/robotiq/` and `robotiq_interfaces/`: gripper integration.
- `estun_s3_60/src/codroid_node/scripts/target_detection4.py`: supplied YOLOv5 + RGB-D + OBB yaw ROS node.

Only the sources and static assets are committed. Generated `build/`, `install/`, `log/`, caches, IDE metadata and temporary reports were omitted intentionally. Rebuild with `colcon build` in a configured ROS 2 environment.

### Detection runtime dependencies

The detection node requires an external YOLOv5 checkout exposing `models.experimental` and `utils.general`, plus `best.pt`, `my.txt`, Python ROS 2 dependencies and compatible RGB/depth camera topics. These were **not** included in the uploaded source archive and have not been fabricated. Camera intrinsics, depth scale and ROS topic names must be checked against the actual hardware before use.

**Safety:** Physical robot motion scripts and controller configurations are provided as-is; validate in simulation and with safety interlocks before hardware operation. No hardware execution or live integration tests were performed here.
