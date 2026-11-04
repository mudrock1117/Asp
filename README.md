# Asp — ESTUN S3-60 ROS 2 workspace

This repository is intended for the ESTUN S3-60 ROS 2 workspace from the supplied `estun_s3_60` archive and a separately provided `target_detection4.py` vision node.

## Intended source layout

- `estun_s3_60/src/codroid_node/`: robot-control and motion-planning scripts.
- `estun_s3_60/src/codroid_msgs/`: ROS messages.
- `estun_s3_60/src/s3_60_eco_2024/`: robot URDF/Xacro and STL meshes.
- `estun_s3_60/src/estun_moveit2_config/`: MoveIt 2 configuration.
- `estun_s3_60/src/robotiq/` and `robotiq_interfaces/`: gripper support.
- `estun_s3_60/src/codroid_node/scripts/target_detection4.py`: separate YOLOv5 RGB-D/OBB detection script.

**Upload status:** The uploaded source archive was locally extracted and prepared, but the bulk files have not yet been pushed to this GitHub repository. Download `Asp_source_ready_to_push.zip` from the ChatGPT conversation, extract it, and run `push_to_github.ps1` in PowerShell to complete the import. The script uses GitHub authentication already configured on your Windows machine.

The prepared package contains source code and static meshes, not generated colcon `build/`, `install/`, `log/`, IDE caches, or compiled artifacts. After upload, build with `colcon build` in a properly configured ROS 2 environment.

The detector's YOLOv5 code, `best.pt`, and `my.txt` are **not** in the uploaded inputs. Check hardware-specific camera parameters and ROS topics. Physical robot control must be validated safely before execution.

**Known source issue:** `estun_s3_60/src/codroid_node/scripts/testMotion.py` has a Python syntax error on line 2 in the supplied archive and was preserved unchanged.
