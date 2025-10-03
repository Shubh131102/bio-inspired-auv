# Bio-Inspired AUV — ROS2 Control, IMU Fusion, and Camera–LiDAR Obstacle Avoidance

**TL;DR:** A fin-driven AUV with PID control and multi-sensor fusion (IMU + vision + LiDAR). Containerized ROS2 stack for repeatable bring-up and deployment.

**Highlights**
- **Stability:** Improved yaw/pitch/roll stability with tuned **PID** and IMU fusion (EKF)
- **Perception:** Camera–LiDAR fusion for obstacle avoidance
- **Deployment:** Dockerized nodes for reproducibility on embedded compute
- **Result (project):** ~**30% better trajectory stability** and improved energy efficiency

## Repo Layout
- `src/` — ROS2 packages (control, fusion, perception)
- `launch/` — launch files (sim vs hardware)
- `config/` — PID gains, EKF params, topics
- `scripts/` — calibration, data logging, analysis
- `docker/` — Dockerfile & compose
- `sim/` — simulation worlds/models (optional)
- `media/` — demos (GIF/MP4)
- `docs/` — design notes / report

## Quick Start (dev)
```bash
# Python deps not coming from ROS
python -m venv .venv
# Windows:
.\.venv\Scripts\activate
pip install -r requirements.txt

# ROS2 workspace (example)
# colcon build
# source install/setup.bash
# ros2 launch auv_bringup sim.launch.py
