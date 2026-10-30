# ats_sentry_nav

Navigation repository for the ATS four-wheel independent-steering sentry robot (ROS 2 Humble):
LiDAR-inertial localization, ROG-Map based mapping, JPS/MINCO planning and omnidirectional SE(2) MPC control.
Goals enter through the `/ats_navigate_to_pose` action; Nav2 is not used.

The maintained documentation is the Chinese [README.md](./README.md): package responsibilities, data flow,
build commands, launch entries and parameters. This page only points to it, so the two cannot drift apart.

Use `ros2 launch <package> <file> --show-args` for the authoritative launch arguments.
Licenses are declared per package (`package.xml`, `LICENSE`, `NOTICE`).
