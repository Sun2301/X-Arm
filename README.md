# X-Arm - Autonomous Robotic Arm with AI

This is a short landing page for the X-Arm project. This repository develops an autonomous pick-and-place system using computer vision (YOLO + OpenCV) and the HiWonder xArm platform.

## Status

Work in progress — AI Club Project (2025–2026).

## Quick Links

- Project proposal and full spec: `docs/PROJECT_PROPOSAL.md`
- Vision module: `computer-vision/` (object detection, camera integration)
- Control module: `control/` (kinematics, trajectory planning, motor control)
- License: `LICENSE`

## Goals

- Real-time object detection
- Compute 3D positions relative to the robot base
- Plan and execute collision-free trajectories
- Grasp and place objects reliably
- Provide a simple visualization of camera feed and robot state

## Quick Start

This repository is split into submodules. Each module has its own README with detailed setup and run instructions:

1. Clone the repo:

   git clone https://github.com/Sun2301/X-Arm.git

2. Open the module you want to run first (e.g., `computer-vision/` or `control/`) and follow that module's README.

3. For project-level details (timeline, hardware, methods), see `docs/PROJECT_PROPOSAL.md`.

## Contributing

Open an issue or submit a pull request.
## Contact

Project maintained by the AI Club of the BENIN Excellence-Fondation Vallet AI Lab. Use GitHub issues for questions or coordination.

---

This README is intended to be a concise entry point; for implementation details see the module READMEs.
