# Project Proposal — AI Club Project No.6 (2025–2026)

## Project Presentation: Autonomous Robotic Arm

### 1. Project Context

In a world where automation and artificial intelligence are transforming industries, manipulation robotics plays a central role. From assembly lines to logistics warehouses and surgical assistance, robotic arms capable of interacting intelligently with their environment have become indispensable.

Mastering the concepts that power these systems is a valuable skill. This project provides a hands-on immersion at the intersection of several core engineering disciplines: computer vision, robotic kinematics, artificial intelligence, and programming. Using the HiWonder xArm robotic arm, the project aims to reproduce an industrial automation task: pick-and-place.

### 2. Problem Statement

How can we program a robotic arm so that it can, fully autonomously, identify the position of an object within its workspace, plan a trajectory to reach it, grasp it precisely, and place it at a predefined location?

This primary question breaks down into several technical challenges:

- **Perception:** How to convert visual information from a 2D or 3D camera into spatial coordinates the robot can use?
- **Decision:** How to choose the correct object and determine an optimal, collision-free sequence of movements to reach it?
- **Action:** How to compute precise joint angles for each arm joint (inverse kinematics) so the gripper reaches the target with the correct orientation?
- **Integration:** How to make the vision module (the “brain”) and the motor control module (the “muscles”) communicate smoothly?

### 3. Expected Results

By the end of the project the club will have developed a working system capable of:

1. Detecting a specific object in the camera field of view in real-time, based on features such as color or shape.
2. Computing the 3D position of the detected object relative to the robot base.
3. Planning and executing a complete motion trajectory to grasp the object.
4. Successfully placing the object in a predefined target area.
5. Providing a simple visualization interface showing the camera feed with the detected object and robot status information.

The final result will be a tangible and impressive demonstration of the skills acquired by club members.

### 4. Hardware

| Component | Ideal | Alternatives / Options | Role |
|---|---|---|---|
| Robot Arm | HiWonder xArm | DOFBOT Yahboom | Main actuator of the project |
| Camera | USB webcam | DOFBOT's mounted camera | Robot’s “eyes” |
| Controller | HiWonder Core X controller | Dedicated laptop with Python and required libraries / DOFBOT's Raspberry Pi 4 (or Jetson Nano) | Compute unit running vision and control algorithms |
| Target Object | A small plastic cube with different colors | Any object easily distinguishable from the background | The object to be manipulated |
| Workspace | Flat, well-lit surface with a contrasting background | N/A | Robot operating environment |

### 5. Software Stack

- **Programming Language:** Python 3.8+
- **Key Python Libraries:**
  - **ultralytics** (YOLOv8 framework)
  - **opencv-python** (image capture, preprocessing, visualization)
  - **numpy** (math and matrix operations for kinematics)

### 6. Methods and Applied Technologies

1. **Computer Vision with YOLO and OpenCV**
   - Acquisition (OpenCV): capture the video stream from the camera.
   - Inference (YOLO): run each frame through a pretrained (or custom) YOLO model to obtain bounding boxes for detected objects.
   - Extraction (OpenCV): compute the center of the bounding box to determine the object’s pixel coordinates.

2. **Robotic Kinematics**
   - Camera–robot calibration: establish a mathematical transformation to map image coordinates to the robot’s world coordinates (a critical step).
   - Inverse kinematics: develop a solver that computes joint angles (θ1, …, θ6) for a given (x, y, z) position.

3. **Programming and Integration (Python)**
   - Robot control: use the xarm-sdk to send commands to the arm.
   - Orchestration: create a main script organizing the logic: Detect (YOLO) → Compute 3D position → Plan (inverse kinematics) → Act (xArm SDK).

---

For full details and ongoing updates, see the team READMEs under `computer-vision/` and `control/`. Feedback and contributions are welcome via issues and pull requests.
