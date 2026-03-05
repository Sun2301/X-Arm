"""
Demo: Use YOLO detection to control the robot simulation
- Detects the largest object in webcam
- Maps its center to robot coordinates
- Moves the simulated arm to the detected position
"""
import sys
import os
import importlib.util
import sys

import cv2
from ultralytics import YOLO
import numpy as np


control_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../control/robot_sim_final.py'))
spec = importlib.util.spec_from_file_location("robot_sim_final", control_path)
robot_sim = importlib.util.module_from_spec(spec)
spec.loader.exec_module(robot_sim)
RobotController = robot_sim.RobotController

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../control')))



# --- Parameters ---
CAM_WIDTH = 640
CAM_HEIGHT = 480
# Robot workspace in cm (adjust to your setup)
ROBOT_X_RANGE = (-20, 20)
ROBOT_Y_RANGE = (-20, 20)
ROBOT_Z_GRASP = 2  # Height for grasping

# --- Mapping function ---
def pixel_to_robot_coords(center_x, center_y, img_width, img_height,
                          x_range=ROBOT_X_RANGE, y_range=ROBOT_Y_RANGE, z_fixed=ROBOT_Z_GRASP):
    norm_x = center_x / img_width
    norm_y = center_y / img_height
    robot_x = x_range[0] + norm_x * (x_range[1] - x_range[0])
    robot_y = y_range[0] + norm_y * (y_range[1] - y_range[0])
    robot_z = z_fixed
    return robot_x, robot_y, robot_z

# --- Main ---
def main():
    # Load YOLO model
    model = YOLO('models/best.pt')
    bot = RobotController()
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)

    print("Press 'q' to quit. The robot will move to the largest detected object.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Camera error.")
            break

        # YOLO detection
        results = model(frame, verbose=False, conf=0.5)
        boxes = results[0].boxes
        if boxes is not None and len(boxes) > 0:
            # Find the largest box (by area)
            areas = [(int(b.xyxy[0][2]) - int(b.xyxy[0][0])) * (int(b.xyxy[0][3]) - int(b.xyxy[0][1])) for b in boxes]
            idx = int(np.argmax(areas))
            box = boxes[idx]
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2
            class_id = int(box.cls[0])
            conf = float(box.conf[0])

            # Draw detection
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
            cv2.circle(frame, (center_x, center_y), 5, (0,0,255), -1)
            label = f"Class {class_id} ({conf:.2f})"
            cv2.putText(frame, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

            # Map to robot coordinates
            robot_x, robot_y, robot_z = pixel_to_robot_coords(center_x, center_y, CAM_WIDTH, CAM_HEIGHT)
            cv2.putText(frame, f"Robot Target: ({robot_x:.1f}, {robot_y:.1f}, {robot_z:.1f})", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,0,0), 2)

            # Move robot (simulate)
            print(f"Moving robot to: X={robot_x:.1f} Y={robot_y:.1f} Z={robot_z:.1f}")
            bot.goTo(robot_x, robot_y, robot_z, duration=2.0)

        cv2.imshow("YOLO Detection - Press q to quit", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
