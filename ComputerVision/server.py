import os
import json
import threading
import time

import numpy as np
import cv2
import uvicorn
from fastapi import FastAPI, Request, HTTPException


#cv2.setNumThreads(1)
#os.environ["OMP_NUM_THREADS"] = "1"
#os.environ["OPENBLAS_NUM_THREADS"] = "1"

# --- CONFIGURAZIONE AMBIENTE ROS 2 WINDOWS ---
extra_dll_dirs = [
    r"C:\Users\david\OneDrive\Desktop\ros2-windows\install\custom_interfaces\bin",
    r"C:\Users\david\OneDrive\Desktop\ros2-windows\bin",
    r"C:\Users\david\OneDrive\Desktop\ros2-windows\.pixi\envs\default\bin",
    r"C:\Users\david\OneDrive\Desktop\ros2-windows\.pixi\envs\default\Library\bin",
]
for directory in extra_dll_dirs:
    if os.path.isdir(directory):
        os.add_dll_directory(directory)
        os.environ["PATH"] = directory + os.pathsep + os.environ.get("PATH", "")

prefix_path = (
    r"C:\Users\david\OneDrive\Desktop\ros2-windows\install\custom_interfaces;"
    r"C:\Users\david\OneDrive\Desktop\ros2-windows"
)
os.environ["AMENT_PREFIX_PATH"] = prefix_path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from box_cross_detector import (
    check_centering,
    extract_white_boxes,
    find_cross_inside_box,
)
from smoke_function import (
    calculate_occupancy,
    check_white_box_near_smoke,
    fuse_and_extract_objects,
)
from test3 import in_range_blue

# --- NODO ROS 2 PER LA COMPUTER VISION ---
class VisionPublisherNode(Node):
    def __init__(self):
        super().__init__('vision_publisher_node')
        self.smoke_pub = self.create_publisher(String, 'cv_smoke_info', 10)
        self.bottom_pub = self.create_publisher(String, 'cv_bottom_info', 10)

rclpy.init()
ros_node = VisionPublisherNode()

# Esegue lo spin del nodo ROS su un thread dedicato
ros_spin_thread = threading.Thread(target=lambda: rclpy.spin(ros_node), daemon=True)
ros_spin_thread.start()

app = FastAPI()


def decode_image(raw_bytes: bytes):
    if not raw_bytes:
        return None
    arr = np.frombuffer(raw_bytes, dtype=np.uint8)
    return cv2.imdecode(arr, cv2.IMREAD_COLOR)


# ==========================================================
# VARIABILI GLOBALI PER LIMITARE LA FREQUENZA (10 FPS MAX)
# ==========================================================
last_process_time_ahead = 0.0
last_process_time_bottom = 0.0
PROCESS_INTERVAL = 0.1  # 0.1 secondi = 10 frame al secondo


@app.post("/image/ahead")
async def receive_ahead(request: Request):
    global last_process_time_ahead
    now = time.time()

    # SE È PASSATO TROPPO POCO TEMPO, IGNORA IL FRAME E RISPONDI SUBITO
    if now - last_process_time_ahead < PROCESS_INTERVAL:
        return {"status": "skipped"}

    last_process_time_ahead = now

    # --- INIZIO ELABORAZIONE ORIGINALE ---
    data = await request.body()
    frame = decode_image(data)
    if frame is None:
        raise HTTPException(status_code=400, detail="Frame non decodificabile")

    h, w, _ = frame.shape
    vis = frame.copy()

    blue_mask = in_range_blue(frame)
    objects = fuse_and_extract_objects(blue_mask, min_area=2000)
    white_boxes = extract_white_boxes(frame, min_area=1500)

    smoke_msg_payload = {
        "detected": False,
        "center_x": 0,
        "center_y": 0,
        "coverage": 0.0,
        "err_x": 0.0,
        "box_nearby": False
    }

    for obj in objects:
        if obj["is_smoke"]:
            cx, cy = obj["center"]
            contour = obj["contour"]
            occupancy = calculate_occupancy([contour], w, h)
            near_box, box_c = check_white_box_near_smoke((cx, cy), white_boxes)

            smoke_msg_payload["detected"] = True
            smoke_msg_payload["center_x"] = int(cx)
            smoke_msg_payload["center_y"] = int(cy)
            smoke_msg_payload["coverage"] = float(occupancy)
            smoke_msg_payload["err_x"] = float((cx - (w / 2.0)) / (w / 2.0))
            smoke_msg_payload["box_nearby"] = bool(near_box)

            cv2.drawContours(vis, [contour], -1, (0, 255, 0), 2)
            cv2.putText(vis, f"Cover: {occupancy:.1f}%", (cx + 10, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)
            if near_box:
                cv2.drawContours(vis, [box_c], -1, (0, 255, 255), 3)
            break

    cv2.putText(vis, "CAM AHEAD", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.imshow("Stream Camera Ahead", vis)
    cv2.waitKey(1)

    msg = String()
    msg.data = json.dumps(smoke_msg_payload)
    ros_node.smoke_pub.publish(msg)

    return {"status": "ok"}


@app.post("/image/bottom")
async def receive_bottom(request: Request):
    global last_process_time_bottom
    now = time.time()

    # SE È PASSATO TROPPO POCO TEMPO, IGNORA IL FRAME
    if now - last_process_time_bottom < PROCESS_INTERVAL:
        return {"status": "skipped"}

    last_process_time_bottom = now

    # --- INIZIO ELABORAZIONE ORIGINALE ---
    data = await request.body()
    frame = decode_image(data)
    if frame is None:
        raise HTTPException(status_code=400, detail="Frame non decodificabile")

    h, w, _ = frame.shape
    vis = frame.copy()
    cx_screen, cy_screen = w // 2, h // 2
    cv2.drawMarker(vis, (cx_screen, cy_screen), (128, 128, 128), cv2.MARKER_CROSS, 40, 1)

    white_boxes = extract_white_boxes(frame, min_area=300)
    target_payload = {
        "box_detected": False,
        "cross_detected": False,
        "err_x_px": 0.0,
        "err_y_px": 0.0,
        "total_err_px": 0.0
    }

    for box in white_boxes:
        target_payload["box_detected"] = True
        cv2.drawContours(vis, [box], -1, (255, 255, 0), 2)
        has_cross, cross_contour = find_cross_inside_box(frame, box)

        if has_cross:
            target_payload["cross_detected"] = True
            cv2.drawContours(vis, [cross_contour], -1, (0, 0, 255), 2)

            M = cv2.moments(cross_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                err_x = cx - cx_screen
                err_y = cy - cy_screen
                err_dist = np.hypot(err_x, err_y)

                target_payload["err_x_px"] = float(err_x)
                target_payload["err_y_px"] = float(err_y)
                target_payload["total_err_px"] = float(err_dist)

                cv2.circle(vis, (cx, cy), 6, (0, 255, 0), -1)
                cv2.line(vis, (cx_screen, cy_screen), (cx, cy), (0, 255, 0), 1)
                cv2.putText(vis, f"Err: {int(err_dist)}px", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            break

    cv2.putText(vis, "CAM BOTTOM", (15, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.imshow("Stream Camera Bottom", vis)
    cv2.waitKey(1)

    msg = String()
    msg.data = json.dumps(target_payload)
    ros_node.bottom_pub.publish(msg)

    return {"status": "ok"}


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)