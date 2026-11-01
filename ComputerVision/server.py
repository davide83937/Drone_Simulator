import cv2
from fastapi import FastAPI, HTTPException, Request
import numpy as np
import uvicorn

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

app = FastAPI()


def decode_image(raw_bytes: bytes):
  if not raw_bytes:
    return None
  arr = np.frombuffer(raw_bytes, dtype=np.uint8)
  return cv2.imdecode(arr, cv2.IMREAD_COLOR)


@app.post("/image/ahead")
async def receive_ahead(request: Request):
  data = await request.body()
  frame = decode_image(data)
  if frame is None:
    raise HTTPException(
        status_code=400, detail="Frame vuoto o non decodificabile"
    )

  h, w, _ = frame.shape
  vis = frame.copy()

  # 1. Rileva fumo (Solidità < 0.95)
  blue_mask = in_range_blue(frame)
  objects = fuse_and_extract_objects(blue_mask, min_area=2000)

  # 2. Rileva box bianchi frontali
  white_boxes = extract_white_boxes(frame, min_area=1500)
  for wb in white_boxes:
    cv2.drawContours(vis, [wb], -1, (255, 255, 255), 2)

  smoke_found = False
  for obj in objects:
    cx, cy = obj["center"]
    contour = obj["contour"]

    if obj["is_smoke"]:
      smoke_found = True
      occupancy = calculate_occupancy([contour], w, h)
      cv2.drawContours(vis, [contour], -1, (0, 255, 0), 2)
      cv2.putText(
          vis,
          "SIGNAL DETECTED",
          (cx + 12, cy - 12),
          cv2.FONT_HERSHEY_SIMPLEX,
          0.6,
          (0, 255, 255),
          2,
      )
      cv2.putText(
          vis,
          f"Cover: {occupancy:.1f}%",
          (cx + 12, cy + 12),
          cv2.FONT_HERSHEY_SIMPLEX,
          0.5,
          (0, 165, 255),
          2,
      )

      near_box, box_c = check_white_box_near_smoke((cx, cy), white_boxes)
      if near_box:
        cv2.putText(
            vis,
            "BOX BIANCO IDENTIFICATO",
            (cx + 12, cy + 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            2,
        )
        cv2.drawContours(vis, [box_c], -1, (0, 255, 255), 3)

  cv2.putText(
      vis,
      "CAM AHEAD (FRONT)",
      (15, 30),
      cv2.FONT_HERSHEY_SIMPLEX,
      0.7,
      (0, 255, 255),
      2,
  )
  cv2.circle(vis, (w // 2, h // 2), 5, (0, 255, 255), -1)

  cv2.imshow("Stream Camera Ahead", vis)
  cv2.waitKey(1)

  return {"status": "ok", "smoke_detected": smoke_found}


@app.post("/image/bottom")
async def receive_bottom(request: Request):
  data = await request.body()
  frame = decode_image(data)
  if frame is None:
    raise HTTPException(
        status_code=400, detail="Frame vuoto o non decodificabile"
    )

  h, w, _ = frame.shape
  vis = frame.copy()

  # Mirino fisso al centro dello schermo
  center_x, center_y = w // 2, h // 2
  cv2.drawMarker(
      vis, (center_x, center_y), (128, 128, 128), cv2.MARKER_CROSS, 40, 1
  )
  cv2.circle(vis, (center_x, center_y), 25, (128, 128, 128), 1)

  white_boxes = extract_white_boxes(frame, min_area=300)
  target_aligned = False
  cross_detected = False

  for box in white_boxes:
    cv2.drawContours(vis, [box], -1, (255, 255, 0), 2)
    has_cross, cross_contour = find_cross_inside_box(frame, box)

    if has_cross:
      cross_detected = True
      cv2.drawContours(vis, [cross_contour], -1, (0, 0, 255), 2)

      M = cv2.moments(cross_contour)
      if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        cv2.circle(vis, (cx, cy), 6, (0, 0, 255), -1)

        is_centered, _, err_px = check_centering(cx, cy, w, h, tolerance=25)

        if is_centered:
          target_aligned = True
          cv2.rectangle(
              vis,
              (center_x - 30, center_y - 30),
              (center_x + 30, center_y + 30),
              (0, 255, 0),
              2,
          )
          cv2.putText(
              vis,
              "ALLINEATO AL CENTRO (READY)",
              (20, 40),
              cv2.FONT_HERSHEY_SIMPLEX,
              0.7,
              (0, 255, 0),
              2,
          )
        else:
          cv2.putText(
              vis,
              f"ALLINEAMENTO IN CORSO: Err {err_px}px",
              (20, 40),
              cv2.FONT_HERSHEY_SIMPLEX,
              0.6,
              (0, 165, 255),
              2,
          )

  cv2.putText(
      vis,
      "CAM BOTTOM (DOWN)",
      (15, h - 20),
      cv2.FONT_HERSHEY_SIMPLEX,
      0.6,
      (0, 255, 0),
      2,
  )

  cv2.imshow("Stream Camera Bottom", vis)
  cv2.waitKey(1)

  return {
      "status": "ok",
      "cross_detected": cross_detected,
      "is_centered": target_aligned,
  }


if __name__ == "__main__":
  uvicorn.run(app, host="0.0.0.0", port=8000)