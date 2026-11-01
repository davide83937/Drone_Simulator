import cv2
import numpy as np


def extract_white_boxes(image, min_area=300):
    """Estrae i box bianchi con range HSV tollerante verso ombre e rumore."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_white = np.array([0, 0, 120])
    upper_white = np.array([180, 80, 255])
    white_mask = cv2.inRange(hsv, lower_white, upper_white)

    kernel = np.ones((5, 5), np.uint8)
    white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(
        white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    boxes = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue

        hull = cv2.convexHull(c)
        hull_area = cv2.contourArea(hull)
        solidity = float(area) / hull_area if hull_area > 0 else 0.0

        if solidity >= 0.75:
            boxes.append(c)

    return boxes


def is_red_cross(contour_red):
    """Verifica se il contorno rosso è un '+' tramite aspect ratio, solidità,

    approssimazione poligonale e difetti di convessità.
    """
    area = cv2.contourArea(contour_red)
    if area < 20:
        return False

    # 1. Aspect Ratio
    _, _, w, h = cv2.boundingRect(contour_red)
    aspect_ratio = float(w) / h
    if not (0.40 <= aspect_ratio <= 2.30):
        return False

    # 2. Solidità
    hull = cv2.convexHull(contour_red)
    hull_area = cv2.contourArea(hull)
    if hull_area == 0:
        return False
    solidity = float(area) / hull_area
    if not (0.20 <= solidity <= 0.92):
        return False

    # 3. Approssimazione poligonale
    epsilon = 0.02 * cv2.arcLength(contour_red, True)
    approx = cv2.approxPolyDP(contour_red, epsilon, True)

    num_vertices = len(approx)
    if 8 <= num_vertices <= 14:
        return True

    # 4. Difetti di Convessità protetti da eccezioni
    if len(approx) >= 4:
        hull_indices = cv2.convexHull(approx, returnPoints=False)
        if hull_indices is not None and len(hull_indices) >= 3:
            try:
                defects = cv2.convexityDefects(approx, hull_indices)
                if defects is not None:
                    defects_reshaped = defects.reshape(-1, 4)
                    prominent_defects = 0
                    for i in range(defects_reshaped.shape[0]):
                        depth = defects_reshaped[i, 3] / 256.0
                        if depth >= 1.5:
                            prominent_defects += 1

                    if 2 <= prominent_defects <= 6:
                        return True
            except cv2.error:
                pass

    if 0.30 <= solidity <= 0.65:
        return True

    return False


def find_cross_inside_box(image, box_contour):
    """Estrae la croce rossa all'interno del box bianco."""
    x, y, w, h = cv2.boundingRect(box_contour)
    roi = image[y : y + h, x : x + w]
    if roi.size == 0:
        return False, None

    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(
        hsv_roi, np.array([0, 50, 50]), np.array([15, 255, 255])
    )
    mask2 = cv2.inRange(
        hsv_roi, np.array([165, 50, 50]), np.array([180, 255, 255])
    )
    red_mask = cv2.bitwise_or(mask1, mask2)

    red_contours, _ = cv2.findContours(
        red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    for rc in red_contours:
        if is_red_cross(rc):
            global_rc = rc + np.array([[[x, y]]])
            return True, global_rc

    return False, None


def check_centering(cx, cy, frame_w, frame_h, tolerance=25):
    """Calcola la distanza dal mirino e valida se il bersaglio è al centro."""
    center_screen_x = frame_w // 2
    center_screen_y = frame_h // 2
    dist_from_center = np.hypot(cx - center_screen_x, cy - center_screen_y)
    is_centered = dist_from_center <= tolerance
    return is_centered, (center_screen_x, center_screen_y), int(dist_from_center)