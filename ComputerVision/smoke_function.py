import cv2
import numpy as np

def get_center(contour):
    """Calcola il baricentro di un contorno tramite i momenti."""
    M = cv2.moments(contour)
    if M["m00"] != 0:
        return int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
    return None, None


def calculate_occupancy(contours, image_width, image_height):
    """Calcola la percentuale di schermo occupata dall'area dei contorni."""
    if not contours:
        return 0.0
    total_screen_area = float(image_width * image_height)
    total_contour_area = sum(cv2.contourArea(c) for c in contours)
    return (total_contour_area / total_screen_area) * 100.0


def fuse_and_extract_objects(binary_mask, min_area=2000):
    """
    Fonde i frammenti di fumo tramite dilatazione ed estrae
    gli oggetti calcolando la loro solidità.
    """
    kernel = np.ones((13, 13), np.uint8)
    fused_mask = cv2.dilate(binary_mask, kernel, iterations=2)

    contours, _ = cv2.findContours(fused_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detected_objects = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue

        cx, cy = get_center(c)
        if cx is None:
            continue

        # Calcolo Solidità: Area contorno / Area Convex Hull
        hull = cv2.convexHull(c)
        hull_area = cv2.contourArea(hull)
        solidity = float(area) / hull_area if hull_area > 0 else 0.0

        # Sotto 0.95 è fumo, pari o sopra è solido compatto
        is_smoke = solidity < 0.95

        detected_objects.append({
            "contour": c,
            "center": (cx, cy),
            "area": area,
            "solidity": solidity,
            "is_smoke": is_smoke
        })

    return detected_objects


def check_white_box_near_smoke(smoke_center, white_boxes, max_distance=250):
    """
    Verifica se tra le scatole bianche identificate ce n'è una
    nelle vicinanze della sorgente del fumo.
    """
    if smoke_center is None or not white_boxes:
        return False, None

    sx, sy = smoke_center
    closest_box = None
    min_dist = float("inf")

    for box in white_boxes:
        M = cv2.moments(box)
        if M["m00"] != 0:
            bx = int(M["m10"] / M["m00"])
            by = int(M["m01"] / M["m00"])
            dist = np.hypot(sx - bx, sy - by)
            if dist < min_dist and dist <= max_distance:
                min_dist = dist
                closest_box = box

    return (closest_box is not None), closest_box