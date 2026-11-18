import numpy as np

def rot_yaw(yaw: float) -> np.ndarray:
    """Rotazione attorno all'asse Y (Yaw / imbardata) in radianti."""
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([
        [ c,  0.0, s],
        [0.0, 1.0, 0.0],
        [-s,  0.0, c]
    ], dtype=np.float32)


def rot_pitch(pitch: float) -> np.ndarray:
    """Rotazione attorno all'asse X (Pitch / beccheggio) in radianti."""
    c, s = np.cos(pitch), np.sin(pitch)
    return np.array([
        [1.0, 0.0, 0.0],
        [0.0,  c,  -s],
        [0.0,  s,   c]
    ], dtype=np.float32)


def rot_roll(roll: float) -> np.ndarray:
    """Rotazione attorno all'asse Z (Roll / rollio) in radianti."""
    c, s = np.cos(roll), np.sin(roll)
    return np.array([
        [ c,  -s,  0.0],
        [ s,   c,  0.0],
        [0.0, 0.0, 1.0]
    ], dtype=np.float32)