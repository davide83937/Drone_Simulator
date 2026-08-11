from dataclasses import dataclass


@dataclass
class State:
    tick:float
    roll_acc: float
    pitch_acc: float
    yaw_magnetometer: float
    angular_velocity: float

    pos_x: float
    pos_y: float
    pos_z: float

    vel_x: float
    vel_y: float
    vel_z: float