import math

gyro_roll = 0.0
gyro_pitch = 0.0
gyro_yaw = 0.0


def get_yaw_from_magnetometer(bx, bz):
    if bx == None or bz == None:
        return 0

    yaw_rad = math.atan2(bz, bx)
    yaw_deg = math.degrees(yaw_rad)
    yaw_deg = from180to360(yaw_deg)
    angle = 270 - yaw_deg
    if angle > 0:
        angle -= 360
    angle = 360 + angle
    return angle


def from180to360(yaw_deg):
    if yaw_deg == None:
        return 0
    if yaw_deg < 0:
        yaw_deg += 360
    return yaw_deg



def get_roll_pitch_accelerometer(ax, ay, az):
    if ax == None or ay == None or az == None:
        return 0, 0
    roll_rad = math.atan2(ax, ay)
    pitch_rad = math.atan2(az, math.sqrt(ax ** 2 + ay ** 2))
    roll_deg, pitch_deg = math.degrees(roll_rad), math.degrees(pitch_rad)
    return roll_deg, pitch_deg


def process_gyro_data(gx, gy, gz, dt):
    global gyro_roll, gyro_pitch, gyro_yaw, last_time

    if gx == None or gy == None or gz == None or dt == None:
        return gyro_roll, gyro_pitch, gyro_yaw
    #print(dt)
    # Integra le velocità angolari (deg/s * s = gradi)
    gyro_roll += gz * dt
    gyro_yaw += gy * dt
    gyro_pitch += gx * dt
    if gyro_yaw > 0:
        gyro_yaw -= 360
    if gyro_yaw < -360:
        gyro_yaw += 360


    return gyro_roll, gyro_pitch, 360+gyro_yaw

