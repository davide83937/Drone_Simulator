from control_lib import SensorFusion
from lib.dds import dds as DDS
from lib.utils.time import *
from my_math import sensor_math as sm
from control_lib import *
from control_scheme import state
from control_scheme import control_scheme

dds = DDS.DDS()
dds.start('127.0.0.1', 4445)
ekf = SensorFusion.DroneEKF()
drone_control_scheme = control_scheme.droneControlScheme()

drone_control_scheme.start(
    y_start=0.0, y_end=10,
    z_start=0.0, x_start=0.0,
    z_end=0.0, x_end=0.0,
    ang_start=0.0, ang_end=0.0
)

previous_yaw = 0

dds.subscribe(['tick'])
dds.subscribe(['gyro_x'])
dds.subscribe(['gyro_y'])
dds.subscribe(['gyro_z'])
dds.subscribe(['a_x'])
dds.subscribe(['a_y'])
dds.subscribe(['a_z'])
dds.subscribe(['b_x'])
dds.subscribe(['b_y'])
dds.subscribe(['b_z'])
dds.subscribe(['pos_x'])
dds.subscribe(['pos_y'])
dds.subscribe(['pos_z'])
dds.subscribe(['vel_x'])
dds.subscribe(['vel_y'])
dds.subscribe(['vel_z'])
t = Time()
t.start()


while True:
    dds.wait('tick')
    delta_t = t.elapsed()
    tick = dds.read('tick')
    rot_x = dds.read('gyro_x')
    rot_y = dds.read('gyro_y')
    rot_z = dds.read('gyro_z')
    a_x = dds.read('a_x')
    a_y = dds.read('a_y')
    a_z = dds.read('a_z')
    b_x = dds.read('b_x')
    b_y = dds.read('b_y')
    b_z = dds.read('b_z')
    pos_x = dds.read('pos_x')
    pos_y = dds.read('pos_y')
    pos_z = dds.read('pos_z')
    vel_x = dds.read('vel_x')
    vel_y = dds.read('vel_y')
    vel_z = dds.read('vel_z')

    if None in (rot_x, rot_y, rot_z, a_x, a_y, a_z, b_x, b_y, b_z, delta_t,
                pos_x, pos_y, pos_z, vel_x, vel_y, vel_z):
        continue

    #roll_gyro, pitch_gyro, yaw_gyro = sm.process_gyro_data(rot_x, rot_y, rot_z, delta_t)
    roll_acc, pitch_acc = sm.get_roll_pitch_accelerometer(a_x, a_y, a_z)
    yaw_magnetometer = sm.get_yaw_from_magnetometer(b_x, b_y)
    delta_yaw = (yaw_magnetometer - previous_yaw+180)%360-180
    angular_velocity = delta_yaw / delta_t

    current_state = state.State(
        roll_acc = roll_acc,
        pitch_acc = pitch_acc,
        yaw_magnetometer = yaw_magnetometer,
        angular_velocity = angular_velocity,
        pos_x = pos_x,
        pos_y = pos_y,
        pos_z = pos_z,
        vel_x = vel_x,
        vel_y = vel_y,
        vel_z = vel_z
    )

    gyro = [rot_z, rot_x, rot_y]
    #accel = [roll_acc, pitch_acc]

    ekf.predict(gyro, delta_t)
    # Correzione usando i vettori assoluti (Può girare anche a frequenze inferiori, es. 100Hz)
    ekf.update(roll_acc, pitch_acc, yaw_magnetometer)

    # 3. ESTRAZIONE DEGLI ANGOLI PER IL CONTROLLORE
    roll, pitch, yaw = ekf.get_euler_angles()

    error_vy, error_vx, error_vz, error_angular_v = drone_control_scheme.outer_loop(delta_t, current_state)
    w1, w2, w3, w4 = drone_control_scheme.inner_loop(error_vy, error_vx, error_vz, error_angular_v, roll, pitch, yaw, rot_z, rot_x, rot_y)

    # 4. CALCOLO ERRORE PER IL PID (Nodo Sottrattore)
    #target_roll, target_pitch, target_yaw = (0.0, 0.0, 0.0)  # Hovering perfetto

    #error_roll = target_roll - roll
    #error_pitch = target_pitch - pitch
    #error_yaw = target_yaw - yaw



    #dds.publish("w1", w1, DDS.DDS.DDS_TYPE_FLOAT)
    #dds.publish("w2", w2, DDS.DDS.DDS_TYPE_FLOAT)
    #dds.publish("w3", w3, DDS.DDS.DDS_TYPE_FLOAT)
    #dds.publish("w4", w4, DDS.DDS.DDS_TYPE_FLOAT)
    #print(tick)
    #print(f"rot_x = {rot_x}, rot_y = {rot_y}")
    #print(f"roll_gyro = {roll_gyro}, pitch_gyro = {pitch_gyro}, yaw_gyro = {yaw_gyro}")
    #print(f"roll_gyro = {roll_gyro}, pitch_gyro = {pitch_gyro}")
    #print(f"roll_acc = {roll_acc}, pitch_acc = {pitch_acc}")
    #print(f"Gyro yaw, {yaw_gyro}")
    #print(f"bx = {b_x}, bz = {b_y}")
    #print(f"bx = {b_x}, by = {b_y}, bz = {b_z}")
    #print(f"Magne yaw, {yaw_magnetometer}")
    #print(f"error roll: {error_roll}, error: {error_pitch}, error: {error_yaw}")
    previous_yaw = yaw_magnetometer