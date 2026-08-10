from control_lib import SensorFusion
from lib.dds import dds as DDS
from lib.utils.time import *
from my_math import sensor_math as sm
from control_lib import *

dds = DDS.DDS()
dds.start('127.0.0.1', 4445)
ekf = SensorFusion.DroneEKF()

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

    #roll_gyro, pitch_gyro, yaw_gyro = sm.process_gyro_data(rot_x, rot_y, rot_z, delta_t)
    roll_acc, pitch_acc = sm.get_roll_pitch_accelerometer(a_x, a_y, a_z)
    yaw_magnetometer = sm.get_yaw_from_magnetometer(b_x, b_y)

    gyro = [rot_z, rot_y, rot_x]
    #accel = [roll_acc, pitch_acc]

    ekf.predict(gyro, delta_t)
    # Correzione usando i vettori assoluti (Può girare anche a frequenze inferiori, es. 100Hz)
    ekf.update(roll_acc, pitch_acc, yaw_magnetometer)

    # 3. ESTRAZIONE DEGLI ANGOLI PER IL CONTROLLORE
    roll, pitch, yaw = ekf.get_euler_angles()

    # 4. CALCOLO ERRORE PER IL PID (Nodo Sottrattore)
    target_roll, target_pitch, target_yaw = (0.0, 0.0, 0.0)  # Hovering perfetto

    error_roll = target_roll - roll
    error_pitch = target_pitch - pitch
    error_yaw = target_yaw - yaw


    dds.publish("f1", 25, DDS.DDS.DDS_TYPE_FLOAT)
    dds.publish("f2", -20, DDS.DDS.DDS_TYPE_FLOAT)
    #print(tick)
    #print(f"rot_x = {rot_x}, rot_y = {rot_y}")
    #print(f"roll_gyro = {roll_gyro}, pitch_gyro = {pitch_gyro}, yaw_gyro = {yaw_gyro}")
    #print(f"roll_gyro = {roll_gyro}, pitch_gyro = {pitch_gyro}")
    #print(f"roll_acc = {roll_acc}, pitch_acc = {pitch_acc}")
    #print(f"Gyro yaw, {yaw_gyro}")
    #print(f"bx = {b_x}, bz = {b_y}")
    #print(f"bx = {b_x}, by = {b_y}, bz = {b_z}")
    #print(f"Magne yaw, {yaw_magnetometer}")
    print(f"error roll: {error_roll}, error: {error_pitch}, error: {error_yaw}")