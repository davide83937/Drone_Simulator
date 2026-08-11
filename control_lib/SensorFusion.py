import numpy as np
import sympy as sp
import math

# =====================================================================
# 1. GENERAZIONE SIMBOLICA DELLA JACOBIANA
# =====================================================================
q0, q1, q2, q3 = sp.symbols('q0 q1 q2 q3')
wx, wy, wz, dt = sp.symbols('wx wy wz dt')

X_sym = sp.Matrix([q0, q1, q2, q3])

# Equazioni cinematiche dei quaternioni
f0 = q0 + 0.5 * dt * (-wx * q1 - wy * q2 - wz * q3)
f1 = q1 + 0.5 * dt * (wx * q0 + wz * q2 - wy * q3)
f2 = q2 + 0.5 * dt * (wy * q0 - wz * q1 + wx * q3)
f3 = q3 + 0.5 * dt * (wz * q0 + wy * q1 - wx * q2)
f_sym = sp.Matrix([f0, f1, f2, f3])

F_sym = f_sym.jacobian(X_sym)

calc_state = sp.lambdify((q0, q1, q2, q3, wx, wy, wz, dt), f_sym, 'numpy')
calc_F = sp.lambdify((q0, q1, q2, q3, wx, wy, wz, dt), F_sym, 'numpy')


# =====================================================================
# 2. FILTRO DI KALMAN ESTESO (EKF) CORRETTO
# =====================================================================
class DroneEKF:
    def __init__(self):
        # Stato iniziale: Quaternione identità (drone orizzontale)
        self.x = np.array([[1.0], [0.0], [0.0], [0.0]])

        # Covarianza Errore (P), Rumore Processo (Q), Rumore Misura (R)
        self.P = np.eye(4) * 0.1
        self.Q = np.eye(4) * 0.001
        self.R = np.eye(4) * 0.1
        self.H = np.eye(4)

    def predict(self, gyro_deg, dt_val):
        """
        gyro_deg: [rot_x, rot_z, rot_y] in gradi/secondo provenienti da Godot
        """
        # CORREZIONE 1: Convertiamo la velocità angolare da Gradi/s a Radianti/s
        gyro_rad = np.radians(gyro_deg)
        _wx, _wy, _wz = gyro_rad
        _q0, _q1, _q2, _q3 = self.x.flatten()

        # Propagazione stato
        x_new = calc_state(_q0, _q1, _q2, _q3, _wx, _wy, _wz, dt_val)
        self.x = x_new / np.linalg.norm(x_new)

        # Propagazione Covarianza
        F_mat = np.array(calc_F(_q0, _q1, _q2, _q3, _wx, _wy, _wz, dt_val))
        self.P = F_mat @ self.P @ F_mat.T + self.Q

    def update(self, roll_acc_deg, pitch_acc_deg, yaw_mag_deg):
        """
        Riceve gli angoli calcolati da Accelerometro e Magnetometro in GRADI
        """
        # CORREZIONE 2: Convertiamo gli angoli misurati in RADIANTI prima di calcolare il Quaternione
        roll_rad = math.radians(roll_acc_deg)
        pitch_rad = math.radians(pitch_acc_deg)
        yaw_rad = math.radians(yaw_mag_deg)

        # Quaternione di misura Z
        z_meas = self.euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)

        # Update di Kalman classico
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        y = z_meas - (self.H @ self.x)

        self.x = self.x + (K @ y)
        self.x = self.x / np.linalg.norm(self.x)
        self.P = (np.eye(4) - K @ self.H) @ self.P

    def get_euler_angles(self):
        """Restituisce [Roll, Pitch, Yaw] stimati in GRADI per il PID"""
        q0, q1, q2, q3 = self.x.flatten()

        # Rollio (X)
        sinr_cosp = 2 * (q0 * q1 + q2 * q3)
        cosr_cosp = 1 - 2 * (q1 ** 2 + q2 ** 2)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Beccheggio (Y)
        sinp = 2 * (q0 * q2 - q3 * q1)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Imbardata (Z)
        siny_cosp = 2 * (q0 * q3 + q1 * q2)
        cosy_cosp = 1 - 2 * (q2 ** 2 + q3 ** 2)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Restituisce in gradi per il PID
        return np.degrees([roll, pitch, yaw])

    @staticmethod
    def euler_to_quaternion(roll_rad, pitch_rad, yaw_rad):
        """Accetta gli angoli strettamente in RADIANTI"""
        cr = math.cos(roll_rad * 0.5)
        sr = math.sin(roll_rad * 0.5)
        cp = math.cos(pitch_rad * 0.5)
        sp = math.sin(pitch_rad * 0.5)
        cy = math.cos(yaw_rad * 0.5)
        sy = math.sin(yaw_rad * 0.5)

        q0 = cr * cp * cy + sr * sp * sy
        q1 = sr * cp * cy - cr * sp * sy
        q2 = cr * sp * cy + sr * cp * sy
        q3 = cr * cp * sy - sr * sp * cy
        return np.array([[q0], [q1], [q2], [q3]])