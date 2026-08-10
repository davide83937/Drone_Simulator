import numpy as np
import sympy as sp
import math
import time

# Ipotetica libreria DDS per agganciarsi al topic di Godot 4
# import dds_library

# =====================================================================
# 1. GENERAZIONE SIMBOLICA DELLA JACOBIANA (Eseguita 1 sola volta all'avvio)
# =====================================================================
print("Generazione Jacobiana con SymPy in corso...")
q0, q1, q2, q3 = sp.symbols('q0 q1 q2 q3')
wx, wy, wz, dt = sp.symbols('wx wy wz dt')

X_sym = sp.Matrix([q0, q1, q2, q3])

# Equazioni non lineari del modello cinematico (Predizione con Giroscopio)
f0 = q0 + 0.5 * dt * (-wx * q1 - wy * q2 - wz * q3)
f1 = q1 + 0.5 * dt * (wx * q0 + wz * q2 - wy * q3)
f2 = q2 + 0.5 * dt * (wy * q0 - wz * q1 + wx * q3)
f3 = q3 + 0.5 * dt * (wz * q0 + wy * q1 - wx * q2)
f_sym = sp.Matrix([f0, f1, f2, f3])

# SymPy calcola la matrice Jacobiana F (4x4)
F_sym = f_sym.jacobian(X_sym)

# LAMBDIFY: Converte la formula simbolica in una funzione Numpy compilata e veloce!
# Queste funzioni prendono in input i valori attuali e restituiscono array numpy
calc_state = sp.lambdify((q0, q1, q2, q3, wx, wy, wz, dt), f_sym, 'numpy')
calc_F = sp.lambdify((q0, q1, q2, q3, wx, wy, wz, dt), F_sym, 'numpy')


# =====================================================================
# 2. LA CLASSE DEL FILTRO DI KALMAN ESTESO (EKF)
# =====================================================================
class DroneEKF:
    def __init__(self):
        # Stato Iniziale: Quaternione identità (Drone perfettamente piatto)
        self.x = np.array([[1.0], [0.0], [0.0], [0.0]])

        # Matrice di Covarianza dell'Errore (P)
        self.P = np.eye(4) * 0.1

        # Matrice del Rumore di Processo (Q) - Quanto ci fidiamo del modello/giroscopio
        self.Q = np.eye(4) * 0.001

        # Matrice del Rumore di Misura (R) - Quanto ci fidiamo di Accel e Mag
        self.R = np.eye(4) * 0.5

        # Matrice di Misura (H) - Poiché convertiremo Acc/Mag in un quaternione misurato,
        # confrontiamo quaternione con quaternione. H è quindi una matrice identità 4x4.
        self.H = np.eye(4)

    def predict(self, gyro, dt_val):
        """Fase 1: Predizione dello stato futuro basata sulle velocità del giroscopio"""
        # Estraiamo i valori correnti
        _q0, _q1, _q2, _q3 = self.x.flatten()
        _wx, _wy, _wz = gyro

        # Calcoliamo il nuovo stato usando la funzione generata da SymPy
        x_new = calc_state(_q0, _q1, _q2, _q3, _wx, _wy, _wz, dt_val)

        # Normalizziamo il quaternione (deve avere sempre lunghezza 1)
        self.x = x_new / np.linalg.norm(x_new)

        # Calcoliamo la Jacobiana F per il frame corrente
        F_mat = np.array(calc_F(_q0, _q1, _q2, _q3, _wx, _wy, _wz, dt_val))

        # Propaghiamo la covarianza: P = F * P * F^T + Q
        self.P = F_mat @ self.P @ F_mat.T + self.Q

    def update(self, roll_acc, pitch_acc, yaw_mag):
        """Fase 2: Correzione dello stato basata su Accelerometro e Magnetometro"""
        # Per semplicità in questo EKF, calcoliamo prima un quaternione grezzo dai sensori.
        # 1. Roll e Pitch dall'accelerometro
        """ax, ay, az = accel
        roll_acc = math.atan2(ay, az)
        pitch_acc = math.atan2(-ax, math.sqrt(ay ** 2 + az ** 2))"""

        # 2. Yaw dal magnetometro (compensato con pitch e roll)
        """mx, my, mz = mag
        mag_x = mx * math.cos(pitch_acc) + mz * math.sin(pitch_acc)
        mag_y = mx * math.sin(roll_acc) * math.sin(pitch_acc) + my * math.cos(roll_acc) - mz * math.sin(
            roll_acc) * math.cos(pitch_acc)
        yaw_mag = math.atan2(-mag_y, mag_x)"""

        # 3. Trasformiamo gli angoli grezzi nel "Quaternione Misurato" (Z)
        z_meas = self.euler_to_quaternion(roll_acc, pitch_acc, yaw_mag)

        # 4. Equazioni classiche di Kalman per l'Update
        # S = H * P * H^T + R
        S = self.H @ self.P @ self.H.T + self.R

        # Guadagno K = P * H^T * S^-1
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # Errore tra misura e predizione (Innovazione)
        y = z_meas - (self.H @ self.x)

        # Aggiornamento stato e covarianza
        self.x = self.x + (K @ y)
        self.x = self.x / np.linalg.norm(self.x)  # Rinormalizza
        self.P = (np.eye(4) - K @ self.H) @ self.P

    def get_euler_angles(self):
        """Converte lo stato interno a quaternioni nei 3 angoli per il PID"""
        q0, q1, q2, q3 = self.x.flatten()

        # Rollio (X)
        sinr_cosp = 2 * (q0 * q1 + q2 * q3)
        cosr_cosp = 1 - 2 * (q1 ** 2 + q2 ** 2)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Beccheggio (Y) - Protezione contro singolarità asin
        sinp = 2 * (q0 * q2 - q3 * q1)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Imbardata (Z)
        siny_cosp = 2 * (q0 * q3 + q1 * q2)
        cosy_cosp = 1 - 2 * (q2 ** 2 + q3 ** 2)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Ritorna in gradi per comodità visiva o calcolo del PID
        return np.degrees([roll, pitch, yaw])

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """Utility function per convertire Eulero in Quaternione vettore colonna"""
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        q0 = cr * cp * cy + sr * sp * sy
        q1 = sr * cp * cy - cr * sp * sy
        q2 = cr * sp * cy + sr * cp * sy
        q3 = cr * cp * sy - sr * sp * cy
        return np.array([[q0], [q1], [q2], [q3]])


# =====================================================================
# 3. IL NODO BACKEND PYTHON (Simulazione Loop DDS)
# =====================================================================
def main():
    ekf = DroneEKF()
    last_time = time.time()

    # Inizializzazione mock del subscriber DDS (Godot -> Python)
    # dds_subscriber = dds_library.Subscriber("DroneSensors")

    print("In attesa dei dati sensore da Godot...")

    while True:
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        # 1. Lettura dei dati in streaming dal DDS
        # payload = dds_subscriber.read()
        # Mocking dei dati (in rad/s per Gyro, g per Accel, Gauss per Mag)
        simulated_gyro = [0.01, -0.02, 0.0]
        simulated_accel = [0.0, 0.0, 9.81]
        simulated_mag = [0.5, 0.0, -0.2]

        # 2. SENSOR FUSION STEP
        # Predizione ad altissima frequenza (es. 400Hz) usando la cinematica
        ekf.predict(simulated_gyro, dt)

        # Correzione usando i vettori assoluti (Può girare anche a frequenze inferiori, es. 100Hz)
        ekf.update(simulated_accel, simulated_mag)

        # 3. ESTRAZIONE DEGLI ANGOLI PER IL CONTROLLORE
        roll, pitch, yaw = ekf.get_euler_angles()

        # 4. CALCOLO ERRORE PER IL PID (Nodo Sottrattore)
        target_roll, target_pitch, target_yaw = (0.0, 0.0, 0.0)  # Hovering perfetto

        error_roll = target_roll - roll
        error_pitch = target_pitch - pitch
        error_yaw = target_yaw - yaw

        # 5. Invia gli errori all'algoritmo PID (che calcolerà i PWM)
        # pid_controller.compute(error_roll, error_pitch, error_yaw)

        print(f"Assetto [Gradi] -> Roll: {roll:.2f} | Pitch: {pitch:.2f} | Yaw: {yaw:.2f}")
        time.sleep(0.01)  # Simuliamo un loop a 100Hz


if __name__ == "__main__":
    main()