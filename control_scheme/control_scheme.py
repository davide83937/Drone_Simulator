import math
from abc import ABC, abstractmethod
from control_lib import VirtualRobot
from control_lib import pid
from control_scheme.state import State
from control_scheme.mixer import mixer
from plotting.plotManager import plotManager


class controlScheme(ABC):
    @abstractmethod
    def __init__(self):
        pass

    @abstractmethod
    def start(self, **kwargs):
        pass

    @abstractmethod
    def inner_loop(self, state, target_thrust, target_roll, target_pitch, target_yaw_rate,
                   roll, pitch, yaw, speed_roll, speed_pitch, speed_yaw):
        pass

    @abstractmethod
    def outer_loop(self, delta_t, state):
        pass


class droneControlScheme(controlScheme):
    def __init__(self):
        self.time = 0.0
        self.nframe = 0

        self.plottino = plotManager(25)


        # --- CONTROLLORI ALTITUDINE (Asse Y in Godot) ---
        self.virtualRobotAltitude = VirtualRobot.StraightLine2DMotion(10, 2, 2)
        self.p_controller_altitude = pid.PID(0.5, 0, 0.1, 50)
        self.pi_controller_speed_altitude = pid.PID(0.5, 0.35, 0.2, 30)

        # --- CONTROLLORI PIANO ORIZZONTALE (Assi X e Z) ---
        self.virtualRobotXY = VirtualRobot.StraightLine2DMotion(20, 2, 2)

        # Posizione e Velocità lungo X (Genera il Roll Target)
        self.p_controller_x = pid.PID(0.21, 0, 0, 0)
        self.pi_controller_speed_x = pid.PID(2.5, 0.01, 0.17, 0)

        # Posizione e Velocità lungo Z (Genera il Pitch Target)
        self.p_controller_z = pid.PID(0.24, 0, 0, 0)
        self.pi_controller_speed_z = pid.PID(0.25, 0.01, 0.15, 0)

        # --- CONTROLLORI ANGOLARI E ROTAZIONE (Yaw Target) ---
        self.virtualRobotAngular = VirtualRobot.StraightLineMotion(0.06, 0.005, 0.005, 0)
        self.p_controller_angular = pid.PID(0.3, 0, 0, 0)
        #self.pi_controller_angular_speed = pid.PID(0.1, 0.1, 0.0, 0)

        # --- INNER LOOP: ASSETTO E RATEI ANGOLARI ----------------------------------------------------------------
        #self.yaw_P = pid.PID(0.3, 0, 0, 0)
        self.yaw_PI = pid.PID(0.1, 0.1, 0, 0)

        self.roll_P = pid.PID(0.5, 0, 0, 0)
        self.roll_PI = pid.PID(0.1, 0.3, 0.02, 0)

        self.pitch_P = pid.PID(0.5, 0, 0, 0)
        self.pitch_PI = pid.PID(0.1, 0.3, 0.5, 0)

        #self.pitch_P = pid.PID(0.3, 0, 0, 0)
        #self.pitch_PI = pid.PID(0.1, 0.3, 0.01, 0)
#---------------------------------------------------------------------------------------------------------
    def start(self, **kwargs):
        # Altitudine lungo Y
        self.virtualRobotAltitude.start_motion((0, kwargs['y_start']), (0, kwargs['y_end']))
        # Posizione orizzontale sul piano (Z, X)
        self.virtualRobotXY.start_motion((kwargs['z_start'], kwargs['x_start']), (kwargs['z_end'], kwargs['x_end']))
        # Angolo di rotta (Yaw)
        self.virtualRobotAngular.start_motion([kwargs['ang_start']], [kwargs['ang_end']])



    def outer_loop(self, delta_t, state: State):

        """
        LOOP ESTERNO (Posizione -> Velocità -> Target di Inclinazione e Spinta)
        """
        # 1. VALUTAZIONE DEI VIRTUAL ROBOT (SETPOINTS TRAIETTORIA)
        target_y = 20
        target_z, target_x = 0, 0
        angle_target = 0
        self.time += delta_t
        #print("angle_target:", angle_target)
        #print(f"target_y: {target_y}, target_z: {target_z}, angle_target: {angle_target}")
        #print(f"target_z, {state.pos_z}")
        if state.pos_z > -10.0:
            _, target_y = self.virtualRobotAltitude.evaluate(delta_t)
            target_z, target_x = self.virtualRobotXY.evaluate(delta_t)
            angle_target = self.virtualRobotAngular.evaluate(delta_t)[0]
            angle_target = math.degrees(angle_target)
            #print("angle_target:", angle_target)
        #print("target_y:", target_y)

        # 2. CONTROLLO ALTITUDINE (ASSE Y VERTICALE)
        # Errore di posizione verticale (target_y vs pos_y)
        print(f"Z : {state.pos_z}")
        self.p_controller_altitude.evaluate_error(target_y, state.pos_z)
        self.p_controller_altitude.evaluate_error_kp()
        self.p_controller_altitude.saturation_p(-10.0, 10.0)
        self.p_controller_altitude.evaluate_error_kd(state.tick)

        error_p_y = self.p_controller_altitude.evaluate_total_error()
        #print(f"error_p_y: {error_p_y}")

        # Errore di velocità verticale (error_p_y vs vel_y)
        self.pi_controller_speed_altitude.evaluate_error(error_p_y, state.vel_z)
        self.pi_controller_speed_altitude.evaluate_error_kp()
        self.pi_controller_speed_altitude.saturation_p(-5.0, 5.0)
        self.pi_controller_speed_altitude.evaluate_error_ki(state.tick)
        self.pi_controller_speed_altitude.saturation_i(-15.0, 15.0)
        #print(f"i error: {self.pi_controller_speed_altitude.pid_i_result}")
        self.pi_controller_speed_altitude.evaluate_error_kd(state.tick)
        error_v_y = self.pi_controller_speed_altitude.evaluate_total_error()
        #print(f"error_v_y: {error_v_y}")



        # Spinta di sostentamento (Feed-Forward per la gravità) + correzione PID
        HOVER_THRUST = 9.81*9  # Valore di spinta necessario per sostenere il peso del drone
        thrust_cmd = HOVER_THRUST + error_v_y
        if thrust_cmd < HOVER_THRUST:
            thrust_cmd = HOVER_THRUST




        # 3. CONTROLLO POSIZIONE X (LATERALE) -> GENERA TARGET ROLL
        #print(f"state_x: {state.pos_x}")
        #print(f"target_x: {target_x}")
        #print(f"Errore px: {target_x - state.pos_x}")
        self.p_controller_x.evaluate_error(target_x, state.pos_x)
        #print(f"Errore px: {target_x - state.pos_x}")
        self.p_controller_x.evaluate_error_kp()
        self.p_controller_x.saturation_p(-20.0, 20.0)
        #print(f"px: {self.p_controller_x.pid_p_result}")
        error_p_x = self.p_controller_x.evaluate_total_error()

        self.pi_controller_speed_x.evaluate_error(error_p_x, state.vel_x)
        self.pi_controller_speed_x.evaluate_error_kp()
        self.pi_controller_speed_x.saturation_p(-17.5, 17.5)
        #print(f"vxp: {self.pi_controller_speed_x.pid_p_result}")
        self.pi_controller_speed_x.evaluate_error_ki(state.tick)
        self.pi_controller_speed_x.saturation_i(-0.01, 0.01)
        #print(f"vxi: {self.pi_controller_speed_x.pid_i_result}")
        self.pi_controller_speed_x.evaluate_error_kd(state.tick)
        raw_target_roll = self.pi_controller_speed_x.evaluate_total_error()
        #print(f"speed_target_roll = {raw_target_roll}")

        # Saturazione dell'angolo di Roll per evitare che il drone si capovolga (es. max +-30 gradi)
        MAX_ROLL_ANGLE = 30.0
        target_roll = max(min(raw_target_roll, MAX_ROLL_ANGLE), -MAX_ROLL_ANGLE)
        #print(f"target_roll: {target_roll}")



        # 4. CONTROLLO POSIZIONE Z (LONGITUDINALE) -> GENERA TARGET PITCH
        #print(f"state_y: {-state.pos_y}")
        #print(f"target: {target_z}")
        self.p_controller_z.evaluate_error(target_z, -state.pos_y)
        #print(f"Errore pz: {target_z- state.pos_y}")
        self.p_controller_z.evaluate_error_kp()
        #print(f"pz: {self.p_controller_z.pid_p_result}")
        self.p_controller_z.saturation_p(-20.0, 20.0)
        #self.p_controller_z.evaluate_error_kd(state.tick)
        error_p_z = -self.p_controller_z.evaluate_total_error()

        self.pi_controller_speed_z.evaluate_error(error_p_z, state.vel_y)
        self.pi_controller_speed_z.evaluate_error_kp()
        self.pi_controller_speed_z.saturation_p(-10.0, 10.0)
        self.pi_controller_speed_z.evaluate_error_ki(state.tick)
        self.pi_controller_speed_z.saturation_i(-0.1, 0.1)
        self.pi_controller_speed_z.evaluate_error_kd(state.tick)
        raw_target_pitch = -self.pi_controller_speed_z.evaluate_total_error()
        #print(f"speed_target_pitch = {raw_target_pitch}")
        # Saturazione dell'angolo di Pitch (es. max +-30 gradi)
        MAX_PITCH_ANGLE = 30.0
        target_pitch = max(min(raw_target_pitch, MAX_PITCH_ANGLE), -MAX_PITCH_ANGLE)
        #print(f"angle_target_pitch: {target_pitch}")



        # 5. CONTROLLO ANGOLARE (YAW)
        #print(f"angle_magne: {state.yaw_magnetometer}")
        self.p_controller_angular.evaluate_error(angle_target, state.yaw_magnetometer)
        self.p_controller_angular.evaluate_error_kp()
        self.p_controller_angular.saturation_p(-10.0, 10.0)
        error_angular = self.p_controller_angular.evaluate_total_error()
        #print(f"yaw_p_error: {error_angular}")




        #self.pi_controller_angular_speed.evaluate_error(error_angular, state.angular_velocity)
        #self.pi_controller_angular_speed.evaluate_error_kp()
        #self.pi_controller_angular_speed.saturation_p(-10.0, 10.0)
        #self.pi_controller_angular_speed.evaluate_error_ki(state.tick)
        #self.pi_controller_angular_speed.saturation_i(-10.0, 10.0)
        #target_yaw_rate = self.pi_controller_angular_speed.evaluate_total_error()

        #print(f"target_yaw: {target_yaw_rate}")
        #print(f"target_roll: {target_roll}, target_pitch: {target_pitch}")
        return thrust_cmd, target_roll, target_pitch, error_angular

    def inner_loop(self, state, target_thrust, target_roll, target_pitch, target_yaw_rate,
                   roll, pitch, yaw, speed_roll, speed_pitch, speed_yaw):
        """
        LOOP INTERNO (Target Assetto/Ratei -> Comandi Motori al Mixer)
        """

        # --- CONTROLLO YAW (IMBARDATA) ---
        #self.yaw_P.evaluate_error(target_yaw_rate, yaw)
        #self.yaw_P.evaluate_error_kp()
        #self.yaw_P.saturation_p(-1.0, 1.0)
        #error_p_yaw = self.yaw_P.evaluate_total_error()

        self.yaw_PI.evaluate_error(target_yaw_rate, speed_yaw)
        self.yaw_PI.evaluate_error_kp()
        self.yaw_PI.evaluate_error_ki(state.tick)
        self.yaw_PI.saturation_i(-20.0, 20.0)
        cmd_yaw = self.yaw_PI.evaluate_total_error()


        #target_roll = 0
        #target_pitch = 0
        #print(f"target_roll: {target_roll}, target_pitch: {target_pitch}, target_yaw_rate: {target_yaw_rate}")
        # --- CONTROLLO ROLL (ROLLIO / ASSE X) ---
        # 1. Confronto tra Angolo Target (dall'Outer Loop) e Angolo Attuale (dall'EKF)
        self.roll_P.evaluate_error(target_roll, roll)
        #print(f"errore roll: {target_roll-roll}")
        self.roll_P.evaluate_error_kp()
        self.roll_P.saturation_p(-30.0, 30.0)
        error_p_roll = self.roll_P.evaluate_total_error()
        #print("error_p_roll", error_p_roll)

        # 2. Controllo della velocità angolare di Roll
        self.roll_PI.evaluate_error(error_p_roll, speed_roll)
        self.roll_PI.evaluate_error_kp()
        self.roll_PI.saturation_p(-5.0, 5.0)
        #error_p_roll = self.roll_PI.evaluate_total_error()
        self.roll_PI.evaluate_error_ki(state.tick)
        self.roll_PI.saturation_i(-10, 10)
        self.roll_PI.evaluate_error_kd(state.tick)
        self.roll_PI.saturation_d(-5, 5)
        cmd_roll = self.roll_PI.evaluate_total_error()
        #print("cmd_roll", cmd_roll)





        # --- CONTROLLO PITCH (BECCHEGGIO / ASSE Z) ---
        # 1. Confronto tra Angolo Target (dall'Outer Loop) e Angolo Attuale (dall'EKF)
        #target_pitch = 0
        self.pitch_P.evaluate_error(target_pitch, pitch)
        #print(f"target pitch: {target_pitch}")
        #print(f"errore pitch: {target_pitch - pitch}")
        self.pitch_P.evaluate_error_kp()
        self.pitch_P.saturation_p(-30.0, 30.0)
        #cmd_pitch = self.pitch_P.evaluate_total_error()
        error_p_pitch = self.pitch_P.evaluate_total_error()
        #print("error_p_pitch", error_p_pitch)

        # 2. Controllo della velocità angolare di Pitch
        self.pitch_PI.evaluate_error(error_p_pitch, speed_pitch)
        self.pitch_PI.evaluate_error_kp()
        self.pitch_PI.saturation_p(-5.0, 5.0)
        pp = self.pitch_PI.pid_p_result
        #print("pp", pp)
        self.pitch_PI.evaluate_error_ki(state.tick)
        self.pitch_PI.saturation_i(-10.0, 10.0)
        #--------------------------------------------------
        #print("error_i_pitch: ", self.pitch_PI.pid_i_result)
        #-----------------------------------------------------
        self.pitch_PI.evaluate_error_kd(state.tick)
        #self.pitch_PI.saturation_d(-1.0, 1.0)
        self.pitch_PI.saturation_d(-5, 5)
        pd = self.pitch_PI.pid_d_result
        #print("pd", pd)
        cmd_pitch = self.pitch_PI.evaluate_total_error()

        self.plottino.fillListsPlot(self.time, target_roll, roll, "roll")
        print(f"t = {self.time}")



        # --- DISTRIBUZIONE AI MOTORI TRAMITE MIXER ---
        return mixer(target_thrust, cmd_yaw, -cmd_roll, cmd_pitch, self.nframe)
