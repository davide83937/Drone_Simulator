from abc import ABC, abstractmethod
from control_lib import VirtualRobot
from control_lib import pid
from my_math import myMath
from control_scheme.state import State
from control_scheme.mixer import mixer

class controlScheme(ABC):
    @abstractmethod
    def __init__(self):
        pass

    @abstractmethod
    def start(self, **kwargs):
        pass

    @abstractmethod
    def inner_loop(self, error_vy, error_vx, error_vz, error_angular_v):
        pass

    @abstractmethod
    def outer_loop(self, delta_t, state):
        pass


class droneControlScheme(controlScheme):
    def __init__(self):
        self.virtualRobotAltitude = VirtualRobot.StraightLine2DMotion(20, 2, 2)
        self.p_controller_z = pid.PID(0.3, 0, 0, 30)
        self.pi_controller_speed_z = pid.PID(0.1, 0.03, 0, 30)

        self.virtualRobotXY = VirtualRobot.StraightLine2DMotion(20, 2, 2)
        self.p_controller_x = pid.PID(0.05, 0, 0, 30)
        self.pi_controller_speed_x = pid.PID(0.05, 0.01, 0, 30)

        self.p_controller_y = pid.PID(0.05, 0, 0, 30)
        self.pi_controller_speed_y = pid.PID(0.05, 0.01, 0, 30)

        self.virtualRobotAngular = VirtualRobot.StraightLineMotion(0.5, 0.2, 0.2, 0)
        self.p_controller_angular = pid.PID(0.05, 0, 0, 30)
        self.pi_controller_angular_speed = pid.PID(0.05, 0.01, 0, 30)

        self.yaw_P = pid.PID(0.005, 0, 0, 0)
        self.yaw_PI = pid.PID(0.005, 0.001, 0, 0)

        self.roll_P = pid.PID(0.05, 0, 0, 0)
        self.roll_PI = pid.PID(0.005, 0.001, 0, 0)

        self.pitch_P = pid.PID(0.005, 0, 0, 0)
        self.pitch_PI = pid.PID(0.005, 0.001, 0, 0)



    def start(self, **kwargs):
        self.virtualRobotAltitude.start_motion((0, kwargs['y_start']), (0, kwargs['y_end']))
        self.virtualRobotXY.start_motion((kwargs['z_start'], kwargs['x_start']), (kwargs['z_end'], kwargs['x_end']))
        self.virtualRobotAngular.start_motion([kwargs['ang_start']], [kwargs['ang_end']])

    def inner_loop(self, error_vy, error_vx, error_vz, error_angular_v, roll, pitch, yaw,
        speed_roll, speed_pitch, speed_yaw):

        self.yaw_P.evaluate_error(error_angular_v, yaw)
        self.yaw_P.evaluate_error_kp()
        error_p_yaw = self.yaw_P.evaluate_total_error()
        self.yaw_PI.evaluate_error(error_p_yaw, speed_yaw)
        self.yaw_PI.evaluate_error_kp()
        self.yaw_PI.evaluate_error_ki()
        self.yaw_PI.saturation_i(-5.0, 5.0)
        error_v_yaw = self.yaw_PI.evaluate_total_error()

        # Il Roll (inclinazione laterale) deve correggere l'errore lungo X
        self.roll_P.evaluate_error(error_vx, roll)
        self.roll_P.evaluate_error_kp()
        error_p_roll = self.roll_P.evaluate_total_error()
        self.roll_PI.evaluate_error(error_p_roll, speed_roll)
        self.roll_PI.evaluate_error_kp()
        self.roll_PI.evaluate_error_ki()
        self.roll_PI.saturation_i(-5.0, 5.0)
        error_v_roll = self.roll_PI.evaluate_total_error()

        # Il Pitch (inclinazione avanti/indietro) deve correggere l'errore lungo Z
        self.pitch_P.evaluate_error(error_vz, pitch)
        self.pitch_P.evaluate_error_kp()
        error_p_pitch = self.pitch_P.evaluate_total_error()
        self.pitch_PI.evaluate_error(error_p_pitch, speed_pitch)
        self.pitch_PI.evaluate_error_kp()
        self.pitch_PI.evaluate_error_ki()
        self.pitch_PI.saturation_i(-5.0, 5.0)
        error_v_pitch = self.pitch_PI.evaluate_total_error()

        return mixer(error_vy, error_v_yaw, error_v_roll, error_v_pitch)

    def outer_loop(self, delta_t, state):

        #VIRTUAL ROBOTS
        n, target_y = self.virtualRobotAltitude.evaluate(delta_t)
        target_z, target_x = self.virtualRobotXY.evaluate(delta_t)
        angle = self.virtualRobotAngular.evaluate(delta_t)
        angle_target = angle[0]

        #CONTROLLO ALTITUDINE
        self.p_controller_z.evaluate_error(target_y, state.pos_y)
        self.p_controller_z.evaluate_error_kp()
        error_py = self.p_controller_z.evaluate_total_error()
        self.pi_controller_speed_z.evaluate_error(error_py, state.vel_y)
        self.pi_controller_speed_z.evaluate_error_kp()
        self.pi_controller_speed_z.evaluate_error_ki()
        self.pi_controller_speed_z.saturation_i(-20.0, 20.0)
        print(self.pi_controller_speed_z.pid_i_result)
        error_vy = self.pi_controller_speed_z.evaluate_total_error()

        #CONTROLLO ZX
        self.p_controller_x.evaluate_error(target_z, state.pos_z)
        self.p_controller_x.evaluate_error_kp()
        error_pz = self.p_controller_x.evaluate_total_error()
        self.pi_controller_speed_x.evaluate_error(error_pz, state.vel_z)
        self.pi_controller_speed_x.evaluate_error_kp()
        self.pi_controller_speed_x.evaluate_error_ki()
        self.pi_controller_speed_x.saturation_i(-20.0, 20.0)
        error_vz = self.pi_controller_speed_x.evaluate_total_error()

        self.p_controller_y.evaluate_error(target_x, state.pos_x)
        self.p_controller_y.evaluate_error_kp()
        error_px = self.p_controller_y.evaluate_total_error()
        self.pi_controller_speed_y.evaluate_error(error_px, state.vel_x)
        self.pi_controller_speed_y.evaluate_error_kp()
        self.pi_controller_speed_y.evaluate_error_ki()
        self.pi_controller_speed_y.saturation_i(-20.0, 20.0)
        error_vx = self.pi_controller_speed_y.evaluate_total_error()


        #CONTROLLO ANGOLARE
        self.p_controller_angular.evaluate_error(angle_target, state.yaw_magnetometer)
        self.p_controller_angular.evaluate_error_kp()
        error_angular = self.p_controller_angular.evaluate_total_error()
        self.pi_controller_angular_speed.evaluate_error(error_angular, state.angular_velocity)
        self.pi_controller_angular_speed.evaluate_error_kp()
        self.pi_controller_angular_speed.evaluate_error_ki()
        self.pi_controller_angular_speed.saturation_i(-30.0, 30.0)
        error_angular_v = self.pi_controller_angular_speed.evaluate_total_error()

        return error_vy, error_vx, error_vz, error_angular_v






