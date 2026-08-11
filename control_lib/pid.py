from idlelib.configdialog import KeysPage

from numpy import dtypes


class PID:
    def __init__(self, Kp, Ki, Kd, target):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = target
        self.error = 0
        self.previousError = 0
        self.pid_p_result = 0
        self.pid_i_result = 0
        self.pid_d_result = 0


    def reset_integral(self):
        self.pid_i_result = 0

    def evaluate_error(self, target, actual_value):
        self.previousError = self.error
        self.error = target - actual_value

    def evaluate_error_kp(self):
        self.pid_p_result = self.Kp*self.error

    def saturation_p(self, min, max):
        if self.pid_p_result > max:
            self.pid_p_result = max
        elif self.pid_p_result < min:
            self.pid_p_result = min


    def evaluate_error_ki(self, dt):
        self.pid_i_result += self.Ki * self.error*dt


    def saturation_i(self, min, max):
        if self.pid_i_result > max:
            self.pid_i_result = max
        elif self.pid_i_result < min:
            self.pid_i_result = min

    def evaluate_error_kd(self, dt):
        self.pid_d_result = self.Kd * (self.error - self.previousError)/dt

    def saturation_d(self, min, max):
        if self.pid_d_result > max:
            self.pid_d_result = max
        elif self.pid_d_result < min:
            self.pid_d_result = min

    def evaluate_total_error(self):
       # print(f"pid_p, {self.pid_p_result}")
        #print(f"pid_int, {self.pid_i_result}")
        total_error = self.pid_p_result + self.pid_i_result + self.pid_d_result
        return total_error






