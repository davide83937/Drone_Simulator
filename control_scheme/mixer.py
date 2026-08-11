def mixer(thrust_cmd, yaw_cmd, roll_cmd, pitch_cmd):
    w1 = thrust_cmd - yaw_cmd + roll_cmd + pitch_cmd
    w2 = thrust_cmd +yaw_cmd - roll_cmd + pitch_cmd
    w3 = thrust_cmd - yaw_cmd - roll_cmd - pitch_cmd
    w4 = thrust_cmd + yaw_cmd + roll_cmd - pitch_cmd

    w1 = max(0.0, w1)
    w2 = max(0.0, w2)
    w3 = max(0.0, w3)
    w4 = max(0.0, w4)
    return w1, w2, w3, w4