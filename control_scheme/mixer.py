


def mixer(thrust_cmd, yaw_cmd, roll_cmd, pitch_cmd, nframe):

    secondi = nframe/60.0



    #thrust_cmd=9.81*1.5
    print(nframe)
    if secondi > 5:
        #thrust_cmd = 0
        pass
    #yaw_cmd=0
    #roll_cmd=0
    #pitch_cmd=0

    #print(f"roll_cmd: {roll_cmd}, pitch_cmd: {pitch_cmd}, yaw_cmd: {yaw_cmd}")

    w2 = thrust_cmd + yaw_cmd + roll_cmd + pitch_cmd
    w4 = thrust_cmd - yaw_cmd - roll_cmd + pitch_cmd
    w1 = thrust_cmd + yaw_cmd - roll_cmd - pitch_cmd
    w3 = thrust_cmd - yaw_cmd + roll_cmd - pitch_cmd

    w1 = max(0.0, w1)
    w2 = max(0.0, w2)
    w3 = max(0.0, w3)
    w4 = max(0.0, w4)
    #print(f"w2: {w2}, w4: {w4}, w1: {w1}, w3: {w3}")


    #print(thrust_cmd)
    #print(f"yaw_cmd: {yaw_cmd}")
    return nframe, w1, w2, w3, w4