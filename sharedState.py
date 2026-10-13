from control_scheme import state

# Istanza unica condivisa
current_state = state.State(
    tick=0,
    roll_acc=0.0,
    pitch_acc=0.0,
    yaw_magnetometer=0.0,
    angular_velocity=0.0,
    pos_x=0.0,
    pos_y=0.0,
    pos_z=0.0,
    vel_x=0.0,
    vel_y=0.0,
    vel_z=0.0
)