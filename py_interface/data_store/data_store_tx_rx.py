# rx_pkt buffer
rx_pkt = []

# rx data
rx_enc_count     = 0
rx_enc_angle     = 0.0
rx_enc_speed     = 0.0
rx_motor_angle   = 0.0
rx_motor_speed   = 0.0
rx_motor_voltage = 0.0
rx_motor_current = 0.0

# tx data
tx_v_percent = 0.0

# gui falags
start_stop_flag = False
open_loop_control_flag = False
closed_loop_control_flag = False
switch_2_angle_feedback_flag = False
set_velocity_control = False
set_position_control = False

# feedback
dth_dt    = 0.0
th        = 0.0
alpha     = 0.0
switch_1  = False

# setpoint
th0      = 0.0
dth0_dt  = 0.0
switch_2 = False

# control system
ctrl_position = None
ctrl_velocity = None

pid_pos_dt    = 1.0
pid_pos_Kp    = 0.0
pid_pos_Ki    = 0.0
pid_pos_Kd    = 0.0
pid_pos_fc    = 0.0
pid_pos_u_max = 0.0

pid_vel_dt    = 1.0
pid_vel_Kp    = 0.0
pid_vel_Ki    = 0.0
pid_vel_Kd    = 0.0
pid_vel_fc    = 0.0
pid_vel_u_max = 0.0

# multithreading
thread_1_freq = 1.0
thread_2_freq = 1.0
thread_3_freq = 1.0

thread_1_flag = None
thread_2_flag = None
thread_3_flag = None

thread_1_time_last = None
thread_2_time_last = None
thread_3_time_last = None
