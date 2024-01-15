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

# pid parameters
pid_dt = 1.0
pid_Kp = 0.0
pid_Ki = 0.0
pid_Kd = 0.0
pid_u_max = 0.0
kp_x0 = 0.0

# pid setpoint
pid_x0 = 0.0

# control system
ctrl_pid = None

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
