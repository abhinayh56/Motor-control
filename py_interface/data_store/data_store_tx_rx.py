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

#gui data
button_status_power               = False
button_status_open_loop_control   = False
button_status_torque_control      = False
button_status_velocity_control_m1 = False
button_status_velocity_control_m2 = False
button_status_position_cotrol_m1  = False
button_status_position_cotrol_m2  = False
button_status_position_cotrol_m3  = False

v_percent_open_loop = 0.0

x0_torque = 0.0
kff_torque = 0.0
kp_torque = 0.0
ki_torque = 0.0
kd_torque = 0.0
fc_torque = 9999999999
i_max_torque = 0.0
u_max_torque = 0.0

x0_velocity_m1 = 0.0
kff_velocity_m1 = 0.0
kp_velocity_m1 = 0.0
ki_velocity_m1 = 0.0
kd_velocity_m1 = 0.0
fc_velocity_m1 = 9999999999
i_max_velocity_m1 = 0.0
u_max_velocity_m1 = 0.0

x01_velocity_m2 = 0.0
kff1_velocity_m2 = 0.0
kp1_velocity_m2 = 0.0
ki1_velocity_m2 = 0.0
kd1_velocity_m2 = 0.0
fc1_velocity_m2 = 9999999999
i_max1_velocity_m2 = 0.0
u_max1_velocity_m2 = 0.0

x02_velocity_m2 = 0.0
kff2_velocity_m2 = 0.0
kp2_velocity_m2 = 0.0
ki2_velocity_m2 = 0.0
kd2_velocity_m2 = 0.0
fc2_velocity_m2 = 9999999999
i_max2_velocity_m2 = 0.0
u_max2_velocity_m2 = 0.0

x0_position_m1 = 0.0
kff_position_m1 = 0.0
kp_position_m1 = 0.0
ki_position_m1 = 0.0
kd_position_m1 = 0.0
fc_position_m1 = 9999999999
i_max_position_m1 = 0.0
u_max_position_m1 = 0.0

x01_position_m2 = 0.0
kff1_position_m2 = 0.0
kp1_position_m2 = 0.0
ki1_position_m2 = 0.0
kd1_position_m2 = 0.0
fc1_position_m2 = 9999999999
i_max1_position_m2 = 0.0
u_max1_position_m2 = 0.0

x02_position_m2 = 0.0
kff2_position_m2 = 0.0
kp2_position_m2 = 0.0
ki2_position_m2 = 0.0
kd2_position_m2 = 0.0
fc2_position_m2 = 9999999999
i_max2_position_m2 = 0.0
u_max2_position_m2 = 0.0

x01_position_m3 = 0.0
kff1_position_m3 = 0.0
kp1_position_m3 = 0.0
ki1_position_m3 = 0.0
kd1_position_m3 = 0.0
fc1_position_m3 = 9999999999
i_max1_position_m3 = 0.0
u_max1_position_m3 = 0.0

x02_position_m3 = 0.0
kff2_position_m3 = 0.0
kp2_position_m3 = 0.0
ki2_position_m3 = 0.0
kd2_position_m3 = 0.0
fc2_position_m3 = 9999999999
i_max2_position_m3 = 0.0
u_max2_position_m3 = 0.0

x03_position_m3 = 0.0
kff3_position_m3 = 0.0
kp3_position_m3 = 0.0
ki3_position_m3 = 0.0
kd3_position_m3 = 0.0
fc3_position_m3 = 9999999999
i_max3_position_m3 = 0.0
u_max3_position_m3 = 0.0