import sys
import time
import serial
import struct
import threading
import data_store.data_store_tx_rx as db
from gui.py_gui import *
from controller.PID import PID

port = 'COM5'
baudrate = 115200
timeout = 1.0

try:
	ser = serial.Serial(port, baudrate, timeout=timeout)
	print(f"INFO   : Serial port {port} opened")
except serial.SerialException as exception_e:
	print(f"ERROR  : Failed to open serial port {port}: {exception_e}")
	exit()

# if(ser.is_open):
# 	ser.set_buffer_size(8192,8192)
# 	print("INFO   : tx and rx buffer size set to 8192 bytes")

def init_data_store():
	db.thread_1_freq = 250.0
	db.thread_2_freq = 250.0
	db.thread_3_freq = 100.0

	db.thread_1_flag = True
	db.thread_2_flag = True
	db.thread_3_flag = True

	db.thread_1_time_last = time.time()
	db.thread_2_time_last = time.time()
	db.thread_3_time_last = time.time()

	db.start_stop_flag              = False
	db.open_loop_control_flag       = False
	db.closed_loop_control_flag     = False
	db.switch_2_angle_feedback_flag = False
	db.set_velocity_control         = False
	db.set_position_control         = False

	db.alpha     = 0.0
	db.switch_1  = False

	db.th0      = 0.0
	db.dth0_dt  = 0.0
	db.switch_2 = False

	db.ctrl_position = PID()
	db.ctrl_velocity = PID()

	db.pid_pos_dt    = 1.0
	db.pid_pos_Kp    = 0.0
	db.pid_pos_Ki    = 0.0
	db.pid_pos_Kd    = 0.0
	db.pid_pos_fc    = 0.0
	db.pid_pos_u_max = 0.0

	db.pid_vel_dt    = 1.0
	db.pid_vel_Kp    = 0.0
	db.pid_vel_Ki    = 0.0
	db.pid_vel_Kd    = 0.0
	db.pid_vel_fc    = 0.0
	db.pid_vel_u_max = 0.0

def byte4_2_int(byte4):
	bytes_obj = bytes(byte4)
	num_int = struct.unpack('<i', bytes_obj)[0]
	return num_int

def byte8_2_double(byte8):
	bytes_obj = bytes(byte8)
	num_double = struct.unpack('<d', bytes_obj)[0]
	return num_double

def parse_rx(packet):
	start         = packet[0]
	enc_count     = byte4_2_int(packet[1:5])
	enc_angle     = byte8_2_double(packet[5:13])
	enc_speed     = byte8_2_double(packet[13:21])
	motor_angle   = byte8_2_double(packet[21:29])
	motor_speed   = byte8_2_double(packet[29:37])
	motor_voltage = byte8_2_double(packet[37:45])
	motor_current = byte8_2_double(packet[45:53])
	crc           = packet[53]
	return enc_count, enc_angle, enc_speed, motor_angle, motor_speed, motor_voltage, motor_current

def receive_data():
	data_byte_arr = ser.read_all()
	for i in range(0,len(data_byte_arr)):
		data_byte = data_byte_arr[i]
		if(data_byte==33):
			db.rx_pkt = []
		db.rx_pkt.append(data_byte)
		if(len(db.rx_pkt)==54):
			db.rx_enc_count, db.rx_enc_angle, db.rx_enc_speed, db.rx_motor_angle, db.rx_motor_speed, db.rx_motor_voltage, db.rx_motor_current = parse_rx(db.rx_pkt)

def send_data(v_percent_double):
	start_byte_array = bytes(b'!')
	v_percent_byte_array = bytearray(struct.pack('d', v_percent_double))
	crc_byte_array = bytes(b'0')
	tx_pkt = start_byte_array + v_percent_byte_array + crc_byte_array
	ser.write(tx_pkt)

def communication_rx_main():
	print("INFO   : Thread-1 started (RX)")
	try:
		while(db.thread_1_flag):
			# print("INFO   : Thread-1 running (RX)")
			receive_data()

			# while ((time.time() - db.thread_1_time_last) < (1.0/db.thread_1_freq)):
			# 	pass
			# print("Thread-1 : ", 1.0/(time.time() - db.thread_1_time_last))
			# db.thread_1_time_last = time.time()
	except Exception as exception_e:
		print(f'ERROR  : Thread-1 {exception_e}')
	except KeyboardInterrupt:
		pass
	finally:
		print("INFO   : Thread-1 closed (RX)")

def communication_tx_main():
	print("INFO   : Thread-2 started (TX)")
	try:
		while(db.thread_2_flag):
			# print("INFO   : Thread-2 running (TX)")
			V_percent_tx = db.tx_v_percent
			send_data(V_percent_tx)

			while ((time.time() - db.thread_2_time_last) < (1.0/db.thread_2_freq)):
				pass
			# print("Thread-2 : ", 1.0/(time.time() - db.thread_2_time_last))
			db.thread_2_time_last = time.time()
	except Exception as exception_e:
		print(f'ERROR  : Thread-2 {exception_e}')
	except KeyboardInterrupt:
		pass
	finally:
		print("INFO   : Thread-2 closed (TX)")

def controller():
	print("INFO   : Thread-3 started (controller)")
	try: 
		# db.ctrl_pid.init(db.pid_dt, db.pid_Kp, db.pid_Ki, db.pid_Kd, db.pid_u_max)

		while (db.thread_3_flag):
			# print("INFO   : Thread-3 running (controller)")

			# u = db.ctrl_pid.calculate(db.pid_x0, db.rx_motor_speed)

			# db.tx_v_percent = u
			# print(db.pid_x0, db.rx_motor_speed) 
			# print(db.rx_enc_count, db.rx_enc_angle, db.rx_enc_speed, db.rx_motor_angle, db.rx_motor_speed, db.rx_motor_voltage, db.rx_motor_current)

			while ((time.time() - db.thread_3_time_last) < (1.0/db.thread_3_freq)):
				pass
			# print("Thread-3 : ", 1.0/(time.time() - db.thread_3_time_last))
			db.thread_3_time_last = time.time()
			# print(db.rx_enc_angle)
	except Exception as exception_e:
		print(f'ERROR  : Thread-3 {exception_e}')
	finally:
		print("INFO   : Thread-3 closed (Controller)")
		db.thread_1_flag = False
		db.thread_2_flag = False

green_color = "background-color: rgb(169, 224, 176);"
red_color   = "background-color: rgb(247, 179, 181);"

def update_ui_button_color(ui):
	if(db.start_stop_flag==True):
		ui.push_button_start_stop.setStyleSheet(green_color)
	else:
		ui.push_button_start_stop.setStyleSheet(red_color)

	if(db.open_loop_control_flag==True):
		ui.push_button_open_loop.setStyleSheet(green_color)
	else:
		ui.push_button_open_loop.setStyleSheet(red_color)

	if(db.closed_loop_control_flag==True):
		ui.push_button_closed_loop.setStyleSheet(green_color)
	else:
		ui.push_button_closed_loop.setStyleSheet(red_color)

	if(db.switch_2_angle_feedback_flag==True):
		ui.push_button_angle_feedback.setStyleSheet(green_color)
	else:
		ui.push_button_angle_feedback.setStyleSheet(red_color)

	if(db.set_velocity_control==True):
		ui.push_button_velocity_control.setStyleSheet(green_color)
	else:
		ui.push_button_velocity_control.setStyleSheet(red_color)

	if(db.set_position_control==True):
		ui.push_button_position_control.setStyleSheet(green_color)
	else:
		ui.push_button_position_control.setStyleSheet(red_color)

def fun_1(ui):
	try:
		db.start_stop_flag = not db.start_stop_flag
		if(db.start_stop_flag==False):
			db.open_loop_control_flag       = False
			db.closed_loop_control_flag     = False
			db.switch_2_angle_feedback_flag = False
			db.set_position_control         = False
			db.set_velocity_control         = False
		update_ui_button_color(ui)
	except Exception as e:
		print(e)

def fun_2(ui):
	try:
		if(db.start_stop_flag==True):
			db.open_loop_control_flag = not db.open_loop_control_flag
		if(db.open_loop_control_flag == True):
			db.closed_loop_control_flag     = False
			db.set_position_control         = False
			db.set_velocity_control         = False
		update_ui_button_color(ui)
	except Exception as e:
		print(e)

def fun_3(ui):
	try:
		db.tx_v_percent = float(ui.line_edit_v_percent.text())
	except Exception as e:
		print(e)

def fun_4(ui):
	try:
		if(db.start_stop_flag==True):
			db.closed_loop_control_flag = not db.closed_loop_control_flag
		if(db.closed_loop_control_flag==True):
			db.open_loop_control_flag       = False
		else:
			db.set_position_control         = False
			db.set_velocity_control         = False
		update_ui_button_color(ui)
	except Exception as e:
		print(e)

def fun_5(ui):
	try:
		db.alpha = float(ui.line_edit_alpha.text())
	except Exception as e:
		print(e)

def fun_6(ui):
	try:
		if(db.start_stop_flag==True):
			db.switch_2_angle_feedback_flag = not db.switch_2_angle_feedback_flag
		update_ui_button_color(ui)
	except Exception as e:
		print(e)

def fun_7(ui):
	try:
		db.th0 = float(ui.line_edit_th0.text())
	except Exception as e:
		print(e)

def fun_8(ui):
	try:
		db.dth0_dt = float(ui.line_edit_dth0_dt.text())
	except Exception as e:
		print(e)

def fun_9(ui):
	try:
		db.pid_pos_dt    = float(ui.line_edit_pid_position_dt.text())
		db.pid_pos_Kp    = float(ui.line_edit_pid_position_kp.text())
		db.pid_pos_Ki    = float(ui.line_edit_pid_position_ki.text())
		db.pid_pos_Kd    = float(ui.line_edit_pid_position_kd.text())
		db.pid_pos_fc    = float(ui.line_edit_pid_position_fc.text())
		db.pid_pos_u_max = float(ui.line_edit_pid_position_u_max.text())
		db.ctrl_position.set_param(db.pid_pos_dt, db.pid_pos_Kp, db.pid_pos_Ki, db.pid_pos_Kd, db.pid_pos_fc, db.pid_pos_u_max)
	except Exception as e:
		print(e)

def fun_10(ui):
	try:
		db.pid_vel_dt    = float(ui.line_edit_pid_velocity_dt.text())
		db.pid_vel_Kp    = float(ui.line_edit_pid_velocity_kp.text())
		db.pid_vel_Ki    = float(ui.line_edit_pid_velocity_ki.text())
		db.pid_vel_Kd    = float(ui.line_edit_pid_velocity_kd.text())
		db.pid_vel_fc    = float(ui.line_edit_pid_velocity_fc.text())
		db.pid_vel_u_max = float(ui.line_edit_pid_velocity_u_max.text())
		db.ctrl_velocity.set_param(db.pid_vel_dt, db.pid_vel_Kp, db.pid_vel_Ki, db.pid_vel_Kd, db.pid_vel_fc, db.pid_vel_u_max)
	except Exception as e:
		print(e)

def fun_11(ui):
	try:
		if(db.set_velocity_control==True):
			db.set_position_control = not db.set_position_control
		update_ui_button_color(ui)
	except Exception as e:
		print(e)

def fun_12(ui):
	try:
		if(db.closed_loop_control_flag==True):
			db.set_velocity_control = not db.set_velocity_control
		if(db.set_velocity_control==False):
			db.set_position_control         = False
		update_ui_button_color(ui)
	except Exception as e:
		print(e)

def gui_main():
	try:
		app = QtWidgets.QApplication(sys.argv)
		MainWindow = QtWidgets.QMainWindow()
		ui = Ui_MainWindow()
		ui.setupUi(MainWindow)
		ui.push_button_start_stop.clicked.connect(lambda: fun_1(ui))
		ui.push_button_open_loop.clicked.connect(lambda: fun_2(ui))
		ui.push_button_set_v_percent.clicked.connect(lambda: fun_3(ui))
		ui.push_button_closed_loop.clicked.connect(lambda: fun_4(ui))
		ui.push_button_set_alpha.clicked.connect(lambda: fun_5(ui))
		ui.push_button_angle_feedback.clicked.connect(lambda: fun_6(ui))
		ui.push_button_set_th0.clicked.connect(lambda: fun_7(ui))
		ui.push_button_set_dth0_dt.clicked.connect(lambda: fun_8(ui))
		ui.push_button_set_position_pid_param.clicked.connect(lambda: fun_9(ui))
		ui.push_button_set_velocity_pid_param.clicked.connect(lambda: fun_10(ui))
		ui.push_button_position_control.clicked.connect(lambda: fun_11(ui))
		ui.push_button_velocity_control.clicked.connect(lambda: fun_12(ui))

		MainWindow.show()
		print("INFO   : Thread-4 started (GUI)")
		sys.exit(app.exec_())
	except Exception as exception_e:
		print(f'ERROR  : Thread-3 {exception_e}')
	finally:
		print("INFO   : Thread-4 closed (GUI)")
		db.thread_1_flag = False
		db.thread_2_flag = False
		db.thread_3_flag = False
		ser.close()
		print(f"INFO   : Serial port {port} closed")

if __name__=="__main__":
	init_data_store()
	
	thread_1 = threading.Thread(target=communication_rx_main)
	thread_2 = threading.Thread(target=communication_tx_main)
	thread_3 = threading.Thread(target=controller)

	thread_1.start()
	thread_2.start()
	thread_3.start()

	gui_main()

	thread_1.join()
	thread_2.join()
	thread_3.join()