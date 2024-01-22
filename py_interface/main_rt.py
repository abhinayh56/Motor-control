import sys
import time
import serial
import struct
import threading
import data_store.data_store_tx_rx as db
from gui.py_gui_2 import *
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

	db.th0      = 0.0
	db.dth0_dt  = 0.0

	db.ctrl_position = PID()
	db.ctrl_velocity = PID()

	db.pid_pos_dt    = 1.0
	db.pid_pos_Kp    = 0.0
	db.pid_pos_Ki    = 0.0
	db.pid_pos_Kd    = 0.0
	db.pid_pos_fc    = 9999999999999.0
	db.pid_pos_u_max = 0.0
	db.ctrl_position.set_param(db.pid_pos_dt, db.pid_pos_Kp, db.pid_pos_Ki, db.pid_pos_Kd, db.pid_pos_fc, db.pid_pos_u_max)

	db.pid_vel_dt    = 1.0
	db.pid_vel_Kp    = 0.0
	db.pid_vel_Ki    = 0.0
	db.pid_vel_Kd    = 0.0
	db.pid_vel_fc    = 9999999999999.0
	db.pid_vel_u_max = 0.0
	db.ctrl_velocity.set_param(db.pid_vel_dt, db.pid_vel_Kp, db.pid_vel_Ki, db.pid_vel_Kd, db.pid_vel_fc, db.pid_vel_u_max)

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
		while (db.thread_3_flag):
			# print("INFO   : Thread-3 running (controller)")
			
			if(db.start_stop_flag==True):
				if(db.closed_loop_control_flag==True):
					if(db.set_position_control==True):
						db.dth0_dt = db.ctrl_position.calculate(db.th0, db.th)

					if(db.set_velocity_control==True):
						db.tx_v_percent = db.ctrl_velocity.calculate(db.dth0_dt, db.dth_dt)
					else:
						db.tx_v_percent = 0.0
			else:
				db.tx_v_percent = 0.0
						
			while ((time.time() - db.thread_3_time_last) < (1.0/db.thread_3_freq)):
				pass
			# print("Thread-3 : ", 1.0/(time.time() - db.thread_3_time_last))
			db.thread_3_time_last = time.time()
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

def update_gui_fields(ui):
	_translate = QtCore.QCoreApplication.translate
	ui.label_e_stop.setText(_translate("MainWindow", str(db.start_stop_flag)))
	ui.label_open_loop_control.setText(_translate("MainWindow", str(db.open_loop_control_flag)))
	ui.label_close_loop_control.setText(_translate("MainWindow", str(db.closed_loop_control_flag)))
	ui.label_position_control.setText(_translate("MainWindow", str(db.set_position_control)))
	ui.label_velocity_control.setText(_translate("MainWindow", str(db.set_velocity_control)))
	ui.label_switch2anglefeed.setText(_translate("MainWindow", str(db.switch_2_angle_feedback_flag)))
	
	ui.label_pos_th0.setText(_translate("MainWindow", str(db.th0)))

	ui.label_pos_kp.setText(_translate("MainWindow", str(db.pid_pos_Kp)))
	ui.label_pos_ki.setText(_translate("MainWindow", str(db.pid_pos_Ki)))
	ui.label_pos_kd.setText(_translate("MainWindow", str(db.pid_pos_Kd)))
	ui.label_pos_fc.setText(_translate("MainWindow", str(db.pid_pos_fc)))
	ui.label_pos_u_max.setText(_translate("MainWindow", str(db.pid_pos_u_max)))

	ui.label_dth0_dt.setText(_translate("MainWindow", str(db.dth0_dt)))

	ui.label_vel_kp.setText(_translate("MainWindow", str(db.pid_vel_Kp)))
	ui.label_vel_ki.setText(_translate("MainWindow", str(db.pid_vel_Ki)))
	ui.label_vel_kd.setText(_translate("MainWindow", str(db.pid_vel_Kd)))
	ui.label_vel_fc.setText(_translate("MainWindow", str(db.pid_vel_fc)))
	ui.label_vel_u_max.setText(_translate("MainWindow", str(db.pid_vel_u_max)))

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
		update_gui_fields(ui)
	except Exception as e:
		print(e)

def gui_main():
	try:
		app = QtWidgets.QApplication(sys.argv)
		MainWindow = QtWidgets.QMainWindow()
		ui = Ui_MainWindow()
		ui.setupUi(MainWindow)
		ui.button_power_on_off.clicked.connect(lambda: fun_1(ui))
		ui.button_start_stop_open_loop_control.clicked.connect(lambda: fun_2(ui))
		ui.button_start_stop_torque_control.clicked.connect(lambda: fun_3(ui))
		ui.button_start_stop_velocity_control_1.clicked.connect(lambda: fun_4(ui))
		ui.button_start_stop_velocity_control_2.clicked.connect(lambda: fun_5(ui))
		ui.button_start_stop_position_control_1.clicked.connect(lambda: fun_6(ui))
		ui.button_start_stop_position_control_2.clicked.connect(lambda: fun_7(ui))
		ui.button_start_stop_position_control_3.clicked.connect(lambda: fun_8(ui))



		update_gui_fields(ui)

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