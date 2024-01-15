import sys
import time
import serial
import struct
import threading
import data_store.data_store_tx_rx as db
from gui.py_gui import *
from controller.PID import PID

port = '/dev/ttyACM0'
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

	db.ctrl_pid = PID()

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
		db.ctrl_pid.init(db.pid_dt, db.pid_Kp, db.pid_Ki, db.pid_Kd, db.pid_u_max)

		while (db.thread_3_flag):
			# print("INFO   : Thread-3 running (controller)")

			# u = db.ctrl_pid.calculate(db.pid_x0, db.rx_motor_angle)
			u1 = db.kp_x0*(db.pid_x0 - db.rx_motor_angle)
			u = db.ctrl_pid.calculate(u1, db.rx_motor_speed)
			db.tx_v_percent = u
			print(db.pid_x0, db.rx_motor_angle)
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

def btn_fun(ui):
	try:
		db.pid_dt    = float(ui.lineEdit.text())
		db.pid_Kp    = float(ui.lineEdit_2.text())
		db.pid_Ki    = float(ui.lineEdit_3.text())
		db.pid_Kd    = float(ui.lineEdit_4.text())
		db.pid_u_max = float(ui.lineEdit_5.text())
		db.ctrl_pid.set_param(db.pid_dt, db.pid_Kp, db.pid_Ki, db.pid_Kd, db.pid_u_max)
	except:
		pass

def btn_fun_2(ui):
	try:
		if(ui.lineEdit_6.text()):
			db.pid_x0 = float(ui.lineEdit_6.text())
			db.kp_x0 = float(ui.lineEdit_7.text())
	except:
		pass

def gui_main():
	try:
		app = QtWidgets.QApplication(sys.argv)
		MainWindow = QtWidgets.QMainWindow()
		ui = Ui_MainWindow()
		ui.setupUi(MainWindow)
		ui.pushButton.clicked.connect(lambda: btn_fun(ui))
		ui.pushButton_2.clicked.connect(lambda: btn_fun_2(ui))
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