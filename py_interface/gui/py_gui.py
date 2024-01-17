# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'py_gui.ui'
#
# Created by: PyQt5 UI code generator 5.15.9
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(564, 570)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(MainWindow.sizePolicy().hasHeightForWidth())
        MainWindow.setSizePolicy(sizePolicy)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("icon.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        MainWindow.setWindowIcon(icon)
        MainWindow.setStyleSheet("background-color: rgb(236, 236, 236);\n"
"background-color: rgb(221, 221, 221);")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout_9 = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout_9.setObjectName("gridLayout_9")
        self.gridLayout_7 = QtWidgets.QGridLayout()
        self.gridLayout_7.setObjectName("gridLayout_7")
        self.push_button_start_stop = QtWidgets.QPushButton(self.centralwidget)
        self.push_button_start_stop.setStyleSheet("background-color: rgb(247, 179, 181);")
        self.push_button_start_stop.setObjectName("push_button_start_stop")
        self.gridLayout_7.addWidget(self.push_button_start_stop, 0, 0, 1, 1)
        self.push_button_open_loop = QtWidgets.QPushButton(self.centralwidget)
        self.push_button_open_loop.setStyleSheet("background-color: rgb(247, 179, 181);")
        self.push_button_open_loop.setObjectName("push_button_open_loop")
        self.gridLayout_7.addWidget(self.push_button_open_loop, 1, 0, 1, 1)
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")
        self.label_20 = QtWidgets.QLabel(self.centralwidget)
        self.label_20.setObjectName("label_20")
        self.gridLayout.addWidget(self.label_20, 0, 0, 1, 1)
        self.line_edit_v_percent = QtWidgets.QLineEdit(self.centralwidget)
        self.line_edit_v_percent.setObjectName("line_edit_v_percent")
        self.gridLayout.addWidget(self.line_edit_v_percent, 0, 1, 1, 1)
        self.push_button_set_v_percent = QtWidgets.QPushButton(self.centralwidget)
        self.push_button_set_v_percent.setStyleSheet("background-color: rgb(144, 224, 237);")
        self.push_button_set_v_percent.setObjectName("push_button_set_v_percent")
        self.gridLayout.addWidget(self.push_button_set_v_percent, 0, 2, 1, 1)
        self.gridLayout_7.addLayout(self.gridLayout, 2, 0, 1, 1)
        self.push_button_closed_loop = QtWidgets.QPushButton(self.centralwidget)
        self.push_button_closed_loop.setStyleSheet("background-color: rgb(247, 179, 181);")
        self.push_button_closed_loop.setObjectName("push_button_closed_loop")
        self.gridLayout_7.addWidget(self.push_button_closed_loop, 3, 0, 1, 1)
        self.label_4 = QtWidgets.QLabel(self.centralwidget)
        self.label_4.setObjectName("label_4")
        self.gridLayout_7.addWidget(self.label_4, 4, 0, 1, 1)
        self.gridLayout_2 = QtWidgets.QGridLayout()
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.label_3 = QtWidgets.QLabel(self.centralwidget)
        self.label_3.setObjectName("label_3")
        self.gridLayout_2.addWidget(self.label_3, 0, 0, 1, 1)
        self.line_edit_alpha = QtWidgets.QLineEdit(self.centralwidget)
        self.line_edit_alpha.setObjectName("line_edit_alpha")
        self.gridLayout_2.addWidget(self.line_edit_alpha, 0, 1, 1, 1)
        self.push_button_set_alpha = QtWidgets.QPushButton(self.centralwidget)
        self.push_button_set_alpha.setStyleSheet("background-color: rgb(144, 224, 237);")
        self.push_button_set_alpha.setObjectName("push_button_set_alpha")
        self.gridLayout_2.addWidget(self.push_button_set_alpha, 0, 2, 1, 1)
        self.gridLayout_7.addLayout(self.gridLayout_2, 5, 0, 1, 1)
        self.push_button_angle_feedback = QtWidgets.QPushButton(self.centralwidget)
        self.push_button_angle_feedback.setStyleSheet("background-color: rgb(247, 179, 181);")
        self.push_button_angle_feedback.setObjectName("push_button_angle_feedback")
        self.gridLayout_7.addWidget(self.push_button_angle_feedback, 6, 0, 1, 1)
        self.label_19 = QtWidgets.QLabel(self.centralwidget)
        self.label_19.setObjectName("label_19")
        self.gridLayout_7.addWidget(self.label_19, 7, 0, 1, 1)
        self.gridLayout_3 = QtWidgets.QGridLayout()
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setObjectName("label")
        self.gridLayout_3.addWidget(self.label, 0, 0, 1, 1)
        self.line_edit_th0 = QtWidgets.QLineEdit(self.centralwidget)
        self.line_edit_th0.setObjectName("line_edit_th0")
        self.gridLayout_3.addWidget(self.line_edit_th0, 0, 1, 1, 1)
        self.push_button_set_th0 = QtWidgets.QPushButton(self.centralwidget)
        self.push_button_set_th0.setStyleSheet("background-color: rgb(144, 224, 237);")
        self.push_button_set_th0.setObjectName("push_button_set_th0")
        self.gridLayout_3.addWidget(self.push_button_set_th0, 0, 2, 1, 1)
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_2.setObjectName("label_2")
        self.gridLayout_3.addWidget(self.label_2, 1, 0, 1, 1)
        self.line_edit_dth0_dt = QtWidgets.QLineEdit(self.centralwidget)
        self.line_edit_dth0_dt.setObjectName("line_edit_dth0_dt")
        self.gridLayout_3.addWidget(self.line_edit_dth0_dt, 1, 1, 1, 1)
        self.push_button_set_dth0_dt = QtWidgets.QPushButton(self.centralwidget)
        self.push_button_set_dth0_dt.setStyleSheet("background-color: rgb(144, 224, 237);")
        self.push_button_set_dth0_dt.setObjectName("push_button_set_dth0_dt")
        self.gridLayout_3.addWidget(self.push_button_set_dth0_dt, 1, 2, 1, 1)
        self.gridLayout_7.addLayout(self.gridLayout_3, 8, 0, 1, 1)
        self.gridLayout_4 = QtWidgets.QGridLayout()
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.label_5 = QtWidgets.QLabel(self.centralwidget)
        self.label_5.setObjectName("label_5")
        self.gridLayout_4.addWidget(self.label_5, 0, 0, 1, 1)
        self.label_6 = QtWidgets.QLabel(self.centralwidget)
        self.label_6.setObjectName("label_6")
        self.gridLayout_4.addWidget(self.label_6, 0, 1, 1, 1)
        self.gridLayout_7.addLayout(self.gridLayout_4, 9, 0, 1, 1)
        self.gridLayout_5 = QtWidgets.QGridLayout()
        self.gridLayout_5.setObjectName("gridLayout_5")
        self.label_7 = QtWidgets.QLabel(self.centralwidget)
        self.label_7.setObjectName("label_7")
        self.gridLayout_5.addWidget(self.label_7, 0, 0, 1, 1)
        self.line_edit_pid_position_dt = QtWidgets.QLineEdit(self.centralwidget)
        self.line_edit_pid_position_dt.setObjectName("line_edit_pid_position_dt")
        self.gridLayout_5.addWidget(self.line_edit_pid_position_dt, 0, 1, 1, 1)
        self.label_13 = QtWidgets.QLabel(self.centralwidget)
        self.label_13.setObjectName("label_13")
        self.gridLayout_5.addWidget(self.label_13, 0, 2, 1, 1)
        self.line_edit_pid_velocity_dt = QtWidgets.QLineEdit(self.centralwidget)
        self.line_edit_pid_velocity_dt.setObjectName("line_edit_pid_velocity_dt")
        self.gridLayout_5.addWidget(self.line_edit_pid_velocity_dt, 0, 3, 1, 1)
        self.label_8 = QtWidgets.QLabel(self.centralwidget)
        self.label_8.setObjectName("label_8")
        self.gridLayout_5.addWidget(self.label_8, 1, 0, 1, 1)
        self.line_edit_pid_position_kp = QtWidgets.QLineEdit(self.centralwidget)
        self.line_edit_pid_position_kp.setObjectName("line_edit_pid_position_kp")
        self.gridLayout_5.addWidget(self.line_edit_pid_position_kp, 1, 1, 1, 1)
        self.label_14 = QtWidgets.QLabel(self.centralwidget)
        self.label_14.setObjectName("label_14")
        self.gridLayout_5.addWidget(self.label_14, 1, 2, 1, 1)
        self.line_edit_pid_velocity_kp = QtWidgets.QLineEdit(self.centralwidget)
        self.line_edit_pid_velocity_kp.setObjectName("line_edit_pid_velocity_kp")
        self.gridLayout_5.addWidget(self.line_edit_pid_velocity_kp, 1, 3, 1, 1)
        self.label_9 = QtWidgets.QLabel(self.centralwidget)
        self.label_9.setObjectName("label_9")
        self.gridLayout_5.addWidget(self.label_9, 2, 0, 1, 1)
        self.line_edit_pid_position_ki = QtWidgets.QLineEdit(self.centralwidget)
        self.line_edit_pid_position_ki.setObjectName("line_edit_pid_position_ki")
        self.gridLayout_5.addWidget(self.line_edit_pid_position_ki, 2, 1, 1, 1)
        self.label_15 = QtWidgets.QLabel(self.centralwidget)
        self.label_15.setObjectName("label_15")
        self.gridLayout_5.addWidget(self.label_15, 2, 2, 1, 1)
        self.line_edit_pid_velocity_ki = QtWidgets.QLineEdit(self.centralwidget)
        self.line_edit_pid_velocity_ki.setObjectName("line_edit_pid_velocity_ki")
        self.gridLayout_5.addWidget(self.line_edit_pid_velocity_ki, 2, 3, 1, 1)
        self.label_10 = QtWidgets.QLabel(self.centralwidget)
        self.label_10.setObjectName("label_10")
        self.gridLayout_5.addWidget(self.label_10, 3, 0, 1, 1)
        self.line_edit_pid_position_kd = QtWidgets.QLineEdit(self.centralwidget)
        self.line_edit_pid_position_kd.setObjectName("line_edit_pid_position_kd")
        self.gridLayout_5.addWidget(self.line_edit_pid_position_kd, 3, 1, 1, 1)
        self.label_16 = QtWidgets.QLabel(self.centralwidget)
        self.label_16.setObjectName("label_16")
        self.gridLayout_5.addWidget(self.label_16, 3, 2, 1, 1)
        self.line_edit_pid_velocity_kd = QtWidgets.QLineEdit(self.centralwidget)
        self.line_edit_pid_velocity_kd.setObjectName("line_edit_pid_velocity_kd")
        self.gridLayout_5.addWidget(self.line_edit_pid_velocity_kd, 3, 3, 1, 1)
        self.label_11 = QtWidgets.QLabel(self.centralwidget)
        self.label_11.setObjectName("label_11")
        self.gridLayout_5.addWidget(self.label_11, 4, 0, 1, 1)
        self.line_edit_pid_position_fc = QtWidgets.QLineEdit(self.centralwidget)
        self.line_edit_pid_position_fc.setObjectName("line_edit_pid_position_fc")
        self.gridLayout_5.addWidget(self.line_edit_pid_position_fc, 4, 1, 1, 1)
        self.label_17 = QtWidgets.QLabel(self.centralwidget)
        self.label_17.setObjectName("label_17")
        self.gridLayout_5.addWidget(self.label_17, 4, 2, 1, 1)
        self.line_edit_pid_velocity_fc = QtWidgets.QLineEdit(self.centralwidget)
        self.line_edit_pid_velocity_fc.setObjectName("line_edit_pid_velocity_fc")
        self.gridLayout_5.addWidget(self.line_edit_pid_velocity_fc, 4, 3, 1, 1)
        self.label_12 = QtWidgets.QLabel(self.centralwidget)
        self.label_12.setObjectName("label_12")
        self.gridLayout_5.addWidget(self.label_12, 5, 0, 1, 1)
        self.line_edit_pid_position_u_max = QtWidgets.QLineEdit(self.centralwidget)
        self.line_edit_pid_position_u_max.setObjectName("line_edit_pid_position_u_max")
        self.gridLayout_5.addWidget(self.line_edit_pid_position_u_max, 5, 1, 1, 1)
        self.label_18 = QtWidgets.QLabel(self.centralwidget)
        self.label_18.setObjectName("label_18")
        self.gridLayout_5.addWidget(self.label_18, 5, 2, 1, 1)
        self.line_edit_pid_velocity_u_max = QtWidgets.QLineEdit(self.centralwidget)
        self.line_edit_pid_velocity_u_max.setObjectName("line_edit_pid_velocity_u_max")
        self.gridLayout_5.addWidget(self.line_edit_pid_velocity_u_max, 5, 3, 1, 1)
        self.gridLayout_7.addLayout(self.gridLayout_5, 10, 0, 1, 1)
        self.gridLayout_6 = QtWidgets.QGridLayout()
        self.gridLayout_6.setObjectName("gridLayout_6")
        self.push_button_set_position_pid_param = QtWidgets.QPushButton(self.centralwidget)
        self.push_button_set_position_pid_param.setStyleSheet("background-color: rgb(144, 224, 237);")
        self.push_button_set_position_pid_param.setObjectName("push_button_set_position_pid_param")
        self.gridLayout_6.addWidget(self.push_button_set_position_pid_param, 0, 0, 1, 1)
        self.push_button_set_velocity_pid_param = QtWidgets.QPushButton(self.centralwidget)
        self.push_button_set_velocity_pid_param.setStyleSheet("background-color: rgb(144, 224, 237);")
        self.push_button_set_velocity_pid_param.setObjectName("push_button_set_velocity_pid_param")
        self.gridLayout_6.addWidget(self.push_button_set_velocity_pid_param, 0, 1, 1, 1)
        self.push_button_position_control = QtWidgets.QPushButton(self.centralwidget)
        self.push_button_position_control.setStyleSheet("background-color: rgb(247, 179, 181);")
        self.push_button_position_control.setObjectName("push_button_position_control")
        self.gridLayout_6.addWidget(self.push_button_position_control, 1, 0, 1, 1)
        self.push_button_velocity_control = QtWidgets.QPushButton(self.centralwidget)
        self.push_button_velocity_control.setStyleSheet("background-color: rgb(247, 179, 181);")
        self.push_button_velocity_control.setObjectName("push_button_velocity_control")
        self.gridLayout_6.addWidget(self.push_button_velocity_control, 1, 1, 1, 1)
        self.gridLayout_7.addLayout(self.gridLayout_6, 11, 0, 1, 1)
        self.gridLayout_9.addLayout(self.gridLayout_7, 0, 0, 1, 1)
        self.gridLayout_8 = QtWidgets.QGridLayout()
        self.gridLayout_8.setObjectName("gridLayout_8")
        self.label_e_stop = QtWidgets.QLabel(self.centralwidget)
        self.label_e_stop.setObjectName("label_e_stop")
        self.gridLayout_8.addWidget(self.label_e_stop, 1, 3, 1, 2)
        self.label_open_loop_control = QtWidgets.QLabel(self.centralwidget)
        self.label_open_loop_control.setObjectName("label_open_loop_control")
        self.gridLayout_8.addWidget(self.label_open_loop_control, 2, 3, 1, 2)
        self.label_close_loop_control = QtWidgets.QLabel(self.centralwidget)
        self.label_close_loop_control.setObjectName("label_close_loop_control")
        self.gridLayout_8.addWidget(self.label_close_loop_control, 3, 3, 1, 2)
        self.label_position_control = QtWidgets.QLabel(self.centralwidget)
        self.label_position_control.setObjectName("label_position_control")
        self.gridLayout_8.addWidget(self.label_position_control, 4, 3, 1, 2)
        self.label_velocity_control = QtWidgets.QLabel(self.centralwidget)
        self.label_velocity_control.setObjectName("label_velocity_control")
        self.gridLayout_8.addWidget(self.label_velocity_control, 5, 3, 1, 2)
        self.label_switch2anglefeed = QtWidgets.QLabel(self.centralwidget)
        self.label_switch2anglefeed.setObjectName("label_switch2anglefeed")
        self.gridLayout_8.addWidget(self.label_switch2anglefeed, 6, 3, 1, 2)
        self.label_25 = QtWidgets.QLabel(self.centralwidget)
        self.label_25.setObjectName("label_25")
        self.gridLayout_8.addWidget(self.label_25, 7, 0, 1, 5)
        self.label_45 = QtWidgets.QLabel(self.centralwidget)
        self.label_45.setObjectName("label_45")
        self.gridLayout_8.addWidget(self.label_45, 8, 0, 1, 2)
        self.label_pos_th0 = QtWidgets.QLabel(self.centralwidget)
        self.label_pos_th0.setObjectName("label_pos_th0")
        self.gridLayout_8.addWidget(self.label_pos_th0, 8, 2, 1, 3)
        self.label_pos_kp = QtWidgets.QLabel(self.centralwidget)
        self.label_pos_kp.setObjectName("label_pos_kp")
        self.gridLayout_8.addWidget(self.label_pos_kp, 9, 2, 1, 3)
        self.label_pos_ki = QtWidgets.QLabel(self.centralwidget)
        self.label_pos_ki.setObjectName("label_pos_ki")
        self.gridLayout_8.addWidget(self.label_pos_ki, 10, 2, 1, 3)
        self.label_pos_kd = QtWidgets.QLabel(self.centralwidget)
        self.label_pos_kd.setObjectName("label_pos_kd")
        self.gridLayout_8.addWidget(self.label_pos_kd, 11, 2, 1, 3)
        self.label_pos_fc = QtWidgets.QLabel(self.centralwidget)
        self.label_pos_fc.setObjectName("label_pos_fc")
        self.gridLayout_8.addWidget(self.label_pos_fc, 12, 2, 1, 3)
        self.label_24 = QtWidgets.QLabel(self.centralwidget)
        self.label_24.setObjectName("label_24")
        self.gridLayout_8.addWidget(self.label_24, 13, 0, 1, 2)
        self.label_pos_u_max = QtWidgets.QLabel(self.centralwidget)
        self.label_pos_u_max.setObjectName("label_pos_u_max")
        self.gridLayout_8.addWidget(self.label_pos_u_max, 13, 2, 1, 3)
        self.label_33 = QtWidgets.QLabel(self.centralwidget)
        self.label_33.setObjectName("label_33")
        self.gridLayout_8.addWidget(self.label_33, 14, 0, 1, 5)
        self.label_30 = QtWidgets.QLabel(self.centralwidget)
        self.label_30.setObjectName("label_30")
        self.gridLayout_8.addWidget(self.label_30, 20, 0, 1, 2)
        self.button_data_log = QtWidgets.QPushButton(self.centralwidget)
        self.button_data_log.setStyleSheet("background-color: rgb(247, 179, 181);")
        self.button_data_log.setObjectName("button_data_log")
        self.gridLayout_8.addWidget(self.button_data_log, 21, 0, 1, 5)
        self.button_system_identification = QtWidgets.QPushButton(self.centralwidget)
        self.button_system_identification.setStyleSheet("background-color: rgb(247, 179, 181);")
        self.button_system_identification.setObjectName("button_system_identification")
        self.gridLayout_8.addWidget(self.button_system_identification, 22, 0, 1, 5)
        self.label_59 = QtWidgets.QLabel(self.centralwidget)
        self.label_59.setObjectName("label_59")
        self.gridLayout_8.addWidget(self.label_59, 0, 0, 1, 5)
        self.label_32 = QtWidgets.QLabel(self.centralwidget)
        self.label_32.setObjectName("label_32")
        self.gridLayout_8.addWidget(self.label_32, 9, 0, 1, 2)
        self.label_21 = QtWidgets.QLabel(self.centralwidget)
        self.label_21.setObjectName("label_21")
        self.gridLayout_8.addWidget(self.label_21, 10, 0, 1, 2)
        self.label_22 = QtWidgets.QLabel(self.centralwidget)
        self.label_22.setObjectName("label_22")
        self.gridLayout_8.addWidget(self.label_22, 11, 0, 1, 2)
        self.label_23 = QtWidgets.QLabel(self.centralwidget)
        self.label_23.setObjectName("label_23")
        self.gridLayout_8.addWidget(self.label_23, 12, 0, 1, 2)
        self.label_52 = QtWidgets.QLabel(self.centralwidget)
        self.label_52.setObjectName("label_52")
        self.gridLayout_8.addWidget(self.label_52, 6, 0, 1, 3)
        self.label_51 = QtWidgets.QLabel(self.centralwidget)
        self.label_51.setObjectName("label_51")
        self.gridLayout_8.addWidget(self.label_51, 5, 0, 1, 3)
        self.label_50 = QtWidgets.QLabel(self.centralwidget)
        self.label_50.setObjectName("label_50")
        self.gridLayout_8.addWidget(self.label_50, 4, 0, 1, 3)
        self.label_49 = QtWidgets.QLabel(self.centralwidget)
        self.label_49.setObjectName("label_49")
        self.gridLayout_8.addWidget(self.label_49, 3, 0, 1, 3)
        self.label_35 = QtWidgets.QLabel(self.centralwidget)
        self.label_35.setObjectName("label_35")
        self.gridLayout_8.addWidget(self.label_35, 2, 0, 1, 3)
        self.label_26 = QtWidgets.QLabel(self.centralwidget)
        self.label_26.setObjectName("label_26")
        self.gridLayout_8.addWidget(self.label_26, 1, 0, 1, 3)
        self.label_31 = QtWidgets.QLabel(self.centralwidget)
        self.label_31.setObjectName("label_31")
        self.gridLayout_8.addWidget(self.label_31, 16, 0, 1, 2)
        self.label_29 = QtWidgets.QLabel(self.centralwidget)
        self.label_29.setObjectName("label_29")
        self.gridLayout_8.addWidget(self.label_29, 17, 0, 1, 2)
        self.label_27 = QtWidgets.QLabel(self.centralwidget)
        self.label_27.setObjectName("label_27")
        self.gridLayout_8.addWidget(self.label_27, 18, 0, 1, 2)
        self.label_28 = QtWidgets.QLabel(self.centralwidget)
        self.label_28.setObjectName("label_28")
        self.gridLayout_8.addWidget(self.label_28, 19, 0, 1, 2)
        self.label_vel_ki = QtWidgets.QLabel(self.centralwidget)
        self.label_vel_ki.setObjectName("label_vel_ki")
        self.gridLayout_8.addWidget(self.label_vel_ki, 17, 2, 1, 3)
        self.label_vel_kp = QtWidgets.QLabel(self.centralwidget)
        self.label_vel_kp.setObjectName("label_vel_kp")
        self.gridLayout_8.addWidget(self.label_vel_kp, 16, 2, 1, 3)
        self.label_vel_kd = QtWidgets.QLabel(self.centralwidget)
        self.label_vel_kd.setObjectName("label_vel_kd")
        self.gridLayout_8.addWidget(self.label_vel_kd, 18, 2, 1, 3)
        self.label_vel_fc = QtWidgets.QLabel(self.centralwidget)
        self.label_vel_fc.setObjectName("label_vel_fc")
        self.gridLayout_8.addWidget(self.label_vel_fc, 19, 2, 1, 3)
        self.label_vel_u_max = QtWidgets.QLabel(self.centralwidget)
        self.label_vel_u_max.setObjectName("label_vel_u_max")
        self.gridLayout_8.addWidget(self.label_vel_u_max, 20, 2, 1, 3)
        self.label_43 = QtWidgets.QLabel(self.centralwidget)
        self.label_43.setObjectName("label_43")
        self.gridLayout_8.addWidget(self.label_43, 15, 0, 1, 2)
        self.label_dth0_dt = QtWidgets.QLabel(self.centralwidget)
        self.label_dth0_dt.setObjectName("label_dth0_dt")
        self.gridLayout_8.addWidget(self.label_dth0_dt, 15, 2, 1, 3)
        self.gridLayout_9.addLayout(self.gridLayout_8, 0, 1, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 564, 21))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Motor Control GUI"))
        self.push_button_start_stop.setText(_translate("MainWindow", "Start / Stop"))
        self.push_button_open_loop.setText(_translate("MainWindow", "Open Loop Control"))
        self.label_20.setText(_translate("MainWindow", "V %"))
        self.line_edit_v_percent.setText(_translate("MainWindow", "0"))
        self.push_button_set_v_percent.setText(_translate("MainWindow", "Set V%"))
        self.push_button_closed_loop.setText(_translate("MainWindow", "Closed Loop Control"))
        self.label_4.setText(_translate("MainWindow", "Feedback"))
        self.label_3.setText(_translate("MainWindow", "fc (dth_dt)"))
        self.line_edit_alpha.setText(_translate("MainWindow", "9999999999"))
        self.push_button_set_alpha.setText(_translate("MainWindow", "Set alpha"))
        self.push_button_angle_feedback.setText(_translate("MainWindow", "Switch to angle feedback"))
        self.label_19.setText(_translate("MainWindow", "Setpoint"))
        self.label.setText(_translate("MainWindow", "th0 (degree)"))
        self.line_edit_th0.setText(_translate("MainWindow", "0"))
        self.push_button_set_th0.setText(_translate("MainWindow", "Set th0"))
        self.label_2.setText(_translate("MainWindow", "dth0_dt (degree/sec)"))
        self.line_edit_dth0_dt.setText(_translate("MainWindow", "0"))
        self.push_button_set_dth0_dt.setText(_translate("MainWindow", "Set dth0_dt"))
        self.label_5.setText(_translate("MainWindow", "Position Control"))
        self.label_6.setText(_translate("MainWindow", "Velocity Control"))
        self.label_7.setText(_translate("MainWindow", "dt"))
        self.line_edit_pid_position_dt.setText(_translate("MainWindow", "1"))
        self.label_13.setText(_translate("MainWindow", "dt"))
        self.line_edit_pid_velocity_dt.setText(_translate("MainWindow", "1"))
        self.label_8.setText(_translate("MainWindow", "Kp"))
        self.line_edit_pid_position_kp.setText(_translate("MainWindow", "0"))
        self.label_14.setText(_translate("MainWindow", "Kp"))
        self.line_edit_pid_velocity_kp.setText(_translate("MainWindow", "0"))
        self.label_9.setText(_translate("MainWindow", "Ki"))
        self.line_edit_pid_position_ki.setText(_translate("MainWindow", "0"))
        self.label_15.setText(_translate("MainWindow", "Ki"))
        self.line_edit_pid_velocity_ki.setText(_translate("MainWindow", "0"))
        self.label_10.setText(_translate("MainWindow", "Kd"))
        self.line_edit_pid_position_kd.setText(_translate("MainWindow", "0"))
        self.label_16.setText(_translate("MainWindow", "Kd"))
        self.line_edit_pid_velocity_kd.setText(_translate("MainWindow", "0"))
        self.label_11.setText(_translate("MainWindow", "fc"))
        self.line_edit_pid_position_fc.setText(_translate("MainWindow", "9999999999"))
        self.label_17.setText(_translate("MainWindow", "fc"))
        self.line_edit_pid_velocity_fc.setText(_translate("MainWindow", "9999999999"))
        self.label_12.setText(_translate("MainWindow", "u_max"))
        self.line_edit_pid_position_u_max.setText(_translate("MainWindow", "0"))
        self.label_18.setText(_translate("MainWindow", "u_max"))
        self.line_edit_pid_velocity_u_max.setText(_translate("MainWindow", "0"))
        self.push_button_set_position_pid_param.setText(_translate("MainWindow", "Set Position Control Param"))
        self.push_button_set_velocity_pid_param.setText(_translate("MainWindow", "Set Velocity Control Param"))
        self.push_button_position_control.setText(_translate("MainWindow", "Position Control"))
        self.push_button_velocity_control.setText(_translate("MainWindow", "Velocity Control"))
        self.label_e_stop.setText(_translate("MainWindow", "TextLabel"))
        self.label_open_loop_control.setText(_translate("MainWindow", "TextLabel"))
        self.label_close_loop_control.setText(_translate("MainWindow", "TextLabel"))
        self.label_position_control.setText(_translate("MainWindow", "TextLabel"))
        self.label_velocity_control.setText(_translate("MainWindow", "TextLabel"))
        self.label_switch2anglefeed.setText(_translate("MainWindow", "TextLabel"))
        self.label_25.setText(_translate("MainWindow", "Position Controller Parameters"))
        self.label_45.setText(_translate("MainWindow", "th0 (deg)"))
        self.label_pos_th0.setText(_translate("MainWindow", "TextLabel"))
        self.label_pos_kp.setText(_translate("MainWindow", "TextLabel"))
        self.label_pos_ki.setText(_translate("MainWindow", "TextLabel"))
        self.label_pos_kd.setText(_translate("MainWindow", "TextLabel"))
        self.label_pos_fc.setText(_translate("MainWindow", "TextLabel"))
        self.label_24.setText(_translate("MainWindow", "u_max"))
        self.label_pos_u_max.setText(_translate("MainWindow", "TextLabel"))
        self.label_33.setText(_translate("MainWindow", "Velocity Controller Parameters"))
        self.label_30.setText(_translate("MainWindow", "u_max"))
        self.button_data_log.setText(_translate("MainWindow", "Log data"))
        self.button_system_identification.setText(_translate("MainWindow", "System Idenfication"))
        self.label_59.setText(_translate("MainWindow", "Current Status"))
        self.label_32.setText(_translate("MainWindow", "Kp"))
        self.label_21.setText(_translate("MainWindow", "Ki"))
        self.label_22.setText(_translate("MainWindow", "Kd"))
        self.label_23.setText(_translate("MainWindow", "fc"))
        self.label_52.setText(_translate("MainWindow", "switch_2_angle feed"))
        self.label_51.setText(_translate("MainWindow", "Velocity controller"))
        self.label_50.setText(_translate("MainWindow", "Position controller"))
        self.label_49.setText(_translate("MainWindow", "Closed loop control"))
        self.label_35.setText(_translate("MainWindow", "Open loop control"))
        self.label_26.setText(_translate("MainWindow", "E-stop"))
        self.label_31.setText(_translate("MainWindow", "Kp"))
        self.label_29.setText(_translate("MainWindow", "Ki"))
        self.label_27.setText(_translate("MainWindow", "Kd"))
        self.label_28.setText(_translate("MainWindow", "fc"))
        self.label_vel_ki.setText(_translate("MainWindow", "TextLabel"))
        self.label_vel_kp.setText(_translate("MainWindow", "TextLabel"))
        self.label_vel_kd.setText(_translate("MainWindow", "TextLabel"))
        self.label_vel_fc.setText(_translate("MainWindow", "TextLabel"))
        self.label_vel_u_max.setText(_translate("MainWindow", "TextLabel"))
        self.label_43.setText(_translate("MainWindow", "dth0_dt (deg/sec)"))
        self.label_dth0_dt.setText(_translate("MainWindow", "TextLabel"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
