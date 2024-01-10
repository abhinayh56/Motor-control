#include <Arduino.h>
#include "Motor.h"
#include "Time_utils.h"
#include "QuadEncoder.h"
#include "Communication.h"

#define LOOP_FREQ 250.0

#define enc_1_PPR 334
#define enc_1_CPR (enc_1_PPR*4)
#define gear_ratio 100.0

#define enc_1_channel 1
#define enc_1_A 2
#define enc_1_B 3
#define enc_1_pullup_req 0

QuadEncoder enc_1(enc_1_channel, enc_1_A, enc_1_B, enc_1_pullup_req);

Time_utils time_utils(LOOP_FREQ);

// encoder calculation
int32_t enc_1_count    = 0.0;
double enc_1_angle     = 0.0;
double enc_1_angle_pre = 0.0;
double motor_1_angle   = 0.0;
double enc_1_speed     = 0.0;
double motor_1_speed   = 0.0;
double dt = 0.0;

// motor driving
#define motor_1_pwm_pin 28
#define motor_1_dir_pin 30
Motor motor;

Communication comm;


void setup(){
	comm.init(115200);

	motor.init(motor_1_pwm_pin, motor_1_dir_pin);
	motor.drive(0);

	enc_1.setInitConfig();
	enc_1.init();

	dt = time_utils.get_loop_time_sec();

	time_utils.set_t_last_loop_micros();
}

void loop(){
	enc_1_count = -enc_1.read();
	enc_1_angle = ((double)enc_1_count*360.0) / ((double)enc_1_CPR); // degree
	enc_1_speed = (enc_1_angle - enc_1_angle_pre) / dt; // degree / seconds
	enc_1_angle_pre = enc_1_angle;

	motor_1_angle = enc_1_angle / gear_ratio;
	motor_1_speed = enc_1_speed / gear_ratio;

	double motor_1_voltage = 0;
	double motor_1_current = 0;

	// feedback
	// enc_1_count     = millis();
	// enc_1_angle     = (double)millis()/1000.0;
	// enc_1_speed     = -9876.54321;
	// motor_1_angle   = -1121;
	// motor_1_speed   = -3141;
	motor_1_voltage = 0.0;
	motor_1_current = 0.0;
	
	comm.send_data(enc_1_count, enc_1_angle, enc_1_speed, motor_1_angle, motor_1_speed, motor_1_voltage, motor_1_current);

	// controller
	// Serial.println(motor_1_angle);

	// controller output
	double v_percent = comm.receive_data();
	motor.drive_percent(v_percent);

	time_utils.wait();
}