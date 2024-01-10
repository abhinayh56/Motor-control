#include "Motor.h"

void Motor::init(uint8_t pwm_pin_, uint8_t dir_pin_, float pwm_freq_, uint32_t pwm_res_){
	pwm_pin  = pwm_pin_;
	dir_pin  = dir_pin_;
	pwm_freq = pwm_freq_;
	pwm_res  = pwm_res_;

	pinMode(dir_pin,OUTPUT);
	digitalWrite(dir_pin,LOW);
	pinMode(pwm_pin,OUTPUT);
	analogWriteResolution(pwm_res); // 0 to 16383, or 16384 for high
	analogWriteFrequency(pwm_pin,pwm_freq);
	analogWrite(pwm_pin,0);
}

void Motor::drive(int16_t pwm){
	pwm = math.saturate(pwm,-pwm_max,pwm_max);

	if(pwm<0){
		digitalWrite(dir_pin,LOW);
		analogWrite(pwm_pin,-pwm);
	}
	else{
		digitalWrite(dir_pin,HIGH);
		analogWrite(pwm_pin,pwm);
	}
}

void Motor::drive_V(double V_1, double V_bat){
	double pwm = (pwm_max*V_1)/V_bat;
	drive((int16_t)pwm);
}

void Motor:: drive_percent(double V_percent){
	V_percent = math.saturate(V_percent,-100.0,100.0);
	double pwm = (((double)pwm_max)*V_percent)/100.0;
	drive((int16_t)pwm);
}