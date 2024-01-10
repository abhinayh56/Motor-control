#ifndef MOTOR
#define MOTOR

#include <Arduino.h>
#include "Math_functions.h"

class Motor{
	public:
		void init(uint8_t pwm_pin_, uint8_t dir_pin_, float pwm_freq_=9155.27, uint32_t pwm_res_=14);
		void drive(int16_t pwm_1);
		void drive_V(double V_1, double V_bat=12.0);
		void drive_percent(double V_percent);

	private:
		uint8_t dir_pin = 31;
		uint8_t pwm_pin = 29;
		int pwm_max = 16383;

		float pwm_freq = 9155.27; // 109.226707678 us pulse width
		uint32_t pwm_res = 14; // // 0 to 16383, or 16384 for high

		Math_functions math;
};

#endif
