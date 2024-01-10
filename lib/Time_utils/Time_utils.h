#ifndef TIME_UTILS
#define TIME_UTILS

#include <Arduino.h>

class Time_utils{
	public:
		Time_utils(double freq);
		void init_ref();
		double get_t_now_micros();
		double get_t_now_millis();
		double get_t_now_sec();
		double get_loop_freq();
		double get_loop_time_micros();
		double get_loop_time_sec();
		void wait();
		void set_t_last_loop_micros();
		uint32_t get_t_last_loop_micros();
		double get_t_now_micros_ref();
		double get_t_now_millis_ref();
		double get_t_now_sec_ref();

	private:
		double LoopFrequency      = 400.0;
		double dt_loop_sec        = 1.0/LoopFrequency;
		double dt_loop_micros     = 1000000.0*dt_loop_sec;
		uint32_t t_last_loop_micros = 0;

		double t_init_micros = 0.0;
		double t_init_millis = 0.0;
		double t_init_sec    = 0.0;

		double t_now = 0.0;
		bool start   = true;
};

#endif