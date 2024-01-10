#include "Time_utils.h"

Time_utils::Time_utils(double freq){
	LoopFrequency      = freq;
	dt_loop_sec        = 1.0/LoopFrequency;
	dt_loop_micros     = 1000000.0*dt_loop_sec;
	t_last_loop_micros = micros();
}

void Time_utils::init_ref(){
	if(start==true){
		t_init_micros = micros();
		t_init_millis = t_init_micros*0.001;
		t_init_sec    = t_init_millis*0.001;
		start = false;
	}
}

double Time_utils::get_t_now_micros(){
	return micros();
}

double Time_utils::get_t_now_millis(){
	return millis();
}

double Time_utils::get_t_now_sec(){
	return 0.001*(double)millis();
}

double Time_utils::get_loop_freq(){
	return LoopFrequency;
}

double Time_utils::get_loop_time_micros(){
	return dt_loop_micros;
}

double Time_utils::get_loop_time_sec(){
	return dt_loop_sec;
}

void Time_utils::wait(){
	while(((double)(micros()- t_last_loop_micros)) < dt_loop_micros){
	}
	t_last_loop_micros = micros();
}

void Time_utils::set_t_last_loop_micros(){
	t_last_loop_micros = micros();
}

uint32_t Time_utils::get_t_last_loop_micros(){
	return t_last_loop_micros;
}

double Time_utils::get_t_now_micros_ref(){
	return micros() - t_init_micros;
}

double Time_utils::get_t_now_millis_ref(){
	return millis() - t_init_millis;
}

double Time_utils::get_t_now_sec_ref(){
	return 0.001*millis() - t_init_sec;
}