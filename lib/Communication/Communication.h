#ifndef COMMUNICATION
#define COMMUNICATION

#include <stdint.h>
#include <Arduino.h>

#pragma pack(push,1)
struct Tx_pkt_str{
	uint8_t start        = 0x21;
	int32_t enc_count    = 0;
	double enc_angle     = 0;
	double enc_speed     = 0;
	double motor_angle   = 0;
	double motor_speed   = 0;
	double motor_voltage = 0;
	double motor_current = 0;
	uint8_t crc          = 0;
};
#pragma pack(pop)

#pragma pack(push,1)
struct Rx_pkt_str{
	uint8_t start    = 0x21;
	double V_percent = 0;
	uint8_t crc      = 0;
};
#pragma pack(pop)


class Communication{
	public:
		Communication();
		void init(long baude_rate=9600);
		void send_data(int32_t enc_count, double enc_angle, double enc_speed, double motor_angle, double motor_speed, double motor_voltage, double motor_current);
		double receive_data();

	private:
		Tx_pkt_str tx_pkt_str;
		Rx_pkt_str rx_pkt_str;
		uint8_t tx_df[sizeof(struct Tx_pkt_str)];
		uint8_t rx_df[sizeof(struct Rx_pkt_str)];
		uint8_t r_index = 0;
		bool r_df_flag = false;
};

#endif