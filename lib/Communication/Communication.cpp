#include "Communication.h"

Communication::Communication(){
}

void Communication::init(long baude_rate){
    Serial.begin(baude_rate);
}

void Communication::send_data(int32_t enc_count, double enc_angle, double enc_speed, double motor_angle, double motor_speed, double motor_voltage, double motor_current){
    tx_pkt_str.enc_count     = enc_count;
    tx_pkt_str.enc_angle     = enc_angle;
    tx_pkt_str.enc_speed     = enc_speed;
    tx_pkt_str.motor_angle   = motor_angle;
    tx_pkt_str.motor_speed   = motor_speed;
    tx_pkt_str.motor_voltage = motor_voltage;
    tx_pkt_str.motor_current = motor_current;
    uint8_t crc = 0;
    tx_pkt_str.crc           = crc;

    memcpy(tx_df, &tx_pkt_str, sizeof(struct Tx_pkt_str));
    Serial.write(tx_df, sizeof(struct Tx_pkt_str));
}

double Communication::receive_data(){
    uint8_t data = 0;
    r_df_flag = false;

    while(Serial.available()){
        data = Serial.read();

        if(data == 0x21){
            r_index = 0;
        }

        if(r_index < sizeof(struct Rx_pkt_str)){
            rx_df[r_index] = data;
        }

        if(r_index == (sizeof(struct Rx_pkt_str) - 1)){
            r_df_flag = true;
            break;
        }
        else{
            r_df_flag = false;
            r_index = r_index + 1;
        }
    }

    if(r_df_flag == true){
        memcpy(&rx_pkt_str, rx_df, sizeof(struct Rx_pkt_str));
    }

    return rx_pkt_str.V_percent;
}