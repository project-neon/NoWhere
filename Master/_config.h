#include <stdint.h>
#include <math.h>

#ifndef CONFIG_H
#define CONFIG_H

//
// PINS
//
#define PIN_LED1               12
#define PIN_LED2               13

#define PIN_BUZZER             A5

#define PIN_M1_EN              5
#define PIN_M1_IN1             A0
#define PIN_M1_IN2             A1

#define PIN_M2_EN              6
#define PIN_M2_IN1             A4
#define PIN_M2_IN2             4

#define PIN_BTN                0

#define PIN_VBAT               A3

#define PIN_RADIO1_CE          8
#define PIN_RADIO1_CSN         9
#define PIN_RADIO1_INTERRUPT   0

#define PIN_RADIO2_CE          10
#define PIN_RADIO2_CSN         11

//
// Project
//
#define PROJECT_NAME           "SlaveFirmware"
#define PROJECT_VERSION        "v2.1"


//
// Serial DEBUG
//
#define SERIAL_SPEED           115200
#define LOG                    Serial.print
#define ENDL                   LOG(F("\r\n"))


//
// Motors config (H-Bridge)
//
#define MOTOR_ABS_MAX          255


//
// VBat Reader
//
#define BAT_R1                 22000
#define BAT_R2                 10000
#define BAT_DROP               0.26
#define VBAT_VOLTAGE(adc)      (adc / (1023 / 5.0)) * (BAT_R1 + BAT_R2) * (1.0 / BAT_R2) + BAT_DROP

#define VBAT_ALARMED           7.10
#define VBAT_WARNED            7.40
#define VBAT_USB               5.10


//
// RADIO Configs (NRF24L01)
//
#define RADIO_MASTER_ADDRESS   1
#define RADIO_TIMEOUT_TO_IDDLE 2000
#define RADIO_PACKET_SIZE      8
#define FLOAT_MULTIPLIER       10.0

//
// EEPROM Addresses
//
#define EEPROM_ROBOT_ID        0x10

#endif
