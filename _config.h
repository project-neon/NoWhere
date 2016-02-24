#include <stdint.h>
#include <math.h>

#ifndef CONFIG_H
#define CONFIG_H

//
// PINS
//
#define PIN_LED1               A1
#define PIN_LED2               A2

#define PIN_BUZZER             A3

#define PIN_M1_EN              6
#define PIN_M1_IN1             4
#define PIN_M1_IN2             3

#define PIN_M2_EN              5
#define PIN_M2_IN1             7
#define PIN_M2_IN2             8

#define PIN_BTN                A6

#define PIN_VBAT               A0

#define PIN_RADIO_CE           9
#define PIN_RADIO_CSN          10


//
// Project
//
#define PROJECT_NAME           "SlaveFirmware"
#define PROJECT_VERSION        "v0.02"


//
// Serial DEBUG
//
#define SERIAL_SPEED           57600
#define LOG                    Serial.print


//
// Motors config (H-Bridge)
//
#define MOTOR_ABS_MAX          250


//
// VBat Reader
//
#define VBAT_VOLTAGE(adc)      ((adc - 40.3) / 88.3)

#define VBAT_ALARMED           7.10
#define VBAT_WARNED            7.40


//
// RADIO Configs (NRF24L01)
//
#define RADIO_MASTER_ADDRESS   1
#define RADIO_TIMEOUT_TO_IDDLE 2000

#endif
