#include <Arduino.h>
#include <Wire.h>
#include<EEPROM.h>
// #include <math.h>
#include <Ps3Controller.h>

TaskHandle_t mainTask;
TaskHandle_t I2CTask;

void mainApplicationTask(void *parameter);
void I2CCommunicationSendTask(void *parameter);

int SLAVE_ADDR_MODULE[4] = {10, 11, 12, 13};
int Drive_SP[4];
int Steer_SP[4];

String data;
String ArrData[3];

bool parsing = false;

short int lx;
short int ly;
short int rx;

#define pinBuzzer 25

#define EEPROM_SIZE 512
#define addsDegree 0
int8_t zeroDegree;
int8_t eepromVal;
