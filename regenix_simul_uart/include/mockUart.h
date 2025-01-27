#ifndef MOCKUART_H_
#define MOCKUART_H_

#include <Arduino.h>

#define NB_PINCH_VALVE 7
#define SM_MAX 0x07         /// MOCK for max state of the default state machine

void setPhGoal(float phGoal);

float getPh(void);

void setTempGoal(float tempGoal);

float getTemp(void);

void setDOGoal(float doGoal);

float getDO(void);

void setPumpGoal(float speedGoal);

float getPumpVal(uint8_t index);

bool getValveStatus(uint8_t index);

uint8_t getState(void);
uint8_t getSubState(void);
void setState(uint8_t state);

#endif //MOCKUART_H_