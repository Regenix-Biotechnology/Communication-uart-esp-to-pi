#include "mockUart.h"

float ph = 7.0;
float temp = 37.0;
float DO = 99.0;
float primaryPumpSpeed = 8.7;
uint8_t state = 0;
uint8_t subState = 0;

void setPhGoal(float phGoal){
    ph = phGoal;
}

float getPh(void){
    return ph;
}

void setTempGoal(float tempGoal){
    temp = tempGoal;
}

float getTemp(void){
    return temp;
}

void setDOGoal(float doGoal){
    DO = doGoal;
}

float getDO(void){
    return DO;
}

void setPumpGoal(float speedGoal){
    primaryPumpSpeed = speedGoal;
}

float getPumpVal(uint8_t index){
    if (index == 0){
        return primaryPumpSpeed;
    }
    return 100.0;
}

bool getValveStatus(uint8_t index){
    return 1;
}

uint8_t getState(void){
    return state;
}

uint8_t getSubState(void){
    return subState;
}

void setState(uint8_t state){
    state = state;
}