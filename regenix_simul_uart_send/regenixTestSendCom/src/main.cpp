#include <Arduino.h>
#include "serialUart.h"
// put function declarations here:

SerialUart* uartCom;

uint16_t msgLenFromType[] = {0, 0, 0, 0, 1, 4, 4, 4, 4};
uint8_t *liquidStatusContent = NULL;
uint8_t *pumpStatusContent = NULL;
uint8_t *valveStatusContent = NULL;
uint8_t *getStateContent = NULL;
uint8_t setStateContent = 8;
float setPumpGoalContent = -19.34;
float setDOGoalContent = 80.8;
float setTempGoalContent = 38.5;
float setPhGoalContent = 11.5;

uint8_t msgTest[] = {0x55, 0xAA, 0x55, 0xAA, 0x00, 0x00,0x00, 0x2B, 0xDB};



void setup() {
  // Serial.begin(9600);
  uartCom = new SerialUart(&Serial);

  delay(100);

  // Serial.println("Start");
}

void loop() {
  delay(1000);
  // uartCom->sendMsgRxRsp(uartCom->MSG_TYPE_LIQUID_STATUS, (uint8_t*)liquidStatusContent, msgLenFromType[uartCom->MSG_TYPE_LIQUID_STATUS], 2000, true);
  delay(500);
  uartCom->sendMsgRxRsp(uartCom->MSG_TYPE_PUMP_STATUS, (uint8_t*)pumpStatusContent, msgLenFromType[uartCom->MSG_TYPE_PUMP_STATUS], 2000, true);
  delay(500);
  uartCom->sendMsgRxRsp(uartCom->MSG_TYPE_SET_PUMP_GOAL, (uint8_t*)&setPumpGoalContent, msgLenFromType[uartCom->MSG_TYPE_SET_PUMP_GOAL], 2000, false);
  // Serial.write(msgTest, 9);
  // Serial.println("Sent");
  
  // uint32_t readTimeout = millis()+2000;
  // while(millis()<readTimeout){
  //   if (Serial.available()){
  // //     Serial.print(Serial1.read(), HEX);
  // //     
  //       Serial.read();
  //       readTimeout = millis()+ 500;
  //   }
  // }
  // Serial.println();

  // Serial.println("Out of func");
  delay(2000);

  

  // Serial.println("loop\n");
}
