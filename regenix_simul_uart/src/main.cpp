#include <Arduino.h>
#include "serialUart.h"
// put function declarations here:

SerialUart* uartCom;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  uartCom = new SerialUart(&Serial1);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(100);
  uartCom->processUart();
}
