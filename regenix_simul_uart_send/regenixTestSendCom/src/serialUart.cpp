#include "serialUart.h"

// #define MOCK_UART
#ifndef MOCK_UART
//will do normal include
#else
#include "mockUart.h"
#endif

#define BAUDRATE 9600
#define MAGIC_WORD 0xAA55AA55
#define MAGIC_WORD_LEN sizeof(uint32_t)
#define MSG_DATA_TYPE_LEN sizeof(uint8_t)
#define MSG_DATA_SIZE_LEN (sizeof(uint16_t))
#define HEADER_LEN (MAGIC_WORD_LEN + MSG_DATA_TYPE_LEN + MSG_DATA_SIZE_LEN)
#define MAX_TX_DATA_LEN (sizeof(uint32_t))
#define MAX_RX_DATA_LEN (sizeof(uint32_t)*3)
#define CRC_LEN (sizeof(uint16_t))

uint8_t rxHeaderBuff[HEADER_LEN] = {0};
uint8_t rxDataBuff[MAX_RX_DATA_LEN+CRC_LEN] = {0};
uint8_t txMsgBuff[HEADER_LEN+MAX_TX_DATA_LEN+CRC_LEN] = {0};
uint16_t uartTxMsgLenFromType[] = {0, 0, 0, 0, 1, 4, 4, 4, 4};
uint16_t uartRxMsgLenFromType[] = {12, 12, 1, 2, 1, 1, 1, 1, 1};

uint32_t rxTimeoutEnd = 0;

SerialUart::SerialUart(HardwareSerial* pHandle){
    pSerial = pHandle;
    pSerial->begin(BAUDRATE);


}

void SerialUart::sendMsgRxRsp(eUartMsgType msgType, uint8_t *content, uint16_t size, uint32_t timeout, bool readFloat){
    // HEADER
    uint32_t magicWord = MAGIC_WORD;
    memcpy(txMsgBuff, &magicWord, sizeof(magicWord));
    txMsgBuff[MAGIC_WORD_LEN] = msgType;
    memcpy(&(txMsgBuff[MAGIC_WORD_LEN+MSG_DATA_TYPE_LEN]), &(uartTxMsgLenFromType[msgType]), sizeof(uartTxMsgLenFromType[msgType]));
    if(size != uartTxMsgLenFromType[msgType]){
        //Serial.rintln("Tx msg wrong data size");
        return;
    }
    memcpy(&(txMsgBuff[HEADER_LEN]), content, size);

    //CRC
    uint16_t crc = crc16(txMsgBuff, HEADER_LEN + uartTxMsgLenFromType[msgType]);
    // txMsgBuff[HEADER_LEN + uartTxMsgLenFromType[msgType]] = crc;
    memcpy(&(txMsgBuff[HEADER_LEN + uartTxMsgLenFromType[msgType]]), &crc, CRC_LEN);

    //Serial.rintln();
    //Serial.rint("Msg to send: ");
    for (uint8_t i = 0; i < HEADER_LEN + uartTxMsgLenFromType[msgType] + CRC_LEN; i++){
        //Serial.rint(txMsgBuff[i], HEX);
    }
    //Serial.rintln();

    //SEND msg
    uint8_t sizeTotal = HEADER_LEN + uartTxMsgLenFromType[msgType] + CRC_LEN;
    pSerial->write(txMsgBuff, sizeTotal);

    

    //Serial.rintln("Msg sent");

    delay(100);

    /////// Receive message
    rxTimeoutEnd = millis() + timeout;
    while ((uint16_t)(pSerial->available()) < (HEADER_LEN) && rxTimeoutEnd > millis()){}
    if(rxTimeoutEnd <= millis()){
        //Serial.rintln("Timeout reached");
        return;
    }
    
    pSerial->readBytes(rxHeaderBuff, HEADER_LEN);

    //Serial.rint("Rx header: ");
    for(uint8_t i = 0; i<HEADER_LEN; i++){
        //Serial.rint(rxHeaderBuff[i], HEX);
    }
    //Serial.rintln();

    uint32_t magicWordBuff = 0;
    memcpy(&magicWordBuff, rxHeaderBuff, sizeof(magicWordBuff));
    if(magicWordBuff != MAGIC_WORD){
        //pas le header
        //Serial.rint("wrong header msg, Expected: ");
        //Serial.rint(MAGIC_WORD, HEX);
        //Serial.rint(", Received: ");
        //Serial.rintln(magicWordBuff, HEX);
    }
    //Serial.rint("Magic word RX: ");
    //Serial.rintln(MAGIC_WORD, HEX);

    // getType
    eUartMsgType msgRx = (eUartMsgType)rxHeaderBuff[MAGIC_WORD_LEN];
    if (msgType != msgRx){
        //unknown message type
        //Serial.rint("Received message not same as sent, Expected: ");
        //Serial.rint(msgType);
        //Serial.rint(", Received: ");
        //Serial.rintln(msgRx);
    }

    // getLength
    uint16_t rxDataLen = 0;
    memcpy(&rxDataLen, &(rxHeaderBuff[MAGIC_WORD_LEN+MSG_DATA_TYPE_LEN]), sizeof(rxDataLen));
    if (rxDataLen != uartRxMsgLenFromType[msgType]){
        //Serial.rint("Wrong data length received, Expected: ");
        //Serial.rint(uartRxMsgLenFromType[msgType]);
        //Serial.rint(", Received: ");
        //Serial.rintln(rxDataLen);
    }

    rxTimeoutEnd = millis() + timeout;
    while (((uint16_t)(pSerial->available()) < (rxDataLen + CRC_LEN)) && rxTimeoutEnd > millis()){}
    if(rxTimeoutEnd <= millis()){
        //Serial.rintln("Timeout reached");
        return;
    }
    
    pSerial->readBytes(rxDataBuff, rxDataLen + CRC_LEN);

    // crc
    // uint8_t rxMsgBuff[HEADER_LEN + rxDataLen] = {0};
    // memcpy(rxMsgBuff, rxHeaderBuff, HEADER_LEN);
    // memcpy(rxMsgBuff, &(rxHeaderBuff[HEADER_LEN]), rxDataLen);//devrait pas marcher non plus c pas dans header buff
    // uint16_t crcRx = crc16(rxMsgBuff, HEADER_LEN + rxDataLen);
    // if (crcRx != *((uint16_t*)(&rxMsgBuff[HEADER_LEN + rxDataLen]))){//devrait pas vraiment marcher switch pour rxdatabuff
    //     //Serial.rint("wrong crc, Calculated: ");
    //     //Serial.rint(crcRx, HEX);
    //     //Serial.rint(", Received: ");
    //     //Serial.rintln(*((uint16_t*)(&rxMsgBuff[HEADER_LEN + rxDataLen])), HEX);
    // }
    uint8_t rxMsgBuff[HEADER_LEN + rxDataLen] = {0};
    memcpy(rxMsgBuff, rxHeaderBuff, HEADER_LEN);
    memcpy(&(rxMsgBuff[HEADER_LEN]), rxDataBuff, rxDataLen);
    uint16_t crcRx = crc16(rxMsgBuff, HEADER_LEN + rxDataLen);
    if (crcRx != *((uint16_t*)(&rxDataBuff[rxDataLen]))){
        //Serial.rint("wrong crc, Calculated: ");
        //Serial.rint(crcRx, HEX);
        //Serial.rint(", Received: ");
        //Serial.rintln(*((uint16_t*)(&rxMsgBuff[HEADER_LEN + rxDataLen])), HEX);
    }

    //Print data
    //Serial.rint("Tx data for Msg type: ");
    //Serial.rintln(msgType);
    //Serial.rint("With content: ");
    for (uint8_t i = 0; i < size; i++){
        //Serial.rint(content[i], HEX);
    }
    //Serial.rintln();
    //Serial.rint("Rx data: ");
    for (uint8_t i = 0; i < rxDataLen; i++){
        //Serial.rint(rxDataBuff[i], HEX);
    }
    //Serial.rintln();
    //Serial.rint("Total msg: ");
    for (uint8_t i = 0; i < HEADER_LEN; i++){
        //Serial.rint(rxHeaderBuff[i], HEX);
    }
    for (uint8_t i = 0; i < rxDataLen + CRC_LEN; i++){
        //Serial.rint(rxDataBuff[i], HEX);
    }
    //Serial.rintln();

    if(readFloat) {
        //Serial.rintln("Float val: ");
        uint8_t nbFloat = rxDataLen/sizeof(float);
        for (uint8_t i = 0; i < nbFloat; i++){
            //Serial.rint(i);
            //Serial.rint(": ");
            //Serial.rintln(*((float*)(&(rxDataBuff[i*sizeof(float)]))));
        }           
    }

    resetArray(txMsgBuff, HEADER_LEN + MAX_TX_DATA_LEN + CRC_LEN);
    resetArray(rxHeaderBuff, HEADER_LEN);
    resetArray(rxDataBuff, MAX_RX_DATA_LEN + CRC_LEN);

    //Serial.rint("Remaining data: ");
    uint16_t remainingByte = pSerial->available();
    for(uint8_t i = 0; i<remainingByte; i++){
        //Serial.rint(pSerial->read());
    }
    //Serial.rintln();
}

void SerialUart::resetArray(uint8_t* start, uint8_t size){
    for (uint8_t i = 0; i < size; i++) {
        start[i] = 0;
    }
}

// Function to calculate CRC16/MODBUS checksum
// width=16 poly=0x8005 init=0xffff refin=true refout=true xorout=0x0000 check=0x4b37 residue=0x0000 name="CRC-16/MODBUS"
uint16_t SerialUart::crc16(const uint8_t* data, uint8_t length){
    uint16_t crc = 0xFFFF; // Initial value
    uint16_t poly = 0xA001;

    for (uint8_t i = 0; i < length; ++i) {
        crc ^= static_cast<uint16_t>(data[i]);
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ poly;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}