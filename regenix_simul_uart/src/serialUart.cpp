#include "serialUart.h"

#define MOCK_UART
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
#define MAX_RX_DATA_LEN (sizeof(uint32_t))
#define MAX_TX_DATA_LEN (sizeof(uint32_t)*3)
#define CRC_LEN (sizeof(uint16_t))

uint8_t rxHeaderBuff[HEADER_LEN] = {0};
uint8_t rxDataBuff[MAX_RX_DATA_LEN+CRC_LEN] = {0};
uint8_t txMsgBuff[HEADER_LEN+MAX_TX_DATA_LEN+CRC_LEN] = {0};
uint16_t uartRxMsgLenFromType[] = {0, 0, 0, 0, 1, 4, 4, 4, 4};
uint16_t uartTxMsgLenFromType[] = {12, 12, 1, 2, 1, 1, 1, 1, 1};

SerialUart::SerialUart(HardwareSerial* pHandle){
    pSerial = pHandle;
    pSerial->begin(BAUDRATE);

}

void SerialUart::processUart(){
    
    switch (state) {
    case UART_SM_WAIT_HEADER:
        if (pSerial->available() >= HEADER_LEN){
            state = UART_SM_VERIFY_HEADER;
            Serial.println("\nbytes available!!!");
        }
        break;

    case UART_SM_VERIFY_HEADER:
        if (readHeader() == true) {
            state = UART_SM_WAIT_DATA;
            Serial.println("To wait data");
        } else {
            state = UART_SM_WAIT_HEADER;
            Serial.println("Return to wait header");
        }
        break;

    case UART_SM_WAIT_DATA:
        if (pSerial->available() >= (uartRxMsgLenFromType[msgType] + CRC_LEN)){
            state = UART_SM_READ_DATA;
            Serial.println("data bytes available!!!");
        }
        break;

    case UART_SM_READ_DATA:
        if(readData() == true) {
            state = UART_SM_SEND_RESPONSE;
        } else {
            msgErr = 1;
            state = UART_SM_SEND_RESPONSE;// but its an error
        }

    case UART_SM_SEND_RESPONSE:
        if(sendMsg() == true) {
            state = UART_SM_WAIT_HEADER;
        } 
        break;
    
    default:
        break;
    }
}

void SerialUart::test(){
    // uint32_t magicWord = MAGIC_WORD;
    // uint16_t size = 12;
    // memcpy(rxHeaderBuff, &magicWord, sizeof(magicWord));
    // rxHeaderBuff[MAGIC_WORD_LEN] = 0x01;
    // memcpy(&(rxHeaderBuff[MAGIC_WORD_LEN+MSG_DATA_TYPE_LEN]), &size, sizeof(size));
    // readHeader();
    // uint8_t message[] = { 0x01, 0x04, 0x00, 0x00, 0x00, 0x02 };
    // size_t length = sizeof(message) / sizeof(message[0]);
    
    // uint16_t crc = crc16(message, length);
}

bool SerialUart::sendMsg(){
    // HEADER
    uint32_t magicWord = MAGIC_WORD;
    memcpy(txMsgBuff, &magicWord, sizeof(magicWord));
    txMsgBuff[MAGIC_WORD_LEN] = msgType;
    memcpy(&(txMsgBuff[MAGIC_WORD_LEN+MSG_DATA_TYPE_LEN]), &(uartTxMsgLenFromType[msgType]), sizeof(uartTxMsgLenFromType[msgType]));

    //DATA
    switch (msgType) {
        case MSG_TYPE_LIQUID_STATUS:
        {
            float ph = getPh();
            float DO = getDO();
            float temp = getTemp();
            memcpy(&(txMsgBuff[HEADER_LEN]), &ph, sizeof(ph));
            memcpy(&(txMsgBuff[HEADER_LEN+sizeof(uint32_t)]), &DO, sizeof(DO));
            memcpy(&(txMsgBuff[HEADER_LEN+(sizeof(uint32_t)*2)]), &temp, sizeof(temp));
            break;
        }
        case MSG_TYPE_PUMP_STATUS:
        {
            float pumps[] = {getPumpVal(0), getPumpVal(1), getPumpVal(2)};
            memcpy(&(txMsgBuff[HEADER_LEN]), pumps, sizeof(float)*3);
            break;
        }
        case MSG_TYPE_VALVE_STATUS:
        {
            uint8_t pinchValves = 0;
            for (uint8_t i = 0; i < 7; i++){
                pinchValves = getValveStatus(i) << i;
            }
            memcpy(&(txMsgBuff[HEADER_LEN]), &pinchValves, sizeof(uint8_t));
            break;
        }
        case MSG_TYPE_GET_STATE:
        {
            uint8_t state = getState();
            uint8_t subState = getSubState();
            memcpy(&(txMsgBuff[HEADER_LEN]), &state, sizeof(state));
            memcpy(&(txMsgBuff[HEADER_LEN+sizeof(state)]), &subState, sizeof(subState));
            break;
        }
        case MSG_TYPE_SET_STATE:
        {
            txMsgBuff[HEADER_LEN] = msgErr;
            break;
        }
        case MSG_TYPE_SET_PUMP_GOAL:
        {
            txMsgBuff[HEADER_LEN] = msgErr;
            break;
        }
        case MSG_TYPE_SET_DO_GOAL:
        {
            txMsgBuff[HEADER_LEN] = msgErr;
            break;
        }
        case MSG_TYPE_SET_TEMP_GOAL:
        {
            txMsgBuff[HEADER_LEN] = msgErr;
            break;
        }
        case MSG_TYPE_SET_PH_GOAL:
        {
            txMsgBuff[HEADER_LEN] = msgErr;
            break;
        }
        default:
            txMsgBuff[HEADER_LEN] = 1; // unknown type
            
            break;
    }

    //CRC
    uint16_t crc = crc16(txMsgBuff, HEADER_LEN + uartTxMsgLenFromType[msgType]);
    memcpy(&(txMsgBuff[HEADER_LEN+uartTxMsgLenFromType[msgType]]), &crc, sizeof(crc));

    //SEND msg
    uint8_t size = HEADER_LEN + uartTxMsgLenFromType[msgType] + CRC_LEN;
    uint8_t nbByte = pSerial->write(txMsgBuff, size);
    if (nbByte == 0){
        return false;
    }

    Serial.print("Total msg sent: ");
    for(uint8_t i = 0; i<HEADER_LEN + uartTxMsgLenFromType[msgType]+CRC_LEN; i++){
        Serial.print(txMsgBuff[i], HEX);
    }
    Serial.println();

    msgErr = 0;
    msgType = 0;
    memset(rxDataBuff, 0, MAX_RX_DATA_LEN+CRC_LEN);
    memset(rxHeaderBuff, 0, HEADER_LEN);
    memset(txMsgBuff, 0, HEADER_LEN+MAX_TX_DATA_LEN+CRC_LEN);
    return true;
}

bool SerialUart::readHeader(){
    pSerial->readBytes(rxHeaderBuff, HEADER_LEN);

    Serial.print("Rx header: ");
    for(uint8_t i = 0; i<HEADER_LEN; i++){
        Serial.print(rxHeaderBuff[i], HEX);
    }
    Serial.println();

    uint32_t magicWordBuff = 0;
    memcpy(&magicWordBuff, rxHeaderBuff, sizeof(magicWordBuff));
    if(magicWordBuff != MAGIC_WORD){
        //pas le header
        Serial.print("wrong header msg, Expected: ");
        Serial.print(MAGIC_WORD, HEX);
        Serial.print(", Received: ");
        Serial.println(magicWordBuff, HEX);
        return false;
    }

    Serial.print("Magic word RX: ");
    Serial.println(MAGIC_WORD, HEX);

    // getType
    msgType = rxHeaderBuff[MAGIC_WORD_LEN];
    if (msgType >= MSG_TYPE_MAX){
        //unknown message type
        Serial.print("Received message type unknown, Max: ");
        Serial.print(MSG_TYPE_MAX);
        Serial.print(", Received: ");
        Serial.println(msgType);
        return false;
    }
    Serial.print("Type: ");
    Serial.println(msgType);

    // getLength
    uint16_t rxDataLen = 0;
    memcpy(&rxDataLen, &(rxHeaderBuff[MAGIC_WORD_LEN+MSG_DATA_TYPE_LEN]), sizeof(rxDataLen));
    if (rxDataLen != uartRxMsgLenFromType[msgType]){
        Serial.print("Wrong data length received, Expected: ");
        Serial.print(uartRxMsgLenFromType[msgType]);
        Serial.print(", Received: ");
        Serial.println(rxDataLen);
        return false;
    }    
    return true;
}

bool SerialUart::readData(){
    pSerial->readBytes(rxDataBuff, uartRxMsgLenFromType[msgType] + CRC_LEN);
    
    Serial.print("Rx data: ");
    for(uint8_t i = 0; i<uartRxMsgLenFromType[msgType] + CRC_LEN; i++){
        Serial.print(rxDataBuff[i], HEX);
    }
    Serial.println();

    //verify CRC
    uint8_t rxMsgBuff[HEADER_LEN + uartRxMsgLenFromType[msgType]];
    memcpy(rxMsgBuff, rxHeaderBuff, HEADER_LEN);
    memcpy(&(rxMsgBuff[HEADER_LEN]), rxDataBuff, uartRxMsgLenFromType[msgType]);
    uint16_t crc = crc16(rxMsgBuff, HEADER_LEN + uartRxMsgLenFromType[msgType]);
    if (crc != *((uint16_t*)(&rxDataBuff[uartRxMsgLenFromType[msgType]]))){
        Serial.print("wrong crc, Calculated: ");
        Serial.print(crc, HEX);
        Serial.print(", Received: ");
        Serial.println(*((uint16_t*)(&rxDataBuff[uartRxMsgLenFromType[msgType]])), HEX);
        return false;
    }

    Serial.print("Total msg: ");
    for(uint8_t i = 0; i<HEADER_LEN + uartRxMsgLenFromType[msgType]; i++){
        Serial.print(rxMsgBuff[i], HEX);
    }
    Serial.println(*((uint16_t*)(&rxDataBuff[uartRxMsgLenFromType[msgType]])), HEX);

    //read msg
    switch (msgType) {
        case MSG_TYPE_LIQUID_STATUS:
        {
            //check with state if liquid is in the bioreactor 
            break;
        }
        case MSG_TYPE_PUMP_STATUS:
        {
            break;
        }
        case MSG_TYPE_VALVE_STATUS:
        {
            break;
        }
        case MSG_TYPE_GET_STATE:
        {
            break;
        }
        case MSG_TYPE_SET_STATE:
        {
            uint8_t reqState = rxDataBuff[0];
            if (reqState >= SM_MAX) {
                return false;
            }
            setState(reqState);
            break;
        }
        case MSG_TYPE_SET_PUMP_GOAL:
        {
            float pumpGoal = *((float*)rxDataBuff);
            setPumpGoal(pumpGoal);
            break;
        }
        case MSG_TYPE_SET_DO_GOAL:
        {
            float reqDO = *((float*)rxDataBuff);
            setDOGoal(reqDO);
            break;
        }
        case MSG_TYPE_SET_TEMP_GOAL:
        {
            float reqTemp = *((float*)rxDataBuff);
            setTempGoal(reqTemp);
            break;
        }
        case MSG_TYPE_SET_PH_GOAL:
        {
            float reqPh = *((float*)rxDataBuff);
            setPhGoal(reqPh);
            break;
        }
        default:
        {
            return false;
            break;
        }
    }

    return true;
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