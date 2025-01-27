#ifndef SERIALUART_H_
#define SERIALUART_H_

#include <Arduino.h>

class SerialUart
{

public:

    typedef enum {
        MSG_TYPE_LIQUID_STATUS = 0,
        MSG_TYPE_PUMP_STATUS,
        MSG_TYPE_VALVE_STATUS,
        MSG_TYPE_GET_STATE,
        MSG_TYPE_SET_STATE,
        MSG_TYPE_SET_PUMP_GOAL,
        MSG_TYPE_SET_DO_GOAL,
        MSG_TYPE_SET_TEMP_GOAL,
        MSG_TYPE_SET_PH_GOAL,

        MSG_TYPE_MAX
        // !!!! si des modifs sont fait modifie aussi !!!!
        // !!!! uartRxMsgLenFromType et uartTxMsgLenFromType !!!!
    } eUartMsgType;

    typedef enum {
        UART_SM_WAIT_HEADER = 0,
        UART_SM_VERIFY_HEADER,
        UART_SM_WAIT_DATA,
        UART_SM_READ_DATA,
        UART_SM_SEND_RESPONSE,

        UART_SM_MAX
    } eStateMachineReadUart;

    SerialUart(HardwareSerial* pHandle);

    void sendMsgRxRsp(eUartMsgType msgType, uint8_t *content, uint16_t size, uint32_t timeout, bool readFloat);

private:
    HardwareSerial* pSerial;
    eStateMachineReadUart state = UART_SM_WAIT_HEADER;
    uint8_t msgType = 0;

    uint8_t msgErr = 0;

    uint16_t crc16(const uint8_t* data, uint8_t length);
    void resetArray(uint8_t* start, uint8_t size);
};

#endif //SERIALUART_H_