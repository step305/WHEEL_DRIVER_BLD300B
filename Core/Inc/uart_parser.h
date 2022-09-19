#ifndef WHEEL_DRIVER_BLD300B_UART_PARSER_H
#define WHEEL_DRIVER_BLD300B_UART_PARSER_H

#include "main.h"
#include "string.h"
#include "stdio.h"

#define RX_BUFFER_LEN	255
#define RX_TIMEOUT_MS   2000

typedef enum ParserStates {WAIT_START_PACKET, WAIT_END_PACKET} ParserStatesValues;
#pragma pack(1)
typedef struct {
    ParserStatesValues state;
    char buffer[RX_BUFFER_LEN];
    uint16_t data_cnt;
    float speed1;
    float speed2;
    uint8_t is_request;
    uint8_t emergency_stop;
    uint32_t timestamp;
} ParserStateType;
#pragma pack()

#pragma pack(1)
typedef struct {
    float speed1;
    float speed2;
    uint8_t is_request;
    uint8_t emergency_stop;
    uint32_t timestamp;
} CommandType;
#pragma pack()

extern uint8_t rx_buffer_ready;
extern CommandType rx_command;
extern uint8_t rx_buffer;

void process_rx_byte(uint8_t next_byte);
uint8_t is_rx_timeout();

#endif //WHEEL_DRIVER_BLD300B_UART_PARSER_H
