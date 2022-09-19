#include "uart_parser.h"

ParserStateType rx_state;
CommandType rx_command;

uint8_t rx_buffer_ready = 1;
uint8_t rx_buffer = 0;

int8_t parse_next_byte(char byte, ParserStateType* state) {
    int8_t result = -1;
    float speed1 = 0.0f;
    float speed2 = 0.0f;
    char request = 0;

    switch((state->state)) {
        case(WAIT_START_PACKET):
            if(byte == '$') {
                memset(state->buffer, 0, 255);
                state->data_cnt = 0;
                state->buffer[state->data_cnt] = byte;
                state->data_cnt++;
                state->state = WAIT_END_PACKET;
            }
            break;
        case(WAIT_END_PACKET):
            if (state->data_cnt < RX_BUFFER_LEN) {
                if(byte=='\n') {
                    if (state->data_cnt > 0) {
                        if (sscanf(&state->buffer[0], "$%c W1:%f W2:%f", &request, &speed1, &speed2) == 3) {
                            if (request == '?') {
                                state->is_request = 1;
                                state->emergency_stop = 0;
                            } else if (request == '!'){
                                state->emergency_stop = 1;
                                state->is_request = 0;
                            } else {
                                state->is_request = 0;
                                state->emergency_stop = 0;
                            }
                            state->speed1 = speed1;
                            state->speed2 = speed2;
                            state->timestamp = HAL_GetTick();
                            result = 1;
                        }
                    }
                    state->state = WAIT_START_PACKET;
                } else {
                    state->buffer[state->data_cnt] = byte;
                    state->data_cnt++;
                    state->state = WAIT_END_PACKET;
                }
            } else {
                state->state = WAIT_START_PACKET;
            }
            break;
        default:
            state->state = WAIT_START_PACKET;
            break;
    }
    return result;
}

void process_rx_byte(uint8_t next_byte) {
    if (parse_next_byte(next_byte, &rx_state) > 0) {
        rx_command.speed1 = rx_state.speed1;
        rx_command.speed2 = rx_state.speed2;
        rx_command.is_request = rx_state.is_request;
        rx_command.emergency_stop = rx_state.emergency_stop;
        rx_command.timestamp = rx_state.timestamp;
        rx_buffer_ready = 1;
    }
}

uint8_t is_rx_timeout() {
    if ((HAL_GetTick() - rx_command.timestamp) > RX_TIMEOUT_MS) {
        return 1;
    }
    return 0;
}
