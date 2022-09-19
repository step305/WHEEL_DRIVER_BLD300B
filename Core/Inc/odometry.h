#ifndef WHEEL_DRIVER_BLD300B_ODOMETRY_H
#define WHEEL_DRIVER_BLD300B_ODOMETRY_H

#include "main.h"
#include "tim.h"
#include "string.h"

#define DMA_TIMER_CHANNEL_HALF_LEN	2
#define DMA_TIMER_CHANNEL_LEN	(2*DMA_TIMER_CHANNEL_HALF_LEN)
#define WHEEL_POLES 13.0f
#define TIMEOUT_ODOMETRY_MS	70

#define MOVING_AVERAGE_FILTER_LEN 2
typedef struct {
    float data[MOVING_AVERAGE_FILTER_LEN];
    float new_inp;
    float out;
    float filter_state[2];
    uint32_t index;
} MOVING_AVERAGE_FILTER_STATE;

typedef struct {
    float speed;
    uint32_t timestamp;
} ODOMETRY_MEASUREMENT;

extern __IO uint8_t dma_timer_channel_1_ready;
extern __IO uint8_t dma_timer_channel_2_ready;
extern __IO uint32_t dma_buffer_timer_channel_1[DMA_TIMER_CHANNEL_LEN];
extern __IO uint32_t dma_buffer_timer_channel_2[DMA_TIMER_CHANNEL_LEN];
extern ODOMETRY_MEASUREMENT wheel_1_odometry;
extern ODOMETRY_MEASUREMENT wheel_2_odometry;

void calc_wheel_speed();
void validate_odometry();

#endif //WHEEL_DRIVER_BLD300B_ODOMETRY_H
