#include "odometry.h"

float B[3] = {0.067455273889072f, 0.134910547778144f, 0.067455273889072f};
float A[2] = {-1.142980502539901f, 0.412801598096189f};

__IO uint32_t dma_buffer_timer_channel_1[DMA_TIMER_CHANNEL_LEN] = {0,};
__IO uint32_t dma_buffer_timer_channel_2[DMA_TIMER_CHANNEL_LEN] = {0,};

__IO uint32_t dma_buffer_timer_channel_1_safe[DMA_TIMER_CHANNEL_HALF_LEN + 1] = {0,};
__IO uint32_t dma_buffer_timer_channel_2_safe[DMA_TIMER_CHANNEL_HALF_LEN + 1] = {0,};

__IO uint8_t dma_timer_channel_1_ready = 0;
__IO uint8_t dma_timer_channel_2_ready = 0;

uint32_t timestamp_channel_1 = 0;
uint32_t timestamp_channel_2 = 0;

MOVING_AVERAGE_FILTER_STATE filter_1_state;
MOVING_AVERAGE_FILTER_STATE filter_2_state;

ODOMETRY_MEASUREMENT wheel_1_odometry;
ODOMETRY_MEASUREMENT wheel_2_odometry;

void HAL_TIM_IC_CaptureHalfCpltCallback(TIM_HandleTypeDef *htim) {
    switch (htim->Channel) {
        case HAL_TIM_ACTIVE_CHANNEL_1:
            memcpy((uint32_t *)&dma_buffer_timer_channel_1_safe[1], (uint32_t *)dma_buffer_timer_channel_1,
                   DMA_TIMER_CHANNEL_HALF_LEN*4);
            dma_timer_channel_1_ready = 1;
            break;
        case HAL_TIM_ACTIVE_CHANNEL_2:
            memcpy((uint32_t *)&dma_buffer_timer_channel_2_safe[1], (uint32_t *)dma_buffer_timer_channel_2,
                   DMA_TIMER_CHANNEL_HALF_LEN*4);
            dma_timer_channel_2_ready = 1;
            break;
        default:
            break;
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    switch (htim->Channel) {
        case HAL_TIM_ACTIVE_CHANNEL_1:
            memcpy((uint32_t *)&dma_buffer_timer_channel_1_safe[1],
                   (uint32_t *)&dma_buffer_timer_channel_1[DMA_TIMER_CHANNEL_HALF_LEN], DMA_TIMER_CHANNEL_HALF_LEN*4);
            dma_timer_channel_1_ready = 1;
            break;
        case HAL_TIM_ACTIVE_CHANNEL_2:
            memcpy((uint32_t *)&dma_buffer_timer_channel_2_safe[1],
                   (uint32_t *)&dma_buffer_timer_channel_2[DMA_TIMER_CHANNEL_HALF_LEN], DMA_TIMER_CHANNEL_HALF_LEN*4);
            dma_timer_channel_2_ready = 1;
            break;
        default:
            break;
    }
}

float calc_mean_diff(__IO uint32_t array[], uint32_t len) {
    uint32_t sum_delta = 0;
    for (uint32_t i = 1; i < len; i++) {
        sum_delta += (uint32_t)(array[i] - array[i - 1]);
    }
    return ((float)sum_delta) / (float)(len - 1);
}

float calc_mean(float array[], uint32_t len) {
    float sum = 0.0f;
    for (uint32_t i = 0; i < len; i++) {
        sum += array[i];
    }
    return sum / (float)len;
}

void iterate_moving_average_filter(MOVING_AVERAGE_FILTER_STATE *state) {
    state->data[state->index] = state->new_inp;
    state->index++;
    if (state->index == MOVING_AVERAGE_FILTER_LEN) {
        state->index = 0;
    }
    state->out = 170000000.0f /
            calc_mean(state->data, MOVING_AVERAGE_FILTER_LEN) * 60.0f / WHEEL_POLES / 60.0f * 360.0f / 3.0f/1.1125f;
}

float filter_butter(float x, float Z[]){
    float y;
    y = Z[0] + x * B[0];
    Z[0] = (Z[1] + x * B[1]) + -y * A[0];
    Z[1] = x * B[2] + -y * A[1];
    y = 170000000.0f / y * 10.0f;
    return y;
}

void calc_wheel_speed() {
    if (dma_timer_channel_1_ready) {
        dma_timer_channel_1_ready = 0;
        wheel_1_odometry.timestamp = HAL_GetTick();
        filter_1_state.new_inp = calc_mean_diff(dma_buffer_timer_channel_1_safe, DMA_TIMER_CHANNEL_HALF_LEN + 1);
        //filter_1_state.out = filter_butter(filter_1_state.new_inp, filter_1_state.filter_state);
        iterate_moving_average_filter(&filter_1_state);
        dma_buffer_timer_channel_1_safe[0] = dma_buffer_timer_channel_1_safe[DMA_TIMER_CHANNEL_HALF_LEN];
        wheel_1_odometry.speed = filter_1_state.out;
    }
    if (dma_timer_channel_2_ready) {
        dma_timer_channel_2_ready = 0;
        wheel_2_odometry.timestamp = HAL_GetTick();
        filter_2_state.new_inp = calc_mean_diff(dma_buffer_timer_channel_2_safe, DMA_TIMER_CHANNEL_HALF_LEN + 1);
        //filter_1_state.out = filter_butter(filter_1_state.new_inp, filter_1_state.filter_state);
        iterate_moving_average_filter(&filter_2_state);
        dma_buffer_timer_channel_2_safe[0] = dma_buffer_timer_channel_2_safe[DMA_TIMER_CHANNEL_HALF_LEN];
        wheel_2_odometry.speed = filter_2_state.out;
    }
}

void reset_wheel_odometry(uint8_t wheel) {
    if (wheel == 0) {
        filter_1_state.out = 0.0f;
        wheel_1_odometry.speed = 0.0f;
    } else if (wheel == 1) {
        filter_2_state.out = 0.0f;
        wheel_2_odometry.speed = 0.0f;
    }
}

void validate_odometry() {
    uint32_t timestamp = HAL_GetTick();
    if ((timestamp - wheel_1_odometry.timestamp)>= TIMEOUT_ODOMETRY_MS) {
        reset_wheel_odometry(0);
    }
    if ((timestamp - wheel_2_odometry.timestamp)>= TIMEOUT_ODOMETRY_MS) {
        reset_wheel_odometry(1);
    }
}
