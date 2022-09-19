#include "control_wheel.h"

int32_t torque_value[2] = {0,};
float control_speed[2] = {0,};
float target_speed[2] = {0.0f,};
uint32_t loop_cnt = 0;

void set_target_speed(uint8_t wheel, float speed) {
    if (fabsf(speed) < MIN_SPEED) {
        speed = 0.0f;
    }
    target_speed[wheel] = speed;
}

void stop() {
    set_target_speed(0, 0.0f);
    set_target_speed(1, 0.0f);
}

void emergency_stop() {
    stop();
    HAL_GPIO_WritePin(WHEEL1_BRK_GPIO_Port, WHEEL1_BRK_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(WHEEL2_BRK_GPIO_Port, WHEEL2_BRK_Pin, GPIO_PIN_SET);
}

void enable_wheels() {
    HAL_GPIO_WritePin(WHEEL1_BRK_GPIO_Port, WHEEL1_BRK_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(WHEEL2_BRK_GPIO_Port, WHEEL2_BRK_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(WHEEL1_EN_GPIO_Port, WHEEL1_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(WHEEL2_EN_GPIO_Port, WHEEL2_EN_Pin, GPIO_PIN_SET);
}

int32_t calc_torque(float speed) {
    speed = speed / MIN_SPEED;
    if (fabsf(speed) > 1.0f) {
        return (int32_t)(250.0f * speed);
    } else {
        return 0;
    }
}

void force_wheels() {
    HAL_GPIO_WritePin(STOP_SIGNAL_GPIO_Port, STOP_SIGNAL_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(REVERSE_LIGHTS_GPIO_Port, REVERSE_LIGHTS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LEFT_TURN_LIGHT_GPIO_Port, LEFT_TURN_LIGHT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RIGHT_TURN_LIGHT_GPIO_Port, RIGHT_TURN_LIGHT_Pin, GPIO_PIN_RESET);

    // wheel1 - reverse, wheel2 - reverse -> reverse light on
    // wheel1 - reverse, wheel2 - forward -> left light on
    // wheel1 - forward, wheel2 - reverse -> right light on
    // wheel1 - stop, wheel2 - stop -> stop light on

    if (torque_value[0] < 0) {
        HAL_GPIO_WritePin(WHEEL1_FR_GPIO_Port, WHEEL1_FR_Pin, GPIO_PIN_SET);
        if (torque_value[1] < 0) {
            HAL_GPIO_WritePin(REVERSE_LIGHTS_GPIO_Port, REVERSE_LIGHTS_Pin, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(LEFT_TURN_LIGHT_GPIO_Port, LEFT_TURN_LIGHT_Pin, GPIO_PIN_SET);
        }
    } else {
        HAL_GPIO_WritePin(WHEEL1_FR_GPIO_Port, WHEEL1_FR_Pin, GPIO_PIN_RESET);
        if (torque_value[1] < 0) {
            HAL_GPIO_WritePin(RIGHT_TURN_LIGHT_GPIO_Port, RIGHT_TURN_LIGHT_Pin, GPIO_PIN_SET);
        }
    }
    if (torque_value[1] < 0) {
        HAL_GPIO_WritePin(WHEEL2_FR_GPIO_Port, WHEEL2_FR_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(WHEEL2_FR_GPIO_Port, WHEEL2_FR_Pin, GPIO_PIN_RESET);
    }
    if ((torque_value[0] == 0) && (torque_value[1] == 0)) {
        HAL_GPIO_WritePin(STOP_SIGNAL_GPIO_Port, STOP_SIGNAL_Pin, GPIO_PIN_SET);
    }

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, abs(torque_value[0]));
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, abs(torque_value[1]));
    HAL_DACEx_DualStart(&hdac1);
}

float sign(float x) {
    if (x >= 0.0f) {
        return 1.0f;
    } else {
        return -1.0f;
    }
}

void control_iterate(uint8_t wheel, ODOMETRY_MEASUREMENT odom) {
    if (loop_cnt % CONTROL_LOOP_SKIP == 0) {
        if (fabsf(odom.speed - target_speed[wheel]) > 10.0f) {
            control_speed[wheel] += sign(target_speed[wheel] - odom.speed);
            // odom = 1, target = 2 -> control += 1
            // odom = 2, target = 1 -> control -= 1
            // odom = -1, target = -2 -> control -= 1
            // odom = -2, target = -1 -> control += 1
        }
    }
    if (fabsf(target_speed[wheel]) < MIN_SPEED) {
        control_speed[wheel] = 0.0f;
    }
    torque_value[wheel] = calc_torque(control_speed[wheel]);
    loop_cnt++;
}
