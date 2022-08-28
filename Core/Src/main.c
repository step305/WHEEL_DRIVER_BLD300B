/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MOVING_AVERAGE_FILTER_LEN 2
typedef struct {
    float data[MOVING_AVERAGE_FILTER_LEN];
    float new_inp;
    float out;
    float filter_state[2];
    uint32_t index;
} MOVING_AVERAGE_FILTER_STATE;

#define RX_BUFFER_LEN	255
typedef enum ParserStates {WAIT_START_PACKET, WAIT_END_PACKET} ParserStatesValues;
#pragma pack(1)
typedef struct {
    ParserStatesValues state;
    char buffer[RX_BUFFER_LEN];
    uint16_t data_cnt;
    float speed1;
    float speed2;
    uint8_t is_request;
} ParserStateType;
#pragma pack()

#pragma pack(1)
typedef struct {
    float speed1;
    float speed2;
    uint8_t is_request;
} CommandType;
#pragma pack()
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Tx_BUFFER_LEN  256
#define DMA_TIMER_CHANNEL_HALF_LEN	2
#define DMA_TIMER_CHANNEL_LEN	(2*DMA_TIMER_CHANNEL_HALF_LEN)
#define TIMEOUT_MS	70
#define WHEEL_POLES 13.0f
#define UNIT_TORQUE 80.0f
#define MIN_SPEED 80.0f
#define NumCommands	6

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float B[3] = {0.067455273889072f, 0.134910547778144f, 0.067455273889072f};
float A[2] = {-1.142980502539901f, 0.412801598096189f};

uint8_t tx_buffer_ready = 1;
uint8_t tx_buffer[Tx_BUFFER_LEN] = {0,};

uint8_t rx_buffer_ready = 1;
uint8_t rx_buffer = 0;
ParserStateType rx_state;
CommandType rx_command;

__IO uint32_t dma_buffer_timer_channel_1[DMA_TIMER_CHANNEL_LEN] = {0,};
__IO uint32_t dma_buffer_timer_channel_2[DMA_TIMER_CHANNEL_LEN] = {0,};

__IO uint32_t dma_buffer_timer_channel_1_safe[DMA_TIMER_CHANNEL_HALF_LEN + 1] = {0,};
__IO uint32_t dma_buffer_timer_channel_2_safe[DMA_TIMER_CHANNEL_HALF_LEN + 1] = {0,};

__IO uint8_t dma_timer_channel_1_ready = 0;
__IO uint8_t dma_timer_channel_2_ready = 0;

uint32_t timestamp_channel_1 = 0;
uint32_t timestamp_channel_2 = 0;

uint32_t timestamp = 0;

MOVING_AVERAGE_FILTER_STATE filter_1_state;
MOVING_AVERAGE_FILTER_STATE filter_2_state;

uint32_t torque1_value = 0;
uint32_t torque2_value = 0;



__IO uint8_t timer100Hz_elapsed = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        tx_buffer_ready = 1;
    }
}

void try_send_data(uint8_t *buffer, uint16_t len) {
    if (tx_buffer_ready == 1) {
        tx_buffer_ready = 0;
        HAL_UART_Transmit_DMA(&huart2, buffer, len);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim7) {
        timer100Hz_elapsed = 1;
    }
}

void HAL_TIM_IC_CaptureHalfCpltCallback(TIM_HandleTypeDef *htim) {
    switch (htim->Channel) {
        case HAL_TIM_ACTIVE_CHANNEL_1:
            memcpy(&dma_buffer_timer_channel_1_safe[1], dma_buffer_timer_channel_1, DMA_TIMER_CHANNEL_HALF_LEN*4);
            dma_timer_channel_1_ready = 1;
            break;
        case HAL_TIM_ACTIVE_CHANNEL_2:
            memcpy(&dma_buffer_timer_channel_2_safe[1], dma_buffer_timer_channel_2, DMA_TIMER_CHANNEL_HALF_LEN*4);
            dma_timer_channel_2_ready = 1;
            break;
        default:
            break;
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    switch (htim->Channel) {
        case HAL_TIM_ACTIVE_CHANNEL_1:
            memcpy(&dma_buffer_timer_channel_1_safe[1], &dma_buffer_timer_channel_1[DMA_TIMER_CHANNEL_HALF_LEN], DMA_TIMER_CHANNEL_HALF_LEN*4);
            dma_timer_channel_1_ready = 1;
            break;
        case HAL_TIM_ACTIVE_CHANNEL_2:
            memcpy(&dma_buffer_timer_channel_2_safe[1], &dma_buffer_timer_channel_2[DMA_TIMER_CHANNEL_HALF_LEN], DMA_TIMER_CHANNEL_HALF_LEN*4);
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
    state->out = 170000000.0f / calc_mean(state->data, MOVING_AVERAGE_FILTER_LEN) * 60.0f / WHEEL_POLES / 60.0f * 360.0f / 3.0f/1.1125f;
}

float filter_butter(float x, float Z[]){
    float y;
    y = Z[0] + x * B[0];
    Z[0] = (Z[1] + x * B[1]) + -y * A[0];
    Z[1] = x * B[2] + -y * A[1];
    y = 170000000.0f / y * 10.0f;
    return y;
}

void control_wheels(uint32_t torq1, uint32_t torq2) {
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, torq1);
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, torq2);
    HAL_DACEx_DualStart(&hdac1);
}

void set_torque(uint32_t *torq, float speed) {
    speed = speed / UNIT_TORQUE;
    if (speed > 1.0f) {
        *torq = (uint32_t)(250.0f * speed);
        return;
    } else {
        *torq = 0;
    }
}

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
                            } else {
                                state->is_request = 0;
                            }
                            state->speed1 = speed1;
                            state->speed2 = speed2;
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        if (parse_next_byte(rx_buffer, &rx_state) > 0) {
            rx_command.speed1 = rx_state.speed1;
            rx_command.speed2 = rx_state.speed2;
            rx_command.is_request = rx_state.is_request;
            rx_buffer_ready = 1;
        }
        HAL_UART_Receive_IT(&huart2, &rx_buffer, 1);
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  float torq1 = 0.0f;
  uint32_t loop_cnt = 0;
  float target_speed1 = 0.0f;
  float target_speed2 = 0.0f;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
    HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_1, dma_buffer_timer_channel_1, DMA_TIMER_CHANNEL_LEN);
    HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_2, dma_buffer_timer_channel_2, DMA_TIMER_CHANNEL_LEN);

    HAL_TIM_Base_Start_IT(&htim7);
    set_torque(&torque1_value, 80.0f);
    set_torque(&torque2_value, 0.0f);
    control_wheels(torque1_value, torque2_value);
    HAL_Delay(100);
    HAL_UART_Receive_IT(&huart2, &rx_buffer, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      if (dma_timer_channel_1_ready == 1) {
          dma_timer_channel_1_ready = 0;
          timestamp_channel_1 = HAL_GetTick();
          filter_1_state.new_inp = calc_mean_diff(dma_buffer_timer_channel_1_safe, DMA_TIMER_CHANNEL_HALF_LEN + 1);
          //filter_1_state.out = filter_butter(filter_1_state.new_inp, filter_1_state.filter_state);
          iterate_moving_average_filter(&filter_1_state);
          dma_buffer_timer_channel_1_safe[0] = dma_buffer_timer_channel_1_safe[DMA_TIMER_CHANNEL_HALF_LEN];
      }
      if (dma_timer_channel_2_ready == 1) {
          dma_timer_channel_2_ready = 0;
          timestamp_channel_2 = HAL_GetTick();
          filter_2_state.new_inp = calc_mean_diff(dma_buffer_timer_channel_2_safe, DMA_TIMER_CHANNEL_HALF_LEN + 1);
          //filter_2_state.out = filter_butter(filter_2_state.new_inp, filter_2_state.filter_state);
          iterate_moving_average_filter(&filter_2_state);
          dma_buffer_timer_channel_2_safe[0] = dma_buffer_timer_channel_2_safe[DMA_TIMER_CHANNEL_HALF_LEN];
      }

      timestamp = HAL_GetTick();
      if ((timestamp - timestamp_channel_1)>= TIMEOUT_MS) {
          filter_1_state.out = 0.0f;
      }
      if ((timestamp - timestamp_channel_2)>= TIMEOUT_MS) {
          filter_2_state.out = 0.0f;
      }

      if (timer100Hz_elapsed == 1) {
          if (loop_cnt % 100 == 0) {
              if (fabsf(filter_1_state.out - target_speed1) > 10.0f) {
                  if (filter_1_state.out > target_speed1) {
                      torq1 -= 1.0f;
                  } else {
                      torq1 += 1.0f;
                  }
              }
          }
          if (target_speed1 < MIN_SPEED) {
              torq1 = 0.0f;
          } else {
              if (torq1 < UNIT_TORQUE) {
                  torq1 = UNIT_TORQUE;
              }
          }
          set_torque(&torque1_value, torq1);
          set_torque(&torque2_value, 0.0f);
          control_wheels(torque1_value, torque2_value);
          timer100Hz_elapsed = 0;
      }
      if (rx_buffer_ready == 1) {
          rx_buffer_ready = 0;
          if (rx_command.is_request == 1) {
              sprintf((char *)tx_buffer, "%0.7f; %0.7f\r\n",
                      filter_1_state.out,
                      filter_2_state.out);
              try_send_data(tx_buffer, strlen((char *) tx_buffer));
          } else {
              target_speed1 = rx_command.speed1;
              if (target_speed1 < MIN_SPEED) {
                  target_speed1 = 0.0f;
              }
          }
      }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
