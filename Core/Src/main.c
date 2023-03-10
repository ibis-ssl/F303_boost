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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adns3080.h"
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit_DMA(&huart1, (uint8_t *)ptr, len); // 2ms
  return len;
}*/

#define UART_TEMP_BUF_SIZE (800)
static char first_buf[UART_TEMP_BUF_SIZE];
static char second_buf[UART_TEMP_BUF_SIZE];
volatile int second_buf_len = 0, first_buf_len = 0;
volatile bool sending_second_buf = false, sending_first_buf = false;
volatile bool is_in_printf_func = false;
/*
int _write(int file, char *ptr, int len)
{
  if (huart1.hdmatx->State == HAL_DMA_BURST_STATE_BUSY)
  {
    if (len >= UART_TEMP_BUF_SIZE)
      len = UART_TEMP_BUF_SIZE;
    memcpy(temp_buf, ptr, len);
    re_queue_len = len;
    return len;
  }
  memcpy(first_buf, ptr, len);							   // 8ms
  HAL_UART_Transmit_DMA(&huart1, (uint8_t *)first_buf, len); // 2ms
  return len;
}*/

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {

  if (sending_first_buf) {     // FIRST buf complete
    sending_first_buf = false; // complete!

    if (second_buf_len > 0 && is_in_printf_func == false) { // another buffer?
      sending_second_buf = true;
      HAL_UART_Transmit_DMA(&huart1, (uint8_t *)second_buf, second_buf_len);
      second_buf_len = 0;
    }
  } else if (sending_second_buf) { // SECOND buf complete
    sending_second_buf = false;    // complete!

    if (first_buf_len > 0 && is_in_printf_func == false) { // another buffer?
      sending_first_buf = true;
      HAL_UART_Transmit_DMA(&huart1, (uint8_t *)first_buf, first_buf_len);
      first_buf_len = 0;
    }
  }
}

void p(const char *format, ...) {
  va_list ap;
  va_start(ap, format);
  is_in_printf_func = true;

  if (sending_first_buf) {
    if (second_buf_len > UART_TEMP_BUF_SIZE / 2) {
      is_in_printf_func = false;
      return;
    }
    second_buf_len += vsprintf(second_buf + second_buf_len, format, ap);
    va_end(ap);
    if (sending_first_buf == false) {
      second_buf_len = (int)strlen(second_buf);
      HAL_UART_Transmit_DMA(&huart1, (uint8_t *)second_buf, second_buf_len); // 2ms
    }
  } else if (sending_second_buf) {
    if (first_buf_len > UART_TEMP_BUF_SIZE / 2) {

      is_in_printf_func = false;
      return;
    }

    first_buf_len += vsprintf(first_buf + first_buf_len, format, ap);
    va_end(ap);

    if (sending_second_buf == false) {
      first_buf_len = (int)strlen(first_buf);
      HAL_UART_Transmit_DMA(&huart1, (uint8_t *)first_buf, first_buf_len); // 2ms
    }
  } else {
    // start !!
    first_buf_len = vsprintf(first_buf, format, ap);
    va_end(ap);
    sending_first_buf = true;
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)first_buf, first_buf_len); // 2ms
    first_buf_len = (int)strlen(first_buf);
    first_buf_len = 0;
    second_buf_len = 0;
  }
  is_in_printf_func = false;
  return;
}

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TIM_KICK_PERI (2000)
#define PWM_CNT (700)
// <750
// 750 -> 0.76~0.78s(450V)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile struct {
  bool charge_enabled;
  bool kick_chip_selected;
  float target_voltage;
  int kick_power;
  int sw_enable_cnt;
  float min_v, max_v, max_c, fet_temp, coil_temp;
} power_cmd;

typedef union {
  uint8_t data[8];

  struct {
    uint8_t idx;
    float value;
  } power;
  struct {
    uint16_t node_id;
    uint16_t type;
    uint32_t etc;
  } error_notify;

  struct {
    uint8_t cmd;
    uint8_t enable;
  } power_en;
  struct {
    uint8_t idx;
    float value;
  } set_protect_param;
  struct {
    int16_t delta_x;
    int16_t delta_y;
  } mouse;
} uint8_to_float_t;

uint8_to_float_t rx, tx;
void setTargetVoltage(float target) {
  if (target > 450) {
    target = 450;
  }
  if (target < 20) {
    target = 20;
  }
  power_cmd.target_voltage = target;
  // printf("set target voltage = %f\n",power_cmd.target_voltage);
}

volatile struct {
  uint32_t print_loop_cnt, kick_cnt;
  int boost_cnt;
  uint16_t error;
} stat;

void startKick(uint8_t power) {
  if (stat.kick_cnt == 0) {
    stat.kick_cnt = 100;
    stat.boost_cnt = 0;
    power_cmd.kick_power = TIM_KICK_PERI * power / 255;
    //p("start kick! : %d\n", power_cmd.kick_power);
  }
}

void startCharge() {
  if (stat.boost_cnt == 0 && stat.kick_cnt == 0) {
    // printf("boost start!!\n");
    stat.boost_cnt = 1000;
  }
}

uint32_t can_rx_cnt = 0;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  CAN_RxHeaderTypeDef can_rx_header;

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can_rx_header, rx.data) != HAL_OK) {
    /* Reception Error */
    Error_Handler();
  }

  can_rx_cnt++;
  switch (can_rx_header.StdId) {
  case 0x000: // emg stop
    power_cmd.charge_enabled = false;
    break;
  case 0x10:
    switch (rx.data[0]) {
    case POWER_OUT:
      if (rx.power_en.enable) {
        power_cmd.sw_enable_cnt = 100;
      }
      break;
    case MIN_VOLTAGE:
      power_cmd.min_v = rx.set_protect_param.value;
      break;
    case MAX_VOLTAGE:
      power_cmd.max_v = rx.set_protect_param.value;
      break;
    case MAX_CURRENT:
      power_cmd.max_c = rx.set_protect_param.value;
      break;
    case MAX_TEMP_FET:
      power_cmd.fet_temp = rx.set_protect_param.value;
      break;
    case MAX_TEMP_SOLENOID:
      power_cmd.coil_temp = rx.set_protect_param.value;
      break;
    default:
      break;
    }

    break;
  case 0x110: // power control
    switch (rx.power.idx) {
    case 0: // set boost voltage
      setTargetVoltage(rx.power.value);
      break;
    case 1: // charge enable
      if (rx.data[1] == 1) {
        // printf("[CAN] charge enable!\n");
        power_cmd.charge_enabled = true;
        startCharge();
      } else {
        // printf("[CAN] charge disable!\n");
        power_cmd.charge_enabled = false;
      }
      break;
    case 2: // kicker select
      if (rx.data[1] == 1) {
        // printf("[CAN] select chip kicker\n");
        power_cmd.kick_chip_selected = true;
      } else {
        // printf("[CAN] select straight kicker \n");
        power_cmd.kick_chip_selected = false;
      }
      break;
    case 3: // kick !
      //p("[CAN] kick!\n");
      startKick(255);
      break;
    default:
      break;
    }
    break;
  default:
    break;
  }
}

CAN_TxHeaderTypeDef can_header;
uint8_t can_data[8];
uint32_t can_mailbox;

void sendCan(void) {
  can_header.StdId = 0x200;
  can_header.RTR = CAN_RTR_DATA;
  can_header.DLC = 8;
  can_header.TransmitGlobalTime = DISABLE;
  can_data[0] = 0;
  can_data[1] = 0;
  can_data[2] = 1;
  can_data[3] = 1;
  HAL_CAN_AddTxMessage(&hcan, &can_header, can_data, &can_mailbox);
}

void sendCanMouse(int16_t delta_x, int16_t delta_y) {
  can_header.StdId = 0x240;
  can_header.RTR = CAN_RTR_DATA;
  can_header.DLC = 4;
  can_header.TransmitGlobalTime = DISABLE;
  tx.mouse.delta_x = delta_x;
  tx.mouse.delta_y = delta_y;
  HAL_CAN_AddTxMessage(&hcan, &can_header, tx.data, &can_mailbox);
}

struct {
  float boost_v, batt_v, gd_16p, gd_16m, batt_cs;
  float temp_coil_1, temp_coil_2, temp_fet;
} sensor;
struct {
  float batt_v_min, batt_v_max;
  float gd_16p_min, gd_16m_min;
  float batt_cs_max;
} peak;

void updateADCs(void) {
  sensor.batt_v = (float)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1) * 3.3 / 4096 * 11 / 1;
  sensor.gd_16p = (float)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2) * 3.3 / 4096 * 11 / 1;
  sensor.gd_16m = (((float)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3) * 3.3 / 4096) * 21 - sensor.gd_16p * 11) / 10;
  sensor.boost_v = (float)HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_3) * 213 * 3.3 / 4096 * 1.038; // 1.038 is calib
  // INA199x1 : 50 V/V
  //  2m ohm x 50VV -> 100m V / A
  // 33A-max (v3 board)
  sensor.batt_cs = ((float)HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_1) * 3.3 / 4096) * 10;
  sensor.temp_fet = (-((float)HAL_ADCEx_InjectedGetValue(&hadc4, ADC_INJECTED_RANK_1) * 3.3 / 4096) + 1.5) * 70 + 25;
  sensor.temp_coil_1 = (-((float)HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_2) * 3.3 / 4096) + 1.5) * 70 + 25;
  sensor.temp_coil_2 = (-((float)HAL_ADCEx_InjectedGetValue(&hadc4, ADC_INJECTED_RANK_2) * 3.3 / 4096) + 1.5) * 70 + 25;
  // 25 deg = 10k/10k+10k = 0.5*3.3 = 1.85V
  // 75 deg = 1.7k/1.7k+10k = 0.0145*3.3 = 0.48V
  // 1.37V / 50 deg
  // 36.5 deg / V

  // real : normal -> 1.4V
  // 80~100deg -> 0.7V
  // 0.7V / 50 deg ->

  if (sensor.batt_v < peak.batt_v_min) {
    peak.batt_v_min = sensor.batt_v;
  }
  if (sensor.batt_v > peak.batt_v_max) {
    peak.batt_v_max = sensor.batt_v;
  }
  if (sensor.batt_cs > peak.batt_cs_max) {
    peak.batt_cs_max = sensor.batt_cs;
  }
  if (sensor.gd_16p < peak.gd_16p_min) {
    peak.gd_16p_min = sensor.gd_16p;
  }
  if (sensor.gd_16m > peak.gd_16m_min) {
    peak.gd_16m_min = sensor.gd_16m;
  }
}

void protecter(void) {
  static uint16_t pre_sys_error = NONE;
  if (sensor.batt_v < 20) {
    p("\n\n[ERR] UNDER_VOLTAGE\n\n");
    stat.error |= UNDER_VOLTAGE;
  }
  if (sensor.batt_v > 35) {
    p("\n\n[ERR] OVER_VOLTAGE\n\n");
    stat.error |= OVER_VOLTAGE;
  }
  if (sensor.batt_cs > 30) {
    p("\n\n[ERR] SHORT_CURCUIT\n\n");
    stat.error |= SHORT_CURCUIT;
  }
  if (stat.boost_cnt > 10) {
    if (sensor.batt_cs > 25) {
      p("\n\n[ERR] OVER_CURRENT\n\n");
      stat.error |= OVER_CURRENT;
    }
  } else {
    if (sensor.batt_cs > 25) {
      p("\n\n[ERR] OVER_CURRENT\n\n");
      stat.error |= OVER_CURRENT;
    }
  }
  if (sensor.gd_16p < 10 || sensor.gd_16m > -5) {
    p("\n\n[ERR] GD_POWER_FAIL\n\n");
    stat.error |= GD_POWER_FAIL;
  }

  if (sensor.boost_v > 460) {
    p("\n\n[ERR] NO_CAP\n\n");
    stat.error |= NO_CAP;
  }

  if (stat.error && stat.error != pre_sys_error) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);

    p("[ERR] power line error!!! / battv %6.2f battcs %6.3f / GDp %+5.2f GDm %+5.2f boost %6.2f\n", sensor.batt_v, sensor.batt_cs, sensor.gd_16p,
      sensor.gd_16m, sensor.boost_v);
    if (stat.error == UNDER_VOLTAGE) {
      // discharge!!
      p("DISCHARGE!!!\n");
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, TIM_KICK_PERI);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, TIM_KICK_PERI);
      HAL_Delay(1000);
    } else {
      stat.kick_cnt = 0;
      stat.boost_cnt = 0;
    }
  }
  pre_sys_error = stat.error;
}

void boostControl(void) {
  static int temp_pwm_autoreload = 1000, pre_pwm_autoreload = 0;

  if (sensor.boost_v < power_cmd.target_voltage && stat.boost_cnt > 0) {
    stat.boost_cnt--;
    if (sensor.boost_v < 50) {
      temp_pwm_autoreload = PWM_CNT * 10;
    } else if (sensor.boost_v < 100) {
      temp_pwm_autoreload = PWM_CNT * 3;
    } else if (sensor.boost_v < 200) {
      temp_pwm_autoreload = PWM_CNT * 1.5;
    } else if (sensor.boost_v < 300) {
      temp_pwm_autoreload = PWM_CNT * 1.4;
    } else if (sensor.boost_v < 400) {
      temp_pwm_autoreload = PWM_CNT * 1.3;
    } else {
      temp_pwm_autoreload = PWM_CNT * 1.25;
    }
    if (pre_pwm_autoreload != temp_pwm_autoreload) {
      htim2.Instance->CNT = 0;
      __HAL_TIM_SET_AUTORELOAD(&htim2, temp_pwm_autoreload);
    }
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, PWM_CNT);
    pre_pwm_autoreload = temp_pwm_autoreload;

    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
    if (stat.boost_cnt == 0) {
      p("[ERR] boost timeout!!\n");
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
    }
  } else {
    if (stat.boost_cnt != 0) {
      // printf("boost end!!\n\n !! %d cycle !!\n\n", stat.boost_cnt);
      stat.boost_cnt = 0;
    }
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
  }
}

void kickControl(void) {
  if (stat.kick_cnt > 0) {
    // kick!!!
    if (power_cmd.kick_chip_selected) {
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, power_cmd.kick_power); // chip
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    } else {
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, power_cmd.kick_power); // straight
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    }
    stat.kick_cnt--;
    if (stat.kick_cnt == 0) {
      //p("kick end!!\n");
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
      if (power_cmd.charge_enabled) {
        //p("continue charge!!\n");
        startCharge();
      }
    }
  } else {
    // idol
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
  }
}

void userInterface(void) {

  // User SW control
  if (HAL_GPIO_ReadPin(SW_1_GPIO_Port, SW_1_Pin) == GPIO_PIN_RESET) {
    p("[USR] boost start!!\n");
    startKick(255);
  }
  if (HAL_GPIO_ReadPin(SW_2_GPIO_Port, SW_2_Pin) == GPIO_PIN_RESET) {
    p("[USR] boost start!!\n");
    startCharge();
  }

  stat.print_loop_cnt++;
  // debug print
  if (stat.print_loop_cnt > 100) {
    // printf("%8ld %8ld %8ld /
    // ",HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_1),HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_2),HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_3));
    // printf("%8ld %8ld %8ld /
    // ",HAL_ADCEx_InjectedGetValue(&hadc3,ADC_INJECTED_RANK_1),HAL_ADCEx_InjectedGetValue(&hadc3,ADC_INJECTED_RANK_2),HAL_ADCEx_InjectedGetValue(&hadc3,ADC_INJECTED_RANK_3));
    // printf("%8ld
    // %8ld\n",HAL_ADCEx_InjectedGetValue(&hadc4,ADC_INJECTED_RANK_1),HAL_ADCEx_InjectedGetValue(&hadc4,ADC_INJECTED_RANK_2));
    // HAL_ADCEx_InjectedStart(&hadc1);

    // p("pwm = %d : ",temp_pwm_autoreload);
    if (stat.error) {
      p("E:0x%04x ", stat.error);
    } else {
      p("         ");
    }
    // p("Vm %3.1f VM %3.1f CM %3.1f DF %3.1f DC %3.1f
    // ",power_cmd.min_v,power_cmd.max_v,power_cmd.max_c,power_cmd.fet_temp,power_cmd.coil_temp);
    // p("PW %3d BV %3.0f, CK %d, CH %d /", power_cmd.sw_enable_cnt, power_cmd.target_voltage, power_cmd.kick_chip_selected,
    // power_cmd.charge_enabled);
    p("BattVm %3.1f VM %3.1f GD+ %+4.1f GD- %+4.1f BattCS %+5.1f ", peak.batt_v_max, peak.batt_v_min, peak.gd_16p_min, peak.gd_16m_min,
      peak.batt_cs_max);
    // p("%+3d %+3d %4d / ", get_DeltaX_ADNS3080(), get_DeltaY_ADNS3080(), get_Qualty_ADNS3080());
    p("BattV %3.1f, BoostV %5.1f, BattCS %+5.1f fet %3.1f coil1 %3.1f coil2 %3.1f\n", sensor.batt_v, sensor.boost_v, sensor.batt_cs, sensor.temp_fet,
      sensor.temp_coil_1, sensor.temp_coil_2);
    // printf("adc1 : ch1 %8ld / ch2 %8ld / ch3 %8ld / adc3: ch1 %8ld / ch5 %8ld / ch12 %8ld /
    // adc4 : ch3 %8ld / ch4 %8ld \n", adc1_raw_data[0], adc1_raw_data[1], adc1_raw_data[2],
    // adc3_raw_data[0],adc3_raw_data[1],adc3_raw_data[2],adc4_raw_data[0],adc4_raw_data[1]);
    stat.print_loop_cnt = 0;

    if (power_cmd.sw_enable_cnt > 0) {
      power_cmd.sw_enable_cnt -= 10;
    }
    if (power_cmd.sw_enable_cnt == 0 && stat.error) {
      p("!! clear Error : %d !!\n", stat.error);
      stat.error = 0;
    }
  }
  // charge-indication
  if (sensor.boost_v > 100) {
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
  }
}

bool connectionTest(void) {
  while (1) {
    updateADCs();
    HAL_Delay(100);
    p("Pre-test : BattV %3.1f, GD+ %+4.1f GD- %+4.1f,BoostV %5.1f, BattCS %+5.1f fet %3.1f coil1 %3.1f coil2 %3.1f\n", sensor.batt_v, sensor.gd_16p,
      sensor.gd_16m, sensor.boost_v, sensor.batt_cs, sensor.temp_fet, sensor.temp_coil_1, sensor.temp_coil_2);
    if (sensor.batt_v > 20 && sensor.gd_16p > 11 && sensor.gd_16m < 8 && sensor.batt_cs < 0.1 && sensor.temp_fet < 50 && sensor.temp_coil_1 < 70 &&
        sensor.temp_coil_2 < 70) {

      p("Pre-test OK!!\n");
      break;
    }
  }
  HAL_GPIO_WritePin(POWER_SW_EN_GPIO_Port, POWER_SW_EN_Pin, GPIO_PIN_SET);
  int timeout_cnt = 0;
  while (1) {
    timeout_cnt++;
    updateADCs();
    if (sensor.batt_v > 20 && sensor.gd_16p > 11 && sensor.gd_16m < 8 && sensor.batt_cs < 0.1 && sensor.temp_fet < 50 && sensor.temp_coil_1 < 70 &&
        sensor.temp_coil_2 < 70) {
      p("PowerOn-test   OK!! cnt %3d : ", timeout_cnt);
      p("BattV %3.1f, GD+ %+4.1f GD- %+4.1f,BoostV %5.1f, BattCS %+5.1f fet %3.1f coil1 %3.1f coil2 %3.1f\n", sensor.batt_v, sensor.gd_16p,
        sensor.gd_16m, sensor.boost_v, sensor.batt_cs, sensor.temp_fet, sensor.temp_coil_1, sensor.temp_coil_2);
      break;
    }
    if (timeout_cnt > 10) {
      HAL_GPIO_WritePin(POWER_SW_EN_GPIO_Port, POWER_SW_EN_Pin, GPIO_PIN_RESET);
      p("PowerOn-test FAIL!! : ");
      p("BattV %3.1f, GD+ %+4.1f GD- %+4.1f,BoostV %5.1f, BattCS %+5.1f fet %3.1f coil1 %3.1f coil2 %3.1f\n", sensor.batt_v, sensor.gd_16p,
        sensor.gd_16m, sensor.boost_v, sensor.batt_cs, sensor.temp_fet, sensor.temp_coil_1, sensor.temp_coil_2);
      while (1)
        ;
    }
  }

  timeout_cnt = 0;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, TIM_KICK_PERI / 10);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, TIM_KICK_PERI / 10);

  while (1) {
    timeout_cnt++;
    updateADCs();
    HAL_Delay(1);
    if (sensor.batt_v > 20 && sensor.gd_16p > 11 && sensor.gd_16m < 8 && sensor.batt_cs < 0.1 && sensor.temp_fet < 50 && sensor.temp_coil_1 < 70 &&
        sensor.temp_coil_2 < 70 && sensor.boost_v < 20) {
      p("DisCharge-test OK!! cnt %3d : ", timeout_cnt);
      p("BattV %3.1f, GD+ %+4.1f GD- %+4.1f,BoostV %5.1f, BattCS %+5.1f fet %3.1f coil1 %3.1f coil2 %3.1f\n", sensor.batt_v, sensor.gd_16p,
        sensor.gd_16m, sensor.boost_v, sensor.batt_cs, sensor.temp_fet, sensor.temp_coil_1, sensor.temp_coil_2);
      break;
    }
    if (timeout_cnt > 1000) {
      // capacitor is charged & dis charge circuit not works
      p("DisCharge-test FAIL!! : ");
      p("BattV %3.1f, GD+ %+4.1f GD- %+4.1f,BoostV %5.1f, BattCS %+5.1f fet %3.1f coil1 %3.1f coil2 %3.1f\n", sensor.batt_v, sensor.gd_16p,
        sensor.gd_16m, sensor.boost_v, sensor.batt_cs, sensor.temp_fet, sensor.temp_coil_1, sensor.temp_coil_2);
      while (1)
        ;
    }
  }
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);

  timeout_cnt = 0;
  __HAL_TIM_SET_AUTORELOAD(&htim2, 72000);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 100);

  while (1) {

    timeout_cnt++;
    updateADCs();
    if (sensor.boost_v > 30 || sensor.batt_cs > 1.0) {
      HAL_GPIO_WritePin(POWER_SW_EN_GPIO_Port, POWER_SW_EN_Pin, GPIO_PIN_RESET);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
      p("Capacitor-test FAIL!! %d : ", timeout_cnt);
      p("BattV %3.1f, GD+ %+4.1f GD- %+4.1f,BoostV %5.1f, BattCS %+5.1f fet %3.1f coil1 %3.1f coil2 %3.1f\n", sensor.batt_v, sensor.gd_16p,
        sensor.gd_16m, sensor.boost_v, sensor.batt_cs, sensor.temp_fet, sensor.temp_coil_1, sensor.temp_coil_2);
      while (1)
        ;
    }
    if (timeout_cnt > 100) {
      p("Capacitor-test OK!! cnt %3d : ", timeout_cnt);
      p("BattV %3.1f, GD+ %+4.1f GD- %+4.1f,BoostV %5.1f, BattCS %+5.1f fet %3.1f coil1 %3.1f coil2 %3.1f\n", sensor.batt_v, sensor.gd_16p,
        sensor.gd_16m, sensor.boost_v, sensor.batt_cs, sensor.temp_fet, sensor.temp_coil_1, sensor.temp_coil_2);
      break;
    }
    HAL_Delay(1);
  }
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */

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
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_ADC4_Init();
  MX_CAN_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  p("\n\nstart ORION BOOST v3\n\n");

  // kick
  HAL_TIM_PWM_Init(&htim3);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_AUTORELOAD(&htim3, TIM_KICK_PERI);
  __HAL_TIM_SET_AUTORELOAD(&htim3, TIM_KICK_PERI);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  // boost
  HAL_TIM_PWM_Init(&htim2);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
  __HAL_TIM_SET_AUTORELOAD(&htim2, 1000);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  // GD negative PS
  HAL_TIM_PWM_Init(&htim4);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 1000);
  __HAL_TIM_SET_AUTORELOAD(&htim4, 2000);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_Delay(100);
  // wait charging

  // can init
  CAN_Filter_Init();
  HAL_CAN_Start(&hcan);

  setbuf(stdout, NULL);

  HAL_UART_Init(&huart1);

  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc3);
  HAL_ADC_Start(&hadc4);

  HAL_GPIO_WritePin(LED_CURRENT_GPIO_Port, LED_CURRENT_Pin, GPIO_PIN_RESET);

  /*while(1){

      HAL_Delay(100);
      sendCan();
      printf("can rx : %d\n",can_rx_cnt);
      can_rx_cnt = 0;
  }*/
  if (is_connect_ADNS3080()) {
    p("ADNS3080 OK!\n");
  } else {
    p("ADNS3080 not found...\n");
    while (1) {
      /* code */
    }
  }

  init_ADNS3080(true);

  if (HAL_GPIO_ReadPin(SW_1_GPIO_Port, SW_1_Pin) == GPIO_PIN_RESET) {
    while (true) {
      // frame_print_ADNS3080();
      HAL_Delay(1);

      update_ADNS3080();
      p("\n\n%+3d %+3d %4d\n\n", get_DeltaX_ADNS3080(), get_DeltaY_ADNS3080(), get_Qualty_ADNS3080());
      HAL_Delay(100);
    }
  }
  if (HAL_GPIO_ReadPin(SW_2_GPIO_Port, SW_2_Pin) == GPIO_PIN_RESET) {
    while (true) {
      frame_print_ADNS3080();
      HAL_Delay(1);

      // update_ADNS3080();
      // p("\n\n%+3d %+3d %4d\n\n", get_DeltaX_ADNS3080(), get_DeltaY_ADNS3080(),
      // get_Qualty_ADNS3080()); HAL_Delay(100);
    }
  }

  setTargetVoltage(450);
  peak.batt_v_min = 30;
  peak.gd_16m_min = -10;
  peak.gd_16p_min = 20;

  connectionTest();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    update_ADNS3080();
    sendCanMouse(get_DeltaX_ADNS3080(), get_DeltaY_ADNS3080());

    HAL_Delay(1);

    updateADCs();
    protecter();
    userInterface();

    if (stat.error) {
      continue;
    }

    kickControl();
    boostControl();
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_ADC12 | RCC_PERIPHCLK_ADC34;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
