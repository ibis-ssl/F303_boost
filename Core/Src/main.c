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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TIM_KICK_PERI (2000)
#define PWM_CNT (700)
// 750だとちょっとオーバーヒート早いかも
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


void setTargetVoltage(float target) {
  if (target > 450) {
    return;
    target = 450;
  }
  if (target < 20) {
    return;
    target = 20;
  }
  power_cmd.target_voltage = target;
}

volatile struct {
  uint32_t print_loop_cnt, kick_cnt;
  int boost_cnt;
  bool power_enabled;
  uint16_t error;
  uint32_t system_loop_cnt;
} stat;

void startKick(uint8_t power) {
  if (stat.kick_cnt == 0) {
    stat.kick_cnt = 100;
    stat.boost_cnt = 0;
    power_cmd.kick_power = TIM_KICK_PERI * power / 255;
    // p("start kick! : %d\n", power_cmd.kick_power);
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
  can_msg_buf_t rx;

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
        power_cmd.sw_enable_cnt = 200;
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
      // p("[CAN] kick!\n");
      startKick(rx.data[1]);
      break;
    default:
      break;
    }
    break;
  default:
    break;
  }
}

uint8_t uart1_rx_it_buffer;
uint32_t uart1_rx_cnt = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
  if (huart->Instance == USART1) {
    uart1_rx_cnt++;
    HAL_UART_Receive_IT(&huart1, &uart1_rx_it_buffer, 1);
  }
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
  sensor.boost_v = (float)HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_3) * 213 * 3.3 / 4096;// * 1.038; // 1.038 is calib(v3),
  // INA199x1 : 50 V/V
  //  2m ohm x 50VV -> 100m V / A
  // 33A-max (v3 board)

  // ZXCT1085 : 25V/V
  //  2m ohm x 25VV -> 50m V / A
  // 66A-max (v4 board)
  sensor.batt_cs = ((float)HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_1) * 3.3 / 4096) * 20 - 2;	// 2A offset is manual offfset (~0.14V~)
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

#define FET_TEST_TEMP (80)
#define COIL_OVER_HEAT_TEMP (80)

void protecter(void) {
  static uint16_t pre_sys_error = NONE;
  if (sensor.batt_v < 20 && stat.power_enabled) {
    stat.error |= UNDER_VOLTAGE;
    if (pre_sys_error != stat.error) {
      p("\n\n[ERR] UNDER_VOLTAGE\n\n");
    }
  }
  if (sensor.batt_v > 35) {
    stat.error |= OVER_VOLTAGE;
    if (pre_sys_error != stat.error) {
      p("\n\n[ERR] OVER_VOLTAGE\n\n");
    }
  }
  if (sensor.batt_cs > 30) {
    stat.error |= SHORT_CURCUIT;
    if (pre_sys_error != stat.error) {
      p("\n\n[ERR] SHORT_CURCUIT\n\n");
    }
  }
  if (stat.boost_cnt > 10) {
    if (sensor.batt_cs > 25) {
      stat.error |= OVER_CURRENT;
      if (pre_sys_error != stat.error) {
        p("\n\n[ERR] OVER_CURRENT cnt %d\n\n",stat.boost_cnt);
      }
    }
  } else {
    if (sensor.batt_cs > 12) {
      stat.error |= OVER_CURRENT;
      if (pre_sys_error != stat.error) {
        p("\n\n[ERR] OVER_CURRENT %d\n\n",stat.boost_cnt);
      }
    }
  }
  if (sensor.gd_16p < 10 || sensor.gd_16m > -5) {
    stat.error |= GD_POWER_FAIL;
    if (pre_sys_error != stat.error) {
      p("\n\n[ERR] GD_POWER_FAIL\n\n");
    }
  }

  if (sensor.boost_v > 460) {
    stat.error |= NO_CAP;
    if (pre_sys_error != stat.error) {
      p("\n\n[ERR] NO_CAP\n\n");
    }
  }

  if (sensor.temp_coil_1 > COIL_OVER_HEAT_TEMP || sensor.temp_coil_2 > COIL_OVER_HEAT_TEMP) {
    stat.error |= COIL_OVER_HEAT;
    if (pre_sys_error != stat.error) {
      p("\n\n[ERR] COIL_OVER_HEAT\n\n");
    }
  }

  if (sensor.temp_fet > FET_TEST_TEMP) {
    stat.error |= FET_OVER_HEAT;
    if (pre_sys_error != stat.error) {
      p("\n\n[ERR] FET_OVER_HEAT\n\n");
    }
  }

  if (stat.error && stat.error != pre_sys_error) {
    powerOutputDisable(); // output disable
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
    sendCanError(stat.error, 0);

    p("[ERR] power line error!!! / battv %6.2f battcs %6.3f / GDp %+5.2f GDm %+5.2f boost %6.2f\n", sensor.batt_v, sensor.batt_cs, sensor.gd_16p,
      sensor.gd_16m, sensor.boost_v);
    if (stat.error == UNDER_VOLTAGE) {
      // discharge!!
      p("DISCHARGE!!!\n");
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, TIM_KICK_PERI/20);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, TIM_KICK_PERI/20);
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

    setChargingLedHigh();
    if (stat.boost_cnt == 0) {
      p("[ERR] boost timeout!!\n");
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
    }
  } else {
    if (stat.boost_cnt != 0) {
    	if(stat.boost_cnt < 900){
    	      p("boost end!! / %4.2f V / %3d\n",sensor.boost_v,1000 - stat.boost_cnt);
    	}
      stat.boost_cnt = 0;
    }
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
    setChargingLedLow();
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
      // p("kick end!!\n");
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
      if (power_cmd.charge_enabled) {
        // p("continue charge!!\n");
        startCharge();
      }
    }
  } else {
    // idol
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
  }
}

void userInterface(void)
{
  static bool pre_sw_pushed[2], user_control_chip_select = false;
  static uint32_t print_idx = 0;

  // User SW control
  if (isPushedUserSw1()) {
    if (!pre_sw_pushed[0]) {
      pre_sw_pushed[0] = true;
      p("[USR] kick start!! : chip %d\n", user_control_chip_select);
      startKick(255);
      power_cmd.sw_enable_cnt = 1000;
      power_cmd.kick_chip_selected = user_control_chip_select;
      user_control_chip_select = !user_control_chip_select;
    }
  } else {
    pre_sw_pushed[0] = false;
  }
  if (isPushedUserSw2()) {
    if (!pre_sw_pushed[1]) {
      pre_sw_pushed[1] = true;
      p("[USR] boost start!!\n");
      startCharge();
      power_cmd.sw_enable_cnt = 1000;
    }
  } else {
    pre_sw_pushed[1] = false;
  }

  stat.print_loop_cnt++;
  // debug print
  if (stat.print_loop_cnt >= 50) {
    stat.print_loop_cnt = 0;

    if (uart1_rx_cnt) {
      uart1_rx_cnt = 0;
      print_idx++;
    }

    if (stat.error) {
      p("E:0x%04x ", stat.error);
    } else {
      p("         ");
    }
    switch (print_idx) {
      case 0:
        p("PW %3d BV %3.0f, CK %d, Ch %d / TargetV %3.0f,", power_cmd.sw_enable_cnt, power_cmd.target_voltage, power_cmd.kick_chip_selected, power_cmd.charge_enabled, power_cmd.target_voltage);

        p("BattV %3.1f, BoostV %3.0f, BattCS %+5.1f fet %2.0f coil1 %2.0f coil2 %2.0f ", sensor.batt_v, sensor.boost_v, sensor.batt_cs, sensor.temp_fet, sensor.temp_coil_1, sensor.temp_coil_2);
        p("XX %+8.2f YY %+8.2f %6d", get_XX_ADNS3080(), get_YY_ADNS3080(), get_ShutterSpeed_ADNS3080());
        p("loop %4d D x%+3d y%+3d I x%+6d y%+6d Q%3d\n", stat.system_loop_cnt, get_DeltaX_ADNS3080(), get_DeltaY_ADNS3080(), get_X_ADNS3080(), get_Y_ADNS3080(), get_Qualty_ADNS3080());
        break;
      
      case 1:
        //p("Vm %3.1f VM %3.1f CM %3.1f DF %3.1f DC %3.1f", power_cmd.min_v, power_cmd.max_v, power_cmd.max_c, power_cmd.fet_temp, power_cmd.coil_temp);
        p("BattV %3.1f GD-P %+4.1f GD-N %+4.1f BoostV %3.0f BattCS %+5.1f fet %2.0f coil1 %2.0f coil2 %2.0f / ", sensor.batt_v, sensor.gd_16p, sensor.gd_16m, sensor.boost_v, sensor.batt_cs,
          sensor.temp_fet, sensor.temp_coil_1, sensor.temp_coil_2);
        p("PEAK BattV-max %3.1f BattV-min %3.1f GD+ min %+4.1f GD- min %+4.1f BattCS %+5.1f \n", peak.batt_v_max, peak.batt_v_min, peak.gd_16p_min, peak.gd_16m_min, peak.batt_cs_max);
        break;

      default:
        print_idx = 0;
        break;
    }

    if (!stat.power_enabled && stat.error) {
      p("!! clear Error : %d !!\n", stat.error);
      stat.error = 0;
    }
  }
  // charge-indication
  if (sensor.boost_v > 100) {
    setHVWarningLedHigh();
  } else {
    setHVWarningLedLow();
  }
}

void canDataSender()
{
  static uint8_t data_idx = 0;
  data_idx++;
  switch (data_idx) {
    case 1:
      sendCanTemp((uint8_t)sensor.temp_fet, (uint8_t)sensor.temp_coil_1, (uint8_t)sensor.temp_coil_2);
      break;
    case 2:
      sendCanBatteryVoltage(sensor.batt_v);
      break;
    case 3:
      sendCanKickerVoltage(sensor.boost_v);
      break;
    case 4:
      sendCanBatteryCurrent(sensor.batt_cs);
      break;

    default:
      data_idx = 0;
      break;
  }
}

void connectionTest(void) {
  while (1) {
    updateADCs();
    HAL_Delay(100);
    p("Pre-test : BattV %3.1f, GD+ %+4.1f GD- %+4.1f,BoostV %5.1f, BattCS %+5.1f fet %3.1f coil1 %3.1f coil2 %3.1f\n", sensor.batt_v, sensor.gd_16p,
      sensor.gd_16m, sensor.boost_v, sensor.batt_cs, sensor.temp_fet, sensor.temp_coil_1, sensor.temp_coil_2);
    if (sensor.batt_v > 20 && sensor.gd_16p > 11 && sensor.gd_16m < -8 && sensor.batt_cs < 0.5 && sensor.temp_fet < FET_TEST_TEMP && sensor.temp_coil_1 < COIL_OVER_HEAT_TEMP &&
        sensor.temp_coil_2 < COIL_OVER_HEAT_TEMP) {

      p("Pre-test OK!!\n");
      break;
    }
  }
  powerOutputEnable();

  setChargingLedLow();
  setCanEnCmdLedHigh();
  setHVWarningLedLow();
  setOutSwLedHigh();
  setErrorLedLow();

  int timeout_cnt = 0;
  while (1) {
    timeout_cnt++;
    updateADCs();
    if (sensor.batt_v > 20 && sensor.gd_16p > 11 && sensor.gd_16m < -8 && sensor.batt_cs < 0.5 && sensor.temp_fet < FET_TEST_TEMP && sensor.temp_coil_1 < COIL_OVER_HEAT_TEMP &&
        sensor.temp_coil_2 < COIL_OVER_HEAT_TEMP) {
      p("PowerOn-test   OK!! cnt %3d : ", timeout_cnt);
      p("BattV %3.1f, GD+ %+4.1f GD- %+4.1f,BoostV %5.1f, BattCS %+5.1f fet %3.1f coil1 %3.1f coil2 %3.1f\n", sensor.batt_v, sensor.gd_16p,
        sensor.gd_16m, sensor.boost_v, sensor.batt_cs, sensor.temp_fet, sensor.temp_coil_1, sensor.temp_coil_2);
      break;
    }
    if (timeout_cnt > 10) {
      powerOutputDisable();
      p("PowerOn-test FAIL!! : ");
      p("BattV %3.1f, GD+ %+4.1f GD- %+4.1f,BoostV %5.1f, BattCS %+5.1f fet %3.1f coil1 %3.1f coil2 %3.1f\n", sensor.batt_v, sensor.gd_16p,
        sensor.gd_16m, sensor.boost_v, sensor.batt_cs, sensor.temp_fet, sensor.temp_coil_1, sensor.temp_coil_2);
      setErrorLedHigh();
      while (1)
        ;
    }
  }

  setChargingLedLow();
  setCanEnCmdLedHigh();
  setHVWarningLedHigh();
  setOutSwLedHigh();
  setErrorLedLow();

  timeout_cnt = 0;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, TIM_KICK_PERI / 10);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, TIM_KICK_PERI / 10);

  while (1) {
    timeout_cnt++;
    updateADCs();
    HAL_Delay(1);
    if (sensor.batt_v > 20 && sensor.gd_16p > 11 && sensor.gd_16m < -8 && sensor.batt_cs < 0.5 && sensor.temp_fet < FET_TEST_TEMP && sensor.temp_coil_1 < COIL_OVER_HEAT_TEMP &&
        sensor.temp_coil_2 < COIL_OVER_HEAT_TEMP && sensor.boost_v < 20) {
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
      setErrorLedHigh();
      while (1)
        ;
    }
  }

  setChargingLedHigh();
  setCanEnCmdLedLow();
  setHVWarningLedHigh();
  setOutSwLedHigh();
  setErrorLedLow();

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);

  timeout_cnt = 0;
  __HAL_TIM_SET_AUTORELOAD(&htim2, 72000);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 100);

  while (1) {

    timeout_cnt++;
    updateADCs();
    if (sensor.boost_v > 30 || sensor.batt_cs > 1.0) {
      powerOutputDisable();
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
      p("Capacitor-test FAIL!! %d : ", timeout_cnt);
      p("BattV %3.1f, GD+ %+4.1f GD- %+4.1f,BoostV %5.1f, BattCS %+5.1f fet %3.1f coil1 %3.1f coil2 %3.1f\n", sensor.batt_v, sensor.gd_16p,
        sensor.gd_16m, sensor.boost_v, sensor.batt_cs, sensor.temp_fet, sensor.temp_coil_1, sensor.temp_coil_2);
      setErrorLedHigh();
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
int main(void)
{
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  powerOutputDisable();

  mouseLedEnable();
  HAL_Delay(100);
  mouseLedDisable();

  setChargingLedHigh();
  setCanEnCmdLedHigh();
  setHVWarningLedHigh();
  setOutSwLedHigh();
  setErrorLedHigh();

  p("\n\nstart ORION BOOST v4\n\n");

  HAL_TIM_Base_Start(&htim1);

  // 1000ms -> 7400cnt
  // 7.4cnt per 1ms

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

  if (is_connect_ADNS3080()) {
    p("ADNS3080 OK!\n");
  }
  else {
    p("ADNS3080 not found...\n");
    while (1) {
      /* code */
    }
  }

  init_ADNS3080(true);

  if (isPushedUserSw1()) {
    mouseLedEnable();

    while (true) {
      // frame_print_ADNS3080();
      HAL_Delay(1);

      update_ADNS3080();
      p("Xv, %+3d, Yv, %+3d, QL, %4d, PosX, %5.3f, PosY, %5.3f\n", get_DeltaX_ADNS3080(), get_DeltaY_ADNS3080(), get_Qualty_ADNS3080(),(float)get_X_ADNS3080() / 1000 ,(float)get_Y_ADNS3080()/1000 );
      HAL_Delay(10);
    }
  }
  if (isPushedUserSw2()) {

    mouseLedEnable();
    while (true) {
      frame_print_ADNS3080();
      HAL_Delay(100);

      // update_ADNS3080();
      // p("\n\n%+3d %+3d %4d\n\n", get_DeltaX_ADNS3080(), get_DeltaY_ADNS3080(),
      // get_Qualty_ADNS3080()); HAL_Delay(100);
    }
  }

  setTargetVoltage(150);
  peak.batt_v_min = 30;
  peak.gd_16m_min = -10;
  peak.gd_16p_min = 20;

  connectionTest();

  HAL_UART_Receive_IT(&huart1, &uart1_rx_it_buffer, 1);

  uint8_t mouse_read_cnt = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    mouse_read_cnt++;
    if (mouse_read_cnt > 10){
      mouse_read_cnt = 0;

      update_ADNS3080();
      sendCanMouse(get_DeltaX_ADNS3080(), get_DeltaY_ADNS3080(), get_Qualty_ADNS3080());
    }

    // wait 2ms cycle
    stat.system_loop_cnt = htim1.Instance->CNT;
    while (htim1.Instance->CNT < 2000) {
    }
    htim1.Instance->CNT = htim1.Instance->CNT - 1000;

    updateADCs();
    protecter();
    userInterface();
    canDataSender();

    // power SW control (timeout)
    if (power_cmd.sw_enable_cnt > 0) {
      power_cmd.sw_enable_cnt -= 1;
      setCanEnCmdLedHigh();
      stat.power_enabled = true;
    } else {
      setCanEnCmdLedLow();
      stat.power_enabled = false;
    }

    // stop control
    if (stat.error || !stat.power_enabled) {
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
      powerOutputDisable();
      setErrorLedHigh();
      setOutSwLedLow();
      mouseLedDisable();
      stat.boost_cnt = 0;
      stat.kick_cnt = 0;
      continue;
    } else {
        mouseLedEnable();
      setOutSwLedHigh();
      setErrorLedLow();
      powerOutputEnable();
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
void SystemClock_Config(void)
{
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_ADC12|RCC_PERIPHCLK_ADC34;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  while (1) {
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
