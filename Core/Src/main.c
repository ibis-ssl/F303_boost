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
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "adns3080.h"
#include <stdarg.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit_DMA(&huart1, (uint8_t *)ptr, len); // 2ms
  return len;
}*/

#define UART_TEMP_BUF_SIZE (200)
char first_buf[UART_TEMP_BUF_SIZE];
char temp_buf[UART_TEMP_BUF_SIZE];
int re_queue_len = 0;

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
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (re_queue_len)
	{

		HAL_UART_Transmit_DMA(&huart1, (uint8_t *)temp_buf, re_queue_len);
		re_queue_len = 0;
	}
}

void p(const char *format, ...)
{

	va_list ap;
	va_start(ap, format);

	if (huart1.hdmatx->State == HAL_DMA_BURST_STATE_BUSY)
	{
		vsprintf(temp_buf + re_queue_len,format,ap);
		va_end(ap);
		re_queue_len = strlen(temp_buf);
		return;
	}
	vsprintf(first_buf,format,ap);
	va_end(ap);
	HAL_UART_Transmit_DMA(&huart1, (uint8_t *)first_buf, strlen(first_buf)); // 2ms
	return;
}

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TIM_KICK_PERI (2000)

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
uint32_t can_rx_cnt = 0;
CAN_RxHeaderTypeDef can_rx_header;

struct
{
  bool charge_enabled;
  bool kick_chip_selected;
  float target_voltage;
  int kick_power;
} power_cmd;

typedef union
{
  uint8_t data[8];

  struct{
	  uint8_t idx;
	  float value;
  }power;
  struct{
	  uint16_t node_id;
	  uint16_t type;
	  uint32_t etc;
  }error_notify;

  struct{
	  uint8_t cmd;
	  uint8_t enable;
  }power_en;
  struct{
	  uint8_t idx;
	  float value;
  }set_protect_param;
  struct{
	  int16_t delta_x;
	  int16_t delta_y;
  }mouse;
} uint8_to_float_t;

uint8_to_float_t rx,tx;
void setTargetVoltage(float target){
	if(target > 450){
		target = 450;
	}
	if(target < 20){
		target = 20;
	}
	power_cmd.target_voltage = target;
	//printf("set target voltage = %f\n",power_cmd.target_voltage);
}

static uint32_t loop_cnt = 0, kick_cnt = 0;
static int boost_cnt = 0;

void startKick(uint8_t power){
    if (kick_cnt == 0)
    {
      kick_cnt = 100;
      boost_cnt = 0;
      power_cmd.kick_power = TIM_KICK_PERI * power / 255;
      printf("start kick! : %d\n",power_cmd.kick_power);
    }
}

void startCharge(){
    if (boost_cnt == 0 && kick_cnt == 0)
    {
      //printf("boost start!!\n");
      boost_cnt = 1000;
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can_rx_header, rx.data) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler();
  }
  can_rx_cnt++;
  switch (can_rx_header.StdId)
  {
  case 0x000: // emg stop
    power_cmd.charge_enabled = false;
    break;
  case 0x110: // power control
    switch (rx.power.idx)
    {
    case 0: // set boost voltage
    	setTargetVoltage(rx.power.value);
      break;
    case 1: // charge enable
    	if(rx.data[1] == 1){
    		//printf("[CAN] charge enable!\n");
    		power_cmd.charge_enabled = true;
        	startCharge();
    	}else{
    		//printf("[CAN] charge disable!\n");
    		power_cmd.charge_enabled = false;
    	}
      break;
    case 2: // kicker select
    	if(rx.data[1] == 1){
    		//printf("[CAN] select chip kicker\n");
    		power_cmd.kick_chip_selected = true;
    	}else{
    		//printf("[CAN] select straight kicker \n");
    		power_cmd.kick_chip_selected = false;
    	}
      break;
    case 3: // kick !
      printf("[CAN] kick!\n");
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

void sendCan(void)
{
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

void sendCanMouse(int16_t delta_x,int16_t delta_y)
{
  can_header.StdId = 0x240;
  can_header.RTR = CAN_RTR_DATA;
  can_header.DLC = 4;
  can_header.TransmitGlobalTime = DISABLE;
  tx.mouse.delta_x = delta_x;
  tx.mouse.delta_y = delta_y;
  HAL_CAN_AddTxMessage(&hcan, &can_header, tx.data, &can_mailbox);
}

static float boost_v = 0, batt_v, gd_16p, gd_16m, batt_cs;
static int temp_coil_1, temp_coil_2, temp_fet;
void updateADCs(void){
    batt_v = (float)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1) * 3.3 / 4096 * 11 / 1;
    gd_16p = (float)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2) * 3.3 / 4096 * 11 / 1;
    gd_16m = (((float)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3) * 3.3 / 4096) * 21 - gd_16p * 11) / 10;
    boost_v = (float)HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_3) * 213 * 3.3 / 4096 * 1.038; // 1.038 is calib
    // INA199x1 : 50 V/V
    //  2m ohm x 50VV -> 100m V / A
    batt_cs = ((float)HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_1) * 3.3 / 4096) * 10;
    temp_fet = HAL_ADCEx_InjectedGetValue(&hadc4, ADC_INJECTED_RANK_1);
    temp_coil_1 = HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_2);
    temp_coil_2 = HAL_ADCEx_InjectedGetValue(&hadc4, ADC_INJECTED_RANK_2);
}

void protecter(void){
    if (batt_v < 20 || batt_cs > 10 || gd_16p < 10 || gd_16m > -5)
    {
      printf("[ERR] power line error!!! / battv %6.2f battcs %6.3f / GDp %+5.2f GDm %+5.2f\n", batt_v, batt_cs, gd_16p, gd_16m);
      kick_cnt = 0;
      boost_cnt = 0;
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
      HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
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

  p("p()1 hogehoge %d ,%f\n", 10, 1.0);
  p("p()2 hogehoge %d ,%f\n", 10, 1.0);
  p("p()3 hogehoge %d ,%f\n", 10, 1.0);
  p("p()1 hogehoge %d ,%f\n", 10, 1.0);
  p("p()2 hogehoge %d ,%f\n", 10, 1.0);
  p("p()3 hogehoge %d ,%f\n", 10, 1.0);

  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc3);
  HAL_ADC_Start(&hadc4);

  HAL_GPIO_WritePin(POWER_SW_EN_GPIO_Port, POWER_SW_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_CURRENT_GPIO_Port, LED_CURRENT_Pin, GPIO_PIN_RESET);



  /*while(1){

      HAL_Delay(100);
      sendCan();
      printf("can rx : %d\n",can_rx_cnt);
      can_rx_cnt = 0;
  }*/
  if (is_connect_ADNS3080())
  {
    printf("ADNS3080 OK!\n");
  }
  else
  {
    printf("ADNS3080 not found...\n");
    while (1)
    {
      /* code */
    }
  }

  init_ADNS3080(true);

  bool frame_capture_mdoe = false;
  if (HAL_GPIO_ReadPin(SW_1_GPIO_Port, SW_1_Pin) == GPIO_PIN_RESET)
  {
    frame_capture_mdoe = true;
  }
  while (frame_capture_mdoe)
  {
    // frame_print_ADNS3080();
    HAL_Delay(1);

    update_ADNS3080();
    printf("\n\n%+3d %+3d %4d\n\n", get_DeltaX_ADNS3080(), get_DeltaY_ADNS3080(), get_Qualty_ADNS3080());
    HAL_Delay(100);
  }

  setTargetVoltage(400);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    update_ADNS3080();
    sendCanMouse(get_DeltaX_ADNS3080(), get_DeltaY_ADNS3080());

    updateADCs();
    protecter();

    loop_cnt++;
    if (kick_cnt > 0)
    {
        // kick!!!
        if(power_cmd.kick_chip_selected){
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, power_cmd.kick_power);    	  // chip
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        }else{
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, power_cmd.kick_power);	// straight
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
        }
      kick_cnt--;
      if (kick_cnt == 0)
      {
        printf("kick end!!\n");
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
        if(power_cmd.charge_enabled){
        	printf("continue charge!!\n");
            HAL_Delay(10);
        	startCharge();
        }
      }
    }
    else
    {
    	// idol
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    }

    // User SW control
    if (HAL_GPIO_ReadPin(SW_1_GPIO_Port, SW_1_Pin) == GPIO_PIN_RESET)
    {
    	printf("[USR] boost start!!\n");
    	startKick(255);
    }
    if (HAL_GPIO_ReadPin(SW_2_GPIO_Port, SW_2_Pin) == GPIO_PIN_RESET)
    {
    	printf("[USR] boost start!!\n");
    	startCharge();
    }


    // debug print
    if (loop_cnt > 100)
    {
      // printf("%8ld %8ld %8ld / ",HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_1),HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_2),HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_3));
      // printf("%8ld %8ld %8ld / ",HAL_ADCEx_InjectedGetValue(&hadc3,ADC_INJECTED_RANK_1),HAL_ADCEx_InjectedGetValue(&hadc3,ADC_INJECTED_RANK_2),HAL_ADCEx_InjectedGetValue(&hadc3,ADC_INJECTED_RANK_3));
      // printf("%8ld %8ld\n",HAL_ADCEx_InjectedGetValue(&hadc4,ADC_INJECTED_RANK_1),HAL_ADCEx_InjectedGetValue(&hadc4,ADC_INJECTED_RANK_2));
      // HAL_ADCEx_InjectedStart(&hadc1);
    	printf("BV %3.0f, CK %d, CH %d ",power_cmd.target_voltage,power_cmd.kick_chip_selected,power_cmd.charge_enabled);
      printf("%+3d %+3d %4d ", get_DeltaX_ADNS3080(), get_DeltaY_ADNS3080(), get_Qualty_ADNS3080());
      printf("BattV %4.2f, GD16P %+5.2f GD16M %+5.2f BoostV %5.2f, BattCS %+5.2f temp0 %d temp1 %d temp2 %d \n", batt_v, gd_16p, gd_16m, boost_v, batt_cs, temp_fet, temp_coil_1, temp_coil_2);
      // printf("adc1 : ch1 %8ld / ch2 %8ld / ch3 %8ld / adc3: ch1 %8ld / ch5 %8ld / ch12 %8ld / adc4 : ch3 %8ld / ch4 %8ld \n", adc1_raw_data[0], adc1_raw_data[1], adc1_raw_data[2], adc3_raw_data[0],adc3_raw_data[1],adc3_raw_data[2],adc4_raw_data[0],adc4_raw_data[1]);
      loop_cnt = 0;
    }

    //sendCan();
    HAL_Delay(1);
    if (boost_v < power_cmd.target_voltage && boost_cnt > 0)
    {
      boost_cnt--;
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 600);
      HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
      if (boost_cnt == 0)
      {
        printf("[ERR] boost timeout!!\n");
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
      }
    }
    else
    {
      if (boost_cnt != 0)
      {
        //printf("boost end!!\n\n !! %d cycle !!\n\n", boost_cnt);
        boost_cnt = 0;
      }
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
      HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
    }

    // charge-indication
    if (boost_v > 100)
    {
      HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
    }
    else
    {
      HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_ADC12
                              |RCC_PERIPHCLK_ADC34;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV1;
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
