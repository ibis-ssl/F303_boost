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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int f getc(FILE *f)
#endif

void __io_putchar(uint8_t ch)
{
  HAL_UART_Transmit(&huart1, &ch, 1, 1);
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

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



  //kick
  HAL_TIM_PWM_Init(&htim3);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_AUTORELOAD(&htim3, 2000);
  __HAL_TIM_SET_AUTORELOAD(&htim3, 2000);
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
  //wait charging



  setbuf(stdout, NULL);

  HAL_UART_Init(&huart1);

  uint8_t data[] = "orion boost v1 start!!\n";

  HAL_UART_Transmit(&huart1, data, sizeof(data), 100);
  printf("hogehoge %d ,%f\n", 10, 1.0);

  static uint32_t loop_cnt = 0, kick_cnt = 0;
  float boost_v = 0,batt_v,gd_16p,gd_16m,batt_cs;
  int temp_coil_1,temp_coil_2,temp_fet;
  int boost_cnt = 0;


  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc3);
  HAL_ADC_Start(&hadc4);

  HAL_GPIO_WritePin(POWER_SW_EN_GPIO_Port, POWER_SW_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_CURRENT_GPIO_Port, LED_CURRENT_Pin, GPIO_PIN_SET);


  uint8_t sbuf[16] = {0}, rbuf[16] = {0};

  HAL_SPI_TransmitReceive(&hspi1, sbuf, rbuf, 1, 1000);

  HAL_GPIO_WritePin(MOUSE_NSS_GPIO_Port, MOUSE_NSS_Pin, GPIO_PIN_SET);
  HAL_Delay(1);

  HAL_GPIO_WritePin(MOUSE_RST_GPIO_Port, MOUSE_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(MOUSE_RST_GPIO_Port, MOUSE_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);

  HAL_GPIO_WritePin(MOUSE_NSS_GPIO_Port, MOUSE_NSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, sbuf, rbuf, 2, 1000);
  HAL_GPIO_WritePin(MOUSE_NSS_GPIO_Port, MOUSE_NSS_Pin, GPIO_PIN_SET);

  printf("SPI ID : %d\n",rbuf[1]);


  HAL_Delay(10);
/*
  sbuf[0] = 0x0A;	//config read

  HAL_GPIO_WritePin(MOUSE_NSS_GPIO_Port, MOUSE_NSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, sbuf, rbuf, 2, 1000);
  HAL_GPIO_WritePin(MOUSE_NSS_GPIO_Port, MOUSE_NSS_Pin, GPIO_PIN_SET);

  printf("Config : %d\n",rbuf[1]);

  uint8_t mouse_config = rbuf[1];

  HAL_Delay(10);

  sbuf[0] = 0x80 & 0x0A;	//config write
  sbuf[1] = mouse_config | 0x10;

  HAL_GPIO_WritePin(MOUSE_NSS_GPIO_Port, MOUSE_NSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, sbuf, rbuf, 2, 1000);
  HAL_GPIO_WritePin(MOUSE_NSS_GPIO_Port, MOUSE_NSS_Pin, GPIO_PIN_SET);


  HAL_Delay(10);
*/
/*
  char scale[] = "#987654321-,.'` ";

  while(1){
	  sbuf[0] = 0x13 | 0x80;// frame capture write
	  sbuf[1] = 0x83;
	  HAL_GPIO_WritePin(MOUSE_NSS_GPIO_Port, MOUSE_NSS_Pin, GPIO_PIN_RESET);
	  HAL_SPI_TransmitReceive(&hspi1, sbuf, rbuf, 2, 1000);
	  HAL_GPIO_WritePin(MOUSE_NSS_GPIO_Port, MOUSE_NSS_Pin, GPIO_PIN_SET);

	  HAL_Delay(2);

	  for(int pixel_x = 0;pixel_x < 30; pixel_x++){
		  for(int pixel_y = 0;pixel_y < 30; pixel_y++){
			  sbuf[0] = 0x13;// frame capture
			  HAL_GPIO_WritePin(MOUSE_NSS_GPIO_Port, MOUSE_NSS_Pin, GPIO_PIN_RESET);
			  HAL_SPI_TransmitReceive(&hspi1, sbuf, rbuf, 2, 1000);
			  HAL_GPIO_WritePin(MOUSE_NSS_GPIO_Port, MOUSE_NSS_Pin, GPIO_PIN_SET);
			  //printf("%2d ",(rbuf[1] % 0x3F)>>2);
			  printf("%c",scale[(rbuf[1] % 0x3F)>>2]);
		  }
		  printf("\n");
	  }
	  printf("\n\n");


	  HAL_Delay(100);
  }*/


  sbuf[0] = 0x50;	//motion burst
  int pos_x = 0,pos_y = 0;
  while(1){

	  HAL_GPIO_WritePin(MOUSE_NSS_GPIO_Port, MOUSE_NSS_Pin, GPIO_PIN_RESET);
	  HAL_SPI_TransmitReceive(&hspi1, sbuf, rbuf, 5, 1000);
	  HAL_GPIO_WritePin(MOUSE_NSS_GPIO_Port, MOUSE_NSS_Pin, GPIO_PIN_SET);

	  printf("%5d %5d %5d %5d %5d - ",rbuf[0],rbuf[1],rbuf[2],rbuf[3],rbuf[4]);
	  if(rbuf[1] & 0x80){
		  pos_x += (int8_t)rbuf[2];
		  pos_y += (int8_t)rbuf[3];
	  }
	  printf("%2x x %+5d y %+5d q %4d\n",rbuf[0],pos_x,pos_y,(int8_t)rbuf[4]);

	  HAL_Delay(100);
  }



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    batt_v = (float)HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_1) * 3.3 / 4096 * 11/1;
	    gd_16p = (float)HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_2) * 3.3 / 4096 * 11/1;
	    gd_16m = (((float)HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_3) * 3.3 / 4096) * 21 - gd_16p*11)/10;
	    boost_v = (float)HAL_ADCEx_InjectedGetValue(&hadc3,ADC_INJECTED_RANK_3) * 213 * 3.3 / 4096 * 1.038; //1.038 is calib
	    //INA199x1 : 50 V/V
	    // 2m ohm x 50VV -> 100m V / A
	    batt_cs = ((float)HAL_ADCEx_InjectedGetValue(&hadc3,ADC_INJECTED_RANK_1) * 3.3 / 4096) * 10;
	    temp_fet = HAL_ADCEx_InjectedGetValue(&hadc4,ADC_INJECTED_RANK_1);
	    temp_coil_1 = HAL_ADCEx_InjectedGetValue(&hadc3,ADC_INJECTED_RANK_2);
	    temp_coil_2 = HAL_ADCEx_InjectedGetValue(&hadc4,ADC_INJECTED_RANK_2);

	    if(batt_v < 20 || batt_cs > 10 || gd_16p < 10 || gd_16m > -5){
	    	//printf("power line error!!! / battv %6.2f battcs %6.3f / GDp %+5.2f GDm %+5.2f\n",batt_v,batt_cs,gd_16p,gd_16m);
	    	kick_cnt = 0;
	    	boost_cnt = 0;
	        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
	        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
	    }

	    loop_cnt++;
	    if (kick_cnt > 0)
	    {
	      kick_cnt--;
	      if(kick_cnt == 0){
	          printf("kick end!!\n");
	      }
	      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1900);
	      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1900);
	    }
	    else
	    {
	      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	    }
	    if (HAL_GPIO_ReadPin(SW_1_GPIO_Port, SW_1_Pin) == GPIO_PIN_RESET)
	    {
	      if (kick_cnt == 0)
	      {
	        printf("kick start!!\n");
	        kick_cnt = 100;
	        boost_cnt = 0;
	      }
	    }
	    if (HAL_GPIO_ReadPin(SW_2_GPIO_Port, SW_2_Pin) == GPIO_PIN_RESET)
	    {
	      if (boost_cnt == 0)
	      {
	        printf("boost start!!\n");
	      }
	      boost_cnt = 1000;
	    }

	    if (loop_cnt > 100)
	    {
	    //printf("%8ld %8ld %8ld / ",HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_1),HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_2),HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_3));
	    //printf("%8ld %8ld %8ld / ",HAL_ADCEx_InjectedGetValue(&hadc3,ADC_INJECTED_RANK_1),HAL_ADCEx_InjectedGetValue(&hadc3,ADC_INJECTED_RANK_2),HAL_ADCEx_InjectedGetValue(&hadc3,ADC_INJECTED_RANK_3));
	    //printf("%8ld %8ld\n",HAL_ADCEx_InjectedGetValue(&hadc4,ADC_INJECTED_RANK_1),HAL_ADCEx_InjectedGetValue(&hadc4,ADC_INJECTED_RANK_2));
	    //HAL_ADCEx_InjectedStart(&hadc1);
	    printf("BattV %4.2f, GD16P %+5.2f GD16M %+5.2f BoostV %5.2f, BattCS %+5.2f temp0 %d temp1 %d temp2 %d \n",batt_v,gd_16p,gd_16m,boost_v,batt_cs,temp_fet,temp_coil_1,temp_coil_2);
	      //printf("adc1 : ch1 %8ld / ch2 %8ld / ch3 %8ld / adc3: ch1 %8ld / ch5 %8ld / ch12 %8ld / adc4 : ch3 %8ld / ch4 %8ld \n", adc1_raw_data[0], adc1_raw_data[1], adc1_raw_data[2], adc3_raw_data[0],adc3_raw_data[1],adc3_raw_data[2],adc4_raw_data[0],adc4_raw_data[1]);
	      loop_cnt = 0;
	    }
	    HAL_Delay(1);
	    if (boost_v < 450 && boost_cnt > 0)
	    {
	    	boost_cnt--;
	    	if(boost_cnt == 0){
	    		printf("boost timeout!!\n");
	    	}
	      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 600);
	      HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
	    }
	    else
	    {
	    	if(boost_cnt != 0){
	    		printf("boost end!!\n\n !! %d cycle !!\n\n",boost_cnt);
	    		boost_cnt = 0;
	    	}
	      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
	      HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
	    }

	    // charge-indication
	    if(boost_v > 100){
	        HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
	    }else{
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
