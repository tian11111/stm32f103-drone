/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Motor.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "OLED.h"
#include "LED.h"
#include "PowerDetection.h"
#include "BlueSerial.h"
#include "Data_Proc.h"
#include "Variable.h"
#include "Delay.h"
#include "NRF24L01.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  Delay_Init();              /* 先启用 DWT，供设备初始化及收发使用 */
  MPU6050_Init();            /* 复位并等待传感器启动后，再核验身份 */
  uint8_t mpu_who_am_i = 0;
  if (mpu6050_read(MPU_ADDR, MPU_DEVICE_ID_REG, 1, &mpu_who_am_i) != 0 ||
      mpu_who_am_i != 0x68)
  {
    Error_Handler();
  }
  volatile uint8_t mpu_dmp_status = MPU6050_DMP_Init();
  if (mpu_dmp_status != 0)
  {
    Error_Handler();
  }
  LED_Init();
  Motor_Init();              /* HAL_TIM_PWM_Start 四路 + CCR 清零 */
  NRF24L01_Init();           /* 软件 SPI 配寄存器，进 RX 模式 */
  GetBattery_Init();         /* ADC 校准 */
  OLED_Init();               /* 硬件 I2C1 */
  BlueSerial_Init();         /* HAL_UART_Receive_IT 启动接收 */
  if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    /* Zero stays pending until the main loop claims the task. */
    if (Attitude_Proc_time == SlowDown)
    {
        Attitude_Proc_time = 1;
        Attitude_Data_Proc();
    }
    if (Receive_Slow_time == SlowDown)
    {
        Receive_Slow_time = 1;
        Data_Receive();
    }
    if (Proc_Slow_time == SlowDown)
    {
        Proc_Slow_time = 1;
        Data_Proc();
    }
    if (Show_Slow_time == SlowDown)
    {
        Show_Slow_time = 1;
        Data_Display();
        BlueSerial_SendWave();
    }
    if (PowerDete_Slow_time == SlowDown)
    {
        PowerDete_Slow_time = 1;
        GetCur_Power();
    }
    OLED_Task();
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        /* 1 ms tick; pending background tasks remain at zero. */
        if (Receive_Slow_time != SlowDown && ++Receive_Slow_time > 6) Receive_Slow_time = SlowDown;
        if (Proc_Slow_time != SlowDown && ++Proc_Slow_time > 20) Proc_Slow_time = SlowDown;
        if (Show_Slow_time != SlowDown && ++Show_Slow_time > 50) Show_Slow_time = SlowDown;
        if (Attitude_Proc_time != SlowDown && ++Attitude_Proc_time > 4) Attitude_Proc_time = SlowDown;
        /* After reset to zero, > 3 invokes the flight task every 4 ms. */
        if (++Task_Slow_time > 3)
        {
            Task_Slow_time = 0;
            Fly_task();
        }
        if (PowerDete_Slow_time != SlowDown && ++PowerDete_Slow_time > 10) PowerDete_Slow_time = SlowDown;
        LED_Detect();
        if (connect_time < 500) connect_time++;
        if (connect_time >= 500)
        {
            connect_protect = 1;
            Lock = 1;
            Oil_Get = Oil_Set = 0;
            getpitch = getroll = 50;
            getyaw = 100;
        }
    }
}

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
#ifdef USE_FULL_ASSERT
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
