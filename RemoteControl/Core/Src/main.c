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
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "nrf24l01.h"
#include "key.h"
#include "ad.h"
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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t KeyNum;							//本次松手产生的键号，一次性事件
uint16_t AD_LH, AD_LV, AD_RH, AD_RV;	//四路摇杆 ADC 原始值，范围 0~4095
int8_t LH, LV, RH, RV;					//四路摇杆换算值，范围 -100~+100
uint8_t KEY;							//待发送的键号，发送成功后才清零
volatile uint8_t Send_Flag;				//100ms 发送标志，由 1ms 中断置 1（ISR 写 / 主循环读）
uint8_t Send_Result;					//发送结果：1 成功 / 2 重传失败 / 3 状态非法 / 4 超时
uint8_t Sig;							//最近 10 次发送的成功次数，范围 0~10
uint8_t Mode;							//0 摇杆页 / 1 数据回传页
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int8_t DataProcess(uint16_t ADValue);
uint8_t CalculateSuccessRatio(uint8_t SendResult);
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
  /* HAL_Init() 内部固定设置 NVIC_PRIORITYGROUP_4，而本工程按指南要求使用 2 bits
     抢占 + 2 bits 子优先级，必须在这里显式覆盖，否则 tim.c 里的 (2,1) 会被解释成全抢占位 */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();                     /* 硬件 I2C1 初始化 + 清屏 */
  NRF24L01_Init();                 /* 硬件 SPI2 配寄存器，进 RX 模式 */
  AD_Init();                       /* ADC 上电校准 + 启动 DMA 连续转换 */

  /* CubeMX 生成的 MX_TIM1_Init 只初始化不启动，必须显式启动 1ms 中断 */
  HAL_TIM_Base_Start_IT(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 1. 读按键：KeyNum 只用于判断切页，KEY 负责锁存待发送的键号 */
    KeyNum = Key_GetNum();
    if (KeyNum != 0)
    {
      KEY = KeyNum;
    }
    if (KeyNum == 9)                /* KEY9 切换 OLED 页，键号照发给飞控（飞控端忽略） */
    {
      Mode = !Mode;
      OLED_Clear();
    }

    /* 2. 读四路摇杆（DMA 已在后台连续搬运，这里只取最新值），换算成 -100~+100 */
    AD_Read(&AD_LH, &AD_LV, &AD_RH, &AD_RV);

    LH = DataProcess(AD_LH);
    LV = DataProcess(AD_LV);
    RH = DataProcess(AD_RH);
    RV = DataProcess(AD_RV);

    /* 3. 到 100ms 发一帧，即 10Hz */
    if (Send_Flag == 1)
    {
      NRF24L01_TxPacket[0] = Mode;
      NRF24L01_TxPacket[1] = LH;
      NRF24L01_TxPacket[2] = LV;
      NRF24L01_TxPacket[3] = RH;
      NRF24L01_TxPacket[4] = RV;
      NRF24L01_TxPacket[5] = KEY;

      Send_Result = NRF24L01_Send();
      Sig = CalculateSuccessRatio(Send_Result);

      if (Send_Result == 1)
      {
        KEY = 0;                    /* 发送成功才清键号，失败则下一帧重发 */
      }

      Send_Flag = 0;
    }

    /* 4. 显示：Sig 行每轮都刷，与是否收到遥测无关 */
    OLED_Printf(0, 0, OLED_8X16, "Sig:%02d", Sig);

    if (Mode == 0)
    {
      /* 摇杆页 */
      OLED_Printf(0, 16, OLED_8X16, "LH:%+04d", LH);
      OLED_Printf(0, 32, OLED_8X16, "LV:%+04d", LV);
      OLED_Printf(64, 16, OLED_8X16, "RH:%+04d", RH);
      OLED_Printf(64, 32, OLED_8X16, "RV:%+04d", RV);
    }
    else
    {
      /* 回传页才轮询接收，摇杆页不收（原设计） */
      if (NRF24L01_Receive() == 1)
      {
        if (NRF24L01_RxPacket[0] == 0x02)
        {
          int8_t LeftPWM = (int8_t)NRF24L01_RxPacket[1];
          int8_t RightPWM = (int8_t)NRF24L01_RxPacket[2];
          float Angle, LeftSpeed, RightSpeed;

          /* 三路小端 float，用 memcpy 取，避免指针强转的对齐问题 */
          memcpy(&Angle, &NRF24L01_RxPacket[4], 4);
          memcpy(&LeftSpeed, &NRF24L01_RxPacket[8], 4);
          memcpy(&RightSpeed, &NRF24L01_RxPacket[12], 4);

          OLED_Printf(0, 16, OLED_8X16, "Angle:%+06.1f", Angle);
          OLED_Printf(0, 32, OLED_8X16, "PWM:%+05d %+05d", LeftPWM, RightPWM);
          OLED_Printf(0, 48, OLED_8X16, "Spd:%+05.1f %+05.1f", LeftSpeed, RightSpeed);
        }
      }
    }

    OLED_Update();                  /* 整屏刷新，400kHz 下约 23ms */
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
/**
  * @brief  TIM1 1ms 周期中断回调：按键扫描节拍 + 100ms 发送定时
  * @param  htim 触发中断的定时器句柄
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static uint16_t Send_Count;

  if (htim->Instance == TIM1)
  {
    Key_Tick();                     /* 内部每 20ms 真正扫一次 GPIO */

    if (++Send_Count >= 100)        /* 100ms 一帧，即 10Hz */
    {
      Send_Count = 0;
      Send_Flag = 1;
    }
  }
}

/**
  * @brief  ADC 原始值换算成摇杆轴值
  * @param  ADValue ADC 原始值，0~4095
  * @retval -100~+100，中值 2048，死区 ±100
  */
int8_t DataProcess(uint16_t ADValue)
{
  int16_t Value;

  Value = (int16_t)ADValue - 2048;  /* -2048 ~ 2047 */

  if (Value > 100)
  {
    Value -= 100;
  }
  else if (Value < -100)
  {
    Value += 100;
  }
  else
  {
    Value = 0;
  }

  Value = Value * 101 / 1949;       /* -1948 ~ 1947 映射到 -100 ~ 100 */

  return (int8_t)Value;
}

/**
  * @brief  统计最近 10 次发送的成功次数
  * @param  SendResult 单次发送结果，1 表示成功
  * @retval 0~10，只代表链路质量
  */
uint8_t CalculateSuccessRatio(uint8_t SendResult)
{
  static uint8_t SendFlagArray[10];
  static uint8_t p;
  uint8_t i, SuccessCount;

  SendFlagArray[p] = SendResult;
  p++;
  p %= 10;

  SuccessCount = 0;
  for (i = 0; i < 10; i++)
  {
    if (SendFlagArray[i] == 1)
    {
      SuccessCount++;
    }
  }

  return SuccessCount;
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
