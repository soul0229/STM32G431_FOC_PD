/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cordic.h"
#include "dma.h"
#include "fmac.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FocCommon.h"
#include "Motor.h"
#include <stdint.h>
#include <arm_math.h>
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
CORDIC_ConfigTypeDef sCordicConfig;
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
FOC_t *FOC_motor[4] = {NULL};
bool motor[4] = {0};
uint8_t motor_n = 0;
const uint16_t delay[4] = {100, 100, 100, 700};
uint8_t cnt = 0;
extern FocParam *param;
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
  MX_ADC2_Init();
  MX_CORDIC_Init();
  MX_FMAC_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();
  MX_USB_Device_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /*##-2- Configure the CORDIC peripheral ####################################*/
  sCordicConfig.Function         = CORDIC_FUNCTION_SINE;     /* sine function */
  sCordicConfig.Precision        = CORDIC_PRECISION_6CYCLES; /* max precision for q1.31 sine */
  sCordicConfig.Scale            = CORDIC_SCALE_0;           /* no scale */
  sCordicConfig.NbWrite          = CORDIC_NBWRITE_1;         /* One input data: angle. Second input data (modulus) is 1 after cordic reset */
  sCordicConfig.NbRead           = CORDIC_NBREAD_2;          /* Two output data: sine then cosine */
  sCordicConfig.InSize           = CORDIC_INSIZE_32BITS;     /* q1.31 format for input data */
  sCordicConfig.OutSize          = CORDIC_OUTSIZE_32BITS;    /* q1.31 format for output data */

  if (HAL_CORDIC_Configure(&hcordic, &sCordicConfig) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

  // FOC_motor[0] = motor_init(&htim1);
  FOC_motor[1] = motor_init(&htim8, &hadc2);
  // HAL_CORDIC_Calculate_DMA();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_Delay(delay[cnt++%4]);
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 21;
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
/* [last][current] */
uint8_t last[2] = {0}, current[2] = {0};
const int8_t direction[8][8]  = 
{
	[0] = {},

	[1] = {[3] = -100, [5] = 100},
	[5] = {[1] = -100, [4] = 100},
	[4] = {[5] = -100, [6] = 100},
	[6] = {[4] = -100, [2] = 100},
	[2] = {[6] = -100, [3] = 100},
	[3] = {[2] = -100, [1] = 100},

	[7] = {},
};

const uint16_t hall_angle[8] = {0, 28806, 6930, 1614, 17952, 23352, 12234, 0};
const uint16_t hall_angle_r[8] = {0, 32377, 10347, 4747, 21261, 26815, 16157, 0};
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) 
{
  if(htim == &htim2)
  {
    // if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    //   param.speed = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
    // param.HallA[2] = HAL_GPIO_ReadPin(HALL1_U_GPIO_Port, HALL1_U_Pin);
    // param.HallA[1] = HAL_GPIO_ReadPin(HALL1_V_GPIO_Port, HALL1_V_Pin);
    // param.HallA[0] = HAL_GPIO_ReadPin(HALL1_W_GPIO_Port, HALL1_W_Pin);

    // // FOC_motor[1]->radian = ((uint16_t)(hall_angle[param.HallA[0]<<2|param.HallA[1]<<1|param.HallA[2]]-8192))%(uint16_t)32768;
    // // HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&param, sizeof(FocParam));
    // current[0] = param.HallA[0] | param.HallA[1] << 1 | param.HallA[2] << 2;
    // param.Dirction[0] = direction[last[0]][current[0]];
    // last[0] = current[0];
  }
  else if(htim == &htim3)
  {
    // param.HallB[2] = HAL_GPIO_ReadPin(HALL2_U_GPIO_Port, HALL2_U_Pin);
    // param.HallB[1] = HAL_GPIO_ReadPin(HALL2_V_GPIO_Port, HALL2_V_Pin);
    // param.HallB[0] = HAL_GPIO_ReadPin(HALL2_W_GPIO_Port, HALL2_W_Pin);

    // current[1] = param.HallB[0] | param.HallB[1] << 1 | param.HallB[2] << 2;
    // param.Dirction[1] = direction[last[1]][current[1]];
    // last[1] = current[1];
  }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if(hadc == FOC_motor[1]->pRsSamp->priv)
    {
      // FOC_motor[1]->GetPreCurrent(FOC_motor[1]);
      FocControl(FOC_motor[1]);
      // HAL_ADCEx_InjectedGetValue(&hadc2,ADC_INJECTED_RANK_4);
      // param->adc_value[0] = FOC_motor[1]->ia;
      // param->adc_value[1] = FOC_motor[1]->ib;
      // param->adc_value[2] = FOC_motor[1]->ic;
    }
}

bool state = true;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == KEY1_Pin)
  {
    FOC_motor[1]->EnableMotor(FOC_motor[1], state);
    state = state?false:true;
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
