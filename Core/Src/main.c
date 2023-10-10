/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
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
CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN PV */
int dataCheck = 0;   /* Flag for blinking LED based on received CAN data */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
CAN_TxHeaderTypeDef txHeader;
CAN_RxHeaderTypeDef rxHeader;

uint8_t txData[8];
uint8_t rxData[8];

uint32_t txMailbox;

/*
 * @brief : An ISR to handle CAN transmit whenever interrupt arrives on EXT1 on user button push.
 * @param: Activated GPIO PIN number.
 * @return: None.
 * */
void HAL_GPIO_EXT1_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_13) {
		txData[0] = 100 ;  /* 100ms delay */
		txData[1] = 10; /* Repeat loop 10 times */

		HAL_CAN_AddTxMessage(&hcan1, &txHeader,txData ,&txMailbox);
	}
}

/*
 * @brief : A Wrapper to handle CAN transmit.
 * @param1: Payload to be transmitted.
 * @return: None.
 * */
void canTransmit(uint8_t txBuff[])
{
	if(HAL_CAN_AddTxMessage(&hcan1, &txHeader,txBuff ,&txMailbox) != HAL_OK) {
		Error_Handler();
	}

}

/*
 * @brief : An ISR to handle CAN Receive whenever data arrives on FIFO0 on user button push.
 * @param: An instance of CAN HandleTypeDef .
 * @return: None.
 * */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData );
	if (rxHeader.DLC == 2)
		dataCheck = 1;
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
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  //int i = 0;
  HAL_CAN_Start(&hcan1);

  /* Activate notification for data pending in the RX FIFO */
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 28;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*
 * @brief : A function to configure CAN TX Header for transmission with Standard ID.
 * @param1: Length of Data to be transmitted in bytes.
 * @param2: The CAN ID to be transmitted in hex format.
 * @return: None.
 * */
void configCANTxStandard(uint8_t dlc, uint32_t hexID)
{
	txHeader.DLC = dlc;
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.StdId = hexID;       // 11 Bit Standard CAN ID.

}

/* USER CODE BEGIN 4 */
/*
 * @brief : A function to configure CAN TX Header for transmission with Extended ID.
 * @param1: Length of Data to be transmitted in bytes.
 * @param2: The CAN ID to be transmitted in hex format.
 * @return: None.
 * */
void configCANTxExtended(uint8_t dlc, uint32_t hexID)
{
	txHeader.DLC = dlc;
	txHeader.IDE = CAN_ID_EXT;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.ExtId = hexID;             //29 bit Extended CAN ID for ISO Compatibility.

}
/*
 * @brief : A function to configure CAN Filter.
 * @param1: The lower 16 bytes of CAN PID.
 * @param2: The higher 16 bytes of CAN PID.
 * @param3: A boolean flag for choosing between extended filtering/ standard filtering, set false for standard filtering.
 * @return: None.
 * */
void CAN_Filter(uint16_t lowID, uint16_t highID, bool isExtended)
{
	  CAN_FilterTypeDef  sFilterConfig;

	 /* uint16_t highID = mask |= ((mask << 16) & 0xFFFF0000);
	  uint16_t lowID = mask; */


	  /*##-2- Configure the CAN Filter ###########################################*/
	  if (isExtended == false) {
		  sFilterConfig.FilterBank = 0;
		  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
		  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
		  sFilterConfig.FilterIdHigh = (highID << 5);  /* As StdID starts from the 5th bit in the high register, as the bits 0-4 are occupied by extended ID */
		  sFilterConfig.FilterIdLow = 0x00;
		  sFilterConfig.FilterMaskIdHigh = (highID << 5); /* As StdID starts from the 5th bit in the high register, as the bits 0-4 are occupied by extended ID */
		  sFilterConfig.FilterMaskIdLow = 0x00;
		  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
		  sFilterConfig.FilterActivation = ENABLE;
		  sFilterConfig.SlaveStartFilterBank = 14;
	  } else if (isExtended == true) {
		  sFilterConfig.FilterBank = 0;
		  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
		  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
		  sFilterConfig.FilterIdHigh = highID;
		  sFilterConfig.FilterIdLow = lowID;
		  sFilterConfig.FilterMaskIdHigh = highID;
		  sFilterConfig.FilterMaskIdLow = lowID;
		  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
		  sFilterConfig.FilterActivation = ENABLE;
		  sFilterConfig.SlaveStartFilterBank = 14;

	  }
	  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
	  {
	    /* Filter configuration Error */
	    Error_Handler();
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
