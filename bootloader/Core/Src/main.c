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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <stdarg.h>
#include <string.h>
#include <stdio.h>

#include "main.h"
#include "crc.h"
#include "eth.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define D_UART &huart2
#define C_UART &huart3
#define BL_RX_LEN 200
#define BL_VERSION 0xAA
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void print_debug_msg(char *format,...);
void BL_uart_read_data(void);
void BL_jump_to_app(void);
char somedata[] = "Hello from Bootloader\r\n";
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t bl_rx_buffer[BL_RX_LEN];

uint8_t supported_cmd[] = {
		BL_GET_VER,
		BL_GET_HELP
};
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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_CRC_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  print_debug_msg("Hello from Bootloader \r\n");
  if ( HAL_GPIO_ReadPin(USER_Btn_GPIO_Port,USER_Btn_Pin) == GPIO_PIN_SET)
  {
	  print_debug_msg("User button is pressed -> Jumping to BL mode\r\n");
	  BL_uart_read_data();
  }
  else{

	  print_debug_msg("User button is not pressed -> Jumping to App\r\n");
	  BL_jump_to_app();
  }
  /* USER CODE END 2 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
}

/* USER CODE BEGIN 4 */

/* Print formatted string to console over uart */
void print_debug_msg(char *format,...){

#ifdef BL_DEBUG_MSG_EN
	char str[80];

	/* Extract the argument list using VA apis */
	va_list args;
	va_start(args, format);
	vsprintf(str, format, args);
	HAL_UART_Transmit(D_UART, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
#endif
}

/* Package example
 *
 *	| Length to follow 	|
 *	---------------------
 *	| Command code     	|
 *	---------------------
 *	| CRC				|
 */
void BL_uart_read_data(void){
	uint8_t rcv_len = 0;
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
	while(1)
	{
		// Set the buffer in 0
		memset(bl_rx_buffer,0,200);

		// Read Length to follow
		HAL_UART_Receive(C_UART, bl_rx_buffer, 1, HAL_MAX_DELAY);
		rcv_len = bl_rx_buffer[0];

		// Read the command code + CRC
		HAL_UART_Receive(C_UART, &bl_rx_buffer[1], rcv_len, HAL_MAX_DELAY);

		switch(bl_rx_buffer[1])
		{
			case BL_GET_VER:
				BL_handle_getver_cmd(bl_rx_buffer);
				break;
			case BL_GET_HELP:
				BL_handle_gethelp_cmd(bl_rx_buffer);
				break;
		}
	}
}


void BL_jump_to_app(void)
{
	/* Function to hold the address of reset handler of the user app */
	void (*app_reset_handler)(void);

	print_debug_msg("BL_DEBUG_MSG -> Jump to user app\r\n");

	/* Main Stack Pointer */
	uint32_t msp_value = *(volatile uint32_t *)FLASH_APP_BASE_ADDRESS;

	print_debug_msg("MSP Value -> %#x\n", msp_value);

	__set_MSP(msp_value);

	/* Reset handler will be the base address + 4 */
	uint32_t reset_handler_address = *(volatile uint32_t *) (FLASH_APP_BASE_ADDRESS + 4);

	app_reset_handler = (void*)reset_handler_address;

	print_debug_msg("Reset handler value -> %#x\n",reset_handler_address);

	/* Jumping to user application*/
	app_reset_handler();


}

void BL_handle_getver_cmd(uint8_t *pBuffer)
{
	uint8_t bl_version;

	// Get total length of cmd packet
	uint32_t cmd_packet_len = pBuffer[0] + 1;

	// Get Host CRC
	uint32_t host_crc = get_host_crc(pBuffer);

	// Check the CRC
	if (!BL_verify_crc(&pBuffer[0], cmd_packet_len-4, host_crc))
	{
		print_debug_msg("BL_DEBUG_MSG: CRC success\n");

		// Send ACK
		BL_send_ack(pBuffer[0], 1);

		bl_version = BL_get_version();
		print_debug_msg("BL_DEBUG_MSG: BL_VERSION : %d %#x \n", bl_version, bl_version);

		BL_uart_write_data(&bl_version,1);
	}
	else
	{
		print_debug_msg("BL_DEBUG_MSG: CRC Failed\n");
		BL_send_nack();
	}

}

uint32_t get_host_crc(uint8_t *pBuffer){

	// Get CRC32 sent by the host
	// |		1B     	  | 1B  |  1B  |  1B  |  1B  |  1B  |
	// | Length to follow | CMD | CRC1 | CRC2 | CRC3 | CRC4 |
	// |                  |     | Cast 4 uint8_t to 1 uint32_t and dereference

	uint32_t cmd_packet_len = pBuffer[0] + 1;
	return *((uint32_t *) (pBuffer + cmd_packet_len - 4));
}


void BL_handle_gethelp_cmd(uint8_t *pBuffer)
{
	// Get total length of cmd packet
	uint32_t cmd_packet_len = pBuffer[0] + 1;

	// Get Host CRC
	uint32_t host_crc = get_host_crc(pBuffer);

	// Check the CRC
	if (!BL_verify_crc(&pBuffer[0], cmd_packet_len-4, host_crc))
	{
		print_debug_msg("BL_DEBUG_MSG: CRC success\n");

		// Send ACK
		BL_send_ack(pBuffer[0], sizeof(supported_cmd));
		BL_uart_write_data(supported_cmd,sizeof(supported_cmd));
	}
	else
	{
		print_debug_msg("BL_DEBUG_MSG: CRC Failed\n");
		BL_send_nack();
	}

}

/**
 * @brief This function verifies the checksum number
 *
 */
CRC_RET BL_verify_crc(uint8_t *pData, uint32_t len, uint32_t crc_host)
{
	uint32_t uwCRCValue = 0xff;
	for(uint32_t i=0; i < len; i++)
	{
		uint32_t i_data = pData[i];
		uwCRCValue = HAL_CRC_Accumulate(&hcrc, &i_data, 1);
	}

	 __HAL_CRC_DR_RESET(&hcrc);

	if (uwCRCValue == crc_host)
	{
		return VERIFY_CRC_SUCCESS;
	}

	return VERIFY_CRC_FAIL;
}


/**
  * @brief This function will send an ACK + 1Byte
  *
  * | ACK = 0xA5 | Follow length for the reply |
  *
  */
void BL_send_ack(uint8_t cmd, uint8_t follow_len)
{
	uint8_t ack_buf[2];
	ack_buf[0] = BL_ACK;
	ack_buf[1] = follow_len;
	HAL_UART_Transmit(C_UART, ack_buf, 2, HAL_MAX_DELAY);
}


/**
  * @brief This function will send an NACK
  *
  * | NACK = 0x7F |
  *
  */
void BL_send_nack(void)
{
	uint8_t nack = BL_NACK;
	HAL_UART_Transmit(C_UART, &nack, 1, HAL_MAX_DELAY);
}


uint8_t BL_get_version(void)
{
	return BL_VERSION;
}

/**
 * @brief This function is a wrapper of UART Transmit function
 */
void BL_uart_write_data(uint8_t *pBuffer, uint32_t len)
{
	HAL_UART_Transmit(C_UART, pBuffer, len, HAL_MAX_DELAY);
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
