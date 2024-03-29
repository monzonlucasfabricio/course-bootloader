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
void Jump_To(uint32_t address);
uint8_t get_rdp_level(void);
uint8_t verify_address(uint32_t address);
uint8_t execute_flash_erase(uint8_t sector_num, uint8_t n_sectors);
uint8_t execute_mem_write(uint8_t *pBuffer, uint32_t address, uint32_t len);
uint16_t get_mcu_chip_id(void);
char somedata[] = "Hello from Bootloader\r\n";
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t bl_rx_buffer[BL_RX_LEN];

uint8_t supported_cmd[] = {
		BL_GET_VER,
		BL_GET_HELP,
		BL_GET_CID,
		BL_GET_RDP,
		BL_GO_TO,
		BL_FLASH_ERASE,
		BL_WRITE_MEM,
		BL_JUMP_TO_APP
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
			case BL_GET_CID:
				BL_handle_getcid_cmd(bl_rx_buffer);
				break;
			case BL_JUMP_TO_APP:
				BL_handle_jump_to_app(bl_rx_buffer);
				break;
			case BL_GET_RDP:
				BL_handle_getrdp_cmd(bl_rx_buffer);
				break;
			case BL_GO_TO:
				BL_handle_go_cmd(bl_rx_buffer);
				break;
			case BL_FLASH_ERASE:
				BL_handle_flash_erase_cmd(bl_rx_buffer);
				break;
			case BL_WRITE_MEM:
				BL_handle_write_mem_cmd(bl_rx_buffer);
				break;
		}
	}
}


void BL_jump_to_app(void)
{
	/* Function to hold the address of reset handler of the user app */
	typedef void (*app_reset_handler)(void);

	/* Point to vector table (The start of the app) */
	uint32_t *_vectable = (__IO uint32_t *) FLASH_APP_BASE_ADDRESS;

	/* Get the address of reset handler */
	app_reset_handler app_jump = (app_reset_handler) *(_vectable + 1);

	/* SCB-VTOR : Pointer to vector table (Start of app) */
	SCB->VTOR = _vectable;

	/* Set main stack pointer */
	__set_MSP(*_vectable);

	/* Jumping to user application*/
	app_jump();

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

void BL_handle_getcid_cmd(uint8_t *pBuffer)
{
	// Get total length of cmd packet
	uint32_t cmd_packet_len = pBuffer[0] + 1;

	// Get Host CRC
	uint32_t host_crc = get_host_crc(pBuffer);

	// Check the CRC
	if (!BL_verify_crc(&pBuffer[0], cmd_packet_len-4, host_crc))
	{
		print_debug_msg("BL_DEBUG_MSG: CRC success\n");

		// Get CHIP ID
		uint16_t cid = get_mcu_chip_id();

		// Send ACK
		BL_send_ack(pBuffer[0], sizeof(supported_cmd));

		// Casting to (uint8_t *) because is a 16bits number
		// &cid -> memory address of cid (uint16_t *): 		0x0800000 1234
		// 													0x0800002 0001
		//
		// Depending on endianess
		// (uint8_t*) &cid -> Casting to uint8_t *			0x0800000 34
		//													0x0800001 12
		//													0x0800002 01
		//													0x0800003 00
		BL_uart_write_data((uint8_t *)&cid,2);
	}
	else
	{
		print_debug_msg("BL_DEBUG_MSG: CRC Failed\n");
		BL_send_nack();
	}

}

void BL_handle_jump_to_app(uint8_t *pBuffer)
{
	// Get total length of cmd packet
	uint32_t cmd_packet_len = pBuffer[0] + 1;

	// Get Host CRC
	uint32_t host_crc = get_host_crc(pBuffer);

	// Check the CRC
	if (!BL_verify_crc(&pBuffer[0], cmd_packet_len-4, host_crc))
	{
		print_debug_msg("BL_DEBUG_MSG: CRC success\n");
		uint8_t z = 0;
		// Send ACK
		BL_send_ack(pBuffer[0], 1);
		// Send 0 because we are jumping to app
		BL_uart_write_data(&z,1);

		// Jump and stop responding to BL
		BL_jump_to_app();
	}
	else
	{
		print_debug_msg("BL_DEBUG_MSG: CRC Failed\n");
		BL_send_nack();
	}

}

void BL_handle_getrdp_cmd(uint8_t *pBuffer)
{
	// Get total length of cmd packet
	uint32_t cmd_packet_len = pBuffer[0] + 1;

	// Get Host CRC
	uint32_t host_crc = get_host_crc(pBuffer);

	// Check the CRC
	if (!BL_verify_crc(&pBuffer[0], cmd_packet_len-4, host_crc))
	{
		print_debug_msg("BL_DEBUG_MSG: CRC success\n");

		// Get Flash read protection level
		uint8_t rdp = get_rdp_level();

		// Send ACK
		BL_send_ack(pBuffer[0], sizeof(supported_cmd));
		BL_uart_write_data(&rdp,1);
	}
	else
	{
		print_debug_msg("BL_DEBUG_MSG: CRC Failed\n");
		BL_send_nack();
	}

}

void BL_handle_go_cmd(uint8_t *pBuffer)
{
	uint32_t address;
	uint8_t valid = 0;
	uint8_t invalid = 1;
	// Get total length of cmd packet
	uint32_t cmd_packet_len = pBuffer[0] + 1;

	// Get Host CRC
	uint32_t host_crc = get_host_crc(pBuffer);

	// Check the CRC
	if (!BL_verify_crc(&pBuffer[0], cmd_packet_len-4, host_crc))
	{
		print_debug_msg("BL_DEBUG_MSG: CRC success\n");

		// Example of value in memory uint8_t	->  	&pBuffer[0] = 0x20000400	0x05
		//												&pBuffer[1] = 0x20000401	0x06
		//												&pBuffer[2] = 0x20000402	0x80
		//												&pBuffer[3] = 0x20000403	0x00
		//												&pBuffer[4] = 0x20000404	0x08
		//												&pBuffer[5] = 0x20000405	0x10
		// pBuffer[2] = 0x80
		// &pBuffer[2] = 0x20000402
		// *((uint32_t *) &pBuffer[2]) = 0x80000810

		address = *((uint32_t *) &pBuffer[2]);
		print_debug_msg("BL_DEBUG_MSG: Go address -> %#x\n",address);

		if (!verify_address(address)){
			// Send ACK
			print_debug_msg("BL_DEBUG_MSG: Address is valid \n");
			BL_send_ack(pBuffer[0], sizeof(supported_cmd));
			BL_uart_write_data(&valid,1);
			Jump_To(address);
		}
		else{
			print_debug_msg("BL_DEBUG_MSG: Address is valid \n");
			BL_uart_write_data(&invalid,1);
		}
	}
	else
	{
		print_debug_msg("BL_DEBUG_MSG: CRC Failed\n");
		BL_send_nack();
	}

}


void BL_handle_flash_erase_cmd(uint8_t *pBuffer)
{
	HAL_StatusTypeDef status;

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
		
		// pBuffer[2] => Sector Number
		// pBuffer[3] => Number of Sectors
		status = execute_flash_erase(pBuffer[2], pBuffer[3]);
		print_debug_msg("BL_DEBUG_MSG: Flash Erase Status: %#x \n",status);

		// Send the status from erasing the flash
		BL_uart_write_data(&status,1);
	}
	else
	{
		print_debug_msg("BL_DEBUG_MSG: CRC Failed\n");
		BL_send_nack();
	}

}


// Length to follow 	=> pBuffer[0]
// ----------------
// Command code			=> pBuffer[1]
// ----------------
// Base Memory Address 	=> pBuffer[2] -> 4 bytes
// ----------------
// Payload Length		=> pBuffer[6] 
// ----------------
// Payload				=> pBuffer[7]
// ----------------
// CRC
void BL_handle_write_mem_cmd(uint8_t *pBuffer)
{
	uint32_t address;
	// Get total length of cmd packet
	uint32_t cmd_packet_len = pBuffer[0] + 1;
	// Get Host CRC
	uint32_t host_crc = get_host_crc(pBuffer);

	uint8_t len = pBuffer[0];
	uint8_t payload_len = pBuffer[6];
	address = *((uint32_t *) &pBuffer[2]);

	// Check the CRC
	if (!BL_verify_crc(&pBuffer[0], cmd_packet_len-4, host_crc))
	{
		print_debug_msg("BL_DEBUG_MSG: CRC success\n");

		// Send ACK
		BL_send_ack(pBuffer[0], sizeof(supported_cmd));
		
		if (!verify_address(address))
		{
			status = execute_mem_write(&pBuffer[7], address, payload_len);
		}
		// Send the status from writing the memory
		BL_uart_write_data(&status,1);
	}
	else
	{
		print_debug_msg("BL_DEBUG_MSG: CRC Failed\n");
		status = ADDR_INVALID;
		BL_uart_write_data(&status,1);
	}

}

uint8_t execute_mem_write(uint8_t *pBuffer, uint32_t address, uint32_t len)
{
	uint8_t status = HAL_OK;

	HAL_FLASH_Unlock();

	for (uint32_t i = 0; i < len; i++)
	{
		status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address + i, pBuffer[i]);
	}

	HAL_FLASH_Lock();

	return status;
}

uint8_t execute_flash_erase(uint8_t sector_num, uint8_t n_sectors)
{
	FLASH_EraseInitTypeDef erase_struct;
	uint32_t sectorError;
	HAL_StatusTypeDef status;

	if (n_sectors > 8) return INVALID_SECTOR;
	if ((sector_num == 0xFF) || (sector_num <= 7))
	{
		if (sector_num == (uint8_t) 0xFF)
		{
			erase_struct.TypeErase = FLASH_TYPEERASE_MASSERASE;
		}
		else
		{
			uint8_t remaining_sector = FLASH_MAX_SECTORS - sector_num;
			if (n_sectors > remaining_sector) n_sectors = remaining_sector;
			erase_struct.TypeErase = FLASH_TYPEERASE_SECTORS;
			erase_struct.Sector = sector_num;
			erase_struct.NbSectors = n_sectors;
		}
		erase_struct.Banks = FLASH_BANK_1;

		HAL_FLASH_Unlock();
		erase_struct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		status = (uint8_t) HAL_FLASHEx_Erase(&erase_struct, &sectorError);
		HAL_FLASH_Lock();

		return status;
	}

	return FLASH_INVALID_SECTOR;
}

void Jump_To(uint32_t address)
{
	// Make T bit = 1
	address += 1;
	void (*jump)(void) = (void *) address;
	jump();
}

uint8_t verify_address(uint32_t address)
{
	/* Valid addresses:
	 * - System Memory
	 * - SRAM1
	 * - SRAM2
	 * - BACKUP SRAM
	 * - EXTERNAL MEMORY
	 */
	if (address >= FLASH_BL_BASE_ADDRESS && address <= FLASH_END_ADDR)
	{
		return ADDR_VALID;
	}
	else if (address >= RAM_START_ADDR && address <= RAM_END_ADDR){
		return ADDR_VALID;
	}
	return ADDR_INVALID;
}

uint16_t get_mcu_chip_id(void)
{
	uint16_t cid;
	// This CID identifies the MCU part number and DIE revision.
	cid = (uint16_t)(DBGMCU->IDCODE) & 0x0FFF;
	return cid;
}


/**
 *  @brief 	This method will read the Flash Protection Level
 */
uint8_t get_rdp_level(void)
{
	uint8_t rdp_status = 0;
	// It uses the raw value of the address and not the APIs.
	volatile uint32_t *pOB_addr = (uint32_t*) 0x1FFFC000;
	rdp_status = (uint8_t)(*pOB_addr >> 8);
	return rdp_status;
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
