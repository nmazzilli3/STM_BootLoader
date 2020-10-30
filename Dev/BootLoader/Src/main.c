/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdarg.h>
#include <string.h>
#include <stdint.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  NOT_BLINKING = 0U,
  BLINKING,
  TRANSITION_BLINK_TO_NOT_BLINKING,
  TRANSITION_TO_NOT_BLINKING
}Button_state_machine_t;

typedef enum
{
  BUTTON_PRESSED = 0U,
  BUTTON_RELEASED
}Button_status_t;

typedef struct
{
  uint16_t active_cnt;
  uint16_t deactive_cnt;
  const uint16_t hysteresis_cnt;
  Button_status_t current_btn_state_t;
}debounce_button_status_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BL_DEBUG_MSG_EN
#define DEBUG_SERIAL_ENABLED_B 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static void printmsg(char *format,...);

void bootloader_jump_to_user_app(void);
void bootloader_uart_read_data(void);
void bootloader_handle_getver_cmd(uint8_t *bl_rx_buffer);
void bootloader_handle_gethelp_cmd(uint8_t *pBuffer);
void bootloader_handle_getcid_cmd(uint8_t *pBuffer);
void bootloader_handle_getrdp_cmd(uint8_t *pBuffer);
void bootloader_handle_go_cmd(uint8_t *pBuffer);
void bootloader_handle_flash_erase_cmd(uint8_t *pBuffer);
void bootloader_handle_mem_write_cmd(uint8_t *pBuffer);
void bootloader_handle_en_rw_protect(uint8_t *pBuffer);
void bootloader_handle_mem_read (uint8_t *pBuffer);
void bootloader_handle_read_sector_protection_status(uint8_t *pBuffer);
void bootloader_handle_read_otp(uint8_t *pBuffer);
void bootloader_handle_dis_rw_protect(uint8_t *pBuffer);
uint8_t bootloader_verify_crc_swapped(uint8_t *pData, uint32_t len, uint32_t crc_host);
HAL_StatusTypeDef set_flash_rdp_level(uint8_t);
void bootloader_send_ack(uint8_t command_code, uint8_t follow_len);
void bootloader_send_nack(void);
HAL_StatusTypeDef disable_write_flash_prot(uint8_t page_number);

uint8_t bootloader_verify_crc (uint8_t *pData, uint32_t len,uint32_t crc_host);
uint8_t get_bootloader_version(void);
void bootloader_uart_write_data(uint8_t *pBuffer,uint32_t len);

uint16_t get_mcu_chip_id(void);
uint8_t get_flash_rdp_level(void);
uint8_t verify_address(uint32_t go_address);
uint8_t execute_flash_erase(uint8_t page_number , uint8_t number_of_page);
uint8_t execute_mem_write(uint8_t *pBuffer, uint32_t mem_address, uint32_t len);
uint32_t swap_byte_order(uint32_t num);

uint8_t configure_flash_sector_rw_protection(uint8_t sector_details, uint8_t protection_mode, uint8_t disable);

uint16_t read_OB_rw_protection_status(void);

uint32_t page_details_to_2pages(uint8_t page_details);

void init_debounce_struct_t(debounce_button_status_t *);

void init_debounce_struct_t(debounce_button_status_t *struct_t)
{
	struct_t->active_cnt = 0;
	struct_t->current_btn_state_t = BUTTON_RELEASED;
	struct_t->deactive_cnt = 0;
}

void debounc_btn(debounce_button_status_t *,const GPIO_PinState);

void debounc_btn(debounce_button_status_t *btn_status_t,const GPIO_PinState GPIO_return_t)
{
	uint32_t overflow_cntr = 0;
	if(GPIO_return_t == GPIO_PIN_RESET)
	{
		overflow_cntr = btn_status_t->active_cnt +1;
		if(overflow_cntr>65535)
		{
			btn_status_t->active_cnt = 65535;
		}
		else
		{
			btn_status_t->active_cnt = (uint16_t)overflow_cntr;
		}
		if(btn_status_t->active_cnt >= btn_status_t->hysteresis_cnt)
		{
			btn_status_t->current_btn_state_t = BUTTON_PRESSED;
		}
		else
		{
			btn_status_t->current_btn_state_t = BUTTON_RELEASED;
		}


	}
	else
	{
		btn_status_t->active_cnt = 0;
		btn_status_t->current_btn_state_t = BUTTON_RELEASED;
		overflow_cntr = btn_status_t->active_cnt +1;
		if(overflow_cntr>65535)
		{
			btn_status_t->deactive_cnt = 65535;
		}
		else
		{
			btn_status_t->deactive_cnt = (uint16_t)overflow_cntr;
		}
	}

}


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define D_UART &huart2
#define BL_RX_LEN 200
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static GPIO_PinState gpio_status_t = GPIO_PIN_RESET;
static debounce_button_status_t user_btn_status_t = {.hysteresis_cnt = 50};
static Button_state_machine_t button_state_t = NOT_BLINKING;
static HAL_StatusTypeDef uart_status_t;
char somedata[] = "Hello from Bootloader\r\n";
uint8_t bl_rx_buffer[BL_RX_LEN];
uint8_t supported_commands[] = {
                               BL_GET_VER ,
                               BL_GET_HELP,
                               BL_GET_CID,
                               BL_GET_RDP_STATUS,
                               BL_GO_TO_ADDR,
                               BL_FLASH_ERASE,
                               BL_MEM_WRITE,
                               BL_READ_SECTOR_P_STATUS} ;
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
  MX_USART2_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  init_debounce_struct_t(&user_btn_status_t);
  gpio_status_t = HAL_GPIO_ReadPin(USER_BTN_GPIO_Port,USER_BTN_Pin);
  if(gpio_status_t == GPIO_PIN_RESET)
  {
	  if(DEBUG_SERIAL_ENABLED_B)
	  {
		  printmsg("BL_DEBUG_MSG:Button is pressed .. going to BootLoader Mode\r\n");
	  }

	  bootloader_uart_read_data();
  }
  else
  {
	  if(DEBUG_SERIAL_ENABLED_B)
	  {
		  printmsg("BL_DEBUG_MSG:Button is not pressed .. going to execute user app\r\n");
	  }
	  bootloader_jump_to_user_app();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
  hcrc.Init.GeneratingPolynomial = 0x04C11DB7;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.CRCLength = 32;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_WORDS;
  /*
   * hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_HALFWORDS : CRC = 3186128208 when it should be 2091641319
   * hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES : CRC = 3957076664 when it should be 2091641319
   */
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BTN_Pin */
  GPIO_InitStruct.Pin = USER_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**************Implementation of Boot-loader Command Handle functions *********/

void printmsg(char *format,...)
{
#ifdef BL_DEBUG_MSG_EN
	char str[80];
	va_list args;
	va_start(args,format);
	vsprintf(str,format,args);
	HAL_UART_Transmit(D_UART,(uint8_t*)str,strlen(str),HAL_MAX_DELAY);
	va_end(args);
#endif
}

uint32_t swap_byte_order(uint32_t num)
{
	uint32_t swapped = 0;
	swapped = ((num>>24)&0xff) | // move byte 3 to byte 0
	                    ((num<<8)&0xff0000) | // move byte 1 to byte 2
	                    ((num>>8)&0xff00) | // move byte 2 to byte 1
	                    ((num<<24)&0xff000000); // byte 0 to byte 3
	return swapped;
}

/*Helper function to handle BL_GET_VER command */
void bootloader_handle_getver_cmd(uint8_t *bl_rx_buffer)
{
    uint8_t bl_version;

    // 1) verify the checksum
    if(DEBUG_SERIAL_ENABLED_B)
    {
      printmsg("BL_DEBUG_MSG:bootloader_handle_getver_cmd\r\n");
    }

	 //Total length of the command packet
	  uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	  //extract the CRC32 sent by the Host
	  uint32_t host_crc_not_swapped = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;
	  //uint32_t swapped_crc = swap_byte_order(host_crc_not_swapped);

	  uint8_t ret_crc_u8 = bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc_not_swapped);
	  //uint8_t ret_crc_swapped_u8 = bootloader_verify_crc_swapped(&bl_rx_buffer[0],command_packet_len-4,swapped_crc);



    if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc_not_swapped))
    {
    	if(DEBUG_SERIAL_ENABLED_B)
    	{
    		printmsg("BL_DEBUG_MSG:checksum success !!\n");
    	}
        // checksum is correct..
        bootloader_send_ack(bl_rx_buffer[0],1);
        bl_version=get_bootloader_version();
        if(DEBUG_SERIAL_ENABLED_B)
        {
        	printmsg("BL_DEBUG_MSG:BL_VER : %d %#x\n",bl_version,bl_version);
        }
        bootloader_uart_write_data(&bl_version,1);

    }
    else
    {
    	if(DEBUG_SERIAL_ENABLED_B)
    	{
    		printmsg("BL_DEBUG_MSG:checksum fail !!\n");
    	}
        //checksum is wrong send nack
        bootloader_send_nack();
    }


}

/*Helper function to handle BL_GET_HELP command
 * Bootloader sends out All supported Command codes
 */
void bootloader_handle_gethelp_cmd(uint8_t *pBuffer)
{
	if(DEBUG_SERIAL_ENABLED_B)
	{
		printmsg("BL_DEBUG_MSG:bootloader_handle_gethelp_cmd\n");
	}

	//Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
		if(DEBUG_SERIAL_ENABLED_B)
		{
			printmsg("BL_DEBUG_MSG:checksum success !!\n");
		}
        bootloader_send_ack(pBuffer[0],sizeof(supported_commands));
        bootloader_uart_write_data(supported_commands,sizeof(supported_commands) );

	}else
	{
		if(DEBUG_SERIAL_ENABLED_B)
		{
			printmsg("BL_DEBUG_MSG:checksum fail !!\n");
		}
        bootloader_send_nack();
	}

}

/*Helper function to handle BL_GET_CID command */
void bootloader_handle_getcid_cmd(uint8_t *pBuffer)
{
	uint16_t bl_cid_num = 0;
	if(DEBUG_SERIAL_ENABLED_B)
	{
		printmsg("BL_DEBUG_MSG:bootloader_handle_getcid_cmd\n");
	}

    //Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
		if(DEBUG_SERIAL_ENABLED_B)
		{
			printmsg("BL_DEBUG_MSG:checksum success !!\n");
		}
        bootloader_send_ack(pBuffer[0],2);
        bl_cid_num = get_mcu_chip_id();
        if(DEBUG_SERIAL_ENABLED_B)
        {
        	printmsg("BL_DEBUG_MSG:MCU id : %d %#x !!\n",bl_cid_num, bl_cid_num);
        }
        bootloader_uart_write_data((uint8_t *)&bl_cid_num,2);

	}else
	{
        printmsg("BL_DEBUG_MSG:checksum fail !!\n");
        bootloader_send_nack();
	}


}

/*Helper function to handle BL_GET_RDP_STATUS command */
void bootloader_handle_getrdp_cmd(uint8_t *pBuffer)
{
    uint8_t rdp_level = 0x00;
    if(DEBUG_SERIAL_ENABLED_B)
    {
    	printmsg("BL_DEBUG_MSG:bootloader_handle_getrdp_cmd\n");
    }
    //Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
	    if(DEBUG_SERIAL_ENABLED_B)
	    {
	    	printmsg("BL_DEBUG_MSG:checksum success !!\n");
	    }
        bootloader_send_ack(pBuffer[0],1);
        rdp_level = get_flash_rdp_level();
        if(DEBUG_SERIAL_ENABLED_B)
        {
        	printmsg("BL_DEBUG_MSG:RDP level: %d %#x\n",rdp_level,rdp_level);
        }
        bootloader_uart_write_data(&rdp_level,1);

	}else
	{
		if(DEBUG_SERIAL_ENABLED_B)
		{
			printmsg("BL_DEBUG_MSG:checksum fail !!\n");
		}
        bootloader_send_nack();
	}


}

/*Helper function to handle BL_GO_TO_ADDR command */
void bootloader_handle_go_cmd(uint8_t *pBuffer)
{
    uint32_t go_address=0;
    uint8_t addr_valid = ADDR_VALID;
    uint8_t addr_invalid = ADDR_INVALID;
    if(DEBUG_SERIAL_ENABLED_B)
    {
    	printmsg("BL_DEBUG_MSG:bootloader_handle_go_cmd\n");
    }

    //Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
		if(DEBUG_SERIAL_ENABLED_B)
		{
			printmsg("BL_DEBUG_MSG:checksum success !!\n");
		}

        bootloader_send_ack(pBuffer[0],1);

        //extract the go address
        go_address = *((uint32_t *)&pBuffer[2] );
        if(DEBUG_SERIAL_ENABLED_B)
        {
        	printmsg("BL_DEBUG_MSG:GO addr: %#x\n",go_address);
        }

        if( verify_address(go_address) == ADDR_VALID )
        {
            //tell host that address is fine
            bootloader_uart_write_data(&addr_valid,1);

            /*jump to "go" address.
            we dont care what is being done there.
            host must ensure that valid code is present over there
            Its not the duty of bootloader. so just trust and jump */

            /* Not doing the below line will result in hardfault exception for ARM cortex M */
            //watch : https://www.youtube.com/watch?v=VX_12SjnNhY

            go_address+=1; //make T bit =1

            void (*lets_jump)(void) = (void *)go_address;

            if(DEBUG_SERIAL_ENABLED_B)
            {
            	printmsg("BL_DEBUG_MSG: jumping to go address! \n");
            }
            lets_jump();

		}else
		{
			if(DEBUG_SERIAL_ENABLED_B)
			{
				printmsg("BL_DEBUG_MSG:GO addr invalid ! \n");
			}
				//tell host that address is invalid
            bootloader_uart_write_data(&addr_invalid,1);
		}

	}
	else
	{
		if(DEBUG_SERIAL_ENABLED_B)
		{
			printmsg("BL_DEBUG_MSG:checksum fail !!\n");
		}
		bootloader_send_nack();
	}


}

/*Helper function to handle BL_FLASH_ERASE command */
void bootloader_handle_flash_erase_cmd(uint8_t *pBuffer)
{
    uint8_t erase_status = 0x00;
    if(DEBUG_SERIAL_ENABLED_B)
    {
    	printmsg("BL_DEBUG_MSG:bootloader_handle_flash_erase_cmd\n");
    }
    //Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
		if(DEBUG_SERIAL_ENABLED_B)
		{
			printmsg("BL_DEBUG_MSG:checksum success !!\n");
		}
        bootloader_send_ack(pBuffer[0],1);
        if(DEBUG_SERIAL_ENABLED_B)
        {
        	printmsg("BL_DEBUG_MSG:initial_sector : %d  no_ofsectors: %d\n",pBuffer[2],pBuffer[3]);
        }

        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin,1);
        erase_status = execute_flash_erase(pBuffer[2] , pBuffer[3]);
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin,0);

        if(DEBUG_SERIAL_ENABLED_B)
        {
        	printmsg("BL_DEBUG_MSG: flash erase status: %#x\n",erase_status);
        }

        bootloader_uart_write_data(&erase_status,1);

	}else
	{
		if(DEBUG_SERIAL_ENABLED_B)
		{
			printmsg("BL_DEBUG_MSG:checksum fail !!\n");
		}
		bootloader_send_nack();
	}
}

/*Helper function to handle BL_MEM_WRITE command */
void bootloader_handle_mem_write_cmd(uint8_t *pBuffer)
{
	uint8_t addr_valid = ADDR_VALID;
	uint8_t write_status = 0x00;
	uint8_t chksum =0, len=0;
	len = pBuffer[0];
	uint8_t payload_len = pBuffer[6];

	uint32_t mem_address = *((uint32_t *) ( &pBuffer[2]) );

	chksum = pBuffer[len];
	if(DEBUG_SERIAL_ENABLED_B)
	{
		printmsg("BL_DEBUG_MSG:bootloader_handle_mem_write_cmd\n");
	}

    //Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;


	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
		if(DEBUG_SERIAL_ENABLED_B)
		{
			printmsg("BL_DEBUG_MSG:checksum success !!\n");
		}

        bootloader_send_ack(pBuffer[0],1);

        if(DEBUG_SERIAL_ENABLED_B)
        {
        	printmsg("BL_DEBUG_MSG: mem write address : %#x\n",mem_address);
        }

		if( verify_address(mem_address) == ADDR_VALID )
		{

			if(DEBUG_SERIAL_ENABLED_B)
			{
				printmsg("BL_DEBUG_MSG: valid mem write address\n");
			}

            //glow the led to indicate bootloader is currently writing to memory
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);

            //execute mem write
            write_status = execute_mem_write(&pBuffer[7],mem_address, payload_len);

            //turn off the led to indicate memory write is over
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

            //inform host about the status
            bootloader_uart_write_data(&write_status,1);

		}
		else
		{
			if(DEBUG_SERIAL_ENABLED_B)
			{
				printmsg("BL_DEBUG_MSG: invalid mem write address\n");
			}
            write_status = ADDR_INVALID;
            //inform host that address is invalid
            bootloader_uart_write_data(&write_status,1);
		}


	}
	else
	{
		if(DEBUG_SERIAL_ENABLED_B)
		{
			printmsg("BL_DEBUG_MSG:checksum fail !!\n");
		}
        bootloader_send_nack();
	}

}

/*Helper function to handle BL_EN_RW_PROTECT  command */
void bootloader_handle_en_rw_protect(uint8_t *pBuffer)
{
    uint8_t status = 0x00;
    if(DEBUG_SERIAL_ENABLED_B)
    {
    	printmsg("BL_DEBUG_MSG:bootloader_handle_endis_rw_protect\n");
    }

    //Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
		if(DEBUG_SERIAL_ENABLED_B)
		{
			printmsg("BL_DEBUG_MSG:checksum success !!\n");
		}
        bootloader_send_ack(pBuffer[0],1);

        status = configure_flash_sector_rw_protection(pBuffer[2] , pBuffer[3],0);
        if(DEBUG_SERIAL_ENABLED_B)
        {
        	printmsg("BL_DEBUG_MSG: flash erase status: %#x\n",status);
        }

        bootloader_uart_write_data(&status,1);

	}
	else
	{
		if(DEBUG_SERIAL_ENABLED_B)
		{
			printmsg("BL_DEBUG_MSG:checksum fail !!\n");
		}
        bootloader_send_nack();
	}


}


/*Helper function to handle BL_EN_RW_PROTECT  command */
void bootloader_handle_dis_rw_protect(uint8_t *pBuffer)
{
    uint8_t status = 0x00;
    if(DEBUG_SERIAL_ENABLED_B)
    {
    	printmsg("BL_DEBUG_MSG:bootloader_handle_dis_rw_protect\n");
    }

    //Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
		if(DEBUG_SERIAL_ENABLED_B)
		{
			printmsg("BL_DEBUG_MSG:checksum success !!\n");
		}

		bootloader_send_ack(pBuffer[0],1);
        status = configure_flash_sector_rw_protection(0,0,1);
        if(DEBUG_SERIAL_ENABLED_B)
        {
        	printmsg("BL_DEBUG_MSG: flash erase status: %#x\n",status);
        }

        bootloader_uart_write_data(&status,1);

	}
	else
	{
		if(DEBUG_SERIAL_ENABLED_B)
		{
			printmsg("BL_DEBUG_MSG:checksum fail !!\n");
		}
		bootloader_send_nack();
	}


}

/*Helper function to handle BL_MEM_READ command */
void bootloader_handle_mem_read (uint8_t *pBuffer)
{


}

/*Helper function to handle _BL_READ_SECTOR_P_STATUS command */
void bootloader_handle_read_sector_protection_status(uint8_t *pBuffer)
{
	 uint16_t status;
	 if(DEBUG_SERIAL_ENABLED_B)
	 {
		 printmsg("BL_DEBUG_MSG:bootloader_handle_read_sector_protection_status\n");
	 }

    //Total length of the command packet
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//extract the CRC32 sent by the Host
	uint32_t host_crc = *((uint32_t * ) (bl_rx_buffer+command_packet_len - 4) ) ;

	if (! bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
		if(DEBUG_SERIAL_ENABLED_B)
		{
			printmsg("BL_DEBUG_MSG:checksum success !!\n");
		}
        bootloader_send_ack(pBuffer[0],2);
        status=read_OB_rw_protection_status();
        if(DEBUG_SERIAL_ENABLED_B)
        {
        	printmsg("BL_DEBUG_MSG: nWRP status: %#x\n",status);
        }
        bootloader_uart_write_data((uint8_t*)&status,2);

	}
	else
	{
		if(DEBUG_SERIAL_ENABLED_B)
		{
			printmsg("BL_DEBUG_MSG:checksum fail !!\n");
		}
        bootloader_send_nack();
	}

}

/*Helper function to handle BL_OTP_READ command */
void bootloader_handle_read_otp(uint8_t *pBuffer)
{


}

/*This function sends ACK if CRC matches along with "len to follow"*/
void bootloader_send_ack(uint8_t command_code, uint8_t follow_len)
{
	 //here we send 2 byte.. first byte is ack and the second byte is len value
	uint8_t ack_buf[2];
	ack_buf[0] = BL_ACK;
	ack_buf[1] = follow_len;
	HAL_UART_Transmit(D_UART,ack_buf,2,HAL_MAX_DELAY);

}

/*This function sends NACK */
void bootloader_send_nack(void)
{
	uint8_t nack = BL_NACK;
	HAL_UART_Transmit(D_UART,&nack,1,HAL_MAX_DELAY);
}

//This verifies the CRC of the given buffer in pData .
uint8_t bootloader_verify_crc (uint8_t *pData, uint32_t len, uint32_t crc_host)
{
    uint32_t uwCRCValue;
    //uint32_t test = 0xf;
    CRC->CR |= CRC_CR_RESET;

    for (uint32_t i=0 ; i < len ; i++)
	{
        uint32_t i_data = pData[i];
        uint32_t i_data_test = (uint32_t)pData[i];
        uwCRCValue = HAL_CRC_Accumulate(&hcrc, &i_data, 1);
        //test = HAL_CRC_Accumulate(&hcrc, &i_data_test, 1);
	}

	 /* Reset CRC Calculation Unit */
  __HAL_CRC_DR_RESET(&hcrc);

	if( uwCRCValue == crc_host)
	{
		return VERIFY_CRC_SUCCESS;
	}

	return VERIFY_CRC_FAIL;
}

uint8_t bootloader_verify_crc_swapped(uint8_t *pData, uint32_t len, uint32_t crc_host)
{
    uint32_t uwCRCValue;
    //uint32_t test = 0xf;
    CRC->CR |= CRC_CR_RESET;
    for (uint32_t i=len ; i > 0 ; i--)
	{
        uint32_t i_data = pData[i-1];
        uint32_t i_data_test = (uint32_t)pData[i-1];
        uwCRCValue = HAL_CRC_Accumulate(&hcrc, &i_data, 1);
        //test = HAL_CRC_Accumulate(&hcrc, &i_data_test, 1);
	}

	 /* Reset CRC Calculation Unit */
  __HAL_CRC_DR_RESET(&hcrc);

	if( uwCRCValue == crc_host)
	{
		return VERIFY_CRC_SUCCESS;
	}

	return VERIFY_CRC_FAIL;
}


void bootloader_uart_read_data(void)
{
	uint8_t rcv_len = 0;
	HAL_StatusTypeDef uart_t;

	//HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
	while(1)
	{
		memset(bl_rx_buffer,0,200);
		//here we will read and decode the commands coming from host
		//first read only one byte from the host , which is the "length" field of the command packet
		uart_t = HAL_UART_Receive(D_UART,bl_rx_buffer,1,HAL_MAX_DELAY);
		rcv_len= bl_rx_buffer[0];
		uart_t = HAL_UART_Receive(D_UART,&bl_rx_buffer[1],rcv_len,HAL_MAX_DELAY);
		switch(bl_rx_buffer[1])
		{
            case BL_GET_VER:
                bootloader_handle_getver_cmd(bl_rx_buffer);
                break;
            case BL_GET_HELP:
                bootloader_handle_gethelp_cmd(bl_rx_buffer);
                break;
            case BL_GET_CID:
                bootloader_handle_getcid_cmd(bl_rx_buffer);
                break;
            case BL_GET_RDP_STATUS:
                bootloader_handle_getrdp_cmd(bl_rx_buffer);
                break;
            case BL_GO_TO_ADDR:
                bootloader_handle_go_cmd(bl_rx_buffer);
                break;
            case BL_FLASH_ERASE:
                bootloader_handle_flash_erase_cmd(bl_rx_buffer);
                break;
            case BL_MEM_WRITE:
                bootloader_handle_mem_write_cmd(bl_rx_buffer);
                break;
            case BL_EN_RW_PROTECT:
                bootloader_handle_en_rw_protect(bl_rx_buffer);
                break;
            case BL_MEM_READ:
                bootloader_handle_mem_read(bl_rx_buffer);
                break;
            case BL_READ_SECTOR_P_STATUS:
                bootloader_handle_read_sector_protection_status(bl_rx_buffer);
                break;
            case BL_OTP_READ:
                bootloader_handle_read_otp(bl_rx_buffer);
                break;
			case BL_DIS_R_W_PROTECT:
                bootloader_handle_dis_rw_protect(bl_rx_buffer);
                break;
             default:
                printmsg("BL_DEBUG_MSG:Invalid command code received from host \n");
                break;


		}

	}

}
void bootloader_jump_to_user_app(void)
{
	void (*app_reset_handler)(void);
	printmsg("BL_DEBUG_MSG:bootloader_jump_to_user_app\r\n");
	uint32_t msp_value = *(volatile uint32_t *)FLASH_SECTOR2_BASE_ADDRESS;
	printmsg("BL_DEBUG_MSG:msp_value : %#x\r\n",msp_value);

	__set_MSP(msp_value);

	// Now fetch reset handler address of user app
	uint32_t resethandler_address = *(volatile uint32_t *)(FLASH_SECTOR2_BASE_ADDRESS+4);
	app_reset_handler = (void*)resethandler_address;
	printmsg("BL_DEBUG_MSG: app reset handler addr: %#x\r\n",app_reset_handler);
	app_reset_handler();
}


/* This function writes data in to C_UART */
void bootloader_uart_write_data(uint8_t *pBuffer,uint32_t len)
{
    /*you can replace the below ST's USART driver API call with your MCUs driver API call */
	HAL_UART_Transmit(D_UART,pBuffer,len,HAL_MAX_DELAY);

}


//Just returns the macro value .
uint8_t get_bootloader_version(void)
{
  return (uint8_t)BL_VERSION;
}

//Read the chip identifier or device Identifier
uint16_t get_mcu_chip_id(void)
{
/*
	The STM32F446xx MCUs integrate an MCU ID code. This ID identifies the ST MCU partnumber
	and the die revision. It is part of the DBG_MCU component and is mapped on the
	external PPB bus (see Section 33.16 on page 1304). This code is accessible using the
	JTAG debug pCat.2ort (4 to 5 pins) or the SW debug port (two pins) or by the user software.
	It is even accessible while the MCU is under system reset. */
	uint16_t cid;
	cid = (uint16_t)(DBGMCU->IDCODE) & 0x0FFF;
	return  cid;

}


/*This function reads the RDP ( Read protection option byte) value
 *For more info refer "Table 9. Description of the option bytes" in stm32f446xx RM
 */
uint8_t get_flash_rdp_level(void)
{

	uint8_t rdp_status=0;
#if 0
	FLASH_OBProgramInitTypeDef  ob_handle;
	HAL_FLASHEx_OBGetConfig(&ob_handle);
	rdp_status = (uint8_t)ob_handle.RDPLevel;
#else

	 volatile uint32_t *pOB_addr = (uint32_t*) 0x1FFFF800;
	 rdp_status =  (uint8_t)(*pOB_addr & 0xFF) ;
#endif

	return rdp_status;

}



//verify the address sent by the host .
uint8_t verify_address(uint32_t go_address)
{
	//so, what are the valid addresses to which we can jump ?
	//can we jump to system memory ? yes
	//can we jump to sram1 memory ?  yes
	//can we jump to sram2 memory ? yes
	//can we jump to backup sram memory ? yes
	//can we jump to peripheral memory ? its possible , but dont allow. so no
	//can we jump to external memory ? yes.

//incomplete -poorly written .. optimize it
	if ( go_address >= SRAM_BASE && go_address <= SRAM1_END)
	{
		return ADDR_VALID;
	}
	else if ( go_address >= FLASH_BASE && go_address <= 0x0807FFFFU)
	{
		return ADDR_VALID;
	}
	else
	{
		return ADDR_INVALID;
	}
}

 uint8_t execute_flash_erase(uint8_t page_number , uint8_t number_of_page)
{
    //we have totally 256 pages in STM32F303RE mcu .. page[0 to 255]
	//number_of_page has to be in the range of 0 to 255
	// if page_number = 0xff , that means mass erase !
	//Code needs to modified if your MCU supports more flash pages
	 /*
	  * To erase a page, the procedure below should be followed:
		1. Check that no Flash memory operation is ongoing by checking the BSY bit in the
		FLASH_CR register.
		2. Set the PER bit in the FLASH_CR register
		3. Program the FLASH_AR register to select a page to erase
		4. Set the STRT bit in the FLASH_CR register (see below note)
		5. Wait for the BSY bit to be reset
		6. Check the EOP flag in the FLASH_SR register (it is set when the erase operation has
		succeeded), and then clear it by software.
		7. Clear the EOP flag.
		#define FLASH_TYPEERASE_PAGES     (0x00U)  !<Pages erase only
#define FLASH_TYPEERASE_MASSERASE (0x01U)  !<Flash mass erase activation

typedef struct
{
  uint32_t TypeErase;   /!< TypeErase: Mass erase or page erase.
                             This parameter can be a value of @ref FLASHEx_Type_Erase /

  uint32_t PageAddress; /!< PageAdress: Initial FLASH page address to erase when mass erase is disabled
                             This parameter must be a number between Min_Data = FLASH_BASE and Max_Data = FLASH_BANK1_END /

  uint32_t NbPages;     /!< NbPages: Number of pagess to be erased.
                             This parameter must be a value between Min_Data = 1 and Max_Data = (max number of 1pages - value of initial page)/

} FLASH_EraseInitTypeDef;
	  *Page 16 is where app is located
	  *
	  */
	FLASH_EraseInitTypeDef flashErase_handle;
	uint32_t pageError;
	HAL_StatusTypeDef status;


	if( number_of_page > 255 )
		return INVALID_SECTOR;

	if( (page_number == 0xff ) || (page_number <= 255) )
	{
		if(page_number == (uint8_t) 0xff)
		{
			flashErase_handle.TypeErase = FLASH_TYPEERASE_MASSERASE;
			flashErase_handle.PageAddress = FLASH_BASE;
			flashErase_handle.NbPages = 255;
		}
		else
		{
		    // Here we are just calculating how many sectors needs to erased
			uint8_t remanining_pages = 256 - page_number;
			uint32_t page_address = FLASH_BASE+(FLASH_PAGE_SIZE*page_number);
            if( number_of_page > remanining_pages)
            {
            	number_of_page = remanining_pages;
            }
            else if(number_of_page < 1)
            {
            	return INVALID_SECTOR;
            }
            else
            {
            	//do nothing
            }
			flashErase_handle.TypeErase = FLASH_TYPEERASE_PAGES;
			flashErase_handle.PageAddress = page_address; // this is the initial sector
			flashErase_handle.NbPages = number_of_page;
		}
		//flashErase_handle.Banks = FLASH_BANK_1;

		//Get access to touch the flash registers
		HAL_FLASH_Unlock();
		status = (uint8_t) HAL_FLASHEx_Erase(&flashErase_handle, &pageError);
		HAL_FLASH_Lock();

		return status;
	}



	return INVALID_SECTOR;
}

/*This function writes the contents of pBuffer to  "mem_address" byte by byte */
//Note1 : Currently this function supports writing to Flash only .
//Note2 : This functions does not check whether "mem_address" is a valid address of the flash range.
uint8_t execute_mem_write(uint8_t *pBuffer, uint32_t mem_address, uint32_t len)
{
    uint8_t status=HAL_OK;
    uint8_t count = 0;
    uint16_t packed_data_u16 = 0;
    uint64_t packed_data_u64 =0;
    /*
     *
     *
     *  1. Check that no main Flash memory operation is ongoing by checking the BSY bit in the
			FLASH_SR register.
		2. Set the PG bit in the FLASH_CR register.
		3. Perform the data write (half-word) at the desired address.
		4. Wait until the BSY bit is reset in the FLASH_SR register.
		5. Check the EOP flag in the FLASH_SR register (it is set when the programming
			operation has succeeded), and then clear it by software.
     */

    if(IS_FLASH_PROGRAM_ADDRESS(mem_address))
    {
		//We have to unlock flash module to get control of registers
		HAL_FLASH_Unlock();

		for(uint32_t i = 0 ; i <len ; i=i+2)
		{
			count = 0;
			packed_data_u16 = 0;
			packed_data_u64 = 0;
			//Here we program the flash byte by byte but now we need two byte uint16
			//status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,mem_address+i,pBuffer[i] );
			for(count = 0;count<2;count++)
			{
				if(count == 0)
				{
					packed_data_u16 ^= pBuffer[i];

				}
				else if(count == 1 && i+1 < len)
				{
					packed_data_u16 ^= (uint16_t)(pBuffer[i+1] << 8);
				}
				else
				{
					packed_data_u16 ^= 0xFF00u;
				}
			}
			// HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data)
			packed_data_u64 = (uint64_t)packed_data_u16;
			status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,mem_address+i,packed_data_u64);
		}

		HAL_FLASH_Lock();
	}
    else
    {
    	status = HAL_ERROR;
    }

    return status;
}

uint32_t page_details_to_2pages(uint8_t page_details)
{
	uint32_t return_register = 0;
	if(page_details <= 15)
	{
		//bits 0-7 reflect 2 pages WRP0 WRP0: Write-protects pages 0 to 15
		if(page_details < 2)
		{
			return_register =1;
		}
		else if(page_details < 4)
		{
			return_register =2;
		}
		else if(page_details < 6)
		{
			return_register =4;
		}
		else if(page_details < 8)
		{
			return_register =8;
		}
		else if(page_details < 10)
		{
			return_register =16;
		}
		else if(page_details < 12)
		{
			return_register =32;
		}
		else if(page_details < 14)
		{
			return_register =64;
		}
		else
		{
			return_register =128;
		}
	}
	else if(page_details <= 31)
	{
		if(page_details < 18)
		{
			return_register =256;
		}
		else if(page_details < 20)
		{
			return_register =512;
		}
		else if(page_details < 22)
		{
			return_register =1024;
		}
		else if(page_details < 24)
		{
			return_register =2048;
		}
		else if(page_details < 26)
		{
			return_register =4096;
		}
		else if(page_details < 28)
		{
			return_register =8192;
		}
		else if(page_details < 30)
		{
			return_register =16384;
		}
		else
		{
			return_register =32768;
		}
	}
	else if(page_details <= 47)
	{
		if(page_details < 34)
		{
			return_register =65536;
		}
		else if(page_details < 36)
		{
			return_register =131072;
		}
		else if(page_details < 38)
		{
			return_register =262144;
		}
		else if(page_details < 40)
		{
			return_register =524288;
		}
		else if(page_details < 42)
		{
			return_register =1048576;
		}
		else if(page_details < 44)
		{
			return_register =2097152;
		}
		else if(page_details < 46)
		{
			return_register =4194304;
		}
		else
		{
			return_register =8388608;
		}
	}
	else
	{
		return_register =0xFF000000;
	}
	return return_register;
}

HAL_StatusTypeDef set_flash_rdp_level(uint8_t level)
{
	HAL_StatusTypeDef rdp_status = HAL_OK;

	/* Check the parameters */
	assert_param(IS_OB_RDP_LEVEL(level));
	HAL_FLASH_OB_Unlock();
	while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
	FLASH_OBProgramInitTypeDef  ob_handle;
	HAL_FLASHEx_OBGetConfig(&ob_handle);


	/* Enable the Option Bytes Programming operation */
	SET_BIT(FLASH->CR, FLASH_CR_OPTPG);

	WRITE_REG(OB->RDP, level);

	/* Wait for last operation to be completed */
	while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

	/* if the program operation is completed, disable the OPTPG Bit */
	CLEAR_BIT(FLASH->CR, FLASH_CR_OPTPG);

	while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
	HAL_FLASH_OB_Lock();

	rdp_status = ob_handle.RDPLevel;

	return rdp_status;

}

HAL_StatusTypeDef disable_write_flash_prot(uint8_t page_number)
{
    uint16_t WRP0_Data = 0xFFFFU;
    uint16_t WRP1_Data = 0xFFFFU;
    uint16_t WRP2_Data = 0xFFFFU;
    uint16_t WRP3_Data = 0xFFFFU;
    uint32_t flash_reg = (uint32_t)(READ_REG(FLASH->WRPR));
    uint32_t WriteProtectPage = 1;
    HAL_StatusTypeDef status = HAL_OK;
    status = HAL_FLASH_OB_Unlock();
    	WriteProtectPage = page_details_to_2pages(page_number);
    	assert_param(IS_OB_WRP(WriteProtectPage));
    	WriteProtectPage = (flash_reg | WriteProtectPage);
		#if defined(OB_WRP_PAGES0TO15MASK)
		  WRP0_Data = (uint16_t)(WriteProtectPage & OB_WRP_PAGES0TO15MASK);
		#endif /* OB_WRP_PAGES0TO31MASK */

		#if defined(OB_WRP_PAGES16TO31MASK)
		  WRP1_Data = (uint16_t)((WriteProtectPage & OB_WRP_PAGES16TO31MASK) >> 8U);
		#endif /* OB_WRP_PAGES32TO63MASK */

		#if defined(OB_WRP_PAGES32TO47MASK)
		  WRP2_Data = (uint16_t)((WriteProtectPage & OB_WRP_PAGES32TO47MASK) >> 16U);
		#endif /* OB_WRP_PAGES32TO47MASK */

		#if defined(OB_WRP_PAGES48TO127MASK)
		  WRP3_Data = (uint16_t)((WriteProtectPage & OB_WRP_PAGES48TO127MASK) >> 24U);
		#elif defined(OB_WRP_PAGES48TO255MASK)
		  WRP3_Data = (uint16_t)((WriteProtectPage & OB_WRP_PAGES48TO255MASK) >> 24U);
		#endif

	if (status == HAL_OK)
	{
		SET_BIT(FLASH->CR, FLASH_CR_OPTPG);
		#if defined(OB_WRP0_WRP0)
			  if(WRP0_Data != 0xFFU)
			  {
				OB->WRP0 |= WRP0_Data;

				/* Wait for last operation to be completed */
				status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
			  }
		#endif /* OB_WRP0_WRP0 */

		#if defined(OB_WRP1_WRP1)
			  if((status == HAL_OK) && (WRP1_Data != 0xFFU))
			  {
				OB->WRP1 |= WRP1_Data;

				/* Wait for last operation to be completed */
				while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
			  }
		#endif /* OB_WRP1_WRP1 */

		#if defined(OB_WRP2_WRP2)
			  if((status == HAL_OK) && (WRP2_Data != 0xFFU))
			  {
				OB->WRP2 |= WRP2_Data;

				/* Wait for last operation to be completed */
				while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
			  }
		#endif /* OB_WRP2_WRP2 */

		#if defined(OB_WRP3_WRP3)
			  if((status == HAL_OK) && (WRP3_Data != 0xFFU))
			  {
				OB->WRP3 |= WRP3_Data;

				/* Wait for last operation to be completed */
				while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
			  }
		#endif /* OB_WRP3_WRP3 */
		CLEAR_BIT(FLASH->CR, FLASH_CR_OPTPG);
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);
	}
	else
	{

	}

    HAL_FLASH_OB_Lock();
    return status;
}



/*
Modifying user option bytes
To modify the user option value, follow the sequence below:
1. Check that no Flash memory operation is ongoing by checking the BSY bit in the
FLASH_SR register
2. Write the desired option value in the FLASH_OPTCR register.
3. Set the option start bit (OPTSTRT) in the FLASH_OPTCR register
4. Wait for the BSY bit to be cleared.
*/
uint8_t configure_flash_sector_rw_protection(uint8_t page_details, uint8_t protection_mode, uint8_t disable)
{
    //First configure the protection mode
    //protection_mode =1 , means write protect of the user flash sectors
    //protection_mode =2, means read/write protect of the user flash sectors
    //According to RM of stm32f446xx TABLE 9, We have to modify the address 0x1FFF C008 bit 15(SPRMOD)
/*
	Option byte programming
	The option bytes are programmed differently from normal user addresses. The number of
	option bytes is limited to 8 (4 for write protection, 1 for readout protection, 1 for hardware
	configuration, and 2 for data storage). After unlocking the FPEC, the user has to authorize
	the programming of the option bytes by writing the same set of KEYS (KEY1 and KEY2) to
	the FLASH_OPTKEYR register (refer to Unlocking the Flash memory for key values). Then,
	the OPTWRE bit in the FLASH_CR register will be set by hardware and the user has to set
	the OPTPG bit in the FLASH_CR register and perform a half-word write operation at the
	desired Flash address.

	The value of the addressed option byte is first read to check it is really erased. If not, the
	program operation is skipped and a warning is issued by the WRPRTERR bit in the
	FLASH_SR register. The end of the program operation is indicated by the EOP bit in the
	FLASH_SR register.
	The LSB value is automatically complemented into the MSB before the programming
	operation starts. This guarantees that the option byte and its complement are always
	correct.
	The sequence is as follows:
	• Check that no Flash memory operation is ongoing by checking the BSY bit in the
	FLASH_SR register.
	• Unlock the OPTWRE bit in the FLASH_CR register.
	• Set the OPTPG bit in the FLASH_CR register
	• Write the data (half-word) to the desired address
	• Wait for the BSY bit to be reset.
	• Read the programmed value and verify.
	When the Flash memory read protection option is changed from protected to unprotected, a
	Mass Erase of the main Flash memory is performed before reprogramming the read
	protection option. If the user wants to change an option other than the read protection
	option, then the mass erase is not performed. The erased state of the read protection option
	byte protects the Flash memory.
*/
	 //Flash control register (FLASH_CR) 0x4002 2010 - 0x4002 2013 4 FLASH_CR
    volatile uint32_t *pFLASH_CR = (uint32_t*) 0x40022010;
    uint8_t read_protection = 0;
    //AA - Level 0, XX - Level 1, CC = Level 2
    read_protection = get_flash_rdp_level();
    HAL_StatusTypeDef rdp_return = HAL_OK;
    HAL_StatusTypeDef wrp_return = HAL_OK;
    uint32_t page_value = 0;



    if(disable == 0)
    {
    	page_value = page_details_to_2pages(page_details);
    }


	  if(disable)
		{

			//disable all r/w protection on sectors
		  /*
		   * 	Changing read protection level
				It is easy to move from level 0 to level 1 by changing the value of the RDP byte to any value
				(except 0xCC). By programming the 0xCC value in the RDP byte, it is possible to go to level
				2 either directly from level 0 or from level 1. On the contrary, the change to level 0 (no
				protection) is not possible without a main Flash memory Mass Erase operation. This Mass
				Erase is generated as soon as 0xAA is programmed in the RDP byte.
				Note: When the Mass Erase command is used, the backup registers (RTC_BKPxR in the RTC)
				are also reset.
				To validate the protection level change, the option bytes must be reloaded through the
				OBL_LAUNCH bit in Flash control register
		   */

		  /* Only allow */
		  	if(read_protection != 0xAA)
		  	{
		  		//Level 0 no protection
		  		//no change needed

		  		rdp_return = set_flash_rdp_level(OB_RDP_LEVEL_0);
		  	}


		  	for(uint8_t idx=0;idx<255;idx++)
		  	{
		  		wrp_return =  disable_write_flash_prot(idx);
		  	}
		  	//HAL_StatusTypeDef FLASH_OB_DisableWRP(uint32_t WriteProtectPage)
		  	//wrp_return = FLASH_OB_DisableWRP(page_value);
			//Option byte configuration unlock

			return 0;

		}

	   if(protection_mode == (uint8_t) 1)
    {
           //we are putting write protection on the sectors encoded in sector_details argument

			//Option byte configuration unlock
		  // wrp_return = FLASH_OB_EnableWRP(page_value);
    }
	else if (protection_mode == (uint8_t) 2)
    {
	  	//Option byte configuration unlock
		//not supported
    }

		return 0;
}

uint16_t read_OB_rw_protection_status(void)
{
    //This structure is given by ST Flash driver to hold the OB(Option Byte) contents .
	FLASH_OBProgramInitTypeDef OBInit;

	//First unlock the OB(Option Byte) memory access
	HAL_FLASH_OB_Unlock();
	//get the OB configuration details
	HAL_FLASHEx_OBGetConfig(&OBInit);
	//Lock back .
	HAL_FLASH_Lock();

	//We are just interested in r/w protection status of the PAGES.
	return (uint16_t)OBInit.WRPPage;

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
void assert_failed(char *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
