/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v1.0_Cube
  * @brief          : Usb device for Virtual Com Port.
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
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */

#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include "message_buffer.h"
#include "FreeRTOS_CLI.h"
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
#define NewLine "# "
#define MAX_OUTPUT_LENGTH 512
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */
xSemaphoreHandle sem_usb_tx;
MessageBufferHandle_t msg_buf_rx;
MessageBufferHandle_t msg_buf_tx;
//MessageBufferHandle_t msg_buf_rx;
volatile uint32_t usb_on = 0;

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
void Print_Task(void *param);
void init_usb_rtos_obj(void){
	sem_usb_tx = xSemaphoreCreateBinary();
	msg_buf_rx = xMessageBufferCreate(768);
	msg_buf_tx = xMessageBufferCreate(768);
	/*
	 *
	 */
	xTaskCreate(Print_Task,"Impressao",512, NULL, 5, NULL);
	usb_on = 1;
}


uint32_t usb_is_on(){
	return usb_on;
}


BaseType_t CDC_Receiveq_FS(char *data, TickType_t timeout){
	return xMessageBufferReceive(msg_buf_tx,(void *) data, 128,portMAX_DELAY);
}

uint8_t read_usb_cdc(char *buffer, int buf_len, TickType_t timeout){
	return xMessageBufferReceive(msg_buf_rx, buffer, buf_len, timeout);
}

void queue_print(char *data,int size){
	CDC_Receiveq_FS(data,portMAX_DELAY);
//	xMessageBufferSend(msg_buf_tx,data,size,portMAX_DELAY);
}

void Print_Task(void * param){
	char buffer[768];
	uint8_t qtd=0;
	uint8_t pcInputString[64], pcIndexInput = 0,pcOutputString[512];
	uint8_t xRchar;

	BaseType_t xMoreDataToFollow;


	while(1){
		qtd = xMessageBufferReceive(msg_buf_rx,(void *) buffer,sizeof(buffer),portMAX_DELAY);
		xRchar = buffer[0];
		if(xRchar == '\r'){
			CDC_Transmit_FS((uint8_t *)"\n\r", 2);
			pcInputString[pcIndexInput] = '\0';
			do{
							 /* Send the command string to the command interpreter.  Any
							 output generated by the command interpreter will be placed in the
							 pcOutputString buffer. */
				xMoreDataToFollow = FreeRTOS_CLIProcessCommand
						   (
							   (char *)pcInputString,   /* The command string.*/
							   (char *)pcOutputString,  /* The output buffer. */
							   MAX_OUTPUT_LENGTH/* The size of the output buffer. */
						   );

							 /* Write the output generated by the command interpreter to the
							 console. */
				CDC_Transmit_FS((uint8_t *) pcOutputString, strlen((char *) pcOutputString ) );
			 } while( xMoreDataToFollow != pdFALSE );


			CDC_Transmit_FS((uint8_t *) "\n\r# ", 4);
//			CDC_Transmit_FS((uint8_t *)pcInputString, pcIndexInput);
			pcIndexInput = 0;
		} else {
			if( xRchar == '\0' ){
				CDC_Transmit_FS((uint8_t *)"Welcome to FreeRTOS\n\r", 21);
				CDC_Transmit_FS((uint8_t *)NewLine, 2);
				pcIndexInput = 0;
			} else if (xRchar == 0x7F ){
				/*
				 * Backspace was pressed.
				 */
				if(pcIndexInput > 0){
					CDC_Transmit_FS(&xRchar, 1);
					pcIndexInput--;
					pcInputString[pcIndexInput] = '\0';

				}
			} else if(xRchar == 0x1B){
				/*
				 * Entrou no seta pra cima
				 */
				CDC_Transmit_FS(pcInputString, strlen((const char*)pcInputString));
				pcIndexInput = strlen((const char*)pcInputString)+1;
				pcIndexInput--;
			} else if(pcIndexInput < 64){
				(void) qtd;
				CDC_Transmit_FS(&xRchar, qtd);
				pcInputString[pcIndexInput] = xRchar;
				pcIndexInput++;
			}
		}
	}
}

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  portBASE_TYPE yield = pdFALSE;
  char data = 0;
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:

    break;

    case CDC_GET_LINE_CODING:

    break;

    case CDC_SET_CONTROL_LINE_STATE:
  	  xMessageBufferSendFromISR(msg_buf_rx, &data, 1, &yield);
  	  portYIELD_FROM_ISR(yield);

    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
	portBASE_TYPE yield = pdFALSE;
	USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
	USBD_CDC_ReceivePacket(&hUsbDeviceFS);

	  xMessageBufferSendFromISR(msg_buf_rx, Buf, *Len, &yield);
	  portYIELD_FROM_ISR(yield);

	return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */

  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  xSemaphoreTake(sem_usb_tx, portMAX_DELAY);
  /* USER CODE END 7 */
  return result;
}

/**
  * @brief  CDC_TransmitCplt_FS
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 13 */
  portBASE_TYPE tmp = pdFALSE;
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  xSemaphoreGiveFromISR(sem_usb_tx, &tmp);
  portYIELD_FROM_ISR(&tmp);
  /* USER CODE END 13 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
