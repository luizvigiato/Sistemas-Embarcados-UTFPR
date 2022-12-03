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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "usbd_cdc_if.h"
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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct led_t_ {
	GPIO_TypeDef* port;
	uint16_t pin;
	int timeout;
}led_t;

led_t green_led;

void task_led(void *param){
	led_t *led = (led_t *)param;
	while(1){
		HAL_GPIO_TogglePin(led->port,led->pin);
		vTaskDelay(led->timeout);
	}
}

void task_usb(void *param){
	while(!usb_is_on()){
		vTaskDelay(100);
	}
	while(1){

		(void) CDC_Transmit_FS((uint8_t *) "Teste de USB 2\n\r",16);

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  green_led.timeout = 500;
  green_led.port = LED_GPIO_Port;
  green_led.pin = LED_Pin;
  xTaskCreate(task_led,"Tarefa Led",256, &green_led, 1, NULL);
  //xTaskCreate(task_usb,"Tarefa USB",256, &green_led, 5, NULL);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//uint8_t buffer[128];
//uint32_t len;
uint8_t read_usb_cdc(char *buffer, int buf_len, TickType_t timeout);
#define MAX_INPUT_LENGTH    64
#define MAX_OUTPUT_LENGTH   256
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  init_usb_rtos_obj();
  /* Infinite loop */
  uint8_t cRxedChar, buffer[32], cInputIndex = 0;
  BaseType_t xMoreDataToFollow = pdFALSE;

  static uint8_t pcOutputString[ MAX_OUTPUT_LENGTH ], pcInputString[ MAX_INPUT_LENGTH ];


  for(;;)
  {
	  /*volatile int8_t ret = USBD_Interface_fops_FS.Receive(buffer, &len);
	  if (len){
		  (void) CDC_Transmit_FS(buffer,(uint16_t) len);
	  }*/
	  //osDelay(250);
//	  char data;
//	  CDC_Receiveq_MS(&data,portMAX_DELAY);

	  /**
	   * Inicio para Controle de entrada USB de informacoes
	   */

		 /* This implementation reads a single character at a time.  Wait in the
		 Blocked state until a character is received. */
		(void)read_usb_cdc((char *) buffer, 32, portMAX_DELAY);
		cRxedChar = buffer[0];

		 if( cRxedChar == '\r' )
		 {
			 /* A newline character was received, so the input command string is
			 complete and can be processed.  Transmit a line separator, just to
			 make the output easier to read. */
			 CDC_Transmit_FS((uint8_t*)"\n\r", 2);

			 /* The command interpreter is called repeatedly until it returns
			 pdFALSE.  See the "Implementing a command" documentation for an
			 exaplanation of why this is. */
			 do
			 {
				 /* Write the output generated by the command interpreter to the
				 console. */
				 CDC_Transmit_FS((uint8_t*) pcOutputString,(uint16_t ) strlen( (const char *) pcOutputString ) );

			 } while( xMoreDataToFollow != pdFALSE );

			 /* All the strings generated by the input command have been sent.
			 Processing of the command is complete.  Clear the input string ready
			 to receive the next command. */
			 cInputIndex = 0;
			 memset( pcInputString, 0x00, MAX_INPUT_LENGTH );
			 CDC_Transmit_FS((uint8_t*) ">>", 2);
		 }
		 else
		 {
			 /* The if() clause performs the processing after a newline character
			 is received.  This else clause performs the processing if any other
			 character is received. */

			 if( cRxedChar == '\n' )
			 {
				 /* Ignore carriage returns. */
			 }
			 else if( cRxedChar == 0x7F )
			 {
				 /* Backspace was pressed.  Erase the last character in the input
				 buffer - if there are any. */
				 if( cInputIndex > 0 )
				 {
					 CDC_Transmit_FS( &cRxedChar, 1);
					 cInputIndex--;
					 pcInputString[ cInputIndex ] = '\0';
				 }
			 }
			 else if( cRxedChar == '\0' )
			 {
				 CDC_Transmit_FS((uint8_t*) "Welcome to FreeRTOS\n\r\n\r>>", strlen("Welcome to FreeRTOS\n\r\n\r>>"));
			 }
			 else if( cRxedChar == '\e' )
			 {
				 //repetir ultimo comando
			 }
			 else
			 {
				 /* A character was entered.  It was not a new line, backspace
				 or carriage return, so it is accepted as part of the input and
				 placed into the input buffer.  When a n is entered the complete
				 string will be passed to the command interpreter. */
				 if( cInputIndex < MAX_INPUT_LENGTH )
				 {
					 CDC_Transmit_FS( &cRxedChar, 1);
					 pcInputString[ cInputIndex ] = cRxedChar;
					 cInputIndex++;
				 }
			 }
		 }


//	  (void) CDC_Transmit_FS((uint8_t *) "\n\r",2);
//	  (void) CDC_Transmit_FS((uint8_t *) &data,1);
  }
  /* USER CODE END 5 */
}

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