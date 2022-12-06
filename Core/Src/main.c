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
//#include "FreeRTOS.h"
#include "FreeRTOS_CLI.h"
#include "usbd_cdc_if.h"
#include "arm_const_structs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NewLine "# "
#define MAX_INPUT_LENGTH    64
#define MAX_OUTPUT_LENGTH   512
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;

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
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
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

uint16_t adcBuffer[256];

float ReIm[256*2];
float mod[256];

void task_led(void *param){
	led_t *led = (led_t *)param;
	while(1){
		HAL_GPIO_TogglePin(led->port,led->pin);
		vTaskDelay(led->timeout);
	}
}

void task_adc(void *param){
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuffer, 256);
	HAL_TIM_Base_Start(&htim2);



	while(1){
		int k = 0;
		for(int i = 0; i < 256; i++){
			ReIm[k] = (float) adcBuffer[i] * 0.0007326007;
			ReIm[k+1] = 0.0;
			k += 2;
		}

		arm_cfft_f32(&arm_cfft_sR_f32_len256,ReIm,0,1);
		arm_cmplx_mag_f32(ReIm,mod,256);
		arm_scale_f32(mod, 0.0078125, mod, 128); /* vertor, por quem quero multiplicar, vetor final, quantos pontos */

		volatile float fund_phase = atan2f(ReIm[3],ReIm[2])*180/M_PI;
		(void) fund_phase;
		vTaskDelay(5);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
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
  xTaskCreate(task_adc,"Tarefa ADC",256, &green_led, 2, NULL);
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 21000000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
uint32_t len;

uint8_t read_usb_cdc(char *buffer, int buf_len, TickType_t timeout);

static BaseType_t prvTaskStatsCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ){


        /* For simplicity, this function assumes the output buffer is large enough
        to hold all the text generated by executing the vTaskList() API function,
        so the xWriteBufferLen parameter is not used. */
        char *head = "Name\t\tState  Priority  Stack  Number\n\r";
        ( void ) xWriteBufferLen;

        /* pcWriteBuffer is used directly as the vTaskList() parameter, so the table
        generated by executing vTaskList() is written directly into the output
        buffer. */
        strcpy(pcWriteBuffer, head);
        vTaskList( pcWriteBuffer + strlen(head));

        /* The entire table was written directly to the output buffer.  Execution
        of this command is complete, so return pdFALSE. */
        return pdFALSE;
}

static BaseType_t prvTaskStatsTexto( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ){
	strcpy(pcWriteBuffer,(char*)"Este e um texto teste\r\n");
	return pdFALSE;
}

static const CLI_Command_Definition_t xTasksCommand =
{
    "tasks",
	"\r\ntasks:\r\n Lists all the installed tasks\r\n\r\n",
	prvTaskStatsCommand,
    0
};

static const CLI_Command_Definition_t xTasksTexto =
{
    "texto",
	"\r\ntexto:\r\n Print Text Teste\r\n\r\n",
	prvTaskStatsTexto,
    0
};

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

  static uint8_t pcOutputString[ MAX_OUTPUT_LENGTH ], pcInputString[ MAX_INPUT_LENGTH ];

  FreeRTOS_CLIRegisterCommand( &xTasksCommand );
  FreeRTOS_CLIRegisterCommand( &xTasksTexto );
  BaseType_t xMoreDataToFollow;
  uint8_t cRxedChar, buffer[256], cInputIndex = 0;
  /* Infinite loop */
  for(;;)
  {
	  /*volatile int8_t ret = USBD_Interface_fops_FS.Receive(buffer, &len);
	  if (len){
		  (void) CDC_Transmit_FS(buffer,(uint16_t) len);
	  }*/
	  //osDelay(250);
	  /*char data;
	  CDC_Receiveq_MS(&data,portMAX_DELAY);
	  (void) CDC_Transmit_FS((uint8_t *) "\n\r",2);
	  (void) CDC_Transmit_FS((uint8_t *) &data,1);*/

		(void)read_usb_cdc((char *)buffer, 32, portMAX_DELAY);
		cRxedChar = buffer[0];
		if( cRxedChar == '\r' ){
			/*
			 * Entra quando finalizado envio de comando
			 */
			CDC_Transmit_FS((uint8_t *)"\n\r", 2);
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

			 /* All the strings generated by the input command have been sent.
			 Processing of the command is complete.  Clear the input string ready
			 to receive the next command. */
			 cInputIndex = 0;
			 memset( pcInputString, 0x00, MAX_INPUT_LENGTH );
			 CDC_Transmit_FS((uint8_t *) "\n\r# ", 4);

		} else {
			if( cRxedChar == '\0' ){
				CDC_Transmit_FS((uint8_t *) "Welcome to FreeRTOS\n\r# ", strlen((char *)"Welcome to FreeRTOS\n\r# "));
			} else if (cRxedChar == 0x7F ){
				/*
				 * Backspace was pressed.
				 */
				if(cInputIndex > 0){
					CDC_Transmit_FS(&cRxedChar, 1);
					cInputIndex--;
					pcInputString[ cInputIndex ] = '\0';
				}
			} else if(cInputIndex < MAX_INPUT_LENGTH){
			/*
			 * Replica digitado para tela
			 */
				CDC_Transmit_FS(&cRxedChar, 1);
				pcInputString[ cInputIndex ] = cRxedChar;
				cInputIndex++;
			}
		}
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
