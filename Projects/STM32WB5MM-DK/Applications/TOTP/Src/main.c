/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"

#include "logging_levels.h"
#define LOG_LEVEL LOG_DEBUG
#include "logging.h"
#include "sys_evt.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "cli.h"
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

RNG_HandleTypeDef hrng;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint32_t ulCsrFlags = 0;

aPwmLedGsData_TypeDef aPwmLedGsData_app;

QueueHandle_t xQueueTemperature;
EventGroupHandle_t xSystemEvents = NULL;
SemaphoreHandle_t xMutex;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void vDoSystemReset(void);
void vDetermineResetSource(void);

void LED_Deinit(void);
void LED_On(aPwmLedGsData_TypeDef aPwmLedGsData);
void LED_Off(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void vHeartbeatTask( void * pvParameters )
{
    ( void ) pvParameters;

    while(1)
    {
//    	if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
//    	{
			aPwmLedGsData_app[PWM_LED_RED]  = PWM_LED_GSDATA_7_0;
			aPwmLedGsData_app[PWM_LED_BLUE] = PWM_LED_GSDATA_7_0;

    		LED_On(aPwmLedGsData_app);

    		vTaskDelay( pdMS_TO_TICKS( 1000 ) );

    		LED_Off();

    		vTaskDelay( pdMS_TO_TICKS( 1000 ) );

//			xSemaphoreGive(xMutex);
//    	}
    }
}

static void vSensorsTask( void * pvParameters )
{
    ( void ) pvParameters;

#if defined(USE_ENV_SENSOR_STTS22H_0)
    float temperature;
    temperature = 0;

    uint32_t ret = BSP_ENV_SENSOR_Init(ENV_SENSOR_STTS22H_0, ENV_TEMPERATURE);
    if (ret != BSP_ERROR_NONE)
    {
      Error_Handler();
    }

    ret = BSP_ENV_SENSOR_Enable(ENV_SENSOR_STTS22H_0, ENV_TEMPERATURE);
    if (ret != BSP_ERROR_NONE)
    {
      Error_Handler();
    }

    while(1)
    {
    	ret = BSP_ENV_SENSOR_GetValue(ENV_SENSOR_STTS22H_0, ENV_TEMPERATURE, &temperature);
    	if (ret != BSP_ERROR_NONE)
    	{
    	  Error_Handler();
    	}

    	if (xQueueSend(xQueueTemperature, &temperature, portMAX_DELAY) != pdPASS)
    	{
    		LogError("Failed to get temperature value");
    	}

    	LogInfo("Temperature = %.2f", temperature);
    	vTaskDelay( pdMS_TO_TICKS( 500 ) );

    }
#endif /* defined(USE_ENV_SENSOR_STTS22H_0) */
}

static void vLCDScreenTask( void * pvParameters )
{
    ( void ) pvParameters;

    float receivedTemperature;
    char tempString[32];

    BSP_SPI1_Init();

    BSP_LCD_Init(0, LCD_ORIENTATION_LANDSCAPE);
    UTIL_LCD_SetFuncDriver(&LCD_Driver); /* SetFunc before setting device */
    UTIL_LCD_SetDevice(0);            /* SetDevice after funcDriver is set */
    BSP_LCD_Clear(0,SSD1315_COLOR_BLACK);
    BSP_LCD_DisplayOn(0);
    BSP_LCD_Refresh(0);
    UTIL_LCD_SetFont(&Font12);
    /* Set the LCD Text Color */
    UTIL_LCD_SetTextColor(SSD1315_COLOR_WHITE);
    UTIL_LCD_SetBackColor(SSD1315_COLOR_BLACK);
    BSP_LCD_Clear(0,SSD1315_COLOR_BLACK);
    BSP_LCD_Refresh(0);

    while(1)
    {
        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
        {
            if (xQueueReceive(xQueueTemperature, &receivedTemperature, portMAX_DELAY) == pdTRUE)
            {
                BSP_LCD_Clear(0,SSD1315_COLOR_BLACK);
                snprintf(tempString, sizeof(tempString), "Temperature = %.2f", receivedTemperature);
                UTIL_LCD_DisplayStringAt(0, 0, (uint8_t *)tempString, CENTER_MODE);
                BSP_LCD_Refresh(0);
            }

            xSemaphoreGive(xMutex);
        }

        vTaskDelay( pdMS_TO_TICKS( 500 ) );
    }
}


void vInitTask( void * pvArgs )
{
    BaseType_t xResult;

    ( void ) pvArgs;

    xMutex = xSemaphoreCreateMutex();

    if (xMutex == NULL)
    {
    	LogError("Failed to create xMutex");
    	configASSERT( 0 );
    }

    xQueueTemperature = xQueueCreate(10, sizeof(float));
    if (xQueueTemperature == NULL)
    {
    	LogError("Failed to create xQueueTemperature");
    	configASSERT(0);
    }

    xResult = xTaskCreate( Task_CLI, "cli", 2048, NULL, 10, NULL );
    configASSERT( xResult == pdTRUE );

    xResult = xTaskCreate( vHeartbeatTask, "Heartbeat", 128, NULL, tskIDLE_PRIORITY, NULL );
    configASSERT( xResult == pdTRUE );

    xResult = xTaskCreate( vSensorsTask, "Sensors", 2048, NULL, tskIDLE_PRIORITY, NULL );
    configASSERT( xResult == pdTRUE );

    xResult = xTaskCreate( vLCDScreenTask, "LCDScreen", 2048, NULL, tskIDLE_PRIORITY, NULL );
    configASSERT( xResult == pdTRUE );

    ( void ) xEventGroupSetBits( xSystemEvents, EVT_MASK_INIT );

    while( 1 )
    {
        vTaskSuspend( NULL );
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
  aPwmLedGsData_app[PWM_LED_RED]   = PWM_LED_GSDATA_OFF;
  aPwmLedGsData_app[PWM_LED_GREEN] = PWM_LED_GSDATA_OFF;
  aPwmLedGsData_app[PWM_LED_BLUE]  = PWM_LED_GSDATA_OFF;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
  ulCsrFlags = RCC->CSR;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */

  /* Initialize uart for logging before cli is up and running */
  vInitLoggingEarly();
  vLoggingInit();

  vDetermineResetSource();
  __HAL_RCC_CLEAR_RESET_FLAGS();

  LogInfo( "HW Init Complete" );

  xSystemEvents = xEventGroupCreate();

  xTaskCreate( vInitTask, "Init", 1024, NULL, 8, NULL );

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* Start scheduler */
  vTaskStartScheduler();

  /* Initialize threads */
  LogError( "Kernel start returned." );

  /* This loop should be inaccessible.*/
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI1
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  hrng.Init.ClockErrorDetection = RNG_CED_ENABLE;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_8_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_8_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_Delay( uint32_t ulDelayMs )
{
    if( xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED )
    {
        vTaskDelay( pdMS_TO_TICKS( ulDelayMs ) );
    }
    else
    {
        uint32_t ulStartTick = HAL_GetTick();
        uint32_t ulTicksWaited = ulDelayMs;

        /* Add a freq to guarantee minimum wait */
        if( ulTicksWaited < HAL_MAX_DELAY )
        {
            ulTicksWaited += ( uint32_t ) ( HAL_GetTickFreq() );
        }

        while( ( HAL_GetTick() - ulStartTick ) < ulTicksWaited )
        {
            __NOP();
        }
    }
}

/*-----------------------------------------------------------*/

/* configUSE_STATIC_ALLOCATION is set to 1, so the application must provide an
 * implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
 * used by the Idle task. */
void vApplicationGetIdleTaskMemory( StaticTask_t ** ppxIdleTaskTCBBuffer,
                                    StackType_t ** ppxIdleTaskStackBuffer,
                                    uint32_t * pulIdleTaskStackSize )
{
    /* If the buffers to be provided to the Idle task are declared inside this
     * function then they must be declared static - otherwise they will be allocated on
     * the stack and so not exists after this function exits. */
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
     * state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
     * Note that, as the array is necessarily of type StackType_t,
     * configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

/*-----------------------------------------------------------*/

/* configUSE_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
 * application must provide an implementation of vApplicationGetTimerTaskMemory()
 * to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory( StaticTask_t ** ppxTimerTaskTCBBuffer,
                                     StackType_t ** ppxTimerTaskStackBuffer,
                                     uint32_t * pulTimerTaskStackSize )
{
    /* If the buffers to be provided to the Timer task are declared inside this
     * function then they must be declared static - otherwise they will be allocated on
     * the stack and so not exists after this function exits. */
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

    /* Pass out a pointer to the StaticTask_t structure in which the Timer
     * task's state will be stored. */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task's stack. */
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
     * Note that, as the array is necessarily of type StackType_t,
     * configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
    LogError( "Malloc failed" );

    while( 1 )
    {
        __NOP();
    }
}

/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                    char * pcTaskName )

{
    volatile uint32_t ulSetToZeroToStepOut = 1UL;

    taskENTER_CRITICAL();

    LogSys( "Stack overflow in %s", pcTaskName ); /* WARN: The log message will not be output until ulSetToZeroToStepOut is reset by the user. */
    ( void ) xTask;

    while( ulSetToZeroToStepOut != 0 )
    {
        __NOP();
    }

    taskEXIT_CRITICAL();
}

/*-----------------------------------------------------------*/

#if configUSE_IDLE_HOOK == 1
void vApplicationIdleHook( void )
{

}
#endif /* configUSE_IDLE_HOOK == 1 */

/*-----------------------------------------------------------*/

void vDoSystemReset( void )
{

    if( xTaskGetSchedulerState() == taskSCHEDULER_RUNNING )
    {
        vTaskSuspendAll();
    }

    LogSys( "System Reset in progress." );

    /* Drain log buffers */
    vDyingGasp();

    NVIC_SystemReset();
}

/*-----------------------------------------------------------*/

void vDetermineResetSource( void )
{
    const char * pcResetSource = NULL;

    ulCsrFlags &= ( RCC_CSR_PINRSTF |
    RCC_CSR_BORRSTF | RCC_CSR_SFTRSTF |
	RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF |
    RCC_CSR_LPWRRSTF );

    if( ulCsrFlags & RCC_CSR_PINRSTF )
    {
        pcResetSource = "PINRSTF: pin reset";
    }
    if( ulCsrFlags & RCC_CSR_BORRSTF )
    {
        pcResetSource = ( pcResetSource == NULL ) ? "BORRSTF: BOR" : "BORRSTF: BOR with pin reset";
    }
    else if( ulCsrFlags & RCC_CSR_SFTRSTF )
    {
        pcResetSource = ( pcResetSource == NULL ) ? "SFTRSTF: software system reset" : "SFTRSTF: software system reset with pin reset";
    }
    else if( ulCsrFlags & RCC_CSR_IWDGRSTF )
    {
        pcResetSource = ( pcResetSource == NULL ) ? "IWDGRSTF: independent watchdog" : "IWDGRSTF: independent watchdog with pin reset";
    }
    else if( ulCsrFlags & RCC_CSR_WWDGRSTF )
    {
        pcResetSource = ( pcResetSource == NULL ) ? "WWDGRSTF: window watchdog" : "WWDGRSTF: window watchdog with pin reset";
    }
    else if( ulCsrFlags & RCC_CSR_LPWRRSTF )
    {
        pcResetSource = ( pcResetSource == NULL ) ? "LPWRRSTF: Low-power" : "LPWRRSTF: Low-power with pin reset";
    }

    if( pcResetSource == NULL )
    {
        pcResetSource = "Unknown";
    }

    LogSys( "Reset Source: 0x%x : %s.", ulCsrFlags, pcResetSource );
}

void LED_Deinit(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure = {0};

  /* RGB Led de-init */
  BSP_PWM_LED_DeInit();

  /* configure SPIx MOSI for LCD */
  GPIO_InitStructure.Pin       = BUS_SPI1_MOSI_PIN;
  GPIO_InitStructure.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull      = GPIO_PULLDOWN;
  GPIO_InitStructure.Speed     = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStructure.Alternate = BUS_SPI1_AF;
  HAL_GPIO_Init(BUS_SPI1_GPIO_PORTA, &GPIO_InitStructure);
}


void LED_On(aPwmLedGsData_TypeDef aPwmLedGsData)
{
  BSP_PWM_LED_Init();
  BSP_PWM_LED_On(aPwmLedGsData);
  LED_Deinit();
}

void LED_Off(void)
{
  BSP_PWM_LED_Init();
  BSP_PWM_LED_Off();
  LED_Deinit();
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM16 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM16) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
