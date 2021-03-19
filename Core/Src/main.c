/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "adc.h"
#include "dfsdm.h"
#include "i2c.h"
#include "octospi.h"
#include "spi.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "tx_api.h"
#include "stm32l4s5i_iot01_accelero.h"
#include "stm32l4s5i_iot01_tsensor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE BEGIN PTD */
#define STACK_SIZE 1024
#define BYTE_POOL_SIZE 9120
#define BLOCK_POOL_SIZE 100
#define QUEUE_SIZE 100
#define EVT_BUTTON_PRESSED 0x1
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
TX_THREAD thread_0;
TX_THREAD thread_1;
TX_THREAD thread_2;
TX_BYTE_POOL byte_pool_0;
TX_MUTEX mtx_led;
TX_EVENT_FLAGS_GROUP event_flags_0;
TX_BLOCK_POOL block_pool_0;
UCHAR memory_area[BYTE_POOL_SIZE];

_Noreturn void blink_PA_5(ULONG thread_input);
_Noreturn void blink_PB_14(ULONG thread_input);
_Noreturn void read_temperature_sensor(ULONG thread_input);
_Noreturn void read_accelerometer(ULONG thread_input);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief interrupt handler to set event flags
 * @param GPIO_Pin
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_13) {
        tx_event_flags_set(&event_flags_0, EVT_BUTTON_PRESSED, TX_OR);
    }
}


/**
 * @brief function for defining the ThreadX application
 * @param first_unused_memory
 */
void tx_application_define(void* first_unused_memory) {
    char* pointer = TX_NULL;
    unsigned int status;

    tx_event_flags_create(&event_flags_0, "event flags 0");

    tx_byte_pool_create(&byte_pool_0, "byte pool 0", memory_area, BYTE_POOL_SIZE);

    tx_byte_allocate(&byte_pool_0, (VOID **) &pointer, STACK_SIZE, TX_NO_WAIT);
    status = tx_thread_create(&thread_0, "thread 0", blink_PA_5, 1,
                              pointer, STACK_SIZE,
                              1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
        printf("thread creation failed\r\n");

    tx_byte_allocate(&byte_pool_0, (VOID **) &pointer, STACK_SIZE, TX_NO_WAIT);
    status = tx_thread_create(&thread_1, "thread 1", blink_PB_14, 2,
                              pointer, STACK_SIZE,
                              1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
        printf("thread creation failed\r\n");

    tx_byte_allocate(&byte_pool_0, (VOID **) &pointer, STACK_SIZE, TX_NO_WAIT);
    status = tx_thread_create(&thread_2, "thread 2", read_temperature_sensor, 2,
                              pointer, STACK_SIZE,
                              1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
        printf("thread creation failed\r\n");
}

/**
 * @brief thread that blinks PA5 at given interval.
 * @param thread_input: interval in seconds of blinking at 50% duty cycle
 */
_Noreturn void blink_PA_5(ULONG thread_input) {
    uint32_t ticks_duty_cycle = (thread_input * 100) / 2;  // thread_input is in seconds, 100 ticks/second
    while (1) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        tx_thread_sleep(ticks_duty_cycle);  // this is in ticks, which is default 100 per second.
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        tx_thread_sleep(ticks_duty_cycle);
    }
}

/**
 * @brief thread that blinks PB14 at given interval.
 * @param thread_input: interval in seconds of blinking at 50% duty cycle
 */
_Noreturn void blink_PB_14(ULONG thread_input) {
    uint32_t period_ticks = (thread_input * 100) / 2;  // thread_input is in seconds, 100 ticks/second
    while(1) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        tx_thread_sleep(period_ticks);  // this is in ticks, which is default 100 per second.
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        tx_thread_sleep(period_ticks);
    }
}

/**
 * @brief thread that reads the temperature sensor value at given interval
 * @param thread_input: interval in seconds to read the tsensor
 */
_Noreturn void read_temperature_sensor(ULONG thread_input) {
    uint32_t period_ticks = (thread_input * 100);  // thread_input is in seconds, 100 ticks/second
    float temperature;
    BSP_TSENSOR_Init();
    while (1) {
        tx_thread_sleep(period_ticks);
        temperature = BSP_TSENSOR_ReadTemp();
        printf("%f", temperature);
    }
}

/**
 * @brief thread that reads the accelerometer XYZ at given interval
 * @param thread_input: interval in seconds to read the tsensor
 */
_Noreturn void read_accelerometer(ULONG thread_input) {
    uint32_t period_ticks = (thread_input * 100);  // thread_input is in seconds, 100 ticks/second
    int16_t current_xyz[3];
    ACCELERO_StatusTypeDef xl_status = BSP_ACCELERO_Init();
    if (xl_status != ACCELERO_OK)
        printf("ERROR!");
    while (1) {
        tx_thread_sleep(period_ticks);
        BSP_ACCELERO_AccGetXYZ(current_xyz);
        printf("X: %d, Y: %d, Z: %d", current_xyz[0], current_xyz[1], current_xyz[2]);
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
  MX_ADC1_Init();
  MX_DFSDM1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_OCTOSPI1_Init();
  MX_SPI3_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_USB_Init();
  /* USER CODE BEGIN 2 */
  tx_kernel_enter();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_UART4
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_I2C2
                              |RCC_PERIPHCLK_DFSDM1|RCC_PERIPHCLK_USB
                              |RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_OSPI;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
  PeriphClkInit.OspiClockSelection = RCC_OSPICLKSOURCE_SYSCLK;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
