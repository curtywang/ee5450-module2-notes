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
#include "rng.h"
#include "spi.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "tx_api.h"
#include "nx_api.h"
#include "nx_wifi.h"
#include "cmsis_utils.h"
#include "stm32l4s5i_iot01_accelero.h"
#include "stm32l4s5i_iot01_tsensor.h"
#include "wifi.h"
#include "nxd_mqtt_client.h"
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
#define STACK_SIZE 4096
#define BYTE_POOL_SIZE 32768
#define BLOCK_POOL_SIZE 100
#define QUEUE_SIZE 100
#define EVT_BUTTON_PRESSED 0x1
#define EVT_WIFI_READY 0x4

#define WIFI_AP_SSID "Hello Home"  // TODO: change this to yours
#define WIFI_AP_KEY "TaiwanNumbaOne"  // TODO: change this to yours
#define TX_PACKET_COUNT 20
#define TX_PACKET_SIZE 1200  // default payload size from ES_WIFI_PAYLOAD_SIZE
#define TX_POOL_SIZE ((TX_PACKET_SIZE + sizeof(NX_PACKET)) * TX_PACKET_COUNT)
#define MQTT_BROKER_IP IP_ADDRESS(192, 168, 11, 124)  // TODO: change this IP address to yours
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
struct global_data_t {
    TX_THREAD threads[5];
    TX_THREAD thread_network_setup;
    TX_BYTE_POOL byte_pool_0;
    TX_MUTEX mutex_led;  // TODO: add the button handler back in here
    TX_MUTEX mutex_i2c2;
    TX_MUTEX mutex_mqtt;
    TX_MUTEX mutex_network_reset;
    TX_BLOCK_POOL block_pool_0;
    UCHAR memory_area[BYTE_POOL_SIZE];

    NX_IP nx_ip;
    NX_PACKET_POOL nx_pool;
    UCHAR nx_ip_pool[TX_POOL_SIZE];
    NXD_ADDRESS mqtt_server_ip;
    ULONG mqtt_client_stack[STACK_SIZE / sizeof(ULONG)];
    NXD_MQTT_CLIENT mqtt_client;
    TX_EVENT_FLAGS_GROUP mqtt_event_flags;

    ULONG interval_ld1;
    ULONG interval_ld2;
    ULONG interval_temperature;
    ULONG interval_accelerometer;
    ULONG interval_mqtt;
};

TX_EVENT_FLAGS_GROUP global_event_flags;
TX_BYTE_POOL global_byte_pool;
ULONG global_memory_area[sizeof(struct global_data_t) / sizeof(ULONG)];

void thread_network_setup(ULONG global_data_ulong);
_Noreturn void blink_PA_5(ULONG global_data_ulong);
_Noreturn void blink_PB_14(ULONG global_data_ulong);
_Noreturn void thread_temperature(ULONG global_data_ulong);
_Noreturn void thread_accelerometer(ULONG global_data_ulong);
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
        tx_event_flags_set(&global_event_flags, EVT_BUTTON_PRESSED, TX_OR);
    }
    else if (GPIO_Pin == GPIO_PIN_1) {
        SPI_WIFI_ISR();
    }
}
/**
 * @brief handle the spi3 interrupt (for Wifi/BLE)
 */
void SPI3_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&hspi);
}

/**
 * @brief function for defining the ThreadX application
 * @param first_unused_memory
 */
void tx_application_define(void* first_unused_memory) {
    char* pointer = TX_NULL;
    volatile UINT status;

    systick_interval_set(TX_TIMER_TICKS_PER_SECOND);

    status = tx_event_flags_create(&global_event_flags, "global event flags");
    status = tx_byte_pool_create(&global_byte_pool, "global byte pool",
                                 global_memory_area, sizeof(struct global_data_t) << 1);

    /* allocate global data structure */
    status = tx_byte_allocate(&global_byte_pool, (VOID**)&pointer, sizeof(struct global_data_t), TX_NO_WAIT);
    struct global_data_t* global_data = (struct global_data_t*)pointer;
    global_data->interval_mqtt = 1;
    global_data->interval_accelerometer = 1;
    global_data->interval_temperature = 1;
    global_data->interval_ld1 = 1;
    global_data->interval_ld2 = 2;

    /* initialize synchronization primitives */
    status = tx_mutex_create(&global_data->mutex_mqtt, "MQTT client mutex", TX_NO_INHERIT);
    status = tx_mutex_create(&global_data->mutex_i2c2, "I2C channel 2 mutex", TX_NO_INHERIT);
    status = tx_mutex_create(&global_data->mutex_led, "LED mutex", TX_NO_INHERIT);
    status = tx_mutex_create(&global_data->mutex_network_reset, "network setup mutex", TX_NO_INHERIT);

    status = tx_byte_pool_create(&global_data->byte_pool_0, "byte pool 0",
                        global_data->memory_area, BYTE_POOL_SIZE);

    status = tx_byte_allocate(&global_data->byte_pool_0, (VOID **) &pointer, STACK_SIZE, TX_NO_WAIT);
    status = tx_thread_create(&global_data->thread_network_setup, "thread net test",
                              thread_network_setup, (ULONG)global_data,
                              pointer, STACK_SIZE,
                              1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
        printf("thread creation failed\r\n");

    tx_byte_allocate(&global_data->byte_pool_0, (VOID **) &pointer, STACK_SIZE, TX_NO_WAIT);
    status = tx_thread_create(&global_data->threads[0], "thread 0", blink_PA_5, (ULONG)global_data,
                              pointer, STACK_SIZE,
                              3, 3, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
        printf("thread creation failed\r\n");

    tx_byte_allocate(&global_data->byte_pool_0, (VOID **) &pointer, STACK_SIZE, TX_NO_WAIT);
    status = tx_thread_create(&global_data->threads[1], "thread 1", blink_PB_14, (ULONG)global_data,
                              pointer, STACK_SIZE,
                              3, 3, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
        printf("thread creation failed\r\n");

    tx_byte_allocate(&global_data->byte_pool_0, (VOID **) &pointer, STACK_SIZE, TX_NO_WAIT);
    status = tx_thread_create(&global_data->threads[2], "thread 2",
                              thread_temperature, (ULONG)global_data,
                              pointer, STACK_SIZE,
                              2, 2, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
        printf("thread creation failed\r\n");

    tx_byte_allocate(&global_data->byte_pool_0, (VOID **) &pointer, STACK_SIZE, TX_NO_WAIT);
    status = tx_thread_create(&global_data->threads[3], "thread 3",
                              thread_accelerometer, (ULONG)global_data,
                              pointer, STACK_SIZE,
                              2, 2, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
        printf("thread creation failed\r\n");
}


/**
 * @brief setup the wifi driver (uses global variables in wifi.h; configures SPI3 Inventek module)
 */
UINT setup_wifi(bool scan_for_aps) {
    UINT status;
    uint8_t max_aps = 10;
    status = WIFI_Init();
    if (status != WIFI_STATUS_OK)
        return status;
    if (scan_for_aps == true) {
        WIFI_APs_t aps;
        while (WIFI_ListAccessPoints(&aps, max_aps) != WIFI_STATUS_OK) {
            tx_thread_sleep(100);
        }
        printf("%d", aps.count);
    }
    while (WIFI_Connect(WIFI_AP_SSID, WIFI_AP_KEY, WIFI_ECN_WPA2_PSK) != WIFI_STATUS_OK) {
        tx_thread_sleep(100);
    }
    uint8_t goog_ip[] = {8, 8, 8, 8};
    uint16_t ping_count = 10;
    int32_t ping_result[ping_count];
    while (WIFI_Ping(goog_ip, ping_count, 10, ping_result) != WIFI_STATUS_OK) {
        tx_thread_sleep(100);
    }
    printf("%ld", ping_result[0]);
    return WIFI_STATUS_OK;
}


/**
 * @brief cleanup the wifi driver
 */
void cleanup_wifi() {
    WIFI_Disconnect();
}


/**
 * @brief sets up NetX Duo to work with the wifi driver
 * @param global_data: pointer to the global data structure
 * @return
 */
UINT setup_nx_wifi(struct global_data_t* global_data) {
    UINT status;
    uint8_t ip_address[4];
    nx_system_initialize();
    status = nx_packet_pool_create(&global_data->nx_pool, "NX Packet Pool", TX_PACKET_SIZE,
                                   global_data->nx_ip_pool, TX_POOL_SIZE);
    if (status != NX_SUCCESS) {
        printf("%d", status);
        return status;
    }
    WIFI_GetIP_Address(ip_address);
    status = nx_ip_create(&global_data->nx_ip, "NX IP Instance 0",
                          IP_ADDRESS(ip_address[0], ip_address[1], ip_address[2], ip_address[3]),
                          0xFFFFFF00UL,
                          &global_data->nx_pool, NULL, NULL, 0, 0);
    if (status != NX_SUCCESS) {
        printf("%d", status);
        return status;
    }
    status = nx_wifi_initialize(&global_data->nx_ip, &global_data->nx_pool);
    if (status != NX_SUCCESS) {
        printf("%d", status);
        return status;
    }
    return status;
}


/**
 * @brief cleanup NetX Duo wifi data
 * @param global_data: pointer to the global data structure
 * @return
 */
UINT cleanup_nx_wifi(struct global_data_t* global_data) {
    UINT status;
    status = nx_ip_delete(&global_data->nx_ip);
    if (status != NX_SUCCESS) {
        printf("%d", status);
        return status;
    }
    status = nx_packet_pool_delete(&global_data->nx_pool);
    if (status != NX_SUCCESS) {
        printf("%d", status);
        return status;
    }
    return status;
}


/**
 * @brief setup the netx duo MQTT client
 * @param global_data: pointer to the global data structure
 * @return
 */
UINT setup_nx_mqtt_and_connect(struct global_data_t* global_data) {
    UINT status;
    status = nxd_mqtt_client_create(&global_data->mqtt_client, "MQTT Client", "le_board", sizeof("le_board") - 1,
                           &global_data->nx_ip, &global_data->nx_pool,
                           (void*)&global_data->mqtt_client_stack, sizeof(global_data->mqtt_client_stack),
                           2, NX_NULL, 0);
    if (status != NX_SUCCESS)
        return status;
    tx_event_flags_create(&global_data->mqtt_event_flags, "mqtt events");

    global_data->mqtt_server_ip.nxd_ip_version = 4;
    global_data->mqtt_server_ip.nxd_ip_address.v4 = MQTT_BROKER_IP;
    status = nxd_mqtt_client_connect(&global_data->mqtt_client, &global_data->mqtt_server_ip, NXD_MQTT_PORT,
                                     300, 0, NX_WAIT_FOREVER);
    return status;
}


/**
 * @brief cleanup nx MQTT client data structures
 * @param global_data: pointer to the global data structure
 */
void cleanup_nx_mqtt(struct global_data_t* global_data) {
    UINT status;
    status = nxd_mqtt_client_disconnect(&global_data->mqtt_client);
    printf("%d", status);
    status = nxd_mqtt_client_delete(&global_data->mqtt_client);
    printf("%d", status);
}


/**
 * @brief sends a message with the topic given to the MQTT client
 * @param global_data: pointer to the global data structure
 * @param topic: pointer to the topic buffer (should be null-terminated)
 * @param message: pointer to the message buffer (should be null-terminated)
 * @return status of the client
 */
UINT send_nx_mqtt_message(struct global_data_t* global_data,
                          char* topic, char* message) {
    UINT status;
    tx_mutex_get(&global_data->mutex_mqtt, TX_WAIT_FOREVER);
    status = nxd_mqtt_client_publish(&global_data->mqtt_client,
                                     topic, strlen(topic),
                                     message, strlen(message),
                                     0, 1, NX_WAIT_FOREVER);
    tx_mutex_put(&global_data->mutex_mqtt);
    return status;
}


/**
 * @brief setup the network by connecting to the Wifi network and using nxduo to test MQTT
 * @param global_data_ulong ->interval_mqtt: interval of sending out mqtt messages
 */
void thread_network_setup(ULONG global_data_ulong) {
    struct global_data_t* global_data = (struct global_data_t*)global_data_ulong;
    volatile UINT status;
    uint32_t tick_interval = 100 * global_data->interval_mqtt;
    ULONG current_tick;
    const size_t message_size = 30;
    char message[message_size + 1];
    const size_t topic_size = 30;
    char topic[topic_size + 1];
    memset(topic, 0, topic_size + 1);
    memset(message, 0, message_size + 1);

    status = setup_wifi(false);
    if (status != 0)
        return;
    status = setup_nx_wifi(global_data);
    if (status != 0)
        return;
    status = setup_nx_mqtt_and_connect(global_data);
    if (status != 0)
        return;

    snprintf(topic, topic_size, "board_test/hello");
    snprintf(message, message_size, "board says Hello!!!");
    status = send_nx_mqtt_message(global_data, topic, message);
    if (status != 0)
        return;

    for (size_t i = 0; i < 7; i++) {
        current_tick = tx_time_get();
        snprintf(message, message_size, "hello the time is: %lu", current_tick);
        status = send_nx_mqtt_message(global_data, topic, message);
        if (status != 0)
            return;
        tx_thread_sleep(tick_interval / 4);
    }

    tx_event_flags_set(&global_event_flags, EVT_WIFI_READY, TX_OR);
}


/**
 * @brief thread that blinks PA5 at given interval.
 * @param global_data_ulong ->interval_ld1: interval of blinking ld1 at 50% duty cycle
 */
_Noreturn void blink_PA_5(ULONG global_data_ulong) {
    struct global_data_t* global_data = (struct global_data_t*)global_data_ulong;
    uint32_t ticks_duty_cycle = (global_data->interval_ld1 * 100) / 2;
    while (1) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        tx_thread_sleep(ticks_duty_cycle);  // this is in ticks, which is default 100 per second.
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        tx_thread_sleep(ticks_duty_cycle);
    }
}


/**
 * @brief thread that blinks PB14 at given interval.
 * @param global_data_ulong ->interval_ld2: interval of blinking ld2 at 50% duty cycle
 */
_Noreturn void blink_PB_14(ULONG global_data_ulong) {
    struct global_data_t* global_data = (struct global_data_t*)global_data_ulong;
    uint32_t period_ticks = (global_data->interval_ld2 * 100) / 2;
    while(1) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        tx_thread_sleep(period_ticks);  // this is in ticks, which is default 100 per second.
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        tx_thread_sleep(period_ticks);
    }
}


/**
 * @brief utility for resetting the network setup thread
 * @param global_data: pointer to the global data strcuture
 */
void reset_network_thread(struct global_data_t* global_data) {
    ULONG actual_flags;
    // the mutex check that another thread isn't resetting
    UINT status = tx_mutex_get(&global_data->mutex_network_reset, TX_NO_WAIT);
    tx_event_flags_set(&global_event_flags, EVT_WIFI_READY, TX_AND_CLEAR);
    if (status == TX_SUCCESS) {
        status = tx_thread_reset(&global_data->thread_network_setup);  // reset network setup thread
        printf("%d", status);
        status = tx_thread_resume(&global_data->thread_network_setup);  // start the network setup thread
    }
    tx_event_flags_get(&global_event_flags, EVT_WIFI_READY, TX_AND, &actual_flags, TX_WAIT_FOREVER);
    tx_mutex_put(&global_data->mutex_network_reset);  // if not owned, will error out anyway
}


/**
 * @brief format temperature message for sending via some protocol (such as mqtt)
 * @param global_data: pointer to the global data structure
 * @param message: message buffer
 * @param message_size: size of message buffer
 */
void get_temperature_message(struct global_data_t* global_data, char* message, size_t message_size) {
    tx_mutex_get(&global_data->mutex_i2c2, TX_WAIT_FOREVER);
    float temperature = BSP_TSENSOR_ReadTemp();
    uint32_t current_tick = tx_time_get();
    tx_mutex_put(&global_data->mutex_i2c2);
    snprintf(message, message_size, "time: %lu, temp: %.2f degC", current_tick, temperature);
}


/**
 * @brief thread that reads the temperature sensor value at given interval
 * @param global_data_ulong ->interval_temperature: interval in seconds to read the tsensor
 */
_Noreturn void thread_temperature(ULONG global_data_ulong) {
    UINT status;
    ULONG actual_flags;
    struct global_data_t* global_data = (struct global_data_t*)global_data_ulong;
    uint32_t period_ticks = global_data->interval_temperature * 100;

    const size_t message_size = 40;
    char message[message_size + 1];
    const size_t topic_size = 30;
    char topic[topic_size + 1];
    memset(topic, 0, topic_size + 1);
    memset(message, 0, message_size + 1);
    snprintf(topic, topic_size, "board_test/temperature");

    BSP_TSENSOR_Init();
    tx_event_flags_get(&global_event_flags, EVT_WIFI_READY, TX_AND, &actual_flags, TX_WAIT_FOREVER);

    while (1) {
        get_temperature_message(global_data, message, message_size);
        status = send_nx_mqtt_message(global_data, topic, message);
        if (status != NX_SUCCESS) {
            reset_network_thread(global_data);
        }
        tx_thread_sleep(period_ticks);
    }
}


/**
 * @brief format accelerometer message for sending via some protocol (such as mqtt)
 * @param global_data: pointer to the global data structure
 * @param message: message buffer
 * @param message_size: size of message buffer
 */
void get_accelerometer_message(struct global_data_t* global_data, char* message, size_t message_size) {
    int16_t current_xyz[3];
    tx_mutex_get(&global_data->mutex_i2c2, TX_WAIT_FOREVER);
    BSP_ACCELERO_AccGetXYZ(current_xyz);
    uint32_t current_tick = tx_time_get();
    tx_mutex_put(&global_data->mutex_i2c2);
    snprintf(message, message_size, "time: %lu, xl: %.2hd %.2hd %.2hd",
             current_tick, current_xyz[0], current_xyz[1], current_xyz[2]);
}


/**
 * @brief thread that reads the accelerometer XYZ at given interval
 * @param global_data_ulong ->interval_accelerometer: interval in seconds to read the accelerometer
 */
_Noreturn void thread_accelerometer(ULONG global_data_ulong) {
    UINT status;
    ULONG actual_flags;
    struct global_data_t* global_data = (struct global_data_t*)global_data_ulong;
    uint32_t period_ticks = (global_data->interval_accelerometer * 100);

    const size_t message_size = 40;
    char message[message_size + 1];
    const size_t topic_size = 30;
    char topic[topic_size + 1];
    memset(topic, 0, topic_size + 1);
    memset(message, 0, message_size + 1);
    snprintf(topic, topic_size, "board_test/accelerometer");

    ACCELERO_StatusTypeDef xl_status = BSP_ACCELERO_Init();
    if (xl_status != ACCELERO_OK)
        printf("ERROR!");
    tx_event_flags_get(&global_event_flags, EVT_WIFI_READY, TX_AND, &actual_flags, TX_WAIT_FOREVER);

    while (1) {
        get_accelerometer_message(global_data, message, message_size);
        status = send_nx_mqtt_message(global_data, topic, message);
        if (status != NX_SUCCESS) {
            reset_network_thread(global_data);
        }
        tx_thread_sleep(period_ticks);
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
  MX_RNG_Init();
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
                              |RCC_PERIPHCLK_RNG|RCC_PERIPHCLK_ADC
                              |RCC_PERIPHCLK_OSPI;
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
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_PLLSAI1;
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
