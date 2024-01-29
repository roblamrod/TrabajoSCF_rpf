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
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "es_wifi.h"
#include "wifi.h"
#include "stm32l4xx_hal_uart.h"
#include <stdio.h>
#include <stdlib.h>
#include "stm32l4xx_hal_uart.h"
#include "stm32l475e_iot01.h"
#include "stm32l475e_iot01_tsensor.h"
#include "stm32l475e_iot01_hsensor.h"
#include <math.h>
#include <string.h>
#include "mqtt_priv.h"
#include "stm32l475e_iot01_accelero.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//#define SSID     "pabkramar" //"plutow" //"iPhone de Fernando"
//#define PASSWORD "12345678" //"abcdefgh" //"garruchin9"
char SSID [31];
char PASSWORD [31];
//#define WIFISECURITY WIFI_ECN_OPEN
#define WIFISECURITY WIFI_ECN_WPA2_PSK



#ifdef  TERMINAL_USE
#define LOG(a) printf a
#else
#define LOG(a)
#endif
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;

I2C_HandleTypeDef hi2c2;

QSPI_HandleTypeDef hqspi;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for wifiStart */
osThreadId_t wifiStartHandle;
const osThreadAttr_t wifiStart_attributes = {
  .name = "wifiStart",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for acel_task */
osThreadId_t acel_taskHandle;
const osThreadAttr_t acel_task_attributes = {
  .name = "acel_task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for print_task */
osThreadId_t print_taskHandle;
const osThreadAttr_t print_task_attributes = {
  .name = "print_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RTC_set */
osThreadId_t RTC_setHandle;
const osThreadAttr_t RTC_set_attributes = {
  .name = "RTC_set",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for wifi_set */
osThreadId_t wifi_setHandle;
const osThreadAttr_t wifi_set_attributes = {
  .name = "wifi_set",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for print_queue */
osMessageQueueId_t print_queueHandle;
const osMessageQueueAttr_t print_queue_attributes = {
  .name = "print_queue"
};
/* Definitions for receive_queue */
osMessageQueueId_t receive_queueHandle;
const osMessageQueueAttr_t receive_queue_attributes = {
  .name = "receive_queue"
};
/* Definitions for receive_wifi_queue */
osMessageQueueId_t receive_wifi_queueHandle;
const osMessageQueueAttr_t receive_wifi_queue_attributes = {
  .name = "receive_wifi_queue"
};
/* USER CODE BEGIN PV */
extern  SPI_HandleTypeDef hspi;
static  uint8_t  IP_Addr[4];
#if defined (TERMINAL_USE)
extern UART_HandleTypeDef hDiscoUart;
#endif /* TERMINAL_USE */

static  uint8_t  IP_Addr[4];

int _write(int file, char *ptr, int len){
	int DataIdx;
	for(DataIdx=0; DataIdx<len; DataIdx++)
	{
	ITM_SendChar(*ptr++);
	}
	return len;
}

int16_t pDataAcc[3];
int16_t lista_acelx[10]; //Lista para guardar los valores de las aceleraciones para luego hacerles la media.
int16_t lista_acely[10];
int16_t lista_acelz[10];

int modo_operacion = 0;

ACCELERO_StatusTypeDef iniAcc;
char str_x[14] = "";				/* cadena para la aceleraciÃ³n en el eje X */
char str_y[14] = "";				/* cadena para la aceleraciÃ³n en el eje X */
char str_z[18] = "";				/* cadena para la aceleraciÃ³n en el eje X */

int16_t acel_x = 0;
int16_t acel_y = 0;
int16_t acel_z = 0;

uint8_t acel_flag=0;

char timestamp[36] = "";			/* cadena para el timestamp */

uint8_t msg1[] = "****** Acceleration values measurement ******\n\n\r";
uint8_t msg2[] = "=====> Initialize Acceleration sensor LSM6DSL \r\n";
uint8_t msg3[] = "=====> Acceleration sensor LSM6DSL initialized \r\n ";

uint16_t subsec = 0;		/* variable para los ms */
RTC_TimeTypeDef varTime; /* estructura para recibir la hora del RTC */
RTC_DateTypeDef varDate; /* estructura para recibir la fecha */

uint8_t drdyPulsedCfg = 0;
uint8_t ctrlDrdy = 0;
uint8_t ctrlMaster = 0;

RTC_DateTypeDef GetDate; //Estructura para fijar/leer fecha
RTC_TimeTypeDef GetTime; //Estructura para fijar/leer hora

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_I2C2_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_RTC_Init(void);
void StartDefaultTask(void *argument);
void wifiStartTask(void *argument);
void acel_task_function(void *argument);
void print_task_func(void *argument);
void RTC_set_func(void *argument);
void wifi_set_func(void *argument);

/* USER CODE BEGIN PFP */

static uint8_t IP_Addr[4];
void LSM6DSL_AccInt_Drdy(void);							/* FunciÃ³n para la activaciÃ³n de la interrupciÃ³n Data Ready */
void  HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rec_data;
uint8_t cont = 0;
uint8_t config_state = 0;

#if defined (TERMINAL_USE)
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
#endif /* TERMINAL_USE */

static  int wifi_start(void);
static  int wifi_connect(void);
int wifi_get_http(void);
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
  MX_DFSDM1_Init();
  MX_I2C2_Init();
  MX_QUADSPI_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  #if defined (TERMINAL_USE)
  /* Initialize all configured peripherals */
  hDiscoUart.Instance = DISCOVERY_COM1;
  hDiscoUart.Init.BaudRate = 115200;
  hDiscoUart.Init.WordLength = UART_WORDLENGTH_8B;
  hDiscoUart.Init.StopBits = UART_STOPBITS_1;
  hDiscoUart.Init.Parity = UART_PARITY_NONE;
  hDiscoUart.Init.Mode = UART_MODE_TX_RX;
  hDiscoUart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hDiscoUart.Init.OverSampling = UART_OVERSAMPLING_16;
  hDiscoUart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hDiscoUart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;


  BSP_COM_Init(COM1, &hDiscoUart);

  #endif /* TERMINAL_USE */
  BSP_TSENSOR_Init(); // Inicializamos temperatura
  BSP_HSENSOR_Init(); // Inicializamos humedad
  printf("****** Sistemas Ciberfisicos ****** \n\r");

  HAL_UART_Transmit(&huart1,msg1,sizeof(msg1),1000); 			/* TransmisiÃ³n de mensajes por UART */
  HAL_UART_Transmit(&huart1,msg2,sizeof(msg2),1000);
  HAL_UART_Transmit(&huart1,msg3,sizeof(msg3),1000);


  iniAcc = BSP_ACCELERO_Init();									/* InicializaciÃ³n del acelerÃ³metro */
  LSM6DSL_AccInt_Drdy();											/* ConfiguraciÃ³n del acelerÃ³metro*/
  BSP_ACCELERO_LowPower(0);										/* Deshabilitado del modo de bajo consumo*/


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

  /* Create the queue(s) */
  /* creation of print_queue */
  print_queueHandle = osMessageQueueNew (64, sizeof(uintptr_t), &print_queue_attributes);

  /* creation of receive_queue */
  receive_queueHandle = osMessageQueueNew (3, sizeof(char), &receive_queue_attributes);

  /* creation of receive_wifi_queue */
  receive_wifi_queueHandle = osMessageQueueNew (31, sizeof(char), &receive_wifi_queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of wifiStart */
  wifiStartHandle = osThreadNew(wifiStartTask, NULL, &wifiStart_attributes);

  /* creation of acel_task */
  acel_taskHandle = osThreadNew(acel_task_function, NULL, &acel_task_attributes);

  /* creation of print_task */
  print_taskHandle = osThreadNew(print_task_func, NULL, &print_task_attributes);

  /* creation of RTC_set */
  RTC_setHandle = osThreadNew(RTC_set_func, NULL, &RTC_set_attributes);

  /* creation of wifi_set */
  wifi_setHandle = osThreadNew(wifi_set_func, NULL, &wifi_set_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_channel1.Instance = DFSDM1_Channel1;
  hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel1.Init.OutputClock.Divider = 2;
  hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
  hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel1.Init.Awd.Oversampling = 1;
  hdfsdm1_channel1.Init.Offset = 0;
  hdfsdm1_channel1.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00000E14;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 2;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 23;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x12;
  sTime.Minutes = 0x49;
  sTime.Seconds = 0x50;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x3;
  sDate.Year = 0x24;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
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
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_OTG_FS_PWR_EN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPBTLE_RF_SPI3_CSN_GPIO_Port, SPBTLE_RF_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPSGRF_915_SPI3_CSN_GPIO_Port, SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : M24SR64_Y_RF_DISABLE_Pin M24SR64_Y_GPO_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_OVRCR_EXTI3_Pin SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin ISM43362_DRDY_EXTI1_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_EXTI3_Pin|SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin|ISM43362_DRDY_EXTI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_EXTI13_Pin */
  GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_A5_Pin ARD_A4_Pin ARD_A3_Pin ARD_A2_Pin
                           ARD_A1_Pin ARD_A0_Pin */
  GPIO_InitStruct.Pin = ARD_A5_Pin|ARD_A4_Pin|ARD_A3_Pin|ARD_A2_Pin
                          |ARD_A1_Pin|ARD_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D1_Pin ARD_D0_Pin */
  GPIO_InitStruct.Pin = ARD_D1_Pin|ARD_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D10_Pin SPBTLE_RF_RST_Pin ARD_D9_Pin */
  GPIO_InitStruct.Pin = ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D4_Pin */
  GPIO_InitStruct.Pin = ARD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(ARD_D4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D7_Pin */
  GPIO_InitStruct.Pin = ARD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D13_Pin ARD_D12_Pin ARD_D11_Pin */
  GPIO_InitStruct.Pin = ARD_D13_Pin|ARD_D12_Pin|ARD_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D3_Pin */
  GPIO_InitStruct.Pin = ARD_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D6_Pin */
  GPIO_InitStruct.Pin = ARD_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D8_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin LED2_Pin
                           SPSGRF_915_SDN_Pin ARD_D5_Pin SPSGRF_915_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin|SPSGRF_915_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI0_Pin LSM6DSL_INT1_EXTI11_Pin ARD_D2_Pin HTS221_DRDY_EXTI15_Pin
                           PMOD_IRQ_EXTI12_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin|LSM6DSL_INT1_EXTI11_Pin|ARD_D2_Pin|HTS221_DRDY_EXTI15_Pin
                          |PMOD_IRQ_EXTI12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin SPBTLE_RF_SPI3_CSN_Pin PMOD_RESET_Pin STSAFE_A100_RESET_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin|SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
  GPIO_InitStruct.Pin = VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = VL53L0X_GPIO1_EXTI7_Pin|LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PMOD_SPI2_SCK_Pin */
  GPIO_InitStruct.Pin = PMOD_SPI2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PMOD_SPI2_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PMOD_UART2_CTS_Pin PMOD_UART2_RTS_Pin PMOD_UART2_TX_Pin PMOD_UART2_RX_Pin */
  GPIO_InitStruct.Pin = PMOD_UART2_CTS_Pin|PMOD_UART2_RTS_Pin|PMOD_UART2_TX_Pin|PMOD_UART2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D15_Pin ARD_D14_Pin */
  GPIO_InitStruct.Pin = ARD_D15_Pin|ARD_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void LSM6DSL_AccInt_Drdy()						/* InicializaciÃ³n del acelerÃ³metro */
	{

		uint8_t ctrl = 0x00;
		uint8_t tmp;
		/* Read DRDY_PULSE_CFG_G value  (LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_CTRL1_XL);*/
		drdyPulsedCfg = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_DRDY_PULSE_CFG_G);     /*Buscar en archivo lsm6dsl.h*/

		/* Set Drdy interruption to INT1  */
		drdyPulsedCfg |= 0b10000000;

		/* write back control register */
		SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_DRDY_PULSE_CFG_G, drdyPulsedCfg);

		/* Read INT1_CTRL value */
		ctrlDrdy = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_INT1_CTRL);

		/* Set Drdy interruption to INT1  */
	    ctrlDrdy |= 0b00000011;

		/* write back control register */
		SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_INT1_CTRL, ctrlDrdy);

		/* Read MASTER_CONFIG value */
		ctrlMaster = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_MASTER_CONFIG);

		ctrlMaster |= 0b00000011;

		/* write back control register */
		SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_MASTER_CONFIG, ctrlMaster);
	}

static int wifi_start(void)
{
  printf("Wifi Start Function. \n\r");
  uint8_t  MAC_Addr[6];

 /*Initialize and use WIFI module */
  if(WIFI_Init() ==  WIFI_STATUS_OK)
  {
    printf("ES-WIFI Initialized.\n\r");
    if(WIFI_GetMAC_Address(MAC_Addr) == WIFI_STATUS_OK)
    {
      printf("> eS-WiFi module MAC Address : %02X:%02X:%02X:%02X:%02X:%02X\n\r",
               MAC_Addr[0],
               MAC_Addr[1],
               MAC_Addr[2],
               MAC_Addr[3],
               MAC_Addr[4],
               MAC_Addr[5]);
    }
    else
    {
      printf("> ERROR : CANNOT get MAC address.\n\r");
      return -1;
    }
  }
  else
  {
    return -1;
  }
  return 0;
}

int wifi_connect(void)
{
    uint8_t MAX_tries = 3;
    uint8_t return_value=-1;
    uint8_t try=1;
	while (try<=MAX_tries || return_value!=0){
	  printf("Wifi connect function.. try %d/%d\n\r", try,MAX_tries);
	  wifi_start();
	  printf("Connecting to %s , %s.\n\r",SSID,PASSWORD);
	  if( WIFI_Connect(SSID, PASSWORD, WIFISECURITY) == WIFI_STATUS_OK)
	  {
		if(WIFI_GetIP_Address(IP_Addr) == WIFI_STATUS_OK)
		{
		  printf("> es-wifi module connected: got IP Address : %d.%d.%d.%d.\n\r",
				   IP_Addr[0],
				   IP_Addr[1],
				   IP_Addr[2],
				   IP_Addr[3]);
		  return_value=0; // CORRECTO
		  try=MAX_tries+1;
		  //osThreadFlagsSet(wifiStartHandle, 0x0001U);

		  // Activa la tarea de aceleracion
		  osThreadFlagsSet(acel_taskHandle, 0x000002U);
		}
		else
		{
		  printf("ERROR : es-wifi module CANNOT get IP address\n\r");
		  return_value= -1;
		}
	  }
	  else
	  {
		  printf("ERROR : es-wifi module NOT connected\n\r");
		  return_value= -1;
	  }
	  try=try+1;
	}
  return return_value;
}



/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
    case (GPIO_PIN_1):
    {
      //printf("GPIO_Pin.\n\r");
      SPI_WIFI_ISR();
      break;
    }
    case (GPIO_PIN_11):
      osThreadFlagsSet(acel_taskHandle, 0x000001U);
      break;
    default:
    {
      break;
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	char recibido = (char)rec_data;

	if (config_state == 0) {
		osStatus_t estado;
		if (huart == &huart1)
		{
			printf("Recibido un caracter: %c\r\n", recibido);
			cont++;
			if (cont > 3){
				osThreadFlagsSet(RTC_setHandle, 0x0002U);
				cont = 0;
			}else{
				estado = osMessageQueuePut(receive_queueHandle,&recibido, 0, 0);
				if (estado == osOK){
					printf("Caracter anadido a la cola de recepcion\r\n");
					if (recibido == '\n' || recibido == '\r'){
						osThreadFlagsSet(RTC_setHandle, 0x0001U);
						cont = 0;
					}
				}else if (estado == osErrorTimeout){
					osThreadFlagsSet(RTC_setHandle, 0x0002U);
				}else if (estado == osErrorParameter){
					printf("OsErrorParameter\r\n");
				}
				HAL_UART_Receive_IT(&huart1, &rec_data, sizeof(rec_data));
			}
		}
	}
	else if (config_state==1) {
		osStatus_t estado;
		if (huart == &huart1)
		{
			printf("Recibido un caracter: %c\r\n", recibido);
			cont++;
			if (cont > 30){
				osThreadFlagsSet(wifi_setHandle, 0x0002U);
				cont = 0;
			}else{
				estado = osMessageQueuePut(receive_wifi_queueHandle,&recibido, 0, 0);
				if (estado == osOK){
					printf("Caracter anadido a la cola de recepcion\r\n");
					if (recibido == '\n' || recibido == '\r'){
						osThreadFlagsSet(wifi_setHandle, 0x0001U);
						cont = 0;
					}
				}else if (estado == osErrorTimeout){
					osThreadFlagsSet(wifi_setHandle, 0x0002U);
				}else if (estado == osErrorParameter){
					printf("OsErrorParameter\r\n");
				}
				HAL_UART_Receive_IT(&huart1, &rec_data, sizeof(rec_data));
			}
		}

	}
}

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
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_wifiStartTask */
/**
* @brief Function implementing the wifiStart thread.
* @param argument: Not used
* @retval None
*/


void MQTTTask(void)
{
const uint32_t ulMaxPublishCount = 5UL;
NetworkContext_t xNetworkContext = { 0 };
MQTTContext_t xMQTTContext;
MQTTStatus_t xMQTTStatus;
TransportStatus_t xNetworkStatus;
float ftemp;
float fhum;
char payLoad[128];
 /* Attempt to connect to the MQTT broker. The socket is returned in
 * the network context structure. */
 xNetworkStatus = prvConnectToServer( &xNetworkContext );
 configASSERT( xNetworkStatus == PLAINTEXT_TRANSPORT_SUCCESS );
 //LOG(("Trying to create an MQTT connection\n"));
 prvCreateMQTTConnectionWithBroker( &xMQTTContext, &xNetworkContext );

 // subscribirse a un topic
 printf("Trying to subscribe to topic\n");
 modo_operacion = 0;
 prvMQTTSubscribeToTopic(&xMQTTContext,pcModOpTopic);
 for( ; ; )
 {
   /* Publicar cada 5 segundos */
   osDelay(5000);
   ftemp=BSP_TSENSOR_ReadTemp();
   fhum=BSP_HSENSOR_ReadHumidity();

   // Media de las aceleraciones

   if (acel_flag==1){
	   sprintf(payLoad,"{\"temperatura\":%02.2f, \"humedad\":%02.2f, \"acel_x\":%d, \"acel_y\":%d, \"acel_z\":%d}",ftemp, fhum, acel_x,acel_y,acel_z);
	   acel_flag = 0;
   }
   else{
	   sprintf(payLoad,"{\"temperatura\":%02.2f, \"humedad\":%02.2f}",ftemp, fhum);
   }

   prvMQTTPublishToTopic(&xMQTTContext,pcBaseTopic,payLoad);

   MQTT_ProcessLoop(&xMQTTContext);
 }
}





#if defined (TERMINAL_USE)
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&hDiscoUart, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}
#endif /* TERMINAL_USE */

/**
  * @brief  SPI3 line detection callback.
  * @param  None
  * @retval None
  */
void SPI3_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi);
}

/* USER CODE END Header_wifiStartTask */
void wifiStartTask(void *argument)
{
  /* USER CODE BEGIN wifiStartTask */
  uint8_t ret_flag;
  uint8_t control = 1;
  /* Infinite loop */
  while (control) {
	  ret_flag = osThreadFlagsWait(0x0001U, osFlagsWaitAny, osWaitForever);
	  if (ret_flag == 1U) {
		  control = 0;
		  wifi_connect();
		  for(;;)
		  {
			MQTTTask();
			osDelay(1);
		  }
	  }
  }
  /* USER CODE END wifiStartTask */
}

/* USER CODE BEGIN Header_acel_task_function */
/**
* @brief Function implementing the acel_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_acel_task_function */
void acel_task_function(void *argument)
{
  /* USER CODE BEGIN acel_task_function */
    uint32_t ret_flag = 0U;
  ret_flag = osThreadFlagsWait(0x00000002U, osFlagsWaitAny,osWaitForever);
  printf("Llamada desde la tarea de wifi.\n\r");
  uint8_t contador = 0;
  int16_t temp_acel_x = 0;
  int16_t temp_acel_y = 0;
  int16_t temp_acel_z = 0;
  // Infinite loop //
  for(;;)
  {
      ret_flag = osThreadFlagsWait(0x00000001U, osFlagsWaitAny,osWaitForever);
      if (ret_flag == 1U){
        BSP_ACCELERO_AccGetXYZ(pDataAcc);                            // Toma de Aceleración /
        HAL_RTC_GetTime(&hrtc, &varTime, RTC_FORMAT_BIN);            // Toma de timestamp /
        subsec = (varTime.SecondFraction-varTime.SubSeconds)*1000/varTime.SecondFraction;      /* ms del timestamp */
        HAL_RTC_GetDate(&hrtc, &varDate, RTC_FORMAT_BCD);            // Toma de fecha /

        snprintf(str_x,14,"Eje_X = %d, ",pDataAcc[0]);                /* Formateo del mensaje de aceleración del eje X */
        snprintf(str_y,14,"Eje_Y = %d, ",pDataAcc[1]);
        snprintf(str_z,18,"Eje_Z = %d, \r\n",pDataAcc[2]);

        if (subsec <10){
            snprintf(timestamp,27,"\r\nTimestamp = %d:%d:%d.00%d - ",varTime.Hours, varTime.Minutes, varTime.Seconds,subsec);

        }
        else if (10<=subsec && subsec <100) {
            snprintf(timestamp,28,"\r\nTimestamp = %d:%d:%d.0%d - ",varTime.Hours, varTime.Minutes, varTime.Seconds,subsec);

        }
        else{
            snprintf(timestamp,28,"\r\nTimestamp = %d:%d:%d.%d - ",varTime.Hours, varTime.Minutes, varTime.Seconds,subsec);
        }


    	HAL_UART_Transmit(&huart1,(uint8_t *)timestamp,26,1000);		/* TransmisiÃ³n de la informaciÃ³n por UART */


    	lista_acelx[contador] = pDataAcc[0];
        lista_acely[contador] = pDataAcc[1];
        lista_acelz[contador] = pDataAcc[2];
        //printf("Contador: %d. \n\r",contador);
        //printf("Modo op %d. \n\r",modo_operacion);
        if (contador >= 9){
        	acel_x=0;
        	acel_y=0;
        	acel_z=0;
        	for (int i = 0; i <= 9; i++){
        		temp_acel_x += lista_acelx[i];
        		temp_acel_y += lista_acely[i];
        		temp_acel_z += lista_acelz[i];
        	}

        	acel_x = temp_acel_x/10;
        	acel_y = temp_acel_y/10;
        	acel_z = temp_acel_z/10;

        	acel_flag=1;
        	contador=0;
        	temp_acel_x=0;
        	temp_acel_y=0;
        	temp_acel_z=0;
        	printf("Modo operacion en main: %d. \n\r", modo_operacion);
        	if (modo_operacion == 1){
            	osDelay(pdMS_TO_TICKS(20000));//(20000));
        	}
        	else
        	{
        		osDelay(pdMS_TO_TICKS(60000));//(60000));
        	}

        }
        contador=contador+1;
      }
    osDelay(1);
  }
  /* USER CODE END acel_task_function */
}

/* USER CODE BEGIN Header_print_task_func */
/**
* @brief Function implementing the print_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_print_task_func */
void print_task_func(void *argument)
{
  /* USER CODE BEGIN print_task_func */
	uintptr_t rec;
	osStatus_t estado;
	const char *cadto = "Timeout agotado recepcion\r\n";

  /* Infinite loop */
  for(;;)
  {
	  estado = osMessageQueueGet(print_queueHandle, &rec, NULL, osWaitForever);
	  if (estado == osOK)
		  HAL_UART_Transmit(&huart1, (uint8_t *)rec, strlen((const char *)rec), 10);
	  else if (estado == osErrorTimeout)
		  HAL_UART_Transmit(&huart1, (uint8_t *)cadto, strlen(cadto), 10);
  }
  /* USER CODE END print_task_func */
}

/* USER CODE BEGIN Header_RTC_set_func */
/**
* @brief Function implementing the RTC_set thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RTC_set_func */
void RTC_set_func(void *argument)
{
  /* USER CODE BEGIN RTC_set_func */
	char recibido[3] = {0};
	uint32_t flag_rec = 0x0000U;
	osStatus_t estado;
	char rec;
	const char* msg_hora_ok = "\r\nHora cambiada correctamente\r\n";
	const char* msg_fecha_ok = "Fecha cambiada correctamente\r\n";
	const char* msg_error = "\r\nERROR: Valor no válido\r\n";
	const char* msg_rtc1 = "\r\n\r\n========================\r\n" "| Configurar rtc |\r\n" "========================\r\n\r\n";
	const char* msg[6] = {
	"Hora (0-23): ", "\r\nMinuto (0-59): ","\r\nSegundo (0-59): ","\r\nDía (1-31): ","\r\nMes (1-12): ",
	"\r\nAño (0-99): "};
	uint8_t limit[6][2] = {{0,23},{0,59},{0,59},{1,31},{1,12},{0,99}};
	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef sDate = {0};
	uint8_t *toChange[6] = {&sTime.Hours, &sTime.Minutes, &sTime.Seconds, &sDate.Date,
	&sDate.Month, &sDate.Year};

	/* Infinite loop */
	for(;;)
	{
	  restart_loop:
	  HAL_UART_Transmit(&huart1, (uint8_t *)msg_rtc1, strlen(msg_rtc1), 10);
	  for (int i = 0; i < 6; i++){
		  HAL_UART_Transmit(&huart1, (uint8_t *)msg[i], strlen(msg[i]), 10);
		  HAL_UART_Receive_IT(&huart1, &rec_data, sizeof(rec_data));
		  flag_rec = osThreadFlagsWait(0x00000003U, osFlagsWaitAny, osWaitForever);
		  if ( flag_rec == 0x0001U){
			  printf("Salto de linea pulsado, bandera 0 recibida\r\n");
			  for (int i = 0; i<3;i++){
				  estado = osMessageQueueGet(receive_queueHandle, &rec, NULL, pdMS_TO_TICKS(500));
				  if (estado == osOK){
					  if (i == 2)
						  recibido[i] = '\0';
					  else
						  recibido[i] = rec;
				  }
			  }
			  if (((uint8_t)strtol(recibido, NULL, 10) > limit[i][1]) || ((uint8_t)strtol(recibido, NULL, 10) < limit[i][0])){
				  HAL_UART_Transmit(&huart1, (uint8_t *)msg_error, strlen(msg_error), 10);
				  osMessageQueueReset(receive_queueHandle);
				  strcpy(recibido, "\0\0\0");
				  goto restart_loop;
			  }
			  printf("recibido= %s\r\n",recibido);
			  *toChange[i] = strtol(recibido, NULL, 16);
			  osMessageQueueReset(receive_queueHandle);
			  strcpy(recibido, "\0\0\0");

		  } else if (flag_rec == 0x0002U){
			  HAL_UART_Transmit(&huart1, (uint8_t *)msg_error, strlen(msg_error), 10);
			  osMessageQueueReset(receive_queueHandle);
			  goto restart_loop;
		  }
	  }
	  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
	  HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD);
	  HAL_UART_Transmit(&huart1, (uint8_t *)msg_hora_ok, strlen(msg_hora_ok), 10);
	  HAL_UART_Transmit(&huart1, (uint8_t *)msg_fecha_ok, strlen(msg_fecha_ok), 10);
	  osThreadFlagsSet(wifi_setHandle, 0x0001U);
	  config_state=1; // Para que en la funcion HAL_UART_RxCpltCallback se encargue de configurar el wifi en vez de RTC
	  osThreadSuspend(RTC_setHandle);
	}
  /* USER CODE END RTC_set_func */
}

/* USER CODE BEGIN Header_wifi_set_func */
/**
* @brief Function implementing the wifi_set thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_wifi_set_func */
void wifi_set_func(void *argument)
{
  /* USER CODE BEGIN wifi_set_func */
	char recibido[31] = {0};
	uint8_t ret_flag;
	uint32_t flag_rec = 0x0000U;
	uint8_t bandera_cola = 1;
	osStatus_t estado;
	char rec;
	const char* msg_ssid_ok = "\r\nSSID cambiado\r\n";
	const char* msg_clave_ok = "Clave cambiada\r\n";
	const char* msg_error = "\r\nERROR: Valor no válido\r\n";
	const char* msg_rtc1 = "\r\n\r\n========================\r\n" "| Configurar wifi |\r\n" "========================\r\n\r\n";
	const char* msg[2] = {"SSID: ", "\r\nClave: "};
	uint8_t contador_wifi = 0;
  /* Infinite loop */
  for(;;)
  {
	  ret_flag = osThreadFlagsWait(0x0001U, osFlagsWaitAny, osWaitForever);
	  if (ret_flag == 1U) {
		  restart_loop:
		  HAL_UART_Transmit(&huart1, (uint8_t *)msg_rtc1, strlen(msg_rtc1), 10);
		  for (int i = 0; i < 2; i++){
			  HAL_UART_Transmit(&huart1, (uint8_t *)msg[i], strlen(msg[i]), 10);
			  HAL_UART_Receive_IT(&huart1, &rec_data, sizeof(rec_data));
			  flag_rec = osThreadFlagsWait(0x00000003U, osFlagsWaitAny, osWaitForever);
			  if ( flag_rec == 0x0001U){
				  printf("Salto de linea pulsado, bandera 0 recibida\r\n");
				  while(bandera_cola) {
					  estado = osMessageQueueGet(receive_wifi_queueHandle, &rec, NULL, pdMS_TO_TICKS(500));
					  if (estado == osOK){
						  if (rec == '\n' || rec == '\r') {
							  recibido[contador_wifi] = '\0';
							  bandera_cola = 0;
						  }
						  else
							  recibido[contador_wifi] = rec;
						  contador_wifi++;
					  }
				  }
				  contador_wifi = 0;
				  printf("recibido= %s\r\n",recibido);
				  if (i==0)
					  strcpy(SSID, recibido);
				  else if (i==1)
					  strcpy(PASSWORD, recibido);
				  osMessageQueueReset(receive_wifi_queueHandle);
				  bandera_cola = 1;
				  for (int j=0;j<=31;j++){
					  recibido[j]='\0';
				  }

			  } else if (flag_rec == 0x0002U){
				  HAL_UART_Transmit(&huart1, (uint8_t *)msg_error, strlen(msg_error), 10);
				  osMessageQueueReset(receive_queueHandle);
				  goto restart_loop;
			  }
		  }

		  printf("SSID: %s | PASSWORD: %s",SSID, PASSWORD);
		  HAL_UART_Transmit(&huart1, (uint8_t *)msg_ssid_ok, strlen(msg_ssid_ok), 10);
		  HAL_UART_Transmit(&huart1, (uint8_t *)msg_clave_ok, strlen(msg_clave_ok), 10);
		  osThreadFlagsSet(wifiStartHandle, 0x0001U);
		  bandera_cola = 0;
		  osThreadSuspend(wifi_setHandle);
	  }
  }
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
