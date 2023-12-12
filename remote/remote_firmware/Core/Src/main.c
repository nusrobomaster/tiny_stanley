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
#include "adc.h"
#include "spi.h"
#include "usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nrf24.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TOTAL_CHANNELS 3
#define X_DEAD_ZONE 300 
#define Y_DEAD_ZONE 200
#define T_DEAD_ZONE 200
#define ADC_MAX 4095
#define ADC_MIN 0
#define X_MID_BOUND 1770
#define Y_MID_BOUND 2023
#define T_MID_BOUND 2023
#define X_TOP_BOUND X_MID_BOUND + X_DEAD_ZONE
#define X_BOT_BOUND X_MID_BOUND - X_DEAD_ZONE
#define Y_TOP_BOUND Y_MID_BOUND + Y_DEAD_ZONE
#define Y_BOT_BOUND Y_MID_BOUND - Y_DEAD_ZONE
#define T_TOP_BOUND T_MID_BOUND + T_DEAD_ZONE
#define T_BOT_BOUND T_MID_BOUND - T_DEAD_ZONE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
struct __attribute__((__packed__)) remote_msg {
    uint8_t start;
    uint8_t button;
    int16_t x_axis;
    int16_t y_axis;
    int16_t twist;
};

union msg_tx {
    struct remote_msg deserial;
    uint8_t serial[8];
};

volatile union msg_tx msg_tx;
volatile uint8_t button_down = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    msg_tx.deserial.start = 0x01;
    msg_tx.deserial.button = 0x00;
    msg_tx.deserial.x_axis = 0x0000;
    msg_tx.deserial.y_axis = 0x0000;
    msg_tx.deserial.twist = 0x0000;
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
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */
    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_Delay(10);

    nRF24_Init();
    while(!nRF24_Check());

    nRF24_SetAddrWidth(5);
    uint8_t id[6] = {1, 0, 0, 0, 255, 0};
    nRF24_DisableAA(0xFF);
    nRF24_SetRFChannel(70);
    nRF24_SetTXPower(nRF24_TXPWR_0dBm);
    nRF24_SetDataRate(nRF24_DR_250kbps);
    nRF24_SetAddr(nRF24_PIPETX,id);
    nRF24_SetOperationalMode(nRF24_MODE_TX);
    nRF24_SetPowerMode(nRF24_PWR_UP);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    uint8_t adc_channel = 0;
    uint32_t timestamp = HAL_GetTick();

    uint8_t status = 0;
    while (1) {
        volatile int32_t x_axis, y_axis, twist; 
        adc_channel = 0;
        x_axis = 0;
        y_axis = 0;
        twist = 0;
        if (HAL_GetTick() - timestamp >= 5) {
            while(adc_channel < TOTAL_CHANNELS) {
                HAL_ADC_Start(&hadc1);
                HAL_ADC_PollForConversion(&hadc1, 1);

                switch(adc_channel) {
                    case 0:
                        x_axis = HAL_ADC_GetValue(&hadc1);
                        break;
                    case 1:
                        y_axis = HAL_ADC_GetValue(&hadc1);
                        break;
                    case 2:
                        twist = HAL_ADC_GetValue(&hadc1);
                        break;
                    default:
                        adc_channel = 4;
                };

                adc_channel += 1;
            }

            if (x_axis < X_BOT_BOUND) {
                msg_tx.deserial.x_axis = (x_axis - X_BOT_BOUND) * 1000/ X_BOT_BOUND;
            }
            else if (x_axis > X_TOP_BOUND) {
                msg_tx.deserial.x_axis = (x_axis - X_TOP_BOUND) * 1000 / X_BOT_BOUND;
            }
            else {
                msg_tx.deserial.x_axis = 0;
            }

            if (y_axis < Y_BOT_BOUND) {
                msg_tx.deserial.y_axis = (y_axis - Y_BOT_BOUND) * 1000 / Y_BOT_BOUND;
            }
            else if (y_axis > Y_TOP_BOUND) {
                msg_tx.deserial.y_axis = (y_axis - Y_TOP_BOUND) * 1000 / Y_BOT_BOUND;
            }
            else {
                msg_tx.deserial.y_axis = 0;
            }

            if (twist < T_BOT_BOUND) {
                msg_tx.deserial.twist = (twist - T_BOT_BOUND) * 1000 / T_BOT_BOUND;
            }
            else if (twist > T_TOP_BOUND) {
                msg_tx.deserial.twist = (twist - T_TOP_BOUND) * 1000 / T_BOT_BOUND;
            }
            else {
                msg_tx.deserial.twist = 0;
            }

            msg_tx.deserial.button = HAL_GPIO_ReadPin(JOY_BUTTON_GPIO_Port, JOY_BUTTON_Pin);

            timestamp = HAL_GetTick();
        }

        nRF24_WritePayload((uint8_t*) msg_tx.serial, sizeof(msg_tx.serial)); // pass data pointer
        nRF24_CE_H; // activate coms to nrf, starts tx
        while (1) {
            status = nRF24_GetStatus();
            if (status & (nRF24_FLAG_TX_DS | nRF24_FLAG_MAX_RT)) {
                break;
            }
        }
        nRF24_CE_L; // deactivate coms to nrf
        nRF24_ClearIRQFlags();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
