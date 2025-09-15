/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "NRF24.h"
#include "NRF24_conf.h"
#include "stdio.h"
#include "string.h"
#include "NRF24_reg_addresses.h"

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
extern ADC_HandleTypeDef hadc1;
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_ADC1_Init(void);
void MX_SPI1_Init(void);
void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t tx_address[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

void debug_dump_settings();
void debug_dump_settings();
void transmit_text();
static uint32_t ReadADC_Channel();
void nrf24_transmitter_setup();


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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */


  nrf24_init();

 // char buf[64];
  nrf24_transmitter_setup();
 // debug_dump_settings();
 //nrf24_dpl(enable);                  // ← 여기에 추가
 //nrf24_set_rx_dpl(0, enable);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // 1) X/Y/Z 읽기
	     uint16_t x = ReadADC_Channel(ADC_CHANNEL_10);   // 0~4095
	     uint16_t y = ReadADC_Channel(ADC_CHANNEL_11);
	     uint16_t z = ReadADC_Channel(ADC_CHANNEL_12);

	     // 2) 6바이트 이진 패킹
	     uint8_t payload[6];
	     payload[0] = (uint8_t)( x        & 0xFF);      // X LSB
	     payload[1] = (uint8_t)((x >> 8) & 0x0F);       // X MSB (12bit)
	     payload[2] = (uint8_t)( y        & 0xFF);      // Y LSB
	     payload[3] = (uint8_t)((y >> 8) & 0x0F);       // Y MSB
	     payload[4] = (uint8_t)( z        & 0xFF);      // Z LSB
	     payload[5] = (uint8_t)((z >> 8) & 0x0F);       // Z MSB

	     // 3) 무선 전송 (6바이트)
	     nrf24_transmit(payload, 6);

	     // 4) UART 디버그: ASCII 숫자만 보기
	     char dbg[64];
	     int dlen = snprintf(dbg, sizeof(dbg),
	                        "X:%u  Y:%u  Z:%u\r\n",
	                        x, y, z);
	     HAL_UART_Transmit(&huart2, (uint8_t*)dbg, dlen, HAL_MAX_DELAY);

	     // 5) 전송 속도 제어
	     HAL_Delay(10);
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
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void nrf24_transmitter_setup(void)
{
    nrf24_defaults();                //레지스터 기본값으로 초기화
    nrf24_pwr_up();
    nrf24_flush_tx();
    nrf24_flush_rx();
    nrf24_clear_rx_dr();
    nrf24_clear_tx_ds();
    nrf24_clear_max_rt();
    nrf24_stop_listen();             //수신모드 비활성화 하여 송신 전용 모드로 전환

    nrf24_set_channel(40);           //무선 채널 40번으로 설정
    nrf24_auto_ack_all(disable);     //자동 ack기능 off=>단순 송신만 수행
    nrf24_set_payload_size(6);      //한번에 전송할 페이로드 크기 최대 32바이트
    nrf24_tx_pwr(3);
    nrf24_data_rate(_1mbps);
    nrf24_open_tx_pipe(tx_address);  //파이프 0에 tx_address를 열어 송신 대상 지정
    nrf24_pwr_up();                  //모듈 power up=>송신 준비 완료

   // nrf24_dpl(enable);               //dpl활성화
     // nrf24_set_rx_dpl(0, enable);
}

void debug_dump_settings(void) {     //주소 확인 용
    uint8_t ch     = nrf24_r_reg(RF_CH, 1);
    uint8_t pw     = nrf24_r_reg(RX_PW_P0, 1);
    uint8_t addr[5];

    csn_low();
    uint8_t cmd = R_REGISTER | RX_ADDR_P0;
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
    HAL_SPI_Receive(&hspi1, addr, 5, 100);
    csn_high();

    printf("=== DEBUG SETTINGS ===\r\n");
    printf(" RF_CH       = %u\r\n", ch);
    printf(" RX_PW_P0    = %u bytes\r\n", pw);
    printf(" RX_ADDR_P0  = %02X %02X %02X %02X %02X\r\n",
           addr[0], addr[1], addr[2], addr[3], addr[4]);
    printf("======================\r\n");
}



int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}


void transmit_text(const char* msg) {
    nrf24_transmit((uint8_t*)msg, strlen(msg));
}


static uint32_t ReadADC_Channel(uint32_t channel) {

	   ADC_ChannelConfTypeDef sConfig = {0};              //채널 구성 구조체 초기화
	    sConfig.Channel      = channel;                    //변환할 채널 지정
	    sConfig.Rank         = ADC_REGULAR_RANK_1;
	    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5; //샘플링 타임 설정
	    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	    HAL_ADC_Start(&hadc1);                             //변환 시작
	    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);  //변환 완료 대기
	    uint32_t value = HAL_ADC_GetValue(&hadc1);         //변환 결과 읽기
	    HAL_ADC_Stop(&hadc1);
	    return value;

    return value;
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
#ifdef USE_FULL_ASSERT
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
