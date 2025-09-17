/* USER CODE BEGIN Header */

/**

******************************************************************************

* @file : main.c

* @brief : Main program body

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
#include "dma.h"
#include "spi.h"
#include "tim.h"
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

/* USER CODE BEGIN PV */
//volatile uint16_t adc_values[3];
volatile uint8_t  adc_data_ready_flag = 0;
volatile uint32_t adc_cb_cnt = 0;
volatile uint16_t adc_buffer[3] = {0};	//ADC가 변환한 x, y, z값을 DMA가 담는 버퍼 - x, y, z 3개
volatile uint8_t adc_conversion_complete = 0;

uint8_t payload[6];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t tx_address[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};


void nrf24_transmitter_setup(void);
void transmit_sensor_data(void);

extern ADC_HandleTypeDef hadc1;
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_adc1;



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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */


HAL_ADCEx_Calibration_Start(&hadc1);
//타이머 인터럽트 시작 20ms마다
HAL_TIM_Base_Start_IT(&htim2);


nrf24_init();
nrf24_transmitter_setup();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

while (1)
{

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	 if(adc_conversion_complete){	//DMA가 메모리 저장을 완료하여 콜백함수에 의해 adc_conversion_complete = 1이 되어 조건이 참이된다면
			  transmit_sensor_data();
		  }
	  __WFI();



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
nrf24_defaults();
nrf24_pwr_up();
nrf24_flush_tx();
nrf24_flush_rx();
nrf24_clear_rx_dr();
nrf24_clear_tx_ds();
nrf24_clear_max_rt();
nrf24_stop_listen();                        //수신모드 비활성화 하여 송신 전용 모드로 전환
nrf24_set_channel(40);                      //무선 채널 40번으로 설정
nrf24_auto_ack_all(disable);                //자동 ack기능 off=>단순 송신만 수행
nrf24_set_payload_size(6);                  //한번에 전송할 페이로드 크기 최대 32바이트
nrf24_tx_pwr(3);
nrf24_data_rate(_1mbps);
nrf24_open_tx_pipe(tx_address);             //파이프 0에 tx_address를 열어 송신 대상 지정
nrf24_pwr_up();                             //모듈 power up=>송신 준비 완료

    uint8_t cfg = nrf24_r_reg(CONFIG, 1);
    cfg &= ~((1<<5)|(1<<4));                // MASK_TX_DS=5, MASK_MAX_RT=4 -> 0
    nrf24_w_reg(CONFIG, &cfg, 1);
}

int __io_putchar(int ch)
{
HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
return ch;
}



void transmit_sensor_data(void){
	adc_conversion_complete = 0;

	uint16_t local_adc_buffer[3];

	//adc_buffer값을 local_adc_buffer에 복사
	__disable_irq(); //복사중 모든 인터럽트 중지
	memcpy(local_adc_buffer, (void*)adc_buffer, sizeof(adc_buffer));
	__enable_irq(); //인터럽트 허용
/*memcpy함수는 메모리 특정 영역을 다른 영역으로 복사한다
 *함수 기본형태-void *memcpy(붙여넣을 메모리 시작주소, 복사할 내용이 있는 메모리 시작주소, sizeof(원본);
 *함수
 */

	uint16_t x = local_adc_buffer[0];
	uint16_t y = local_adc_buffer[1];
	uint16_t z = local_adc_buffer[2];

	//6바이트 2진 패킹
	uint8_t payload[6];
	payload[0] = (uint8_t)(x & 0xff);	//x하위8비트
	payload[1] = (uint8_t)(x >> 8);		//x상위8비트 시프트
	payload[2] = (uint8_t)(y & 0xff);	//y하위8비트
	payload[3] = (uint8_t)(y >> 8);		//y상위8비트
	payload[4] = (uint8_t)(z & 0xff);	//z하위8비트
	payload[5] = (uint8_t)(z >> 8);		//z상위8비트

	//최종 데이터 발송
	nrf24_transmit(payload, 6);

	//uart디버깅
	char dbg[64];
	int dlen = snprintf(dbg, sizeof(dbg), "X: %u | Y: %u | Z: %u\r\n", x, y, z);
	HAL_UART_Transmit(&huart2, (uint8_t*)dbg, dlen, 100);

}

//타이머가 만료될 때마다 호출되는 콜백함수 20ms주기
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // 이벤트가 발생한 타이머가 TIM2인지 확인
    if (htim->Instance == TIM2)
    {
    	//3개의 ADC값을 변환해서 그 결과를 adc_buffer에 DMA로 저장 시작
        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 3);
    }
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    // 이벤트가 발생한 ADC가 ADC1인지 확인
    if (hadc->Instance == ADC1)
    {
        // Main루프의 if문 조건이 참이 되어 transmit_sensor_data함수호출
        adc_conversion_complete = 1;
    }
}


//ADC변환 중 오류가 발생했을 때 자동으로 호출, DMA전송을 중지
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        // ADC 오류 발생 시, 진행 중이던 DMA를 안전하게 중지
        // 다음 타이머 주기(20ms 후)에 HAL_ADC_Start_DMA가 다시 호출되며 자동으로 복구를 시도함
        HAL_ADC_Stop_DMA(&hadc1);
    }
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
