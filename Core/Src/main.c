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
#include "voice_proto.h"
#include "stdlib.h"//절대값(abs)
#include "stdbool.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum { ST_IDLE=0, ST_JOYSTICK, ST_VOICE } ctrl_state_t;

// 3축 데이터 구조체
typedef struct { uint16_t x,y,z; } triplet_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_MAX 4090    // 실제 최대값
#define ADC_MIN 0
#define ADC_NEU 2045	//ADC 중간값 4020/2
#define ADC_DEAD_ZONE 300	//데드존 처리

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t tx_address[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

volatile uint16_t adc_buffer[3] = {0};	//ADC가 변환한 x, y, z값을 DMA가 담는 버퍼 - x, y, z 3개
volatile uint8_t adc_conversion_complete = 0;
volatile uint32_t g_stop_until_ms = 0; // E-Stop 해제 시간
static uint32_t last_pb9_press_time = 0; // 디바운싱용

volatile uint8_t  adc_data_ready_flag = 0;
volatile uint32_t adc_cb_cnt = 0;

// ===== 음성 & 상태 머신 =====
static uint8_t rx3_byte = 0;              // USART3 1바이트 수신 버퍼
static volatile ctrl_state_t g_state = ST_IDLE;
static uint8_t last_cmd = 0x03;           // 기본 정지(STOP)

static const triplet_t VOICE_MAP[6] = {
/*0*/ {ADC_NEU,ADC_NEU,ADC_NEU},
/*1 FWD  */ {ADC_NEU,4090,ADC_NEU},
/*2 BACK */ {ADC_NEU,0,ADC_NEU},
/*3 STOP */ {ADC_NEU,ADC_NEU,ADC_NEU},
/*4 LEFT */ {0,ADC_NEU,ADC_NEU},
/*5 RIGHT*/ {4090,ADC_NEU,ADC_NEU},
};


uint8_t payload[6];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void nrf24_transmitter_setup(void);
void transmit_sensor_data(void);
static bool joystick_is_active(int x, int y, int z);
//static inline int  iabs_int(int v) { return v>=0? v : -v; };
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

nrf24_init();
nrf24_transmitter_setup();

// 음성 FSM + UART3 인터럽트 수신 시작
Voice_Init();
HAL_UART_Receive_IT(&huart3, &rx3_byte, 1);

HAL_ADCEx_Calibration_Start(&hadc1);

//타이머 인터럽트 시작 20ms마다
HAL_TIM_Base_Start_IT(&htim2);


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

	    __WFI(); // 저전력 대기(인터럽트가 깨움)
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
static bool joystick_is_active(int x, int y, int z) {
    return (abs(x - ADC_NEU) > ADC_DEAD_ZONE) ||
           (abs(y - ADC_NEU) > ADC_DEAD_ZONE) ||
           (abs(z - ADC_NEU) > ADC_DEAD_ZONE);
}

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
nrf24_auto_ack_all(0);                //자동 ack기능 off=>단순 송신만 수행
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

	// 1. 최신 ADC 값 안전하게 복사
	uint16_t local_adc_buffer[3];
	__disable_irq();
	memcpy(local_adc_buffer, (void*)adc_buffer, sizeof(adc_buffer));
	__enable_irq();


	uint16_t x = local_adc_buffer[0];
		uint16_t y = local_adc_buffer[1];
		uint16_t z = local_adc_buffer[2];

    // ★★★★★ E-Stop 로직 삽입 시작 ★★★★★
    uint32_t current_time = HAL_GetTick();

    if (g_stop_until_ms != 0) { // E-Stop 상태인가?
        if (current_time < g_stop_until_ms) {
            // [정지] 아직 3초가 안 지났으면: 강제 중립값(정지) 전송
            x = ADC_NEU;
            y = ADC_NEU;
            z = ADC_NEU;
            // E-Stop 중에는 상태 머신 로직을 스킵하고 바로 전송합니다.

            // [추가된 부분] 상태 머신과 마지막 명령을 초기화합니다!
                        g_state = ST_IDLE;      // 상태를 대기 모드로 강제 변경
                        last_cmd = 0x03;        // 마지막 명령을 STOP(0x03)으로 초기화

        } else {
            // [복귀] 3초가 지났으면: E-Stop 해제 및 상태 머신으로 복귀
            g_stop_until_ms = 0;
        }
    }
    // ★★★★★ E-Stop 로직 삽입 끝 ★★★★★


    // E-Stop 상태가 아니거나 E-Stop 시간이 끝났다면, 기존 로직을 실행
    if (g_stop_until_ms == 0) {

        // 2) 조이스틱 활성 판정
        bool active = joystick_is_active((int)x,(int)y,(int)z);

        // 3) (조이스틱 중립일 때만) 음성 프레임 소비
        if (!active && Voice_FrameAvailable()){
            voice_frame_t vf;
            __disable_irq();
            bool ok = Voice_TryPopFrame(&vf);
            __enable_irq();
            if (ok && vf.cmd >= 0x01 && vf.cmd <= 0x05){
                last_cmd = vf.cmd;
                g_state  = ST_VOICE;       // 음성 모드 진입
            }
        }

        // 4) 상태머신으로 이번 주기 전송값 결정
        uint16_t tx_x = ADC_NEU, tx_y = ADC_NEU, tx_z = ADC_NEU;

        switch (g_state)
        {
          case ST_IDLE:
            if (active) g_state = ST_JOYSTICK;
            break;

          case ST_JOYSTICK:
            if (!active){
              g_state = ST_IDLE;
            } else {
              tx_x = x; tx_y = y; tx_z = z;      // 조이스틱 값 그대로
            }
            break;

          case ST_VOICE:
          default:
            if (active){
              g_state = ST_JOYSTICK;            // 조이스틱 우선
              tx_x = x; tx_y = y; tx_z = z;
            } else {
              triplet_t t = VOICE_MAP[last_cmd];
              tx_x = t.x; tx_y = t.y; tx_z = t.z; // 음성 등가값
            }
            break;
        }

        // 5) 최종 전송할 페이로드를 E-Stop 로직 밖에서 설정합니다.
        // E-Stop 로직에서 x, y, z 값이 이미 결정되었으므로, E-Stop이 아닐 때만 tx_x, tx_y, tx_z를 사용합니다.
        x = tx_x; y = tx_y; z = tx_z;
    } // g_stop_until_ms == 0 (E-Stop 아닐 때) 끝

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

    // 6) 디버그 로그 (tx_x, tx_y, tx_z가 아닌 실제 전송된 x,y,z 값을 사용)
	const char* s = (g_stop_until_ms != 0) ? "E-STOP" : (g_state==ST_JOYSTICK)?"JOY":(g_state==ST_VOICE)?"VOICE":"IDLE";
	if (g_stop_until_ms != 0) {
        printf("TX[%s] STOP (until:%lu)\r\n", s, g_stop_until_ms);
	} else if (g_state==ST_VOICE) {
        printf("TX[%s] CMD:0x%02X | X:%u Y:%u Z:%u\r\n", s, last_cmd, x, y, z);
    } else {
        printf("TX[%s] X:%u Y:%u Z:%u\r\n", s, x, y, z);
    }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // GPIO_PIN_8 (E-Stop 버튼 핀)에서 인터럽트가 발생했는지 확인
    if(GPIO_Pin == GPIO_PIN_8) // 핀 번호가 맞는지 확인해주세요.
    {
    	printf("!!! E-STOP BUTTON PRESSED !!!\r\n");
        // --- 소프트웨어 디바운싱 (노이즈 제거) ---
        uint32_t current_time = HAL_GetTick();
        if (current_time - last_pb9_press_time < 300) // 0.3초 이내의 재입력은 무시
        {
            return; // 바운싱으로 간주하고 무시
        }
        last_pb9_press_time = current_time;
        // --- 디바운싱 끝 ---

        // E-Stop 발생: "3초 뒤" 시간을 g_stop_until_ms 변수에 저장
        g_stop_until_ms = current_time + 3000; // 3000ms = 3초 정지
    }
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    Voice_RxByteFromIRQ(rx3_byte);
    HAL_UART_Receive_IT(&huart3, &rx3_byte, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    HAL_UART_Receive_IT(&huart3, &rx3_byte, 1);
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

	__disable_irq();
	  while (1) { }

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
