
/*  ____  ____      _    __  __  ____ ___
 * |  _ \|  _ \    / \  |  \/  |/ ___/ _ \
 * | | | | |_) |  / _ \ | |\/| | |  | | | |
 * | |_| |  _ <  / ___ \| |  | | |__| |_| |
 * |____/|_| \_\/_/   \_\_|  |_|\____\___/
 *                           research group
 *                             dramco.be/
 *
 *  KU Leuven - Technology Campus Gent,
 *  Gebroeders De Smetstraat 1,
 *  B-9000 Gent, Belgium
 *
 *         File: main.c
 *      Created: 2020-11-27
 *       Author: Guus Leenders
 *      Version: 0.2
 *
 *  Description: text
 *      some more text
 *
 */
 
/* Includes ------------------------------------------------------------------*/
#include "hw.h"
#include "low_power_manager.h"
#include "microFFT.h"
//#include "sigfox.h"
#include "lorawan.h"
#include "nbiot.h"
#include "energy.h"

#include "stm32l0xx_hal_wwdg.h"
#include "stm32l0xx_hal_rng.h"
#include "stm32l0xx_hal_gpio.h"
#include "stm32l0xx_hal_iwdg.h"

#include "hw_eeprom.h"

#include "mlm32l07x01.h"

#include "timeServer.h"
#include "vcom2.h"
//#include <stdint.h>

//#include <stdbool.h>
//#include <stdarg.h>
#include "spiram.h"
#include "main.h"
#include <stdlib.h>
//#include "arm_math.h"
//#include "arm_const_structs.h"
#define NBIOT
#define LORAWAN

#ifdef DEVICE_3
#define DEVICE_ID 1
#else
#define DEVICE_ID 0
#endif

#define USER_BUTTON_ALT_PIN                         GPIO_PIN_0
#define USER_BUTTON_ALT_GPIO_PORT                   GPIOA
//#define STDBY_ON
//#define DEBUG	
#define SEND_DELAY																	300*1000//300*1000
#define FFT_SIZE 256


// ---------------------------- GENERAL FUNCTIONS ---------------------------------
static void sendTest(void);
static void onTimerEvent(void *context);
static void onTimerResult(void *context);
void send_data_request_result( void );
static void sendResult(void);
static void send_data_irq(void * context);
static void user_button_init( void );
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void ADC_Select_CH0 (void);
static void ADC_Select_CH4 (void);
static void spiramschrijf1h(void);
static void spiramschrijf2h(void);
static void spiramprint(uint16_t len);
static void start_meting(void);
static void RemoveOffset(float* array, size_t array_len);
static void MX_IWDG_Init(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

// -------------------------- GENERAL VARIABLES ---------------------------------
static TimerEvent_t TxTimer;
static TimerEvent_t ResultTimer;

RNG_HandleTypeDef hrng;
WWDG_HandleTypeDef   WwdgHandle;
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;
TIM_HandleTypeDef htim2;
IWDG_HandleTypeDef hiwdg;
#define ADC_BUF_LEN 2560
uint16_t adc_buf[ADC_BUF_LEN];
uint8_t* ptr = (uint8_t *) &adc_buf;
char* ptrc = (char*) &adc_buf;
uint8_t ch = 0;

uint8_t zendstatu = 0;
uint8_t adci = 0;
uint8_t spii = 0;

uint16_t adres= 0;
uint16_t maxf = 0;
uint16_t minf = 4096;

//static arm_rfft_instance_q15 fft_instance;
//static q15_t output[FFT_SIZE*2]; //has to be twice FFT size
// -------------------------------- MAIN ---------------------------------------
int main( void ){


  HAL_Init(); 					// STM32 HAL library initialization
  SystemClock_Config(); // Configure the system clock  
  DBG_Init();   				// Configure the debug mode
	HW_Init();						// Configure the hardware
	#ifdef DEBUG
	vcom2_Init( NULL );
	#endif
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
  GPIO_InitTypeDef  GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET); 
	
//  BSP_LED_On(LED_GREEN);
//  BSP_LED_Init(LED_BLUE);
//  BSP_LED_Init(LED_GREEN);
//  BSP_LED_Init(LED_RED2);
	if(spii == 0){
		RAM_SPI_Init();
		begin(SEQ);
		spii =1;
	}
	


	LPM_SetOffMode(LPM_APPLI_Id, LPM_Disable);
	LPM_SetStopMode(LPM_APPLI_Id, LPM_Enable);
	
	 /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	
		 /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	
	
	
	/*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
//	
	#ifdef DEBUG
	PRINTF_LN("Started...");
	#endif

// 
//	arm_status status;
// 
//	status = arm_rfft_init_q15(&fft_instance, 256/*bin count*/, 0/*forward FFT*/, 1/*output bit order is normal*/);
	
//	if(__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != RESET){ 
//    PRINTF_LN("- Started from WDT");
//    __HAL_RCC_CLEAR_RESET_FLAGS();
//  }
//	
//	PRINTF_LN("- Setting up WDT");
//	WwdgHandle.Instance = WWDG;

//  WwdgHandle.Init.Prescaler = WWDG_PRESCALER_8;
//  WwdgHandle.Init.Window    = 80;
//  WwdgHandle.Init.Counter   = 127;

	#ifdef DEBUG
	PRINTF_LN("Initializing...");
	#endif
	
	// Set low power mode: stop mode (timers on)
	LPM_SetOffMode(LPM_APPLI_Id, LPM_Disable);
	LPM_SetStopMode(LPM_APPLI_Id, LPM_Enable);
	
	
	// Init button 
  user_button_init();										// Initialise user button
  
//	// Set timers for every 30 seconds (defined by SEND_DELAY in ms)
//	TimerInit(&TxTimer, onTimerEvent);
//	TimerInit(&ResultTimer, onTimerResult);
//	TimerSetValue(&TxTimer,  SEND_DELAY);
//	TimerSetValue(&ResultTimer,  SEND_DELAY/4);

	//sendTest();
	HW_GPIO_SetIrq( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 1, send_data_irq );

  while( 1 ){
		if(zendstatu == 1){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		zendstatu = 0;
		PRINTF_LN("sch idle if zenstatu 1");
		sendTest();
		}
    SCH_Run( ); 
  }
}
static void MX_IWDG_Init(void){

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 4000;
	hiwdg.Init.Window= 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}
void SCH_Idle( void ){
	if (AppProcessRequest == LORA_SET){
    AppProcessRequest = LORA_RESET; 				// Reset notification flag
		sendLoRaWAN(0);													// Send package
	}
	if (LoraMacProcessRequest == LORA_SET){
		LoraMacProcessRequest = LORA_RESET;		  // Reset notification flag
		LoRaMacProcess();
	}
	HAL_IWDG_Refresh(&hiwdg);
  BACKUP_PRIMASK();
  DISABLE_IRQ( );
	LPM_EnterLowPower();
  RESTORE_PRIMASK( );
}

static void sendTest(void){
	PRINTF_LN("Starting testing sequence...");
	LPM_SetOffMode(LPM_APPLI_Id, LPM_Disable);
	LPM_SetStopMode(LPM_APPLI_Id, LPM_Enable);

	HAL_IWDG_Refresh(&hiwdg);
	HW_GPIO_SetIrq( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 1, NULL ); // Disable irq to forbidd user to press button while transmitting
	PRINTF_LN("db waarde berekenen"); 
	uint16_t db = maxf-minf;
	PRINTF_LN("dbv : %d",maxf-minf);
	PRINTF_LN("ch : %d",ch);	
	if(ch == 0){
		db = 31.92*log10((float)db/4.74);
	}
	if(ch == 4){
		db = 31.92*log10((float)db/52.9);
	}	
	PRINTF_LN("db : %d",db);
	#ifdef LORAWAN
	PRINTF("3. LORAWAN \n");
	float outputfft[FFT_SIZE/2];
	float fft_Re[FFT_SIZE];
	float fft_Im[FFT_SIZE];
	float som = 0;
	for(uint8_t d = 0;d<5;d++){		
		read(2560*2*d,2560*2,ptr);
		for(uint8_t r = 0;r<9;r++){ 
	//		read(2560*2*r,2560*2,ptr);
			for (int i = 0; i < FFT_SIZE; i++) {
				fft_Re[i] = adc_buf[i+r*FFT_SIZE];
				PRINTF("%d,", (int)fft_Re[i]);
				
			}
			for (int i = 0; i < FFT_SIZE; i++) {
				fft_Im[i] = 0;
			}
			RemoveOffset(fft_Re, FFT_SIZE);
			FFT_Init(fft_Re,fft_Im,FFT_SIZE,8000);
			FFT_Compute(FFT_FORWARD);
			FFT_ComplexToMagnitude();		
			for (int i = 0; i < FFT_SIZE/2; i++) {
				//PRINTF("%d,", (int)fft_Re[i]);
				outputfft[i] = outputfft[i] + fft_Re[i];				
			}
		}
	}		
	for (int i = 0; i < FFT_SIZE/2; i++) {				
			som = som + outputfft[i];				
	}
	PRINTF_LN("");
	for (int i = 0; i < FFT_SIZE/2; i++)
	{
		outputfft[i] = outputfft[i]/som*(float)255;
//		PRINTF("%d,",(int)outputfft[i]);
	}
	PRINTF_LN("");


	uint8_t bufff[11];
	bufff[10] = db;
	for(uint8_t r = 1;r<10;r = r+2){
		bufff[r] = 0;
		for (int i = 0; i < FFT_SIZE/2; i++) {
			if(outputfft[i]>bufff[r]){
				bufff[r]=(int)outputfft[i];
				outputfft[i] = 0;
				bufff[r-1] = i;
			}		
		}
		PRINTF_LN("index %d  procent tov 256 %d",bufff[r-1],bufff[r]);
	}
	LoRaMacInitializationReset();
	sendLoRaWAN(bufff);
	#endif	
	#ifdef NBIOT
	PRINTF_LN("1. NB-IoT \n");
	char array[1449];
//	for (int i = 0; i < 1449; i++) {
//  array[i]=(char)72;
//	}
//	sendNBIoT(array);
	if(spii == 0){
		RAM_SPI_Init();
		begin(SEQ);
		spii =1;
	}
	sendNBIoTi();
	MX_IWDG_Init();
	HAL_IWDG_Refresh(&hiwdg);
	//uint64_t mul = ((uint64_t)(255*10000))/((uint64_t)(maxf-minf));
	float mul = ((float)(254))/((float)(maxf-minf));
	for(uint8_t r = 0;r<6;r++){		
		PRINTF_LN("pakket %d",r);
		read(1447*2*r,1447*2,ptr);		
		PRINTF_LN("max : %d",maxf);
		PRINTF_LN("min : %d",minf);
		PRINTF_LN("mul : %d",(uint8_t)((float)mul*(float)100));
		array[0] = r+1;
		array[1] = db;
		for (int i = 2; i < 1449; i++) {
			array[i]=(char)(uint8_t)(((float)(adc_buf[i-2]-minf))*mul)+1;
		}
		sendNBIoTs(array);
		HAL_IWDG_Refresh(&hiwdg);
		//HAL_Delay(2000);
	}
	sendNBIoTc();
	HAL_IWDG_Refresh(&hiwdg);
	#endif
	

	HAL_Delay(500);
	

	HW_GPIO_SetIrq( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 1, send_data_irq );
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
	 // Enable user to press button after transmittion
	PRINTF_LN("back to sleep");
//	SCH_RegTask(RESULT_TASK, sendResult);		  // Record send data task
//	TimerInit(&ResultTimer, onTimerResult);
//	TimerSetValue(&ResultTimer,  SEND_DELAY/4);
//	TimerStart(&ResultTimer); // Schedule next testing cycle
	LPM_SetOffMode(LPM_APPLI_Id, LPM_Disable);
	LPM_SetStopMode(LPM_APPLI_Id, LPM_Enable);

}
static void RemoveOffset(float* array, size_t array_len)
{
	// calculate the mean of vData
	float mean = 0;
	for (size_t i = 0; i < array_len; i++)
	{
		mean += array[i];
	}
	mean /= array_len;
	// Subtract the mean from vData
	for (size_t i = 0; i < array_len; i++)
	{
		array[i] -= mean;
	}
}
static void sendResult(void){
//	HW_GPIO_SetIrq( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 1, NULL ); // Disable irq to forbidd user to press button while transmitting
//	HW_GPIO_SetIrq( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 1, send_data_irq ); // Enable user to press button after transmittion
//	
//	SCH_RegTask(SEND_TASK, sendTest);		  // Record send data task
//	TimerInit(&TxTimer, onTimerEvent);
//	TimerSetValue(&TxTimer,  SEND_DELAY-SEND_DELAY/4);
//	TimerStart(&TxTimer); // Schedule next testing cycle
//	
//	PRINTF_LN("- Waiting for next schedule");
}

static void onTimerEvent(void *context){
	SCH_SetTask( SEND_TASK ); 
	
}

static void onTimerResult(void *context){
	send_data_request_result(); 
	
}

void send_data_request_result( void ){
  /* send task to background*/
  SCH_SetTask( RESULT_TASK );
}
void start_meting(void){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_IWDG_Refresh(&hiwdg);
	LPM_SetOffMode(LPM_APPLI_Id, LPM_Disable);
	LPM_SetStopMode(LPM_APPLI_Id, LPM_Disable);
	adres = 0;
	maxf = 0;
	minf = 5000;
	if (adci == 0){
		adci = 1;
		MX_DMA_Init();
		MX_ADC_Init();
		ADC_Select_CH0();
		MX_TIM2_Init();
		if(HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED) != HAL_OK)
									Error_Handler();
		HAL_Delay(500);
		//	PRINTF_LN("dma state: %d",HAL_DMA_GetState(&hdma_adc));	
		//	PRINTF_LN("dma error: %d",HAL_DMA_GetError(&hdma_adc));
//			PRINTF_LN("adc error: %d",HAL_ADC_GetState(&hadc));
	}
		//	PRINTF_LN("callibratie adc gedaan");
	HAL_ADC_Start_DMA(&hadc, (uint32_t*)adc_buf, ADC_BUF_LEN);
			//             Error_Handler();
	PRINTF_LN("DMA gestart");

  // start pwm generation
  if(HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4) != HAL_OK)
                Error_Handler();
	PRINTF_LN("timer 2 gestart");
}
void send_data_irq(void * context){
	PRINTF_LN("send data irq");
	 start_meting();
}
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = ENABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE; 
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }


}
void ADC_Select_CH0 (void)
{
	ch =0;
	ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_RANK_NONE;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
	if(HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED) != HAL_OK)
                Error_Handler();
}
void ADC_Select_CH4 (void)
{
	ch =4;
	ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_NONE;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = 1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
	if(HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED) != HAL_OK)
                Error_Handler();
}
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
 // htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}
static void MX_DMA_Init(void)
{
	PRINTF_LN("dma init");
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}
void HAL_ADC_ConvHalfCpltCallback (ADC_HandleTypeDef * hadc){



	if(adres ==0){
		PRINTF_LN("calibreren channel adc");
		HW_GPIO_SetIrq( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 1, NULL );
		uint16_t min = 4000;
		uint16_t max = 0;
		for (uint16_t i =0;i<ADC_BUF_LEN/2;i++)
		{
			 if (adc_buf[i] > max)
			 {
					max = adc_buf[i];
			 }
			 else if (adc_buf[i] < min)
			 {
					min = adc_buf[i];
			 }
		}
		PRINTF_LN("verschil %d" , max-min); 
		PRINTF_LN("ch : %d",ch);	
		if(max - min < 320 & ch == 0){ //320  20
			HAL_ADC_Stop_DMA(hadc);
		// start pwm generation
			if(HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4) != HAL_OK){
									Error_Handler();}
			ADC_Select_CH4();
			PRINTF_LN("CH4");
			HAL_ADC_Start_DMA(hadc, (uint32_t*)adc_buf, ADC_BUF_LEN);
			if(HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4) != HAL_OK)
										Error_Handler();		
		}
		else if(max - min > 3800 & ch == 4){ //3800  237
			HAL_ADC_Stop_DMA(hadc);
		// start pwm generation
			if(HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4) != HAL_OK){
									Error_Handler();}
			ADC_Select_CH0();	
			PRINTF_LN("CH0");
			HAL_ADC_Start_DMA(hadc, (uint32_t*)adc_buf, ADC_BUF_LEN);
			if(HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4) != HAL_OK)
										Error_Handler();
		}
		else{				
			spiramschrijf1h();
		}
	}
	else{
		PRINTF_LN("spis1");
		spiramschrijf1h();
		for (uint16_t i =0;i<ADC_BUF_LEN/2;i++)
		{
			 if (adc_buf[i] > maxf)
			 {
					maxf = adc_buf[i];
			 }
			 else if (adc_buf[i] < minf)
			 {
					minf = adc_buf[i];
			 } 
		}
	}	
		
}
void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef * hadc)
{	
	PRINTF_LN("spis2");
	spiramschrijf2h();
	for (uint16_t i =ADC_BUF_LEN/2;i<ADC_BUF_LEN;i++)
	{
		 if (adc_buf[i] > maxf)
		 {
				maxf = adc_buf[i];
		 }
		 else if (adc_buf[i] < minf)
		 {
				minf = adc_buf[i];
		 }
	}
	if (adres >=35000){
		HAL_ADC_Stop_DMA(hadc);
		// start pwm generation
		if(HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4) != HAL_OK)
									Error_Handler();
		PRINTF_LN("meting gedaan");	
		PRINTF_LN("Perfect om door te sturen");
		LPM_SetOffMode(LPM_APPLI_Id, LPM_Disable);
		LPM_SetStopMode(LPM_APPLI_Id, LPM_Enable);
		//spiramprint(16000);
		HW_GPIO_SetIrq( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 1, send_data_irq );
		zendstatu = 1;
	}
	
}

void user_button_init( void ){

  GPIO_InitTypeDef initStruct={0};

  initStruct.Mode =GPIO_MODE_IT_RISING;
  initStruct.Pull = GPIO_PULLUP;
  initStruct.Speed = GPIO_SPEED_HIGH;
  HW_GPIO_Init( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, &initStruct );
  
  /* send everytime button is pushed */
  HW_GPIO_SetIrq( USER_BUTTON_GPIO_PORT, USER_BUTTON_PIN, 1, send_data_irq );
}
void spiramprint(uint16_t len){
	uint16_t index = 0;
	len = len*2;
	while( index < len){
		read(index,ADC_BUF_LEN*2,ptr);
		for (int i = 0; i < ADC_BUF_LEN; i++){
			PRINTF_LN("%d",adc_buf[i]);			
		}
		index = index + (ADC_BUF_LEN*2);		
	}
}
void spiramschrijf1h(void){
	if (spii == 0){
			RAM_SPI_Init();
			begin(SEQ);
			spii =1;
	}
	write(adres,ADC_BUF_LEN,ptr);
	adres = adres + ADC_BUF_LEN;
	//spiramprint(ADC_BUF_LEN);
}
void spiramschrijf2h(void){
	if (spii == 0){
			RAM_SPI_Init();
			begin(SEQ);
			spii =1;
	}
	write(adres,ADC_BUF_LEN,&ptr[ADC_BUF_LEN]);
	adres = adres + ADC_BUF_LEN;
	//spiramprint(ADC_BUF_LEN);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
		
    if(GPIO_Pin == GPIO_PIN_10) // If The INT Source Is EXTI Line9 (A9 Pin)
    {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11,GPIO_PIN_RESET);			// Toggle The Output (LED) Pin
			PRINTF_LN("wake mic");
			start_meting();
    }
		else if(GPIO_Pin == 4){
			PRINTF_LN("drukknop test");
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11,GPIO_PIN_RESET);	
			start_meting();
		}	
		else{
			PRINTF_LN("interupt pin: %d",GPIO_Pin);
		}
			
}

void Error_Handler(void)
{
  PRINTF_LN("Error_Handler\n\r");
	PRINTF_LN("dma state: %d",HAL_DMA_GetState(&hdma_adc));	
	PRINTF_LN("dma error: %d",HAL_DMA_GetError(&hdma_adc));
	PRINTF_LN("adc error: %d",HAL_ADC_GetState(&hadc));
  while(1);
}

