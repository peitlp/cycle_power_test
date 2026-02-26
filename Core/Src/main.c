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
  * Code written for AMTEC.PRO by Primož Peitl
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DO_NOCURRENT_TEST 0
#define NO_CURRENT_ADC_VAL			1600
#define CURRENT_PRESENT_ADC_VAL		1700
#define ENABLE_IDLE_NTC_TOGGLE		0

//output pins
#define RUNNING				GPIO_PIN_0		//GPIOF
#define NTC_BANK			GPIO_PIN_8		//GPIOF
#define GEN_NO_ERROR		GPIO_PIN_7		//GPIOF
#define GEN_WARNING			GPIO_PIN_2		//GPIOD
#define CH2_TRIGGER			GPIO_PIN_9		//GPIOF
#define DEBUG_OFF_OUTPUT	0


//input pins


//Errors
#define LOW_CURRENT_ERROR	1


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */



/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

ETH_HandleTypeDef heth;

TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
volatile int activeCount 	= 0;

int i;
int cntDly 					= 0;
int my_HAL_Delay 			= 1;
float voltageSense;
int shuntValues[11];
int StartSet 				= 0;
int CurrADC_Val				= 0;
int cntNr [11];
int CycleNr 				= 0;
int NrOfADCs 				= 0;
int currVals[2000];
int curValsCntr 			= 0;
int currValsMax, currValsMin;
long  	IdleCounter 		= 0;
long  	OnStartDelay 		= 0;
int 	CH2PulsCounter		= 0;
uint32_t CH1PulsCounter		= 0;
uint32_t CH1NegCounter		= 0;
volatile uint32_t intCH1PulsCounter	= 0;
volatile uint32_t intCH1NegCounter 	= 0;
volatile int32_t intDetected 		= 0;
volatile uint32_t startCNT;
uint32_t startNTCtoggle	= 0;
GPIO_PinState prev_CH1;
GPIO_PinState prev_CH2;
GPIO_PinState current_CH1;
GPIO_PinState current_CH2;
GPIO_PinState current_sense;
GPIO_PinState StartProcess;
GPIO_PinState GPTestEnabled;
GPIO_PinState RstBlockValues;

GPIO_PinState PLCClk;
GPIO_PinState PLCResData;
GPIO_PinState prev_PLCClock;
GPIO_PinState prev_PLCResData;

GPIO_PinState PLCClk_GP		= 0;
GPIO_PinState PLCResData_GP	= 0;
GPIO_PinState prev_PLCClock_GP;
GPIO_PinState prev_PLCResData_GP;

uint32_t ADC_currentPresent_val;
uint32_t ADC_noCurrent_val;

int PLCClockCounter_val;
uint16_t PLC_ClockIdleTicks = 0;
uint16_t ValuesPLCResData 	= 0x3ff;

int PLCClockCounterGP_val;
uint16_t GP_Cycles_tmp 		= 0;
uint16_t GP_Cycles			= 0;
uint32_t PLC_ClockIdleTicks_GP;

int FalseLowCurrentCounter;
int ErrorNr = 0;

unsigned int isPG0low 		= 0;
char myString[50];
char msg[255];
unsigned int FalsePG1;
unsigned int NrOfChanges_PG1;
unsigned char TestRunning;
unsigned int Counter1, Counter2,Counter3;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM14_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */
void ToggleNTCBank();
int CheckIdleCurrent();
void IdleCurrentMeasurementTest();
void DWT_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// ----------------------------------------------------------------------------
// communication between PLC and MPU
// which parts are good and which are not
// ----------------------------------------------------------------------------
void BadPartDetect() {
uint16_t tmpVal;
	PLCClk		= HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_1);			//PLC Clock
	PLCResData	= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3);			//PLC resistor data

	if (PLCClockCounter_val > 9) PLCClockCounter_val = 0;

	//detect clock rise signal
	if (prev_PLCClock == GPIO_PIN_RESET && PLCClk == GPIO_PIN_SET)
	{
		if (PLCResData == GPIO_PIN_RESET) {
			tmpVal = (1 << PLCClockCounter_val);
			tmpVal = ~tmpVal;
			ValuesPLCResData &= tmpVal;
		}
		PLCClockCounter_val++;
	}

	// longer PLCClk reset means reset for PLCClockCounter_val
	if (PLCClk == GPIO_PIN_RESET) PLC_ClockIdleTicks++;
	else PLC_ClockIdleTicks = 0;

	if (PLC_ClockIdleTicks >= 10000) {
		/*
		if ((PLC_ClockIdleTicks == 10000) && (ValuesPLCResData != 0x3FF)) {
			sprintf(msg, "PLCClockCounter_val: %d, ValuesPLCResData:%d \r\n ",PLCClockCounter_val,ValuesPLCResData);
			HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		}
		*/
		PLC_ClockIdleTicks = 10001;
		PLCClockCounter_val = 0;
	}

	prev_PLCClock = PLCClk;
	prev_PLCResData = PLCResData;
}

void GPTest_NrOfCycles() {
	PLCClk_GP		= HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_1);				//PLC Clock
	PLCResData_GP	= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3);				//PLC resistor data
	if (PLCClockCounterGP_val > 9) PLCClockCounterGP_val = 0;
	//detect clock rise signal
	if (prev_PLCClock_GP == GPIO_PIN_RESET && PLCClk_GP == GPIO_PIN_SET)
	{
		//GP_Cycles_tmp = GP_Cycles_tmp + 2^PLCClockCounterGP_val;
		if (PLCResData_GP == GPIO_PIN_SET) {
			GP_Cycles_tmp = GP_Cycles_tmp + (uint16_t)(1 << PLCClockCounterGP_val);
			//GP_Cycles_tmp += 1;
			//uint16_t GP_Cycles = 0;
		}
		PLCClockCounterGP_val++;
	}

	// longer PLCClk reset means reset for PLCClockCounter_val
	if (PLCClk_GP == GPIO_PIN_RESET) PLC_ClockIdleTicks_GP++;
	else PLC_ClockIdleTicks_GP = 0;

	if (PLC_ClockIdleTicks_GP >= 10000) {
		if (PLC_ClockIdleTicks_GP == 10000){
			GP_Cycles = GP_Cycles_tmp;
			GP_Cycles_tmp = 0;
			sprintf(msg, "GP_Cycles: %d\r\n ",GP_Cycles);
			HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		}
		PLC_ClockIdleTicks_GP = 10001;
		PLCClockCounterGP_val = 0;
	}

	prev_PLCClock_GP = PLCClk_GP;
	prev_PLCResData_GP = PLCResData_GP;
}



// ----------------------------------------------------------------------------
// ADC read task
// ----------------------------------------------------------------------------
void ADCReadTask(){
	//start ADC conversion every 20 cycles
	if (cntDly % 20 == 0) {
		HAL_ADC_Start(&hadc1);

		HAL_StatusTypeDef myADC_Status 								= HAL_ADC_PollForConversion(&hadc1 , 100);
		if 		(myADC_Status == HAL_OK )			CurrADC_Val 	+= HAL_ADC_GetValue(&hadc1);
		else if (myADC_Status == HAL_TIMEOUT) 		CurrADC_Val 	= 9999;
		else 										CurrADC_Val 	= 9998;

		NrOfADCs++;
		shuntValues[activeCount] = CurrADC_Val / NrOfADCs;
		HAL_ADC_Stop(&hadc1);
	}
	cntNr[activeCount] = cntDly;
	cntDly++;
	return;
}

void ReadADC() {
	if (prev_CH1 == GPIO_PIN_RESET) {
		cntDly 			= 0;
		CurrADC_Val 	= 0;
		NrOfADCs 		= 0;
	}
	ADCReadTask();
	return;
}

void ReadADC_Low() {
	if (prev_CH1 == GPIO_PIN_SET) {
		cntDly 			= 0;
		CurrADC_Val 	= 0;
		NrOfADCs 		= 0;
	}
	ADCReadTask();
	return;
}


uint32_t ReadADC_cont() {
uint32_t myADC_val;

	//start ADC conversion
	HAL_ADC_Start(&hadc1);
	HAL_StatusTypeDef myADC_Status = HAL_ADC_PollForConversion(&hadc1 , 100);
	if 		(myADC_Status == HAL_OK )			myADC_val = HAL_ADC_GetValue(&hadc1);
	else if (myADC_Status == HAL_TIMEOUT) 		myADC_val = 9999;
	else 										myADC_val = 9998;

	myADC_val = HAL_ADC_GetValue(&hadc1);

	HAL_ADC_Stop(&hadc1);
	if (curValsCntr >= 0 && curValsCntr < 2000) {
		currVals[curValsCntr] = myADC_val;
		curValsCntr++;
	}
	return myADC_val;
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

	/* Reset of all peripherals, Initialises the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialise all configured peripherals */
	MX_GPIO_Init();
	if (0) MX_ETH_Init();				// disabled
	MX_USART3_UART_Init();
	if (0) MX_USB_OTG_FS_PCD_Init();  // disabled
	if (0) MX_TIM14_Init();			// disabled
	MX_ADC1_Init();

	/* USER CODE BEGIN 2 */
	DWT_Init();
	/* USER CODE END 2 */


	/* USER CODE BEGIN WHILE */
	HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);
	sprintf(msg, "BOOTING \r\n");
	HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	ADC_currentPresent_val 	= CURRENT_PRESENT_ADC_VAL;		// store constant values to variables
	ADC_noCurrent_val 		= NO_CURRENT_ADC_VAL;
	StartSet = 1;												// low start state must be detected before
															// before doing first start
	HAL_GPIO_WritePin(GPIOF, CH2_TRIGGER, GPIO_PIN_RESET);	// reset trigger for CH2

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 	GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 	GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 	GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, 	GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 	GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, 	GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 	GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, 	GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, 	GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, 	GPIO_PIN_RESET);
	activeCount = 0;		//select first IGBT. It will switch on after start signal
/* Infinite loop */
	startNTCtoggle	= HAL_GetTick();
	while (1) {
//detect input signals
		current_CH1 	= HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_1);			//CH1 from generator
		current_CH2 	= HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_0);			//CH2 from generator
		StartProcess 	= HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_2);			//Start pulse from PLC
		GPTestEnabled	= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6);			//GP signal from PLC
		RstBlockValues	= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1);			//Start pulse from PLC

//detect start pulse from PLC
		if (OnStartDelay++ > 20000) OnStartDelay = 20000;
		if (StartProcess == GPIO_PIN_SET && StartSet == 0 && OnStartDelay > 10000)
		{
			HAL_GPIO_WritePin(GPIOF, CH2_TRIGGER, GPIO_PIN_SET);	//trigger ch2
			HAL_GPIO_WritePin(GPIOF, NTC_BANK, GPIO_PIN_RESET);		//reset temperature bank select pin
			StartSet 		= 1;
			ErrorNr 		= 0;											//reset errors when start pulse
			Counter1++;
			CH2PulsCounter 	= 0;
			startNTCtoggle	= HAL_GetTick();
		}
		else if (StartProcess == GPIO_PIN_RESET && StartSet == 1)
		{
			HAL_GPIO_WritePin(GPIOF, CH2_TRIGGER, GPIO_PIN_RESET);	//reset trigger for CH2
			StartSet = 0;
			Counter2++;
		}
// this happens only at the start of the cycle
// if cyclecount > 1 this doesn't occurs
		if  (StartProcess == GPIO_PIN_RESET && GPTestEnabled == GPIO_PIN_SET)
		{
			ValuesPLCResData = 0x3ff;
			intCH1PulsCounter = 0;
		}
		if (RstBlockValues)
		{
			ValuesPLCResData = 0x3ff;
			intCH1PulsCounter = 0;
		}
		if (Counter1 > 65000) Counter1=0;
		if (Counter2 > 65000) Counter2=0;

//detect not allowed input signals
		if (current_CH1 == GPIO_PIN_SET && (current_CH2 == GPIO_PIN_RESET || StartProcess == GPIO_PIN_RESET)) FalsePG1++;
		if (prev_CH1 != current_CH1) NrOfChanges_PG1++;

// toggle NTC bank
		if ((HAL_GetTick() - startNTCtoggle >= 500) && (StartProcess == GPIO_PIN_SET))
		{
			startNTCtoggle += 500;
			HAL_GPIO_TogglePin(GPIOF, NTC_BANK);
			CH2PulsCounter++;
			sprintf(msg, "%u \r\n",  CH2PulsCounter);
			HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		}


		if(current_CH2 == GPIO_PIN_SET && StartProcess == GPIO_PIN_SET && ErrorNr == 0 && OnStartDelay > 10000) {
			//this is done in interrupt
		}
		else {
	// switch off outputs
	// interrupt hasen't occurred for a intDetected cycles
			intDetected--;
			int32_t t_initDetect = intDetected;
			if (t_initDetect == 0) {
				__disable_irq();
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, 	GPIO_PIN_RESET);
				activeCount = 0; 											//select first IGBT. It will switch on after start signal
				__enable_irq();
				uint32_t end = DWT->CYCCNT;
				uint32_t cycles = end - startCNT;
				uint32_t us = cycles / (SystemCoreClock / 1000000U);
				sprintf(msg, "Reset IGBTs: %1u\r\n", us);
				HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
			}
			else if (t_initDetect < 0) {
				intDetected = -1;
			//reset output RUNNING
				IdleCounter++;

				if (IdleCounter == 42000) {
					uint32_t end = DWT->CYCCNT;
					uint32_t cycles = end - startCNT;
					uint32_t us = cycles / (SystemCoreClock / 1000000U);
					HAL_GPIO_WritePin(GPIOF, RUNNING,	GPIO_PIN_RESET);
					sprintf(msg, "Reset RUNNING: %1u\r\n", us);
					HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
				}
				if (IdleCounter > 100000) IdleCounter = 100000;

		//toggle NTC bank in idle state - if enabled
				if (ENABLE_IDLE_NTC_TOGGLE) ToggleNTCBank();

				//write low level values - test
				if (DO_NOCURRENT_TEST) IdleCurrentMeasurementTest();
			}
		}
	 // call bad part detection procedure
		if (!GPTestEnabled)
		{
			BadPartDetect();
		}
		// check for GP test nr of cycles
		if (StartProcess == GPIO_PIN_RESET && GPTestEnabled ) {
			GPTest_NrOfCycles();
		}

		// remember state of inputs
		 prev_CH1 = current_CH1;
		 prev_CH2 = current_CH2;
		 CH1PulsCounter = intCH1PulsCounter;
		 CH1NegCounter 	= intCH1NegCounter;
	}
  /* USER CODE END 3 */
}


void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin)	//Interrupt for GPIO pins
{
	GPIO_PinState	EXTI_current_CH1 	= HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_1);			//CH1 from generator
	GPIO_PinState 	EXTI_current_CH2 	= HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_0);			//CH2 from generator
	GPIO_PinState 	EXTI_StartProcess 	= HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_2);			//Start pulse from PLC

	//if (EXTI_current_CH1 == GPIO_PIN_SET) intCH1PulsCounter++;
	intDetected = 40000;
	startCNT = DWT->CYCCNT;

	if(GPIO_Pin == GPIO_PIN_1 && EXTI_current_CH1 == GPIO_PIN_SET && EXTI_current_CH2 == GPIO_PIN_SET && EXTI_StartProcess == GPIO_PIN_SET && ErrorNr == 0 && OnStartDelay > 10000) {
		 HAL_GPIO_WritePin(GPIOF, RUNNING,	GPIO_PIN_SET);
		 IdleCounter = 0;
		 curValsCntr = 0;
		 intCH1PulsCounter++;
	 //***************************************************
	 //************* Forcing all IGBTs to work
	 //***************************************************
		 if (intCH1PulsCounter == 1) ValuesPLCResData = 0x3ff;
	 //***************************************************
	 //***************************************************
	 //***************************************************
		 switch (activeCount) {
			case 0:
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13,	GPIO_PIN_SET && !(GPTestEnabled && CH2PulsCounter > 50) && (ValuesPLCResData & (1 << activeCount)));
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, 	GPIO_PIN_RESET);
				//sprintf(msg, "%03d, %d \r\n", EXTI_current_CH1 * 100 + EXTI_current_CH2 * 10 + EXTI_StartProcess, intCH1PulsCounter);
				//sprintf(msg, "%03u \r\n",  intCH1PulsCounter);
				//HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

				break;
			case 1:
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 	GPIO_PIN_SET && !(GPTestEnabled && CH2PulsCounter > 70) && (ValuesPLCResData & (1 << activeCount)));
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, 	GPIO_PIN_RESET);
				break;
			case 2:
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 	GPIO_PIN_SET && !(GPTestEnabled && CH2PulsCounter > 90) && (ValuesPLCResData & (1 << activeCount)));
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, 	GPIO_PIN_RESET);
				break;
			case 3:
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, 	GPIO_PIN_SET && !(GPTestEnabled && CH2PulsCounter > 110) && (ValuesPLCResData & (1 << activeCount)));
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, 	GPIO_PIN_RESET);
				break;
			case 4:
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 	GPIO_PIN_SET && !(GPTestEnabled && CH2PulsCounter > 130) && (ValuesPLCResData & (1 << activeCount)));
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, 	GPIO_PIN_RESET);
				break;
			case 5:
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, 	GPIO_PIN_SET && !(GPTestEnabled && CH2PulsCounter > 150) && (ValuesPLCResData & (1 << activeCount)));
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, 	GPIO_PIN_RESET);
				break;
			case 6:
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 	GPIO_PIN_SET && !(GPTestEnabled && CH2PulsCounter > 170) && (ValuesPLCResData & (1 << activeCount)));
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, 	GPIO_PIN_RESET);
				break;
			case 7:
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, 	GPIO_PIN_SET && !(GPTestEnabled && CH2PulsCounter > 190) && (ValuesPLCResData & (1 << activeCount)));
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, 	GPIO_PIN_RESET);
				break;
			case 8:
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, 	GPIO_PIN_SET && !(GPTestEnabled && CH2PulsCounter > 210) && (ValuesPLCResData & (1 << activeCount)));
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, 	GPIO_PIN_RESET);
				break;
			case 9:
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, 	GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, 	GPIO_PIN_SET && !(GPTestEnabled && CH2PulsCounter > 230) && (ValuesPLCResData & (1 << activeCount)));
				break;
			default:
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 	GPIO_PIN_RESET); 	//
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 	GPIO_PIN_RESET);  	//
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 	GPIO_PIN_RESET); 	//
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, 	GPIO_PIN_RESET); 	//
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 	GPIO_PIN_RESET);  	//
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, 	GPIO_PIN_RESET);  	//
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 	GPIO_PIN_RESET);  	//
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, 	GPIO_PIN_RESET);  	//
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, 	GPIO_PIN_RESET);  	//
				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, 	GPIO_PIN_RESET);  	//
				break;
		 }
	}
	else {
		activeCount++;
		cntDly = 0;
		if (activeCount == 10) {
			activeCount = 0;
		}

		 //CH1 = low, CH2 = high, Start = HIGH
		 HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 	GPIO_PIN_RESET); //
		 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 	GPIO_PIN_RESET);  //
		 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 	GPIO_PIN_RESET); //
		 HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, 	GPIO_PIN_RESET); //
		 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 	GPIO_PIN_RESET);  //
		 HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, 	GPIO_PIN_RESET);  //
		 HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 	GPIO_PIN_RESET);  //
		 HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, 	GPIO_PIN_RESET);  //
		 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, 	GPIO_PIN_RESET);  //
		 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, 	GPIO_PIN_RESET);  //

	}
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 6;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hadc1.Instance 					= ADC1;
  hadc1.Init.ClockPrescaler 		= ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution 			= ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode 			= ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode 	= DISABLE;
  hadc1.Init.DiscontinuousConvMode 	= DISABLE;
  hadc1.Init.ExternalTrigConvEdge 	= ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv 		= ADC_SOFTWARE_START;
  hadc1.Init.DataAlign 				= ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion 		= 1;
  hadc1.Init.DMAContinuousRequests 	= DISABLE;
  hadc1.Init.EOCSelection 			= ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance 				= ETH;
  heth.Init.AutoNegotiation 	= ETH_AUTONEGOTIATION_ENABLE;
  heth.Init.Speed 				= ETH_SPEED_100M;
  heth.Init.DuplexMode 			= ETH_MODE_FULLDUPLEX;
  heth.Init.PhyAddress 			= LAN8742A_PHY_ADDRESS;
  MACAddr[0] 					= 0x00;
  MACAddr[1] 					= 0x80;
  MACAddr[2] 					= 0xE1;
  MACAddr[3] 					= 0x00;
  MACAddr[4]					= 0x00;
  MACAddr[5] 					= 0x00;
  heth.Init.MACAddr 			= &MACAddr[0];
  heth.Init.RxMode 				= ETH_RXPOLLING_MODE;
  heth.Init.ChecksumMode 		= ETH_CHECKSUM_BY_HARDWARE;
  heth.Init.MediaInterface 		= ETH_MEDIA_INTERFACE_RMII;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance 				= TIM14;
  htim14.Init.Prescaler 		= 7200 -1 ;
  htim14.Init.CounterMode 		= TIM_COUNTERMODE_UP;
  htim14.Init.Period 			= 10000;
  htim14.Init.ClockDivision 	= TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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
  huart3.Instance 					= USART3;
  huart3.Init.BaudRate 				= 115200;
  huart3.Init.WordLength			= UART_WORDLENGTH_8B;
  huart3.Init.StopBits 				= UART_STOPBITS_1;
  huart3.Init.Parity 				= UART_PARITY_NONE;
  huart3.Init.Mode 					= UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl 			= UART_HWCONTROL_NONE;
  huart3.Init.OverSampling 			= UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling 		= UART_ONE_BIT_SAMPLE_DISABLE;
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
  hpcd_USB_OTG_FS.Instance 					= USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints 		= 6;
  hpcd_USB_OTG_FS.Init.speed 				= PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable 			= DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface 			= PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable 			= ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable 	= DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable 			= DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable 	= ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 	= DISABLE;
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

	  /* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOC_CLK_ENABLE();
	  __HAL_RCC_GPIOF_CLK_ENABLE();
	  __HAL_RCC_GPIOH_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();
	  __HAL_RCC_GPIOG_CLK_ENABLE();
	  __HAL_RCC_GPIOE_CLK_ENABLE();
	  __HAL_RCC_GPIOD_CLK_ENABLE();

	/* USER CODE BEGIN PIPO1 */
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 	GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, 	GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 	GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, 	GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 	GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, 	GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, 	GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_9, 	GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, 	GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, 	GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF, CH2_TRIGGER, 	GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF, NTC_BANK, 		GPIO_PIN_RESET);
	/* USER CODE END PIPO1 */

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0|GPIO0_Pin|GPIO3_Pin|GPIO5_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOE, GPIO9_Pin|GPIO8_Pin|GPIO1_Pin|GPIO2_Pin
	                          |GPIO4_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOG, USB_PowerSwitchOn_Pin|GPIO7_Pin|GPIO6_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);

	  /*Configure GPIO pin : USER_Btn_Pin */
	  GPIO_InitStruct.Pin = USER_Btn_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pins : PG0 PG1 USB_OverCurrent_Pin */
	  //GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|USB_OverCurrent_Pin;
	  GPIO_InitStruct.Pin = GPIO_PIN_0;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	  /*Configure GPIO pin : CH1_Input_interrupt_Pin */
	  GPIO_InitStruct.Pin 	= GPIO_PIN_1;
	  GPIO_InitStruct.Mode 	= GPIO_MODE_IT_RISING_FALLING;
	  GPIO_InitStruct.Pull 	= GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	  /*Configure GPIO pins : PF2 - start from PLC */
	  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_1;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	  /*Configure GPIO pin : PD0 */
	  GPIO_InitStruct.Pin = GPIO_PIN_0;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	  /*Configure GPIO pins : PE */
	  GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_6;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	  /*Configure GPIO pins : GPIO9_Pin GPIO8_Pin GPIO1_Pin GPIO2_Pin
	                           GPIO4_Pin */
	  GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_13;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	  /*Configure GPIO pins : PF0 GPIO0_Pin GPIO3_Pin GPIO5_Pin */
	  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9| GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 ;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	  /*Configure GPIO pins : USB_PowerSwitchOn_Pin GPIO7_Pin GPIO6_Pin */
	  GPIO_InitStruct.Pin = GPIO_PIN_9| GPIO_PIN_14;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
	  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  /*Configure GPIO pin : PD1 */
	  GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_0;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	  /* EXTI interrupt init*/
	  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

#if 0
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0|GPIO0_Pin|GPIO3_Pin|GPIO5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO9_Pin|GPIO8_Pin|GPIO1_Pin|GPIO2_Pin
                          |GPIO4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, USB_PowerSwitchOn_Pin|GPIO7_Pin|GPIO6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin 	= USER_Btn_Pin;
  GPIO_InitStruct.Mode	= GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull 	= GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PF0 GPIO0_Pin GPIO3_Pin GPIO5_Pin */
  GPIO_InitStruct.Pin 	= GPIO_PIN_0|GPIO0_Pin|GPIO3_Pin|GPIO5_Pin;
  GPIO_InitStruct.Mode 	= GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull 	= GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin 	= LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode 	= GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull 	= GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CH2_Input_Pin USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin 	= CH2_Input_Pin|USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode 	= GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull 	= GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : CH1_Input_interrupt_Pin */
  GPIO_InitStruct.Pin 	= CH1_Input_interrupt_Pin;
  GPIO_InitStruct.Mode 	= GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull 	= GPIO_NOPULL;
  HAL_GPIO_Init(CH1_Input_interrupt_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO9_Pin GPIO8_Pin GPIO1_Pin GPIO2_Pin
                           GPIO4_Pin */
  GPIO_InitStruct.Pin 	= GPIO9_Pin|GPIO8_Pin|GPIO1_Pin|GPIO2_Pin
                          |GPIO4_Pin;
  GPIO_InitStruct.Mode 	= GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull 	= GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_PowerSwitchOn_Pin GPIO7_Pin GPIO6_Pin */
  GPIO_InitStruct.Pin 	= USB_PowerSwitchOn_Pin|GPIO7_Pin|GPIO6_Pin;
  GPIO_InitStruct.Mode 	= GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull 	= GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PD0 */
  GPIO_InitStruct.Pin 	= GPIO_PIN_0;
  GPIO_InitStruct.Mode 	= GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull 	= GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD1 */
  GPIO_InitStruct.Pin 	= GPIO_PIN_1;
  GPIO_InitStruct.Mode 	= GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull 	= GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}
#endif
/* USER CODE BEGIN 4 */
void ToggleNTCBank()
{
	//toggle NTC banks for Dewesoft measurement
	if (IdleCounter >= 72000){
		IdleCounter = 12001;
	}
	else if (IdleCounter >= 42000){
		HAL_GPIO_WritePin(GPIOF, NTC_BANK, GPIO_PIN_SET);	//reset temperature bank select pin
	}
	else if (IdleCounter >= 12000){
		HAL_GPIO_WritePin(GPIOF, NTC_BANK, GPIO_PIN_RESET);	//reset temperature bank select pin
		CH2PulsCounter = 0;
		if (IdleCounter == 12000)  {
			sprintf(msg, "Counter1: %d, Counter2: %d \r\n", Counter1, Counter2);
			HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		}
	}
	return;
}

int CheckIdleCurrent() {
int tResult = 0;
	//check if current dropped under defined values
	//after IGBTs are switched off
	if (ReadADC_cont() > ADC_noCurrent_val) {
		FalseLowCurrentCounter++;
	}
	else {
		FalseLowCurrentCounter--;
	}
	if (FalseLowCurrentCounter > 20) {
		FalseLowCurrentCounter = 20;
		tResult = -1;
	}
	else if (FalseLowCurrentCounter < 0) {
		FalseLowCurrentCounter = 0;
		tResult = 1;
	}
	return tResult;
}

void IdleCurrentMeasurementTest() {
	if (curValsCntr == 2000) {
		currValsMax = currVals[0];
		currValsMin = currVals[0];
		for (i = 0; i < 1999; i++) {
			sprintf(msg, "%d\r\n ",currVals[i]);
			HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
			if (currVals[i] > currValsMax) currValsMax = currVals[i];
			if (currVals[i] < currValsMin) currValsMin = currVals[i];
			currVals[i] = 0;
		}
		sprintf(msg, "currValsMax = %d ; currValsMin= %d \r\n", currValsMax, currValsMin);
		HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		curValsCntr++;
	}
	return;
}

void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // enable trace
    DWT->LAR = 0xC5ACCE55;                            // unlock (F7!)
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;              // enable counter
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
