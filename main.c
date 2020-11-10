/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum{
	timeState,
	dateState,
	pastTimeState1,
	pastTimeState2,
	setState1,
	setState2,
	storeState,
	CNTState
}state_s;

typedef struct{
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
}time_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ACTIVE 1
#define INACTIVE 0


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define EEPROM_ADDRESS  0xA0
__IO HAL_StatusTypeDef Hal_status;
__IO uint16_t memLocation = 0x000A;
uint16_t EE_status;
char lcd_buffer[6];    // LCD display buffer
char timestring[10]={0};  
char datestring[6]={0};

state_s state;

/* Strings */

uint8_t wd, dd, mo, yy, ss, mm, hh; // for weekday, day, month, year, second, minute, hour
uint8_t mem_offset_w, recent, recent2; //variables for reading and writing times to memory
uint8_t param, modd;//variables to traverse date and time parameters and increment them appropriately
uint8_t setting[]={12,60,60,100,12,31,7};//mods for date and time parameters
uint8_t setData[]={0,0,0,0,0,0,0};//data for date and time

//char a[8]={"hh","mm","ss","yy","mo","dd","wd"};

char yearStr[] = {"Yr"};
char monStr[] = {"Mon"};
char dayStr[] = {"Day"};
char hrStr[] = {"hr"};
char minStr[] = {"min"};
char secStr[] = {"sec"};

__IO uint32_t SEL_Pressed_StartTick;   //sysTick when the User button is pressed
__IO uint8_t leftpressed, rightpressed, uppressed, downpressed, selpressed;  // button pressed 
__IO uint8_t  sel_held;   // if the selection button is held for a while (>750ms)

/* RTC Varaibles */

RTC_HandleTypeDef      RTCHandle;
RTC_DateTypeDef        RTC_DateStructure;
RTC_TimeTypeDef        RTC_TimeStructure;
I2C_HandleTypeDef      pI2c_Handle;

/* Flags */

uint8_t RTCFlag = ACTIVE;

/* Temp */
uint8_t t1,t2,t3;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
int sprintf(char *str, const char *format, ...);
int snprintf(char *str, size_t size, const char *format, ...);

static void getCurrent(void);
static void displayTime(void);
static void write_offset (void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *RTCHandle){
	//BSP_LED_Toggle(LED4);
	RTCFlag = ACTIVE;
/* Your code here
*/
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
switch (GPIO_Pin) {
	case GPIO_PIN_0:      //SELECT button                    
		break;    

	case GPIO_PIN_1:     //left button                        
		break;
	case GPIO_PIN_2:    //right button                        
		break;
	case GPIO_PIN_3:    //up button         
		break;
	case GPIO_PIN_5:    //down button                        
		break;
	default://
		break;
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
	
	state = timeState;
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	HAL_InitTick (0x0000);
  BSP_LCD_GLASS_Init();
	BSP_JOY_Init(JOY_MODE_EXTI);
	BSP_LED_Init(LED4);
	BSP_LCD_GLASS_DisplayString((uint8_t*)"LAB3");
	HAL_Delay(1000);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
	I2C_Init(&pI2c_Handle);
  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if (BSP_JOY_GetState() == JOY_SEL) {
		SEL_Pressed_StartTick=HAL_GetTick(); 
		while(BSP_JOY_GetState() == JOY_SEL) {  //while the selection button is pressed)	
			if ((HAL_GetTick()-SEL_Pressed_StartTick)>750) {
				if(state==timeState){
				state=dateState;
				}
			} 
		}
	}
			switch(state){
				
			case timeState:
				if (RTCFlag == ACTIVE){
					if (HAL_RTC_GetTime(&RTCHandle, &RTC_TimeStructure, RTC_FORMAT_BCD) == HAL_OK){
						BSP_LED_Toggle(LED5);
						displayTime();
					}
				RTCFlag = INACTIVE;
				}
				if (selpressed == 1){
					selpressed = INACTIVE;
					getCurrent();
				EE_status=I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, memLocation+mem_offset_w, hh);
				EE_status=I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, memLocation+mem_offset_w+1, mm);
				EE_status=I2C_ByteWrite(&pI2c_Handle,EEPROM_ADDRESS, memLocation+mem_offset_w+2, ss);
				write_offset();
				}
				break;
				
			case dateState:
				getCurrent();
				snprintf(lcd_buffer,8,"%d%d%d",dd,mo,yy);
				BSP_LCD_GLASS_Clear();
				BSP_LCD_GLASS_DisplayString((uint8_t*)lcd_buffer);
				HAL_Delay(1500);
				state = timeState;
				break;
			
			case pastTimeState1:
				t1=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation+recent);
				t2=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation+recent+1);
				t3=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation+recent+2);
				snprintf(lcd_buffer,8,"%d%d%d",t1,t2,t3);
				BSP_LCD_GLASS_Clear(); 
				BSP_LCD_GLASS_DisplayString((uint8_t*) lcd_buffer);			
				break;
			
			case pastTimeState2:
				//show memeory from EEPROM2 (2nd most recent click)
				t1=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation+recent2);
				t2=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation+recent2+1);
				t3=I2C_ByteRead(&pI2c_Handle,EEPROM_ADDRESS, memLocation+recent2+2);
				snprintf(lcd_buffer,8,"%d%d%d",t1,t2,t3);
				BSP_LCD_GLASS_Clear(); 
				BSP_LCD_GLASS_DisplayString((uint8_t*) lcd_buffer);
				break;
			
			case setState1:
				break;
			
			case setState2:
				break;
			
			case storeState:
				break;
			
			case CNTState:
				break;
		}
		
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  pI2c_Handle.Instance = I2C1;
  pI2c_Handle.Init.Timing = 0x00000E14;
  pI2c_Handle.Init.OwnAddress1 = 0;
  pI2c_Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  pI2c_Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  pI2c_Handle.Init.OwnAddress2 = 0;
  pI2c_Handle.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  pI2c_Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  pI2c_Handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&pI2c_Handle) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&pI2c_Handle, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&pI2c_Handle, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  RTCHandle.Instance = RTC;
  RTCHandle.Init.HourFormat = RTC_HOURFORMAT_24;
  RTCHandle.Init.AsynchPrediv = 127;
  RTCHandle.Init.SynchPrediv = 255;
  RTCHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
  RTCHandle.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  RTCHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RTCHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&RTCHandle) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&RTCHandle, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x20;

  if (HAL_RTC_SetDate(&RTCHandle, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_ALL;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&RTCHandle, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : JOY_CENTRE_Pin */
  GPIO_InitStruct.Pin = JOY_CENTRE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(JOY_CENTRE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : JOY_LEFT_Pin JOY_RIGHT_Pin JOY_UP_Pin JOY_DOWN_Pin */
  GPIO_InitStruct.Pin = JOY_LEFT_Pin|JOY_RIGHT_Pin|JOY_UP_Pin|JOY_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

void getData(void){
	// stores copy of date and time setting for user to edit
	setData[0]=hh;
	setData[1]=mm;
	setData[2]=ss;
	setData[3]=yy;
	setData[4]=mo;
	setData[5]=dd;
	setData[6]=wd;
}

void write_offset(void){
	//logic to update what location in memory to overwerite
	if (mem_offset_w==0){
		mem_offset_w=3;
		recent=0;
		recent2=3;
	}
	else  if (mem_offset_w==3){
		mem_offset_w=0;
		recent=3;
		recent2=0;
	}
}

void getCurrent(void){
	//updates current time and date and stores data in respective variables
	HAL_RTC_GetTime(&RTCHandle,&RTC_TimeStructure, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&RTCHandle,&RTC_DateStructure, RTC_FORMAT_BIN);
	ss=RTC_TimeStructure.Seconds;
	mm=RTC_TimeStructure.Minutes;
	hh=RTC_TimeStructure.Hours;
	yy=RTC_DateStructure.Year;
	mo=RTC_DateStructure.Month;
	dd=RTC_DateStructure.Date;
}

void displayTime(){
	getCurrent();
	//updates time and date data then shows time on screen
	snprintf(timestring,8,"%d%d%d",hh,mm,ss);
	BSP_LCD_GLASS_Clear(); 
	BSP_LCD_GLASS_DisplayString((uint8_t*) timestring);
}

void updateData(){
	// updates date and time settings from copy of variables that user to modified 
	RTC_DateStructure.Year = setData[3];
	RTC_DateStructure.Month = setData[4];
	RTC_DateStructure.Date = setData[5];
	RTC_DateStructure.WeekDay = setData[6];
	
	RTC_TimeStructure.Hours = setData[0];  

	RTC_TimeStructure.Minutes = setData[1]; 

	RTC_TimeStructure.Seconds = setData[2];

		HAL_RTC_SetTime(&RTCHandle,&RTC_TimeStructure, RTC_FORMAT_BIN);

	HAL_RTC_SetDate(&RTCHandle,&RTC_DateStructure, RTC_FORMAT_BIN);

	

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
  BSP_LED_On(LED4);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/