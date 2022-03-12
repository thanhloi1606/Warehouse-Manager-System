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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ds1307.h"
#include "i2c-lcd.h"
#include "rc522.h"
#include "stdio.h"
#include "flash.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int i=0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
DS1307_Handle ds1307;
DS1307_TIME time;
I2C_HandleTypeDef i2c_handle;
void delay (uint16_t times)
{
	/* change your code here for the delay in microseconds */
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	while ((__HAL_TIM_GET_COUNTER(&htim2))<times);
}
uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t SUM, RH, TEMP;
uint8_t Presence = 0;
int Temperature = 0;
int Humidity = 0;
char str1[20];
char str2[20];

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define DHT11_PORT GPIOC
#define DHT11_PIN GPIO_PIN_14
/* USER CODE END 0 */
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
void DHT11_Start (void)
{
	Set_Pin_Output (DHT11_PORT, DHT11_PIN);  // set the pin as output
	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
	delay(1000);//1ms
	
	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);   // pull the pin low
	delay (18000);   // wait for 18ms
    HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);   // pull the pin high
	delay (20);   // wait for 20us
	Set_Pin_Input(DHT11_PORT, DHT11_PIN);    // set as input
}
uint8_t DHT11_Check_Response (void)
{
	uint8_t Response = 0;
	delay (40);
	if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))
	{
		delay (80);
		if ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))) Response = 1;
		else Response = -1; // 255
	}
	while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));		// wait for the pin to go low

	return Response;
}
uint8_t DHT11_Read (void)
{
	uint8_t k,l;
	for (l=0;l<8;l++)
	{
		while (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));   // wait for the pin to go high
		delay (40);   // wait for 40 us
		if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))   // if the pin is low
		{
			k&= ~(1<<(7-l)); 			// write 0
		}
		else k|= (1<<(7-l));  // if the pin is high, write 1
		while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));		// wait for the pin to go low
	}
	return k;
}
DS1307_RESULT confDS1307(void) {

ds1307.i2c = &i2c_handle;
 ds1307.DS1307_ADDRESS = 0xD0;
 ds1307.DS1307_CLOCK = 100000;
 ds1307.DS1307_I2Cx = I2C1;
 ds1307.TIMEOUT = 1000;
 DS1307_RESULT res;
 res = DS1307_Init(&ds1307);
 if (res != DS1307_RES_OK) {
 return DS1307_RES_ERROR;
 }
 res = DS1307_ClockResume(&ds1307);
 if (res != DS1307_RES_OK) {
 return DS1307_RES_ERROR;
 }
 res = DS1307_EnableSquareWave(&ds1307);
 if (res != DS1307_RES_OK) {
 return DS1307_RES_ERROR;
 }
 res = DS1307_SelectRate(&ds1307, DS1307_RATE_1HZ);
 if (res != DS1307_RES_OK) {
 return DS1307_RES_ERROR;
 }
 return DS1307_RES_OK;
}
char Date[20];//ngay thang
			char Now[20];//gio phut
			char Th[20];//thu 
unsigned char CardID[5];
char dataOut[100];
char data[100];
int gio=0,phut=0,giay=0,ngay=0,thang=0,nam=0,thu=0;
int giosang=6,phutsang=0;
int giotoi=18,phuttoi=0;
char giosang2[10],giotoi2[10],phutsang2[10],phuttoi2[10];
char gio2[10],phut2[10],giay2[10],ngay2[10],thang2[10],nam2[10];
int o;
int nhietdo =40;
int doam = 70;
char nhietdo2[10];
char doam2[10];
//int state=0;
//int bug=0;
#define DATA_START_ADDRESS 		 	((uint32_t)0x0801FC00)	//Page 127
#define LENGTH_START_ADDRESS 		((uint32_t)0x0801F810)	//Page 126
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
HAL_TIM_Base_Start(&htim2);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 56-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFF-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
 // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;//|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4 
                           PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	if (confDS1307()!=DS1307_RES_OK)
		{
			}
		time.hour=gio;
		time.minute=phut;
		time.second=giay;
		time.day=ngay;
		time.month=thang;
		time.year=nam;
		time.date=thu;
			DS1307_SetTime(&ds1307,time);
	lcd_init();
			MFRC522_Init();
			lcd_clear_display();
			HAL_Delay(20);
		//	DHT11_Start();
  /* Infinite loop */
  for(;;)
  {
		//xuat nhiet do do am
		DHT11_Start();
	Presence = DHT11_Check_Response();
		if(Presence==1)
			{
		Rh_byte1 = DHT11_Read();
	  Rh_byte2 = DHT11_Read ();
	  Temp_byte1 = DHT11_Read ();
	  Temp_byte2 = DHT11_Read ();
	  SUM = DHT11_Read();
				TEMP = Temp_byte1;
	  RH = Rh_byte1;
				}
			HAL_Delay(1100);
					Temp_byte2=(int) Temp_byte2;
	  Temperature = (int) TEMP;
	  Humidity = (int) RH;
				sprintf (str1, "T:%0.2d.%0.2d'C  ", Temperature,Temp_byte2);
				sprintf (str2, "H:%0.1d", Humidity);
				lcd_goto_XY(0,0);
				lcd_send_string(str1);
				lcd_goto_XY(0,11);
				lcd_send_string(str2);
				lcd_goto_XY(0,15);
				lcd_send_data('%');
			//cai dat gio -- nut 1 -PA10
					if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_10)==GPIO_PIN_RESET)
			{			
				lcd_clear_display();
				HAL_Delay(50);
				lcd_goto_XY(1,0);
				lcd_send_string("Cai dat ngay gio");
				HAL_Delay(1000);
				lcd_clear_display();
				HAL_Delay(500);				
				while(!(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12)==GPIO_PIN_RESET))
					{
						sprintf(gio2,"%0.1d",gio);
						sprintf(phut2,"%0.1d",phut);
						//sprintf(giay2,"%0.1d",giay);
						lcd_goto_XY(1,0);
						lcd_send_string("Time is: ");
						lcd_goto_XY(0,2);
						lcd_send_string("h ");
						lcd_goto_XY(0,6);
						lcd_send_string("m ");
						//lcd_goto_XY(0,10);
						//lcd_send_string("s ");
						lcd_goto_XY(0,0);
						lcd_send_string(gio2);
						lcd_goto_XY(0,4);
						lcd_send_string(phut2);
						//lcd_goto_XY(0,8);
						//lcd_send_string(giay2);
						
				HAL_Delay(300); 
					if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15)==GPIO_PIN_RESET)
						{
							gio=gio+1;
							if(gio>23)
								{
									gio=0;
								lcd_goto_XY(0,1);
									lcd_send_string(" ");
								}
							sprintf(gio2,"%0.2d",gio);						
						}
							if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)==GPIO_PIN_RESET)
						{
							phut=phut+1;
							if(phut>60)
								{
									phut=0;
									lcd_goto_XY(0,5);
									lcd_send_string(" ");
								}
							sprintf(phut2,"%0.2d",phut);						
					}
				}
		time.hour=gio;
		time.minute=phut;
		time.second=giay;
		time.day=ngay;
		time.month=thang;
		time.year=nam;
		time.date=thu;
					DS1307_SetTime(&ds1307,time);
				}
			//mode New Card
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15)==GPIO_PIN_RESET)
		{
			lcd_clear_display();
			HAL_Delay(20);
			lcd_goto_XY(0,0);
			lcd_send_string("Hay quet the");
			while(!(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12)==GPIO_PIN_RESET))
				{
			if(MFRC522_Check(CardID)==MI_OK)
				{
					//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
					sprintf(data,"%02X%02X%02X%02X%02X",CardID[0],CardID[1],CardID[2],CardID[3],CardID[4]);
					lcd_clear_display();
					HAL_Delay(20);
					lcd_goto_XY(0,3);
					lcd_send_string(data);
					HAL_Delay(1000);
					Flash_ProgramPage(data,DATA_START_ADDRESS,LENGTH_START_ADDRESS);
					lcd_goto_XY(1,0);
					lcd_send_string("Saved");
				}
				}
		}
		//luc binh thuong
		if(MFRC522_Check(CardID)==MI_OK)
			{
				sprintf(data,"%02X%02X%02X%02X%02X",CardID[0],CardID[1],CardID[2],CardID[3],CardID[4]);
				Flash_ReadChar(dataOut,DATA_START_ADDRESS,LENGTH_START_ADDRESS);
				if(strcmp(data,dataOut)==0)
					{
						lcd_clear_display();
						HAL_Delay(20);
						lcd_goto_XY(0,0);
						lcd_send_string("Hop le");
						HAL_Delay(300);
						lcd_goto_XY(0,0);
						lcd_send_string("Set-up:ON");
						HAL_Delay(1000);
						while(!(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12)==GPIO_PIN_RESET))
							{
								//mode cai nhiet do
								if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15)==GPIO_PIN_RESET)
								{			
									lcd_clear_display();
									HAL_Delay(50);
									lcd_goto_XY(1,0);
									lcd_send_string("Cai dat nhiet do ");
									HAL_Delay(1000);
									lcd_clear_display();
									HAL_Delay(500);				
									while(!(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12)==GPIO_PIN_RESET))
										{	
											sprintf(nhietdo2,"%0.1d",nhietdo);
											lcd_goto_XY(0,0);
											lcd_send_string("Nhiet do la:    ");
											lcd_goto_XY(0,13);
											lcd_send_string(nhietdo2);
									HAL_Delay(300); 
										if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15)==GPIO_PIN_RESET)
											{
												nhietdo=nhietdo+1;
												sprintf(nhietdo2,"%0.2d",nhietdo);						
											}
												if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)==GPIO_PIN_RESET)
											{
												nhietdo=nhietdo-1;
												sprintf(nhietdo2,"%0.2d",nhietdo);						
											}
										}
								}
								//mode cai do am
								if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)==GPIO_PIN_RESET)
							{			
								lcd_clear_display();
								HAL_Delay(50);
								lcd_goto_XY(1,0);
								lcd_send_string("Cai dat do am ");
								HAL_Delay(1000);
								lcd_clear_display();
								HAL_Delay(300);				
								while(!(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12)==GPIO_PIN_RESET))
									{
										sprintf(doam2,"%0.1d",doam);
										//sprintf(phut2,"%0.1d",phut);
										//sprintf(giay2,"%0.1d",giay);
										lcd_goto_XY(0,0);
										lcd_send_string("Do am la:       ");
										lcd_goto_XY(0,12); 
										lcd_send_string(doam2);
								HAL_Delay(300); 
									if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15)==GPIO_PIN_RESET)
										{
											doam=doam+1;
											sprintf(doam2,"%0.2d",doam);						
										}
											if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)==GPIO_PIN_RESET)
										{
											doam=doam-1;
											sprintf(doam2,"%0.2d",doam);						
										}
									}
								}
							//mode cai lai thoi gian led tu dong sang
							if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13)==GPIO_PIN_RESET)
							{			
										lcd_clear_display();
										HAL_Delay(50);
										lcd_goto_XY(1,0);
										lcd_send_string("Cai dat auto led");
										HAL_Delay(1000);
										lcd_clear_display();
										HAL_Delay(500);				
										while(!(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12)==GPIO_PIN_RESET))
									{
										sprintf(giotoi2,"%0.1d",giotoi);
										sprintf(giosang2,"%0.1d",giosang);
										lcd_goto_XY(1,0);
										lcd_send_string("Time is: ");
										lcd_goto_XY(0,0);
										lcd_send_string("ON:  h   ");
										lcd_goto_XY(0,9);
										lcd_send_string("OFF:  h ");
										lcd_goto_XY(0,3);
										lcd_send_string(giotoi2);
										lcd_goto_XY(0,13);
										lcd_send_string(giosang2);
										HAL_Delay(300); 
												if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15)==GPIO_PIN_RESET)
													{
														giotoi=giotoi+1;
														if(giotoi>23)
															{
																giotoi=0;
															lcd_goto_XY(0,1);
																lcd_send_string(" ");
															}
														sprintf(giotoi2,"%0.2d",giotoi);						
													}
														if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)==GPIO_PIN_RESET)
													{
														giosang=giosang+1;
														if(giosang>23)
															{
																giosang=0;
																//lcd_goto_XY(0,5);
																//lcd_send_string(" ");
															}
														sprintf(giosang2,"%0.2d",giosang);						
													}
											}
										}
									}
								}
				else
					{
						lcd_clear_display();
						HAL_Delay(20);
						lcd_goto_XY(0,0);
						lcd_send_string("Khong hop le");
						HAL_Delay(1000);
					}
				}
					if(Temperature>=nhietdo||Humidity>=doam)
					{
						HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_2);
					}
					if(Temperature<nhietdo&&Humidity<doam)
					{
						HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
					}
					if((time.hour>=0&&time.hour<=5)||(time.hour>=18&&time.hour<=23))
					{
						HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
					}
					else
					{
						HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
					}
					if(time.hour==giotoi)
					{
						HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
					}
					if (time.hour==giosang)
					{
						HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
					}
			
//			
		time=DS1307_GetTime(&ds1307);
			sprintf(Now,"%0.2d:%0.2d:%0.2d ",time.hour,time.minute,time.second);
		lcd_goto_XY(1,0);
		lcd_send_string(Now);
    osDelay(1);
  }
}
  /* USER CODE END 5 */ 

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
