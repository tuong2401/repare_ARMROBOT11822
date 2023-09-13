/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "math.h"
#include "PORT.h"
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;

/* USER CODE BEGIN PV */
//------doc_encoder-------------
volatile uint32_t encoder_cnt1 = 0;
volatile uint32_t encoder_cnt2 = 0;
volatile uint32_t encoder_cnt3 = 0;

uint32_t read_encoder_1() //QUAY
	{
		encoder_cnt1 = __HAL_TIM_GET_COUNTER(&htim2); //A-B3, B-A5
		return encoder_cnt1;
	}
uint32_t read_encoder_2() //TINH TIEN
	{
		encoder_cnt2 = __HAL_TIM_GET_COUNTER(&htim5); // A-A0, B-A1
		return encoder_cnt2;
	}
uint32_t read_encoder_3() //CUOI
	{
		encoder_cnt3 = __HAL_TIM_GET_COUNTER(&htim4); //D13D12
		return encoder_cnt3;
	}	
//*******************************************

typedef enum {OFF,DC1_OFF, DC1_LEFT,DC1_RIGHT, DC2_UP, DC2_DOWN, DC2_OFF, DC3_LEFT, DC3_RIGHT,DC3_OFF} System_TYPE;
static System_TYPE current_TYPE = OFF; 
void DC(System_TYPE current_TYPE)
	{
   	switch (current_TYPE)
		{ case OFF:
						{   
							//DC1
							  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
							//DC2
             		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
							//DC3
							  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);				
			break;
						}
      case DC1_LEFT:
						{
							  __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 500);
                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);					
			break;
						}
      case DC1_RIGHT:
						{
							 __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
               __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 500);					
			break;
						}
			case DC1_OFF:
						{
							__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
              __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);					
			break;
						}
			case DC2_UP:
						{
							  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
			break;
						}
      case DC2_DOWN:
						{
							 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
               __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1500);					
			break;
						}
			case DC2_OFF:
						{
							__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
              __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
			break;
						}
			case DC3_LEFT:
						{
							  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 900);
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
			break;
						}
      case DC3_RIGHT:
						{
							 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
               __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 900);
			break;
						}
			case DC3_OFF:
						{
							__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
              __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
			break;
            }							
		}
	}
//================================================================================
//-----CURRENT_STATE_ROBOT---------------
typedef enum {STATE1, STATE2, STATE3, STATE4} System_State;
static System_State current_state = STATE1; //AUTO HOME
void AUTO_HOME()
{   
	switch (current_state)
		{ case STATE1:
						{
              DC(DC1_RIGHT);
							DC(DC2_OFF);
							DC(DC3_OFF);
					if((GPIOA->IDR & CAMBIEN_1)==0)
							{
                   __HAL_TIM_SET_COUNTER(&htim2, 0);						 
			             current_state = STATE2;									
							}
			break;
						}
      case STATE2:
						{
							DC(DC1_OFF);
						  DC(DC2_DOWN);
              DC(DC3_OFF);							
		          if((GPIOA->IDR & CAMBIEN_3)==0)
								{
										 __HAL_TIM_SET_COUNTER(&htim5, 0);	
										 current_state = STATE3;												
								}	
			break;
						}
      case STATE3:
						{
							DC(DC1_OFF);
							DC(DC2_OFF);
						  DC(DC3_LEFT); 
		          if(encoder_cnt3 == 10000)
								{
										//__HAL_TIM_SET_COUNTER(&htim4, 0);	
										 current_state = STATE4;								
								}
			break;
						}
			case STATE4:
				{
					DC(DC1_OFF);
					DC(DC2_OFF);
					DC(DC3_OFF);
					      if((GPIOC->IDR & NUTNHAN_3)==0)
								{
						       current_state = STATE1;		;												
								}				
      break;
				}
		}
}
//============================================================================
typedef enum {STATE1_1, STATE1_2, STATE1_3, STATE1_4,STATE1_5, STATE1_6, STATE1_7} System_TRANGTHAI_1;
static System_TRANGTHAI_1 TRANGTHAI_1 = STATE1_1; //AUTO HOME
void TRANG_THAI_1() 
{   
	switch (TRANGTHAI_1)
		{ case STATE1_1:
						{
              DC(DC1_RIGHT);
							DC(DC2_OFF);
							DC(DC3_OFF);
					if((GPIOA->IDR & CAMBIEN_2)==0)
							{	 
			             TRANGTHAI_1 = STATE1_2;										
							}
			break;
						}
      case STATE1_2:
						{
							DC(DC1_OFF);
						  DC(DC2_UP); 
							DC(DC3_OFF);
		          if((GPIOE->IDR & CAMBIEN_4)==0)
								{
										 TRANGTHAI_1 = STATE1_3;								
								}
			break;
						}
      case STATE1_3:
						{
							DC(DC1_OFF);
							DC(DC2_OFF);
						  DC(DC3_RIGHT);
              if(encoder_cnt3 == 5000)
								{
										TRANGTHAI_1 = STATE1_4;								
								}							
			break;
						}
			case STATE1_4:
				{
					DC(DC1_OFF);
					DC(DC2_OFF);
					DC(DC3_OFF);
				  TRANGTHAI_1 = STATE1_5;	
				
      break;
				}
			case STATE1_5:
				{
					DC(DC1_OFF);
					DC(DC2_OFF);
					DC(DC3_LEFT);
					 if(encoder_cnt3 == 60000)//doc lan 1 CAM BIEN 5
							{
								
									 TRANGTHAI_1 = STATE1_6;							
							}	

      break;
				}
			case STATE1_6:
				{
					DC(DC1_OFF);
					DC(DC2_DOWN); 
					DC(DC3_OFF);
					if((GPIOA->IDR & CAMBIEN_3)==0)//doc lan 1 CAM BIEN 3
						{
								 TRANGTHAI_1 = STATE1_7;								
						}
			break;
				}
		 case STATE1_7:
						{
              DC(DC1_LEFT);
							DC(DC2_OFF);
							DC(DC3_OFF);
					if((GPIOE->IDR & CAMBIEN_1)==0)//doc lan 1 CAM BIEN 1
							{	 
			             TRANGTHAI_1 = STATE1_1;										
							}
						
			break;
						}
		}
}
//===========================================================================================

//===============================================================================
typedef enum {STATE2_1, STATE2_2, STATE2_3, STATE2_4, STATE2_5, STATE2_6, STATE2_7, STATE2_8} System_TRANGTHAI_2;
static System_TRANGTHAI_2 TRANGTHAI_2 = STATE2_1; //AUTO HOME
void TRANG_THAI_2() // CHUA CAI DAT TOC DO CHO DC
{   
	switch (TRANGTHAI_2)
		{ case STATE2_1:
						{
              DC(DC1_LEFT);
							DC(DC2_OFF); 
						  DC(DC3_OFF);						
							if( encoder_cnt1 == 100000)
								{
									TRANGTHAI_2 = STATE2_2;
								}
			break;
						}
      case STATE2_2:
						{
							DC(DC1_OFF);
							DC(DC2_UP);
							DC(DC3_OFF); 
							        if((GPIOE->IDR & CAMBIEN_4)==0)//doc lan 1 CAM BIEN 4
								{
												TRANGTHAI_2 = STATE2_3;								
								}		
			break;
						}
      case STATE2_3:
						{
							DC(DC1_OFF);
							DC(DC2_OFF);
						  DC(DC3_LEFT); 
							if( encoder_cnt3 == 60000)
								{
									TRANGTHAI_2 = STATE2_4;
								}	
// vi tri A								
			break;
						}
			case STATE2_4:
				{
							DC(DC1_LEFT);
							DC(DC2_OFF);
						  DC(DC3_OFF); 
							if( encoder_cnt1 == 150000)
								{
									TRANGTHAI_2 = STATE2_5;
								}
      break;
				}
			case STATE2_5:
				{
							DC(DC1_OFF);
							DC(DC2_DOWN);
						  DC(DC3_OFF); 
							if( encoder_cnt2 == 10000)
								{
									TRANGTHAI_2 = STATE2_6;
								}
      break;
				}
			case STATE2_6:
				{
							DC(DC1_OFF);
							DC(DC2_DOWN);
						  DC(DC3_OFF); 
							if( encoder_cnt2 == 0)
								{
									TRANGTHAI_2 = STATE2_7;
								}
      break;
				}
			case STATE2_7:
				{
							DC(DC1_OFF);
							DC(DC2_OFF);
						  DC(DC3_RIGHT); 
							if( encoder_cnt3 == 0)
								{
									TRANGTHAI_2 = STATE2_8;
								}
      break;
				}
			case STATE2_8:
						{
              DC(DC1_RIGHT);
							DC(DC2_OFF); 
						  DC(DC3_OFF);						
							if( encoder_cnt1 == 100000)
								{
									TRANGTHAI_2 = STATE2_1;
								}
			break;
						}
			
		}
}
//========================================================================================
typedef enum {TH1, TH2, TH3} System_TH;
static System_TH current_TH = TH1; 
void TONG_HOP() 
{   
	switch (current_TH)
		{ case TH1:
						{
							AUTO_HOME();
						  if((GPIOC->IDR & NUTNHAN_1)==0)
								{	 
										 current_TH = TH2;
																			
								}
							if((GPIOC->IDR & NUTNHAN_2)==0)
								{		
										 current_TH = TH3;										
								}
			break;
						}
      case TH2:
						{
              TRANG_THAI_1();
							if((GPIOC->IDR & NUTNHAN_3)==0)
							  {
										 current_TH = TH1;														
								}
			break;
						}
      case TH3:
						{
             TRANG_THAI_2();
						 	if((GPIOC->IDR & NUTNHAN_3)==0)
							  {
										 current_TH = TH1;														
								}
			break;
					 }
		}
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
  // HAL_TIM_Base_Start_IT(&htim2); // hàm cho phep bat dau ngat
	// HAL_TIM_Base_Start_IT(&htim3);

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
		HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_1 | TIM_CHANNEL_2);
		HAL_TIM_Encoder_Start_IT(&htim5, TIM_CHANNEL_1 | TIM_CHANNEL_2);
		HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_1 | TIM_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		read_encoder_1();
		read_encoder_2();
		read_encoder_3();
	  TONG_HOP();
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 2000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 25-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 25-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 1000-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : PC13 PC14 PC15 PC5
                           PC6 PC7 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA8 PA9 PA10
                           PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PE7 PE8 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
