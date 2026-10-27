/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "INA219.h"
#include "task.h"
#include "queue.h"
#include "mpu9250.h"
#include<math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU9250_TIMEOUT_MS 20U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//Les variables FreeRTOS
TaskHandle_t receiver_rx_read_task_handle;
TaskHandle_t battery_monitoring_task_handle;
TaskHandle_t PID_stabilization_task_handle;
TaskHandle_t motors_command_task_handle;
TaskHandle_t MPU9250_task_handle;

volatile uint8_t MPU9250_DataValid = 0;
volatile uint32_t MPU9250_ConsecutiveErrors = 0;
volatile uint32_t MPU9250_LastValidTick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
INA219_t my_ina219;
float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;
int RateCalibrationNumber;

//float ReceiverValue[]={0,0,0,0};
//float pulse_ms[4]={0.0};
volatile float ReceiverValue[4] = {1500.0f, 1500.0f, 1000.0f, 1500.0f};
volatile float pulse_us[4] = {1500.0f, 1500.0f, 1000.0f, 1500.0f};

int BatteryRemaining;
int BatteryDefault=2200;
uint16_t Qconsommee;
int t_now,t_prev=0;

uint32_t LoopTimer;

// Variables for the PID ccontroler
float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputPitch, InputYaw, InputThrottle;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
float PIDReturn[]={0,0,0};
// InputRoll=f(ErrorRoll, PRoll,IRoll,DRoll, PrevErrorRoll, PrevItermRoll)
float PRateRoll=0.6; float PRatePitch=0.6; float PRateYaw=2;
float IRateRoll=3.5; float IRatePitch=3.5; float IRateYaw=12;
float DRateRoll=0.03; float DRatePitch=0.03; float DRateYaw=0;

float Motor_front_left_Input, Motor_front_right_Input, Motor_rear_right_Input, Motor_rear_left_Input;

// Qtotal représente ici batteryDefault, c'est la même chose
int battery_capacity_monitoring(int Qtotal){
  static int SOC;
  t_now=__HAL_TIM_GET_COUNTER(&htim4);
  Qconsommee=round(INA219_ReadCurrent(&my_ina219))*((t_now-t_prev)/3600000);
  t_prev=t_now;
  SOC=100-(Qconsommee/Qtotal)*100;
  return SOC;
}

long map(int valeur,int a,int b,int c,int d){
	return c+(valeur-a)*(d-c)/(b-a);
}

static float constrain_float(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

static void mix_and_limit_motors(float throttle, float roll, float pitch, float yaw, float motor[4])
{
  /* Corrections différentielles de chaque moteur */
  motor[0] = -roll - pitch - yaw;  // Avant gauche
  motor[1] = -roll + pitch + yaw;  // Avant droit
  motor[2] =  roll + pitch - yaw;  // Arrière droit
  motor[3] =  roll - pitch + yaw;  // Arrière gauche

  float min_correction = motor[0];
  float max_correction = motor[0];

  for (int i = 1; i < 4; i++)
  {
    if (motor[i] < min_correction) min_correction = motor[i];
    if (motor[i] > max_correction) max_correction = motor[i];
  }

  /* Réduire les corrections seulement si nécessaire */
  float span = max_correction - min_correction;

  if (span > 1000.0f)
  {
    float scale = 1000.0f / span;
    for (int i = 0; i < 4; i++) motor[i] *= scale;
  }

  /* Recalculer les bornes des corrections */
  min_correction = motor[0];
  max_correction = motor[0];

  for (int i = 1; i < 4; i++)
  {
      if (motor[i] < min_correction) min_correction = motor[i];
      if (motor[i] > max_correction) max_correction = motor[i];
  }

  /* Déterminer le throttle admissible */
  float min_throttle = 1000.0f - min_correction;
  float max_throttle = 2000.0f - max_correction;

  throttle = constrain_float(throttle, min_throttle, max_throttle);

  /* Commandes finales des quatre moteurs */
  for (int i = 0; i < 4; i++) motor[i] += throttle;
}

void read_receiver(void){
	ReceiverValue[0]=pulse_us[1];		//Roll
	ReceiverValue[1]=pulse_us[2];		//Pitch
	ReceiverValue[2]=pulse_us[0];		//Throttle
	ReceiverValue[3]=pulse_us[3];		//Yaw
}

I2C_Status gyro_signals(void) {
  I2C_Status status;
  status = MPU9250_GetGyroData(&RateRoll,&RatePitch,&RateYaw);
  if(status != I2C_OK) return status;
  else return I2C_OK;
}

// PID equation
void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm){
  float Pterm=P*Error;
  float Iterm=PrevIterm+I*(Error+PrevError)*0.004/2;

  // To avoid integral windup
  if(Iterm>400) Iterm=400;
  else if (Iterm<-400) Iterm=-400;

  float Dterm=D*(Error-PrevError)/0.004;
  float PIDOutput=Pterm+Iterm+Dterm;

  // To avoid integral windup
  if(PIDOutput>400) PIDOutput=400;
  else if (PIDOutput<-400) PIDOutput=-400;

  PIDReturn[0]=PIDOutput;
  PIDReturn[1]=Error;
  PIDReturn[2]=Iterm;
}

void reset_pid(void){
  PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
  PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;
}

void receiver_rx_read_task(void *params){
  for(;;){
    (void)params;
    read_receiver();
  }
}

void battery_monitoring_task(void *params){
  for(;;){
    (void)params;
    //Power Switch
    BatteryRemaining=battery_capacity_monitoring(BatteryDefault);
    if(BatteryRemaining<=30) HAL_GPIO_WritePin(battery_led_GPIO_Port,battery_led_Pin,GPIO_PIN_SET);//digitalWrite(5,HIGH);
    else HAL_GPIO_WritePin(battery_led_GPIO_Port,battery_led_Pin,GPIO_PIN_SET);;
  }
}

void PID_stabilization_task(void *params)
{
    (void)params;
    TickType_t last_wake_time = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(MPU9250_TIMEOUT_MS);
    for (;;)
    {
      TickType_t now = xTaskGetTickCount();

      /* 1. Vérifier la disponibilité des mesures */
      if ((MPU9250_DataValid == 0U) && ((TickType_t)(now - MPU9250_LastValidTick) >= timeout_ticks))
      {
        /*
          * Perte prolongée du gyroscope.
          * Ne pas calculer de nouvelles corrections PID.
          * Signaler ce défaut au superviseur de sécurité.
          */

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(4));
        continue;
      }

      if (MPU9250_DataValid == 0U)
      {
        /*
          * Échec ponctuel :
          * ne pas recalculer le PID sur des mesures périmées.
          * Les dernières corrections restent inchangées.
          */

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(4));
        continue;
      }
      /* Consignes de vitesse angulaire */
      DesiredRateRoll  = 0.15f * (ReceiverValue[0] - 1500.0f);
      DesiredRatePitch = 0.15f * (ReceiverValue[1] - 1500.0f);
      DesiredRateYaw   = 0.15f * (ReceiverValue[3] - 1500.0f);

      /* Commande collective */
      InputThrottle = ReceiverValue[2];
      if (InputThrottle < 1050.0f)
      {
        /* Réinitialiser les états internes des PID */
        reset_pid();
        /* Annuler les corrections d'attitude */
        InputRoll  = 0.0f;
        InputPitch = 0.0f;
        InputYaw   = 0.0f;
      }
      else
      {
        /* Calcul des erreurs */
        ErrorRateRoll  = DesiredRateRoll  - RateRoll;
        ErrorRatePitch = DesiredRatePitch - RatePitch;
        ErrorRateYaw   = DesiredRateYaw   - RateYaw;

        /* PID Roll */
        pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
        InputRoll = PIDReturn[0];
        PrevErrorRateRoll = PIDReturn[1];
        PrevItermRateRoll = PIDReturn[2];

        /* PID Pitch */
        pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
        InputPitch = PIDReturn[0];
        PrevErrorRatePitch = PIDReturn[1];
        PrevItermRatePitch = PIDReturn[2];

        /* PID Yaw */
        pid_equation(ErrorRateYaw, PRateYaw, IRateYaw, DRateYaw, PrevErrorRateYaw, PrevItermRateYaw);
        InputYaw = PIDReturn[0];
        PrevErrorRateYaw = PIDReturn[1];
        PrevItermRateYaw = PIDReturn[2];
      }
      /* Période nominale : 4 ms, soit 250 Hz */
      vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(4));
    }
}

void motors_command_task(void *params){
  (void)params;

  float motor[4];
  for(;;){
    /* Sécurité : throttle minimal */
    if (!RC_IsValid() || ReceiverValue[2] < 1050)
    {
      InputThrottle = 1000.0f;
      Motor_front_left_Input= 1000.0f;
      Motor_front_right_Input= 1000.0f;
      Motor_rear_right_Input = 1000.0f;
      Motor_rear_left_Input= 1000.0f;
    }
    else
    {
      /* Mixage et gestion de la saturation */
      mix_and_limit_motors(InputThrottle, InputRoll, InputPitch, InputYaw, motor);

      Motor_front_left_Input = motor[0];
      Motor_front_right_Input = motor[1];
      Motor_rear_right_Input = motor[2];
      Motor_rear_left_Input = motor[3];
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Motor_front_left_Input);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, Motor_front_right_Input);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, Motor_rear_right_Input);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, Motor_rear_left_Input);

    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void MPU9250_task(void *params){
  (void)params;
  for(;;){
    I2C_Status status = gyro_signals();
    if (status == I2C_OK)
    {
      RateRoll -= RateCalibrationRoll;
      RatePitch -= RateCalibrationPitch;
      RateYaw -= RateCalibrationYaw;

      MPU9250_LastValidTick = xTaskGetTickCount();
      MPU9250_ConsecutiveErrors = 0;
      MPU9250_DataValid = 1;
    }
    else
    {
      MPU9250_DataValid = 0;
      MPU9250_ConsecutiveErrors++;
    }

    vTaskDelay(pdMS_TO_TICKS(4));
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
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  MPU9250_Init();

  HAL_NVIC_SetPriority(TIM1_CC_IRQn, 5, 0);
  HAL_NVIC_SetPriority(TIM2_IRQn,    5, 0);
  HAL_NVIC_SetPriority(TIM3_IRQn,    5, 0);
  HAL_NVIC_SetPriority(TIM4_IRQn,    5, 0);

  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  my_ina219.ina219_i2c = &hi2c2;
  my_ina219.Address = 0x40;

  // Tasks and semaphores creation

  BaseType_t status;

  status = xTaskCreate(receiver_rx_read_task,"receiver_rx_read_task", 256, NULL, 3, &receiver_rx_read_task_handle);
  if(status != pdPASS) {while (1);}

  status = xTaskCreate(battery_monitoring_task,"battery_monitoring_task", 256, NULL, 3, &battery_monitoring_task_handle);
  if(status != pdPASS) {while (1);}

  status = xTaskCreate(PID_stabilization_task,"PID_stabilization_task", 256, NULL, 3, &PID_stabilization_task_handle);
  if(status != pdPASS) {while (1);}

  status = xTaskCreate(motors_command_task,"motors_command_task", 256, NULL, 3, &motors_command_task_handle);
  if(status != pdPASS) {while (1);}

  status = xTaskCreate(MPU9250_task,"MPU9250_task", 256, NULL, 3, &MPU9250_task_handle);
  if(status != pdPASS) {while (1);}
  {
    /* code */
  }
  
  /* USER CODE END 2 */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c2.Init.Timing = 0x10D19CE4;
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
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x10D19CE4;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 79;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
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
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 79;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(battery_led_GPIO_Port, battery_led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : battery_led_Pin */
  GPIO_InitStruct.Pin = battery_led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(battery_led_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
#define RC_TIMEOUT_MS 100U
static volatile TickType_t RC_LastUpdateTick[4] = {0};

static uint32_t t_rising[4] = {0};
static uint32_t t_falling[4] = {0};
static uint8_t edge[4] = {0};

void test(uint32_t *t_rising, uint32_t *t_falling, uint8_t *edge, volatile float* pulse,TIM_HandleTypeDef *htim, uint32_t channel){
	uint32_t captured;

  if(*edge == 0)
	{
		*t_rising = HAL_TIM_ReadCapturedValue(htim, channel);

		__HAL_TIM_SET_CAPTUREPOLARITY(htim, channel, TIM_INPUTCHANNELPOLARITY_FALLING);

		*edge = 1;
	}
	else
  {
    /* Front descendant : fin de l'impulsion */
    *t_falling = HAL_TIM_ReadCapturedValue(htim, channel);

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);

    /* Calcul de la largeur en ticks, avec gestion du débordement */
    if (*t_falling >= *t_rising)
    {
      captured = *t_falling - *t_rising;
    }
    else
    {
      captured = (arr + 1U - *t_rising) + *t_falling;
    }

    /*
      1 MHz counter means 1 tick = 1 us.
    */
    *pulse = (float)captured;
    RC_LastUpdateTick[htim->Instance == TIM1 ? 0 : htim->Instance == TIM2 ? 1 : htim->Instance == TIM3 ? 2 : 3] = xTaskGetTickCountFromISR();

    /* Revenir au front montant pour la prochaine impulsion */
    __HAL_TIM_SET_CAPTUREPOLARITY(htim, channel, TIM_INPUTCHANNELPOLARITY_RISING);

    *edge = 0U;
  }
}

static uint8_t RC_IsValid(void)
{
  TickType_t now = xTaskGetTickCount();
  TickType_t timeout = pdMS_TO_TICKS(RC_TIMEOUT_MS);

  for (uint8_t i = 0; i < 4; i++)
  {
      if ((TickType_t)(now - RC_LastUpdateTick[i]) >= timeout) return 0;
  }
  return 1;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1 && htim==&htim1)
    {
    	test(&t_rising[0],&t_falling[0],&edge[0],&pulse_us[0],htim,TIM_CHANNEL_1);
    }
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1 && htim==&htim2)
    {
       	test(&t_rising[1],&t_falling[1],&edge[1],&pulse_us[1],htim,TIM_CHANNEL_1);
    }
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1 && htim==&htim3)
    {
    	test(&t_rising[2],&t_falling[2],&edge[2],&pulse_us[2],htim,TIM_CHANNEL_1);
    }
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1 && htim==&htim4)
    {
    	test(&t_rising[3],&t_falling[3],&edge[3],&pulse_us[3],htim,TIM_CHANNEL_1);
    }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
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
