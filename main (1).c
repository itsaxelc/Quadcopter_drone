/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - Quadcopter flight controller
  *                   STM32L476 + FreeRTOS + MPU9250, rate mode (acro)
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
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "mpu9250.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum { FLIGHT_DISARMED = 0, FLIGHT_ARMED = 1 } FlightState_t;

typedef enum {
  DISARM_NONE = 0,
  DISARM_STICKS,        /* geste de desarmement */
  DISARM_RC_LOST,       /* perte du signal radio */
  DISARM_GYRO_LOST,     /* perte prolongee du gyroscope */
  DISARM_IDLE_TIMEOUT   /* arme mais throttle bas trop longtemps */
} DisarmReason_t;

/* Etat d'un PID (un par axe) */
typedef struct {
  float kp, ki, kd;
  float integral;
  float prev_error;
  float prev_meas;
  float d_filt;
  bool  primed;
} PidAxis_t;

typedef struct {
  float roll, pitch, throttle, yaw;   /* largeurs d'impulsion en us */
} RcFrame_t;

/* Indices des axes */
enum { AXIS_ROLL = 0, AXIS_PITCH = 1, AXIS_YAW = 2 };

/* Indices des moteurs (= ordre des timers : TIM1, TIM2, TIM3, TIM4) */
enum { MOTOR_FL = 0, MOTOR_FR = 1, MOTOR_RR = 2, MOTOR_RL = 3 };

/* Indices des canaux radio (= timers de capture) */
enum { RC_CH_THROTTLE = 0, RC_CH_ROLL = 1, RC_CH_PITCH = 2, RC_CH_YAW = 3 };
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ===================== CONVENTION DES SIGNES =====================
 * Gyro (apres GYRO_SIGN_xxx) :
 *   RateRoll  > 0 : cote DROIT qui descend
 *   RatePitch > 0 : nez qui descend
 *   RateYaw   > 0 : nez qui tourne vers la droite (sens horaire vu de dessus)
 * Helices vues de dessus : FR et RL tournent en sens horaire (CW),
 *                          FL et RR tournent en sens anti-horaire (CCW).
 * Si ton MPU est monte autrement, change les GYRO_SIGN_xxx.
 * Si tes sens de rotation moteurs sont inverses, change MIX_YAW_SIGN.
 * ================================================================ */
#define GYRO_SIGN_ROLL            (1.0f)
#define GYRO_SIGN_PITCH           (1.0f)
#define GYRO_SIGN_YAW             (1.0f)
#define MIX_YAW_SIGN              (1.0f)

/* ---- Boucle de vol ---- */
#define FLIGHT_PERIOD_MS          4U          /* 250 Hz */
#define FLIGHT_DT_MIN_S           0.001f
#define FLIGHT_DT_MAX_S           0.012f
#define GYRO_TIMEOUT_MS           20U         /* gyro invalide au-dela => desarmement */

/* ---- Calibration gyro (drone immobile au demarrage) ---- */
#define GYRO_CAL_SAMPLES          1000U
#define GYRO_CAL_PERIOD_MS        2U
#define GYRO_CAL_MAX_STD          1.5f        /* bruit max tolere (unites du gyro) */
#define GYRO_CAL_MAX_BIAS         40.0f       /* biais max plausible */
#define GYRO_CAL_MAX_FAILS        20U

/* ---- Filtres ---- */
#define GYRO_LPF_HZ               80.0f
#define DTERM_LPF_HZ              40.0f

/* ---- Radio ---- */
#define RC_TIMEOUT_MS             100U
#define RC_PULSE_MIN_US           900U
#define RC_PULSE_MAX_US           2100U
#define RC_CENTER_US              1500.0f
#define RC_DEADBAND_US            8.0f
#define RC_RATE_SCALE             0.15f       /* (us de stick) -> (unites/s) */
#define RC_IRQ_PRIORITY           5U          /* >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY */

/* ---- Armement ---- */
#define THROTTLE_LOW_US           1040.0f     /* hysteresis : bas si < 1040 */
#define THROTTLE_HIGH_US          1060.0f     /*              haut si > 1060 */
#define ARM_YAW_HIGH_US           1900.0f
#define DISARM_YAW_LOW_US         1100.0f
#define ARM_HOLD_MS               1000U
#define DISARM_HOLD_MS            1000U
#define IDLE_DISARM_MS            10000U
#define ARM_STICK_CENTER_TOL_US   250.0f
#define SPIN_YAW_CENTER_TOL_US    150.0f
#define ARM_HOLD_CYCLES           (ARM_HOLD_MS / FLIGHT_PERIOD_MS)
#define DISARM_HOLD_CYCLES        (DISARM_HOLD_MS / FLIGHT_PERIOD_MS)
#define IDLE_DISARM_CYCLES        (IDLE_DISARM_MS / FLIGHT_PERIOD_MS)

/* ---- Moteurs (impulsions en us, timers a 1 MHz) ---- */
#define MOTOR_STOP_US             1000U
#define MOTOR_IDLE_US             1070U       /* ralenti mini quand ca tourne : a regler selon tes ESC */
#define MOTOR_MAX_US              2000U
#define MOTOR_PWM_PERIOD_US       2500U       /* 400 Hz */

/* ---- PID (rate mode) ---- */
#define PID_I_LIMIT               400.0f
#define PID_OUT_LIMIT             400.0f
#define PID_ROLL_KP               0.6f
#define PID_ROLL_KI               3.5f
#define PID_ROLL_KD               0.03f
#define PID_PITCH_KP              0.6f
#define PID_PITCH_KI              3.5f
#define PID_PITCH_KD              0.03f
#define PID_YAW_KP                2.0f
#define PID_YAW_KI                12.0f
#define PID_YAW_KD                0.0f

/* ---- Supervision / watchdog ---- */
#define IWDG_TIMEOUT_MS           250U
#define SUPERVISOR_PERIOD_MS      20U
#define SUPERVISOR_STALL_LIMIT    5U

/* ---- Taches FreeRTOS ---- */
#define FLIGHT_TASK_STACK_WORDS   512U
#define FLIGHT_TASK_PRIORITY      (tskIDLE_PRIORITY + 3U)
#define SUPERVISOR_TASK_STACK_WORDS 192U
#define SUPERVISOR_TASK_PRIORITY  (tskIDLE_PRIORITY + 1U)
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
static TaskHandle_t flight_control_task_handle;
static TaskHandle_t supervisor_task_handle;

/* Variables observables au debugger */
volatile FlightState_t Flight_State = FLIGHT_DISARMED;
volatile DisarmReason_t Flight_LastDisarmReason = DISARM_NONE;
volatile uint32_t Flight_Heartbeat = 0;
volatile uint32_t Flight_GyroErrors = 0;

/* Etat interne de la boucle de vol (un seul ecrivain : flight_control_task) */
static PidAxis_t s_pid[3] = {
  [AXIS_ROLL]  = { .kp = PID_ROLL_KP,  .ki = PID_ROLL_KI,  .kd = PID_ROLL_KD  },
  [AXIS_PITCH] = { .kp = PID_PITCH_KP, .ki = PID_PITCH_KI, .kd = PID_PITCH_KD },
  [AXIS_YAW]   = { .kp = PID_YAW_KP,   .ki = PID_YAW_KI,   .kd = PID_YAW_KD   },
};
static uint32_t s_arm_hold = 0;
static uint32_t s_disarm_hold = 0;
static uint32_t s_idle_cycles = 0;
static bool     s_spin_inhibit = true;

/* Radio : ecrites par l'ISR de capture, lues par la tache de vol */
static volatile uint16_t   rc_pulse_us[4]    = {1500U, 1500U, 1000U, 1500U};
static volatile TickType_t rc_last_update[4] = {0};
static volatile uint8_t    rc_has_signal[4]  = {0};
static volatile uint32_t   rc_glitch_count[4] = {0};
static uint32_t rc_t_rising[4] = {0};
static uint8_t  rc_edge[4]     = {0};
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
void Motors_EmergencyStop(void);   /* appelable depuis HardFault_Handler */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ---------- Verifications de configuration FreeRTOS a la compilation ---------- */
_Static_assert(configTICK_RATE_HZ >= 1000U,
               "configTICK_RATE_HZ doit etre >= 1000 (sinon pdMS_TO_TICKS(4) = 0)");
_Static_assert(sizeof(TickType_t) == 4U,
               "TickType_t doit etre sur 32 bits (configUSE_16_BIT_TICKS = 0)");
#if defined(configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY)
_Static_assert(RC_IRQ_PRIORITY >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY,
               "Priorite IRQ radio trop haute pour appeler l'API FreeRTOS FromISR");
#endif

/* ============================== Utilitaires ============================== */
static inline float constrain_float(float value, float min, float max)
{
  if (value < min) return min;
  if (value > max) return max;
  return value;
}

/* Coefficient d'un filtre passe-bas du 1er ordre (PT1) */
static inline float lpf_alpha(float cutoff_hz, float dt)
{
  const float rc = 1.0f / (2.0f * 3.14159265f * cutoff_hz);
  return dt / (rc + dt);
}

static inline void flight_heartbeat(void)
{
  Flight_Heartbeat++;
}

/* Compteur de cycles pour mesurer le vrai dt */
static void dwt_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

/* ================================ Moteurs ================================ */
/* Ordre : FL = TIM1, FR = TIM2, RR = TIM3, RL = TIM4 (canal 2 = sortie PWM) */
static TIM_HandleTypeDef * const motor_tim[4] = { &htim1, &htim2, &htim3, &htim4 };

/* Passe les timers moteurs en PWM rapide (400 Hz) avec precharge du compare.
 * A appeler AVANT HAL_TIM_PWM_Start (compteur a 0, donc pas de risque de
 * debordement). Les capteurs radio (canal 1) gardent une base de 1 MHz. */
static void motors_pwm_config(void)
{
  for (uint8_t i = 0; i < 4U; i++)
  {
    TIM_HandleTypeDef *h = motor_tim[i];
    h->Init.Period = MOTOR_PWM_PERIOD_US - 1U;
    __HAL_TIM_SET_AUTORELOAD(h, MOTOR_PWM_PERIOD_US - 1U);
    __HAL_TIM_SET_COUNTER(h, 0U);
    __HAL_TIM_SET_COMPARE(h, TIM_CHANNEL_2, MOTOR_STOP_US);
    __HAL_TIM_ENABLE_OCxPRELOAD(h, TIM_CHANNEL_2);
  }
}

static inline void motor_set_us(uint8_t i, uint32_t us)
{
  __HAL_TIM_SET_COMPARE(motor_tim[i], TIM_CHANNEL_2, us);
}

static void motors_write_all(uint32_t us)
{
  for (uint8_t i = 0; i < 4U; i++) motor_set_us(i, us);
}

/* Ecriture explicite : arrondi + saturation [1000 ; 2000] */
static void motors_write(const float m[4])
{
  for (uint8_t i = 0; i < 4U; i++)
  {
    float v = constrain_float(m[i], (float)MOTOR_STOP_US, (float)MOTOR_MAX_US);
    motor_set_us(i, (uint32_t)lroundf(v));
  }
}

/* Coupure d'urgence : acces direct aux registres (utilisable dans un handler
 * de faute, sans dependre de HAL ni de FreeRTOS). */
void Motors_EmergencyStop(void)
{
  TIM1->CCR2 = MOTOR_STOP_US;
  TIM2->CCR2 = MOTOR_STOP_US;
  TIM3->CCR2 = MOTOR_STOP_US;
  TIM4->CCR2 = MOTOR_STOP_US;
}

/* Mixage X-quad avec saturation sure :
 *  - jamais de throttle "ajoute" pour compenser des corrections negatives
 *  - si un moteur depasse le max, on baisse l'ensemble (on garde les differences)
 *  - chaque moteur est ensuite borne a [ralenti ; max] */
static void mix_motors(float throttle, float roll, float pitch, float yaw, float m[4])
{
  throttle = constrain_float(throttle, (float)MOTOR_STOP_US, (float)MOTOR_MAX_US);
  yaw *= MIX_YAW_SIGN;

  m[MOTOR_FL] = throttle + roll - pitch + yaw;
  m[MOTOR_FR] = throttle - roll - pitch - yaw;
  m[MOTOR_RR] = throttle - roll + pitch + yaw;
  m[MOTOR_RL] = throttle + roll + pitch - yaw;

  float max_v = m[0];
  for (uint8_t i = 1; i < 4U; i++) if (m[i] > max_v) max_v = m[i];

  if (max_v > (float)MOTOR_MAX_US)
  {
    const float excess = max_v - (float)MOTOR_MAX_US;
    for (uint8_t i = 0; i < 4U; i++) m[i] -= excess;
  }

  for (uint8_t i = 0; i < 4U; i++)
    m[i] = constrain_float(m[i], (float)MOTOR_IDLE_US, (float)MOTOR_MAX_US);
}

/* ================================= Radio ================================= */
/* Appelee dans l'ISR de capture (canal 1 des timers 1 a 4) */
static void rc_capture_isr(uint8_t idx, TIM_HandleTypeDef *htim)
{
  const uint32_t captured = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

  if (rc_edge[idx] == 0U)
  {
    /* Front montant : debut d'impulsion */
    rc_t_rising[idx] = captured;
    __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
    rc_edge[idx] = 1U;
  }
  else
  {
    /* Front descendant : fin d'impulsion */
    const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
    uint32_t width;

    if (captured >= rc_t_rising[idx]) width = captured - rc_t_rising[idx];
    else                              width = (arr + 1U - rc_t_rising[idx]) + captured;

    __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
    rc_edge[idx] = 0U;

    /* 1 tick = 1 us. On ignore les impulsions aberrantes (parasites). */
    if ((width >= RC_PULSE_MIN_US) && (width <= RC_PULSE_MAX_US))
    {
      rc_pulse_us[idx]    = (uint16_t)width;
      rc_last_update[idx] = xTaskGetTickCountFromISR();
      rc_has_signal[idx]  = 1U;
    }
    else
    {
      rc_glitch_count[idx]++;
    }
  }
}

static int8_t rc_index_from_handle(const TIM_HandleTypeDef *htim)
{
  if (htim == &htim1) return 0;
  if (htim == &htim2) return 1;
  if (htim == &htim3) return 2;
  if (htim == &htim4) return 3;
  return -1;
}

static void rc_start(void)
{
  HAL_NVIC_SetPriority(TIM1_CC_IRQn, RC_IRQ_PRIORITY, 0);
  HAL_NVIC_SetPriority(TIM2_IRQn,    RC_IRQ_PRIORITY, 0);
  HAL_NVIC_SetPriority(TIM3_IRQn,    RC_IRQ_PRIORITY, 0);
  HAL_NVIC_SetPriority(TIM4_IRQn,    RC_IRQ_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);

  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
}

/* Retourne true seulement si les 4 canaux sont frais (jamais lus = invalide) */
static bool rc_read(RcFrame_t *rc)
{
  const TickType_t now = xTaskGetTickCount();
  const int32_t timeout = (int32_t)pdMS_TO_TICKS(RC_TIMEOUT_MS);
  bool ok = true;

  for (uint8_t i = 0; i < 4U; i++)
  {
    /* Difference signee : une mise a jour ISR juste apres notre lecture
     * du tick donne un age negatif (= frais), pas un faux timeout. */
    const int32_t age = (int32_t)(now - rc_last_update[i]);
    if ((rc_has_signal[i] == 0U) || (age >= timeout)) ok = false;
  }

  rc->throttle = (float)rc_pulse_us[RC_CH_THROTTLE];
  rc->roll     = (float)rc_pulse_us[RC_CH_ROLL];
  rc->pitch    = (float)rc_pulse_us[RC_CH_PITCH];
  rc->yaw      = (float)rc_pulse_us[RC_CH_YAW];
  return ok;
}

/* Stick centre, avec zone morte, dans [-500 ; +500] */
static float rc_stick(float pulse_us)
{
  float v = constrain_float(pulse_us, 1000.0f, 2000.0f) - RC_CENTER_US;
  if      (v >  RC_DEADBAND_US) v -= RC_DEADBAND_US;
  else if (v < -RC_DEADBAND_US) v += RC_DEADBAND_US;
  else                          v  = 0.0f;
  return v;
}

/* ================================= Gyro ================================== */
/* Lecture dans des variables locales : rien n'est publie en cas d'erreur */
static bool gyro_read(float g[3])
{
  float x, y, z;

  if (MPU9250_GetGyroData(&x, &y, &z) != I2C_OK) return false;
  if (!isfinite(x) || !isfinite(y) || !isfinite(z)) return false;

  g[AXIS_ROLL]  = GYRO_SIGN_ROLL  * x;
  g[AXIS_PITCH] = GYRO_SIGN_PITCH * y;
  g[AXIS_YAW]   = GYRO_SIGN_YAW   * z;
  return true;
}

/* Moyenne sur GYRO_CAL_SAMPLES echantillons, drone immobile.
 * Echoue si le gyro bouge (ecart-type trop grand), si le biais est
 * improbable ou si trop de lectures echouent. */
static bool gyro_calibrate(float offset[3])
{
  float sum[3]    = {0.0f, 0.0f, 0.0f};
  float sum_sq[3] = {0.0f, 0.0f, 0.0f};
  uint32_t n = 0U, fails = 0U;

  while (n < GYRO_CAL_SAMPLES)
  {
    vTaskDelay(pdMS_TO_TICKS(GYRO_CAL_PERIOD_MS));
    flight_heartbeat();

    float g[3];
    if (gyro_read(g))
    {
      for (uint8_t i = 0; i < 3U; i++)
      {
        sum[i]    += g[i];
        sum_sq[i] += g[i] * g[i];
      }
      n++;
    }
    else if (++fails > GYRO_CAL_MAX_FAILS)
    {
      return false;
    }
  }

  float result[3];
  for (uint8_t i = 0; i < 3U; i++)
  {
    const float mean = sum[i] / (float)n;
    float var = (sum_sq[i] / (float)n) - (mean * mean);
    if (var < 0.0f) var = 0.0f;

    if ((sqrtf(var) > GYRO_CAL_MAX_STD) || (fabsf(mean) > GYRO_CAL_MAX_BIAS)) return false;
    result[i] = mean;
  }

  for (uint8_t i = 0; i < 3U; i++) offset[i] = result[i];
  return true;
}

/* ================================== PID ================================== */
static void pid_reset(PidAxis_t *p)
{
  p->integral   = 0.0f;
  p->prev_error = 0.0f;
  p->prev_meas  = 0.0f;
  p->d_filt     = 0.0f;
  p->primed     = false;
}

static void pids_reset(void)
{
  for (uint8_t i = 0; i < 3U; i++) pid_reset(&s_pid[i]);
}

/* PID avec dt mesure, integrale trapezoidale, anti-windup,
 * D calcule sur la MESURE (pas de "derivative kick") puis filtre. */
static float pid_update(PidAxis_t *p, float setpoint, float meas, float dt, float d_alpha)
{
  const float error = setpoint - meas;

  if (!p->primed)
  {
    p->prev_error = error;
    p->prev_meas  = meas;
    p->d_filt     = 0.0f;
    p->primed     = true;
  }

  const float pterm = p->kp * error;

  p->integral += p->ki * 0.5f * (error + p->prev_error) * dt;
  p->integral  = constrain_float(p->integral, -PID_I_LIMIT, PID_I_LIMIT);

  const float d_raw = -(meas - p->prev_meas) / dt;
  p->d_filt += d_alpha * (d_raw - p->d_filt);
  const float dterm = p->kd * p->d_filt;

  p->prev_error = error;
  p->prev_meas  = meas;

  return constrain_float(pterm + p->integral + dterm, -PID_OUT_LIMIT, PID_OUT_LIMIT);
}

/* =============================== Watchdog ================================ */
/* IWDG pilote en registres (pas besoin du module HAL_IWDG).
 * LSI ~32 kHz, prescaler /32 => ~1 kHz => RLR = timeout en ms. */
static void watchdog_start(void)
{
  DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_IWDG_STOP;   /* gele le watchdog en debug */

  IWDG->KR  = 0x0000CCCCU;          /* demarrage */
  IWDG->KR  = 0x00005555U;          /* acces aux registres */
  IWDG->PR  = 3U;                   /* /32 */
  IWDG->RLR = IWDG_TIMEOUT_MS;

  uint32_t guard = 1000000U;
  while ((IWDG->SR != 0U) && (--guard != 0U)) { }

  IWDG->KR  = 0x0000AAAAU;          /* premier rechargement */
}

static inline void watchdog_feed(void)
{
  IWDG->KR = 0x0000AAAAU;
}

/* ============================ Armement / arret =========================== */
static void flight_disarm(DisarmReason_t reason)
{
  Flight_State = FLIGHT_DISARMED;
  Flight_LastDisarmReason = reason;
  motors_write_all(MOTOR_STOP_US);
  pids_reset();
  s_arm_hold = 0U;
  s_disarm_hold = 0U;
  s_idle_cycles = 0U;
  s_spin_inhibit = true;
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
}

static void flight_arm(void)
{
  pids_reset();
  s_arm_hold = 0U;
  s_disarm_hold = 0U;
  s_idle_cycles = 0U;
  s_spin_inhibit = true;     /* pas de rotation tant que throttle bas + yaw recentre */
  Flight_LastDisarmReason = DISARM_NONE;
  Flight_State = FLIGHT_ARMED;
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);   /* LD2 = arme */
}

/* ================================= Taches ================================ */
/* Tache unique et prioritaire : lecture gyro -> filtre -> PID -> mixage -> PWM.
 * Tout est sequentiel et periodique (vTaskDelayUntil), donc le PID ne lit
 * jamais une mesure perimee ni deux fois la meme, et un seul ecrivain
 * touche aux sorties moteurs. */
static void flight_control_task(void *params)
{
  (void)params;

  float gyro_offset[3] = {0.0f, 0.0f, 0.0f};
  float gyro_f[3]      = {0.0f, 0.0f, 0.0f};
  bool  filter_init    = false;
  bool  throttle_low   = true;

  motors_write_all(MOTOR_STOP_US);
  pids_reset();

  /* Phase 1 : calibration. Moteurs coupes, desarme, on recommence tant
   * que le drone bouge ou que le gyro ne repond pas (LD2 clignote). */
  while (!gyro_calibrate(gyro_offset))
  {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    for (uint8_t k = 0; k < 10U; k++)
    {
      vTaskDelay(pdMS_TO_TICKS(10));
      flight_heartbeat();
    }
  }
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /* Phase 2 : boucle de vol */
  TickType_t gyro_last_valid = xTaskGetTickCount();
  uint32_t   last_cycle      = DWT->CYCCNT;
  TickType_t wake            = xTaskGetTickCount();

  for (;;)
  {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(FLIGHT_PERIOD_MS));
    flight_heartbeat();

    /* --- dt reel --- */
    const uint32_t cyc = DWT->CYCCNT;
    float dt = (float)(cyc - last_cycle) / (float)SystemCoreClock;
    last_cycle = cyc;
    dt = constrain_float(dt, FLIGHT_DT_MIN_S, FLIGHT_DT_MAX_S);

    /* --- 1. Gyro : lecture, calibration, filtre --- */
    float g[3];
    const bool gyro_fresh = gyro_read(g);
    const TickType_t now = xTaskGetTickCount();

    if (gyro_fresh)
    {
      gyro_last_valid = now;
      const float a = lpf_alpha(GYRO_LPF_HZ, dt);
      for (uint8_t i = 0; i < 3U; i++)
      {
        g[i] -= gyro_offset[i];
        if (!filter_init) gyro_f[i] = g[i];
        else              gyro_f[i] += a * (g[i] - gyro_f[i]);
      }
      filter_init = true;
    }
    else
    {
      Flight_GyroErrors++;
    }

    /* Age mesure a chaque tour : ne depend d'aucun flag que la tache IMU
     * pourrait laisser a 1 en cas de blocage. */
    const bool gyro_ok = filter_init &&
                         ((TickType_t)(now - gyro_last_valid) < pdMS_TO_TICKS(GYRO_TIMEOUT_MS));

    /* --- 2. Radio --- */
    RcFrame_t rc;
    const bool rc_ok = rc_read(&rc);

    if (rc_ok)
    {
      if      (rc.throttle < THROTTLE_LOW_US)  throttle_low = true;
      else if (rc.throttle > THROTTLE_HIGH_US) throttle_low = false;
    }
    else
    {
      throttle_low = true;
    }

    /* --- 3. Failsafes : perte gyro ou radio => coupure + desarmement --- */
    if (Flight_State == FLIGHT_ARMED)
    {
      if      (!gyro_ok) flight_disarm(DISARM_GYRO_LOST);
      else if (!rc_ok)   flight_disarm(DISARM_RC_LOST);
    }

    /* --- 4. Desarme : moteurs coupes, attente du geste d'armement --- */
    if (Flight_State == FLIGHT_DISARMED)
    {
      motors_write_all(MOTOR_STOP_US);

      const bool sticks_centered = rc_ok &&
          (fabsf(rc.roll  - RC_CENTER_US) < ARM_STICK_CENTER_TOL_US) &&
          (fabsf(rc.pitch - RC_CENTER_US) < ARM_STICK_CENTER_TOL_US);

      /* Conditions : radio OK, gyro OK, throttle bas, sticks centres,
       * yaw a fond a droite pendant 1 s. */
      if (rc_ok && gyro_ok && gyro_fresh && throttle_low && sticks_centered &&
          (rc.yaw > ARM_YAW_HIGH_US))
      {
        if (++s_arm_hold >= ARM_HOLD_CYCLES) flight_arm();
      }
      else
      {
        s_arm_hold = 0U;
      }
      continue;
    }

    /* --- 5. Arme --- */
    /* Mesure ratee mais encore dans la tolerance : on ne recalcule pas le PID
     * sur du perime, les dernieres commandes restent en place. */
    if (!gyro_fresh) continue;

    /* Geste de desarmement : throttle bas + yaw a fond a gauche pendant 1 s */
    if (throttle_low && (rc.yaw < DISARM_YAW_LOW_US))
    {
      if (++s_disarm_hold >= DISARM_HOLD_CYCLES)
      {
        flight_disarm(DISARM_STICKS);
        continue;
      }
    }
    else
    {
      s_disarm_hold = 0U;
    }

    /* Desarmement automatique si on reste au ralenti trop longtemps */
    if (throttle_low)
    {
      if (++s_idle_cycles >= IDLE_DISARM_CYCLES)
      {
        flight_disarm(DISARM_IDLE_TIMEOUT);
        continue;
      }
    }
    else
    {
      s_idle_cycles = 0U;
    }

    /* Apres armement : pas de rotation tant que throttle bas et yaw recentre */
    if (s_spin_inhibit && throttle_low &&
        (fabsf(rc.yaw - RC_CENTER_US) < SPIN_YAW_CENTER_TOL_US))
    {
      s_spin_inhibit = false;
    }

    /* Throttle bas ou rotation interdite : moteurs a l'arret, PID remis a zero */
    if (throttle_low || s_spin_inhibit)
    {
      pids_reset();
      motors_write_all(MOTOR_STOP_US);
      continue;
    }

    /* --- 6. Consignes, PID, mixage, sortie --- */
    const float sp_roll  = RC_RATE_SCALE * rc_stick(rc.roll);
    const float sp_pitch = RC_RATE_SCALE * rc_stick(rc.pitch);
    const float sp_yaw   = RC_RATE_SCALE * rc_stick(rc.yaw);

    const float d_alpha = lpf_alpha(DTERM_LPF_HZ, dt);
    const float out_roll  = pid_update(&s_pid[AXIS_ROLL],  sp_roll,  gyro_f[AXIS_ROLL],  dt, d_alpha);
    const float out_pitch = pid_update(&s_pid[AXIS_PITCH], sp_pitch, gyro_f[AXIS_PITCH], dt, d_alpha);
    const float out_yaw   = pid_update(&s_pid[AXIS_YAW],   sp_yaw,   gyro_f[AXIS_YAW],   dt, d_alpha);

    float m[4];
    mix_motors(rc.throttle, out_roll, out_pitch, out_yaw, m);
    motors_write(m);
  }
}

/* Surveillance : alimente l'IWDG tant que la tache de vol avance.
 * Si elle se bloque (I2C, boucle infinie...), on coupe les moteurs et on
 * laisse le watchdog reinitialiser le MCU. */
static void supervisor_task(void *params)
{
  (void)params;

  uint32_t last_heartbeat = Flight_Heartbeat;
  uint32_t stalled = 0U;
  TickType_t wake = xTaskGetTickCount();

  for (;;)
  {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(SUPERVISOR_PERIOD_MS));

    const uint32_t hb = Flight_Heartbeat;
    if (hb != last_heartbeat)
    {
      last_heartbeat = hb;
      stalled = 0U;
    }
    else if (stalled < SUPERVISOR_STALL_LIMIT)
    {
      stalled++;
    }

    if (stalled >= SUPERVISOR_STALL_LIMIT)
    {
      Motors_EmergencyStop();   /* et on ne nourrit plus l'IWDG => reset */
    }
    else
    {
      watchdog_feed();
    }
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

  /* Moteurs : PWM 400 Hz, signal d'arret (1000 us) des le depart pour que
   * les ESC s'initialisent proprement. */
  motors_pwm_config();
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

  dwt_init();

  /* Si l'init du MPU echoue, la calibration echouera aussi : le drone reste
   * desarme (LD2 clignote). */
  (void)MPU9250_Init();

  /* Radio : capture d'impulsions sur le canal 1 de TIM1..TIM4 */
  rc_start();

  /* Taches */
  BaseType_t status;

  status = xTaskCreate(flight_control_task, "flight", FLIGHT_TASK_STACK_WORDS, NULL,
                       FLIGHT_TASK_PRIORITY, &flight_control_task_handle);
  if (status != pdPASS) { Error_Handler(); }

  status = xTaskCreate(supervisor_task, "supervisor", SUPERVISOR_TASK_STACK_WORDS, NULL,
                       SUPERVISOR_TASK_PRIORITY, &supervisor_task_handle);
  if (status != pdPASS) { Error_Handler(); }

  watchdog_start();

  /* Demarrage de l'ordonnanceur : ne retourne jamais */
  vTaskStartScheduler();

  /* Si on arrive ici, pas assez de memoire pour l'IDLE task / timers */
  Error_Handler();
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

/* Callback de capture d'impulsions (canal 1 de TIM1..TIM4) */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1) return;

  const int8_t idx = rc_index_from_handle(htim);
  if (idx >= 0) rc_capture_isr((uint8_t)idx, htim);
}

#if (configCHECK_FOR_STACK_OVERFLOW > 0)
/* Debordement de pile : on coupe les moteurs, le watchdog fera le reste */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  (void)pcTaskName;
  Motors_EmergencyStop();
  __disable_irq();
  for (;;) { }
}
#endif

#if (configUSE_MALLOC_FAILED_HOOK == 1)
void vApplicationMallocFailedHook(void)
{
  Motors_EmergencyStop();
  __disable_irq();
  for (;;) { }
}
#endif

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
  /* Moteurs a l'arret AVANT de figer le MCU (les timers continuent de
   * generer le dernier compare sinon). Si l'IWDG tourne, il fera un reset. */
  Motors_EmergencyStop();
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
