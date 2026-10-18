#ifndef MPU9250_H
#define MPU9250_H

#include "i2c1.h"
#include <stdint.h>

#define MPU9250_I2C_ADDRESS                     0x68U

#define MPU9250_REG_PWR_MGMT_1                  0x6BU
#define MPU9250_REG_PWR_MGMT_2                  0x6CU
#define MPU9250_REG_WHO_AM_I                    0x75U
#define MPU9250_REG_SMPLRT_DIV                  0x19U
#define MPU9250_REG_CONFIG                      0x1AU
#define MPU9250_REG_GYRO_CONFIG                 0x1BU
#define MPU9250_REG_ACCEL_CONFIG                0x1CU
#define MPU9250_REG_ACCEL_CONFIG2               0x1DU

/* PWR MGMT1 */
#define MPU9250_PWR_MGMT_1_H_RESET_Pos          7U
#define MPU9250_PWR_MGMT_1_H_RESET_Msk          (1UL << MPU9250_PWR_MGMT_1_H_RESET_Pos)

#define MPU9250_PWR_MGMT_1_SLEEP_Pos            6U
#define MPU9250_PWR_MGMT_1_SLEEP_Msk            (1UL << MPU9250_PWR_MGMT_1_SLEEP_Pos)

#define MPU9250_PWR_MGMT_1_CYCLE_Pos            5U
#define MPU9250_PWR_MGMT_1_CYCLE_Msk            (1UL << MPU9250_PWR_MGMT_1_CYCLE_Pos)

#define MPU9250_PWR_MGMT_1_GYRO_STANDBY_Pos     4U
#define MPU9250_PWR_MGMT_1_GYRO_STANDBY_Msk     (1UL << MPU9250_PWR_MGMT_1_GYRO_STANDBY_Pos)

#define MPU9250_PWR_MGMT_1_PD_PTAT_Pos          3U
#define MPU9250_PWR_MGMT_1_PD_PTAT_Msk          (1UL << MPU9250_PWR_MGMT_1_PD_PTAT_Pos)

#define MPU9250_PWR_MGMT_1_CLKSEL_Pos           0U
#define MPU9250_PWR_MGMT_1_CLKSEL_Msk           (0x07UL << MPU9250_PWR_MGMT_1_CLKSEL_Pos)

/* PWR MGMT2 */
#define MPU9250_PWR_MGMT_2_DISABLE_XA_Pos       5U
#define MPU9250_PWR_MGMT_2_DISABLE_XA_Msk       (1UL << MPU9250_PWR_MGMT_2_DISABLE_XA_Pos)

#define MPU9250_PWR_MGMT_2_DISABLE_YA_Pos       4U
#define MPU9250_PWR_MGMT_2_DISABLE_YA_Msk       (1UL << MPU9250_PWR_MGMT_2_DISABLE_YA_Pos)

#define MPU9250_PWR_MGMT_2_DISABLE_ZA_Pos       3U
#define MPU9250_PWR_MGMT_2_DISABLE_ZA_Msk       (1UL << MPU9250_PWR_MGMT_2_DISABLE_ZA_Pos)

#define MPU9250_PWR_MGMT_2_DISABLE_XG_Pos       2U
#define MPU9250_PWR_MGMT_2_DISABLE_XG_Msk       (1UL << MPU9250_PWR_MGMT_2_DISABLE_XG_Pos)

#define MPU9250_PWR_MGMT_2_DISABLE_YG_Pos       1U
#define MPU9250_PWR_MGMT_2_DISABLE_YG_Msk       (1UL << MPU9250_PWR_MGMT_2_DISABLE_YG_Pos)

#define MPU9250_PWR_MGMT_2_DISABLE_ZG_Pos       0U
#define MPU9250_PWR_MGMT_2_DISABLE_ZG_Msk       (1UL << MPU9250_PWR_MGMT_2_DISABLE_ZG_Pos)


#define MPU9250_REG_WHO_AM_I                    0x75U
#define MPU9250_WHO_AM_I_VALUE                  0x71U


#define MPU9250_GYRO_CONFIG_GYRO_FS_SEL_Pos    3U
#define MPU9250_GYRO_CONFIG_GYRO_FS_SEL_Msk    (0x03UL << MPU9250_GYRO_CONFIG_GYRO_FS_SEL_Pos)

#define MPU9250_ACCEL_CONFIG_ACCEL_FS_SEL_Pos   3U
#define MPU9250_ACCEL_CONFIG_ACCEL_FS_SEL_Msk   (0x03UL << MPU9250_ACCEL_CONFIG_ACCEL_FS_SEL_Pos)

#define MPU9250_CONFIG_DLPF_CFG_Pos             0U
#define MPU9250_CONFIG_DLPF_CFG_Msk             (0x07UL << MPU9250_CONFIG_DLPF_CFG_Pos)

#define MPU9250_ACCEL_CONFIG2_A_DLPF_CFG_Pos    0U
#define MPU9250_ACCEL_CONFIG2_A_DLPF_CFG_Msk    (0x07UL << MPU9250_ACCEL_CONFIG2_A_DLPF_CFG_Pos)

#define MPU9250_ACCEL_CONFIG2_ACCEL_FCHOICE_B_Pos  3U
#define MPU9250_ACCEL_CONFIG2_ACCEL_FCHOICE_B_Msk  (1UL << MPU9250_ACCEL_CONFIG2_ACCEL_FCHOICE_B_Pos)

#define MPU9250_REG_ACCEL_XOUT_H                    0x3B
#define MPU9250_REG_ACCEL_XOUT_L                    0x3C
#define MPU9250_REG_ACCEL_YOUT_H                    0x3D
#define MPU9250_REG_ACCEL_YOUT_L                    0x3E
#define MPU9250_REG_ACCEL_ZOUT_H                    0x3F
#define MPU9250_REG_ACCEL_ZOUT_L                    0x40

#define MPU9250_REG_TEMP_OUT_H                      0x41
#define MPU9250_REG_TEMP_OUT_L                      0x42

#define MPU9250_REG_GYRO_XOUT_H                     0x43
#define MPU9250_REG_GYRO_XOUT_L                     0x44
#define MPU9250_REG_GYRO_YOUT_H                     0x45
#define MPU9250_REG_GYRO_YOUT_L                     0x46
#define MPU9250_REG_GYRO_ZOUT_H                     0x47
#define MPU9250_REG_GYRO_ZOUT_L                     0x48

#define MPU9250_REG_USER_CTRL       0x6AU
#define MPU9250_REG_INT_PIN_CFG     0x37U

#define MPU9250_USER_CTRL_I2C_MST_EN_Pos            5U
#define MPU9250_USER_CTRL_I2C_MST_EN_Msk            (1U << MPU9250_USER_CTRL_I2C_MST_EN_Pos)

#define MPU9250_INT_PIN_CFG_BYPASS_EN_Pos           1U
#define MPU9250_INT_PIN_CFG_BYPASS_EN_Msk           (1U << MPU9250_INT_PIN_CFG_BYPASS_EN_Pos)

typedef struct
{
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;

    int16_t temperature;

    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;

} MPU9250_RawData;

typedef struct
{
    float accelX;
    float accelY;
    float accelZ;

    float temperature_c;

    float gyroX;
    float gyroY;
    float gyroZ;

    float magX;
    float magY;
    float magZ;
} MPU9250_Data;

/**
    @brief Reads the WHO_AM_I register of the MPU9250.
    Reads the device identification register (0x75) via I2C
    and stores the received value in the variable pointed to by id.

    @param id Pointer to the variable that receives the device ID. Must not be NULL.
    @return I2C_OK if the I2C transaction succeeds, an error status otherwise.
    @note The expected WHO_AM_I value for an MPU9250 is 0x71.
*/
I2C_Status MPU9250_ReadWhoAmI(uint8_t *id);

I2C_Status MPU9250_Init(void);

/**
    @brief Converts raw MPU9250 measurements into physical units.

    Converts accelerometer readings into meter per seconds squared (mps2), gyroscope readings
    into degrees per second (°/s), and temperature into degrees Celsius (°C).
    
    Conversion factors depend on the sensor configuration:
    - Accelerometer: ±2 g  -> 16384 LSB/g
    - Gyroscope:     ±250 °/s -> 131 LSB/(°/s)
    - Temperature:   333.87 LSB/°C with a 21 °C reference offset

    @param raw Pointer to the raw sensor measurements. Must not be NULL.
    @param data Pointer to the structure receiving converted values. Must not be NULL
    @note The conversion factors must match the full-scale ranges configured in MPU9250_Init().
    @warning If either pointer is NULL, the function returns without performing any conversion.
 */
I2C_Status MPU9250_GetData(MPU9250_Data *data);

I2C_Status MPU9250_GetGyroData(float *gyroX, float *gyroY, float *gyroZ);
I2C_Status MPU9250_GetAccelData(float *accelX, float *accelY, float *accelZ);
I2C_Status MPU9250_GetMagData(float *magX, float *magY, float *magZ);

I2C_Status MPU9250_EnableBypass(void);

#endif

