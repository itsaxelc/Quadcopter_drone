#include "mpu9250.h"
#include "ak8963.h"
#include <stddef.h>
#define MPU9250_GRAVITY            9.80665f

/* Forward declaration of static functions */
static I2C_Status MPU9250_WriteRegister(uint8_t reg, uint8_t value);
static I2C_Status MPU9250_ReadRegister(uint8_t reg, uint8_t *value);

I2C_Status MPU9250_ReadWhoAmI(uint8_t *id)
{
    uint8_t reg = MPU9250_REG_WHO_AM_I;
    if (id == 0) return I2C_ERROR;
    return I2C1_WriteRead(MPU9250_I2C_ADDRESS, &reg, 1U, id, 1U);
}

static I2C_Status MPU9250_WriteRegister(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    return I2C1_Write(MPU9250_I2C_ADDRESS, data, 2U);
}

static I2C_Status MPU9250_ReadRegister(uint8_t reg, uint8_t *value)
{
    if (value == NULL)
    {
        return I2C_ERROR;
    }
    return I2C1_WriteRead(MPU9250_I2C_ADDRESS, &reg, 1U,value, 1U);
}

I2C_Status MPU9250_Init(void)
{
    I2C_Status status;
    uint8_t value;
    uint8_t id;

    /* Verify device identity */
    status = MPU9250_ReadWhoAmI(&id);
    if (status != I2C_OK) return status;
    if (id != MPU9250_WHO_AM_I_VALUE) return I2C_ERROR;

    /* ------------------------------------------
       PWR_MGMT_1
       SLEEP        = 0
       CYCLE        = 0
       GYRO_STANDBY = 0
       PD_PTAT      = 0
       CLKSEL       = 1 Auto selects the best available clock source 
                      PLL if ready, else use the Internal oscillator
       ------------------------------------------ */

    value = 0U;
    value |= (1U << MPU9250_PWR_MGMT_1_CLKSEL_Pos);
    status = MPU9250_WriteRegister(MPU9250_REG_PWR_MGMT_1, value);

    if (status != I2C_OK) return status;


    /* ------------------------------------------
       PWR_MGMT_2
       Tous les axes actifs
       ------------------------------------------ */

    value = 0U;
    status = MPU9250_WriteRegister(MPU9250_REG_PWR_MGMT_2, value);
    if (status != I2C_OK) return status;


    /* ------------------------------------------
       SMPLRT_DIV
       
       Fsample = Gyro Output Rate / (1 + DIV)

       Avec DIV = 0 :
       Fsample = 1 kHz
       ------------------------------------------ */

    value = 0U;
    status = MPU9250_WriteRegister(MPU9250_REG_SMPLRT_DIV, value);

    if (status != I2C_OK)
        return status;


    /* ------------------------------------------
       CONFIG
       Gyroscope DLPF
       ------------------------------------------ */

    value = 0U;
    value |= (1U << MPU9250_CONFIG_DLPF_CFG_Pos);
    status = MPU9250_WriteRegister(MPU9250_REG_CONFIG, value);

    if (status != I2C_OK) return status;

    /* ------------------------------------------
       GYRO_CONFIG
       FS_SEL = 00 -> ±250 °/s -> Sensitivity Scale Factor = 131 LSB/(°/s)
       FCHOICE_B = 00
       Self-test = 0
       ------------------------------------------ */

    value = 0U;
    value &= ~MPU9250_GYRO_CONFIG_GYRO_FS_SEL_Msk;
    status = MPU9250_WriteRegister(MPU9250_REG_GYRO_CONFIG, value);

    if (status != I2C_OK)
        return status;


    /* ------------------------------------------
       ACCEL_CONFIG
       FS_SEL = 00 -> ±2 g -> Sensitivity Scale Factor = 16384 LSB/g
       Self-test = 0
       ------------------------------------------ */

    value = 0U;
    value &= ~MPU9250_ACCEL_CONFIG_ACCEL_FS_SEL_Msk;
    status = MPU9250_WriteRegister(MPU9250_REG_ACCEL_CONFIG, value);

    if (status != I2C_OK)
        return status;


    /* ------------------------------------------
       ACCEL_CONFIG2
       DLPF activé
       ------------------------------------------ */

    value = 0U;
    value &= ~MPU9250_ACCEL_CONFIG2_ACCEL_FCHOICE_B_Msk;
    value |= (1U << MPU9250_ACCEL_CONFIG2_A_DLPF_CFG_Pos);
    status = MPU9250_WriteRegister(MPU9250_REG_ACCEL_CONFIG2, value);

    if (status != I2C_OK) return status;

    return I2C_OK;
}


static I2C_Status MPU9250_GetRawData(MPU9250_RawData *data)
{
    uint8_t buffer[14];
    uint8_t start_register = MPU9250_REG_ACCEL_XOUT_H;

    I2C_Status status;

    status = I2C1_WriteRead(MPU9250_I2C_ADDRESS, &start_register, 1, buffer, 14);

    if (status != I2C_OK) return status;

    data->accelX = (int16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
    data->accelY = (int16_t)(((uint16_t)buffer[2] << 8) | buffer[3]);
    data->accelZ = (int16_t)(((uint16_t)buffer[4] << 8) | buffer[5]);

    data->temperature = (int16_t)(((uint16_t)buffer[6] << 8) | buffer[7]);

    data->gyroX = (int16_t)(((uint16_t)buffer[8] << 8) | buffer[9]);
    data->gyroY = (int16_t)(((uint16_t)buffer[10] << 8) | buffer[11]);
    data->gyroZ = (int16_t)(((uint16_t)buffer[12] << 8) | buffer[13]);

    return I2C_OK;
}

I2C_Status MPU9250_GetData(MPU9250_Data *data)
{
    if (data == NULL) return I2C_ERROR;

    I2C_Status status;
    MPU9250_RawData raw;
    AK8963_Data ak8963_data;

    status = MPU9250_GetRawData(&raw);
    if (status != I2C_OK) return status;

    status = AK8963_GetData(&ak8963_data);
    if (status != I2C_OK) return status;

    /* ±2 g -> Sensitivity Scale Factor = 16384 LSB/g */
    data->accelX = (float)(raw.accelX / 16384.0f)*MPU9250_GRAVITY;
    data->accelY = (float)(raw.accelY / 16384.0f)*MPU9250_GRAVITY;
    data->accelZ = (float)(raw.accelZ / 16384.0f)*MPU9250_GRAVITY;

    /* Temperature Sensitivity -> 333.87 LSB/(°C) with 21°C offset */
    /* TEMP_degC = (TEMP_OUT / 333.87) + 21degC */
    data->temperature_c = (float)raw.temperature / 333.87f + 21.0f;

    /* ±250 °/s -> Sensitivity Scale Factor = 131 LSB/(°/s) */
    data->gyroX = (float)raw.gyroX / 131.0f;
    data->gyroY = (float)raw.gyroY / 131.0f;
    data->gyroZ = (float)raw.gyroZ / 131.0f;

    /* Got directly from AK8963 driver, no need of conversion */
    data->magX = ak8963_data.HX;
    data->magY = ak8963_data.HY;
    data->magZ = ak8963_data.HZ;

    return I2C_OK;
}

I2C_Status MPU9250_EnableBypass(void)
{
    I2C_Status status;
    uint8_t value;
    /* Disable the MPU9250 internal I2C master */
    status = MPU9250_WriteRegister(MPU9250_REG_USER_CTRL, 0x00U);

    if (status != I2C_OK) return status;

    /* Enable direct access to the AK8963 */
    value = MPU9250_INT_PIN_CFG_BYPASS_EN_Msk;
    status = MPU9250_WriteRegister(MPU9250_REG_INT_PIN_CFG, value);

    return status;
}

I2C_Status MPU9250_GetGyroData(float *gyroX, float *gyroY, float *gyroZ){
    if(gyroX ==NULL || gyroY == NULL || gyroZ == NULL) return I2C_ERROR;

    I2C_Status status;
    MPU9250_Data data;

    status = MPU9250_GetData(&data);
    if(status != I2C_OK) return status;

    *gyroX = data.gyroX;
    *gyroY = data.gyroY;
    *gyroZ = data.gyroZ;

    return I2C_OK;
}

I2C_Status MPU9250_GetAccelData(float *accelX, float *accelY, float *accelZ){
    if(accelX ==NULL || accelY == NULL || accelZ == NULL) return I2C_ERROR;

    I2C_Status status;
    MPU9250_Data data;

    status = MPU9250_GetData(&data);
    if(status != I2C_OK) return status;

    *accelX = data.accelX;
    *accelY = data.accelY;
    *accelZ = data.accelZ;

    return I2C_OK;
}

I2C_Status MPU9250_GetMagData(float *magX, float *magY, float *magZ){
    if(magX ==NULL || magY == NULL || magZ == NULL) return I2C_ERROR;

    I2C_Status status;
    MPU9250_Data data;

    status = MPU9250_GetData(&data);
    if(status != I2C_OK) return status;

    *magX = data.magX;
    *magY = data.magY;
    *magZ = data.magZ;

    return I2C_OK;
}