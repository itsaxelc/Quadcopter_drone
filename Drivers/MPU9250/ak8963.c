#include "ak8963.h"
#include <stddef.h>

AK8963_Sensitivity sensitivity;

/* Forward declaration of static functions */
static I2C_Status AK8963_ReadRegister(uint8_t reg, uint8_t *value);
static I2C_Status AK8963_WriteRegister(uint8_t reg, uint8_t value);

I2C_Status AK8963_ReadWhoAmI(uint8_t *id)
{
    return AK8963_ReadRegister(AK8963_REG_WHO_AM_I, id);
}

I2C_Status AK8963_Init(void){
    uint8_t dev_id;
    I2C_Status status;
    uint8_t value;
    status = AK8963_ReadWhoAmI(&dev_id);
    if(status != I2C_OK) return status;
    if(dev_id != AK8963_WHO_AM_I_VALUE) return I2C_ERROR;

    /* Power down mode */
    value=0;
    status=AK8963_WriteRegister(AK8963_REG_CNTL1, value);
    if(status!=I2C_OK) return status;

    /* Fuse ROM access mode */
    value=0x0F;
    status=AK8963_WriteRegister(AK8963_REG_CNTL1, value);
    if(status!=I2C_OK) return status;

    /* Reading and storing sensitivity values */
    status=AK8963_ReadRegister(AK8963_REG_ASAX, &sensitivity.ASAX);
    if(status!=I2C_OK) return status;
    status=AK8963_ReadRegister(AK8963_REG_ASAY, &sensitivity.ASAY);
    if(status!=I2C_OK) return status;
    status=AK8963_ReadRegister(AK8963_REG_ASAZ, &sensitivity.ASAZ);
    if(status!=I2C_OK) return status;

    /* Back to Power down mode */
    value=0;
    status=AK8963_WriteRegister(AK8963_REG_CNTL1, value);
    if(status!=I2C_OK) return status;

    /* Continuous measurement 100Hz mode */
    value=0x16;
    status=AK8963_WriteRegister(AK8963_REG_CNTL1, value);
    if(status!=I2C_OK) return status;

    return I2C_OK;
}

static I2C_Status AK8963_GetRawData(AK8963_RawData *raw_data){
    if(raw_data == NULL) return I2C_ERROR;

    uint8_t value;
    I2C_Status status;

    status=AK8963_ReadRegister(AK8963_REG_ST1, &value);
    if(status != I2C_OK) return status;

    if(value==AK8963_REG_ST1_DRDY_Msk)
    {
        uint8_t buffer[7];
        uint8_t start_register=MPU9250_REG_MAG_HXL;
        
        status = I2C1_WriteRead(AK8963_I2C_ADDRESS, &start_register, 1, buffer, 7);
        if (status != I2C_OK) return status;

        uint8_t HOFL_value = buffer[6];
        if ((HOFL_value & AK8963_REG_ST2_HOFL_Msk) == 0U)
        {
            raw_data->HX_raw = (uint16_t)((buffer[1]<<8) | buffer[0]);
            raw_data->HY_raw = (uint16_t)((buffer[3]<<8) | buffer[2]);
            raw_data->HZ_raw = (uint16_t)((buffer[5]<<8) | buffer[4]);
        }
        else return I2C_ERROR;
    }
     return I2C_OK;
}

I2C_Status AK8963_GetData(AK8963_Data *data){
    if(data == NULL) return I2C_ERROR;

    AK8963_RawData raw_data;
    if(AK8963_GetRawData(&raw_data) == I2C_OK){
        raw_data.HX_raw*=((((float)sensitivity.ASAX - 128.0f)*0.5f)/128.0f)+1.0f;
        raw_data.HY_raw*=((((float)sensitivity.ASAY - 128.0f)*0.5f)/128.0f)+1.0f;
        raw_data.HZ_raw*=((((float)sensitivity.ASAZ - 128.0f)*0.5f)/128.0f)+1.0f;

        data->HX=raw_data.HX_raw * 0.15f;
        data->HY=raw_data.HY_raw * 0.15f;
        data->HZ=raw_data.HZ_raw * 0.15f;
    }
    else return I2C_ERROR;

    return I2C_OK;
}

static I2C_Status AK8963_ReadRegister(uint8_t reg, uint8_t *value)
{
    if (value == 0) return I2C_ERROR;
    return I2C1_WriteRead(AK8963_I2C_ADDRESS, &reg, 1U, value, 1U);
}

static I2C_Status AK8963_WriteRegister(uint8_t reg, uint8_t value)
{
    uint8_t buffer[2];

    buffer[0] = reg;
    buffer[1] = value;

    return I2C1_Write(AK8963_I2C_ADDRESS, buffer, 2U);
}