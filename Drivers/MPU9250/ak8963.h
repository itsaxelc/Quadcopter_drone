#ifndef AK8963_H
#define AK8963_H

#include <stdint.h>
#include "i2c1.h"


#define AK8963_I2C_ADDRESS                          0x0CU
#define AK8963_REG_WHO_AM_I                         0x00U
#define AK8963_WHO_AM_I_VALUE                       0x48U

#define MPU9250_REG_MAG_HXL                         0x03
#define MPU9250_REG_MAG_HXH                         0x04
#define MPU9250_REG_MAG_HYL                         0x05
#define MPU9250_REG_MAG_HYH                         0x06
#define MPU9250_REG_MAG_HZL                         0x07
#define MPU9250_REG_MAG_HZH                         0x08

#define AK8963_REG_ST1                              0x02U
#define AK8963_REG_ST2                              0x09U
#define AK8963_REG_CNTL1                            0x0AU
#define AK8963_REG_CNTL2                            0x0BU
#define AK8963_REG_ASAX                             0x10U
#define AK8963_REG_ASAY                             0x11U
#define AK8963_REG_ASAZ                             0x12U

#define AK8963_REG_ST1_DRDY_Pos                     0U
#define AK8963_REG_ST2_HOFL_Pos                     3U
#define AK8963_REG_ST2_BITM_Pos                     4U
#define AK8963_REG_CNTL1_Mode_Pos                   0U
#define AK8963_REG_CNTL1_Bit_Pos                    4U
#define AK8963_REG_CNTL2_SRST_Pos                   0U

#define AK8963_REG_ST2_HOFL_Msk                     (1U<<AK8963_REG_ST2_HOFL_Pos)
#define AK8963_REG_ST1_DRDY_Msk                     (1U<<AK8963_REG_ST1_DRDY_Pos)
#define AK8963_REG_ST2_BITM_Msk                     (1U<<AK8963_REG_ST2_BITM_Pos)

typedef struct{
    uint8_t ASAX;
    uint8_t ASAY;
    uint8_t ASAZ;
} AK8963_Sensitivity;

typedef struct{
    int16_t HX_raw;
    int16_t HY_raw;
    int16_t HZ_raw;
} AK8963_RawData;

typedef struct{
    float HX;
    float HY;
    float HZ;
} AK8963_Data;

I2C_Status AK8963_Init(void);
I2C_Status AK8963_ReadWhoAmI(uint8_t *id);
I2C_Status AK8963_GetData(AK8963_Data *data);

#endif