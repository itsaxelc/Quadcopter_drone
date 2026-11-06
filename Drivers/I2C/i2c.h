#ifndef I2C_H
#define I2C_H

#include <stdint.h>

/* I2C register layout. Reserved fields are used to preserve the hardware register offsets */

typedef struct
{
    volatile uint32_t CR1;          /* Offset: 0x00 */
    volatile uint32_t CR2;          /* Offset: 0x04 */

    uint32_t RESERVED0[2];          /* Offsets: 0x08, 0x0C */

    volatile uint32_t TIMINGR;      /* Offset: 0x10 */

    uint32_t RESERVED1;             /* Offset: 0x14 */

    volatile uint32_t ISR;          /* Offset: 0x18 */
    volatile uint32_t ICR;          /* Offset: 0x1C */

    uint32_t RESERVED2;             /* Offset: 0x20 */

    volatile uint32_t RXDR;         /* Offset: 0x24 */
    volatile uint32_t TXDR;         /* Offset: 0x28 */
} I2C_Registers;

/* Public types */

typedef enum
{
    I2C_OK = 0,
    I2C_ERROR,
    I2C_TIMEOUT,
    I2C_NACK
} I2C_Status;

typedef enum
{
    I2C_INSTANCE_1 = 0,
    I2C_INSTANCE_2,
    I2C_INSTANCE_3
} I2C_Instance;

typedef enum
{
    I2C1_PINS_PB6_PB7 = 0,
    I2C1_PINS_PB8_PB9,

    I2C2_PINS_PB10_PB11,
    I2C2_PINS_PB13_PB14,

    I2C3_PINS_PC0_PC1
} I2C_PinConfig;

typedef struct
{
    I2C_Registers *instance;
} I2C_Handle;

/* Public API */

I2C_Status I2C_Init(I2C_Handle *hi2c, I2C_Instance instance, I2C_PinConfig pins);
I2C_Status I2C_Write(I2C_Handle *hi2c, uint8_t slave_addr, const uint8_t *data, uint8_t size);
I2C_Status I2C_Read(I2C_Handle *hi2c, uint8_t slave_addr, uint8_t *data, uint8_t size);
I2C_Status I2C_WriteRead(I2C_Handle *hi2c, uint8_t slave_addr, const uint8_t *wdata, uint8_t wsize, uint8_t *rdata, uint8_t rsize);

#endif /* I2C_H */
