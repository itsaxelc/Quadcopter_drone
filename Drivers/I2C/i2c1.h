#ifndef I2C1_H
#define I2C1_H

#include <stdint.h>

/* Registers addresses */
#define I2C1_BASE       0x40005400
#define I2C1_CR1        (*(volatile uint32_t *)(I2C1_BASE + 0x00))
#define I2C1_CR2        (*(volatile uint32_t *)(I2C1_BASE + 0x04))
#define I2C1_ISR        (*(volatile uint32_t *)(I2C1_BASE + 0x18UL))
#define I2C1_ICR        (*(volatile uint32_t *)(I2C1_BASE + 0x1CUL))
#define I2C1_RXDR       (*(volatile uint32_t *)(I2C1_BASE + 0x24UL))
#define I2C1_TXDR       (*(volatile uint32_t *)(I2C1_BASE + 0x28UL))
#define I2C1_TIMINGR    (*(volatile uint32_t *)(I2C1_BASE + 0x10UL))    

/* Bits positions */
#define I2C_CR1_PE_Pos      0U
#define I2C_CR2_RD_WRN_Pos  10U
#define I2C_CR2_START_Pos   13U
#define I2C_CR2_STOP_Pos    14U
#define I2C_CR2_NBYTES_Pos  16U
#define I2C_CR2_AUTOEND_Pos 25U

#define I2C_ISR_TXIS_Pos    1U
#define I2C_ISR_STOPF_Pos   5U
#define I2C_ISR_NACKF_Pos   4U
#define I2C_ISR_BUSY_Pos    15U
#define I2C_ISR_BERR_Pos    8U
#define I2C_ISR_ARLO_Pos    9U
#define I2C_ISR_OVR_Pos     10U
#define I2C_ISR_RXNE_Pos    2U
#define I2C_ISR_TC_Pos      8U

#define I2C_ICR_STOPCF_Pos    5U
#define I2C_ICR_NACKCF_Pos    4U
#define I2C_ICR_NACKCF_Pos    4U
#define I2C_ICR_BERRCF_Pos    8U
#define I2C_ICR_ARLOCF_Pos    9U
#define I2C_ICR_OVRCF_Pos    10U

/* Masks */
#define I2C_CR1_PE_Msk      (1UL << I2C_CR1_PE_Pos)
#define I2C_CR2_RD_WRN      (1UL << I2C_CR2_RD_WRN_Pos)
#define I2C_CR2_START       (1UL << I2C_CR2_START_Pos)
#define I2C_CR2_AUTOEND     (1UL << I2C_CR2_AUTOEND_Pos)

#define I2C_ISR_RXNE        (1UL << I2C_ISR_RXNE_Pos)
#define I2C_ISR_TC          (1UL << I2C_ISR_TC_Pos)
#define I2C_ISR_TXIS        (1UL << I2C_ISR_TXIS_Pos)
#define I2C_ISR_STOPF       (1UL << I2C_ISR_STOPF_Pos)
#define I2C_ISR_NACKF       (1UL << I2C_ISR_NACKF_Pos)
#define I2C_ISR_BERR        (1UL << I2C_ISR_BERR_Pos)
#define I2C_ISR_ARLO        (1UL << I2C_ISR_ARLO_Pos)
#define I2C_ISR_OVR         (1UL << I2C_ISR_OVR_Pos)
#define I2C_ISR_TXIS        (1UL << I2C_ISR_TXIS_Pos)
#define I2C_ISR_STOPF       (1UL << I2C_ISR_STOPF_Pos)
#define I2C_ISR_NACKF       (1UL << I2C_ISR_NACKF_Pos)
#define I2C_ISR_BUSY        (1UL << I2C_ISR_BUSY_Pos)

#define I2C_ICR_NACKCF      (1UL << I2C_ICR_NACKCF_Pos)
#define I2C_ICR_STOPCF      (1UL << I2C_ICR_STOPCF_Pos)

/* Clock configurations */
#define RCC_BASE                    0x40021000
#define RCC_CCIPR                   (*(volatile uint32_t *)(RCC_BASE + 0x88UL))
#define RCC_APB1ENR1                (*(volatile uint32_t *)(RCC_BASE + 0x58UL))
#define RCC_AHB2ENR                 (*(volatile uint32_t *)(RCC_BASE + 0x4CUL))
#define RCC_APB1ENR1_I2C1EN_Pos     21U
#define RCC_APB1ENR1_I2C1EN_Msk     (1 << RCC_APB1ENR1_I2C1EN_Pos)
#define RCC_AHB2ENR_GPIOBEN_Pos     1U
#define RCC_AHB2ENR_GPIOBEN_Msk     (1 << RCC_AHB2ENR_GPIOBEN_Pos)
#define RCC_CCIPR_I2C1SEL_Pos       12U
#define RCC_CCIPR_I2C1SEL_Msk       (1 << RCC_CCIPR_I2C1SEL_Pos)

#define GPIOB_BASE                  0x48000400
#define GPIOB_MODER                 (*(volatile uint32_t *)(GPIOB_BASE + 0x00UL))
#define GPIOB_OTYPER                (*(volatile uint32_t *)(GPIOB_BASE + 0x04UL))
#define GPIOB_AFRL                  (*(volatile uint32_t *)(GPIOB_BASE + 0x20UL))
#define GPIOB_AFRH                  (*(volatile uint32_t *)(GPIOB_BASE + 0x24UL))
#define GPIOB_OSPEEDR               (*(volatile uint32_t *)(GPIOB_BASE + 0x08UL))

#define I2C1_GPIO_Pin_SCL_Pos1           6U
#define I2C1_GPIO_Pin_SDA_Pos1           7U
#define I2C1_GPIO_Pin_SCL_Pos2           8U
#define I2C1_GPIO_Pin_SDA_Pos2           9U

typedef enum
{
    I2C_OK,
    I2C_ERROR,
    I2C_TIMEOUT,
    I2C_NACK
} I2C_Status;

/* I2C1_PINS_SCLpin_SDApin options :
    1: I2C1_PINS_PB6_PB7 -> PB6/PB7
    2: I2C1_PINS_PB8_PB9 -> PB8/PB9
*/
typedef enum
{
    I2C1_PINS_PB6_PB7,
    I2C1_PINS_PB8_PB9
} I2C1_PinConfig;

void I2C1_Init(I2C1_PinConfig pin_cfg);
I2C_Status I2C1_Read(uint8_t slave_addr, uint8_t *data, uint8_t size);
I2C_Status I2C1_Write(uint8_t slave_addr, const uint8_t *data, uint8_t size);
I2C_Status I2C1_WriteRead(uint8_t slave_addr, const uint8_t *wdata, uint8_t wsize, uint8_t *rdata, uint8_t rsize);

#endif