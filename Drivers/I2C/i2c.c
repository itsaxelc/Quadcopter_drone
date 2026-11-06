#include "i2c.h"

#include <stddef.h>
#include <stdint.h>

/* Compile-time checks to ensure that the public I2C_Registers structure 
    matches the memory layout of the STM32 I2C peripheral registers.
    The offsetof() macro retrieves the offset of each member relative to the beginning of the structure.
 
    The conditional expression evaluates to 1 if the offset is correct, 
    resulting in a valid one-element char array. Otherwise, it evaluates
    to -1, causing a compilation error because an array cannot have a negative size.
*/

/* Private configuration STM32L476xx */

#define I2C_WAIT_LIMIT                  1000000UL

/* I2C peripheral base addresses (STM32L476xx). */
#define I2C1_BASE_ADDR                  0x40005400UL
#define I2C2_BASE_ADDR                  0x40005800UL
#define I2C3_BASE_ADDR                  0x40005C00UL

/* TIMINGR value */
#define I2C_DEFAULT_TIMINGR             0x00420F13UL

/* RCC register addresses */
#define RCC_BASE_ADDR                   0x40021000UL
#define RCC_AHB2ENR_ADDR                (RCC_BASE_ADDR + 0x4CUL)
#define RCC_APB1ENR1_ADDR               (RCC_BASE_ADDR + 0x58UL)
#define RCC_CCIPR_ADDR                  (RCC_BASE_ADDR + 0x88UL)

#define REG32(address)                  (*(volatile uint32_t *)(uintptr_t)(address))

#define RCC_AHB2ENR                     REG32(RCC_AHB2ENR_ADDR)
#define RCC_APB1ENR1                    REG32(RCC_APB1ENR1_ADDR)
#define RCC_CCIPR                       REG32(RCC_CCIPR_ADDR)

/* GPIO port base addresses */
#define GPIOB_BASE_ADDR                 0x48000400UL
#define GPIOC_BASE_ADDR                 0x48000800UL

/* RCC AHB2 GPIO clock enable bits */
#define RCC_AHB2ENR_GPIOBEN_Msk         (1UL << 1U)
#define RCC_AHB2ENR_GPIOCEN_Msk         (1UL << 2U)

/* RCC APB1ENR1 I2C clock enable bits */
#define RCC_APB1ENR1_I2C1EN_Msk         (1UL << 21U)
#define RCC_APB1ENR1_I2C2EN_Msk         (1UL << 22U)
#define RCC_APB1ENR1_I2C3EN_Msk         (1UL << 23U)

/* RCC_CCIPR I2C kernel clock selection fields */
#define RCC_CCIPR_I2C1SEL_Pos           12U
#define RCC_CCIPR_I2C2SEL_Pos           14U
#define RCC_CCIPR_I2C3SEL_Pos           16U
#define RCC_CCIPR_I2CSEL_Msk            0x3UL
#define RCC_I2C_CLOCK_SYSCLK            0x1UL

/* I2C_CR1 bits */
#define I2C_CR1_PE_Pos                  0U
#define I2C_CR1_PE_Msk                  (1UL << I2C_CR1_PE_Pos)

/* I2C_CR2 bits */
#define I2C_CR2_RD_WRN_Pos              10U
#define I2C_CR2_START_Pos               13U
#define I2C_CR2_STOP_Pos                14U
#define I2C_CR2_NBYTES_Pos              16U
#define I2C_CR2_AUTOEND_Pos             25U

/* I2C_ISR bits */
#define I2C_ISR_TXIS_Pos                1U
#define I2C_ISR_RXNE_Pos                2U
#define I2C_ISR_NACKF_Pos               4U
#define I2C_ISR_STOPF_Pos               5U
#define I2C_ISR_TC_Pos                  6U
#define I2C_ISR_BERR_Pos                8U
#define I2C_ISR_ARLO_Pos                9U
#define I2C_ISR_OVR_Pos                 10U
#define I2C_ISR_BUSY_Pos                15U

#define I2C_ISR_BERR_Msk                (1UL << I2C_ISR_BERR_Pos)
#define I2C_ISR_ARLO_Msk                (1UL << I2C_ISR_ARLO_Pos)
#define I2C_ISR_OVR_Msk                 (1UL << I2C_ISR_OVR_Pos)

/* I2C_ICR bits */
#define I2C_ICR_NACKCF_Pos              4U
#define I2C_ICR_STOPCF_Pos              5U
#define I2C_ICR_BERRCF_Pos              8U
#define I2C_ICR_ARLOCF_Pos              9U
#define I2C_ICR_OVRCF_Pos               10U

/* GPIO register layout (STM32L476) */
typedef struct
{
    volatile uint32_t MODER;          /* 0x00 */
    volatile uint32_t OTYPER;         /* 0x04 */
    volatile uint32_t OSPEEDR;        /* 0x08 */
    volatile uint32_t PUPDR;          /* 0x0C */
    volatile uint32_t IDR;            /* 0x10 */
    volatile uint32_t ODR;            /* 0x14 */
    volatile uint32_t BSRR;           /* 0x18 */
    volatile uint32_t LCKR;           /* 0x1C */
    volatile uint32_t AFR[2];         /* 0x20, 0x24 */
} GPIO_Registers;

/* Hardware-specific information hidden from the application. */
typedef struct
{
    I2C_Instance instance_id;
    I2C_PinConfig pins;
    uint32_t i2c_base_addr;
    uint32_t gpio_base_addr;
    uint32_t gpio_clock_mask;
    uint32_t i2c_clock_mask;
    uint8_t scl_pin;
    uint8_t sda_pin;
    uint8_t alternate_function;
    uint8_t ccipr_sel_pos;
} I2C_HardwareConfig;

/* These are the supported pin combinations for STM32L476xx */
static const I2C_HardwareConfig I2C_HARDWARE_CONFIGS[] =
{
    {I2C_INSTANCE_1, I2C1_PINS_PB6_PB7, I2C1_BASE_ADDR, GPIOB_BASE_ADDR, RCC_AHB2ENR_GPIOBEN_Msk, 
        RCC_APB1ENR1_I2C1EN_Msk, 6U, 7U, 4U, RCC_CCIPR_I2C1SEL_Pos},
    {I2C_INSTANCE_1, I2C1_PINS_PB8_PB9, I2C1_BASE_ADDR, GPIOB_BASE_ADDR, RCC_AHB2ENR_GPIOBEN_Msk, 
        RCC_APB1ENR1_I2C1EN_Msk, 8U, 9U, 4U, RCC_CCIPR_I2C1SEL_Pos},
    {I2C_INSTANCE_2, I2C2_PINS_PB10_PB11, I2C2_BASE_ADDR, GPIOB_BASE_ADDR, RCC_AHB2ENR_GPIOBEN_Msk, 
        RCC_APB1ENR1_I2C2EN_Msk, 10U, 11U, 4U, RCC_CCIPR_I2C2SEL_Pos},
    {I2C_INSTANCE_2, I2C2_PINS_PB13_PB14, I2C2_BASE_ADDR, GPIOB_BASE_ADDR, RCC_AHB2ENR_GPIOBEN_Msk, 
        RCC_APB1ENR1_I2C2EN_Msk, 13U, 14U, 4U, RCC_CCIPR_I2C2SEL_Pos},
    {I2C_INSTANCE_3, I2C3_PINS_PC0_PC1, I2C3_BASE_ADDR, GPIOC_BASE_ADDR, RCC_AHB2ENR_GPIOCEN_Msk,
        RCC_APB1ENR1_I2C3EN_Msk, 0U, 1U, 4U, RCC_CCIPR_I2C3SEL_Pos}
};

/* Private function declarations */

static const I2C_HardwareConfig *I2C_FindHardwareConfig(I2C_Instance instance, I2C_PinConfig pins);
static void GPIO_Config(GPIO_Registers *gpio, uint8_t scl_pin, uint8_t sda_pin, uint8_t alternate_function);
static I2C_Status I2C_CheckErrors(uint32_t isr);
static void I2C_AbortTransfer(I2C_Handle *hi2c);
static I2C_Status I2C_WaitForTXIS(I2C_Handle *hi2c, uint32_t timeout);
static I2C_Status I2C_WaitForSTOP(I2C_Handle *hi2c, uint32_t timeout);
static I2C_Status I2C_WaitForBusFree(I2C_Handle *hi2c, uint32_t timeout);
static I2C_Status I2C_WaitForRXNE(I2C_Handle *hi2c, uint32_t timeout);
static I2C_Status I2C_WaitForTC(I2C_Handle *hi2c, uint32_t timeout);

/* Hardware configuration helpers 
This function will select the corresponding configurations based on the I2C instance and the pins config chosen by the user*/
static const I2C_HardwareConfig *I2C_FindHardwareConfig(I2C_Instance instance, I2C_PinConfig pins)
{
    uint32_t i;
    const uint32_t config_count = sizeof(I2C_HARDWARE_CONFIGS)/sizeof(I2C_HARDWARE_CONFIGS[0]);
    for (i=0; i<config_count; i++)
    {
        if ((I2C_HARDWARE_CONFIGS[i].instance_id == instance) && (I2C_HARDWARE_CONFIGS[i].pins == pins))
        {
            return &I2C_HARDWARE_CONFIGS[i];
        }
    }
    return NULL;
}

/* This function will configure the GPIO for the use of I2C alternate function */
static void GPIO_Config(GPIO_Registers *gpio, uint8_t scl_pin, uint8_t sda_pin, uint8_t alternate_function)
{
    uint32_t pin_mode_mask;
    uint32_t pin_type_mask;
    uint32_t af_index;
    uint32_t scl_af_shift;
    uint32_t sda_af_shift;
    uint32_t af_mask;

    pin_mode_mask = (3UL << ((uint32_t)scl_pin * 2U)) | (3UL << ((uint32_t)sda_pin * 2U));
    pin_type_mask = (1UL << scl_pin) | (1UL << sda_pin);

    /* Select alternate-function mode for SCL and SDA. */
    gpio->MODER &= ~pin_mode_mask;
    gpio->MODER |=  (2UL << ((uint32_t)scl_pin * 2U)) | (2UL << ((uint32_t)sda_pin * 2U));

    /* I2C pins must use open-drain output type. */
    gpio->OTYPER |= pin_type_mask;

    /* Select very-high GPIO speed, as in the original driver. */
    gpio->OSPEEDR &= ~pin_mode_mask;
    gpio->OSPEEDR |= pin_mode_mask;

    /* Select the alternate-function number for each pin. */
    af_index = (uint32_t)scl_pin / 8U;
    scl_af_shift = ((uint32_t)scl_pin % 8U) * 4U;
    sda_af_shift = ((uint32_t)sda_pin % 8U) * 4U;

    af_mask = (0xFUL << scl_af_shift) | (0xFUL << sda_af_shift);

    gpio->AFR[af_index] &= ~af_mask;
    gpio->AFR[af_index] |= ((uint32_t)alternate_function << scl_af_shift) | ((uint32_t)alternate_function << sda_af_shift);
}


/* This is a generic I2C initialization function
The user will just provide a handle to an I2C bus, the I2C instance he wants to use and inside of this instance, 
the pins he wants to use. All the configs and inits will be done in background */

I2C_Status I2C_Init(I2C_Handle *hi2c, I2C_Instance instance, I2C_PinConfig pins)
{
    const I2C_HardwareConfig *config;
    GPIO_Registers *gpio;
    uint32_t ccipr_mask;

    if (hi2c == NULL)
    {
        return I2C_ERROR;
    }

    config = I2C_FindHardwareConfig(instance, pins);
    if (config == NULL)
    {
        hi2c->instance = NULL;
        return I2C_ERROR;
    }

    /* Set the peripheral register block selected by the caller. */
    hi2c->instance = (I2C_Registers *)(uintptr_t)config->i2c_base_addr;

    /* 1. Enable the GPIO port clock. */
    RCC_AHB2ENR |= config->gpio_clock_mask;
    (void)RCC_AHB2ENR;

    /* 2. Configure the selected SCL/SDA pins. */
    gpio = (GPIO_Registers *)(uintptr_t)config->gpio_base_addr;
    GPIO_Config(gpio,config->scl_pin, config->sda_pin, config->alternate_function);

    /* 3. Enable the selected I2C peripheral clock. */
    RCC_APB1ENR1 |= config->i2c_clock_mask;
    (void)RCC_APB1ENR1;

    /* 4. Select SYSCLK (01) as this I2C peripheral's kernel clock. */
    ccipr_mask = RCC_CCIPR_I2CSEL_Msk << config->ccipr_sel_pos;
    RCC_CCIPR = (RCC_CCIPR & ~ccipr_mask) | (RCC_I2C_CLOCK_SYSCLK << config->ccipr_sel_pos);

    /* 5. Disable I2C, program timing, then enable the peripheral. */
    hi2c->instance->CR1 &= ~I2C_CR1_PE_Msk;
    hi2c->instance->TIMINGR = I2C_DEFAULT_TIMINGR;
    hi2c->instance->CR1 |= I2C_CR1_PE_Msk;

    return I2C_OK;
}


/* Error handling */
static I2C_Status I2C_CheckErrors(uint32_t isr)
{
    if ((isr & (1UL << I2C_ISR_NACKF_Pos)) != 0U) return I2C_NACK;
    if ((isr & (I2C_ISR_BERR_Msk | I2C_ISR_ARLO_Msk | I2C_ISR_OVR_Msk)) != 0U) return I2C_ERROR;
    
    return I2C_OK;
}

static void I2C_AbortTransfer(I2C_Handle *hi2c)
{
    uint32_t timeout = I2C_WAIT_LIMIT;
    const uint32_t error_mask = I2C_ISR_BERR_Msk | I2C_ISR_ARLO_Msk | I2C_ISR_OVR_Msk;

    if ((hi2c == NULL) || (hi2c->instance == NULL)) return;

    /* Request STOP. */
    hi2c->instance->CR2 |= (1UL << I2C_CR2_STOP_Pos);

    /* Wait for STOPF, or stop waiting if a bus error occurs. */
    while ((hi2c->instance->ISR & (1UL << I2C_ISR_STOPF_Pos)) == 0U)
    {
        if ((hi2c->instance->ISR & error_mask) != 0U) break;

        if (timeout == 0U) break;
        timeout--;
    }

    /* Clear STOPF, NACKF, BERR, ARLO and OVR. */
    hi2c->instance->ICR = (1UL << I2C_ICR_STOPCF_Pos)  | (1UL << I2C_ICR_NACKCF_Pos)  | (1UL << I2C_ICR_BERRCF_Pos)  |
                         (1UL << I2C_ICR_ARLOCF_Pos)  | (1UL << I2C_ICR_OVRCF_Pos);
}

/* I2C status polling helpers */
static I2C_Status I2C_WaitForTXIS(I2C_Handle *hi2c, uint32_t timeout)
{
    while (1)
    {
        uint32_t isr = hi2c->instance->ISR;
        I2C_Status status = I2C_CheckErrors(isr);

        if (status != I2C_OK)
        {
            I2C_AbortTransfer(hi2c);
            return status;
        }

        if ((isr & (1UL << I2C_ISR_TXIS_Pos)) != 0U)
        {
            return I2C_OK;
        }

        if (timeout == 0U)
        {
            I2C_AbortTransfer(hi2c);
            return I2C_TIMEOUT;
        }

        timeout--;
    }
}

static I2C_Status I2C_WaitForSTOP(I2C_Handle *hi2c, uint32_t timeout)
{
    while (1)
    {
        uint32_t isr = hi2c->instance->ISR;
        I2C_Status status = I2C_CheckErrors(isr);

        if (status != I2C_OK)
        {
            I2C_AbortTransfer(hi2c);
            return status;
        }

        if ((isr & (1UL << I2C_ISR_STOPF_Pos)) != 0U)
        {
            return I2C_OK;
        }

        if (timeout == 0U)
        {
            I2C_AbortTransfer(hi2c);
            return I2C_TIMEOUT;
        }

        timeout--;
    }
}

static I2C_Status I2C_WaitForBusFree(I2C_Handle *hi2c, uint32_t timeout)
{
    while (1)
    {
        uint32_t isr = hi2c->instance->ISR;

        if ((isr & (1UL << I2C_ISR_BUSY_Pos)) == 0U)
        {
            return I2C_OK;
        }

        if (timeout == 0U)
        {
            return I2C_TIMEOUT;
        }

        timeout--;
    }
}

static I2C_Status I2C_WaitForRXNE(I2C_Handle *hi2c, uint32_t timeout)
{
    while (1)
    {
        uint32_t isr = hi2c->instance->ISR;
        I2C_Status status = I2C_CheckErrors(isr);

        if (status != I2C_OK)
        {
            I2C_AbortTransfer(hi2c);
            return status;
        }

        if ((isr & (1UL << I2C_ISR_RXNE_Pos)) != 0U)
        {
            return I2C_OK;
        }

        if (timeout == 0U)
        {
            I2C_AbortTransfer(hi2c);
            return I2C_TIMEOUT;
        }

        timeout--;
    }
}

static I2C_Status I2C_WaitForTC(I2C_Handle *hi2c, uint32_t timeout)
{
    while (1)
    {
        uint32_t isr = hi2c->instance->ISR;
        I2C_Status status = I2C_CheckErrors(isr);

        if (status != I2C_OK)
        {
            I2C_AbortTransfer(hi2c);
            return status;
        }

        if ((isr & (1UL << I2C_ISR_TC_Pos)) != 0U)
        {
            return I2C_OK;
        }

        if (timeout == 0U)
        {
            I2C_AbortTransfer(hi2c);
            return I2C_TIMEOUT;
        }

        timeout--;
    }
}

/* Public transfer API */
I2C_Status I2C_Write(I2C_Handle *hi2c, uint8_t slave_addr, const uint8_t *data, uint8_t size)
{
    I2C_Status status;
    uint32_t i;

    if ((hi2c == NULL) || (hi2c->instance == NULL) || (data == NULL) || (size == 0U) || (slave_addr > 0x7FU))
    {
        return I2C_ERROR;
    }

    status = I2C_WaitForBusFree(hi2c, I2C_WAIT_LIMIT);
    if (status != I2C_OK)
    {
        return status;
    }

    /* START + AUTOEND: hardware generates STOP after the last byte. */
    hi2c->instance->CR2 = ((uint32_t)slave_addr << 1U) | ((uint32_t)size << I2C_CR2_NBYTES_Pos) |
                          (1UL << I2C_CR2_AUTOEND_Pos) | (1UL << I2C_CR2_START_Pos);

    for (i = 0U; i < (uint32_t)size; i++)
    {
        status = I2C_WaitForTXIS(hi2c, I2C_WAIT_LIMIT);
        if (status != I2C_OK)
        {
            return status;
        }

        hi2c->instance->TXDR = data[i];
    }

    status = I2C_WaitForSTOP(hi2c, I2C_WAIT_LIMIT);
    if (status != I2C_OK)
    {
        return status;
    }

    hi2c->instance->ICR = (1UL << I2C_ICR_STOPCF_Pos);

    return I2C_OK;
}

I2C_Status I2C_Read(I2C_Handle *hi2c, uint8_t slave_addr, uint8_t *data, uint8_t size)
{
    I2C_Status status;
    uint32_t i;

    if ((hi2c == NULL) || (hi2c->instance == NULL) || (data == NULL) || (size == 0U) || (slave_addr > 0x7FU))
    {
        return I2C_ERROR;
    }

    status = I2C_WaitForBusFree(hi2c, I2C_WAIT_LIMIT);
    if (status != I2C_OK)
    {
        return status;
    }

    /* START + RD_WRN + AUTOEND: hardware generates STOP after the read. */
    hi2c->instance->CR2 = ((uint32_t)slave_addr << 1U) | ((uint32_t)size << I2C_CR2_NBYTES_Pos) |
                          (1UL << I2C_CR2_RD_WRN_Pos) | (1UL << I2C_CR2_AUTOEND_Pos) | (1UL << I2C_CR2_START_Pos);

    for (i = 0U; i < (uint32_t)size; i++)
    {
        status = I2C_WaitForRXNE(hi2c, I2C_WAIT_LIMIT);
        if (status != I2C_OK)
        {
            return status;
        }

        data[i] = (uint8_t)hi2c->instance->RXDR;
    }

    status = I2C_WaitForSTOP(hi2c, I2C_WAIT_LIMIT);
    if (status != I2C_OK)
    {
        return status;
    }

    hi2c->instance->ICR = (1UL << I2C_ICR_STOPCF_Pos);

    return I2C_OK;
}

I2C_Status I2C_WriteRead(I2C_Handle *hi2c, uint8_t slave_addr, const uint8_t *wdata, uint8_t wsize, uint8_t *rdata, uint8_t rsize)
{
    I2C_Status status;
    uint32_t i;

    if ((hi2c == NULL) || (hi2c->instance == NULL) || (wdata == NULL) || (rdata == NULL) ||
        (wsize == 0U) || (rsize == 0U) || (slave_addr > 0x7FU))
    {
        return I2C_ERROR;
    }

    status = I2C_WaitForBusFree(hi2c, I2C_WAIT_LIMIT);
    if (status != I2C_OK)
    {
        return status;
    }

    /* Write phase without AUTOEND: do not issue STOP yet. */
    hi2c->instance->CR2 = ((uint32_t)slave_addr << 1U) | ((uint32_t)wsize << I2C_CR2_NBYTES_Pos) | (1UL << I2C_CR2_START_Pos);

    for (i = 0U; i < (uint32_t)wsize; i++)
    {
        status = I2C_WaitForTXIS(hi2c, I2C_WAIT_LIMIT);
        if (status != I2C_OK)
        {
            return status;
        }

        hi2c->instance->TXDR = wdata[i];
    }

    /* TC indicates that the write phase is complete; issue repeated START. */
    status = I2C_WaitForTC(hi2c, I2C_WAIT_LIMIT);
    if (status != I2C_OK)
    {
        return status;
    }

    hi2c->instance->CR2 = ((uint32_t)slave_addr << 1U) | ((uint32_t)rsize << I2C_CR2_NBYTES_Pos) | (1UL << I2C_CR2_RD_WRN_Pos) |
                          (1UL << I2C_CR2_AUTOEND_Pos) | (1UL << I2C_CR2_START_Pos);

    for (i = 0U; i < (uint32_t)rsize; i++)
    {
        status = I2C_WaitForRXNE(hi2c, I2C_WAIT_LIMIT);
        if (status != I2C_OK)
        {
            return status;
        }

        rdata[i] = (uint8_t)hi2c->instance->RXDR;
    }

    status = I2C_WaitForSTOP(hi2c, I2C_WAIT_LIMIT);
    if (status != I2C_OK)
    {
        return status;
    }

    hi2c->instance->ICR = (1UL << I2C_ICR_STOPCF_Pos);

    return I2C_OK;
}
