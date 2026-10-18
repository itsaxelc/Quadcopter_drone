#include "i2c2.h"
#include<stdint.h>

#define I2C_WAIT_LIMIT    1000000UL

static void I2C2_AbortTransfer(void);
static I2C_Status I2C2_WaitForTXIS(uint32_t timeout);
static I2C_Status I2C2_WaitForSTOP(uint32_t timeout);
static I2C_Status I2C2_WaitForBusFree(uint32_t limit);
static I2C_Status I2C2_CheckErrors(uint32_t isr);
static I2C_Status I2C2_WaitForRXNE(uint32_t timeout);
static I2C_Status I2C2_WaitForTC(uint32_t timeout);

/*
    Vérifie les flags d'erreur d'un instantané du registre ISR.
    @param  isr  Valeur de I2C2_ISR déjà lue.
    @return_val I2C_NACK si NACKF, I2C_ERROR si BERR/ARLO/OVR, sinon I2C_OK.
*/
static I2C_Status I2C2_CheckErrors(uint32_t isr)
{
    if (isr & (1UL << I2C_ISR_NACKF_Pos)) return I2C_NACK;
    if (isr & ((1UL << I2C_ISR_BERR_Pos) |
               (1UL << I2C_ISR_ARLO_Pos) |
               (1UL << I2C_ISR_OVR_Pos))) return I2C_ERROR;
    return I2C_OK;
}

/*
    @brief  Interrompt un transfert en erreur : demande un STOP, attend STOPF
    (sortie anticipée sur BERR/OVR/ARLO ou limite atteinte, puis efface les flags d'état.
*/
static void I2C2_AbortTransfer(void)
{
    // Demander un STOP
    I2C2_CR2 |= (1UL << I2C_CR2_STOP_Pos);

    // Attendre que le STOP soit détecté
    uint32_t limit = I2C_WAIT_LIMIT;
    uint32_t error_mask = (I2C_ISR_BERR) | (I2C_ISR_OVR) | (I2C_ISR_ARLO);

    while (!(I2C2_ISR & (1UL << I2C_ISR_STOPF_Pos)))
    {
        if(I2C2_ISR & error_mask)  break;

        if (limit == 0)
            break;

        limit--;
    }

    // Effacer les flags d'état
    I2C2_ICR = (1UL << I2C_ICR_STOPCF_Pos) | (1UL << I2C_ICR_NACKCF_Pos) | (1UL << I2C_ICR_BERRCF_Pos)
                | (1UL << I2C_ICR_ARLOCF_Pos) | (1UL << I2C_ICR_OVRCF_Pos);
}

/*
    @brief  Attend que TXDR soit prêt à recevoir un octet (TXIS). Les erreurs sont testées avant le flag.
    @param  timeout  Nombre maximal d'itérations d'attente.
    @retval I2C_OK, I2C_NACK, I2C_ERROR ou I2C_TIMEOUT (abort appelé en cas d'échec).
*/
static I2C_Status I2C2_WaitForTXIS(uint32_t timeout)
{
    while (1)
    {
        uint32_t isr = I2C2_ISR;

        if (isr & (1UL << I2C_ISR_TXIS_Pos))
        {
            return I2C_OK;
        }

        I2C_Status status;

        status = I2C2_CheckErrors(isr);

        if(status != I2C_OK)
        {
            I2C2_AbortTransfer();
            return status;
        }

        if (timeout == 0)
        {
            I2C2_AbortTransfer();
            return I2C_TIMEOUT;
        }

        timeout--;
    }
}

/*
    @brief  Attend la détection du STOP (STOPF).
    @param  timeout  Nombre maximal d'itérations d'attente.
    @retval I2C_OK, I2C_NACK, I2C_ERROR ou I2C_TIMEOUT (abort appelé en cas d'échec).
*/
static I2C_Status I2C2_WaitForSTOP(uint32_t timeout)
{
    while (1)
    {
        uint32_t isr = I2C2_ISR;

        I2C_Status status;

        status = I2C2_CheckErrors(isr);

        if(status != I2C_OK)
        {
            I2C2_AbortTransfer();
            return status;
        }

        if (isr & (1UL << I2C_ISR_STOPF_Pos))
        {
            return I2C_OK;
        }

        if (timeout == 0)
        {
            I2C2_AbortTransfer();
            return I2C_TIMEOUT;
        }

        timeout--;
    }
}

/*
    @brief  Attend que le bus soit libre (BUSY = 0).
    @param  limit  Nombre maximal d'itérations d'attente.
    @retval I2C_OK si le bus est libre, I2C_TIMEOUT sinon.
*/
static I2C_Status I2C2_WaitForBusFree(uint32_t limit)
{
    while (1)
    {
        uint32_t isr = I2C2_ISR;

        if (!(isr & (1UL << I2C_ISR_BUSY_Pos)))
        {
            return I2C_OK;
        }

        if (limit == 0)
        {
            return I2C_TIMEOUT;
        }

        limit--;
    }
}

/*
    @brief  Attend qu'un octet reçu soit disponible dans RXDR (RXNE).
    @param  timeout  Nombre maximal d'itérations d'attente.
    @retval I2C_OK, I2C_NACK, I2C_ERROR ou I2C_TIMEOUT (abort appelé en cas d'échec).
*/
static I2C_Status I2C2_WaitForRXNE(uint32_t timeout)
{
    I2C_Status status;

    while (1)
    {
        uint32_t isr = I2C2_ISR;

        status = I2C2_CheckErrors(isr);

        if (status != I2C_OK)
        {
            I2C2_AbortTransfer();
            return status;
        }

        if (isr & (1UL << I2C_ISR_RXNE_Pos))
        {
            return I2C_OK;
        }

        if (timeout == 0)
        {
            I2C2_AbortTransfer();
            return I2C_TIMEOUT;
        }

        timeout--;
    }
}

/*
    @brief  Attend la fin d'un transfert sans AUTOEND (TC), SCL restant tenu bas. Sert à enchaîner un repeated START.
    @param  timeout  Nombre maximal d'itérations d'attente.
    @retval I2C_OK, I2C_NACK, I2C_ERROR ou I2C_TIMEOUT (abort appelé en cas d'échec).
*/
static I2C_Status I2C2_WaitForTC(uint32_t timeout)
{
    I2C_Status status;

    while (1)
    {
        uint32_t isr = I2C2_ISR;

        status = I2C2_CheckErrors(isr);

        if (status != I2C_OK)
        {
            I2C2_AbortTransfer();
            return status;
        }

        if (isr & (1UL << I2C_ISR_TC_Pos))
        {
            return I2C_OK;
        }

        if (timeout == 0)
        {
            I2C2_AbortTransfer();
            return I2C_TIMEOUT;
        }

        timeout--;
    }
}

static void GPIO_config(uint32_t scl_pin_pos, uint32_t sda_pin_pos){
    GPIOB_MODER   &= ~((3UL << (scl_pin_pos*2)) | (3UL << (sda_pin_pos*2)));
    GPIOB_MODER   |=  ((2UL << (scl_pin_pos*2)) | (2UL << (sda_pin_pos*2)));   // alternate function
    GPIOB_OTYPER  |=  (1UL << scl_pin_pos) | (1UL << sda_pin_pos);             // open-drain
    GPIOB_OSPEEDR |=  ((3UL << (scl_pin_pos*2)) | (3UL << (sda_pin_pos*2)));   // very high
    GPIOB_AFRH    &= ~((0xFUL << ((scl_pin_pos - 8)*4)) | (0xFUL << ((sda_pin_pos - 8)*4)));
    GPIOB_AFRH    |=  (4UL << ((scl_pin_pos - 8)*4)) | (4UL << ((sda_pin_pos - 8)*4));   // AF4
}


void I2C2_Init(I2C2_PinConfig pin_cfg)
{
    // Invalid parameter
    if(pin_cfg != I2C2_PINS_PB10_PB11 && pin_cfg != I2C2_PINS_PB13_PB14)
    {
        return;
    }
    // 1. Horloges
    RCC_AHB2ENR  |= RCC_AHB2ENR_GPIOBEN_Msk;
    RCC_APB1ENR1 |= RCC_APB1ENR1_I2C2EN_Msk;
    (void)RCC_APB1ENR1;                          // lecture dummy

    if(pin_cfg == I2C2_PINS_PB10_PB11)
    {
        // 2. GPIO PB10 (SCL) et PB11 (SDA) en AF4, open-drain
        GPIO_config(I2C2_GPIO_Pin_SCL_Pos1, I2C2_GPIO_Pin_SDA_Pos1);
    }
    if(pin_cfg == I2C2_PINS_PB13_PB14)
    {
        // 2. GPIO PB13 (SCL) et PB14 (SDA) en AF4, open-drain
        GPIO_config(I2C2_GPIO_Pin_SCL_Pos2, I2C2_GPIO_Pin_SDA_Pos2);
    }

    // Source d'horloge I2C2 (ex. SYSCLK = 0b01), à adapter
    RCC_CCIPR = (RCC_CCIPR & ~RCC_CCIPR_I2C2SEL_Msk) | (1UL << RCC_CCIPR_I2C2SEL_Pos);

    // 3. I2C2
    I2C2_CR1 &= ~I2C_CR1_PE_Msk;                 // désactiver pour configurer
    I2C2_TIMINGR = 0x00420F13;                 // valeur calculée
    I2C2_CR1 |= I2C_CR1_PE_Msk;                  // activer
}


// slave_addr est l'adresse 7 bits
/*
    @brief  Écrit des octets vers un esclave (START, données, STOP automatique).
    @param  slave_addr : Adresse 7 bits de l'esclave (sans le bit R/W).
    @param  data : Octets à envoyer.
    @param  size : Nombre d'octets (1 à 255).
    @retval I2C_OK, I2C_NACK, I2C_ERROR ou I2C_TIMEOUT.
*/
I2C_Status I2C2_Write(uint8_t slave_addr, const uint8_t *data, uint8_t size)
{
    if (size == 0) return I2C_ERROR;

    // 1. attendre BUSY = 0
    I2C_Status status;

    status = I2C2_WaitForBusFree(I2C_WAIT_LIMIT);

    if (status != I2C_OK)
    {
        return status;
    }

    // 2. configurer CR2
    I2C2_CR2 = ((uint32_t)(slave_addr & 0x7FU) << 1) | ((uint32_t)size << I2C_CR2_NBYTES_Pos) 
                | (1UL << I2C_CR2_AUTOEND_Pos)   // AUTOEND
                | (1UL << I2C_CR2_START_Pos);  // START

    // 3. Envoyer les octets
    for (uint8_t i = 0; i < size; i++)
    {
        status = I2C2_WaitForTXIS(I2C_WAIT_LIMIT);

        if (status != I2C_OK)
        {
            return status;
        }

        I2C2_TXDR = data[i];
    }

    // 4. Attendre STOP

    status = I2C2_WaitForSTOP(I2C_WAIT_LIMIT);

    if (status != I2C_OK)
    {
        return status;
    }

    // 5. Effacer STOPF
    I2C2_ICR = (1UL << I2C_ICR_STOPCF_Pos);

    return I2C_OK;
}

/*
    @brief  Lit des octets depuis un esclave (START, données, STOP automatique).
    @param  slave_addr : Adresse 7 bits de l'esclave (sans le bit R/W).
    @param  data : Buffer de réception.
    @param  size : Nombre d'octets à lire (1 à 255).
    @retval I2C_OK, I2C_NACK, I2C_ERROR ou I2C_TIMEOUT.
 */
I2C_Status I2C2_Read(uint8_t slave_addr, uint8_t *data, uint8_t size)
{
    if (size == 0) return I2C_ERROR;

    // 1. attendre BUSY = 0
    I2C_Status status;

    status = I2C2_WaitForBusFree(I2C_WAIT_LIMIT);

    if (status != I2C_OK)
    {
        return status;
    }

    // 2. configurer CR2
    I2C2_CR2 = ((uint32_t)(slave_addr & 0x7FU) << 1) | ((uint32_t)size << I2C_CR2_NBYTES_Pos)
                | (1UL << I2C_CR2_RD_WRN_Pos)    // lecture
                | (1UL << I2C_CR2_AUTOEND_Pos)   // AUTOEND
                | (1UL << I2C_CR2_START_Pos);    // START

    // 3. Recevoir les octets
    for (uint8_t i = 0; i < size; i++)
    {
        status = I2C2_WaitForRXNE(I2C_WAIT_LIMIT);

        if (status != I2C_OK)
        {
            return status;
        }

        data[i] = (uint8_t)I2C2_RXDR;
    }

    // 4. Attendre STOP

    status = I2C2_WaitForSTOP(I2C_WAIT_LIMIT);

    if (status != I2C_OK)
    {
        return status;
    }

    // 5. Effacer STOPF
    I2C2_ICR = (1UL << I2C_ICR_STOPCF_Pos);

    return I2C_OK;
}

/*
    @brief  Écrit puis lit avec un repeated START (pas de STOP entre les deux).
    Typiquement : envoi d'une adresse de registre, puis lecture de sa valeur.
    @param : slave_addr  Adresse 7 bits de l'esclave (sans le bit R/W).
    @param  wdata : Octets à écrire (ex. adresse de registre).
    @param  wsize : Nombre d'octets à écrire (1 à 255).
    @param  rdata : Buffer de réception.
    @param  rsize : Nombre d'octets à lire (1 à 255).
    @retval I2C_OK, I2C_NACK, I2C_ERROR ou I2C_TIMEOUT.
*/
I2C_Status I2C2_WriteRead(uint8_t slave_addr, const uint8_t *wdata, uint8_t wsize, uint8_t *rdata, uint8_t rsize)
{
    if (wsize == 0 || rsize == 0) return I2C_ERROR;

    // 1. attendre BUSY = 0
    I2C_Status status;

    status = I2C2_WaitForBusFree(I2C_WAIT_LIMIT);

    if (status != I2C_OK)
    {
        return status;
    }

    // 2. configurer CR2 (écriture, SANS AUTOEND pour permettre le repeated START)
    I2C2_CR2 = ((uint32_t)(slave_addr & 0x7FU) << 1) | ((uint32_t)wsize << I2C_CR2_NBYTES_Pos)
                | (1UL << I2C_CR2_START_Pos);  // START

    // 3. Envoyer les octets
    for (uint8_t i = 0; i < wsize; i++)
    {
        status = I2C2_WaitForTXIS(I2C_WAIT_LIMIT);

        if (status != I2C_OK)
        {
            return status;
        }

        I2C2_TXDR = wdata[i];
    }

    // 4. Attendre TC (transfert terminé, SCL tenu bas)
    status = I2C2_WaitForTC(I2C_WAIT_LIMIT);

    if (status != I2C_OK)
    {
        return status;
    }

    // 5. configurer CR2 (lecture) : génère le repeated START
    I2C2_CR2 = ((uint32_t)(slave_addr & 0x7FU) << 1) | ((uint32_t)rsize << I2C_CR2_NBYTES_Pos)
                | (1UL << I2C_CR2_RD_WRN_Pos)    // lecture
                | (1UL << I2C_CR2_AUTOEND_Pos)   // AUTOEND
                | (1UL << I2C_CR2_START_Pos);    // repeated START

    // 6. Recevoir les octets
    for (uint8_t i = 0; i < rsize; i++)
    {
        status = I2C2_WaitForRXNE(I2C_WAIT_LIMIT);

        if (status != I2C_OK)
        {
            return status;
        }

        rdata[i] = (uint8_t)I2C2_RXDR;
    }

    // 7. Attendre STOP

    status = I2C2_WaitForSTOP(I2C_WAIT_LIMIT);

    if (status != I2C_OK)
    {
        return status;
    }

    // 8. Effacer STOPF
    I2C2_ICR = (1UL << I2C_ICR_STOPCF_Pos);

    return I2C_OK;
}