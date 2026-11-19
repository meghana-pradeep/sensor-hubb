/*
 * Multi-protocol sensor hub for STM32F401RE, using CMSIS device headers only.
 * Use as Core/Src/main.c with the STM32F401xE startup and linker files.
 * Start from the reset clock configuration: HSI 16 MHz, AHB/APB dividers 1.
 *
 * USART2: PA2 TX, PA3 RX, 115200 8N1 (Nucleo ST-LINK virtual COM).
 * ADC1: PA0, channel 0, analog input between GND and VDDA.
 * I2C1: PB8 SCL, PB9 SDA, external pull-ups to 3.3 V, TMP102 ADD0 to GND.
 * SPI1: PA5 SCK, PA6 MISO, PA7 MOSI, PA4 active-low CS, MCP3008 CH0.
 * Power both sensors at 3.3 V and connect all grounds. MCP3008 VREF to 3.3 V.
 *
 * TIM2 triggers an ADC sample every 20 ms. ADC1 end-of-conversion IRQ
 * hands off the sample. Bounded I2C/SPI transfers run in the foreground.
 * UART reports signed integer temperature, 10-bit SPI ADC and 12-bit ADC.
 * us measures from ADC trigger to snapshot complete, excluding UART.
 * Missing/slow peripherals report ERR. Latency must be measured on hardware.
 */

#include "stm32f4xx.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define CLOCK_HZ 16000000u
#define CYCLES_PER_US (CLOCK_HZ / 1000000u)
#define BUS_TIMEOUT (1500u * CYCLES_PER_US)
#define I2C_ERRORS (I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF | I2C_SR1_OVR)

static volatile uint32_t adc_trigger_cycles;
static volatile uint32_t sample_cycles;
static volatile uint16_t adc_sample;
static volatile bool sample_ready;

static void gpio_mode(GPIO_TypeDef *port, unsigned pin, unsigned mode)
{
    unsigned shift = pin * 2u;
    port->MODER = (port->MODER & ~(3u << shift)) | (mode << shift);
}

static void gpio_af(GPIO_TypeDef *port, unsigned pin, unsigned af)
{
    unsigned shift = (pin % 8u) * 4u;
    unsigned reg = pin / 8u;
    port->AFR[reg] = (port->AFR[reg] & ~(15u << shift)) | (af << shift);
    gpio_mode(port, pin, 2);
}

static bool expired(uint32_t start)
{
    return (uint32_t)(DWT->CYCCNT - start) >= BUS_TIMEOUT;
}

static void uart_init(void)
{
    gpio_af(GPIOA, 2, 7);
    gpio_af(GPIOA, 3, 7);
    USART2->BRR = (CLOCK_HZ + 57600u) / 115200u;
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

static void uart_write(const char *text)
{
    while (*text) {
        while (!(USART2->SR & USART_SR_TXE)) {}
        USART2->DR = (uint8_t)*text++;
    }
}

static void i2c_configure(void)
{
    I2C1->CR1 = 0;
    I2C1->CR2 = 16;       /* APB1 clock in MHz. */
    I2C1->OAR1 = 1u << 14; /* Required reserved-bit value on STM32F401. */
    I2C1->CCR = 80;       /* 16 MHz / (2 * 100 kHz). */
    I2C1->TRISE = 17;
    I2C1->CR1 = I2C_CR1_PE;
}

static bool i2c_wait(uint32_t flag, uint32_t start)
{
    do {
        uint32_t status = I2C1->SR1;
        if (status & I2C_ERRORS)
            return false;
        if (status & flag)
            return true;
    } while (!expired(start));
    return false;
}

static bool tmp102_read(int *temperature_c)
{
    const uint8_t address = 0x48;
    uint32_t start = DWT->CYCCNT;

    while (I2C1->SR2 & I2C_SR2_BUSY)
        if (expired(start))
            goto failed;

    I2C1->CR1 |= I2C_CR1_START;
    if (!i2c_wait(I2C_SR1_SB, start))
        goto failed;
    I2C1->DR = address << 1;
    if (!i2c_wait(I2C_SR1_ADDR, start))
        goto failed;
    (void)I2C1->SR2;
    I2C1->DR = 0;         /* Temperature register pointer. */
    if (!i2c_wait(I2C_SR1_BTF, start))
        goto failed;

    I2C1->CR1 |= I2C_CR1_START;
    if (!i2c_wait(I2C_SR1_SB, start))
        goto failed;
    I2C1->DR = (address << 1) | 1u;
    if (!i2c_wait(I2C_SR1_ADDR, start))
        goto failed;

    /* Read only the MSB in default 12-bit mode for integer degrees.
       For a one-byte read, clear ACK, clear ADDR, then request STOP. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    I2C1->CR1 &= ~I2C_CR1_ACK;
    (void)I2C1->SR1;
    (void)I2C1->SR2;
    I2C1->CR1 |= I2C_CR1_STOP;
    __set_PRIMASK(primask);
    if (!i2c_wait(I2C_SR1_RXNE, start))
        goto failed;
    unsigned raw = I2C1->DR & 0xffu;
    *temperature_c = raw < 128u ? (int)raw : (int)raw - 256;
    return true;

failed:
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
    i2c_configure();
    return false;
}

static bool spi_byte(uint8_t out, uint8_t *in, uint32_t start)
{
    while (!(SPI1->SR & SPI_SR_TXE))
        if (expired(start))
            return false;
    *(volatile uint8_t *)&SPI1->DR = out;
    while (!(SPI1->SR & SPI_SR_RXNE))
        if (expired(start))
            return false;
    *in = *(volatile uint8_t *)&SPI1->DR;
    return true;
}

static bool mcp3008_read(uint16_t *value)
{
    uint8_t ignored, high, low;
    uint32_t start = DWT->CYCCNT;
    GPIOA->BSRR = 1u << (4 + 16);
    bool ok = spi_byte(0x01, &ignored, start) &&
              spi_byte(0x80, &high, start) &&
              spi_byte(0x00, &low, start);
    while (ok && (SPI1->SR & SPI_SR_BSY))
        if (expired(start))
            ok = false;
    GPIOA->BSRR = 1u << 4;
    if (ok)
        *value = (uint16_t)(((high & 3u) << 8) | low);
    return ok;
}

void TIM2_IRQHandler(void)
{
    TIM2->SR = 0;
    adc_trigger_cycles = DWT->CYCCNT;
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

void ADC_IRQHandler(void)
{
    if (ADC1->SR & ADC_SR_EOC) {
        adc_sample = (uint16_t)ADC1->DR;  /* Read clears EOC. */
        sample_cycles = adc_trigger_cycles;
        sample_ready = true;
    }
}

static void setup(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN | RCC_APB1ENR_I2C1EN |
                    RCC_APB1ENR_TIM2EN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_ADC1EN;
    (void)RCC->APB2ENR;

    uart_init();

    GPIOB->OTYPER |= (1u << 8) | (1u << 9);
    gpio_af(GPIOB, 8, 4);
    gpio_af(GPIOB, 9, 4);
    i2c_configure();

    GPIOA->BSRR = 1u << 4;
    gpio_mode(GPIOA, 4, 1);
    gpio_af(GPIOA, 5, 5);
    gpio_af(GPIOA, 6, 5);
    gpio_af(GPIOA, 7, 5);
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI |
                SPI_CR1_BR_1 | SPI_CR1_BR_0;  /* Mode 0, 1 MHz. */
    SPI1->CR1 |= SPI_CR1_SPE;

    gpio_mode(GPIOA, 0, 3);
    ADC1->SMPR2 = 4u;    /* Channel 0 sample time: 84 ADC cycles. */
    ADC1->SQR1 = 0;
    ADC1->SQR3 = 0;
    ADC1->CR1 = ADC_CR1_EOCIE;
    ADC1->CR2 = ADC_CR2_ADON;
    NVIC_EnableIRQ(ADC_IRQn);

    TIM2->PSC = 15999;    /* 1 ms per timer count. */
    TIM2->ARR = 19;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0;
    TIM2->DIER = TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 = TIM_CR1_CEN;
}

int main(void)
{
    setup();
    uart_write("Sensor hub ready\r\n");

    for (;;) {
        /* Keep the data and its matching timestamp together. */
        __disable_irq();
        if (!sample_ready) {
            __WFI();     /* A pending enabled IRQ wakes the core. */
            __enable_irq();
            continue;
        }
        uint16_t analog = adc_sample;
        uint32_t start = sample_cycles;
        sample_ready = false;
        __enable_irq();

        int temperature = 0;
        uint16_t spi_adc = 0;
        bool i2c_ok = tmp102_read(&temperature);
        bool spi_ok = mcp3008_read(&spi_adc);
        uint32_t latency = (uint32_t)(DWT->CYCCNT - start) / CYCLES_PER_US;

        char temp_text[12] = "ERR";
        char spi_text[12] = "ERR";
        char line[96];
        if (i2c_ok)
            snprintf(temp_text, sizeof temp_text, "%d", temperature);
        if (spi_ok)
            snprintf(spi_text, sizeof spi_text, "%u", (unsigned)spi_adc);
        snprintf(line, sizeof line, "adc=%u temp_c=%s spi=%s us=%lu\r\n",
                 (unsigned)analog, temp_text, spi_text, (unsigned long)latency);
        uart_write(line);
    }
}
