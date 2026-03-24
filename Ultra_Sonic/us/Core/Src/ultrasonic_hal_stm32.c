#include "ultrasonic_sensor.h"
#include "main.h"
#include "stm32f3xx_hal.h"
#include <stdio.h>
#include <string.h>

/* --- UART debug helper --- */
static void debug_print(const char *msg)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

/* --- HAL Init --- */
void us_init_hal(int trig_port, int trig_pin,
                 int echo_port, int echo_pin)
{
    /* trig pin is GPIO output, already configured by CubeMX */
    /* just make sure it starts LOW */
    HAL_GPIO_WritePin((GPIO_TypeDef *)trig_port, trig_pin, GPIO_PIN_RESET);

    /* start input capture on TIM2 CH1 */
    HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_1);

    debug_print("US Init done\r\n");
}

/* --- HAL Read --- */
float us_read_hal(int trig_port, int trig_pin,
                  int echo_port, int echo_pin)
{
    uint32_t rising_tick  = 0;
    uint32_t falling_tick = 0;
    char buf[64];

    /* reset timer counter */
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    /* send 10us trigger pulse */
    HAL_GPIO_WritePin((GPIO_TypeDef *)trig_port, trig_pin, GPIO_PIN_SET);
    HAL_Delay(0); /* ~1ms is too long, use DWT or loop for 10us */
    us_delay_us(10);
    HAL_GPIO_WritePin((GPIO_TypeDef *)trig_port, trig_pin, GPIO_PIN_RESET);

    /* wait for echo HIGH (rising edge) with timeout */
    uint32_t timeout = HAL_GetTick() + 30; /* 30ms timeout */
    while (HAL_GPIO_ReadPin((GPIO_TypeDef *)echo_port, echo_pin) == GPIO_PIN_RESET) {
        if (HAL_GetTick() > timeout) {
            debug_print("US Timeout: no echo rising\r\n");
            return -1.0f;
        }
    }
    rising_tick = __HAL_TIM_GET_COUNTER(&htim2);

    /* wait for echo LOW (falling edge) with timeout */
    timeout = HAL_GetTick() + 30;
    while (HAL_GPIO_ReadPin((GPIO_TypeDef *)echo_port, echo_pin) == GPIO_PIN_SET) {
        if (HAL_GetTick() > timeout) {
            debug_print("US Timeout: echo stuck HIGH\r\n");
            return -1.0f;
        }
    }
    falling_tick = __HAL_TIM_GET_COUNTER(&htim2);

    /* calculate pulse width in microseconds */
    uint32_t pulse_us = falling_tick - rising_tick;

    /* distance = pulse_us / 2 / 29.1 */
    float distance_cm = (float)pulse_us / 58.0f;

    /* print to UART2 */
    snprintf(buf, sizeof(buf), "Distance: %.2f cm\r\n", distance_cm);
    debug_print(buf);

    return distance_cm;
}
/* uses DWT cycle counter for microsecond delay */
void us_delay_us(uint32_t us)
{
    /* enable DWT if not already */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < ticks);
}