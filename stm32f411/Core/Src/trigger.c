#include "trigger.h"
#include "stm32f4xx_hal.h"

#define TRIG_GPIO_PORT  GPIOB
#define TRIG_GPIO_PIN   GPIO_PIN_1

static volatile uint32_t s_pulse_deadline_ms = 0;
static volatile bool     s_fired_latched = false;

void Trigger_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Pin   = TRIG_GPIO_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_PULLDOWN;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TRIG_GPIO_PORT, &g);

    HAL_GPIO_WritePin(TRIG_GPIO_PORT, TRIG_GPIO_PIN, GPIO_PIN_RESET);
    s_pulse_deadline_ms = 0;
    s_fired_latched = false;
}

void Trigger_Abort(void)
{
    s_pulse_deadline_ms = 0;
    HAL_GPIO_WritePin(TRIG_GPIO_PORT, TRIG_GPIO_PIN, GPIO_PIN_RESET);
}

void Trigger_RequestFire(void)
{
    if (s_pulse_deadline_ms != 0u) {
        return;
    }
    s_pulse_deadline_ms = HAL_GetTick() + (uint32_t)TRIGGER_PULSE_MS;
    if (s_pulse_deadline_ms == 0u) {
        s_pulse_deadline_ms = 1u;
    }
    HAL_GPIO_WritePin(TRIG_GPIO_PORT, TRIG_GPIO_PIN, GPIO_PIN_SET);
    s_fired_latched = true;
}

void Trigger_Service(void)
{
    if (s_pulse_deadline_ms == 0u) {
        return;
    }
    if ((int32_t)(HAL_GetTick() - s_pulse_deadline_ms) >= 0) {
        s_pulse_deadline_ms = 0;
        HAL_GPIO_WritePin(TRIG_GPIO_PORT, TRIG_GPIO_PIN, GPIO_PIN_RESET);
    }
}

void Trigger_Tick1ms(void)
{
    Trigger_Service();
}

bool Trigger_IsBusy(void)
{
    return s_pulse_deadline_ms != 0u;
}

bool Trigger_ConsumeFiredFlag(void)
{
    bool f = s_fired_latched;
    s_fired_latched = false;
    return f;
}
