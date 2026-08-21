#include "trigger.h"
#include "stm32f4xx_hal.h"

#define TRIG_GPIO_PORT  GPIOB
#define TRIG_GPIO_PIN   GPIO_PIN_1
#define RELAY_OFF_LEVEL GPIO_PIN_RESET
#define RELAY_ON_LEVEL  GPIO_PIN_SET

static volatile uint32_t s_pulse_deadline_ms = 0;
static volatile bool     s_fired_latched = false;

static void RelayOff(void)
{
    HAL_GPIO_WritePin(TRIG_GPIO_PORT, TRIG_GPIO_PIN, RELAY_OFF_LEVEL);
}

static void RelayOn(void)
{
    HAL_GPIO_WritePin(TRIG_GPIO_PORT, TRIG_GPIO_PIN, RELAY_ON_LEVEL);
}

void Trigger_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    RelayOff();
    g.Pin   = TRIG_GPIO_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_PULLDOWN;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TRIG_GPIO_PORT, &g);

    s_pulse_deadline_ms = 0;
    s_fired_latched = false;
}

void Trigger_Abort(void)
{
    s_pulse_deadline_ms = 0;
    RelayOff();
}

void Trigger_RequestFire(void)
{
    if (s_pulse_deadline_ms != 0u) {
        return;
    }

    RelayOn();
    s_pulse_deadline_ms = HAL_GetTick() + (uint32_t)TRIGGER_PULSE_MS;
    if (s_pulse_deadline_ms == 0u) {
        s_pulse_deadline_ms = 1u;
    }
    s_fired_latched = true;
}

void Trigger_Service(void)
{
    if (s_pulse_deadline_ms == 0u) {
        RelayOff();
        return;
    }
    if ((int32_t)(HAL_GetTick() - s_pulse_deadline_ms) >= 0) {
        Trigger_Abort();
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
