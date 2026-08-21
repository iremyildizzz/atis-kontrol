/**
 * @file trigger.h
 * @brief MOSFET tetik — PB1, jumper H (LOW=OFF, HIGH=ON)
 *
 * KEY test firmware ile aynı: 1 istek → 1 kısa atış (TRIG_PULSE_MS).
 * Pulse süresini STM yönetir; FIRE=0 ortada kesmez.
 */
#ifndef ATIS_TRIGGER_H
#define ATIS_TRIGGER_H

#include <stdint.h>
#include <stdbool.h>

/* KEY testte çalışan değer — 2 atış olursa 170; yarım kalırsa 190–210 */
#define TRIGGER_PULSE_MS  180u

void Trigger_Init(void);
void Trigger_Abort(void);
void Trigger_RequestFire(void);
void Trigger_Tick1ms(void);
void Trigger_Service(void);

bool Trigger_IsBusy(void);
bool Trigger_ConsumeFiredFlag(void);

#endif /* ATIS_TRIGGER_H */
