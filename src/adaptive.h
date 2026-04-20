#pragma once
#include <stdint.h>
 
void adaptiveSetup();
 
/**
 * Run one full adaptive cycle across all 13 channels.
 *
 * @param cycleTimeMs  Total time budget for this cycle (ms).
 *                     Dwell times are proportionally scaled.
 *                     Can be changed at runtime between calls.
 *                     Original behaviour: pass 10000.
 */
void Adaptive(uint32_t cycleTimeMs = 10000);
 