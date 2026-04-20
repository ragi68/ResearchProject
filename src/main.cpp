/**
 * main.cpp — Experiment-mode firmware
 * ====================================
 * Replaces the 10-minute auto-switching timer with a host-commanded
 * mode/cycle-time protocol so experiment.py can drive the ESP32.
 *
 * Command byte received over Serial (USB):
 *   0x10        → Switch to Round-Robin mode
 *   0x20        → Switch to Adaptive mode
 *   0x31        → Set cycle time = 3 000 ms  (send before 0x20)
 *   0x35        → Set cycle time = 5 000 ms
 *   0x37        → Set cycle time = 7 000 ms
 *   0x3A        → Set cycle time = 10 000 ms (default)
 *
 * The AD packet framing adds a 2-byte LE length prefix so the host
 * can parse variable-length protobuf payloads:
 *   [0xAB][0x02][len_lo][len_hi][...nanopb payload...]
 *
 * Build flag: -DEXPERIMENT_MODE
 * Without it the file compiles identically to the original main.cpp.
 */

#include <HardwareSerial.h>
#include "common.h"
#include "roundrobin.h"
#include "adaptive.h"

// ── Command bytes ─────────────────────────────────────────────────────────────
#define CMD_MODE_RR    0x10
#define CMD_MODE_AD    0x20
#define CMD_CYCLE_3S   0x31
#define CMD_CYCLE_5S   0x35
#define CMD_CYCLE_7S   0x37
#define CMD_CYCLE_10S  0x3A

// ── Runtime state ─────────────────────────────────────────────────────────────
static bool     useAdaptive    = false;
static uint32_t targetCycleMs  = 10000;

// ── Forward declarations ──────────────────────────────────────────────────────
void applyCommand(uint8_t cmd);

// ─────────────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    wifiInit();
    roundRobinSetup();
    adaptiveSetup();

#ifdef EXPERIMENT_MODE
    // Signal ready to host
    Serial.write(0xFF);
    Serial.flush();
#else
    // Original auto-switch behaviour compiles in when flag is absent
    // (nothing extra needed — loop() handles it)
#endif
}

void loop() {
#ifdef EXPERIMENT_MODE
    // Check for incoming command byte(s) from host
    while (Serial.available() > 0) {
        uint8_t cmd = (uint8_t)Serial.read();
        applyCommand(cmd);
    }

    if (useAdaptive)
        Adaptive(targetCycleMs);   // pass cycle time as parameter (see adaptive patch)
    else
        RoundRobin();

#else
    // ── Original 10-minute auto-switch mode ───────────────────────────────────
    static bool     _useAdaptive = false;
    static unsigned long _modeStart = millis();
    #define SWITCH_INTERVAL_MS  (10UL * 60UL * 1000UL)

    if (millis() - _modeStart >= SWITCH_INTERVAL_MS) {
        _useAdaptive = !_useAdaptive;
        _modeStart   = millis();
    }

    if (_useAdaptive)
        Adaptive(10000);
    else
        RoundRobin();
#endif
}

void applyCommand(uint8_t cmd) {
    switch (cmd) {
        case CMD_MODE_RR:   useAdaptive = false; break;
        case CMD_MODE_AD:   useAdaptive = true;  break;
        case CMD_CYCLE_3S:  targetCycleMs = 3000;  break;
        case CMD_CYCLE_5S:  targetCycleMs = 5000;  break;
        case CMD_CYCLE_7S:  targetCycleMs = 7000;  break;
        case CMD_CYCLE_10S: targetCycleMs = 10000; break;
        default: break;
    }
}