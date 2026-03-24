#include <HardwareSerial.h>
#include "common.h"
#include "roundrobin.h"
#include "adaptive.h"

// ── Mode switching config ─────────────────────────────────────────────────────
#define SWITCH_INTERVAL_MS  (10UL * 60UL * 1000UL)   // 10 minutes

static bool          useAdaptive  = false;   // start with Round-Robin
static unsigned long modeStart    = 0;

void setup() {
    Serial.begin(115200);

    wifiInit();

    // Init both — they have independent queues and print tasks
    roundRobinSetup();
    adaptiveSetup();

    modeStart = millis();
}

void loop() {
    // Check if it's time to switch
    if (millis() - modeStart >= SWITCH_INTERVAL_MS) {
        useAdaptive = !useAdaptive;
        modeStart   = millis();
    }

    if (useAdaptive)
        Adaptive();
    else
        RoundRobin();
}
