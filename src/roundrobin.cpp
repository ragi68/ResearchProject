/**
 * ESP32 Round-Robin Channel Hopper (2.4 GHz, Channels 1-13)
 * -----------------------------------------------------------
 * - Hops channels 1-13 sequentially with equal, constant dwell
 *   time on each channel.
 * - Full cycle stats are collected into a single snapshot and
 *   pushed to a FreeRTOS queue, then printed in one clean block
 *   over USB-Serial on Core 0 so the sniffer loop on Core 1 is
 *   never blocked by I/O.
 */

#include "roundrobin.h"
#include "common.h"
#include "HardwareSerial.h"

// ── Configuration ────────────────────────────────────────────────────────────

#define DWELL_MS        796      // Constant dwell time per channel (ms)
#define RR_QUEUE_LEN    4

// ── Data types ───────────────────────────────────────────────────────────────

struct RR_ChannelResult {
    uint32_t packets;
    float    pps;
};

struct RR_CycleSnapshot {
    RR_ChannelResult channels[NUM_CHANNELS];
    uint32_t cycleNum;
    uint32_t cycleTotal;
    uint32_t allTimeTotal;
};

// ── Globals ──────────────────────────────────────────────────────────────────

static QueueHandle_t rr_statsQueue   = NULL;
static uint32_t      rr_cycleNum     = 0;
static uint32_t      rr_allTimeTotal = 0;

// ── Stats print task — pinned to Core 0 ──────────────────────────────────────

static void rr_statsPrintTask(void* param) {
    (void)param;
    RR_CycleSnapshot snap;

    for (;;) {
        if (xQueueReceive(rr_statsQueue, &snap, portMAX_DELAY) == pdTRUE) {
            RR_SerialPacket pkt;
            pkt.magic        = SERIAL_MAGIC;
            pkt.type         = PKT_TYPE_RR;
            pkt.cycleNum     = snap.cycleNum;
            pkt.cycleTotal   = snap.cycleTotal;
            pkt.allTimeTotal = snap.allTimeTotal;
            for (int i = 0; i < NUM_CHANNELS; i++) {
                pkt.channels[i].packets = snap.channels[i].packets;
                pkt.channels[i].pps     = snap.channels[i].pps;
            }
            Serial.write((const uint8_t*)&pkt, sizeof(pkt));
        }
    }
}

// ── Public API ───────────────────────────────────────────────────────────────

void roundRobinSetup() {
    rr_statsQueue = xQueueCreate(RR_QUEUE_LEN, sizeof(RR_CycleSnapshot));

    xTaskCreatePinnedToCore(
        rr_statsPrintTask,
        "RR_StatsPrint",
        4096,
        NULL,
        1,
        NULL,
        0   // Core 0
    );

}

void RoundRobin() {
    rr_cycleNum++;
    uint32_t cycleTotal = 0;

    RR_CycleSnapshot snap;
    snap.cycleNum = rr_cycleNum;

    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        esp_wifi_set_channel(ch + 1, WIFI_SECOND_CHAN_NONE);

        uint32_t before = g_pktCounter;
        unsigned long t0 = millis();

        delay(DWELL_MS);

        uint32_t pkts    = g_pktCounter - before;
        float    elapsed = (millis() - t0) / 1000.0f;
        float    pps     = (elapsed > 0.001f) ? pkts / elapsed : 0.0f;

        snap.channels[ch].packets = pkts;
        snap.channels[ch].pps     = pps;
        cycleTotal += pkts;
    }

    rr_allTimeTotal   += cycleTotal;
    snap.cycleTotal    = cycleTotal;
    snap.allTimeTotal  = rr_allTimeTotal;

    xQueueSend(rr_statsQueue, &snap, 0);
}
