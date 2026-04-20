
#include "adaptive.h"
#include "common.h"
#include "HardwareSerial.h"
#include <math.h>
#include <pb_encode.h>

// ─── Fixed configuration ──────────────────────────────────────────────────────
#define AD_QUEUE_LEN       4
#define HISTORY_CYCLES     10
#define INITIAL_CYCLES     3
#define MIN_DWELL_FRAC     0.02f   // each channel gets at least 2% of cycle time

// ─── Data types ───────────────────────────────────────────────────────────────

struct AD_ChannelStats {
    uint32_t packetCount;
    float    ppsHistory[HISTORY_CYCLES];
    uint8_t  historyIdx;
    uint8_t  historyLen;
    float    avgPps;
    uint32_t dwellMs;
};

struct AD_StatsSnapshot {
    float    avgPps[NUM_CHANNELS];
    uint32_t dwellMs[NUM_CHANNELS];
    float    globalMean;
    float    globalStdDev;
    uint32_t cycleNum;
    uint32_t cycleTotal;
    uint32_t allTimeTotal;
    uint32_t cycleTimeMs;   // NEW: carried for host-side analysis
};

// ─── Globals ──────────────────────────────────────────────────────────────────

static AD_ChannelStats ad_channels[NUM_CHANNELS];
static QueueHandle_t   ad_statsQueue   = NULL;
static uint32_t        ad_cycleNum     = 0;
static uint32_t        ad_cycleTotal   = 0;
static uint32_t        ad_allTimeTotal = 0;

// ─── Statistics helpers ───────────────────────────────────────────────────────

static void updateChannelPps(AD_ChannelStats& ch, float pps) {
    ch.ppsHistory[ch.historyIdx] = pps;
    ch.historyIdx = (ch.historyIdx + 1) % HISTORY_CYCLES;
    if (ch.historyLen < HISTORY_CYCLES) ch.historyLen++;
    float sum = 0;
    for (uint8_t i = 0; i < ch.historyLen; i++) sum += ch.ppsHistory[i];
    ch.avgPps = sum / ch.historyLen;
}

static void recalcDwellTimes(float& outMean, float& outStdDev,
                              uint32_t cycleTimeMs) {
    const uint32_t MIN_DWELL_MS = (uint32_t)(cycleTimeMs * MIN_DWELL_FRAC);

    float mean = 0;
    for (int i = 0; i < NUM_CHANNELS; i++) mean += ad_channels[i].avgPps;
    mean /= NUM_CHANNELS;

    float variance = 0;
    for (int i = 0; i < NUM_CHANNELS; i++) {
        float d = ad_channels[i].avgPps - mean;
        variance += d * d;
    }
    float stddev = sqrtf(variance / NUM_CHANNELS);

    outMean   = mean;
    outStdDev = stddev;

    float weights[NUM_CHANNELS];
    float weightSum = 0;
    for (int i = 0; i < NUM_CHANNELS; i++) {
        float z = (stddev > 1e-6f) ? (ad_channels[i].avgPps - mean) / stddev : 0.0f;
        float w;
        if (z >= 0.0f)       w = 1.0f + 0.5f * z;
        else if (z > -1.0f)  w = 1.0f + 0.3f * z;
        else if (z > -2.0f)  w = 0.5f;
        else                  w = 0.2f;
        weights[i]  = w;
        weightSum  += w;
    }

    bool    floored[NUM_CHANNELS] = {};
    int32_t deficit = 0;
    float   unfloored_weight = 0;

    for (int i = 0; i < NUM_CHANNELS; i++) {
        ad_channels[i].dwellMs = (uint32_t)((weights[i] / weightSum) * cycleTimeMs);
    }
    for (int i = 0; i < NUM_CHANNELS; i++) {
        if (ad_channels[i].dwellMs < MIN_DWELL_MS) {
            deficit += (MIN_DWELL_MS - ad_channels[i].dwellMs);
            ad_channels[i].dwellMs = MIN_DWELL_MS;
            floored[i] = true;
        } else {
            unfloored_weight += weights[i];
        }
    }
    if (deficit > 0 && unfloored_weight > 1e-6f) {
        for (int i = 0; i < NUM_CHANNELS; i++) {
            if (!floored[i]) {
                int32_t reduction = (int32_t)((weights[i] / unfloored_weight) * deficit);
                if ((int32_t)ad_channels[i].dwellMs - reduction >= (int32_t)MIN_DWELL_MS)
                    ad_channels[i].dwellMs -= reduction;
                else
                    ad_channels[i].dwellMs = MIN_DWELL_MS;
            }
        }
    }

    uint32_t totalAssigned = 0;
    for (int i = 0; i < NUM_CHANNELS; i++) totalAssigned += ad_channels[i].dwellMs;
    int32_t correction = (int32_t)cycleTimeMs - (int32_t)totalAssigned;
    if (correction != 0) {
        int maxIdx = 0;
        for (int i = 1; i < NUM_CHANNELS; i++)
            if (ad_channels[i].dwellMs > ad_channels[maxIdx].dwellMs) maxIdx = i;
        ad_channels[maxIdx].dwellMs += correction;
        if (ad_channels[maxIdx].dwellMs < MIN_DWELL_MS)
            ad_channels[maxIdx].dwellMs = MIN_DWELL_MS;
    }
}

// ─── Stats-print task (runs on Core 0) ───────────────────────────────────────

static void ad_statsPrintTask(void* param) {
    (void)param;
    AD_StatsSnapshot snap;

    for (;;) {
        if (xQueueReceive(ad_statsQueue, &snap, portMAX_DELAY) == pdTRUE) {
            AD_SerialPacket pkt = AD_SerialPacket_init_default;
            pkt.magic          = SERIAL_MAGIC;
            pkt.type           = PKT_TYPE_AD;
            pkt.cycleNum       = snap.cycleNum;
            pkt.cycleTotal     = snap.cycleTotal;
            pkt.allTimeTotal   = snap.allTimeTotal;
            pkt.globalMean     = snap.globalMean;
            pkt.globalStdDev   = snap.globalStdDev;
            pkt.channels_count = NUM_CHANNELS;
            for (int i = 0; i < NUM_CHANNELS; i++) {
                pkt.channels[i].avgPps  = snap.avgPps[i];
                pkt.channels[i].dwellMs = snap.dwellMs[i];
                pkt.channels[i].zScore  = (snap.globalStdDev > 1e-6f)
                    ? (snap.avgPps[i] - snap.globalMean) / snap.globalStdDev
                    : 0.0f;
            }

            // Encode
            uint8_t buf[AD_SerialPacket_size];
            pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));
            pb_encode(&stream, AD_SerialPacket_fields, &pkt);
            uint16_t plen = (uint16_t)stream.bytes_written;

            // Length-prefixed framing: [magic][type][len_lo][len_hi][payload]
            uint8_t hdr[4] = {
                SERIAL_MAGIC,
                PKT_TYPE_AD,
                (uint8_t)(plen & 0xFF),
                (uint8_t)(plen >> 8)
            };
            Serial.write(hdr, 4);
            Serial.write(buf, plen);
        }
    }
}

// ─── Public API ───────────────────────────────────────────────────────────────

void adaptiveSetup() {
    memset(ad_channels, 0, sizeof(ad_channels));
    // Initial equal split — will be set properly on first Adaptive() call
    ad_statsQueue = xQueueCreate(AD_QUEUE_LEN, sizeof(AD_StatsSnapshot));
    xTaskCreatePinnedToCore(
        ad_statsPrintTask, "AD_StatsPrint", 4096, NULL, 1, NULL, 0);
}

void Adaptive(uint32_t cycleTimeMs) {
    // Re-initialise dwell on first call or after cycle-time change
    static uint32_t lastCycleTimeMs = 0;
    if (cycleTimeMs != lastCycleTimeMs) {
        for (int i = 0; i < NUM_CHANNELS; i++)
            ad_channels[i].dwellMs = cycleTimeMs / NUM_CHANNELS;
        lastCycleTimeMs = cycleTimeMs;
    }

    ad_cycleNum++;
    ad_cycleTotal = 0;

    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        uint8_t wifiCh = ch + 1;
        esp_wifi_set_channel(wifiCh, WIFI_SECOND_CHAN_NONE);

        g_pktCounter = 0;
        uint32_t dwell = ad_channels[ch].dwellMs;
        unsigned long t0 = millis();
        delay(dwell);

        uint32_t pkts    = g_pktCounter;
        float    elapsed = (millis() - t0) / 1000.0f;
        float    pps     = (elapsed > 0.001f) ? pkts / elapsed : 0.0f;

        ad_channels[ch].packetCount = pkts;
        updateChannelPps(ad_channels[ch], pps);

        ad_cycleTotal   += pkts;
        ad_allTimeTotal += pkts;
    }

    float globalMean, globalStdDev;
    if (ad_cycleNum >= INITIAL_CYCLES) {
        recalcDwellTimes(globalMean, globalStdDev, cycleTimeMs);
    } else {
        globalMean = 0;
        for (int i = 0; i < NUM_CHANNELS; i++) globalMean += ad_channels[i].avgPps;
        globalMean /= NUM_CHANNELS;
        float var = 0;
        for (int i = 0; i < NUM_CHANNELS; i++) {
            float d = ad_channels[i].avgPps - globalMean;
            var += d * d;
        }
        globalStdDev = sqrtf(var / NUM_CHANNELS);
    }

    AD_StatsSnapshot snap;
    snap.cycleNum     = ad_cycleNum;
    snap.globalMean   = globalMean;
    snap.globalStdDev = globalStdDev;
    snap.cycleTotal   = ad_cycleTotal;
    snap.allTimeTotal = ad_allTimeTotal;
    snap.cycleTimeMs  = cycleTimeMs;
    for (int i = 0; i < NUM_CHANNELS; i++) {
        snap.avgPps[i]  = ad_channels[i].avgPps;
        snap.dwellMs[i] = ad_channels[i].dwellMs;
    }
    xQueueSend(ad_statsQueue, &snap, 0);
}