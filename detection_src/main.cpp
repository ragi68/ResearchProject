/**
 * main.cpp — Adaptive Channel Hopper + Suspicious Device Detector
 * ════════════════════════════════════════════════════════════════
 *
 * ARCHITECTURE (3 FreeRTOS tasks)
 * ────────────────────────────────
 *  Core 1 │ hopperTask      — adaptive channel hopping (1–13), owns dwell
 *          │                   timing and per-channel pps history.
 *          │
 *  Core 0 │ detectorTask    — consumes DwellResult items from dwellQueue,
 *          │                   maintains per-MAC device table, computes
 *          │                   suspicion scores, raises alerts.
 *          │
 *  Core 0 │ serialPrintTask — consumes items from printQueue (channel stats
 *                              + device alerts), serialises everything to
 *                              USB-Serial without blocking Core 1.
 *
 * SUSPICION SCORE  (0 – 100, weighted sum of four normalised signals)
 * ────────────────────────────────────────────────────────────────────
 *  W_RSSI   (30%) — rssiMax normalised over [RSSI_FLOOR, RSSI_CEIL]
 *                    stronger signal = physically closer = more suspicious
 *
 *  W_RATE   (30%) — avgPps / MAX_PPS_SCALE
 *                    high packet rate = lots of traffic from this device
 *
 *  W_SPREAD (25%) — channelCount / NUM_CHANNELS
 *                    seen across many channels = the device itself is
 *                    scanning / channel-hopping (classic spy-device pattern)
 *
 *  W_BURST  (15%) — (peakPps / avgPps) / BURST_RATIO_CEIL
 *                    short aggressive probe bursts above the average
 *
 * ALERT LEVELS
 * ────────────
 *  WATCH      score >= SUSPICION_WATCH  (50)
 *  SUSPICIOUS score >= SUSPICION_ALERT  (65)
 *  HIGH       score >= SUSPICION_HIGH   (85)
 *
 * THREADING / ISR SAFETY
 * ────────────────────────
 *  g_pktCounter  — volatile uint32, written in ISR, read in hopperTask only
 *  g_devTable    — protected by g_devMux (portMUX spinlock):
 *                    written by ISR (promiscuousCallback) and detectorTask,
 *                    read by detectorTask and serialPrintTask under same mux.
 *  Queues        — FreeRTOS queues carry all cross-task data.
 *
 * 802.11 MAC HEADER (offset reference used in promiscuousCallback)
 * ─────────────────────────────────────────────────────────────────
 *  Bytes  0– 1  Frame Control
 *  Bytes  2– 3  Duration
 *  Bytes  4– 9  Address 1 (receiver / BSSID)
 *  Bytes 10–15  Address 2 (transmitter — the source we fingerprint)
 *  Bytes 16–21  Address 3 (destination)
 */

// ═══════════════════════════════════════════════════════════════════════════
// Includes
// ═══════════════════════════════════════════════════════════════════════════

#include <Arduino.h>
#include <HardwareSerial.h>
#include <esp_wifi.h>
#include <esp_wifi_types.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <math.h>
#include <string.h>

// ═══════════════════════════════════════════════════════════════════════════
// Configuration — tune these without touching anything else
// ═══════════════════════════════════════════════════════════════════════════

// ── Hopper ───────────────────────────────────────────────────────────────────
#define NUM_CHANNELS        13
#define CYCLE_TIME_MS       10000   // total time budget across all 13 channels
#define MIN_DWELL_MS        100     // every channel gets at least this (ms)
#define HISTORY_CYCLES      10      // rolling pps window length (per channel)
#define INITIAL_CYCLES      3       // warm-up cycles before adaptive weighting

// ── Suspicion weights (must sum to 1.0) ──────────────────────────────────────
#define W_RSSI              0.30f
#define W_RATE              0.30f
#define W_SPREAD            0.25f
#define W_BURST             0.15f

// ── Suspicion normalisation ceilings ─────────────────────────────────────────
#define RSSI_FLOOR          (-90)   // dBm — frames weaker than this ignored
#define RSSI_CEIL           (-30)   // dBm — full-strength ceiling
#define MAX_PPS_SCALE       50.0f   // pps where rate score saturates at 1.0
#define BURST_RATIO_CEIL    5.0f    // peak/avg ratio that saturates burst score

// ── Alert thresholds ─────────────────────────────────────────────────────────
#define SUSPICION_WATCH     50.0f
#define SUSPICION_ALERT     65.0f
#define SUSPICION_HIGH      85.0f

// ── Device table ──────────────────────────────────────────────────────────────
#define MAX_DEVICES         64
#define DEV_HISTORY_LEN     10      // rolling pps window per device

// ── Queue depths ──────────────────────────────────────────────────────────────
#define DWELL_QUEUE_LEN     8       // hopperTask  → detectorTask
#define PRINT_QUEUE_LEN     16      // detectorTask → serialPrintTask

// ── Periodic dump ─────────────────────────────────────────────────────────────
#define DUMP_INTERVAL_MS    (60UL * 1000UL)   // full device table every 60 s

// ═══════════════════════════════════════════════════════════════════════════
// Data structures
// ═══════════════════════════════════════════════════════════════════════════

// Per-channel adaptive state (owned entirely by hopperTask)
struct ChannelState {
    float    ppsHistory[HISTORY_CYCLES];
    uint8_t  histIdx;
    uint8_t  histLen;
    float    avgPps;
    uint32_t dwellMs;
};

// Sent from hopperTask → detectorTask after every channel dwell window
struct DwellResult {
    uint8_t  channel;       // 1-13
    uint32_t elapsedMs;     // actual dwell duration measured by millis()
    uint32_t packets;       // total frames seen on this channel this dwell
};

// Per-device fingerprint record (shared between ISR and detectorTask)
struct DeviceRecord {
    uint8_t  mac[6];
    bool     active;

    // RSSI tracking
    int8_t   rssiMin;
    int8_t   rssiMax;
    float    rssiAvg;           // exponential moving average (alpha=0.1)

    // Packet-rate history (per-dwell rolling window)
    float    ppsHistory[DEV_HISTORY_LEN];
    uint8_t  ppsIdx;
    uint8_t  ppsLen;
    float    avgPps;
    float    peakPps;

    // Channel spread — bitmask of channels this MAC has been heard on
    uint16_t channelMask;
    uint8_t  channelCount;

    // Accumulator for packets seen in the current dwell window (ISR writes)
    uint32_t dwellPackets;

    // Timing
    uint32_t firstSeenMs;
    uint32_t lastSeenMs;

    // Suspicion
    float    score;
    uint8_t  riskLevel;     // 0=normal, 1=watch, 2=suspicious, 3=high
};

// Print queue item types
enum PrintItemType : uint8_t {
    PRINT_CYCLE_STATS = 1,
    PRINT_ALERT,
    PRINT_DUMP_HEADER,
};

struct CycleStats {
    uint32_t cycleNum;
    uint32_t cycleTotal;
    uint32_t allTimeTotal;
    float    globalMean;
    float    globalStdDev;
    float    avgPps[NUM_CHANNELS];
    uint32_t dwellMs[NUM_CHANNELS];
    float    zScore[NUM_CHANNELS];
};

struct AlertInfo {
    uint8_t  mac[6];
    int8_t   rssiMax;
    float    avgPps;
    float    peakPps;
    uint8_t  channelCount;
    float    score;
    uint8_t  riskLevel;
};

struct PrintItem {
    PrintItemType type;
    union {
        CycleStats cycle;
        AlertInfo  alert;
    };
};

// ═══════════════════════════════════════════════════════════════════════════
// Globals
// ═══════════════════════════════════════════════════════════════════════════

static ChannelState   g_ch[NUM_CHANNELS];

static DeviceRecord   g_devTable[MAX_DEVICES];
static uint8_t        g_devCount  = 0;
static portMUX_TYPE   g_devMux    = portMUX_INITIALIZER_UNLOCKED;

static volatile uint32_t g_pktCounter = 0;

static QueueHandle_t  g_dwellQueue = NULL;
static QueueHandle_t  g_printQueue = NULL;

// ═══════════════════════════════════════════════════════════════════════════
// Utility
// ═══════════════════════════════════════════════════════════════════════════

static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline bool macEq(const uint8_t* a, const uint8_t* b) {
    return memcmp(a, b, 6) == 0;
}

static inline bool isBroadcastOrMulticast(const uint8_t* m) {
    // Broadcast: all 0xFF. Multicast: LSB of first octet is 1.
    return (m[0] == 0xFF) || (m[0] & 0x01);
}

// ═══════════════════════════════════════════════════════════════════════════
// Promiscuous callback  (ISR — called on Core 1 by the Wi-Fi driver)
// ═══════════════════════════════════════════════════════════════════════════

static void IRAM_ATTR promiscuousCallback(void* buf, wifi_promiscuous_pkt_type_t type) {
    // 1. Keep the global packet counter the hopper reads
    g_pktCounter++;

    const wifi_promiscuous_pkt_t* ppkt =
        reinterpret_cast<const wifi_promiscuous_pkt_t*>(buf);

    // Minimum 802.11 MAC header = 24 bytes
    if (ppkt->rx_ctrl.sig_len < 24) return;

    int8_t  rssi    = (int8_t)ppkt->rx_ctrl.rssi;
    uint8_t channel = (uint8_t)ppkt->rx_ctrl.channel;

    if (rssi < RSSI_FLOOR) return;

    // Transmitter address is at bytes 10-15 of the 802.11 MAC header
    const uint8_t* srcMac = ppkt->payload + 10;

    if (isBroadcastOrMulticast(srcMac)) return;

    // ── Update device table under ISR spinlock ────────────────────────────────
    portENTER_CRITICAL_ISR(&g_devMux);

    int idx = -1;
    for (int i = 0; i < g_devCount; i++) {
        if (g_devTable[i].active && macEq(g_devTable[i].mac, srcMac)) {
            idx = i;
            break;
        }
    }

    if (idx < 0 && g_devCount < MAX_DEVICES) {
        idx = (int)g_devCount++;
        DeviceRecord& d = g_devTable[idx];
        memset(&d, 0, sizeof(d));
        memcpy(d.mac, srcMac, 6);
        d.active      = true;
        d.rssiMin     = rssi;
        d.rssiMax     = rssi;
        d.rssiAvg     = (float)rssi;
        d.firstSeenMs = millis();
    }

    if (idx >= 0) {
        DeviceRecord& d = g_devTable[idx];

        // RSSI
        if (rssi < d.rssiMin) d.rssiMin = rssi;
        if (rssi > d.rssiMax) d.rssiMax = rssi;
        d.rssiAvg = 0.9f * d.rssiAvg + 0.1f * (float)rssi;

        // Channel bitmask
        if (channel >= 1 && channel <= 13) {
            uint16_t bit = (uint16_t)(1u << (channel - 1));
            if (!(d.channelMask & bit)) {
                d.channelMask  |= bit;
                d.channelCount++;
            }
        }

        // Per-dwell accumulator
        d.dwellPackets++;
        d.lastSeenMs = millis();
    }

    portEXIT_CRITICAL_ISR(&g_devMux);
}

// ═══════════════════════════════════════════════════════════════════════════
// Wi-Fi initialisation
// ═══════════════════════════════════════════════════════════════════════════

static void wifiInit() {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_mode(WIFI_MODE_NULL);
    esp_wifi_start();
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(promiscuousCallback);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
}

// ═══════════════════════════════════════════════════════════════════════════
// Adaptive hopper helpers  (called only from hopperTask)
// ═══════════════════════════════════════════════════════════════════════════

static void updateChannelPps(ChannelState& cs, float pps) {
    cs.ppsHistory[cs.histIdx] = pps;
    cs.histIdx = (cs.histIdx + 1) % HISTORY_CYCLES;
    if (cs.histLen < HISTORY_CYCLES) cs.histLen++;

    float sum = 0;
    for (uint8_t i = 0; i < cs.histLen; i++) sum += cs.ppsHistory[i];
    cs.avgPps = sum / cs.histLen;
}

/**
 * Recompute per-channel dwell allocations using z-score weighting:
 *
 *   z >= 0    →  w = 1.0 + 0.5z   (reward busy channels)
 *  -1 < z < 0 →  w = 1.0 + 0.3z   (mild penalty)
 *  -2 < z <=-1 →  w = 0.5          (moderate penalty)
 *       z <= -2 →  w = 0.2         (heavy penalty — near-silent channel)
 *
 * Total dwell is always corrected back to exactly CYCLE_TIME_MS ms.
 */
static void recalcDwellTimes(float& outMean, float& outStdDev) {
    // Global mean and std-dev of per-channel avgPps
    float mean = 0;
    for (int i = 0; i < NUM_CHANNELS; i++) mean += g_ch[i].avgPps;
    mean /= NUM_CHANNELS;

    float var = 0;
    for (int i = 0; i < NUM_CHANNELS; i++) {
        float d = g_ch[i].avgPps - mean;
        var += d * d;
    }
    float stddev = sqrtf(var / NUM_CHANNELS);

    outMean   = mean;
    outStdDev = stddev;

    // Compute weights
    float weights[NUM_CHANNELS], weightSum = 0;
    for (int i = 0; i < NUM_CHANNELS; i++) {
        float z = (stddev > 1e-6f) ? (g_ch[i].avgPps - mean) / stddev : 0.0f;
        float w;
        if      (z >=  0.0f) w = 1.0f + 0.5f * z;
        else if (z >  -1.0f) w = 1.0f + 0.3f * z;
        else if (z >  -2.0f) w = 0.5f;
        else                  w = 0.2f;
        weights[i] = w;
        weightSum  += w;
    }

    // Proportional allocation
    for (int i = 0; i < NUM_CHANNELS; i++)
        g_ch[i].dwellMs = (uint32_t)((weights[i] / weightSum) * CYCLE_TIME_MS);

    // Enforce floor and compute deficit
    bool    floored[NUM_CHANNELS] = {};
    int32_t deficit = 0;
    float   unflooredWeight = 0;
    for (int i = 0; i < NUM_CHANNELS; i++) {
        if (g_ch[i].dwellMs < MIN_DWELL_MS) {
            deficit += (int32_t)(MIN_DWELL_MS - g_ch[i].dwellMs);
            g_ch[i].dwellMs = MIN_DWELL_MS;
            floored[i] = true;
        } else {
            unflooredWeight += weights[i];
        }
    }

    // Redistribute deficit proportionally from unfloored channels
    if (deficit > 0 && unflooredWeight > 1e-6f) {
        for (int i = 0; i < NUM_CHANNELS; i++) {
            if (!floored[i]) {
                int32_t cut     = (int32_t)((weights[i] / unflooredWeight) * deficit);
                int32_t newDwl  = (int32_t)g_ch[i].dwellMs - cut;
                g_ch[i].dwellMs = (uint32_t)(newDwl >= MIN_DWELL_MS ? newDwl : MIN_DWELL_MS);
            }
        }
    }

    // Apply rounding correction to the busiest channel
    uint32_t total = 0;
    for (int i = 0; i < NUM_CHANNELS; i++) total += g_ch[i].dwellMs;
    int32_t corr = (int32_t)CYCLE_TIME_MS - (int32_t)total;
    if (corr != 0) {
        int best = 0;
        for (int i = 1; i < NUM_CHANNELS; i++)
            if (g_ch[i].dwellMs > g_ch[best].dwellMs) best = i;
        g_ch[best].dwellMs = (uint32_t)((int32_t)g_ch[best].dwellMs + corr);
        if (g_ch[best].dwellMs < MIN_DWELL_MS)
            g_ch[best].dwellMs = MIN_DWELL_MS;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Task 1 — hopperTask  (pinned to Core 1)
// ═══════════════════════════════════════════════════════════════════════════

static void hopperTask(void* param) {
    (void)param;

    memset(g_ch, 0, sizeof(g_ch));
    for (int i = 0; i < NUM_CHANNELS; i++)
        g_ch[i].dwellMs = CYCLE_TIME_MS / NUM_CHANNELS;

    uint32_t cycleNum     = 0;
    uint32_t allTimeTotal = 0;

    for (;;) {
        cycleNum++;
        uint32_t cycleTotal = 0;

        for (int ch = 0; ch < NUM_CHANNELS; ch++) {
            uint8_t wifiCh = (uint8_t)(ch + 1);
            esp_wifi_set_channel(wifiCh, WIFI_SECOND_CHAN_NONE);

            // Reset shared ISR counter before the dwell window
            g_pktCounter = 0;
            unsigned long t0    = millis();
            uint32_t      dwell = g_ch[ch].dwellMs;

            delay(dwell);

            uint32_t pkts    = g_pktCounter;
            uint32_t elapsed = (uint32_t)(millis() - t0);
            float    elapsedS = elapsed / 1000.0f;
            float    pps     = (elapsedS > 0.001f) ? pkts / elapsedS : 0.0f;

            updateChannelPps(g_ch[ch], pps);
            cycleTotal   += pkts;
            allTimeTotal += pkts;

            // Tell detector this dwell window is done
            DwellResult dr;
            dr.channel   = wifiCh;
            dr.elapsedMs = elapsed;
            dr.packets   = pkts;
            xQueueSend(g_dwellQueue, &dr, 0);
        }

        // Recompute adaptive weights
        float globalMean = 0, globalStdDev = 0;
        if (cycleNum >= INITIAL_CYCLES) {
            recalcDwellTimes(globalMean, globalStdDev);
        } else {
            for (int i = 0; i < NUM_CHANNELS; i++) globalMean += g_ch[i].avgPps;
            globalMean /= NUM_CHANNELS;
            float v = 0;
            for (int i = 0; i < NUM_CHANNELS; i++) {
                float d = g_ch[i].avgPps - globalMean;
                v += d * d;
            }
            globalStdDev = sqrtf(v / NUM_CHANNELS);
        }

        // Push cycle stats to the print queue
        PrintItem pi;
        pi.type               = PRINT_CYCLE_STATS;
        pi.cycle.cycleNum     = cycleNum;
        pi.cycle.cycleTotal   = cycleTotal;
        pi.cycle.allTimeTotal = allTimeTotal;
        pi.cycle.globalMean   = globalMean;
        pi.cycle.globalStdDev = globalStdDev;
        for (int i = 0; i < NUM_CHANNELS; i++) {
            pi.cycle.avgPps[i]  = g_ch[i].avgPps;
            pi.cycle.dwellMs[i] = g_ch[i].dwellMs;
            pi.cycle.zScore[i]  = (globalStdDev > 1e-6f)
                ? (g_ch[i].avgPps - globalMean) / globalStdDev
                : 0.0f;
        }
        xQueueSend(g_printQueue, &pi, 0);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Suspicion scoring  (called only from detectorTask)
// ═══════════════════════════════════════════════════════════════════════════

static float computeScore(const DeviceRecord& d) {
    // Component 1: RSSI proximity
    float rssiScore  = clampf(
        (float)(d.rssiMax - RSSI_FLOOR) / (float)(RSSI_CEIL - RSSI_FLOOR),
        0.0f, 1.0f);

    // Component 2: packet rate
    float rateScore  = clampf(d.avgPps / MAX_PPS_SCALE, 0.0f, 1.0f);

    // Component 3: channel spread (device hopping its own channels)
    float spreadScore = (float)d.channelCount / (float)NUM_CHANNELS;

    // Component 4: burst factor (peak / avg — probing behaviour)
    float baseline   = (d.avgPps > 0.01f) ? d.avgPps : 0.01f;
    float burstScore = clampf((d.peakPps / baseline) / BURST_RATIO_CEIL, 0.0f, 1.0f);

    return clampf(
        100.0f * (W_RSSI * rssiScore + W_RATE * rateScore +
                  W_SPREAD * spreadScore + W_BURST * burstScore),
        0.0f, 100.0f);
}

static uint8_t scoreToRisk(float score) {
    if (score >= SUSPICION_HIGH)  return 3;
    if (score >= SUSPICION_ALERT) return 2;
    if (score >= SUSPICION_WATCH) return 1;
    return 0;
}

// ═══════════════════════════════════════════════════════════════════════════
// Task 2 — detectorTask  (pinned to Core 0)
// ═══════════════════════════════════════════════════════════════════════════

static void detectorTask(void* param) {
    (void)param;

    unsigned long lastDump = millis();

    for (;;) {
        DwellResult dr;
        // Block until a dwell result arrives (or 1-second timeout for dump check)
        if (xQueueReceive(g_dwellQueue, &dr, pdMS_TO_TICKS(1000)) != pdTRUE) {
            // Nothing arrived — fall through to dump check below
            goto checkDump;
        }

        {
            float elapsedSec = dr.elapsedMs / 1000.0f;
            if (elapsedSec < 0.001f) elapsedSec = 0.001f;

            // Snapshot device count outside the loop (safe: count only grows)
            portENTER_CRITICAL(&g_devMux);
            uint8_t count = g_devCount;
            portEXIT_CRITICAL(&g_devMux);

            for (int i = 0; i < count; i++) {

                // Step 1: harvest and reset the dwell-window packet count
                portENTER_CRITICAL(&g_devMux);
                if (!g_devTable[i].active) {
                    portEXIT_CRITICAL(&g_devMux);
                    continue;
                }
                uint32_t dwellPkts        = g_devTable[i].dwellPackets;
                g_devTable[i].dwellPackets = 0;
                portEXIT_CRITICAL(&g_devMux);

                float pps = dwellPkts / elapsedSec;

                // Step 2: update pps history and recompute score
                portENTER_CRITICAL(&g_devMux);
                DeviceRecord& d = g_devTable[i];

                d.ppsHistory[d.ppsIdx] = pps;
                d.ppsIdx = (d.ppsIdx + 1) % DEV_HISTORY_LEN;
                if (d.ppsLen < DEV_HISTORY_LEN) d.ppsLen++;

                float sum = 0;
                for (uint8_t j = 0; j < d.ppsLen; j++) sum += d.ppsHistory[j];
                d.avgPps = sum / d.ppsLen;

                if (pps > d.peakPps) d.peakPps = pps;

                float   newScore = computeScore(d);
                uint8_t newRisk  = scoreToRisk(newScore);
                d.score = newScore;

                bool raisedAlert = (newRisk > d.riskLevel);
                if (raisedAlert) d.riskLevel = newRisk;

                // Snapshot for alert (still inside lock to keep it consistent)
                AlertInfo ai;
                if (raisedAlert) {
                    memcpy(ai.mac, d.mac, 6);
                    ai.rssiMax      = d.rssiMax;
                    ai.avgPps       = d.avgPps;
                    ai.peakPps      = d.peakPps;
                    ai.channelCount = d.channelCount;
                    ai.score        = d.score;
                    ai.riskLevel    = d.riskLevel;
                }
                portEXIT_CRITICAL(&g_devMux);

                if (raisedAlert) {
                    PrintItem pi;
                    pi.type  = PRINT_ALERT;
                    pi.alert = ai;
                    xQueueSend(g_printQueue, &pi, 0);
                }
            }
        }

        checkDump:
        if (millis() - lastDump >= DUMP_INTERVAL_MS) {
            lastDump = millis();

            // Dump header
            PrintItem hdr;
            hdr.type = PRINT_DUMP_HEADER;
            memset(&hdr.alert, 0, sizeof(hdr.alert));
            xQueueSend(g_printQueue, &hdr, 0);

            // One PrintItem per active device
            portENTER_CRITICAL(&g_devMux);
            uint8_t n = g_devCount;
            portEXIT_CRITICAL(&g_devMux);

            for (int i = 0; i < n; i++) {
                portENTER_CRITICAL(&g_devMux);
                if (!g_devTable[i].active) {
                    portEXIT_CRITICAL(&g_devMux);
                    continue;
                }
                AlertInfo ai;
                memcpy(ai.mac, g_devTable[i].mac, 6);
                ai.rssiMax      = g_devTable[i].rssiMax;
                ai.avgPps       = g_devTable[i].avgPps;
                ai.peakPps      = g_devTable[i].peakPps;
                ai.channelCount = g_devTable[i].channelCount;
                ai.score        = g_devTable[i].score;
                ai.riskLevel    = g_devTable[i].riskLevel;
                portEXIT_CRITICAL(&g_devMux);

                PrintItem pi;
                pi.type  = PRINT_ALERT;
                pi.alert = ai;
                xQueueSend(g_printQueue, &pi, 0);
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Task 3 — serialPrintTask  (pinned to Core 0)
// ═══════════════════════════════════════════════════════════════════════════

static const char* riskLabel(uint8_t r) {
    switch (r) {
        case 3:  return "!! HIGH !!";
        case 2:  return "SUSPICIOUS";
        case 1:  return "WATCH";
        default: return "normal";
    }
}

static void serialPrintTask(void* param) {
    (void)param;
    PrintItem pi;

    for (;;) {
        if (xQueueReceive(g_printQueue, &pi, portMAX_DELAY) != pdTRUE) continue;

        switch (pi.type) {

        case PRINT_CYCLE_STATS: {
            const CycleStats& c = pi.cycle;
            Serial.printf(
                "\n─── Cycle %lu  total=%lu  allTime=%lu  "
                "mean=%.2f pps  stddev=%.2f ───\n",
                (unsigned long)c.cycleNum,
                (unsigned long)c.cycleTotal,
                (unsigned long)c.allTimeTotal,
                c.globalMean, c.globalStdDev);
            for (int i = 0; i < NUM_CHANNELS; i++) {
                Serial.printf(
                    "  ch%2d  dwell=%4lu ms  avgPps=%6.2f  z=%+5.2f\n",
                    i + 1,
                    (unsigned long)c.dwellMs[i],
                    c.avgPps[i],
                    c.zScore[i]);
            }
            break;
        }

        case PRINT_DUMP_HEADER:
            Serial.printf(
                "\n═══════════════════════ DEVICE TABLE DUMP ═══════════════════════\n"
                "  %-17s  %8s  %8s  %8s  %7s  %6s  %-12s\n",
                "MAC", "RSSI_max", "avgPps", "peakPps", "ch/13", "score", "risk");
            break;

        case PRINT_ALERT: {
            const AlertInfo& a = pi.alert;
            Serial.printf(
                "  %02X:%02X:%02X:%02X:%02X:%02X  "
                "%5d dBm  %7.2f  %7.2f  "
                "%3d/13  %5.1f  %s\n",
                a.mac[0], a.mac[1], a.mac[2],
                a.mac[3], a.mac[4], a.mac[5],
                (int)a.rssiMax,
                a.avgPps, a.peakPps,
                (int)a.channelCount,
                a.score,
                riskLabel(a.riskLevel));
            break;
        }

        default: break;
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Arduino entry points
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("[boot] Adaptive hopper + device detector starting...");

    wifiInit();

    g_dwellQueue = xQueueCreate(DWELL_QUEUE_LEN, sizeof(DwellResult));
    g_printQueue = xQueueCreate(PRINT_QUEUE_LEN, sizeof(PrintItem));

    // Core 0: serial printer (priority 1 — lowest, I/O-bound)
    xTaskCreatePinnedToCore(serialPrintTask, "SerialPrint", 4096, NULL, 1, NULL, 0);

    // Core 0: detector (priority 2 — scores devices, can preempt printer)
    xTaskCreatePinnedToCore(detectorTask,    "Detector",    8192, NULL, 2, NULL, 0);

    // Core 1: hopper (priority 1 — owns Wi-Fi channel switching and timing)
    xTaskCreatePinnedToCore(hopperTask,      "Hopper",      4096, NULL, 1, NULL, 1);

    Serial.println("[boot] All tasks running.");
}

void loop() {
    // All work is in FreeRTOS tasks. Idle loop.
    vTaskDelay(portMAX_DELAY);
}