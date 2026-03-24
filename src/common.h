#pragma once

#include <esp_wifi.h>
#include <esp_wifi_types.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <stdint.h>

#define NUM_CHANNELS 13

// ── Serial packet framing ─────────────────────────────────────────────────────
#define SERIAL_MAGIC    0xAB
#define PKT_TYPE_RR     0x01
#define PKT_TYPE_AD     0x02

// ── Round-Robin serial packet ─────────────────────────────────────────────────
struct __attribute__((packed)) RR_ChannelEntry {
    uint32_t packets;
    float    pps;
};

struct __attribute__((packed)) RR_SerialPacket {
    uint8_t          magic;               // SERIAL_MAGIC
    uint8_t          type;                // PKT_TYPE_RR
    uint32_t         cycleNum;
    uint32_t         cycleTotal;
    uint32_t         allTimeTotal;
    RR_ChannelEntry  channels[NUM_CHANNELS];
};

// ── Adaptive serial packet (nanopb generated) ─────────────────────────────────
#include "CycleStruct.pb.h"

extern volatile uint32_t g_pktCounter;

void IRAM_ATTR promiscuousCallback(void* buf, wifi_promiscuous_pkt_type_t type);
void wifiInit();
