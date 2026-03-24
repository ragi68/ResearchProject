#include "common.h"

volatile uint32_t g_pktCounter = 0;

void IRAM_ATTR promiscuousCallback(void* buf, wifi_promiscuous_pkt_type_t type) {
    (void)buf;
    (void)type;
    g_pktCounter++;
}

void wifiInit() {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_set_mode(WIFI_MODE_NULL);
    esp_wifi_start();
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(promiscuousCallback);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
}
