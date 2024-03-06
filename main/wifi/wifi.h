#pragma once

#include "sdkconfig.h"

#include "esp_netif.h"
#include "esp_err.h"

#define NETIF_DESC_STA "example_netif_sta"

esp_err_t wifi_connect(void);
esp_err_t wifi_disconnect(void);
esp_netif_t *get_netif_from_desc(const char *desc);
bool is_our_netif(const char *prefix, esp_netif_t *netif);