idf_component_register(
    SRCS "network.c"
    INCLUDE_DIRS "."
    REQUIRES esp_event esp_wifi esp_netif nvs_flash wifi_provisioning
)