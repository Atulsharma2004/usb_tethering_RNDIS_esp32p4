idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        nvs_flash
        esp_event
        esp_netif
        esp_http_client
        mbedtls
        driver
        usb
        lwip
    PRIV_REQUIRES
        iot_usbh_cdc
        iot_usbh_rndis
        iot_eth
)
