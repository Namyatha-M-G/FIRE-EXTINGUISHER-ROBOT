idf_component_register(
    SRCS
        "main.c"
        "motor.c"
        "servo.c"
        "flame.c"
        "relay.c"
        "wifi_server.c"
        "camera_link.c"
        "oled.c"

    INCLUDE_DIRS
        "."

    REQUIRES
        esp_driver_gpio
        esp_driver_i2c
        esp_driver_ledc
        esp_driver_uart
        esp_wifi
        esp_http_server
        esp_netif
        nvs_flash
        app_update
)
