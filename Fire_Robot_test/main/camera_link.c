#include "camera_link.h"

#include "driver/uart.h"
#include "esp_log.h"

#include "robot_pins.h"

#define CAM_UART_PORT UART_NUM_2
#define CAM_UART_BAUD_RATE 115200
#define CAM_UART_RX_BUFFER_SIZE 256

static const char *TAG = "CAM_LINK";

esp_err_t camera_link_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = CAM_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(CAM_UART_PORT, CAM_UART_RX_BUFFER_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(CAM_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(CAM_UART_PORT, CAM_UART_TX_PIN, CAM_UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "ESP32-CAM UART ready on TX=%d RX=%d", CAM_UART_TX_PIN, CAM_UART_RX_PIN);
    return ESP_OK;
}
