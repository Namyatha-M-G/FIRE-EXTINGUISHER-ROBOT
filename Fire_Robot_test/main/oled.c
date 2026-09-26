#include "oled.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "robot_pins.h"

#define OLED_I2C_PORT I2C_NUM_0
#define OLED_I2C_ADDR 0x3C
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_PAGES (OLED_HEIGHT / 8)
#define OLED_LINE_CHARS 21

static const char *TAG = "OLED";
static i2c_master_bus_handle_t s_bus_handle;
static i2c_master_dev_handle_t s_dev_handle;
static bool s_oled_ready;

static const uint8_t FLAME_LOGO[4][32] = {
    {0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0x70, 0x38,
     0x1C, 0x0E, 0x87, 0xC3, 0xE1, 0xF1, 0x79, 0x3D,
     0x1F, 0x0F, 0x1F, 0x3D, 0x79, 0xF1, 0xE1, 0xC3,
     0x87, 0x0E, 0x1C, 0x38, 0x70, 0xE0, 0x80, 0x00},
    {0x00, 0x80, 0xC0, 0xE0, 0xF0, 0x78, 0x3C, 0x1E,
     0x8F, 0xC7, 0xE3, 0xF1, 0xF8, 0x7C, 0x3E, 0x1F,
     0x0F, 0x1F, 0x3E, 0x7C, 0xF8, 0xF1, 0xE3, 0xC7,
     0x8F, 0x1E, 0x3C, 0x78, 0xF0, 0xE0, 0x80, 0x00},
    {0x00, 0x03, 0x07, 0x0F, 0x1F, 0x3E, 0x7C, 0xF8,
     0xF1, 0xE3, 0xC7, 0x8F, 0x1F, 0x3E, 0x7C, 0xF8,
     0xF0, 0xF8, 0x7C, 0x3E, 0x1F, 0x8F, 0xC7, 0xE3,
     0xF1, 0xF8, 0x7C, 0x3E, 0x1F, 0x0F, 0x03, 0x00},
    {0x00, 0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F,
     0x7E, 0x7C, 0x78, 0x71, 0x63, 0x47, 0x0F, 0x1F,
     0x1F, 0x0F, 0x47, 0x63, 0x71, 0x78, 0x7C, 0x7E,
     0x3F, 0x1F, 0x0F, 0x07, 0x03, 0x01, 0x00, 0x00},
};

static const uint8_t FONT_DIGITS[10][5] = {
    {0x3E, 0x51, 0x49, 0x45, 0x3E},
    {0x00, 0x42, 0x7F, 0x40, 0x00},
    {0x42, 0x61, 0x51, 0x49, 0x46},
    {0x21, 0x41, 0x45, 0x4B, 0x31},
    {0x18, 0x14, 0x12, 0x7F, 0x10},
    {0x27, 0x45, 0x45, 0x45, 0x39},
    {0x3C, 0x4A, 0x49, 0x49, 0x30},
    {0x01, 0x71, 0x09, 0x05, 0x03},
    {0x36, 0x49, 0x49, 0x49, 0x36},
    {0x06, 0x49, 0x49, 0x29, 0x1E},
};

static const uint8_t FONT_ALPHA[26][5] = {
    {0x7E, 0x11, 0x11, 0x11, 0x7E},
    {0x7F, 0x49, 0x49, 0x49, 0x36},
    {0x3E, 0x41, 0x41, 0x41, 0x22},
    {0x7F, 0x41, 0x41, 0x22, 0x1C},
    {0x7F, 0x49, 0x49, 0x49, 0x41},
    {0x7F, 0x09, 0x09, 0x09, 0x01},
    {0x3E, 0x41, 0x49, 0x49, 0x7A},
    {0x7F, 0x08, 0x08, 0x08, 0x7F},
    {0x00, 0x41, 0x7F, 0x41, 0x00},
    {0x20, 0x40, 0x41, 0x3F, 0x01},
    {0x7F, 0x08, 0x14, 0x22, 0x41},
    {0x7F, 0x40, 0x40, 0x40, 0x40},
    {0x7F, 0x02, 0x0C, 0x02, 0x7F},
    {0x7F, 0x04, 0x08, 0x10, 0x7F},
    {0x3E, 0x41, 0x41, 0x41, 0x3E},
    {0x7F, 0x09, 0x09, 0x09, 0x06},
    {0x3E, 0x41, 0x51, 0x21, 0x5E},
    {0x7F, 0x09, 0x19, 0x29, 0x46},
    {0x46, 0x49, 0x49, 0x49, 0x31},
    {0x01, 0x01, 0x7F, 0x01, 0x01},
    {0x3F, 0x40, 0x40, 0x40, 0x3F},
    {0x1F, 0x20, 0x40, 0x20, 0x1F},
    {0x7F, 0x20, 0x18, 0x20, 0x7F},
    {0x63, 0x14, 0x08, 0x14, 0x63},
    {0x07, 0x08, 0x70, 0x08, 0x07},
    {0x61, 0x51, 0x49, 0x45, 0x43},
};

static esp_err_t oled_write(const uint8_t *data, size_t len)
{
    if (!s_oled_ready && s_dev_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    return i2c_master_transmit(s_dev_handle, data, len, 100);
}

static esp_err_t oled_cmd(uint8_t cmd)
{
    uint8_t data[] = {0x00, cmd};
    return oled_write(data, sizeof(data));
}

static esp_err_t oled_data(const uint8_t *data, size_t len)
{
    uint8_t buffer[OLED_WIDTH + 1] = {0};
    if (len > OLED_WIDTH) {
        len = OLED_WIDTH;
    }

    buffer[0] = 0x40;
    memcpy(&buffer[1], data, len);
    return oled_write(buffer, len + 1);
}

static const uint8_t *font_for_char(char c)
{
    static const uint8_t space[5] = {0, 0, 0, 0, 0};
    static const uint8_t colon[5] = {0, 0x36, 0x36, 0, 0};
    static const uint8_t dash[5] = {0x08, 0x08, 0x08, 0x08, 0x08};
    static const uint8_t dot[5] = {0, 0x60, 0x60, 0, 0};

    c = (char)toupper((unsigned char)c);

    if (c >= 'A' && c <= 'Z') {
        return FONT_ALPHA[c - 'A'];
    }
    if (c >= '0' && c <= '9') {
        return FONT_DIGITS[c - '0'];
    }
    if (c == ':') {
        return colon;
    }
    if (c == '-') {
        return dash;
    }
    if (c == '.') {
        return dot;
    }
    return space;
}

static void oled_set_cursor(uint8_t page, uint8_t column)
{
    oled_cmd(0xB0 | (page & 0x07));
    oled_cmd(0x00 | (column & 0x0F));
    oled_cmd(0x10 | ((column >> 4) & 0x0F));
}

static void oled_draw_bitmap(uint8_t page, uint8_t column, const uint8_t bitmap[][32], uint8_t pages)
{
    if (!s_oled_ready) {
        return;
    }

    for (uint8_t i = 0; i < pages; i++) {
        oled_set_cursor(page + i, column);
        oled_data(bitmap[i], 32);
    }
}

static void oled_draw_progress(uint8_t percent)
{
    uint8_t row[OLED_WIDTH] = {0};
    uint8_t fill_cols = (106 * percent) / 100;

    for (uint8_t i = 10; i < 118; i++) {
        row[i] = 0x42;
    }

    row[10] = 0x7E;
    row[117] = 0x7E;

    for (uint8_t i = 11; i < 11 + fill_cols && i < 117; i++) {
        row[i] = 0x7E;
    }

    oled_set_cursor(7, 0);
    oled_data(row, sizeof(row));
}

static void oled_show_startup(void)
{
    oled_clear();
    oled_draw_bitmap(0, 48, FLAME_LOGO, 4);
    oled_print_line(4, "FIRE FIGHTING");
    oled_print_line(5, "SYSTEM READY");

    for (uint8_t progress = 0; progress <= 100; progress += 25) {
        oled_draw_progress(progress);
        vTaskDelay(pdMS_TO_TICKS(180));
    }
}

esp_err_t oled_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = OLED_I2C_PORT,
        .sda_io_num = OLED_SDA_PIN,
        .scl_io_num = OLED_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&bus_config, &s_bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(err));
        return err;
    }

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = OLED_I2C_ADDR,
        .scl_speed_hz = 400000,
    };

    err = i2c_master_bus_add_device(s_bus_handle, &dev_config, &s_dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OLED device init failed: %s", esp_err_to_name(err));
        return err;
    }

    s_oled_ready = true;

    const uint8_t init_cmds[] = {
        0xAE, 0x20, 0x00, 0xB0, 0xC8, 0x00, 0x10, 0x40,
        0x81, 0x7F, 0xA1, 0xA6, 0xA8, 0x3F, 0xA4, 0xD3,
        0x00, 0xD5, 0x80, 0xD9, 0xF1, 0xDA, 0x12, 0xDB,
        0x40, 0x8D, 0x14, 0xAF,
    };

    for (size_t i = 0; i < sizeof(init_cmds); i++) {
        err = oled_cmd(init_cmds[i]);
        if (err != ESP_OK) {
            s_oled_ready = false;
            ESP_LOGE(TAG, "OLED command failed: %s", esp_err_to_name(err));
            return err;
        }
    }

    oled_show_startup();
    ESP_LOGI(TAG, "OLED initialized on SDA=%d SCL=%d", OLED_SDA_PIN, OLED_SCL_PIN);
    return ESP_OK;
}

void oled_clear(void)
{
    if (!s_oled_ready) {
        return;
    }

    uint8_t blank[OLED_WIDTH] = {0};
    for (uint8_t page = 0; page < OLED_PAGES; page++) {
        oled_set_cursor(page, 0);
        oled_data(blank, sizeof(blank));
    }
}

void oled_print_line(uint8_t line, const char *text)
{
    if (!s_oled_ready || line >= OLED_PAGES || text == NULL) {
        return;
    }

    uint8_t row[OLED_WIDTH] = {0};
    uint8_t column = 0;

    for (uint8_t i = 0; text[i] != '\0' && i < OLED_LINE_CHARS; i++) {
        const uint8_t *glyph = font_for_char(text[i]);
        for (uint8_t glyph_col = 0; glyph_col < 5 && column < OLED_WIDTH; glyph_col++) {
            row[column++] = glyph[glyph_col];
        }
        if (column < OLED_WIDTH) {
            row[column++] = 0x00;
        }
    }

    oled_set_cursor(line, 0);
    oled_data(row, sizeof(row));
}

void oled_show_status(bool auto_mode,
                      robot_state_t state,
                      bool flame_left,
                      bool flame_front,
                      bool flame_right,
                      bool pump_on)
{
    char line[OLED_LINE_CHARS + 1];

    snprintf(line, sizeof(line), "MODE:%s", auto_mode ? "AUTO" : "MANUAL");
    oled_print_line(0, line);

    const char *state_text = "IDLE";
    switch (state) {
    case ROBOT_SCAN:
        state_text = "SCAN";
        break;
    case ROBOT_TURN_LEFT:
        state_text = "LEFT";
        break;
    case ROBOT_TURN_RIGHT:
        state_text = "RIGHT";
        break;
    case ROBOT_EXTINGUISH:
        state_text = "PUMP";
        break;
    case ROBOT_VERIFY:
        state_text = "VERIFY";
        break;
    case ROBOT_MOVE_FORWARD:
        state_text = "FORWARD";
        break;
    default:
        break;
    }

    snprintf(line, sizeof(line), "STATE:%s", state_text);
    oled_print_line(1, line);

    snprintf(line, sizeof(line), "FLAME L%d F%d R%d", flame_left, flame_front, flame_right);
    oled_print_line(2, line);

    snprintf(line, sizeof(line), "PUMP:%s", pump_on ? "ON" : "OFF");
    oled_print_line(3, line);

    oled_print_line(5, "AP:FIREROBOT");
    oled_print_line(6, "IP:192.168.4.1");
}
