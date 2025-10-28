#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include <i2c_test_wrapper.h>
#include <bme280.h>



// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9


int run_hardware_test();
void run_string_test();
void run_font_test();

// void test_render_image(uint8_t* buf, ssd1306_render_area_t* area, ssd1306_render_area_t* image);
// void test_write_string(uint8_t* buf, ssd1306_render_area_t* area);
// void test_inversion();
// void test_draw_lines(uint8_t* buf, ssd1306_render_area_t* complete_display_area);
// void render_page_through_string(char* text, uint8_t* buf, ssd1306_render_area_t* complete_display_area);

int main() {
    stdio_init_all();

    printf("Hardware test for BME280 driver\n");

    //If you want to use the i2c hardware instance 1, call i2c_handler_selectHwInstance first.
    // uint32_t status = i2c_handler_initialise(SSD1306_I2C_CLK * 1000);

    // printf("initialised i2c, got status code %u\n", status);

    // // run through the complete initialization process
    // SSD1306_init(SSD1306_I2C_ADDR, 128, 32);
    // printf("initialised ssd1306\n");

    // run_hardware_test();
    // run_string_test();
    // run_font_test();
    return 0;
}
