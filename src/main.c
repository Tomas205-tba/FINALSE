#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"



static const char *TAG = "MCP4132";

#define MISO_PIN  19
#define MOSI_PIN  23
#define CLK_PIN   18
#define CS_PIN     5

#define MCP_WIPER0  0x00
#define MCP_TCON    0x04

#define CMD_WRITE   0x00   // C1C0 = 00
#define CMD_READ    0x03   // C1C0 = 11
#define CMD_INC     0x01   // C1C0 = 01
#define CMD_DEC     0x02   // C1C0 = 10

static spi_device_handle_t mcp4132_spi_handle;

// spi_init
void spi_init(void) {
    spi_bus_config_t bus = {
        .miso_io_num   = MISO_PIN,
        .mosi_io_num   = MOSI_PIN,
        .sclk_io_num   = CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };
    spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO);
    spi_device_interface_config_t dev = {
        .clock_speed_hz = 1000000,  
        .mode           = 3,        
        .spics_io_num   = CS_PIN,
        .queue_size     = 5
    };
    spi_bus_add_device(SPI2_HOST, &dev, &mcp4132_spi_handle);
}
// mcp4132_write_register
void mcp4132_write_register(uint8_t reg, uint8_t valor) {
    uint8_t tx[2];
    tx[0] = (reg << 4) | (CMD_WRITE << 2) | ((valor >> 8) & 0x01);
    tx[1] = (uint8_t)(valor & 0xFF);
    spi_transaction_t t = {
        .length    = 16,
        .tx_buffer = tx,
        .rx_buffer = NULL
    };
    spi_device_transmit(mcp4132_spi_handle, &t);
}

uint16_t mcp4132_read_register(uint8_t reg) {
    uint8_t tx[2] = { (reg << 4) | (CMD_READ << 2), 0x00 };
    uint8_t rx[2] = { 0x00, 0x00 };
    spi_transaction_t t = {
        .length    = 16,
        .tx_buffer = tx,
        .rx_buffer = rx
    };
    spi_device_transmit(mcp4132_spi_handle, &t);

    return (uint16_t)((rx[0] & 0x01) << 8) | rx[1];
}
// mcp4132_set_wiper
void mcp4132_set_wiper(uint8_t valor) {
    if (valor > 128) {
        ESP_LOGE("MCP4132", "set_wiper: valor %d fuera de rango (0-128)", valor);
        return;
    }
    mcp4132_write_register(MCP_WIPER0, valor);
}

// mcp4132_set_cutoff_frequency
#define MCP4132_RAB  10000.0f 
#define MCP4132_C  100e-9f   
void mcp4132_set_cutoff_frequency(float fc_hz) {
    if (fc_hz <= 0) {
        ESP_LOGE("MCP4132", "set_cutoff_frequency: fc debe ser > 0 Hz");
        return;
    }
    float R_target = 1.0f / (2.0f * 3.14159265f * fc_hz * MCP4132_C);
    int   N        = (int)(R_target * 128.0f / MCP4132_RAB + 0.5f);
    if (N < 0)   N = 0;
    if (N > 128) N = 128;
    ESP_LOGI("MCP4132", "fc=%.1f Hz → N=%d (R≈%.0f Ω)", fc_hz, N, R_target);
    mcp4132_write_register(MCP_WIPER0, (uint8_t)N);
}

void app_main(void) {
    nvs_flash_init();
    spi_init();

    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t adc_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    adc_oneshot_new_unit(&adc_config, &adc_handle);
    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_0, &chan_config);

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_driver_install(UART_NUM_1, 1024 * 2, 0, 0, NULL, 0);

    while (1) {
        int raw;
        adc_oneshot_read(adc_handle, ADC_CHANNEL_0, &raw);
        float voltage = raw * (3.3f / 4095.0f); // Convertir a voltios

        if (voltage > 1.4f) {
            mcp4132_set_wiper(95);
            uart_write_bytes(UART_NUM_1, "Wiper set to 95\n", strlen("Wiper set to 95\n"));
        } else if (voltage < 0.9f) {
            mcp4132_set_wiper(42);
            uart_write_bytes(UART_NUM_1, "Wiper set to 42\n", strlen("Wiper set to 42\n"));
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Esperar 1 segundo para la próxima lectura
    }
}







