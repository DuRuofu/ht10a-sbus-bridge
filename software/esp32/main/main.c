#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/uart.h"
#include "esp_log.h"

/* ================= 配置 ================= */

#define SBUS_UART          UART_NUM_1
#define SBUS_RX_PIN        4

#define SBUS_FRAME_SIZE    25
#define SBUS_CHANNEL_COUNT 16

#define SBUS_HEADER        0x0F
#define SBUS_FOOTER        0x00
#define SBUS_FOOTER_ALT    0x04

static const char *TAG = "sbus";

/* ================= 数据结构 ================= */

typedef struct {
    uint16_t raw[SBUS_CHANNEL_COUNT];   // 原始 SBUS 值
    uint16_t pwm[SBUS_CHANNEL_COUNT];   // 映射后 1000~2000
    bool failsafe;
    bool lost_frame;
} sbus_data_t;

static QueueHandle_t sbus_queue;

/* ================= SBUS 映射 ================= */

static int sbus_to_pwm(uint16_t sbus)
{
    const int SBUS_MIN = 192;
    const int SBUS_MAX = 1792;

    if (sbus < SBUS_MIN) sbus = SBUS_MIN;
    if (sbus > SBUS_MAX) sbus = SBUS_MAX;

    return (sbus - SBUS_MIN) * 1000 / (SBUS_MAX - SBUS_MIN) + 1000;
}

/* ================= SBUS 解析 ================= */

static bool parse_sbus_frame(uint8_t *frame, sbus_data_t *data)
{
    if (frame[0] != SBUS_HEADER) return false;
    if (frame[24] != SBUS_FOOTER && frame[24] != SBUS_FOOTER_ALT) return false;

    data->raw[0]  = ((frame[1]  | frame[2]  << 8) & 0x07FF);
    data->raw[1]  = ((frame[2]  >> 3 | frame[3]  << 5) & 0x07FF);
    data->raw[2]  = ((frame[3]  >> 6 | frame[4]  << 2 | frame[5]  << 10) & 0x07FF);
    data->raw[3]  = ((frame[5]  >> 1 | frame[6]  << 7) & 0x07FF);
    data->raw[4]  = ((frame[6]  >> 4 | frame[7]  << 4) & 0x07FF);
    data->raw[5]  = ((frame[7]  >> 7 | frame[8]  << 1 | frame[9]  << 9) & 0x07FF);
    data->raw[6]  = ((frame[9]  >> 2 | frame[10] << 6) & 0x07FF);
    data->raw[7]  = ((frame[10] >> 5 | frame[11] << 3) & 0x07FF);

    data->raw[8]  = ((frame[12] | frame[13] << 8) & 0x07FF);
    data->raw[9]  = ((frame[13] >> 3 | frame[14] << 5) & 0x07FF);
    data->raw[10] = ((frame[14] >> 6 | frame[15] << 2 | frame[16] << 10) & 0x07FF);
    data->raw[11] = ((frame[16] >> 1 | frame[17] << 7) & 0x07FF);
    data->raw[12] = ((frame[17] >> 4 | frame[18] << 4) & 0x07FF);
    data->raw[13] = ((frame[18] >> 7 | frame[19] << 1 | frame[20] << 9) & 0x07FF);
    data->raw[14] = ((frame[20] >> 2 | frame[21] << 6) & 0x07FF);
    data->raw[15] = ((frame[21] >> 5 | frame[22] << 3) & 0x07FF);

    data->lost_frame = frame[23] & 0x04;
    data->failsafe   = frame[23] & 0x08;

    /* 映射成 1000~2000 */
    for (int i = 0; i < SBUS_CHANNEL_COUNT; i++) {
        data->pwm[i] = sbus_to_pwm(data->raw[i]);
    }

    return true;
}

/* ================= SBUS 接收任务 ================= */

static void sbus_rx_task(void *arg)
{
    uint8_t frame[SBUS_FRAME_SIZE];
    int index = 0;
    bool syncing = false;
    sbus_data_t data;

    while (1) {
        uint8_t byte;
        uart_read_bytes(SBUS_UART, &byte, 1, portMAX_DELAY);

        if (!syncing) {
            if (byte == SBUS_HEADER) {
                syncing = true;
                index = 0;
                frame[index++] = byte;
            }
        } else {
            frame[index++] = byte;
            if (index >= SBUS_FRAME_SIZE) {
                if (parse_sbus_frame(frame, &data)) {
                    xQueueOverwrite(sbus_queue, &data);
                }
                syncing = false;
            }
        }
    }
}

/* ================= 输出任务 ================= */

static void sbus_print_task(void *arg)
{
    sbus_data_t data;

    while (1) {
        if (xQueueReceive(sbus_queue, &data, portMAX_DELAY)) {

            if (data.failsafe) {
                printf("[SBUS] FAILSAFE\n");
                continue;
            }

            printf(
                "CH1:%4d CH2:%4d CH3:%4d CH4:%4d "
                "CH5:%4d CH6:%4d CH7:%4d CH8:%4d "
                "CH9:%4d CH10:%4d\n",
                data.pwm[0], data.pwm[1], data.pwm[2], data.pwm[3],
                data.pwm[4], data.pwm[5], data.pwm[6], data.pwm[7],
                data.pwm[8], data.pwm[9]
            );
        }
    }
}

/* ================= 主函数 ================= */

void app_main(void)
{
    printf("\nHT-10A SBUS -> PWM(1000~2000)\n");

    const uart_config_t uart_config = {
        .baud_rate = 100000,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_2,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_install(SBUS_UART, 256, 0, 0, NULL, 0);
    uart_param_config(SBUS_UART, &uart_config);
    uart_set_pin(SBUS_UART, UART_PIN_NO_CHANGE, SBUS_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_set_line_inverse(SBUS_UART, UART_SIGNAL_RXD_INV);

    sbus_queue = xQueueCreate(1, sizeof(sbus_data_t));

    xTaskCreate(sbus_rx_task, "sbus_rx", 4096, NULL, 10, NULL);
    xTaskCreate(sbus_print_task, "sbus_print", 2048, NULL, 5, NULL);

    ESP_LOGI(TAG, "SBUS ready (mapped to 1000~2000)");
}
