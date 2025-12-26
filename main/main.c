#include <stdio.h>
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/uart.h"
#include "esp_mac.h"



#define DEBUG 1

#define RESOLUTION 1 * 1000 * 1000
#define RMT_PIN 5
#define TAG "RMT"
#define FRAME_DURATION_US 25000
#define CHANNEL_NUM 8
#define PPM_PULSE_WIDTH 100

#define BUF_SIZE (1024)
#define UART_RX_PIN 17 
#define UART_BAUDRATE 9600

#define RANGE_CHANNEL 2000 - 1000
#define CHANNEL_LOW 1000
#define CHANNEL_HIGH 2000
#define CAM_STEP 1000
#define DRONE_STEP 500
#define PAYLOAD_STEP 100




enum channel{
    ROLL = 0,
    PITCH,
    YAW,
    THROTTLE,
    ARM,
    CAM, 
    DRONE, 
    PAYLOAD
};

uint16_t channel_val[CHANNEL_NUM] = {0};

rmt_channel_handle_t rmt_channel = NULL;
rmt_encoder_handle_t encoder = NULL;

rmt_transmit_config_t rmt_tx = {
    .loop_count = 0,
};


gpio_config_t io_conf = {
    .intr_type = GPIO_INTR_DISABLE,
    .mode = GPIO_MODE_OUTPUT,
    .pull_down_en = 0,
    .pull_down_en = 0
};




static size_t ppm_encoder_callback(const void *data, size_t data_size,
    size_t symbols_written, size_t symbols_free,
    rmt_symbol_word_t *symbols, bool *done, void *arg)
    {
        if(symbols_free < (CHANNEL_NUM)) return 0;
    size_t total_time = 0;
    size_t current_symbol = 0;

    for(int i = 0; i < CHANNEL_NUM; i++){
        uint16_t val = channel_val[current_symbol];

        val = val > 2000 ? 2000 : val;
        val = val < 1000 ? 1000 : val;

        symbols[current_symbol].level0 = 1;
        symbols[current_symbol].duration0  = PPM_PULSE_WIDTH;
        symbols[current_symbol].level1 = 0;
        symbols[current_symbol].duration1 = val - PPM_PULSE_WIDTH;
        current_symbol ++;
        total_time += val;

    }
        symbols[current_symbol].level0 = 1;
        symbols[current_symbol].duration0  = PPM_PULSE_WIDTH;
        symbols[current_symbol].level1 = 0;
        symbols[current_symbol].duration1 = (FRAME_DURATION_US - total_time) - PPM_PULSE_WIDTH;



    *done = 1;
    return current_symbol + 1;
}


const uart_port_t uart_num = UART_NUM_2;
void execute_command(uint8_t cmd) {
    ESP_LOGI(TAG, "Executing: Cmd %d", cmd);

    switch (cmd) {
        case PAYLOAD:
        if(channel_val[PAYLOAD] < CHANNEL_HIGH){
            channel_val[PAYLOAD] += PAYLOAD_STEP;
        } else {
            channel_val[PAYLOAD] = CHANNEL_LOW;
        }
        ESP_LOGI(TAG, "Action: Drop Payload, ch %d %d",PAYLOAD,  channel_val[PAYLOAD]);
            break;

        case CAM:
        if(channel_val[CAM] < CHANNEL_HIGH){
            channel_val[CAM] += CAM_STEP;
        } else {
            channel_val[CAM] = CHANNEL_LOW;
        }
        ESP_LOGI(TAG, "Action: Cam switch, ch %d %d",CAM,  channel_val[CAM]);
            break;


        case DRONE:
        if(channel_val[DRONE] < CHANNEL_HIGH){
            channel_val[DRONE] += DRONE_STEP;
        } else {
            channel_val[DRONE] = CHANNEL_LOW;
        }
        ESP_LOGI(TAG, "Action: Toggle Switch, ch %d %d",DRONE,  channel_val[DRONE]);
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown Command ID");
            break;
    }
}



void rmt_task(){
    int counter = 0;
    while(1){
        if(DEBUG){
        for(int i = 0; i < CHANNEL_NUM; i ++){
                channel_val[i] = counter;
            }
        counter +=10;
        if(counter > 1900) counter = 1000;
        }
        // ESP_LOGI("ch", "%d %d", channel_val[0], channel_val[1]);
        ESP_ERROR_CHECK(rmt_transmit(rmt_channel, encoder, channel_val, sizeof(channel_val), &rmt_tx));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(rmt_channel, portMAX_DELAY));
        // vTaskDelay(pdMS_TO_TICKS(10));
    }

}

uart_config_t uart_config = {
    .baud_rate = UART_BAUDRATE,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
};





void uart_task(void *arg) {
    uint8_t packet[3];
    
    while (1) {
        int len = uart_read_bytes(uart_num, packet, 3, 100 / portTICK_PERIOD_MS);

        if (len == 3) {
            if (packet[0] == 0xAA) {
                uint8_t cmd = packet[1];
                uint8_t checksum = packet[2];

                if(checksum == (cmd & 0xFF)){
                ESP_LOGI(TAG, "Received command: Cmd %d", cmd);
                execute_command(cmd);
                }
                
            }
        }
          else if (len > 0) {

            uart_flush_input(uart_num);
        }
    }
}

void app_main(void)
{
    for(int i = 0; i < CHANNEL_NUM; i ++){
        channel_val[i] = 1800;
    }

ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0));
ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
ESP_ERROR_CHECK(uart_set_pin(uart_num, -1, UART_RX_PIN, -1, -1));

ESP_LOGI(TAG, "Create RMT TX channel");
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, 
        .gpio_num = RMT_PIN,
        .mem_block_symbols = 128, 
        .resolution_hz = RESOLUTION,
        .trans_queue_depth = 4, 
    };


    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &rmt_channel));

    rmt_simple_encoder_config_t encoder_config = {
        .callback = ppm_encoder_callback,
    };

    ESP_ERROR_CHECK (rmt_new_simple_encoder(&encoder_config, &encoder));
    ESP_ERROR_CHECK(rmt_enable(rmt_channel));



    xTaskCreatePinnedToCore(rmt_task, "rmt", 2048, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(uart_task, "uart", 2048, NULL, 4, NULL, 0);
 

}