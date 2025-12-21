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





#define RESOLUTION 1 * 1000 * 1000
#define RMT_PIN 5
#define TAG "RMT"
#define FRAME_DURATION_US 25000
#define CHANNEL_NUM 8
#define PPM_PULSE_WIDTH 100


enum channel{
    CH_1 = 0,
    CH_2,
    CH_3,
    CH_4,
    CH_5,
    CH_6, 
    CH_7,
    CH_8
};

// uint16_t channel_val[CHANNEL_NUM+1] = {2000, 1000, 1300, 1800, 1200, 1600, 2000, 1000};
uint16_t channel_val[CHANNEL_NUM+1] = {0};

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


void channel_task(){
    int counter = 2000;
    while(1){
        // for(int x = 1000; x < 2000; x+=10){
        for(int i = 0; i < CHANNEL_NUM; i ++){
                channel_val[i] = counter;
            }
            // }
        // channel_val[BT_1] = 1000  + (counter * 100);
        // channel_val[BT_2] = 2000 - (counter * 100);
        // channel_val[BT_3] = 1000 + (counter * 120);
        // channel_val[BT_4] = 1000 + (counter * 135);
        // channel_val[BT_5] = 1000 - (counter * 135);
        // channel_val[BT_6] = 1000 - (counter * 135);
        // channel_val[BT_7] = 1000 - (counter * 135);
        // channel_val[BT_8] = 1000 - (counter * 135);

        
        
        counter --;
        if(counter < 1000) counter = 2000;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void rmt_task(){
    int counter = 0;
    while(1){
        
        // ESP_LOGI("ch", "%d %d", channel_val[0], channel_val[1]);
        ESP_ERROR_CHECK(rmt_transmit(rmt_channel, encoder, channel_val, sizeof(channel_val), &rmt_tx));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(rmt_channel, portMAX_DELAY));
    }

}

const uart_port_t uart_num = UART_NUM_2;
uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
};


void uart_task(void){
    uint8_t data[128];
    int len = 0;
    
    while(1){
        ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t*)&len));
        len = uart_read_bytes(uart_num, data, len, 100);
        if(len > 0){
            ESP_LOGI("uart", "%d", data[0]);
        }
        uart_flush(uart_num);

    }
}

void app_main(void)
{

ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));


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
    xTaskCreatePinnedToCore(channel_task, "channel", 2048, NULL, 4, NULL, 0);
 

}