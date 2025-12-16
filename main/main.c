#include <stdio.h>
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"





#define RESOLUTION 1 * 1000 * 1000
#define RMT_PIN 5
#define TAG "RMT"
#define FRAME_DURATION_US 20000
#define CHANNEL_NUM 7
#define PPM_PULSE_WIDTH 200


enum channel{
    BT_1 = 0,
    BT_2,
    BT_3,
    BT_4,
    BT_5,
    BT_6,
};

uint16_t channel_val[CHANNEL_NUM+1] = {2000, 1000, 1300, 1800, 1700, 1650, 1430};

rmt_channel_handle_t rmt_channel = NULL;
rmt_encoder_handle_t encoder = NULL;

rmt_transmit_config_t rmt_tx = {
    .loop_count = 0,
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
    int counter = 0;
    while(1){

        channel_val[BT_1] = 1000  + (counter * 100);
        channel_val[BT_2] = 2000 - (counter * 100);
        
        counter ++;
        if(counter > 10) counter = 0;

        ESP_LOGI("ch", "%d %d", channel_val[0], channel_val[1]);
        ESP_ERROR_CHECK(rmt_transmit(rmt_channel, encoder, channel_val, sizeof(channel_val), &rmt_tx));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(rmt_channel, portMAX_DELAY));
    }

}


void app_main(void)
{


ESP_LOGI(TAG, "Create RMT TX channel");
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, 
        .gpio_num = RMT_PIN,
        .mem_block_symbols = 64, 
        .resolution_hz = RESOLUTION,
        .trans_queue_depth = 4, 
    };


    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &rmt_channel));

    rmt_simple_encoder_config_t encoder_config = {
        .callback = ppm_encoder_callback,
    };

    ESP_ERROR_CHECK (rmt_new_simple_encoder(&encoder_config, &encoder));
    ESP_ERROR_CHECK(rmt_enable(rmt_channel));



    xTaskCreatePinnedToCore(channel_task, "channel", 2048, NULL, 5, NULL, 1);
 

}