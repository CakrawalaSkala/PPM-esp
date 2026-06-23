#include <stdio.h>
#include <string.h>
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "portmacro.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/uart.h"
#include "oled.h"
#include "nvs_flash.h"

#define DEBUG 0

#define RESOLUTION 1 * 1000 * 1000
#define RMT_PIN 5
#define TAG "RMT"
#define FRAME_DURATION_US 20000
#define CHANNEL_NUM 8
#define PPM_PULSE_WIDTH 100

#define BUF_SIZE (1024)
#define UART_RX_PIN 16
#define UART_BAUDRATE 115200

#define RANGE_CHANNEL 2000 - 1000
#define CHANNEL_LOW 1000
#define CHANNEL_HIGH 2000
#define CAM_STEP 1000
#define DRONE_STEP 500
#define LOADER_STEP 300

#define MULTI_CLICK_TIME 500
#define AFK_TIMEOUT 5000
#define TAP_CONFIRM_TIMEOUT 3000
#define FINAL_TAP_TIMEOUT 3000
#define SYNC_WINDOW 120

uint8_t syncCount = 0;
uint32_t lastSyncTime = 0;

bool prevR = true;
bool prevL = true;

bool readyR = false;
bool readyL = false;

bool afkLock = false;

bool syncWaiting = false;
uint32_t syncStartTime = 0;

bool dropperTriggeredA = false;
bool dropperTriggeredB = false;

int countR = 0;
int countL = 0;

uint32_t lastReleaseR = 0;
uint32_t lastReleaseL = 0;

bool cmd5Pending = false;

uint8_t manualCMD;
uint8_t PaySeq, DronePos, DropperPosA, DropperPosB;

#define SWpin 27

bool lastManualMode = false;
bool ManualMode;
bool enVoiceA;
bool enVoiceB;
bool resetMan = 0;

int8_t buzz;

int16_t touchR, touchL, RawtouchR, RawtouchL;

extern const uint8_t epd_bitmap_logo[];

enum data_map
{
    ERROR_CMD = 1,
    PAYLOAD_LEFT_CMD = 2,
    PAYLOAD_RIGHT_CMD = 3,
    SWITCH_CAMERA_CMD = 4,
    SWITCH_DRONE_CMD = 5,
    PAYLOAD_RESET_CMD = 6,
    PAYLOAD_LOADER_CMD = 7,
    RESET_DROPPER = 8,
    DRONE_ROTATE = 9,
};

enum channel_map
{
    ROLL = 0,
    PITCH,
    RESET_DROPPER_CH,
    PAYLOAD_LOADER_CH,
    ROTATE_CH,
    CAM_CH,
    DRONE_CH,
    PAYLOAD_CH,
};

static int drone   = 1;
static int cam     = 1;
static int mode    = 0;
static int payload = 0;
static int pos     = 0;
static int dropA   = 0;
static int dropB   = 0;

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
    .pull_down_en = 0};

static size_t ppm_encoder_callback(const void *data, size_t data_size,
                                   size_t symbols_written, size_t symbols_free,
                                   rmt_symbol_word_t *symbols, bool *done, void *arg)
{
    if (symbols_free < (CHANNEL_NUM))
        return 0;
    size_t total_time = 0;
    size_t current_symbol = 0;

    for (int i = 0; i < CHANNEL_NUM; i++)
    {
        uint16_t val = channel_val[current_symbol];

        val = val > 2000 ? 2000 : val;
        val = val < 1000 ? 1000 : val;

        symbols[current_symbol].level0 = 1;
        symbols[current_symbol].duration0 = PPM_PULSE_WIDTH;
        symbols[current_symbol].level1 = 0;
        symbols[current_symbol].duration1 = val - PPM_PULSE_WIDTH;
        current_symbol++;
        total_time += val;
    }
    symbols[current_symbol].level0 = 1;
    symbols[current_symbol].duration0 = PPM_PULSE_WIDTH;
    symbols[current_symbol].level1 = 0;
    symbols[current_symbol].duration1 = (FRAME_DURATION_US - total_time) - PPM_PULSE_WIDTH;

    *done = 1;
    return current_symbol + 1;
}

const uart_port_t uart_num = UART_NUM_2;

void execute_command(uint8_t cmd)
{
    ESP_LOGI(TAG, "Try to Executing: Cmd %d", cmd);

    if (cmd == PAYLOAD_RIGHT_CMD && enVoiceA)
    {
        buzz = 4;
        channel_val[PAYLOAD_CH] = 2000;
        DropperPosA = (DropperPosA + 1) % 3;
        ESP_LOGI(TAG, "PAYLOAD FRONT, ch %d %d", PAYLOAD_CH, channel_val[PAYLOAD_CH]);

        vTaskDelay(pdMS_TO_TICKS(100));

        channel_val[PAYLOAD_CH] = 1500;
        ESP_LOGI(TAG, "PAYLOAD FRONT, ch %d %d", PAYLOAD_CH, channel_val[PAYLOAD_CH]);
    }
    else if (cmd == PAYLOAD_LEFT_CMD && enVoiceA)
    {
        buzz = 4;
        channel_val[PAYLOAD_CH] = 1000;
        DropperPosB = (DropperPosB + 1) % 3;
        ESP_LOGI(TAG, "PAYLOAD REAR, ch %d %d", PAYLOAD_CH, channel_val[PAYLOAD_CH]);

        vTaskDelay(pdMS_TO_TICKS(100));

        channel_val[PAYLOAD_CH] = 1500;
        ESP_LOGI(TAG, "PAYLOAD REAR, ch %d %d", PAYLOAD_CH, channel_val[PAYLOAD_CH]);
    }
    else if (cmd == SWITCH_CAMERA_CMD && enVoiceA)
    {
        buzz = 1;
        if (channel_val[CAM_CH] < CHANNEL_HIGH)
        {
            channel_val[CAM_CH] += CAM_STEP;
        }
        else
        {
            channel_val[CAM_CH] = CHANNEL_LOW;
        }

        ESP_LOGI(TAG, "Action: Cam switch, ch %d %d", CAM_CH, channel_val[CAM_CH]);
    }
    else if (cmd == SWITCH_DRONE_CMD && enVoiceB)
    {
        buzz = 2;
        DropperPosA = 0;
        DropperPosB = 0;

        if (channel_val[DRONE_CH] < CHANNEL_HIGH)
        {
            channel_val[DRONE_CH] += DRONE_STEP;
        }
        else
        {
            channel_val[DRONE_CH] = CHANNEL_LOW;
        }

        channel_val[RESET_DROPPER_CH] = 1000;
        vTaskDelay(pdMS_TO_TICKS(100));
        channel_val[RESET_DROPPER_CH] = 1500;

        ESP_LOGI(TAG, "Action: Drone Switch, ch %d %d", DRONE_CH, channel_val[DRONE_CH]);
    }
    else if (cmd == PAYLOAD_LOADER_CMD && enVoiceA)
    {
        buzz = 1;
        PaySeq = (PaySeq + 1) % 4;

        if (channel_val[PAYLOAD_LOADER_CH] < 1800)
        {
            channel_val[PAYLOAD_LOADER_CH] += LOADER_STEP;
        }
        else
        {
            channel_val[PAYLOAD_LOADER_CH] = CHANNEL_LOW;
        }
        DropperPosA = 0;
        DropperPosB = 0;
        channel_val[RESET_DROPPER_CH] = 1000;
        vTaskDelay(pdMS_TO_TICKS(100));
        channel_val[RESET_DROPPER_CH] = 1500;

        ESP_LOGI(TAG, "Action: Reload Payload, ch %d %d", PAYLOAD_LOADER_CH, channel_val[PAYLOAD_LOADER_CH]);
    }
    else if ((cmd == PAYLOAD_RESET_CMD && enVoiceA) || resetMan)
    {
        buzz = 3;
        channel_val[ROTATE_CH] = 1000;
        channel_val[PAYLOAD_LOADER_CH] = 1000;
        PaySeq = 0;
        DronePos = 0;
        ESP_LOGI(TAG, "PAYLOAD RESET, ch %d %d", ROTATE_CH, channel_val[ROTATE_CH]);
        ESP_LOGI(TAG, "PAYLOAD RESET, ch %d %d", PAYLOAD_LOADER_CH, channel_val[PAYLOAD_LOADER_CH]);
    }
    else if (cmd == DRONE_ROTATE && enVoiceA)
    {
        buzz = 1;
        DronePos = (DronePos + 1) % 4;

        switch (DronePos)
        {
        case (1):
            channel_val[ROTATE_CH] = 1500;
            break;

        case (2):
            channel_val[ROTATE_CH] = 1000;
            break;

        case (3):
            channel_val[ROTATE_CH] = 2000;
            break;

        default:
            channel_val[ROTATE_CH] = 1000;
            break;
        }

        ESP_LOGI(TAG, "DRONE ROTATE, ch %d %d", ROTATE_CH, channel_val[ROTATE_CH]);
    }
    else if ((cmd == RESET_DROPPER && enVoiceA) || resetMan)
    {
        buzz = 3;
        channel_val[RESET_DROPPER_CH] = 1000;
        DropperPosA = 0;
        DropperPosB = 0;
        ESP_LOGI(TAG, "DROPPER RESET, ch %d %d", RESET_DROPPER_CH, channel_val[RESET_DROPPER_CH]);

        vTaskDelay(pdMS_TO_TICKS(100));

        channel_val[RESET_DROPPER_CH] = 1500;
        ESP_LOGI(TAG, "DROPPER RESET, ch %d %d", RESET_DROPPER_CH, channel_val[RESET_DROPPER_CH]);
    }
    else
    {
        ESP_LOGW(TAG, "Unknown Command ID or Button Has Not Been touched");
    }

    if (DropperPosA == 2 && !dropperTriggeredA)
    {
        channel_val[ROTATE_CH] = 1000;

        if(PaySeq>1) {
            if(DronePos<2) DronePos=2;
            if(DronePos>2) DronePos=0;
        }

        dropperTriggeredA = true;
    }

    if (DropperPosA == 0)
    {
        dropperTriggeredA = false;
    }

    if (DropperPosB == 2 && !dropperTriggeredB)
    {
        channel_val[ROTATE_CH] = 1000;

        if(PaySeq>1) {
            if(DronePos<2) DronePos=2;
            if(DronePos>2) DronePos=0;
        }

        dropperTriggeredB = true;
    }

    if (DropperPosB == 0)
    {
        dropperTriggeredB = false;
    }
}

void manual_command()
{
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

    if (prevR && !touchR)
        readyR = true;
    if (prevL && !touchL)
        readyL = true;

    if (!prevR && touchR && readyR)
    {
        syncWaiting = true;
        syncStartTime = now;

        countR++;
        lastReleaseR = now;
        readyR = false;
    }

    if (!prevL && touchL && readyL)
    {
        syncWaiting = true;
        syncStartTime = now;

        countL++;
        lastReleaseL = now;
        readyL = false;
    }

    if (syncWaiting)
    {
        if (touchR && touchL &&
            (now - syncStartTime) <= SYNC_WINDOW)
        {
            syncCount++;
            lastSyncTime = now;

            countR = 0;
            countL = 0;
            syncWaiting = false;
        }

        if ((now - syncStartTime) > SYNC_WINDOW)
        {
            syncWaiting = false;
        }
    }

    if (syncCount > 0 &&
        (now - lastSyncTime) > MULTI_CLICK_TIME &&
        touchR == false &&
        touchL == false)
    {
        if (syncCount == 1)
        {
            ESP_LOGI(TAG, "4");
            manualCMD = 4;
        }
        else if (syncCount == 2)
        {
            ESP_LOGI(TAG, "5");
            manualCMD = 5;
        }

        execute_command(manualCMD);
        syncCount = 0;
    }

    if (countR > 0 &&
        (now - lastReleaseR) > TAP_CONFIRM_TIMEOUT &&
        touchR == true)
    {
        countR = 0;
        ESP_LOGI(TAG, "Right tap timeout");
    }

    if (countL > 0 &&
        (now - lastReleaseL) > TAP_CONFIRM_TIMEOUT &&
        touchL == true)
    {
        countL = 0;
        ESP_LOGI(TAG, "Left tap timeout");
    }

    if (!syncWaiting &&
        !cmd5Pending &&
        countR > 0 &&
        (now - lastReleaseR) > MULTI_CLICK_TIME &&
        touchR == false)
    {
        if (countR == 1)
        {
            ESP_LOGI(TAG, "3");
            manualCMD = 3;
            execute_command(manualCMD);
        }
        else if (countR == 2)
        {
            ESP_LOGI(TAG, "7");
            manualCMD = 7;
            execute_command(manualCMD);
        }
        else if (countR == 3)
        {
            ESP_LOGI(TAG, "8");
            manualCMD = 8;
            execute_command(manualCMD);
        }
        countR = 0;
    }

    if (!syncWaiting &&
        !cmd5Pending &&
        countL > 0 &&
        (now - lastReleaseL) > MULTI_CLICK_TIME &&
        touchL == false)
    {
        if (countL == 1)
        {
            ESP_LOGI(TAG, "2");
            manualCMD = 2;
            execute_command(manualCMD);
        }
        else if (countL == 2)
        {
            ESP_LOGI(TAG, "9");
            manualCMD = 9;
            execute_command(manualCMD);
        }
        else if (countL == 3)
        {
            ESP_LOGI(TAG, "6");
            manualCMD = 6;
            execute_command(manualCMD);
        }

        countL = 0;
    }

    prevR = touchR;
    prevL = touchL;
}

void rmt_task()
{
    int counter = 0;
    while (1)
    {
        if (DEBUG)
        {
            channel_val[PAYLOAD_CH] = 2000;
            ESP_LOGI(TAG, "PAYLOAD RIGHT, ch %d %d", PAYLOAD_CH, channel_val[PAYLOAD_CH]);

            vTaskDelay(pdMS_TO_TICKS(1000));

            channel_val[PAYLOAD_CH] = 1500;
            vTaskDelay(pdMS_TO_TICKS(1000));
            counter += 10;
            if (counter > 1900)
                counter = 1000;
        }

        ESP_ERROR_CHECK(rmt_transmit(rmt_channel, encoder, channel_val, sizeof(channel_val), &rmt_tx));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(rmt_channel, portMAX_DELAY));
    }
}

uart_config_t uart_config = {
    .baud_rate = UART_BAUDRATE,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
};

void uart_task(void *arg)
{
    uint8_t packet[3];

    gpio_set_direction(SWpin, GPIO_MODE_INPUT);
    gpio_set_direction(GPIO_NUM_12, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_NUM_12, GPIO_PULLDOWN_ONLY);
    gpio_set_direction(GPIO_NUM_14, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_NUM_14, GPIO_PULLDOWN_ONLY);

    while (1)
    {
        int len = uart_read_bytes(uart_num, packet, 3, 50 / portTICK_PERIOD_MS);

        if (len == 3)
        {
            if (packet[0] == 0xAA)
            {
                uint8_t cmd = packet[1];
                uint8_t checksum = packet[2];

                if (checksum == (cmd & 0xFF) && ManualMode == 0)
                {
                    ESP_LOGI(TAG, "Received command: Cmd %d", cmd);
                    if(cmd == 5 &&(gpio_get_level(12) || gpio_get_level(14))) {
                        execute_command(cmd);
                    }
                    if (cmd != 5)
                    {
                        execute_command(cmd);
                    }
                    
                    
                    gpio_set_level(GPIO_NUM_2, 1);
                }
            }
        }
        else if (len > 0)
        {
            uart_flush_input(uart_num);
        }

        ManualMode = gpio_get_level(SWpin);

        if (ManualMode != lastManualMode)
        {
            if (ManualMode)
            {
                ESP_LOGI(TAG, "MANUAL ON");
            }
            else
            {
                ESP_LOGI(TAG, "VOICE ON");
            }

            lastManualMode = ManualMode;
        }

        if (ManualMode)
        {
            touchR = gpio_get_level(12);
            touchL = gpio_get_level(14);

            enVoiceA = 1;
            enVoiceB = 1;

            manual_command();
        }
        else
        {
            enVoiceA = 1;
            enVoiceB = 1;
        }
    }
}

void oled_task()
{
    while (1)
    {
        if (channel_val[DRONE_CH] == 1000)
        {
            oled_set_cursor(1, 0);
            oled_print("DRONE 1");
        }
        else if (channel_val[DRONE_CH] == 1500)
        {
            oled_set_cursor(1, 0);
            oled_print("DRONE 2");
        }
        else if (channel_val[DRONE_CH] == 2000)
        {
            oled_set_cursor(1, 0);
            oled_print("DRONE 3");
        }

        if (channel_val[CAM_CH] == 1000)
        {
            oled_set_cursor(99, 0);
            oled_print("CAM 1");
        }
        else
        {
            oled_set_cursor(99, 0);
            oled_print("CAM 2");
        }

        oled_set_cursor(0, 2);
        oled_print("Mode       : ");
        if (ManualMode)
            oled_print("Manual");
        else
            oled_print("Voice ");

        oled_set_cursor(0, 3);
        oled_print("Payload Seq: ");
        if (PaySeq == 0)
            oled_print("0");
        else if (PaySeq == 1)
            oled_print("1");
        else if (PaySeq == 2)
            oled_print("2");
        else if (PaySeq == 3)
            oled_print("3");

        oled_set_cursor(0, 4);
        oled_print("Drone Pos  : ");
        if (DronePos == 0)
            oled_print("Front");
        else if (DronePos == 1)
            oled_print("Right");
        else if (DronePos == 2)
            oled_print("Front");
        else if (DronePos == 3)
            oled_print("Left ");

        oled_set_cursor(0, 5);
        oled_print("Dropper Pos: ");

        oled_set_cursor(15, 6);
        oled_print("Front: ");
        if (DropperPosA == 0)
            oled_print("Right");
        else if (DropperPosA == 1)
            oled_print("Mid  ");
        else if (DropperPosA == 2)
            oled_print("Left ");

        oled_set_cursor(15, 7);
        oled_print("Rear : ");
        if (DropperPosB == 0)
            oled_print("Right");
        else if (DropperPosB == 1)
            oled_print("Mid  ");
        else if (DropperPosB == 2)
            oled_print("Left ");
    }
}

void buzzer_task()
{
    while (1)
    {
        if (buzz == 1)
        {
            gpio_set_level(GPIO_NUM_26, 1);
            vTaskDelay(pdMS_TO_TICKS(200));
            gpio_set_level(GPIO_NUM_26, 0);
            vTaskDelay(pdMS_TO_TICKS(20));
            gpio_set_level(GPIO_NUM_26, 1);
            vTaskDelay(pdMS_TO_TICKS(200));
            gpio_set_level(GPIO_NUM_26, 0);
            vTaskDelay(pdMS_TO_TICKS(20));
            buzz = 0;
        }
        else if (buzz == 2)
        {
            gpio_set_level(GPIO_NUM_26, 1);
            vTaskDelay(pdMS_TO_TICKS(200));
            gpio_set_level(GPIO_NUM_26, 0);
            vTaskDelay(pdMS_TO_TICKS(20));
            gpio_set_level(GPIO_NUM_26, 1);
            vTaskDelay(pdMS_TO_TICKS(200));
            gpio_set_level(GPIO_NUM_26, 0);
            vTaskDelay(pdMS_TO_TICKS(20));
            gpio_set_level(GPIO_NUM_26, 1);
            vTaskDelay(pdMS_TO_TICKS(200));
            gpio_set_level(GPIO_NUM_26, 0);
            vTaskDelay(pdMS_TO_TICKS(20));
            buzz = 0;
        }
        else if (buzz == 3)
        {
            gpio_set_level(GPIO_NUM_26, 1);
            vTaskDelay(pdMS_TO_TICKS(700));
            gpio_set_level(GPIO_NUM_26, 0);
            vTaskDelay(pdMS_TO_TICKS(20));
            gpio_set_level(GPIO_NUM_26, 1);
            vTaskDelay(pdMS_TO_TICKS(200));
            gpio_set_level(GPIO_NUM_26, 0);
            vTaskDelay(pdMS_TO_TICKS(20));
            buzz = 0;
        }
        else if (buzz == 4)
        {
            gpio_set_level(GPIO_NUM_26, 1);
            vTaskDelay(pdMS_TO_TICKS(300));
            gpio_set_level(GPIO_NUM_26, 0);
            buzz = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    oled_init();
    oled_clear();

    gpio_set_direction(GPIO_NUM_26, GPIO_MODE_OUTPUT);
    for (int i = 0; i < CHANNEL_NUM; i++)
    {
        channel_val[i] = 1000;
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

    ESP_ERROR_CHECK(rmt_new_simple_encoder(&encoder_config, &encoder));
    ESP_ERROR_CHECK(rmt_enable(rmt_channel));

    channel_val[RESET_DROPPER_CH] = 1500;
    channel_val[PAYLOAD_CH] = 1500;

    xTaskCreatePinnedToCore(uart_task, "uart", 4096, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(rmt_task, "rmt", 4096, NULL, 7, NULL, 1);
    xTaskCreatePinnedToCore(oled_task, "oled", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(buzzer_task, "buzzer", 4096, NULL, 4, NULL, 1);
}