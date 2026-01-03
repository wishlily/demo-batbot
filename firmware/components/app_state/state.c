#include "state.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "led.h"
#include "morse.h"

static const char* TAG = "STAT";

static morse_handle_t morse_handle;

static void state_task(void* arg)
{
    ESP_LOGI(TAG, "Start Motor_Task with core:%d", xPortGetCoreID());

    const TickType_t period = pdMS_TO_TICKS(APP_STATE_UNIT_MS);
    TickType_t last_time = xTaskGetTickCount();
    while (1) {
        morse_update(&morse_handle, APP_STATE_UNIT_MS);
        vTaskDelayUntil(&last_time, period);
    }

    vTaskDelete(NULL);
}

inline static void state_led_set(bool state)
{
    led_set(LED_ORANGE, state);
}

void app_state_init()
{
    led_init(LED_ORANGE);
    morse_init(&morse_handle, state_led_set);

    xTaskCreatePinnedToCore(state_task, "state_task", 2048, NULL, APP_STATE_PRIO, NULL, 0);
}

static const char* STATE_CODE[] = {
    "TEST",
};

void app_state(app_state_t state)
{
    if (state <= APP_STATE_OK) {
        morse_stop(&morse_handle);
        ESP_LOGI(TAG, "State: OK");
        return;
    }
    morse_send(&morse_handle, STATE_CODE[state - 1]);
    ESP_LOGI(TAG, "State: %s", STATE_CODE[state - 1]);
}