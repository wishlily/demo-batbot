#include "encoder.h"

#include <stdio.h>

#include "driver/pulse_cnt.h"
#include "esp_log.h"

static const char* TAG = "Encoder";

#define CHECK_RETURN_IF_ERROR(x)                                \
    do {                                                        \
        esp_err_t err = (x);                                    \
        if (err != ESP_OK) {                                    \
            ESP_LOGE(TAG, "Error (%s:%d)", __FILE__, __LINE__); \
            return err;                                         \
        }                                                       \
    } while (0)

static pcnt_unit_handle_t g_encoder_unit[ENCODER_ID_MAX] = {NULL};

esp_err_t encoder_init(void)
{
    ESP_LOGI(TAG, "Init pcnt driver to decode");

    struct encoder_conf {
        int ioa;
        int iob;
    } conf[ENCODER_ID_MAX] = {
        {ENCODER_GPIO_H1A, ENCODER_GPIO_H1B},
        {ENCODER_GPIO_H2A, ENCODER_GPIO_H2B},
        {ENCODER_GPIO_H3B, ENCODER_GPIO_H3A},
        {ENCODER_GPIO_H4B, ENCODER_GPIO_H4A},
    };

    for (int i = 0; i < ENCODER_ID_MAX; i++) {
        pcnt_unit_config_t unit_config = {
            .high_limit = ENCODER_PCNT_HIGH_LIMIT,
            .low_limit = ENCODER_PCNT_LOW_LIMIT,
            .flags.accum_count = true, // enable counter accumulation
        };
        CHECK_RETURN_IF_ERROR(pcnt_new_unit(&unit_config, &g_encoder_unit[i]));
        pcnt_unit_handle_t pcnt_unit = g_encoder_unit[i];
        pcnt_glitch_filter_config_t filter_config = {
            .max_glitch_ns = 1000,
        };
        CHECK_RETURN_IF_ERROR(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));
        pcnt_chan_config_t chan_a_config = {
            .edge_gpio_num = conf[i].ioa,
            .level_gpio_num = conf[i].iob,
        };
        pcnt_channel_handle_t pcnt_chan_a = NULL;
        CHECK_RETURN_IF_ERROR(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
        pcnt_chan_config_t chan_b_config = {
            .edge_gpio_num = conf[i].iob,
            .level_gpio_num = conf[i].ioa,
        };
        pcnt_channel_handle_t pcnt_chan_b = NULL;
        CHECK_RETURN_IF_ERROR(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));
        CHECK_RETURN_IF_ERROR(pcnt_channel_set_edge_action(
            pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
        CHECK_RETURN_IF_ERROR(pcnt_channel_set_level_action(
            pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
        CHECK_RETURN_IF_ERROR(pcnt_channel_set_edge_action(
            pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
        CHECK_RETURN_IF_ERROR(pcnt_channel_set_level_action(
            pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

        CHECK_RETURN_IF_ERROR(pcnt_unit_enable(pcnt_unit));
        CHECK_RETURN_IF_ERROR(pcnt_unit_clear_count(pcnt_unit));
        CHECK_RETURN_IF_ERROR(pcnt_unit_start(pcnt_unit));
    }

    return ESP_OK;
}

esp_err_t encoder_get_count(encoder_id_t encoder_id, int* count)
{
    if ((encoder_id < ENCODER_ID_MAX) && (count != NULL)) {
        return pcnt_unit_get_count(g_encoder_unit[encoder_id], count);
    }
    return ESP_ERR_INVALID_ARG;
}