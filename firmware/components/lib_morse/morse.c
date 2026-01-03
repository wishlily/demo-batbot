#include "morse.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"

static const char* TAG = "MORSE";

void morse_init(morse_handle_t* handle, morse_set_switch_fn set_switch)
{
    memset(handle, 0, sizeof(morse_handle_t));
    handle->state = MORSE_IDLE;
    handle->set_switch = set_switch;
    handle->set_switch(MORSE_IDLE_SW_DEF);
    ESP_LOGI(TAG, "Morse init");
}

// msg don't free and '\0' end
void morse_send(morse_handle_t* handle, const char* msg)
{
    handle->msg = msg;
    handle->msg_idx = 0;
    handle->state = MORSE_WORD_GAP;
    handle->target_ms = 0;
    ESP_LOGD(TAG, "Morse data: %s", msg);
}

typedef struct {
    uint8_t code; // bit 0 is ".", bit 1 is "-"
    uint8_t size;
} morse_code_t;

static const morse_code_t MORSE_TABLE[] = {
    {0b01, 2},    // A .-
    {0b1000, 4},  // B -...
    {0b1010, 4},  // C -.-.
    {0b100, 3},   // D -..
    {0b0, 1},     // E .
    {0b0010, 4},  // F ..-.
    {0b110, 3},   // G --.
    {0b0000, 4},  // H ....
    {0b00, 2},    // I ..
    {0b0111, 4},  // J .---
    {0b101, 3},   // K -.-
    {0b0100, 4},  // L .-..
    {0b11, 2},    // M --
    {0b10, 2},    // N -.
    {0b111, 3},   // O ---
    {0b0110, 4},  // P .--.
    {0b1101, 4},  // Q --.-
    {0b010, 3},   // R .-.
    {0b000, 3},   // S ...
    {0b1, 1},     // T -
    {0b001, 3},   // U ..-
    {0b0001, 4},  // V ...-
    {0b011, 3},   // W .--
    {0b1001, 4},  // X -..-
    {0b1011, 4},  // Y -.--
    {0b1100, 4},  // Z --..
    {0b11111, 5}, // 0 -----
    {0b01111, 5}, // 1 .----
    {0b00111, 5}, // 2 ..---
    {0b00011, 5}, // 3 ...--
    {0b00001, 5}, // 4 ....-
    {0b00000, 5}, // 5 .....
    {0b10000, 5}, // 6 -....
    {0b11000, 5}, // 7 --...
    {0b11100, 5}, // 8 ---..
    {0b11110, 5}  // 9 ----.
};

const morse_code_t* morse_get_code(char c)
{
    if (c >= 'a' && c <= 'z') {
        c -= 32; // to upper
    }
    if (c >= 'A' && c <= 'Z') {
        return &MORSE_TABLE[c - 'A'];
    }
    if (c >= '0' && c <= '9') {
        return &MORSE_TABLE[c - '0' + 26];
    }
    return NULL;
}

inline static bool morse_msg_is_end(morse_handle_t* handle)
{
    return (handle->msg != NULL && handle->msg_idx >= strlen(handle->msg));
}

static void morse_chk_char(morse_handle_t* handle)
{
    // read next char
    for (int i = 0; handle->state == MORSE_CHAR && i < 10; i++) { // Prevent endless cycles
        if (morse_msg_is_end(handle)) {
            handle->msg_idx = 0; // loop send
            continue;
        }
        char c = handle->msg[handle->msg_idx];
        const morse_code_t* code = morse_get_code(c);
        if (code == NULL) {
            handle->msg_idx++;
            ESP_LOGW(TAG, "Unknown char: %c", c);
            continue;
        }
        handle->curr_code = code->code;
        handle->curr_code_idx = code->size;
        ESP_LOGD(TAG, "Morse char: %c (0x%x, %d)", c, handle->curr_code, handle->curr_code_idx);
        handle->state = MORSE_CODE;
        handle->msg_idx++;
        break;
    }
}

inline static void morse_set_timer(morse_handle_t* handle, int count)
{
    handle->timer_ms = 0;
    handle->target_ms = count * MORSE_UNIT_MS;
}
void morse_update(morse_handle_t* handle, uint32_t elapsed_ms)
{
    if (handle->state == MORSE_IDLE)
        return;

    handle->timer_ms += elapsed_ms;
    if (handle->timer_ms < handle->target_ms) {
        return;
    }

    morse_chk_char(handle);
    switch (handle->state) {
    case MORSE_CODE:
        if (handle->curr_code_idx <= 0) { // never
            handle->state = MORSE_CHAR;
            break;
        }
        handle->set_switch(true);
        if (handle->curr_code & (1 << (handle->curr_code_idx - 1))) { // dash
            morse_set_timer(handle, 3);
        } else { // dot
            morse_set_timer(handle, 1);
        }
        handle->curr_code_idx--;
        if (handle->curr_code_idx) {
            handle->state = MORSE_CODE_GAP;
        } else if (morse_msg_is_end(handle)) {
            handle->state = MORSE_WORD_GAP;
        } else {
            handle->state = MORSE_CHAR_GAP;
        }
        break;
    case MORSE_CODE_GAP:
        handle->set_switch(false);
        morse_set_timer(handle, 1);
        handle->state = MORSE_CODE;
        break;
    case MORSE_CHAR_GAP:
        handle->set_switch(false);
        morse_set_timer(handle, 3);
        handle->state = MORSE_CHAR;
        break;
    case MORSE_WORD_GAP:
        handle->set_switch(false);
        morse_set_timer(handle, 7);
        handle->state = MORSE_CHAR;
        break;
    default:
        break;
    }
}

void morse_stop(morse_handle_t* handle)
{
    handle->state = MORSE_IDLE;
    handle->set_switch(MORSE_IDLE_SW_DEF);
    ESP_LOGD(TAG, "Morse stop");
}