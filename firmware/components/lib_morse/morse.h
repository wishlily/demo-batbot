#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include <stdbool.h>
#include <stdint.h>

#define MORSE_UNIT_MS 200
#define MORSE_IDLE_SW_DEF false

typedef enum {
    MORSE_IDLE,     // Idle
    MORSE_WORD_GAP, // Gap between words (space)
    MORSE_CHAR,     // Processing character
    MORSE_CODE,     // Processing code
    MORSE_CODE_GAP, // Gap between codes
    MORSE_CHAR_GAP, // Gap between characters
} morse_state_t;

typedef void (*morse_set_switch_fn)(bool on);

typedef struct {
    morse_state_t state;

    const char* msg;  // String to send '\0' end
    uint16_t msg_idx; // Current character index

    uint8_t curr_code;
    uint8_t curr_code_idx;

    uint32_t timer_ms;  // Internal timer
    uint32_t target_ms; // Duration to maintain current state

    morse_set_switch_fn set_switch;
} morse_handle_t;

void morse_init(morse_handle_t* handle, morse_set_switch_fn set_switch);
void morse_send(morse_handle_t* handle, const char* msg);
void morse_update(morse_handle_t* handle, uint32_t elapsed_ms);
void morse_stop(morse_handle_t* handle);

#ifdef __cplusplus
}
#endif
