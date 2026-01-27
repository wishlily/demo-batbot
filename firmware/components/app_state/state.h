#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#define APP_STATE_PRIO 1      // task priority
#define APP_STATE_UNIT_MS 100 // task period

typedef enum {
    APP_STATE_OK,
    APP_STATE_TEST,
    APP_STATE_DISCONN,
} app_state_t;

void app_state_init(void);
void app_state(app_state_t state);

#ifdef __cplusplus
}
#endif