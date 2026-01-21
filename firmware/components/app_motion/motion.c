#include "motion.h"

#include <stdbool.h>
#include <stdio.h>

#include "motor.h"

static const float ROBOT_APB = ((ROBOT_WIDTH + ROBOT_LENGTH) / 2);

esp_err_t motion_stop(bool brake)
{
    return motor_stop(brake);
}

esp_err_t motion_ctrl(motion_cmd_t* cmd)
{
    float speed_L1_setup = cmd->Vx - cmd->Wz * ROBOT_APB;
    float speed_L2_setup = cmd->Vx - cmd->Wz * ROBOT_APB;
    float speed_R1_setup = cmd->Vx + cmd->Wz * ROBOT_APB;
    float speed_R2_setup = cmd->Vx + cmd->Wz * ROBOT_APB;
    motor_set_speed(speed_L1_setup, speed_L2_setup, speed_R1_setup, speed_R2_setup);
    return ESP_OK;
}

esp_err_t motion_get_speed(motion_cmd_t* out)
{
    float speed_m1 = 0, speed_m2 = 0, speed_m3 = 0, speed_m4 = 0;
    motor_get_speed(&speed_m1, &speed_m2, &speed_m3, &speed_m4);

    out->Vx = (speed_m1 + speed_m2 + speed_m3 + speed_m4) / 4;
    out->Vy = 0;
    out->Wz = -(speed_m1 + speed_m2 - speed_m3 - speed_m4) / 4.0f / ROBOT_APB;

    return ESP_OK;
}

esp_err_t motion_init(void)
{
    return motor_init();
}