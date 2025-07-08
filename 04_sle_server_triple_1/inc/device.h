#ifndef DEVICE_H
#define DEVICE_H

#include <stdint.h>
#include <stdbool.h>
#include "errcode.h"

#ifdef __cplusplus
extern "C" {
#endif



errcode_t device_oled_init(void);
errcode_t device_oled_show_hello(void);
errcode_t device_oled_turn_off(void);
errcode_t device_oled_reset(void);
errcode_t device_oled_show_tunnel1(void);
errcode_t device_oled_show_tunnel2(void);
errcode_t device_servo_init(void);
errcode_t device_servo_turn_counterclockwise_180(void);
errcode_t device_servo_turn_clockwise_180(void);
errcode_t device_motor_init(void);
errcode_t device_motor_start(void);
errcode_t device_motor_stop(void);
errcode_t device_motor_reverse(void);
bool device_get_oled_initialized(void);
bool device_get_servo_initialized(void);
bool device_get_motor_initialized(void);
errcode_t device_sw4_operation(bool enable);
errcode_t device_sw5_operation(bool enable);
errcode_t device_sw6_operation(bool enable);
errcode_t device_rgb_all_white_on(void);
errcode_t device_rgb_all_red_on(void);
errcode_t device_rgb_all_off(void);
bool device_get_rgb_initialized(void);

#ifdef __cplusplus
}
#endif

#endif /* DEVICE_H */ 