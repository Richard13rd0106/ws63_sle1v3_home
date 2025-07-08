#include "securec.h"
#include "errcode.h"
#include "osal_addr.h"
#include "sle_common.h"
#include "sle_errcode.h"
#include "sle_ssap_server.h"
#include "sle_connection_manager.h"
#include "sle_device_discovery.h"
#include "../inc/SLE_Server_adv.h"
#include "../inc/SLE_Server.h"
#include "cmsis_os2.h"
#include "debug_print.h"
#include "soc_osal.h"
#include "app_init.h"
#include "common_def.h"
#include "pinctrl.h"
#include "gpio.h"
#include "hal_gpio.h"
#include "systick.h"
#include <string.h>
#include "pwm.h"
#include "tcxo.h"
#include <math.h>
#include "osal_event.h"

#define OCTET_BIT_LEN 8
#define UUID_LEN_2 2

#define encode2byte_little(_ptr, data)                     \
    do {                                                   \
        *(uint8_t *)((_ptr) + 1) = (uint8_t)((data) >> 8); \
        *(uint8_t *)(_ptr) = (uint8_t)(data);              \
    } while (0)

/* sle server app uuid for sample */
static char g_sle_uuid_app_uuid[UUID_LEN_2] = {0x0, 0x0};
/* server property value for sample */
static char g_sle_property_value[OCTET_BIT_LEN] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
/* sle connect id */
static uint16_t g_conn_id = 0;
/* sle server id */
static uint8_t g_server_id = 0;
/* sle service handle */
static uint16_t g_service_handle = 0;
/* sle ntf property handle */
static uint16_t g_property_handle = 0;

// 舵机状态
typedef enum {
    SERVO_STATE_UNKNOWN = 0,
    SERVO_STATE_LEFT_90,
    SERVO_STATE_RIGHT_90
} servo_state_t;

// LED状态
typedef enum {
    LED_STATE_OFF = 0,
    LED_STATE_FULL_BRIGHT,
    LED_STATE_BREATHING,
    LED_STATE_DIM
} led_state_t;

// 雨滴传感器状态
static osal_task *g_rain_task = NULL;
static bool g_rain_task_running = false;
static bool g_rain_detected = false;
static bool g_window_closed_by_rain = false; // 窗户是否被雨滴自动关闭
static servo_state_t g_servo_state = SERVO_STATE_UNKNOWN;
static osal_mutex g_servo_mutex;

// 压力传感器状态
static osal_task *g_pressure_task = NULL;
static bool g_pressure_task_running = false;
static bool g_pressure_detected = false;
static bool g_led_off_by_pressure = false;      // LED是否被压力自动关闭
static bool g_window_closed_by_pressure = false; // 窗户是否被压力自动关闭
static led_state_t g_led_state = LED_STATE_OFF;
static osal_mutex g_led_mutex;

// PWM呼吸灯任务
static osal_task *g_pwm_breathing_task = NULL;
static bool g_breathing_task_running = false;
static osal_mutex g_pwm_mutex;

static uint8_t sle_uuid_base[] = {0x37, 0xBE, 0xA8, 0x80, 0xFC, 0x70, 0x11, 0xEA,
                                  0xB7, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static void example_sle_uuid_set_base(sle_uuid_t *out)
{
    (void)memcpy_s(out->uuid, SLE_UUID_LEN, sle_uuid_base, SLE_UUID_LEN);
    out->len = UUID_LEN_2;
}

static void example_sle_uuid_setu2(uint16_t u2, sle_uuid_t *out)
{
    example_sle_uuid_set_base(out);
    out->len = UUID_LEN_2;
    encode2byte_little(&out->uuid[14], u2);
}

static void example_ssaps_read_request_cbk(uint8_t server_id,
                                           uint16_t conn_id,
                                           ssaps_req_read_cb_t *read_cb_para,
                                           errcode_t status)
{
    PRINT("[SLE Server] ssaps read request cbk server_id:0x%x, conn_id:0x%x, handle:0x%x, type:0x%x, status:0x%x\r\n",
          server_id, conn_id, read_cb_para->handle, read_cb_para->type, status);
}

// PWM LED参数
#define PWM_LED_PIN             9          // GPIO_09
#define PWM_LED_PIN_MODE        1
#define PWM_LED_CHANNEL         1
#define PWM_LED_GROUP_ID        0
#define PWM_PERIOD_TOTAL        250
#define PWM_MIN_BRIGHTNESS      0
#define PWM_MAX_BRIGHTNESS      250
#define PWM_DIM_BRIGHTNESS      15         // 微亮亮度
#define BREATHING_PERIOD_MS     2000       // 呼吸周期(ms)
#define BREATHING_UPDATE_MS     30         // 更新间隔(ms)
#define PWM_TASK_STACK_SIZE     0x1000
#define PWM_TASK_PRIO           26
#define PI                      3.14159265359

// 舵机参数
#define SERVO_PIN       3
#define SERVO_COUNT     10
#define SERVO_FREQ_TIME 20000

// 雨滴传感器参数
#define RAIN_SENSOR_PIN 12
#define RAIN_TASK_STACK_SIZE 0x1000
#define RAIN_TASK_PRIO 25
#define RAIN_CHECK_INTERVAL_MS 500

// 压力传感器参数
#define PRESSURE_SENSOR_PIN 2
#define PRESSURE_TASK_STACK_SIZE 0x1000
#define PRESSURE_TASK_PRIO 23
#define PRESSURE_CHECK_INTERVAL_MS 100

static errcode_t pwm_led_callback(uint8_t channel)
{
    unused(channel);
    return ERRCODE_SUCC;
}

// 计算呼吸灯亮度 - 正弦波
static uint32_t calculate_breathing_brightness(uint32_t time_ms)
{
    float t = (float)(time_ms % BREATHING_PERIOD_MS) * 2.0 * PI / BREATHING_PERIOD_MS;
    float sine_value = sin(t);
    float normalized = (sine_value + 1.0) / 2.0;
    uint32_t brightness = PWM_MIN_BRIGHTNESS +
                         (uint32_t)(normalized * (PWM_MAX_BRIGHTNESS - PWM_MIN_BRIGHTNESS));
    return brightness;
}

static void PwmLedInit(void)
{
    uapi_pin_set_mode(PWM_LED_PIN, PWM_LED_PIN_MODE);
    uapi_pwm_deinit();
    uapi_pwm_init();
    uapi_pwm_register_interrupt(PWM_LED_CHANNEL, pwm_led_callback);
    PRINT("[SLE Server] PWM LED initialized on GPIO %d, Channel %d\r\n", PWM_LED_PIN, PWM_LED_CHANNEL);
}

// 设置PWM亮度
static void SetPwmBrightness(uint32_t brightness)
{
    static bool pwm_initialized = false;
    static uint8_t channel_id = PWM_LED_CHANNEL;

    pwm_config_t pwm_config = {
        PWM_PERIOD_TOTAL - brightness,  // low_time
        brightness,                     // high_time
        0,
        0,
        true
    };

    if (osal_mutex_lock(&g_pwm_mutex) != OSAL_SUCCESS) {
        PRINT("[SLE Server] Failed to lock PWM mutex\r\n");
        return;
    }

    if (!pwm_initialized) {
        uapi_pwm_close(PWM_LED_CHANNEL);
        errcode_t ret = uapi_pwm_open(PWM_LED_CHANNEL, &pwm_config);
        if (ret != ERRCODE_SUCC) {
            PRINT("[SLE Server] PWM open failed: 0x%x\r\n", ret);
            osal_mutex_unlock(&g_pwm_mutex);
            return;
        }
        uapi_pwm_set_group(PWM_LED_GROUP_ID, &channel_id, 1);
        uapi_pwm_start_group(PWM_LED_GROUP_ID);
        pwm_initialized = true;
    } else {
        uapi_pwm_close(PWM_LED_CHANNEL);
        errcode_t ret = uapi_pwm_open(PWM_LED_CHANNEL, &pwm_config);
        if (ret != ERRCODE_SUCC) {
            PRINT("[SLE Server] PWM reopen failed: 0x%x\r\n", ret);
            pwm_initialized = false;
            osal_mutex_unlock(&g_pwm_mutex);
            return;
        }
        uapi_pwm_set_group(PWM_LED_GROUP_ID, &channel_id, 1);
        uapi_pwm_start_group(PWM_LED_GROUP_ID);
    }

    osal_mutex_unlock(&g_pwm_mutex);
}

static void *pwm_breathing_task(const char *arg)
{
    unused(arg);

    uint32_t start_time = uapi_tcxo_get_ms();
    uint32_t loop_count = 0;

    PRINT("[SLE Server] PWM breathing task started\r\n");

    while (g_breathing_task_running) {
        loop_count++;

        if (osal_mutex_lock(&g_led_mutex) != OSAL_SUCCESS) {
            PRINT("[SLE Server] Breathing task failed to lock LED mutex at loop %d\r\n", loop_count);
            uapi_tcxo_delay_ms(BREATHING_UPDATE_MS);
            continue;
        }

        if (g_led_state != LED_STATE_BREATHING) {
            osal_mutex_unlock(&g_led_mutex);
            PRINT("[SLE Server] LED state changed, exiting breathing task at loop %d\r\n", loop_count);
            break;
        }
        osal_mutex_unlock(&g_led_mutex);

        uint32_t current_time = uapi_tcxo_get_ms();
        uint32_t elapsed_time = current_time - start_time;
        uint32_t brightness = calculate_breathing_brightness(elapsed_time);

        SetPwmBrightness(brightness);

        osal_msleep(BREATHING_UPDATE_MS);
    }

    PRINT("[SLE Server] PWM breathing task ended after %d loops\r\n", loop_count);
    return NULL;
}

static errcode_t start_breathing_task(void)
{
    if (g_breathing_task_running) {
        PRINT("[SLE Server] Breathing task already running\r\n");
        return ERRCODE_SUCC;
    }

    g_breathing_task_running = true;

    osal_kthread_lock();
    g_pwm_breathing_task = osal_kthread_create((osal_kthread_handler)pwm_breathing_task,
                                              NULL,
                                              "PWMBreathingTask",
                                              PWM_TASK_STACK_SIZE);
    if (g_pwm_breathing_task == NULL) {
        PRINT("[SLE Server] Failed to create PWM breathing task\r\n");
        g_breathing_task_running = false;
        osal_kthread_unlock();
        return ERRCODE_FAIL;
    }

    osal_kthread_set_priority(g_pwm_breathing_task, PWM_TASK_PRIO);
    osal_kfree(g_pwm_breathing_task);
    osal_kthread_unlock();

    PRINT("[SLE Server] PWM breathing task started successfully\r\n");
    return ERRCODE_SUCC;
}

static void stop_breathing_task(void)
{
    if (g_breathing_task_running) {
        PRINT("[SLE Server] Stopping breathing task\r\n");
        g_breathing_task_running = false;
        osal_msleep(BREATHING_UPDATE_MS * 2);
        g_pwm_breathing_task = NULL;
    }
}

static void SetLedState(led_state_t new_state)
{
    if (osal_mutex_lock(&g_led_mutex) != OSAL_SUCCESS) {
        PRINT("[SLE Server] Failed to lock LED mutex in SetLedState\r\n");
        return;
    }

    if (g_led_state == new_state) {
        osal_mutex_unlock(&g_led_mutex);
        PRINT("[SLE Server] LED state unchanged: %d\r\n", new_state);
        return;
    }

    led_state_t old_state = g_led_state;
    g_led_state = new_state;

    PRINT("[SLE Server] LED state changing from %d to %d\r\n", old_state, new_state);

    if (old_state == LED_STATE_BREATHING) {
        osal_mutex_unlock(&g_led_mutex);
        stop_breathing_task();
        if (osal_mutex_lock(&g_led_mutex) != OSAL_SUCCESS) {
            PRINT("[SLE Server] Failed to relock LED mutex after stopping breathing task\r\n");
            return;
        }
    }

    switch (new_state) {
        case LED_STATE_OFF:
            SetPwmBrightness(PWM_MIN_BRIGHTNESS);
            PRINT("[SLE Server] LED turned off\r\n");
            break;

        case LED_STATE_FULL_BRIGHT:
            SetPwmBrightness(PWM_MAX_BRIGHTNESS);
            g_led_off_by_pressure = false; // 手动开灯，清除压力自动关灯标志
            PRINT("[SLE Server] LED set to full brightness\r\n");
            break;

        case LED_STATE_DIM:
            SetPwmBrightness(PWM_DIM_BRIGHTNESS);
            g_led_off_by_pressure = false; // 手动开灯，清除压力自动关灯标志
            PRINT("[SLE Server] LED set to very dim (brightness: %d)\r\n", PWM_DIM_BRIGHTNESS);
            break;

        case LED_STATE_BREATHING:
            g_led_off_by_pressure = false; // 手动开灯，清除压力自动关灯标志
            osal_mutex_unlock(&g_led_mutex);
            if (start_breathing_task() != ERRCODE_SUCC) {
                PRINT("[SLE Server] Failed to start breathing task\r\n");
                if (osal_mutex_lock(&g_led_mutex) == OSAL_SUCCESS) {
                    g_led_state = LED_STATE_FULL_BRIGHT;
                    SetPwmBrightness(PWM_MAX_BRIGHTNESS);
                    osal_mutex_unlock(&g_led_mutex);
                }
            } else {
                PRINT("[SLE Server] LED set to breathing mode\r\n");
            }
            return;
    }

    osal_mutex_unlock(&g_led_mutex);
}

static void RainSensorInit(void)
{
    uapi_pin_set_mode(RAIN_SENSOR_PIN, PIN_MODE_0);
    uapi_gpio_set_dir(RAIN_SENSOR_PIN, GPIO_DIRECTION_INPUT);
    PRINT("[SLE Server] Rain sensor initialized on GPIO_%d\r\n", RAIN_SENSOR_PIN);
}

static void PressureSensorInit(void)
{
    uapi_pin_set_mode(PRESSURE_SENSOR_PIN, PIN_MODE_0);
    uapi_gpio_init();
    uapi_gpio_set_dir(PRESSURE_SENSOR_PIN, GPIO_DIRECTION_INPUT);
    PRINT("[SLE Server] Pressure sensor initialized on GPIO_%d\r\n", PRESSURE_SENSOR_PIN);
}

static void ServoInit(void)
{
    uapi_pin_set_mode(SERVO_PIN, PIN_MODE_0);
    uapi_gpio_set_dir(SERVO_PIN, GPIO_DIRECTION_OUTPUT);
    uapi_gpio_set_val(SERVO_PIN, GPIO_LEVEL_LOW);
    PRINT("[SLE Server] Servo initialized on GPIO_%d\r\n", SERVO_PIN);
}

static void ServoSetAngle(unsigned int duty)
{
    uapi_gpio_set_val(SERVO_PIN, GPIO_LEVEL_HIGH);
    uapi_systick_delay_us(duty);
    uapi_gpio_set_val(SERVO_PIN, GPIO_LEVEL_LOW);
    uapi_systick_delay_us(SERVO_FREQ_TIME - duty);
}

// 开窗
static void ServoTurnRight90(void)
{
    osal_mutex_lock(&g_servo_mutex);
    for (int i = 0; i < SERVO_COUNT; i++) {
        ServoSetAngle(1500);
    }
    g_servo_state = SERVO_STATE_RIGHT_90;
    g_window_closed_by_rain = false;    // 手动开窗，清除自动关窗标志
    g_window_closed_by_pressure = false;
    osal_mutex_unlock(&g_servo_mutex);
    PRINT("[SLE Server] Servo turned right 90° (Window opened manually)\r\n");
}

// 关窗
static void ServoTurnLeft90(void)
{
    osal_mutex_lock(&g_servo_mutex);
    for (int i = 0; i < SERVO_COUNT; i++) {
        ServoSetAngle(500);
    }
    g_servo_state = SERVO_STATE_LEFT_90;
    osal_mutex_unlock(&g_servo_mutex);
    PRINT("[SLE Server] Servo turned left 90° (Window closed manually)\r\n");
}

// 雨滴自动关窗
static void RainAutoCloseWindow(void)
{
    osal_mutex_lock(&g_servo_mutex);
    if (g_servo_state == SERVO_STATE_RIGHT_90 &&
        !g_window_closed_by_rain && !g_window_closed_by_pressure) {
        for (int i = 0; i < SERVO_COUNT; i++) {
            ServoSetAngle(2500);
        }
        g_servo_state = SERVO_STATE_LEFT_90;
        g_window_closed_by_rain = true; // 标记为雨滴自动关闭
        PRINT("[SLE Server] Rain detected! Window automatically closed by rain\r\n");
    }
    osal_mutex_unlock(&g_servo_mutex);
}

// 压力自动关灯
static void PressureAutoTurnOffLed(void)
{
    osal_mutex_lock(&g_led_mutex);
    if (g_led_state != LED_STATE_OFF && !g_led_off_by_pressure) {
        led_state_t old_state = g_led_state;
        g_led_state = LED_STATE_OFF;
        g_led_off_by_pressure = true; // 标记为压力自动关闭
        if (old_state == LED_STATE_BREATHING) {
            osal_mutex_unlock(&g_led_mutex);
            stop_breathing_task();
            osal_mutex_lock(&g_led_mutex);
        }
        SetPwmBrightness(PWM_MIN_BRIGHTNESS);
        PRINT("[SLE Server] Pressure detected! LED automatically turned off (was in %s state)\r\n",
              old_state == LED_STATE_FULL_BRIGHT ? "full bright" :
              old_state == LED_STATE_BREATHING ? "breathing" :
              old_state == LED_STATE_DIM ? "dim" : "unknown");
    }
    osal_mutex_unlock(&g_led_mutex);
}

// 压力自动关窗
static void PressureAutoCloseWindow(void)
{
    osal_mutex_lock(&g_servo_mutex);
    if (g_servo_state == SERVO_STATE_RIGHT_90 &&
        !g_window_closed_by_rain && !g_window_closed_by_pressure) {
        for (int i = 0; i < SERVO_COUNT; i++) {
            ServoSetAngle(2500);
        }
        g_servo_state = SERVO_STATE_LEFT_90;
        g_window_closed_by_pressure = true; // 标记为压力自动关闭
        PRINT("[SLE Server] Pressure detected! Window automatically closed by pressure\r\n");
    }
    osal_mutex_unlock(&g_servo_mutex);
}

static int rain_detection_task(const char *arg)
{
    unused(arg);
    RainSensorInit();
    PRINT("[SLE Server] Rain detection task started\r\n");
    bool last_rain_state = false;
    while (g_rain_task_running) {
        gpio_level_t sensor_value = uapi_gpio_get_val(RAIN_SENSOR_PIN);
        bool current_rain_detected = (sensor_value == GPIO_LEVEL_LOW); // 低电平表示有雨
        if (current_rain_detected != last_rain_state) {
            g_rain_detected = current_rain_detected;
            if (current_rain_detected) {
                PRINT("[SLE Server] Rain detected by sensor!\r\n");
                RainAutoCloseWindow();
            } else {
                PRINT("[SLE Server] No rain detected.\r\n");
            }
            last_rain_state = current_rain_detected;
        }
        osal_msleep(RAIN_CHECK_INTERVAL_MS);
    }
    PRINT("[SLE Server] Rain detection task exited\r\n");
    return 0;
}

static errcode_t start_rain_detection_task(void)
{
    if (g_rain_task_running) {
        PRINT("[SLE Server] Rain detection task already running\r\n");
        return ERRCODE_SUCC;
    }
    g_rain_task_running = true;
    osal_kthread_lock();
    g_rain_task = osal_kthread_create((osal_kthread_handler)rain_detection_task, NULL, "RainDetectTask", RAIN_TASK_STACK_SIZE);
    if (g_rain_task != NULL) {
        osal_kthread_set_priority(g_rain_task, RAIN_TASK_PRIO);
        osal_kfree(g_rain_task);
    } else {
        PRINT("[SLE Server] Failed to create rain detection task\r\n");
        g_rain_task_running = false;
        osal_kthread_unlock();
        return ERRCODE_FAIL;
    }
    osal_kthread_unlock();
    PRINT("[SLE Server] Rain detection task started successfully\r\n");
    return ERRCODE_SUCC;
}

// 保留备用
static errcode_t __attribute__((unused)) stop_rain_detection_task(void)
{
    if (!g_rain_task_running) {
        return ERRCODE_SUCC;
    }
    g_rain_task_running = false;
    int wait_count = 0;
    while (g_rain_task_running && wait_count < 10) {
        osal_msleep(100);
        wait_count++;
    }
    g_rain_task = NULL;
    PRINT("[SLE Server] Rain detection task stopped\r\n");
    return ERRCODE_SUCC;
}

static int pressure_detection_task(const char *arg)
{
    unused(arg);
    PressureSensorInit();
    PRINT("[SLE Server] Pressure detection task started on GPIO_%d\r\n", PRESSURE_SENSOR_PIN);
    bool last_pressure_state = false;
    uint32_t pressure_count = 0;
    uint32_t loop_count = 0;
    while (g_pressure_task_running) {
        loop_count++;
        gpio_level_t sensor_value = uapi_gpio_get_val(PRESSURE_SENSOR_PIN);
        bool current_pressure_detected = (sensor_value == GPIO_LEVEL_HIGH); // 高电平表示有压力
        if (current_pressure_detected != last_pressure_state) {
            g_pressure_detected = current_pressure_detected;
            pressure_count++;
            if (current_pressure_detected) {
                PRINT("[SLE Server] [%d] *** PRESSURE DETECTED! *** GPIO_%d = HIGH\r\n",
                      pressure_count, PRESSURE_SENSOR_PIN);
                PressureAutoTurnOffLed();
                PressureAutoCloseWindow();
            } else {
                PRINT("[SLE Server] [%d] --- Pressure released --- GPIO_%d = LOW\r\n",
                      pressure_count, PRESSURE_SENSOR_PIN);
            }
            last_pressure_state = current_pressure_detected;
        }
        osal_msleep(PRESSURE_CHECK_INTERVAL_MS);
    }
    PRINT("[SLE Server] Pressure detection task exited after %d loops\r\n", loop_count);
    return 0;
}

static errcode_t start_pressure_detection_task(void)
{
    if (g_pressure_task_running) {
        PRINT("[SLE Server] Pressure detection task already running\r\n");
        return ERRCODE_SUCC;
    }
    g_pressure_task_running = true;
    osal_kthread_lock();
    g_pressure_task = osal_kthread_create((osal_kthread_handler)pressure_detection_task, NULL, "PressureDetectTask", PRESSURE_TASK_STACK_SIZE);
    if (g_pressure_task != NULL) {
        osal_kthread_set_priority(g_pressure_task, PRESSURE_TASK_PRIO);
        osal_kfree(g_pressure_task);
    } else {
        PRINT("[SLE Server] Failed to create pressure detection task\r\n");
        g_pressure_task_running = false;
        osal_kthread_unlock();
        return ERRCODE_FAIL;
    }
    osal_kthread_unlock();
    PRINT("[SLE Server] Pressure detection task started successfully\r\n");
    return ERRCODE_SUCC;
}

// 保留备用
static errcode_t __attribute__((unused)) stop_pressure_detection_task(void)
{
    if (!g_pressure_task_running) {
        return ERRCODE_SUCC;
    }
    g_pressure_task_running = false;
    int wait_count = 0;
    while (g_pressure_task_running && wait_count < 10) {
        osal_msleep(100);
        wait_count++;
    }
    g_pressure_task = NULL;
    PRINT("[SLE Server] Pressure detection task stopped\r\n");
    return ERRCODE_SUCC;
}

static void example_ssaps_write_request_cbk(uint8_t server_id,
                                           uint16_t conn_id,
                                           ssaps_req_write_cb_t *write_cb_para,
                                           errcode_t status)
{
    PRINT("[SLE Server] ssaps write request cbk server_id:0x%x, conn_id:0x%x, handle:0x%x, status:0x%x\r\n",
          server_id, conn_id, write_cb_para->handle, status);
    for (uint16_t idx = 0; idx < write_cb_para->length; idx++) {
        PRINT("[SLE Server] write request cbk[0x%x] 0x%02x\r\n", idx, write_cb_para->value[idx]);
    }

    if (status == ERRCODE_SUCC) {
        char cmd_buf[16] = {0};
        uint16_t len = write_cb_para->length < (sizeof(cmd_buf) - 1) ? write_cb_para->length : (sizeof(cmd_buf) - 1);
        memcpy_s(cmd_buf, sizeof(cmd_buf), write_cb_para->value, len);
        cmd_buf[len] = '\0';

        PRINT("[SLE Server] Processing command: %s\r\n", cmd_buf);

        if (strcmp(cmd_buf, "SW9_ON") == 0) {
            PRINT("[SLE Server] Command SW9_ON - Setting LED to full brightness\r\n");
            SetLedState(LED_STATE_FULL_BRIGHT);
            PRINT("[SLE Server] SW9_ON completed\r\n");
        } else if (strcmp(cmd_buf, "SW9_OFF") == 0) {
            PRINT("[SLE Server] Command SW9_OFF - Turning LED off\r\n");
            SetLedState(LED_STATE_OFF);
            PRINT("[SLE Server] SW9_OFF completed\r\n");
        } else if (strcmp(cmd_buf, "SW10_ON") == 0) {
            PRINT("[SLE Server] Command SW10_ON - Setting LED to breathing mode\r\n");
            SetLedState(LED_STATE_BREATHING);
            PRINT("[SLE Server] SW10_ON completed\r\n");
        } else if (strcmp(cmd_buf, "SW10_OFF") == 0) {
            PRINT("[SLE Server] Command SW10_OFF - Setting LED to dim mode\r\n");
            SetLedState(LED_STATE_DIM);
            PRINT("[SLE Server] SW10_OFF completed\r\n");
        } else if (strcmp(cmd_buf, "SW8_ON") == 0) {
            PRINT("[SLE Server] Command SW8_ON - Opening window\r\n");
            ServoInit();
            ServoTurnRight90();
            PRINT("[SLE Server] SW8_ON completed\r\n");
        } else if (strcmp(cmd_buf, "SW8_OFF") == 0) {
            PRINT("[SLE Server] Command SW8_OFF - Closing window\r\n");
            ServoInit();
            ServoTurnLeft90();
            PRINT("[SLE Server] SW8_OFF completed\r\n");
        } else {
            PRINT("[SLE Server] Unknown command: %s (length: %d)\r\n", cmd_buf, len);
        }
    } else {
        PRINT("[SLE Server] write request failed, error code: 0x%x\r\n", status);
    }
}

static void example_ssaps_mtu_changed_cbk(uint8_t server_id,
                                          uint16_t conn_id,
                                          ssap_exchange_info_t *mtu_size,
                                          errcode_t status)
{
    PRINT("[SLE Server] ssaps mtu changed cbk server_id:0x%x, conn_id:0x%x, mtu_size:0x%x, status:0x%x\r\n", server_id,
          conn_id, mtu_size->mtu_size, status);
}

static void example_ssaps_start_service_cbk(uint8_t server_id, uint16_t handle, errcode_t status)
{
    PRINT("[SLE Server] start service cbk server_id:0x%x, handle:0x%x, status:0x%x\r\n", server_id, handle, status);
}

static errcode_t example_sle_ssaps_register_cbks(void)
{
    ssaps_callbacks_t ssaps_cbk = {0};
    ssaps_cbk.start_service_cb = example_ssaps_start_service_cbk;
    ssaps_cbk.mtu_changed_cb = example_ssaps_mtu_changed_cbk;
    ssaps_cbk.read_request_cb = example_ssaps_read_request_cbk;
    ssaps_cbk.write_request_cb = example_ssaps_write_request_cbk;
    return ssaps_register_callbacks(&ssaps_cbk);
}

static errcode_t example_sle_server_service_add(void)
{
    errcode_t ret = ERRCODE_FAIL;
    sle_uuid_t service_uuid = {0};
    example_sle_uuid_setu2(SLE_UUID_SERVER_SERVICE, &service_uuid);
    ret = ssaps_add_service_sync(g_server_id, &service_uuid, true, &g_service_handle);
    if (ret != ERRCODE_SUCC) {
        PRINT("[SLE Server] sle uuid add service fail, ret:0x%x\r\n", ret);
        return ERRCODE_FAIL;
    }

    PRINT("[SLE Server] sle uuid add service service_handle: %u\r\n", g_service_handle);

    return ERRCODE_SUCC;
}

static errcode_t example_sle_server_property_add(void)
{
    errcode_t ret = ERRCODE_FAIL;
    ssaps_property_info_t property = {0};
    ssaps_desc_info_t descriptor = {0};
    uint8_t ntf_value[] = {0x01, 0x0};

    property.permissions = SSAP_PERMISSION_READ | SSAP_PERMISSION_WRITE;
    example_sle_uuid_setu2(SLE_UUID_SERVER_PROPERTY, &property.uuid);
    property.value = osal_vmalloc(sizeof(g_sle_property_value));
    if (property.value == NULL) {
        PRINT("[SLE Server] sle property mem fail\r\n");
        return ERRCODE_MALLOC;
    }

    if (memcpy_s(property.value, sizeof(g_sle_property_value), g_sle_property_value, sizeof(g_sle_property_value)) !=
        EOK) {
        osal_vfree(property.value);
        PRINT("[SLE Server] sle property mem cpy fail\r\n");
        return ERRCODE_MEMCPY;
    }
    ret = ssaps_add_property_sync(g_server_id, g_service_handle, &property, &g_property_handle);
    if (ret != ERRCODE_SUCC) {
        PRINT("[SLE Server] sle uuid add property fail, ret:0x%x\r\n", ret);
        osal_vfree(property.value);
        return ERRCODE_FAIL;
    }

    PRINT("[SLE Server] sle uuid add property property_handle: %u\r\n", g_property_handle);

    descriptor.permissions = SSAP_PERMISSION_READ | SSAP_PERMISSION_WRITE;
    descriptor.value = osal_vmalloc(sizeof(ntf_value));
    if (descriptor.value == NULL) {
        PRINT("[SLE Server] sle descriptor mem fail\r\n");
        osal_vfree(property.value);
        return ERRCODE_MALLOC;
    }
    if (memcpy_s(descriptor.value, sizeof(ntf_value), ntf_value, sizeof(ntf_value)) != EOK) {
        PRINT("[SLE Server] sle descriptor mem cpy fail\r\n");
        osal_vfree(property.value);
        osal_vfree(descriptor.value);
        return ERRCODE_MEMCPY;
    }
    ret = ssaps_add_descriptor_sync(g_server_id, g_service_handle, g_property_handle, &descriptor);
    if (ret != ERRCODE_SUCC) {
        PRINT("[SLE Server] sle uuid add descriptor fail, ret:0x%x\r\n", ret);
        osal_vfree(property.value);
        osal_vfree(descriptor.value);
        return ERRCODE_FAIL;
    }
    osal_vfree(property.value);
    osal_vfree(descriptor.value);
    return ERRCODE_SUCC;
}

static errcode_t example_sle_server_add(void)
{
    errcode_t ret = ERRCODE_FAIL;
    sle_uuid_t app_uuid = {0};

    PRINT("[SLE Server] sle uuid add service in\r\n");
    app_uuid.len = sizeof(g_sle_uuid_app_uuid);
    if (memcpy_s(app_uuid.uuid, app_uuid.len, g_sle_uuid_app_uuid, sizeof(g_sle_uuid_app_uuid)) != EOK) {
        return ERRCODE_MEMCPY;
    }
    ssaps_register_server(&app_uuid, &g_server_id);

    if (example_sle_server_service_add() != ERRCODE_SUCC) {
        ssaps_unregister_server(g_server_id);
        return ERRCODE_FAIL;
    }

    if (example_sle_server_property_add() != ERRCODE_SUCC) {
        ssaps_unregister_server(g_server_id);
        return ERRCODE_FAIL;
    }
    PRINT("[SLE Server] sle uuid add service, server_id:0x%x, service_handle:0x%x, property_handle:0x%x\r\n",
          g_server_id, g_service_handle, g_property_handle);
    ret = ssaps_start_service(g_server_id, g_service_handle);
    if (ret != ERRCODE_SUCC) {
        PRINT("[SLE Server] sle uuid add service fail, ret:0x%x\r\n", ret);
        return ERRCODE_FAIL;
    }
    PRINT("[SLE Server] sle uuid add service out\r\n");
    return ERRCODE_SUCC;
}
static void example_sle_connect_state_changed_cbk(uint16_t conn_id,
                                                  const sle_addr_t *addr,
                                                  sle_acb_state_t conn_state,
                                                  sle_pair_state_t pair_state,
                                                  sle_disc_reason_t disc_reason)
{
    PRINT(
        "[SLE Server] connect state changed conn_id:0x%02x, conn_state:0x%x, pair_state:0x%x, \
        disc_reason:0x%x\r\n",
        conn_id, conn_state, pair_state, disc_reason);
    PRINT("[SLE Server] connect state changed addr:%02x:**:**:**:%02x:%02x\r\n", addr->addr[0], addr->addr[4],
          addr->addr[5]);
    g_conn_id = conn_id;
}

static void example_sle_pair_complete_cbk(uint16_t conn_id, const sle_addr_t *addr, errcode_t status)
{
    PRINT("[SLE Server] pair complete conn_id:0x%02x, status:0x%x\r\n", conn_id, status);
    PRINT("[SLE Server] pair complete addr:%02x:**:**:**:%02x:%02x\r\n", addr->addr[0], addr->addr[4], addr->addr[5]);

    if (status == ERRCODE_SUCC) {
        PRINT("[SLE Server] Pairing successful, initializing components\r\n");
        PwmLedInit();
        RainSensorInit();
        PressureSensorInit();
        if (osal_mutex_init(&g_servo_mutex) != OSAL_SUCCESS) {
            PRINT("[SLE Server] Failed to init servo mutex\r\n");
        }
        if (osal_mutex_init(&g_led_mutex) != OSAL_SUCCESS) {
            PRINT("[SLE Server] Failed to init LED mutex\r\n");
        }
        if (osal_mutex_init(&g_pwm_mutex) != OSAL_SUCCESS) {
            PRINT("[SLE Server] Failed to init PWM mutex\r\n");
        }
        if (start_rain_detection_task() != ERRCODE_SUCC) {
            PRINT("[SLE Server] Failed to start rain detection task\r\n");
        } else {
            PRINT("[SLE Server] Rain detection task started\r\n");
        }
        if (start_pressure_detection_task() != ERRCODE_SUCC) {
            PRINT("[SLE Server] Failed to start pressure detection task\r\n");
        } else {
            PRINT("[SLE Server] Pressure detection task started\r\n");
        }
        PRINT("[SLE Server] All components initialized successfully\r\n");
    }
}

static errcode_t example_sle_conn_register_cbks(void)
{
    sle_connection_callbacks_t conn_cbks = {0};
    conn_cbks.connect_state_changed_cb = example_sle_connect_state_changed_cbk;
    conn_cbks.pair_complete_cb = example_sle_pair_complete_cbk;
    return sle_connection_register_callbacks(&conn_cbks);
}

static int example_sle_led_server_task(const char *arg)
{
    unused(arg);

    (void)osal_msleep(5000); /* wait for sle init done */

    osal_mutex_init(&g_led_mutex);
    osal_mutex_init(&g_servo_mutex);
    osal_mutex_init(&g_pwm_mutex);

    uapi_gpio_init();

    PwmLedInit();
    ServoInit();

    SetLedState(LED_STATE_OFF);

    PRINT("[SLE Server] PWM LED and Servo initialized.\r\n");

    if (start_rain_detection_task() != ERRCODE_SUCC) {
        PRINT("[SLE Server] Failed to start rain detection task\r\n");
        return -1;
    }

    if (start_pressure_detection_task() != ERRCODE_SUCC) {
        PRINT("[SLE Server] Failed to start pressure detection task\r\n");
        return -1;
    }

    PRINT("[SLE Server] try enable.\r\n");
    if (enable_sle() != ERRCODE_SUCC) {
        PRINT("[SLE Server] sle enbale fail !\r\n");
        return -1;
    }

    if (example_sle_conn_register_cbks() != ERRCODE_SUCC) {
        PRINT("[SLE Server] sle conn register cbks fail !\r\n");
        return -1;
    }

    if (example_sle_ssaps_register_cbks() != ERRCODE_SUCC) {
        PRINT("[SLE Server] sle ssaps register cbks fail !\r\n");
        return -1;
    }

    if (example_sle_server_add() != ERRCODE_SUCC) {
        PRINT("[SLE Server] sle server add fail !\r\n");
        return -1;
    }

    if (example_sle_server_adv_init() != ERRCODE_SUCC) {
        PRINT("[SLE Server] sle server adv fail !\r\n");
        return -1;
    }

    PRINT("[SLE Server] init ok\r\n");

    return 0;
}

#define SLE_LED_SER_TASK_PRIO 24
#define SLE_LED_SER_STACK_SIZE 0x2000

static void example_sle_led_server_entry(void)
{
    osal_task *task_handle = NULL;

    osal_kthread_lock();
    task_handle = osal_kthread_create((osal_kthread_handler)example_sle_led_server_task, 0, "SLELedServerTask",
                                      SLE_LED_SER_STACK_SIZE);
    if (task_handle != NULL) {
        osal_kthread_set_priority(task_handle, SLE_LED_SER_TASK_PRIO);
        osal_kfree(task_handle);
    }
    osal_kthread_unlock();
}

/* Run the example_sle_led_server_entry. */
 app_run(example_sle_led_server_entry);