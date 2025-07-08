#include "../inc/device.h"
#include "pinctrl.h"
#include "gpio.h"
#include "i2c.h"
#include "hal_gpio.h"
#include "systick.h"
#include "debug_print.h"
#include "tcxo.h"
#include "soc_osal.h"
#include "app_init.h"

#include "../inc/ssd1306.h"
#include "../inc/ssd1306_fonts.h"

#define CONFIG_I2C_SCL_MASTER_PIN 15    // I2C SCL
#define CONFIG_I2C_SDA_MASTER_PIN 16    // I2C SDA
#define CONFIG_I2C_MASTER_PIN_MODE 2
#define I2C_MASTER_ADDR 0x0
#define I2C_SET_BANDRATE 400000

#define SERVO_COUNT 10                  // 舵机PWM发送次数
#define SERVO_GPIO_PIN 3                // 舵机控制引脚
#define SERVO_FREQ_TIME 20000           // PWM周期 (us)
#define SERVO_ANGLE_0 500               // 0度脉宽 (us)
#define SERVO_ANGLE_90 1500             // 90度脉宽 (us)
#define SERVO_ANGLE_180 2500            // 180度脉宽 (us)

#define MOTOR_INA_PIN 11                // L9110 INA
#define MOTOR_INB_PIN 12                // L9110 INB

#define RGB_LED_PIN 5                   // RGB灯引脚
#define RGB_LED_PIN_MODE 4
#define RGB_LED_COUNT 16                // RGB灯数量
#define RGB_LED_COUNT_EXTRA 18


static bool g_oled_initialized = false;
static bool g_servo_initialized = false;
static bool g_motor_initialized = false;
static bool g_rgb_initialized = false;

// OLED I2C引脚初始化
static void oled_i2c_init_pin(void)
{
    uapi_pin_set_mode(CONFIG_I2C_SCL_MASTER_PIN, CONFIG_I2C_MASTER_PIN_MODE);
    uapi_pin_set_mode(CONFIG_I2C_SDA_MASTER_PIN, CONFIG_I2C_MASTER_PIN_MODE);
}

// OLED完整初始化
static errcode_t oled_full_init(void)
{
    uint32_t baudrate = I2C_SET_BANDRATE;
    uint32_t hscode = I2C_MASTER_ADDR;

    oled_i2c_init_pin();

    errcode_t ret = uapi_i2c_master_init(1, baudrate, hscode);
    if (ret != ERRCODE_SUCC) {
        PRINT("[Device] OLED I2C init failed, ret = 0x%x\r\n", ret);
        return ret;
    }

    ssd1306_Init();
    PRINT("[Device] OLED I2C and display initialization completed\r\n");
    return ERRCODE_SUCC;
}


static void servo_set_angle(unsigned int pulse_width)
{
    uapi_gpio_set_val(SERVO_GPIO_PIN, GPIO_LEVEL_HIGH);
    uapi_systick_delay_us(pulse_width);
    uapi_gpio_set_val(SERVO_GPIO_PIN, GPIO_LEVEL_LOW);
    uapi_systick_delay_us(SERVO_FREQ_TIME - pulse_width);
}

static void servo_move_to_angle(unsigned int pulse_width)
{
    for (int i = 0; i < SERVO_COUNT; i++) {
        servo_set_angle(pulse_width);
    }
}

static void motor_forward(void)
{
    uapi_gpio_set_val(MOTOR_INA_PIN, GPIO_LEVEL_HIGH);
    uapi_gpio_set_val(MOTOR_INB_PIN, GPIO_LEVEL_LOW);
}

static void motor_backward(void)
{
    uapi_gpio_set_val(MOTOR_INA_PIN, GPIO_LEVEL_LOW);
    uapi_gpio_set_val(MOTOR_INB_PIN, GPIO_LEVEL_HIGH);
}

static void motor_brake(void)
{
    uapi_gpio_set_val(MOTOR_INA_PIN, GPIO_LEVEL_LOW);
    uapi_gpio_set_val(MOTOR_INB_PIN, GPIO_LEVEL_LOW);
}

static void rgb_one_bit(void)
{
    uapi_reg_setbit(0x44028030, 5);
    uapi_tcxo_delay_us(8);
    uapi_reg_setbit(0x44028034, 5);
    uapi_tcxo_delay_us(2);
}

static void rgb_zero_bit(void)
{
    uint32_t preg = 0;
    preg = preg;
    uapi_reg_setbit(0x44028030, 5);
    for (int i = 0; i < 7; i++) {
        uapi_reg_read32(0x44028030, preg);
    }
    uapi_reg_setbit(0x44028034, 5);
    uapi_tcxo_delay_us(2);
}

static void rgb_shutdown_led(void)
{
    for (int i = 0; i < 8; i++) {
        rgb_zero_bit();  // R
    }
    for (int i = 0; i < 8; i++) {
        rgb_zero_bit();  // G
    }
    for (int i = 0; i < 8; i++) {
        rgb_zero_bit();  // B
    }
}

static void rgb_ensure_all_leds_off(void)
{
    uapi_reg_setbit(0x44028034, 5);
    uapi_tcxo_delay_us(300);

    for (int j = 0; j < 3; j++) {
        uapi_reg_setbit(0x44028034, 5);
        uapi_tcxo_delay_us(300);

        for (int i = 0; i < RGB_LED_COUNT + 2; i++) {
            rgb_shutdown_led();
        }
        uapi_tcxo_delay_us(100);
    }
    uapi_reg_setbit(0x44028034, 5);
    uapi_tcxo_delay_us(300);
}

static void rgb_white_led(void)
{
    for (int i = 0; i < 8; i++) {
        rgb_one_bit();  // R
    }
    for (int i = 0; i < 8; i++) {
        rgb_one_bit();  // G
    }
    for (int i = 0; i < 8; i++) {
        rgb_one_bit();  // B
    }
}

static void rgb_red_led(uint8_t brightness)
{
    for (int i = 0; i < 8; i++) {
        rgb_zero_bit();
    }

    for (int i = 7; i >= 0; i--) {
        if (brightness & (1 << i)) {
            rgb_one_bit();
        } else {
            rgb_zero_bit();
        }
    }

    for (int i = 0; i < 8; i++) {
        rgb_zero_bit();
    }
}

static errcode_t rgb_init(void)
{
    if (!g_rgb_initialized) {
        uapi_pin_set_mode(RGB_LED_PIN, RGB_LED_PIN_MODE);
        uapi_gpio_set_dir(RGB_LED_PIN, GPIO_DIRECTION_OUTPUT);
        uapi_reg_setbit(0x44028034, 5);

        uapi_tcxo_delay_us(100);

        rgb_ensure_all_leds_off();

        g_rgb_initialized = true;
        PRINT("[Device] RGB LEDs initialized\r\n");
    }
    return ERRCODE_SUCC;
}

#include "osal_task.h"
#include "osal_event.h"

typedef enum {
    RGB_EFFECT_NONE = 0,
    RGB_EFFECT_WHITE,      // 白灯常亮
    RGB_EFFECT_BREATH,     // 红色呼吸灯
    RGB_EFFECT_RAINBOW     // 彩色跑马灯
} rgb_effect_type_t;

#define RGB_TASK_STACK_SIZE 4096 // RGB效果线程栈大小
#define RGB_EVENT_STOP 0x01
#define RGB_EVENT_CHANGE 0x02
#define BREATH_CYCLE_MS 1000     // 呼吸周期

static osal_task *g_rgb_task = NULL;
static osal_event g_rgb_event;
static bool g_rgb_task_running = false;
static rgb_effect_type_t g_current_effect = RGB_EFFECT_NONE;
static rgb_effect_type_t g_target_effect = RGB_EFFECT_NONE;


static bool check_effect_change(void)
{
    // 检查是否收到更改事件
    if (osal_event_read(&g_rgb_event, RGB_EVENT_CHANGE, 0, OSAL_WAITMODE_OR) == OSAL_SUCCESS) {
        return true;
    }

    // 检查是否收到停止事件
    if (osal_event_read(&g_rgb_event, RGB_EVENT_STOP, 0, OSAL_WAITMODE_OR) == OSAL_SUCCESS) {
        return true;
    }

    // 检查目标效果是否与当前效果不同
    if (g_target_effect != g_current_effect) {
        return true;
    }

    return false;
}

static void run_white_effect(void)
{
    PRINT("[Device] Running white light effect\r\n");
    rgb_ensure_all_leds_off();
    uapi_reg_setbit(0x44028034, 5);
    uapi_tcxo_delay_us(300);
    for (int j = 0; j < 3; j++) {
        uapi_reg_setbit(0x44028034, 5); // 复位
        uapi_tcxo_delay_us(300);
        for (int i = 0; i < RGB_LED_COUNT + 2; i++) {
            rgb_white_led();
        }
        uapi_tcxo_delay_us(100);
    }
    while (!check_effect_change()) {
        if ((uapi_tcxo_get_ms() % 2000) < 10) {
            uapi_reg_setbit(0x44028034, 5);
            uapi_tcxo_delay_us(300);

            for (int i = 0; i < RGB_LED_COUNT; i++) {
                rgb_white_led();
            }
        }
        osal_msleep(50);
    }
    PRINT("[Device] White light effect stopped\r\n");
}

static uint8_t calculate_breath_brightness(uint32_t time_ms, uint32_t cycle_ms)
{
    uint32_t position = time_ms % cycle_ms;

    uint8_t brightness;
    if (position < cycle_ms / 2) {
        // 亮度增加
        brightness = (uint8_t)((position * 255) / (cycle_ms / 2));
    } else {
        // 亮度减少
        brightness = (uint8_t)(((cycle_ms - position) * 255) / (cycle_ms / 2));
    }

    return brightness;
}

static void run_breath_effect(void)
{
    PRINT("[Device] Running red breath effect\r\n");
    rgb_ensure_all_leds_off();
    uint32_t start_time = (uint32_t)uapi_tcxo_get_ms();
    uint32_t current_time;
    uint8_t last_brightness = 0xFF;

    while (!check_effect_change()) {
        current_time = (uint32_t)uapi_tcxo_get_ms();
        uint8_t brightness = calculate_breath_brightness(current_time - start_time, BREATH_CYCLE_MS);
        if (brightness != last_brightness) {
            uapi_reg_setbit(0x44028034, 5);
            uapi_tcxo_delay_us(300);
            for (int i = 0; i < RGB_LED_COUNT; i++) {
                rgb_red_led(brightness);
            }
            last_brightness = brightness;
        }
        osal_msleep(10);
    }
    PRINT("[Device] Red breath effect stopped\r\n");
}
static void rgb_color_led(uint8_t red, uint8_t green, uint8_t blue)
{
    for (int i = 7; i >= 0; i--) {
        if (green & (1 << i)) {
            rgb_one_bit();
        } else {
            rgb_zero_bit();
        }
    }

    for (int i = 7; i >= 0; i--) {
        if (red & (1 << i)) {
            rgb_one_bit();
        } else {
            rgb_zero_bit();
        }
    }

    for (int i = 7; i >= 0; i--) {
        if (blue & (1 << i)) {
            rgb_one_bit();
        } else {
            rgb_zero_bit();
        }
    }
}
static void run_rainbow_effect(void)
{
    PRINT("[Device] Running rainbow effect\r\n");

    const uint8_t colors[][3] = {
        {255, 0, 0},     // 红
        {0, 255, 0},     // 绿
        {0, 0, 255},     // 蓝
        {255, 255, 0},   // 黄
        {255, 0, 255},   // 紫
        {0, 255, 255},   // 青
        {255, 255, 255}  // 白
    };
    const int num_colors = sizeof(colors) / sizeof(colors[0]);

    rgb_ensure_all_leds_off();

    uint8_t led_states[RGB_LED_COUNT][3];
    for (int i = 0; i < RGB_LED_COUNT; i++) {
        led_states[i][0] = 0; // R
        led_states[i][1] = 0; // G
        led_states[i][2] = 0; // B
    }

    uint32_t current_time = (uint32_t)uapi_tcxo_get_ms();
    uint32_t last_update_time = current_time;
    int color_index = 0;
    int position = 0;

    const uint32_t UPDATE_INTERVAL_MS = 100;

    while (1) {
        if (check_effect_change()) {
            PRINT("[Device] Rainbow effect interrupted\r\n");
            return;
        }

        current_time = (uint32_t)uapi_tcxo_get_ms();

        if ((current_time - last_update_time) >= UPDATE_INTERVAL_MS) {
            led_states[position][0] = 0;
            led_states[position][1] = 0;
            led_states[position][2] = 0;

            position = (position + 1) % RGB_LED_COUNT;

            led_states[position][0] = colors[color_index][0]; // R
            led_states[position][1] = colors[color_index][1]; // G
            led_states[position][2] = colors[color_index][2]; // B

            color_index = (color_index + 1) % num_colors;

            uapi_reg_setbit(0x44028034, 5);
            uapi_tcxo_delay_us(300);

            for (int i = 0; i < RGB_LED_COUNT; i++) {
                rgb_color_led(led_states[i][0], led_states[i][1], led_states[i][2]);
            }

            last_update_time = current_time;
        }

        osal_msleep(10);
    }

    rgb_ensure_all_leds_off();

    PRINT("[Device] Rainbow effect completed\r\n");
}

static int rgb_effect_thread(void *arg)
{
    (void)arg;
    PRINT("[Device] RGB effect thread started\r\n");
    while (g_rgb_task_running) {
        g_current_effect = g_target_effect;
        switch (g_current_effect) {
            case RGB_EFFECT_WHITE:
                run_white_effect();
                break;
            case RGB_EFFECT_BREATH:
                run_breath_effect();
                break;
            case RGB_EFFECT_RAINBOW:
                run_rainbow_effect();
                break;
            case RGB_EFFECT_NONE:
            default:
                rgb_ensure_all_leds_off();
                if (!check_effect_change()) {
                    osal_msleep(100);
                }
                break;
        }
    }
    rgb_ensure_all_leds_off();
    PRINT("[Device] RGB effect thread exited\r\n");
    return 0;
}

// 启动RGB灯效果线程
static errcode_t start_rgb_effect_thread(void)
{
    if (g_rgb_task_running) {
        PRINT("[Device] RGB effect thread already running\r\n");
        return ERRCODE_SUCC;
    }
    if (rgb_init() != ERRCODE_SUCC) {
        PRINT("[Device] RGB LEDs initialization failed\r\n");
        return ERRCODE_FAIL;
    }
    if (osal_event_init(&g_rgb_event) != OSAL_SUCCESS) {
        PRINT("[Device] Failed to initialize RGB event\r\n");
        return ERRCODE_FAIL;
    }
    if (osal_event_clear(&g_rgb_event, RGB_EVENT_STOP | RGB_EVENT_CHANGE) != OSAL_SUCCESS) {
        PRINT("[Device] Failed to clear RGB event\r\n");
        osal_event_destroy(&g_rgb_event);
        return ERRCODE_FAIL;
    }
    g_current_effect = RGB_EFFECT_NONE;
    g_target_effect = RGB_EFFECT_NONE;
    g_rgb_task_running = true;
    g_rgb_task = osal_kthread_create(rgb_effect_thread, NULL, "rgb_effect", RGB_TASK_STACK_SIZE);
    if (g_rgb_task == NULL) {
        PRINT("[Device] Failed to create RGB effect thread\r\n");
        g_rgb_task_running = false;
        osal_event_destroy(&g_rgb_event);
        return ERRCODE_FAIL;
    }
    if (osal_kthread_set_priority(g_rgb_task, 25) != OSAL_SUCCESS) { // 低优先级
        PRINT("[Device] Warning: Failed to set RGB effect thread priority\r\n");
    }
    PRINT("[Device] RGB effect thread started\r\n");
    return ERRCODE_SUCC;
}

// 停止RGB灯效果线程
static errcode_t stop_rgb_effect_thread(void)
{
    if (!g_rgb_task_running) {
        PRINT("[Device] RGB effect thread not running\r\n");
        return ERRCODE_SUCC;
    }
    g_target_effect = RGB_EFFECT_NONE;
    if (osal_event_write(&g_rgb_event, RGB_EVENT_STOP) != OSAL_SUCCESS) {
        PRINT("[Device] Failed to set stop event for RGB effect\r\n");
    }
    rgb_ensure_all_leds_off();
    int wait_count = 0;
    while (g_rgb_task_running && wait_count < 20) {
        if (wait_count % 5 == 0) {
            rgb_ensure_all_leds_off();
        }
        osal_msleep(100);
        wait_count++;
    }
    if (g_rgb_task_running) {
        PRINT("[Device] RGB effect thread did not exit properly\r\n");
        g_rgb_task_running = false;
    }
    rgb_ensure_all_leds_off();
    if (g_rgb_task != NULL) {
        g_rgb_task = NULL;
    }
    osal_event_destroy(&g_rgb_event);
    g_current_effect = RGB_EFFECT_NONE;
    g_target_effect = RGB_EFFECT_NONE;
    osal_msleep(100);
    PRINT("[Device] RGB effect thread stopped\r\n");
    return ERRCODE_SUCC;
}

// 设置RGB灯效果
static errcode_t set_rgb_effect(rgb_effect_type_t effect)
{
    if (!g_rgb_task_running) {
        errcode_t ret = start_rgb_effect_thread();
        if (ret != ERRCODE_SUCC) {
            PRINT("[Device] Failed to start RGB effect thread\r\n");
            return ret;
        }
    }
    g_target_effect = effect;
    if (osal_event_write(&g_rgb_event, RGB_EVENT_CHANGE) != OSAL_SUCCESS) {
        PRINT("[Device] Failed to set change event for RGB effect\r\n");
    }
    if (effect == RGB_EFFECT_NONE) {
        return stop_rgb_effect_thread();
    }
    return ERRCODE_SUCC;
}

// 关闭所有RGB灯
errcode_t device_rgb_all_off(void)
{
    return set_rgb_effect(RGB_EFFECT_NONE);
}

// 打开所有白色RGB灯
errcode_t device_rgb_all_white_on(void)
{
    return set_rgb_effect(RGB_EFFECT_WHITE);
}

// 打开红色呼吸灯效果
errcode_t device_rgb_breath_effect_on(void)
{
    return set_rgb_effect(RGB_EFFECT_BREATH);
}

// 打开彩色跑马灯效果
errcode_t device_rgb_rainbow_effect_on(void)
{
    return set_rgb_effect(RGB_EFFECT_RAINBOW);
}

// 处理SW5按钮操作
errcode_t device_sw5_operation(bool enable)
{
    PRINT("[Device] SW5 operation: enable = %d\r\n", enable);
    if (enable) {
        return device_rgb_all_white_on(); // 开灯
    } else {
        return device_rgb_all_off(); // 关灯
    }
}

// 处理SW6按钮操作
errcode_t device_sw6_operation(bool enable)
{
    PRINT("[Device] SW6 operation: enable = %d\r\n", enable);
    if (enable) {
        return device_rgb_breath_effect_on(); // 红色呼吸灯
    } else {
        return device_rgb_rainbow_effect_on(); // 彩色跑马灯
    }
}

errcode_t device_oled_init(void)
{
    if (!g_oled_initialized) {
        if (oled_full_init() == ERRCODE_SUCC) {
            g_oled_initialized = true;
            return ERRCODE_SUCC;
        } else {
            return ERRCODE_FAIL;
        }
    }
    return ERRCODE_SUCC;
}

errcode_t device_oled_show_hello(void)
{
    if (device_oled_init() != ERRCODE_SUCC) {
        PRINT("[Device] OLED initialization failed\r\n");
        return ERRCODE_FAIL;
    }
    ssd1306_SetDisplayOn(1);
    ssd1306_Fill(Black);
    ssd1306_SetCursor(30, 25);
    ssd1306_DrawString("Hello", Font_16x26, White);
    ssd1306_UpdateScreen();
    PRINT("[Device] OLED display ON and showing Hello\r\n");
    return ERRCODE_SUCC;
}

errcode_t device_oled_turn_off(void)
{
    if (!g_oled_initialized) {
        PRINT("[Device] OLED not initialized, nothing to turn off\r\n");
        return ERRCODE_FAIL;
    }
    ssd1306_Fill(Black);
    ssd1306_UpdateScreen();
    ssd1306_SetDisplayOn(0);
    PRINT("[Device] OLED display turned OFF\r\n");
    return ERRCODE_SUCC;
}

errcode_t device_oled_reset(void)
{
    if (g_oled_initialized) {
        ssd1306_Fill(Black);
        ssd1306_UpdateScreen();
        ssd1306_SetDisplayOn(0);
        PRINT("[Device] OLED display reset: turned off\r\n");
    }
    // 重置初始化状态，强制下次重新初始化
    g_oled_initialized = false;
    PRINT("[Device] OLED reset completed, ready for fresh initialization\r\n");
    return ERRCODE_SUCC;
}

errcode_t device_oled_show_tunnel1(void)
{
    if (device_oled_init() != ERRCODE_SUCC) {
        PRINT("[Device] OLED initialization failed\r\n");
        return ERRCODE_FAIL;
    }
    ssd1306_SetDisplayOn(1);
    ssd1306_Fill(Black);
    ssd1306_SetCursor(30, 25);
    ssd1306_DrawString("CH1", Font_16x26, White);
    ssd1306_UpdateScreen();
    PRINT("[Device] OLED display ON and showing CH1\r\n");
    return ERRCODE_SUCC;
}

errcode_t device_oled_show_tunnel2(void)
{
    if (device_oled_init() != ERRCODE_SUCC) {
        PRINT("[Device] OLED initialization failed\r\n");
        return ERRCODE_FAIL;
    }
    ssd1306_SetDisplayOn(1);
    ssd1306_Fill(Black);
    ssd1306_SetCursor(30, 25);
    ssd1306_DrawString("CH2", Font_16x26, White);
    ssd1306_UpdateScreen();
    PRINT("[Device] OLED display ON and showing CH2\r\n");
    return ERRCODE_SUCC;
}

/* ========== 舵机设备接口实现 ========== */

errcode_t device_servo_init(void)
{
    if (!g_servo_initialized) {
        uapi_pin_set_mode(SERVO_GPIO_PIN, HAL_PIO_FUNC_GPIO);
        uapi_gpio_set_dir(SERVO_GPIO_PIN, GPIO_DIRECTION_OUTPUT);
        uapi_gpio_set_val(SERVO_GPIO_PIN, GPIO_LEVEL_LOW);

        g_servo_initialized = true;
        PRINT("[Device] Servo initialized successfully\r\n");
    }
    return ERRCODE_SUCC;
}

errcode_t device_servo_turn_counterclockwise_180(void)
{
    if (device_servo_init() != ERRCODE_SUCC) {
        PRINT("[Device] Servo initialization failed\r\n");
        return ERRCODE_FAIL;
    }
    servo_move_to_angle(SERVO_ANGLE_0);      // 转到0度
    PRINT("[Device] Servo turned counterclockwise 180 degrees\r\n");
    return ERRCODE_SUCC;
}

errcode_t device_servo_turn_clockwise_180(void)
{
    if (device_servo_init() != ERRCODE_SUCC) {
        PRINT("[Device] Servo initialization failed\r\n");
        return ERRCODE_FAIL;
    }
    servo_move_to_angle(SERVO_ANGLE_180);  // 转到180度
    PRINT("[Device] Servo turned clockwise 180 degrees\r\n");
    return ERRCODE_SUCC;
}

errcode_t device_motor_init(void)
{
    if (!g_motor_initialized) {
        uapi_pin_set_mode(MOTOR_INA_PIN, HAL_PIO_FUNC_GPIO);
        uapi_pin_set_mode(MOTOR_INB_PIN, HAL_PIO_FUNC_GPIO);

        uapi_gpio_set_dir(MOTOR_INA_PIN, GPIO_DIRECTION_OUTPUT);
        uapi_gpio_set_dir(MOTOR_INB_PIN, GPIO_DIRECTION_OUTPUT);

        motor_brake();

        g_motor_initialized = true;
        PRINT("[Device] L9110 Motor initialized successfully\r\n");
    }
    return ERRCODE_SUCC;
}

errcode_t device_motor_start(void)
{
    if (device_motor_init() != ERRCODE_SUCC) {
        PRINT("[Device] Motor initialization failed\r\n");
        return ERRCODE_FAIL;
    }
    motor_forward();
    PRINT("[Device] Motor started (forward rotation)\r\n");
    return ERRCODE_SUCC;
}

errcode_t device_motor_stop(void)
{
    if (!g_motor_initialized) {
        PRINT("[Device] Motor not initialized, nothing to stop\r\n");
        return ERRCODE_FAIL;
    }
    motor_brake();
    PRINT("[Device] Motor stopped\r\n");
    return ERRCODE_SUCC;
}

errcode_t device_motor_reverse(void)
{
    if (device_motor_init() != ERRCODE_SUCC) {
        PRINT("[Device] Motor initialization failed\r\n");
        return ERRCODE_FAIL;
    }
    motor_backward();
    PRINT("[Device] Motor reverse rotation\r\n");
    return ERRCODE_SUCC;
}

// 获取RGB灯初始化状态
bool device_get_rgb_initialized(void)
{
    return g_rgb_initialized;
}


bool device_get_oled_initialized(void)
{
    return g_oled_initialized;
}

bool device_get_servo_initialized(void)
{
    return g_servo_initialized;
}

bool device_get_motor_initialized(void)
{
    return g_motor_initialized;
}

errcode_t device_sw4_operation(bool enable)
{
    if (enable) {
        // 启动电机
        if (device_motor_start() == ERRCODE_SUCC) {
            PRINT("[Device] SW4 operation: Motor started successfully\r\n");
            return ERRCODE_SUCC;
        } else {
            PRINT("[Device] SW4 operation: Failed to start motor\r\n");
            return ERRCODE_FAIL;
        }
    } else {
        // 停止电机
        if (device_motor_stop() == ERRCODE_SUCC) {
            PRINT("[Device] SW4 operation: Motor stopped successfully\r\n");
            return ERRCODE_SUCC;
        } else {
            PRINT("[Device] SW4 operation: Failed to stop motor\r\n");
            return ERRCODE_FAIL;
        }
    }
}

