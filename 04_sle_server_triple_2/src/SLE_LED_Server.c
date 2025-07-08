#include "securec.h"                   // 安全C库，提供安全的字符串和内存操作函数
#include "errcode.h"                   // 错误码定义，提供错误码的定义和处理函数
#include "osal_addr.h"                 // OS抽象层地址管理，提供地址相关的操作函数
#include "sle_common.h"                // SLE通用定义，包含SLE协议的通用数据结构和宏定义
#include "sle_errcode.h"               // SLE错误码定义，提供SLE协议的错误码定义和处理函数
#include "sle_ssap_server.h"           // SLE SSAP服务端，提供SSAP服务端的实现和接口
#include "sle_connection_manager.h"    // SLE连接管理，提供连接管理的实现和接口
#include "sle_device_discovery.h"      // SLE设备发现，提供设备发现的实现和接口
#include "../inc/SLE_LED_Server_adv.h" // SLE LED Server 广播，提供LED Server的广播相关函数
#include "../inc/SLE_LED_Server.h"     // SLE LED Server，提供LED Server的主要功能实现

#include "cmsis_os2.h"   // CMSIS-RTOS v2 API，提供RTOS的API接口
#include "debug_print.h" // 调试打印，提供调试信息的打印函数
#include "soc_osal.h"    // SOC OS抽象层，提供SOC相关的操作函数
#include "app_init.h"    // 应用初始化，提供应用初始化相关的函数
#include "common_def.h"  // 通用定义，包含通用的数据结构和宏定义
#include "i2c.h"         // I2C接口，提供I2C通信的实现和接口
#include "osal_debug.h"  // OS抽象层调试，提供调试相关的操作函数
#include "cmsis_os2.h"   // CMSIS-RTOS v2 API，提供RTOS的API接口（重复包含）
#include "watchdog.h"

#include "pinctrl.h" // 引脚控制相关的头文件
#include "gpio.h"    // GPIO操作相关的头文件
#include "std_def.h"

#include "tcxo.h"
#include "systick.h"
#include "../inc/radar.h"

typedef enum { DOOR_CLOSED = 0, DOOR_OPEN_CMD, DOOR_OPEN_RADAR } door_state_t;
static volatile door_state_t g_door_state = DOOR_CLOSED;

static uint8_t g_last_human_presence = 0;

static void sle_radar_detection_cb(uint32_t lower_boundary, uint32_t upper_boundary, uint8_t is_human_presence);

static void S92RInit(void);
static void RegressMiddle(void);
static void EngineTurnLeft(void);

#define OCTET_BIT_LEN 8 // 八位字节长度
#define UUID_LEN_2 2    // UUID 长度为 2 字节

/**
 * @brief 将数据编码为小端格式的 2 字节
 * @param _ptr 指向目标存储位置的指针
 * @param data 要编码的数据
 */
#define encode2byte_little(_ptr, data)                     \
    do                                                     \
    {                                                      \
        *(uint8_t *)((_ptr) + 1) = (uint8_t)((data) >> 8); \
        *(uint8_t *)(_ptr) = (uint8_t)(data);              \
    } while (0)

/* SLE 服务应用 UUID 示例 */
static char g_sle_uuid_app_uuid[UUID_LEN_2] = {0x0, 0x0};
/* 服务属性值示例 */
static char g_sle_property_value[OCTET_BIT_LEN] = {0x1, 0x2, 0x3, 0x4, 0x5, 0x6};
/* SLE 连接 ID */
static uint16_t g_conn_id = 0;
/* SLE 服务 ID */
static uint8_t g_server_id = 0;
/* SLE 服务句柄 */
static uint16_t g_service_handle = 0;
/* SLE 通知属性句柄 */
static uint16_t g_property_handle = 0;

/**
 * @brief 通过句柄发送通知示例
 * @param data 要发送的数据
 * @param len 数据长度
 * @return 错误码
 */
// static errcode_t example_sle_server_send_notify_by_handle(const uint8_t *data, uint8_t len);

// 头部大小数组
const unsigned char headSize[] = {64, 64}; // 定义两个头部大小，均为 64 字节

// SLE UUID基准数组
static uint8_t sle_uuid_base[] = {0x37, 0xBE, 0xA8, 0x80, 0xFC, 0x70, 0x11, 0xEA,
                                  0xB7, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // UUID基准值，用于标识SLE服务

/* 设置 UUID 基础部分的函数 */
static void example_sle_uuid_set_base(sle_uuid_t *out)
{
    // 使用安全的 memcpy 函数将 sle_uuid_base 的内容复制到 out->uuid 中
    // (void) 用于显式忽略 memcpy_s 的返回值
    (void)memcpy_s(out->uuid, SLE_UUID_LEN, sle_uuid_base, SLE_UUID_LEN);

    // 设置 UUID 的长度为 2 字节
    out->len = UUID_LEN_2;
}

/* 设置 UUID 的函数 */
static void example_sle_uuid_setu2(uint16_t u2, sle_uuid_t *out)
{
    // 调用函数设置 UUID 基础部分
    example_sle_uuid_set_base(out);

    // 设置 UUID 长度为 2 字节
    out->len = UUID_LEN_2;

    // 将 2 字节的 u2 编码为小端格式，并存储在 UUID 的第 14 和 15 字节位置
    encode2byte_little(&out->uuid[14], u2);
}

// 读取请求的回调函数
static void
example_ssaps_read_request_cbk(uint8_t server_id,
                               uint16_t conn_id,
                               ssaps_req_read_cb_t *read_cb_para,
                               errcode_t status)
{
    // 打印读取请求的详细信息，包括服务器ID、连接ID、句柄、类型和状态
    PRINT("[SLE Server] ssaps read request cbk server_id:0x%x, conn_id:0x%x, handle:0x%x, type:0x%x, status:0x%x\r\n",
          server_id, conn_id, read_cb_para->handle, read_cb_para->type, status);
}

// 写入请求的回调函数
static void example_ssaps_write_request_cbk(uint8_t server_id,
                                            uint16_t conn_id,
                                            ssaps_req_write_cb_t *write_cb_para,
                                            errcode_t status)
{
    // 打印写入请求的详细信息，包括服务器ID、连接ID、句柄和状态
    PRINT("[SLE Server] ssaps write request cbk server_id:0x%x, conn_id:0x%x, handle:0x%x, status:0x%x\r\n", server_id,
          conn_id, write_cb_para->handle, status);

    // 打印写入请求的数据，每个字节的索引和值
    for (uint16_t idx = 0; idx < write_cb_para->length; idx++)
    {
        PRINT("[SLE Server] write request cbk[0x%x] 0x%02x\r\n", idx, write_cb_para->value[idx]);
    }

    if (status == ERRCODE_SUCC)
    {
        char cmd_buf[16] = {0};
        uint16_t len = write_cb_para->length < sizeof(cmd_buf)-1 ? write_cb_para->length : sizeof(cmd_buf)-1;
        memcpy_s(cmd_buf, sizeof(cmd_buf), write_cb_para->value, len);
        cmd_buf[len] = '\0';

        if (strcmp(cmd_buf, "SW7_ON") == 0) {
            if (g_door_state == DOOR_CLOSED) {
                S92RInit();
                EngineTurnLeft();
                g_door_state = DOOR_OPEN_CMD;
                PRINT("[SLE Server] received SW7_ON, door opened by command\r\n");
            } else {
                PRINT("[SLE Server] SW7_ON ignored, door already open (state=%d)\r\n", g_door_state);
            }
        } else if (strcmp(cmd_buf, "SW7_OFF") == 0) {
            if (g_door_state == DOOR_OPEN_CMD) {
                S92RInit();
                RegressMiddle();
                g_door_state = DOOR_CLOSED;
                PRINT("[SLE Server] received SW7_OFF, door closed by command\r\n");
            } else {
                PRINT("[SLE Server] SW7_OFF ignored, door not opened by command (state=%d)\r\n", g_door_state);
            }
        } else {
            PRINT("[SLE Server] unknown command: %s\r\n", cmd_buf);
        }
    }
    else
    {
        PRINT("[SLE Server] write request failed, error code: 0x%x\r\n", status);
    }
}

static void example_ssaps_mtu_changed_cbk(uint8_t server_id,
                                          uint16_t conn_id,
                                          ssap_exchange_info_t *mtu_size,
                                          errcode_t status)
{
    // 打印MTU改变的详细信息，包括服务器ID、连接ID、MTU大小和状态
    PRINT("[SLE Server] ssaps mtu changed cbk server_id:0x%x, conn_id:0x%x, mtu_size:0x%x, status:0x%x\r\n", server_id,
          conn_id, mtu_size->mtu_size, status);
}

// 服务启动的回调函数
static void example_ssaps_start_service_cbk(uint8_t server_id, uint16_t handle, errcode_t status)
{
    // 打印服务启动的详细信息，包括服务器ID、句柄和状态
    PRINT("[SLE Server] start service cbk server_id:0x%x, handle:0x%x, status:0x%x\r\n", server_id, handle, status);
}

// 注册所有的SSAPS回调函数
static errcode_t example_sle_ssaps_register_cbks(void)
{
    ssaps_callbacks_t ssaps_cbk = {0};                            // 初始化回调函数结构体
    ssaps_cbk.start_service_cb = example_ssaps_start_service_cbk; // 注册服务启动回调函数
    ssaps_cbk.mtu_changed_cb = example_ssaps_mtu_changed_cbk;     // 注册MTU改变回调函数
    ssaps_cbk.read_request_cb = example_ssaps_read_request_cbk;   // 注册读取请求回调函数
    ssaps_cbk.write_request_cb = example_ssaps_write_request_cbk; // 注册写入请求回调函数
    return ssaps_register_callbacks(&ssaps_cbk);                  // 注册所有回调函数并返回状态
}

/* 添加 SLE 服务 */
static errcode_t example_sle_server_service_add(void)
{
    sle_uuid_t service_uuid = {0}; // 初始化服务 UUID

    // 设置服务 UUID
    example_sle_uuid_setu2(SLE_UUID_SERVER_SERVICE, &service_uuid);

    // 同步添加服务
    ssaps_add_service_sync(g_server_id, &service_uuid, true, &g_service_handle);

    // 打印调试信息，表示服务添加成功
    PRINT("[SLE Server] sle uuid add service service_handle: %u\r\n", g_service_handle);

    return ERRCODE_SUCC; // 返回成功错误码
}

static errcode_t example_sle_server_property_add(void)
{
    ssaps_property_info_t property = {0}; // 初始化属性信息结构体
    ssaps_desc_info_t descriptor = {0};   // 初始化描述符信息结构体
    uint8_t ntf_value[] = {0x01, 0x0};    // 通知值

    // 设置属性的权限为可读和可写
    property.permissions = SSAP_PERMISSION_READ | SSAP_PERMISSION_WRITE;
    // 设置属性的 UUID
    example_sle_uuid_setu2(SLE_UUID_SERVER_PROPERTY, &property.uuid);
    // 为属性值分配内存
    osal_vmalloc(sizeof(g_sle_property_value));

    // 将全局属性值复制到属性的值中
    memcpy_s(property.value, sizeof(g_sle_property_value), g_sle_property_value, sizeof(g_sle_property_value));

    // 同步添加属性
    ssaps_add_property_sync(g_server_id, g_service_handle, &property, &g_property_handle);
    // 打印调试信息，表示属性添加成功
    PRINT("[SLE Server] sle uuid add property property_handle: %u\r\n", g_property_handle);

    // 设置描述符的权限为可读和可写
    descriptor.permissions = SSAP_PERMISSION_READ | SSAP_PERMISSION_WRITE;
    // 为描述符值分配内存
    osal_vmalloc(sizeof(ntf_value));

    // 将通知值复制到描述符的值中
    memcpy_s(descriptor.value, sizeof(ntf_value), ntf_value, sizeof(ntf_value));

    // 同步添加描述符
    ssaps_add_descriptor_sync(g_server_id, g_service_handle, g_property_handle, &descriptor);

    // 释放已分配的属性和描述符内存
    osal_vfree(property.value);
    osal_vfree(descriptor.value);

    return ERRCODE_SUCC; // 返回成功错误码
}

/* 添加 SLE 服务器 */
static errcode_t example_sle_server_add(void)
{
    /*
    UUID（Universally Unique Identifier，通用唯一标识符）是一种用于标识信息的标准。UUID 的目的是在分布式系统中使得所有元素都能有一个唯一的标识符，而不需要中央协调。
    0.app uuid
    1.服务（Service）uuid
    2.特性（Characteristic）uuid
    3.属性（Property）uuid
    */

    sle_uuid_t app_uuid = {0}; // 初始化应用 UUID

    PRINT("[SLE Server] sle uuid add service in\r\n"); // 打印调试信息，表示开始添加服务
    app_uuid.len = sizeof(g_sle_uuid_app_uuid);        // 设置 UUID 长度
    memcpy_s(app_uuid.uuid, app_uuid.len, g_sle_uuid_app_uuid, sizeof(g_sle_uuid_app_uuid));

    // 注册服务器
    ssaps_register_server(&app_uuid, &g_server_id);
    // 添加服务
    example_sle_server_service_add();
    // 添加属性
    example_sle_server_property_add();
    // 启动服务
    ssaps_start_service(g_server_id, g_service_handle);
    // 打印调试信息，表示服务添加完成
    PRINT("[SLE Server] sle uuid add service out\r\n");
    return ERRCODE_SUCC; // 返回成功错误码
}
typedef void (*sle_connect_state_changed_callback)(uint16_t conn_id, const sle_addr_t *addr,
                                                   sle_acb_state_t conn_state, sle_pair_state_t pair_state, sle_disc_reason_t disc_reason);

// 连接状态改变的回调函数
static void example_sle_connect_state_changed_cbk(uint16_t conn_id,
                                                  const sle_addr_t *addr,
                                                  sle_acb_state_t conn_state,
                                                  sle_pair_state_t pair_state,
                                                  sle_disc_reason_t disc_reason)
{
    // 打印连接状态改变的详细信息，包括连接ID、连接状态、配对状态和断开原因
    PRINT(
        "[SLE Server] connect state changed conn_id:0x%02x, conn_state:0x%x, pair_state:0x%x, \
        disc_reason:0x%x\r\n",
        conn_id, conn_state, pair_state, disc_reason);
    // 打印连接设备的地址
    PRINT("[SLE Server] connect state changed addr:%02x:**:**:**:%02x:%02x\r\n", addr->addr[0], addr->addr[4],
          addr->addr[5]);
    g_conn_id = conn_id; // 更新全局连接ID
}

// 配对完成的回调函数
static void example_sle_pair_complete_cbk(uint16_t conn_id, const sle_addr_t *addr, errcode_t status)
{
    // 打印配对完成的详细信息，包括连接ID和状态
    PRINT("[SLE Server] pair complete conn_id:0x%02x, status:0x%x\r\n", conn_id, status);
    // 打印配对设备的地址
    PRINT("[SLE Server] pair complete addr:%02x:**:**:**:%02x:%02x\r\n", addr->addr[0], addr->addr[4], addr->addr[5]);
}

// 注册连接相关的回调函数/
static errcode_t example_sle_conn_register_cbks(void)
{
    // 初始化连接回调函数结构体
    sle_connection_callbacks_t conn_cbks = {0};
    // 注册连接状态改变回调函数
    conn_cbks.connect_state_changed_cb = example_sle_connect_state_changed_cbk;
    // 注册配对完成回调函数
    conn_cbks.pair_complete_cb = example_sle_pair_complete_cbk;
    // 注册所有连接回调函数并返回状态

    return sle_connection_register_callbacks(&conn_cbks);
}

static int example_sle_led_server_task(const char *arg)
{
    unused(arg);

    (void)osal_msleep(4999); /* 延时5s，等待SLE初始化完毕  */
    PRINT("[SLE Server] try enable.\r\n");
    /* 使能SLE */
    enable_sle();

    /*注册连接管理回调函数 */
    example_sle_conn_register_cbks();

    /* 注册 SSAP server 回调函数 */
    example_sle_ssaps_register_cbks();

    /* 注册Server, 添加Service和property, 启动Service */
    example_sle_server_add();

    /* 设置设备公开，并公开设备 */
    example_sle_server_adv_init();

    PRINT("[SLE Server] init ok\r\n");
    /* 初始化雷达模块 */
    radar_init();
    radar_register_detection_callback(sle_radar_detection_cb);
    radar_start_detection_task();

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

    if (task_handle != NULL)
    {

        osal_kthread_set_priority(task_handle, SLE_LED_SER_TASK_PRIO);
        osal_kfree(task_handle);
    }
    osal_kthread_unlock(); 
}

app_run(example_sle_led_server_entry);

#define SG92R_PIN 3
#define SG92R_COUNT 10
#define SG92R_FREQ_TIME 20000

static void S92RInit(void)
{
    uapi_pin_set_mode(SG92R_PIN, HAL_PIO_FUNC_GPIO);
    uapi_gpio_set_dir(SG92R_PIN, GPIO_DIRECTION_OUTPUT);
    uapi_gpio_set_val(SG92R_PIN, GPIO_LEVEL_LOW);
}

static void SetAngle(unsigned int duty)
{
    uapi_gpio_set_val(SG92R_PIN, GPIO_LEVEL_HIGH);
    uapi_systick_delay_us(duty);
    uapi_gpio_set_val(SG92R_PIN, GPIO_LEVEL_LOW);
    uapi_systick_delay_us(SG92R_FREQ_TIME - duty);
}

static void RegressMiddle(void)
{
    for (int i = 0; i < SG92R_COUNT; i++) {
        SetAngle(500);
    }
}

static void EngineTurnLeft(void)
{
    for (int i = 0; i < SG92R_COUNT; i++) {
        SetAngle(1500);
    }
}

static void sle_radar_detection_cb(uint32_t lower_boundary, uint32_t upper_boundary, uint8_t is_human_presence)
{
    (void)upper_boundary;
    
    if (g_last_human_presence != is_human_presence) {
        if (is_human_presence) {
            PRINT("[SLE Server] 雷达检测到人体 (距离: %u cm)\r\n", lower_boundary);
        } else {
            PRINT("[SLE Server] 雷达检测到无人\r\n");
        }
        g_last_human_presence = is_human_presence;
    }
    
    if (is_human_presence && lower_boundary <= RADAR_DESK_RANGE_NEAR) {
        if (g_door_state == DOOR_CLOSED) {
            S92RInit();
            EngineTurnLeft(); 
            g_door_state = DOOR_OPEN_RADAR;
        }
    } else {
        if (!is_human_presence && g_door_state == DOOR_OPEN_RADAR) {
            S92RInit();
            RegressMiddle();
            g_door_state = DOOR_CLOSED;
        }
    }
}
