#include "securec.h"                   // 安全C库，用于安全的内存操作
#include "errcode.h"                   // 错误码定义
#include "osal_addr.h"                 // OS抽象层地址操作
#include "sle_common.h"                // SLE通用定义
#include "sle_device_discovery.h"      // SLE设备发现功能
#include "sle_errcode.h"               // SLE错误码定义
#include "../inc/SLE_LED_Server_adv.h" // SLE LED服务器广播头文件

#include "cmsis_os2.h"   // CMSIS-RTOS API
#include "debug_print.h" // 调试打印功能

/* sle device name */
#define NAME_MAX_LENGTH 16 // 设备名称的最大长度
/* 连接调度间隔12.5ms，单位125us */
#define SLE_CONN_INTV_MIN_DEFAULT 0x64 // 最小连接间隔默认值
/* 连接调度间隔12.5ms，单位125us */
#define SLE_CONN_INTV_MAX_DEFAULT 0x64 // 最大连接间隔默认值
/* 连接调度间隔25ms，单位125us */
#define SLE_ADV_INTERVAL_MIN_DEFAULT 0xC8 // 最小广播间隔默认值
/* 连接调度间隔25ms，单位125us */
#define SLE_ADV_INTERVAL_MAX_DEFAULT 0xC8 // 最大广播间隔默认值
/* 超时时间5000ms，单位10ms */
#define SLE_CONN_SUPERVISION_TIMEOUT_DEFAULT 0x1F4 // 连接监督超时默认值
/* 超时时间4990ms，单位10ms */
#define SLE_CONN_MAX_LATENCY 0x1F3 // 最大连接延迟默认值
/* 广播发送功率 */
#define SLE_ADV_TX_POWER 10 // 广播发送功率
/* 广播ID */
#define SLE_ADV_HANDLE_DEFAULT 1 // 广播句柄默认值
/* 最大广播数据长度 */
#define SLE_ADV_DATA_LEN_MAX 251 // 最大广播数据长度
/* 广播名称 */
static uint8_t sle_local_name[NAME_MAX_LENGTH] = {'A', 'i', 'r', 'c', 'o', 'n', ' ', 'C', 'o', 'n', 't', 'r', 'o', 'l', '\0'}; // 本地设备名称
// 设置广播本地名称
static uint16_t example_sle_set_adv_local_name(uint8_t *adv_data, uint16_t max_len)
{
    errno_t ret = -1;  // 错误码初始化为-1
    uint8_t index = 0; // 广播数据索引初始化为0

    uint8_t *local_name = sle_local_name;                         // 获取本地设备名称
    uint8_t local_name_len = (uint8_t)strlen((char *)local_name); // 获取本地设备名称长度
    for (uint8_t i = 0; i < local_name_len; i++)
    {
        PRINT("[SLE Adv] local_name[%d] = 0x%02x\r\n", i, local_name[i]); // 打印本地设备名称的每个字符
    }

    adv_data[index++] = local_name_len + 1;                                        // 设置广播数据长度
    adv_data[index++] = SLE_ADV_DATA_TYPE_COMPLETE_LOCAL_NAME;                     // 设置广播数据类型为完整本地名称
    ret = memcpy_s(&adv_data[index], max_len - index, local_name, local_name_len); // 复制本地名称到广播数据中
    if (ret != EOK)
    {                                       // 检查复制是否成功
        PRINT("[SLE Adv] memcpy fail\r\n"); // 打印复制失败信息
        return 0;                           // 返回0表示失败
    }
    return (uint16_t)index + local_name_len; // 返回广播数据的总长度
}

// 设置广播数据
static uint16_t example_sle_set_adv_data(uint8_t *adv_data)
{
    size_t len = 0;   // 数据长度初始化为0
    uint16_t idx = 0; // 广播数据索引初始化为0
    errno_t ret = 0;  // 错误码初始化为0

    len = sizeof(struct sle_adv_common_value); // 获取通用广播数据结构体的长度
    struct sle_adv_common_value adv_disc_level = {
        .length = len - 1,                         // 设置数据长度
        .type = SLE_ADV_DATA_TYPE_DISCOVERY_LEVEL, // 设置数据类型为发现级别
        .value = SLE_ANNOUNCE_LEVEL_NORMAL,        // 设置数据值为正常级别
    };
    ret = memcpy_s(&adv_data[idx], SLE_ADV_DATA_LEN_MAX - idx, &adv_disc_level, len); // 复制发现级别数据到广播数据中
    if (ret != EOK)
    {                                                      // 检查复制是否成功
        PRINT("[SLE Adv] adv_disc_level memcpy fail\r\n"); // 打印复制失败信息
        return 0;                                          // 返回0表示失败
    }
    idx += len; // 更新广播数据索引

    len = sizeof(struct sle_adv_common_value); // 获取通用广播数据结构体的长度
    struct sle_adv_common_value adv_access_mode = {
        .length = len - 1,                     // 设置数据长度
        .type = SLE_ADV_DATA_TYPE_ACCESS_MODE, // 设置数据类型为访问模式
        .value = 0,                            // 设置数据值为0
    };
    ret = memcpy_s(&adv_data[idx], SLE_ADV_DATA_LEN_MAX - idx, &adv_access_mode, len); // 复制访问模式数据到广播数据中
    if (ret != EOK)
    {                                       // 检查复制是否成功
        PRINT("[SLE Adv] memcpy fail\r\n"); // 打印复制失败信息
        return 0;                           // 返回0表示失败
    }
    idx += len; // 更新广播数据索引
    return idx; // 返回广播数据的总长度
}

// 设置扫描响应数据
static uint16_t example_sle_set_scan_response_data(uint8_t *scan_rsp_data)
{
    uint16_t idx = 0;                                               // 扫描响应数据索引初始化为0
    errno_t ret = -1;                                               // 错误码初始化为-1
    size_t scan_rsp_data_len = sizeof(struct sle_adv_common_value); // 获取通用广播数据结构体的长度

    struct sle_adv_common_value tx_power_level = {
        .length = scan_rsp_data_len - 1,          // 设置数据长度
        .type = SLE_ADV_DATA_TYPE_TX_POWER_LEVEL, // 设置数据类型为发送功率级别
        .value = SLE_ADV_TX_POWER,                // 设置数据值为广播发送功率
    };
    ret = memcpy_s(scan_rsp_data, SLE_ADV_DATA_LEN_MAX, &tx_power_level, scan_rsp_data_len); // 复制发送功率级别数据到扫描响应数据中
    if (ret != EOK)
    {                                                              // 检查复制是否成功
        PRINT("[SLE Adv] sle scan response data memcpy fail\r\n"); // 打印复制失败信息
        return 0;                                                  // 返回0表示失败
    }
    idx += scan_rsp_data_len; // 更新扫描响应数据索引

    /* set local name */
    idx += example_sle_set_adv_local_name(&scan_rsp_data[idx], SLE_ADV_DATA_LEN_MAX - idx); // 设置本地名称到扫描响应数据中
    return idx;                                                                             // 返回扫描响应数据的总长度
}

static uint8_t g_sle_local_addr[SLE_ADDR_LEN] = {0x0A, 0x01, 0x06, 0x08, 0x06, 0x03};
// 设置本地设备地址
static void example_sle_set_addr(void)
{
    uint8_t *addr = g_sle_local_addr; // 获取本地设备地址

    sle_addr_t sle_addr = {0}; // 初始化SLE地址结构体
    sle_addr.type = 0;         // 设置地址类型为0
    if (memcpy_s(sle_addr.addr, SLE_ADDR_LEN, addr, SLE_ADDR_LEN) != EOK)
    {                                             // 复制本地设备地址到SLE地址结构体中
        PRINT("[SLE Adv] addr memcpy fail \r\n"); // 打印复制失败信息
    }

    if (sle_set_local_addr(&sle_addr) == ERRCODE_SUCC)
    {                                              // 设置本地设备地址并检查是否成功
        PRINT("[SLE Adv] set sle addr SUCC \r\n"); // 打印设置成功信息
    }
}

static uint8_t g_local_device_name[] = {'s', 'l', 'e', '_', 'l', 'e', 'd', '_', 's', 'e', 'r', 'v', 'e', 'r'};    

// 设置本地设备名称
static void example_sle_set_name(void)
{
    errcode_t ret = ERRCODE_SUCC;                                               // 错误码初始化为成功
    ret = sle_set_local_name(g_local_device_name, sizeof(g_local_device_name)); // 设置本地设备名称并获取返回码
    if (ret != ERRCODE_SUCC)
    {                                                            // 检查设置是否成功
        PRINT("[SLE Adv] set local name fail, ret:%x\r\n", ret); // 打印设置失败信息
    }
}

// 设置默认的广播参数
static errcode_t example_sle_set_default_announce_param(void)
{
    sle_announce_param_t param = {0};                                      // 初始化广播参数结构体
    param.announce_mode = SLE_ANNOUNCE_MODE_CONNECTABLE_SCANABLE;          // 设置广播模式为可连接可扫描
    param.announce_handle = SLE_ADV_HANDLE_DEFAULT;                        // 设置广播句柄为默认值
    param.announce_gt_role = SLE_ANNOUNCE_ROLE_T_CAN_NEGO;                 // 设置广播角色为可协商
    param.announce_level = SLE_ANNOUNCE_LEVEL_NORMAL;                      // 设置广播级别为正常
    param.announce_channel_map = SLE_ADV_CHANNEL_MAP_DEFAULT;              // 设置广播信道映射为默认值
    param.announce_interval_min = SLE_ADV_INTERVAL_MIN_DEFAULT;            // 设置最小广播间隔为默认值
    param.announce_interval_max = SLE_ADV_INTERVAL_MAX_DEFAULT;            // 设置最大广播间隔为默认值
    param.conn_interval_min = SLE_CONN_INTV_MIN_DEFAULT;                   // 设置最小连接间隔为默认值
    param.conn_interval_max = SLE_CONN_INTV_MAX_DEFAULT;                   // 设置最大连接间隔为默认值
    param.conn_max_latency = SLE_CONN_MAX_LATENCY;                         // 设置最大连接延迟为默认值
    param.conn_supervision_timeout = SLE_CONN_SUPERVISION_TIMEOUT_DEFAULT; // 设置连接监督超时为默认值

    if (memcpy_s(param.own_addr.addr, SLE_ADDR_LEN, g_sle_local_addr, SLE_ADDR_LEN) != EOK)
    {                                                              // 复制本地设备地址到广播参数中
        PRINT("[SLE Adv] set sle adv param addr memcpy fail\r\n"); // 打印复制失败信息
        return ERRCODE_MEMCPY;                                     // 返回内存复制错误码
    }

    return sle_set_announce_param(param.announce_handle, &param); // 设置广播参数并返回状态码
}

// 设置默认的广播数据
static errcode_t example_sle_set_default_announce_data(void)
{
    errcode_t ret = ERRCODE_FAIL;                      // 错误码初始化为失败
    uint8_t announce_data_len = 0;                     // 广播数据长度初始化为0
    uint8_t seek_data_len = 0;                         // 扫描响应数据长度初始化为0
    sle_announce_data_t data = {0};                    // 初始化广播数据结构体
    uint8_t adv_handle = SLE_ADV_HANDLE_DEFAULT;       // 广播句柄初始化为默认值
    uint8_t announce_data[SLE_ADV_DATA_LEN_MAX] = {0}; // 初始化广播数据数组
    uint8_t seek_rsp_data[SLE_ADV_DATA_LEN_MAX] = {0}; // 初始化扫描响应数据数组

    PRINT("[SLE Adv] set adv data default\r\n");                 // 打印设置默认广播数据信息
    announce_data_len = example_sle_set_adv_data(announce_data); // 设置广播数据并获取长度
    data.announce_data = announce_data;                          // 设置广播数据
    data.announce_data_len = announce_data_len;                  // 设置广播数据长度

    seek_data_len = example_sle_set_scan_response_data(seek_rsp_data); // 设置扫描响应数据并获取长度
    data.seek_rsp_data = seek_rsp_data;                                // 设置扫描响应数据
    data.seek_rsp_data_len = seek_data_len;                            // 设置扫描响应数据长度

    ret = sle_set_announce_data(adv_handle, &data); // 设置广播数据并获取返回码
    if (ret == ERRCODE_SUCC)
    {                                                  // 检查设置是否成功
        PRINT("[SLE Adv] set announce data success."); // 打印设置成功信息
    }
    else
    {
        PRINT("[SLE Adv] set adv param fail."); // 打印设置失败信息
    }
    return ERRCODE_SUCC; // 返回成功码
}

// 广播启用回调函数
void example_sle_announce_enable_cbk(uint32_t announce_id, errcode_t status)
{
    PRINT("[SLE Adv] sle announce enable id:%02x, state:%02x\r\n", announce_id, status); // 打印广播启用信息
}

// 广播禁用回调函数
void example_sle_announce_disable_cbk(uint32_t announce_id, errcode_t status)
{
    PRINT("[SLE Adv] sle announce disable id:%02x, state:%02x\r\n", announce_id, status); // 打印广播禁用信息
}

// 广播终止回调函数
void example_sle_announce_terminal_cbk(uint32_t announce_id)
{
    PRINT("[SLE Adv] sle announce terminal id:%02x\r\n", announce_id); // 打印广播终止信息
}

// SLE启用回调函数
void example_sle_enable_cbk(errcode_t status)
{
    PRINT("[SLE Adv] sle enable status:%02x\r\n", status); // 打印SLE启用状态信息
}

// 注册广播相关的回调函数
// 注册SLE广播相关的回调函数
void example_sle_announce_register_cbks(void)
{
    sle_announce_seek_callbacks_t seek_cbks = {0}; // 初始化广播回调函数结构体

    // 注册广播启用回调函数
    seek_cbks.announce_enable_cb = example_sle_announce_enable_cbk;

    // 注册广播禁用回调函数
    seek_cbks.announce_disable_cb = example_sle_announce_disable_cbk;

    // 注册广播终止回调函数
    seek_cbks.announce_terminal_cb = example_sle_announce_terminal_cbk;

    // 注册SLE启用回调函数
    seek_cbks.sle_enable_cb = example_sle_enable_cbk;

    // 注册所有广播回调函数
    sle_announce_seek_register_callbacks(&seek_cbks);
}

// 初始化SLE服务器广播的函数
errcode_t example_sle_server_adv_init(void)
{
    PRINT("[SLE Adv] example_sle_server_adv_init in\r\n"); // 打印初始化开始信息

    example_sle_announce_register_cbks(); // 注册广播相关的回调函数

    example_sle_set_default_announce_param(); // 设置默认的广播参数

    example_sle_set_default_announce_data(); // 设置默认的广播数据

    example_sle_set_addr(); // 设置设备地址

    example_sle_set_name(); // 设置设备名称

    sle_start_announce(SLE_ADV_HANDLE_DEFAULT); // 启动广播

    PRINT("[SLE Adv] example_sle_server_adv_init out\r\n"); // 打印初始化结束信息

    return ERRCODE_SUCC; // 返回成功码
}