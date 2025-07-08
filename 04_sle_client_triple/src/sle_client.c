#include "systick.h"
#include "tcxo.h"
#include "los_memory.h"
#include "sle_client.h"
#include "securec.h"
#include "sle_device_discovery.h"
#include "sle_connection_manager.h"
#include "soc_osal.h"
#include "app_init.h"
#include "common_def.h"
#include "sle_ssap_server.h"
#include "debug_print.h"
#include "pinctrl.h"
#include "gpio.h"
#include "cmsis_os2.h"
#include "uart.h"
#include "watchdog.h"
#include <string.h>
#include <stdio.h>
#include "sle_client_cmd.h"

static void sle_uart_control_entry(void);
static void sle_uart_rx_callback(const void *buffer, uint16_t length, bool error);
static int sle_uart_control_task(const char *arg);

#define SLE_SEEK_INTERVAL_DEFAULT 100
#define SLE_SEEK_WINDOW_DEFAULT 100
#define UUID_16BIT_LEN 2
#define UUID_128BIT_LEN 16
#define UART_BAUDRATE           115200
#define UART_DATA_BITS          3 
#define UART_STOP_BITS          1   
#define UART_PARITY_BIT         0 
#define UART_TRANSFER_SIZE      32
#define CONFIG_UART1_TXD_PIN    15
#define CONFIG_UART1_RXD_PIN    16
#define CONFIG_UART1_PIN_MODE   1
#define CONFIG_UART1_BUS_ID     1
#define CONFIG_UART_INT_WAIT_MS 5

#define OCTET_BIT_LEN 8
#define UUID_LEN_2 2

#define DEBOUNCE_DELAY_MS 100

#define LED_CONTROL_TASK_STACK_SIZE 0x1300
#define LED_CONTROL_TASK_PRIO (osPriority_t)(17)

#undef THIS_FILE_ID
#define THIS_FILE_ID BTH_GLE_SAMPLE_UUID_CLIENT

// 默认的 SLE MTU 大小
#define SLE_MTU_SIZE_DEFAULT 512

// 默认的查找间隔
#define SLE_SEEK_INTERVAL_DEFAULT 100

// 默认的查找窗口
#define SLE_SEEK_WINDOW_DEFAULT 100

// 16 位 UUID 的长度
#define UUID_16BIT_LEN 2

// 128 位 UUID 的长度
#define UUID_128BIT_LEN 16

// SLE 速度常量
#define SLE_SPEED_HUNDRED 100 /* 100  */

// 默认的连接间隔
#define SPEED_DEFAULT_CONN_INTERVAL 0x09

// 默认的超时乘数
#define SPEED_DEFAULT_TIMEOUT_MULTIPLIER 0x1f4

// 默认的扫描间隔
#define SPEED_DEFAULT_SCAN_INTERVAL 400

// 默认的扫描窗口
#define SPEED_DEFAULT_SCAN_WINDOW 20

// 全局查找回调结构体
static sle_announce_seek_callbacks_t g_seek_cbk = {0};

// 全局连接回调结构体
static sle_connection_callbacks_t g_connect_cbk = {0};

// 全局 SSAPC 回调结构体
static ssapc_callbacks_t g_ssapc_cbk = {0};

// 全局远程设备地址
static sle_addr_t g_remote_addr = {0};

// 全局连接 ID
uint16_t g_conn_id = 0;

// 全局服务查找结果结构体
ssapc_find_service_result_t g_find_service_result = {0};

connected_device_t g_connected_devices[MAX_CONNECTED_DEVICES];
uint8_t g_connected_device_count = 0;

static uint8_t addr_kt[SLE_ADDR_LEN] = {0x04, 0x01, 0x06, 0x08, 0x06, 0x03};
static uint8_t addr_mj[SLE_ADDR_LEN] = {0x0A, 0x01, 0x06, 0x08, 0x06, 0x03};
static uint8_t addr_ws[SLE_ADDR_LEN] = {0x02, 0x01, 0x06, 0x08, 0x06, 0x03};
int compare_addr(const sle_addr_t *addr1, const sle_addr_t *addr2);

uint8_t find_device_index_by_addr(const sle_addr_t *addr)
{
    for (uint8_t i = 0; i < g_connected_device_count; i++)
    {
        if (compare_addr(&g_connected_devices[i].addr, addr))
        {
            return i; // 找到设备，返回索引
        }
    }
    return MAX_CONNECTED_DEVICES; 
}

void print_addr(sle_addr_t *device_addr, char *prefix)
{
    printf("%s addr: ", prefix);
    for (int i = 0; i < SLE_ADDR_LEN; i++)
    {
        printf("%02X", device_addr->addr[i]);
        if (i < SLE_ADDR_LEN - 1)
        {
            printf(":");
        }
    }
    printf("\r\n");
}

int compare_addr(const sle_addr_t *addr1, const sle_addr_t *addr2)
{
    for (int i = 0; i < SLE_ADDR_LEN; ++i)
    {
        if (addr1->addr[i] != addr2->addr[i])
        {
            return 0; 
        }
    }
    return 1; 
}

/**
 * @brief SLE 启用回调函数。
 *
 * 该函数在 SLE 启用完成时被调用，如果启用成功，则开始扫描操作。
 *
 * @param status 启用操作的状态码。
 */
static void sle_sample_sle_enable_cbk(errcode_t status)
{
    if (status == 0)
    {
        sle_start_scan();
    }
}

/**
 * @brief 查找启用回调函数。
 *
 * 该函数在查找启用完成时被调用，如果启用成功，则返回。
 *
 * @param status 启用操作的状态码。
 */
static void sle_sample_seek_enable_cbk(errcode_t status)
{
    if (status == 0)
    {
        return;
    }
}

/**
 * @brief 查找禁用回调函数。
 *
 * 该函数在查找禁用完成时被调用，如果禁用成功，则连接远程设备。
 *
 * @param status 禁用操作的状态码。
 */
static void sle_sample_seek_disable_cbk(errcode_t status)
{
    if (status == 0)
    {
        sle_connect_remote_device(&g_remote_addr);
    }
}

/**
 * @brief 回调函数，用于处理查找结果信息。
 *
 * 该函数在查找结果信息可用时被调用，并打印设备地址信息。
 * 如果查找结果数据不为空，则会打印连接设备的提示信息，
 * 并将查找结果中的地址复制到全局变量 g_remote_addr 中，然后停止查找。
 *
 * @param seek_result_data 指向查找结果信息的指针。
 */
static void sle_sample_seek_result_info_cbk(sle_seek_result_info_t *seek_result_data)
{
    // 打印查找结果设备的地址信息
    PRINT("[ssap client] sle_sample_seek_result_info_cbk  [%02x,%02x,%02x,%02x,%02x,%02x]\n",
           seek_result_data->addr.addr[0],
           seek_result_data->addr.addr[1],
           seek_result_data->addr.addr[2],
           seek_result_data->addr.addr[3],
           seek_result_data->addr.addr[4],
           seek_result_data->addr.addr[5]);

    // 检查查找结果数据是否不为空
    if (seek_result_data != NULL)
    {
        // 打印连接设备的提示信息
        PRINT("will connect dev\n");

        // 将查找结果中的地址复制到全局变量 g_remote_addr 中
        (void)memcpy_s(&g_remote_addr, sizeof(sle_addr_t), &seek_result_data->addr, sizeof(sle_addr_t));

        // 停止查找
        sle_stop_seek();
    }
}

static void sle_uart_notification_cb(uint8_t client_id, uint16_t conn_id, ssapc_handle_value_t *data,
                                     errcode_t status)
{
    unused(client_id);
    unused(conn_id);
    unused(status);
    unused(data);
}

static void sle_uart_indication_cb(uint8_t client_id, uint16_t conn_id, ssapc_handle_value_t *data,
                                   errcode_t status)
{
    unused(client_id);
    unused(conn_id);
    unused(status);
    osal_printk("\n sle_uart_indication_cb sle uart recived data : %s\r\n", data->data);
}

/**
 * @brief 注册查找相关的回调函数。
 *
 * 该函数将查找相关的回调函数指针赋值给全局回调结构体 g_seek_cbk。
 * 这些回调函数包括 SLE 启用回调、查找启用回调、查找禁用回调和查找结果回调。
 */
static void sle_sample_seek_cbk_register(void)
{
    // 设置 SLE 启用回调函数
    g_seek_cbk.sle_enable_cb = sle_sample_sle_enable_cbk;

    // 设置查找启用回调函数
    g_seek_cbk.seek_enable_cb = sle_sample_seek_enable_cbk;

    // 设置查找禁用回调函数
    g_seek_cbk.seek_disable_cb = sle_sample_seek_disable_cbk;

    // 设置查找结果回调函数
    g_seek_cbk.seek_result_cb = sle_sample_seek_result_info_cbk;
}

/**
 * @brief 回调函数，用于处理连接状态变化。
 *
 * 该函数在连接状态变化时被调用，并根据不同的连接状态执行相应的操作。
 * 当连接状态为已连接时，设置全局连接 ID，并发送信息交换请求。
 * 无论连接状态如何，都会启动查找操作。
 *
 * @param conn_id 连接 ID。
 * @param addr 指向设备地址的指针。
 * @param conn_state 当前连接状态。
 * @param pair_state 当前配对状态。
 * @param disc_reason 断开连接的原因。
 */
static void sle_sample_connect_state_changed_cbk(uint16_t conn_id, const sle_addr_t *addr,
                                                 sle_acb_state_t conn_state, sle_pair_state_t pair_state, sle_disc_reason_t disc_reason)
{
    unused(pair_state);
    PRINT("changed_cbk触发执行");
    // 打印连接状态变化的信息，包括连接 ID 和设备地址
    PRINT("[ssap client] conn state changed conn_id:%d, addr:%02x***%02x%02x\n", conn_id, addr->addr[0],
                          addr->addr[4], addr->addr[5]); /* 0 4 5: addr index */

    // 打印断开连接的原因
    PRINT("[ssap client] conn state changed disc_reason:0x%x\n", disc_reason);

    if (conn_state == SLE_ACB_STATE_CONNECTED)
    {
        for (int i = 0; i < g_connected_device_count; i++)
        {
            if (memcmp(&g_connected_devices[i].addr, addr, sizeof(sle_addr_t)) == 0)
            {
                return; // 设备已在列表中
            }
        }
        if (g_connected_device_count < MAX_CONNECTED_DEVICES)
        {
            g_connected_devices[g_connected_device_count].addr = *addr;
            g_connected_devices[g_connected_device_count].conn_id = conn_id;
            g_connected_device_count++;
            // 启动查找操作
            sle_start_seek();
        }
    }
    else if (conn_state == SLE_ACB_STATE_DISCONNECTED)
    {
        for (int i = 0; i < g_connected_device_count; i++)
        {
            if (memcmp(&g_connected_devices[i].addr, addr, sizeof(sle_addr_t)) == 0)
            {
                for (int j = i; j < g_connected_device_count - 1; j++)
                {
                    g_connected_devices[j] = g_connected_devices[j + 1];
                }
                g_connected_device_count--;
                break;
            }
        }
    }
    sle_uart_control_entry();
    ssap_exchange_info_t info = {0};
    info.mtu_size = SLE_MTU_SIZE_DEFAULT;
    info.version = 1;
    ssapc_exchange_info_req(1, conn_id, &info); /* 此处没有使用默认的client ID 0 */
    g_conn_id = conn_id;                        // 更新连接ID
}

/**
 * @brief 配对完成回调函数。
 *
 * 该函数在配对操作完成时被调用，并打印配对结果信息。
 * 包括连接 ID、设备地址和配对状态。
 *
 * @param conn_id 连接 ID。
 * @param addr 指向设备地址的指针。
 * @param status 配对操作的状态码。
 */
static void sle_sample_pair_complete_cbk(uint16_t conn_id, const sle_addr_t *addr, errcode_t status)
{
    // 打印配对完成的信息，包括连接 ID、设备地址和配对状态
    PRINT("[ssap client] pair complete conn_id:%d, addr:%02x***%02x%02x, %d\n", conn_id, addr->addr[0],
                          addr->addr[4], addr->addr[5], status); 
}

/**
 * @brief 连接参数更新回调函数。
 *
 * 该函数在连接参数更新时被调用，并打印更新后的连接参数信息。
 * 包括连接 ID 和更新后的连接间隔。
 *
 * @param conn_id 连接 ID。
 * @param status 更新操作的状态码。
 * @param param 指向连接参数更新事件结构体的指针。
 */
static void sle_sample_update_cbk(uint16_t conn_id, errcode_t status, const sle_connection_param_update_evt_t *param)
{
    // 未使用的参数 status
    unused(status);

    // 打印连接参数更新的信息，包括连接 ID 和更新后的连接间隔
    PRINT("[ssap client] update state changed conn_id:%d, interval = %02x\n", conn_id, param->interval);
}

/**
 * @brief 连接参数更新请求回调函数。
 *
 * 该函数在接收到连接参数更新请求时被调用，并打印请求的连接参数信息。
 * 包括最小连接间隔和最大连接间隔。
 *
 * @param conn_id 连接 ID。
 * @param status 请求操作的状态码。
 * @param param 指向连接参数更新请求结构体的指针。
 */
static void sle_sample_update_req_cbk(uint16_t conn_id, errcode_t status, const sle_connection_param_update_req_t *param)
{
    // 未使用的参数 conn_id 和 status
    unused(conn_id);
    unused(status);

    // 打印连接参数更新请求的信息，包括最小连接间隔和最大连接间隔
    PRINT("[ssap client] sle_sample_update_req_cbk interval_min = %02x, interval_max = %02x\n",
                          param->interval_min, param->interval_max);
}

/**
 * @brief 注册连接相关的回调函数。
 *
 * 该函数将连接相关的回调函数指针赋值给全局回调结构体 g_connect_cbk。
 * 这些回调函数包括连接状态变化回调、配对完成回调、连接参数更新请求回调和连接参数更新回调。
 */
static void sle_sample_connect_cbk_register(void)
{
    // 设置连接状态变化回调函数
    g_connect_cbk.connect_state_changed_cb = sle_sample_connect_state_changed_cbk;

    // 设置配对完成回调函数
    g_connect_cbk.pair_complete_cb = sle_sample_pair_complete_cbk;

    // 设置连接参数更新请求回调函数
    g_connect_cbk.connect_param_update_req_cb = sle_sample_update_req_cbk;

    // 设置连接参数更新回调函数
    g_connect_cbk.connect_param_update_cb = sle_sample_update_cbk;
}

/**
 * @brief 信息交换回调函数。
 *
 * 该函数在信息交换完成时被调用，并打印交换的 MTU 大小和版本信息。
 * 然后初始化结构查找参数并开始查找主服务。
 *
 * @param client_id 客户端 ID。
 * @param conn_id 连接 ID。
 * @param param 指向信息交换参数结构体的指针。
 * @param status 信息交换操作的状态码。
 */
static void sle_sample_exchange_info_cbk(uint8_t client_id, uint16_t conn_id, ssap_exchange_info_t *param,
                                         errcode_t status)
{
    // 打印配对完成的信息，包括客户端 ID 和状态码
    PRINT("[ssap client] pair complete client id:%d status:%d\n", client_id, status);

    // 打印交换的 MTU 大小和版本信息
    PRINT("[ssap client] exchange mtu, mtu size: %d, version: %d.\n",
                          param->mtu_size, param->version);

    // 初始化结构查找参数
    ssapc_find_structure_param_t find_param = {0};
    find_param.type = SSAP_FIND_TYPE_PRIMARY_SERVICE; // 设置查找类型为主服务
    find_param.start_hdl = 1;                         // 设置查找的起始句柄
    find_param.end_hdl = 0xFFFF;                      // 设置查找的结束句柄

    // 开始查找主服务
    ssapc_find_structure(0, conn_id, &find_param);
}

static void sle_sample_find_structure_cbk(uint8_t client_id, uint16_t conn_id, ssapc_find_service_result_t *service,
                                          errcode_t status)
{
    PRINT("[ssap client] find structure cbk client: %d conn_id:%d status: %d \n",
                          client_id, conn_id, status);
    PRINT("[ssap client] find structure start_hdl:[0x%02x], end_hdl:[0x%02x], uuid len:%d\r\n",
                          service->start_hdl, service->end_hdl, service->uuid.len);
    if (service->uuid.len == UUID_16BIT_LEN)
    {
        PRINT("[ssap client] structure uuid:[0x%02x][0x%02x]\r\n",
                              service->uuid.uuid[14], service->uuid.uuid[15]); /* 14 15: uuid index */
    }
    else
    {
        for (uint8_t idx = 0; idx < UUID_128BIT_LEN; idx++)
        {
            PRINT("[ssap client] structure uuid[%d]:[0x%02x]\r\n", idx, service->uuid.uuid[idx]);
        }
    }
    g_find_service_result.start_hdl = service->start_hdl;
    g_find_service_result.end_hdl = service->end_hdl;
    memcpy_s(&g_find_service_result.uuid, sizeof(sle_uuid_t), &service->uuid, sizeof(sle_uuid_t));
}

static void sle_sample_find_structure_cmp_cbk(uint8_t client_id, uint16_t conn_id,
                                              ssapc_find_structure_result_t *structure_result, errcode_t status)
{
    PRINT("[ssap client] find structure cmp cbk client id:%d status:%d type:%d uuid len:%d \r\n",
                          client_id, status, structure_result->type, structure_result->uuid.len);
    if (structure_result->uuid.len == UUID_16BIT_LEN)
    {
        PRINT("[ssap client] find structure cmp cbk structure uuid:[0x%02x][0x%02x]\r\n",
                              structure_result->uuid.uuid[14], structure_result->uuid.uuid[15]); /* 14 15: uuid index */
    }
    else
    {
        for (uint8_t idx = 0; idx < UUID_128BIT_LEN; idx++)
        {
            PRINT("[ssap client] find structure cmp cbk structure uuid[%d]:[0x%02x]\r\n", idx,
                                  structure_result->uuid.uuid[idx]);
        }
    }
    uint8_t data[] = {0x11, 0x22, 0x33, 0x44};
    uint8_t len = sizeof(data);
    ssapc_write_param_t param = {0};
    param.handle = g_find_service_result.start_hdl;
    param.type = SSAP_PROPERTY_TYPE_VALUE;
    param.data_len = len;
    param.data = data;
    ssapc_write_req(0, conn_id, &param);
}

/**
 * @brief 属性查找回调函数。
 *
 * 该函数在属性查找完成时被调用，并打印查找结果信息。
 * 包括客户端 ID、连接 ID、操作指示、描述符数量和状态码。
 * 如果属性的 UUID 长度为 16 位，则打印 UUID 的最后两个字节。
 * 如果属性的 UUID 长度为 128 位，则打印完整的 UUID。
 *
 * @param client_id 客户端 ID。
 * @param conn_id 连接 ID。
 * @param property 指向属性查找结果结构体的指针。
 * @param status 属性查找操作的状态码。
 */
static void sle_sample_find_property_cbk(uint8_t client_id, uint16_t conn_id,
                                         ssapc_find_property_result_t *property, errcode_t status)
{
    // 打印属性查找回调的信息，包括客户端 ID、连接 ID、操作指示、描述符数量和状态码
    PRINT("[ssap client] find property cbk, client id: %d, conn id: %d, operate ind: %d, "
                          "descriptors count: %d status:%d.\n",
                          client_id, conn_id, property->operate_indication,
                          property->descriptors_count, status);

    // 遍历并打印每个描述符的类型
    for (uint16_t idx = 0; idx < property->descriptors_count; idx++)
    {
        PRINT("[ssap client] find property cbk, descriptors type [%d]: 0x%02x.\n",
                              idx, property->descriptors_type[idx]);
    }

    // 如果 UUID 长度为 16 位，打印 UUID 的最后两个字节
    if (property->uuid.len == UUID_16BIT_LEN)
    {
        PRINT("[ssap client] find property cbk, uuid: %02x %02x.\n",
                              property->uuid.uuid[14], property->uuid.uuid[15]); /* 14 15: uuid index */
    }
    // 如果 UUID 长度为 128 位，打印完整的 UUID
    else if (property->uuid.len == UUID_128BIT_LEN)
    {
        for (uint16_t idx = 0; idx < UUID_128BIT_LEN; idx++)
        {
            PRINT("[ssap client] find property cbk, uuid [%d]: %02x.\n",
                                  idx, property->uuid.uuid[idx]);
        }
    }
}

static void sle_sample_write_cfm_cbk(uint8_t client_id, uint16_t conn_id, ssapc_write_result_t *write_result,
                                     errcode_t status)
{
    PRINT("[ssap client] write cfm cbk, client id: %d status:%d.\n", client_id, status);
    ssapc_read_req(0, conn_id, write_result->handle, write_result->type);
}

static void sle_sample_read_cfm_cbk(uint8_t client_id, uint16_t conn_id, ssapc_handle_value_t *read_data,
                                    errcode_t status)
{
    PRINT("[ssap client] read cfm cbk client id: %d conn id: %d status: %d\n",
                          client_id, conn_id, status);
    PRINT("[ssap client] read cfm cbk handle: %d, type: %d , len: %d\n",
                          read_data->handle, read_data->type, read_data->data_len);
    for (uint16_t idx = 0; idx < read_data->data_len; idx++)
    {
        PRINT("[ssap client] read cfm cbk[%d] 0x%02x\r\n", idx, read_data->data[idx]);
    }
}

/**
 * @brief 注册 SSAPC 相关的回调函数。
 *
 * 该函数将 SSAPC 相关的回调函数指针赋值给全局回调结构体 g_ssapc_cbk。
 * 这些回调函数包括信息交换回调、结构查找回调、结构查找完成回调、
 * 属性查找回调、写确认回调、读确认回调、通知回调和指示回调。
 *
 * @param notification_cb 通知回调函数。
 * @param indication_cb 指示回调函数。
 */
void sle_sample_ssapc_cbk_register(ssapc_notification_callback notification_cb,
                                   ssapc_notification_callback indication_cb)
{
    // 设置信息交换回调函数
    g_ssapc_cbk.exchange_info_cb = sle_sample_exchange_info_cbk;

    // 设置结构查找回调函数
    g_ssapc_cbk.find_structure_cb = sle_sample_find_structure_cbk;

    // 设置结构查找完成回调函数
    g_ssapc_cbk.find_structure_cmp_cb = sle_sample_find_structure_cmp_cbk;

    // 设置属性查找回调函数
    g_ssapc_cbk.ssapc_find_property_cbk = sle_sample_find_property_cbk;

    // 设置写确认回调函数
    g_ssapc_cbk.write_cfm_cb = sle_sample_write_cfm_cbk;

    // 设置读确认回调函数
    g_ssapc_cbk.read_cfm_cb = sle_sample_read_cfm_cbk;

    // 设置通知回调函数
    g_ssapc_cbk.notification_cb = notification_cb;

    // 设置指示回调函数
    g_ssapc_cbk.indication_cb = indication_cb;
}

/**
 * @brief 初始化快速连接参数。
 *
 * 该函数用于初始化快速连接参数，并将其设置为默认连接参数。
 * 连接参数包括过滤策略、GT 协商、物理层、最大和最小连接间隔、扫描间隔、扫描窗口和超时。
 */
static void sle_speed_connect_param_init(void)
{
    // 定义并初始化默认连接参数结构体
    sle_default_connect_param_t param = {0};
    param.enable_filter_policy = 0;                    // 设置过滤策略
    param.gt_negotiate = 0;                            // 设置 GT 协商
    param.initiate_phys = 1;                           // 设置物理层
    param.max_interval = SPEED_DEFAULT_CONN_INTERVAL;  // 设置最大连接间隔
    param.min_interval = SPEED_DEFAULT_CONN_INTERVAL;  // 设置最小连接间隔
    param.scan_interval = SPEED_DEFAULT_SCAN_INTERVAL; // 设置扫描间隔
    param.scan_window = SPEED_DEFAULT_SCAN_WINDOW;     // 设置扫描窗口
    param.timeout = SPEED_DEFAULT_TIMEOUT_MULTIPLIER;  // 设置超时

    // 设置默认连接参数
    sle_default_connection_param_set(&param);
}
/**
 * @brief 初始化 SLE 客户端。
 *
 * 该函数用于初始化 SLE 客户端，包括设置本地地址、注册各种回调函数以及启用 SLE 功能。
 *
 * @param notification_cb 通知回调函数。
 * @param indication_cb 指示回调函数。
 */
void sle_client_init(ssapc_notification_callback notification_cb, ssapc_indication_callback indication_cb)
{
    // 定义并初始化本地地址
    uint8_t local_addr[SLE_ADDR_LEN] = {0x13, 0x67, 0x5c, 0x07, 0x00, 0x51};
    sle_addr_t local_address;
    local_address.type = 0; // 设置地址类型为 0

    // 将本地地址复制到 sle_addr_t 结构体中
    (void)memcpy_s(local_address.addr, SLE_ADDR_LEN, local_addr, SLE_ADDR_LEN);

    // 初始化快速连接参数
    sle_speed_connect_param_init();

    // 注册查找回调函数
    sle_sample_seek_cbk_register();

    // 注册连接回调函数
    sle_sample_connect_cbk_register();

    // 注册 SSAPC 回调函数
    sle_sample_ssapc_cbk_register(notification_cb, indication_cb);

    // 注册SLE announce seek回调函数，如果失败则打印错误信息并返回-1
    if (sle_announce_seek_register_callbacks(&g_seek_cbk) != ERRCODE_SUCC)
    {
        PRINT("[SLE Client] sle announce seek register callbacks fail !\r\n");
        // return -1;
    }

    // 注册SLE连接回调函数，如果失败则打印错误信息并返回-1
    if (sle_connection_register_callbacks(&g_connect_cbk) != ERRCODE_SUCC)
    {
        PRINT("[SLE Client] sle connection register callbacks fail !\r\n");
        // return -1;
    }

    // 注册SSAPC回调函数，如果失败则打印错误信息并返回-1
    if (ssapc_register_callbacks(&g_ssapc_cbk) != ERRCODE_SUCC)
    {
        PRINT("[SLE Client] sle ssapc register callbacks fail !\r\n");
        // return -1;
    }

    // 启用SLE功能，如果失败则打印错误信息并返回-1
    if (enable_sle() != ERRCODE_SUCC)
    {
        PRINT("[SLE Client] sle enable fail !\r\n");
        // return -1;
    }

    // 设置本地地址
    sle_set_local_addr(&local_address);
}

/**
 * @brief 开始扫描操作。
 *
 * 该函数用于初始化扫描参数并启动扫描操作。
 * 扫描参数包括本地地址类型、重复过滤、扫描过滤策略、扫描物理层、扫描类型、扫描间隔和扫描窗口。
 * 初始化完成后，调用 sle_start_seek() 函数开始扫描。
 */
void sle_start_scan()
{
    // 定义并初始化扫描参数结构体
    sle_seek_param_t param = {0};
    param.own_addr_type = 0;                            // 设置本地地址类型
    param.filter_duplicates = 0;                        // 设置是否过滤重复扫描结果
    param.seek_filter_policy = 0;                       // 设置扫描过滤策略
    param.seek_phys = 1;                                // 设置扫描物理层
    param.seek_type[0] = 0;                             // 设置扫描类型
    param.seek_interval[0] = SLE_SEEK_INTERVAL_DEFAULT; // 设置扫描间隔
    param.seek_window[0] = SLE_SEEK_WINDOW_DEFAULT;     // 设置扫描窗口

    // 设置扫描参数
    sle_set_seek_param(&param);

    // 开始扫描
    sle_start_seek();
}

static int sle_speed_init(void)
{
    osal_msleep(1000);

    sle_client_init(sle_uart_notification_cb, sle_uart_indication_cb);
    return 0;
}

#define SLE_SPEED_TASK_PRIO 26
#define SLE_SPEED_STACK_SIZE 0x4000
static void sle_speed_entry(void)
{
    osal_task *task_handle = NULL;
    osal_kthread_lock();
    task_handle = osal_kthread_create((osal_kthread_handler)sle_speed_init, 0, "SLE_Task", SLE_SPEED_STACK_SIZE);
    if (task_handle != NULL)
    {
        osal_kthread_set_priority(task_handle, SLE_SPEED_TASK_PRIO);
        osal_kfree(task_handle);
    }
    osal_kthread_unlock();
}

#define UART_CMD_BUFFER_SIZE 16
static uint8_t g_uart_rx_buffer[UART_CMD_BUFFER_SIZE] = {0};
static uint8_t g_uart_rx_flag = 0;
static uart_buffer_config_t g_uart_buffer_config = {
    .rx_buffer = g_uart_rx_buffer,
    .rx_buffer_size = UART_CMD_BUFFER_SIZE
};

static void sle_uart_rx_callback(const void *buffer, uint16_t length, bool error)
{
    if (error || buffer == NULL || length == 0) {
        return;
    }
    uint16_t copy_len = (length < UART_CMD_BUFFER_SIZE) ? length : (UART_CMD_BUFFER_SIZE - 1);
    memcpy(g_uart_rx_buffer, buffer, copy_len);

    while (copy_len > 0 && (g_uart_rx_buffer[copy_len - 1] == '\r' || g_uart_rx_buffer[copy_len - 1] == '\n')) {
        copy_len--;
    }
    g_uart_rx_buffer[copy_len] = '\0';
    g_uart_rx_flag = 1;
}

#define UART_CONTROL_TASK_STACK_SIZE 0x1000
#define UART_CONTROL_TASK_PRIO (osPriority_t)(17)
static int sle_uart_control_task(const char *arg)
{
    unused(arg);

    uapi_pin_set_mode(CONFIG_UART1_TXD_PIN, PIN_MODE_1);
    uapi_pin_set_mode(CONFIG_UART1_RXD_PIN, PIN_MODE_1);
    uart_attr_t attr = {
        .baud_rate = UART_BAUDRATE,
        .data_bits = UART_DATA_BITS,
        .stop_bits = UART_STOP_BITS,
        .parity = UART_PARITY_BIT
    };
    uart_pin_config_t pin_config = {
        .tx_pin = CONFIG_UART1_TXD_PIN,
        .rx_pin = CONFIG_UART1_RXD_PIN,
        .cts_pin = PIN_NONE,
        .rts_pin = PIN_NONE
    };
    (void)uapi_uart_deinit(CONFIG_UART1_BUS_ID);
    (void)uapi_uart_init(CONFIG_UART1_BUS_ID, &pin_config, &attr, NULL, &g_uart_buffer_config);
    (void)uapi_uart_register_rx_callback(CONFIG_UART1_BUS_ID,
                                         UART_RX_CONDITION_FULL_OR_SUFFICIENT_DATA_OR_IDLE,
                                         UART_CMD_BUFFER_SIZE,
                                         sle_uart_rx_callback);
    while (1) {
        (void)uapi_watchdog_kick();
        if (g_uart_rx_flag) {
            g_uart_rx_flag = 0;
            char *uart_cmd = (char *)g_uart_rx_buffer;

            const char *prefixes[] = {"DS","PD","CL","KT","ZM","MS","MJ","CH","DG","MD"};
            int sw_id = -1;
            
            printf("[SLE Client] UART received command: '%s' (length: %d)\r\n", uart_cmd, strlen(uart_cmd));
            
            for (int i = 0; i < 10; i++) {  
                if (uart_cmd[0] == prefixes[i][0] && uart_cmd[1] == prefixes[i][1]) {
                    if (i < 8) {
                        sw_id = i + 1;  // DS->1 ... CH->8
                    } else if (i == 8) {
                        sw_id = 9;      // DG->9 
                    } else {
                        sw_id = 10;     // MD->10
                    }
                    printf("[SLE Client] Matched prefix '%s' -> sw_id = %d\r\n", prefixes[i], sw_id);
                    break;
                }
            }
            
            if (sw_id > 0) {
                printf("[SLE Client] Command format check: cmd[2]='%c', cmd[3]='%c', cmd[4]='%c'\r\n", 
                       uart_cmd[2], uart_cmd[3], uart_cmd[4]);
            }
            
            if (sw_id > 0 && uart_cmd[2] == '_' && (uart_cmd[3] == '1' || uart_cmd[3] == '0') && uart_cmd[4] == '\0') {
                const char *state = (uart_cmd[3] == '1') ? "ON" : "OFF";
                char sle_cmd[16] = {0}; 
                
                int len = snprintf(sle_cmd, sizeof(sle_cmd), "SW%d_%s", sw_id, state);
                if (len < 0 || (size_t)len >= sizeof(sle_cmd)) {
                    printf("[SLE Client] Error: Command formatting failed\r\n");
                    continue;
                }
                
                sle_addr_t temp_addr;
                const char *target_device = "";
                
                // 根据sw_id选择目标设备地址
                if (sw_id >= 1 && sw_id <= 6) {
                    // SW1-6: 发送到servo设备
                    memcpy(temp_addr.addr, addr_kt, SLE_ADDR_LEN);
                    target_device = "servo";
                } else if (sw_id == 7) {
                    // SW7: 发送到ac设备
                    memcpy(temp_addr.addr, addr_mj, SLE_ADDR_LEN);
                    target_device = "ac";
                } else {
                    // SW8-10: 发送到cz设备
                    memcpy(temp_addr.addr, addr_ws, SLE_ADDR_LEN);
                    target_device = "cz";
                }
                
                // 打印详细的调试信息
                printf("[SLE Client] UART '%s' -> SLE '%s' (len:%d) -> %s device\r\n", 
                       uart_cmd, sle_cmd, len, target_device);
                printf("[SLE Client] Target address: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                       temp_addr.addr[0], temp_addr.addr[1], temp_addr.addr[2],
                       temp_addr.addr[3], temp_addr.addr[4], temp_addr.addr[5]);
                
                // 通过封装的驱动接口发送
                errcode_t ret = sle_client_cmd_send((uint8_t *)sle_cmd, len, &temp_addr);
                if (ret != ERRCODE_SUCC) {
                    printf("[SLE Client] Error: Failed to send command, ret=0x%x\r\n", ret);
                } else {
                    printf("[SLE Client] Command sent successfully\r\n");
                }
            } else {
                if (sw_id <= 0) {
                    printf("[SLE Client] Error: Invalid command prefix\r\n");
                } else {
                    printf("[SLE Client] Error: Invalid command format\r\n");
                }
            }
        }
        osal_msleep(CONFIG_UART_INT_WAIT_MS);
    }
    return 0;
}

static void sle_uart_control_entry(void)
{
    osal_task *task_handle = NULL;
    osal_kthread_lock();
    task_handle = osal_kthread_create((osal_kthread_handler)sle_uart_control_task,
                                      0,
                                      "UARTTask",
                                      UART_CONTROL_TASK_STACK_SIZE);
    if (task_handle != NULL) {
        osal_kthread_set_priority(task_handle, UART_CONTROL_TASK_PRIO);
        osal_kfree(task_handle);
    }
    osal_kthread_unlock();
}

// 启动 SLE 速度任务
app_run(sle_speed_entry);
// 启动 UART 控制任务，处理串口命令输入
app_run(sle_uart_control_entry);