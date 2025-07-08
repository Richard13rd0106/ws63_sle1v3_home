#ifndef SLE_CLIENT_CMD_H
#define SLE_CLIENT_CMD_H

#include "errcode.h"
#include "sle_common.h"
#include "sle_connection_manager.h"
#include "sle_device_discovery.h"
#include "sle_client.h"

#ifdef __cplusplus
extern "C" {
#endif

// 客户端全局状态和设备列表
#include "common_def.h"
#include "sle_ssap_client.h"

typedef struct {
    sle_addr_t addr;
    uint16_t conn_id;
} connected_device_t;

#define MAX_CONNECTED_DEVICES 8

extern connected_device_t g_connected_devices[MAX_CONNECTED_DEVICES];
extern uint8_t g_connected_device_count;
extern uint16_t g_conn_id;
extern ssapc_find_service_result_t g_find_service_result;

/**
 * @brief 查找设备索引
 *
 * @param addr 目标设备地址
 * @return uint8_t 已连接设备索引, 未找到返回 MAX_CONNECTED_DEVICES
 */
uint8_t find_device_index_by_addr(const sle_addr_t *addr);

/**
 * @brief 初始化 SLE 客户端并启动扫描、连接任务。
 *
 * @return errcode_t ERRCODE_SUCC 成功，其他失败
 */
errcode_t sle_client_cmd_init(void);

/**
 * @brief 通过 SLE 向指定地址发送命令。
 *
 * @param data 数据缓冲
 * @param len 数据长度
 * @param addr 目标设备地址
 * @return errcode_t ERRCODE_SUCC 成功，其他失败
 */
errcode_t sle_client_cmd_send(const uint8_t *data, uint16_t len, const sle_addr_t *addr);

#ifdef __cplusplus
}
#endif

#endif /* SLE_CLIENT_CMD_H */ 