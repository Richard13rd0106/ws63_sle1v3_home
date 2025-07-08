#include "sle_client_cmd.h"
#include "debug_print.h"
#include <string.h>
#include "soc_osal.h"
#include "osal_debug.h"
#include "los_memory.h"
#include "securec.h"

/**
 * @brief 初始化 SLE 客户端任务
 *
 * @return errcode_t ERRCODE_SUCC
 */
errcode_t sle_client_cmd_init(void)
{
    return ERRCODE_SUCC;
}

/**
 * @brief 通过 SLE 向指定地址发送命令。
 *
 * @param data 数据缓冲
 * @param len 数据长度
 * @param addr 目标设备地址
 * @return errcode_t ERRCODE_SUCC 成功，其他失败
 */
errcode_t sle_client_cmd_send(const uint8_t *data, uint16_t len, const sle_addr_t *addr)
{
    // 查找目标设备索引
    uint8_t idx = find_device_index_by_addr(addr);
    if (idx >= g_connected_device_count) {
        PRINT("[SLE Client CMD] target not connected\n");
        return ERRCODE_FAIL;
    }
    // 更新全局连接 ID
    g_conn_id = g_connected_devices[idx].conn_id;

    // 构造写请求参数
    ssapc_write_param_t param = {0};
    param.handle = g_find_service_result.start_hdl;
    param.type = SSAP_PROPERTY_TYPE_VALUE;
    param.data_len = len;
    param.data = osal_vmalloc(param.data_len);
    if (param.data == NULL) {
        PRINT("[SLE Client CMD] malloc fail\n");
        return ERRCODE_MALLOC;
    }
    if (memcpy_s(param.data, param.data_len, data, len) != EOK) {
        osal_vfree(param.data);
        return ERRCODE_MEMCPY;
    }

    // 发送写请求
    errcode_t ret = ssapc_write_req(0, g_conn_id, &param);
    osal_vfree(param.data);
    if (ret != ERRCODE_SUCC) {
        PRINT("[SLE Client CMD] write_req fail: %d\n", ret);
    }
    return ret;
} 