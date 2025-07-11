#include "pinctrl.h"
#include "soc_osal.h"
#include "app_init.h"
#include <stdint.h>
#include "gpio.h"
#include "securec.h"
#include "errcode.h"
#include "osal_addr.h"
#include "sle_common.h"
#include "sle_errcode.h"
#include "sle_ssap_server.h"
#include "sle_connection_manager.h"
#include "sle_device_discovery.h"
#include "../inc/SLE_LED_Server_adv.h"
#include "../inc/SLE_LED_Server.h"
#include "cmsis_os2.h"
#include "debug_print.h"
#include "soc_osal.h"
#include "app_init.h"
#include "common_def.h"
#include "sle_ssap_client.h"
#include "watchdog.h"
#include "systick.h"
#include <string.h>
#include "../inc/device.h"
#define GPIO5_MODE 0
#define BSP_LED 7 
#define OCTET_BIT_LEN 8
#define UUID_LEN_2 2
#define encode2byte_little(_ptr, data)                     \
    do                                                     \
    {                                                      \
        *(uint8_t *)((_ptr) + 1) = (uint8_t)((data) >> 8); \
        *(uint8_t *)(_ptr) = (uint8_t)(data);              \
    } while (0)

/* sle server app uuid for sample */
static char g_sle_uuid_app_uuid[UUID_LEN_2] = {0x0, 0x0};
/* server property value for sample */
static char g_sle_property_value[OCTET_BIT_LEN] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
/* sle connect id */
static uint16_t g_conn_id = 1;
/* sle server id */
static uint8_t g_server_id = 0;
/* sle service handle */
static uint16_t g_service_handle = 0;
/* sle ntf property handle */
static uint16_t g_property_handle = 0;

#define LED_CONTROL_TASK_STACK_SIZE 0x3000
#define LED_CONTROL_TASK_PRIO (osPriority_t)(17)

errcode_t device_servo_turn_clockwise_180(void);
errcode_t device_servo_turn_counterclockwise_180(void);

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

static void example_ssaps_write_request_cbk(uint8_t server_id,
                                            uint16_t conn_id,
                                            ssaps_req_write_cb_t *write_cb_para,
                                            errcode_t status)
{
    PRINT("[SLE Server] ssaps write request cbk server_id:0x%x, conn_id:0x%x, handle:0x%x, status:0x%x\r\n", server_id,
          conn_id, write_cb_para->handle, status);

    for (uint16_t idx = 0; idx < write_cb_para->length; idx++)
    {
        PRINT("[SLE Server] write request cbk[0x%x] 0x%02x\r\n", idx, write_cb_para->value[idx]);
    }

    if (status == ERRCODE_SUCC)
    {
        PRINT("[SLE Server] received SW command: %.*s\r\n", write_cb_para->length, write_cb_para->value);
        char cmd_buf[16] = {0};
        uint16_t len = write_cb_para->length < sizeof(cmd_buf)-1 ? write_cb_para->length : sizeof(cmd_buf)-1;
        memcpy_s(cmd_buf, sizeof(cmd_buf), write_cb_para->value, len);
        cmd_buf[len] = '\0';
        if (strcmp(cmd_buf, "SW1_ON") == 0) { device_oled_show_hello(); }
        else if (strcmp(cmd_buf, "SW1_OFF") == 0) { device_oled_turn_off(); }
        else if (strcmp(cmd_buf, "SW2_ON") == 0) { device_oled_show_tunnel1(); }
        else if (strcmp(cmd_buf, "SW2_OFF") == 0) { device_oled_show_tunnel2(); }
        else if (strcmp(cmd_buf, "SW3_ON") == 0) {
            device_servo_turn_clockwise_180();
            PRINT("[SLE Server] Curtain opened\r\n");
        }
        else if (strcmp(cmd_buf, "SW3_OFF") == 0) {
            device_servo_turn_counterclockwise_180();
            PRINT("[SLE Server] Curtain closed\r\n");
        }
        else if (strcmp(cmd_buf, "SW4_ON") == 0) { device_sw4_operation(true); }
        else if (strcmp(cmd_buf, "SW4_OFF") == 0) { device_sw4_operation(false); }
        else if (strcmp(cmd_buf, "SW5_ON") == 0) { device_sw5_operation(true); }
        else if (strcmp(cmd_buf, "SW5_OFF") == 0) { device_sw5_operation(false); }
        else if (strcmp(cmd_buf, "SW6_ON") == 0) { device_sw6_operation(true); }
        else if (strcmp(cmd_buf, "SW6_OFF") == 0) { device_sw6_operation(false); }
        else { PRINT("[SLE Server] unknown SW command\r\n"); }
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
    if (ret != ERRCODE_SUCC)
    {
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
    if (property.value == NULL)
    {
        PRINT("[SLE Server] sle property mem fail\r\n");
        return ERRCODE_MALLOC;
    }

    if (memcpy_s(property.value, sizeof(g_sle_property_value), g_sle_property_value, sizeof(g_sle_property_value)) !=
        EOK)
    {
        osal_vfree(property.value);
        PRINT("[SLE Server] sle property mem cpy fail\r\n");
        return ERRCODE_MEMCPY;
    }
    ret = ssaps_add_property_sync(g_server_id, g_service_handle, &property, &g_property_handle);
    if (ret != ERRCODE_SUCC)
    {
        PRINT("[SLE Server] sle uuid add property fail, ret:0x%x\r\n", ret);
        osal_vfree(property.value);
        return ERRCODE_FAIL;
    }

    PRINT("[SLE Server] sle uuid add property property_handle: %u\r\n", g_property_handle);

    descriptor.permissions = SSAP_PERMISSION_READ | SSAP_PERMISSION_WRITE;
    descriptor.value = osal_vmalloc(sizeof(ntf_value));
    if (descriptor.value == NULL)
    {
        PRINT("[SLE Server] sle descriptor mem fail\r\n");
        osal_vfree(property.value);
        return ERRCODE_MALLOC;
    }
    if (memcpy_s(descriptor.value, sizeof(ntf_value), ntf_value, sizeof(ntf_value)) != EOK)
    {
        PRINT("[SLE Server] sle descriptor mem cpy fail\r\n");
        osal_vfree(property.value);
        osal_vfree(descriptor.value);
        return ERRCODE_MEMCPY;
    }
    ret = ssaps_add_descriptor_sync(g_server_id, g_service_handle, g_property_handle, &descriptor);
    if (ret != ERRCODE_SUCC)
    {
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
    if (memcpy_s(app_uuid.uuid, app_uuid.len, g_sle_uuid_app_uuid, sizeof(g_sle_uuid_app_uuid)) != EOK)
    {
        return ERRCODE_MEMCPY;
    }
    ssaps_register_server(&app_uuid, &g_server_id);

    if (example_sle_server_service_add() != ERRCODE_SUCC)
    {
        ssaps_unregister_server(g_server_id);
        return ERRCODE_FAIL;
    }

    if (example_sle_server_property_add() != ERRCODE_SUCC)
    {
        ssaps_unregister_server(g_server_id);
        return ERRCODE_FAIL;
    }
    PRINT("[SLE Server] sle uuid add service, server_id:0x%x, service_handle:0x%x, property_handle:0x%x\r\n",
          g_server_id, g_service_handle, g_property_handle);
    ret = ssaps_start_service(g_server_id, g_service_handle);
    if (ret != ERRCODE_SUCC)
    {
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

    if (status == ERRCODE_SUCC)
    {
        // example_led_control_entry();
        printf("success");
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

    (void)osal_msleep(5000); /* 延时5s，等待SLE初始化完毕 */

    PRINT("[SLE Server] try enable.\r\n");
    /* 使能SLE */
    if (enable_sle() != ERRCODE_SUCC)
    {
        PRINT("[SLE Server] sle enbale fail !\r\n");
        return -1;
    }

    /* 注册连接管理回调函数 */
    if (example_sle_conn_register_cbks() != ERRCODE_SUCC)
    {
        PRINT("[SLE Server] sle conn register cbks fail !\r\n");
        return -1;
    }

    /* 注册 SSAP server 回调函数 */
    if (example_sle_ssaps_register_cbks() != ERRCODE_SUCC)
    {
        PRINT("[SLE Server] sle ssaps register cbks fail !\r\n");
        return -1;
    }

    /* 注册Server, 添加Service和property, 启动Service */
    if (example_sle_server_add() != ERRCODE_SUCC)
    {
        PRINT("[SLE Server] sle server add fail !\r\n");
        return -1;
    }

    /* 设置设备公开，并公开设备 */
    if (example_sle_server_adv_init() != ERRCODE_SUCC)
    {
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
    if (task_handle != NULL)
    {
        osal_kthread_set_priority(task_handle, SLE_LED_SER_TASK_PRIO);
        osal_kfree(task_handle);
    }
    osal_kthread_unlock();
}

app_run(example_sle_led_server_entry);