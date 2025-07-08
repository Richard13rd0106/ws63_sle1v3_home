#include "securec.h"
#include "errcode.h"
#include "../inc/radar.h"

#include "cmsis_os2.h"
#include "debug_print.h"
#include "soc_osal.h"
#include "common_def.h"
#include "watchdog.h"
#include "osal_debug.h"
#include "uart.h"
#include <string.h>
#include <stdio.h>

/* 引入雷达服务API */
#include "radar_service.h"

/* WiFi相关头文件 - 为雷达提供稳定的WiFi信道 */
#include "lwip/netifapi.h"
#include "wifi_hotspot.h"
#include "wifi_hotspot_config.h"

/* 任务相关定义  */
#define RADAR_TASK_STACK_SIZE       0x1000      /* 4KB栈空间 */
#define RADAR_TASK_PRIO             24          /* 高优先级 */

/* 雷达检测参数  */
#define RADAR_STATUS_CALI_ISO       4
#define RADAR_DEFAULT_LOOP          8
#define RADAR_DEFAULT_PERIOD        5000        /* 5秒间隔 */
#define RADAR_DEFAULT_DBG_TYPE      0           /* 关闭调试输出 */
#define RADAR_DEFAULT_WAVE          2

/* 重新添加WiFi相关定义 - 为雷达提供稳定的WiFi信道 */
#define WIFI_IFNAME_MAX_SIZE        16
#define WIFI_INIT_WAIT_TIME         5000        /* WiFi初始化等待时间 */
#define WIFI_SOFTAP_DELAY           1000        /* WiFi SoftAP启动延迟 */

/* API检测范围定义  */
#define RADAR_API_NO_HUMAN          0
#define RADAR_API_RANGE_CLOSE       50
#define RADAR_API_RANGE_NEAR        5  // changed to 5cm for detection only within 5cm
#define RADAR_API_RANGE_MEDIUM      200
#define RADAR_API_RANGE_FAR         600
typedef enum {
    DESK_STATUS_SLEEP = 0,
    DESK_STATUS_WAKE,
} desk_status_t;

/* 全局变量 */
static uint8_t g_radar_running = 0;                    /* 雷达运行状态 */
static radar_detection_callback_t g_detection_callback = NULL;  /* 检测回调函数 */
static osal_task *g_radar_task_handle = NULL;          /* 雷达任务句柄 */
static desk_status_t g_current_desk_status = DESK_STATUS_SLEEP; /* 当前桌面状态 */
static uint32_t g_no_human_count = 0;                  /* 无人检测计数器 */

#define NO_HUMAN_RESET_THRESHOLD    1                   /* 连续1次无人检测后重置为SLEEP */

static void radar_process_status_change(radar_result_t *res)
{
    if (res->is_human_presence) {
        /* 重置无人计数 */
        g_no_human_count = 0;
        
        if (res->lower_boundary == 0 && res->upper_boundary == RADAR_API_RANGE_NEAR) {
            if (g_current_desk_status == DESK_STATUS_SLEEP) {
                g_current_desk_status = DESK_STATUS_WAKE;
            }
        }
    } else {
        /* 无人检测 - 增加计数器，达到阈值后重置为SLEEP */
        g_no_human_count++;
        
        if (g_current_desk_status == DESK_STATUS_WAKE && g_no_human_count >= NO_HUMAN_RESET_THRESHOLD) {
            g_current_desk_status = DESK_STATUS_SLEEP;
            g_no_human_count = 0;  /* 重置计数器 */
        }
    }
}

/**
 * @brief WiFi SoftAP初始化
 */
static errcode_t radar_wifi_softap_init(void)
{
    char ssid[WIFI_MAX_SSID_LEN] = "NL_DK63_Radar_SoftAP";
    char pre_shared_key[WIFI_MAX_KEY_LEN] = "radar123";
    softap_config_stru hapd_conf = {0};
    
    char ifname[WIFI_IFNAME_MAX_SIZE] = "ap0";
    struct netif *netif_p = NULL;
    ip4_addr_t st_gw = {0};
    ip4_addr_t st_ipaddr = {0};
    ip4_addr_t st_netmask = {0};
    
    IP4_ADDR(&st_ipaddr, 192, 168, 66, 1);
    IP4_ADDR(&st_netmask, 255, 255, 255, 0);
    IP4_ADDR(&st_gw, 192, 168, 66, 2);
    
    /* 配置SoftAP */
    (void)memcpy_s(hapd_conf.ssid, sizeof(hapd_conf.ssid), ssid, sizeof(ssid));
    (void)memcpy_s(hapd_conf.pre_shared_key, WIFI_MAX_KEY_LEN, pre_shared_key, WIFI_MAX_KEY_LEN);
    hapd_conf.security_type = WIFI_SEC_TYPE_WPA2_WPA_PSK_MIX;
    hapd_conf.channel_num = 6;
    
    if (wifi_softap_enable(&hapd_conf) != ERRCODE_SUCC) {
        return ERRCODE_FAIL;
    }
    
    netif_p = netif_find(ifname);
    if (netif_p == NULL) {
        (void)wifi_softap_disable();
        return ERRCODE_FAIL;
    }
    
    if (netifapi_netif_set_addr(netif_p, &st_ipaddr, &st_netmask, &st_gw) != ERR_OK) {
        (void)wifi_softap_disable();
        return ERRCODE_FAIL;
    }
    
    if (netifapi_dhcps_start(netif_p, NULL, 0) != ERR_OK) {
        (void)wifi_softap_disable();
        return ERRCODE_FAIL;
    }
    
    return ERRCODE_SUCC;
}

/**
 * @brief 雷达检测结果回调函数 
 */
static void radar_result_callback(radar_result_t *res)
{
    if (res == NULL) {
        return;
    }

    /* 处理状态变化 */
    radar_process_status_change(res);

    /* 调用用户回调 */
    if (g_detection_callback != NULL) {
        g_detection_callback(res->lower_boundary, res->upper_boundary, res->is_human_presence);
    }
}

/**
 * @brief 初始化雷达参数
 */
static void radar_init_parameters(void)
{
    /* 雷达调试参数 - 禁用UART输出 */
    radar_dbg_para_t dbg_para;
    dbg_para.times = 0;
    dbg_para.loop = 8;
    dbg_para.ant = 0;
    dbg_para.wave = 2;
    dbg_para.dbg_type = 0;  /* 禁用UART调试输出 */
    dbg_para.period = 5000;
    uapi_radar_set_debug_para(&dbg_para);

    /* 算法选择参数 */
    radar_sel_para_t sel_para;
    sel_para.height = 0;
    sel_para.scenario = 0;
    sel_para.material = 2;
    sel_para.fusion_track = 1;
    sel_para.fusion_ai = 1;
    uapi_radar_select_alg_para(&sel_para);

    /* 算法参数 */
    radar_alg_para_t alg_para;
    alg_para.d_th_1m = 32;
    alg_para.d_th_2m = 27;
    alg_para.p_th = 30;
    alg_para.t_th_1m = 13;
    alg_para.t_th_2m = 26;
    alg_para.b_th_ratio = 50;
    alg_para.b_th_cnt = 15;
    alg_para.a_th = 70;
    uapi_radar_set_alg_para(&alg_para, 0);

    PRINT("[Radar] 雷达参数配置完成 (UART调试已禁用)\r\n");
}

/**
 * @brief 雷达检测任务函数
 */
static int radar_detection_task(const char *arg)
{
    unused(arg);

    PRINT("[Radar] 雷达检测任务启动\r\n");

    /* 等待系统初始化 */
    osal_msleep(WIFI_INIT_WAIT_TIME);

    /* 初始化WiFi SoftAP */
    if (radar_wifi_softap_init() != ERRCODE_SUCC) {
        PRINT("[Radar] WiFi init failed\r\n");
    }

    osal_msleep(WIFI_SOFTAP_DELAY);

    /* 注册回调 */
    errcode_t ret = uapi_radar_register_result_cb(radar_result_callback);
    if (ret != ERRCODE_SUCC) {
        PRINT("[Radar] Callback failed: 0x%x\r\n", ret);
        return -1;
    }

    /* 初始化参数 */
    radar_init_parameters();

    /* 启动校准 */
    ret = uapi_radar_set_status(RADAR_STATUS_CALI_ISO);
    if (ret != ERRCODE_SUCC) {
        PRINT("[Radar] Start failed: 0x%x\r\n", ret);
        return -1;
    }

    PRINT("[Radar] 开始检测 (状态: SLEEP)\r\n");
    
    g_radar_running = 1;
    while (g_radar_running) {
        uapi_watchdog_kick();
        uapi_radar_set_delay_time(8);
        osal_msleep(RADAR_DEFAULT_PERIOD);
    }
    
    PRINT("[Radar] 任务退出\r\n");
    return 0;
}

/**
 * @brief 初始化驱动
 */
errcode_t radar_init(void)
{
    PRINT("[Radar] 驱动初始化\r\n");
    g_radar_running = 0;
    g_detection_callback = NULL;
    g_radar_task_handle = NULL;
    g_current_desk_status = DESK_STATUS_SLEEP;
    g_no_human_count = 0;
    PRINT("[Radar] 驱动初始化完成\r\n");
    return ERRCODE_SUCC;
}

/**
 * @brief 启动检测任务
 */
errcode_t radar_start_detection_task(void)
{
    if (g_radar_running) {
        PRINT("[Radar] 任务已运行\r\n");
        return ERRCODE_SUCC;
    }
    
    osal_kthread_lock();
    g_radar_task_handle = osal_kthread_create((osal_kthread_handler)radar_detection_task, 0,
                                              "RadarTask", RADAR_TASK_STACK_SIZE);
    if (g_radar_task_handle) {
        osal_kthread_set_priority(g_radar_task_handle, RADAR_TASK_PRIO);
        osal_kfree(g_radar_task_handle);
        PRINT("[Radar] 任务创建成功\r\n");
    } else {
        osal_kthread_unlock();
        PRINT("[Radar] 任务创建失败\r\n");
        return ERRCODE_FAIL;
    }
    osal_kthread_unlock();
    return ERRCODE_SUCC;
}

/**
 * @brief 停止检测
 */
errcode_t radar_stop_detection(void)
{
    if (!g_radar_running) {
        return ERRCODE_SUCC;
    }
    
    if (uapi_radar_set_status(RADAR_STATUS_STOP) != ERRCODE_SUCC) {
        PRINT("[Radar] 停止失败\r\n");
    }
    g_radar_running = 0;
    PRINT("[Radar] 检测已停止\r\n");
    return ERRCODE_SUCC;
}

/**
 * @brief 注册检测回调
 */
errcode_t radar_register_detection_callback(radar_detection_callback_t cb)
{
    if (!cb) {
        return ERRCODE_INVALID_PARAM;
    }
    g_detection_callback = cb;
    PRINT("[Radar] 检测回调已注册\r\n");
    return ERRCODE_SUCC;
}

/**
 * @brief 获取运行状态
 */
errcode_t radar_get_detection_status(uint8_t *is_running)
{
    if (!is_running) return ERRCODE_INVALID_PARAM;
    *is_running = g_radar_running;
    return ERRCODE_SUCC;
} 