/*
 * 雷达驱动模块 - 桌面人体检测
 */

#ifndef RADAR_H
#define RADAR_H

#include <stdint.h>
#include "errcode.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RADAR_DESK_RANGE_CLOSE      50     
#define RADAR_DESK_RANGE_NEAR       100    
#define RADAR_DESK_RANGE_MEDIUM     200     

#define RADAR_STATUS_CALI_ISO       4      
#define RADAR_STATUS_QUERY_DELAY    8000    

typedef void (*radar_detection_callback_t)(uint32_t lower_boundary, uint32_t upper_boundary, uint8_t is_human_presence);


errcode_t radar_init(void);
errcode_t radar_start_detection_task(void);
errcode_t radar_stop_detection(void);
errcode_t radar_register_detection_callback(radar_detection_callback_t callback);
errcode_t radar_get_detection_status(uint8_t *is_running);

#ifdef __cplusplus
}
#endif

#endif /* RADAR_H */