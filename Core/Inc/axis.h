/*
 * axis.h
 *
 *  Created on: Mar 27, 2026
 *      Author: miyab
 */

#ifndef INC_AXIS_H_
#define INC_AXIS_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

//#define AXIS_COUNT 3

//#define AXIS_CONTROL_PERIOD_S   0.001f
//#define AXIS_CONTROL_FREQ_HZ    (1.0f / AXIS_CONTROL_PERIOD_S)

typedef struct {
    int32_t target_step;
    float current_rate_step_s;
    float max_rate_step_s;
    float max_acc_step_s2;
    bool active;
    bool busy;
} AxisControlState_t;

/* 初期化 */
HAL_StatusTypeDef axis_init(void);

/* モータ空間での移動目標設定 */
HAL_StatusTypeDef axis_set_motion_target(uint8_t axis,
                                         int32_t target_step,
                                         float max_rate_step_s,
                                         float max_acc_step_s2);

/* 停止要求 */
HAL_StatusTypeDef axis_stop(uint8_t axis);

/* 周期更新 */
HAL_StatusTypeDef axis_update(uint8_t axis);
HAL_StatusTypeDef axis_update_all(void);

/* 状態取得 */
HAL_StatusTypeDef axis_is_busy(uint8_t axis, bool *busy);
HAL_StatusTypeDef axis_get_target_step(uint8_t axis, int32_t *target_step);

#endif
