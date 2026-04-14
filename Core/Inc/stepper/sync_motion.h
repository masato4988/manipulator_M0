/*
 * joint.h
 *
 *  Created on: Mar 30, 2026
 *      Author: miyab
 */

#ifndef INC_SYNC_MOTION_H_
#define INC_SYNC_MOTION_H_

#include "main.h"
#include <stdbool.h>

typedef struct {
    float q1_target_rad;
    float q2_target_rad;
    float q3_target_rad;

    float dq_max_rad_s;
    float ddq_max_rad_s2;

    bool active;
} AppSyncMotionState_t;

HAL_StatusTypeDef sync_motion_init(void);

/* 同期移動開始 */
HAL_StatusTypeDef sync_motion_move_joint_target(
		float q1_target_rad,
		float q2_target_rad,
		float q3_target_rad,
		float dq_max_rad_s,
		float ddq_max_rad_s2);

//HAL_StatusTypeDef app_move_joint_target_sync_time(
//		float q1_target_rad,
//		float q2_target_rad,
//		float q3_target_rad,
//		float dq_max_rad_s,
//		float ddq_max_rad_s2,
//		float target_time_s);

/* 周期更新 */
//HAL_StatusTypeDef app_update(uint32_t now_ms);
HAL_StatusTypeDef sync_motion_update(void);
HAL_StatusTypeDef sync_motion_stop_all(void);

#endif /* INC_SYNC_MOTION_H_ */
