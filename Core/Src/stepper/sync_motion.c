/*
K_M1_TO_Q1 = 2π / 200 / 16 * (1/10)
           = 0.000196349540849362

K_M2_TO_Q2 = 2π / 200 / 16 * (40/160) * (16/64)
           = 0.000122718463030815

K_M3_TO_Q3 = 2π / 200 / 16 * (40/160) * (16/45)
           = 0.000174532925199433

K_M2_TO_Q3 = 2π / 200 / 16            * (16/45) * ((40/160) * (16/64))
           = 2π / 200 / 16            * (16/45) * (1/4) * (1/4)
           = 2π / 200 / 16            * (1/45)
           = 0.0000426332312998582

q1 = m1 * K_M1_TO_Q1
m1 = q1 / K_M1_TO_Q1

q2 = m2 * K_M2_TO_Q2
m2 = q2 / K_M2_TO_Q2

q3 = (m3 * K_M3_TO_Q3) - (m2 * K_M2_TO_Q3)
(m3 * K_M3_TO_Q3) = (m2 * K_M2_TO_Q3) + q3
m3 = (q3 + (m2 * K_M2_TO_Q3)) / K_M3_TO_Q3
*/
#include <stepper/homing_control.h>
#include "stepper/axis.h"
#include "stepper/stepper_hw.h"
#include "math_utils.h"
#include <math.h>
#include <stdbool.h>
#include <stepper/sync_motion.h>

/* --- 関節→モータ変換係数 [rad/step] --- */
#define K_M1_TO_Q1   (0.000196349540849362f)
#define K_M2_TO_Q2   (0.000122718463030815f)
#define K_M3_TO_Q3   (0.000174532925199433f)
#define K_M2_TO_Q3   (0.0000426332312998582f)

typedef enum {
    HOME_SEQ_IDLE = 0,
    HOME_SEQ_AXIS1,
    HOME_SEQ_AXIS2,
    HOME_SEQ_AXIS3,
    HOME_SEQ_DONE,
    HOME_SEQ_ERROR
} HomeSeqState_t;

//static HomeSeqState_t home_seq_state = HOME_SEQ_IDLE;

static AppSyncMotionState_t AppSyncMotionState;

/* ---------- 内部補助 ---------- */

static HAL_StatusTypeDef app_joint_to_motor_target_step(float q1_rad,
                                                        float q2_rad,
                                                        float q3_rad,
                                                        int32_t *m1_step,
                                                        int32_t *m2_step,
                                                        int32_t *m3_step)
{
    float m3_f;

    if (m1_step == NULL || m2_step == NULL || m3_step == NULL)
        return HAL_ERROR;

    *m1_step = (int32_t)lroundf(q1_rad / K_M1_TO_Q1);
    *m2_step = (int32_t)lroundf(q2_rad / K_M2_TO_Q2);

    m3_f = (q3_rad + ((float)(*m2_step) * K_M2_TO_Q3)) / K_M3_TO_Q3;
    *m3_step = (int32_t)lroundf(m3_f);

    return HAL_OK;
}

static HAL_StatusTypeDef app_joint_rate_to_motor_rate(float dq_max_rad_s,
                                                      float ddq_max_rad_s2,
                                                      float *m1_rate,
                                                      float *m2_rate,
                                                      float *m3_rate,
                                                      float *m1_acc,
                                                      float *m2_acc,
                                                      float *m3_acc)
{
    float dm2_rate;
    float dm2_acc;

    if (m1_rate == NULL || m2_rate == NULL || m3_rate == NULL ||
        m1_acc == NULL || m2_acc == NULL || m3_acc == NULL)
        return HAL_ERROR;

    *m1_rate = fabsf(dq_max_rad_s / K_M1_TO_Q1);
    *m2_rate = fabsf(dq_max_rad_s / K_M2_TO_Q2);

    dm2_rate = *m2_rate;
    *m3_rate = fabsf((fabsf(dq_max_rad_s) + fabsf(K_M2_TO_Q3) * dm2_rate) / K_M3_TO_Q3);

    *m1_acc = fabsf(ddq_max_rad_s2 / K_M1_TO_Q1);
    *m2_acc = fabsf(ddq_max_rad_s2 / K_M2_TO_Q2);

    dm2_acc = *m2_acc;
    *m3_acc = fabsf((fabsf(ddq_max_rad_s2) + fabsf(K_M2_TO_Q3) * dm2_acc) / K_M3_TO_Q3);

    return HAL_OK;
}
//static float app_calc_min_time_1axis(float distance_step,
//                                     float vmax_step_s,
//                                     float amax_step_s2)
//{
//    float d_acc;
//
//    if (distance_step <= 0.0f) return 0.0f;
//    if (vmax_step_s <= 0.0f) return 0.0f;
//    if (amax_step_s2 <= 0.0f) return 0.0f;
//
//    d_acc = (vmax_step_s * vmax_step_s) / amax_step_s2;
//
//    if (distance_step <= d_acc) {
//        /* 三角速度 */
//        return 2.0f * sqrtf(distance_step / amax_step_s2);
//    } else {
//        /* 台形速度 */
//        return 2.0f * (vmax_step_s / amax_step_s2)
//             + (distance_step - d_acc) / vmax_step_s;
//    }
//}
/* ---------- 公開関数 ---------- */

HAL_StatusTypeDef sync_motion_init(void)
{
//	if(stepper_init() != HAL_OK)return HAL_ERROR;
//	if(axis_home_init() != HAL_OK)return HAL_ERROR;
    AppSyncMotionState.q1_target_rad = 0.0f;
    AppSyncMotionState.q2_target_rad = 0.0f;
    AppSyncMotionState.q3_target_rad = 0.0f;
    AppSyncMotionState.dq_max_rad_s = 0.0f;
    AppSyncMotionState.ddq_max_rad_s2 = 0.0f;
    AppSyncMotionState.active = false;

    return HAL_OK;
}

HAL_StatusTypeDef sync_motion_move_joint_target(float q1_target_rad,
                                             float q2_target_rad,
                                             float q3_target_rad,
                                             float dq_max_rad_s,
                                             float ddq_max_rad_s2)
{
    if (dq_max_rad_s <= 0.0f) return HAL_ERROR;
    if (ddq_max_rad_s2 <= 0.0f) return HAL_ERROR;

    AppSyncMotionState.q1_target_rad = q1_target_rad;
    AppSyncMotionState.q2_target_rad = q2_target_rad;
    AppSyncMotionState.q3_target_rad = q3_target_rad;
    AppSyncMotionState.dq_max_rad_s = dq_max_rad_s;
    AppSyncMotionState.ddq_max_rad_s2 = ddq_max_rad_s2;
    AppSyncMotionState.active = true;

    return HAL_OK;
}


//HAL_StatusTypeDef app_update(uint32_t now_ms)
HAL_StatusTypeDef sync_motion_update(void)
{
    int32_t target_m1, target_m2, target_m3;
    int32_t current_m1, current_m2, current_m3;
    int32_t delta_m1, delta_m2, delta_m3;
    int32_t max_delta;

    float base_m1_rate, base_m2_rate, base_m3_rate;
    float base_m1_acc,  base_m2_acc,  base_m3_acc;

    float sync_m1_rate, sync_m2_rate, sync_m3_rate;
    float sync_m1_acc,  sync_m2_acc,  sync_m3_acc;

//    bool done;
////
//	axis_home_update(now_ms);
//
//	switch (home_seq_state) {
//	case HOME_SEQ_IDLE:
//		break;
//
//	case HOME_SEQ_AXIS1:
//		axis_home_is_done(1, &done);
//		if (done) {
//			home_seq_state = HOME_SEQ_AXIS2;
//			axis_home_start(2);
//		}
//		break;
//
//	case HOME_SEQ_AXIS2:
//		axis_home_is_done(2, &done);
//		if (done) {
//			home_seq_state = HOME_SEQ_AXIS3;
//			axis_home_start(3);
//		}
//		break;
//
//	case HOME_SEQ_AXIS3:
//		axis_home_is_done(3, &done);
//		if (done) {
//			home_seq_state = HOME_SEQ_DONE;
//		}
//		break;
//
//	case HOME_SEQ_DONE:
//		break;
//
//	case HOME_SEQ_ERROR:
//		break;
//
//	default:
//		home_seq_state = HOME_SEQ_ERROR;
//		break;
//	}

//nomal_operation

    if (AppSyncMotionState.active == false) {
        return HAL_OK;
    }

    /* 最終目標位置をモータ空間へ変換 */
    if (app_joint_to_motor_target_step(AppSyncMotionState.q1_target_rad,
                                       AppSyncMotionState.q2_target_rad,
                                       AppSyncMotionState.q3_target_rad,
                                       &target_m1, &target_m2, &target_m3) != HAL_OK) {
        return HAL_ERROR;
    }

    /* 上位の共通制約をモータ空間へ変換 */
    if (app_joint_rate_to_motor_rate(AppSyncMotionState.dq_max_rad_s,
                                     AppSyncMotionState.ddq_max_rad_s2,
                                     &base_m1_rate, &base_m2_rate, &base_m3_rate,
                                     &base_m1_acc,  &base_m2_acc,  &base_m3_acc) != HAL_OK) {
        return HAL_ERROR;
    }

    /* 現在位置取得 */
    if (stepper_get_current_step(1, &current_m1) != HAL_OK) return HAL_ERROR;
    if (stepper_get_current_step(2, &current_m2) != HAL_OK) return HAL_ERROR;
    if (stepper_get_current_step(3, &current_m3) != HAL_OK) return HAL_ERROR;

    /* 残り距離 */
    delta_m1 = target_m1 - current_m1;
    if (delta_m1 < 0) delta_m1 = -delta_m1;

    delta_m2 = target_m2 - current_m2;
    if (delta_m2 < 0) delta_m2 = -delta_m2;

    delta_m3 = target_m3 - current_m3;
    if (delta_m3 < 0) delta_m3 = -delta_m3;

    max_delta = delta_m1;
    if (delta_m2 > max_delta) max_delta = delta_m2;
    if (delta_m3 > max_delta) max_delta = delta_m3;

    /* 全軸到達 */
    if (max_delta == 0) {
        AppSyncMotionState.active = false;
        return HAL_OK;
    }

    /* 毎周期、残り距離比で速度・加速度を再配分 */
    sync_m1_rate = base_m1_rate * ((float)delta_m1 / (float)max_delta);
    sync_m2_rate = base_m2_rate * ((float)delta_m2 / (float)max_delta);
    sync_m3_rate = base_m3_rate * ((float)delta_m3 / (float)max_delta);

    sync_m1_acc  = base_m1_acc  * ((float)delta_m1 / (float)max_delta);
    sync_m2_acc  = base_m2_acc  * ((float)delta_m2 / (float)max_delta);
    sync_m3_acc  = base_m3_acc  * ((float)delta_m3 / (float)max_delta);

    /* 動く軸には最低限の値を持たせる */
    if (delta_m1 > 0 && sync_m1_rate < 1.0f) sync_m1_rate = 1.0f;
    if (delta_m2 > 0 && sync_m2_rate < 1.0f) sync_m2_rate = 1.0f;
    if (delta_m3 > 0 && sync_m3_rate < 1.0f) sync_m3_rate = 1.0f;

    if (delta_m1 > 0 && sync_m1_acc < 1.0f) sync_m1_acc = 1.0f;
    if (delta_m2 > 0 && sync_m2_acc < 1.0f) sync_m2_acc = 1.0f;
    if (delta_m3 > 0 && sync_m3_acc < 1.0f) sync_m3_acc = 1.0f;

    /*
     * axisには
     * - 最終目標位置は固定
     * - 速度・加速度だけ毎周期更新
     */
    if (axis_set_motion_target(1, target_m1, sync_m1_rate, sync_m1_acc) != HAL_OK)
        return HAL_ERROR;
    if (axis_set_motion_target(2, target_m2, sync_m2_rate, sync_m2_acc) != HAL_OK)
        return HAL_ERROR;
    if (axis_set_motion_target(3, target_m3, sync_m3_rate, sync_m3_acc) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}

HAL_StatusTypeDef sync_motion_stop_all(void)
{
    return HAL_OK;
}
