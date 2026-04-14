#include "stepper/axis.h"
#include "system_config.h"
#include "stepper/stepper_hw.h"
#include <math.h>

static AxisControlState_t AxisControlState[AXIS_COUNT];

/* axis層側の保護用上限
 * 上位が変な値を入れても最後にここで守る
 */
static const float axis_rate_limit_step_s[AXIS_COUNT] = {
    10000.0f, 10000.0f, 10000.0f
};

static const float axis_acc_limit_step_s2[AXIS_COUNT] = {
    50000.0f, 50000.0f, 50000.0f
};

HAL_StatusTypeDef axis_init(void)
{
    uint8_t i;

    for (i = 0; i < AXIS_COUNT; i++) {
        AxisControlState[i].target_step = 0;
        AxisControlState[i].current_rate_step_s = 0.0f;
        AxisControlState[i].max_rate_step_s = 0.0f;
        AxisControlState[i].max_acc_step_s2 = 0.0f;
        AxisControlState[i].active = false;
        AxisControlState[i].busy = false;
    }

    return HAL_OK;
}

HAL_StatusTypeDef axis_set_motion_target(uint8_t axis,
                                         int32_t target_step,
                                         float max_rate_step_s,
                                         float max_acc_step_s2)
{
    uint8_t i;
    float rate_cmd;
    float acc_cmd;

    if (axis < 1 || axis > AXIS_COUNT) return HAL_ERROR;
    if (max_rate_step_s < 0.0f) return HAL_ERROR;
    if (max_acc_step_s2 < 0.0f) return HAL_ERROR;

    i = axis - 1;

    /* 保護的クリップ */
    rate_cmd = max_rate_step_s;
    if (rate_cmd > axis_rate_limit_step_s[i]) {
        rate_cmd = axis_rate_limit_step_s[i];
    }

    acc_cmd = max_acc_step_s2;
    if (acc_cmd > axis_acc_limit_step_s2[i]) {
        acc_cmd = axis_acc_limit_step_s2[i];
    }

    AxisControlState[i].target_step = target_step;
    AxisControlState[i].max_rate_step_s = rate_cmd;
    AxisControlState[i].max_acc_step_s2 = acc_cmd;
    AxisControlState[i].active = true;
    AxisControlState[i].busy = true;

    return HAL_OK;
}

HAL_StatusTypeDef axis_stop(uint8_t axis)
{
    uint8_t i;

    if (axis < 1 || axis > AXIS_COUNT) return HAL_ERROR;
    i = axis - 1;

    if (step_timer_set_rate(axis, 0.0f) != HAL_OK) {
        return HAL_ERROR;
    }

    AxisControlState[i].current_rate_step_s = 0.0f;
    AxisControlState[i].active = false;
    AxisControlState[i].busy = false;

    return HAL_OK;
}

HAL_StatusTypeDef axis_update(uint8_t axis)
{
    uint8_t i;
    int32_t current_step;
    int32_t error_step;
    int32_t remaining_steps;
    AxisDir_t desired_dir;
    AxisDir_t current_dir;
    bool is_running;

    float v;
    float a;
    float dt;
    float stop_distance;
    float next_rate;
    float safe_rate;

    if (axis < 1 || axis > AXIS_COUNT) return HAL_ERROR;
    i = axis - 1;

    if (AxisControlState[i].active == false) {
        return HAL_OK;
    }

    if (stepper_get_current_step(axis, &current_step) != HAL_OK) {
        return HAL_ERROR;
    }

    error_step = AxisControlState[i].target_step - current_step;

    if (error_step == 0) {
        AxisControlState[i].current_rate_step_s = 0.0f;
        AxisControlState[i].active = false;
        AxisControlState[i].busy = false;
        return HAL_OK;
    }

    if (error_step == 1 || error_step == -1) {
        if (step_timer_set_rate(axis, 0.0f) != HAL_OK) {
            return HAL_ERROR;
        }
        return HAL_OK;
    }

    desired_dir = (error_step > 0) ? DIR_POSITIVE : DIR_NEGATIVE;
    remaining_steps = (error_step > 0) ? error_step : -error_step;

    if (stepper_is_running(axis, &is_running) != HAL_OK) {
        return HAL_ERROR;
    }

    if (stepper_get_dir(axis, &current_dir) != HAL_OK) {
        return HAL_ERROR;
    }

    /* 方向変更は必ず停止後 */
    if (is_running == false) {
        if (current_dir != desired_dir) {
            if (set_dir(axis, desired_dir) != HAL_OK) {
                return HAL_ERROR;
            }
            return HAL_OK;
        }
    } else {
        if (current_dir != desired_dir) {
            if (step_timer_set_rate(axis, 0.0f) != HAL_OK) {
                return HAL_ERROR;
            }
            return HAL_OK;
        }
    }

    v = AxisControlState[i].current_rate_step_s;
    a = AxisControlState[i].max_acc_step_s2;
    dt = AXIS_CONTROL_PERIOD_S;

    /* 停止距離（連続系近似） */
    if (a > 0.0f) {
        stop_distance = (v * v) / (2.0f * a);
    } else {
        stop_distance = 0.0f;
    }

    /* 加速 or 減速 */
    if ((float)remaining_steps <= stop_distance) {
        next_rate = v - a * dt;
        if (next_rate < 0.0f) {
            next_rate = 0.0f;
        }
    } else {
        next_rate = v + a * dt;
        if (next_rate > AxisControlState[i].max_rate_step_s) {
            next_rate = AxisControlState[i].max_rate_step_s;
        }
    }

    /* 1制御周期で進みすぎないための安全レート制限
     * 最後の1stepは下位stop要求に任せるので (remaining_steps - 1)
     */
    safe_rate = ((float)(remaining_steps - 1)) / AXIS_CONTROL_PERIOD_S;
    if (next_rate > safe_rate) {
        next_rate = safe_rate;
    }

    if (next_rate < 0.0f) {
        next_rate = 0.0f;
    }

    AxisControlState[i].current_rate_step_s = next_rate;

    if (step_timer_set_rate(axis, next_rate) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef axis_update_all(void)
{
    uint8_t axis;

    for (axis = 1; axis <= AXIS_COUNT; axis++) {
        if (axis_update(axis) != HAL_OK) {
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

HAL_StatusTypeDef axis_is_busy(uint8_t axis, bool *busy)
{
    if (axis < 1 || axis > AXIS_COUNT) return HAL_ERROR;
    if (busy == NULL) return HAL_ERROR;

    *busy = AxisControlState[axis - 1].busy;
    return HAL_OK;
}

HAL_StatusTypeDef axis_get_target_step(uint8_t axis, int32_t *target_step)
{
    if (axis < 1 || axis > AXIS_COUNT) return HAL_ERROR;
    if (target_step == NULL) return HAL_ERROR;

    *target_step = AxisControlState[axis - 1].target_step;
    return HAL_OK;
}
