/*
 * stepper_hw.h
 *
 *  Created on: Mar 27, 2026
 *      Author: miyab
 */

#ifndef INC_STEPPER_HW_H_
#define INC_STEPPER_HW_H_

#include "stdbool.h"

typedef enum {
    DIR_POSITIVE = 1,
    DIR_NEGATIVE = -1
} AxisDir_t;

HAL_StatusTypeDef stepper_init(void);
HAL_StatusTypeDef set_dir(uint8_t axis, AxisDir_t dir);
HAL_StatusTypeDef step_timer_set_rate(uint8_t axis, float step_rate_hz);

HAL_StatusTypeDef stepper_request_stop(uint8_t axis);

HAL_StatusTypeDef stepper_get_current_step(uint8_t axis, int32_t *current_step);
HAL_StatusTypeDef stepper_set_current_step(uint8_t axis, int32_t current_step);

HAL_StatusTypeDef stepper_is_running(uint8_t axis, bool *is_running);
HAL_StatusTypeDef stepper_get_dir(uint8_t axis, AxisDir_t *dir);

#endif /* INC_STEPPER_HW_H_ */
