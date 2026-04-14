/*
 * axis_home.h
 *
 *  Created on: Mar 30, 2026
 *      Author: miyab
 */

#ifndef INC_HOMING_CONTROL_H_
#define INC_HOMING_CONTROL_H_


#include "main.h"
#include "stdbool.h"
#include "stepper_hw.h"

typedef enum {
    HOMING_STATE_IDLE = 0,
    HOMING_STATE_START,			//1
    HOMING_STATE_ESCAPE,		//2
    HOMING_STATE_SEARCH_FAST,	//3
    HOMING_STATE_BACKOFF,		//4
    HOMING_STATE_SEARCH_SLOW,	//5
    HOMING_STATE_SET_ORIGIN,	//6
    HOMING_STATE_DONE,			//7
    HOMING_STATE_ERROR			//8
} HomingState_t;

HAL_StatusTypeDef homing_control_init(void);
HAL_StatusTypeDef homing_control_start(uint8_t axis);
HAL_StatusTypeDef homing_control_update(void);

HAL_StatusTypeDef homing_control_is_busy(uint8_t axis, bool *is_busy);
HAL_StatusTypeDef homing_control_is_done(uint8_t axis, bool *is_done);
HAL_StatusTypeDef homing_control_is_homed(uint8_t axis, bool *is_homed);
HAL_StatusTypeDef homing_control_get_state(uint8_t axis, HomingState_t *state);

#endif /* INC_HOMING_CONTROL_H_ */
