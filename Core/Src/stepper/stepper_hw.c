/*
 * stepper_hw.c
 *
 *  Created on: Mar 27, 2026
 *      Author: miyab
 */
#include "main.h"
#include "stepper/stepper_hw.h"
#include "tim.h"
#include <stdbool.h>
#include <math.h>
#include <stdio.h>

#define STEP_PULSE_WIDTH_US 10U

typedef struct {//GPIO_output_level
	GPIO_PinState POSITIVE_LEVEL;
	GPIO_PinState NEGATIVE_LEVEL;
}AxisDirConfig_t;

typedef struct {
	GPIO_TypeDef* DIR_GPIO_Port;
	uint16_t DIR_Pin;
	GPIO_TypeDef* STEP_GPIO_Port;
	uint16_t STEP_Pin;
	GPIO_TypeDef* EN_GPIO_Port;
	uint16_t EN_Pin;
}StepperGPIOs_t;

typedef struct {
    volatile int32_t current_step;
    volatile AxisDir_t dir;
    volatile bool is_running;
} AxisMotorState_t;

typedef struct {  //step/s
	float axis_step_rate_max;
	float axis_step_rate_min;
	float axis_step_rate_stop;
} AxisStepRateConfig_t;

static const StepperGPIOs_t StepperGPIOs[3] = {
		{DIR_1_GPIO_Port, DIR_1_Pin, STEP_1_GPIO_Port, STEP_1_Pin, EN_1_GPIO_Port, EN_1_Pin},
		{DIR_2_GPIO_Port, DIR_2_Pin, STEP_2_GPIO_Port, STEP_2_Pin, EN_2_GPIO_Port, EN_2_Pin},
		{DIR_3_GPIO_Port, DIR_3_Pin, STEP_3_GPIO_Port, STEP_3_Pin, EN_3_GPIO_Port, EN_3_Pin}
};

static const AxisDirConfig_t AxisDirConfig[3] = {
		{GPIO_PIN_RESET, GPIO_PIN_SET},
		{GPIO_PIN_SET, GPIO_PIN_RESET},
		{GPIO_PIN_RESET, GPIO_PIN_SET}
};

volatile AxisMotorState_t AxisMotorState[3] = {
		{0, DIR_POSITIVE, false},
		{0, DIR_POSITIVE, false},
		{0, DIR_POSITIVE, false}
};

static const AxisStepRateConfig_t axisStepRateConfig[3] = {
		{10000.0f, 20.0f, 2.0f},
		{10000.0f, 20.0f, 2.0f},
		{10000.0f, 20.0f, 2.0f}
};

static volatile bool flag_pwm_stop[3] = {false, false, false};

static HAL_StatusTypeDef step_timer_prepare_idle(
    TIM_HandleTypeDef *htim,
    uint32_t channel,
    uint32_t cc_flag,
    uint32_t arr_init)
{
    if (HAL_TIM_PWM_Stop_IT(htim, channel) != HAL_OK) {
        return HAL_ERROR;
    }

    __HAL_TIM_SET_COUNTER(htim, 0);
    __HAL_TIM_SET_AUTORELOAD(htim, arr_init);
    __HAL_TIM_SET_COMPARE(htim, channel, STEP_PULSE_WIDTH_US);

	HAL_TIM_GenerateEvent(htim, TIM_EVENTSOURCE_UPDATE);

    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
    __HAL_TIM_CLEAR_FLAG(htim, cc_flag);

    return HAL_OK;
}

//static bool home_sw_gpio_to_bool(GPIO_PinState pin_state)
//{
//#if HOME_SW_ACTIVE_LEVEL_IS_SET
//    return (pin_state == GPIO_PIN_SET);
//#else
//    return (pin_state == GPIO_PIN_RESET);
//#endif
//}

HAL_StatusTypeDef stepper_init(void){
    for (int i = 0; i < 3; i++) {
        AxisMotorState[i].current_step = 0;
        AxisMotorState[i].dir = DIR_POSITIVE;
        AxisMotorState[i].is_running = false;
        flag_pwm_stop[i] = false;
		HAL_GPIO_WritePin(StepperGPIOs[i].EN_GPIO_Port, StepperGPIOs[i].EN_Pin, GPIO_PIN_RESET);
		if(set_dir(i + 1, DIR_POSITIVE) != HAL_OK) return HAL_ERROR;
    }

    const uint32_t arr_init = 10000 - 1;

    if (step_timer_prepare_idle(&htim4, TIM_CHANNEL_1, TIM_FLAG_CC1, arr_init) != HAL_OK)
        return HAL_ERROR;

    if (step_timer_prepare_idle(&htim3, TIM_CHANNEL_1, TIM_FLAG_CC1, arr_init) != HAL_OK)
        return HAL_ERROR;

    if (step_timer_prepare_idle(&htim2, TIM_CHANNEL_3, TIM_FLAG_CC3, arr_init) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}

HAL_StatusTypeDef set_dir(uint8_t axis, AxisDir_t dir){
	uint8_t i = axis - 1;

    if (axis < 1 || axis > 3){
    	return HAL_ERROR;
    }

    if (dir != DIR_NEGATIVE && dir != DIR_POSITIVE){
    	return HAL_ERROR;
    }

	if (AxisMotorState[i].is_running == true){
		return HAL_BUSY;
	}

    if (dir == DIR_POSITIVE) {
		HAL_GPIO_WritePin(
				StepperGPIOs[i].DIR_GPIO_Port,
				StepperGPIOs[i].DIR_Pin,
				AxisDirConfig[i].POSITIVE_LEVEL);
    } else {
		HAL_GPIO_WritePin(
				StepperGPIOs[i].DIR_GPIO_Port,
				StepperGPIOs[i].DIR_Pin,
				AxisDirConfig[i].NEGATIVE_LEVEL);
    }
    AxisMotorState[i].dir = dir;

    return HAL_OK;
}

static HAL_StatusTypeDef step_timer_start(uint8_t axis){
    switch (axis) {
    case 1:
        return HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1);
    case 2:
        return HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
    case 3:
        return HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_3);
    default:
        return HAL_ERROR;
    }
}

static HAL_StatusTypeDef step_timer_stop(uint8_t axis){
    switch (axis) {
    case 1:
        return HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_1);
    case 2:
        return HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
    case 3:
        return HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_3);
    default:
        return HAL_ERROR;
    }
}

//---------- ccr interrupt callback ----------
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
	uint8_t i;
    AxisDir_t dir;

    if (htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
    	i = 0;
    }else if(htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
    	i = 1;
    }else if(htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){
    	i = 2;
    }else{
    	return;
    }

    dir = AxisMotorState[i].dir;

	if(dir == DIR_POSITIVE){
		AxisMotorState[i].current_step++;
	}else if(dir == DIR_NEGATIVE){
		AxisMotorState[i].current_step--;
	}

	if(flag_pwm_stop[i] == true){
		flag_pwm_stop[i] = false;
		if (step_timer_stop(i + 1) == HAL_OK) {
			AxisMotorState[i].is_running = false;
		} else {
			Error_Handler();
		}
	}
}

HAL_StatusTypeDef step_timer_set_rate(uint8_t axis, float step_abs_rate_hz){
	float rate;
    uint32_t arr;
    uint8_t i;

    if (axis < 1 || axis > 3)
    	return HAL_ERROR;

    i = axis - 1;
    rate = fabsf(step_abs_rate_hz);

    if (rate < axisStepRateConfig[i].axis_step_rate_stop) {
    	if (AxisMotorState[i].is_running == true) {
			flag_pwm_stop[i] = true;
		} else {
			flag_pwm_stop[i] = false;
		}
		return HAL_OK;
	}

    if (rate < axisStepRateConfig[i].axis_step_rate_min) {
		rate = axisStepRateConfig[i].axis_step_rate_min;
	} else if (rate > axisStepRateConfig[i].axis_step_rate_max) {
		rate = axisStepRateConfig[i].axis_step_rate_max;
	}

    arr = (uint32_t)(1000000.0f / rate - 1.0f);
    if (arr <= STEP_PULSE_WIDTH_US + 1U) {
        return HAL_ERROR;
    }

    switch(axis){
    case 1:
		__HAL_TIM_SET_AUTORELOAD(&htim4, arr);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, STEP_PULSE_WIDTH_US);
		break;
    case 2:
		__HAL_TIM_SET_AUTORELOAD(&htim3, arr);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, STEP_PULSE_WIDTH_US);
		break;
    case 3:
		__HAL_TIM_SET_AUTORELOAD(&htim2, arr);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, STEP_PULSE_WIDTH_US);
		break;
    default:
		return HAL_ERROR;
    }

    if (AxisMotorState[i].is_running == false) {
    	switch (axis) {
		case 1:
			__HAL_TIM_SET_COUNTER(&htim4, 0);
			__HAL_TIM_CLEAR_FLAG(&htim4,TIM_FLAG_CC1);
			break;
		case 2:
			__HAL_TIM_SET_COUNTER(&htim3, 0);
			__HAL_TIM_CLEAR_FLAG(&htim3,TIM_FLAG_CC1);
			break;
		case 3:
			__HAL_TIM_SET_COUNTER(&htim2, 0);
			__HAL_TIM_CLEAR_FLAG(&htim2,TIM_FLAG_CC3);
			break;
		default:
			return HAL_ERROR;
		}
    	flag_pwm_stop[i] = false;
		AxisMotorState[i].is_running = true;
		if (step_timer_start(axis) != HAL_OK){
			AxisMotorState[i].is_running = false;
			return HAL_ERROR;
		}
	}

    return HAL_OK;
}

HAL_StatusTypeDef stepper_request_stop(uint8_t axis)
{
    uint8_t i;

    if (axis < 1 || axis > 3) {
        return HAL_ERROR;
    }

    i = axis - 1;

    if (AxisMotorState[i].is_running == true) {
        flag_pwm_stop[i] = true;
    } else {
        flag_pwm_stop[i] = false;
    }

    return HAL_OK;
}

HAL_StatusTypeDef stepper_get_current_step(uint8_t axis, int32_t *current_step){
	if(axis < 1 || axis > 3) return HAL_ERROR;
    if(current_step == NULL) return HAL_ERROR;

	*current_step = AxisMotorState[axis - 1].current_step;
	return HAL_OK;
}

HAL_StatusTypeDef stepper_set_current_step(uint8_t axis, int32_t current_step)
{
    if(axis < 1 || axis > 3) return HAL_ERROR;

    AxisMotorState[axis - 1].current_step = current_step;
    return HAL_OK;
}

HAL_StatusTypeDef stepper_is_running(uint8_t axis, bool *is_running){
    if (axis < 1 || axis > 3) return HAL_ERROR;
    if (is_running == NULL) return HAL_ERROR;

    *is_running = AxisMotorState[axis - 1].is_running;
    return HAL_OK;
}

HAL_StatusTypeDef stepper_get_dir(uint8_t axis, AxisDir_t *dir){
    if (axis < 1 || axis > 3) return HAL_ERROR;
    if (dir == NULL) return HAL_ERROR;

    *dir = AxisMotorState[axis - 1].dir;
    return HAL_OK;
}
