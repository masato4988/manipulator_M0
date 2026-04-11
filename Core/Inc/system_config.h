/*
 * system_config.h
 *
 *  Created on: Mar 31, 2026
 *      Author: miyab
 */

#ifndef INC_SYSTEM_CONFIG_H_
#define INC_SYSTEM_CONFIG_H_


/* ===== 制御周期 ===== */

//#define AXIS_CONTROL_FREQ_HZ     1000U
//#define AXIS_CONTROL_PERIOD_MS   (1000U / AXIS_CONTROL_FREQ_HZ)
#define AXIS_CONTROL_PERIOD_S   0.001f
#define AXIS_CONTROL_FREQ_HZ    1000.0f//(1.0f / AXIS_CONTROL_PERIOD_S)

/* ===== 軸数 ===== */

#define AXIS_COUNT 3U


#endif /* INC_SYSTEM_CONFIG_H_ */
