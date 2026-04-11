/*
 * home_sw.h
 *
 *  Created on: Mar 31, 2026
 *      Author: miyab
 */

#ifndef INC_HOME_SW_H_
#define INC_HOME_SW_H_

#include "main.h"
#include "stdbool.h"

HAL_StatusTypeDef home_sw_read(uint8_t axis, bool *is_on);


#endif /* INC_HOME_SW_H_ */
