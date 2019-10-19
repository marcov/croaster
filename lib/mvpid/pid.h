/*
 * pid.h
 *
 *  Created on: Feb 8, 2016
 *      Author: Marco Vedovati
 */

#ifndef PID_H_
#define PID_H_

#include <stdint.h>


void pid_update_setpoint(float sp);

void pid_tune(float coeff_p, float coeff_i, float coeff_d);

void pid_start(float new_sp, bool bumpless, uint32_t manual_cs);

int32_t pid_get_control_value(float pv);

int32_t pid_abs_2_pct(int32_t pid_abs);

int32_t pid_get_scaled_cs(int32_t pid_abs, int32_t min, int32_t max);

/**
 * @brief converts a control signal from its scaled value into a discrete value.
 *
 */
int32_t pid_discretize(int value, unsigned discrete_step);

#endif /* PID_H_ */
