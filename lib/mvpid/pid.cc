/*
 * pid.c
 *
 *  Created on: Feb 8, 2016
 *      Author: Marco Vedovati
 */
#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>

#include "pid.h"
#include "debug.h"

typedef struct
{
    float kp;
    float ki;
    float kd;
    float setpoint;
    float error_sum;
    float error_prev;
    bool first_run;
    bool bumpless;
    int32_t  manual_cs_pct;
    // Compute derivative on dPV/dt instead of on de/dt
    bool derivative_on_pv;
    float pv_prev;
} pid_ctxt_t;

pid_ctxt_t pid_ctxt;

//#define PID_PRINTF(...) DEBUG_PRINTF(__VA_ARGS__)
#define PID_PRINTF(...)

// This is the maximum value expect as output from the PID controller
#define PID_OUTPUT_MAX    (1000)
#define PID_OUTPUT_MIN    (0)


void pid_tune(float coeff_p, float coeff_i, float coeff_d)
{
    pid_ctxt.kp = coeff_p;
    pid_ctxt.ki = coeff_i;
    pid_ctxt.kd = coeff_d;
}

uint32_t getRange() {
    int32_t range = PID_OUTPUT_MAX - PID_OUTPUT_MIN;

    if (range < 0) {
        range = -range;
    }

    return range;
}

void pid_start(float new_sp, bool bumpless, uint32_t manual_cs_pct)
{
    pid_ctxt.setpoint = new_sp;
    pid_ctxt.error_sum = 0;
    pid_ctxt.error_prev = 0;
    pid_ctxt.pv_prev = 0;
    pid_ctxt.first_run = true;
    pid_ctxt.bumpless = bumpless;
    pid_ctxt.derivative_on_pv = true;

    if (bumpless) {
        auto range = getRange();

        pid_ctxt.manual_cs_pct = PID_OUTPUT_MIN + (range * manual_cs_pct / (uint32_t)100);
        PID_PRINTF("bmpls cs=%d man=%d\n", manual_cs_pct, pid_ctxt.manual_cs_pct);
    }
}

void pid_update_setpoint(float sp)
{
    pid_ctxt.setpoint   = sp;
}

int32_t pid_get_control_value(float pv)
{
    float proportional;
    float integral;
    float derivative;
    float error;
    float pid_combi;

    error = pid_ctxt.setpoint - pv;

    proportional = pid_ctxt.kp * error;

    pid_ctxt.error_sum += error;

    integral = pid_ctxt.ki * pid_ctxt.error_sum;

    float delta = (pid_ctxt.derivative_on_pv) ?
                    (pv - pid_ctxt.pv_prev) :
                    (error - pid_ctxt.error_prev) ;

    derivative = pid_ctxt.kd * delta;

    pid_ctxt.error_prev = error;
    pid_ctxt.pv_prev = pv;

    /* Bumpless transfer */
    if (pid_ctxt.first_run) {
        PID_PRINTF("1st run\n");
        pid_ctxt.first_run = false;

        // Avoid derivative kicking in too much on 1st run,
        // because of a big delta step
        derivative = 0;

        if (pid_ctxt.bumpless) {
            // Force the integral value to get the same output as the manal CS.
            integral = pid_ctxt.manual_cs_pct - proportional - integral;

            pid_ctxt.error_sum  = integral / pid_ctxt.ki;
        }
    }

    pid_combi = proportional + integral + derivative;

    /* Anti windup technique: conditional integration.
     * The integration is stopped when the control variable saturates and
     * the control variable and control error have the same sign. */
    if ( ((pid_combi >= PID_OUTPUT_MAX)  && (error > 0)) ||
         ((pid_combi <= PID_OUTPUT_MIN) && (error < 0)) )
    {
        pid_combi -= integral;
        pid_ctxt.error_sum -= error;
        integral   = pid_ctxt.ki * pid_ctxt.error_sum;
        pid_combi += integral;
    }

    /* Anti-windup technique: back calculation of the integral part. */
    if ( ((pid_combi > PID_OUTPUT_MAX)  && (integral > 0)) ||
         ((pid_combi < PID_OUTPUT_MIN) && (integral < 0)))
    {
        float saturation_val = pid_combi > PID_OUTPUT_MAX ? PID_OUTPUT_MAX : PID_OUTPUT_MIN;

        pid_ctxt.error_sum = (saturation_val - proportional - derivative) / pid_ctxt.ki;

        integral = pid_ctxt.ki * pid_ctxt.error_sum;

        pid_combi = proportional + integral + derivative;
    }

    if (pid_combi > PID_OUTPUT_MAX)      pid_combi = PID_OUTPUT_MAX;
    else if (pid_combi < PID_OUTPUT_MIN) pid_combi = PID_OUTPUT_MIN;

    PID_PRINTF("#P %" PRId32 " %" PRId32 " %" PRId32 " %" PRId32 " \n",
            proportional, integral, derivative, pid_combi);

    return pid_combi;
}


int32_t pid_abs_2_pct(int32_t pid_abs)
{
    int32_t control_pct = (pid_abs - PID_OUTPUT_MIN) * 100 / getRange();

    if (control_pct > 100)
    {
        control_pct = 100;
    }
    else if (control_pct < 0)
    {
        control_pct = 0;
    }

    return control_pct;
}


int32_t pid_get_scaled_cs(int32_t pid_abs, int32_t min, int32_t max)
{
    float   gain      = (float)(max - min) / (float)getRange();
    int32_t delta_pid = (pid_abs - PID_OUTPUT_MIN);
    int32_t delta_cs  = delta_pid * gain;
    int32_t scaled_cs = min + delta_cs;

    if (scaled_cs < min)
    {
        scaled_cs = min;
    }
    else if (scaled_cs > max)
    {
        scaled_cs = max;
    }

#if 0
    PID_PRINTF("Gain: %f - delta_pid %ld - delta_cs %ld - scaled_cs = %ld \n",
               gain, delta_pid, delta_cs, scaled_cs);
#endif
    return scaled_cs;
}


int32_t pid_discretize(int val, unsigned discrete_step)
{
    // can't use div on ESP8266 for some unk. reason...
    unsigned quot;
    unsigned rem;
    int discr_pct;
    unsigned half_of_res = discrete_step / 2;

    quot      = val / discrete_step;
    discr_pct = quot * discrete_step;
    rem       = val - (discr_pct);

    if (half_of_res > 0 && rem >= half_of_res)
    {
        discr_pct += discrete_step;
    }

    return discr_pct;
}


