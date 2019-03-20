// pidcontroller.cxx - implementation of PID controller
//
// Written by Torsten Dreyer
// Based heavily on work created by Curtis Olson, started January 2004.
//
// Copyright (C) 2004  Curtis L. Olson  - http://www.flightgear.org/~curt
// Copyright (C) 2010  Torsten Dreyer - Torsten (at) t3r (dot) de
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
//

#include "pidcontroller.hxx"
extern "C" {
#include "user.h"
}

static inline float get_output_min(void) {
    return MIN_OT1;
}

static inline float get_output_max(void) {
    return MAX_OT1;
}

static inline float get_output_value(void) {
    extern int levelOT1;
    return levelOT1;
}

static inline void set_output_value(float val) {
    extern int levelOT1;
    levelOT1 = val;
}

static inline float get_input_value(void) {
    extern double Input;
    return Input;
}

static inline float get_reference_value(void) {
    extern double Setpoint;
    return Setpoint;
}

PIDController::PIDController():
    alpha( 0.1 ),
    beta( 1.0 ),
    gamma( 0.0 ),
    ep_n_1( 0.0 ),
    edf_n_1( 0.0 ),
    edf_n_2( 0.0 ),
    u_n_1( 0.0 ),
    desiredTs( 0.0 ),
    elapsedTime( 0.0 )
{
}

/*
 * Roy Vegard Ovesen:
 *
 * Ok! Here is the PID controller algorithm that I would like to see
 * implemented:
 *
 *   delta_u_n = Kp * [ (ep_n - ep_n-1) + ((Ts/Ti)*e_n)
 *               + (Td/Ts)*(edf_n - 2*edf_n-1 + edf_n-2) ]
 *
 *   u_n = u_n-1 + delta_u_n
 *
 * where:
 *
 * delta_u : The incremental output
 * Kp      : Proportional gain
 * ep      : Proportional error with reference weighing
 *           ep = beta * r - y
 *           where:
 *           beta : Weighing factor
 *           r    : Reference (setpoint)
 *           y    : Process value, measured
 * e       : Error
 *           e = r - y
 * Ts      : Sampling interval
 * Ti      : Integrator time
 * Td      : Derivator time
 * edf     : Derivate error with reference weighing and filtering
 *           edf_n = edf_n-1 / ((Ts/Tf) + 1) + ed_n * (Ts/Tf) / ((Ts/Tf) + 1)
 *           where:
 *           Tf : Filter time
 *           Tf = alpha * Td , where alpha usually is set to 0.1
 *           ed : Unfiltered derivate error with reference weighing
 *             ed = gamma * r - y
 *             where:
 *             gamma : Weighing factor
 *
 * u       : absolute output
 *
 * Index n means the n'th value.
 *
 *
 * Inputs:
 * enabled ,
 * y_n , r_n , beta=1 , gamma=0 , alpha=0.1 ,
 * Kp , Ti , Td , Ts (is the sampling time available?)
 * u_min , u_max
 *
 * Output:
 * u_n
 */

void PIDController::update( bool firstTime, double dt )
{
    if( firstTime ) {
      ep_n_1 = 0.0;
      edf_n_2 = edf_n_1 = 0.0;

      // first time being enabled, seed with current property tree value
      u_n_1 = get_output_value();
    }

    double u_min = get_output_min();
    double u_max = get_output_max();

    elapsedTime += dt;
    if( elapsedTime <= desiredTs ) {
        // do nothing if time step is not positive (i.e. no time has
        // elapsed)
        return;
    }
    double Ts = elapsedTime; // sampling interval (sec)
    elapsedTime = 0.0;


    double y_n = get_input_value();
    double r_n = get_reference_value();


    // Calculates proportional error:
    double ep_n = beta * r_n - y_n;

    // Calculates error:
    double e_n = r_n - y_n;

    double edf_n = 0.0;
    double td = Td;
    if ( td > 0.0 ) { // do we need to calcluate derivative error?

      // Calculates derivate error:
        double ed_n = gamma * r_n - y_n;

        // Calculates filter time:
        double Tf = alpha * td;

        // Filters the derivate error:
        edf_n = edf_n_1 / (Ts/Tf + 1)
            + ed_n * (Ts/Tf) / (Ts/Tf + 1);
    } else {
        edf_n_2 = edf_n_1 = edf_n = 0.0;
    }

    // Calculates the incremental output:
    double ti = Ti;
    double delta_u_n = 0.0; // incremental output
    if ( ti > 0.0 ) {
        delta_u_n = Kp * ( (ep_n - ep_n_1)
                           + ((Ts/ti) * e_n)
                           + ((td/Ts) * (edf_n - 2*edf_n_1 + edf_n_2)) );

    }

    // Integrator anti-windup logic:
    if ( delta_u_n > (u_max - u_n_1) ) {
        delta_u_n = u_max - u_n_1;
    } else if ( delta_u_n < (u_min - u_n_1) ) {
        delta_u_n = u_min - u_n_1;
    }

    // Calculates absolute output:
    double u_n = u_n_1 + delta_u_n;

    // Updates indexed values;
    u_n_1   = u_n;
    ep_n_1  = ep_n;
    edf_n_2 = edf_n_1;
    edf_n_1 = edf_n;

    set_output_value( u_n );
}

//------------------------------------------------------------------------------
bool PIDController::configure(float kp, float ti, float td, float ts)
{
    Kp = kp;
    Ti = ti;
    Td = td;
    desiredTs = ts;
    return true;
}
