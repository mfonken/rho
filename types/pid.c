//
//  pid.c
//  rho_client
//
//  Created by Matthew Fonken on 10/28/18.
//  Copyright © 2019 Marbl. All rights reserved.
//

#include "pid.h"

#ifndef TIMESTAMP
#include "timestamp.h"
#endif

void RhoPIDInitialize( pid_filter_t * pid, pid_gain_t k )
{
    /* Zero Entire PID */
    memset( pid, 0, sizeof(pid_filter_t) );
    
    if( !k.kp && !k.ki && !k.kd )
    {
        k.pu = 1.;
        pid->gain.kp = DEFAULT_PROPORTIONAL_FACTOR * k.ku;
        pid->gain.ki = DEFAULT_INTEGRAL_FACTOR * ( pid->gain.kp / k.pu );
        pid->gain.kd = DEFAULT_DERIVATIVE_FACTOR * ( pid->gain.kd * k.pu );
    }
    else
    {
        pid->gain.kp = k.kp;
        pid->gain.ki = k.ki;
        pid->gain.kd = k.kd;
    }
    pid->timestamp = TIMESTAMP();
}

void RhoPIDUpdate( pid_filter_t * pid, floating_t actual, floating_t target )
{
    pid->error = actual - target;
    
    pid->pv = pid->error * pid->gain.kp;
    
    pid->dt = TIMESTAMP() - pid->timestamp;
    pid->total_error += pid->error * pid->dt;
    pid->iv = pid->gain.ki * pid->total_error;
    
    pid->delta_error = pid->prev_error - pid->error;
    pid->dv = ZDIV( ( pid->gain.kd * pid->delta_error ), pid->dt);
    
    pid->value = pid->pv + pid->iv + pid->dv + pid->bias;
    
    if( pid->max_value > 0 )
        pid->value = BOUND(pid->value, pid->min_value, pid->max_value);
    
    pid->prev_error = pid->error;

    pid->total_error *= 0.9;
}

void RhoPIDPrint( pid_filter_t * pid )
{
    printf("\tValue:%3.4f\tBias:%3.4f\tError:%3.4f\tTotalError:%3.4f\t[P%3.2f\tI%3.2f\tD%3.2f]",
           pid->value, pid->bias, pid->error, pid->total_error, pid->pv, pid->iv, pid->dv);
}
