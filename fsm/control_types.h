//
//  control_type.h
//
//  Created by Matthew Fonken on 2/10/19.
//  Copyright © 2019 Matthew Fonken. All rights reserved.
//

#ifndef control_types_h
#define control_types_h

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "psm_config.h"
#include "maths_master.h"
#include "kalman.h"

#define TARGET_STATE TARGET_POPULATED
#define NUM_STATE_GROUPS NUM_STATES
#define DEFAULT_STATE_VECTOR        { 0.2, 0.05, 0.05, 0.7 }

    typedef enum
    {
        UNKNOWN_STATE = -1,
        UNDER_POPULATED,
        TARGET_POPULATED,
        OVER_POPULATED,
        CHAOTIC,
        NUM_STATES
    } state_t;

    /* Stability tracking for selections */
    typedef struct
    {
        kalman_t
            state,
            system;
    } stability_t;

    typedef struct
    {
        uint16_t density;
        uint8_t thresh;
        uint8_t label;
    } observation_t;

    typedef struct
    {
        observation_t observations[MAX_OBSERVATIONS];
        uint8_t length;
    } observation_list_t;

    /* FSM state tree with fsm base */
    typedef struct
    {
        double map[NUM_STATES][NUM_STATES];
        uint8_t length;
    } fsm_map_t;

    typedef struct
    {
        double
        lower_boundary,
        upper_boundary,
        variance;
        vec2_t
        true_center;
    } band_t;

    typedef struct
    {
        uint8_t length;
        band_t band[NUM_STATES];
    } band_list_t;

    static floating_t WeightedAverage( floating_t a, floating_t b, floating_t w )
   {
       return ( ( a * w ) + ( b * ( 1 - w ) ) );
   }

#ifdef __cplusplus
}
#endif

#endif /* control_types_h */
