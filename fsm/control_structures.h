//
//  control_structures.h
//  hmmtest
//
//  Created by Matthew Fonken on 2/11/19.
//  Copyright © 2019 Matthew Fonken. All rights reserved.
//

#ifndef control_structures_h
#define control_structures_h

#ifdef __cplusplus
extern "C" {
#endif

#include "control_types.h"

typedef floating_t transition_matrix_t[NUM_STATES][NUM_STATES];
    
/* System self-diagnostic state control type */
typedef struct
{
    state_t         state;
    state_t         prev;
    state_t         next;
    uint8_t         selection_index;
    stability_t     stability;
    transition_matrix_t *P;
    const char *    name;                // Instance name
} fsm_system_t;

/* Proposal elements */
typedef struct
{
    floating_t
        density,
        thresh;
    uint8_t
        num,
        primary_id,
        secondary_id;
} psm_proposal_t;

typedef struct
{
    kumaraswamy_t kumaraswamy;
    band_list_t state_bands;
    psm_proposal_t proposed;
    uint8_t best_state;
    uint8_t best_cluster_id;
    uint8_t observation_state;
    state_t current_state;
    floating_t previous_thresh;
    floating_t best_confidence;
    floating_t best_cluster_weight;
    floating_t state_intervals[NUM_STATE_GROUPS];
    const char * name;                // Instance name
} psm_t;
    
#ifdef __cplusplus
}
#endif

#endif /* control_structures_h */
