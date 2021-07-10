//
//  fsm.hpp
//  tau+
//
//  Created by Matthew Fonken on 2/8/18.
//  Copyright © 2019 Marbl. All rights reserved.
//

#ifndef fsm_h
#define fsm_h

#include <stdint.h>
#include <stdbool.h>

#include "control_structures.h"

#ifdef __cplusplus
extern "C" {
#endif
   
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    /*                          DEFINITIONS & MACROS                                       */
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
//#define FSM_DECAY_INACTIVE
    
    static inline uint8_t stateToSelection(uint8_t s) {return ((uint8_t)((s+1)/2) - 1);};
    static inline const char *stateString(int8_t s)
    {
        static const char *strings[] = {
            "UN",
//            "NP",
            "UP",
            "TP",
            "OP",
            "CH"
        };
        return strings[(uint8_t)s+1];
    }
    
    
#define STATE_DECAY                     0.95
#define STATE_PUNISH                    0.025
    
#define FSM_LIFESPAN                    10.
#define FSM_STATE_LIFESPAN              3.
#define FSM_STABILITY_VALUE_UNCERTAINTY 0.04
#define FSM_STABILTIY_BIAS_UNCERTAINTY  0.1
#define FSM_STABILITY_INPUT_UNCERTAINTY 0.4
#define FSM_STABLIITY_UNCERTAINTY       (kalman_uncertainty_c){ FSM_STABILITY_VALUE_UNCERTAINTY, FSM_STABILTIY_BIAS_UNCERTAINTY, FSM_STABILITY_INPUT_UNCERTAINTY }
#define FSM_STATE_VALUE_UNCERTAINTY     0.04
#define FSM_STATE_BIAS_UNCERTAINTY      0.1
#define FSM_STATE_INPUT_UNCERTAINTY     0.4
#define FSM_STATE_UNCERTAINTY           (kalman_uncertainty_c){ FSM_STATE_VALUE_UNCERTAINTY, FSM_STATE_BIAS_UNCERTAINTY, FSM_STATE_INPUT_UNCERTAINTY }
    
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    
    /*~ Goals ~~
     *  - Column is current state
     *  - Row is next state
     *  - Diagonal is probability of remaining in state, should tend to be most probable
     *  - System should try to stay in S2
     *  - All probabilities are <= 1.0 (100%) and...
     *  - ...all row probabilities add to <= 1.0 (100%)
     */

    
    void FiniteStateMachine_InitializeMap(          transition_matrix_t *                 );
    void FiniteStateMachine_ResetState(             transition_matrix_t *,       uint8_t  );
    void FiniteStateMachine_NormalizeMap(           transition_matrix_t *                 );
    uint8_t FiniteStateMachine_NormalizeState(      transition_matrix_t *,       uint8_t  );

    void FiniteStateMachine_InitializeSystem(       fsm_system_t *, const char *, transition_matrix_t *, state_t );
    void FiniteStateMachine_DecayInactiveSystem(    fsm_system_t *                        );
    void FiniteStateMachine_UpdateSystem(           fsm_system_t *,    double[NUM_STATES] );
    void FiniteStateMachine_UpdateProbabilities(    fsm_system_t *,    double[NUM_STATES] );
    void FiniteStateMachine_UpdateState(            fsm_system_t *                        );
    void FiniteStateMachine_PrintSystem(            fsm_system_t *                        );
    
    typedef struct
    {
        void (*Initialize)(         transition_matrix_t *);
        void (*ResetState)(         transition_matrix_t *, uint8_t );
        void (*Normalize)(          transition_matrix_t * );
        uint8_t (*NormalizeState)(  transition_matrix_t *, uint8_t );
    } fsm_map_functions_t;
    
    typedef struct
    {
        void (*Initialize)(          fsm_system_t *, const char * , transition_matrix_t *, state_t );
        void (*DecayInactive)(       fsm_system_t *                         );
        void (*UpdateProbabilities)( fsm_system_t *, double[NUM_STATES]     );
        void (*UpdateState)(         fsm_system_t *                         );
        void (*Update)(              fsm_system_t *, double[NUM_STATES]     );
        void (*Print)(               fsm_system_t *                         );
    } fsm_system_functions_t;
    
    typedef struct
    {
        fsm_map_functions_t    Map;
        fsm_system_functions_t Sys;
    } fsm_functions_t;
    
    static const fsm_functions_t FSMFunctions =
    {
        { /* Map functions */
            .Initialize             = FiniteStateMachine_InitializeMap,
            .Normalize              = FiniteStateMachine_NormalizeMap,
            .NormalizeState         = FiniteStateMachine_NormalizeState,
            .ResetState             = FiniteStateMachine_ResetState
        },
        { /* System functions */
            .Initialize             = FiniteStateMachine_InitializeSystem,
            .DecayInactive          = FiniteStateMachine_DecayInactiveSystem,
            .Update                 = FiniteStateMachine_UpdateSystem,
            .UpdateProbabilities    = FiniteStateMachine_UpdateProbabilities,
            .UpdateState            = FiniteStateMachine_UpdateState,
            .Print                  = FiniteStateMachine_PrintSystem
        }
    };
    
#ifdef __cplusplus
}
#endif

#endif /* fsm_hpp */
