//
//  fsm.cpp
//  Finite State Machine
//
//  Created by Matthew Fonken on 2/8/18.
//  Copyright © 2019 Marbl. All rights reserved.
//

#include "fsm.h"

void FiniteStateMachine_InitializeMap( transition_matrix_t * P )
{
    for(uint8_t i = 0; i < NUM_STATES; i++ )
        FSMFunctions.Map.ResetState( P, i );
}

void FiniteStateMachine_ResetState( transition_matrix_t * P, uint8_t i )
{
    for( uint8_t j = 0; j < NUM_STATES; j++ )
        (*P)[i][j] = 0.0;
    (*P)[i][i] = 1.0;
}

void FiniteStateMachine_NormalizeMap( transition_matrix_t * P )
{
    for( uint8_t i = 0; i < NUM_STATES; i++ )
        FSMFunctions.Map.NormalizeState( P, i );
}

uint8_t FiniteStateMachine_NormalizeState( transition_matrix_t * P, uint8_t i )
{
    uint8_t max_index = i, j;
    double total = 0, invtotal, curr, max = 0;
    for( j = 0; j < NUM_STATES; j++ )
    {
        curr = (*P)[i][j];
        if( curr >= 0. )
            total += curr;
    }
    if(total)
    {
        invtotal = ZDIV( 1., total );
        for( j = 0; j < NUM_STATES; j++ )
        {
            (*P)[i][j] *= invtotal;
            if( (*P)[i][j] > max )
            {
                max = (*P)[i][j];
                max_index = j;
            }
        }
    }
    else (*P)[i][i] = 1.0;
    
    return max_index;
}

void FiniteStateMachine_InitializeSystem( fsm_system_t * sys, const char * name, transition_matrix_t * P, state_t initial_state )
{
    sys->name               = name;
    sys->state              = initial_state;
    sys->prev               = UNKNOWN_STATE;
    sys->next               = UNKNOWN_STATE;
    sys->selection_index    = 0;
    sys->P                  = P;
    
    Kalman.Init( &sys->stability.system, 0., FSM_STATE_VALUE_UNCERTAINTY, FSM_STATE_INPUT_UNCERTAINTY, FSM_LIFESPAN, 0., 1. );
    Kalman.Init( &sys->stability.state, 0., FSM_STATE_VALUE_UNCERTAINTY, FSM_STATE_INPUT_UNCERTAINTY, FSM_STATE_LIFESPAN, 0., 1. );
    
    if( sys->P != NULL )
        FSMFunctions.Map.Initialize( sys->P );
}

void FiniteStateMachine_DecayInactiveSystem( fsm_system_t * sys )
{
    state_t c = sys->state;
    if( c == UNKNOWN_STATE ) return;
    for( uint8_t i = 0, j; i < NUM_STATES; i++ )
    {
        uint8_t state_distance = abs( i - c );
        for( j = 0; j < NUM_STATES; j++ ) // Punish based on relevance(distance) from current state
        {
            (*sys->P)[i][j] -= STATE_PUNISH * state_distance;
            if( (*sys->P)[i][j] < 0 ) (*sys->P)[i][j] = 0.;
        }
    }
}

void FiniteStateMachine_UpdateSystem( fsm_system_t * sys, double p[NUM_STATES] )
{
    FSMFunctions.Sys.UpdateProbabilities( sys, p );
    
    /* Normalize state and find most probable next state */
    sys->next = (state_t)FSMFunctions.Map.NormalizeState( sys->P, sys->state );
    
    FSMFunctions.Sys.UpdateState( sys );
    FSMFunctions.Sys.Print(       sys );
    
    sys->stability.system.t = TIMESTAMP(KALMAN_TIME_UNITS);
    sys->stability.state.t = TIMESTAMP(KALMAN_TIME_UNITS);
}

void FiniteStateMachine_UpdateProbabilities( fsm_system_t * sys, double p[NUM_STATES] )
{
    if( sys->P == NULL )
        return;
    state_t c = sys->state;

    LOG_FSM( FSM_DEBUG_UPDATE, "%s: Update probabilities are [%5.3f, %5.3f, %5.3f, %5.3f].\n", sys->name, p[0], p[1], p[2], p[3]);
    LOG_FSM( FSM_DEBUG_UPDATE, "State %s has stability %.4f\n", stateString(c), sys->stability.state.value );

    floating_t curr = 0;
    for( uint8_t i = 0; i < NUM_STATES; i++ )
    {
        LOG_FSM(FSM_DEBUG_UPDATE, "Updating %s by %.2f.\n", stateString(i), p[i]);
        curr = WeightedAverage( (*sys->P)[c][i], p[i], ( sys->stability.state.x.p + 1 ) / 2 );
        if( curr <= MAX_SINGLE_CONFIDENCE )
            (*sys->P)[c][i] = curr;
    }
    
//    floating_t state_change_rate = TIMESTAMP(KALMAN_TIME_UNITS) - sys->stability.state.t;
    Kalman.Step( &sys->stability.state, p[sys->state] );//, state_change_rate );
}

void FiniteStateMachine_UpdateState( fsm_system_t * sys )
{
    /* State change */

    LOG_FSM(FSM_DEBUG, "~~~ State is %s ~~~\n", stateString(sys->next));
    if( sys->next != sys->state && sys->next != UNKNOWN_STATE )
    {
        LOG_FSM(FSM_DEBUG, "Updating state from %s to %s\n", stateString((int)sys->state), stateString((int)sys->next));
        sys->prev   = sys->state;
        sys->state  = sys->next;
        sys->next   = UNKNOWN_STATE;
        
        Kalman.Reset( &sys->stability.state, 0. );
        
//        floating_t system_change_rate = TIMESTAMP(KALMAN_TIME_UNITS) - sys->stability.system.t;
        Kalman.Step( &sys->stability.system, (*sys->P)[sys->state][sys->state] );//, system_change_rate );
        
#ifdef FSM_DECAY_INACTIVE
        FSMFunctions.Sys.DecayInactive( sys );
#endif
    }
}

void FiniteStateMachine_PrintSystem( fsm_system_t * sys )
{
#ifdef FSM_DEBUG_PRINT
    LOG_FSM(FSM_DEBUG_PRINT, "%s:\n", sys->name );
    LOG_FSM(FSM_DEBUG_PRINT, "t+1  t->" );// "\t\t\t  ");
    for( uint8_t i = 0; i < NUM_STATES; i++ ) LOG_FSM_BARE(FSM_DEBUG_PRINT, "%s-[%d]\t  ", stateString((uint8_t)i), i);
    LOG_FSM_BARE(FSM_DEBUG_PRINT, "\n");
    for( uint8_t i = 0, j; i < NUM_STATES; i++ )
    {
        LOG_FSM(FSM_DEBUG_PRINT, " %s-[%d]", stateString((uint8_t)i), i);
        for( j = 0; j < NUM_STATES; j++ )
        {
            char c = ((j==(uint8_t)sys->state)?'|':' ');
            LOG_FSM_BARE(FSM_DEBUG_PRINT, "\t%c[%.5f]%c", c, (*sys->P)[j][i], c);
        }
        LOG_FSM_BARE(FSM_DEBUG_PRINT, "\n");
    }
    LOG_FSM_BARE(FSM_DEBUG_PRINT, "\n");
#endif
}
