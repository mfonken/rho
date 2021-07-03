//
//  hmm.h
//  hmmtest
//
//  Created by Matthew Fonken on 2/10/19.
//  Copyright © 2019 Matthew Fonken. All rights reserved.
//

#ifdef __PSM__

#ifndef hmm_h
#define hmm_h

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif
    
#include "gmm.h"
    
    void HiddenMarkovModel_Initialize(     hidden_markov_model_t *, const char * );
    void HiddenMarkovModel_InitializeTransitionMatrix( hidden_markov_model_t * );
    uint8_t HiddenMarkovModel_ReportObservationTo( hidden_markov_model_t *, hmm_observation_t );
    void HiddenMarkovModel_BaumWelchSolve( hidden_markov_model_t *, floating_t );
    void HiddenMarkovModel_Print(          hidden_markov_model_t * );
    floating_t HiddenMarkovModel_UpdateAll(    hidden_markov_model_t * );
    void HiddenMarkovModel_UpdateAlpha(    hidden_markov_model_t * );
    void HiddenMarkovModel_UpdateBeta(     hidden_markov_model_t * );
    void HiddenMarkovModel_UpdateGamma(    hidden_markov_model_t * );
    void HiddenMarkovModel_UpdateXi(       hidden_markov_model_t * );
    floating_t HiddenMarkovModel_UpdateProbability( hidden_markov_model_t * );
    void HiddenMarkovModel_UpdateInitialProbabilities( hidden_markov_model_t * );
    void HiddenMarkovModel_UpdateTransitionProbabilities( hidden_markov_model_t * );
    void HiddenMarkovModel_UpdateEmissionProbabilities( hidden_markov_model_t * );
    
    typedef struct
    {
        floating_t (*All)(  hidden_markov_model_t * );
        void (*Alpha)(      hidden_markov_model_t * );
        void (*Beta)(       hidden_markov_model_t * );
        void (*Gamma)(      hidden_markov_model_t * );
        void (*Xi)(         hidden_markov_model_t * );
        floating_t (*Probability)( hidden_markov_model_t * );
        void (*Pi)(         hidden_markov_model_t * );
        void (*A)(          hidden_markov_model_t * );
        void (*B)(          hidden_markov_model_t * );
    } hidden_markov_model_update_functions;
    typedef struct
    {
        void   (*Initialize)(         hidden_markov_model_t *, const char * );
        void   (*InitializeTransitionMatrix)( hidden_markov_model_t * );
        uint8_t (*ReportObservation)( hidden_markov_model_t *, hmm_observation_t );
        void   (*BaumWelchSolve)(     hidden_markov_model_t *, floating_t );
        void   (*Print)(              hidden_markov_model_t * );
        hidden_markov_model_update_functions Update;
    } hidden_markov_model_functions_t;
    
    static const hidden_markov_model_functions_t HiddenMarkovModel_Functions =
    {
        .Initialize = HiddenMarkovModel_Initialize,
        .InitializeTransitionMatrix = HiddenMarkovModel_InitializeTransitionMatrix,
        .ReportObservation = HiddenMarkovModel_ReportObservationTo,
        .BaumWelchSolve = HiddenMarkovModel_BaumWelchSolve,
        .Print = HiddenMarkovModel_Print,
        .Update.All = HiddenMarkovModel_UpdateAll,
        .Update.Alpha = HiddenMarkovModel_UpdateAlpha,
        .Update.Beta = HiddenMarkovModel_UpdateBeta,
        .Update.Gamma = HiddenMarkovModel_UpdateGamma,
        .Update.Xi = HiddenMarkovModel_UpdateXi,
        .Update.Probability = HiddenMarkovModel_UpdateProbability,
        .Update.Pi = HiddenMarkovModel_UpdateInitialProbabilities,
        .Update.A = HiddenMarkovModel_UpdateTransitionProbabilities,
        .Update.B = HiddenMarkovModel_UpdateEmissionProbabilities
    };
    
#ifdef __cplusplus
}
#endif

#endif /* hmm_h */

#endif
