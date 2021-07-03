//
//  psm.h
//  hmmtest
//
//  Created by Matthew Fonken on 2/12/19.
//  Copyright © 2019 Matthew Fonken. All rights reserved.
//
// Description: Predictive State Model

#ifdef __PSM__

#ifndef psm_h
#define psm_h

#ifdef __cplusplus
extern "C" {
#endif
    
#include <stdio.h>
    
#include "hmm.h"
    
    //// TEMP LOCATION
    //#define HEIGHT 700
    //#define WIDTH HEIGHT
    
#define MAX_CLUSTER_Y_VARIANCE 30

#ifdef __RHO__
#define PSM_OBSERVATION_MAX THRESH_MAX
#else
#define PSM_OBSERVATION_MAX (1<<7)
#endif
    
    
    void ProbabilisticStateModel_Initialize(                     psm_t *, const char * );
    void ProbabilisticStateModel_ReportObservations(             psm_t *, observation_list_t *, floating_t, uint8_t );
    void ProbabilisticStateModel_UpdateStateIntervals(           psm_t * );//, floating_t );
    void ProbabilisticStateModel_Update(                         psm_t * );//, observation_list_t *, floating_t, uint8_t );
    void ProbabilisticStateModel_UpdateStateBand(                band_list_t *, uint8_t, int8_t, gaussian2d_t * );
    void ProbabilisticStateModel_DiscoverStateBands(             psm_t *, band_list_t * );
    uint8_t ProbabilisticStateModel_FindMostLikelyHiddenState(   psm_t *, uint8_t, floating_t * );
    void ProbabilisticStateModel_UpdateBestCluster(              psm_t *, band_list_t * );
    uint8_t ProbabilisticStateModel_GetCurrentBand(              psm_t *, band_list_t * );
    void ProbabilisticStateModel_GenerateProposals(              psm_t * );
    
    typedef struct
    {
        void (*Initialize)(                     psm_t *, const char * );
        void (*ReportObservations)(             psm_t *, observation_list_t *, floating_t, uint8_t );
        void (*UpdateStateIntervals)(           psm_t * );//, floating_t );
        void (*Update)(                         psm_t * );//, observation_list_t *, floating_t, uint8_t );
        void (*UpdateStateBand)(                band_list_t *, uint8_t, int8_t, gaussian2d_t * );
        void (*DiscoverStateBands)(             psm_t *, band_list_t * );
        uint8_t (*FindMostLikelyHiddenState)(   psm_t *, uint8_t, floating_t * );
        void (*UpdateBestCluster)(              psm_t *, band_list_t * );
        uint8_t (*GetCurrentBand)(              psm_t *, band_list_t * );
        void (*GenerateProposals)(              psm_t * );
    } psm_functions_t;
    
    static const psm_functions_t PSMFunctions =
    {
        .Initialize                 = ProbabilisticStateModel_Initialize,
        .ReportObservations         = ProbabilisticStateModel_ReportObservations,
        .UpdateStateIntervals       = ProbabilisticStateModel_UpdateStateIntervals,
        .Update                     = ProbabilisticStateModel_Update,
        .UpdateStateBand            = ProbabilisticStateModel_UpdateStateBand,
        .DiscoverStateBands         = ProbabilisticStateModel_DiscoverStateBands,
        .FindMostLikelyHiddenState  = ProbabilisticStateModel_FindMostLikelyHiddenState,
        .UpdateBestCluster          = ProbabilisticStateModel_UpdateBestCluster,
        .GetCurrentBand             = ProbabilisticStateModel_GetCurrentBand,
        .GenerateProposals          = ProbabilisticStateModel_GenerateProposals
    };
    
#ifdef __cplusplus
}
#endif

#endif /* psm_h */

#endif
