/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  File: rho_utility.h
 *  Group: Rho Core
 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

#ifndef rho_utility_h
#define rho_utility_h

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *                             Includes                                 *
 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "../types/rho_types.h"
#include "timestamp.h"

#define RHO_UTILITY_NAME "RhoUtility"
#define RHO_UTILITY_NAME_BUILDER(DATA) NAME_BUILDER(RHO_UTILITY_NAME, DATA)

#ifdef __USE_DECOUPLING__
#include "rho_deco.h"
#endif
#ifdef __cplusplus
extern "C" {
#endif
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *                          Static Buffers                              *
 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
  static sdensity_t
    FOREGROUND_DENSITY_MAP_Y[DENSITY_MAP_W_SIZE],
    FOREGROUND_DENSITY_MAP_X[DENSITY_MAP_H_SIZE],
    BACKGROUND_DENSITY_MAP_Y[DENSITY_MAP_W_SIZE],
    BACKGROUND_DENSITY_MAP_X[DENSITY_MAP_H_SIZE],
    BOUND_DENSITY_MAP_Y[DENSITY_MAP_W_SIZE],
    BOUND_DENSITY_MAP_X[DENSITY_MAP_H_SIZE];

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *                       Function Declarations                          *
 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    typedef struct
    {
        void (*Data)( rho_core_t *, uint16_t, uint16_t );
        void (*Filters)( rho_core_t * );
        void (*Prediction)( prediction_t *, const char *, uint16_t );
        void (*DensityMap)( density_map_t *, const char *, uint16_t, uint16_t );
    } rho_utility_initializer_functions;

    typedef struct
    {
        void (*Detect)( rho_detection_variables *, density_map_t *, prediction_t * );
        void (*Prediction)( prediction_predict_variables *, prediction_pair_t *, index_pair_t );
        void (*DensityMapPairKalmans)( rho_core_t * );
    } rho_utility_reset_functions;

    typedef struct
    {
        void (*PeakFilter)( rho_detection_variables *, density_map_t *, prediction_t * );
        void (*CombineProbabilities)( prediction_pair_t * );
        void (*UpdateCorePredictionData)( prediction_predict_variables *, rho_core_t * );
        void (*GenerateObservationList)( prediction_t *, uint8_t );
        void (*GenerateObservationLists)( rho_core_t * );
        void (*ReportObservationLists)( rho_core_t * );
        void (*UpdatePredictiveStateModelPair)(rho_core_t * );
    } rho_utility_predict_functions;

    typedef struct
    {
        void (*Perform)( rho_detection_variables *, density_map_t *, prediction_t * );
        bool (*LowerBound)( rho_detection_variables * );
        void (*Regions)( rho_detection_variables *, density_map_t *, prediction_t *);
        void (*Region)( rho_detection_variables *, density_map_t *, prediction_t * );
#ifdef __USE_ZSCORE_THRESHOLD__
        uint16_t (*ZLower)( rho_detection_variables * );
        bool (*ZRegion)( rho_detection_variables *, bool );
#endif
        void (*SubtractBackground)( rho_detection_variables *, sdensity_t );
        void (*CalculateChaos)( rho_detection_variables *, prediction_t * );
        void (*ScoreRegions)( rho_detection_variables *, density_map_t *, prediction_t * );
        void (*SortRegions)( rho_detection_variables *, prediction_t * );
        void (*Centroid)( rho_detection_variables *, density_map_t * );
        void (*CalculateFrameStatistics)( rho_detection_variables *, prediction_t * );
    } rho_utility_detect_functions;

    typedef struct
    {
        uint16_t (*PredictionCenter)( uint16_t, uint16_t, uint16_t );
        void (*Tune)( rho_core_t * );
        void (*BackgroundTuneFactor)( rho_core_t * );
        void (*StateTuneFactor)( rho_core_t * );
        void (*TargetTuneFactor)( rho_core_t * );
        void (*TargetCoverageFactor)( rho_core_t * );
        void (*CumulativeMoments)( floating_t, floating_t, floating_t *, floating_t *, floating_t * );
        void (*CumulativeAverage)( floating_t, floating_t *, uint16_t * );
        void (*CumulateAverageStandardDeviation)( floating_t, cumulative_avg_stdv_t * );
        floating_t (*Variance)( cumulative_avg_stdv_t * );
        void (*RegionScore)( region_t *, density_t, byte_t );
        density_2d_t (*Centroid)( sdensity_t *, uint16_t, uint16_t *, density_t );
        void (*Background)( rho_core_t * );
        void (*Packet)( rho_core_t * );
    } rho_utility_calculate_functions;
    
    typedef struct
    {
        void (*Packet)( packet_t *, uint16_t );
    } rho_utility_print_functions;

    typedef struct
    {
        rho_utility_initializer_functions Initialize;
        rho_utility_reset_functions Reset;
        rho_utility_predict_functions Predict;
        rho_utility_detect_functions Detect;
        rho_utility_calculate_functions Calculate;
        rho_utility_print_functions Print;
    } rho_utility_functions;

extern const rho_utility_functions RhoUtility;

#ifdef __cplusplus
}
#endif

#endif
