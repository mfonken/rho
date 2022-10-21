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
//    void RhoUtility_InitializeData( rho_core_t *, index_t, index_t );
//    void RhoUtility_InitializeFilters( rho_core_t * );
//    void RhoUtility_InitializePrediction( prediction_t *, const char *, uint16_t );
//    void RhoUtility_InitializeDensityMap( density_map_t *, const char *, uint16_t, uint16_t );
//
//    void RhoUtility_ResetDetect( rho_detection_variables *, density_map_t *, prediction_t * );
//    void RhoUtility_ResetPrediction( prediction_predict_variables *, prediction_pair_t *, index_pair_t );
//    void RhoUtility_ResetDensityMapPairKalmans( rho_core_t * );
//
//    void RhoUtility_PredictPeakFilter( rho_detection_variables *, density_map_t *, prediction_t * );
//    void RhoUtility_PredictTrackingFilters( prediction_t * );
//    uint16_t RhoUtility_CalculateValidTracks( prediction_t * );
//    void RhoUtility_SortTrackingFilters( prediction_t * );
//    void RhoUtility_PredictTrackingProbabilities( prediction_t * );
//
//    void RhoUtility_DetectPerform( rho_detection_variables *, density_map_t *, prediction_t * );
//    bool RhoUtility_DetectLowerBound( rho_detection_variables * );
//    void RhoUtility_DetectRegions( rho_detection_variables *, density_map_t *, prediction_t * );
//    void RhoUtility_DetectRegion( rho_detection_variables *, density_map_t *, prediction_t * );
//#ifdef __USE_ZSCORE_THRESHOLD__
//    uint16_t RhoUtility_DetectZLower( rho_detection_variables * );
//    bool RhoUtility_DetectZRegion( rho_detection_variables *, bool );
//#endif
//    void RhoUtility_SubtractBackgroundForDetection( rho_detection_variables * );
//    void RhoUtility_CalculateChaos( rho_detection_variables *, prediction_t * );
//    void RhoUtility_ScoreRegions( rho_detection_variables *, density_map_t *, prediction_t * );
//    void RhoUtility_SortRegions( rho_detection_variables *, prediction_t * );
//    void RhoUtility_CalculatedFrameStatistics( rho_detection_variables *, prediction_t * );
//
//    void RhoUtility_CorrectPredictionAmbiguity( prediction_predict_variables *, rho_core_t * );
//    void RhoUtility_CombineAxisProbabilites( prediction_pair_t * );
//    void RhoUtility_UpdateCorePredictionData( prediction_predict_variables *, rho_core_t * );
//
//    uint16_t RhoUtility_CalculatePredictionCenter( uint16_t, uint16_t, uint16_t );
//    void RhoUtility_CalculateTune( rho_core_t * );
//    void RhoUtility_CalculateBackgroundTuneFactor( rho_core_t * );
//    void RhoUtility_CalculateStateTuneFactor( rho_core_t * );
//    void RhoUtility_CalculateTargetTuneFactor( rho_core_t * );
//    void RhoUtility_CalculateTargetCoverageFactor( rho_core_t * core );
//
//    void RhoUtility_Calculate_RegionScore( region_t *, density_t, byte_t );
//    density_2d_t RhoUtility_Calculate_Centroid( sdensity_t *, uint16_t, uint16_t *, density_t );
//    void RhoUtility_PrintPacket( packet_t *, uint16_t );
//    void RhoUtility_Calculate_Background( rho_core_t * );
//    void RhoUtility_Calculate_Packet( rho_core_t * );
//
//    void RhoUtility_GenerateObservationListFromPredictions( prediction_t *, uint8_t );
//    void RhoUtility_GenerateObservationListsFromPredictions( rho_core_t * );
//    void RhoUtility_ReportObservationListsFromPredictions( rho_core_t * );
//    void RhoUtility_UpdatePredictiveStateModelPair( rho_core_t * );

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
        void (*TrackingFilters)( prediction_t * );
        uint16_t (*CalculateValidTracks)( prediction_t * );
        void (*SortFilters)( prediction_t * );
        void (*TrackingProbabilities)( prediction_t * );
        void (*CorrectAmbiguity)( prediction_predict_variables *, rho_core_t * );
        void (*CombineProbabilities)( prediction_pair_t * );
//        void (*RedistributeDensities)(  rho_core_t * );
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
        floating_t (*TrackerScore)( tracker_t * );
        void (*PunishTracker)( tracker_t * );
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
