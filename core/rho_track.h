//
//  rho_track.h
//  ots-proto
//
//  Created by Matthew Fonken on 8/4/22.
//

#ifndef rho_track_h
#define rho_track_h

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *                             Includes                                 *
 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "../types/rho_types.h"
#include "timestamp.h"

#define RHO_TRACK_NAME "RhoTrack"
#define RHO_TRACK_NAME_BUILDER(DATA) NAME_BUILDER(RHO_TRACK_NAME, DATA)

#ifdef __cplusplus
extern "C" {
#endif

#define DEBUG_RHO_TRK DEBUG_2

#ifdef DEBUG_RHO_TRK
#define LOG_RHO_TRK(L, ...) LOG(L, "<RhoTrack> " __VA_ARGS__)
#define LOG_RHO_TRK_BARE(L, ...) LOG_BARE(L, __VA_ARGS__)
#else
#define LOG_RHO_TRK(L, ...)
#define LOG_RHO_TRK_BARE(L, ...)
#endif

#define RHO_TRACK_USE_MAX_VELOCITY_LIMIT
#define RHO_MIN_BLOB_CONFIDENCE 0.2
#define RHO_MIN_BLOB_DENSITY_CONTINUITY 0.8
#define RHO_BLOB_PADDING_FACTOR 3 // 2 //
#define RHO_MIN_TRACKER_AGE_SEC 0
#define RHO_MAX_TRACKER_VELOCITY 100
#define RHO_TRACK_KALMAN_SLOW_FACTOR 0.5
#define RHO_TRACK_MAX_KALMAN_POSITION_SIMILARITY 5
#define RHO_TRACK_MAX_KALMAN_VELOCITY_SIMILARITY 5


typedef struct
{
    void (*TrackRegions)( prediction_t * );
    uint16_t (*SortActiveTrackers)( prediction_t * );
    void (*TrackingProbabilities)( prediction_t * );
    floating_t (*TrackerScore)( tracker_t * );
    bool (*PunishTracker)( tracker_t * );
    byte_t (*MinFit)( density_2d_t [], byte_t [], byte_t, density_2d_t, density_2d_t, byte_t );
    void (*GeneratePairWeights)( rho_core_t *, floating_t [MAX_TRACKERS][MAX_TRACKERS] );
    void (*Deshadow)( int8_t [MAX_TRACKERS], byte_t, floating_t [MAX_TRACKERS][MAX_TRACKERS], prediction_t *, prediction_t * );
    void (*PairPredictions)( rho_core_t * );
    byte_t (*UpdateBlobs)( prediction_pair_t *, index_pair_t );
    bool (*UpdateBlob)( blob_t *, byte_t, index_pair_t *, tracker_t *, tracker_t * );
    density_2d_t* (*RedistributeDensities)( rho_core_t * );
} rho_track_functions;

extern const rho_track_functions RhoTrack;

#ifdef __cplusplus
}
#endif

#endif /* rho_track_h */
