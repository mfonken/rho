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

void RhoTrack_PairPredictions( rho_core_t * );
void RhoTrack_DisambiguatePair( rho_core_t *, byte_pair_t[2] );
void RhoTrack_PairXY( prediction_pair_t *, byte_pair_t );
void RhoTrack_RedistributeDensities( rho_core_t * );

typedef struct
{
    void (*PairPredictions)( rho_core_t * );
    void (*DisambiguatePair)( rho_core_t *, byte_pair_t[2] );
    void (*PairXY)( prediction_pair_t *, byte_pair_t );
    void (*RedistributeDensities)( rho_core_t * );
} rho_track_functions;

static const rho_track_functions RhoTrack =
{
    .PairPredictions = RhoTrack_PairPredictions,
    .DisambiguatePair = RhoTrack_DisambiguatePair,
    .PairXY = RhoTrack_PairXY,
    .RedistributeDensities = RhoTrack_RedistributeDensities
};

#ifdef __cplusplus
}
#endif

#endif /* rho_track_h */
