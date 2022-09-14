//
//  rho_track.c
//  ots-proto
//
//  Created by Matthew Fonken on 8/4/22.
//

#include "rho_track.h"

void RhoTrack_PairPredictions( rho_core_t * core )
{
    prediction_pair_t * predictions = &core->prediction_pair;
    predictions->num_blobs = 0;
    
    byte_t n = 0;
    byte_t xo[MAX_TRACKERS];
    byte_t yo[MAX_TRACKERS];
    
    for( byte_t i = 0; i < MAX_TRACKERS; i++ )
    {
        xo[i] = i;
        yo[i] = i;
        if( !predictions->x.trackers[xo[i]].valid || !predictions->y.trackers[yo[i]].valid )
            break;
        n++;
    }
    
    // Create combination matrix (n!) of possible pairs
    // If two, immediately pair using centroid and background -> verify on next frame
    // If more, check on next frame -> For each y active, try each x element and fill thresh buffer. Keep best
    byte_pair_t pts[2];
    for( byte_t i = 0; i < n; i++ )
    {
        pts[0] = (byte_pair_t){ xo[i], yo[i] };
        if( n >= 2 && i < 2 )
        {
            i++;
            pts[1] = (byte_pair_t){ xo[i], yo[i] };
            RhoTrack.DisambiguatePair( core, pts );
        }
        else
            RhoTrack.PairXY( predictions, pts[0] );
    }
    
    for( byte_t xi = 0; xi < n; xi++ )
    {
        
    }
}

void RhoTrack_DisambiguatePair( rho_core_t * core, byte_pair_t pts[2] )
{
    prediction_pair_t * predictions = &core->prediction_pair;
    
    floating_t x0 = predictions->x.trackers[pts[0].x].kalman.value;
    floating_t x1 = predictions->x.trackers[pts[1].x].kalman.value;
    
    floating_t y0 = predictions->y.trackers[pts[0].y].kalman.value;
    floating_t y1 = predictions->y.trackers[pts[1].y].kalman.value;
    
//    index_pair_t centroid = core->centroid;
    /// TODO: Decide if centroid check is valid
    RhoTrack.RedistributeDensities( core );
    int8_t quadrant_check = (  core->quadrant_final[0] > core->quadrant_final[1] ) + ( core->quadrant_final[2] < core->quadrant_final[3] ) - 1;

    if( quadrant_check == 0 )
    {
        printf("!"); /// TODO: Make case for quadrant_check == 0
        return;
    }
    else if( ( x0 < x1 ) ^ ( ( quadrant_check > 0 ) ^ ( y0 > y1 ) ) )
        SWAP(pts[0].x, pts[1].x);
    
    core->prediction_pair.descending = quadrant_check > 0;
    
//    if(swap) SWAP(pts[0].y, pts[1].y);
    RhoTrack.PairXY( &core->prediction_pair, pts[0] );
    RhoTrack.PairXY( &core->prediction_pair, pts[1] );
}

void RhoTrack_PairXY( prediction_pair_t * predictions, byte_pair_t pt )
{
    byte_t n = predictions->num_blobs;
    if( n >= MAX_REGIONS) return;
    
    predictions->blobs_order[n].x = pt.x;
    predictions->blobs_order[n].y = pt.y;
    predictions->num_blobs = n + 1;
    /// TODO: Add use of 2D kalman!
}

/* Perform density redistribution from combining current frame and background */
void RhoTrack_RedistributeDensities( rho_core_t * core )
{
    LOG_RHO(RHO_DEBUG_2, "Redistributing densities.\n");
    redistribution_variables _ =
    {
        { core->secondary.x, abs(core->centroid.x-core->secondary.x), core->width - core->centroid.x  },
        { core->secondary.y, abs(core->centroid.y-core->secondary.y), core->height - core->centroid.y },
        { 0 }, 0
    };
    if( core->centroid.x < core->secondary.x )
    {
        _.xl[0] = core->centroid.x;
        _.xl[2] = core->width - core->secondary.x;
        _.c |= 0x01;
    }
    if( core->centroid.y < core->secondary.y )
    {
        _.yl[0] = core->centroid.y;
        _.yl[2] = core->width - core->secondary.y;
        _.c |= 0x02;
    }
    while( _.y < 3 )
        for( _.x = 0; _.x < 3; )
            _.area[_.p++] = _.xl[_.x++] * _.yl[_.y++];
    for( ; _.q < 4; _.q++ )
    {
        _.l  = rlookup.config[_.c].length[    _.q];
        _.l_ = rlookup.config[_.c].length[3 - _.q];
        for( _.x = 0, _.b = 0; _.x < _.l; _.x++ )
        {
            _.a = _.area[rlookup.config[_.c].current[_.q][_.x]];
            for( _.y = 0; _.y < _.l_; _.y++ )
                _.b += _.area[rlookup.config[_.c].background[rlookup.config[_.c].factor[_.q][_.x]][_.y]];
            _.d += ZDIV( _.a, _.b ) * core->quadrant_background[_.q];
        }
#ifndef ALLOW_NEGATIVE_REDISTRIBUTION
        if( _.d > core->quadrant[_.q] ) core->quadrant_final[_.q] = 0;
        else
#endif
            core->quadrant_final[_.q] = core->quadrant[_.q] - _.d;
    }
}
