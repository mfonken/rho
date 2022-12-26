//
//  rho_track.c
//  ots-proto
//
//  Created by Matthew Fonken on 8/4/22.
//

#include "rho_track.h"

#define RHO_TRACK_MIN_SIMILARITY 0.95
#define RHO_TRACK_SHADOW_FIT_BUFFER 0.2
#define RHO_TRACK_SHADOW_FIT_MAX_LEVEL 3
#define RHO_TRACK_SHADOW_FIT_FACTOR 100
#define RHO_TRACK_SHADOW_FIT_FACTORED_BUFFER (density_2d_t)( RHO_TRACK_SHADOW_FIT_BUFFER * RHO_TRACK_SHADOW_FIT_FACTOR )
#define RHO_TRACK_WEIGHT_PUNISHMENT_FACTOR 0.5

static void PRINT_PAIR_WEIGHTS( floating_t a[MAX_TRACKERS][MAX_TRACKERS] )
{
    LOG_RHO_TRK(DEBUG_RHO_TRK, "Pair Weights:\n");
    LOG_RHO_TRK(DEBUG_RHO_TRK, "  ");
    for(int y = 0; y < MAX_TRACKERS; y++)
        LOG_RHO_TRK_BARE(DEBUG_RHO_TRK, "  %2d  ", y);
    LOG_RHO_TRK_BARE(DEBUG_RHO_TRK, "\n");
    for(int x = 0; x < MAX_TRACKERS; x++)
    {
        LOG_RHO_TRK(DEBUG_RHO_TRK, "%d: ", x);
        for(int y = 0; y < MAX_TRACKERS; y++)
            LOG_RHO_TRK_BARE(DEBUG_RHO_TRK, "%.3f ", a[x][y]);
        LOG_RHO_TRK_BARE(DEBUG_RHO_TRK, "\n");
    }
    LOG_RHO_TRK_BARE(DEBUG_RHO_TRK, "\n");
}

// Prediction: Input-regions | Output-tracked predictions
void RhoTrack_PredictTrackRegions( prediction_t * prediction )
{
    prediction->num_trackers = 0;
    uint16_t valid_tracks = RhoTrack.SortActiveTrackers( prediction );
    LOG_RHO( RHO_DEBUG_PREDICT_2, "Found %d valid/active tracking filter(s)\n", valid_tracks);

    region_t * r;
    tracker_t * t;
    bool passed[MAX_TRACKERS] = { false };
    byte_t ti = 0, ri = 0;

    if( prediction->num_regions == 1 )
        printf("!");
    
    // 1. Match trackers to regions.
    for( ri = 0; ri < prediction->num_regions; ri++ )
    {
        r = prediction->regions_order[ri];
        if( r == NULL )
            break;
        r->num_trackers = 0;
        
        LOG_RHO_TRK(DEBUG_RHO_TRK, "Region i:%d l:%d w:%d b:%d\n", ri, r->location, r->width, r->blob_id);
        
        bool matched = false;
        // If no blob match, match to closest
        floating_t best_d = 1e6;
        for( ti = 0; ti < valid_tracks; ti++ )
        {
            t = prediction->trackers_order[ti];
            if( t == NULL )
                break;
            bool blob_id_match = r->blob_id == t->blob_id;
            floating_t p = Kalman.TestSelf( &t->kalman );
            floating_t d = fabs((floating_t)r->location - p);
            if( !blob_id_match
               || d > MAX_TRACKING_TRAVEL_PX || d >= best_d )
                continue;
            LOG_RHO_TRK(DEBUG_RHO_TRK, "\tMatch(%d): %d-%.2f @ %d %c\n", ri, ti, d, t->region == NULL ? -1 : t->region->location, blob_id_match?'y':'n');
            t = prediction->trackers_order[ti];
            t->region = r;
            t->valid = true;
            t->region->tracker_id = ti;
            r->num_trackers++;
            if( d <= EXP_TRACKING_TRAVEL_PX )
                Kalman.Step( &t->kalman, r->location );
            else
                Kalman.Predict( &t->kalman );
            passed[ti] = true;
            best_d = d;
            matched = true;
            if( blob_id_match )
                break;
        }
        if( !matched )
        { // 2. Activate new tracker for region
            for( byte_t ti = 0; ti < MAX_TRACKERS; ti++ )
            {
                if( passed[ti] )
                    continue;
                t = &prediction->trackers[ti];
                prediction->trackers_order[ti] = t;
                prediction->trackers_index[ti] = ti;
                t->region = r;
                t->valid = true;
                t->region->tracker_id = ti;
                t->blob_id = -1;
                Kalman.Step( &t->kalman, r->location );
                t->kalman.x.v = 0;
                LOG_RHO( RHO_DEBUG_PREDICT, "Activating filter at index %d - %d\n", ti, t->region->location );
                passed[ti] = true;
                break;
            }
        }
    }

    for( ti = 0; ti < MAX_TRACKERS; ti++ )
    {
        if( passed[ti] )
            continue;
//        LOG_RHO( RHO_DEBUG_PREDICT_2, "Punishing filter at index %d[%d]\n", prediction->regions[n].tracking_id, n );
        t = prediction->trackers_order[ti];
        if( t == NULL )
            continue;
        if( ri < MAX_TRACKERS )
            t->region = NULL;
        if( RhoTrack.PunishTracker( t ) )
        {
            t->kalman.x.v *= RHO_TRACK_KALMAN_SLOW_FACTOR;
            Kalman.Predict( &t->kalman );
        }
        else
        {
            Kalman.Reset( &t->kalman, 0 );
            prediction->trackers_order[ti] = NULL;
            t->blob_id = -1;
        }
    }

    for( ti = 0; ti < MAX_TRACKERS; ti++ )
    {
        if( passed[ti] )
            prediction->num_trackers++;
    }
//
//    /* Calculate confidence */
//    if( updated )
//    {
//        average_difference = total_difference / (floating_t)updated;
//        if( average_difference > MAX_TRACKING_MATCH_DIFFERENCE ) prediction->probabilities.confidence = 0.;
//        else prediction->probabilities.confidence = TRACKING_MATCH_TRUST * ( 1 - ( average_difference / MAX_TRACKING_MATCH_DIFFERENCE ) );
//    }
//
////    for( byte_t i = 0; i < MAX_TRACKERS; i++ )
////        printf("(%d):[%d]>[%d] ", i, prediction->regions_order[i].index, prediction->trackers_order[i]);
////    printf("\n");
}

uint16_t RhoTrack_SortActiveTrackers( prediction_t * prediction )
{
    uint16_t valid_trackers = 0;
    for( byte_t i = 0; i < MAX_TRACKERS; i++ )
    {
        tracker_t * curr = &prediction->trackers[i];
        curr->valid = false;
        floating_t score = RhoTrack.TrackerScore( curr );
        if( false //ISTIMEDOUT( curr->kalman.t, curr->lifespan, TIME_SEC )
           || ( score < MIN_TRACKING_KALMAN_SCORE ) )
        {
            prediction->trackers[i].valid = false;
            if( curr->kalman.x.p > 0 )
                LOG_RHO(RHO_DEBUG_PREDICT_2, "%s > K%d is invalid %s\n", prediction->name, i, score < MIN_TRACKING_KALMAN_SCORE ? "[Low score]" : "[Expired]");
            continue;
        }
        LOG_RHO(RHO_DEBUG_PREDICT_2, "%s> K%d->score=%.4f\n", prediction->name, i, curr->score);
        
        // Check is similar to another
        for( byte_t ci = 0; ci < valid_trackers; ci++ )
        {
            tracker_t * check = prediction->trackers_order[ci];
            if( curr->region == check->region
               && fabs(curr->kalman.x.p - check->kalman.x.p) < RHO_TRACK_MAX_KALMAN_POSITION_SIMILARITY
               && fabs(curr->kalman.x.v - check->kalman.x.v) < RHO_TRACK_MAX_KALMAN_VELOCITY_SIMILARITY)
            {
                prediction->trackers[i].valid = false;
                continue;
            }
        }
        
        curr->valid = true;
        prediction->trackers_order[valid_trackers++] = curr;
    }
    for( byte_t i = valid_trackers + 1; i < MAX_TRACKERS; i++ )
    {
        prediction->trackers_order[i] = NULL;
    }
    return valid_trackers;
}

/// Trackers/quadrants > blobs_order
void RhoTrack_PairPredictions( rho_core_t * core )
{
    prediction_pair_t * predictions = &core->prediction_pair;
    
    RhoTrack.GeneratePairWeights( core, predictions->pair_weights );
    
    LOG_RHO_TRK(DEBUG_RHO_TRK, "---from pairing---\n");
    PRINT_PAIR_WEIGHTS(predictions->pair_weights);
    
    index_pair_t max = { core->width, core->height };
    predictions->num_blobs = RhoTrack.UpdateBlobs( predictions, max );
    core->density_map_pair.x.num_blobs = predictions->num_blobs; /// TODO: Find proper place for this
    core->density_map_pair.y.num_blobs = predictions->num_blobs;
}

bool RhoTrack_IsInRegion( region_t * r, coord_t c )
{
    coord_t w_half = r->width >> 1;
    return c > r->location - w_half && c < r->location + w_half;
}

void RhoTrack_GeneratePairWeights( rho_core_t * core, floating_t pair_weights[MAX_TRACKERS][MAX_TRACKERS] )
{
    density_2d_t * q_final = RhoTrack.RedistributeDensities( core );
    density_2d_t q_dist[5] = { 0 }, *q = q_dist;
    memcpy( q, q_final, 4 * sizeof(density_2d_t) );
    prediction_t * x = &core->prediction_pair.x;
    prediction_t * y = &core->prediction_pair.y;
    index_pair_t centroid = core->centroid;
    
    int8_t shadow_tag[MAX_TRACKERS]; // [-] x-axis | [+] y-axis
    byte_t num_shadows = 0;
    printf(">>>Q: ");
    for(int i = 0; i < 4; i++) printf("%d|%d ", i, q[i]);
    printf("<<<\n");
    
    LOG_RHO_TRK(DEBUG_RHO_TRK, "---from prev blobs---\n");
    PRINT_PAIR_WEIGHTS( pair_weights );
    
    if( x->num_regions != y->num_regions )
        printf("!");
    
    int cc = 0;
    for( byte_t xi = 0; xi < MAX_TRACKERS/*x->num_trackers*/; xi++ )
    {
        tracker_t * xt = x->trackers_order[xi];
        if( xt == NULL || !xt->valid || xt->region == NULL )
        {
            for( byte_t yi = 0; yi < MAX_TRACKERS; yi++ )
                pair_weights[xi][yi] *= RHO_TRACK_WEIGHT_PUNISHMENT_FACTOR;
            continue;
        }
        region_t * xr = xt->region;
        bool down = xr->location > centroid.y;
        for( byte_t yi = 0; yi < MAX_TRACKERS/*y->num_trackers*/; yi++ )
        {
            tracker_t * yt = y->trackers_order[yi];
            if( yt == NULL || !yt->valid || yt->region == NULL )
            {
                pair_weights[xi][yi] *= RHO_TRACK_WEIGHT_PUNISHMENT_FACTOR;
                continue;
            }
            
            region_t * yr = yt->region;
            floating_t xd = (floating_t)xr->density;
            floating_t yd = (floating_t)yr->density;
            
            bool right = yr->location > centroid.x;
            byte_t qi = ( down << 1 ) + right;
            floating_t qd = (floating_t)q[qi];
            if( RhoTrack_IsInRegion( xr, centroid.y ) )
                qd += (floating_t)q[(!down << 1 ) + right];
            if( RhoTrack_IsInRegion( yr, centroid.x ) )
                qd += (floating_t)q[(down << 1 ) + !right];
            
            floating_t min = MIN( xd, yd );
            floating_t max = MAX( xd, yd );
            floating_t similarity = min / max;
            
            // Pair tracked
            if( xt->blob_id == yt->blob_id && yt->blob_id >= 0 )
            {
                pair_weights[xi][yi] = 1.0;
            }
#ifdef RHO_TRACK_BLOBS
            // Shadow if region is more dense than quadrant.
            else if( max > qd && qd >= min * 0.9 )
            {
                pair_weights[xi][yi] = -similarity;
                bool shadowed = false;
                region_t * r = xd > yd ? xr : yr; /// TODO: Is shadow region checking necessary?
                for( byte_t ri = 0; ri < MAX_TRACKERS; ri++ )
                {
                    if( shadow_region[ri] == r )
                    {
                        shadowed = true;
                        break;
                    }
                }
                if( !shadowed )
                {
                    shadow_tag[num_shadows] = xd > yd ? xi + 1 : -( yi + 1 );
                    shadow_region[num_shadows++] = r;
                }
            }
#endif
            // Match if x and y densities are similar.
            else if( qd >= max && similarity >= RHO_TRACK_MIN_SIMILARITY )
            {
                pair_weights[xi][yi] = similarity;
            }
            else
                pair_weights[xi][yi] *= RHO_TRACK_WEIGHT_PUNISHMENT_FACTOR;
            cc += pair_weights[xi][yi] >= RHO_TRACK_MIN_SIMILARITY;
        }
    }
    LOG_RHO_TRK(DEBUG_RHO_TRK, "---from similarity---\n");
    PRINT_PAIR_WEIGHTS(pair_weights);
    if( cc > 2 )
        printf("!");
    if( num_shadows > 0 )
        RhoTrack.Deshadow( shadow_tag, num_shadows, pair_weights, x, y );
}


void RhoTrack_PredictTrackingProbabilities( prediction_t * prediction )
{
    floating_t a = prediction->nu_regions+1, b = (floating_t)NUM_STATE_GROUPS+1, curr_CDF, prev_CDF = 0.,
    interval[4] = STATE_KUMARASWAMY_INTERVALS;
    for( uint8_t i = 0; i < NUM_STATE_GROUPS; i++ )
    {
        curr_CDF = KUMARASWAMY_CDF(interval[i],a,b);
        prediction->probabilities.P[i] = curr_CDF - prev_CDF;
        prev_CDF = curr_CDF;
    }
}

void RhoTrack_CombineAxisProbabilites( prediction_pair_t * prediction )
{
    /* Combine X & Y probabilities with confidence factor */
    double x_confidence, y_confidence;
    for( uint8_t i = 0; i < NUM_STATE_GROUPS; i++ )
    {
        x_confidence = prediction->x.probabilities.confidence * prediction->x.probabilities.P[i];
        y_confidence = prediction->y.probabilities.confidence * prediction->y.probabilities.P[i];
        prediction->probabilities.P[i] = AVG2( x_confidence, y_confidence );
    }
    prediction->probabilities.confidence = AVG2( prediction->x.probabilities.confidence, prediction->y.probabilities.confidence );
}

floating_t RhoTrack_Calculate_TrackerScore( tracker_t * t )
{
    t->score = Kalman.Confidence(&t->kalman);
    return t->score;
}
    
bool RhoTrack_Calculate_PunishTracker( tracker_t * t )
{
    t->kalman.K[0] *= RHO_TRACKER_PUNISH_FACTOR;
    t->valid = RhoTrack.TrackerScore( t ) >= RHO_TRACKER_MIN_SCORE;
    return t->valid;
}

void RhoTrack_Deshadow( int8_t shadow_tag[MAX_TRACKERS], byte_t num_shadows, floating_t pair_weights[MAX_TRACKERS][MAX_TRACKERS], prediction_t * x, prediction_t * y )
{
    for( byte_t si = 0; si < num_shadows; si++ )
    {
        int8_t ti = shadow_tag[si];
        byte_t shadow[MAX_TRACKERS] = { 0 };
        byte_t num_densities = 0;
        byte_t num_in_shadow = 0;
        density_2d_t densities[MAX_TRACKERS] = { 0 };
    
        bool x_axis = ti >= 0;
        byte_t ai = x_axis ? ti - 1 : -ti - 1;
        byte_t bl = x_axis ? y->num_trackers : x->num_trackers;
        bool resolved = false;
        for( byte_t bi = 0; bi < bl; bi++ )
        {
            floating_t w = x_axis ? pair_weights[ai][bi] : pair_weights[bi][ai];
            if( w > 0 )
            {
                resolved = true;
                break;
            }
            densities[bi] = (density_2d_t)( -w * RHO_TRACK_SHADOW_FIT_FACTOR );
        }
        if( resolved )
            continue;
        
        num_densities = bl;
        num_in_shadow = RhoTrack.MinFit( densities, shadow, num_densities, RHO_TRACK_SHADOW_FIT_FACTOR, RHO_TRACK_SHADOW_FIT_FACTORED_BUFFER, RHO_TRACK_SHADOW_FIT_MAX_LEVEL );
        
        if( num_in_shadow == 0 )
            return;
        
        LOG_RHO_TRK(DEBUG_RHO_TRK, "Shadow: ");
        for( byte_t i = 0; i < num_in_shadow; i++ )
        {
            LOG_RHO_TRK_BARE(DEBUG_RHO_TRK, "%d ", shadow[i]);
        }
        LOG_RHO_TRK_BARE(DEBUG_RHO_TRK, "\n");
        
        for( byte_t si = 0; si < num_in_shadow; si++ )
        {
            if( x_axis )
                pair_weights[ai][shadow[si]] = 1;
            else
                pair_weights[shadow[si]][ai] = 1;
        }
    }
}

void RhoTrack_BubbleOrder( density_2d_t a[], byte_t order[], byte_t n);
inline void RhoTrack_BubbleOrder( density_2d_t a[], byte_t order[], byte_t n)
{
    int8_t i = n - 1, j;
    for( ; i >= 0; i-- )
        order[i] = i;
    for( i = 0; i < n - 1; i++ )
        for( j = 0; j < n - i - 1; j++ )
            if( a[order[j]] < a[order[j + 1]] )
                SWAP( order[j], order[j + 1] );
}

int RhoTrack_CombinationSum(density_2d_t a[], const byte_t a_order[], byte_t c[], const density_2d_t target, const density_2d_t buffer, density_2d_t sum, byte_t start, byte_t end, byte_t index, byte_t r )
{
    if( abs( (int)sum - (int)target ) <= buffer )
        return index;
    if( index == r )
        return 0;
    for( byte_t i = start, l = 0; i <= end && end - i + 1 > r - index; i++ )
    {
        c[index] = a_order[i];
        if( ( l = RhoTrack_CombinationSum(a, a_order, c, target, buffer, sum + a[a_order[i]], i + 1, end, index + 1, r ) ) )
            return l;
    }
    return 0;
}

byte_t RhoTrack_MinFit( density_2d_t a[], byte_t out[], byte_t n, density_2d_t target, density_2d_t buffer, byte_t max_level )
{
    byte_t a_order[MAX_TRACKERS];
    RhoTrack_BubbleOrder( a, a_order, n );
    byte_t l = RhoTrack_CombinationSum( a, a_order, out, target, buffer, 0, 0, n, 0, MIN(n, max_level) );
    if( l == 0 )
    {
        if( a[a_order[0]] > 0 )
        {
            out[0] = a_order[0];
            l = 1;
        }
    }
    return l;
}

bool RhoTrack_PairBlob( prediction_pair_t * predictions, byte_t xi, byte_t yi, byte_t * num_blobs, index_pair_t max )
{
    tracker_t * xt = predictions->x.trackers_order[xi];
    if( xt == NULL )
        return false;
    tracker_t * yt = predictions->y.trackers_order[yi];
    if( yt == NULL )
        return false;
    blob_t * b = &predictions->blobs[*num_blobs];
    if( !RhoTrack.UpdateBlob( b, *num_blobs, &max, yt, xt ) )
        return false;
    ++*num_blobs;
    return false;
}

byte_t RhoTrack_UpdateBlobs( prediction_pair_t * predictions, index_pair_t max )
{
    byte_t num_blobs = 0;
    for( byte_t xi = 0; xi < MAX_TRACKERS; xi++ )
    {
        int8_t best_yi = -1.0;
        floating_t best_y = 0.0;
        for( byte_t yi = 0; yi < MAX_TRACKERS; yi++ )
        {
            floating_t v = fabs(predictions->pair_weights[xi][yi]);
            if( v > best_y || v == 1.0 )
            {
                best_y = v;
                if( v == 1.0 )
                {
                    RhoTrack_PairBlob( predictions, xi, yi, &num_blobs, max );
                    if( num_blobs >= MAX_BLOBS )
                        break;
                }
                else
                {
                    best_yi = yi;
                }
            }
        }
        
        if( best_yi == -1 )
            continue;
        
        if( best_y < RHO_TRACK_MIN_SIMILARITY )
            continue;
        
        RhoTrack_PairBlob( predictions, xi, best_yi, &num_blobs, max );
        if( num_blobs >= MAX_BLOBS )
            break;
    }
    if(num_blobs > 2)
        printf("!");
    return num_blobs;
}

floating_t GetBlobAxis( tracker_t * t, coord_t max, floating_t * w, coord_t * x, floating_t * confidence )
{
    if( t->region == NULL )
        return 0.0;
    kalman_t * k = &t->kalman;
    double now = TIMESTAMP(TIME_SEC);
    if( now - k->t_origin < RHO_MIN_TRACKER_AGE_SEC )
        return 0.0;
    floating_t test_x = Kalman.TestSelf( k );
    *w = MAX( t->region->width * ( 1 + RHO_BLOB_PADDING_FACTOR ), 0);// t->region->width + v );
    *x = (coord_t)MIN( max, MAX( 0, (test_x - *w / 2) ) );
    *confidence = Kalman.Confidence( k );
    return true;
}

bool RhoTrack_UpdateBlob( blob_t * b, byte_t i, index_pair_t * max, tracker_t * tx, tracker_t * ty)
{
    coord_t x, y;
    floating_t w, h, cx, cy;
    if( tx == NULL && ty == NULL )
    {
        tx = b->motion.x;
        ty = b->motion.y;
    }
    if( !GetBlobAxis( tx, max->x, &w, &x, &cx ) )
        return false;
    if( !GetBlobAxis( ty, max->y, &h, &y, &cy ) )
        return false;
    
    b->x = x;
    b->y = y;
    b->w = w;
    b->h = h;
    b->motion.x = tx;
    b->motion.y = ty;
    tx->blob_id = i;
    ty->blob_id = i;
    b->confidence = AVG2( cx, cy );
//    printf("-.-.-. (%d, %d) - %dx%d - <%.2f, %.2f>[%.2f] | x%p y%p | %d-%d\n", b->x, b->y, b->w, b->h, b->motion.x->kalman.x.v, b->motion.y->kalman.x.v, b->confidence, &b->motion.x->region, &b->motion.y->region, b->motion.x->region->tracking_id, b->motion.y->region->tracking_id);
    return true;
    /// TODO: figure out why w1xh1, w2xh2 swaps to w2xh1, w1h2
}

/* Perform density redistribution from combining current frame and background */
density_2d_t * RhoTrack_RedistributeDensities( rho_core_t * core )
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
    return core->quadrant_final;
}

const rho_track_functions RhoTrack =
{
    .TrackRegions = RhoTrack_PredictTrackRegions,
    .SortActiveTrackers = RhoTrack_SortActiveTrackers,
    .TrackingProbabilities = RhoTrack_PredictTrackingProbabilities,
    .TrackerScore = RhoTrack_Calculate_TrackerScore,
    .PunishTracker = RhoTrack_Calculate_PunishTracker,
    .MinFit = RhoTrack_MinFit,
    .GeneratePairWeights = RhoTrack_GeneratePairWeights,
    .Deshadow = RhoTrack_Deshadow,
    .PairPredictions = RhoTrack_PairPredictions,
    .UpdateBlobs = RhoTrack_UpdateBlobs,
    .UpdateBlob = RhoTrack_UpdateBlob,
    .RedistributeDensities = RhoTrack_RedistributeDensities
};
