/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  File: rho_utility.c
 *  Group: Rho Core
 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *                             Includes                                 *
 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
#include "rho_utility.h"

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *                       Function Definitions                           *
 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

// ------------------------ Initialization --------------------------- //
void RhoUtility_InitializeData( rho_core_t * core, index_t width, index_t height )
{
    /* reset entire structure */
    memset(core, 0, sizeof(rho_core_t));

    /* Core frame */
    core->width = width;
    core->height = height;

    /* Centroid */
    core->centroid.x  = (floating_t)width /2.;
    core->centroid.y  = (floating_t)height/2.;
    core->primary.x   = core->centroid.x;
    core->primary.y   = core->centroid.y;
    core->secondary.x = core->centroid.x;
    core->secondary.y = core->centroid.y;

    core->target_coverage_factor = FILTERED_COVERAGE_TARGET;

    /* Packet */
    core->packet.header.id       = PACKET_HEADER_ID;
    core->packet.header.includes = PACKET_INCLUDES;
    memset(core->packet.data, 0, sizeof(packet_offset_lookup_t));

    /* Background */
    core->background_counter = 0;
    core->background_period = BACKGROUNDING_PERIOD;

    core->thresh = (double)AVG2( MAX_THRESH, MIN_THRESH );
    core->thresh_byte = (byte_t)core->thresh;

    core->density_map_pair.x.map          = FOREGROUND_DENSITY_MAP_X;
    core->density_map_pair.x.background   = BACKGROUND_DENSITY_MAP_X;
//    core->density_map_pair.x.bound        = BOUND_DENSITY_MAP_X;
    core->density_map_pair.y.map          = FOREGROUND_DENSITY_MAP_Y;
    core->density_map_pair.y.background   = BACKGROUND_DENSITY_MAP_Y;
//    core->density_map_pair.y.bound        = BOUND_DENSITY_MAP_Y;

    KumaraswamyFunctions.Initialize( &core->kumaraswamy, NUM_STATES + 1, (floating_t[])DEFAULT_KUMARASWAMY_BANDS );
    FSMFunctions.Sys.Initialize( &core->state_machine, "A", &core->state_transitions, CHAOTIC );

    core->timestamp = TIMESTAMP_MS();
}

void RhoUtility_InitializeFilters( rho_core_t * core )
{
    /* Threshold Filter */
    RhoPID.Initialize( &core->thresh_filter, DEFAULT_PID_GAIN );

    /* Coverage Filter */
    core->target_coverage_factor  = (floating_t)FILTERED_COVERAGE_TARGET;
    Kalman.Init(&core->target_filter, core->target_coverage_factor, RHO_DEFAULT_VU, RHO_DEFAULT_SU, RHO_TARGET_LS, RHO_TARGET_FILTER_MIN, RHO_TARGET_FILTER_MAX );
}

void RhoUtility_InitializePrediction( prediction_t * prediction, const char * name, index_t length )
{
    prediction->name = name;

    /* Prediction probabilities */
    memset( &prediction->probabilities, 0, sizeof(floating_t) * 4 );
    for(uint8_t i = 0; i < MAX_TRACKERS; i++)
    {
        prediction->trackers_order[i] = i;
        tracker_t * t = &prediction->trackers[i];
        Kalman.Init( &t->kalman, 0., RHO_DEFAULT_VU, RHO_DEFAULT_SU, RHO_PREDICTION_LS, 0, length );
        t->region = NULL;
        t->valid = false;
        t->lifespan = RHO_DEFAULT_LS;
        t->score = 0.0;
    }
    /* Regions */
    for(uint8_t i = 0; i < MAX_REGIONS; i++)
    {
        memset(&prediction->regions[i], 0, sizeof(region_t));
        prediction->regions_order[i] = (order_t){ false, i};
    }
}

void RhoUtility_InitializeDensityMap( density_map_t * density_map, const char * name, uint16_t length, uint16_t centroid )
{
    density_map->name = name;

//    size_t size = sizeof(sdensity_t)*length;
//    memset(density_map->map, 0, size);
//    memset(density_map->background, 0, size);
    density_map->length = length;
    density_map->max[0] = 0;
    density_map->max[1] = 0;
    density_map->centroid = centroid;
    Kalman.Init( &density_map->kalmans[0],  0, RHO_DEFAULT_VU, RHO_DEFAULT_SU, RHO_DEFAULT_LS, 0, length );
    Kalman.Init( &density_map->kalmans[1], 0, RHO_DEFAULT_VU, RHO_DEFAULT_SU, RHO_DEFAULT_LS, 0, length );
}

// ------------------------ Reset --------------------------- //
void RhoUtility_ResetDetect( rho_detection_variables * _, density_map_t * density_map, prediction_t * prediction )
{
    memset( _, 0, sizeof(rho_detection_variables) );

    _->len          = density_map->length;
    _->range[2]     = 0;
    _->range[1]     = density_map->buffer_loc[0] > 0 ? density_map->buffer_loc[0] : density_map->centroid;
    _->range[0]     = density_map->buffer_loc[1] > 0 ? density_map->buffer_loc[1] : density_map->length;

    memset( &prediction->probabilities, 0, sizeof(floating_t)*NUM_STATES );
    memset( &prediction->probabilities, 0, sizeof(floating_t)*NUM_STATES );

    prediction->nu_regions    = 0;
    prediction->total_density = 0;
    prediction->num_regions   = 0;

    for( uint16_t i = 0; i < MAX_REGIONS; i++ )
    {
        memset( &prediction->regions[i], 0, sizeof(region_t) );
        prediction->regions_order[i] = (order_t){ false, i };
    }
//    for( uint16_t i = 0; i < MAX_TRACKERS; i++ )
//        prediction->trackers_order[i] = i;
}

void RhoUtility_ResetDensityMapPairKalmans( rho_core_t * core )
{
    Kalman.Reset( &core->density_map_pair.x.kalmans[0], core->prediction_pair.x.previous_peak[0] );
    Kalman.Reset( &core->density_map_pair.x.kalmans[1], core->prediction_pair.x.previous_peak[1] );
    Kalman.Reset( &core->density_map_pair.y.kalmans[0], core->prediction_pair.y.previous_peak[0] );
    Kalman.Reset( &core->density_map_pair.y.kalmans[1], core->prediction_pair.y.previous_peak[1] );
}

void RhoUtility_DetectPerform( rho_detection_variables * _, density_map_t * density_map, prediction_t * prediction )
{
    _->raw_density_moment = 0;
    _->total_density = 0;
    _->filtered_density = 0;
#ifdef __USE_ZSCORE_THRESHOLD__
    _->z_thresh_factor = 5;
    _->z_stat.max_n = 5;//density_map->length;
#endif
#ifdef __USE_REGION_BOUNDARY_OFFSET__
    memset( _->region_boundaries, 0, sizeof(_->region_boundaries[0])*MAX_REGIONS*2);
    _->region_boundary_index = 0;
#endif

    for( _->cycle = 0, _->cycle_ = 1; _->cycle < 2; _->cycle++, _->cycle_++ )
    {
        _->maximum = 0;
        _->start = _->range[_->cycle];
        _->end = _->range[_->cycle_];
//        _->buffer_loc = density_map->buffer_loc[_->cycle];
        _->offset = density_map->offset[2 - _->cycle_];
        _->recalculation_counter = 0;
        _->recalculate = false;
        
//        if(_->offset > 0)
//        {
//            printf("spoof:");
//            for(int i = 0; i < _->start; i++)
//            {
//                density_map->map[i] = i;
//                printf(" %d", i);
//            }
//            printf("\n");
//        }

        RhoUtility.Predict.PeakFilter( _, density_map, prediction );
        if( RhoUtility.Detect.LowerBound( _ ) )
        {
            do
            {
//                LOG_RHO(RHO_DEBUG_DETECT_2, "%s:%d> Recalc: %d\n", prediction->name, _->cycle, _->recalculation_counter);
                RhoUtility.Detect.Regions( _, density_map, prediction );
                if( !_->recalculate )
                  RhoUtility.Detect.CalculateChaos( _, prediction );
                RhoUtility.Detect.ScoreRegions( _, density_map, prediction );
                if(_->recalculation_counter == 0)
                  _->first_filtered_density = _->filtered_density;
            } while( _->recalculate && ++_->recalculation_counter < MAX_RHO_RECALCULATION_LEVEL );
        }

        prediction->previous_peak[_->cycle] = BOUNDU( _->maximum, _->len );
//        LOG_RHO(RHO_DEBUG_DETECT_2, "%s:%d> Peak: %d\n", prediction->name, _->cycle, prediction->previous_peak[_->cycle]);
        prediction->previous_density[_->cycle] = _->first_filtered_density;
        density_map->max[_->cycle] = _->maximum;
    }
    uint8_t valid_regions = 0;
    for( uint8_t i = 0; i < _->total_regions; i++ )
        if( prediction->regions_order[i].valid ) valid_regions++;
    _->total_regions = valid_regions;
    LOG_RHO(RHO_DEBUG_DETECT_2, "Regions: %d\n", _->total_regions);
}

void RhoUtility_PredictPeakFilter( rho_detection_variables * _, density_map_t * density_map, prediction_t * prediction )
{
    _->filter_peak      = (uint16_t)Kalman.Step( &density_map->kalmans[_->cycle], prediction->previous_peak[_->cycle] )->p;
    _->filter_peak_2    = _->filter_peak << 1;
    _->filter_variance  = BOUND((uint16_t)(RHO_VARIANCE( density_map->kalmans[_->cycle].P[0][0]) ), MIN_VARIANCE, MAX_VARIANCE);
//    density_map->kalmans[_->cycle].variance = _->filter_variance;
}

bool RhoUtility_DetectLowerBound( rho_detection_variables * _ );
inline bool RhoUtility_DetectLowerBound( rho_detection_variables * _ )
{
    _->filter_band_lower = (_->filter_peak > _->filter_variance) * (_->filter_peak - _->filter_variance); // Branchless edit
    return _->filter_variance > 0;
}

void RhoUtility_DetectRegions( rho_detection_variables * _, density_map_t * density_map, prediction_t * prediction );
inline void RhoUtility_DetectRegions( rho_detection_variables * _, density_map_t * density_map, prediction_t * prediction )
{ /* Detect regions - START */
    for( _->x = _->start, _->x_ = (_->offset == 0) * _->x + _->offset; _->x >= _->end;  ) // TODO: Don't use end for x_ here
    {
        _->curr = density_map->map[--_->x];
        --_->x_;
//        printf("[%d]\n", _->x);
//        density_map->bound[_->x] = 0;
        if( !_->recalculate )
        {
            sdensity_t b = density_map->background[_->x_];
            RhoUtility.Detect.SubtractBackground( _, b ); // Also finds peak - required even with black background
        }
        RhoUtility.Detect.Region( _, density_map, prediction );
    }
//    printf("\n");
} /* Detect regions - END */

void RhoUtility_SubtractBackgroundForDetection( rho_detection_variables * _, sdensity_t background_curr );
inline void RhoUtility_SubtractBackgroundForDetection( rho_detection_variables * _, sdensity_t background_curr )
{
    if( _->curr > background_curr ) 
    {
//        printf("%d[%d](%d) ", _->curr, _->x, _->x_);
        _->total_density += _->curr;
        _->raw_density_moment += _->curr * (density_t)_->x_;

        // TODO: Why is this here?
        /* Update max */
        if(_->curr > _->maximum)
            _->maximum = _->curr;

#ifdef USE_BACKGROUNDING
        /* Subtract background */
        _->curr -= background_curr;

        /* Punish values above the filter peak */
        if( ( _->curr > _->filter_peak )
           && ( _->filter_peak_2 > _->curr ) )
            _->curr = _->filter_peak_2 - _->curr; ///TODO: Double-check this
#endif
    }
    else
        _->curr = 0;
}

#ifdef __USE_ZSCORE_THRESHOLD__
inline uint16_t RhoUtility_DetectZLower( rho_detection_variables * _ )
{
    floating_t variance = RhoUtility.Calculate.Variance( &_->z_stat );
    _->z_thresh = (uint16_t)(sqrt(variance) * _->z_thresh_factor);
    return _->z_thresh;
}

inline bool RhoUtility_DetectZRegion( rho_detection_variables * _, bool update )
{
    if(update)
    {
        RhoUtility.Calculate.CumulateAverageStandardDeviation( _->curr, &_->z_stat );
        _->has_stat_update = true;
    }
    if(_->z_stat.n >= _->z_stat.max_n)
    {
        sdensity_t delta = _->curr - (sdensity_t)_->z_stat.avg;
        if( -delta > ( _->has_stat_update ? RhoUtility.Detect.ZLower( _ ) : _->z_thresh ) )
          return false;
    }
    return true;
}
#endif

void RhoUtility_DetectRegion( rho_detection_variables * _, density_map_t * density_map, prediction_t * prediction );
inline void RhoUtility_DetectRegion( rho_detection_variables * _, density_map_t * density_map, prediction_t * prediction )
{
#ifdef __USE_REGION_BOUNDARY_OFFSET__
    bool had_region = _->has_region;
#endif

#ifdef __USE_ZSCORE_THRESHOLD__
        density_map->bound[_->x] = MAX((sdensity_t)(_->z_stat.avg - (floating_t)_->z_thresh), 0) * (RhoUtility.Detect.ZRegion( _, false ) ? 1 : -1);
#endif

    /* Check if CMA value is in band */
    if( _->curr > _->filter_band_lower)
    {
        
        /* reset flag and counter */
#ifdef __USE_ZSCORE_THRESHOLD__
        if(!_->has_region)
            _->z_stat.avg = _->curr;
        _->has_region = RhoUtility.Detect.ZRegion( _, true );
#else
        _->has_region = 1;
#endif
        _->gap_counter = 0;

        /* De-offset valid values */
        _->curr -= _->filter_band_lower;

        /* Process new values into region */
        if( _->curr > 0 )
        {
            RhoUtility.Calculate.CumulativeMoments( (floating_t)_->curr, (floating_t)(_->x_), &_->average_curr, &_->average_moment, &_->average_counter ); /// TODO: Use blob x instead of thresh_loc
        }
        /* Increment width */
        _->width++;
    }

    /* Process completed regions and increment count */
    else if( ++_->gap_counter > RHO_GAP_MAX && _->has_region && _->total_regions < MAX_REGIONS
#ifdef __USE_ZSCORE_THRESHOLD__
            /* Check if region continues below boundary within z-threshold (doesn't drop off) */
            && !( RhoUtility.Detect.ZRegion( _, false ) && ( _->curr > ( (uint16_t)_->z_stat.avg - _->z_thresh ) ) )
#endif
#ifdef __USE_RUNNING_AVERAGE__
    )
    {
        _->current_density = (density_2d_t)_->average_curr;
#else
    && _->average_counter )
    {
        _->current_density = (density_2d_t)( _->average_curr / _->average_counter );
#endif
        _->filtered_density += _->current_density;

        /* Check if new region is dense enough to be saved */
        if( _->current_density > MIN_REGION_DENSITY)
        {
            /* Create new region at secondary */
            uint16_t x_loc = (uint16_t)ZDIV( _->average_moment, _->average_curr );
            uint16_t b_loc = _->offset == 0 ? x_loc : _->start - (_->offset - x_loc);
            uint8_t next_index = _->total_regions;
            if(_->recalculate)
            {
                for( uint8_t i = 0; i < next_index; i++ )
                {
                    if( !prediction->regions_order[i].valid )
                    {
                        next_index = i;
                        break;
                    }
                }
            }
            prediction->regions[prediction->regions_order[next_index].index] = (region_t){ density_map->map[b_loc], _->current_density, x_loc, _->width };
            prediction->regions_order[next_index].valid = true;
            if(next_index == _->total_regions)
                _->total_regions++;
        }

        /* reset variables */
        _->average_moment = 0.; _->average_curr = 0.; _->average_counter = 0.;
        _->has_region = 0; _->gap_counter = 0;

#ifdef __USE_ZSCORE_THRESHOLD__
        _->z_stat.avg = 0.; _->z_stat.S = 0;
        _->z_stat.n = 0; _->z_index = 0;
#endif
    }
    else if (!_->has_region )
    { /* reset width */
        _->width = 0;
    }
#ifdef __USE_REGION_BOUNDARY_OFFSET__
    if(had_region != _->has_region
       && _->region_boundary_index < MAX_REGIONS*2 )
    {
        _->region_boundaries[_->region_boundary_index++] = _->x;
    }
#endif
}

void RhoUtility_CalculateChaos( rho_detection_variables * _, prediction_t * prediction )
{
    _->chaos = MAX( ZDIV( (floating_t)prediction->previous_density[_->cycle], (floating_t)_->filtered_density ), MIN_CHAOS);
//    LOG_RHO(RHO_DEBUG_DETECT_2, "%s:%d> Chaos:%.4f\n", prediction->name, _->cycle, _->chaos);
}

void RhoUtility_ScoreRegions( rho_detection_variables * _, density_map_t * density_map, prediction_t * prediction )
{
    /* Assume no recalculations are needed */
    _->recalculate = false;

    /* Return on empty frames */
    if(_->filtered_density == 0 ) return;

    /* Cycle regions */
    for(uint8_t i = 0; i < _->total_regions; i++)
    {
        /* Get region at index from order array and check validity */
        if(!prediction->regions_order[i].valid) continue;
        uint8_t jo = prediction->regions_order[i].index;
        region_t * curr = &prediction->regions[jo];

        /* Score current region */
        RhoUtility.Calculate.RegionScore( curr, _->filtered_density, _->maximum );

        /* Recalculate regions with chaos */
        floating_t chaos = ( _->recalculation_chaos > 0 && _->recalculation_chaos < ZDIV_LNUM )
            ? _->recalculation_chaos : _->chaos;
        _->recalculation_chaos = ZDIV_LNUM;
        if( curr->score > chaos )
        {
            uint16_t x = (uint16_t)ZDIV( _->average_moment, _->average_curr );
            sdensity_t v = density_map->map[x];
            RhoUtility.Calculate.RegionScore( curr, _->filtered_density, _->maximum );
            
            _->recalculation_chaos = MIN( curr->score, _->recalculation_chaos );
            LOG_RHO(RHO_DEBUG_DETECT_2, "%s:%d> R%d Score: %.4f | Chaos: %.4f\n", prediction->name, _->cycle, jo, curr->score, chaos);
            /* Recalculate around chaotic region */
            _->recalculate = true;

            /* Remove this index */
            prediction->regions_order[i].valid = false;

            /* Get centroid peak and update filter peak double */
            sdensity_t current_peak = (sdensity_t)density_map->map[curr->location];
            _->filter_peak_2 = current_peak << 1;

            /* When below band center, raise lower band half of region centroid */
            if( current_peak < _->filter_peak)
                _->filter_band_lower += abs( (sdensity_t)_->filter_band_lower - current_peak) >> 1;

            /* Otherwise raise to centroid peak */
            else _->filter_band_lower = current_peak;

            /* Recalculate _->cycle width */
            uint16_t half_region_width = curr->width >> 1;
            if(curr->location - half_region_width > _->range[_->cycle_])
                _->end = curr->location - half_region_width;
            else _->end = _->range[_->cycle_];
            if(curr->location + half_region_width < _->range[_->cycle])
                _->start = curr->location + half_region_width;
            else _->start = _->range[_->cycle];
            break;
        }
    }
}

void RhoUtility_SortRegions( rho_detection_variables * _, prediction_t * prediction )
{
    /// NOTE: Smaller scores are better
    /* Assume all regions are valid */
    uint16_t i, io, j, jo, best_index;
    floating_t best_score;
    region_t *curr, *check;

    /* Cycle through found regions */
    for( i = 0; i < _->total_regions; i++)
    {
        if( !prediction->regions_order[i].valid ) continue;
        io = prediction->regions_order[i].index;
        curr = &prediction->regions[io];
        if( curr->sort ) continue;

        best_score = curr->score;
        best_index = i;

        /* Cycle through other regions */
        for( j = i+1; j < _->total_regions; j++ )
        {
            if( !prediction->regions_order[j].valid ) continue;
            jo = prediction->regions_order[j].index;
            check = &prediction->regions[jo];
            if( check->sort ) continue;

            /* If unscored and less than min, set as new min */
            if( check->score < best_score )
            {
                best_score = check->score;
                best_index = j;
            }
        }
        if( best_index == i ) continue;

        prediction->regions_order[i].index = best_index;
        prediction->regions_order[best_index].valid = true;
        prediction->regions_order[best_index].index = i;
        prediction->regions[best_index].sort = true;
//        SWAP(prediction->trackers_order[i], prediction->trackers_order[best_index] );
        
//        curr->sort = true;
        if( curr->score > 100. )
            printf("");
    }
    for(; i < MAX_REGIONS; i++ )
    {
        prediction->regions_order[i] = (order_t){ false, i };
    }

    int c2[MAX_REGIONS] = {-1, -1, -1, -1};
    for( i = 0; i < _->total_regions; i++)
    {
        order_t* o = &prediction->regions_order[i];
        region_t * curr = &prediction->regions[o->index];
        LOG_RHO(RHO_DEBUG_DETECT, "%s> R%d%c:{M%d, D%d, X%d, W%d} = S%.4f\n",
            prediction->name,
            i,
            o->valid?'y':'n',
            curr->maximum,
            curr->density,
            curr->location,
            curr->width,
            curr->score
            );
        c2[i] = o->index;
        bool already_has = false;
        for(int j = i - 1; j >= 0; j--)
        {
            if(c2[j] == c2[i])
            {
                already_has = true;
                LOG_RHO(RHO_DEBUG_DETECT_2, "Already has %d at %d and %d\n", c2[i], i, j);
                break;
            }
        }
    }
}
    
void RhoUtility_Detect_Centroid( rho_detection_variables * _, density_map_t * density_map )
{
    floating_t proposed_center = ZDIV((floating_t)_->raw_density_moment, (floating_t)_->total_density);
#ifdef __USE_REGION_BOUNDARY_OFFSET__
    if(_->region_boundary_index > MAX_REGIONS)
        printf("");
    for( uint8_t i = 0; i < _->region_boundary_index - 1; i += 2)
    {
        if( proposed_center < _->region_boundaries[i]
           && proposed_center > _->region_boundaries[i+1] )
        {
            if( fabs(_->region_boundaries[i] - proposed_center)
               > fabs(_->region_boundaries[i+1] - proposed_center) )
                i++;
            proposed_center = _->region_boundaries[i] + ( RHO_GAP_MAX >> 1 );
            break;
        }
    }
#endif
    density_map->centroid = BOUNDU(proposed_center, density_map->length);

    LOG_RHO(RHO_DEBUG_DETECT, "%s> Centroid: %d | Moment: %d | Density: %d\n", density_map->name, (int)density_map->centroid, _->raw_density_moment, _->total_density);
}

void RhoUtility_CalculatedFrameStatistics( rho_detection_variables * _, prediction_t * prediction )
{
    /* Update frame statistics */
    _->assumed_regions = (floating_t)_->total_regions;
    if( _->assumed_regions == 0. ) _->assumed_regions = 1.;

    prediction->nu_regions = BOUNDU( ZDIV( (floating_t)_->total_density * (floating_t)_->assumed_regions, _->target_density ), MAX_NU_REGIONS );
    prediction->num_regions = _->total_regions;
    prediction->total_density = _->total_density;
    prediction->average_density = ZDIV( (floating_t)prediction->total_density, prediction->nu_regions );

    /* Reset sort flags */
    for( uint8_t i = 0; i < MAX_REGIONS; i++ )
        prediction->regions[i].sort = false;

    LOG_RHO(RHO_DEBUG_DETECT_2, "Total:%d | Target:%d | Frame:%d\n", _->total_density, (int)_->target_density, TOTAL_RHO_PIXELS);
    LOG_RHO(RHO_DEBUG_DETECT_2, "Regions: %d{%.3f}\n", prediction->num_regions, prediction->nu_regions);
}

// Prediction: Input-regions | Output-tracked predictions
    
void RhoUtility_PredictTrackingFilters( prediction_t * prediction )
{
    uint16_t valid_tracks = RhoUtility.Predict.CalculateValidTracks( prediction );
    LOG_RHO( RHO_DEBUG_PREDICT_2, "Found %d valid/active tracking filter(s)\n", valid_tracks);

    /* Declare essential variables */
    floating_t aa, bb, ab, ba, curr_difference = 0., total_difference = 0., average_difference = 0.;

    /* Match regions to Kalmans */
    uint16_t m = 0, n = 0, v = 0, updated = 0;
    while( m < ( valid_tracks - 1 ) && n < ( prediction->num_regions - 1 ) )
    { /* Update tracking filters in pairs following determinant */
        /* Retreive current region pair */
        int8_t rAi = m, rBi = m+1, rAo = -1, rBo = -1;
        rAo = prediction->regions_order[rAi].index;
        rBo = prediction->regions_order[rBi].index;
        
        region_t *regionA = &prediction->regions[rAo];
        region_t *regionB = &prediction->regions[rBo];
        
        /* Retreive tracking filters pair */
        kalman_t *filterA = &prediction->trackers[rAi].kalman;
        kalman_t *filterB = &prediction->trackers[rBi].kalman;

        floating_t testA = Kalman.TestPosition( filterA, filterA->x.p, false );
        floating_t testB = Kalman.TestPosition( filterB, filterB->x.p, false );
        
        /* Calculate distances between filters and regions */
        aa = fabs(testA - regionA->location );
        bb = fabs(testB - regionB->location );
        ab = fabs(testA - regionB->location );
        ba = fabs(testB - regionA->location );

        LOG_RHO(RHO_DEBUG_PREDICT, "%s(%d,%d)> aa:%.3f ab:%.3f ba:%.3f bb:%.3f\n", prediction->name, m, n, aa, ab, ba, bb);
        
        LOG_RHO(RHO_DEBUG_PREDICT, "R[");
        for( int i = 0; i < MAX_TRACKERS; i++ )
            LOG_RHO_BARE(RHO_DEBUG_PREDICT, "%d%c", prediction->regions_order[i].index, (i < MAX_TRACKERS -1 ? '|' : ']'));
        LOG_RHO(RHO_DEBUG_PREDICT, "T[");
        for( int i = 0; i < MAX_TRACKERS; i++ )
            LOG_RHO_BARE(RHO_DEBUG_PREDICT, "%d%c", prediction->trackers_order[i], (i < MAX_TRACKERS -1 ? '|' : ']'));

        bool swapped = false;
        /* Swap on upward determinant */
        if( aa * bb < ab * ba )
        {
            curr_difference = aa + bb;
        }
        else
        {
            swapped = true;
            curr_difference = ab + ba;
        }
        LOG_RHO(RHO_DEBUG_PREDICT, "Predict ∆: %.2f\n", curr_difference);
        total_difference += curr_difference;

        if( curr_difference < MAX_TRACKING_MATCH_DIFFERENCE )
        {
            /* Update filters */
            if(swapped)
            {
                Kalman.Step( filterA, regionB->location );
                Kalman.Step( filterB, regionA->location );
                regionA->tracking_id = rBi;
                regionB->tracking_id = rAi;
                prediction->trackers_order[rAi] = rBo;
                prediction->trackers_order[rBi] = rAo; /// TODO: Finish using tracking order to align with regions instead of alone
            }
            else
            {
                Kalman.Step( filterA, regionA->location );
                Kalman.Step( filterB, regionB->location );
                regionA->tracking_id = rAi;
                regionB->tracking_id = rBi;
                prediction->trackers_order[rAi] = rAi;
                prediction->trackers_order[rBi] = rBi;
            }
            updated += 2;

            LOG_RHO(RHO_DEBUG_PREDICT, "Updating %d: %.4f\n", rAi, filterA->x.p);
            LOG_RHO(RHO_DEBUG_PREDICT, "Updating %d: %.4f\n", rBi, filterB->x.p);
            
            Kalman.Print( filterA );
        }
        m+=2; n+=2;
    }

    /* Account for odd number and spare regions */
    if( m < valid_tracks && n < prediction->num_regions )
    {
        int8_t ri = -1, fi = -1;
        for( ; v < MAX_REGIONS && !prediction->regions_order[v].valid; v++ );
        ri = prediction->regions_order[v++].index;
        region_t *region = &prediction->regions[ri];
        
        while( m < valid_tracks )
        {
            fi = prediction->trackers_order[m];
            kalman_t *filter = &prediction->trackers[fi].kalman;
            floating_t test = Kalman.TestPosition( filter, filter->x.v, false );
            curr_difference = fabs( region->location - test );
            
            LOG_RHO(RHO_DEBUG_PREDICT, "Odd %s(%d,%d)> ∆:%.3f\n", prediction->name, m, n, curr_difference);

            if( curr_difference < MAX_TRACKING_MATCH_DIFFERENCE_SINGLE )
            {
                LOG_RHO(RHO_DEBUG_PREDICT, "Updating %d: %.4f\n", region->tracking_id, filter->x.p);
                Kalman.Step(filter, region->location);
                
                region->tracking_id = fi;
                prediction->trackers_order[m] = ri;

                total_difference += curr_difference;
                updated++;
            }
            m++;
        }
        n++;
    }

    /* Activate new filters */
    for( ; n < prediction->num_regions && n < MAX_REGIONS; n++ )
    {
        int8_t ri = prediction->regions_order[n].index;
        prediction->trackers_order[n] = ri;
        tracker_t * tracker = &prediction->trackers[ri];
        tracker->region = &prediction->regions[ri];
        tracker->valid = true;
        Kalman.Step( &tracker->kalman, tracker->region->location );
        tracker->kalman.x.v = 0;
        LOG_RHO( RHO_DEBUG_PREDICT, "Activating filter at index %d[%d] - %d\n", prediction->trackers_order[n], n, prediction->regions[ri].location );
    }

    /* Punish unused ones */
    for( ; n < MAX_TRACKERS; n++ )
    {
//        LOG_RHO( RHO_DEBUG_PREDICT_2, "Punishing filter at index %d[%d]\n", prediction->tracking_filters_order[m], m );
        RhoUtility.Calculate.PunishTracker( &prediction->trackers[prediction->trackers_order[n]] );
    }
    
    /* Calculate confidence */
    if( updated )
    {
        average_difference = total_difference / (floating_t)updated;
        if( average_difference > MAX_TRACKING_MATCH_DIFFERENCE ) prediction->probabilities.confidence = 0.;
        else prediction->probabilities.confidence = TRACKING_MATCH_TRUST * ( 1 - ( average_difference / MAX_TRACKING_MATCH_DIFFERENCE ) );
    }

//    for( byte_t i = 0; i < MAX_TRACKERS; i++ )
//        printf("(%d):[%d]>[%d] ", i, prediction->regions_order[i].index, prediction->trackers_order[i]);
//    printf("\n");
    
//    RhoUtility.Predict.SortFilters( prediction );
}

void RhoUtility_SortTrackingFilters( prediction_t * prediction )
{
    tracker_t *curr, *check;
    floating_t best_score;
    uint16_t i, io, j, jo, best_index = 0;

    /* Score all filters */
    for( i = 0; i < MAX_TRACKERS; i++ )
        Kalman.Score( &prediction->trackers[i].kalman );

    /* Swap sort - Cycle through found regions */
    for( i = 0; i < MAX_TRACKERS; i++)
    {
        io = prediction->trackers_order[i];
        curr = &prediction->trackers[io];

        best_score = curr->score;
        best_index = i;
        /* Cycle through other regions */
        for( j = i+1; j < MAX_TRACKERS; j++ )
        {
            jo = prediction->trackers_order[j];
            check = &prediction->trackers[jo];

            /* If unscored and less than min, set as new min */
            if( check->score > best_score )
            {
                best_score = curr->score;
                best_index = j;
            }
        }
        prediction->trackers_order[i] = best_index;
        prediction->trackers_order[best_index] = i;
    }
}

uint16_t RhoUtility_CalculateValidTracks( prediction_t * prediction )
{
    uint16_t valid_tracks = 0, i, io;
    for( i = 0; i < MAX_TRACKERS; i++ )
    {
        io = prediction->trackers_order[i];
        tracker_t * curr = &prediction->trackers[io];
        curr->valid = false;
        floating_t score = RhoUtility.Calculate.TrackerScore( curr );// Kalman.Score( curr );
//        if(isnan(score))
//            printf("!\n");
        if( false //ISTIMEDOUT( curr->kalman.t, curr->lifespan, TIME_SEC )//  Kalman.IsExpired( curr )
           || ( score < MIN_TRACKING_KALMAN_SCORE ) )
        {
            prediction->trackers[io].valid = false;
            if( curr->kalman.x.p > 0 )
                LOG_RHO(RHO_DEBUG_PREDICT_2, "%s > K%d is invalid %s\n", prediction->name, io, score < MIN_TRACKING_KALMAN_SCORE ? "[Low score]" : "[Expired]");
            break;
        }
        Kalman.Predict( &curr->kalman );
        LOG_RHO(RHO_DEBUG_PREDICT_2, "%s> K%d->score=%.4f\n", prediction->name, io, curr->score);
        curr->valid = true;
        valid_tracks++;
    }
    return valid_tracks;
}

void RhoUtility_PredictTrackingProbabilities( prediction_t * prediction )
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

// Construct Prediction
void RhoUtility_ResetPrediction( prediction_predict_variables * _, prediction_pair_t * prediction, index_pair_t centroid )
{
//    kalman_t * yk, * xk;
//    uint8_t yi, xi;
//    for( byte_t i = 0; i < 2; i++ )
//    {
//        yi = prediction->y.trackers_order[i];
//        xi = prediction->x.trackers_order[i];
//        yk = &prediction->y.trackers[yi].kalman;
//        xk = &prediction->x.trackers[xi].kalman;
//        _->pts[i] = (index_pair_t){ yk->value, xk->value };
//    }
//    _->centroid  = (index_pair_t){ centroid.x, centroid.y };
}
    
void RhoUtility_CombineTrackers( rho_core_t * core )
{
    
}

void RhoUtility_CorrectPredictionAmbiguity( prediction_predict_variables * _, rho_core_t * core )
{
//    index_pair_t centroid = core->centroid;
//
//    /* Check if X or Y are ambiguous */
//    if(   !( ( _->pts[0].x < _->centroid.x ) ^ ( _->pts[1].x > _->centroid.x ) )
//       || !( ( _->pts[0].y < _->centroid.y ) ^ ( _->pts[1].y > _->centroid.y ) ) )
//    {
//        RhoUtility.Predict.RedistributeDensities( core );
//        _->quadrant_check = (  core->quadrant_final[0] > core->quadrant_final[1] ) + ( core->quadrant_final[2] < core->quadrant_final[3] ) - 1;
//        if( ( _->pts[0].x > _->pts[1].x ) ^ ( ( _->quadrant_check > 0 ) ^ ( _->pts[0].y < _->pts[1].y ) ) )
//        {
//            SWAP(_->pts[0].x, _->pts[1].x);
//        }
//    }
//
//    // Note descending or ascending
//    core->prediction_pair.descending = ( _->pts[0].x < _->pts[1].x ) ^ (_->pts[0].y > _->pts[1].y );
}

void RhoUtility_CombineAxisProbabilites( prediction_pair_t * prediction )
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

void RhoUtility_UpdateCorePredictionData( prediction_predict_variables * _, rho_core_t * core )
{
    _->centroid.x = (uint16_t)core->density_map_pair.y.centroid;
    _->centroid.y = (uint16_t)core->density_map_pair.x.centroid;
    
    LOG_RHO(RHO_DEBUG_UPDATE_2, "Primary (%d, %d) | Secondary (%d, %d)\n", _->pts[0].x, _->pts[0].y, _->pts[1].x, _->pts[1].y);
    LOG_RHO(RHO_DEBUG_UPDATE_2, "Centroid.x>%d %c Centroid.y>%d \n", _->centroid.x, core->prediction_pair.descending ? '\\' : '/', _->centroid.y);
    LOG_RHO(RHO_DEBUG_UPDATE_2, "Q[%d / %d | %d / %d]\n", core->quadrant_final[0], core->quadrant_final[1], core->quadrant_final[2], core->quadrant_final[3]);

    /* NOTE: density maps invert axes */
    core->density_map_pair.y.centroid         = _->centroid.x;
    core->density_map_pair.x.centroid         = _->centroid.y;
    core->prediction_pair.y.previous_centroid = _->centroid.x;
    core->prediction_pair.x.previous_centroid = _->centroid.y;
    
//    core->primary   = _->pts[0];
//    core->secondary = _->pts[1];
    core->centroid  = _->centroid;
}

uint16_t RhoUtility_CalculatePredictionCenter( uint16_t primary, uint16_t secondary, uint16_t width )
{
    uint16_t sum = primary + secondary;
    if( primary == 0 || secondary == 0 ) return sum;
    else return BOUNDU(sum >> 1, width );
}

void RhoUtility_CalculateTune( rho_core_t * core )
{
    /* Background-Tune on significant background */
    RhoUtility.Calculate.BackgroundTuneFactor( core );

    /* State-Tune by FSM state */
    RhoUtility.Calculate.StateTuneFactor( core );

    /* Filtered-Tune on target difference */
    RhoUtility.Calculate.TargetTuneFactor( core );

    core->tune.proposed = BOUND( core->tune.background + core->tune.state + core->tune.target, -THRESH_STEP_MAX, THRESH_STEP_MAX);

    core->thresh = BOUND(core->thresh + core->tune.proposed, THRESH_MIN, THRESH_MAX);
//    if(core->PredictiveStateModelPair.proposed_threshold > 0)
//        core->Thresh = WeightedAverage(core->PredictiveStateModelPair.proposed_threshold, core->Thresh, 0.5);
    core->thresh_byte = (byte_t)core->thresh;
}

void RhoUtility_CalculateBackgroundTuneFactor( rho_core_t * core )
{
    floating_t background_tune_factor = 0.;
    if( core->quadrant_background_total > BACKGROUND_COVERAGE_MIN )
    {
        floating_t background_coverage_factor = 1 - ZDIV( BACKGROUND_COVERAGE_MIN, core->quadrant_background_total );
        background_tune_factor = -pow( BOUND(background_coverage_factor, -BACKGROUND_TUNE_MAX, BACKGROUND_TUNE_MAX), BACKGROUND_TUNE_EXPONENT);
    }
    core->tune.background = background_tune_factor;
}

void RhoUtility_CalculateStateTuneFactor( rho_core_t * core )
{
    core->target_coverage_factor = core->target_filter.x.p;
    core->prediction_pair.average_density = MAX( core->prediction_pair.x.average_density, core->prediction_pair.y.average_density );
    LOG_RHO(RHO_DEBUG_UPDATE, "Filtered|Total %%: %.7f|%.7f\n", core->filtered_percentage, core->total_percentage);

//#ifdef __PSM__
////    Kalman.Step( &core->TargetFilter, core->TotalPercentage, 0. );
//    core->TargetFilter.value = core->TotalPercentage;
////    Kalman.Print( &core->TargetFilter );
////    switch(-1)
//
//    LOG_RHO( RHO_DEBUG_UPDATE, "Current state: %s\n", stateString(core->PredictiveStateModelPair.current_state));
//    switch(core->PredictiveStateModelPair.current_state)
//#else
    LOG_RHO(RHO_DEBUG_UPDATE, "Current state: %s\n", stateString(core->state_machine.state));
    switch(core->state_machine.state)
//#endif
    {
        default:
            RhoUtility.Calculate.TargetCoverageFactor( core );
            //RhoUtility.Reset.DensityMapPairKalmans( core );
            break;
        case TARGET_POPULATED:
            Kalman.Step( &core->target_filter, core->total_percentage );
            Kalman.Print( &core->target_filter );
        case OVER_POPULATED:
        case UNDER_POPULATED:
            RhoUtility.Calculate.TargetCoverageFactor( core );
            break;
    }

    RhoPID.Update( &core->thresh_filter, core->target_coverage_factor, core->target_filter.x.p );
    core->tune.state = core->thresh_filter.value;

    LOG_RHO(RHO_DEBUG_2, "Avg:%d | Nu:%.4f\n", (int)core->prediction_pair.average_density, core->prediction_pair.nu_regions);
    LOG_RHO(RHO_DEBUG_2, "Target cov.:%.4f | Target val: %.4f | Thresh val:%.4f\n", core->target_coverage_factor, core->target_filter.x.p, core->thresh_filter.value);
}

void RhoUtility_CalculateTargetTuneFactor( rho_core_t * core )
{
    core->tune.target = TARGET_TUNE_FACTOR * ZDIV( core->thresh_filter.value, core->target_filter.x.p );
}

void RhoUtility_CalculateTargetCoverageFactor( rho_core_t * core )
{
    if( core->prediction_pair.probabilities.confidence > MIN_STATE_CONFIDENCE )
        core->target_coverage_factor = core->prediction_pair.nu_regions * core->prediction_pair.average_density / (floating_t)TOTAL_RHO_PIXELS;
    else
        core->target_coverage_factor = core->total_coverage / (floating_t)TOTAL_RHO_PIXELS;
#ifdef __PSM__
    if( core->predictive_state_model_pair.proposed_avg_den > 0 && core->predictive_state_model_pair.best_confidence > MIN_STATE_CONFIDENCE )
    {
        LOG_RHO(RHO_DEBUG_2, "Proposed: Num - %d | Density - %.2f | State - %s\n", core->predictive_state_model_pair.proposed_num, core->predictive_state_model_pair.proposed_avg_den, stateString(core->predictive_state_model_pair.current_state));
//        core->TargetCoverageFactor = core->PredictiveStateModelPair.proposed_num * core->PredictiveStateModelPair.proposed_avg_den / (floating_t)TOTAL_RHO_PIXELS;
    }
#endif
}


void RhoUtility_Calculate_RegionScore( region_t * region, density_t total_density, byte_t peak )
{
    floating_t delta_d = ZDIV((floating_t)region->density, (floating_t)total_density) - 0.5;
    floating_t delta_p = ZDIV((floating_t)peak, (floating_t)region->maximum);
    region->score = sqrt( REGION_SCORE_FACTOR * ( ( delta_d * delta_d ) + ( delta_p * delta_p ) ) );
    if( region->score > 100. )
        printf("");
}

floating_t RhoUtility_Calculate_TrackerScore( tracker_t * t )
{
    t->score = t->kalman.K[0];
    return t->score;
}
    
void RhoUtility_Calculate_PunishTracker( tracker_t * t )
{
    t->kalman.K[0] *= RHO_TRACKER_PUNISH_FACTOR;
    if( RhoUtility.Calculate.TrackerScore( t ) < RHO_TRACKER_MIN_SCORE )
        Kalman.Reset( &t->kalman, 0 );
//    return t->score;
}

/* Generic centroid and mass calculator */
density_2d_t RhoUtility_Calculate_Centroid( sdensity_t * density_map, uint16_t length, uint16_t * centroid, register density_t thresh )
{
    floating_t avg = 0, average_moment = 0, count = 0, total = 0;
    for( uint16_t i = 0; i < length; i++ )
    {
        sdensity_t curr = density_map[i];
        if( curr > thresh )
        {
            /* Note only fraction m1/m0 is needed so either average method works*/
            RhoUtility.Calculate.CumulativeMoments((floating_t)curr, (floating_t)i, &avg, &average_moment, &count);
            total += curr;
        }
    }
    *centroid = (uint16_t)(average_moment/avg);
    return(density_2d_t)count;
}

void RhoUtility_PrintPacket( packet_t * packet, uint16_t length )
{
    LOG_PACKET(DEBUG_0,"Packet Size - %lubytes\n", sizeof(packet_t));
    for(int i = 0; i < sizeof(packet_t); )
    {
        LOG_PACKET(DEBUG_0,"(%02d)", i);
        for( int j = 0; j < length && i < sizeof(packet_t); j++, i++ )
            LOG_PACKET(DEBUG_0," 0x%02x", *(byte_t*)(&((byte_t*)&packet->header)[i]));
        LOG_PACKET(DEBUG_0,"\n");
    }
    LOG_PACKET(DEBUG_0,"{%02x}{%02x}{%02x}{%02x} %f\n",packet->data[0],packet->data[1],packet->data[2],packet->data[3],*(floating_t*)packet->data);
}

void RhoUtility_Calculate_Background( rho_core_t * core )
{
    density_2d_t xt = RhoUtility.Calculate.Centroid( core->density_map_pair.x.background, core->density_map_pair.x.length, &core->secondary.x, BACKGROUND_CENTROID_CALC_THRESH );
    density_2d_t yt = RhoUtility.Calculate.Centroid( core->density_map_pair.y.background, core->density_map_pair.y.length, &core->secondary.y, BACKGROUND_CENTROID_CALC_THRESH );
    core->quadrant_background_total = MAX(xt, yt);
}

void RhoUtility_Calculate_Packet( rho_core_t * core )
{
//    packet_value_lookup_t  packet_value_lookup  = PACKET_ADDRESS_INITIALIZER(core->prediction_pair);
//    packet_offset_lookup_t packet_offset_lookup = PACKET_OFFSETS;
//    packet_generation_variables _ =
//    {
//        &core->packet,
//        (address_t)&core->packet.data,
//        (address_t)&packet_offset_lookup,
//        (address_t*)&packet_value_lookup,
//        0
//    };
//    _.packet->header.timestamp = TIMESTAMP_MS();
//    while( _.i++ < NUM_PACKET_ELEMENTS )
//    {
//        if( _.packet->header.includes & 0x01 )
//        {
//            if(!_.t) _.l = (*(packing_template_t*)_.llPtr).a;
//            else     _.l = (*(packing_template_t*)_.llPtr).b;
//            for( _.j = 0; _.j < _.l; _.j++)
//                ((byte_t*)_.pdPtr)[_.j] = *(((byte_t*)* _.alPtr)+_.j);
//            _.pdPtr += _.l;
//        }
//        _.alPtr++;
//        _.includes >>= 1;
//        if((_.t=!_.t )) ++_.llPtr;
//    }
}

void RhoUtility_GenerateObservationListFromPredictions( prediction_t * prediction, uint8_t thresh )
{
    LOG_RHO(RHO_DEBUG_2, "Creating observation list for %s:\n", prediction->name);
    uint16_t i = 0;
    for( ; i < prediction->num_regions && i < MAX_OBSERVATIONS; i++ )
    {
        uint16_t io = prediction->trackers_order[i];
        if( io >= MAX_TRACKERS ) continue;
        floating_t x = prediction->trackers[io].kalman.x.p;
        if( !prediction->regions_order[i].valid ) continue;
        region_t * region = prediction->trackers[io].region;// &prediction->regions[prediction->regions_order[i].index];
        bool below_centroid = (density_t)x < prediction->previous_centroid;

        uint16_t density =  (uint16_t)region->density + (uint16_t)prediction->previous_peak[(uint8_t)below_centroid];
        density = BOUNDU( density, MAX_REGION_HEIGHT );
        LOG_RHO_BARE(RHO_DEBUG_2, "\t\t(%d) <%d %d %d>\n", i, density, thresh, io);
        prediction->observation_list.observations[i] = (observation_t){ density/2, thresh, io };
    }
    prediction->observation_list.length = i;
}

void RhoUtility_GenerateObservationListsFromPredictions( rho_core_t * core )
{
    if(core->prediction_pair.x.num_regions > 0)
        RhoUtility.Predict.GenerateObservationList( &core->prediction_pair.x, core->thresh_byte );
    if(core->prediction_pair.y.num_regions > 0)
        RhoUtility.Predict.GenerateObservationList( &core->prediction_pair.y, core->thresh_byte );
}

void RhoUtility_ReportObservationListsFromPredictions( rho_core_t * core )
{
    RhoUtility.Predict.GenerateObservationLists( core );

#ifdef __PSM__
    PSMFunctions.ReportObservations( &core->PredictiveStateModelPair.x, &core->PredictionPair.x.ObservationList, core->PredictionPair.x.NuRegions, core->ThreshByte );
    PSMFunctions.ReportObservations( &core->PredictiveStateModelPair.y, &core->PredictionPair.y.ObservationList, core->PredictionPair.y.NuRegions, core->ThreshByte );
#endif
}

void RhoUtility_UpdatePredictiveStateModelPair( rho_core_t * core )
{
#ifdef __PSM__
    PSMFunctions.Update( &core->PredictiveStateModelPair.x );
    PSMFunctions.Update( &core->PredictiveStateModelPair.y );
    core->predictive_state_model_pair.current_state = core->state_machine.state;// MAX( core->PredictiveStateModelPair.x.current_state, core->PredictiveStateModelPair.y.current_state );
    core->predictive_state_model_pair.best_confidence = AVG2( core->predictive_state_model_pair.x.best_confidence, core->predictive_state_model_pair.y.best_confidence );
    core->predictive_state_model_pair.proposed_num = MAX( core->predictive_state_model_pair.x.proposed.num, core->predictive_state_model_pair.y.proposed.num );
    core->predictive_state_model_pair.proposed_avg_den = AVG2( core->predictive_state_model_pair.x.proposed.density, core->predictive_state_model_pair.y.proposed.density );
    core->predictive_state_model_pair.proposed_threshold = AVG2( core->predictive_state_model_pair.x.proposed.thresh, core->predictive_state_model_pair.y.proposed.thresh );
#endif
}
    
    
const rho_utility_functions RhoUtility =
{
    .Initialize.Data = RhoUtility_InitializeData,
    .Initialize.Filters = RhoUtility_InitializeFilters,
    .Initialize.Prediction = RhoUtility_InitializePrediction,
    .Initialize.DensityMap = RhoUtility_InitializeDensityMap,

    .Reset.Detect = RhoUtility_ResetDetect,
    .Reset.Prediction = RhoUtility_ResetPrediction,
    .Reset.DensityMapPairKalmans = RhoUtility_ResetDensityMapPairKalmans,

    .Predict.PeakFilter = RhoUtility_PredictPeakFilter,
    .Predict.TrackingFilters = RhoUtility_PredictTrackingFilters,
    .Predict.CalculateValidTracks = RhoUtility_CalculateValidTracks,
    .Predict.SortFilters = RhoUtility_SortTrackingFilters,
    .Predict.TrackingProbabilities = RhoUtility_PredictTrackingProbabilities,
    .Predict.CorrectAmbiguity = RhoUtility_CorrectPredictionAmbiguity,
    .Predict.CombineProbabilities = RhoUtility_CombineAxisProbabilites,
//        .Predict.RedistributeDensities = RhoUtility_RedistributeDensities,
    .Predict.UpdateCorePredictionData = RhoUtility_UpdateCorePredictionData,
    .Predict.GenerateObservationList = RhoUtility_GenerateObservationListFromPredictions,
    .Predict.GenerateObservationLists = RhoUtility_GenerateObservationListsFromPredictions,
    .Predict.ReportObservationLists = RhoUtility_ReportObservationListsFromPredictions,
    
    .Predict.UpdatePredictiveStateModelPair = RhoUtility_UpdatePredictiveStateModelPair,

    .Detect.Perform = RhoUtility_DetectPerform,
    .Detect.LowerBound = RhoUtility_DetectLowerBound,
    .Detect.Regions = RhoUtility_DetectRegions,
    .Detect.Region = RhoUtility_DetectRegion,
#ifdef __USE_ZSCORE_THRESHOLD__
    .Detect.ZLower = RhoUtility_DetectZLower,
    .Detect.ZRegion = RhoUtility_DetectZRegion,
#endif
    .Detect.SubtractBackground = RhoUtility_SubtractBackgroundForDetection,
    .Detect.CalculateChaos = RhoUtility_CalculateChaos,
    .Detect.ScoreRegions = RhoUtility_ScoreRegions,
    .Detect.SortRegions = RhoUtility_SortRegions,
    .Detect.Centroid = RhoUtility_Detect_Centroid,
    .Detect.CalculateFrameStatistics = RhoUtility_CalculatedFrameStatistics,

    .Calculate.PredictionCenter = RhoUtility_CalculatePredictionCenter,
    .Calculate.Tune = RhoUtility_CalculateTune,
    .Calculate.BackgroundTuneFactor = RhoUtility_CalculateBackgroundTuneFactor,
    .Calculate.StateTuneFactor = RhoUtility_CalculateStateTuneFactor,
    .Calculate.TargetTuneFactor = RhoUtility_CalculateTargetTuneFactor,
    .Calculate.TargetCoverageFactor = RhoUtility_CalculateTargetCoverageFactor,
    
    // TODO: Reorganize these!
    .Calculate.CumulativeMoments = Statistics_Calculate_CumulativeMoments,
    .Calculate.CumulativeAverage = Statistics_Calculate_CumulativeAverage,
    .Calculate.CumulateAverageStandardDeviation = Statistics_Calculate_CumulateAverageStandardDeviation,
    .Calculate.Variance = Statistic_Calculate_Variance,
    .Calculate.RegionScore = RhoUtility_Calculate_RegionScore,
    .Calculate.TrackerScore = RhoUtility_Calculate_TrackerScore,
    .Calculate.PunishTracker = RhoUtility_Calculate_PunishTracker,
    .Calculate.Centroid = RhoUtility_Calculate_Centroid,
    .Calculate.Background = RhoUtility_Calculate_Background,
    
    .Calculate.Packet = RhoUtility_Calculate_Packet,
    
    .Print.Packet = RhoUtility_PrintPacket,
};
