/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  File: rho_detect.c
 *  Group: Rho Core
 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *                             Includes                                 *
 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
#include "rho_detect.h"

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *                       Function Definitions                           *
 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

// ------------------------ Initialization --------------------------- //
void RhoDetect_InitializeData( rho_core_t * core, coord_t width, coord_t height )
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

void RhoDetect_InitializeFilters( rho_core_t * core )
{
    /* Threshold Filter */
    RhoPID.Initialize( &core->thresh_filter, DEFAULT_PID_GAIN );

    /* Coverage Filter */
    core->target_coverage_factor  = (floating_t)FILTERED_COVERAGE_TARGET;
    Kalman.Init(&core->target_filter, core->target_coverage_factor, RHO_TARGET_ACC, RHO_TARGET_VU, RHO_TARGET_SU, RHO_TARGET_LS, RHO_TARGET_FILTER_MIN, RHO_TARGET_FILTER_MAX );
}

void RhoDetect_InitializePrediction( prediction_t * prediction, const char * name, coord_t length )
{
    prediction->name = name;

    /* Prediction probabilities */
    memset( &prediction->probabilities, 0, sizeof(floating_t) * 4 );
    for(uint8_t i = 0; i < MAX_TRACKERS; i++)
    {
//        prediction->trackers_order[i] = i;
        tracker_t * t = &prediction->trackers[i];
        Kalman.Init( &t->kalman, 0., RHO_DEFAULT_ACC, RHO_DEFAULT_VU, RHO_DEFAULT_SU, RHO_PREDICTION_LS, 0, length );
        t->region = NULL;
        t->valid = false;
        t->lifespan = RHO_DEFAULT_LS;
        t->score = 0.0;
    }
    /* Regions */
    for(uint8_t i = 0; i < MAX_REGIONS; i++)
    {
        memset(&prediction->regions[i], 0, sizeof(region_t));
        prediction->regions_order[i] = &prediction->regions[i];
    }
}

void RhoDetect_InitializeDensityMap( density_map_t * density_map, const char * name, uint16_t length, uint16_t centroid )
{
    density_map->name = name;

//    size_t size = sizeof(sdensity_t)*length;
//    memset(density_map->map, 0, size);
//    memset(density_map->background, 0, size);
    density_map->length = length;
    density_map->max[0] = 0;
    density_map->max[1] = 0;
    density_map->centroid = centroid;
    Kalman.Init( &density_map->peak[0],  0, RHO_DEFAULT_ACC, RHO_DEFAULT_VU, RHO_DEFAULT_SU, RHO_DEFAULT_LS, 0, length );
    Kalman.Init( &density_map->peak[1], 0, RHO_DEFAULT_ACC, RHO_DEFAULT_VU, RHO_DEFAULT_SU, RHO_DEFAULT_LS, 0, length );
}

// ------------------------ Reset --------------------------- //
void RhoDetect_ResetDetect( rho_detection_variables * _, density_map_t * density_map, prediction_t * prediction )
{
    memset( _, 0, sizeof(rho_detection_variables) );

    _->len          = density_map->length;
    _->range[2]     = 0;
    _->range[1]     = density_map->centroid;
    _->range[0]     = density_map->length;

    memset( &prediction->probabilities, 0, sizeof(floating_t)*NUM_STATES );
    memset( &prediction->probabilities, 0, sizeof(floating_t)*NUM_STATES );

    prediction->nu_regions    = 0;
    prediction->total_density = 0;
    prediction->num_regions   = 0;

    for( uint16_t i = 0; i < MAX_REGIONS; i++ )
    {
        memset(&prediction->regions[i], 0, sizeof(region_t));
        prediction->regions_order[i] = &prediction->regions[i];
    }
}

void RhoDetect_ResetDensityMapPairKalmans( rho_core_t * core )
{
    Kalman.Reset( &core->density_map_pair.x.peak[0], core->prediction_pair.x.previous_peak[0] );
    Kalman.Reset( &core->density_map_pair.x.peak[1], core->prediction_pair.x.previous_peak[1] );
    Kalman.Reset( &core->density_map_pair.y.peak[0], core->prediction_pair.y.previous_peak[0] );
    Kalman.Reset( &core->density_map_pair.y.peak[1], core->prediction_pair.y.previous_peak[1] );
}

void RhoDetect_DetectPerform( rho_detection_variables * _, density_map_t * density_map, prediction_t * prediction )
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
        if( density_map->num_blobs > 0 )
        {
            coord_t bi = density_map->blob_id[_->cycle];
            _->start = density_map->buffer_loc[bi];
            _->end = bi == 0 ? 0 : density_map->buffer_loc[bi - 1];
            _->offset = density_map->offset[bi];
            _->blob_id = bi;
        }
        else
        {
            _->start = _->range[_->cycle];
            _->end = _->range[_->cycle_];
            _->offset = 0;
            _->blob_id = -1;
        }
        _->recalculation_counter = 0;
        _->recalculate = false;

        RhoDetect.Predict.PeakFilter( _, density_map, prediction );
        if( RhoDetect.Detect.LowerBound( _ ) )
        {
            do
            {
//                LOG_RHO(RHO_DEBUG_DETECT_2, "%s:%d> Recalc: %d\n", prediction->name, _->cycle, _->recalculation_counter);
                RhoDetect.Detect.Regions( _, density_map, prediction );
                if( !_->recalculate )
                  RhoDetect.Detect.CalculateChaos( _, prediction );
                RhoDetect.Detect.ScoreRegions( _, density_map, prediction );
                if(_->recalculation_counter == 0)
                  _->first_filtered_density = _->filtered_density;
            } while( _->recalculate && ++_->recalculation_counter < MAX_RHO_RECALCULATION_LEVEL );
        }

        density_t peak = BOUNDU( _->maximum, _->len );
        Kalman.Update( &density_map->peak[_->cycle], peak );
        prediction->previous_peak[_->cycle] = peak;
//        LOG_RHO(RHO_DEBUG_DETECT_2, "%s:%d> Peak: %d\n", prediction->name, _->cycle, prediction->previous_peak[_->cycle]);
        prediction->previous_density[_->cycle] = _->first_filtered_density;
        density_map->max[_->cycle] = _->maximum;
    }
    
    _->filtered_density = 0;
    uint8_t valid_regions = 0;
    for( uint8_t i = 0; i < _->total_regions; i++ )
    {
        if( prediction->regions_order[i] != NULL )
        {
            _->filtered_density = prediction->regions_order[i]->density;
            valid_regions++;
        }
    }
    _->total_regions = valid_regions;
    LOG_RHO(RHO_DEBUG_DETECT_2, "Regions: %d\n", _->total_regions);
}

uint16_t RhoDetect_PredictPeakVariance( kalman_t * k );
inline uint16_t RhoDetect_PredictPeakVariance( kalman_t * k )
{
    floating_t gain = k->K[0];
    floating_t var = RHO_VARIANCE_SCALE * gain;
    
    return BOUND((uint16_t)var, MIN_VARIANCE, MAX_VARIANCE);
}

void RhoDetect_PredictPeakFilter( rho_detection_variables * _, density_map_t * density_map, prediction_t * prediction )
{
    kalman_t * k = &density_map->peak[_->cycle];
    Kalman.Predict( k );
    _->filter_peak      = (uint16_t)k->x.p;
    _->filter_peak_2    = _->filter_peak << 1;
    _->filter_variance  = RhoDetect_PredictPeakVariance( k );
//    density_map->kalmans[_->cycle].variance = _->filter_variance;
}

bool RhoDetect_DetectLowerBound( rho_detection_variables * _ );
inline bool RhoDetect_DetectLowerBound( rho_detection_variables * _ )
{
    _->filter_band_lower = (_->filter_peak > _->filter_variance) * (_->filter_peak - _->filter_variance); // Branchless edit
    return _->filter_variance > 0;
}

void RhoDetect_DetectRegions( rho_detection_variables * _, density_map_t * density_map, prediction_t * prediction );
inline void RhoDetect_DetectRegions( rho_detection_variables * _, density_map_t * density_map, prediction_t * prediction )
{ /* Detect regions - START */
    for( _->x = _->start, _->x_ = (_->offset == 0) * _->x + _->offset; _->x >= _->end;  )
    {
        _->curr = density_map->map[--_->x];
        --_->x_;
//        density_map->bound[_->x] = 0;
        if( !_->recalculate )
        {
            sdensity_t b = density_map->background[_->x_];
            RhoDetect.Detect.SubtractBackground( _, b ); // Also finds peak - required even with black background
        }
        RhoDetect.Detect.Region( _, density_map, prediction );
    }
    if( _->has_region )
    {
        _->curr = 0;
        RhoDetect.Detect.Region( _, density_map, prediction );
    }
} /* Detect regions - END */

void RhoDetect_SubtractBackgroundForDetection( rho_detection_variables * _, sdensity_t background_curr );
inline void RhoDetect_SubtractBackgroundForDetection( rho_detection_variables * _, sdensity_t background_curr )
{
    if( _->curr > background_curr ) 
    {
        _->total_density += _->curr;
        _->raw_density_moment += _->curr * (density_t)_->x_;

        /* Update max */
        if(_->curr > _->maximum)
            _->maximum = _->curr;

#ifdef USE_BACKGROUNDING
        /* Subtract background */
        _->curr -= background_curr;

        /* Punish values above the filter peak */
        if( ( _->curr > _->filter_peak )
           && ( _->filter_peak_2 > _->curr ) )
            _->curr = _->filter_peak_2 - _->curr;
#endif
    }
    else
        _->curr = 0;
}

#ifdef __USE_ZSCORE_THRESHOLD__
inline uint16_t RhoDetect_DetectZLower( rho_detection_variables * _ )
{
    floating_t variance = RhoDetect.Calculate.Variance( &_->z_stat );
    _->z_thresh = (uint16_t)(sqrt(variance) * _->z_thresh_factor);
    return _->z_thresh;
}

inline bool RhoDetect_DetectZRegion( rho_detection_variables * _, bool update )
{
    if(update)
    {
        RhoDetect.Calculate.CumulateAverageStandardDeviation( _->curr, &_->z_stat );
        _->has_stat_update = true;
    }
    if(_->z_stat.n >= _->z_stat.max_n)
    {
        sdensity_t delta = _->curr - (sdensity_t)_->z_stat.avg;
        if( -delta > ( _->has_stat_update ? RhoDetect.Detect.ZLower( _ ) : _->z_thresh ) )
          return false;
    }
    return true;
}
#endif

void RhoDetect_DetectRegion( rho_detection_variables * _, density_map_t * density_map, prediction_t * prediction );
inline void RhoDetect_DetectRegion( rho_detection_variables * _, density_map_t * density_map, prediction_t * prediction )
{
#ifdef __USE_REGION_BOUNDARY_OFFSET__
    bool had_region = _->has_region;
#endif

#ifdef __USE_ZSCORE_THRESHOLD__
        density_map->bound[_->x] = MAX((sdensity_t)(_->z_stat.avg - (floating_t)_->z_thresh), 0) * (RhoDetect.Detect.ZRegion( _, false ) ? 1 : -1);
#endif

    /* Check if CMA value is in band */
    if( _->curr > _->filter_band_lower)
    {
        /* reset flag and counter */
#ifdef __USE_ZSCORE_THRESHOLD__
        if(!_->has_region)
            _->z_stat.avg = _->curr;
        _->has_region = RhoDetect.Detect.ZRegion( _, true );
#else
        _->has_region = 1;
#endif
        _->gap_counter = 0;

        /* De-offset valid values */
        _->curr -= _->filter_band_lower;

        /* Process new values into region */
        if( _->curr > 0 )
        {
            RhoDetect.Calculate.CumulativeMoments( (floating_t)_->curr, (floating_t)(_->x_), &_->average_curr, &_->average_moment, &_->average_counter );
        }
        /* Increment width */
        _->width++;
    }

    /* Process completed regions and increment count */
    else if( ++_->gap_counter > RHO_GAP_MAX && _->has_region && _->total_regions < MAX_REGIONS
#ifdef __USE_ZSCORE_THRESHOLD__
            /* Check if region continues below boundary within z-threshold (doesn't drop off) */
            && !( RhoDetect.Detect.ZRegion( _, false ) && ( _->curr > ( (uint16_t)_->z_stat.avg - _->z_thresh ) ) )
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
        if( _->current_density >= MIN_REGION_DENSITY )
        {
            /* Create new region at secondary */
            uint16_t x_loc = (uint16_t)ZDIV( _->average_moment, _->average_curr );
            uint16_t b_loc = _->offset == 0 ? x_loc : _->start - (_->offset - x_loc);
            uint8_t next_index = _->total_regions;
            if(_->recalculate)
            {
                for( uint8_t i = 0; i < next_index; i++ )
                {
                    if( prediction->regions_order[i] == NULL )
                    {
                        next_index = i;
                        break;
                    }
                }
            }
            prediction->regions[next_index] = (region_t){ density_map->map[b_loc], _->current_density, x_loc, _->width, _->blob_id, -1 };
            prediction->regions_order[next_index] = &prediction->regions[next_index];
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
    if( had_region != _->has_region
        && _->region_boundary_index < MAX_REGIONS*2 )
    {
        _->region_boundaries[_->region_boundary_index++] = _->x;
    }
#endif
}

void RhoDetect_CalculateChaos( rho_detection_variables * _, prediction_t * prediction )
{
    _->chaos = MAX( ZDIV( (floating_t)prediction->previous_density[_->cycle], (floating_t)_->filtered_density ), MIN_CHAOS);
//    LOG_RHO(RHO_DEBUG_DETECT_2, "%s:%d> Chaos:%.4f\n", prediction->name, _->cycle, _->chaos);
}

void RhoDetect_ScoreRegions( rho_detection_variables * _, density_map_t * density_map, prediction_t * prediction )
{
    /* Assume no recalculations are needed */
    _->recalculate = false;

    /* Return on empty frames */
    if(_->filtered_density == 0 ) return;

    /* Cycle regions */
    for( byte_t i = 0; i < _->total_regions; i++ )
    {
        /* Get region at index from order array and check validity */
        if( prediction->regions_order[i] == NULL ) continue;
        region_t * curr = prediction->regions_order[i];

        /* Score current region */
        RhoDetect.Calculate.RegionScore( curr, _->filtered_density, _->maximum );

        /* Recalculate regions with chaos */
        floating_t chaos = ( _->recalculation_chaos > 0 && _->recalculation_chaos < ZDIV_LNUM )
            ? _->recalculation_chaos : _->chaos;
        _->recalculation_chaos = ZDIV_LNUM;
        if( curr->score > chaos )
        {
            RhoDetect.Calculate.RegionScore( curr, _->filtered_density, _->maximum );
            
            _->recalculation_chaos = MIN( curr->score, _->recalculation_chaos );
            LOG_RHO(RHO_DEBUG_DETECT_2, "%s:%d> R%d Score: %.4f | Chaos: %.4f\n", prediction->name, _->cycle, i, curr->score, chaos);
            /* Recalculate around chaotic region */
            _->recalculate = true;

            /* Remove this index */
            prediction->regions_order[i] = NULL;
            prediction->num_regions--;

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

void RhoDetect_SortRegions( rho_detection_variables * _, prediction_t * prediction )
{
    /// NOTE: Smaller scores are better
    /* Assume all regions are valid */
    uint16_t i, j;
    
    /* Cycle through found regions */
    for( i = 0; i < _->total_regions - 1; i++)
    {
        /* Cycle through other regions */
        for( j = 0; j < _->total_regions - i - 1; j++ )
        {
            if( prediction->regions_order[j + 1] == NULL )
                continue;
            if( prediction->regions_order[j] == NULL ||
                    prediction->regions_order[j]->score > prediction->regions_order[j + 1]->score )
                SWAP( prediction->regions_order[j], prediction->regions_order[j + 1] );
        }
    }
    for( i++ ; i < MAX_REGIONS; i++ )
    {
//        printf("Inactive %d\n", i);
        prediction->regions_order[i] = NULL;
    }

//    int c2[MAX_REGIONS] = {-1, -1, -1, -1};
    for( i = 0; i < _->total_regions; i++)
    {
        region_t * curr = prediction->regions_order[i];
        if( curr == NULL )
        {
            LOG_RHO(RHO_DEBUG_DETECT, "%s> R%d: null\n",
                    prediction->name,
                    i);
            continue;
        }
        LOG_RHO(RHO_DEBUG_DETECT, "%s> R%d:{M%d, D%d, X%d, W%d} = S%.4f\n",
            prediction->name,
            i,
            curr->maximum,
            curr->density,
            curr->location,
            curr->width,
            curr->score
            );
//        c2[i] = o->index;
//        bool already_has = false;
//        for(int j = i - 1; j >= 0; j--)
//        {
//            if(c2[j] == c2[i])
//            {
//                already_has = true;
//                LOG_RHO(RHO_DEBUG_DETECT_2, "Already has %d at %d and %d\n", c2[i], i, j);
//                break;
//            }
//        }
    }
}
    
void RhoDetect_Detect_Centroid( rho_detection_variables * _, density_map_t * density_map )
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
    if(proposed_center > FRAME_WIDTH_BASE * 0.75)
        printf(">");
    density_map->centroid = BOUNDU(proposed_center, density_map->length);

    LOG_RHO(RHO_DEBUG_DETECT, "%s> Centroid: %d | Moment: %d | Density: %d\n", density_map->name, (int)density_map->centroid, _->raw_density_moment, _->total_density);
}

void RhoDetect_CalculatedFrameStatistics( rho_detection_variables * _, prediction_t * prediction )
{
    /* Update frame statistics */
    _->assumed_regions = (floating_t)_->total_regions;
    if( _->assumed_regions == 0. ) _->assumed_regions = 1.;

    prediction->nu_regions = _->assumed_regions;//BOUNDU( ZDIV( (floating_t)_->total_density * (floating_t)_->assumed_regions, _->target_density ), MAX_NU_REGIONS );
    prediction->num_regions = _->total_regions;
    prediction->total_density = _->total_density;
    prediction->average_density = ZDIV( (floating_t)prediction->total_density, prediction->nu_regions );

    LOG_RHO(RHO_DEBUG_DETECT_2, "Total:%d | Target:%d | Frame:%d\n", _->total_density, (int)_->target_density, TOTAL_RHO_PIXELS);
    LOG_RHO(RHO_DEBUG_DETECT_2, "Regions: %d{%.3f}\n", prediction->num_regions, prediction->nu_regions);
    if( prediction->num_regions == 0 )
        printf("-");
}


// Construct Prediction
void RhoDetect_ResetPrediction( prediction_predict_variables * _, prediction_pair_t * prediction, index_pair_t centroid )
{
//    kalman_t * yk, * xk;
//    uint8_t yi, xi;
//    for( byte_t i = 0; i < 2; i++ )
//    {
//        xk = &prediction->x.trackers[i].kalman;
//        yk = &prediction->y.trackers[i].kalman;
//        _->pts[i] = (index_pair_t){ yk->value, xk->value };
//    }
    _->centroid  = (index_pair_t){ centroid.x, centroid.y };
}

void RhoDetect_UpdateCorePredictionData( prediction_predict_variables * _, rho_core_t * core )
{
    _->centroid.x = (uint16_t)core->density_map_pair.y.centroid;
    _->centroid.y = (uint16_t)core->density_map_pair.x.centroid;
    
//    LOG_RHO(RHO_DEBUG_UPDATE_2, "Primary (%d, %d) | Secondary (%d, %d)\n", _->pts[0].x, _->pts[0].y, _->pts[1].x, _->pts[1].y);
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

uint16_t RhoDetect_CalculatePredictionCenter( uint16_t primary, uint16_t secondary, uint16_t width )
{
    uint16_t sum = primary + secondary;
    if( primary == 0 || secondary == 0 ) return sum;
    else return BOUNDU(sum >> 1, width );
}

void RhoDetect_CalculateTune( rho_core_t * core )
{
    /* Background-Tune on significant background */
    RhoDetect.Calculate.BackgroundTuneFactor( core );

    /* State-Tune by FSM state */
    RhoDetect.Calculate.StateTuneFactor( core );

    /* Filtered-Tune on target difference */
    RhoDetect.Calculate.TargetTuneFactor( core );

    core->tune.proposed = BOUND( core->tune.background + core->tune.state + core->tune.target, -THRESH_STEP_MAX, THRESH_STEP_MAX);

    core->thresh = BOUND(core->thresh + core->tune.proposed, THRESH_MIN, THRESH_MAX);
//    if(core->PredictiveStateModelPair.proposed_threshold > 0)
//        core->Thresh = WeightedAverage(core->PredictiveStateModelPair.proposed_threshold, core->Thresh, 0.5);
    core->thresh_byte = (byte_t)core->thresh;
    
    LOG_RHO(RHO_DEBUG_UPDATE, "Threshold: %.2f\n", core->thresh);
}

void RhoDetect_CalculateBackgroundTuneFactor( rho_core_t * core )
{
    floating_t background_tune_factor = 0.;
    if( core->quadrant_background_total > BACKGROUND_COVERAGE_MIN )
    {
        floating_t background_coverage_factor = 1 - ZDIV( BACKGROUND_COVERAGE_MIN, core->quadrant_background_total );
        background_tune_factor = -pow( BOUND(background_coverage_factor, -BACKGROUND_TUNE_MAX, BACKGROUND_TUNE_MAX), BACKGROUND_TUNE_EXPONENT);
    }
    core->tune.background = background_tune_factor;
}

void RhoDetect_CalculateStateTuneFactor( rho_core_t * core )
{
    core->target_coverage_factor = core->target_filter.x.p;
    core->prediction_pair.average_density = MAX( core->prediction_pair.x.average_density, core->prediction_pair.y.average_density );
    LOG_RHO(RHO_DEBUG_UPDATE, "Filtered|Total %%: %.7f|%.7f\n", core->filtered_percentage, core->total_percentage);

    LOG_RHO(RHO_DEBUG_UPDATE, "Current state: %s\n", stateString(core->state_machine.state));
    switch(core->state_machine.state)
    {
        default:
            RhoDetect.Calculate.TargetCoverageFactor( core );
            RhoDetect.Reset.DensityMapPairKalmans( core );
            break;
        case TARGET_POPULATED:
            Kalman.Step( &core->target_filter, core->total_percentage );
            Kalman.Print( &core->target_filter );
        case OVER_POPULATED:
        case UNDER_POPULATED:
            RhoDetect.Calculate.TargetCoverageFactor( core );
            break;
    }

    RhoPID.Update( &core->thresh_filter, core->target_coverage_factor, core->target_filter.x.p );
    core->tune.state = core->thresh_filter.value * PID_SCALE;

    LOG_RHO(RHO_DEBUG_2, "Avg:%d | Nu:%.4f\n", (int)core->prediction_pair.average_density, core->prediction_pair.nu_regions);
    LOG_RHO(RHO_DEBUG_2, "Target cov.:%.4f | Target val: %.4f | Thresh val:%.4f\n", core->target_coverage_factor, core->target_filter.x.p, core->thresh_filter.value);
}

void RhoDetect_CalculateTargetTuneFactor( rho_core_t * core )
{
    core->tune.target = TARGET_TUNE_FACTOR * ZDIV( core->thresh_filter.value, core->target_filter.x.p );
}

void RhoDetect_CalculateTargetCoverageFactor( rho_core_t * core )
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


void RhoDetect_Calculate_RegionScore( region_t * region, density_t total_density, byte_t peak )
{
    floating_t delta_d = ZDIV((floating_t)region->density, (floating_t)total_density) - 0.5;
    floating_t delta_p = ZDIV((floating_t)peak, (floating_t)region->maximum);
    region->score = ( delta_d * delta_d ) + ( delta_p * delta_p );
    if( region->score > 100. )
        printf("");
}
    
void RhoDetect_CombineAxisProbabilites( prediction_pair_t * prediction )
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
    
/* Generic centroid and mass calculator */
density_2d_t RhoDetect_Calculate_Centroid( sdensity_t * density_map, uint16_t length, uint16_t * centroid, register density_t thresh )
{
    floating_t avg = 0, average_moment = 0, count = 0, total = 0;
    for( uint16_t i = 0; i < length; i++ )
    {
        sdensity_t curr = density_map[i];
        if( curr > thresh )
        {
            /* Note only fraction m1/m0 is needed so either average method works*/
            RhoDetect.Calculate.CumulativeMoments((floating_t)curr, (floating_t)i, &avg, &average_moment, &count);
            total += curr;
        }
    }
    *centroid = (uint16_t)(average_moment/avg);
    return(density_2d_t)count;
}

void RhoDetect_PrintPacket( packet_t * packet, uint16_t length )
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

void RhoDetect_Calculate_Background( rho_core_t * core )
{
    density_2d_t xt = RhoDetect.Calculate.Centroid( core->density_map_pair.x.background, core->density_map_pair.x.length, &core->secondary.x, BACKGROUND_CENTROID_CALC_THRESH );
    density_2d_t yt = RhoDetect.Calculate.Centroid( core->density_map_pair.y.background, core->density_map_pair.y.length, &core->secondary.y, BACKGROUND_CENTROID_CALC_THRESH );
    core->quadrant_background_total = MAX(xt, yt);
}

void RhoDetect_Calculate_Packet( rho_core_t * core )
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

void RhoDetect_GenerateObservationListFromPredictions( prediction_t * prediction, uint8_t thresh )
{
    LOG_RHO(RHO_DEBUG_2, "Creating observation list for %s:\n", prediction->name);
    uint16_t i = 0;
    for( ; i < prediction->num_regions && i < MAX_OBSERVATIONS; i++ )
    {
        uint16_t io = prediction->regions[i].tracker_id;
        if( io >= MAX_TRACKERS ) continue;
        floating_t x = prediction->trackers[io].kalman.x.p;
        if( prediction->regions_order[i] == NULL ) continue;
        region_t * region = prediction->trackers[io].region;// &prediction->regions[prediction->regions_order[i].index];
        bool below_centroid = (density_t)x < prediction->previous_centroid;

        uint16_t density =  (uint16_t)region->density + (uint16_t)prediction->previous_peak[(uint8_t)below_centroid];
        density = BOUNDU( density, MAX_REGION_HEIGHT );
        LOG_RHO_BARE(RHO_DEBUG_2, "\t\t(%d) <%d %d %d>\n", i, density, thresh, io);
        prediction->observation_list.observations[i] = (observation_t){ density/2, thresh, io };
    }
    prediction->observation_list.length = i;
}

void RhoDetect_GenerateObservationListsFromPredictions( rho_core_t * core )
{
    if(core->prediction_pair.x.num_regions > 0)
        RhoDetect.Predict.GenerateObservationList( &core->prediction_pair.x, core->thresh_byte );
    if(core->prediction_pair.y.num_regions > 0)
        RhoDetect.Predict.GenerateObservationList( &core->prediction_pair.y, core->thresh_byte );
}

void RhoDetect_ReportObservationListsFromPredictions( rho_core_t * core )
{
    RhoDetect.Predict.GenerateObservationLists( core );

#ifdef __PSM__
    PSMFunctions.ReportObservations( &core->PredictiveStateModelPair.x, &core->PredictionPair.x.ObservationList, core->PredictionPair.x.NuRegions, core->ThreshByte );
    PSMFunctions.ReportObservations( &core->PredictiveStateModelPair.y, &core->PredictionPair.y.ObservationList, core->PredictionPair.y.NuRegions, core->ThreshByte );
#endif
}

void RhoDetect_UpdatePredictiveStateModelPair( rho_core_t * core )
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
    
    
const rho_detect_functions RhoDetect =
{
    .Initialize.Data = RhoDetect_InitializeData,
    .Initialize.Filters = RhoDetect_InitializeFilters,
    .Initialize.Prediction = RhoDetect_InitializePrediction,
    .Initialize.DensityMap = RhoDetect_InitializeDensityMap,

    .Reset.Detect = RhoDetect_ResetDetect,
    .Reset.Prediction = RhoDetect_ResetPrediction,
    .Reset.DensityMapPairKalmans = RhoDetect_ResetDensityMapPairKalmans,

    .Predict.PeakFilter = RhoDetect_PredictPeakFilter,
    
    .Predict.UpdateCorePredictionData = RhoDetect_UpdateCorePredictionData,
    .Predict.CombineProbabilities = RhoDetect_CombineAxisProbabilites,
    .Predict.GenerateObservationList = RhoDetect_GenerateObservationListFromPredictions,
    .Predict.GenerateObservationLists = RhoDetect_GenerateObservationListsFromPredictions,
    .Predict.ReportObservationLists = RhoDetect_ReportObservationListsFromPredictions,
    
    .Predict.UpdatePredictiveStateModelPair = RhoDetect_UpdatePredictiveStateModelPair,

    .Detect.Perform = RhoDetect_DetectPerform,
    .Detect.LowerBound = RhoDetect_DetectLowerBound,
    .Detect.Regions = RhoDetect_DetectRegions,
    .Detect.Region = RhoDetect_DetectRegion,
#ifdef __USE_ZSCORE_THRESHOLD__
    .Detect.ZLower = RhoDetect_DetectZLower,
    .Detect.ZRegion = RhoDetect_DetectZRegion,
#endif
    .Detect.SubtractBackground = RhoDetect_SubtractBackgroundForDetection,
    .Detect.CalculateChaos = RhoDetect_CalculateChaos,
    .Detect.ScoreRegions = RhoDetect_ScoreRegions,
    .Detect.SortRegions = RhoDetect_SortRegions,
    .Detect.Centroid = RhoDetect_Detect_Centroid,
    .Detect.CalculateFrameStatistics = RhoDetect_CalculatedFrameStatistics,

    .Calculate.PredictionCenter = RhoDetect_CalculatePredictionCenter,
    .Calculate.Tune = RhoDetect_CalculateTune,
    .Calculate.BackgroundTuneFactor = RhoDetect_CalculateBackgroundTuneFactor,
    .Calculate.StateTuneFactor = RhoDetect_CalculateStateTuneFactor,
    .Calculate.TargetTuneFactor = RhoDetect_CalculateTargetTuneFactor,
    .Calculate.TargetCoverageFactor = RhoDetect_CalculateTargetCoverageFactor,
    
    // TODO: Reorganize these!
    .Calculate.CumulativeMoments = Statistics_Calculate_CumulativeMoments,
    .Calculate.CumulativeAverage = Statistics_Calculate_CumulativeAverage,
    .Calculate.CumulateAverageStandardDeviation = Statistics_Calculate_CumulateAverageStandardDeviation,
    .Calculate.Variance = Statistic_Calculate_Variance,
    .Calculate.RegionScore = RhoDetect_Calculate_RegionScore,
    .Calculate.Centroid = RhoDetect_Calculate_Centroid,
    .Calculate.Background = RhoDetect_Calculate_Background,
    
    .Calculate.Packet = RhoDetect_Calculate_Packet,
    
    .Print.Packet = RhoDetect_PrintPacket,
};
