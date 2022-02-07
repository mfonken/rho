/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  File: rho_core.h
 *  Group: Rho Core
 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *                          Includes                                    *
 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
#include "rho_core.h"

#ifdef __USE_DETECTION_MAP__
#include "detection_map.h"
#endif
#ifndef TIMESTAMP
#include "timestamp.h"
#endif

static const char * X_INSTANCE_NAME = "X";
static const char * Y_INSTANCE_NAME = "Y";

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *                       Local Instance                                 *
 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
const rho_core_functions RhoCore =
{
    .Initialize         = RhoCore_Initialize,
    .Perform            = RhoCore_Perform,
    .DetectPairs        = RhoCore_DetectPairs,
    .Detect             = RhoCore_Detect,
    .UpdatePrediction   = RhoCore_UpdatePrediction,
    .UpdatePredictions  = RhoCore_UpdatePredictions,
    .UpdateThreshold    = RhoCore_UpdateThreshold,
    .GeneratePacket     = RhoCore_GeneratePacket
};

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *                      Functions Declarations                          *
 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void RhoCore_Initialize( rho_core_t * core, index_t width, index_t height )
{
    /* Generic Data */
    RhoUtility.Initialize.Data( core, width, height );

    /* Filters */
    RhoUtility.Initialize.Filters( core );

    /* Density Data */
    RhoUtility.Initialize.DensityMap( &core->density_map_pair.x, X_INSTANCE_NAME, height, core->centroid.y );
    RhoUtility.Initialize.DensityMap( &core->density_map_pair.y, Y_INSTANCE_NAME, width, core->centroid.x  );

    /* Prediction Structures */
    RhoUtility.Initialize.Prediction( &core->prediction_pair.x, X_INSTANCE_NAME, core->height );
    RhoUtility.Initialize.Prediction( &core->prediction_pair.y, Y_INSTANCE_NAME, core->width  );

#ifdef __USE_DETECTION_MAP__
    /* Detection map */
    DetectionMapFunctions.Init( &core->DetectionMap, DETECTION_BUFFER_SIZE );
#endif
#ifdef __USE_DECOUPLING__
    /* Frame Conversion Model Connection */
    RhoInterrupts.InitFromCore( core );
#endif
#ifdef __PSM__
    PSMFunctions.Initialize( &core->PredictiveStateModelPair.x, X_INSTANCE_NAME );
    PSMFunctions.Initialize( &core->PredictiveStateModelPair.y, Y_INSTANCE_NAME );
#endif
}

void RhoCore_Perform( rho_core_t * core, bool background_event )
{
    if(background_event)
        RhoUtility.Generate.Background( core );
    else
    {
        RhoCore.DetectPairs( core );
        RhoCore.UpdatePredictions( core );
        RhoCore.UpdateThreshold( core );
//        RhoCore.GeneratePacket( core );
    }
}

/* Calculate and process data in variance band from density filter to generate predictions */
void RhoCore_Detect( rho_core_t * core, density_map_t * density_map, prediction_t * prediction )
{
    LOG_RHO(RHO_DEBUG_2, "Detecting %s Map:\n", density_map->name );
    static rho_detection_variables _;
    RhoUtility.Reset.Detect( &_, density_map, prediction );
    core->total_coverage = 0;
    core->filtered_coverage = 0;
    _.target_density = core->target_filter.value * (floating_t)TOTAL_RHO_PIXELS;

    /* Perform detect */
    LOG_RHO(RHO_DEBUG_2, "Performing detect:\n");
    RhoUtility.Detect.Perform( &_, density_map, prediction );

    /* Update frame statistics */
    LOG_RHO(RHO_DEBUG_2, "Calculating frame statistics:\n");
    RhoUtility.Detect.CalculateFrameStatistics( &_, prediction );

    /* Update core */
    core->total_coverage     += _.total_density;// target_density;
    core->filtered_coverage  += _.filtered_density;
}

void RhoCore_DetectPairs( rho_core_t * core )
{
    LOG_RHO(RHO_DEBUG_2,"Filtering and selecting pairs.\n");
    RhoCore.Detect( core, &core->density_map_pair.x, &core->prediction_pair.x );
    RhoCore.Detect( core, &core->density_map_pair.y, &core->prediction_pair.y );

    /* Calculate accumulated filtered percentage from both axes */
    core->filtered_percentage        = ZDIV( (floating_t)core->filtered_coverage, (floating_t)TOTAL_RHO_PIXELS );
    core->total_percentage           = ZDIV( (floating_t)core->total_coverage, (floating_t)TOTAL_RHO_PIXELS );
    core->prediction_pair.num_regions = MAX( core->prediction_pair.x.num_regions, core->prediction_pair.y.num_regions );
    core->prediction_pair.nu_regions  = MAX( core->prediction_pair.x.nu_regions, core->prediction_pair.y.nu_regions );
}

/* Correct and factor predictions from variance band filtering into global model */
void RhoCore_UpdatePrediction( prediction_t * prediction )
{
    LOG_RHO(RHO_DEBUG_PREDICT,"Updating %s Map:\n", prediction->name);

    /* Step predictions of all Kalmans */
    RhoUtility.Predict.TrackingFilters( prediction );
    RhoUtility.Predict.TrackingProbabilities( prediction );
}

void RhoCore_UpdatePredictions( rho_core_t * core )
{
    LOG_RHO(RHO_DEBUG_2,"Updating predictions.\n");
    RhoCore.UpdatePrediction( &core->prediction_pair.x );
    RhoCore.UpdatePrediction( &core->prediction_pair.y );

#ifdef __USE_DETECTION_MAP__
    RhoUtility.Predict.ReportObservationLists( core );
    DetectionMapFunctions.AddSet( &core->DetectionMap, &core->PredictionPair );
#endif

#ifdef __PSM__
    if( ISTIMEDOUT( core->timestamp, PSM_UPDATE_PERIOD ) )
    { /* Process both dimensions' predictive state */
        LOG_PSM(PSM_DEBUG, "Updating PSM\n");
        RhoUtility.Predict.UpdatePredictiveStateModelPair( core );
        core->timestamp = TIMESTAMP();
    }
#endif

    double state_intervals[NUM_STATE_GROUPS];
    KumaraswamyFunctions.GetVector( &core->kumaraswamy, core->prediction_pair.nu_regions, state_intervals );
    FSMFunctions.Sys.Update( &core->state_machine, state_intervals );

    prediction_predict_variables _;
    RhoUtility.Reset.Prediction( &_, &core->prediction_pair, core->centroid );
    RhoUtility.Predict.CorrectAmbiguity( &_, core );
    RhoUtility.Predict.CombineProbabilities( &core->prediction_pair );
    RhoUtility.Predict.UpdateCorePredictionData( &_, core );
}

/* Use background and state information to update image threshold */
void RhoCore_UpdateThreshold( rho_core_t * core )
{
    LOG_RHO(RHO_DEBUG_2,"Updating threshold.\n");
    RhoUtility.Calculate.Tune( core );
}

void RhoCore_GeneratePacket( rho_core_t * core )
{
    LOG_RHO(RHO_DEBUG_2,"Generating packets.\n");
    RhoUtility.Generate.Packet( core );
    RhoUtility.Print.Packet( &core->packet, PACKET_SIZE );
}
