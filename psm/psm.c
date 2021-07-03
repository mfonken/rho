//
//  psm.c
//  hmmtest
//
//  Created by Matthew Fonken on 2/12/19.
//  Copyright © 2019 Matthew Fonken. All rights reserved.
//

#ifdef __PSM__

#include "psm.h"
//#include "statistics.h"

void ProbabilisticStateModel_Initialize( psm_t * model, const char * name )
{
    model->name = (char *)name;
    FSMFunctions.Sys.Initialize( &model->fsm, name, &model->hmm.A, UNDER_POPULATED );
    GMMFunctions.Model.Initialize( &model->gmm, name );
    HMMFunctions.Initialize( &model->hmm, name );
    model->state_bands.length = NUM_STATE_GROUPS;
    
    floating_t default_bands_intervals[] = STATE_KUMARASWAMY_INTERVALS;
    floating_t prev_boundary = 0., vertical_center = 0;
    for( uint8_t i = 0; i < NUM_STATE_GROUPS; i++ )
    {
        band_t * band = &model->state_bands.band[i];
        band->lower_boundary = prev_boundary;
        band->upper_boundary = default_bands_intervals[i];
        vertical_center = ( ( (floating_t)( band->upper_boundary + band->lower_boundary ) ) * THRESH_RANGE ) / 2;
        band->true_center = (vec2_t){ 0, vertical_center };
    }
    
    KumaraswamyFunctions.Initialize( &model->kumaraswamy, NUM_STATES + 1, (floating_t[])DEFAULT_KUMARASWAMY_BANDS );
}
static int c = 0;
void ProbabilisticStateModel_ReportObservations( psm_t * model, observation_list_t * observation_list, floating_t nu, uint8_t thresh )
{
    /* Report tracking observations & update state band knowledge  */
    if( observation_list->length == 0 ) return;
    
#ifdef HMM_2D_EMISSIONS
    model->current_observation = (vec2_t){ nu, thresh };
#else
    model->current_observation = nu;
#endif
    HMMFunctions.ReportObservation( &model->hmm, model->current_observation );
    
    /* Cycle through and add observations to gaussian mixture model */
    vec2_t value = { 0 };
    for( uint8_t i = 0; i < observation_list->length && i < MAX_OBSERVATIONS; i++ )
    {
        c++;
        observation_t * observation = &observation_list->observations[i];
        value = (vec2_t){ (double)observation->density, (double)observation->thresh };
        GMMFunctions.Model.AddValue( &model->gmm, observation, &value );
    }
    
    if( observation_list->length > 0 )
        model->previous_thresh = observation_list->observations[0].thresh;
    
    /// TODO: Analyze value
}

void ProbabilisticStateModel_UpdateStateIntervals( psm_t * model )//, floating_t nu )
{
    floating_t cumulative = 0., current = 0.;
    LOG_PSM(PSM_DEBUG_UPDATE, "Hidden:");
    for( uint8_t i = 0; i < NUM_STATE_GROUPS; i++ )
    {
        current = GetProbabilityFromEmission( &model->hmm.B[i], model->current_observation );
        cumulative += current;
        model->kumaraswamy.bands[i] = cumulative;
        LOG_PSM_BARE(PSM_DEBUG_UPDATE, "[%.4f]", current);
    }
    LOG_PSM_BARE(PSM_DEBUG, "\n");
    
//    KumaraswamyFunctions.GetVector( &core->Kumaraswamy, core->PredictionPair.NuRegions, state_intervals );
    floating_t o
#ifdef HMM_2D_EMISSIONS
     = model->current_observation.a;
#else
     = model->current_observation;
#endif
    KumaraswamyFunctions.GetVector( &model->kumaraswamy, o, model->state_intervals );
    
    LOG_PSM(PSM_DEBUG_UPDATE, "Update:");
    for( uint8_t i = 0; i < NUM_STATE_GROUPS; i++ )
        LOG_PSM_BARE(PSM_DEBUG_UPDATE, "[%.4f]", model->state_intervals[i]);
    LOG_PSM_BARE(PSM_DEBUG_UPDATE, "\n");
}

void ProbabilisticStateModel_Update( psm_t * model )//, observation_list_t * observation_list, floating_t nu, uint8_t thresh )
{
    /* Update state path prediction to best cluster */
    HMMFunctions.BaumWelchSolve( &model->hmm, HMM_UPDATE_DELTA );
//    return;
    /* Calculate current observation and update observation matrix */
    PSMFunctions.DiscoverStateBands( model, &model->state_bands );
    model->observation_state = PSMFunctions.GetCurrentBand( model, &model->state_bands );
    ///TODO: Compare model->observation_state with hmm recommendation

    /* Update state bands */
    //PSMFunctions.UpdateStateIntervals( model );//, nu );

//    /* Update states/transition matrix */
//    FSMFunctions.Sys.Update( &model->fsm, model->state_intervals );
//    model->current_state = model->fsm.state;

    /* Generate proposals to complete update */
    PSMFunctions.GenerateProposals( model );
}

void ProbabilisticStateModel_UpdateStateBand( band_list_t * band_list, uint8_t i, int8_t c, gaussian2d_t * band_gaussian )
{
    if( i > band_list->length ) i = band_list->length - 1;
    if( c == 0 )
    { /* If no gaussian for band, zero state info */
        if( !i )
            band_list->band[i] = (band_t){ PSM_OBSERVATION_MAX, PSM_OBSERVATION_MAX,  0., (vec2_t){ 0., 0. } };
        else
            memcpy( &band_list->band[i], &band_list->band[i-1], sizeof(band_t) );
    }
    else if( c == -1 )
    {
        floating_t boundary = band_list->band[i-1].upper_boundary;
        band_list->band[i].lower_boundary = boundary;
        band_list->band[i].upper_boundary = boundary;
        band_list->band[i].true_center = (vec2_t){ band_list->band[i-1].true_center.a, boundary };
        band_list->band[i].variance = band_list->band[i-1].variance;
//        printf("A ");
    }
    else
    { /* Otherwise set using cumulated band gaussian */
        floating_t radius = band_gaussian->covariance.d * VALID_CLUSTER_STD_DEV * 5;
        band_list->band[i].lower_boundary = band_gaussian->mean.b + radius;
        band_list->band[i].upper_boundary = band_gaussian->mean.b - radius;
        band_list->band[i].true_center.a = band_gaussian->mean.a;
        band_list->band[i].true_center.b = band_gaussian->mean.b;
        band_list->band[i].variance = band_gaussian->covariance.d;
        if( i )
            band_list->band[i-1].upper_boundary = band_list->band[i].lower_boundary;
//        printf("B ");
    }
//    printf("\tTrue center [%d] is (%7.3f, %7.3f)\n", i, band_list->band[i].true_center.a, band_list->band[i].true_center.b);
}

void ProbabilisticStateModel_DiscoverStateBands( psm_t * model, band_list_t * band_list )
{
//    for(uint32_t i = 0; i < model->gmm.num_clusters; i++)
//    {
//        gaussian2d_t * g = &model->gmm.cluster[i]->gaussian_in;
//        int c = CountSet(GetValidLabels(&model->gmm.cluster[i]->labels));
//        printf("[%2d|%d](%7.3f, %7.3f)<%6.3f, %6.3f, %6.3f, %6.3f>\n", i, c, g->mean.a, g->mean.b, g->covariance.a, g->covariance.b, g->covariance.c, g->covariance.d);
////        mat2x2 * cov = &model->gmm.cluster[i]->gaussian_in.covariance;
////        cov->b = 0; cov->c = 0;
////        floating_t max = MAX(cov->a, cov->d);
////        cov->a = max; cov->d = max;
//    }
    
    LOG_PSM(PSM_DEBUG_UPDATE, "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Discovering state bands~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
#ifdef SPOOF_STATE_BANDS
    band_list->length = NUM_STATE_GROUPS;
    floating_t prev = 0, curr, center;
    floating_t spoof_bands[] = { 0.20, 0.5, 0.75, 1. };
    floating_t spoof_deviation = 40.;
    for( uint8_t i = 0; i < NUM_STATE_GROUPS; i++ )
    {
        curr = spoof_bands[i] * PSM_OBSERVATION_MAX;
        center = ( curr + prev ) / 2;
        band_list->band[NUM_STATE_GROUPS - 1 - i] = (band_t){ curr, prev, spoof_deviation, (vec2_t){ (1 - spoof_bands[i]) * CAPTURE_WIDTH, center } };
        prev = curr;
    }
    return;
#else
    uint32_t processed_clusters = { 0 };
    for( uint8_t i = 0; i < model->gmm.num_clusters; i++ )
        GMMFunctions.Cluster.UpdateLimits( model->gmm.cluster[i] );
    
    gaussian2d_t band_gaussian = { 0 };
    band_list->band[0].lower_boundary = 0;
    /// NOTE: Boundaries are found from greatest to least
    int8_t current_band_id = 0, num_clusters_in_band = 0, num_to_process = model->gmm.num_clusters;
    uint32_t running_label_vector = 0;
    

//########################################################//
    gaussian2d_t band_gaussians[5] = { 0 };
    int8_t num_clusters_in_bands[5] = { 0 };
    cumulative_avg_t band_centers[5][3] = { 0 };
//########################################################//
    while(num_to_process-- > 0)
    {
        /* Get next min cluster */
        uint8_t max_id = 0;
        floating_t max_boundary = 0, check_boundary;
        for(uint32_t i = 0, m = 1; i < model->gmm.num_clusters; i++, m <<= 1 )
        {
            gaussian_mixture_cluster_t * cluster = model->gmm.cluster[i];
            if( cluster->gaussian_in.covariance.d > MAX_CLUSTER_Y_VARIANCE
               || cluster->max_y < 100 )
            {
                processed_clusters |= m;
                continue;
            }
            
            if( processed_clusters & m ) continue;
            if( cluster->gaussian_in.mean.b > PSM_OBSERVATION_MAX ) continue;
            check_boundary = cluster->max_y;
            LOG_PSM(PSM_DEBUG_2, "Checking cluster %d: %6.3f|%6.3f (%.2f, %.2f) <%.2f, %.2f, %.2f, %.2f>... ", i, check_boundary, max_boundary, cluster->gaussian_in.mean.a, cluster->gaussian_in.mean.b, cluster->gaussian_in.covariance.a, cluster->gaussian_in.covariance.b, cluster->gaussian_in.covariance.c, cluster->gaussian_in.covariance.d);
            if( check_boundary > max_boundary )
            {
                max_boundary = check_boundary;
                max_id = i;
                LOG_PSM_BARE(PSM_DEBUG_2, "is max");
            }
            LOG_PSM_BARE(PSM_DEBUG_2, "\n");
        }
        
//        if( !band_gaussian.mean.a && !band_gaussian.mean.b )
//            band_gaussian = model->gmm.cluster[max_id]->gaussian_in;
            
        /* Check if new cluster has new label(s) */
        uint32_t current_label_vector = GetValidLabels( &model->gmm.cluster[max_id]->labels );
//########################################################//
        current_band_id = CountSet(current_label_vector);
        if(current_band_id > 4) current_band_id = 4;
        gaussian2d_t * gaussian = &model->gmm.cluster[max_id]->gaussian_in;
        //            band_gaussian.covariance.b = 0; band_gaussian.covariance.c = 0;
        if(band_gaussians[current_band_id].mean.a == 0)
        {
            band_centers[current_band_id][0] = (cumulative_avg_t){ gaussian->mean.a, 1 };
            band_centers[current_band_id][1] = (cumulative_avg_t){ gaussian->mean.b, 1 };
            band_centers[current_band_id][2] = (cumulative_avg_t){ gaussian->covariance.d, 1 };
            band_gaussians[current_band_id] = *gaussian;
            num_clusters_in_bands[current_band_id]++;
        }
        else
        {
            CumulateAverageStatistics( model->gmm.cluster[max_id]->gaussian_in.mean.a, &band_centers[current_band_id][0] );
            CumulateAverageStatistics( model->gmm.cluster[max_id]->gaussian_in.mean.b, &band_centers[current_band_id][1] );
            CumulateAverageStatistics( model->gmm.cluster[max_id]->gaussian_in.covariance.d, &band_centers[current_band_id][2] );
            gaussian2d_t working_gaussian;
            MatVec.Gaussian2D.Multiply( &band_gaussians[current_band_id], &model->gmm.cluster[max_id]->gaussian_in, &working_gaussian );
            MatVec.Mat2x2.ScalarMultiply( 2., &working_gaussian.covariance, &working_gaussian.covariance );
            band_gaussians[current_band_id] = working_gaussian;
            num_clusters_in_bands[current_band_id]++;
        }
//########################################################//
//        uint32_t check = current_label_vector & ~running_label_vector;
//        if( check )
//        { /* If new labels in cluster */
//            uint32_t new_label_vector = running_label_vector | current_label_vector;
//
//            /* If new update skipped bands... */
//            current_band_id = CountSet(new_label_vector);
//
//            for( uint8_t i = CountSet(running_label_vector); i < current_band_id; i++ )
//            {
//                LOG_PSM( PSM_DEBUG, "Completing band %d: |%d|(%.2f, %.2f) <%.2f, %.2f, %.2f, %.2f>\n", i, num_clusters_in_band, band_gaussian.mean.a, band_gaussian.mean.b, band_gaussian.covariance.a, band_gaussian.covariance.b, band_gaussian.covariance.c, band_gaussian.covariance.d);
//                PSMFunctions.UpdateStateBand( band_list, i, num_clusters_in_band, &band_gaussian );
//                num_clusters_in_band = -1;
//            }
//            band_gaussian = model->gmm.cluster[max_id]->gaussian_in;
//            band_gaussian.covariance.b = 0; band_gaussian.covariance.c = 0;
//            num_clusters_in_band = 1;
//            running_label_vector = new_label_vector;
//        }
//        else
//        { /* Otherwise cumulate current gaussian into band gaussian */
//            LOG_PSM(PSM_DEBUG_UPDATE, "bcy: %.4f vs min: %d | stddevs: %.4f\n", band_gaussian.covariance.d, MIN_VARIANCE_SPAN_TO_REJECT_FOR_BAND_CALC, NumStdDevsFromYMean( &band_gaussian, model->gmm.cluster[max_id]->gaussian_in.mean.b ));
//            if( band_gaussian.covariance.d < MIN_VARIANCE_SPAN_TO_REJECT_FOR_BAND_CALC
//               && NumStdDevsFromYMean( &band_gaussian, model->gmm.cluster[max_id]->gaussian_in.mean.b ) < MAX_STD_DEVS_TO_BE_INCLUDED_IN_BAND_CALC )
//            {
//                floating_t a = 0, b = 0, c = 0;
//                LOG_PSM(PSM_DEBUG_UPDATE, "Combining cluster %d to band %d\n", max_id, current_band_id);
//                LOG_PSM_BARE(PSM_DEBUG_UPDATE, "  (%.2f, %.2f) <%.2f, %.2f, %.2f, %.2f>\n", band_gaussian.mean.a, band_gaussian.mean.b, band_gaussian.covariance.a, band_gaussian.covariance.b, band_gaussian.covariance.c, band_gaussian.covariance.d);
//
//                gaussian2d_t g = model->gmm.cluster[max_id]->gaussian_in;
//                LOG_PSM_BARE(PSM_DEBUG_UPDATE, "x (%.2f, %.2f) <%.2f, %.2f, %.2f, %.2f>\n", g.mean.a, g.mean.b, g.covariance.a, g.covariance.b, g.covariance.c, g.covariance.d);
//                a = band_gaussian.mean.a; b = model->gmm.cluster[max_id]->gaussian_in.mean.a;
//                gaussian2d_t working_gaussian;
//                MatVec.Gaussian2D.Multiply( &band_gaussian, &model->gmm.cluster[max_id]->gaussian_in, &working_gaussian );
//                c = working_gaussian.mean.a;
//                MatVec.Mat2x2.ScalarMultiply( 2., &working_gaussian.covariance, &working_gaussian.covariance );
//                LOG_PSM_BARE(PSM_DEBUG_UPDATE, "= (%.2f, %.2f) <%.2f, %.2f, %.2f, %.2f>\n", working_gaussian.mean.a, working_gaussian.mean.b, working_gaussian.covariance.a, working_gaussian.covariance.b, working_gaussian.covariance.c, working_gaussian.covariance.d);
//                num_clusters_in_band++;
//
//                if( (a - c >= 0 && b - c > 0) || (a - c <= 0 && b - c < 0))
//                {
//                    MatVec.Gaussian2D.Multiply( &band_gaussian, &model->gmm.cluster[max_id]->gaussian_in, &working_gaussian );
//                }
//                band_gaussian = working_gaussian;
//            }
//        }
//
//        LOG_PSM(PSM_DEBUG_UPDATE, "Band %d gaussian is <%.4f %.4f> [%.4f %.4f %.4f %.4f]\n", current_band_id, band_gaussian.mean.a, band_gaussian.mean.b, band_gaussian.covariance.a, band_gaussian.covariance.b, band_gaussian.covariance.c, band_gaussian.covariance.d );
//
//        if( !num_to_process )
//        { /* Always update band on last cluster */
//            PSMFunctions.UpdateStateBand( band_list, current_band_id, num_clusters_in_band, &band_gaussian );
//        }

        processed_clusters |= 1 << max_id;
    }

//########################################################//
    for( uint8_t i = 0; i < 5; i++ )
    {
        band_gaussians[i].mean.a = band_centers[i][0].value;
        band_gaussians[i].mean.b = band_centers[i][1].value;
        band_gaussians[i].covariance.a = band_centers[i][2].value;
        band_gaussians[i].covariance.d = band_centers[i][2].value;
        num_clusters_in_bands[i] = band_centers[i][0].count;
        
        PSMFunctions.UpdateStateBand( band_list, i, num_clusters_in_bands[i], &band_gaussians[i] );
    }
//########################################################//
    
//    for( uint8_t i = current_band_id+1; i < band_list->length; i++ )
//        PSMFunctions.UpdateStateBand( band_list, i, -1, NULL );
//    band_list->band[band_list->length-1].upper_boundary = 0;
//
//    if(band_list->length > 0)
//    {
//        LOG_PSM(PSM_DEBUG_UPDATE, "State bands: \n");
//        for( uint8_t i = 0; i < band_list->length; i++ )
//            LOG_PSM(PSM_DEBUG_UPDATE, " %2d: (%7.3f %7.3f)  C<%7.3f %7.3f>[%7.3f]\n", i, band_list->band[i].lower_boundary, band_list->band[i].upper_boundary, band_list->band[i].true_center.a, band_list->band[i].true_center.b, band_list->band[i].variance);
//        LOG_PSM(PSM_DEBUG_UPDATE, "\n");
//    }
#endif
}


/// TODO: Fix this, HMM order may be scrabbled and/or incomplete
uint8_t ProbabilisticStateModel_FindMostLikelyHiddenState( psm_t * model, uint8_t observation_state, floating_t * confidence )
{
    /* Determine target observation band */
    emission_t * state_emission = &model->hmm.B[observation_state];//GetProbabilityFromEmission( &model->hmm.B[observation_state],  );
#ifdef HMM_GAUSSIAN_EMISSIONS
#ifdef HMM_2D_EMISSIONS
    if( confidence != NULL) *confidence = state_emission->covariance.a;
    return state_emission->mean.a;
#else
    if( confidence != NULL) *confidence = state_emission->std_dev;
    return state_emission->mean;
#endif
#else
    // TODO: Do discrete emiisions have confidence
    return (uint8_t)*state_emission;
#endif
}

void ProbabilisticStateModel_UpdateBestCluster( psm_t * model, band_list_t * band_list )
{
    floating_t
    lower_bound = band_list->band[model->best_state].lower_boundary,
    upper_bound = ( model->best_state + 1 >= NUM_STATE_GROUPS ) ?
    MAX_THRESH : band_list->band[model->best_state + 1].lower_boundary,
    best_cluster_weight = 0.;
    int8_t best_cluster_id = -1;
    for( uint8_t i = 0; i < model->gmm.num_clusters; i++ )
    {
        gaussian_mixture_cluster_t * cluster = model->gmm.cluster[i];
        if( IN_RANGE( cluster->gaussian_in.mean.b, upper_bound, lower_bound ) )
        {
            GMMFunctions.Cluster.Weigh( cluster );
            if( cluster->weight > best_cluster_weight )
            {
                best_cluster_weight = cluster->weight;
                best_cluster_id = i;
            }
        }
    }
    model->best_cluster_id = best_cluster_id;
    model->best_cluster_weight = best_cluster_weight;
}

uint8_t ProbabilisticStateModel_GetCurrentBand( psm_t * model, band_list_t * band_list )
{
    uint8_t state = 0;
    for( ; state < band_list->length-1; state++ )
    { /* If next boundary is above thresh, band is in current thresh */
        if( model->previous_thresh > band_list->band[state+1].lower_boundary )
            break;
    }
    return state;
}

//#define MIN_VALID_TARGET_BAND_VARIANCE 3
//#define MAX_VALID_TARGET_BAND_VARIANCE 7

void ProbabilisticStateModel_GenerateProposals( psm_t * model )
{
    if( !model->gmm.num_clusters ) return;

//    /* Update current state */
//    model->best_state = PSMFunctions.FindMostLikelyHiddenState( model, TARGET_STATE, &model->best_confidence );
//    PSMFunctions.UpdateBestCluster( model, &model->state_bands );
//
//    if( model->best_cluster_id >= model->gmm.num_clusters
//       || model->best_cluster_id < 0) return;
    
    
    
    double target_band_variance = model->state_bands.band[2].variance;
    LOG_PSM(PSM_DEBUG_UPDATE, "Target Band: var>%.4f\n", target_band_variance);
    
//    if(target_band_variance > 0 && target_band_variance < MAX_VALID_TARGET_BAND_VARIANCE)
        model->best_state = 2;
    
    
#define MIN_VALID_TARGET_BAND_VARIANCE 1
#define MAX_VALID_TARGET_BAND_VARIANCE 10
    /* Update predictions */
    vec2_t * proposed_center = &model->state_bands.band[model->best_state].true_center;
//    if(target_band_variance < MIN_VALID_TARGET_BAND_VARIANCE
//       || target_band_variance > MAX_VALID_TARGET_BAND_VARIANCE)
//        proposed_center->b = 0;
//    else
//        proposed_center->b += 0.00001;
    
    /* Update primary & secondary to be reconstructed */
    gaussian_mixture_cluster_t * cluster = model->gmm.cluster[model->best_cluster_id];
    
    model->proposed = (psm_proposal_t)
    {
        proposed_center->a,
        proposed_center->b,
        2,//GetNumberOfValidLabels( &cluster->labels ),
        0,//cluster->primary_id,
        1//cluster->secondary_id
    };
    LOG_PSM(PSM_DEBUG_UPDATE, "Proposal: Density - %.2f | Thresh - %.2f | Num - %d | P_id - %d | S_id - %d\n", model->proposed.density, model->proposed.thresh, model->proposed.num, model->proposed.primary_id, model->proposed.secondary_id);
}

#endif
