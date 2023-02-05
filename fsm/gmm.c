//
//  gmm.c
//  GaussianMixtureModel
//
//  Created by Matthew Fonken on 2/9/19.
//  Copyright © 2019 Matthew Fonken. All rights reserved.
//
#ifdef __PSM__
#include "gmm.h"
#include "timestamp.h"

void GaussianMixtureCluster_Initialize( gaussian_mixture_cluster_t * cluster, observation_t * observation, vec2_t * output )
{
    if(cluster == NULL) return;
    LOG_GMM(GMM_DEBUG_2, "Initializing cluster %p\n", cluster);
    
    cluster->gaussian_in.mean.a = observation->density;
    cluster->gaussian_in.mean.b = observation->thresh;
    cluster->gaussian_out.mean.a = output->a;
    cluster->gaussian_out.mean.b = output->b;
    
    memset( &cluster->gaussian_out.covariance, 0, sizeof(mat2x2) );
    cluster->gaussian_in.covariance = (mat2x2){ INITIAL_VARIANCE, 0, 0, INITIAL_VARIANCE };
    cluster->inv_covariance_in = (mat2x2){ INV_INITIAL_VARIANCE, 0, 0, INV_INITIAL_VARIANCE };
    cluster->score = 1.;
    
    memset( &cluster->labels, 0, sizeof(cluster->labels) );
    cluster->labels.average[observation->label] = 1;
    cluster->labels.count[observation->label]++;
    
    MatVec.Mat2x2.LLT( &cluster->gaussian_in.covariance, &cluster->llt_in );
    
    GMMFunctions.Cluster.UpdateNormal( cluster );
}

void GaussianMixtureCluster_Update( gaussian_mixture_cluster_t * cluster, observation_t * observation, vec2_t * output )
{
    LOG_GMM(GMM_DEBUG, "Log gaussian norm factor: %.2f\n", cluster->log_gaussian_norm_factor);
    if (isnan(cluster->log_gaussian_norm_factor))
        return;
    LOG_GMM(GMM_DEBUG, "Mahalanobis sq: %.2f\n", cluster->mahalanobis_sq);
    if (cluster->mahalanobis_sq > MAX_MAHALANOBIS_SQ_FOR_UPDATE)
        return;
    double score_weight = ALPHA * SafeExp( -BETA * cluster->mahalanobis_sq );
    cluster->score += score_weight * ( cluster->probability_condition_input - cluster->score );
    
    double weight = ALPHA * cluster->probability_condition_input;
    
    vec2_t delta_mean_in = MatVec.Gaussian2D.WeightedMeanUpdate( (vec2_t *)observation, &cluster->gaussian_in, weight );
    vec2_t delta_mean_out = MatVec.Gaussian2D.WeightedMeanUpdate( output, &cluster->gaussian_out, weight );
    
    LOG_GMM(GMM_DEBUG, "Gaussian mean in: [%.2f %.2f]\n", cluster->gaussian_in.mean.a, cluster->gaussian_in.mean.b);
    
    MatVec.Gaussian2D.WeightedUpdate( &delta_mean_in, &delta_mean_in,  &cluster->gaussian_in,  weight );
    MatVec.Gaussian2D.WeightedUpdate( &delta_mean_in, &delta_mean_out, &cluster->gaussian_out, weight );
    
    MatVec.Mat2x2.LLT( &cluster->gaussian_in.covariance, &cluster->llt_in );
    MatVec.Mat2x2.Inverse( &cluster->gaussian_in.covariance, &cluster->inv_covariance_in );
    
    GMMFunctions.Cluster.UpdateNormal( cluster );
    GMMFunctions.Cluster.UpdateLimits( cluster );
    
    ReportLabel( &cluster->labels, observation->label );
    
    cluster->timestamp = TIMESTAMP();
    
//    MatVec.Mat2x2.ScalarMultiply( 0.9999, &cluster->gaussian_in.covariance, &cluster->gaussian_in.covariance );
}

void GaussianMixtureCluster_GetScore( gaussian_mixture_cluster_t * cluster, vec2_t * input)
{
    vec2_t input_delta = { 0 };
    MatVec.Vec2.Subtract(input, &cluster->gaussian_in.mean, &input_delta);
    LOG_GMM(GMM_DEBUG_2, "Input delta 2: <%7.3f, %7.3f>\n", input_delta.a, input_delta.b);
    cluster->mahalanobis_sq = BOUNDU( MatVec.Gaussian2D.Covariance.MahalanobisSq( &cluster->inv_covariance_in, &input_delta), MAX_DISTANCE );
    cluster->probability_of_in = SafeExp( cluster->log_gaussian_norm_factor - 0.5 * cluster->mahalanobis_sq );
}

void GaussianMixtureCluster_UpdateNormal( gaussian_mixture_cluster_t * cluster )
{
    LOG_GMM(GMM_DEBUG_2, "LLT in: [%.2f %.2f | %.2f %.2f]", cluster->llt_in.a, cluster->llt_in.b, cluster->llt_in.c, cluster->llt_in.d);
    
#ifdef GMM_DEBUG_2
    double cholesky_dms = cluster->llt_in.a * cluster->llt_in.d;
#endif
    double norm_factor = -log( 2 * M_PI * sqrt( cluster->llt_in.a ) * sqrt( cluster->llt_in.d ) );
    LOG_GMM_BARE(GMM_DEBUG_2, " %.2f %.2f\n", cholesky_dms, norm_factor);
    cluster->log_gaussian_norm_factor = norm_factor;
}

void GaussianMixtureCluster_UpdateInputProbability( gaussian_mixture_cluster_t * cluster, double total_probability )
{
    cluster->probability_condition_input = total_probability > MIN_TOTAL_MIXTURE_PROBABILITY
    ? ZDIV( cluster->probability_of_in, total_probability ) : 0.f;
}

void GaussianMixtureCluster_ContributeToOutput( gaussian_mixture_cluster_t * cluster, vec2_t * input, vec2_t * output )
{
    vec2_t input_delta;
    MatVec.Vec2.Subtract(input, &cluster->gaussian_in.mean, &input_delta);
    LOG_GMM(GMM_DEBUG_2, "Input delta 1: <%7.3f, %7.3f>\n", input_delta.a, input_delta.b);
    vec2_t inv_covariance_delta = { 0 };
    MatVec.Mat2x2.DotVec2(&cluster->inv_covariance_in, &input_delta, &inv_covariance_delta);
    
    vec2_t input_covariance, pre_condition = { 0 }, pre_output;
    mat2x2 cov_out_T = { cluster->gaussian_out.covariance.a, cluster->gaussian_out.covariance.c,
                         cluster->gaussian_out.covariance.b, cluster->gaussian_out.covariance.d };
    MatVec.Mat2x2.DotVec2( &cov_out_T, &inv_covariance_delta, &input_covariance );
    MatVec.Vec2.Add( &cluster->gaussian_out.mean, &input_covariance, &pre_condition);
    MatVec.Vec2.ScalarMultiply( cluster->probability_condition_input, &pre_condition, &pre_output );
    MatVec.Vec2.Add( output, &pre_output, output);
}

void GaussianMixtureCluster_UpdateLimits( gaussian_mixture_cluster_t * cluster )
{
    double radius_y = cluster->gaussian_in.covariance.d * VALID_CLUSTER_STD_DEV;
    cluster->max_y = cluster->gaussian_in.mean.b + radius_y;
    cluster->min_y = cluster->gaussian_in.mean.b - radius_y;
}

void GaussianMixtureCluster_Weigh( gaussian_mixture_cluster_t * cluster )
{
    /* Find best two label contributes */
    double first = cluster->labels.average[0], second = 0., check;
    for( uint8_t i = 1; i < cluster->labels.num_valid; i++ )
    {
        check = cluster->labels.average[i];
        if( check > first )
        {
            second = first;
            first = check;
        }
        else if ( check > second )
            second = check;
    }
    double a = ( cluster->gaussian_in.covariance.b * cluster->gaussian_in.covariance.c ),
    b = ( cluster->gaussian_in.covariance.a * cluster->gaussian_in.covariance.d );
    double eccentricity_factor = ZDIV( a, b );
    cluster->weight = ( first + second ) * eccentricity_factor;
    cluster->primary_id = first;
    cluster->secondary_id = second;
}

void GaussianMixtureModel_Initialize( gaussian_mixture_model_t * model, const char * name )
{
    memset( model, 0, sizeof(gaussian_mixture_model_t) );
    model->name = name;
    LOG_GMM(GMM_DEBUG, "Initializing %s GMM\n", model->name);
    for( uint16_t i = 0; i < MAX_CLUSTERS; i++ )
        model->cluster[i] = &(model->cluster_mem[i]);
}

double GaussianMixtureModel_GetScoreSumOfClusters( gaussian_mixture_model_t * model, vec2_t * input )
{
    double score_sum = 0;
    gaussian_mixture_cluster_t * cluster;
    for( uint8_t i = 0; i < model->num_clusters; i++)
    {
        cluster = model->cluster[i];
        GMMFunctions.Cluster.GetScore( cluster, input );
        score_sum += cluster->probability_of_in;
    }
    return score_sum;
}

double GaussianMixtureModel_GetOutputAndBestDistance( gaussian_mixture_model_t * model, double total_probability, vec2_t * input, vec2_t * output )
{
    double best_match_distance = MAX_DISTANCE;
    gaussian_mixture_cluster_t * cluster;
    for( uint8_t i = 0; i < model->num_clusters; i++)
    {
        cluster = model->cluster[i];
        GMMFunctions.Cluster.UpdateInputProbability( cluster, total_probability );
        if( cluster->score > MIN_CLUSTER_SCORE)
            GMMFunctions.Cluster.ContributeToOutput( cluster, input, output );
        if( cluster->mahalanobis_sq < best_match_distance )
            best_match_distance = cluster->mahalanobis_sq;
    }
    return best_match_distance;
}

double GaussianMixtureModel_GetMaxError( gaussian_mixture_model_t * model, vec2_t * output, vec2_t * value, vec2_t * min_max_delta )
{
    vec2_t output_delta;
    MatVec.Vec2.Subtract( value, output, &output_delta );
    LOG_GMM(GMM_DEBUG_2, "Output delta: <%7.3f, %7.3f>\n", output_delta.a, output_delta.b);
    double a_error = fabs( ZDIV( output_delta.a, min_max_delta->a ) ),
    b_error = fabs( ZDIV( output_delta.b, min_max_delta->b ) );
    return MAX( a_error, b_error );
}

void GaussianMixtureModel_AddCluster( gaussian_mixture_model_t * model, observation_t * observation, vec2_t * value )
{
    uint16_t i = 0;
    for( ; i < model->num_clusters; i++ )
        if( model->cluster[i] == NULL ) break;
    GMMFunctions.Cluster.Initialize( model->cluster[i], observation, value );
    model->num_clusters++;
}

void GaussianMixtureModel_Update( gaussian_mixture_model_t * model, observation_t * observation, vec2_t * value )
{
    gaussian_mixture_cluster_t * cluster;
    LOG_GMM(GMM_DEBUG_CLUSTERS, "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
    for( uint8_t i = 0; i < model->num_clusters; i++ )
    {
        cluster = model->cluster[i];
        GMMFunctions.Cluster.Update( cluster, observation, value );
    }
    for( uint8_t i = 0; i < model->num_clusters; i++ )
    {
        gaussian_mixture_cluster_t * cluster = model->cluster[i];
//        double age = SECONDSSINCE( cluster->timestamp );
//        LOG_GMM(GMM_DEBUG, "Cluster %d age is %.3f\n", i, age);
        if( cluster->score < MIN_CLUSTER_SCORE
           || ISTIMEDOUT( cluster->timestamp, MAX_CLUSTER_LIFETIME )
           || isnan(cluster->log_gaussian_norm_factor) )
            GMMFunctions.Model.RemoveCluster( model, i );
    }
#ifdef GMM_DEBUG_CLUSTERS
    for( uint8_t i = 0; i < model->num_clusters; i++ )
    {
        cluster = model->cluster[i];
        LOG_GMM(GMM_DEBUG_CLUSTERS, "%d: µ<%6.3f, %7.3f> ∑[%6.3f, %6.3f; %6.3f, %6.3f] : weight:%5.3f score:%5.3f\n", i, cluster->gaussian_in.mean.a, cluster->gaussian_in.mean.b, cluster->gaussian_in.covariance.a, cluster->gaussian_in.covariance.b, cluster->gaussian_in.covariance.c, cluster->gaussian_in.covariance.d, cluster->weight, cluster->score);
    }
    LOG_GMM(GMM_DEBUG_CLUSTERS, "\n");
#endif
}

void GaussianMixtureModel_AddValue( gaussian_mixture_model_t * model, observation_t * observation, vec2_t * value )
{
    if( !model->num_clusters )
    {
        model->min_in = *(vec2_t *)( observation );
        model->max_in = model->min_in;
        model->min_out = *value;
        model->max_out = model->min_out;
    }
    else
    {
        model->min_in  = (vec2_t){ MIN( model->min_in.a, observation->density ), MIN( model->min_in.b, observation->thresh ) };
        model->max_in  = (vec2_t){ MAX( model->max_in.a, observation->density ), MAX( model->max_in.b, observation->thresh ) };
        model->min_out = (vec2_t){ MIN( model->min_out.a, value->a ), MIN( model->min_out.b, value->b ) };
        model->max_out = (vec2_t){ MAX( model->max_out.a, value->a ), MAX( model->max_out.b, value->b ) };
    }
    vec2_t output = { 0., 0. };
    vec2_t observation_vec = (vec2_t){ (double)observation->density, (double)observation->thresh };
    
    double total_probability = GMMFunctions.Model.GetScoreSumOfClusters( model, &observation_vec );
    double best_distance = GMMFunctions.Model.GetOutputAndBestDistance( model, total_probability, &observation_vec, &output);
    
    vec2_t min_max_delta;
    MatVec.Vec2.Subtract( &model->max_out, &model->min_out, &min_max_delta );
    double max_error = GMMFunctions.Model.GetMaxError( model, &output, value, &min_max_delta );
    
    GMMFunctions.Model.Update( model, observation, value );
    LOG_GMM(GMM_DEBUG_2, "Max error: %.2f\n", max_error);
    
    /* Add cluster if error or distance is to high for a cluster match */
    if( model->num_clusters < MAX_CLUSTERS
       && ( !model->num_clusters
           || ( ( max_error > MAX_ERROR )
               && ( best_distance > MAX_MAHALANOBIS_SQ ) ) ) )
        GMMFunctions.Model.AddCluster( model, observation, value );
}

void GaussianMixtureModel_RemoveCluster( gaussian_mixture_model_t * model, uint16_t index )
{
    model->num_clusters--;
    
    /* Swap pointer to cluster being removed with the point to the last cluster */
    gaussian_mixture_cluster_t * remove_cluster = model->cluster[index];
    model->cluster[index] = model->cluster[model->num_clusters];
    model->cluster[model->num_clusters] = remove_cluster;
}

#endif
