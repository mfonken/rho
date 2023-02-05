//
//  hmm.c
//  hmmtest
//
//  Created by Matthew Fonken on 2/10/19.
//  Copyright © 2019 Matthew Fonken. All rights reserved.
//

#ifdef __PSM__

#include "hmm.h"

#define HMM_ERROR_PADDING 1e-4
#define SOFTEN(X) (X==1?(1-HMM_ERROR_PADDING):(X==0?HMM_ERROR_PADDING:X))

void HiddenMarkovModel_Initialize( hidden_markov_model_t * model, const char * name )
{
    memset( model, 0, sizeof(hidden_markov_model_t) );
    model->name = name;
    
    KumaraswamyFunctions.Initialize( &model->k_dist, NUM_STATES + 1, (floating_t[])DEFAULT_KUMARASWAMY_BANDS );
    HiddenMarkovModel_.InitializeTransitionMatrix( model );
    
    memcpy( &model->B, (emission_t[])DEFAULT_OBSERVATION_LIST, sizeof((emission_t[])DEFAULT_OBSERVATION_LIST) );
    memcpy( &model->pi, (state_vector_t)DEFAULT_STATE_VECTOR, sizeof(state_vector_t) );
    
    HiddenMarkovModel_.Print( model );
}

void HiddenMarkovModel_InitializeTransitionMatrix( hidden_markov_model_t * model )
{
    for( uint8_t i = 0; i < NUM_STATES; i++ )
        KumaraswamyFunctions.GetVector( &model->k_dist, i + 1, model->A[i] );
}

uint8_t HiddenMarkovModel_ReportObservationTo( hidden_markov_model_t * model, hmm_observation_t o )
{
#ifdef HMM_2D_EMISSIONS
    LOG_HMM(HMM_REPORT, "Reporting: {%.4f, %.4f} - [%d]\n", o.a, o.b, model->O.index.next);
#else
    LOG_HMM(HMM_REPORT, "Reporting %.4f - [%d]\n", o, model->O.index.next);
#endif
    return PushToObservationBuffer( &model->O, o );
}

floating_t HiddenMarkovModel_UpdateAll( hidden_markov_model_t * model )
{
    HiddenMarkovModel_.Update.Alpha(  model );
    HiddenMarkovModel_.Update.Beta(   model );
    HiddenMarkovModel_.Update.Gamma(  model );
    HiddenMarkovModel_.Update.Xi(     model );
    floating_t new_P = HiddenMarkovModel_.Update.Probability( model ),
    delta_P = model->P - new_P;
    model->P = new_P;
//    LOG_HMM(HMM_DEBUG, "Update %s probability ∆: %.4f\n", model->name, delta_P);
    return delta_P;
}

void HiddenMarkovModel_UpdateAlpha( hidden_markov_model_t * model )
{
    memset( model->alpha, 0., sizeof(model->alpha) );
    uint8_t T = model->O.length;
    hmm_observation_t o = GetIndexObservationBuffer( &model->O, 0 );
    floating_t p;
    
    /* Source - https://en.wikipedia.org/wiki/Baum–Welch_algorithm#cite_note-6 */
    /* 1. α_i(0) = π_i x B_i(y_0) */
    for( uint8_t i = 0; i < NUM_STATES; i++ )
    {
        floating_t p = model->pi[i], P = GetProbabilityFromEmission( &model->B[i], o );
        model->alpha[0][i] = p * P;
    }
    
    /*  2. α_i(t) = B_i(y_(t)) x ∑_{j=0}^N ( α_j(t-1) x A_{ji} ) */
    for( uint8_t t = 1; t < T; t++ )
    {
        for( uint8_t i = 0; i < NUM_STATES; i++ )
        {
            floating_t sum = 0;
            for( uint8_t j = 0; j < NUM_STATES; j++ )
                sum += model->alpha[t - 1][j] * model->A[j][i];
            o = GetIndexObservationBuffer( &model->O, t );
            p = GetProbabilityFromEmission( &model->B[i], o );
            model->alpha[t][i] = sum * p;
        }
    }
}

void HiddenMarkovModel_UpdateBeta(hidden_markov_model_t * model )
{
    memset( model->beta, 0., sizeof(model->beta) );
    uint8_t T = model->O.length;
    hmm_observation_t o;
    floating_t p;
    
    /* Source - https://en.wikipedia.org/wiki/Baum–Welch_algorithm#cite_note-6 */
    /* 1. β_i(T-1) = 1 */
    for( uint8_t i = 0; i < NUM_STATES; i++ )
        model->beta[T - 1][i] = 1.;
    
    /* 2. β_i(t) = ∑_{j=1}^N ( β_j(t+1) x A_{ij} x B_j(y_{t+1}) ) */
    for( int8_t t = T - 2; t >= 0; --t)
    {
        for( uint8_t i = 0; i < NUM_STATES; i++ )
        {
            floating_t sum = 0;
            for( uint8_t j = 0; j < NUM_STATES; j++ )
            {
                o = GetIndexObservationBuffer( &model->O, t + 1 );
                p = GetProbabilityFromEmission( &model->B[j], o );
                sum += model->beta[t + 1][j] * model->A[i][j] * p;
            }
            model->beta[t][i] = SOFTEN( sum );
        }
    }
}

void HiddenMarkovModel_UpdateGamma(hidden_markov_model_t * model )
{
    memset( model->gamma, 0., sizeof(model->gamma) );
    uint8_t T = model->O.length;
    
    /* Source - https://en.wikipedia.org/wiki/Baum–Welch_algorithm#cite_note-6 */
    /* γ_i(t) = ( α_i(t) x β_i(t) ) / ( ∑_{j=1)^N ( α_j(t) x β_j(t) ) ) */
    for( uint8_t t = 0; t < T; t++ )
    {
        floating_t sum = 0;
        for( uint8_t i = 0; i < NUM_STATES; i++ )
        {
            model->gamma[t][i] = model->alpha[t][i] * model->beta[t][i];
            sum += model->gamma[t][i];
        }
        if( sum == 0 ) continue;
        for( uint8_t i = 0; i < NUM_STATES; i++ )
            model->gamma[t][i] /= sum;
    }
}

void HiddenMarkovModel_UpdateXi( hidden_markov_model_t * model )
{
    memset( model->xi, 0., sizeof(model->xi) );
    uint8_t T = model->O.length;
    hmm_observation_t o;
    floating_t p;
    
    /* Source - https://en.wikipedia.org/wiki/Baum–Welch_algorithm#cite_note-6 */
    /* ξ_{ij}(t) = ( α_i(t) x A_{ij} x β_j(t+1) x B_j(y_{t+1}) ) / ( ∑_{i=1}^N ∑_{j=1)^N ( α_i(t) x A_{ij} x β_j(t+1) x B_j(y_{t+1}) ) ) */
    for( uint8_t t = 0; t < T; t++ )
    {
        floating_t sum = 0;
        for( uint8_t i = 0; i < NUM_STATES; i++ )
        {
            for( uint8_t j = 0; j < NUM_STATES; j++ )
            {
                o = GetIndexObservationBuffer( &model->O, t + 1 );
                p = GetProbabilityFromEmission( &model->B[j], o );
                model->xi[i][t][j] = model->alpha[t][i] * model->A[i][j] * model->beta[t + 1][j] * p;
                sum += model->xi[i][t][j];
            }
        }
        if( sum == 0 ) continue;
        for( uint8_t i = 0; i < NUM_STATES; i++ )
            for( uint8_t j = 0; j < NUM_STATES; j++ )
                model->xi[i][t][j] /= sum;
    }
}

floating_t HiddenMarkovModel_UpdateProbability( hidden_markov_model_t * model )
{
    /* Source - http://www.utstat.toronto.edu/*rsalakhu/sta4273/notes/Lecture11.pdf - Computing Likelihood */
    floating_t sum = 0;
    uint8_t T = model->O.length;
    
    /* P(T) = ∑_{i=1}^N α_i(T) */
    for( uint8_t i = 0; i < NUM_STATES; i++ )
        sum += model->alpha[T - 1][i];
    return sum;
}

void HiddenMarkovModel_UpdateInitialProbabilities( hidden_markov_model_t * model )
{
    /* Source - https://en.wikipedia.org/wiki/Baum–Welch_algorithm#cite_note-6 */
    /* π_i^* = γ_i(0) */
    double sum = 0.;
    for( uint8_t i = 0; i < NUM_STATES; i++ )
    {
        sum += model->gamma[0][i];
        model->pi[i] = SOFTEN( model->gamma[0][i] );
    }
    if( sum == 0 )
        model->pi[0] = SOFTEN( 1. );
    /// TODO: Add better failure case
}

void HiddenMarkovModel_UpdateTransitionProbabilities( hidden_markov_model_t * model )
{
    uint8_t T = model->O.length;
    floating_t n = 0, d = 0;
    
    /* Source - https://en.wikipedia.org/wiki/Baum–Welch_algorithm#cite_note-6 */
    /* A_{ij}^* = ( ∑_{t=1}^{T-1} ξ_{ij}(t) ) / ( ∑_{t=1}^{T-1} γ_i(t) ) */
    for( uint8_t i = 0; i < NUM_STATES; i++ )
    {
        floating_t sum = 0, curr;
        for( uint8_t j = 0; j < NUM_STATES; j++ )
        {
            n = 0; d = 0;
            for( uint8_t t = 0; t < T - 1; t++ )
            {
                n += model->xi[i][t][j];
                d += model->gamma[t][i];
            }
            curr = ZDIV( n, d );
            model->A[i][j] = SOFTEN( curr );
            sum += curr;
        }
        if( sum == 0 )
            model->A[i][i] = SOFTEN( 1. );
        /// TODO: Add better failure case (maybe with Beta(or K) distribution)
    }
}

void HiddenMarkovModel_UpdateEmissionProbabilities( hidden_markov_model_t * model )
{
#ifdef HMM_GAUSSIAN_EMISSIONS
    /* Source - https://sambaiga.github.io/ml/hmm/2017/06/12/hmm-gausian.html */
    /* P( y_t | s_t=i ) = Ν(nu)( y_t | µ_i, ∑_i ) */
    
    uint8_t T = model->O.length;
    floating_t gamma_sum = 0.;
    hmm_observation_t o;
#ifdef HMM_2D_EMISSIONS
    vec2_t mean_diff = { 0. }, mean_sum = { 0. }, working_vec;
    mat2x2 cov_est = { 0. }, cov_sum = { 0. }, working_mat;
#else
    floating_t mean_diff = 0., sum = 0.;
#endif
    
    for( uint8_t i = 0; i < NUM_STATES; i++ )
    {
        /* µ_i = ( ∑_{t=1}^T γ_i(t) x y(t) ) / ( ∑_{t=1}^T γ_i(t) ) */
#ifdef HMM_2D_EMISSIONS
        mean_sum = (vec2_t){ 0. };
#else
        sum = 0.;
#endif
        gamma_sum = 0.;
        for( uint8_t t = 0; t < T; t++ )
        {
            o = GetIndexObservationBuffer( &model->O, t );
#ifdef HMM_2D_EMISSIONS
            MatVec.Vec2.ScalarMultiply(model->gamma[t][i], &o, &working_vec);
            MatVec.Vec2.Add( &mean_sum, &working_vec, &mean_sum );
#else
            sum += model->gamma[t][i] * o;
#endif
            gamma_sum += model->gamma[t][i];
        }
#ifdef HMM_2D_EMISSIONS
        if( gamma_sum == 0 ) continue;
        floating_t inv_gamma_sum = ZDIV( 1, gamma_sum );
        MatVec.Vec2.ScalarMultiply( inv_gamma_sum, &mean_sum, &model->B[i].mean );
#else
        model->B[i].mean = SOFTEN( ZDIV( sum, gamma_sum ) );
#endif
        
        /* ∑_i = ( ∑_{t=1}^T γ_i(t) x [y(t) - µ_i][y(t) - µ_i]^T ) / ( ∑_{t=1}^T γ_i(t) ) */
#ifdef HMM_2D_EMISSIONS
        cov_sum = (mat2x2){ 0. };
#else
        sum = 0.;
#endif
        for( uint8_t t = 0; t < T; t++ )
        {
            o = GetIndexObservationBuffer( &model->O, t );
#ifdef HMM_2D_EMISSIONS
            MatVec.Vec2.Subtract( &o, &model->B[i].mean, &mean_diff );
            MatVec.Vec2.AAT( &mean_diff, &cov_est );
            MatVec.Mat2x2.ScalarMultiply( model->gamma[t][i], &cov_est, &working_mat );
            MatVec.Mat2x2.Add( &cov_sum, &working_mat, &cov_sum );
#else
            mean_diff = (floating_t)o - model->B[i].mean;
            sum += model->gamma[t][i] * mean_diff * mean_diff;
#endif
        }
#ifdef HMM_2D_EMISSIONS
        MatVec.Mat2x2.ScalarMultiply( gamma_sum, &cov_sum, &model->B[i].covariance );
        MatVec.Gaussian2D.Covariance.Limit( &model->B[i].covariance );
#else
        model->B[i].std_dev = SOFTEN( ZDIV( sum, gamma_sum ) );
#endif
    }
#else
    floating_t n, d; uint8_t T = model->O.length; hmm_observation_t o;
    /* Source - https://en.wikipedia.org/wiki/Baum–Welch_algorithm#cite_note-6 */
    /* B_i^*(v_k) = ( ∑_{t=1}^{T-1} ( 1_{y_t=v_k} x γ_i(t) ) ) / ( ∑_{t=1}^{T-1} γ_i(t) ) */
    for( uint8_t j = 0; j < NUM_STATES; j++ )
    {
        for( uint8_t k = 0; k < NUM_OBSERVATION_SYMBOLS; ++k)
        {
            n = 0; d = 0;
            for( uint8_t t = 0; t < T; ++t )
            {
                o = BOUNDU(GetIndexObservationBuffer( &model->O, t ), NUM_OBSERVATION_SYMBOLS - 1);
                if( o == k )
                    n += model->gamma[t][j];
                d += model->gamma[t][j];
            }
            model->B[j][k] = SOFTEN( ZDIV( n, d ) );
        }
    }
#endif
}

void HiddenMarkovModel_BaumWelchSolve( hidden_markov_model_t * model, floating_t DELTA )
{
    if( model->O.length < MAX_OBSERVATIONS - 1 ) return;
    HiddenMarkovModel_.Update.All( model );
    do
    {
        HiddenMarkovModel_.Update.Pi( model );
        HiddenMarkovModel_.Update.A( model );
        HiddenMarkovModel_.Update.B( model );
#ifdef HMM_DEBUG
        HiddenMarkovModel_.Print( model );
#endif
    } while( HiddenMarkovModel_.Update.All( model ) > DELTA );
    model->O.length = 0;
}

void HiddenMarkovModel_Print( hidden_markov_model_t * model )
{
    LOG_HMM(HMM_DEBUG, "%s:\n", model->name );
    LOG_HMM_BARE(HMM_DEBUG, "A:\n");
    for( uint8_t i = 0; i < NUM_STATES; i++ )
    {
        LOG_HMM_BARE(HMM_DEBUG, "\t[");
        for( uint8_t j = 0; j < NUM_STATES; j++ )
            LOG_HMM_BARE(HMM_DEBUG, "%6.4f%s", model->A[i][j], (j==NUM_STATES-1?"":", "));
        LOG_HMM_BARE(HMM_DEBUG, "]\n");
    }
    LOG_HMM_BARE(HMM_DEBUG, "B:\n");
    for( uint8_t i = 0; i < NUM_STATES; i++ )
    {
        LOG_HMM_BARE(HMM_DEBUG, "\t");
#ifdef HMM_GAUSSIAN_EMISSIONS
#ifdef HMM_2D_EMISSIONS
        LOG_HMM_BARE(HMM_DEBUG, "µ:(%8.4f, %8.4f) ∑:[%7.4f, %7.4f; %7.4f, %7.4f]", model->B[i].mean.a, model->B[i].mean.b, model->B[i].covariance.a, model->B[i].covariance.b, model->B[i].covariance.c, model->B[i].covariance.d);
#else
        LOG_HMM_BARE(HMM_DEBUG, "[%8.4f,%8.4f]", model->B[i].mean, model->B[i].std_dev);
#endif
#else
        LOG_HMM_BARE(HMM_DEBUG, "[");
        for( uint8_t k = 0; k < NUM_OBSERVATION_SYMBOLS; k++)
            LOG_HMM_BARE(HMM_DEBUG, "%8.4f%s", model->B[i][k], (k==NUM_OBSERVATION_SYMBOLS-1?"":","));
        LOG_HMM_BARE(HMM_DEBUG, "]");
#endif
        if( !i ) { LOG_HMM_BARE(HMM_DEBUG, " \tpi: "); }
        else { LOG_HMM_BARE(HMM_DEBUG, " \t    "); }
        LOG_HMM_BARE(HMM_DEBUG, "[%.4f]\n", model->pi[i]);
    }
    LOG_HMM_BARE(HMM_DEBUG, "\n");
}
#endif
