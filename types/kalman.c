#include "kalman.h"
#ifndef TIMESTAMP
#include "timestamp.h"
#endif
/*~ SOURCE: http://preview.tinyurl.com/9djhrem */

void InitializeKalman( kalman_filter_t * k, floating_t v, floating_t ls, floating_t minv, floating_t maxv, kalman_uncertainty_c uncertainty )
{
    k->lifespan    = ls;
    k->uncertainty.value   = uncertainty.value;
    k->uncertainty.bias    = uncertainty.bias;
    k->uncertainty.sensor  = uncertainty.sensor;
    
    k->origin    = TIMESTAMP();
    k->timestamp = k->origin;
    
    k->min_value = minv;
    k->max_value = maxv;
    
    k->acceleration_mode = false;
    
    ResetKalman(k, v);
}

void ResetKalman( kalman_filter_t * k, floating_t v )
{
    k->K[0]        = 0; /* Gain */
    k->K[1]        = 0;
    k->P[0][0]     = 0; /* Error covariance */
    k->P[0][1]     = 0;
    k->P[1][0]     = 0;
    k->P[1][1]     = 0;
    k->rate        = 0;
    k->bias        = 0;
    k->prev        = 0;
    k->velocity    = 0;
    k->variance    = 0;
    k->flag        = 0;
    k->score       = 0;
    k->value       = v;
    k->origin      = TIMESTAMP();
}

void PredictKalman( kalman_filter_t * k, floating_t rate_new )
{
    floating_t delta_time = TIMESTAMP() - k->timestamp;
    
    /* Quick expiration check */
    if(delta_time > k->lifespan)
    {
        k->timestamp = TIMESTAMP();
        return;
    }
    
    /* \hat{x}_{k\mid k-1} = F \hat{x_{k-1\mid k-1}} + B \dot{\theta}_k */
    k->velocity = k->value - k->prev;
    k->prev     = k->value;
    k->rate     = k->acceleration_mode ?
        delta_time * rate_new - k->bias + k->velocity
        : rate_new - k->bias;
    k->value   += delta_time * k->rate;
    k->value    = BOUND(k->value, k->min_value, k->max_value);
    
    /* P_{k\mid k-1} = F P_{k-1\mid k-1} F^T + Q_k */
    floating_t dt_P_1_1 = delta_time * k->P[1][1];
    k->P[0][0]   += delta_time * ( dt_P_1_1 -
                                  k->P[0][1] -
                                  k->P[1][0] +
                                  k->uncertainty.value );
    k->P[0][1]   -= dt_P_1_1;
    k->P[1][0]   -= dt_P_1_1;
    k->P[1][1]   += k->uncertainty.bias * delta_time;
    
    LOG_KALMAN(KALMAN_DEBUG_2, "Prediction - Value:%.2f Rate:%.2f Velocity:%.2f\n", k->value, k->rate, k->velocity);
}

void UpdateKalman( kalman_filter_t * k, floating_t value_new )
{
    /* S_k = H P_{k\mid k-1} H^T + R */
    floating_t S_ = 1. / ( k->P[0][0] + k->uncertainty.sensor );
    
    /* K_k = P_{k\mid k-1} H^T S^{-1}_k */
    k->K[0]       = k->P[0][0] * S_;
    k->K[1]       = k->P[1][0] * S_;
    
    /* \tilde{y} = z_k - H \hat{x}_{k\mid k-1} */
    floating_t delta_value = value_new - k->value;
    k->value     += k->K[0] * delta_value;
    k->bias      += k->K[1] * delta_value;
    
    k->P[0][0]   -= k->K[0] * k->P[0][0];
    k->P[0][1]   -= k->K[0] * k->P[0][1];
    k->P[1][0]   -= k->K[1] * k->P[0][0];
    k->P[1][1]   -= k->K[1] * k->P[0][1];
    
    k->timestamp  = TIMESTAMP();
    
    k->value = BOUND(k->value, k->min_value, k->max_value);
    
    k->estimation_error = k->value - value_new;
    LOG_KALMAN(KALMAN_DEBUG_2, "Update - Value:%.2f Bias:%.2f K:%.2f|%.2f\n", k->value, k->bias, k->K[0], k->K[1]);
};

floating_t TickKalman( kalman_filter_t * k, floating_t value_new )
{
    return Kalman.Step( k, value_new, k->velocity );
}

floating_t StepKalman( kalman_filter_t * k, floating_t value_new, floating_t rate_new )
{
    LOG_KALMAN(KALMAN_DEBUG_2, "Step - Id:%p NewVal:%.2f NewRate:%.2f\n", k, value_new, rate_new);
    PredictKalman(k, rate_new);
    UpdateKalman(k, value_new);
    return k->value;
}

bool IsKalmanExpired( kalman_filter_t * k )
{
    return ((TIMESTAMP() - k->timestamp) > k->lifespan);
}

inline floating_t ScoreKalman( kalman_filter_t * k )
{
    floating_t score = k->K[0];
    if(k->flag) score = 0.;
    k->score = score;
    return score;
}

void PunishKalman( kalman_filter_t * k )
{
    k->K[0] *= KALMAN_PUNISH_FACTOR;
    if( ScoreKalman(k) < MIN_KALMAN_GAIN )
        ResetKalman(k, 0);
}

void PrintKalman( kalman_filter_t * k )
{
    LOG_KALMAN(KALMAN_DEBUG, "Val: %.4f | Rate: %.4f | Vel:%.4f\n", k->value, k->rate, k->velocity);
    LOG_KALMAN(KALMAN_DEBUG, "Bias: %.4f | Var: %.4f | Scr:%.4f\n", k->bias, k->variance, k->score);
    LOG_KALMAN(KALMAN_DEBUG, "K:\t[%.4f][%.4f]\n", k->K[0], k->K[1]);
    LOG_KALMAN(KALMAN_DEBUG, "P:\t[%.4f][%.4f]\n", k->P[0][0], k->P[0][1]);
    LOG_KALMAN(KALMAN_DEBUG, "  \t[%.4f][%.4f]\n", k->P[1][0], k->P[1][1]);
}

#ifdef MATVEC_LIB
gaussian1d_t GetGaussian1DKalman( kalman_filter_t * k )
{
    return (gaussian1d_t){ k->value, k->P[0][0] };
}
#endif
