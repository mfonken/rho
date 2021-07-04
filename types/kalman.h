#ifndef kalman_h
#define kalman_h

#include <stdlib.h>
#include <stdbool.h>

#include "rho_global.h"

//#define MATVEC_LIB

#ifdef MATVEC_LIB
#include "matvec.h"
#endif

#define KALMAN_PUNISH_FACTOR  0.7
#define MIN_KALMAN_GAIN       0.001
#define KALMAN_MATURATION     3 // Seconds

#ifdef __cplusplus
extern "C" {
#endif
    
    /*~ Kalman Uncertainties */
    typedef struct
    {
        floating_t value; /* Q */
        floating_t bias;
        floating_t sensor; /* R */
    } kalman_uncertainty_c;
    
    /*~ Kalman structure */
    typedef struct
    {
      floating_t
        K[2],
        P[2][2],
        rate,
        bias,
        value,
        prev,
        velocity,
        variance,
        lifespan,
        min_value,
        max_value,
        timestamp,
        origin,
        score,
        estimation_error;
      kalman_uncertainty_c
        uncertainty;
      bool
        acceleration_mode,
        sorted,
        valid,
        flag;
    } kalman_filter_t;
    
    void  InitializeKalman( kalman_filter_t *, floating_t, floating_t, floating_t, floating_t, kalman_uncertainty_c );
    void       ResetKalman( kalman_filter_t *, floating_t );
    void     PredictKalman( kalman_filter_t *, floating_t );
    void      UpdateKalman( kalman_filter_t *, floating_t );
    floating_t  TickKalman( kalman_filter_t *, floating_t );
    floating_t  StepKalman( kalman_filter_t *, floating_t, floating_t );
    bool   IsKalmanExpired( kalman_filter_t * );
    floating_t ScoreKalman( kalman_filter_t * );
    void      PunishKalman( kalman_filter_t * );
    void       PrintKalman( kalman_filter_t * );
#ifdef MATVEC_LIB
    gaussian1d_t GetGaussian1DKalman( kalman_filter_t * );
#endif
    
    struct kalman {
        void (* Initialize)( kalman_filter_t *, floating_t, floating_t, floating_t, floating_t, kalman_uncertainty_c );
        void (*      Reset)( kalman_filter_t *, floating_t );
        void (*    Predict)( kalman_filter_t *, floating_t );
        void (*     Update)( kalman_filter_t *, floating_t );
        floating_t (* Tick)( kalman_filter_t *, floating_t );
        floating_t (* Step)( kalman_filter_t *, floating_t, floating_t );
        bool (*  IsExpired)( kalman_filter_t * );
        floating_t (*Score)( kalman_filter_t * );
        void (*     Punish)( kalman_filter_t * );
        void (*      Print)( kalman_filter_t * );
#ifdef MATVEC_LIB
        gaussian1d_t (*Gaussian1D)( kalman_filter_t * );
#endif
    };
    
    static const struct kalman Kalman =
    {
        .Initialize = InitializeKalman,
        .Reset = ResetKalman,
        .Predict = PredictKalman,
        .Update = UpdateKalman,
        .Tick = TickKalman,
        .Step = StepKalman,
        .IsExpired = IsKalmanExpired,
        .Score = ScoreKalman,
        .Punish = PunishKalman,
        .Print = PrintKalman,
#ifdef MATVEC_LIB
        .Gaussian1D = GetGaussian1DKalman
#endif
    };
    
#ifdef __cplusplus
}
#endif

#endif /* kalman_h */

