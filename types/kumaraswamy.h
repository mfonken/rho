//
//  kumaraswamy.h
//  hmmtest
//
//  Created by Matthew Fonken on 2/11/19.
//  Copyright © 2019 Matthew Fonken. All rights reserved.
//

#ifndef kumaraswamy_h
#define kumaraswamy_h

#ifdef __cplusplus
extern "C" {
#endif

//#include "control_types.h"

#define STATE_KUMARASWAMY_INTERVALS { 0.2, 0.4, 0.65, 1.0 }
    
#define KUMARASWAMY_PDF(X,A,B) ( A * B * pow( x, ( A - 1 ) ) * pow( 1 - pow( x, A ), B - 1 ) )
#define KUMARASWAMY_CDF(X,A,B) ( 1 - pow( 1 - pow( X, A ), B) )
#define KUMARASWAMY_NUM_BANDS   4

typedef struct
{
    double alpha, beta;
    double bands[KUMARASWAMY_NUM_BANDS];
} kumaraswamy_t;

static void Kumaraswamy_Initialize( kumaraswamy_t * k, double beta, double * bands )
{
    k->beta = beta;
    memcpy( &k->bands, bands, sizeof(double[KUMARASWAMY_NUM_BANDS]) );
}

static inline double Kumaraswamy_PerformCDF( kumaraswamy_t * k, double x )
{
    return KUMARASWAMY_CDF( x, k->alpha, k->beta );
}

static void Kumaraswamy_GetVector( kumaraswamy_t * k, double alpha, double * interval )
{
    k->alpha = alpha;
    double curr_CDF, prev_CDF = 0.;
    for( uint8_t i = 0; i < KUMARASWAMY_NUM_BANDS; i++ )
    {
        curr_CDF = Kumaraswamy_PerformCDF( k, k->bands[i] );
        if( curr_CDF < prev_CDF )
            interval[i] = 0.;
        else
        {
            interval[i] = curr_CDF - prev_CDF;
            prev_CDF = curr_CDF;
        }
    }
}

typedef struct
{
    void (*Initialize)( kumaraswamy_t *, double, double * );
    double(*PerformCDF)( kumaraswamy_t *, double );
    void (*GetVector)( kumaraswamy_t *, double, double * );
} kumaraswamy_functions_t;

static const kumaraswamy_functions_t KumaraswamyFunctions =
{
    .Initialize = Kumaraswamy_Initialize,
    .PerformCDF = Kumaraswamy_PerformCDF,
    .GetVector = Kumaraswamy_GetVector
};
    
#ifdef __cplusplus
}
#endif

#endif /* kumaraswamy_h */
