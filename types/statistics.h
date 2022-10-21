//
//  statistics.h
//  combine
//
//  Created by Matthew Fonken on 8/10/19.
//  Copyright © 2019 Matthew Fonken. All rights reserved.
//

#ifndef statistics_h
#define statistics_h

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

////#include "rho_global.h"
typedef double floating_t;
//typedef int16_t uint16_t;

typedef struct
{
    floating_t avg;
    uint16_t n;
} cumulative_avg_t;

typedef struct
{
    floating_t avg;
    floating_t S;
    uint16_t n, max_n;
} cumulative_avg_stdv_t;

void Statistics_Calculate_CumulativeMoments( floating_t, floating_t, floating_t *, floating_t *, floating_t * );
void Statistics_Calculate_CumulativeAverage( floating_t, floating_t *, uint16_t * );
void CumulateAverageStatistics( floating_t, cumulative_avg_t * );
void Statistics_Calculate_CumulateAverageStandardDeviation( floating_t, cumulative_avg_stdv_t * );
floating_t Statistic_Calculate_Variance( cumulative_avg_stdv_t * );

#ifdef __cplusplus
}
#endif

#endif /* statistics_h */
