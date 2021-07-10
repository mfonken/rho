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

//#include "rho_global.h"
typedef double floating_t;
typedef int16_t index_t;

typedef struct
{
    floating_t avg;
    index_t n;
} cumulative_avg_t;

typedef struct
{
    floating_t avg;
    floating_t S;
    index_t n, max_n;
} cumulative_avg_stdv_t;

void GenerateCumulativeMomentsStatistics( floating_t, floating_t, floating_t *, floating_t *, floating_t * );
void GenerateCumulativeAverageStatistics( floating_t, floating_t *, index_t * );
void CumulateAverageStatistics( floating_t, cumulative_avg_t * );
void CumulateAverageStandardDeviationStatistics( floating_t, cumulative_avg_stdv_t * );
floating_t GetVarianceFromStatistic( cumulative_avg_stdv_t * );

#ifdef __cplusplus
}
#endif

#endif /* statistics_h */
