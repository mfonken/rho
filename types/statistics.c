//
//  statistics.c
//  combine
//
//  Created by Matthew Fonken on 8/10/19.
//  Copyright © 2019 Matthew Fonken. All rights reserved.
//

#include "statistics.h"

#ifndef MIN
#define MIN(A,B) (A<B?A:B)
#endif

void GenerateCumulativeMomentsStatistics( floating_t v, floating_t i, floating_t *m0, floating_t *m1, floating_t *n )
{
#ifdef __USE_RUNNING_AVERAGE__
    floating_t n_=1/(++(*n));
    *m0+=((v-*m0)*n_);
    *m1+=(((v*i)-*m1)*n_);
#else
    ++(*n);
    *m0+=v;
    *m1+=v*i;
#endif
}

void GenerateCumulativeAverageStatistics( floating_t new_val, floating_t *avg, uint16_t * n )
{
    *avg+=(new_val-*avg)/(floating_t)(++(*n));
}

inline void CumulateAverageStatistics( floating_t new_val, cumulative_avg_t * stat )
{
    GenerateCumulativeAverageStatistics( new_val, &stat->avg, &stat->n );
}

inline void CumulateAverageStandardDeviationStatistics( floating_t new_val, cumulative_avg_stdv_t * stat )
{
    floating_t avg_ = stat->avg;
    uint16_t t = MIN(stat->n, stat->max_n);
    GenerateCumulativeAverageStatistics( new_val, &stat->avg, &t);
    floating_t s = ( new_val - avg_ ) * ( new_val - stat->avg );
    stat->S += MIN(s, 10);
    stat->n++;
    //if(stat->n++ > stat->max_n) stat->n = stat->max_n;
}

inline floating_t GetVarianceFromStatistic( cumulative_avg_stdv_t * stat )
{
    if( stat->n <= 1 ) return 0.;
    return ( stat->S / ( stat->n - 1 ) );
}
