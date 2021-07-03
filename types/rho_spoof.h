//
//  rho_spoof.h
//  combine
//
//  Created by Matthew Fonken on 10/20/19.
//  Copyright © 2019 Matthew Fonken. All rights reserved.
//

#ifndef rho_spoof_h
#define rho_spoof_h

#define LOG_KALMAN(...)
#define LOG_FSM(...)

#ifndef BOUND
#define BOUND(X,MIN,MAX)        ( ( X < MIN ) ? MIN : BOUNDU( X, MAX ) ) // Bound in upper and lower range
#endif

#define MAX_OBSERVATIONS 1 << 5

typedef double floating_t;
typedef uint16_t index_t;

#endif /* rho_spoof_h */
