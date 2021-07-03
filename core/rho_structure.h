/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  File: rho_structure.h
 *  Group: Rho Core
 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

#ifndef rho_structure_h
#define rho_structure_h

#define DUAL_FILTER_CYCLE(X) \
for( X = 0, X##_ = 1; X##_ <= 2; X++, X##_++ )

#define BOUNDED_CYCLE_DUAL(A,B,C,D,E,F,G) \
for(A = B, D = 0, F = 0; A > C; --A, D = E[A], F = G[A] )

#define PACKET_ADDRESS_INITIALIZER(r)               \
{                                                   \
(address_t)&r.y.tracking_filters[0].value, /* px */  \
(address_t)&r.x.tracking_filters[0].value, /* py */  \
(address_t)&r.y.tracking_filters[1].value, /* sx */  \
(address_t)&r.x.tracking_filters[1].value, /* sy */  \
(address_t)&r.probabilities.P[1],         /* pp */  \
(address_t)&r.probabilities.P[2],         /* ap */  \
(address_t)&r.probabilities.P[3]          /* ap */  \
}

#endif /* rho_structure_h */
