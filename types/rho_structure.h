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

#endif /* rho_structure_h */
