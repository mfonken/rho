/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  File: rho_global.h
 *  Group: Rho Core
 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

#ifndef rho_global_h
#define rho_global_h

#ifndef __RHO_STAND_ALONE__
#include "../UniSM/system_master.h"
#include "../UniLog/unilog.h"
#include "../App/states.h"
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "unilog.h"

typedef double          floating_t;
typedef uint8_t         byte_t;
typedef uint16_t        index_t;
typedef uint16_t        density_t;
typedef int16_t         sdensity_t;
typedef int16_t         variance_t;
typedef uint32_t        density_2d_t;
typedef floating_t      timestamp_t;
typedef uint8_t         capture_t;

#if defined __linux || defined __APPLE__
typedef uint32_t *          address_t;
#else
typedef uint32_t	    address_t;
#endif

#define byte_t_max          ( (sizeof(byte_t)       << 3 ) - 1 )
#define uint16_t_max        ( (sizeof(uint16_t)      << 3 ) - 1 )
#define density_t_max       ( (sizeof(density_t)    << 3 ) - 1 )
#define density_2d_t_max    ( (sizeof(density_2d_t) << 3 ) - 1 )
#define sdensity_t_max      ( (sizeof(sdensity_t) << 3 ) - 1 )

//#define __USE_DECOUPLING__
//#define USE_BACKGROUNDING
// #define ALLOW_NEGATIVE_REDISTRIBUTION

#define RHO_DEBUG               DEBUG_2
#define RHO_DEBUG_2             DEBUG_1
#define RHO_DEBUG_INIT          DEBUG_1
#define RHO_DEBUG_DETECT        DEBUG_1
#define RHO_DEBUG_DETECT_2      DEBUG_1
#define RHO_DEBUG_PREDICT       DEBUG_2
#define RHO_DEBUG_PREDICT_2     DEBUG_1
#define RHO_DEBUG_UPDATE        DEBUG_1
#define RHO_DEBUG_UPDATE_2      DEBUG_1

#define KALMAN_DEBUG            RHO_DEBUG
#define KALMAN_DEBUG_2          RHO_DEBUG_2

//#define PSM_DEBUG               DEBUG_2
//#define PSM_DEBUG_2             DEBUG_1
//#define PSM_DEBUG_UPDATE        PSM_DEBUG
#define HMM_DEBUG               RHO_DEBUG
#define HMM_REPORT              RHO_DEBUG_2
//#define GMM_DEBUG               PSM_DEBUG
//#define GMM_DEBUG_2             PSM_DEBUG_2
//#define GMM_DEBUG_CLUSTERS      PSM_DEBUG_2
//#define FSM_DEBUG               RHO_DEBUG
//#define FSM_DEBUG_2             RHO_DEBUG
//#define FSM_DEBUG_UPDATE        RHO_DEBUG
//#define FSM_DEBUG_PRINT         RHO_DEBUG

//#define PACKET_DEBUG            RHO_DEBUG
//#define PACKET_DEBUG_2          RHO_DEBUG_2

#ifdef RHO_DEBUG
#define LOG_RHO(L,...)          LOG(L,"<Rho> " __VA_ARGS__)
#define LOG_RHO_BARE(L,...)     LOG_BARE(L,"" __VA_ARGS__)
#else
#define LOG_RHO(...)
#define LOG_RHO_BARE(L,...)
#endif

#ifdef PACKET_DEBUG
#define LOG_PACKET(L,...)       LOG(L,"<Packet> " __VA_ARGS__)
#else
#define LOG_PACKET(...)
#endif

#ifdef PSM_DEBUG
#define LOG_PSM(L,...)          LOG(L,"<PSM> " __VA_ARGS__)
#define LOG_PSM_BARE(L,...)     LOG_BARE(L,"" __VA_ARGS__)
#else
#define LOG_PSM(...)
#define LOG_PSM_BARE(...)
#endif

#ifdef HMM_DEBUG
#define LOG_HMM(L,...)          LOG(L,"<HMM> " __VA_ARGS__)
#define LOG_HMM_BARE(L,...)     LOG_BARE(L,"" __VA_ARGS__)
#else
#define LOG_HMM(...)
#define LOG_HMM_BARE(...)
#endif

#ifdef GMM_DEBUG
#define LOG_GMM(L,...)          LOG(L,"<GMM> " __VA_ARGS__)
#define LOG_GMM_BARE(L,...)     LOG_BARE(L,"" __VA_ARGS__)
#else
#define LOG_GMM(...)
#define LOG_GMM_BARE(L,...)
#endif

#ifdef FSM_DEBUG
#define LOG_FSM(L,...)          LOG(L,"<FSM> " __VA_ARGS__)
#define LOG_FSM_BARE(L,...)     LOG_BARE(L,"" __VA_ARGS__)
#else
#define LOG_FSM(...)
#define LOG_FSM_BARE(L,...)
#endif

#ifndef MAX
#define MAX(A,B)                ( ( A > B ) ? A : B )
#endif

#define AVG2(A,B)               ( ( A + B ) / 2. )
#define SWAP(A,B)               { typeof(A) temp = A; A = B; B = temp; }

#define BOUNDU(X,MAX)           ( ( X > MAX ) ? MAX : X )         // Bound in upper range
#define BOUND(X,MIN,MAX)        ( ( X < MIN ) ? MIN : BOUNDU( X, MAX ) ) // Bound in upper and lower range

#define FBOUND(X,MIN,MAX)       ( ( X < MIN ) ? MIN : ( ( X > MAX ) ? MAX : X ) )

#define SQUARE(X)               ( X * X )
#define DISTANCE_SQ(X,Y)        ( SQUARE(X) + SQUARE(Y) )
#define INRANGE(X,Y,T)          ( abs( X - Y ) < T )

#ifndef ZDIV
#define ZDIV_LNUM               ( 1 << 10 )
#define ZDIV(X,Y)               ( ( Y == 0 ) ? ( X == 0 ? 0 : ZDIV_LNUM ) : X / Y )
#endif

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#define NAME_BUILDER_FUNCTION_STRINGIFY(x,y) x ## _ ## y
#define NAME_BUILDER(x,y) NAME_BUILDER_FUNCTION_STRINGIFY(x,y)

#endif /* rho_global_h */
