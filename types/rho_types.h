/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  File: rho_types.c
 *  Group: Rho Core
 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

#ifndef rho_c_types_h
#define rho_c_types_h

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*                                    Includes                                         */
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "rho_structure.h"
#include "../rho_config.h"
#ifdef __PSM__
#include "../psm/psm.h"
#else
#include "../psm/fsm.h"
#endif
#include "kalman2d.h"

//#ifndef MAX_OBSERVATIONS
//#define MAX_OBSERVATIONS        (1 << 3) // Length of history
//#endif

typedef struct
{
    uint16_t x,y;
} index_pair_t;

typedef struct
{
    byte_t x, y;
} byte_pair_t;

typedef struct
{
    index_t x, y;
    index_t w, h;
//    kalman2d_t k;
} blob_t;

typedef struct
{
density_t
    maximum,
    density;
uint16_t
    location,
    width,
    sort;
floating_t
    score;
byte_t
    tracking_id;
} region_t;

typedef struct
{
    region_t * x, * y;
} region_pair_t;

typedef struct
{
    region_t * region;
    kalman_t kalman;
    bool valid;
} tracker_t;

/* Rho Structures* */
typedef struct
{
    sdensity_t
        *map,
        *background,
        *bound,
        max[2];
    uint16_t
        length;
    floating_t
        centroid;
    bool
        has_background;
    kalman_t
        kalmans[2];
    const char *
        name;
} density_map_t;

typedef struct
{
    density_map_t x,y;
} density_map_pair_t;

typedef struct
{
    byte_t
        background[4][4],
        current[4][4],
        factor[4][4],
        length[4];
} density_redistribution_lookup_config_t;

typedef struct
{
    density_redistribution_lookup_config_t config[4];
} density_redistribution_lookup_t;

typedef struct
{
    floating_t  P[NUM_STATES],
                confidence;
} prediction_probabilities;

typedef struct
{
    uint8_t valid;
    uint8_t index;
} order_t;

typedef struct
{
    const char *    name;
    tracker_t       trackers[MAX_TRACKERS];
    uint8_t         trackers_order[MAX_TRACKERS];
    region_t        regions[MAX_REGIONS];
    order_t         regions_order[MAX_REGIONS];
    uint8_t         num_regions;
    density_t       previous_peak[2],
                    previous_centroid;
    density_2d_t    previous_density[2],
                    total_density;
    floating_t      nu_regions,
                    primary,
                    secondary,
                    average_density;
    observation_list_t observation_list;
    prediction_probabilities probabilities;
} prediction_t;

typedef struct
{
    prediction_t    x,y;
    prediction_probabilities probabilities;

    floating_t      nu_regions,
                    num_regions,
//                    BestConfidence,
                    average_density;
    bool            descending;
    index_pair_t         blobs_order[MAX_REGIONS];
    byte_t          num_blobs;
} prediction_pair_t;

typedef struct
{
    psm_t x,y;
    state_t current_state;
    uint8_t
        proposed_num;
    double
        best_confidence,
        proposed_threshold,
        proposed_avg_den;
} psm_pair_t;


//typedef struct
//{
//  void (*Init)(void);
//  void (*Pause)(void);
//  void (*Resume)(void);
//  void (*Reset)(void);
//} platform_dma_interface_functions;

//typedef struct
//{
//  uint8_t (*Transmit)( byte_t *, uint16_t);
//} rho_platform_uart_interace_functions;

#ifndef __RHO_STAND_ALONE__
#ifndef __USE_DECOUPLING__
typedef struct
{
  void (*Activate)( camera_application_flags * );
} rho_platform_flag_interace_functions;
#endif
#endif

typedef struct
{
  uint32_t (*Now)( void );
} rho_platform_time_interace_functions;

typedef struct
{
    uint16_t xl[3], yl[3];
    density_2d_t area[9];
    density_2d_t a, b, c, d, l, l_, p, q, x, y;
} redistribution_variables;

typedef struct
{
    floating_t
        background,
        state,
        target,
        proposed;
} rho_tune_t;

typedef struct
{
    uint16_t
      len,
      range[3],
      cycle,
      cycle_,
      gap_counter,
      width,
      total_regions,
#ifdef __USE_ZSCORE_THRESHOLD__
      z_index,
      z_thresh_factor,
      z_thresh,
#endif
#ifdef __USE_REGION_BOUNDARY_OFFSET__
      region_boundaries[MAX_REGIONS*2],
      region_boundary_index,
#endif
      x,
      start,
      end;
#ifdef __USE_ZSCORE_THRESHOLD__
    cumulative_avg_stdv_t
      z_stat;
#endif
    density_t
      filter_peak,
      filter_peak_2,
      filter_band_lower,
      background_curr,
      maximum;
    sdensity_t
      curr;
    variance_t
      filter_variance;
    density_2d_t
      current_density,
      total_density,
      filtered_density,
      first_filtered_density, /* Initial filtered density before retries */
      raw_density_moment;
    bool
#ifdef __USE_ZSCORE_THRESHOLD__
      has_stat_update,
#endif
      has_region,
      recalculate;
    byte_t
      recalculation_counter;
    floating_t
      average_counter,
      average_curr,   /* cumulative average */
      average_moment,   /* moment average */
      chaos,
      recalculation_chaos,
      target_density,
      assumed_regions;
} rho_detection_variables;

typedef struct
{
    index_pair_t pts[2],
        centroid;
    int8_t quadrant_check;
} prediction_predict_variables;

typedef struct
{
    packet_t *packet;
    address_t pdPtr, llPtr, *alPtr;
    byte_t includes, i, j, l, t;
} packet_generation_variables;

#define DETECTION_BUFFER_SIZE ( 1 << 9 )
#define DETECTION_BUFFER_MASK ( DETECTION_BUFFER_SIZE - 1 )
//#define MAX_DENSITY           ( 1 << 9 )

typedef struct
{
    uint8_t thresh;
    uint16_t density;
    uint8_t tracking_id;
} detection_element_t;

typedef struct
{
    uint16_t index, fill, length, first;
    detection_element_t buffer[DETECTION_BUFFER_SIZE];
} detection_ring_buffer_t;

typedef detection_ring_buffer_t detection_map_t;

typedef struct
{
    density_map_pair_t density_map_pair;
    index_t width;
	index_t height;
    byte_t subsample;
    byte_t thresh_byte;
//	index_t rows_left;
    index_pair_t primary;
    index_pair_t secondary;
	index_pair_t centroid;
	index_pair_t background_centroid;
    byte_t background_counter;
	density_2d_t quadrant[4];
    density_2d_t quadrant_background[4];
    density_2d_t quadrant_final[4];
    density_2d_t quadrant_background_total;
	density_2d_t total_coverage;
	density_2d_t filtered_coverage;
//	density_2d_t target_coverage;
	density_2d_t background_period;
	floating_t total_percentage;
	floating_t filtered_percentage;
	floating_t target_coverage_factor;
//	floating_t coverage_factor;
//	floating_t variance_factor;
//	floating_t previous_thresh_filter_value;
	floating_t thresh;
    rho_tune_t          tune;
    prediction_pair_t   prediction_pair;
    pid_filter_t        thresh_filter;
    kalman_t     target_filter;
//    detection_map_t     detection_map;

#ifdef __PSM__
    psm_pair_t          predictive_state_model_pair;
#endif
    transition_matrix_t state_transitions;
    kumaraswamy_t       kumaraswamy;
    fsm_system_t        state_machine;
    packet_t            packet;

#ifdef __USE_DECOUPLING__
    uint8_t             cframe[C_FRAME_SIZE];
#endif

    double              timestamp;
} rho_core_t;

/* Quadrant density redistribution lookup table */
static const density_redistribution_lookup_t rlookup =
{
    {
        {
            { { 0, 1, 3, 4 }, { 2, 5 }, { 6, 7 }, { 8 } },
            { { 0 }, { 1, 2 }, { 3, 6 }, { 4, 5, 7, 8 } },
            { { 0, 1, 2, 3 }, { 1, 3 }, { 2, 3 }, { 3 } },
            { 4, 2, 2, 1 }
        },
        {
            { { 0, 3 }, { 1, 2, 4, 5 }, { 6 }, { 7, 8 } },
            { { 0, 1 }, { 2 }, { 3, 4, 6, 7 }, { 5, 8 } },
            { { 0, 2 }, { 0, 1, 2, 3 }, { 2 }, { 2, 3 } },
            { 2, 4, 1, 2 }
        },
        {
            { { 0, 1 }, { 2 }, { 3, 4, 6, 7 }, { 5, 8 } },
            { { 0, 3 }, { 1, 2, 4, 5 }, { 6 }, { 7, 8 } },
            { { 0, 1 }, { 1 }, { 0, 1, 2, 3 }, { 1, 3 } },
            { 2, 1, 4, 2 }
        },
        {
            { { 0 }, { 1, 2 }, { 3, 6 }, { 4, 5, 7, 8 } },
            { { 0, 1, 3, 4 }, { 2, 5 }, { 6, 7 }, { 8 } },
            { { 0 }, { 0, 1 }, { 0, 2 }, { 0, 1, 2, 3 } },
            { 1, 2, 2, 4 }
        }
    }
};

#endif /* rho_c_types_h */
