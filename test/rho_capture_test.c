/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  File: rho_client_test.c
 *  Group: Rho Core Test
 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "../core/rho_capture.h"

#define CAPTURE_BUFFER_LEN (1 << 8)
#define THRESHOLD_BUFFER_LEN (1 << 8)

#if CAPTURE_BUFFER_LEN < THRESHOLD_BUFFER_LEN
#error "Capture buffer length must be leq the thresh buffer"
#endif
static uint8_t * capture_register;
static uint16_t threshold_buffer[THRESHOLD_BUFFER_LEN];
static uint8_t THRESHOLD_VALUE = 1 << 7;
static uint8_t sub_sample = 0;
static uint8_t capture_buffer[CAPTURE_BUFFER_LEN];
static uint32_t Dx[(1 << 8)];
static uint32_t Dy[(1 << 8)];
static bool flag;

static int rho_capture_test()
{
    const uint8_t * capture_end = capture_buffer + CAPTURE_BUFFER_LEN;
    
//    printf("Capture buffer:\n");
    for( int i = 0; i < CAPTURE_BUFFER_LEN; i++ )
    {
        capture_buffer[i] = rand() % 256;// i < CAPTURE_BUFFER_LEN >> 1 ? 0 : 255;
//        printf("- (%3d): %d\n", i,  capture_buffer[i]);
    }
    capture_register = capture_buffer;
    
    CaptureRow(capture_register,
               threshold_buffer,
               THRESHOLD_VALUE,
               sub_sample,
               capture_end,
               capture_buffer,
               (const uint8_t *)&flag);
    
    printf("Thresh buffer:\n");
    int i = 0;
    for( ; i < THRESHOLD_BUFFER_LEN && threshold_buffer[i]; i++ )
        printf("- (%3d): %d > %d\n", i,  threshold_buffer[i], capture_buffer[threshold_buffer[i]-1]);
    printf("%d vals above thresh\n", i);

    uint16_t * threshold_address = threshold_buffer;
    const uint16_t * threshold_end = threshold_buffer + THRESHOLD_BUFFER_LEN;
    uint32_t * Dx_i = Dx;
    section_process_t d = ProcessFrameSection(1,
                                              threshold_address,
                                              threshold_end,
                                              128,
                                              Dy,
                                              Dx_i);
    
    return 0;
}

