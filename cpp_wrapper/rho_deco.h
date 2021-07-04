/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  File: rho_deco.h
 *  Group: Tau+
 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

#ifndef rho_deco_h
#define rho_deco_h

#include <stdio.h>
#include <string.h>

#include <pthread.h>

#include "rho_types.h"
#include "image_types.h"

#ifdef __cplusplus
extern "C" {
#endif

    void RhoInterruptModel_RhoFunction( cimage_t );

    void RhoInterruptModel_InitFromCore( rho_core_t * );
    void RhoInterruptModel_FrameInit(  void );
    void RhoInterruptModel_FrameStart( void );
    void RhoInterruptModel_FrameEnd(   void );

    typedef struct rho_interrupts rho_interrupts;
    struct rho_interrupts
    {
        void (*InitFromCore)( rho_core_t * );
        void (*FrameStart)( void );
        void (*FrameEnd)(   void );
        void (*RhoFunction)( const cimage_t );
    };
    static const rho_interrupts RhoInterrupts =
    {
        .InitFromCore = RhoInterruptModel_InitFromCore,
        .FrameStart = RhoInterruptModel_FrameStart,
        .FrameEnd   = RhoInterruptModel_FrameEnd,
        .RhoFunction = RhoInterruptModel_RhoFunction
    };

    typedef struct
    {
    uint32_t
        c_frame_max;
    index_t
        counter,
        y_delimiter,
        W,
        H;
    } rho_global_variables;

    typedef struct
    {
        index_t
        x,
        p,
        Cx,
        Cy;
        pixel_base_t
        *wr,
        *rd;
        byte_t
        thresh;
        byte_t
        QS,
        PTOG;
    } rho_register_variables;

    typedef struct
    {
        density_map_unit_t
            *Dx,
            *Dy;
        density_t
            *THRESH_ADDR;
        density_2d_t
            *Q,
            QT,
            QN,
            QN_;
        index_t
            y,
            *CX_ADDR,
            *CY_ADDR;
        pixel_base_t
            *C_FRAME,
            *C_FRAME_END;
    } rho_sram_variables;

    typedef struct
    {
        bool connected;
        rho_global_variables    global;
        rho_register_variables  registers;
        rho_sram_variables      ram;
    } rho_variables;

    extern rho_variables RhoVariables;

#ifdef __cplusplus
}
#endif


#endif /* rho_interrupt_h */
