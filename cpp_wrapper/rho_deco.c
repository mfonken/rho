/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  File: rho_deco.c
 *  Group: Tau+
 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

#include "rho_deco.h"

#define BURN_ROWS 0
#define BURN_COLS 0

rho_variables RhoVariables = { 0 };

void RhoInterruptModel_InitFromCore( rho_core_t * core )
{
    /* Connect to Interrupt Model variable structure */
    //printf(">>%d:%p\n", core->DensityMapPair.y.length, core->DensityMapPair.y.map);
    RhoVariables.ram.Dx      =  core->density_map_pair.x.map;
    RhoVariables.ram.Dy      =  core->density_map_pair.y.map;
    RhoVariables.ram.Q       =  core->quadrant;
    RhoVariables.ram.CX_ADDR = &core->centroid.x;
    RhoVariables.ram.CY_ADDR = &core->centroid.y;
    RhoVariables.ram.C_FRAME =  core->cframe;
    RhoVariables.ram.THRESH_ADDR = (density_t *)&core->thresh_byte;

    RhoVariables.global.c_frame_max = C_FRAME_SIZE;
    RhoVariables.global.y_delimiter = Y_DEL;
    RhoVariables.global.W    =  core->width;
    RhoVariables.global.H    =  core->height;

    RhoVariables.connected = true;
}

void RhoInterruptModel_FrameStart( void )
{
    RhoVariables.registers.x    = 0;
    RhoVariables.registers.p    = 0;
    RhoVariables.registers.QS   = 0;
    RhoVariables.registers.PTOG = 0;

    RhoVariables.ram.y          = 0;
    RhoVariables.ram.QN         = 0;
    RhoVariables.ram.QN_        = 0;
    RhoVariables.ram.QT         = 0;
    RhoVariables.ram.C_FRAME_END = RhoVariables.ram.C_FRAME + RhoVariables.global.c_frame_max;

//    memset(RhoVariables.ram.C_FRAME, 0, sizeof(char)*RhoVariables.global.C_FRAME_MAX);
    //printf(">>%d:%p | %d\n", RhoVariables.global.W, RhoVariables.ram.Dy, (int)sizeof(density_map_unit_t));
    //printf("%d | %d\n", (int)sizeof(density_map_unit_t) * RhoVariables.global.W, (int)sizeof(RhoVariables.ram.Dy));
    memset(RhoVariables.ram.Dy, 0, sizeof(density_map_unit_t) * RhoVariables.global.W);
    memset(RhoVariables.ram.Dx, 0, sizeof(density_map_unit_t) * RhoVariables.global.H);
    memset(RhoVariables.ram.Q,  0, sizeof(density_2d_t) * 4);

    RhoVariables.registers.Cx   = *RhoVariables.ram.CX_ADDR;
    RhoVariables.registers.Cy   = *RhoVariables.ram.CY_ADDR;
    RhoVariables.registers.wr   = RhoVariables.ram.C_FRAME;
    RhoVariables.registers.rd   = RhoVariables.ram.C_FRAME;
    RhoVariables.registers.thresh = (uint8_t)*RhoVariables.ram.THRESH_ADDR;
}

void RhoInterruptModel_FrameEnd( void )
{
}

#define RPC(X,T) if(X&T)
#define RPCB(X,Y,N,T) {RPC(X,T){Q##Y++;N[x]++;}}

void RhoInterruptModel_RhoFunction( const cimage_t image )
{
    if(!RhoVariables.connected) return;
    index_t w = image.width, h = image.height;

    index_t y = 0, x;
    uint32_t p = 0;

    RhoInterrupts.FrameStart();
    
    RhoVariables.ram.QT = 0;
    density_2d_t Q0 = 0, Q1 = 0, Q2 = 0, Q3 = 0, QT = 0, QP = 0;
    uint8_t THRESH = RhoVariables.registers.thresh;
    for(; y < RhoVariables.registers.Cy; y++ )
    {
        for( x = 0; x < RhoVariables.registers.Cx; x++, p++ )
        {
            if(image.pixels[p] > THRESH)
            {
                Q0++;
                RhoVariables.ram.Dy[x]++;
            }
        }
        for( ; x <  w; x++, p++ )
        {
            if(image.pixels[p] > THRESH)
            {
                Q1++;
                RhoVariables.ram.Dy[x]++;
            }
        }
        QT = Q0 + Q1;
        RhoVariables.ram.Dx[y] = QT - QP;
        QP = QT;
    }
    for( QP = 0; y < h; y++ )
    {
        for( x = 0; x < RhoVariables.registers.Cx; x++, p++ )
        {
            if(image.pixels[p] > THRESH)
            {
                Q2++;
                RhoVariables.ram.Dy[x]++;
            }
        }
        for( ; x < w; x++, p++ )
        {
            if(image.pixels[p] > THRESH)
            {
                Q3++;
                RhoVariables.ram.Dy[x]++;
            }
        }
        QT = Q2 + Q3;
        RhoVariables.ram.Dx[y] = QT - QP;
        QP = QT;
    }

    RhoVariables.ram.Q[0] = Q0;
    RhoVariables.ram.Q[1] = Q1;
    RhoVariables.ram.Q[2] = Q2;
    RhoVariables.ram.Q[3] = Q3;
    RhoVariables.ram.QT   = Q0 + Q1 + Q2 + Q3;

    LOG_RHO(DEBUG_0, "Quadrants are [%d][%d][%d][%d] (%d|%d)\n", Q0, Q1, Q2, Q3, RhoVariables.registers.Cx, RhoVariables.registers.Cy);
    LOG_RHO(DEBUG_0, "# Total coverage is %.3f%%\n", ((double)RhoVariables.ram.QT)/((double)w*h)*100);
    LOG_RHO(DEBUG_0, "\t\t\t\tQT %d\n", RhoVariables.ram.QT);
}
