/************************************************************************
 *  File: rho_client.c
 *  Group: Rho Core
 ***********************************************************************/

/************************************************************************
 *                             Includes                                 *
 ***********************************************************************/
#include "rho_client.h"

/************************************************************************
 *                        Local Configuration                           *
 ***********************************************************************/
 #define __ASSEMBLY_RHO__
// #define __CHECK_FRAME_FLAG__

volatile uint32_t capture_buffer;

/************************************************************************
 *                          Local Instance                              *
 ***********************************************************************/
rho_system_t RhoSystem =
{
    { /* VARIABLES */
        { /* Utility */
            { /* Density map pair */
                { /* Dy */
                    FOREGROUND_DENSITY_MAP_Y,
                    BACKGROUND_DENSITY_MAP_Y
                },
                { /* Dx */
                    FOREGROUND_DENSITY_MAP_X,
                    BACKGROUND_DENSITY_MAP_X
                }
            },
            CAPTURE_WIDTH,
            CAPTURE_HEIGHT,
            CAPTURE_SUB_SAMPLE
        },
        { 0 },/* Addresses */
        { 0 },/* Flags */
        { /* Buffers */
            _capture_buffer_internal,
            _thresh_buffer_internal
        }
    },
    { /* FUNCTIONS */
        { /* Perform */
            .Initialize         = InitializeRhoSystem,
            .CaptureRowCallback = CaptureRowCallback,
            .FrameCapture       = ProcessRhoSystemFrameCapture,
            .CoreProcess        = PerformRhoSystemProcess,
            .ConnectToInterface = ConnectRhoSystemPlatformInterface,
            .TransmitPacket     = TransmitRhoSystemPacket,
            .Activate           = ActivateRhoSystem,
            .Deactivate         = DeactivateRhoSystem
        },
        { 0 }, /* Platform */
        { /* Memory */
            .Zero               = ZeroRhoSystemMemory
        }
    }
};

/************************************************************************
 *                      Functions Declarations                          *
 ***********************************************************************/
/* Main application process */
void PerformRhoSystemProcess( void )
{
    if( RhoSystem.Variables.Flags->Active == false ) return;
    RhoSystem.Functions.Perform.FrameCapture();
//    RhoCore.Perform( &RhoSystem.Variables.Utility, RhoSystem.Variables.Flags->Backgrounding );
//    RhoSystem.Functions.Perform.TransmitPacket();
}

void ProcessRhoSystemFrameCapture( void )
{
    RhoSystem.Functions.Memory.Zero();
    RhoSystem.Functions.Platform.Interrupt.Enable();
    CaptureAndProcessFrame();
    RhoSystem.Functions.Platform.Interrupt.Disable();
}

void CaptureAndProcessFrame( void )
{
#ifdef __ENABLE_BACKGROUNDING__
    /* Check for pixel drop event */
    if( HasPixelCountDrop() ) ActivateBackgrounding();
    else DeactivateBackgrounding();
#endif

    /* Reset buffer indeces */
    RhoSystem.Variables.Addresses.ThreshIndex = (address_t)RhoSystem.Variables.Buffers.Thresh;
    RhoSystem.Variables.Utility.RowsLeft = (index_t)RhoSystem.Variables.Utility.Height;
    RhoSystem.Variables.Addresses.CaptureIndex = RhoSystem.Variables.Addresses.ThreshIndex;
    RhoSystem.Variables.Flags->EvenRowToggle = false;

    capture_buffer = (uint32_t)RhoSystem.Variables.Buffers.Capture;
    /* Manually start First Row Capture */
    while(!RhoSystem.Variables.Flags->Row);
    CaptureRowCallback();

    section_process_t ProcessedTopSectionData, ProcessedBtmSectionData;
    do{ ProcessedTopSectionData = ProcessFrameSection( RhoSystem.Variables.Utility.Cy );
    } while( !ProcessedTopSectionData.complete );
    do{ ProcessedBtmSectionData = ProcessFrameSection( RhoSystem.Variables.Utility.Height );
    } while( !ProcessedBtmSectionData.complete );

    DisableCaptureCallback();

    RhoSystem.Variables.Buffers.Quadrant[FRAME_QUADRANT_TOP_LEFT_INDEX]  = ProcessedTopSectionData.left;
    RhoSystem.Variables.Buffers.Quadrant[FRAME_QUADRANT_TOP_RIGHT_INDEX] = ProcessedTopSectionData.right;
    RhoSystem.Variables.Buffers.Quadrant[FRAME_QUADRANT_BTM_LEFT_INDEX]  = ProcessedBtmSectionData.left;
    RhoSystem.Variables.Buffers.Quadrant[FRAME_QUADRANT_BTM_RIGHT_INDEX] = ProcessedBtmSectionData.right;
}

void CaptureRowCallback( void )
{
    DisableCaptureCallback();
    RhoSystem.Variables.Addresses.CaptureIndex = (address_t)((uint32_t)RhoSystem.Variables.Buffers.Capture + (uint32_t)RhoSystem.Variables.Flags->EvenRowToggle);
    CaptureRow(
        (uint8_t *)RhoSystem.Variables.Addresses.CaptureIndex,
        (index_t *)RhoSystem.Variables.Addresses.ThreshIndex,
        (byte_t)RhoSystem.Variables.Utility.Thresh,
        (byte_t)RhoSystem.Variables.Utility.Subsample,
        (address_t)RhoSystem.Variables.Addresses.CaptureEnd,
        (address_t)RhoSystem.Variables.Buffers.Capture,
        (address_t)RhoSystem.Variables.Flags->Row );
        *(index_t *)(RhoSystem.Variables.Addresses.ThreshIndex++) = Y_DEL;
    RhoSystem.Variables.Flags->EvenRowToggle = !RhoSystem.Variables.Flags->EvenRowToggle;

    /* Only re-enable capture if buffers are not full and there is more to process */
    if( ( (uint32_t)RhoSystem.Variables.Addresses.ThreshIndex < (uint32_t)RhoSystem.Variables.Addresses.ThreshMax )
        && ( --RhoSystem.Variables.Utility.RowsLeft > 0 ))
        EnableCaptureCallback();
}

//__attribute__((naked))
void CaptureRow( register byte_t * capture_address,   // capture buffer index
                 register index_t * thresh_address,    // address of thresh buffer
                 const register byte_t thresh_value,
                 const register byte_t sub_sample,
                 const register address_t capture_end,
                 const register address_t capture_buffer,
                 const register address_t flag_address )
{
    register uint32_t working_register = 0;
#ifndef __ASSEMBLY_RHO__
    while( 1 )
    {
#ifdef __CHECK_FRAME_FLAG__
        working_register = *flag_address;
        if( working_register ) return;
#endif
        capture_address += sub_sample;
        if((uint32_t)capture_address >= (uint32_t)capture_end) return;

        working_register = *(capture_address);
        if(working_register >= thresh_value )
        {
            working_register = (uint32_t)capture_address - (uint32_t)capture_buffer;
            *(thresh_address++) = working_register;
        }
    }
#else
    __asm volatile
    (
    "capture:                                                              \n\t"
#ifdef __CHECK_FRAME_FLAG__
        "ldrb    %0, [%7]       ; Load row end flag byte                   \n\t"
        "cmp     %0, #1         ; Check if end is reached (set)            \n\t"
        "bge     end            ; If so, end                               \n\t"
#endif
        "add     %2, %2, %1     ; Increment capture index by sub_sample    \n\t"
        "cmp     %2, %3         ; Check if capture reached max width       \n\t"
        "bge     end            ; If so,end                                \n\t"

        "ldrb    %0, [%2]       ; Load byte at capture index into RO       \n\t"
        "cmp     %0, %4         ; Compare with threshold value             \n\t"
        "blt     capture        ; If less than, continue to next capture   \n\t"

        "sub     %0, %2, %5     ; Subtract capture buffer start from index \n\t"
        "strh    %0, [%6], #2   ; Store offset word at thresh index        \n\t"
        "b       capture        ; Branch back to next capture              \n"
    "end:                                                                  \n"
        ::
        "r"(working_register),  // %0
        "r"(sub_sample),        // %1
        "r"(capture_address),   // %2
        "r"(capture_end),       // %3
        "r"(thresh_value),      // %4
        "r"(capture_buffer),    // %5
        "r"(thresh_address),    // %6
        "r"(flag_address)       // %7
    );
#endif
}

section_process_t ProcessFrameSection( const index_t rows )
{
    register uint32_t
        value_register      = 0,
        calc_register       = 0,
        t_addr              = (uint32_t)RhoSystem.Variables.Addresses.CaptureIndex,
        t_last              = (uint32_t)RhoSystem.Variables.Addresses.ThreshIndex,
        Cx                  = (uint32_t)RhoSystem.Variables.Utility.Cx,
        Dy                  = (uint32_t)RhoSystem.Variables.Utility.DensityMapPair.y.map,
        Dx_i                = (uint32_t)RhoSystem.Variables.Utility.DensityMapPair.x.map,
        Dx_end              = Dx_i + (uint32_t)rows,
        Q_total             = 0,
        Q_prev              = 0,
        Q_left              = 0,
        Q_right             = 0;

    bool complete = false;

#ifndef __ASSEMBLY_RHO__
    while( t_addr <= t_end )
    {
        value_register = *(address_t)t_addr++;
        if(value_register < CAPTURE_BUFFER_WIDTH)
        {
            if( value_register < Cx )
                Q_left++;
            else
                Q_right++;
            ((address_t)Dy)[value_register]++;
        }
        else
        {
            Q_total = Q_left + Q_right;
            *(((address_t)Dx_i)++) = Q_total - Q_prev;
            Q_prev = Q_total;
            if( Dx_i >= Dx_end )
            {
                complete = true;
                break;
            }
        }
    }
#else
    __asm volatile
    (
    "loop_process:                                                      \n\t"
        "ldrh    %0, [%2], #1   ; Load next threshold buffer            \n\t"
        "cmp     %0, #"STR(CWL)"; Is value outside or equal frame width \n\t"
        "bge     row_end        ; Go to end row                         \n"
    "left_value:                                                        \n\t"
        "cmp     %0, %4         ; If value is right (greater) x centroid\n\t"
        "bgt     right_value    ; Branch to right quadrant updated      \n\t"
        "add     %10, %10, #1   ; Increment left quadrant               \n\t"
        "b       row_update     ; Branch to row map update              \n"
    "right_value:                                                       \n\t"
        "add     %11, %11, #1   ; Increment right quadrant              \n"
    "row_update:                                                        \n\t"
        "ldrb    %1, [%5, %0]   ; Load word at row map                  \n\t"
        "add     %1, %1, #1     ; Increment row map value               \n\t"
        "strb    %1, [%5, %0]   ; Store incremented value               \n\t"
        "b       loop_process   ; Loop back to start next values        \n"
    "row_end:                                                           \n\t"
        "add     %8, %10, %11   ; Add left and right quadrants to total \n\t"
        "uqsub16 %1, %8, %9     ; Calculate active pixels in row        \n\t"
        "strb    %1, [%6], #1   ; Store at next column address          \n\t"
        "cmp     %6, %7         ; Check if all rows are processed       \n\t"
        "bge     end            ; If so, end                            \n\t"
        "mov     %9, %10        ; Move current total px to previous     \n\t"
        "cmp     %2, %3         ; Check for end of threshold buffer     \n\t"
        "bge     end            ; If so, end                            \n\t"
        "blt     loop_process   ; Loop back to start next values        \n"
    "end:                       ; End if all rows are processed         \n"
        ::
        "r"(value_register),    // %0
        "r"(calc_register),     // %1
        "r"(t_addr),            // %2
        "r"(t_last)             // %3
        "r"(Cx),                // %4
        "r"(Dy),                // %5
        "r"(Dx_i),              // %6
        "r"(Dx_end)             // %7
        "r"(Q_total),           // %8
        "r"(Q_prev),            // %9
        "r"(Q_left),            // %10
        "r"(Q_right)            // %11
    );
#endif
    return (section_process_t){ Q_left, Q_right, complete };
}

void ActivateBackgrounding( void )
{
    RhoSystem.Variables.Flags->Backgrounding = true;
    RhoSystem.Variables.Buffers.DensityX = RhoSystem.Variables.Utility.DensityMapPair.x.background;
    RhoSystem.Variables.Buffers.DensityY = RhoSystem.Variables.Utility.DensityMapPair.y.background;
    RhoSystem.Variables.Buffers.Quadrant = RhoSystem.Variables.Utility.Qb;
}

void DeactivateBackgrounding( void )
{
    RhoSystem.Variables.Flags->Backgrounding = false;
    RhoSystem.Variables.Buffers.DensityX = RhoSystem.Variables.Utility.DensityMapPair.x.map;
    RhoSystem.Variables.Buffers.DensityY = RhoSystem.Variables.Utility.DensityMapPair.y.map;
    RhoSystem.Variables.Buffers.Quadrant = RhoSystem.Variables.Utility.Q;
}

inline void FilterPixelCount( index_t * PixelCount, index_t NewCount )
{
    *PixelCount = (index_t)( ( (floating_t)(*PixelCount) * ( 1. - PIXEL_COUNT_TRUST_FACTOR ) ) + ( (floating_t)NewCount * PIXEL_COUNT_TRUST_FACTOR ) );
}

inline bool HasPixelCountDrop( void )
{
    index_t * PixelCount = (index_t *)RhoSystem.Variables.Addresses.PixelCount,
        NewCount = (index_t)((uint32_t)RhoSystem.Variables.Addresses.ThreshIndex - (uint32_t)RhoSystem.Variables.Buffers.Thresh);
    floating_t FactoredOldCount = (floating_t)((*PixelCount) * PIXEL_COUNT_DROP_FACTOR );
    FilterPixelCount( PixelCount, NewCount );
    return ( *PixelCount < FactoredOldCount );
}

void ActivateRhoSystem( void  )
{
    RhoSystem.Variables.Flags->Active = true;
    RhoSystem.Variables.Flags->IRQ = true;
    RhoSystem.Functions.Perform.TransmitPacket();
}

void DeactivateRhoSystem( void )
{
    // TODO: zero period
    RhoSystem.Variables.Flags->Active = false;
    RhoSystem.Variables.Flags->IRQ = false;
    RhoSystem.Functions.Perform.TransmitPacket();
}

inline void TransmitRhoSystemPacket( void )
{
    //RhoSystem.Functions.Platform.Host.Transmit( (byte_t *)&RhoSystem.Variables.Utility.Packet, sizeof(packet_t) );
}
/***************************************************************************************/
/*                                  Initializers                                       */
/***************************************************************************************/
void InitializeRhoSystem( uint32_t CameraPort, uint32_t HostTxPort )
{
    /* Connect camera/hardware connection */
    RhoSystem.Variables.Addresses.CameraPort  = CameraPort;
    RhoSystem.Variables.Addresses.HostTxPort  = HostTxPort;
    RhoSystem.Functions.Platform.DMA.Init( RhoSystem.Variables.Addresses.CameraPort, (uint32_t)RhoSystem.Variables.Buffers.Capture, CAPTURE_BUFFER_SIZE, true );
    while(1);
    /* Connect capture and processing buffers */
    RhoSystem.Variables.Addresses.CaptureEnd  = (address_t)RhoSystem.Variables.Buffers.Capture;
    RhoSystem.Variables.Addresses.CaptureMax  = (address_t)RhoSystem.Variables.Buffers.Capture[THRESH_BUFFER_SIZE];
    RhoSystem.Variables.Addresses.ThreshMax   = (address_t)RhoSystem.Variables.Buffers.Thresh[THRESH_BUFFER_MAX];
    RhoSystem.Variables.Addresses.ThreshEnd   = (address_t)RhoSystem.Variables.Buffers.Thresh;

    /* Initialize beacon */
    RhoSystem.Variables.Buffers.BeaconPacket  = malloc( sizeof( packet_t ) );
    RhoSystem.Variables.Buffers.BeaconPacket->header.ID = BEACON_PACKET_ID;
    RhoSystem.Variables.Buffers.BeaconPacket->header.includes = BEACON_DEFAULT_PERIOD;

    /* Initialize Rho Core */
    RhoSystem.Functions.Memory.Zero();
    RhoCore.Initialize( &RhoSystem.Variables.Utility, CAPTURE_WIDTH, CAPTURE_HEIGHT );

    /* Connect caputre callback */
    RhoSystem.Variables.Flags->Capture.Callback = RhoSystem.Functions.Perform.CaptureRowCallback;

    /* Start with backgrounding disabled */
    DeactivateBackgrounding();
}

void ConnectRhoSystemPlatformInterface( platform_interface_functions * platform_interface, camera_application_flags * flags )
{
  memcpy( (void *)&RhoSystem.Functions.Platform, platform_interface, sizeof(platform_interface_functions) );
  RhoSystem.Variables.Flags = flags;
}

void ZeroRhoSystemMemory( void )
{
    memset( RhoSystem.Variables.Buffers.Thresh,   0, sizeof(index_t)   * THRESH_BUFFER_SIZE );
    memset( RhoSystem.Variables.Buffers.Quadrant, 0, sizeof(density_t) * 4                  );
}
