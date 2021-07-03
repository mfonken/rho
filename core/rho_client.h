/************************************************************************
 *  File: rho_client.h
 *  Group: Rho Core
 ***********************************************************************/

#ifndef rho_client_h
#define rho_client_h

/************************************************************************
 *                             Includes                                 *
 ***********************************************************************/
#include <stdio.h>
#include "rho_core.h"

/************************************************************************
 *                      Global Function Prototypes                      *
 ***********************************************************************/
void PerformRhoSystemProcess( void );
void ProcessRhoSystemFrameCapture( void );
void CaptureAndProcessFrame( void );
void CaptureRowCallback( void );
void CaptureRow( register byte_t *,
                      register index_t *,
                      const register byte_t,
                      const register byte_t,
                      const register address_t,
                      const register address_t,
                      const register address_t );
section_process_t ProcessFrameSection( const index_t );
void ActivateBackgrounding( void );
void DeactivateBackgrounding( void );
void FilterPixelCount( index_t *, index_t );
bool HasPixelCountDrop( void );
void ActivateRhoSystem( void );
void DeactivateRhoSystem( void );
void InitializeRhoSystem( uint32_t, uint32_t );
void ZeroRhoSystemMemory( void );
//void ConnectRhoSystemPlatformInterface( platform_interface_functions *, camera_application_flags * );
void TransmitRhoSystemPacket( void );

/************************************************************************
 *                      Global Buffers                                  *
 ***********************************************************************/
static capture_t _capture_buffer_internal[CAPTURE_BUFFER_SIZE];
static index_t _thresh_buffer_internal[THRESH_BUFFER_SIZE];

/************************************************************************
 *                      Rho Core Variables                              *
 ***********************************************************************/
typedef struct
{
  uint32_t
  CameraPort,                     /* Parallel port register to camera */
  HostTxPort;                     /* Output channel to host */

address_t
  CaptureEnd,                     /* Effective end address for capture buffer */
  CaptureMax,                     /* Actual end address for capture buffer */
  ThreshEnd,                      /* Actual end of thresh buffer */
  ThreshMax,                      /* Shared address of threshold value */
  PixelCount,                     /* Shared address of pixel count value */
  CaptureIndex,                   /* Address capture buffer is processed */
  ThreshIndex,                    /* Address threshold buffer is filled */
  ProcessIndex;                   /* Address threhold buffer is processed */
} rho_system_address_variables;

typedef struct
{
capture_t
    *Capture;                       /* Raw capture buffer for DMA */
index_t
    *Thresh;                        /* Threshold processing buffer */
dmap_t
    *DensityY,                      /* Processed density X array */
    *DensityX;                      /* Processed density Y array */
density_2d_t
    *Quadrant;                      /* Quadrant density array */
packet_t
    *BeaconPacket;                  /* Data packet for beacon comm */
} rho_system_buffer_variables;

typedef struct
{
    rho_core_t                      Utility;
//    rho_system_address_variables    Addresses;
//    camera_application_flags       *Flags;
//    rho_system_buffer_variables     Buffers;
} rho_system_variables;

/************************************************************************
 *                      Rho Core Functions                              *
 ***********************************************************************/
typedef struct
{
    void (*Initialize)( uint32_t, uint32_t );
    void (*CaptureRowCallback)( void );
    void (*FrameCapture)( void );
    void (*CoreProcess)( void );
//    void (*ConnectToInterface)( platform_interface_functions *, camera_application_flags * );
    void (*TransmitPacket)( void );
    void (*Activate)( void );
    void (*Deactivate)( void );
} rho_perform_functions;

typedef struct
{
    void (*Zero)( void );
} rho_system_memory_functions;

typedef struct
{
    rho_perform_functions           Perform;
//    platform_interface_functions    Platform;
    rho_system_memory_functions     Memory;
} rho_system_functions;

/************************************************************************
 *                      Rho Core System Definition                      *
 ***********************************************************************/
typedef struct
{
    rho_system_variables Variables;
    rho_system_functions Functions;
} rho_system_t;

extern rho_system_t RhoSystem;

//static inline void EnableCaptureCallback(  void ) { RhoSystem.Variables.Flags->Capture.Flag  = 1; }
//static inline void DisableCaptureCallback( void ) { RhoSystem.Variables.Flags->Capture.Flag  = 0; }

#endif /* rho_client_h */
