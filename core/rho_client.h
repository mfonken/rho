/************************************************************************
 *  File: rho_client.h
 *  Group: Rho Core
 ***********************************************************************/

#ifndef rho_client_h
#define rho_client_h

#ifndef STAND_ALONE

/************************************************************************
 *                             Includes                                 *
 ***********************************************************************/
#include "rho_capture.h"

/************************************************************************
 *                      Global Function Prototypes                      *
 ***********************************************************************/
void RhoSystem_PerformProcess( void );
void RhoSystem_ProcessFrameCapture( void );
void CaptureAndProcessFrame( void );
void CaptureRows( void );
void CaptureRowCallback( void );
void ActivateBackgrounding( void );
void DeactivateBackgrounding( void );
void FilterPixelCount( uint16_t *, uint16_t );
bool HasPixelCountDrop( void );
void RhoSystem_Activate( void );
void DeactivateRhoSystem( void );
void RhoSystem_Initialize( uint32_t, uint32_t );
void ZeroRhoSystemMemory( void );
void RhoSystem_ConnectToInterface( platform_interface_functions *, camera_application_flags *, dma_info_t * );
void RhoSystem_TransmitPacket( void );

/************************************************************************
 *                      Global Buffers                                  *
 ***********************************************************************/
extern capture_t _capture_buffer_internal[];
extern index_t _thresh_buffer_internal[];

/************************************************************************
 *                      Rho Core Variables                              *
 ***********************************************************************/
typedef struct
{
	uint32_t 	CameraPort;                     /* Parallel port register to camera */
	uint32_t 	HostTxPort;                     /* Output channel to host */
	address_t 	CaptureEnd;                     /* Effective end address for capture buffer */
	address_t 	CaptureMax;                     /* Actual end address for capture buffer */
	address_t 	ThreshEnd;                      /* Actual end of thresh buffer */
	address_t 	ThreshMax;                      /* Shared address of threshold value */
	address_t 	PixelCount;                     /* Shared address of pixel count value */
	address_t 	ProcessIndex;              		/* Address threhold buffer is processed */
	byte_t 		*Capture;                   	/* Address capture buffer is processed */
	index_t 	*Thresh;                    	/* Address threshold buffer is filled */
	dma_info_t 	*CameraDMA;					  	/* Address to camera DMA info */
} rho_system_address_variables;

typedef struct
{
	capture_t 	 *Capture;                      /* Raw capture buffer for DMA */
	index_t 	 *Thresh;                       /* Threshold processing buffer */
	sdensity_t   *DensityY;                     /* Processed density X array */
	sdensity_t   *DensityX;                     /* Processed density Y array */
	density_2d_t *Quadrant;                     /* Quadrant density array */
	packet_t 	 *BeaconPacket;                 /* Data packet for beacon comm */
} rho_system_buffer_variables;

typedef struct
{
    rho_core_t                      Utility;
    rho_system_address_variables    Addresses;
    rho_system_buffer_variables     Buffers;
    camera_application_flags       *Flags;
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
    void (*ConnectToInterface)( platform_interface_functions *, camera_application_flags *, dma_info_t * );
    void (*TransmitPacket)( void );
    void (*Activate)( void );
    void (*Deactivate)( void );

    index_t * (*CaptureRow)( byte_t, const byte_t *, const byte_t, index_t *);
    section_process_t (*ProcessSection)( const index_t, index_t *, const index_t *, const density_t, sdensity_t *, sdensity_t *);
} rho_perform_functions;

typedef struct
{
    void (*Zero)( void );
} rho_system_memory_functions;

typedef struct
{
    rho_perform_functions           Perform;
    platform_interface_functions    Platform;
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

static inline void EnableCaptureCallback(  void ) { RhoSystem.Variables.Flags->IRQ  = 1; } //RhoSystem.Variables.Flags->Capture.Flag
static inline void DisableCaptureCallback( void ) { RhoSystem.Variables.Flags->IRQ  = 0; }

#endif

#endif /* rho_client_h */
