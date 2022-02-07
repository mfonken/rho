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
volatile uint32_t capture_buffer;

extern GPIO_t LED_GPIO;

/************************************************************************
 *                          Local Instance                              *
 ***********************************************************************/
capture_t _capture_buffer_internal[CAPTURE_BUFFER_SIZE];
index_t _thresh_buffer_internal[THRESH_BUFFER_SIZE];
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
        { 0 }, /* Addresses */
        { /* Buffers */
            _capture_buffer_internal,
            _thresh_buffer_internal
        },
        0 /* Flags */
    },
    { /* FUNCTIONS */
        { /* Perform */
            .Initialize         = RhoSystem_Initialize,
            .CaptureRowCallback = CaptureRowCallback,
            .FrameCapture       = RhoSystem_ProcessFrameCapture,
            .CoreProcess        = RhoSystem_PerformProcess,
            .ConnectToInterface = RhoSystem_ConnectToInterface,
            .TransmitPacket     = RhoSystem_TransmitPacket,
            .Activate           = RhoSystem_Activate,
            .Deactivate         = DeactivateRhoSystem,

			.CaptureRow			= RhoCapture_CaptureRow,
			.ProcessSection		= RhoCapture_ProcessFrameSection
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
static uint16_t nl = 0x0123;
void SendImage()
{
//	HAL_GPIO_WritePin( LED_GPIO_Port, LED_Pin, 0 );
	TransmitToHost((uint8_t *)&nl, 1);
//	HAL_GPIO_WritePin( LED_GPIO_Port, LED_Pin, 1 );
	TransmitToHost(RhoSystem.Variables.Buffers.Capture, CAPTURE_BUFFER_SIZE);
//	HAL_GPIO_WritePin( LED_GPIO_Port, LED_Pin, 0 );
}

static int frame_n = 3;
void SendDensityMaps()
{
//	HAL_GPIO_WritePin( LED_GPIO_Port, LED_Pin, 0 );
	TransmitToHost((uint8_t *)&nl, 2);
//	HAL_GPIO_WritePin( LED_GPIO_Port, LED_Pin, 1 );
	TransmitToHost((uint8_t *)RhoSystem.Variables.Utility.density_map_pair.x.map, RhoSystem.Variables.Utility.density_map_pair.x.length * sizeof(sdensity_t));
//	HAL_GPIO_WritePin( LED_GPIO_Port, LED_Pin, 0 );
	TransmitToHost((uint8_t *)RhoSystem.Variables.Utility.density_map_pair.y.map, RhoSystem.Variables.Utility.density_map_pair.y.length * sizeof(sdensity_t));
//	HAL_GPIO_WritePin( LED_GPIO_Port, LED_Pin, 1 );
//	TransmitToHost((uint8_t *)&nl, 1);
//	if(frame_n-- == 0)
//	{
//		TransmitToHost((uint8_t *)"done\r\n", 6);
//		while(1);
//	}
}

/* Main application process */
void RhoSystem_PerformProcess( void )
{
    if( RhoSystem.Variables.Flags->Active == false ) return;
//    while(!RhoSystem.Variables.Flags->Frame);
    RhoSystem.Functions.Perform.FrameCapture();
//    RhoCore.Perform( &RhoSystem.Variables.Utility, RhoSystem.Variables.Flags->Backgrounding );
//    RhoSystem.Functions.Perform.TransmitPacket();
}

void RhoSystem_ProcessFrameCapture( void )
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
    RhoSystem.Variables.Addresses.Thresh = RhoSystem.Variables.Buffers.Thresh;
    RhoSystem.Variables.Utility.rows_left = (uint16_t)RhoSystem.Variables.Utility.height;
    RhoSystem.Variables.Addresses.Capture = (byte_t *)RhoSystem.Variables.Addresses.Thresh;
    RhoSystem.Variables.Flags->EvenRowToggle = false;

    capture_buffer = (uint32_t)RhoSystem.Variables.Buffers.Capture;

    EnableCaptureCallback();
	CaptureRows();
//	SendImage();

    section_process_t ProcessedSectionData[2];
    uint16_t rows = RhoSystem.Variables.Utility.centroid.y;
    for( byte_t i = 0; i < 2; i++ )
    {
//		HAL_GPIO_WritePin( LED_GPIO_Port, LED_Pin, 0 );
		do{ ProcessedSectionData[i] = RhoCapture_ProcessFrameSection( rows,
				RhoSystem.Variables.Buffers.Thresh,
				RhoSystem.Variables.Addresses.Thresh,
				RhoSystem.Variables.Utility.centroid.x,
				RhoSystem.Variables.Utility.density_map_pair.y.map,
				RhoSystem.Variables.Utility.density_map_pair.x.map );
		} while( !ProcessedSectionData[i].complete );
//		HAL_GPIO_WritePin( LED_GPIO_Port, LED_Pin, 1 );
		rows = RhoSystem.Variables.Utility.height;
    }
    DisableCaptureCallback();
//	HAL_GPIO_WritePin( LED_GPIO_Port, LED_Pin, 0 );

    RhoSystem.Variables.Buffers.Quadrant[FRAME_QUADRANT_TOP_LEFT_INDEX]  = ProcessedSectionData[0].left;
    RhoSystem.Variables.Buffers.Quadrant[FRAME_QUADRANT_TOP_RIGHT_INDEX] = ProcessedSectionData[0].right;
    RhoSystem.Variables.Buffers.Quadrant[FRAME_QUADRANT_BTM_LEFT_INDEX]  = ProcessedSectionData[1].left;
    RhoSystem.Variables.Buffers.Quadrant[FRAME_QUADRANT_BTM_RIGHT_INDEX] = ProcessedSectionData[1].right;

	SendDensityMaps();
}

static bool flag = false;
void CaptureRows(void)
{
	while(RhoSystem.Variables.Utility.rows_left-- > 0)
	{
		HAL_GPIO_WritePin( LED_GPIO_Port, LED_Pin, 0 );
		while(!RhoSystem.Variables.Flags->Row);
		HAL_GPIO_WritePin( LED_GPIO_Port, LED_Pin, 1 );
		while(RhoSystem.Variables.Flags->Row);
		HAL_GPIO_WritePin( LED_GPIO_Port, LED_Pin, 0 );
//		stopwatch_reset();
//		STOPWATCH_START;
		if(RhoSystem.Variables.Utility.rows_left == FRAME_HEIGHT / 2)
			flag = true;
		CaptureRowCallback();
//	    STOPWATCH_STOP;
//	    int dur = STOPWATCH_DUR;
		HAL_GPIO_WritePin( LED_GPIO_Port, LED_Pin, 1 );
	}
}

void CaptureRowCallback( void )
{
    DisableCaptureCallback();
    RhoSystem.Functions.Platform.DMA.Pause( RhoSystem.Variables.Addresses.CameraDMA );
    RhoSystem.Variables.Addresses.Capture = RhoSystem.Variables.Buffers.Capture + RhoSystem.Variables.Flags->EvenRowToggle;
    RhoSystem.Variables.Addresses.Thresh = RhoCapture_CaptureRow(
			RhoSystem.Variables.Utility.subsample,
    		RhoSystem.Variables.Addresses.Capture,
			RhoSystem.Variables.Utility.thresh_byte,
			RhoSystem.Variables.Addresses.Thresh );
	*(RhoSystem.Variables.Addresses.Thresh++) = CAPTURE_ROW_END;
    RhoSystem.Variables.Flags->EvenRowToggle = !RhoSystem.Variables.Flags->EvenRowToggle;

    RhoSystem.Functions.Platform.DMA.Reset( RhoSystem.Variables.Addresses.CameraDMA );

//    /* Only re-enable capture if buffers are not full and there is more to process */
//    if( ( (uint32_t)( RhoSystem.Variables.Addresses.Thresh + CAPTURE_BUFFER_SIZE ) < (uint32_t)RhoSystem.Variables.Addresses.ThreshMax )
//        && ( --RhoSystem.Variables.Utility.rows_left > 0 ))
	EnableCaptureCallback();
}


void ActivateBackgrounding( void )
{
    RhoSystem.Variables.Flags->Backgrounding = true;
    RhoSystem.Variables.Buffers.DensityX = RhoSystem.Variables.Utility.density_map_pair.x.background;
    RhoSystem.Variables.Buffers.DensityY = RhoSystem.Variables.Utility.density_map_pair.y.background;
    RhoSystem.Variables.Buffers.Quadrant = RhoSystem.Variables.Utility.quadrant_background;
}

void DeactivateBackgrounding( void )
{
    RhoSystem.Variables.Flags->Backgrounding = false;
    RhoSystem.Variables.Buffers.DensityX = RhoSystem.Variables.Utility.density_map_pair.x.map;
    RhoSystem.Variables.Buffers.DensityY = RhoSystem.Variables.Utility.density_map_pair.y.map;
    RhoSystem.Variables.Buffers.Quadrant = RhoSystem.Variables.Utility.quadrant;
}

inline void FilterPixelCount( uint16_t * PixelCount, uint16_t NewCount )
{
    *PixelCount = (uint16_t)( ( (floating_t)(*PixelCount) * ( 1. - PIXEL_COUNT_TRUST_FACTOR ) ) + ( (floating_t)NewCount * PIXEL_COUNT_TRUST_FACTOR ) );
}

inline bool HasPixelCountDrop( void )
{
    uint16_t * PixelCount = (uint16_t *)RhoSystem.Variables.Addresses.PixelCount,
        NewCount = (uint16_t)((uint32_t)RhoSystem.Variables.Addresses.Thresh - (uint32_t)RhoSystem.Variables.Buffers.Thresh);
    floating_t FactoredOldCount = (floating_t)((*PixelCount) * PIXEL_COUNT_DROP_FACTOR );
    FilterPixelCount( PixelCount, NewCount );
    return ( *PixelCount < FactoredOldCount );
}

void RhoSystem_Activate( void  )
{
    RhoSystem.Variables.Flags->Active = true;
    RhoSystem.Variables.Flags->IRQ = true;
    RhoSystem.Functions.Platform.DMA.Resume( RhoSystem.Variables.Addresses.CameraDMA );
    RhoSystem.Functions.Perform.TransmitPacket();
}

void DeactivateRhoSystem( void )
{
    // TODO: zero period
    RhoSystem.Variables.Flags->Active = false;
    RhoSystem.Variables.Flags->IRQ = false;
    RhoSystem.Functions.Platform.DMA.Pause( RhoSystem.Variables.Addresses.CameraDMA );
    RhoSystem.Functions.Perform.TransmitPacket();
}

inline void RhoSystem_TransmitPacket( void )
{
    //RhoSystem.Functions.Platform.Host.Transmit( (byte_t *)&RhoSystem.Variables.Utility.Packet, sizeof(packet_t) );
}
/***************************************************************************************/
/*                                  Initializers                                       */
/***************************************************************************************/
void RhoSystem_Initialize( uint32_t CameraPort, uint32_t HostTxPort )
{
    /* Connect camera/hardware connection */
    RhoSystem.Variables.Addresses.CameraPort  = CameraPort;
    RhoSystem.Variables.Addresses.HostTxPort  = HostTxPort;
    RhoSystem.Variables.Addresses.CameraDMA->dst = (uint32_t)RhoSystem.Variables.Buffers.Capture;
    RhoSystem.Functions.Platform.DMA.Init( RhoSystem.Variables.Addresses.CameraDMA );

    /* Connect capture and processing buffers */
    RhoSystem.Variables.Addresses.CaptureEnd  = (address_t)RhoSystem.Variables.Buffers.Capture;
    RhoSystem.Variables.Addresses.CaptureMax  = (address_t)RhoSystem.Variables.Buffers.Capture[THRESH_BUFFER_SIZE];
    RhoSystem.Variables.Addresses.ThreshMax   = (address_t)RhoSystem.Variables.Buffers.Thresh[THRESH_BUFFER_MAX];
    RhoSystem.Variables.Addresses.ThreshEnd   = (address_t)RhoSystem.Variables.Buffers.Thresh;

    /* Initialize beacon */
    RhoSystem.Variables.Buffers.BeaconPacket  = malloc( sizeof( packet_t ) );
    RhoSystem.Variables.Buffers.BeaconPacket->header.id = BEACON_PACKET_ID;
    RhoSystem.Variables.Buffers.BeaconPacket->header.includes = BEACON_DEFAULT_PERIOD;

    /* Initialize Rho Core */
    RhoSystem.Functions.Memory.Zero();
    RhoCore.Initialize( &RhoSystem.Variables.Utility, CAPTURE_WIDTH, CAPTURE_HEIGHT );

    /* Connect capture callback */
    RhoSystem.Variables.Flags->Capture.Callback = RhoSystem.Functions.Perform.CaptureRowCallback;

    RhoSystem.Variables.Utility.subsample = DEFAULT_SUBSAMPLE;
    /* Start with backgrounding disabled */
    DeactivateBackgrounding();
}

void RhoSystem_ConnectToInterface( platform_interface_functions * platform_interface, camera_application_flags * flags, dma_info_t * camera_dma )
{
  memcpy( (void *)&RhoSystem.Functions.Platform, platform_interface, sizeof(platform_interface_functions) );
  RhoSystem.Variables.Flags = flags;
  RhoSystem.Variables.Addresses.CameraDMA = camera_dma;
}

void ZeroRhoSystemMemory( void )
{
    memset( RhoSystem.Variables.Buffers.Thresh,   0, sizeof(uint16_t)   * THRESH_BUFFER_SIZE );
    memset( RhoSystem.Variables.Buffers.Quadrant, 0, sizeof(density_t) * 4                   );
    memset( RhoSystem.Variables.Utility.density_map_pair.x.map, 0, RhoSystem.Variables.Utility.density_map_pair.x.length * sizeof(density_t) );
    memset( RhoSystem.Variables.Utility.density_map_pair.y.map, 0, RhoSystem.Variables.Utility.density_map_pair.y.length * sizeof(density_t) );
}
