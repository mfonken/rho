/************************************************************************
 *  File: rho_client.h
 *  Group: Rho Core
 ***********************************************************************/

#ifndef APPLICATION_USER_RHO_CORE_RHO_CAPTURE_H_
#define APPLICATION_USER_RHO_CORE_RHO_CAPTURE_H_

/************************************************************************
 *                             Includes                                 *
 ***********************************************************************/
#include "rho_core.h"
#include "stopwatch.h"

#define __ASSEMBLY_RHO__

index_t * RhoCapture_CaptureRow( byte_t,
				  const byte_t *,
				  const byte_t,
				  index_t * );
section_process_t RhoCapture_ProcessFrameSection( const index_t,
				  index_t *,
				  const index_t *,
				  const density_t,
				  sdensity_t *,
				  sdensity_t * );


#endif /* APPLICATION_USER_RHO_CORE_RHO_CAPTURE_H_ */
