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

void RhoCapture_CaptureRow( register byte_t *,
				  register index_t *,
				  const register byte_t,
				  const register byte_t,
				  const register byte_t *,
				  const register byte_t *,
				  const register byte_t * );
section_process_t RhoCapture_ProcessFrameSection( const index_t,
				  register index_t *,
				  const register index_t *,
				  const register density_2d_t,
				  register density_2d_t *,
				  register density_2d_t * );


#endif /* APPLICATION_USER_RHO_CORE_RHO_CAPTURE_H_ */
