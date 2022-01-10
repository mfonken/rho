/*
 * timestamp.h
 *
 *  Created on: Jul 3, 2021
 *      Author: Matthew
 */

#ifndef TIMESTAMP_H_
#define TIMESTAMP_H_

#if defined __linux || defined __APPLE__
#else
#include "../UniSM/system_master.h"
//#define TIMESTAMP() 0 /* SPOOF TIMESTAMP */
#endif

#endif /* TIMESTAMP_H_ */
