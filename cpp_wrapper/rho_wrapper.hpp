//
//  rho_wrapper.hpp
//  tau+
//
//  Created by Matthew Fonken on 3/26/18.
//  Copyright © 2018 Marbl. All rights reserved.
//

#ifndef rho_wrapper_hpp
#define rho_wrapper_hpp

#include "tau_structures.hpp"
#include "rho_master.h"
#include "image_types.h"

class Rho
{
public:
    int                 width,
                        height;
    rho_core_t          core;
    pthread_mutex_t     c_mutex;
    pthread_mutex_t     density_map_pair_mutex;
    bool                backgrounding_event;
    
    Rho( int, int );
    double Perform( cimage_t&, GlobalPacket * );
    void Decouple( const cimage_t, bool );
    void PrintSizes( void );
};

#endif /* rho_wrapper_hpp */
