//
//  rho_packet.h
//  combine
//
//  Created by Matthew Fonken on 9/25/19.
//  Copyright © 2019 Matthew Fonken. All rights reserved.
//

#ifndef rho_packet_h
#define rho_packet_h

/* Packet Generation Settings */
#define YES 1
#define NO  0

#define PACKET_ADDRESS_INITIALIZER(r)               \
{                                                   \
(address_t)&r.y.trackers[0].value, /* px */ \
(address_t)&r.x.trackers[0].value, /* py */ \
(address_t)&r.y.trackers[1].value, /* sx */ \
(address_t)&r.x.trackers[1].value, /* sy */ \
(address_t)&r.probabilities.P[1],         /* pp */  \
(address_t)&r.probabilities.P[2],         /* ap */  \
(address_t)&r.probabilities.P[3]          /* ap */  \
}

#define PACKET_HEADER_SIZE (sizeof(packet_header_t));
#define PACKET_HEADER_ID    0xab

#define PACKET_DATA_DECIMAL       floating_t
#define PACKET_DATA_INTEGER       uint8_t
#define PACKET_DATA_DECIMAL_SIZE  ((uint8_t)sizeof(PACKET_DATA_DECIMAL))
#define PACKET_DATA_INTEGER_SIZE  ((uint8_t)sizeof(PACKET_DATA_INTEGER))

#define PACKET_PX_DATA_OFFSET 0
#define PACKET_PY_DATA_OFFSET 1
#define PACKET_SX_DATA_OFFSET 2
#define PACKET_SY_DATA_OFFSET 3
#define PACKET_PP_DATA_OFFSET 4
#define PACKET_SP_DATA_OFFSET 5
#define PACKET_AP_DATA_OFFSET 6

#define PACKET_PX_DATA_SIZE PACKET_DATA_DECIMAL_SIZE
#define PACKET_PY_DATA_SIZE PACKET_DATA_DECIMAL_SIZE
#define PACKET_SX_DATA_SIZE PACKET_DATA_DECIMAL_SIZE
#define PACKET_SY_DATA_SIZE PACKET_DATA_DECIMAL_SIZE
#define PACKET_PP_DATA_SIZE PACKET_DATA_DECIMAL_SIZE
#define PACKET_SP_DATA_SIZE PACKET_DATA_DECIMAL_SIZE
#define PACKET_AP_DATA_SIZE PACKET_DATA_DECIMAL_SIZE

#define INCLUDE_PACKET_PX_DATA   YES
#define INCLUDE_PACKET_PY_DATA   YES
#define INCLUDE_PACKET_SX_DATA   YES
#define INCLUDE_PACKET_SY_DATA   YES
#define INCLUDE_PACKET_PP_DATA   YES
#define INCLUDE_PACKET_SP_DATA   YES
#define INCLUDE_PACKET_AP_DATA   YES

#define PACKET_INCLUDES  (  INCLUDE_PACKET_PX_DATA << PACKET_PX_DATA_OFFSET \
| INCLUDE_PACKET_PY_DATA << PACKET_PY_DATA_OFFSET \
| INCLUDE_PACKET_SX_DATA << PACKET_SX_DATA_OFFSET \
| INCLUDE_PACKET_SY_DATA << PACKET_SY_DATA_OFFSET \
| INCLUDE_PACKET_PP_DATA << PACKET_PP_DATA_OFFSET \
| INCLUDE_PACKET_SP_DATA << PACKET_SP_DATA_OFFSET \
| INCLUDE_PACKET_AP_DATA << PACKET_AP_DATA_OFFSET \
)
#define PACKET_SIZE    (  INCLUDE_PACKET_PX_DATA * PACKET_PX_DATA_SIZE \
+ INCLUDE_PACKET_PY_DATA * PACKET_PY_DATA_SIZE \
+ INCLUDE_PACKET_SX_DATA * PACKET_SX_DATA_SIZE \
+ INCLUDE_PACKET_SY_DATA * PACKET_SY_DATA_SIZE \
+ INCLUDE_PACKET_PP_DATA * PACKET_PP_DATA_SIZE \
+ INCLUDE_PACKET_SP_DATA * PACKET_SP_DATA_SIZE \
+ INCLUDE_PACKET_AP_DATA * PACKET_AP_DATA_SIZE \
)
#define PACKET_OFFSETS  {                    \
                        PACKET_PX_DATA_SIZE, \
                        PACKET_PY_DATA_SIZE, \
                        PACKET_SX_DATA_SIZE, \
                        PACKET_SY_DATA_SIZE, \
                        PACKET_PP_DATA_SIZE, \
                        PACKET_SP_DATA_SIZE, \
                        PACKET_AP_DATA_SIZE, \
                        }
#define LOG2_MAX_PACKET_TYPE_SIZE 4
#define PACKET_OFFSET_WIDTH LOG2_MAX_PACKET_TYPE_SIZE
typedef struct
{
    uint8_t px:PACKET_OFFSET_WIDTH;
    uint8_t py:PACKET_OFFSET_WIDTH;
    uint8_t sx:PACKET_OFFSET_WIDTH;
    uint8_t sy:PACKET_OFFSET_WIDTH;
    uint8_t pp:PACKET_OFFSET_WIDTH;
    uint8_t sp:PACKET_OFFSET_WIDTH;
    uint8_t ap:PACKET_OFFSET_WIDTH;
} packet_offset_lookup_t;

typedef struct
{
uint8_t
    a:PACKET_OFFSET_WIDTH,
    b:PACKET_OFFSET_WIDTH;
}packing_template_t;

typedef struct
{
uint32_t *
    px,
    py,
    sx,
    sy,
    pp,
    sp,
    ap;
} packet_value_lookup_t;

typedef struct
{
uint8_t
    id,
    includes;
uint32_t
    timestamp;
uint8_t
    padding[6];
} packet_header_t;

typedef struct
{
    packet_header_t header;
    uint8_t          data[PACKET_SIZE];
} packet_t;

#define NUM_PACKET_ELEMENTS ((uint8_t)( sizeof(packet_offset_lookup_t) * ( 8 / PACKET_OFFSET_WIDTH ) ))

#endif /* rho_packet_h */
