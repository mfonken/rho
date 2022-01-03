#include "rho_capture.h"

__attribute__((naked))
void RhoCapture_CaptureRow( register byte_t * capture_address,   // capture buffer index
                 register index_t * thresh_address,    // address of thresh buffer
                 const register byte_t thresh_value,
                 const register byte_t sub_sample,
                 const register byte_t * capture_end,
                 const register byte_t * capture_start,
                 const register byte_t * flag_address )
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
        if(capture_address >= capture_end) return;

        working_register = *(capture_address);
        if(working_register >= thresh_value )
        {
            working_register = capture_address - capture_start;
            *(thresh_address++) = working_register;
        }
    }
#else
    __asm volatile
    (
    "capture:                                                              \n\t"
#ifdef __CHECK_FRAME_FLAG__
        "ldrb    %0, [%7]       @ Load row end flag byte                   \n\t"
        "cmp     %0, #1         @ Check if end is reached (set)            \n\t"
        "bge     end            @ If so, end                               \n\t"
#endif
        "add     %2, %1     @ Increment capture index by sub_sample    \n\t"
        "cmp     %2, %3         @ Check if capture reached max width       \n\t"
        "bge     end1           @ If so,end                                \n\t"

        "ldrb    %0, [%2]       @ Load byte at capture index into RO       \n\t"
        "cmp     %0, %4         @ Compare with threshold value             \n\t"
        "blt     capture        @ If less than, continue to next capture   \n\t"

        "sub     %0, %2, %5     @ Subtract capture buffer start from index \n\t"
        "strh    %0, [%6], #2   @ Store offset word at thresh index        \n\t"
        "b       capture        @ Branch back to next capture              \n"
    "end1:                                                                 \n"
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

__attribute__((naked))
section_process_t RhoCapture_ProcessFrameSection( const index_t rows,
				register index_t * thresh_address,
				const register index_t * thresh_end,
				const register density_2d_t Cx,
				register density_2d_t * Dy,
				register density_2d_t * Dx_i )
{
    register density_2d_t value_register 	= 0;
    register density_2d_t * Dx_end          = Dx_i + rows;
	register density_2d_t Q_total           = 0;
	register density_2d_t Q_prev            = 0;
	register density_2d_t Q_left            = 0;
	register density_2d_t Q_right           = 0;
    bool complete = false;

#ifndef __ASSEMBLY_RHO__
    while( thresh_address <= thresh_end )
    {
        value_register = *thresh_address++;
        if(value_register < CAPTURE_WIDTH)
        {
            if( value_register < Cx )
                Q_left++;
            else
                Q_right++;
            Dy[value_register]++;
        }
        else
        {
            Q_total = Q_left + Q_right;
            *(Dx_i++) = Q_total - Q_prev; /// TODO: Ensure Dx_i increments by proper width
            Q_prev = Q_total;
            if( Dx_i >= Dx_end )
            {
                complete = true;
                break;
            }
        }
    }
#else
    register density_2d_t calc_register		= 0;
    __asm volatile
    (
    "loop_process:                                                      \n\t"
        "ldrh    %0, [%2], #1   @ Load next threshold buffer            \n\t"
        "cmp     %0, #"STR(CWL)"@ Is value outside or equal frame width \n\t"
        "bge     row_end        @ Go to end row                         \n"
    "left_value:                                                        \n\t"
        "cmp     %0, %4         @ If value is right (greater) x centroid\n\t"
        "bgt     right_value    @ Branch to right quadrant updated      \n\t"
        "add     %10, %10, #1   @ Increment left quadrant               \n\t"
        "b       row_update     @ Branch to row map update              \n"
    "right_value:                                                       \n\t"
        "add     %11, %11, #1   @ Increment right quadrant              \n"
    "row_update:                                                        \n\t"
        "ldrb    %1, [%5, %0]   @ Load word at row map                  \n\t"
        "add     %1, %1, #1     @ Increment row map value               \n\t"
        "strb    %1, [%5, %0]   @ Store incremented value               \n\t"
        "b       loop_process   @ Loop back to start next values        \n"
    "row_end:                                                           \n\t"
        "add     %8, %10, %11   @ Add left and right quadrants to total \n\t"
        "uqsub16 %1, %8, %9     @ Calculate active pixels in row        \n\t"
        "strb    %1, [%6], #1   @ Store at next column address          \n\t"
        "cmp     %6, %7         @ Check if all rows are processed       \n\t"
        "bge     end2           @ If so, end                            \n\t"
        "mov     %9, %10        @ Move current total px to previous     \n\t"
        "cmp     %2, %3         @ Check for end of threshold buffer     \n\t"
        "bge     end2           @ If so, end                            \n\t"
        "blt     loop_process   @ Loop back to start next values        \n"
    "end2:                      @ End if all rows are processed         \n"
        ::
        "r"(value_register),    // %0
        "r"(calc_register),     // %1
        "r"(thresh_address),    // %2
        "r"(thresh_end),        // %3
        "r"(Cx),                // %4
        "r"(Dy),                // %5
        "r"(Dx_i),              // %6
        "r"(Dx_end),            // %7
        "r"(Q_total),           // %8
        "r"(Q_prev),            // %9
        "r"(Q_left),            // %10
        "r"(Q_right)            // %11
    );
#endif
    return (section_process_t){ Q_left, Q_right, complete };
}
