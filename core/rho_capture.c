#include "rho_capture.h"

index_t * RhoCapture_CaptureRow( register byte_t sub_sample,   // capture buffer index
				 const register byte_t * capture_address,
                 const register byte_t thresh_value,    // address of thresh buffer
				 register index_t * thresh_address)
{
    register uint32_t working_register = 0;
    register uint32_t capture_offset = 0;

#ifndef __ASSEMBLY_RHO__
    while( 1 )
    {
        working_register = *( capture_address + capture_offset );
        capture_offset += sub_sample;
        if( capture_offset >= CAPTURE_BUFFER_SIZE )
        	break;
        if( working_register >= thresh_value )
            *(thresh_address++) = capture_offset;
    }
#else
    __asm volatile
	(
	"capture_start:"
		"ldrb 	%[wrk_r], [%[cap_a], %[cap_o]]" 									"\n\t"
		"add    %[cap_o], %[sub_s]    	@ Increment capture index by sub_sample"    "\n\t"
		"cmp    %[cap_o], %[cap_s]     	@ Check if capture reached max width"       "\n\t"
		"bge     capture_end           	@ If so, end"                               "\n\t"

		"cmp     %[wrk_r], %[thr_v]    	@ Compare with threshold value"             "\n\t"
		"blt     capture_start        	@ If less than, continue to next capture"   "\n\t"

		"strh    %[cap_o], [%[thr_a]], %[thr_w] @ Store offset word at thresh index"   	"\n\t"
		"b       capture_start        	@ Branch back to next capture"              "\n"
	"capture_end:"                                                                 	"\n"
		:
		[thr_a] "+r"(thresh_address)
		:
		[wrk_r] "r"(working_register),
		[sub_s] "r"(sub_sample),
		[cap_a] "r"(capture_address),
		[cap_o] "r"(capture_offset),
		[thr_v] "r"(thresh_value),
		[cap_s] "I"(CAPTURE_BUFFER_SIZE),
		[thr_w] "I"(sizeof(index_t))
	);
#endif
    return thresh_address;
}

section_process_t RhoCapture_ProcessFrameSection( const index_t rows,
				register index_t * thresh_address,
				const register index_t * thresh_end,
				const register density_t Cx,
				register sdensity_t * Dy,
				register sdensity_t * Dx_i )
{
    register uint32_t value_register 	= 0;
    const register sdensity_t * Dx_end  = Dx_i + rows;
	register density_2d_t Q_total       = 0;
	register density_2d_t Q_prev        = 0;
	register density_2d_t Q_left        = 0;
	register density_2d_t Q_right       = 0;
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
            *(Dx_i++) = Q_total - Q_prev; /// Ensure Dx_i increments by proper width
            Q_prev = Q_total;
            if( Dx_i >= Dx_end )
            {
                complete = true;
                break;
            }
        }
    }
#else
    register uint32_t working_register		= 0;
    __asm volatile
    (
	"sec_proc_loop:"                                                      			"\n\t"
		"ldrh    %[val_r], [%[thr_a]], %[thr_w] @ Load next threshold buffer"       "\n\t"
		"cmp     %[val_r], %[cap_s] 		@ Is value outside or equal frame width""\n\t"
		"bge     row_end        			@ Go to end row"                        "\n"
	"left_value:"                                                        			"\n\t"
		"cmp     %[val_r], %[c_x]         	@ If value is > (right) x centroid"		"\n\t"
		"bgt     right_value    			@ Branch to right quadrant updated"     "\n\t"
		"add     %[q_lft], #1   			@ Increment left quadrant"              "\n\t"
		"b       row_update    		 		@ Branch to row map update"             "\n"
	"right_value:"                                                       			"\n\t"
		"add     %[q_rgt], #1   			@ Increment right quadrant"             "\n"
	"row_update:"                                                        			"\n\t"
		"lsl 	 %[val_r], #1				@ Double value to match half-word width""\n\t"
		"ldrh    %[wrk_r], [%[d_y], %[val_r]]   @ Load word at row map"             "\n\t"
		"add     %[wrk_r], #1     			@ Increment row map value"              "\n\t"
		"strh    %[wrk_r], [%[d_y], %[val_r]]   @ Store incremented value"          "\n\t"
		"b       sec_proc_loop   			@ Loop back to start next values"       "\n"
	"row_end:"                                                           			"\n\t"
		"add     %[q_tot], %[q_lft], %[q_rgt]	@ Add left and right quadrants to total" "\n\t"
		"uqsub16 %[wrk_r], %[q_tot], %[q_prv]	@ Calculate active pixels in row"   "\n\t"
		"strh    %[wrk_r], [%[d_x]], %[d_w] 	@ Store at next column address"         "\n\t"
		"cmp     %[d_x], %[d_x_e]         	@ Check if all rows are processed"      "\n\t"
		"bge     sec_proc_end  			 	@ If so, end"                           "\n\t"
		"mov     %[q_prv], %[q_tot]        	@ Move current total px to previous"    "\n\t"
		"cmp     %[thr_a], %[thr_e]         @ Check for end of threshold buffer"    "\n\t"
		"blt     sec_proc_loop   			@ Loop back to start next values"       "\n"
	"sec_proc_end:                      	@ End if all rows are processed"        "\n"
		:
		[q_lft] "+r"(Q_left),
		[q_rgt] "+r"(Q_right),
		[d_x] "+r"(Dx_i)
		:
        [val_r] "r"(value_register),
        [wrk_r] "r"(working_register),
        [thr_a] "r"(thresh_address),
        [thr_e] "r"(thresh_end),
        [c_x] "r"(Cx),
        [d_y] "r"(Dy),
        [d_x_e] "r"(Dx_end),
        [q_tot] "r"(Q_total),
        [q_prv] "r"(Q_prev),
		[thr_w] "I"(sizeof(index_t)),
		[d_w] "I"(sizeof(sdensity_t)),
		[cap_s] "I" (CAPTURE_BUFFER_SIZE)
    );
#endif
    return (section_process_t){ Q_left, Q_right, Dx_i >= Dx_end };
}
