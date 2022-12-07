#include "rho_capture.h"

index_t * RhoCapture_CaptureRow(
				 const register byte_t * capture_address,
                 const register byte_t thresh_value,
				 register index_t * thresh_address,
                 const index_t start_offset,
                 const index_t end_offset,
                 register byte_t sub_sample )
{
    register uint32_t working_register = 0;
    register uint32_t capture_offset = start_offset;
    int i = 0, j = 0;
#ifndef __ASSEMBLY_RHO__
    while( 1 )
    {
        working_register = *( capture_address + capture_offset );
        capture_offset += sub_sample;
        if( capture_offset >= end_offset )
        	break;
        if( working_register >= thresh_value )
        {
            *(++thresh_address) = capture_offset;
            j++;
        }
        i++;
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

index_t * RhoCapture_CaptureBlobs( rho_capture_t * _,
    index_t row,
    const register byte_t * capture_address,
    const register byte_t thresh_value,
    register index_t * thresh_address,
    const index_t length,
    register byte_t sub_sample )
{
    RhoCapture.RowEdgeTick( _, row );
    if( _->num_active > 0 )
    {
        // Capture active blobs
        bool blob_found = false;
        for( byte_t i = 0; i < MAX_BLOBS; i++ )
        {
            if( !_->blob_i_active[i] ) continue;
            index_t x = _->blobs[i].x; // TODO: On shadow, wrong blob may start first
            index_t l = _->blobs[i].w;
            if( x + l > length )
                l = length - x;
            if( l <= 0 )
                continue;
            index_t * new_thresh_address = RhoCapture.CaptureRow( capture_address, thresh_value, thresh_address, x, l + x, sub_sample );
            if( new_thresh_address != thresh_address )
            {
//                if(blob_found)
//                    printf("$");
                *thresh_address = CAPTURE_ROW_END * ( i + 1 + (blob_found ? 1 : 0) ) + row;
                thresh_address = new_thresh_address + 1;
                blob_found = true;
            }
        }
    }
    return thresh_address;
}

section_process_t RhoCapture_ProcessFrameSection( const index_t end_row,
				register index_t * thresh_address,
				const register index_t * thresh_end,
				const register density_t Cx,
				register sdensity_t * Dy,
				register sdensity_t * Dx,
                register index_t y,
                rho_capture_t * capture )
{
    if(end_row == 0) return (section_process_t) { 0, 0, true };
    register uint32_t v = 0; // Delimiters and column value
    density_2d_t Q_total       = 0;
    density_2d_t Q_prev        = 0;
    density_2d_t Q_left        = 0;
    density_2d_t Q_right       = 0;
    index_t thresh_proc = 0;
    int16_t x_offset = 0;
    int16_t y_offset = 0;
    int8_t curr_blob = -1;

#ifndef __ASSEMBLY_RHO__
    while( thresh_address < thresh_end )
    {
        v = *(thresh_address++);
        thresh_proc++;
        if( v < CAPTURE_ROW_END )
        {
            if( v < Cx )
                Q_left++;
            else
                Q_right++;
            Dy[v - x_offset]++;
        }
        else
        {
            Q_total = Q_left + Q_right;
            Dx[y] = Q_total - Q_prev;
            Q_prev = Q_total;
            
            v -= CAPTURE_ROW_END;
            if( v >= CAPTURE_ROW_END )
            {
                int8_t b = -1;
                do
                {
                    v -= CAPTURE_ROW_END;
                    b++;
                } while ( v >= CAPTURE_ROW_END );
                if( b != curr_blob && b >= 0 && capture != NULL )
                {
                    byte_t bi = capture->blobs_order[b]; // Watch out for possible need for edge_order instead
                    x_offset = capture->blobs[b].x - ( bi > 0 ) * capture->thresh_blob_loc[bi - 1].x;
                    y_offset = capture->blobs[b].y - ( bi > 0 ) * capture->thresh_blob_loc[bi - 1].y;
                    curr_blob = b;
                }
            }
//            else if( v == 0 ) v = end_row;
            if( v >= end_row ) break;
            if( (int16_t)v < y_offset )
                break;
            y = v - y_offset;
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
    return (section_process_t){ Q_left, Q_right, thresh_proc, v, v >= end_row };
}

void RhoCapture_RowEdgeTick( rho_capture_t * _, index_t row )
{
    if( row >= _->curr_edge->i )
    {
        _->blob_i_active[_->curr_edge->id] = _->curr_edge->open;
        _->num_active += _->curr_edge->open ? 1 : -1;
        _->edge_proc++;
        _->done = _->edge_proc >= *_->num_blobs * 2;
        _->curr_edge = &_->edges[_->edge_order[_->edge_proc]];
    }
}

void PrintBlobs( blob_t * blobs, byte_t * order, byte_t n )
{
    LOG_RHO_CAP(DEBUG_RHO_CAP, "Blobs: ");
    for( byte_t i = 0; i < n; i++ )
    {
        blob_t * b = &blobs[order[i]];
        LOG_RHO_CAP_BARE(DEBUG_RHO_CAP, "[x(%d -%d- %d)|", b->x, order[i], b->x + b->w);
        LOG_RHO_CAP_BARE(DEBUG_RHO_CAP, "y(%d -%d- %d)] ", b->y, order[i], b->y + b->h);
    }
    LOG_RHO_CAP_BARE(DEBUG_RHO_CAP, "\n");
}
void PrintEdges( edge_t * edges, byte_t * order, byte_t n )
{
    LOG_RHO_CAP(DEBUG_RHO_CAP, "Edges: ");
    for( byte_t i = 0; i < n * 2; i++ )
    {
        edge_t * e = &edges[order[i]];
        if(e->open)
        {
            LOG_RHO_CAP_BARE(DEBUG_RHO_CAP, "[%d -%d- ", e->i, e->id);
        }
        else
        {
            LOG_RHO_CAP_BARE(DEBUG_RHO_CAP, "%d] ", e->i);
        }
    }
    LOG_RHO_CAP_BARE(DEBUG_RHO_CAP, "\n");
}

void RhoCapture_OrderBlobs( blob_t * blobs, byte_t * order, byte_t n )
{
    int8_t i = n, j;
    for( ; i >= 0; i-- )
        order[i] = i;
//    PrintBlobs(blobs, order, n);
    for( i = 0; i < n - 1; i++ )
        for( j = 0; j < n - i - 1; j++ )
            if( blobs[order[j]].y > blobs[order[j + 1]].y )
                SWAP( order[j], order[j + 1] );
    PrintBlobs(blobs, order, n);
}

void RhoCapture_OrderEdges( blob_t * blobs, byte_t * blobs_order, byte_t n, edge_t * edges, byte_t * edges_order, index_t max_i )
{
    for( byte_t i = 0; i < n * 2; i++ )
    {
        edges_order[i] = i;
        edge_t *edge = &edges[i];
        edge->open = !(i % 2);
        byte_t i_ = blobs_order[i >> 1];
        edge->i = i % 2 ? blobs[i_].y + blobs[i_].h : blobs[i_].y;
        edge->id = i_;
    }
//    PrintEdges(edges, edges_order, n);
    for( byte_t i = 0; i < n * 2 - 1; i++ )
    {
        for( byte_t j = 0; j < n * 2 - i - 1; j++ )
        {
            if( edges[edges_order[j]].i > edges[edges_order[j+1]].i )
            {
                byte_t v = edges_order[j];
                edges_order[j] = edges_order[j+1];
                edges_order[j+1] = v;
            }
        }
    }
//    PrintEdges(edges, edges_order, n);
    for( byte_t i = 0; i < n * 2; i++ )
    {
        edge_t * edge = &edges[edges_order[i]];
        edge->i = MIN( edge->i, max_i-1 );
    }
    PrintEdges(edges, edges_order, n);
}

index_t RhoCapture_RealWidthOnAxis( blob_t * blob, index_t offset, index_t max, bool x_axis, byte_t * n )
{
    if(blob->h > 1000)
        printf(",");
    index_t * dim = x_axis ? &blob->w : &blob->h;
    index_t v = *dim + offset;
    if( v > max )
    {
        *dim -= v - max;
        if( *dim <= CAPTURE_THRESH_MIN_BLOB_DIM )
            (*n)--; // Note elimination when blob doesn't fit
        v = max;
    }
    return v;
}

void RhoCapture_AssignBlobsInThreshBuffer( index_pair_t * thresh_blob_loc, index_pair_t thresh_max, blob_t * blobs, byte_t * n, edge_t * edges, byte_t * edge_order )
{
    thresh_blob_loc[0].x = 0;
    thresh_blob_loc[0].y = 0;
    for( byte_t i = 0; i < *n; i++ )
    {
        edge_t * edge = &edges[i << 1];
        blob_t * b = &blobs[edge->id];
        if(b->h > 1000)
            printf(",");
        byte_t i_ = i ? i - 1 : 0;
        thresh_blob_loc[i].x = RhoCapture_RealWidthOnAxis( b, thresh_blob_loc[i_].x, thresh_max.x, true, n );
        thresh_blob_loc[i].y = RhoCapture_RealWidthOnAxis( b, thresh_blob_loc[i_].y, thresh_max.y, false, n );
    }
    LOG_RHO_CAP(DEBUG_RHO_CAP, "Assign Blobs:\n X -");
    for( int f = -1; f < *n; f++ )
        LOG_RHO_CAP_BARE(DEBUG_RHO_CAP, "| %d[%d] ", f+1, f < 0 ? 0 : thresh_blob_loc[f].x);
    LOG_RHO_CAP_BARE(DEBUG_RHO_CAP, "|\n Y -");
    for( int f = -1; f < *n; f++ )
        LOG_RHO_CAP_BARE(DEBUG_RHO_CAP, "| %d[%d] ", f+1, f < 0 ? 0 : thresh_blob_loc[f].y);
    LOG_RHO_CAP_BARE(DEBUG_RHO_CAP, "|\n");
}

void RhoCapture_PrepareBlobsForCapture( rho_capture_t * _ )
{
    RhoCapture.OrderBlobs( _->blobs, _->blobs_order, *_->num_blobs );
    RhoCapture.OrderEdges( _->blobs, _->blobs_order, *_->num_blobs, _->edges, _->edge_order, _->capture_size.y );
    RhoCapture.AssignBlobsInThreshBuffer( _->thresh_blob_loc, _->capture_size, _->blobs, _->num_blobs, _->edges, _->edge_order );
    
    _->curr_edge = &_->edges[_->edge_order[_->edge_proc]];
}

void RhoCapture_AddBlob( rho_capture_t * _, blob_t blob )
{
    memcpy( &_->blobs[*_->num_blobs++], &blob, sizeof(blob_t) );
}

void RhoCapture_ResetEdges( rho_capture_t * _ )
{
    _->edge_proc = 0;
    _->num_active = 0;
    _->capture_size.x = CAPTURE_THRESH_X_MAX;
    _->capture_size.y = CAPTURE_THRESH_Y_MAX;
    _->done = false;
    memset( _->edges, 0, sizeof(_->edges[0]) * MAX_BLOBS * 2);
    memset( _->edge_order, 0, sizeof(_->edge_order[0]) * MAX_BLOBS * 2);
    memset( _->thresh_blob_loc, 0, sizeof(_->thresh_blob_loc[0]) * MAX_BLOBS);
}

void RhoCapture_ResetBlobs( rho_capture_t * _ )
{
    _->num_blobs = 0;
    memset( _->blobs, 0, sizeof(_->blobs[0]) * MAX_BLOBS);
    memset( _->blob_i_active, 0, sizeof(_->blob_i_active[0]) * MAX_BLOBS);
}

void RhoCapture_ResetAll( rho_capture_t * _ )
{
    RhoCapture_ResetEdges( _ );
//    RhoCapture_ResetBlobs( _ );
    memset( _->blob_i_active, 0, sizeof(_->blob_i_active[0]) * MAX_BLOBS);
}

rho_capture_functions RhoCapture =
{
    .CaptureRow = RhoCapture_CaptureRow,
    .CaptureBlobs = RhoCapture_CaptureBlobs,
    .ProcessFrameSection = RhoCapture_ProcessFrameSection,
    .RowEdgeTick = RhoCapture_RowEdgeTick,
    .OrderBlobs = RhoCapture_OrderBlobs,
    .OrderEdges = RhoCapture_OrderEdges,
    .AssignBlobsInThreshBuffer = RhoCapture_AssignBlobsInThreshBuffer,
    .PrepareBlobsForCapture = RhoCapture_PrepareBlobsForCapture,
    .AddBlob = RhoCapture_AddBlob,
    .ResetEdges = RhoCapture_ResetEdges,
    .ResetBlobs = RhoCapture_ResetBlobs,
    .ResetAll = RhoCapture_ResetAll
};
