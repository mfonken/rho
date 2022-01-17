.syntax unified
.text
.global CaptureRow

#define wrk_r	r0
#define sub_s	r1
#define cap_a 	r2  // capture buffer index
#define cap_w	r3
#define cap_s	r4
#define thr_v	r5
#define thr_a 	r6  // address of thresh buffer
#define flg_a	r7
#define cap_o r8

	mov 		cap_o, #0
CaptureRow:
#ifdef __CHECK_FRAME_FLAG__
	ldrb    wrk_r, [flg_a]  	// Load row end flag byte
	cmp     wrk_r, #1         	// Check if end is reached (set)
	bge     CaptureRowEnd  		// If so, end
#endif

    ldrb   	wrk_r, [cap_a, cap_o]      // Load byte at capture index into RO
    add     cap_o, sub_s     	// Increment capture index by sub_sample
    cmp     cap_o, cap_w        // Check if capture reached max width
    bge     CaptureRowEnd       // If so,end

    cmp     wrk_r, thr_v        // Compare with threshold value
    blt     CaptureRow         	// If less than, continue to next capture

    strh    cap_o, [thr_a], #2  // Store offset word at thresh index
    b       CaptureRow    		// Branch back to next capture
CaptureRowEnd:
