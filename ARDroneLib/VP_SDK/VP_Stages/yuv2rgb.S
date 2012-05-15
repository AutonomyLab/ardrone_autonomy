
#define	ycc			r0
#define	y			r1
#define	r			r2
#define	g			r3
#define	b			r4
#define	s			r5
#define	d			r6
#define	err			r7
#define	count		r8
#define	_1fe_		r9
#define	_0_			r10

	.global	asm_yuvrgb565

//static void flushline_565(uint16_t* dst, int32_t dst_rbytes, uint32_t* src, int y);
asm_yuvrgb565:
	stmdb	sp!, {r4-r10, lr}
  mov   s, r2
	mov		d, r0
	mla		d, r1, r3, d
	mov		_1fe_, #0xFF
	tst		r3, #1
	mov		err, #4
	eorne	err, err, #0x60000
	eorne	err, err, #0x6
	add		_1fe_, _1fe_, _1fe_
	mov		_0_, #0

	mov		count, #320
.loop:
	ldr		ycc, [s]
	mov		y, ycc, lsr #24
	str		_0_, [s], #4
	and		r, _1fe_, ycc, lsr #15
	and		g, _1fe_, ycc, lsr #7
	and		b, _1fe_, ycc, lsl #1
	sub		r, r, #180
	sub		g, g, #135
	sub		b, b, #227
	add		r, r, y
	sub		g, y, g
	add		b, b, y
	adds	r, r, err, lsr #16
	movmi	r, _0_
	cmp		r, #0xF8
	movgt	r, #0xF8
  and   r, r, #0xF8
	adds	g, g, err, lsr #17
	movmi	g, _0_
	cmp		g, #0xFC
	movgt	g, #0xFC
  and   g, g, #0xFC
	adds	b, b, err, lsr #16
	movmi	b, _0_
	cmp		b, #0xF8
	movgt	b, #0xF8
	mov		r, r, lsl #8
	add		r, r, g, lsl #3
	add		r, r, b, lsr #3
	strh	r, [d], #2
	mov		err, err, ror #16
	subs	count, count, #1
	bne		.loop

	ldmia	sp!, {r4-r10, pc}
