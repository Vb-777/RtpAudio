/* SPDX-License-Identifier: MIT */
/*
 * jitterbuf.h - Reordering jitter buffer with packet-loss concealment.
 *
 * WHAT A JITTER BUFFER IS FOR
 * ===========================
 * UDP gives no ordering and no delivery guarantee, but audio playout is
 * strictly periodic: the sound card wants the next frame every 20 ms whether or
 * not the network cooperated. The jitter buffer is the component that absorbs
 * that mismatch, and every decision it makes is a latency-versus-dropout trade:
 *
 *   - Too shallow and a packet that arrives slightly late is discarded as
 *     useless, producing an audible gap.
 *   - Too deep and every packet waits behind a buffer it did not need, adding
 *     delay to the whole call.
 *
 * So depth is a parameter here, not a constant, and the benchmark measures the
 * resulting curve rather than asserting a best value.
 *
 * SEQUENCE NUMBER WRAPAROUND
 * ==========================
 * RTP sequence numbers are 16-bit and wrap every 65,536 packets -- about 22
 * minutes at 20 ms packets, so any real call hits it. Comparisons must be done
 * in modular arithmetic: `(int16_t)(a - b) > 0` rather than `a > b`. Using the
 * naive comparison makes the buffer treat the wrap as a 65,000-packet jump
 * backwards and discard everything for the rest of the call. This is the bug
 * the wraparound test exists to pin down.
 *
 * CONCEALMENT
 * ===========
 * When a packet never arrives, something must be played. Three strategies are
 * implemented so their cost can be compared:
 *
 *   SILENCE  - zero-fill. Simple, and audibly the worst: a hard edge into and
 *              out of silence is heard as a click on top of the gap.
 *   HOLD     - repeat the last received frame. Cheap, but a repeated pitch
 *              period buzzes on sustained sounds.
 *   FADE     - repeat the last frame with a linear fade to zero, which removes
 *              the discontinuity at both edges. This is the default and is the
 *              basic form of what G.711 Appendix I does properly.
 */

#ifndef JITTERBUF_H
#define JITTERBUF_H

#include <stddef.h>
#include <stdint.h>

#define JB_MAX_DEPTH      64
#define JB_MAX_PACKET   1500   /* one Ethernet MTU */

enum jb_conceal {
	JB_CONCEAL_SILENCE = 0,
	JB_CONCEAL_HOLD    = 1,
	JB_CONCEAL_FADE    = 2,
};

struct jb_slot {
	uint8_t  data[JB_MAX_PACKET];
	size_t   len;
	uint16_t sequence;
	int      occupied;
};

struct jitterbuf {
	struct jb_slot slots[JB_MAX_DEPTH];
	unsigned depth;               /* packets held before playout starts */
	enum jb_conceal conceal;

	int      started;             /* has playout begun */
	uint16_t next_sequence;       /* sequence expected next */
	uint16_t highest_seen;        /* highest sequence ever accepted */
	int      have_highest;

	/* Last successfully decoded frame, for HOLD and FADE concealment. */
	int16_t  last_frame[JB_MAX_PACKET * 2];
	size_t   last_frame_samples;

	/* Statistics. */
	unsigned received;
	unsigned played;
	unsigned concealed;           /* frames synthesised for missing packets */
	unsigned reordered;           /* arrived out of order but still usable */
	unsigned too_late;            /* arrived after their playout slot */
	unsigned duplicates;
	unsigned overflows;           /* dropped because the buffer was full */
};

void jb_init(struct jitterbuf *jb, unsigned depth, enum jb_conceal conceal);

/*
 * Offer a received packet to the buffer. Returns:
 *    1  accepted
 *    0  rejected as a duplicate or as too late to be useful
 *   -1  rejected because the buffer is full, or the packet is oversized
 */
int jb_put(struct jitterbuf *jb, const uint8_t *pkt, size_t len);

/*
 * Produce the next frame of PCM for playout. Returns samples written.
 *
 * Returns 0 only while priming (fewer than @depth packets buffered and playout
 * has not started). Once started it always produces a frame, synthesising one
 * by concealment when the expected packet is absent -- because a playout path
 * that can return nothing is a playout path that underruns.
 */
size_t jb_get(struct jitterbuf *jb, int16_t *pcm, size_t pcm_cap);

/* Packets currently held. */
unsigned jb_occupancy(const struct jitterbuf *jb);

/* Modular sequence comparison: >0 if a is after b, <0 before, 0 equal. */
static inline int jb_seq_cmp(uint16_t a, uint16_t b)
{
	return (int16_t)(a - b);
}

#endif /* JITTERBUF_H */
