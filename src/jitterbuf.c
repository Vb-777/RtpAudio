/* SPDX-License-Identifier: MIT */

#include <string.h>

#include "jitterbuf.h"
#include "rtp.h"

void jb_init(struct jitterbuf *jb, unsigned depth, enum jb_conceal conceal)
{
	memset(jb, 0, sizeof(*jb));
	if (depth == 0)
		depth = 1;
	if (depth > JB_MAX_DEPTH)
		depth = JB_MAX_DEPTH;
	jb->depth = depth;
	jb->conceal = conceal;
}

unsigned jb_occupancy(const struct jitterbuf *jb)
{
	unsigned i, n = 0;

	for (i = 0; i < JB_MAX_DEPTH; i++)
		if (jb->slots[i].occupied)
			n++;
	return n;
}

static struct jb_slot *find_slot(struct jitterbuf *jb, uint16_t sequence)
{
	unsigned i;

	for (i = 0; i < JB_MAX_DEPTH; i++) {
		if (jb->slots[i].occupied &&
		    jb->slots[i].sequence == sequence)
			return &jb->slots[i];
	}
	return NULL;
}

static struct jb_slot *free_slot(struct jitterbuf *jb)
{
	unsigned i;

	for (i = 0; i < JB_MAX_DEPTH; i++)
		if (!jb->slots[i].occupied)
			return &jb->slots[i];
	return NULL;
}

/* ------------------------------------------------------------------ */

int jb_put(struct jitterbuf *jb, const uint8_t *pkt, size_t len)
{
	struct rtp_header h;
	struct jb_slot *slot;

	if (len > JB_MAX_PACKET || rtp_parse_header(pkt, len, &h) < 0)
		return -1;

	jb->received++;

	if (find_slot(jb, h.sequence)) {
		jb->duplicates++;
		return 0;
	}

	/*
	 * Once playout has started, a packet whose sequence is already behind
	 * the playout cursor cannot be used: its slot in time has passed and
	 * inserting it would play audio out of order. Counted separately from
	 * loss, because the two call for different fixes -- too_late means the
	 * buffer is too shallow for the path's jitter, while a never-arriving
	 * packet means real loss.
	 */
	if (jb->started && jb_seq_cmp(h.sequence, jb->next_sequence) < 0) {
		jb->too_late++;
		return 0;
	}

	slot = free_slot(jb);
	if (!slot) {
		jb->overflows++;
		return -1;
	}

	memcpy(slot->data, pkt, len);
	slot->len = len;
	slot->sequence = h.sequence;
	slot->occupied = 1;

	/*
	 * Reordering means this packet arrived *after* one with a higher
	 * sequence number, so it is out of order on the wire.
	 *
	 * Comparing against the playout cursor instead counts every normally
	 * buffered packet as reordered -- a packet sitting ahead of the cursor
	 * is exactly what a jitter buffer is supposed to contain. That bug made
	 * a clean in-order 150-packet run report 147 reorderings, which is
	 * worse than having no statistic at all.
	 */
	if (jb->have_highest && jb_seq_cmp(h.sequence, jb->highest_seen) < 0)
		jb->reordered++;
	if (!jb->have_highest ||
	    jb_seq_cmp(h.sequence, jb->highest_seen) > 0) {
		jb->highest_seen = h.sequence;
		jb->have_highest = 1;
	}

	return 1;
}

/* ------------------------------------------------------------------ */

static uint16_t oldest_sequence(const struct jitterbuf *jb, int *found)
{
	uint16_t best = 0;
	unsigned i;

	*found = 0;
	for (i = 0; i < JB_MAX_DEPTH; i++) {
		if (!jb->slots[i].occupied)
			continue;
		if (!*found || jb_seq_cmp(jb->slots[i].sequence, best) < 0) {
			best = jb->slots[i].sequence;
			*found = 1;
		}
	}
	return best;
}

static size_t conceal_frame(struct jitterbuf *jb, int16_t *pcm, size_t pcm_cap)
{
	size_t n = jb->last_frame_samples;
	size_t i;

	jb->concealed++;

	if (n == 0 || n > pcm_cap) {
		/* Nothing to repeat yet (loss before the first packet), or the
		 * caller's buffer is smaller than the last frame. Silence is the
		 * only safe answer. */
		n = n > pcm_cap ? pcm_cap : n;
		memset(pcm, 0, n * sizeof(int16_t));
		return n;
	}

	switch (jb->conceal) {
	case JB_CONCEAL_SILENCE:
		memset(pcm, 0, n * sizeof(int16_t));
		break;

	case JB_CONCEAL_HOLD:
		memcpy(pcm, jb->last_frame, n * sizeof(int16_t));
		break;

	case JB_CONCEAL_FADE:
	default:
		/* Linear ramp to zero across the frame. Removes the step
		 * discontinuity at both edges of the gap, which is what is
		 * actually heard as a click. */
		for (i = 0; i < n; i++) {
			int32_t scaled = (int32_t)jb->last_frame[i] *
					 (int32_t)(n - i);

			pcm[i] = (int16_t)(scaled / (int32_t)n);
		}
		/* Decay the stored frame too, so consecutive losses fade out
		 * rather than each restarting from full amplitude. */
		for (i = 0; i < n; i++)
			jb->last_frame[i] = pcm[i];
		break;
	}
	return n;
}

size_t jb_get(struct jitterbuf *jb, int16_t *pcm, size_t pcm_cap)
{
	struct jb_slot *slot;
	size_t produced;
	int found;

	/* Priming: hold output until the buffer has its target depth, which is
	 * what gives later packets room to be late. */
	if (!jb->started) {
		if (jb_occupancy(jb) < jb->depth)
			return 0;
		jb->next_sequence = oldest_sequence(jb, &found);
		if (!found)
			return 0;
		jb->started = 1;
	}

	slot = find_slot(jb, jb->next_sequence);

	if (!slot) {
		/*
		 * The expected packet is not here. Two cases:
		 *   - the buffer still holds later packets, so this one is
		 *     genuinely lost or still in flight: conceal and move on;
		 *   - the buffer is empty, meaning the sender has stopped or
		 *     the path has stalled: report nothing so the caller can
		 *     decide to stop rather than generate endless concealment.
		 */
		if (jb_occupancy(jb) == 0)
			return 0;
		jb->next_sequence++;
		return conceal_frame(jb, pcm, pcm_cap);
	}

	produced = rtp_unpack(slot->data, slot->len, pcm, pcm_cap, NULL);
	slot->occupied = 0;
	jb->next_sequence++;

	if (produced == 0) {
		/* Parsed as RTP but failed to decode: treat as a loss rather
		 * than emitting an empty frame into the playout path. */
		return conceal_frame(jb, pcm, pcm_cap);
	}

	/* Remember for concealment. */
	if (produced <= sizeof(jb->last_frame) / sizeof(jb->last_frame[0])) {
		memcpy(jb->last_frame, pcm, produced * sizeof(int16_t));
		jb->last_frame_samples = produced;
	}

	jb->played++;
	return produced;
}
