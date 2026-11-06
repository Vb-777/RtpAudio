/* SPDX-License-Identifier: MIT */
/*
 * adpcm.c - IMA ADPCM encoder and decoder.
 *
 * The two tables below are the standard IMA ones and are what make encoder and
 * decoder agree on step size without transmitting it. `step_table` is a
 * roughly exponential ladder (each entry ~1.1x the last, spanning 7 to 32767)
 * so the quantiser adapts across the full dynamic range in a bounded number of
 * steps; `index_adjust` moves up the ladder fast when the signal is loud and
 * down slowly when it is quiet, which is the asymmetry that keeps the codec
 * from ringing after a transient.
 */

#include <math.h>

#include "adpcm.h"

/* Quantiser step sizes: 89 entries, exponential. */
static const int16_t step_table[89] = {
	    7,     8,     9,    10,    11,    12,    13,    14,    16,    17,
	   19,    21,    23,    25,    28,    31,    34,    37,    41,    45,
	   50,    55,    60,    66,    73,    80,    88,    97,   107,   118,
	  130,   143,   157,   173,   190,   209,   230,   253,   279,   307,
	  337,   371,   408,   449,   494,   544,   598,   658,   724,   796,
	  876,   963,  1060,  1166,  1282,  1411,  1552,  1707,  1878,  2066,
	 2272,  2499,  2749,  3024,  3327,  3660,  4026,  4428,  4871,  5358,
	 5894,  6484,  7132,  7845,  8630,  9493, 10442, 11487, 12635, 13899,
	15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
};

/* Step-index delta per code magnitude: climb fast, decay slowly. */
static const int8_t index_adjust[16] = {
	-1, -1, -1, -1, 2, 4, 6, 8,
	-1, -1, -1, -1, 2, 4, 6, 8
};

static inline int clamp_index(int idx)
{
	if (idx < 0)
		return 0;
	if (idx > 88)
		return 88;
	return idx;
}

static inline int16_t clamp_sample(int v)
{
	if (v > 32767)
		return 32767;
	if (v < -32768)
		return -32768;
	return (int16_t)v;
}

/* ------------------------------------------------------------------ */

static uint8_t encode_sample(struct adpcm_state *st, int16_t sample)
{
	int step = step_table[st->step_index];
	int diff = sample - st->predictor;
	uint8_t code = 0;
	int diffq;

	if (diff < 0) {
		code = 8;          /* sign bit */
		diff = -diff;
	}

	/* Successive-approximation of diff against step/1, /2, /4.
	 * Equivalent to diff * 4 / step clamped to 7, but without the divide --
	 * which is the form that matters on a DSP or an MCU with no divider. */
	diffq = step;
	if (diff >= diffq) {
		code |= 4;
		diff -= diffq;
	}
	diffq >>= 1;
	if (diff >= diffq) {
		code |= 2;
		diff -= diffq;
	}
	diffq >>= 1;
	if (diff >= diffq)
		code |= 1;

	/* Reconstruct exactly as the decoder will, so the predictor on both
	 * sides stays identical. Computing the predictor from the *input*
	 * sample instead is the classic ADPCM bug: it drifts immediately. */
	{
		int delta = step >> 3;

		if (code & 4)
			delta += step;
		if (code & 2)
			delta += step >> 1;
		if (code & 1)
			delta += step >> 2;
		if (code & 8)
			delta = -delta;
		st->predictor = clamp_sample(st->predictor + delta);
	}

	st->step_index = (int8_t)clamp_index(st->step_index +
					     index_adjust[code]);
	return code;
}

static int16_t decode_sample(struct adpcm_state *st, uint8_t code)
{
	int step = step_table[st->step_index];
	int delta = step >> 3;

	if (code & 4)
		delta += step;
	if (code & 2)
		delta += step >> 1;
	if (code & 1)
		delta += step >> 2;
	if (code & 8)
		delta = -delta;

	st->predictor = clamp_sample(st->predictor + delta);
	st->step_index = (int8_t)clamp_index(st->step_index +
					     index_adjust[code]);
	return st->predictor;
}

/* ------------------------------------------------------------------ */

size_t adpcm_encode(struct adpcm_state *state, const int16_t *pcm,
		    size_t n_samples, uint8_t *out)
{
	size_t i, written = 0;
	uint8_t packed = 0;

	for (i = 0; i < n_samples; i++) {
		uint8_t code = encode_sample(state, pcm[i]);

		if ((i & 1) == 0) {
			packed = code & 0x0f;          /* low nibble first */
		} else {
			packed |= (uint8_t)(code << 4);
			out[written++] = packed;
			packed = 0;
		}
	}
	/* Odd sample count: flush the dangling nibble with a zero high half. */
	if (n_samples & 1)
		out[written++] = packed;

	return written;
}

size_t adpcm_decode(struct adpcm_state *state, const uint8_t *in,
		    size_t n_samples, int16_t *out)
{
	size_t i;

	for (i = 0; i < n_samples; i++) {
		uint8_t byte = in[i >> 1];
		uint8_t code = (i & 1) ? (uint8_t)(byte >> 4) : (byte & 0x0f);

		out[i] = decode_sample(state, code);
	}
	return n_samples;
}

/* ------------------------------------------------------------------ */

double snr_db(const int16_t *reference, const int16_t *actual, size_t n)
{
	double signal = 0.0, noise = 0.0;
	size_t i;

	for (i = 0; i < n; i++) {
		double s = (double)reference[i];
		double e = (double)reference[i] - (double)actual[i];

		signal += s * s;
		noise += e * e;
	}
	if (noise == 0.0)
		return INFINITY;
	if (signal == 0.0)
		return -INFINITY;
	return 10.0 * log10(signal / noise);
}

int max_abs_error(const int16_t *reference, const int16_t *actual, size_t n)
{
	int worst = 0;
	size_t i;

	for (i = 0; i < n; i++) {
		int e = reference[i] - actual[i];

		if (e < 0)
			e = -e;
		if (e > worst)
			worst = e;
	}
	return worst;
}
