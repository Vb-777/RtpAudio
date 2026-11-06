/* SPDX-License-Identifier: MIT */
/*
 * adpcm.h - IMA ADPCM audio codec, 4:1 compression, 16-bit PCM in/out.
 *
 * WHY ADPCM AND NOT A TRANSFORM CODEC
 * ===================================
 * ADPCM is the codec class that actually ships on constrained real-time links:
 * it is sample-synchronous, has no lookahead, no frame reordering and a fixed
 * 4-bit-per-sample output, so end-to-end algorithmic delay is a single sample
 * rather than the 20-60 ms a transform codec needs for its analysis window.
 * On a voice or telemetry path where the budget is latency rather than bitrate,
 * that trade is the right one -- which is why it remains in G.726, in Bluetooth
 * mSBC's lineage and in a great deal of embedded audio.
 *
 * HOW IT WORKS
 * ============
 * The encoder predicts each sample from the previous reconstructed one and
 * transmits only a quantised 4-bit version of the error. The quantiser step
 * size is not transmitted: both sides derive it from the decoded data using the
 * same table-driven rule, so the decoder's step size tracks the encoder's
 * exactly as long as no data is lost.
 *
 * That shared-state design is the interesting engineering consequence. The
 * codec is *stateful across packets*: predictor and step index carry over, so a
 * lost packet does not merely delete its own audio -- it desynchronises the
 * decoder's state and corrupts everything after it until the state reconverges.
 * Handling that is the job of the sender's per-packet state reset (see rtp.h)
 * and the receiver's concealment, and the measured cost of getting it wrong is
 * in the project's benchmark.
 */

#ifndef ADPCM_H
#define ADPCM_H

#include <stddef.h>
#include <stdint.h>

/* Codec state. Must be identical on both sides at the start of a block. */
struct adpcm_state {
	int16_t predictor;   /* last reconstructed sample */
	int8_t  step_index;  /* index into the step-size table, 0..88 */
};

static inline void adpcm_init(struct adpcm_state *s)
{
	s->predictor = 0;
	s->step_index = 0;
}

/*
 * Encode @n_samples of 16-bit PCM into 4-bit codes, two per output byte
 * (low nibble first). @out must hold at least (n_samples + 1) / 2 bytes.
 * Returns bytes written. @state is updated in place.
 */
size_t adpcm_encode(struct adpcm_state *state, const int16_t *pcm,
		    size_t n_samples, uint8_t *out);

/*
 * Decode @n_samples samples from @in. @out must hold @n_samples int16_t.
 * Returns samples written.
 */
size_t adpcm_decode(struct adpcm_state *state, const uint8_t *in,
		    size_t n_samples, int16_t *out);

/* Compressed size for a sample count, and the inverse. */
static inline size_t adpcm_encoded_bytes(size_t n_samples)
{
	return (n_samples + 1) / 2;
}

static inline size_t adpcm_sample_count(size_t n_bytes)
{
	return n_bytes * 2;
}

/* ------------------------------------------------------------------ */
/* Quality measurement                                                 */
/* ------------------------------------------------------------------ */

/*
 * Signal-to-noise ratio in dB between a reference and a reconstruction.
 * Returns +INFINITY for an exact match and -INFINITY when the reference is
 * silent but the reconstruction is not.
 *
 * SNR is reported rather than a perceptual score because it is reproducible
 * without a listening panel. It is the right metric for a waveform codec,
 * which ADPCM is -- it tries to reproduce the waveform, not merely something
 * that sounds similar.
 */
double snr_db(const int16_t *reference, const int16_t *actual, size_t n);

/* Peak absolute sample error, which bounds the worst-case click. */
int max_abs_error(const int16_t *reference, const int16_t *actual, size_t n);

#endif /* ADPCM_H */
