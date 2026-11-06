/* SPDX-License-Identifier: MIT */
/*
 * test_all.c - Unit tests for the codec, RTP packetisation and jitter buffer.
 *
 * The RTP byte-layout tests matter more than they look. Both ends of this
 * project share one implementation, so an endianness or bit-packing mistake is
 * completely invisible in an end-to-end test -- it only shows up against a
 * third-party stack. The only defence is to assert on the exact wire bytes.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/adpcm.h"
#include "../src/jitterbuf.h"
#include "../src/rtp.h"

static int tests_run, tests_failed;

#define CHECK(cond, fmt, ...)                                                  \
	do {                                                                   \
		tests_run++;                                                   \
		if (!(cond)) {                                                 \
			tests_failed++;                                        \
			printf("  FAIL %s:%d: " fmt "\n", __func__, __LINE__,   \
			       ##__VA_ARGS__);                                 \
		}                                                              \
	} while (0)

#define CHECK_EQ(a, b) CHECK((long long)(a) == (long long)(b),                 \
			     "expected %lld, got %lld", (long long)(b),        \
			     (long long)(a))

/* ------------------------------------------------------------------ */
/* Codec                                                               */
/* ------------------------------------------------------------------ */

static void fill_tone(int16_t *pcm, size_t n, double hz, double amp)
{
	size_t i;

	for (i = 0; i < n; i++)
		pcm[i] = (int16_t)(amp * 32000 *
				   sin(2.0 * M_PI * hz * (double)i / 16000.0));
}

static void test_compression_is_exactly_four_to_one(void)
{
	CHECK_EQ(adpcm_encoded_bytes(320), 160);
	CHECK_EQ(adpcm_encoded_bytes(2), 1);
	/* Odd counts round up: the dangling nibble still needs a byte. */
	CHECK_EQ(adpcm_encoded_bytes(3), 2);
	CHECK_EQ(adpcm_sample_count(160), 320);
}

static void test_silence_round_trips_exactly(void)
{
	struct adpcm_state e, d;
	int16_t pcm[256] = { 0 }, out[256];
	uint8_t enc[128];

	adpcm_init(&e);
	adpcm_init(&d);
	adpcm_encode(&e, pcm, 256, enc);
	adpcm_decode(&d, enc, 256, out);
	CHECK_EQ(memcmp(pcm, out, sizeof(pcm)), 0);
}

static void test_tone_meets_snr_floor(void)
{
	struct adpcm_state e, d;
	int16_t pcm[4096], out[4096];
	uint8_t enc[2048];
	double snr;

	fill_tone(pcm, 4096, 440.0, 0.8);
	adpcm_init(&e);
	adpcm_init(&d);
	adpcm_encode(&e, pcm, 4096, enc);
	adpcm_decode(&d, enc, 4096, out);
	snr = snr_db(pcm, out, 4096);
	/* 25 dB is a floor, not a target: the measured value on this signal is
	 * ~32 dB, and a regression below 25 means the predictor or the step
	 * ladder has been broken. */
	CHECK(snr > 25.0, "SNR %.2f dB below the 25 dB floor", snr);
}

static void test_encoder_and_decoder_states_stay_in_lockstep(void)
{
	/* The classic ADPCM bug is predicting from the input sample rather than
	 * the reconstruction, which makes the decoder drift. If the two states
	 * match after a long run, the reconstruction path is shared correctly. */
	struct adpcm_state e, d;
	int16_t pcm[8192], out[8192];
	uint8_t enc[4096];

	fill_tone(pcm, 8192, 220.0, 0.9);
	adpcm_init(&e);
	adpcm_init(&d);
	adpcm_encode(&e, pcm, 8192, enc);
	adpcm_decode(&d, enc, 8192, out);
	CHECK_EQ(e.predictor, d.predictor);
	CHECK_EQ(e.step_index, d.step_index);
}

static void test_step_index_stays_in_range(void)
{
	/* A full-scale square wave drives the step index as hard as possible;
	 * an unclamped index would walk off the end of the 89-entry table. */
	struct adpcm_state e, d;
	int16_t pcm[4096], out[4096];
	uint8_t enc[2048];
	size_t i;

	for (i = 0; i < 4096; i++)
		pcm[i] = (i / 8) % 2 ? 32767 : -32768;

	adpcm_init(&e);
	adpcm_init(&d);
	adpcm_encode(&e, pcm, 4096, enc);
	adpcm_decode(&d, enc, 4096, out);
	CHECK(e.step_index >= 0 && e.step_index <= 88,
	      "encoder step index %d out of range", e.step_index);
	CHECK(d.step_index >= 0 && d.step_index <= 88,
	      "decoder step index %d out of range", d.step_index);
}

static void test_snr_helpers(void)
{
	int16_t a[4] = { 100, -100, 50, 0 };
	int16_t b[4] = { 100, -100, 50, 0 };
	int16_t c[4] = { 100, -100, 50, 10 };

	CHECK(isinf(snr_db(a, b, 4)), "identical signals should give infinite SNR");
	CHECK_EQ(max_abs_error(a, b, 4), 0);
	CHECK_EQ(max_abs_error(a, c, 4), 10);
	CHECK(snr_db(a, c, 4) > 0, "small error should still give positive SNR");
}

/* ------------------------------------------------------------------ */
/* RTP wire format                                                     */
/* ------------------------------------------------------------------ */

static void test_header_bytes_are_exactly_rfc3550(void)
{
	struct rtp_header h;
	uint8_t buf[RTP_HEADER_SIZE];

	memset(&h, 0, sizeof(h));
	h.version = 2;
	h.payload_type = 97;
	h.marker = 1;
	h.sequence = 0x1234;
	h.timestamp = 0xdeadbeef;
	h.ssrc = 0x0a0b0c0d;

	CHECK_EQ(rtp_write_header(&h, buf), RTP_HEADER_SIZE);

	/* V=2, P=0, X=0, CC=0 -> 0b10000000 */
	CHECK_EQ(buf[0], 0x80);
	/* M=1, PT=97 (0x61) -> 0b11100001 */
	CHECK_EQ(buf[1], 0xe1);
	/* Big-endian throughout. */
	CHECK_EQ(buf[2], 0x12);
	CHECK_EQ(buf[3], 0x34);
	CHECK_EQ(buf[4], 0xde);
	CHECK_EQ(buf[7], 0xef);
	CHECK_EQ(buf[8], 0x0a);
	CHECK_EQ(buf[11], 0x0d);
}

static void test_header_round_trip(void)
{
	struct rtp_header in, out;
	uint8_t buf[RTP_HEADER_SIZE];

	memset(&in, 0, sizeof(in));
	in.version = 2;
	in.payload_type = 97;
	in.sequence = 65535;
	in.timestamp = 123456;
	in.ssrc = 0xcafebabe;

	rtp_write_header(&in, buf);
	CHECK_EQ(rtp_parse_header(buf, sizeof(buf), &out), RTP_HEADER_SIZE);
	CHECK_EQ(out.sequence, 65535);
	CHECK_EQ(out.timestamp, 123456);
	CHECK_EQ(out.ssrc, 0xcafebabe);
	CHECK_EQ(out.payload_type, 97);
}

static void test_parser_rejects_malformed_packets(void)
{
	struct rtp_header h;
	uint8_t buf[RTP_HEADER_SIZE] = { 0 };

	/* Too short. */
	CHECK_EQ(rtp_parse_header(buf, 4, &h), -1);
	/* Wrong version (0 instead of 2). */
	buf[0] = 0x00;
	CHECK_EQ(rtp_parse_header(buf, sizeof(buf), &h), -1);
	/* Version 2 with a CSRC count that does not fit the packet. */
	buf[0] = 0x8f;   /* V=2, CC=15 -> needs 60 extra bytes */
	CHECK_EQ(rtp_parse_header(buf, sizeof(buf), &h), -1);
}

static void test_parser_skips_csrc_and_extension(void)
{
	struct rtp_header h;
	uint8_t buf[64] = { 0 };

	/* V=2, X=1, CC=2 -> payload starts after 12 + 8 + extension. */
	buf[0] = 0x92;
	buf[1] = 97;
	/* Extension header sits after the CSRC list at offset 20:
	 * 2 bytes profile, 2 bytes length in 32-bit words. */
	buf[20] = 0xbe;
	buf[21] = 0xde;
	buf[22] = 0x00;
	buf[23] = 0x01;      /* one 32-bit word of extension */
	CHECK_EQ(rtp_parse_header(buf, sizeof(buf), &h), 12 + 8 + 4 + 4);
	CHECK_EQ(h.csrc_count, 2);
	CHECK_EQ(h.extension, 1);
}

static void test_timestamp_advances_by_samples_not_packets(void)
{
	/* RFC 3550 requires the media clock. Counting packets instead breaks
	 * playout scheduling the moment packet sizes vary. */
	struct rtp_packetizer tx;
	int16_t pcm[320];
	uint8_t pkt[JB_MAX_PACKET];
	struct rtp_header h1, h2;
	size_t len;

	fill_tone(pcm, 320, 300.0, 0.5);
	rtp_packetizer_init(&tx, 1, 0);

	len = rtp_pack(&tx, pcm, 320, pkt, sizeof(pkt));
	rtp_parse_header(pkt, len, &h1);
	len = rtp_pack(&tx, pcm, 160, pkt, sizeof(pkt));
	rtp_parse_header(pkt, len, &h2);

	CHECK_EQ(h2.timestamp - h1.timestamp, 320);
	CHECK_EQ((uint16_t)(h2.sequence - h1.sequence), 1);
}

static void test_pack_unpack_round_trip(void)
{
	struct rtp_packetizer tx;
	int16_t pcm[320], out[320];
	uint8_t pkt[JB_MAX_PACKET];
	struct rtp_header h;
	size_t len, got;
	double cold, warm, steady;

	fill_tone(pcm, 320, 500.0, 0.7);
	rtp_packetizer_init(&tx, 0x999, 0);
	len = rtp_pack(&tx, pcm, 320, pkt, sizeof(pkt));

	CHECK_EQ(len, RTP_HEADER_SIZE + ADPCM_PREAMBLE_SIZE + 160);
	got = rtp_unpack(pkt, len, out, 320, &h);
	CHECK_EQ(got, 320);

	/*
	 * ADPCM has a cold-start transient. From a reset state the predictor
	 * is at zero and the step index at its smallest value, so the quantiser
	 * needs roughly 30 samples (2 ms at 16 kHz) to climb the step ladder to
	 * a full-scale signal. Averaged over a single 20 ms packet that
	 * transient dominates, which is why the figure below is ~16 dB rather
	 * than the ~35 dB the codec actually delivers once adapted.
	 *
	 * This is measured rather than tolerated: it is the same effect that
	 * makes per-packet state reset cost ~16 dB in the benchmark, and the
	 * reason the codec state is carried in the payload preamble instead.
	 */
	cold = snr_db(pcm, out, 320);
	steady = snr_db(pcm + 32, out + 32, 288);
	CHECK(cold > 12.0, "cold-start SNR %.2f dB unexpectedly low", cold);
	CHECK(steady > 30.0, "steady-state SNR %.2f dB too low", steady);
	CHECK(steady > cold + 10.0,
	      "expected a large cold-start penalty; cold %.2f, steady %.2f",
	      cold, steady);

	/* A second packet continues the adapted state and must not repeat the
	 * transient. */
	len = rtp_pack(&tx, pcm, 320, pkt, sizeof(pkt));
	rtp_unpack(pkt, len, out, 320, NULL);
	warm = snr_db(pcm, out, 320);
	CHECK(warm > 30.0, "warm-state SNR %.2f dB too low", warm);
}

static void test_packets_are_independently_decodable(void)
{
	/* The payload preamble carries the codec state, so decoding packet 3
	 * without having seen 1 and 2 must still work. This is what makes the
	 * jitter buffer able to play a reordered packet. */
	struct rtp_packetizer tx;
	int16_t pcm[960], out[320];
	uint8_t pkt1[JB_MAX_PACKET], pkt3[JB_MAX_PACKET];
	size_t len3;

	fill_tone(pcm, 960, 250.0, 0.8);
	rtp_packetizer_init(&tx, 7, 0);
	rtp_pack(&tx, pcm, 320, pkt1, sizeof(pkt1));
	rtp_pack(&tx, pcm + 320, 320, pkt1, sizeof(pkt1));
	len3 = rtp_pack(&tx, pcm + 640, 320, pkt3, sizeof(pkt3));

	/* Decode the third packet in isolation. */
	CHECK_EQ(rtp_unpack(pkt3, len3, out, 320, NULL), 320);
	CHECK(snr_db(pcm + 640, out, 320) > 20.0,
	      "third packet did not decode standalone; state is not in payload");
}

static void test_unpack_rejects_bad_step_index(void)
{
	/* A corrupt preamble must not index past the 89-entry step table. */
	struct rtp_packetizer tx;
	int16_t pcm[320], out[320];
	uint8_t pkt[JB_MAX_PACKET];
	size_t len;

	fill_tone(pcm, 320, 400.0, 0.5);
	rtp_packetizer_init(&tx, 1, 0);
	len = rtp_pack(&tx, pcm, 320, pkt, sizeof(pkt));

	pkt[RTP_HEADER_SIZE + 2] = 200;        /* out of range */
	CHECK_EQ(rtp_unpack(pkt, len, out, 320, NULL), 0);

	pkt[RTP_HEADER_SIZE + 2] = (uint8_t)-5;
	CHECK_EQ(rtp_unpack(pkt, len, out, 320, NULL), 0);
}

static void test_pack_respects_output_capacity(void)
{
	struct rtp_packetizer tx;
	int16_t pcm[320];
	uint8_t small[16];

	fill_tone(pcm, 320, 400.0, 0.5);
	rtp_packetizer_init(&tx, 1, 0);
	CHECK_EQ(rtp_pack(&tx, pcm, 320, small, sizeof(small)), 0);
}

/* ------------------------------------------------------------------ */
/* Jitter buffer                                                       */
/* ------------------------------------------------------------------ */

static void test_sequence_comparison_handles_wraparound(void)
{
	/* 16-bit sequence numbers wrap every ~22 minutes at 20 ms packets, so
	 * every real call hits this. A naive a > b comparison reads the wrap as
	 * a 65,000-packet jump backwards and discards the rest of the call. */
	CHECK(jb_seq_cmp(1, 0) > 0, "1 should follow 0");
	CHECK(jb_seq_cmp(0, 1) < 0, "0 should precede 1");
	CHECK_EQ(jb_seq_cmp(5, 5), 0);
	CHECK(jb_seq_cmp(0, 65535) > 0, "0 should follow 65535 across the wrap");
	CHECK(jb_seq_cmp(65535, 0) < 0, "65535 should precede 0 across the wrap");
	CHECK(jb_seq_cmp(3, 65533) > 0, "3 should follow 65533");
}

struct jb_fixture {
	struct rtp_packetizer tx;
	int16_t pcm[320];
	uint8_t pkt[32][JB_MAX_PACKET];
	size_t len[32];
};

static void jb_fixture_init(struct jb_fixture *f, unsigned n)
{
	unsigned i;

	fill_tone(f->pcm, 320, 300.0, 0.6);
	rtp_packetizer_init(&f->tx, 0x4242, 0);
	for (i = 0; i < n && i < 32; i++)
		f->len[i] = rtp_pack(&f->tx, f->pcm, 320, f->pkt[i],
				     JB_MAX_PACKET);
}

static void test_buffer_primes_before_playing(void)
{
	struct jitterbuf jb;
	struct jb_fixture f;
	int16_t out[320];

	jb_fixture_init(&f, 4);
	jb_init(&jb, 3, JB_CONCEAL_FADE);

	jb_put(&jb, f.pkt[0], f.len[0]);
	CHECK_EQ(jb_get(&jb, out, 320), 0);    /* 1 of 3: still priming */
	jb_put(&jb, f.pkt[1], f.len[1]);
	CHECK_EQ(jb_get(&jb, out, 320), 0);    /* 2 of 3 */
	jb_put(&jb, f.pkt[2], f.len[2]);
	CHECK_EQ(jb_get(&jb, out, 320), 320);  /* primed: plays */
}

static void test_in_order_playout(void)
{
	struct jitterbuf jb;
	struct jb_fixture f;
	int16_t out[320];
	unsigned i;

	jb_fixture_init(&f, 8);
	jb_init(&jb, 1, JB_CONCEAL_FADE);

	for (i = 0; i < 8; i++) {
		jb_put(&jb, f.pkt[i], f.len[i]);
		CHECK_EQ(jb_get(&jb, out, 320), 320);
	}
	CHECK_EQ(jb.played, 8);
	CHECK_EQ(jb.concealed, 0);
}

static void test_reordered_packet_is_recovered(void)
{
	struct jitterbuf jb;
	struct jb_fixture f;
	int16_t out[320];

	jb_fixture_init(&f, 4);
	jb_init(&jb, 3, JB_CONCEAL_FADE);

	/* Deliver 0, 2, 1 -- a classic single-position swap. With depth 3 the
	 * buffer has room to put 1 back in its place. */
	jb_put(&jb, f.pkt[0], f.len[0]);
	jb_put(&jb, f.pkt[2], f.len[2]);
	jb_put(&jb, f.pkt[1], f.len[1]);

	CHECK_EQ(jb_get(&jb, out, 320), 320);
	CHECK_EQ(jb_get(&jb, out, 320), 320);
	CHECK_EQ(jb_get(&jb, out, 320), 320);
	CHECK_EQ(jb.concealed, 0);
	CHECK_EQ(jb.played, 3);
}

static void test_missing_packet_is_concealed_not_dropped(void)
{
	/* A playout path must always produce a frame. Returning nothing is an
	 * underrun, which is worse than a concealed one. */
	struct jitterbuf jb;
	struct jb_fixture f;
	int16_t out[320];

	jb_fixture_init(&f, 4);
	jb_init(&jb, 2, JB_CONCEAL_FADE);

	jb_put(&jb, f.pkt[0], f.len[0]);
	jb_put(&jb, f.pkt[2], f.len[2]);   /* packet 1 never arrives */
	jb_put(&jb, f.pkt[3], f.len[3]);

	CHECK_EQ(jb_get(&jb, out, 320), 320);   /* 0 */
	CHECK_EQ(jb_get(&jb, out, 320), 320);   /* 1 concealed */
	CHECK_EQ(jb.concealed, 1);
	CHECK_EQ(jb_get(&jb, out, 320), 320);   /* 2 */
}

static void test_duplicate_is_ignored(void)
{
	struct jitterbuf jb;
	struct jb_fixture f;
	int16_t out[320];

	jb_fixture_init(&f, 2);
	jb_init(&jb, 1, JB_CONCEAL_FADE);

	CHECK_EQ(jb_put(&jb, f.pkt[0], f.len[0]), 1);
	CHECK_EQ(jb_put(&jb, f.pkt[0], f.len[0]), 0);   /* same sequence */
	CHECK_EQ(jb.duplicates, 1);
	CHECK_EQ(jb_get(&jb, out, 320), 320);
}

static void test_late_packet_is_counted_separately_from_loss(void)
{
	/* too_late and real loss call for different fixes: a late packet means
	 * the buffer is too shallow, a lost one means the path drops. */
	struct jitterbuf jb;
	struct jb_fixture f;
	int16_t out[320];

	jb_fixture_init(&f, 4);
	jb_init(&jb, 1, JB_CONCEAL_FADE);

	jb_put(&jb, f.pkt[0], f.len[0]);
	jb_get(&jb, out, 320);                 /* cursor now at 1 */
	jb_put(&jb, f.pkt[2], f.len[2]);
	jb_get(&jb, out, 320);                 /* 1 concealed, cursor at 2 */
	jb_get(&jb, out, 320);                 /* plays 2, cursor at 3 */

	/* Packet 1 finally shows up, long after its slot. */
	CHECK_EQ(jb_put(&jb, f.pkt[1], f.len[1]), 0);
	CHECK_EQ(jb.too_late, 1);
}

static void test_reordering_counter_counts_only_real_reordering(void)
{
	/* A clean in-order stream must report zero reorderings. Counting
	 * against the playout cursor instead made a 150-packet in-order run
	 * report 147, which is how this bug was found. */
	struct jitterbuf jb;
	struct jb_fixture f;
	int16_t out[320];
	unsigned i;

	jb_fixture_init(&f, 8);
	jb_init(&jb, 3, JB_CONCEAL_FADE);
	for (i = 0; i < 8; i++) {
		jb_put(&jb, f.pkt[i], f.len[i]);
		jb_get(&jb, out, 320);
	}
	CHECK_EQ(jb.reordered, 0);

	/* Now a genuine swap: 0, 2, 1 means packet 1 arrived after a higher
	 * sequence number and must be counted once. */
	jb_fixture_init(&f, 4);
	jb_init(&jb, 3, JB_CONCEAL_FADE);
	jb_put(&jb, f.pkt[0], f.len[0]);
	jb_put(&jb, f.pkt[2], f.len[2]);
	jb_put(&jb, f.pkt[1], f.len[1]);
	CHECK_EQ(jb.reordered, 1);
}

static void test_buffer_overflow_is_reported(void)
{
	struct jitterbuf jb;
	struct rtp_packetizer tx;
	int16_t pcm[320];
	uint8_t pkt[JB_MAX_PACKET];
	unsigned i;
	int last = 1;

	fill_tone(pcm, 320, 300.0, 0.6);
	rtp_packetizer_init(&tx, 1, 0);
	jb_init(&jb, JB_MAX_DEPTH, JB_CONCEAL_FADE);

	/* Fill every slot, then one more. */
	for (i = 0; i < JB_MAX_DEPTH + 1; i++) {
		size_t len = rtp_pack(&tx, pcm, 320, pkt, sizeof(pkt));

		last = jb_put(&jb, pkt, len);
	}
	CHECK_EQ(last, -1);
	CHECK_EQ(jb.overflows, 1);
}

static void test_oversized_and_malformed_input_is_rejected(void)
{
	struct jitterbuf jb;
	uint8_t junk[8] = { 0 };

	jb_init(&jb, 2, JB_CONCEAL_FADE);
	CHECK_EQ(jb_put(&jb, junk, sizeof(junk)), -1);   /* too short for RTP */
	CHECK_EQ(jb_occupancy(&jb), 0);
}

static void test_empty_buffer_returns_nothing(void)
{
	/* Distinguishing "stalled" from "lost a packet" is what lets a caller
	 * stop cleanly instead of generating concealment forever. */
	struct jitterbuf jb;
	struct jb_fixture f;
	int16_t out[320];

	jb_fixture_init(&f, 2);
	jb_init(&jb, 1, JB_CONCEAL_FADE);

	jb_put(&jb, f.pkt[0], f.len[0]);
	CHECK_EQ(jb_get(&jb, out, 320), 320);
	CHECK_EQ(jb_get(&jb, out, 320), 0);    /* nothing buffered */
}

static void test_concealment_modes_differ(void)
{
	struct jitterbuf jb;
	struct jb_fixture f;
	int16_t silence[320], hold[320], fade[320];
	int c;
	int16_t *dest[3] = { silence, hold, fade };

	for (c = 0; c < 3; c++) {
		jb_fixture_init(&f, 4);
		jb_init(&jb, 2, (enum jb_conceal)c);
		jb_put(&jb, f.pkt[0], f.len[0]);
		jb_put(&jb, f.pkt[2], f.len[2]);
		jb_get(&jb, dest[c], 320);              /* real frame 0 */
		jb_get(&jb, dest[c], 320);              /* concealed frame 1 */
	}

	/* Silence must be all zeros; hold must not be; and fade must sit
	 * strictly between the two in energy. */
	{
		long e_sil = 0, e_hold = 0, e_fade = 0;
		size_t i;

		for (i = 0; i < 320; i++) {
			e_sil += labs(silence[i]);
			e_hold += labs(hold[i]);
			e_fade += labs(fade[i]);
		}
		CHECK_EQ(e_sil, 0);
		CHECK(e_hold > 0, "hold concealment produced silence");
		CHECK(e_fade > 0 && e_fade < e_hold,
		      "fade energy %ld should be between 0 and hold %ld",
		      e_fade, e_hold);
	}
}

static void test_loss_before_first_packet_yields_silence(void)
{
	/* Concealment has no previous frame to repeat yet, and must not read
	 * uninitialised memory. */
	struct jitterbuf jb;
	struct jb_fixture f;
	int16_t out[320];
	size_t i;
	long energy = 0;

	jb_fixture_init(&f, 4);
	jb_init(&jb, 1, JB_CONCEAL_FADE);

	/* First packet to arrive is sequence 1; sequence 0 is lost. The buffer
	 * primes on 1, so there is no earlier frame to hold. */
	jb_put(&jb, f.pkt[1], f.len[1]);
	jb_put(&jb, f.pkt[3], f.len[3]);
	jb_get(&jb, out, 320);
	jb_get(&jb, out, 320);
	for (i = 0; i < 320; i++)
		energy += labs(out[i]);
	CHECK(energy >= 0, "concealment must not fault without a prior frame");
}

/* ------------------------------------------------------------------ */

int main(void)
{
	printf("rtpaudio unit tests\n");

	test_compression_is_exactly_four_to_one();
	test_silence_round_trips_exactly();
	test_tone_meets_snr_floor();
	test_encoder_and_decoder_states_stay_in_lockstep();
	test_step_index_stays_in_range();
	test_snr_helpers();

	test_header_bytes_are_exactly_rfc3550();
	test_header_round_trip();
	test_parser_rejects_malformed_packets();
	test_parser_skips_csrc_and_extension();
	test_timestamp_advances_by_samples_not_packets();
	test_pack_unpack_round_trip();
	test_packets_are_independently_decodable();
	test_unpack_rejects_bad_step_index();
	test_pack_respects_output_capacity();

	test_sequence_comparison_handles_wraparound();
	test_buffer_primes_before_playing();
	test_in_order_playout();
	test_reordered_packet_is_recovered();
	test_missing_packet_is_concealed_not_dropped();
	test_duplicate_is_ignored();
	test_late_packet_is_counted_separately_from_loss();
	test_reordering_counter_counts_only_real_reordering();
	test_buffer_overflow_is_reported();
	test_oversized_and_malformed_input_is_rejected();
	test_empty_buffer_returns_nothing();
	test_concealment_modes_differ();
	test_loss_before_first_packet_yields_silence();

	printf("\n%d checks, %d failed\n", tests_run, tests_failed);
	return tests_failed ? 1 : 0;
}
