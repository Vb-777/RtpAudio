/* SPDX-License-Identifier: MIT */
/*
 * bench_e2e.c - End-to-end measurement of the codec + RTP + jitter buffer path.
 *
 * Four experiments, each answering a design question the code had to decide:
 *
 *   1. Codec quality vs. packet size, with and without per-packet state reset.
 *      Quantifies what packet-independence costs in SNR, which is the trade
 *      documented in rtp.h.
 *
 *   2. Quality vs. packet loss, for each concealment strategy. Shows whether
 *      the fade is worth implementing over a plain hold or silence.
 *
 *   3. Reordering tolerance vs. buffer depth. Shows the depth needed to absorb
 *      a given reorder distance, which is the latency/dropout curve.
 *
 *   4. Throughput and per-packet latency over a real UDP loopback socket,
 *      reported as percentiles -- the tail is what decides whether a real-time
 *      path underruns.
 */

#define _GNU_SOURCE
#include <arpa/inet.h>
#include <math.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "../src/adpcm.h"
#include "../src/jitterbuf.h"
#include "../src/rtp.h"

#define SAMPLE_RATE 16000
#define TOTAL_SAMPLES (SAMPLE_RATE * 8)   /* 8 seconds of audio */

static int16_t source[TOTAL_SAMPLES];
static int16_t output[TOTAL_SAMPLES * 2];

static double now_sec(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static uint64_t now_ns(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000000ull + ts.tv_nsec;
}

static int cmp_u64(const void *a, const void *b)
{
	uint64_t x = *(const uint64_t *)a, y = *(const uint64_t *)b;

	return (x > y) - (x < y);
}

/* A speech-like test signal: a low fundamental with an amplitude envelope and
 * a formant-ish overtone. More representative than a pure tone, which flatters
 * a predictive codec, and than white noise, which punishes it unrealistically. */
static void build_source(void)
{
	int i;

	for (i = 0; i < TOTAL_SAMPLES; i++) {
		double t = (double)i / SAMPLE_RATE;
		double envelope = 0.5 + 0.5 * sin(2 * M_PI * 2.5 * t);
		double v = 0.62 * sin(2 * M_PI * 185 * t) * envelope +
			   0.16 * sin(2 * M_PI * 1480 * t) * envelope +
			   0.06 * sin(2 * M_PI * 2600 * t);

		source[i] = (int16_t)(v * 30000);
	}
}

/*
 * Worst and mean sample-to-sample jump, in LSBs.
 *
 * SNR cannot see a click. Concealment that inserts a loud-but-wrong signal
 * scores worse on squared error than inserting silence, yet silence is what
 * actually clicks, because the waveform steps discontinuously to zero and back.
 * This metric measures that step directly, so the two effects can be reported
 * separately instead of one hiding the other.
 */
static void discontinuity(const int16_t *x, size_t n, int *worst, double *mean)
{
	size_t i;
	double sum = 0.0;

	*worst = 0;
	for (i = 1; i < n; i++) {
		int d = x[i] - x[i - 1];

		if (d < 0)
			d = -d;
		if (d > *worst)
			*worst = d;
		sum += d;
	}
	*mean = n > 1 ? sum / (double)(n - 1) : 0.0;
}

/* Deterministic PRNG so every reported figure is reproducible. */
static uint32_t rng_state = 2463534242u;

static uint32_t rng_next(void)
{
	rng_state ^= rng_state << 13;
	rng_state ^= rng_state >> 17;
	rng_state ^= rng_state << 5;
	return rng_state;
}

static double rng_unit(void)
{
	return (double)(rng_next() >> 8) / 16777216.0;
}

/* ------------------------------------------------------------------ */
/* 1. Codec quality vs packet size                                     */
/* ------------------------------------------------------------------ */

static void exp_packet_size(void)
{
	const size_t sizes[] = { 80, 160, 320, 640 };  /* 5/10/20/40 ms @16 kHz */
	size_t i;

	printf("1. Quality vs packet size (no loss)\n");
	printf("   %8s %8s %12s %12s %10s\n", "samples", "ms",
	       "SNR stateful", "SNR reset", "overhead");

	for (i = 0; i < sizeof(sizes) / sizeof(sizes[0]); i++) {
		size_t frame = sizes[i];
		double snr[2];
		int mode;
		size_t pkt_len = 0;

		for (mode = 0; mode < 2; mode++) {
			struct rtp_packetizer tx;
			uint8_t pkt[JB_MAX_PACKET];
			size_t off = 0, out_n = 0;

			rtp_packetizer_init(&tx, 0x1234abcd, mode);
			while (off + frame <= TOTAL_SAMPLES) {
				size_t len = rtp_pack(&tx, source + off, frame,
						      pkt, sizeof(pkt));
				size_t got = rtp_unpack(pkt, len,
							output + out_n,
							frame, NULL);
				pkt_len = len;
				out_n += got;
				off += frame;
			}
			snr[mode] = snr_db(source, output, out_n);
		}
		printf("   %8zu %8.1f %12.2f %12.2f %9.1f%%\n",
		       frame, 1000.0 * frame / SAMPLE_RATE, snr[0], snr[1],
		       100.0 * (RTP_HEADER_SIZE + ADPCM_PREAMBLE_SIZE) /
			       (double)pkt_len);
	}
}

/* ------------------------------------------------------------------ */
/* 2. Quality vs loss, per concealment strategy                        */
/* ------------------------------------------------------------------ */

static double run_loss(double loss_rate, enum jb_conceal conceal,
		       unsigned depth, unsigned *concealed_out,
		       int *click_out, double *mean_jump_out)
{
	struct rtp_packetizer tx;
	struct jitterbuf jb;
	const size_t frame = 320;             /* 20 ms */
	uint8_t pkt[JB_MAX_PACKET];
	size_t off = 0, out_n = 0;

	rtp_packetizer_init(&tx, 0xfeed0001, 0);
	jb_init(&jb, depth, conceal);
	rng_state = 2463534242u;              /* same loss pattern every run */

	/*
	 * Playout is driven by the clock, not by arrivals: exactly one frame is
	 * pulled per packet interval, as a sound card would. This is essential
	 * for the measurement to mean anything -- draining only when a packet
	 * arrives means a loss that empties the buffer yields no frame at all,
	 * the output becomes shorter than the input, and the SNR that follows is
	 * measuring misalignment rather than audio quality. That mistake made
	 * an earlier version of this benchmark report -3.5 dB at 20% loss.
	 */
	while (off + frame <= TOTAL_SAMPLES) {
		size_t len = rtp_pack(&tx, source + off, frame, pkt,
				      sizeof(pkt));

		if (rng_unit() >= loss_rate)
			jb_put(&jb, pkt, len);

		if (out_n + frame <= TOTAL_SAMPLES * 2) {
			size_t got = jb_get(&jb, output + out_n, frame);

			/* While priming, the buffer legitimately produces
			 * nothing; pad with silence so the output stays aligned
			 * to the source timeline. */
			if (got == 0) {
				memset(output + out_n, 0,
				       frame * sizeof(int16_t));
				got = frame;
			}
			out_n += got;
		}
		off += frame;
	}

	if (concealed_out)
		*concealed_out = jb.concealed;
	if (click_out) {
		double mean_jump;
		size_t measure = out_n < TOTAL_SAMPLES ? out_n : TOTAL_SAMPLES;

		discontinuity(output, measure, click_out, &mean_jump);
		if (mean_jump_out)
			*mean_jump_out = mean_jump;
	}

	/*
	 * Align before measuring. Priming emits a few frames of silence, so the
	 * output lags the source by a whole number of frames that depends on the
	 * depth and on exactly when playout started. Rather than hard-code that
	 * offset -- an earlier version used `depth` frames and was off by one,
	 * which reported -4 dB even at zero loss -- the lag is found by searching
	 * a small window and taking the best alignment. Aligning before computing
	 * SNR is standard practice; without it the metric reports delay as
	 * distortion.
	 */
	{
		double best = -INFINITY;
		size_t lag_frames;

		for (lag_frames = 0; lag_frames <= depth + 2; lag_frames++) {
			size_t lag = lag_frames * frame;
			size_t n;
			double snr;

			if (out_n <= lag)
				break;
			n = out_n - lag;
			if (n > TOTAL_SAMPLES)
				n = TOTAL_SAMPLES;
			snr = snr_db(source, output + lag, n);
			if (snr > best)
				best = snr;
		}
		return best;
	}
}

static void exp_loss(void)
{
	const double rates[] = { 0.0, 0.01, 0.03, 0.05, 0.10, 0.20 };
	size_t i;
	int c;

	printf("\n2. Quality vs packet loss (20 ms packets, depth 3)\n");
	printf("   SNR is a waveform metric and cannot see a click; peak jump is\n");
	printf("   the sample-to-sample discontinuity that is actually audible.\n");
	printf("   %6s  %-26s %-26s %-26s %s\n", "loss",
	       "silence (SNR/peak jump)", "hold (SNR/peak jump)",
	       "fade (SNR/peak jump)", "concealed");

	for (i = 0; i < sizeof(rates) / sizeof(rates[0]); i++) {
		double snr[3], mean_jump[3];
		int click[3];
		unsigned concealed = 0;

		for (c = 0; c < 3; c++)
			snr[c] = run_loss(rates[i], (enum jb_conceal)c, 3,
					  &concealed, &click[c], &mean_jump[c]);

		printf("   %5.0f%%  %10.2f dB /%7d  %10.2f dB /%7d  %10.2f dB /%7d  %6u\n",
		       rates[i] * 100,
		       snr[0], click[0], snr[1], click[1], snr[2], click[2],
		       concealed);
	}

	/* Baseline: the undamaged signal's own peak jump, so the numbers above
	 * can be read against what the source itself contains. */
	{
		int click;
		double mean_jump;

		discontinuity(source, TOTAL_SAMPLES, &click, &mean_jump);
		printf("   source signal's own peak sample-to-sample jump: %d\n",
		       click);
	}
}

/* ------------------------------------------------------------------ */
/* 3. Reordering tolerance vs depth                                    */
/* ------------------------------------------------------------------ */

static void exp_reorder(void)
{
	const unsigned depths[] = { 1, 2, 3, 5, 8 };
	const unsigned distances[] = { 1, 2, 4 };
	size_t d, r;

	printf("\n3. Reordering tolerance: every 7th packet delayed by N positions\n");
	printf("   Playout pulls exactly one frame per packet interval, as a sound\n");
	printf("   card would, so buffer depth genuinely buys time for late arrivals.\n");
	printf("   %6s", "depth");
	for (r = 0; r < sizeof(distances) / sizeof(distances[0]); r++)
		printf("   dist=%u late/concealed", distances[r]);
	printf("\n");

	for (d = 0; d < sizeof(depths) / sizeof(depths[0]); d++) {
		printf("   %6u", depths[d]);
		for (r = 0; r < sizeof(distances) / sizeof(distances[0]); r++) {
			struct rtp_packetizer tx;
			struct jitterbuf jb;
			const size_t frame = 320;
			uint8_t pkt[JB_MAX_PACKET];
			uint8_t held[8][JB_MAX_PACKET];
			size_t held_len[8];
			unsigned held_age[8];
			unsigned held_n = 0;
			size_t off = 0, out_n = 0;
			unsigned counter = 0;

			rtp_packetizer_init(&tx, 0xabcd, 0);
			jb_init(&jb, depths[d], JB_CONCEAL_FADE);

			while (off + frame <= TOTAL_SAMPLES) {
				size_t len = rtp_pack(&tx, source + off, frame,
						      pkt, sizeof(pkt));
				unsigned k, w = 0;

				/* Release any held packet whose delay has
				 * elapsed, before offering the current one. */
				for (k = 0; k < held_n; k++) {
					held_age[k]++;
					if (held_age[k] >= distances[r]) {
						jb_put(&jb, held[k],
						       held_len[k]);
					} else {
						if (w != k) {
							memcpy(held[w], held[k],
							       held_len[k]);
							held_len[w] = held_len[k];
							held_age[w] = held_age[k];
						}
						w++;
					}
				}
				held_n = w;

				if (counter % 7 == 3 && held_n < 8) {
					memcpy(held[held_n], pkt, len);
					held_len[held_n] = len;
					held_age[held_n] = 0;
					held_n++;
				} else {
					jb_put(&jb, pkt, len);
				}
				counter++;

				/* Exactly one frame per interval. */
				if (out_n + frame <= TOTAL_SAMPLES * 2)
					out_n += jb_get(&jb, output + out_n,
							frame);
				off += frame;
			}
			printf("   %8u/%-10u", jb.too_late, jb.concealed);
		}
		printf("\n");
	}
}

/* ------------------------------------------------------------------ */
/* 4. UDP loopback throughput and latency                              */
/* ------------------------------------------------------------------ */

static void exp_udp(void)
{
	const size_t frame = 320;
	int tx_fd, rx_fd;
	struct sockaddr_in addr;
	socklen_t addr_len = sizeof(addr);
	struct rtp_packetizer tx;
	struct jitterbuf jb;
	uint8_t pkt[JB_MAX_PACKET];
	uint64_t *lat;
	size_t off = 0, out_n = 0, n_packets = 0;
	double t0, dt;
	int rcvbuf = 1 << 20;

	rx_fd = socket(AF_INET, SOCK_DGRAM, 0);
	tx_fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (rx_fd < 0 || tx_fd < 0) {
		printf("\n4. UDP loopback: socket creation failed, skipped\n");
		return;
	}
	setsockopt(rx_fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	addr.sin_port = 0;                    /* kernel picks a free port */
	if (bind(rx_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0 ||
	    getsockname(rx_fd, (struct sockaddr *)&addr, &addr_len) != 0) {
		printf("\n4. UDP loopback: bind failed, skipped\n");
		close(rx_fd);
		close(tx_fd);
		return;
	}

	lat = malloc((TOTAL_SAMPLES / frame + 2) * sizeof(uint64_t));
	rtp_packetizer_init(&tx, 0x5555, 0);
	jb_init(&jb, 3, JB_CONCEAL_FADE);

	t0 = now_sec();
	while (off + frame <= TOTAL_SAMPLES) {
		size_t len = rtp_pack(&tx, source + off, frame, pkt,
				      sizeof(pkt));
		uint8_t rxbuf[JB_MAX_PACKET];
		uint64_t send_ns = now_ns();
		ssize_t got;

		sendto(tx_fd, pkt, len, 0, (struct sockaddr *)&addr,
		       sizeof(addr));
		got = recv(rx_fd, rxbuf, sizeof(rxbuf), 0);
		if (got > 0) {
			lat[n_packets++] = now_ns() - send_ns;
			jb_put(&jb, rxbuf, (size_t)got);
		}

		for (;;) {
			size_t n = jb_get(&jb, output + out_n, frame);

			if (n == 0)
				break;
			out_n += n;
			if (out_n + frame > TOTAL_SAMPLES * 2)
				break;
			if (jb_occupancy(&jb) >= 3)
				continue;
			break;
		}
		off += frame;
	}
	dt = now_sec() - t0;

	qsort(lat, n_packets, sizeof(uint64_t), cmp_u64);
	printf("\n4. UDP loopback (20 ms packets, depth 3)\n");
	printf("   %zu packets, %.0f pkt/s, %.2f Mbit/s payload\n",
	       n_packets, n_packets / dt,
	       8.0 * n_packets * (RTP_HEADER_SIZE + ADPCM_PREAMBLE_SIZE +
				  adpcm_encoded_bytes(frame)) / dt / 1e6);
	printf("   send-to-receive latency: p50 %.1f us, p99 %.1f us, p99.9 %.1f us\n",
	       lat[n_packets / 2] / 1000.0,
	       lat[(size_t)(n_packets * 0.99)] / 1000.0,
	       lat[(size_t)(n_packets * 0.999)] / 1000.0);
	printf("   SNR end to end: %.2f dB  (concealed %u, too late %u, dup %u)\n",
	       snr_db(source, output,
		      out_n < TOTAL_SAMPLES ? out_n : TOTAL_SAMPLES),
	       jb.concealed, jb.too_late, jb.duplicates);

	free(lat);
	close(rx_fd);
	close(tx_fd);
}

/* ------------------------------------------------------------------ */

int main(void)
{
	build_source();
	printf("rtpaudio end-to-end benchmark (%d Hz, %.1f s source)\n\n",
	       SAMPLE_RATE, (double)TOTAL_SAMPLES / SAMPLE_RATE);
	exp_packet_size();
	exp_loss();
	exp_reorder();
	exp_udp();
	return 0;
}
