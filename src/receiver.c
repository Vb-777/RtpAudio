/* SPDX-License-Identifier: MIT */
/*
 * receiver.c - Receive RTP/UDP ADPCM, run it through the jitter buffer and
 *              write raw PCM to a file or stdout.
 *
 *   ./rtp_recv 5004 out.raw [--depth 3] [--conceal fade|hold|silence]
 *                           [--timeout-ms 2000] [--loss 0.05]
 *
 * --loss discards a fraction of received packets, which is how the loss
 * behaviour is exercised without needing tc/netem privileges.
 */

#define _GNU_SOURCE
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "jitterbuf.h"
#include "rtp.h"

static uint32_t rng = 88172645463325252u & 0xffffffff;

static double rng_unit(void)
{
	rng ^= rng << 13;
	rng ^= rng >> 17;
	rng ^= rng << 5;
	return (double)(rng >> 8) / 16777216.0;
}

int main(int argc, char **argv)
{
	int port, fd, i, timeout_ms = 2000;
	unsigned depth = 3;
	double loss = 0.0;
	enum jb_conceal conceal = JB_CONCEAL_FADE;
	const char *path;
	FILE *out;
	struct sockaddr_in addr;
	struct jitterbuf jb;
	int16_t pcm[JB_MAX_PACKET * 2];
	int rcvbuf = 1 << 20;

	if (argc < 3) {
		fprintf(stderr,
			"usage: %s PORT OUT.raw [--depth N]"
			" [--conceal fade|hold|silence] [--timeout-ms N]"
			" [--loss P]\n", argv[0]);
		return 2;
	}
	port = atoi(argv[1]);
	path = argv[2];

	for (i = 3; i < argc; i++) {
		if (!strcmp(argv[i], "--depth") && i + 1 < argc)
			depth = (unsigned)strtoul(argv[++i], NULL, 10);
		else if (!strcmp(argv[i], "--timeout-ms") && i + 1 < argc)
			timeout_ms = atoi(argv[++i]);
		else if (!strcmp(argv[i], "--loss") && i + 1 < argc)
			loss = strtod(argv[++i], NULL);
		else if (!strcmp(argv[i], "--conceal") && i + 1 < argc) {
			const char *m = argv[++i];

			if (!strcmp(m, "silence"))
				conceal = JB_CONCEAL_SILENCE;
			else if (!strcmp(m, "hold"))
				conceal = JB_CONCEAL_HOLD;
			else
				conceal = JB_CONCEAL_FADE;
		}
	}

	out = strcmp(path, "-") ? fopen(path, "wb") : stdout;
	if (!out) {
		perror(path);
		return 1;
	}

	fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (fd < 0) {
		perror("socket");
		return 1;
	}
	setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	addr.sin_port = htons((uint16_t)port);
	if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
		perror("bind");
		return 1;
	}

	jb_init(&jb, depth, conceal);
	fprintf(stderr, "listening on %d (depth %u)\n", port, depth);

	for (;;) {
		struct pollfd pfd = { .fd = fd, .events = POLLIN };
		uint8_t buf[JB_MAX_PACKET];
		int ready = poll(&pfd, 1, timeout_ms);
		size_t n;

		if (ready < 0)
			break;
		if (ready == 0) {
			/* Idle past the timeout: drain whatever is left and
			 * stop, rather than concealing forever. */
			while ((n = jb_get(&jb, pcm, sizeof(pcm) /
						   sizeof(pcm[0]))) > 0)
				fwrite(pcm, sizeof(int16_t), n, out);
			break;
		}

		{
			ssize_t got = recv(fd, buf, sizeof(buf), 0);
			int dropped = 0;

			if (got <= 0)
				continue;
			if (loss > 0.0 && rng_unit() < loss)
				dropped = 1;       /* simulated drop */
			else
				jb_put(&jb, buf, (size_t)got);

			/*
			 * Fall through to the drain even when the packet was
			 * dropped. Skipping it means a loss produces no output
			 * frame at all, so the written stream becomes shorter
			 * than the source and every later sample is misaligned
			 * -- which looks like catastrophic distortion when the
			 * output is compared against the input.
			 */
			(void)dropped;
		}

		while ((n = jb_get(&jb, pcm,
				   sizeof(pcm) / sizeof(pcm[0]))) > 0) {
			fwrite(pcm, sizeof(int16_t), n, out);
			if (jb_occupancy(&jb) < depth)
				break;
		}
	}

	fprintf(stderr,
		"received %u, played %u, concealed %u, reordered %u, late %u,"
		" dup %u, overflow %u\n",
		jb.received, jb.played, jb.concealed, jb.reordered,
		jb.too_late, jb.duplicates, jb.overflows);

	if (out != stdout)
		fclose(out);
	close(fd);
	return 0;
}
