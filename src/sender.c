/* SPDX-License-Identifier: MIT */
/*
 * sender.c - Stream raw 16-bit PCM from a file (or stdin) over RTP/UDP.
 *
 *   ./rtp_send 127.0.0.1 5004 audio.raw [--frame 320] [--rate 16000] [--pace]
 *
 * --pace sleeps to the media clock so the stream runs in real time, which is
 * what a live source does. Without it the file is sent as fast as the socket
 * accepts, which is what the throughput benchmark wants.
 */

#define _GNU_SOURCE
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "jitterbuf.h"
#include "rtp.h"

int main(int argc, char **argv)
{
	const char *host, *path;
	int port, fd, i, pace = 0, reset = 0;
	size_t frame = 320, rate = 16000, sent = 0;
	FILE *in;
	struct sockaddr_in dst;
	struct rtp_packetizer tx;
	int16_t *pcm;
	uint8_t pkt[JB_MAX_PACKET];
	struct timespec next;

	if (argc < 4) {
		fprintf(stderr,
			"usage: %s HOST PORT FILE.raw [--frame N] [--rate HZ]"
			" [--pace] [--reset-per-packet]\n"
			"  FILE.raw is headerless signed 16-bit little-endian mono PCM\n",
			argv[0]);
		return 2;
	}
	host = argv[1];
	port = atoi(argv[2]);
	path = argv[3];

	for (i = 4; i < argc; i++) {
		if (!strcmp(argv[i], "--frame") && i + 1 < argc)
			frame = (size_t)strtoul(argv[++i], NULL, 10);
		else if (!strcmp(argv[i], "--rate") && i + 1 < argc)
			rate = (size_t)strtoul(argv[++i], NULL, 10);
		else if (!strcmp(argv[i], "--pace"))
			pace = 1;
		else if (!strcmp(argv[i], "--reset-per-packet"))
			reset = 1;
	}
	if (frame == 0 || frame > JB_MAX_PACKET) {
		fprintf(stderr, "frame size out of range\n");
		return 2;
	}

	in = strcmp(path, "-") ? fopen(path, "rb") : stdin;
	if (!in) {
		perror(path);
		return 1;
	}

	fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (fd < 0) {
		perror("socket");
		return 1;
	}
	memset(&dst, 0, sizeof(dst));
	dst.sin_family = AF_INET;
	dst.sin_port = htons((uint16_t)port);
	if (inet_pton(AF_INET, host, &dst.sin_addr) != 1) {
		fprintf(stderr, "bad address: %s\n", host);
		return 2;
	}

	pcm = malloc(frame * sizeof(int16_t));
	rtp_packetizer_init(&tx, (uint32_t)getpid(), reset);
	clock_gettime(CLOCK_MONOTONIC, &next);

	for (;;) {
		size_t got = fread(pcm, sizeof(int16_t), frame, in);
		size_t len;

		if (got == 0)
			break;
		/* Zero-pad a short final frame so the packet is well formed. */
		if (got < frame)
			memset(pcm + got, 0, (frame - got) * sizeof(int16_t));

		len = rtp_pack(&tx, pcm, frame, pkt, sizeof(pkt));
		if (len == 0)
			break;
		if (sendto(fd, pkt, len, 0, (struct sockaddr *)&dst,
			   sizeof(dst)) < 0 && errno != ECONNREFUSED) {
			perror("sendto");
			break;
		}
		sent++;

		if (pace) {
			/* Absolute deadlines, not a fixed sleep: sleeping for
			 * the frame duration each time accumulates the
			 * processing cost as drift. */
			long ns = (long)(1000000000ull * frame / rate);

			next.tv_nsec += ns;
			while (next.tv_nsec >= 1000000000L) {
				next.tv_nsec -= 1000000000L;
				next.tv_sec++;
			}
			clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next,
					NULL);
		}
	}

	fprintf(stderr, "sent %zu packets (%zu samples/frame)\n", sent, frame);
	free(pcm);
	close(fd);
	if (in != stdin)
		fclose(in);
	return 0;
}
