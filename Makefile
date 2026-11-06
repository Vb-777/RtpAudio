# SPDX-License-Identifier: MIT
CC      ?= gcc
CFLAGS  ?= -O2 -Wall -Wextra -std=gnu11 -I src
LDLIBS  := -lm

SRC   := src/adpcm.c src/rtp.c src/jitterbuf.c
BINS  := rtp_send rtp_recv test_all bench_e2e

.PHONY: all test bench clean asan

all: $(BINS)

rtp_send: src/sender.c $(SRC)
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

rtp_recv: src/receiver.c $(SRC)
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

test_all: tests/test_all.c $(SRC)
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

bench_e2e: bench/bench_e2e.c $(SRC)
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

test: test_all
	./test_all

bench: bench_e2e
	./bench_e2e

# The jitter buffer and codec are pointer-heavy; these must stay clean.
asan: tests/test_all.c $(SRC)
	$(CC) -O1 -g -fsanitize=address,undefined -std=gnu11 -I src \
	      -o test_asan $^ $(LDLIBS)
	./test_asan

clean:
	rm -f $(BINS) test_asan *.raw
