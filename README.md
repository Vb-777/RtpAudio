# rtpaudio

**A real-time audio codec and RTP streaming stack in C — written to find out what the design trade-offs actually cost, not to assert them.**

An IMA ADPCM codec implemented from scratch, packetised into RFC 3550 RTP, carried over UDP, and reassembled by a jitter buffer that handles reordering, loss and concealment. Every design choice in it is backed by a number from `bench_e2e`.

```console
$ ./rtp_recv 5004 out.raw --depth 3 --conceal fade &
$ ./rtp_send 127.0.0.1 5004 speech.raw --frame 320 --pace
sent 150 packets (320 samples/frame)
received 139, played 139, concealed 11, reordered 0, late 0, dup 0, overflow 0
```

---

## Why ADPCM

ADPCM is the codec class that actually ships on constrained real-time links. It is sample-synchronous: no lookahead, no frame reordering, a fixed 4 bits per sample, so end-to-end *algorithmic* delay is a single sample rather than the 20–60 ms a transform codec needs for its analysis window. On a path where the budget is latency rather than bitrate, that is the right trade — which is why it survives in G.726 and in a great deal of embedded audio.

Measured, on an 8-second speech-like signal at 16 kHz:

| signal | SNR | compression |
|---|---|---|
| speech-like (fundamental + envelope + formant) | **32.2 dB** | exactly 4:1 |
| 440 Hz tone | 31.9 dB | 4:1 |
| frequency sweep | 22.4 dB | 4:1 |
| white noise | 15.6 dB | 4:1 |
| full-scale square wave | 7.1 dB | 4:1 |
| silence | exact (lossless) | 4:1 |

The square wave is the honest worst case: a predictive codec has nothing to predict at a discontinuity.

---

## The problem that shaped the design

ADPCM carries predictor and step-index state across samples. A decoder that misses a packet does not merely lose that packet's audio — **its state diverges from the encoder's and everything afterwards decodes to garbage** until the two happen to reconverge.

There are two ways out, and the benchmark measures both:

**Option 1 — reset the codec at every packet boundary.** Each packet becomes independently decodable, so a loss costs exactly one packet. The price is that the quantiser restarts from step index 0 and must re-adapt:

| packet size | SNR, state carried | SNR, reset per packet | header overhead |
|---|---|---|---|
| 80 samples (5 ms) | 32.53 dB | **10.71 dB** | 28.6% |
| 160 samples (10 ms) | 32.53 dB | **13.63 dB** | 16.7% |
| 320 samples (20 ms) | 32.53 dB | **16.53 dB** | 9.1% |
| 640 samples (40 ms) | 32.53 dB | **18.86 dB** | 4.8% |

Resetting costs **14–22 dB**. The penalty shrinks with larger packets only because the transient is amortised over more audio — and larger packets mean more latency and a bigger loss when one goes missing.

**Option 2 — carry the state and transmit it.** A 4-byte payload preamble (predictor, step index, padding) lets the receiver resynchronise on any packet. This is what the implementation does, and it is how the IMA ADPCM WAV format handles the same problem.

The cost is 4 bytes on top of the 12-byte RTP header: 9.1% total overhead at 20 ms packets, against a 14–22 dB quality gain. That is not a close call, and `test_packets_are_independently_decodable` asserts the property it buys — decoding packet 3 without having seen 1 or 2.

The cold-start transient is also directly measurable: the first packet from a reset state scores **16.1 dB**, the same packet's audio excluding the first 2 ms scores **35.3 dB**, and the second packet scores **35.2 dB**. The 16.53 dB in the table above is the same effect, which is a useful independent cross-check of both measurements.

---

## Jitter buffer

UDP gives no ordering and no delivery guarantee, but playout is strictly periodic: the sound card wants the next frame every 20 ms whether or not the network cooperated. Every decision the buffer makes is a latency-versus-dropout trade, so depth is a parameter and the curve is measured.

### Reordering tolerance

Every 7th packet delayed by N positions; playout pulls exactly one frame per interval, as a sound card would. Figures are `late/concealed` out of 400 packets:

| depth | delayed by 1 | delayed by 2 | delayed by 4 |
|---|---|---|---|
| 1 | 0/0 | 57/57 | 57/57 |
| 2 | 0/0 | 57/57 | 57/57 |
| **3** | 0/0 | **0/0** | 57/57 |
| **5** | 0/0 | 0/0 | **0/0** |
| 8 | 0/0 | 0/0 | 0/0 |

The rule falls out cleanly: **absorbing a reorder distance of N needs a depth of about N+1.** Each frame of depth costs 20 ms of added latency, which is the price list a real deployment needs.

### Sequence number wraparound

RTP sequence numbers are 16-bit and wrap every 65,536 packets — about 22 minutes at 20 ms, so every real call hits it. Comparisons are modular (`(int16_t)(a - b) > 0`), never `a > b`. The naive form reads the wrap as a 65,000-packet jump backwards and discards everything for the rest of the call; `test_sequence_comparison_handles_wraparound` pins it down.

### Concealment — and a result I did not expect

Three strategies, measured at 20 ms packets and depth 3. **SNR cannot see a click**, so the peak sample-to-sample jump is reported alongside it:

| loss | silence SNR / peak jump | hold SNR / peak jump | fade SNR / peak jump | concealed |
|---|---|---|---|---|
| 0% | 32.54 dB / 6862 | 32.54 dB / 6862 | 32.54 dB / 6862 | 0 |
| 1% | **19.49** dB / **14689** | 15.16 dB / 26715 | 17.18 dB / 20150 | 4 |
| 3% | **12.80** dB / **15460** | 8.60 dB / 26715 | 10.64 dB / 23860 | 19 |
| 5% | **11.43** dB / **15460** | 7.19 dB / 26715 | 9.24 dB / 24948 | 26 |
| 10% | **8.85** dB / **15460** | 4.67 dB / 26715 | 6.76 dB / 24948 | 46 |
| 20% | −0.80 dB / 15460 | −1.38 dB / 26715 | −0.97 dB / 24948 | 86 |

(The source signal's own peak jump is 5753, for reference.)

**Silence wins on both metrics, which contradicts the implementation's own default of `fade`.** On SNR that is expected — inserting a loud-but-wrong signal produces more squared error than inserting nothing. But silence also has the *smaller* discontinuity here, because stepping to zero from mid-waveform is at most a one-amplitude jump while splicing in the previous frame can be a two-amplitude jump.

I am not going to claim fade is better than this measurement says. What I will say is what the measurement does and does not cover: the argument for waveform-substitution concealment is that it preserves **pitch and spectral continuity** over a gap, which a listener hears and which neither SNR nor a peak-jump statistic can represent. An 8-second synthetic tone with a 2.5 Hz envelope is also a poor stand-in for speech, where those methods were developed. So:

- `fade` remains the default, on the strength of the published literature rather than on this benchmark;
- the benchmark says silence measured better on this signal by both available metrics;
- settling it properly needs real speech and either a perceptual metric (PESQ, ViSQOL) or a listening test, and that is the honest next step rather than a conclusion I can reach from here.

That gap between "what I measured" and "what I know" is the most useful thing in this project.

### UDP loopback

400 packets, 20 ms frames, depth 3, unpaced to measure throughput:

- **123,000 packets/s, 173 Mbit/s** of payload
- send-to-receive latency **p50 1.5 µs, p99 29.0 µs, p99.9 32.4 µs**
- end-to-end SNR **32.54 dB**, 0 concealed, 0 late, 0 duplicates

Paced in real time over a real socket with 5% simulated loss: 150 frames in, **150 frames out**, 11 concealed, **8.73 dB** — which matches the offline benchmark's 9.24 dB at 5% loss and so validates the two against each other.

---

## Bugs found while measuring

The measurement harness found more real bugs than the unit tests did, which is worth recording:

1. **The reordering counter was meaningless.** It compared each arrival against the *playout cursor*, so every normally buffered packet counted as reordered — a clean in-order 150-packet run reported 147 reorderings. A packet sitting ahead of the cursor is exactly what a jitter buffer is supposed to contain. Fixed to compare against the highest sequence yet accepted, and pinned by a test.

2. **The loss benchmark was measuring misalignment, not quality.** Playout was driven by packet arrivals, so a loss that emptied the buffer produced no frame at all; the output became shorter than the input and every later sample was compared against the wrong source sample. That reported **−3.5 dB at 20% loss** and, worse, numbers that looked plausible at low loss. Fixed by pulling exactly one frame per packet interval, as a sound card would.

3. **Then the alignment compensation was off by one.** Priming emits silence for `depth − 1` frames, not `depth`, so correcting by the wrong amount reported −4.16 dB even at **zero** loss. Fixed by searching a small lag window for the best alignment, which is standard practice and robust to the exact priming rule.

4. **The receiver skipped its drain on a dropped packet**, reproducing bug 2 in the shipped tool: the written file was shorter than the source.

Bugs 2 and 3 are the instructive ones. Both produced output that *looked* like a quality result and would have gone straight into this README as fact.

---

## Verification

```console
$ make test
93 checks, 0 failed

$ make asan        # ASAN + UBSAN
93 checks, 0 failed
```

The RTP tests assert on **exact wire bytes**. Both ends of this project share one implementation, so an endianness or bit-packing mistake is completely invisible to an end-to-end test and would only surface against a third-party stack. The tests also cover CSRC and extension-header skipping, rejection of malformed packets, out-of-range step indices in a corrupt preamble, and that the RTP timestamp advances by *samples* rather than by packets (RFC 3550 requires the media clock; counting packets breaks playout scheduling the moment packet sizes vary).

---

## Build and use

```bash
make              # rtp_send, rtp_recv, test_all, bench_e2e
make test
make bench
make asan
```

```bash
# a 3-second test signal
python3 -c "
import struct,math,sys
for i in range(48000):
    t=i/16000
    v=0.6*math.sin(2*math.pi*185*t)*(0.5+0.5*math.sin(2*math.pi*2.5*t))
    sys.stdout.buffer.write(struct.pack('<h',int(v*30000)))" > tone.raw

./rtp_recv 5004 out.raw --depth 3 --conceal fade --timeout-ms 1500 --loss 0.05 &
./rtp_send 127.0.0.1 5004 tone.raw --frame 320 --pace
```

`--pace` sleeps to the media clock using absolute deadlines, so processing cost does not accumulate as drift. `--loss P` drops a fraction of received packets, which exercises concealment without needing `tc`/`netem` privileges. Input and output are headerless signed 16-bit little-endian mono PCM, playable with `aplay -f S16_LE -r 16000 -c 1 out.raw`.

## Layout

```
src/adpcm.{h,c}      IMA ADPCM encoder/decoder, SNR and peak-error metrics
src/rtp.{h,c}        RFC 3550 header, packetiser, state-carrying preamble
src/jitterbuf.{h,c}  reordering buffer, loss detection, 3 concealment modes
src/sender.c         rtp_send: file or stdin to RTP/UDP, optional pacing
src/receiver.c       rtp_recv: RTP/UDP to PCM, with simulated loss
tests/test_all.c     93 assertions across codec, wire format and buffer
bench/bench_e2e.c    4 experiments: packet size, loss, reordering, UDP
```

## Limitations

- **No RTCP.** No receiver reports, so no feedback-driven rate or depth adaptation.
- **Fixed jitter buffer depth.** A production buffer adapts depth to measured delay variation; this one is told its depth and the benchmark shows what each choice costs.
- **Mono, 16 kHz, one payload type.** No multi-stream mixing, no SRTP, no NAT traversal.
- **Synthetic test signals only.** The concealment conclusion above is explicitly blocked on real speech and a perceptual metric.

## License

MIT
