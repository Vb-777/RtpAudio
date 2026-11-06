/* SPDX-License-Identifier: MIT */
/*
 * rtp.h - RTP packetisation (RFC 3550) for ADPCM audio.
 *
 * Only the fixed 12-byte header is implemented, which is all a point-to-point
 * audio path needs. CSRC lists, header extensions and padding are parsed far
 * enough to be skipped correctly but never generated.
 *
 * THE DESIGN DECISION THAT MATTERS HERE
 * =====================================
 * ADPCM carries predictor state across samples, so a decoder that misses a
 * packet does not just lose that packet's audio -- its predictor and step index
 * diverge from the encoder's and every subsequent packet decodes to garbage
 * until the state happens to reconverge.
 *
 * Two ways out:
 *
 *   1. Reset the codec state at every packet boundary. Each packet becomes
 *      independently decodable, so a loss costs exactly one packet of audio.
 *      The price is a small SNR penalty, because the quantiser restarts from
 *      step index 0 and has to re-adapt at the start of each packet.
 *
 *   2. Carry state across packets and transmit it in the payload header so the
 *      receiver can resynchronise. Costs 3 bytes per packet and keeps full
 *      adaptive quality.
 *
 * This implementation does (2): the predictor and step index are sent in a
 * 4-byte payload preamble, which is how the IMA ADPCM WAV format and several
 * VoIP profiles handle it. The benchmark measures what option (1) and a naive
 * stateful stream would each cost, so the choice is justified by numbers rather
 * than assertion.
 */

#ifndef RTP_H
#define RTP_H

#include <stddef.h>
#include <stdint.h>

#include "adpcm.h"

#define RTP_HEADER_SIZE   12
#define RTP_VERSION        2
/* Dynamic payload type: 96-127 are unassigned in RFC 3551. */
#define RTP_PT_ADPCM      97
/* Predictor (2 bytes) + step index (1) + reserved (1), kept 4-byte aligned so
 * the ADPCM nibbles that follow start on a natural boundary. */
#define ADPCM_PREAMBLE_SIZE 4

struct rtp_header {
	uint8_t  version;
	uint8_t  padding;
	uint8_t  extension;
	uint8_t  csrc_count;
	uint8_t  marker;
	uint8_t  payload_type;
	uint16_t sequence;
	uint32_t timestamp;
	uint32_t ssrc;
};

/*
 * Serialise a header into @out (>= RTP_HEADER_SIZE bytes). All multi-byte
 * fields go out big-endian, as RFC 3550 requires -- getting this wrong is the
 * single most common RTP interop bug, and it is invisible when both ends share
 * the implementation, which is why the tests assert on the exact bytes.
 */
size_t rtp_write_header(const struct rtp_header *h, uint8_t *out);

/*
 * Parse a header from @in of @len bytes. Returns the payload offset in bytes
 * (accounting for CSRC list and any extension), or -1 if the packet is
 * malformed or too short.
 */
int rtp_parse_header(const uint8_t *in, size_t len, struct rtp_header *h);

/* ------------------------------------------------------------------ */
/* Packetisation                                                       */
/* ------------------------------------------------------------------ */

struct rtp_packetizer {
	struct adpcm_state codec;
	uint16_t sequence;
	uint32_t timestamp;
	uint32_t ssrc;
	int      reset_per_packet;   /* option (1) above, for the ablation */
};

void rtp_packetizer_init(struct rtp_packetizer *p, uint32_t ssrc,
			 int reset_per_packet);

/*
 * Encode @n_samples of PCM into one RTP packet in @out.
 * Returns total packet length, or 0 if @out_cap is too small.
 *
 * Required capacity: RTP_HEADER_SIZE + ADPCM_PREAMBLE_SIZE +
 *                    adpcm_encoded_bytes(n_samples)
 */
size_t rtp_pack(struct rtp_packetizer *p, const int16_t *pcm,
		size_t n_samples, uint8_t *out, size_t out_cap);

/*
 * Decode one RTP packet into PCM. Returns samples produced, or 0 on a
 * malformed packet. The decoder state travels in the packet, so this is
 * stateless with respect to packet order -- which is what lets the jitter
 * buffer deliver a late packet without corrupting the ones around it.
 */
size_t rtp_unpack(const uint8_t *pkt, size_t len, int16_t *pcm,
		  size_t pcm_cap, struct rtp_header *h_out);

/* Samples carried by a packet of this length, without decoding it. */
size_t rtp_payload_samples(size_t packet_len);

#endif /* RTP_H */
