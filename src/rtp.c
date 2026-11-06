/* SPDX-License-Identifier: MIT */

#include <string.h>

#include "rtp.h"

/* ------------------------------------------------------------------ */
/* Big-endian helpers. Written out rather than using htons/htonl so the
 * file has no network-header dependency and can be compiled for a target
 * without a socket stack.                                             */
/* ------------------------------------------------------------------ */

static inline void put_be16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v >> 8);
	p[1] = (uint8_t)(v & 0xff);
}

static inline void put_be32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v >> 24);
	p[1] = (uint8_t)(v >> 16);
	p[2] = (uint8_t)(v >> 8);
	p[3] = (uint8_t)(v & 0xff);
}

static inline uint16_t get_be16(const uint8_t *p)
{
	return (uint16_t)((uint16_t)p[0] << 8 | p[1]);
}

static inline uint32_t get_be32(const uint8_t *p)
{
	return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 |
	       (uint32_t)p[2] << 8 | p[3];
}

/* ------------------------------------------------------------------ */
/* Header                                                              */
/* ------------------------------------------------------------------ */

size_t rtp_write_header(const struct rtp_header *h, uint8_t *out)
{
	out[0] = (uint8_t)((h->version & 0x03) << 6 |
			   (h->padding & 0x01) << 5 |
			   (h->extension & 0x01) << 4 |
			   (h->csrc_count & 0x0f));
	out[1] = (uint8_t)((h->marker & 0x01) << 7 |
			   (h->payload_type & 0x7f));
	put_be16(out + 2, h->sequence);
	put_be32(out + 4, h->timestamp);
	put_be32(out + 8, h->ssrc);
	return RTP_HEADER_SIZE;
}

int rtp_parse_header(const uint8_t *in, size_t len, struct rtp_header *h)
{
	size_t offset;

	if (len < RTP_HEADER_SIZE)
		return -1;

	h->version      = (uint8_t)(in[0] >> 6);
	h->padding      = (uint8_t)((in[0] >> 5) & 0x01);
	h->extension    = (uint8_t)((in[0] >> 4) & 0x01);
	h->csrc_count   = (uint8_t)(in[0] & 0x0f);
	h->marker       = (uint8_t)(in[1] >> 7);
	h->payload_type = (uint8_t)(in[1] & 0x7f);
	h->sequence     = get_be16(in + 2);
	h->timestamp    = get_be32(in + 4);
	h->ssrc         = get_be32(in + 8);

	if (h->version != RTP_VERSION)
		return -1;

	offset = RTP_HEADER_SIZE + (size_t)h->csrc_count * 4;
	if (offset > len)
		return -1;

	/* Skip a header extension if present: 4-byte profile/length word
	 * followed by length 32-bit words. A parser that ignores this walks
	 * straight into the extension and decodes it as payload. */
	if (h->extension) {
		if (offset + 4 > len)
			return -1;
		offset += 4 + (size_t)get_be16(in + offset + 2) * 4;
		if (offset > len)
			return -1;
	}
	return (int)offset;
}

/* ------------------------------------------------------------------ */
/* Packetisation                                                       */
/* ------------------------------------------------------------------ */

void rtp_packetizer_init(struct rtp_packetizer *p, uint32_t ssrc,
			 int reset_per_packet)
{
	adpcm_init(&p->codec);
	/* A random initial sequence number is what RFC 3550 recommends; a fixed
	 * start is used here so test output is reproducible. */
	p->sequence = 0;
	p->timestamp = 0;
	p->ssrc = ssrc;
	p->reset_per_packet = reset_per_packet;
}

size_t rtp_pack(struct rtp_packetizer *p, const int16_t *pcm,
		size_t n_samples, uint8_t *out, size_t out_cap)
{
	struct rtp_header h;
	struct adpcm_state snapshot;
	size_t payload_bytes = adpcm_encoded_bytes(n_samples);
	size_t total = RTP_HEADER_SIZE + ADPCM_PREAMBLE_SIZE + payload_bytes;
	uint8_t *pre;

	if (out_cap < total || n_samples == 0)
		return 0;

	if (p->reset_per_packet)
		adpcm_init(&p->codec);

	/* Snapshot before encoding: the receiver needs the state as it was at
	 * the first sample, not as it is afterwards. */
	snapshot = p->codec;

	memset(&h, 0, sizeof(h));
	h.version = RTP_VERSION;
	h.payload_type = RTP_PT_ADPCM;
	h.sequence = p->sequence++;
	h.timestamp = p->timestamp;
	h.ssrc = p->ssrc;
	/* Timestamp advances by samples, not packets: RFC 3550 requires the
	 * media clock, which is what lets the receiver compute playout time
	 * even when packet sizes vary. */
	p->timestamp += (uint32_t)n_samples;

	rtp_write_header(&h, out);

	pre = out + RTP_HEADER_SIZE;
	put_be16(pre, (uint16_t)snapshot.predictor);
	pre[2] = (uint8_t)snapshot.step_index;
	pre[3] = 0;

	adpcm_encode(&p->codec, pcm, n_samples,
		     out + RTP_HEADER_SIZE + ADPCM_PREAMBLE_SIZE);
	return total;
}

size_t rtp_unpack(const uint8_t *pkt, size_t len, int16_t *pcm,
		  size_t pcm_cap, struct rtp_header *h_out)
{
	struct rtp_header h;
	struct adpcm_state st;
	int offset = rtp_parse_header(pkt, len, &h);
	size_t payload_bytes, n_samples;
	const uint8_t *pre;

	if (offset < 0)
		return 0;
	if (h.payload_type != RTP_PT_ADPCM)
		return 0;
	if (len < (size_t)offset + ADPCM_PREAMBLE_SIZE)
		return 0;

	pre = pkt + offset;
	st.predictor = (int16_t)get_be16(pre);
	st.step_index = (int8_t)pre[2];
	/* A corrupt or hostile packet must not index past the step table. */
	if (st.step_index < 0 || st.step_index > 88)
		return 0;

	payload_bytes = len - (size_t)offset - ADPCM_PREAMBLE_SIZE;
	n_samples = adpcm_sample_count(payload_bytes);
	if (n_samples > pcm_cap)
		n_samples = pcm_cap;
	if (n_samples == 0)
		return 0;

	adpcm_decode(&st, pre + ADPCM_PREAMBLE_SIZE, n_samples, pcm);
	if (h_out)
		*h_out = h;
	return n_samples;
}

size_t rtp_payload_samples(size_t packet_len)
{
	if (packet_len <= RTP_HEADER_SIZE + ADPCM_PREAMBLE_SIZE)
		return 0;
	return adpcm_sample_count(packet_len - RTP_HEADER_SIZE -
				  ADPCM_PREAMBLE_SIZE);
}
