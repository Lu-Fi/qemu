/*
 * CCMP as 802.11 frames carry it, for chip models that meet a frame their
 * driver encrypted itself, or hand one over for it to decrypt
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef HW_NET_WIFI_CCMP_H
#define HW_NET_WIFI_CCMP_H

#define WIFI_CCMP_HDR_LEN       8
#define WIFI_CCMP_MIC_LEN       8

/* The 802.11 header's length, from its frame control */
size_t wifi_hdr_len(uint16_t fc);

/*
 * Decrypt a protected frame in place with the temporal key @tk: the CCMP
 * header and MIC go, the Protected bit clears and @len shrinks to match.
 * False, with the frame untouched, if it is short or the MIC is wrong.
 */
bool wifi_ccmp_decrypt(const uint8_t *tk, uint8_t *frame, size_t *len);

/*
 * The frame @f as the air carries it encrypted with @tk under packet
 * number @pn and key @key_id, into @out (len + 16 bytes): Protected set,
 * the CCMP header after the 802.11 header, the MIC last. Its length.
 */
size_t wifi_ccmp_encrypt(const uint8_t *tk, uint64_t pn, unsigned key_id,
                         const uint8_t *f, size_t len, uint8_t *out);

#endif
