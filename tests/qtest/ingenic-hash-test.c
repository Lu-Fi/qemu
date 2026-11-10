/*
 * QTest testcase for the Ingenic HASH engine
 *
 * Known-answer tests for MD5, SHA-1, SHA-224, SHA-256, SHA-384 and SHA-512
 * on all eleven machines: the RFC 1321 and FIPS 180-4 example messages
 * (the empty message, "abc", the two-block messages, a million "a"), and
 * messages of every length around the padding boundaries of the 64-byte
 * and 128-byte blocks. The digests marked "FIPS 180-4" and "RFC 1321" are
 * the standards'; the boundary lengths' were computed with Python's
 * hashlib. The engine does not pad: the test pads as a driver must.
 *
 * Each case is driven two ways as the vendor code does:
 *  - By register, as U-Boot's sha256_update: HSCR enable, algorithm,
 *    DIRVS (SHA only) and INIT, then the message as words through HSDI
 *    (the bytes little-endian), waiting for HSSR.ORD after each block and
 *    clearing it, then HSDO read out and byte-swapped by software.
 *  - By DMA, as the jz-hash driver: HSSA and HSTC, DMAE then DMAS, then
 *    HSDO with DIRVS and DORVS (SHA only), the words in memory order.
 *    HSTC counts 64-byte units; on the T33 it counts words, with HSBNUM at
 *    0x14 and HSDI and HSDO a word up. Messages of three blocks or more
 *    run in two DMA starts as well.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"
#include "libqtest.h"

#define HASH_BASE       0x13480000
#define SRC             0x00100000

#define HSCR            0x00
#define HSSR            0x04
#define HSINTM          0x08
#define HSSA            0x0c
#define HSTC            0x10
#define HSBNUM_T33      0x14

#define HSCR_EN         BIT(0)
#define HSCR_SEL(a)     ((a) << 1)
#define HSCR_INIT       BIT(4)
#define HSCR_DMAE       BIT(5)
#define HSCR_DMAS       BIT(6)
#define HSCR_DIRVS      BIT(7)
#define HSCR_DORVS      BIT(8)
#define HSSR_ORD        BIT(0)
#define HSSR_MRD        BIT(1)

/* The algorithm select values, and the order of the digests' tables */
enum { ALG_MD5, ALG_SHA1, ALG_SHA224, ALG_SHA256, ALG_SHA384, ALG_SHA512 };

static const char *const alg_name[] = {
    "md5", "sha1", "sha224", "sha256", "sha384", "sha512",
};
static const unsigned alg_words[] = { 4, 5, 7, 8, 12, 16 };

typedef struct {
    const char *part;
    const char *machine;
    bool t33;
} HashPart;

static const HashPart parts[] = {
    { "t10", "ingenic-t10" },
    { "t20", "ingenic-t20" },
    { "t21", "ingenic-t21" },
    { "t23", "ingenic-t23" },
    { "t30", "ingenic-t30" },
    { "t31", "ingenic-t31" },
    { "t32", "ingenic-t32" },
    { "t33", "ingenic-t33", .t33 = true },
    { "t40", "ingenic-t40" },
    { "t41", "ingenic-t41" },
    { "a1", "ingenic-a1" },
};

typedef struct {
    const char *name;
    const char *str;            /* repeated */
    unsigned rep;
} HashMsg;

typedef struct {
    unsigned alg;
    unsigned msg;
    const char *digest;
    const char *source;
} HashVec;

static const HashMsg msgs[] = {
    { "empty",
      "", 1 },
    { "abc",
      "abc", 1 },
    { "a",
      "a", 1 },
    { "message-digest",
      "message digest", 1 },
    { "alpha26",
      "abcdefghijklmnopqrstuvwxyz", 1 },
    { "alnum62",
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123"
      "456789", 1 },
    { "digits80",
      "1234567890", 8 },
    { "two-block-448",
      "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq", 1 },
    { "two-block-896",
      "abcdefghbcdefghicdefghijdefghijkefghijklfghijklmghijklmn"
      "hijklmnoijklmnopjklmnopqklmnopqrlmnopqrsmnopqrstnopqrstu", 1 },
    { "million-a",
      "a", 1000000 },
    { "a-x55",
      "a", 55 },
    { "a-x56",
      "a", 56 },
    { "a-x63",
      "a", 63 },
    { "a-x64",
      "a", 64 },
    { "a-x65",
      "a", 65 },
    { "a-x111",
      "a", 111 },
    { "a-x112",
      "a", 112 },
    { "a-x119",
      "a", 119 },
    { "a-x120",
      "a", 120 },
    { "a-x127",
      "a", 127 },
    { "a-x128",
      "a", 128 },
    { "a-x129",
      "a", 129 },
};

static const HashVec vecs[] = {
    { ALG_MD5, 0,
      "d41d8cd98f00b204e9800998ecf8427e",
      "RFC 1321" },
    { ALG_MD5, 1,
      "900150983cd24fb0d6963f7d28e17f72",
      "RFC 1321" },
    { ALG_MD5, 2,
      "0cc175b9c0f1b6a831c399e269772661",
      "RFC 1321" },
    { ALG_MD5, 3,
      "f96b697d7cb7938d525a2f31aaf161d0",
      "RFC 1321" },
    { ALG_MD5, 4,
      "c3fcd3d76192e4007dfb496cca67e13b",
      "RFC 1321" },
    { ALG_MD5, 5,
      "d174ab98d277d9f5a5611c2c9f419d9f",
      "RFC 1321" },
    { ALG_MD5, 6,
      "57edf4a22be3c955ac49da2e2107b67a",
      "RFC 1321" },
    { ALG_MD5, 7,
      "8215ef0796a20bcaaae116d3876c664a",
      "hashlib" },
    { ALG_MD5, 8,
      "03dd8807a93175fb062dfb55dc7d359c",
      "hashlib" },
    { ALG_MD5, 10,
      "ef1772b6dff9a122358552954ad0df65",
      "hashlib" },
    { ALG_MD5, 11,
      "3b0c8ac703f828b04c6c197006d17218",
      "hashlib" },
    { ALG_MD5, 12,
      "b06521f39153d618550606be297466d5",
      "hashlib" },
    { ALG_MD5, 13,
      "014842d480b571495a4a0363793f7367",
      "hashlib" },
    { ALG_MD5, 14,
      "c743a45e0d2e6a95cb859adae0248435",
      "hashlib" },
    { ALG_MD5, 15,
      "089f243d1e831c5879aa375ee364a06e",
      "hashlib" },
    { ALG_MD5, 16,
      "9146ef3527c7cfcc66dc615c3986e391",
      "hashlib" },
    { ALG_MD5, 17,
      "8a7bd0732ed6a28ce75f6dabc90e1613",
      "hashlib" },
    { ALG_MD5, 18,
      "5f61c0ccad4cac44c75ff505e1f1e537",
      "hashlib" },
    { ALG_MD5, 19,
      "020406e1d05cdc2aa287641f7ae2cc39",
      "hashlib" },
    { ALG_MD5, 20,
      "e510683b3f5ffe4093d021808bc6ff70",
      "hashlib" },
    { ALG_MD5, 21,
      "b325dc1c6f5e7a2b7cf465b9feab7948",
      "hashlib" },
    { ALG_MD5, 9,
      "7707d6ae4e027c70eea2a935c2296f21",
      "RFC 1321" },
    { ALG_SHA1, 0,
      "da39a3ee5e6b4b0d3255bfef95601890afd80709",
      "FIPS 180-4" },
    { ALG_SHA1, 1,
      "a9993e364706816aba3e25717850c26c9cd0d89d",
      "FIPS 180-4" },
    { ALG_SHA1, 2,
      "86f7e437faa5a7fce15d1ddcb9eaeaea377667b8",
      "hashlib" },
    { ALG_SHA1, 3,
      "c12252ceda8be8994d5fa0290a47231c1d16aae3",
      "hashlib" },
    { ALG_SHA1, 4,
      "32d10c7b8cf96570ca04ce37f2a19d84240d3a89",
      "hashlib" },
    { ALG_SHA1, 5,
      "761c457bf73b14d27e9e9265c46f4b4dda11f940",
      "hashlib" },
    { ALG_SHA1, 6,
      "50abf5706a150990a08b2c5ea40fa0e585554732",
      "hashlib" },
    { ALG_SHA1, 7,
      "84983e441c3bd26ebaae4aa1f95129e5e54670f1",
      "FIPS 180-4" },
    { ALG_SHA1, 8,
      "a49b2446a02c645bf419f995b67091253a04a259",
      "hashlib" },
    { ALG_SHA1, 10,
      "c1c8bbdc22796e28c0e15163d20899b65621d65a",
      "hashlib" },
    { ALG_SHA1, 11,
      "c2db330f6083854c99d4b5bfb6e8f29f201be699",
      "hashlib" },
    { ALG_SHA1, 12,
      "03f09f5b158a7a8cdad920bddc29b81c18a551f5",
      "hashlib" },
    { ALG_SHA1, 13,
      "0098ba824b5c16427bd7a1122a5a442a25ec644d",
      "hashlib" },
    { ALG_SHA1, 14,
      "11655326c708d70319be2610e8a57d9a5b959d3b",
      "hashlib" },
    { ALG_SHA1, 15,
      "ac877859d427d9192054eea8feb3b8a403ef83a5",
      "hashlib" },
    { ALG_SHA1, 16,
      "689993727ba37386bb032495e9dbdfb4dd1ba744",
      "hashlib" },
    { ALG_SHA1, 17,
      "ee971065aaa017e0632a8ca6c77bb3bf8b1dfc56",
      "hashlib" },
    { ALG_SHA1, 18,
      "f34c1488385346a55709ba056ddd08280dd4c6d6",
      "hashlib" },
    { ALG_SHA1, 19,
      "89d95fa32ed44a7c610b7ee38517ddf57e0bb975",
      "hashlib" },
    { ALG_SHA1, 20,
      "ad5b3fdbcb526778c2839d2f151ea753995e26a0",
      "hashlib" },
    { ALG_SHA1, 21,
      "d96debf1bdcbc896e6c134ea76e8141f40d78536",
      "hashlib" },
    { ALG_SHA1, 9,
      "34aa973cd4c4daa4f61eeb2bdbad27316534016f",
      "FIPS 180-4" },
    { ALG_SHA224, 0,
      "d14a028c2a3a2bc9476102bb288234c415a2b01f828ea62ac5b3e42f",
      "FIPS 180-4" },
    { ALG_SHA224, 1,
      "23097d223405d8228642a477bda255b32aadbce4bda0b3f7e36c9da7",
      "FIPS 180-4" },
    { ALG_SHA224, 2,
      "abd37534c7d9a2efb9465de931cd7055ffdb8879563ae98078d6d6d5",
      "hashlib" },
    { ALG_SHA224, 3,
      "2cb21c83ae2f004de7e81c3c7019cbcb65b71ab656b22d6d0c39b8eb",
      "hashlib" },
    { ALG_SHA224, 4,
      "45a5f72c39c5cff2522eb3429799e49e5f44b356ef926bcf390dccc2",
      "hashlib" },
    { ALG_SHA224, 5,
      "bff72b4fcb7d75e5632900ac5f90d219e05e97a7bde72e740db393d9",
      "hashlib" },
    { ALG_SHA224, 6,
      "b50aecbe4e9bb0b57bc5f3ae760a8e01db24f203fb3cdcd13148046e",
      "hashlib" },
    { ALG_SHA224, 7,
      "75388b16512776cc5dba5da1fd890150b0c6455cb4f58b1952522525",
      "FIPS 180-4" },
    { ALG_SHA224, 8,
      "c97ca9a559850ce97a04a96def6d99a9e0e0e2ab14e6b8df265fc0b3",
      "hashlib" },
    { ALG_SHA224, 10,
      "fb0bd626a70c28541dfa781bb5cc4d7d7f56622a58f01a0b1ddd646f",
      "hashlib" },
    { ALG_SHA224, 11,
      "d40854fc9caf172067136f2e29e1380b14626bf6f0dd06779f820dcd",
      "hashlib" },
    { ALG_SHA224, 12,
      "1d4e051f4d6fed2a63fd2421e65834cec00d64456553de3496ae8b1d",
      "hashlib" },
    { ALG_SHA224, 13,
      "a88cd5cde6d6fe9136a4e58b49167461ea95d388ca2bdb7afdc3cbf4",
      "hashlib" },
    { ALG_SHA224, 14,
      "ff8716f600af42959d0efb52e1f21b01bb328733009344d511c299fb",
      "hashlib" },
    { ALG_SHA224, 15,
      "4aeec1a49b2c1bc663abf2809b36faaa64359523d4f26d02dbc2cba3",
      "hashlib" },
    { ALG_SHA224, 16,
      "0336b66821946e7f1052102e3b9c29f3039efe9b261746370305f894",
      "hashlib" },
    { ALG_SHA224, 17,
      "e000e6709d26667b631faa7fc1bd404eb4774003c5fb4f51a0184875",
      "hashlib" },
    { ALG_SHA224, 18,
      "66924e30a9929327e7a6cf03747397226ed2efc180ebe3dea7132a79",
      "hashlib" },
    { ALG_SHA224, 19,
      "0822db3f33424aead078f71ed05f30edc077a3c254b7c79c89a7a4a1",
      "hashlib" },
    { ALG_SHA224, 20,
      "39873a2441c56608137850f4c54dde157710b9a2b83c8bdc756dd643",
      "hashlib" },
    { ALG_SHA224, 21,
      "321318841bcc3d0da8185fdd8643f3e4ac629d18f298fc141324074f",
      "hashlib" },
    { ALG_SHA224, 9,
      "20794655980c91d8bbb4c1ea97618a4bf03f42581948b2ee4ee7ad67",
      "FIPS 180-4" },
    { ALG_SHA256, 0,
      "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b"
      "7852b855",
      "FIPS 180-4" },
    { ALG_SHA256, 1,
      "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61"
      "f20015ad",
      "FIPS 180-4" },
    { ALG_SHA256, 2,
      "ca978112ca1bbdcafac231b39a23dc4da786eff8147c4e72b9807785"
      "afee48bb",
      "hashlib" },
    { ALG_SHA256, 3,
      "f7846f55cf23e14eebeab5b4e1550cad5b509e3348fbc4efa3a1413d"
      "393cb650",
      "hashlib" },
    { ALG_SHA256, 4,
      "71c480df93d6ae2f1efad1447c66c9525e316218cf51fc8d9ed832f2"
      "daf18b73",
      "hashlib" },
    { ALG_SHA256, 5,
      "db4bfcbd4da0cd85a60c3c37d3fbd8805c77f15fc6b1fdfe614ee0a7"
      "c8fdb4c0",
      "hashlib" },
    { ALG_SHA256, 6,
      "f371bc4a311f2b009eef952dd83ca80e2b60026c8e935592d0f9c308"
      "453c813e",
      "hashlib" },
    { ALG_SHA256, 7,
      "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd4"
      "19db06c1",
      "FIPS 180-4" },
    { ALG_SHA256, 8,
      "cf5b16a778af8380036ce59e7b0492370b249b11e8f07a51afac4503"
      "7afee9d1",
      "hashlib" },
    { ALG_SHA256, 10,
      "9f4390f8d30c2dd92ec9f095b65e2b9ae9b0a925a5258e241c9f1e91"
      "0f734318",
      "hashlib" },
    { ALG_SHA256, 11,
      "b35439a4ac6f0948b6d6f9e3c6af0f5f590ce20f1bde7090ef797068"
      "6ec6738a",
      "hashlib" },
    { ALG_SHA256, 12,
      "7d3e74a05d7db15bce4ad9ec0658ea98e3f06eeecf16b4c6fff2da45"
      "7ddc2f34",
      "hashlib" },
    { ALG_SHA256, 13,
      "ffe054fe7ae0cb6dc65c3af9b61d5209f439851db43d0ba5997337df"
      "154668eb",
      "hashlib" },
    { ALG_SHA256, 14,
      "635361c48bb9eab14198e76ea8ab7f1a41685d6ad62aa9146d301d4f"
      "17eb0ae0",
      "hashlib" },
    { ALG_SHA256, 15,
      "6374f73208854473827f6f6a3f43b1f53eaa3b82c21c1a6d69a2110b"
      "2a79baad",
      "hashlib" },
    { ALG_SHA256, 16,
      "f54353008a2553262ecdc4a34749563ba0950e8b0fc8652780b0a614"
      "b99683c1",
      "hashlib" },
    { ALG_SHA256, 17,
      "31eba51c313a5c08226adf18d4a359cfdfd8d2e816b13f4af952f7ea"
      "6584dcfb",
      "hashlib" },
    { ALG_SHA256, 18,
      "2f3d335432c70b580af0e8e1b3674a7c020d683aa5f73aaaedfdc55a"
      "f904c21c",
      "hashlib" },
    { ALG_SHA256, 19,
      "c57e9278af78fa3cab38667bef4ce29d783787a2f731d4e12200270f"
      "0c32320a",
      "hashlib" },
    { ALG_SHA256, 20,
      "6836cf13bac400e9105071cd6af47084dfacad4e5e302c94bfed24e0"
      "13afb73e",
      "hashlib" },
    { ALG_SHA256, 21,
      "c12cb024a2e5551cca0e08fce8f1c5e314555cc3fef6329ee994a3db"
      "752166ae",
      "hashlib" },
    { ALG_SHA256, 9,
      "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39cc"
      "c7112cd0",
      "FIPS 180-4" },
    { ALG_SHA384, 0,
      "38b060a751ac96384cd9327eb1b1e36a21fdb71114be07434c0cc7bf"
      "63f6e1da274edebfe76f65fbd51ad2f14898b95b",
      "hashlib" },
    { ALG_SHA384, 1,
      "cb00753f45a35e8bb5a03d699ac65007272c32ab0eded1631a8b605a"
      "43ff5bed8086072ba1e7cc2358baeca134c825a7",
      "FIPS 180-4" },
    { ALG_SHA384, 2,
      "54a59b9f22b0b80880d8427e548b7c23abd873486e1f035dce9cd697"
      "e85175033caa88e6d57bc35efae0b5afd3145f31",
      "hashlib" },
    { ALG_SHA384, 3,
      "473ed35167ec1f5d8e550368a3db39be54639f828868e9454c239fc8"
      "b52e3c61dbd0d8b4de1390c256dcbb5d5fd99cd5",
      "hashlib" },
    { ALG_SHA384, 4,
      "feb67349df3db6f5924815d6c3dc133f091809213731fe5c7b5f4999"
      "e463479ff2877f5f2936fa63bb43784b12f3ebb4",
      "hashlib" },
    { ALG_SHA384, 5,
      "1761336e3f7cbfe51deb137f026f89e01a448e3b1fafa64039c1464e"
      "e8732f11a5341a6f41e0c202294736ed64db1a84",
      "hashlib" },
    { ALG_SHA384, 6,
      "b12932b0627d1c060942f5447764155655bd4da0c9afa6dd9b9ef531"
      "29af1b8fb0195996d2de9ca0df9d821ffee67026",
      "hashlib" },
    { ALG_SHA384, 7,
      "3391fdddfc8dc7393707a65b1b4709397cf8b1d162af05abfe8f450d"
      "e5f36bc6b0455a8520bc4e6f5fe95b1fe3c8452b",
      "hashlib" },
    { ALG_SHA384, 8,
      "09330c33f71147e83d192fc782cd1b4753111b173b3b05d22fa08086"
      "e3b0f712fcc7c71a557e2db966c3e9fa91746039",
      "FIPS 180-4" },
    { ALG_SHA384, 10,
      "5d91ac7e74e62b5c728904b40f10784d66b7af9cb6302123e48c92f0"
      "432ceb8d2a92c02de77dcb29ed75c4b42bde46f4",
      "hashlib" },
    { ALG_SHA384, 11,
      "8a8d9649ea04e993a6ca7135af7e3392cc5fca84f8531cac7aa3feed"
      "4eb98f55dcbe0f3284b61c6f35f98b02cc644b4c",
      "hashlib" },
    { ALG_SHA384, 12,
      "7e7f097e95b52bb8f53383450ecaf9868187c130981730c03b6d573a"
      "dfc0b991e365244e5b4bfa082bfb43c517d37120",
      "hashlib" },
    { ALG_SHA384, 13,
      "2e404b9339da795776e510d96930b3be2904c500395b8cb7413334b8"
      "2d4dec413b4b8113045a05bbbcff846f027423f6",
      "hashlib" },
    { ALG_SHA384, 14,
      "a2b797de45aa1d20594e2ea0cb1801d6dd7ffa3a58281c37dde04029"
      "f1c98157bb6eb005e90dd76b79a1d6ddd52dd58d",
      "hashlib" },
    { ALG_SHA384, 15,
      "3c37955051cb5c3026f94d551d5b5e2ac38d572ae4e07172085fed81"
      "f8466b8f90dc23a8ffcdea0b8d8e58e8fdacc80a",
      "hashlib" },
    { ALG_SHA384, 16,
      "187d4e07cb306103c69967bf544d0dfbe9042577599c73c330abc0cb"
      "64c61236d5ed565ee19119d8c31779a38f791fcd",
      "hashlib" },
    { ALG_SHA384, 17,
      "c2fbb1911d6889e3db556b482236ab82f3c736f00a22c088641a09fd"
      "bbca27e3f1e3b6235bad20aee1ca083c76ac590c",
      "hashlib" },
    { ALG_SHA384, 18,
      "ca2f7755efa04d43651f9bcb466044511102e472c2a3981c836b487e"
      "e4508ca8461f8c396653123400762de4d6d17e63",
      "hashlib" },
    { ALG_SHA384, 19,
      "9bd06b1763c2cf7aef40e795dc65bc96d59c41b537f3ad72ebdefd48"
      "5476b5717c1aeb37c327fe9c1831b12b9efd08ae",
      "hashlib" },
    { ALG_SHA384, 20,
      "edb12730a366098b3b2beac75a3bef1b0969b15c48e2163c23d96994"
      "f8d1bef760c7e27f3c464d3829f56c0d53808b0b",
      "hashlib" },
    { ALG_SHA384, 21,
      "39b6f5a7b0e781dbc419f72e49b30eaac10f2c98c4403bc610da3106"
      "7fd1b48f324138c8615d2b496d08d73d5e865326",
      "hashlib" },
    { ALG_SHA384, 9,
      "9d0e1809716474cb086e834e310a4a1ced149e9c00f248527972cec5"
      "704c2a5b07b8b3dc38ecc4ebae97ddd87f3d8985",
      "FIPS 180-4" },
    { ALG_SHA512, 0,
      "cf83e1357eefb8bdf1542850d66d8007d620e4050b5715dc83f4a921"
      "d36ce9ce47d0d13c5d85f2b0ff8318d2877eec2f63b931bd47417a81"
      "a538327af927da3e",
      "FIPS 180-4" },
    { ALG_SHA512, 1,
      "ddaf35a193617abacc417349ae20413112e6fa4e89a97ea20a9eeee6"
      "4b55d39a2192992a274fc1a836ba3c23a3feebbd454d4423643ce80e"
      "2a9ac94fa54ca49f",
      "FIPS 180-4" },
    { ALG_SHA512, 2,
      "1f40fc92da241694750979ee6cf582f2d5d7d28e18335de05abc54d0"
      "560e0f5302860c652bf08d560252aa5e74210546f369fbbbce8c12cf"
      "c7957b2652fe9a75",
      "hashlib" },
    { ALG_SHA512, 3,
      "107dbf389d9e9f71a3a95f6c055b9251bc5268c2be16d6c13492ea45"
      "b0199f3309e16455ab1e96118e8a905d5597b72038ddb372a8982604"
      "6de66687bb420e7c",
      "hashlib" },
    { ALG_SHA512, 4,
      "4dbff86cc2ca1bae1e16468a05cb9881c97f1753bce3619034898faa"
      "1aabe429955a1bf8ec483d7421fe3c1646613a59ed5441fb0f321389"
      "f77f48a879c7b1f1",
      "hashlib" },
    { ALG_SHA512, 5,
      "1e07be23c26a86ea37ea810c8ec7809352515a970e9253c26f536cfc"
      "7a9996c45c8370583e0a78fa4a90041d71a4ceab7423f19c71b9d5a3"
      "e01249f0bebd5894",
      "hashlib" },
    { ALG_SHA512, 6,
      "72ec1ef1124a45b047e8b7c75a932195135bb61de24ec0d191404224"
      "6e0aec3a2354e093d76f3048b456764346900cb130d2a4fd5dd16abb"
      "5e30bcb850dee843",
      "hashlib" },
    { ALG_SHA512, 7,
      "204a8fc6dda82f0a0ced7beb8e08a41657c16ef468b228a8279be331"
      "a703c33596fd15c13b1b07f9aa1d3bea57789ca031ad85c7a71dd703"
      "54ec631238ca3445",
      "hashlib" },
    { ALG_SHA512, 8,
      "8e959b75dae313da8cf4f72814fc143f8f7779c6eb9f7fa17299aead"
      "b6889018501d289e4900f7e4331b99dec4b5433ac7d329eeb6dd2654"
      "5e96e55b874be909",
      "FIPS 180-4" },
    { ALG_SHA512, 10,
      "b0220c772cbf6c1822e2cb38a437d0e1d58772417a4bbb21c961364f"
      "8b6143e05aa6316dca8d1d7b19e16448419076395f6086cb55101fbd"
      "6d5497b148e1745f",
      "hashlib" },
    { ALG_SHA512, 11,
      "962b64aae357d2a4fee3ded8b539bdc9d325081822b0bfc55583133a"
      "ab44f18bafe11d72a7ae16c79ce2ba620ae2242d5144809161945f13"
      "67f41b3972e26e04",
      "hashlib" },
    { ALG_SHA512, 12,
      "c1b0f5c6d3b03dfe4a2602e67242f54e344090b66e01100a469b129f"
      "583f016c7e27dddeaa438393dcc7ec54b0b57c9ba7af007f9b56db5f"
      "6fb677d972a31362",
      "hashlib" },
    { ALG_SHA512, 13,
      "01d35c10c6c38c2dcf48f7eebb3235fb5ad74a65ec4cd016e2354c63"
      "7a8fb49b695ef3c1d6f7ae4cd74d78cc9c9bcac9d4f23a73019998a7"
      "f73038a5c9b2dbde",
      "hashlib" },
    { ALG_SHA512, 14,
      "b83086cd8494e55708ad7ecd82dfb4bca1bda61ecbb7caf0c6896790"
      "2e709345e5d8305eb7ac0d588afc6cbb75161aa9c8c7e0ea986bd833"
      "dafe5e1ccd37345a",
      "hashlib" },
    { ALG_SHA512, 15,
      "fa9121c7b32b9e01733d034cfc78cbf67f926c7ed83e82200ef86818"
      "196921760b4beff48404df811b953828274461673c68d04e297b0eb7"
      "b2b4d60fc6b566a2",
      "hashlib" },
    { ALG_SHA512, 16,
      "c01d080efd492776a1c43bd23dd99d0a2e626d481e16782e75d54c25"
      "03b5dc32bd05f0f1ba33e568b88fd2d970929b719ecbb152f58f130a"
      "407c8830604b70ca",
      "hashlib" },
    { ALG_SHA512, 17,
      "130396a75cb483f2eee8c56d8a668bb3d2641f5243212c0bee2bd33d"
      "a096ad9eb8179fe18f9eaacf76e09fae9de4c3f14ba13341e345be05"
      "bf76c182cc3468cb",
      "hashlib" },
    { ALG_SHA512, 18,
      "f241de612b01aa2fa3cf01531d2a8e5e17fc761dfd48a704a834a47f"
      "57d6eade7804ecc39be42fdef16ec6adeaf7c01c2fd0c4cc97d38609"
      "07cfa4a3b36d0c05",
      "hashlib" },
    { ALG_SHA512, 19,
      "828613968b501dc00a97e08c73b118aa8876c26b8aac93df128502ab"
      "360f91bab50a51e088769a5c1eff4782ace147dce364255419987637"
      "4291f5d921629502",
      "hashlib" },
    { ALG_SHA512, 20,
      "b73d1929aa615934e61a871596b3f3b33359f42b8175602e89f7e06e"
      "5f658a243667807ed300314b95cacdd579f3e33abdfbe351909519a8"
      "46d465c59582f321",
      "hashlib" },
    { ALG_SHA512, 21,
      "4f681e0bd53cda4b5a2041cc8a06f2eabde44fb16c951fbd5b87702f"
      "07aeab611565b19c47fde30587177ebb852e3971bbd8d3fd30da18d7"
      "1037dfbd98420429",
      "hashlib" },
    { ALG_SHA512, 9,
      "e718483d0ce769644e2e42c7bc15b4638e1f98b13b2044285632a803"
      "afa973ebde0ff244877ea60a4cb0432ce577c31beb009c5c2c49aa2e"
      "4eadb217ad8cc09b",
      "FIPS 180-4" },
};
typedef struct {
    QTestState *qts;
    const HashPart *p;
    unsigned hsdi, hsdo;
} Hash;

static char *hex(const uint8_t *b, size_t n)
{
    GString *s = g_string_new(NULL);

    for (size_t i = 0; i < n; i++) {
        g_string_append_printf(s, "%02x", b[i]);
    }
    return g_string_free(s, false);
}

static uint32_t rd(Hash *h, unsigned reg)
{
    return qtest_readl(h->qts, HASH_BASE + reg);
}

static void wr(Hash *h, unsigned reg, uint32_t v)
{
    qtest_writel(h->qts, HASH_BASE + reg, v);
}

static void expect(const HashPart *p, const HashVec *v, const char *what,
                   const uint8_t *got, size_t n)
{
    g_autofree char *g = hex(got, n);
    g_autofree char *gl = g_strdup_printf("hash %s %s %s %s (%s): %s",
                                          p->part, alg_name[v->alg],
                                          msgs[v->msg].name, what,
                                          v->source, g);
    g_autofree char *wl = g_strdup_printf("hash %s %s %s %s (%s): %s",
                                          p->part, alg_name[v->alg],
                                          msgs[v->msg].name, what,
                                          v->source, v->digest);

    g_assert_cmpstr(gl, ==, wl);
}

static size_t block_size(unsigned alg)
{
    return alg >= ALG_SHA384 ? 128 : 64;
}

/* The message and its padding: 0x80, zeros, the bit count (LE for MD5) */
static uint8_t *pad(const HashMsg *m, unsigned alg, size_t *len)
{
    size_t bs = block_size(alg), lenbytes = bs / 8;
    size_t n = strlen(m->str) * m->rep;
    size_t total = (n + 1 + lenbytes + bs - 1) / bs * bs;
    uint8_t *b = g_malloc0(total);
    uint64_t bits = (uint64_t)n * 8;

    for (unsigned i = 0; i < m->rep; i++) {
        memcpy(b + i * strlen(m->str), m->str, strlen(m->str));
    }
    b[n] = 0x80;
    if (alg == ALG_MD5) {
        stq_le_p(b + total - 8, bits);
    } else {
        stq_be_p(b + total - 8, bits);
    }
    *len = total;
    return b;
}

static void start(Hash *h, unsigned alg, uint32_t extra)
{
    wr(h, HSCR, HSCR_EN | HSCR_SEL(alg) | extra);
    wr(h, HSCR, rd(h, HSCR) | HSCR_INIT);
    wr(h, HSSR, HSSR_ORD | HSSR_MRD);
    g_assert_cmphex(rd(h, HSSR), ==, 0);
}

static void pio_run(Hash *h, const HashVec *v, const uint8_t *msg, size_t len,
                    uint8_t *out)
{
    unsigned words = alg_words[v->alg];
    size_t bs = block_size(v->alg);

    start(h, v->alg, v->alg == ALG_MD5 ? 0 : HSCR_DIRVS);
    for (size_t i = 0; i < len; i += 4) {
        wr(h, h->hsdi, ldl_le_p(msg + i));
        if ((i + 4) % bs == 0) {
            g_assert_cmphex(rd(h, HSSR) & HSSR_ORD, ==, HSSR_ORD);
            wr(h, HSSR, HSSR_ORD);
            g_assert_cmphex(rd(h, HSSR) & HSSR_ORD, ==, 0);
        }
    }
    for (unsigned i = 0; i < words; i++) {
        uint32_t w = rd(h, h->hsdo);

        if (v->alg == ALG_MD5) {
            stl_le_p(out + 4 * i, w);
        } else {
            stl_be_p(out + 4 * i, w);
        }
    }
}

static void dma_chunk(Hash *h, size_t off, size_t bytes)
{
    wr(h, HSSR, HSSR_ORD | HSSR_MRD);
    wr(h, HSSA, SRC + off);
    if (h->p->t33) {
        wr(h, HSTC, bytes / 4);
        wr(h, HSBNUM_T33, bytes / 16);
    } else {
        wr(h, HSTC, bytes / 64);
    }
    wr(h, HSCR, rd(h, HSCR) | HSCR_DMAE);
    wr(h, HSCR, rd(h, HSCR) | HSCR_DMAS);
    g_assert_cmphex(rd(h, HSSR) & (HSSR_ORD | HSSR_MRD), ==,
                    HSSR_ORD | HSSR_MRD);
}

static void dma_run(Hash *h, const HashVec *v, const uint8_t *msg, size_t len,
                    unsigned chunks, uint8_t *out)
{
    unsigned words = alg_words[v->alg];
    size_t bs = block_size(v->alg);
    size_t first = chunks == 1 ? len : len / 2 / bs * bs;

    qtest_memwrite(h->qts, SRC, msg, len);
    start(h, v->alg, v->alg == ALG_MD5 ? 0 : HSCR_DIRVS | HSCR_DORVS);
    dma_chunk(h, 0, first);
    if (first < len) {
        dma_chunk(h, first, len - first);
    }
    for (unsigned i = 0; i < words; i++) {
        stl_le_p(out + 4 * i, rd(h, h->hsdo));
    }
}

static void test_part(const void *data)
{
    const HashPart *p = data;
    Hash h = { .p = p, .hsdi = p->t33 ? 0x18 : 0x14,
               .hsdo = p->t33 ? 0x1c : 0x18 };

    h.qts = qtest_initf("-machine %s", p->machine);
    g_assert_cmphex(rd(&h, HSCR), ==, 0x30000000);

    for (int i = 0; i < ARRAY_SIZE(vecs); i++) {
        const HashVec *v = &vecs[i];
        const HashMsg *m = &msgs[v->msg];
        size_t len, dlen = alg_words[v->alg] * 4;
        g_autofree uint8_t *msg = pad(m, v->alg, &len);
        uint8_t out[64];

        g_assert_cmpuint(strlen(v->digest), ==, dlen * 2);
        if (len < 4096) {
            memset(out, 0xa5, sizeof(out));
            pio_run(&h, v, msg, len, out);
            expect(p, v, "registers", out, dlen);
        }
        memset(out, 0xa5, sizeof(out));
        dma_run(&h, v, msg, len, 1, out);
        expect(p, v, "dma", out, dlen);
        if (len > 2 * block_size(v->alg)) {
            memset(out, 0xa5, sizeof(out));
            dma_run(&h, v, msg, len, 2, out);
            expect(p, v, "dma in two starts", out, dlen);
        }
    }
    qtest_quit(h.qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    for (int i = 0; i < ARRAY_SIZE(parts); i++) {
        g_autofree char *path = g_strdup_printf("/ingenic-hash/%s",
                                                parts[i].part);

        qtest_add_data_func(path, &parts[i], test_part);
    }
    return g_test_run();
}
