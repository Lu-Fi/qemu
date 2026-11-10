/*
 * QTest testcase for the Ingenic RSA accelerator
 *
 * Modular exponentiation with 1024 and 2048-bit moduli, driven as U-Boot's
 * jz_rsa_PRJ007 does: enable, the key length in CTRL, 64 exponent words
 * (zeros, then the exponent), the modulus words most significant first,
 * the PERS/PERD/PERC handshake, the message words, RSAS and RSAD, the
 * result read least significant word first, then RSAC and disable. The
 * T33 has another protocol, which is run on it instead: the
 * message first, then exponent and modulus, N_CFG_DONE and INT_DONE in
 * RSAINFO, and RSACLR.
 *
 * The expected values are computed independently of the model with
 * Python's pow(m, e, n) from the keys listed (made from a fixed seed,
 * e = 65537 and 3), plus the textbook p = 61, q = 53 key. A private
 * exponent of a full modulus's width goes through all 64 exponent words
 * (the "-wide-exponent" cases), as does a private exponent of 31 bits and
 * the textbook one. The T33 runs its U-Boot's sequence: RSAINFO's
 * N_CFG_DONE (0x10) after the modulus, then the result with DONE.
 *
 * Copyright (C) 2026 Alfonso Gamboa <gtxent@gmail.com>
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"
#include "libqtest.h"

#define RSA_BASE        0x134c0000

#define RSAC            0x00
#define RSAE            0x04
#define RSAN            0x08
#define RSAM            0x0c
#define RSAP            0x10
#define RSACLR          0x14
#define RSAINFO         0x18

#define RSAC_EN         BIT(0)
#define RSAC_PERS       BIT(1)
#define RSAC_PERD       BIT(2)
#define RSAC_PERC       BIT(3)
#define RSAC_RSAS       BIT(4)
#define RSAC_RSAD       BIT(5)
#define RSAC_RSAC       BIT(6)
#define RSAC_RSA_2048   BIT(7)
#define RSAC_RSA_1024_8 (32 << 4)       /* the T33's */
#define RSAC_RSA_2048_8 (64 << 4)
#define RSAINFO_N_DONE  BIT(4)          /* the T33's */
#define RSAINFO_DONE    BIT(0)
#define RSACLR_N        BIT(4)
#define RSACLR_DONE     BIT(0)

#define MAXWORDS        64

typedef struct {
    const char *name;
    unsigned bits;
    bool wide;                  /* the exponent is wider than a word */
    const char *n;
    const char *e;
    const char *in;
    const char *out;
    const char *how;            /* how the answer was computed */
} RsaVec;

static const RsaVec vecs[] = {
    { "rsa1024-e65537-encrypt", 1024, false,
      "b0421fdbb15c8590d8825191e39bb7c6272ee41a5baaa872387b3437"
      "b475799fdc22c1ac34977c58982e90f4c7a9a9919d4c49ad882cdaac"
      "f248e7919f8c78fd7ec1db92b751f88cf8e6b526648624d3ce454a74"
      "a2f0cde844bc8276eb054f3cd7e0de707b8dfdf41b8dcbdc16e2e302"
      "1642db2d3f3e42fd6157b09c4284ff45",
      "10001",
      "62b756a67844684a7cc89334f790d3aeb1153cee21b1add5e4387b7c"
      "35608f30dc68d0812e0d79ebc100a8e10c6c7ab0874f460560299cb7"
      "fcc3608107d37a8aa72cfe316d942bd78fc98acbd657c21329139a7e"
      "e7f2b1528be0a77717954c70f60223164a4967a981fe6b575d920740"
      "6e12ee1f3b089c228e69906119e6d6e7",
      "952f40f9f9a7d9ddc6c96aaf60b53943cc57aa6a1e0ae973c713f556"
      "ebb6dceecc100df98bb6c773ba2d4329e5bd5fe95aa18323e1b4c8f9"
      "d420eaa214fe3699744d826b2c7c5f0f882e72751f5a48e1acd99726"
      "79c029063ce58cda23c79aa5607715770164d3b8e4fb84be1b5a0797"
      "86cd8c5e1b73f17809a49c71c6dbcc87",
      "pow(m, 65537, n)" },
    { "rsa1024-d-decrypt", 1024, true,
      "b0421fdbb15c8590d8825191e39bb7c6272ee41a5baaa872387b3437"
      "b475799fdc22c1ac34977c58982e90f4c7a9a9919d4c49ad882cdaac"
      "f248e7919f8c78fd7ec1db92b751f88cf8e6b526648624d3ce454a74"
      "a2f0cde844bc8276eb054f3cd7e0de707b8dfdf41b8dcbdc16e2e302"
      "1642db2d3f3e42fd6157b09c4284ff45",
      "565f20e37467656054688ff650b5f6f9582c27452f5452951e4e6989"
      "3e9a2e8e82be596fbc99db2a42f0d6c1ccb1bc5189e65f5875199dd3"
      "102b5fa6c1ab8181cf1303a33cea3941639d2884608340a013668999"
      "f643d668e0b468ccdb21d84e8dcbc7152599e10970d09a757a231380"
      "08ea4ae0dfc8330eb69581dd5e82b665",
      "952f40f9f9a7d9ddc6c96aaf60b53943cc57aa6a1e0ae973c713f556"
      "ebb6dceecc100df98bb6c773ba2d4329e5bd5fe95aa18323e1b4c8f9"
      "d420eaa214fe3699744d826b2c7c5f0f882e72751f5a48e1acd99726"
      "79c029063ce58cda23c79aa5607715770164d3b8e4fb84be1b5a0797"
      "86cd8c5e1b73f17809a49c71c6dbcc87",
      "62b756a67844684a7cc89334f790d3aeb1153cee21b1add5e4387b7c"
      "35608f30dc68d0812e0d79ebc100a8e10c6c7ab0874f460560299cb7"
      "fcc3608107d37a8aa72cfe316d942bd78fc98acbd657c21329139a7e"
      "e7f2b1528be0a77717954c70f60223164a4967a981fe6b575d920740"
      "6e12ee1f3b089c228e69906119e6d6e7",
      "pow(c, d, n)" },
    { "rsa2048-e65537-encrypt", 2048, false,
      "ca570e852d2236917aa42678844b337fe22393e7cc67fd4ce0ee3ac9"
      "13c7d64f750873b9d80545cd06fd3cb2ceec94757e2913171c48ab54"
      "ef0fa4959b291230727229dd27d840465478f09d22c8567785ec075f"
      "deb8167a9b7b49a75dd974967cca1f22cc454779476a90420bb9acda"
      "75d016550cfcd41110693afaaa1b3619cc417ec3c2486ee08f8bd3ee"
      "445d5e282aeaf0a3ae90bcbc4ab0a733a8d8edf44013fc0fb9ae4a07"
      "9a89b22d8e5dc8e4ffa5d69306baa448736be0986eafb0e8c7070672"
      "4a2a8e435f8dba98b3c335803bb44066c02b8b50839d4d0fbd934d83"
      "20c6f974dedc7e0ccc1b7558c25584b5bce43658422350d3f60d6949"
      "26427b03",
      "10001",
      "69964baeff308398470d2c4be9c09b4bd3eb14fd5825bfd0714f708c"
      "e385019dae6d6b13651007f432cb9f1e011ffc290aded5d75b630bd3"
      "115dc202789fc3d94b6f895997e9a63ba9fa0d2e56ed76e560ec1479"
      "5f395adaef620a07728d6ee79679538c604af0c4a5a16c83326a11dc"
      "1e2a1a17ae7caebd6e74cbfe1e9d071c63bc710a045563707c0db088"
      "21a91957c6014cf89f6d0d123a0f8318c1752169afc18b5c5f2de7e3"
      "51199ffde74c3cc88627d65f4873fe74d8077d0d4679a1e8d889742b"
      "0db186e6b07f8b14c9a54b61a29c4614edad802f8f7e7a247bf3bacf"
      "ee29cd4636b6385cc0b933ec66e2776f447904598287e5c0d6ad09b0"
      "e5e9aa56",
      "2e81bfa49f90ca0a957bca7581a7fb3afc1af089fac24ce450718804"
      "5d28b9d0fc27dbf406e7f67b2d48d89d15f9841ac999af61c39e9974"
      "24d91e927ff536bf4092ca8af8f02ed564b76957c95d717d75f9e168"
      "229938d24a53f2226ffb73d9358e95e308291f19e9e85a45c8cba11f"
      "09dbc2539ff8500c83e9a0a70ecfb15c4166f22fe37f924004079980"
      "2220884994d807ee74e599b30c33bf9393e84b47d992c36506c310fa"
      "ec580f5dd435282e8f967c797f5395892abe12c4cc1e5c84231cbcf0"
      "2e6c007649d622e89cba52e990a01c4e29218c9421f738d971656f13"
      "33e94526eec0c6d7a1d3c7ecd6e2721bdca7c4bfde962704985ec862"
      "4ca852f1",
      "pow(m, 65537, n)" },
    { "rsa2048-d-decrypt", 2048, true,
      "ca570e852d2236917aa42678844b337fe22393e7cc67fd4ce0ee3ac9"
      "13c7d64f750873b9d80545cd06fd3cb2ceec94757e2913171c48ab54"
      "ef0fa4959b291230727229dd27d840465478f09d22c8567785ec075f"
      "deb8167a9b7b49a75dd974967cca1f22cc454779476a90420bb9acda"
      "75d016550cfcd41110693afaaa1b3619cc417ec3c2486ee08f8bd3ee"
      "445d5e282aeaf0a3ae90bcbc4ab0a733a8d8edf44013fc0fb9ae4a07"
      "9a89b22d8e5dc8e4ffa5d69306baa448736be0986eafb0e8c7070672"
      "4a2a8e435f8dba98b3c335803bb44066c02b8b50839d4d0fbd934d83"
      "20c6f974dedc7e0ccc1b7558c25584b5bce43658422350d3f60d6949"
      "26427b03",
      "49e74c3d6643c5cc930fdba14f28ac71bc262482988d2ede629580d1"
      "2ab165dc75b1959b3e84867f215b634d27f90f2c6edd0fc4f426b2b5"
      "7defa7bca1047a4949b3db0894e20e6fe1b09ea9b627983565abd4f1"
      "b64664a7799ca945412908c56264ecc467f9b1e6612851f25d97fec3"
      "781e7dd9a8b9bbb753d82b9b9c7ffddcfd14e99a12b9328348e2d6b2"
      "dfd52d2b8fc2152312bc7e6d8d886016a1623a2fdd2f649264919e95"
      "41e749123b9d153598ba366bdd0c6c1491c4766197e660e66739204c"
      "bc6cb9942ecc206f7116f9efd55e9102d0e5b499dd0f4eeb3ed224e2"
      "497dcdf8d0e4f4fba3c59e1550f6e568cd91b520b2b8d5f2fb8a42c0"
      "be4b8f31",
      "2e81bfa49f90ca0a957bca7581a7fb3afc1af089fac24ce450718804"
      "5d28b9d0fc27dbf406e7f67b2d48d89d15f9841ac999af61c39e9974"
      "24d91e927ff536bf4092ca8af8f02ed564b76957c95d717d75f9e168"
      "229938d24a53f2226ffb73d9358e95e308291f19e9e85a45c8cba11f"
      "09dbc2539ff8500c83e9a0a70ecfb15c4166f22fe37f924004079980"
      "2220884994d807ee74e599b30c33bf9393e84b47d992c36506c310fa"
      "ec580f5dd435282e8f967c797f5395892abe12c4cc1e5c84231cbcf0"
      "2e6c007649d622e89cba52e990a01c4e29218c9421f738d971656f13"
      "33e94526eec0c6d7a1d3c7ecd6e2721bdca7c4bfde962704985ec862"
      "4ca852f1",
      "69964baeff308398470d2c4be9c09b4bd3eb14fd5825bfd0714f708c"
      "e385019dae6d6b13651007f432cb9f1e011ffc290aded5d75b630bd3"
      "115dc202789fc3d94b6f895997e9a63ba9fa0d2e56ed76e560ec1479"
      "5f395adaef620a07728d6ee79679538c604af0c4a5a16c83326a11dc"
      "1e2a1a17ae7caebd6e74cbfe1e9d071c63bc710a045563707c0db088"
      "21a91957c6014cf89f6d0d123a0f8318c1752169afc18b5c5f2de7e3"
      "51199ffde74c3cc88627d65f4873fe74d8077d0d4679a1e8d889742b"
      "0db186e6b07f8b14c9a54b61a29c4614edad802f8f7e7a247bf3bacf"
      "ee29cd4636b6385cc0b933ec66e2776f447904598287e5c0d6ad09b0"
      "e5e9aa56",
      "pow(c, d, n)" },
    { "rsa1024-e3-encrypt", 1024, false,
      "b578f75d5a5ddb3f16b9e9fc940d22b14dba3f40e52471197282581f"
      "4af48295f37cf9b00dcac0c6e4ec57894ec4fba59c05ca6636a29911"
      "1a7c4aa6117384447de1aa3f5aac26c58c783f63c00a7a2d3aeedbf5"
      "fdd1eebdca7c3ae3d587d178c6fee65f4717090a9b32e8cd8813ded7"
      "9fcf208556e63ff5c6ad3e96e01bd1f3",
      "3",
      "a29f23d44922a69907dd9c95657962895b0c552e964c32df1c0419db"
      "6e46e26a55dfe720e5da4bb6d1565a163a019d74048fa722fa843863"
      "de37e7afd98cfe2b98a434587b785da2252e2fcad17c0df2465c64fb"
      "f0aa9422dbe89803c5cdf34b3fe38694af8b536812db3dc9eff4cf21"
      "3ac9c1bb7552baa4d6ebf163289887f3",
      "3c0a726d7c2e380d68b560538387b8accdd429d738db4773a426b646"
      "1c7595f463ec02dd50fbca938eaf60e74c20fccaf8a552911999ebf8"
      "e0f87582a404884af4ce18c4098b1be5d4221094412e641efb4c5c95"
      "2061e2cda6fc50f78020a04d524c6d91ebae4c218ec42fd6c1a3fe53"
      "3aacb60008f331c2159f5ac1910e938b",
      "pow(m, 3, n)" },
    { "rsa1024-e3-decrypt", 1024, true,
      "b578f75d5a5ddb3f16b9e9fc940d22b14dba3f40e52471197282581f"
      "4af48295f37cf9b00dcac0c6e4ec57894ec4fba59c05ca6636a29911"
      "1a7c4aa6117384447de1aa3f5aac26c58c783f63c00a7a2d3aeedbf5"
      "fdd1eebdca7c3ae3d587d178c6fee65f4717090a9b32e8cd8813ded7"
      "9fcf208556e63ff5c6ad3e96e01bd1f3",
      "78fb4f939193e77f647bf1530d5e1720de7c2a2b436da0bba1ac3abf"
      "874dac63f7a8a6755e872b2f43483a5b89d8a7c3bd59319979c1bb60"
      "bc52dc6eb64d02d7337bdf13f2f20c30a4a46714d11e7a0272e7aaef"
      "a7997900fbba5c04a157f189e3620c05531947997bd5ab2094c74f7d"
      "dfe1ad373986ff6440f6e72a0c669d0b",
      "3c0a726d7c2e380d68b560538387b8accdd429d738db4773a426b646"
      "1c7595f463ec02dd50fbca938eaf60e74c20fccaf8a552911999ebf8"
      "e0f87582a404884af4ce18c4098b1be5d4221094412e641efb4c5c95"
      "2061e2cda6fc50f78020a04d524c6d91ebae4c218ec42fd6c1a3fe53"
      "3aacb60008f331c2159f5ac1910e938b",
      "a29f23d44922a69907dd9c95657962895b0c552e964c32df1c0419db"
      "6e46e26a55dfe720e5da4bb6d1565a163a019d74048fa722fa843863"
      "de37e7afd98cfe2b98a434587b785da2252e2fcad17c0df2465c64fb"
      "f0aa9422dbe89803c5cdf34b3fe38694af8b536812db3dc9eff4cf21"
      "3ac9c1bb7552baa4d6ebf163289887f3",
      "pow(c, d, n)" },
    { "rsa1024-small-d-decrypt", 1024, false,
      "ce4a61584138b07f6998255b8e837581aac14f2a7a3aedb9780bac71"
      "aece2b75edb39c40aa854578bd9a7f192658bea8e55a93dab77d49d6"
      "6e683368ce172c74f4554fb88eec4e7c45ff93e9953d246661190f44"
      "6aeafd99524d0960dfd1eab12b144d88d2b20aa3891f4d7e07f9e1be"
      "eefbe83b876df94f6f9ef37f1eebe1b7",
      "493a26c5",
      "516682703cc3f556dd32ab975e0ca09792a264883109d916603c8520"
      "f22d7d86df0537931fe00b356063d6fc1d7199d6e2bce2501fe5df3e"
      "29c4e17c8bb31350f24be26b254c1e7093617bc8f9d19e148f3babc1"
      "f2da52b333d19a0c4110191bc817763443b008534aba519e24b210d6"
      "5e5b4d93482a8133c7581101be43565a",
      "b9945ab90f252f00315f3292cda100b2ece92a77695f7d61e580327b"
      "25511f7e1ca9f530a80caa6beb23c5f704bc1914f7a6665ebc4d85f9"
      "ebd2e29354ce76d08f9cc6ef92c1ae9ef0b46df8a6e1765cef738930"
      "7b330c2deab1e8ec666fda92b5a65a87c452ab77a885dbd2ec22f3e0"
      "66f057721ae289d16338ef69d30b2306",
      "pow(c, d, n), d of 31 bits" },
    { "textbook-3233-e17-encrypt", 1024, false,
      "00000000000000000000000000000000000000000000000000000000"
      "00000000000000000000000000000000000000000000000000000000"
      "00000000000000000000000000000000000000000000000000000000"
      "00000000000000000000000000000000000000000000000000000000"
      "00000000000000000000000000000ca1",
      "11",
      "00000000000000000000000000000000000000000000000000000000"
      "00000000000000000000000000000000000000000000000000000000"
      "00000000000000000000000000000000000000000000000000000000"
      "00000000000000000000000000000000000000000000000000000000"
      "00000000000000000000000000000041",
      "00000000000000000000000000000000000000000000000000000000"
      "00000000000000000000000000000000000000000000000000000000"
      "00000000000000000000000000000000000000000000000000000000"
      "00000000000000000000000000000000000000000000000000000000"
      "00000000000000000000000000000ae6",
      "the textbook p=61, q=53 key, n padded to 1024 bits" },
    { "textbook-3233-d2753-decrypt", 1024, false,
      "00000000000000000000000000000000000000000000000000000000"
      "00000000000000000000000000000000000000000000000000000000"
      "00000000000000000000000000000000000000000000000000000000"
      "00000000000000000000000000000000000000000000000000000000"
      "00000000000000000000000000000ca1",
      "ac1",
      "00000000000000000000000000000000000000000000000000000000"
      "00000000000000000000000000000000000000000000000000000000"
      "00000000000000000000000000000000000000000000000000000000"
      "00000000000000000000000000000000000000000000000000000000"
      "00000000000000000000000000000ae6",
      "00000000000000000000000000000000000000000000000000000000"
      "00000000000000000000000000000000000000000000000000000000"
      "00000000000000000000000000000000000000000000000000000000"
      "00000000000000000000000000000000000000000000000000000000"
      "00000000000000000000000000000041",
      "the textbook p=61, q=53 key, n padded to 1024 bits" },
};

typedef struct {
    QTestState *qts;
} Rsa;

static uint32_t rd(Rsa *r, unsigned reg)
{
    return qtest_readl(r->qts, RSA_BASE + reg);
}

static void wr(Rsa *r, unsigned reg, uint32_t v)
{
    qtest_writel(r->qts, RSA_BASE + reg, v);
}

/* A hex number as words, most significant first, in 'n' words */
static void to_words(const char *hex, uint32_t *w, unsigned n)
{
    size_t len = strlen(hex);
    g_autofree char *padded = g_malloc0(n * 8 + 1);

    g_assert_cmpuint(len, <=, n * 8);
    memset(padded, '0', n * 8);
    memcpy(padded + n * 8 - len, hex, len);
    for (unsigned i = 0; i < n; i++) {
        char word[9] = { 0 };

        memcpy(word, padded + 8 * i, 8);
        g_assert_cmpint(sscanf(word, "%x", &w[i]), ==, 1);
    }
}

static char *words_hex(const uint32_t *w, unsigned n)
{
    GString *s = g_string_new(NULL);

    for (unsigned i = 0; i < n; i++) {
        g_string_append_printf(s, "%08x", w[i]);
    }
    return g_string_free(s, false);
}

static void rsac_or(Rsa *r, uint32_t bits)
{
    wr(r, RSAC, rd(r, RSAC) | bits);
}

static void rsac_clear(Rsa *r, uint32_t bits)
{
    wr(r, RSAC, rd(r, RSAC) & ~bits);
}

/* U-Boot's jz_rsa_2048_init, _update and _final, for any key length */
static void jz_run(Rsa *r, const RsaVec *v, const uint32_t *e,
                       const uint32_t *n, const uint32_t *m, uint32_t *out)
{
    unsigned klen = v->bits / 32;

    rsac_or(r, RSAC_EN);
    if (klen == 32) {
        rsac_clear(r, RSAC_RSA_2048);
    } else {
        rsac_or(r, RSAC_RSA_2048);
    }
    for (unsigned i = 0; i < MAXWORDS; i++) {
        wr(r, RSAE, e[i]);
    }
    for (unsigned i = 0; i < klen; i++) {
        wr(r, RSAN, n[MAXWORDS - klen + i]);
    }
    rsac_or(r, RSAC_PERS);
    g_assert_cmphex(rd(r, RSAC) & RSAC_PERD, ==, RSAC_PERD);
    rsac_clear(r, RSAC_PERS);
    rsac_or(r, RSAC_PERC);
    rsac_clear(r, RSAC_PERC);

    for (unsigned i = 0; i < klen; i++) {
        wr(r, RSAM, m[MAXWORDS - klen + i]);
    }
    rsac_or(r, RSAC_RSAS);
    g_assert_cmphex(rd(r, RSAC) & RSAC_RSAD, ==, RSAC_RSAD);
    rsac_clear(r, RSAC_RSAS);
    for (unsigned i = 0; i < klen; i++) {
        out[klen - 1 - i] = rd(r, RSAP);
    }
    rsac_or(r, RSAC_RSAC);
    rsac_clear(r, RSAC_RSAC);
    rsac_clear(r, RSAC_EN);
}

/* U-Boot's jz_rsa_PRJ008 do_rsa_2048: message, exponent, modulus first */
static void t33_run(Rsa *r, const RsaVec *v, const uint32_t *e,
                       const uint32_t *n, const uint32_t *m, uint32_t *out)
{
    unsigned klen = v->bits / 32;
    uint32_t c = rd(r, RSAC) & ~(0x7fu << 4);

    wr(r, RSAC, c);
    wr(r, RSAC, c | (klen == 32 ? RSAC_RSA_1024_8 : RSAC_RSA_2048_8));
    for (unsigned i = 0; i < MAXWORDS - klen; i++) {
        wr(r, RSAM, 0);
    }
    for (unsigned i = 0; i < klen; i++) {
        wr(r, RSAM, m[MAXWORDS - klen + i]);
    }
    for (unsigned i = 0; i < MAXWORDS; i++) {
        wr(r, RSAE, e[i]);
    }
    for (unsigned i = 0; i < klen; i++) {
        wr(r, RSAN, n[MAXWORDS - klen + i]);
    }
    /* U-Boot polls for it */
    g_assert_cmphex(rd(r, RSAINFO) & RSAINFO_N_DONE, ==, RSAINFO_N_DONE);
    wr(r, RSACLR, rd(r, RSACLR) | RSACLR_N);
    rsac_or(r, RSAC_EN);
    g_assert_cmphex(rd(r, RSAINFO) & RSAINFO_DONE, ==, RSAINFO_DONE);
    for (unsigned i = 0; i < klen; i++) {
        out[klen - 1 - i] = rd(r, RSAP);
    }
    wr(r, RSACLR, rd(r, RSACLR) | RSACLR_DONE);
    rsac_clear(r, RSAC_EN);
}

static void run(Rsa *r, bool t33, const RsaVec *v, const uint32_t *e,
                const uint32_t *n, const uint32_t *m, uint32_t *out)
{
    if (t33) {
        t33_run(r, v, e, n, m, out);
    } else {
        jz_run(r, v, e, n, m, out);
    }
}

static void check(Rsa *r, const char *part, const RsaVec *v, const char *what,
                  const uint32_t *got, const char *want)
{
    unsigned klen = v->bits / 32;
    g_autofree char *g = words_hex(got, klen);
    g_autofree char *w = NULL;
    uint32_t ww[MAXWORDS];
    g_autofree char *gl = NULL;
    g_autofree char *wl = NULL;

    to_words(want, ww, klen);
    w = words_hex(ww, klen);
    gl = g_strdup_printf("rsa %s %s %s (%s): %s", part, v->name, what,
                         v->how, g);
    wl = g_strdup_printf("rsa %s %s %s (%s): %s", part, v->name, what,
                         v->how, w);
    g_assert_cmpstr(gl, ==, wl);
}

typedef struct {
    const char *part;
    const char *machine;
    bool t33;
} RsaPart;

static const RsaPart parts[] = {
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

/* Round trips: the hardware's ciphertext through the hardware, back */
static const unsigned pairs[][2] = { { 0, 1 }, { 2, 3 }, { 4, 5 } };

typedef struct {
    const RsaPart *p;
    bool wide;                  /* the cases with a full-width exponent */
} RsaCase;

static RsaCase cases[2 * ARRAY_SIZE(parts)];

static void test_part(const void *data)
{
    const RsaCase *c = data;
    const RsaPart *p = c->p;
    Rsa r = { 0 };

    r.qts = qtest_initf("-machine %s", p->machine);
    g_assert_cmphex(rd(&r, RSAC), ==, 0x00030200);

    for (int i = 0; i < ARRAY_SIZE(vecs); i++) {
        const RsaVec *v = &vecs[i];
        uint32_t e[MAXWORDS], n[MAXWORDS], m[MAXWORDS], out[MAXWORDS];

        if (v->wide != c->wide) {
            continue;
        }
        to_words(v->e, e, MAXWORDS);
        to_words(v->n, n, MAXWORDS);
        to_words(v->in, m, MAXWORDS);
        memset(out, 0xa5, sizeof(out));
        run(&r, p->t33, v, e, n, m, out);
        check(&r, p->part, v, "operation", out, v->out);
    }

    for (int i = 0; c->wide && i < ARRAY_SIZE(pairs); i++) {
        const RsaVec *enc = &vecs[pairs[i][0]], *dec = &vecs[pairs[i][1]];
        uint32_t e[MAXWORDS], n[MAXWORDS], m[MAXWORDS], ct[MAXWORDS];
        uint32_t back[MAXWORDS];
        RsaVec step = *dec;

        to_words(enc->e, e, MAXWORDS);
        to_words(enc->n, n, MAXWORDS);
        to_words(enc->in, m, MAXWORDS);
        memset(ct, 0, sizeof(ct));
        run(&r, p->t33, enc, e, n, m, ct + MAXWORDS - enc->bits / 32);
        to_words(dec->e, e, MAXWORDS);
        memset(back, 0, sizeof(back));
        run(&r, p->t33, &step, e, n, ct, back);
        step.name = g_strdup_printf("%s round trip", dec->name);
        check(&r, p->part, &step, "of the hardware's ciphertext", back,
              enc->in);
        g_free((char *)step.name);
    }
    qtest_quit(r.qts);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    for (int i = 0; i < ARRAY_SIZE(parts); i++) {
        for (int w = 0; w < 2; w++) {
            RsaCase *c = &cases[2 * i + w];
            g_autofree char *path = g_strdup_printf("/ingenic-rsa/%s%s",
                parts[i].part, w ? "-wide-exponent" : "");

            c->p = &parts[i];
            c->wide = w;
            qtest_add_data_func(path, c, test_part);
        }
    }
    return g_test_run();
}
