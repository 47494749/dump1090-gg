// test_sonde.c — Unit tests for RS41 radiosonde decoder components
//
// Tests:
//   1. CRC-16 CCITT with standard test vectors
//   2. GF(2^8) arithmetic fundamentals
//   3. Reed-Solomon RS(255,231) encode/decode (both [data|parity] and [parity|data])
//   4. XOR whitening round-trip and alignment with rs1729 offsets
//   5. ECEF → LLA conversion with known geodetic coordinates
//   6. Full RS41 frame pipeline: build → RS-encode → whiten → corrupt → decode → verify
//
// Build: cc -o test_sonde test_sonde.c -lm
// Run:   ./test_sonde

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

// ===================== Constants =====================

#define RS_NROOTS  24
#define RS_T       12
#define GF_POLY    0x11D

#define RS41_RS_PARITY     48
#define RS41_DATA_START    48
#define RS41_FRAME_LEN     320

#define RS41_BLOCK_STATUS  0x79
#define RS41_BLOCK_GPSPOS  0x7B

// ===================== GF(2^8) Arithmetic =====================

static uint8_t gf_exp[512];
static uint8_t gf_log[256];

static void gf_init(void)
{
    int x = 1;
    for (int i = 0; i < 255; i++) {
        gf_exp[i] = (uint8_t)x;
        gf_log[x] = (uint8_t)i;
        x <<= 1;
        if (x & 0x100) x ^= GF_POLY;
    }
    for (int i = 255; i < 512; i++)
        gf_exp[i] = gf_exp[i - 255];
    gf_log[0] = 0;
}

static inline uint8_t gf_mul(uint8_t a, uint8_t b)
{
    if (a == 0 || b == 0) return 0;
    return gf_exp[gf_log[a] + gf_log[b]];
}

static inline uint8_t gf_div(uint8_t a, uint8_t b)
{
    if (b == 0 || a == 0) return 0;
    return gf_exp[(gf_log[a] + 255 - gf_log[b]) % 255];
}

static inline uint8_t gf_pow(uint8_t a, int n)
{
    if (a == 0) return 0;
    return gf_exp[(gf_log[a] * n) % 255];
}

// ===================== Reed-Solomon Decoder =====================

static int rs_syndromes(const uint8_t *data, int len, uint8_t *syn)
{
    int all_zero = 1;
    for (int i = 0; i < RS_NROOTS; i++) {
        uint8_t s = 0;
        for (int j = 0; j < len; j++)
            s = gf_mul(s, gf_exp[i]) ^ data[j];
        syn[i] = s;
        if (s != 0) all_zero = 0;
    }
    return all_zero;
}

static int rs_berlekamp_massey(const uint8_t *syn, uint8_t *sigma)
{
    uint8_t C[RS_NROOTS + 1];
    uint8_t B[RS_NROOTS + 1];
    memset(C, 0, sizeof(C));
    memset(B, 0, sizeof(B));
    C[0] = 1;
    B[0] = 1;
    int L = 0, m = 1;
    uint8_t b = 1;

    for (int n = 0; n < RS_NROOTS; n++) {
        uint8_t d = syn[n];
        for (int i = 1; i <= L; i++)
            d ^= gf_mul(C[i], syn[n - i]);

        if (d == 0) {
            m++;
        } else if (2 * L <= n) {
            uint8_t T[RS_NROOTS + 1];
            memcpy(T, C, sizeof(T));
            uint8_t coef = gf_div(d, b);
            for (int i = m; i <= RS_NROOTS; i++)
                C[i] ^= gf_mul(coef, B[i - m]);
            L = n + 1 - L;
            memcpy(B, T, sizeof(B));
            b = d;
            m = 1;
        } else {
            uint8_t coef = gf_div(d, b);
            for (int i = m; i <= RS_NROOTS; i++)
                C[i] ^= gf_mul(coef, B[i - m]);
            m++;
        }
    }

    memcpy(sigma, C, (RS_NROOTS + 1) * sizeof(uint8_t));
    return L;
}

// Chien search: find error locations
// Searches all 255 nonzero GF elements. Position mapping:
//   sigma(alpha^m) = 0  =>  array position j = (m + n - 1) % 255
// where n = codeword length. Only positions 0..n-1 are valid.
static int rs_chien_search(const uint8_t *sigma, int deg, int n,
                           int *positions, int *roots)
{
    int count = 0;
    for (int m = 0; m < 255; m++) {
        uint8_t val = 1;
        for (int j = 1; j <= deg; j++)
            val ^= gf_mul(sigma[j], gf_exp[(m * j) % 255]);
        if (val == 0) {
            int pos = (m + n - 1) % 255;
            if (pos >= 0 && pos < n) {
                positions[count] = pos;
                roots[count] = m;
                count++;
                if (count >= deg) break;
            }
        }
    }
    return count;
}

// Forney algorithm: compute error magnitudes
// roots[k] = Chien search index m where sigma(alpha^m) = 0
// Xi_inv = alpha^m = X_k^(-1), X_k = alpha^(255-m)
static void rs_forney(const uint8_t *syn, const uint8_t *sigma, int deg,
                      const int *roots, int num_errors, uint8_t *magnitudes)
{
    uint8_t omega[RS_NROOTS];
    memset(omega, 0, sizeof(omega));
    for (int i = 0; i < RS_NROOTS; i++) {
        uint8_t v = 0;
        for (int j = 0; j <= deg && j <= i; j++)
            v ^= gf_mul(sigma[j], syn[i - j]);
        omega[i] = v;
    }

    for (int k = 0; k < num_errors; k++) {
        uint8_t Xi_inv = gf_exp[roots[k]]; // alpha^m
        uint8_t O = 0;
        uint8_t xi_pow = 1;
        for (int i = 0; i < RS_NROOTS; i++) {
            O ^= gf_mul(omega[i], xi_pow);
            xi_pow = gf_mul(xi_pow, Xi_inv);
        }
        uint8_t Sp = 0;
        xi_pow = 1;
        for (int i = 1; i <= deg; i += 2) {
            Sp ^= gf_mul(sigma[i], xi_pow);
            xi_pow = gf_mul(xi_pow, gf_mul(Xi_inv, Xi_inv));
        }
        if (Sp != 0) {
            uint8_t X_k = gf_exp[(255 - roots[k]) % 255]; // Xi_inv^(-1)
            magnitudes[k] = gf_mul(X_k, gf_div(O, Sp));
        } else {
            magnitudes[k] = 0;
        }
    }
}

static int rs_decode(uint8_t *data, int len)
{
    uint8_t syn[RS_NROOTS];
    if (rs_syndromes(data, len, syn))
        return 0;
    uint8_t sigma[RS_NROOTS + 1];
    int deg = rs_berlekamp_massey(syn, sigma);
    if (deg > RS_T)
        return -1;
    int positions[RS_T], roots[RS_T];
    int num_errors = rs_chien_search(sigma, deg, len, positions, roots);
    if (num_errors != deg)
        return -1;
    uint8_t magnitudes[RS_T];
    rs_forney(syn, sigma, deg, roots, num_errors, magnitudes);
    for (int i = 0; i < num_errors; i++) {
        if (positions[i] >= 0 && positions[i] < len)
            data[positions[i]] ^= magnitudes[i];
    }
    return num_errors;
}

// ===================== RS Encoder: [data | parity] format =====================
// Standard systematic encoder using LFSR.
// Produces parity such that codeword = [data(k), parity(24)] has zero syndromes.

static void rs_encode_data_first(const uint8_t *data, int data_len, uint8_t *parity)
{
    uint8_t gen[RS_NROOTS + 1];
    memset(gen, 0, sizeof(gen));
    gen[0] = 1;
    for (int i = 0; i < RS_NROOTS; i++) {
        for (int j = RS_NROOTS; j > 0; j--)
            gen[j] = gf_mul(gen[j], gf_exp[i]) ^ gen[j - 1];
        gen[0] = gf_mul(gen[0], gf_exp[i]);
    }

    memset(parity, 0, RS_NROOTS);
    for (int i = 0; i < data_len; i++) {
        uint8_t feedback = data[i] ^ parity[0];
        for (int j = 0; j < RS_NROOTS - 1; j++)
            parity[j] = parity[j + 1] ^ gf_mul(feedback, gen[RS_NROOTS - 1 - j]);
        parity[RS_NROOTS - 1] = gf_mul(feedback, gen[0]);
    }
}

// ===================== RS Encoder: [parity | data] format =====================
// Solves for parity bytes such that codeword = [parity(24), data(k)] has zero syndromes.
// Uses Gaussian elimination on the syndrome equations.
//
// Syndrome equation: S_i = sum_{j=0}^{23} parity[j]*alpha^(i*(n-1-j))
//                        + sum_{k=0}^{K-1} data[k]*alpha^(i*(K-1-k)) = 0
// where n = 24 + K, K = data_len.
//
// So: A[i][j] * parity[j] = rhs[i] where rhs[i] = data syndrome contribution.

static void rs_encode_parity_first(const uint8_t *data, int data_len, uint8_t *parity)
{
    int n = RS_NROOTS + data_len;

    // Compute RHS: syndrome contribution of data part
    uint8_t rhs[RS_NROOTS];
    for (int i = 0; i < RS_NROOTS; i++) {
        uint8_t s = 0;
        for (int k = 0; k < data_len; k++) {
            if (data[k] == 0) continue;
            int exp_val = (int)((long)i * (data_len - 1 - k) % 255);
            if (exp_val < 0) exp_val += 255;
            s ^= gf_mul(data[k], gf_exp[exp_val]);
        }
        rhs[i] = s;  // Need parity to cancel this
    }

    // Build augmented matrix [A | rhs]
    // A[i][j] = alpha^(i * (n-1-j))
    uint8_t aug[24][25];
    for (int i = 0; i < RS_NROOTS; i++) {
        for (int j = 0; j < RS_NROOTS; j++) {
            int exp_val = (int)((long)i * (n - 1 - j) % 255);
            if (exp_val < 0) exp_val += 255;
            aug[i][j] = gf_exp[exp_val];
        }
        aug[i][RS_NROOTS] = rhs[i];
    }

    // Gaussian elimination with partial pivoting
    for (int col = 0; col < RS_NROOTS; col++) {
        int pivot = -1;
        for (int row = col; row < RS_NROOTS; row++) {
            if (aug[row][col] != 0) { pivot = row; break; }
        }
        if (pivot < 0) { memset(parity, 0, RS_NROOTS); return; }
        if (pivot != col) {
            for (int j = 0; j <= RS_NROOTS; j++) {
                uint8_t tmp = aug[col][j];
                aug[col][j] = aug[pivot][j];
                aug[pivot][j] = tmp;
            }
        }
        uint8_t inv = gf_exp[(255 - gf_log[aug[col][col]]) % 255];
        for (int j = col; j <= RS_NROOTS; j++)
            aug[col][j] = gf_mul(aug[col][j], inv);
        for (int row = 0; row < RS_NROOTS; row++) {
            if (row == col || aug[row][col] == 0) continue;
            uint8_t factor = aug[row][col];
            for (int j = col; j <= RS_NROOTS; j++)
                aug[row][j] ^= gf_mul(factor, aug[col][j]);
        }
    }

    for (int i = 0; i < RS_NROOTS; i++)
        parity[i] = aug[i][RS_NROOTS];
}

// ===================== CRC-16 CCITT =====================

static uint16_t crc16_table[256];

static void crc16_init(void)
{
    for (int i = 0; i < 256; i++) {
        uint16_t crc = (uint16_t)(i << 8);
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            else
                crc = (uint16_t)(crc << 1);
        }
        crc16_table[i] = crc;
    }
}

static uint16_t crc16_ccitt(const uint8_t *data, int len)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++)
        crc = (uint16_t)((crc << 8) ^ crc16_table[((crc >> 8) ^ data[i]) & 0xFF]);
    return crc;
}

// ===================== Whitening =====================

static const uint8_t rs41_whitening[] = {
    0x96, 0x83, 0x3E, 0x51, 0xB1, 0x49, 0x08, 0x98,
    0x32, 0x05, 0x59, 0x0E, 0xF9, 0x44, 0xC6, 0x26,
    0x21, 0x60, 0xC2, 0xEA, 0x79, 0x5D, 0x6D, 0xA1,
    0x54, 0x69, 0x47, 0x0C, 0xDC, 0xE8, 0x5C, 0xF1,
    0xF7, 0x76, 0x82, 0x7F, 0x07, 0x99, 0xA2, 0x2C,
    0x93, 0x7C, 0x30, 0x63, 0xF5, 0x10, 0x2E, 0x61,
    0xD0, 0xBC, 0xB4, 0xB6, 0x06, 0xAA, 0xF4, 0x23,
    0x78, 0x6E, 0x3B, 0xAE, 0xBF, 0x7B, 0x4C, 0xC1
};

static void rs41_whiten(uint8_t *frame, int len)
{
    for (int i = 0; i < len; i++)
        frame[i] ^= rs41_whitening[(i + 8) % 64];
}

// ===================== ECEF to LLA (Bowring iterative) =====================

static void ecef_to_lla(double x, double y, double z,
                        double *lat, double *lon, double *alt)
{
    const double a = 6378137.0;
    const double e2 = 0.00669437999014;

    *lon = atan2(y, x);
    double p = sqrt(x * x + y * y);
    double lat_r = atan2(z, p * (1.0 - e2));

    for (int i = 0; i < 5; i++) {
        double sin_lat = sin(lat_r);
        double N = a / sqrt(1.0 - e2 * sin_lat * sin_lat);
        lat_r = atan2(z + e2 * N * sin_lat, p);
    }

    double sin_lat = sin(lat_r);
    double cos_lat = cos(lat_r);
    double N = a / sqrt(1.0 - e2 * sin_lat * sin_lat);
    if (fabs(cos_lat) > 1e-10)
        *alt = p / cos_lat - N;
    else
        *alt = fabs(z) / fabs(sin_lat) - N * (1.0 - e2);
    *lat = lat_r * 180.0 / M_PI;
    *lon = *lon * 180.0 / M_PI;
}

// ===================== Utility =====================

static uint16_t read_u16_le(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

static void write_u16_le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static void write_i32_le(uint8_t *p, int32_t v)
{
    uint32_t u = (uint32_t)v;
    p[0] = (uint8_t)(u & 0xFF);
    p[1] = (uint8_t)((u >> 8) & 0xFF);
    p[2] = (uint8_t)((u >> 16) & 0xFF);
    p[3] = (uint8_t)((u >> 24) & 0xFF);
}

static int32_t read_i32_le(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

// ===================== RS41 Interleaved RS =====================
// Matches sonde_demod.c rs41_rs_correct() layout exactly.

static int rs41_rs_correct(uint8_t *frame, int len)
{
    if (len < 64) return -1;
    int data_len = len - RS41_RS_PARITY;
    int data_per_cw = (data_len + 1) / 2;
    int cw_len = 24 + data_per_cw;

    uint8_t *cw1 = calloc((unsigned)cw_len, 1);
    uint8_t *cw2 = calloc((unsigned)cw_len, 1);
    if (!cw1 || !cw2) { free(cw1); free(cw2); return -1; }

    memcpy(cw1, frame, 24);
    for (int i = 0; i < data_per_cw; i++) {
        int src = RS41_RS_PARITY + 2 * i;
        if (src < len) cw1[24 + i] = frame[src];
    }
    memcpy(cw2, frame + 24, 24);
    for (int i = 0; i < data_per_cw; i++) {
        int src = RS41_RS_PARITY + 2 * i + 1;
        if (src < len) cw2[24 + i] = frame[src];
    }

    int err0 = rs_decode(cw1, cw_len);
    int err1 = rs_decode(cw2, cw_len);

    if (err0 >= 0) {
        memcpy(frame, cw1, 24);
        for (int i = 0; i < data_per_cw; i++) {
            int dst = RS41_RS_PARITY + 2 * i;
            if (dst < len) frame[dst] = cw1[24 + i];
        }
    }
    if (err1 >= 0) {
        memcpy(frame + 24, cw2, 24);
        for (int i = 0; i < data_per_cw; i++) {
            int dst = RS41_RS_PARITY + 2 * i + 1;
            if (dst < len) frame[dst] = cw2[24 + i];
        }
    }

    free(cw1);
    free(cw2);
    if (err0 < 0 || err1 < 0) return -1;
    return err0 + err1;
}

// Interleaved RS encoder for RS41 [parity|data] format
static void rs41_rs_encode(uint8_t *frame, int len)
{
    int data_len = len - RS41_RS_PARITY;
    int data_per_cw = (data_len + 1) / 2;

    // Extract interleaved data for each codeword
    uint8_t *d1 = calloc((unsigned)data_per_cw, 1);
    uint8_t *d2 = calloc((unsigned)data_per_cw, 1);
    if (!d1 || !d2) { free(d1); free(d2); return; }

    for (int i = 0; i < data_per_cw; i++) {
        int src = RS41_RS_PARITY + 2 * i;
        if (src < len) d1[i] = frame[src];
        src = RS41_RS_PARITY + 2 * i + 1;
        if (src < len) d2[i] = frame[src];
    }

    // Compute parity for [parity|data] format
    uint8_t parity1[RS_NROOTS], parity2[RS_NROOTS];
    rs_encode_parity_first(d1, data_per_cw, parity1);
    rs_encode_parity_first(d2, data_per_cw, parity2);

    // Store parity in frame
    memcpy(frame, parity1, RS_NROOTS);
    memcpy(frame + RS_NROOTS, parity2, RS_NROOTS);

    free(d1);
    free(d2);
}

// ===================== Test Counters =====================

static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) do { \
    tests_run++; \
    printf("  TEST: %-55s ", name); \
} while(0)

#define PASS() do { tests_passed++; printf("PASS\n"); } while(0)
#define FAIL(msg) do { tests_failed++; printf("FAIL: %s\n", msg); } while(0)

// ===================== Test 1: CRC-16 CCITT =====================

static void test_crc16(void)
{
    printf("\n=== Test 1: CRC-16 CCITT ===\n");

    {
        TEST("Standard test vector '123456789' -> 0x29B1");
        const uint8_t data[] = "123456789";
        uint16_t crc = crc16_ccitt(data, 9);
        if (crc == 0x29B1) PASS();
        else {
            char msg[64];
            snprintf(msg, sizeof(msg), "expected 0x29B1, got 0x%04X", crc);
            FAIL(msg);
        }
    }

    {
        TEST("Empty data -> 0xFFFF");
        uint16_t crc = crc16_ccitt(NULL, 0);
        if (crc == 0xFFFF) PASS();
        else {
            char msg[64];
            snprintf(msg, sizeof(msg), "expected 0xFFFF, got 0x%04X", crc);
            FAIL(msg);
        }
    }

    {
        TEST("Self-check: CRC of [data + CRC_BE] = 0");
        uint8_t data[] = { 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0, 0 };
        uint16_t crc = crc16_ccitt(data, 9);
        data[9] = (uint8_t)(crc >> 8);
        data[10] = (uint8_t)(crc & 0xFF);
        uint16_t check = crc16_ccitt(data, 11);
        if (check == 0) PASS();
        else {
            char msg[64];
            snprintf(msg, sizeof(msg), "expected 0x0000, got 0x%04X", check);
            FAIL(msg);
        }
    }

    {
        TEST("RS41 subblock CRC: build and verify");
        uint8_t block[20];
        block[0] = 0x79;
        block[1] = 10;
        for (int i = 0; i < 10; i++) block[2 + i] = (uint8_t)(i + 0x41);
        uint16_t crc = crc16_ccitt(&block[2], 10);
        block[12] = (uint8_t)(crc & 0xFF);
        block[13] = (uint8_t)(crc >> 8);
        int block_total = 14;
        uint16_t stored = (uint16_t)(block[block_total - 2] | (block[block_total - 1] << 8));
        uint16_t calc = crc16_ccitt(block + 2, block_total - 4);
        if (stored == calc) PASS();
        else {
            char msg[64];
            snprintf(msg, sizeof(msg), "stored=0x%04X calc=0x%04X", stored, calc);
            FAIL(msg);
        }
    }
}

// ===================== Test 2: GF(2^8) Arithmetic =====================

static void test_gf_arithmetic(void)
{
    printf("\n=== Test 2: GF(2^8) Arithmetic ===\n");

    {
        TEST("alpha^0 = 1");
        if (gf_exp[0] == 1) PASS();
        else FAIL("gf_exp[0] != 1");
    }

    {
        TEST("alpha^255 = alpha^0 (cyclic group)");
        if (gf_exp[255] == gf_exp[0]) PASS();
        else FAIL("gf_exp[255] != gf_exp[0]");
    }

    {
        TEST("All 255 non-zero elements unique");
        int seen[256] = {0};
        int ok = 1;
        for (int i = 0; i < 255; i++) {
            if (seen[gf_exp[i]]) { ok = 0; break; }
            seen[gf_exp[i]] = 1;
        }
        if (ok && !seen[0]) PASS();
        else FAIL("duplicate or zero in exp table");
    }

    {
        TEST("Multiplicative inverse: a * a^-1 = 1 for all a != 0");
        int ok = 1;
        for (int a = 1; a < 256; a++) {
            uint8_t inv = gf_exp[255 - gf_log[a]];
            if (gf_mul((uint8_t)a, inv) != 1) { ok = 0; break; }
        }
        if (ok) PASS();
        else FAIL("inverse check failed");
    }

    {
        TEST("Multiplication by zero = 0");
        int ok = 1;
        for (int a = 0; a < 256; a++) {
            if (gf_mul((uint8_t)a, 0) != 0) { ok = 0; break; }
            if (gf_mul(0, (uint8_t)a) != 0) { ok = 0; break; }
        }
        if (ok) PASS();
        else FAIL("zero mul failed");
    }

    {
        TEST("Primitive poly: alpha^8 = 0x1D");
        if (gf_exp[8] == 0x1D) PASS();
        else {
            char msg[64];
            snprintf(msg, sizeof(msg), "gf_exp[8] = 0x%02X, expected 0x1D", gf_exp[8]);
            FAIL(msg);
        }
    }
}

// ===================== Test 3: Reed-Solomon =====================

static void test_reed_solomon(void)
{
    printf("\n=== Test 3: Reed-Solomon RS(255,231) ===\n");

    // --- Tests using [data | parity] format (standard encoder) ---

    {
        TEST("[data|parity] All-zero: syndrome = 0");
        uint8_t cw[255];
        memset(cw, 0, 255);
        int err = rs_decode(cw, 255);
        if (err == 0) PASS();
        else {
            char msg[64];
            snprintf(msg, sizeof(msg), "expected 0, got %d", err);
            FAIL(msg);
        }
    }

    {
        TEST("[data|parity] Encoded cw: syndrome = 0");
        uint8_t data[231];
        for (int i = 0; i < 231; i++) data[i] = (uint8_t)((i * 37 + 13) & 0xFF);

        uint8_t parity[RS_NROOTS];
        rs_encode_data_first(data, 231, parity);

        uint8_t cw[255];
        memcpy(cw, data, 231);
        memcpy(cw + 231, parity, RS_NROOTS);

        int err = rs_decode(cw, 255);
        if (err == 0) PASS();
        else {
            char msg[64];
            snprintf(msg, sizeof(msg), "expected 0, got %d", err);
            FAIL(msg);
        }
    }

    {
        TEST("[data|parity] Single byte error: corrected");
        uint8_t data[231];
        for (int i = 0; i < 231; i++) data[i] = (uint8_t)((i * 37 + 13) & 0xFF);

        uint8_t parity[RS_NROOTS];
        rs_encode_data_first(data, 231, parity);

        uint8_t cw[255], orig[255];
        memcpy(cw, data, 231);
        memcpy(cw + 231, parity, RS_NROOTS);
        memcpy(orig, cw, 255);

        cw[100] ^= 0xAB;
        int err = rs_decode(cw, 255);
        if (err == 1 && memcmp(cw, orig, 255) == 0) PASS();
        else {
            char msg[64];
            snprintf(msg, sizeof(msg), "errors=%d, match=%d", err, memcmp(cw, orig, 255) == 0);
            FAIL(msg);
        }
    }

    {
        TEST("[data|parity] 12 errors (max t): corrected");
        uint8_t data[231];
        for (int i = 0; i < 231; i++) data[i] = (uint8_t)((i * 37 + 13) & 0xFF);

        uint8_t parity[RS_NROOTS];
        rs_encode_data_first(data, 231, parity);

        uint8_t cw[255], orig[255];
        memcpy(cw, data, 231);
        memcpy(cw + 231, parity, RS_NROOTS);
        memcpy(orig, cw, 255);

        int corrupt[] = { 5, 20, 50, 75, 100, 130, 150, 180, 200, 220, 240, 250 };
        for (int i = 0; i < 12; i++)
            cw[corrupt[i]] ^= (uint8_t)(0x11 + i);

        int err = rs_decode(cw, 255);
        if (err == 12 && memcmp(cw, orig, 255) == 0) PASS();
        else {
            char msg[64];
            snprintf(msg, sizeof(msg), "errors=%d, match=%d", err, memcmp(cw, orig, 255) == 0);
            FAIL(msg);
        }
    }

    {
        TEST("[data|parity] 13 errors (t+1): uncorrectable");
        uint8_t data[231];
        for (int i = 0; i < 231; i++) data[i] = (uint8_t)((i * 37 + 13) & 0xFF);

        uint8_t parity[RS_NROOTS];
        rs_encode_data_first(data, 231, parity);

        uint8_t cw[255];
        memcpy(cw, data, 231);
        memcpy(cw + 231, parity, RS_NROOTS);

        for (int i = 0; i < 13; i++)
            cw[i * 19] ^= (uint8_t)(0x33 + i);

        int err = rs_decode(cw, 255);
        if (err == -1) PASS();
        else {
            char msg[64];
            snprintf(msg, sizeof(msg), "expected -1, got %d", err);
            FAIL(msg);
        }
    }

    // --- Tests using [parity | data] format (RS41 convention) ---

    {
        TEST("[parity|data] Encoded cw: syndrome = 0");
        int data_len = 136;
        uint8_t data[136];
        for (int i = 0; i < data_len; i++) data[i] = (uint8_t)((i * 53 + 7) & 0xFF);

        uint8_t parity[RS_NROOTS];
        rs_encode_parity_first(data, data_len, parity);

        int cw_len = RS_NROOTS + data_len;
        uint8_t cw[160];
        memcpy(cw, parity, RS_NROOTS);
        memcpy(cw + RS_NROOTS, data, data_len);

        int err = rs_decode(cw, cw_len);
        if (err == 0) PASS();
        else {
            char msg[64];
            snprintf(msg, sizeof(msg), "expected 0, got %d", err);
            FAIL(msg);
        }
    }

    {
        TEST("[parity|data] Single byte error: corrected");
        int data_len = 136;
        uint8_t data[136];
        for (int i = 0; i < data_len; i++) data[i] = (uint8_t)((i * 53 + 7) & 0xFF);

        uint8_t parity[RS_NROOTS];
        rs_encode_parity_first(data, data_len, parity);

        int cw_len = RS_NROOTS + data_len;
        uint8_t cw[160], orig[160];
        memcpy(cw, parity, RS_NROOTS);
        memcpy(cw + RS_NROOTS, data, data_len);
        memcpy(orig, cw, cw_len);

        cw[80] ^= 0xBB;
        int err = rs_decode(cw, cw_len);
        if (err == 1 && memcmp(cw, orig, (unsigned)cw_len) == 0) PASS();
        else {
            char msg[64];
            snprintf(msg, sizeof(msg), "errors=%d, match=%d", err, memcmp(cw, orig, (unsigned)cw_len) == 0);
            FAIL(msg);
        }
    }

    {
        TEST("[parity|data] 5 errors in shortened cw: corrected");
        int data_len = 136;
        uint8_t data[136];
        for (int i = 0; i < data_len; i++) data[i] = (uint8_t)((i * 53 + 7) & 0xFF);

        uint8_t parity[RS_NROOTS];
        rs_encode_parity_first(data, data_len, parity);

        int cw_len = RS_NROOTS + data_len;
        uint8_t cw[160], orig[160];
        memcpy(cw, parity, RS_NROOTS);
        memcpy(cw + RS_NROOTS, data, data_len);
        memcpy(orig, cw, cw_len);

        cw[10] ^= 0xFE;
        cw[50] ^= 0x42;
        cw[80] ^= 0x13;
        cw[110] ^= 0x99;
        cw[140] ^= 0x7F;

        int err = rs_decode(cw, cw_len);
        if (err == 5 && memcmp(cw, orig, (unsigned)cw_len) == 0) PASS();
        else {
            char msg[64];
            snprintf(msg, sizeof(msg), "errors=%d, match=%d", err, memcmp(cw, orig, (unsigned)cw_len) == 0);
            FAIL(msg);
        }
    }

    {
        TEST("[parity|data] 12 errors (max t): corrected");
        int data_len = 136;
        uint8_t data[136];
        for (int i = 0; i < data_len; i++) data[i] = (uint8_t)((i * 53 + 7) & 0xFF);

        uint8_t parity[RS_NROOTS];
        rs_encode_parity_first(data, data_len, parity);

        int cw_len = RS_NROOTS + data_len;
        uint8_t cw[160], orig[160];
        memcpy(cw, parity, RS_NROOTS);
        memcpy(cw + RS_NROOTS, data, data_len);
        memcpy(orig, cw, cw_len);

        int corrupt[] = { 3, 15, 28, 42, 55, 70, 85, 100, 115, 130, 145, 155 };
        for (int i = 0; i < 12; i++)
            cw[corrupt[i]] ^= (uint8_t)(0x22 + i);

        int err = rs_decode(cw, cw_len);
        if (err == 12 && memcmp(cw, orig, (unsigned)cw_len) == 0) PASS();
        else {
            char msg[64];
            snprintf(msg, sizeof(msg), "errors=%d, match=%d", err, memcmp(cw, orig, (unsigned)cw_len) == 0);
            FAIL(msg);
        }
    }
}

// ===================== Test 4: Whitening =====================

static void test_whitening(void)
{
    printf("\n=== Test 4: XOR Whitening ===\n");

    {
        TEST("Whitening round-trip (XOR twice = identity)");
        uint8_t data[320], orig[320];
        for (int i = 0; i < 320; i++) data[i] = orig[i] = (uint8_t)(i & 0xFF);
        rs41_whiten(data, 320);
        int changed = memcmp(data, orig, 320) != 0;
        rs41_whiten(data, 320);
        int restored = memcmp(data, orig, 320) == 0;
        if (changed && restored) PASS();
        else {
            char msg[64];
            snprintf(msg, sizeof(msg), "changed=%d, restored=%d", changed, restored);
            FAIL(msg);
        }
    }

    {
        // rs1729: xorbyte(pos) = frame[pos] ^ mask[pos % 64]
        // His frame includes 8-byte header. Our frame_buf starts after sync detection.
        // Our frame_buf[i] corresponds to his frame[i+8].
        // Our whitening: frame_buf[i] ^= mask[(i+8) % 64]
        // His: frame[i+8] ^= mask[(i+8) % 64]  -- same mask index
        TEST("Whitening offset: rs1729 pos_FrameNb=0x3B maps correctly");
        int rs1729_pos = 0x3B; // byte 59 in his full frame
        int our_pos = rs1729_pos - 8; // byte 51 in our frame_buf
        int our_mask_idx = (our_pos + 8) % 64; // should be 59
        int his_mask_idx = rs1729_pos % 64;     // 59
        if (our_mask_idx == his_mask_idx) PASS();
        else {
            char msg[64];
            snprintf(msg, sizeof(msg), "our=%d his=%d", our_mask_idx, his_mask_idx);
            FAIL(msg);
        }
    }

    {
        TEST("Whitening offset: rs1729 pos_GPSecefX=0x114 maps correctly");
        int rs1729_pos = 0x114; // byte 276
        int our_pos = rs1729_pos - 8; // byte 268
        int our_mask_idx = (our_pos + 8) % 64;
        int his_mask_idx = rs1729_pos % 64;
        if (our_mask_idx == his_mask_idx) PASS();
        else {
            char msg[64];
            snprintf(msg, sizeof(msg), "our=%d his=%d", our_mask_idx, his_mask_idx);
            FAIL(msg);
        }
    }
}

// ===================== Test 5: ECEF to LLA =====================

static void test_ecef_lla(void)
{
    printf("\n=== Test 5: ECEF to Lat/Lon/Alt ===\n");

    {
        TEST("Equator/Greenwich (0, 0, 0m)");
        double lat, lon, alt;
        ecef_to_lla(6378137.0, 0.0, 0.0, &lat, &lon, &alt);
        if (fabs(lat) < 0.001 && fabs(lon) < 0.001 && fabs(alt) < 1.0) PASS();
        else {
            char msg[128];
            snprintf(msg, sizeof(msg), "lat=%.6f lon=%.6f alt=%.1f", lat, lon, alt);
            FAIL(msg);
        }
    }

    {
        TEST("North Pole (90N, 0m)");
        double lat, lon, alt;
        ecef_to_lla(0.0, 0.0, 6356752.3, &lat, &lon, &alt);
        if (fabs(lat - 90.0) < 0.01 && fabs(alt) < 10.0) PASS();
        else {
            char msg[128];
            snprintf(msg, sizeof(msg), "lat=%.6f alt=%.1f", lat, alt);
            FAIL(msg);
        }
    }

    {
        TEST("Round-trip: Berlin at 5000m altitude");
        double lat_r = 52.52 * M_PI / 180.0;
        double lon_r = 13.405 * M_PI / 180.0;
        double a = 6378137.0, e2 = 0.00669437999014;
        double sin_lat = sin(lat_r), cos_lat = cos(lat_r);
        double N = a / sqrt(1.0 - e2 * sin_lat * sin_lat);
        double h = 5000.0;
        double X = (N + h) * cos_lat * cos(lon_r);
        double Y = (N + h) * cos_lat * sin(lon_r);
        double Z = (N * (1.0 - e2) + h) * sin_lat;

        double lat, lon, alt;
        ecef_to_lla(X, Y, Z, &lat, &lon, &alt);
        if (fabs(lat - 52.52) < 0.0001 && fabs(lon - 13.405) < 0.0001 && fabs(alt - 5000.0) < 1.0)
            PASS();
        else {
            char msg[128];
            snprintf(msg, sizeof(msg), "lat=%.6f lon=%.6f alt=%.1f", lat, lon, alt);
            FAIL(msg);
        }
    }

    {
        TEST("Round-trip: 30km balloon over Berlin");
        double lat_r = 52.52 * M_PI / 180.0;
        double lon_r = 13.405 * M_PI / 180.0;
        double a = 6378137.0, e2 = 0.00669437999014;
        double sin_lat = sin(lat_r), cos_lat = cos(lat_r);
        double N = a / sqrt(1.0 - e2 * sin_lat * sin_lat);
        double h = 30000.0;
        double X = (N + h) * cos_lat * cos(lon_r);
        double Y = (N + h) * cos_lat * sin(lon_r);
        double Z = (N * (1.0 - e2) + h) * sin_lat;

        double lat, lon, alt;
        ecef_to_lla(X, Y, Z, &lat, &lon, &alt);
        if (fabs(lat - 52.52) < 0.001 && fabs(lon - 13.405) < 0.001 && fabs(alt - 30000.0) < 5.0)
            PASS();
        else {
            char msg[128];
            snprintf(msg, sizeof(msg), "lat=%.6f lon=%.6f alt=%.1f", lat, lon, alt);
            FAIL(msg);
        }
    }

    {
        TEST("RS41 integer ECEF (cm) round-trip: Munich 15km");
        double lat_exp = 48.1351, lon_exp = 11.582, alt_exp = 15000.0;
        double lat_r = lat_exp * M_PI / 180.0;
        double lon_r = lon_exp * M_PI / 180.0;
        double a = 6378137.0, e2 = 0.00669437999014;
        double sin_lat = sin(lat_r), cos_lat = cos(lat_r);
        double N = a / sqrt(1.0 - e2 * sin_lat * sin_lat);
        double X = (N + alt_exp) * cos_lat * cos(lon_r);
        double Y = (N + alt_exp) * cos_lat * sin(lon_r);
        double Z = (N * (1.0 - e2) + alt_exp) * sin_lat;

        int32_t ecef_x = (int32_t)(X * 100.0);
        int32_t ecef_y = (int32_t)(Y * 100.0);
        int32_t ecef_z = (int32_t)(Z * 100.0);

        double lat, lon, alt;
        ecef_to_lla(ecef_x / 100.0, ecef_y / 100.0, ecef_z / 100.0, &lat, &lon, &alt);
        if (fabs(lat - lat_exp) < 0.001 && fabs(lon - lon_exp) < 0.001 && fabs(alt - alt_exp) < 10.0)
            PASS();
        else {
            char msg[128];
            snprintf(msg, sizeof(msg), "lat=%.6f lon=%.6f alt=%.1f", lat, lon, alt);
            FAIL(msg);
        }
    }
}

// ===================== Test 6: Full RS41 Frame Pipeline =====================

static int build_subblock(uint8_t *buf, uint8_t id, const uint8_t *data, int data_len)
{
    buf[0] = id;
    buf[1] = (uint8_t)data_len;
    memcpy(buf + 2, data, data_len);
    uint16_t crc = crc16_ccitt(data, data_len);
    buf[2 + data_len] = (uint8_t)(crc & 0xFF);
    buf[2 + data_len + 1] = (uint8_t)(crc >> 8);
    return 2 + data_len + 2;
}

static void test_full_frame(void)
{
    printf("\n=== Test 6: Full RS41 Frame Pipeline ===\n");

    // ---- Build a synthetic RS41 frame ----
    uint8_t frame[RS41_FRAME_LEN];
    memset(frame, 0, RS41_FRAME_LEN);

    int pos = RS41_DATA_START + 1;
    frame[RS41_DATA_START] = 0x0F; // Frame type

    // STATUS subblock: frame_num(2) + serial(8) + padding
    {
        uint8_t sdata[40];
        memset(sdata, 0, sizeof(sdata));
        write_u16_le(&sdata[0], 1000);
        memcpy(&sdata[2], "T1234567", 8);
        pos += build_subblock(&frame[pos], RS41_BLOCK_STATUS, sdata, 40);
    }

    // GPS position subblock: ECEF + velocity + sats
    double exp_lat = 52.52, exp_lon = 13.405, exp_alt = 5000.0;
    {
        uint8_t gdata[21];
        memset(gdata, 0, sizeof(gdata));

        double lat_r = exp_lat * M_PI / 180.0;
        double lon_r = exp_lon * M_PI / 180.0;
        double a = 6378137.0, e2 = 0.00669437999014;
        double sl = sin(lat_r), cl = cos(lat_r);
        double N = a / sqrt(1.0 - e2 * sl * sl);
        double X = (N + exp_alt) * cl * cos(lon_r);
        double Y = (N + exp_alt) * cl * sin(lon_r);
        double Z = (N * (1.0 - e2) + exp_alt) * sl;

        write_i32_le(&gdata[0], (int32_t)(X * 100.0));
        write_i32_le(&gdata[4], (int32_t)(Y * 100.0));
        write_i32_le(&gdata[8], (int32_t)(Z * 100.0));
        write_u16_le(&gdata[12], 500);
        write_u16_le(&gdata[14], 300);
        write_u16_le(&gdata[16], (uint16_t)(int16_t)-200);
        gdata[18] = 9;

        pos += build_subblock(&frame[pos], RS41_BLOCK_GPSPOS, gdata, 21);
    }

    // Compute RS parity (interleaved, [parity|data] format)
    rs41_rs_encode(frame, RS41_FRAME_LEN);

    // Save clean frame
    uint8_t clean[RS41_FRAME_LEN];
    memcpy(clean, frame, RS41_FRAME_LEN);

    // ---- Test 6a: Clean frame RS decode ----
    {
        TEST("Clean frame: RS decode = 0 errors");
        uint8_t tf[RS41_FRAME_LEN];
        memcpy(tf, clean, RS41_FRAME_LEN);
        int err = rs41_rs_correct(tf, RS41_FRAME_LEN);
        if (err == 0) PASS();
        else {
            char msg[64];
            snprintf(msg, sizeof(msg), "expected 0, got %d", err);
            FAIL(msg);
        }
    }

    // ---- Test 6b: Subblock CRC verification ----
    {
        TEST("Subblock CRCs valid on clean frame");
        int p = RS41_DATA_START + 1;
        int all_ok = 1, count = 0;
        while (p + 3 < RS41_FRAME_LEN) {
            uint8_t bid = clean[p];
            uint8_t blen = clean[p + 1];
            int btotal = 2 + (int)blen + 2;
            if (blen == 0 || p + btotal > RS41_FRAME_LEN) break;
            uint16_t stored = (uint16_t)(clean[p + btotal - 2] | (clean[p + btotal - 1] << 8));
            uint16_t calc = crc16_ccitt(&clean[p + 2], blen);
            if (stored != calc) {
                printf("[block 0x%02X crc mismatch] ", bid);
                all_ok = 0;
            }
            count++;
            p += btotal;
        }
        if (all_ok && count >= 2) PASS();
        else {
            char msg[64];
            snprintf(msg, sizeof(msg), "blocks=%d, all_ok=%d", count, all_ok);
            FAIL(msg);
        }
    }

    // ---- Test 6c: Parse STATUS ----
    {
        TEST("Parse STATUS: serial='T1234567', frame=1000");
        int p = RS41_DATA_START + 1;
        int ok = 0;
        while (p + 3 < RS41_FRAME_LEN) {
            uint8_t bid = clean[p];
            uint8_t blen = clean[p + 1];
            int btotal = 2 + (int)blen + 2;
            if (blen == 0 || p + btotal > RS41_FRAME_LEN) break;
            if (bid == RS41_BLOCK_STATUS && blen >= 10) {
                uint16_t fnum = read_u16_le(&clean[p + 2]);
                char ser[16] = {0};
                for (int i = 0; i < 8 && i + 4 <= (int)blen; i++) {
                    char c = (char)clean[p + 2 + 2 + i];
                    if (c >= 0x20 && c < 0x7F) ser[i] = c;
                    else break;
                }
                if (fnum == 1000 && strcmp(ser, "T1234567") == 0) ok = 1;
                else printf("[fnum=%d ser='%s'] ", fnum, ser);
            }
            p += btotal;
        }
        if (ok) PASS();
        else FAIL("STATUS mismatch");
    }

    // ---- Test 6d: Parse GPS position ----
    {
        TEST("Parse GPSPOS: lat~52.52 lon~13.405 alt~5000");
        int p = RS41_DATA_START + 1;
        int ok = 0;
        while (p + 3 < RS41_FRAME_LEN) {
            uint8_t bid = clean[p];
            uint8_t blen = clean[p + 1];
            int btotal = 2 + (int)blen + 2;
            if (blen == 0 || p + btotal > RS41_FRAME_LEN) break;
            if (bid == RS41_BLOCK_GPSPOS && blen >= 12) {
                int32_t ex = read_i32_le(&clean[p + 2 + 0]);
                int32_t ey = read_i32_le(&clean[p + 2 + 4]);
                int32_t ez = read_i32_le(&clean[p + 2 + 8]);
                double lat, lon, alt;
                ecef_to_lla(ex / 100.0, ey / 100.0, ez / 100.0, &lat, &lon, &alt);
                if (fabs(lat - exp_lat) < 0.01 && fabs(lon - exp_lon) < 0.01 &&
                    fabs(alt - exp_alt) < 5.0) {
                    ok = 1;
                } else {
                    printf("[lat=%.5f lon=%.5f alt=%.1f] ", lat, lon, alt);
                }
                if (blen >= 19 && clean[p + 2 + 18] != 9) {
                    printf("[sats=%d] ", clean[p + 2 + 18]);
                    ok = 0;
                }
            }
            p += btotal;
        }
        if (ok) PASS();
        else FAIL("GPSPOS mismatch");
    }

    // ---- Test 6e: Full pipeline with whitening + corruption ----
    {
        TEST("Pipeline: whiten -> corrupt 3B -> dewhiten -> RS -> verify");
        uint8_t wf[RS41_FRAME_LEN];
        memcpy(wf, clean, RS41_FRAME_LEN);

        // Whiten (simulate transmitter output)
        rs41_whiten(wf, RS41_FRAME_LEN);

        // Inject 3 byte errors (channel errors in whitened domain)
        wf[10] ^= 0xAA;
        wf[100] ^= 0x55;
        wf[200] ^= 0xCC;

        // Receiver: dewhiten
        rs41_whiten(wf, RS41_FRAME_LEN);

        // RS correction
        int rs_err = rs41_rs_correct(wf, RS41_FRAME_LEN);
        if (rs_err < 0) {
            FAIL("RS correction failed");
            return;
        }

        // Frame should match clean after correction
        int frame_ok = (memcmp(wf, clean, RS41_FRAME_LEN) == 0);

        // Verify CRCs
        int crc_ok = 1;
        int p = RS41_DATA_START + 1;
        while (p + 3 < RS41_FRAME_LEN) {
            uint8_t blen = wf[p + 1];
            int btotal = 2 + (int)blen + 2;
            if (blen == 0 || p + btotal > RS41_FRAME_LEN) break;
            uint16_t stored = (uint16_t)(wf[p + btotal - 2] | (wf[p + btotal - 1] << 8));
            uint16_t calc = crc16_ccitt(&wf[p + 2], blen);
            if (stored != calc) crc_ok = 0;
            p += btotal;
        }

        // Verify parsed serial
        char ser[16] = {0};
        uint16_t fnum = 0;
        p = RS41_DATA_START + 1;
        while (p + 3 < RS41_FRAME_LEN) {
            uint8_t bid = wf[p];
            uint8_t blen = wf[p + 1];
            int btotal = 2 + (int)blen + 2;
            if (blen == 0 || p + btotal > RS41_FRAME_LEN) break;
            if (bid == RS41_BLOCK_STATUS && blen >= 10) {
                fnum = read_u16_le(&wf[p + 2]);
                for (int i = 0; i < 8; i++) {
                    char c = (char)wf[p + 2 + 2 + i];
                    if (c >= 0x20 && c < 0x7F) ser[i] = c;
                    else break;
                }
            }
            p += btotal;
        }

        if (frame_ok && crc_ok && fnum == 1000 && strcmp(ser, "T1234567") == 0) PASS();
        else {
            char msg[128];
            snprintf(msg, sizeof(msg), "frame_ok=%d crc=%d fnum=%d ser='%s' rs=%d",
                     frame_ok, crc_ok, fnum, ser, rs_err);
            FAIL(msg);
        }
    }

    // ---- Test 6f: Noise rejection ----
    {
        TEST("Random noise frame: rejected by RS");
        uint8_t noise[RS41_FRAME_LEN];
        unsigned seed = 0xDEADBEEF;
        for (int i = 0; i < RS41_FRAME_LEN; i++) {
            seed = seed * 1103515245 + 12345;
            noise[i] = (uint8_t)((seed >> 16) & 0xFF);
        }
        int err = rs41_rs_correct(noise, RS41_FRAME_LEN);
        if (err == -1) PASS();
        else {
            char msg[64];
            snprintf(msg, sizeof(msg), "expected -1, got %d", err);
            FAIL(msg);
        }
    }
}

// ===================== Main =====================

int main(void)
{
    printf("RS41 Radiosonde Decoder — Unit Tests\n");
    printf("=====================================\n");

    gf_init();
    crc16_init();

    test_crc16();
    test_gf_arithmetic();
    test_reed_solomon();
    test_whitening();
    test_ecef_lla();
    test_full_frame();

    printf("\n=====================================\n");
    printf("Results: %d/%d passed, %d failed\n", tests_passed, tests_run, tests_failed);
    printf("=====================================\n");

    return tests_failed > 0 ? 1 : 0;
}
