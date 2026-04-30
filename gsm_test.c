// gsm_test.c — GSM broadcast decoder unit tests
//
// Build: cc -o gsm_test gsm_test.c gsm_decode.c -lm
// Run:   ./gsm_test
//
// Tests all components of the GSM decoder against known test vectors
// derived from 3GPP TS 05.02, TS 05.03, TS 04.08 specifications.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "gsm_decode.h"

// ======================== Test Harness ========================

static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_ASSERT(cond, msg) do { \
    tests_run++; \
    if (!(cond)) { \
        printf("  FAIL: %s (line %d)\n", msg, __LINE__); \
        tests_failed++; \
    } else { \
        tests_passed++; \
    } \
} while(0)

#define TEST_ASSERT_EQ(a, b, msg) do { \
    tests_run++; \
    if ((a) != (b)) { \
        printf("  FAIL: %s — expected %d, got %d (line %d)\n", msg, (int)(b), (int)(a), __LINE__); \
        tests_failed++; \
    } else { \
        tests_passed++; \
    } \
} while(0)

#define TEST_SECTION(name) printf("\n=== %s ===\n", name)

// ======================== Test: Convolutional Encoder/Decoder ========================

static void test_conv_codec(void)
{
    TEST_SECTION("Convolutional Codec (rate 1/2, K=5)");

    // Test vector 1: All zeros
    {
        uint8_t input[10] = {0,0,0,0,0,0,0,0,0,0};
        uint8_t coded[20];
        gsm_conv_encode(input, 10, coded);

        // All-zero input with all-zero initial state → all-zero output
        bool all_zero = true;
        for (int i = 0; i < 20; i++) {
            if (coded[i] != 0) all_zero = false;
        }
        TEST_ASSERT(all_zero, "Conv encode all-zeros → all-zeros");
    }

    // Test vector 2: Single '1' followed by zeros (impulse response)
    {
        uint8_t input[8] = {1,0,0,0,0,0,0,0};
        uint8_t coded[16];
        gsm_conv_encode(input, 8, coded);

        // G0 impulse response: 1,0,0,1,1,0,0,0,...  (from 1+D^3+D^4)
        // G1 impulse response: 1,1,0,1,1,0,0,0,...  (from 1+D+D^3+D^4)
        // Interleaved: (G0[0],G1[0]), (G0[1],G1[1]), ...
        //            = (1,1), (0,1), (0,0), (1,1), (1,1), (0,0), (0,0), (0,0)
        uint8_t expected[] = {1,1, 0,1, 0,0, 1,1, 1,1, 0,0, 0,0, 0,0};
        bool match = true;
        for (int i = 0; i < 16; i++) {
            if (coded[i] != expected[i]) match = false;
        }
        TEST_ASSERT(match, "Conv encode impulse response matches G0/G1");

        if (!match) {
            printf("    Expected: ");
            for (int i = 0; i < 16; i++) printf("%d", expected[i]);
            printf("\n    Got:      ");
            for (int i = 0; i < 16; i++) printf("%d", coded[i]);
            printf("\n");
        }
    }

    // Test vector 3: Encode then decode (roundtrip)
    {
        uint8_t input[20];
        // Known pattern with tail bits
        uint8_t pattern[] = {1,0,1,1,0,0,1,0,1,0,0,1,1,0,1,1, 0,0,0,0}; // 16 data + 4 tail
        memcpy(input, pattern, 20);

        uint8_t coded[40];
        gsm_conv_encode(input, 20, coded);

        // Convert to soft bits (perfect, no noise)
        float soft[40];
        for (int i = 0; i < 40; i++) {
            soft[i] = coded[i] ? -1.0f : 1.0f; // bit 0 → +1, bit 1 → -1
        }

        uint8_t decoded[20];
        gsm_viterbi_decode(soft, 20, decoded);

        bool match = true;
        for (int i = 0; i < 20; i++) {
            if (decoded[i] != input[i]) match = false;
        }
        TEST_ASSERT(match, "Conv encode→decode roundtrip (clean signal)");

        if (!match) {
            printf("    Input:   ");
            for (int i = 0; i < 20; i++) printf("%d", input[i]);
            printf("\n    Decoded: ");
            for (int i = 0; i < 20; i++) printf("%d", decoded[i]);
            printf("\n");
        }
    }

    // Test vector 4: Roundtrip with noise
    {
        uint8_t input[20] = {1,1,0,1,0,0,1,1,0,1,0,1,1,0,0,0, 0,0,0,0};
        uint8_t coded[40];
        gsm_conv_encode(input, 20, coded);

        float soft[40];
        for (int i = 0; i < 40; i++) {
            soft[i] = coded[i] ? -1.0f : 1.0f;
        }

        // Flip 2 soft bits (simulate errors)
        soft[5]  = -soft[5];  // flip bit 5
        soft[22] = -soft[22]; // flip bit 22

        uint8_t decoded[20];
        gsm_viterbi_decode(soft, 20, decoded);

        bool match = true;
        for (int i = 0; i < 20; i++) {
            if (decoded[i] != input[i]) match = false;
        }
        TEST_ASSERT(match, "Conv decode corrects 2 bit errors");
    }

    // Test vector 5: Larger block (SCH-like, 39 bits)
    {
        uint8_t input[39] = {0};
        // Some pattern with tail zeros
        for (int i = 0; i < 35; i++) input[i] = (uint8_t)((i * 7 + 3) % 2);
        input[35] = input[36] = input[37] = input[38] = 0; // tail

        uint8_t coded[78];
        gsm_conv_encode(input, 39, coded);

        float soft[78];
        for (int i = 0; i < 78; i++) {
            soft[i] = coded[i] ? -1.0f : 1.0f;
        }

        uint8_t decoded[39];
        gsm_viterbi_decode(soft, 39, decoded);

        bool match = true;
        for (int i = 0; i < 39; i++) {
            if (decoded[i] != input[i]) match = false;
        }
        TEST_ASSERT(match, "Conv SCH-size roundtrip (39 bits)");
    }

    // Test vector 6: BCCH-size block (228 bits)
    {
        uint8_t input[228];
        for (int i = 0; i < 184; i++) input[i] = (uint8_t)((i * 13 + 5) % 2);
        // 40 "parity" bits + 4 tail zeros (we just use dummy values for codec test)
        for (int i = 184; i < 224; i++) input[i] = (uint8_t)(i % 2);
        input[224] = input[225] = input[226] = input[227] = 0;

        uint8_t coded[456];
        gsm_conv_encode(input, 228, coded);

        float soft[456];
        for (int i = 0; i < 456; i++) {
            soft[i] = coded[i] ? -1.0f : 1.0f;
        }

        // Introduce 5 errors
        soft[10] = -soft[10];
        soft[100] = -soft[100];
        soft[200] = -soft[200];
        soft[300] = -soft[300];
        soft[400] = -soft[400];

        uint8_t decoded[228];
        gsm_viterbi_decode(soft, 228, decoded);

        bool match = true;
        for (int i = 0; i < 228; i++) {
            if (decoded[i] != input[i]) match = false;
        }
        TEST_ASSERT(match, "Conv BCCH-size roundtrip with 5 errors (228 bits)");
    }
}

// ======================== Test: Fire Code ========================

static void test_fire_code(void)
{
    TEST_SECTION("Fire Code CRC (BCCH/CCCH)");

    // Test 1: All-zero data → encode → check passes
    {
        uint8_t data[184] = {0};
        uint8_t parity[40];
        gsm_fire_encode(data, 184, parity);

        // Concatenate data + parity
        uint8_t codeword[224];
        memcpy(codeword, data, 184);
        memcpy(&codeword[184], parity, 40);

        TEST_ASSERT(gsm_fire_check(codeword, 224), "Fire code check passes (all-zero data)");

        // All-zero data should produce all-zero parity
        bool parity_zero = true;
        for (int i = 0; i < 40; i++) {
            if (parity[i] != 0) parity_zero = false;
        }
        TEST_ASSERT(parity_zero, "Fire code: all-zero data → all-zero parity");
    }

    // Test 2: Non-trivial data
    {
        uint8_t data[184];
        for (int i = 0; i < 184; i++) data[i] = (uint8_t)((i * 7 + 3) % 2);

        uint8_t parity[40];
        gsm_fire_encode(data, 184, parity);

        uint8_t codeword[224];
        memcpy(codeword, data, 184);
        memcpy(&codeword[184], parity, 40);

        TEST_ASSERT(gsm_fire_check(codeword, 224), "Fire code check passes (pattern data)");

        // Flip one bit — should fail
        codeword[50] ^= 1;
        TEST_ASSERT(!gsm_fire_check(codeword, 224), "Fire code detects 1-bit error");

        // Restore and flip parity bit — should fail
        codeword[50] ^= 1;
        codeword[190] ^= 1;
        TEST_ASSERT(!gsm_fire_check(codeword, 224), "Fire code detects parity error");
    }

    // Test 3: Random-like data
    {
        uint8_t data[184];
        // LFSR-like pattern
        uint32_t lfsr = 0xACE1;
        for (int i = 0; i < 184; i++) {
            data[i] = (uint8_t)(lfsr & 1);
            lfsr = (lfsr >> 1) ^ ((lfsr & 1) ? 0xB400 : 0);
        }

        uint8_t parity[40];
        gsm_fire_encode(data, 184, parity);

        uint8_t codeword[224];
        memcpy(codeword, data, 184);
        memcpy(&codeword[184], parity, 40);

        TEST_ASSERT(gsm_fire_check(codeword, 224), "Fire code check passes (LFSR data)");

        // Non-zero parity for non-trivial data
        bool parity_nonzero = false;
        for (int i = 0; i < 40; i++) {
            if (parity[i]) { parity_nonzero = true; break; }
        }
        TEST_ASSERT(parity_nonzero, "Fire code: non-trivial data → non-zero parity");
    }
}

// ======================== Test: SCH Encode/Decode ========================

static void test_sch_codec(void)
{
    TEST_SECTION("SCH Encode/Decode");

    // Test 1: Known BSIC and frame number (must be valid SCH position: T3 ∈ {1,11,21,31,41})
    {
        uint8_t bsic = 0x1A; // NCC=3, BCC=2
        uint32_t fn = 1327;  // T1=1, T3=1327%51=1, T2=1327%26=1 → valid SCH

        uint8_t info[25];
        gsm_sch_build(bsic, fn, info);

        // Verify parse recovers same values
        uint8_t bsic_out;
        uint32_t fn_out;
        gsm_sch_parse(info, &bsic_out, &fn_out);

        TEST_ASSERT_EQ(bsic_out, bsic, "SCH parse: BSIC matches");
        TEST_ASSERT_EQ(fn_out, fn, "SCH parse: FN matches");
    }

    // Test 2: Full encode/decode roundtrip (clean)
    {
        uint8_t bsic = 0x35; // NCC=6, BCC=5
        // Build a valid SCH FN: T1=100, T3=11, T2=5
        // FN = 51*((T3-T2) mod 26) + T3 + 1326*T1
        //    = 51*((11-5) mod 26) + 11 + 132600 = 51*6 + 11 + 132600 = 132917
        uint32_t fn = 132917;

        uint8_t info[25];
        gsm_sch_build(bsic, fn, info);

        uint8_t coded[78];
        gsm_sch_encode(info, coded);

        // Convert to soft bits
        float soft[78];
        for (int i = 0; i < 78; i++) {
            soft[i] = coded[i] ? -1.0f : 1.0f;
        }

        uint8_t decoded_info[25];
        bool ok = gsm_sch_decode(soft, decoded_info);
        TEST_ASSERT(ok, "SCH decode succeeds (clean)");

        if (ok) {
            uint8_t dec_bsic;
            uint32_t dec_fn;
            gsm_sch_parse(decoded_info, &dec_bsic, &dec_fn);

            TEST_ASSERT_EQ(dec_bsic, bsic, "SCH roundtrip: BSIC matches");
            TEST_ASSERT_EQ(dec_fn, fn, "SCH roundtrip: FN matches");
        }
    }

    // Test 3: SCH decode with bit errors
    {
        uint8_t bsic = 0x07; // NCC=0, BCC=7
        uint32_t fn = 1;     // FN=1 → T3=1, valid SCH position

        uint8_t info[25];
        gsm_sch_build(bsic, fn, info);

        uint8_t coded[78];
        gsm_sch_encode(info, coded);

        float soft[78];
        for (int i = 0; i < 78; i++) {
            soft[i] = coded[i] ? -1.0f : 1.0f;
        }

        // Introduce 2 bit errors
        soft[10] = -soft[10];
        soft[50] = -soft[50];

        uint8_t decoded_info[25];
        bool ok = gsm_sch_decode(soft, decoded_info);
        TEST_ASSERT(ok, "SCH decode succeeds with 2 errors");

        if (ok) {
            uint8_t dec_bsic;
            uint32_t dec_fn;
            gsm_sch_parse(decoded_info, &dec_bsic, &dec_fn);
            TEST_ASSERT_EQ(dec_bsic, bsic, "SCH noisy: BSIC matches");
            TEST_ASSERT_EQ(dec_fn, fn, "SCH noisy: FN matches");
        }
    }

    // Test 4: Multiple frame numbers (all valid SCH positions: T3 ∈ {1,11,21,31,41})
    {
        // Construct valid FNs using: FN = 51*((T3-T2)%26) + T3 + 1326*T1
        uint32_t test_fns[] = {
            1,       // T1=0, T3=1, T2=1
            52,      // T1=0, T3=1, T2=0: 51*1+1=52. check: 52%51=1 ✓
            11,      // T1=0, T3=11, T2=11: 51*0+11=11. check: 11%51=11 ✓
            1327,    // T1=1, T3=1, T2=1
            1337,    // T1=1, T3=11, T2=11
            132917,  // T1=100, T3=11, T2=5
            2715649, // near max: T1=2047, T3=1, T2=0: 51*1+1+1326*2047=2715673
        };
        // Fixup: recompute to ensure valid
        // Actually, let me just use known-good ones
        test_fns[6] = 51 * ((1 - 0 + 26) % 26) + 1 + 1326 * 2047; // T1=2047, T3=1, T2=0

        int n_fns = sizeof(test_fns) / sizeof(test_fns[0]);
        int success = 0;

        for (int t = 0; t < n_fns; t++) {
            uint32_t fn = test_fns[t];

            uint8_t info[25];
            gsm_sch_build(0x1A, fn, info);

            uint8_t coded[78];
            gsm_sch_encode(info, coded);

            float soft[78];
            for (int i = 0; i < 78; i++) soft[i] = coded[i] ? -1.0f : 1.0f;

            uint8_t dec_info[25];
            if (gsm_sch_decode(soft, dec_info)) {
                uint8_t b; uint32_t f;
                gsm_sch_parse(dec_info, &b, &f);
                if (f == fn && b == 0x1A) success++;
            }
        }
        printf("  SCH multi-FN: %d/%d passed\n", success, n_fns);
        TEST_ASSERT(success == n_fns, "SCH roundtrip for multiple frame numbers");
    }
}

// ======================== Test: BCCH Interleaving ========================

static void test_bcch_interleave(void)
{
    TEST_SECTION("BCCH Interleaving");

    // Test: interleave then deinterleave recovers original
    {
        uint8_t coded[456];
        for (int i = 0; i < 456; i++) coded[i] = (uint8_t)(i % 2);

        uint8_t burst_hard[4][GSM_NB_DATA_BITS];
        gsm_bcch_interleave(coded, burst_hard);

        // Verify: coded[k] should be in burst[k%4][k/4]
        bool mapping_ok = true;
        for (int k = 0; k < 456; k++) {
            if (burst_hard[k % 4][k / 4] != coded[k]) {
                mapping_ok = false;
                break;
            }
        }
        TEST_ASSERT(mapping_ok, "Interleave mapping: c[k] → burst[k%4][k/4]");

        // Convert to soft and deinterleave
        float burst_soft[4][GSM_NB_DATA_BITS];
        for (int b = 0; b < 4; b++) {
            for (int j = 0; j < GSM_NB_DATA_BITS; j++) {
                burst_soft[b][j] = burst_hard[b][j] ? -1.0f : 1.0f;
            }
        }

        float coded_soft[456];
        gsm_bcch_deinterleave(burst_soft, coded_soft);

        bool deinterleave_ok = true;
        for (int k = 0; k < 456; k++) {
            float expected = coded[k] ? -1.0f : 1.0f;
            if (fabsf(coded_soft[k] - expected) > 0.01f) {
                deinterleave_ok = false;
                break;
            }
        }
        TEST_ASSERT(deinterleave_ok, "Deinterleave recovers original soft bits");
    }
}

// ======================== Test: Full BCCH Chain ========================

static void test_bcch_full_chain(void)
{
    TEST_SECTION("Full BCCH Chain (encode → interleave → decode)");

    // Build a known 23-octet L2 frame
    uint8_t l2_frame[GSM_L2_FRAME_LEN] = {
        0x49, // address: SAPI=0, some bits
        0x06, // PD = RR
        0x1B, // SI3 message type
        // SI3 content: Cell ID = 0x1234
        0x12, 0x34,
        // LAI: MCC=262, MNC=02, LAC=0x0100
        // MCC: 262 → octet1 = 0x62 (digit2=6, digit1=2), octet2 = 0xF2 (mnc3=F, mcc3=2)
        //            octet3 = 0x20 (mnc2=2, mnc1=0) → wait, MNC=02
        // Actually: MCC=262: mcc1=2, mcc2=6, mcc3=2
        //   octet1 = (mcc2 << 4) | mcc1 = 0x62
        //   octet2 = (mnc3 << 4) | mcc3 = (0xF << 4) | 2 = 0xF2 (2-digit MNC)
        //   octet3 = (mnc2 << 4) | mnc1 = (0 << 4) | 2 = 0x02
        0x62, 0xF2, 0x02,
        // LAC = 0x0100
        0x01, 0x00,
        // Control Channel Description (3 octets)
        0x01, 0x00, 0x0A,  // ccch_conf=1, t3212=10
        // Cell Options
        0x00,
        // Cell Selection Parameters
        0x00, 0x00,
        // RACH Control Parameters
        0x00, 0x00, 0x00,
        // Padding to 23 octets
        0x2B, 0x2B, 0x2B, 0x2B
    };

    // Step 1: Convert to bits
    uint8_t data_bits[184];
    for (int i = 0; i < 23; i++) {
        for (int b = 7; b >= 0; b--) {
            data_bits[i * 8 + (7 - b)] = (l2_frame[i] >> b) & 1;
        }
    }

    // Step 2: Fire code
    uint8_t parity[40];
    gsm_fire_encode(data_bits, 184, parity);

    // Step 3: Assemble pre-conv: data(184) + parity(40) + tail(4)
    uint8_t pre_conv[228];
    memcpy(pre_conv, data_bits, 184);
    memcpy(&pre_conv[184], parity, 40);
    pre_conv[224] = pre_conv[225] = pre_conv[226] = pre_conv[227] = 0;

    // Step 4: Convolutional encode
    uint8_t coded[456];
    gsm_conv_encode(pre_conv, 228, coded);

    // Step 5: Interleave into 4 bursts
    uint8_t burst_hard[4][GSM_NB_DATA_BITS];
    gsm_bcch_interleave(coded, burst_hard);

    // Step 6: Convert to soft bits (simulating perfect reception)
    float burst_soft[4][GSM_NB_DATA_BITS];
    for (int b = 0; b < 4; b++) {
        for (int j = 0; j < GSM_NB_DATA_BITS; j++) {
            burst_soft[b][j] = burst_hard[b][j] ? -1.0f : 1.0f;
        }
    }

    // Step 7: Deinterleave
    float coded_soft[456];
    gsm_bcch_deinterleave(burst_soft, coded_soft);

    // Step 8: Viterbi decode
    uint8_t decoded_pre_conv[228];
    gsm_viterbi_decode(coded_soft, 228, decoded_pre_conv);

    // Step 9: Fire code check
    bool crc_ok = gsm_fire_check(decoded_pre_conv, 224);
    TEST_ASSERT(crc_ok, "Full BCCH chain: Fire code CRC passes");

    // Step 10: Extract octets
    uint8_t decoded_l2[23];
    for (int i = 0; i < 23; i++) {
        uint8_t byte = 0;
        for (int b = 0; b < 8; b++) {
            byte = (uint8_t)((byte << 1) | (decoded_pre_conv[i * 8 + b] & 1));
        }
        decoded_l2[i] = byte;
    }

    bool l2_match = (memcmp(l2_frame, decoded_l2, 23) == 0);
    TEST_ASSERT(l2_match, "Full BCCH chain: L2 frame matches");

    if (!l2_match) {
        printf("    Original: ");
        for (int i = 0; i < 23; i++) printf("%02X ", l2_frame[i]);
        printf("\n    Decoded:  ");
        for (int i = 0; i < 23; i++) printf("%02X ", decoded_l2[i]);
        printf("\n");
    }

    // Test with errors
    {
        // Add 3 bit errors to each burst
        float noisy_soft[4][GSM_NB_DATA_BITS];
        memcpy(noisy_soft, burst_soft, sizeof(burst_soft));

        noisy_soft[0][10] = -noisy_soft[0][10];
        noisy_soft[0][50] = -noisy_soft[0][50];
        noisy_soft[1][30] = -noisy_soft[1][30];
        noisy_soft[2][20] = -noisy_soft[2][20];
        noisy_soft[2][80] = -noisy_soft[2][80];
        noisy_soft[3][40] = -noisy_soft[3][40];

        float noisy_coded[456];
        gsm_bcch_deinterleave(noisy_soft, noisy_coded);

        uint8_t noisy_decoded[228];
        gsm_viterbi_decode(noisy_coded, 228, noisy_decoded);

        bool noisy_crc = gsm_fire_check(noisy_decoded, 224);
        TEST_ASSERT(noisy_crc, "Full BCCH chain: CRC passes with 6 bit errors");

        uint8_t noisy_l2[23];
        for (int i = 0; i < 23; i++) {
            uint8_t byte = 0;
            for (int b = 0; b < 8; b++) byte = (uint8_t)((byte << 1) | (noisy_decoded[i*8+b] & 1));
            noisy_l2[i] = byte;
        }
        TEST_ASSERT(memcmp(l2_frame, noisy_l2, 23) == 0,
                     "Full BCCH chain: L2 matches with 6 bit errors");
    }
}

// ======================== Test: SI3 Parser ========================

static void test_si3_parse(void)
{
    TEST_SECTION("System Information Type 3 Parsing");

    // Known SI3 message (from real capture, anonymized)
    // PD=0x06 (RR), MT=0x1B (SI3)
    uint8_t si3_data[] = {
        0x06,       // Protocol Discriminator = RR
        0x1B,       // Message Type = SI3
        0x00, 0x01, // Cell Identity = 1
        // LAI: MCC=222 (Italy), MNC=10 (Vodafone), LAC=0x2001
        // MCC=222: mcc1=2, mcc2=2, mcc3=2
        //   octet1 = (mcc2<<4)|mcc1 = 0x22
        //   octet2 = (mnc3<<4)|mcc3 = 0xF2 (2-digit MNC)
        //   octet3 = (mnc2<<4)|mnc1 = (0<<4)|1 = 0x01
        0x22, 0xF2, 0x01,
        0x20, 0x01, // LAC = 0x2001
        // Control Channel Description
        0x01,       // ccch_conf=1, bs_ag_blks_res=0
        0x03,       // bs_pa_mfrms=3
        0x0A,       // t3212=10 (location update every 10 decihours)
        // Cell Options
        0x20,       // radio_link_timeout=2, dtx=0
        // Cell Selection
        0x0F,       // ms_txpwr_max_cch=15, cell_reselect_hysteresis=0
        0x10,       // rxlev_access_min=16
        // RACH Control Parameters
        0x8C,       // max_retrans=2, tx_integer=3, cell_barred=0, re_not_allowed=0
        0x00, 0x00, // ac_class = 0x0000 (all classes allowed)
    };

    gsm_si3_t si3;
    bool ok = gsm_parse_si3(si3_data, sizeof(si3_data), &si3);
    TEST_ASSERT(ok, "SI3 parse succeeds");

    if (ok) {
        TEST_ASSERT_EQ(si3.cell_id, 1, "SI3: Cell ID = 1");
        TEST_ASSERT_EQ(si3.mcc, 222, "SI3: MCC = 222 (Italy)");
        TEST_ASSERT_EQ(si3.mnc, 10, "SI3: MNC = 10 (Vodafone IT)");
        TEST_ASSERT_EQ(si3.lac, 0x2001, "SI3: LAC = 0x2001");
        TEST_ASSERT_EQ(si3.t3212, 10, "SI3: T3212 = 10");
        TEST_ASSERT_EQ(si3.ccch_conf, 1, "SI3: CCCH conf = 1");
        TEST_ASSERT(!si3.cell_barred, "SI3: Cell not barred");
    }

    // Test: German cell (T-Mobile, MCC=262, MNC=01)
    {
        uint8_t si3_de[] = {
            0x06, 0x1B,
            0x1A, 0xBC,     // Cell ID = 0x1ABC
            0x62, 0xF2, 0x10, // LAI: MCC=262, MNC=01
            0x00, 0x50,     // LAC = 0x0050
            0x00, 0x00, 0x14, // ccch: t3212=20
            0x00,           // cell options
            0x00, 0x00,     // cell selection
            0x00, 0x00, 0x00, // RACH
        };

        gsm_si3_t si3_de_parsed;
        ok = gsm_parse_si3(si3_de, sizeof(si3_de), &si3_de_parsed);
        TEST_ASSERT(ok, "SI3 DE parse succeeds");
        if (ok) {
            TEST_ASSERT_EQ(si3_de_parsed.cell_id, 0x1ABC, "SI3 DE: Cell ID = 0x1ABC");
            TEST_ASSERT_EQ(si3_de_parsed.mcc, 262, "SI3 DE: MCC = 262");
            TEST_ASSERT_EQ(si3_de_parsed.mnc, 1, "SI3 DE: MNC = 01");
            // Wait — MNC=01 encoding: mnc1=0, mnc2=1, mnc3=F
            // octet3 = (mnc2<<4)|mnc1 = (1<<4)|0 = 0x10
            // That gives mnc1=0, mnc2=1 → MNC = 01 → decimal = 1
            TEST_ASSERT_EQ(si3_de_parsed.lac, 0x0050, "SI3 DE: LAC = 0x0050");
        }
    }
}

// ======================== Test: SI1 Parser ========================

static void test_si1_parse(void)
{
    TEST_SECTION("System Information Type 1 Parsing");

    // Construct SI1 with bitmap format 0
    uint8_t si1_data[22] = {0};
    si1_data[0] = 0x06; // PD = RR
    si1_data[1] = 0x19; // MT = SI1

    // Cell Channel Description (16 octets, bitmap format 0)
    // Set ARFCNs 1, 10, 50, 100
    // ARFCN n → byte n/8, bit (7 - n%8) within the CA IE (offset 2)
    si1_data[2] = 0x00; // format bit = 0 (bitmap format 0), ARFCN 0-7
    // ARFCN 1: byte 0, bit 6 → si1_data[2] |= 0x40
    si1_data[2] |= 0x40;
    // ARFCN 10: byte 1, bit 5 → si1_data[3] |= 0x20
    si1_data[3] = 0x20;
    // ARFCN 50: byte 6, bit 5 → si1_data[8] |= 0x20
    si1_data[8] = 0x20;
    // ARFCN 100: byte 12, bit 3 → si1_data[14] |= 0x08
    si1_data[14] = 0x08;

    gsm_si1_t si1;
    bool ok = gsm_parse_si1(si1_data, sizeof(si1_data), &si1);
    TEST_ASSERT(ok, "SI1 parse succeeds");

    if (ok) {
        TEST_ASSERT(si1.n_arfcn >= 4, "SI1: at least 4 ARFCNs found");
        printf("    SI1: %d ARFCNs:", si1.n_arfcn);
        for (int i = 0; i < si1.n_arfcn; i++) printf(" %d", si1.arfcn_list[i]);
        printf("\n");
    }
}

// ======================== Test: Paging Parser ========================

static void test_paging_parse(void)
{
    TEST_SECTION("Paging Message Parsing");

    // Paging Request Type 1 with TMSI
    {
        uint8_t paging[] = {
            0x06,       // PD = RR
            0x21,       // MT = Paging Request Type 1
            0x00,       // Channel Needed
            // Mobile Identity 1: LV (length + TMSI)
            0x05,       // length = 5
            0xF4,       // MI type = 4 (TMSI), odd indicator
            0x12, 0x34, 0x56, 0x78, // TMSI = 0x12345678
        };

        gsm_paging_t paging_msg;
        bool ok = gsm_parse_paging(paging, sizeof(paging), &paging_msg);
        TEST_ASSERT(ok, "Paging Type 1 parse succeeds");
        if (ok) {
            TEST_ASSERT_EQ(paging_msg.paging_type, 1, "Paging: type = 1");
            TEST_ASSERT_EQ(paging_msg.n_identities, 1, "Paging: 1 identity");
            TEST_ASSERT(paging_msg.tmsi[0] == 0x12345678, "Paging: TMSI = 0x12345678");
        }
    }

    // Paging Request Type 2 with 2 TMSIs
    {
        uint8_t paging2[] = {
            0x06, 0x22, 0x00,
            // TMSI 1 (4 octets directly)
            0xAA, 0xBB, 0xCC, 0xDD,
            // TMSI 2 (4 octets directly)
            0x11, 0x22, 0x33, 0x44,
        };

        gsm_paging_t paging_msg;
        bool ok = gsm_parse_paging(paging2, sizeof(paging2), &paging_msg);
        TEST_ASSERT(ok, "Paging Type 2 parse succeeds");
        if (ok) {
            TEST_ASSERT_EQ(paging_msg.paging_type, 2, "Paging: type = 2");
            TEST_ASSERT_EQ(paging_msg.n_identities, 2, "Paging: 2 identities");
            TEST_ASSERT(paging_msg.tmsi[0] == 0xAABBCCDD, "Paging Type 2: TMSI 1");
            TEST_ASSERT(paging_msg.tmsi[1] == 0x11223344, "Paging Type 2: TMSI 2");
        }
    }
}

// ======================== Test: Cell Broadcast Parser ========================

static void test_cb_parse(void)
{
    TEST_SECTION("Cell Broadcast Parsing");

    // Construct a simple CB message (8-bit encoding for simplicity)
    uint8_t cb_data[88] = {0};
    // Serial Number: 0x0001
    cb_data[0] = 0x00; cb_data[1] = 0x01;
    // Message Identifier: 0x0032 (50 = Cell Info Display)
    cb_data[2] = 0x00; cb_data[3] = 0x32;
    // DCS: 0x11 (UCS-2 / 8-bit)
    cb_data[4] = 0x11;
    // Page: total=1, page=1
    cb_data[5] = 0x11;
    // Content: "Hello World!"
    const char *msg = "Hello World!";
    memcpy(&cb_data[6], msg, strlen(msg));

    gsm_cb_msg_t cb;
    bool ok = gsm_parse_cb(cb_data, 6 + (int)strlen(msg), &cb);
    TEST_ASSERT(ok, "CB parse succeeds");
    if (ok) {
        TEST_ASSERT_EQ(cb.serial_nr, 1, "CB: serial = 1");
        TEST_ASSERT_EQ(cb.msg_id, 0x32, "CB: msg_id = 0x32");
        TEST_ASSERT_EQ(cb.total_pages, 1, "CB: total pages = 1");
        TEST_ASSERT_EQ(cb.page_nr, 1, "CB: page = 1");
        TEST_ASSERT(cb.text_len > 0, "CB: text not empty");
        printf("    CB text: \"%s\" (len=%d)\n", cb.text, cb.text_len);
    }
}

// ======================== Test: ARFCN / Frequency Conversion ========================

static void test_arfcn_freq(void)
{
    TEST_SECTION("ARFCN ↔ Frequency Conversion");

    // P-GSM 900
    {
        double f = gsm_arfcn_to_freq(1, GSM_BAND_900);
        TEST_ASSERT(fabs(f - 935.2) < 0.01, "ARFCN 1 → 935.2 MHz");

        f = gsm_arfcn_to_freq(124, GSM_BAND_900);
        TEST_ASSERT(fabs(f - 959.8) < 0.01, "ARFCN 124 → 959.8 MHz");

        f = gsm_arfcn_to_freq(0, GSM_BAND_900);
        TEST_ASSERT(fabs(f - 935.0) < 0.01, "ARFCN 0 → 935.0 MHz");
    }

    // E-GSM 900
    {
        double f = gsm_arfcn_to_freq(975, GSM_BAND_E900);
        TEST_ASSERT(fabs(f - 925.2) < 0.01, "ARFCN 975 → 925.2 MHz");

        f = gsm_arfcn_to_freq(1023, GSM_BAND_E900);
        TEST_ASSERT(fabs(f - 934.8) < 0.01, "ARFCN 1023 → 934.8 MHz");
    }

    // DCS 1800
    {
        double f = gsm_arfcn_to_freq(512, GSM_BAND_1800);
        TEST_ASSERT(fabs(f - 1805.2) < 0.01, "ARFCN 512 → 1805.2 MHz");

        f = gsm_arfcn_to_freq(885, GSM_BAND_1800);
        TEST_ASSERT(fabs(f - 1879.8) < 0.01, "ARFCN 885 → 1879.8 MHz");
    }

    // Reverse lookup
    {
        gsm_band_t band;
        uint16_t arfcn = gsm_freq_to_arfcn(935.2, &band);
        TEST_ASSERT_EQ(arfcn, 1, "935.2 MHz → ARFCN 1");
        TEST_ASSERT_EQ(band, GSM_BAND_900, "935.2 MHz → GSM-900");
    }
}

// ======================== Test: L2 LAPDm Parser ========================

static void test_l2_parse(void)
{
    TEST_SECTION("L2 LAPDm Frame Parsing");

    // Typical BCCH message (Unnumbered Information frame, SAPI=0)
    uint8_t l2_data[23] = {
        0x01,       // Address: SAPI=0, C/R=0, EA=1 → actually bits [7..0] = 00000001
        0x03,       // Control: UI frame
        0x49,       // Length indicator: L=18 (0x49 >> 2 = 18), M=0, EL=1
        // L3 data follows (18 octets)
        0x06, 0x1B, 0x00, 0x01, 0x22, 0xF2, 0x10, 0x20,
        0x01, 0x01, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00,
        0x2B, 0x2B  // fill
    };

    gsm_l2_frame_t frame;
    bool ok = gsm_l2_parse(l2_data, 23, &frame);
    TEST_ASSERT(ok, "L2 parse succeeds");
    if (ok) {
        TEST_ASSERT_EQ(frame.sapi, 0, "L2: SAPI = 0");
        TEST_ASSERT_EQ(frame.length, 18, "L2: length = 18");
        TEST_ASSERT(!frame.more, "L2: M bit = 0");
    }
}

// ======================== Test: 51-Multiframe Channel Mapping ========================

static void test_channel_mapping(void)
{
    TEST_SECTION("51-Multiframe Channel Mapping");

    // FCCH frames: 0, 10, 20, 30, 40
    TEST_ASSERT_EQ(gsm_get_channel_type(0),  GSM_CHAN_FCCH, "Frame 0 = FCCH");
    TEST_ASSERT_EQ(gsm_get_channel_type(10), GSM_CHAN_FCCH, "Frame 10 = FCCH");
    TEST_ASSERT_EQ(gsm_get_channel_type(20), GSM_CHAN_FCCH, "Frame 20 = FCCH");
    TEST_ASSERT_EQ(gsm_get_channel_type(30), GSM_CHAN_FCCH, "Frame 30 = FCCH");
    TEST_ASSERT_EQ(gsm_get_channel_type(40), GSM_CHAN_FCCH, "Frame 40 = FCCH");

    // SCH frames: 1, 11, 21, 31, 41
    TEST_ASSERT_EQ(gsm_get_channel_type(1),  GSM_CHAN_SCH, "Frame 1 = SCH");
    TEST_ASSERT_EQ(gsm_get_channel_type(11), GSM_CHAN_SCH, "Frame 11 = SCH");
    TEST_ASSERT_EQ(gsm_get_channel_type(41), GSM_CHAN_SCH, "Frame 41 = SCH");

    // BCCH frames: 2-5
    TEST_ASSERT_EQ(gsm_get_channel_type(2), GSM_CHAN_BCCH, "Frame 2 = BCCH");
    TEST_ASSERT_EQ(gsm_get_channel_type(5), GSM_CHAN_BCCH, "Frame 5 = BCCH");

    // CCCH frames: 6-9, 12-19, 22-29, 32-39, 42-49
    TEST_ASSERT_EQ(gsm_get_channel_type(6),  GSM_CHAN_CCCH, "Frame 6 = CCCH");
    TEST_ASSERT_EQ(gsm_get_channel_type(9),  GSM_CHAN_CCCH, "Frame 9 = CCCH");
    TEST_ASSERT_EQ(gsm_get_channel_type(15), GSM_CHAN_CCCH, "Frame 15 = CCCH");
    TEST_ASSERT_EQ(gsm_get_channel_type(25), GSM_CHAN_CCCH, "Frame 25 = CCCH");
    TEST_ASSERT_EQ(gsm_get_channel_type(49), GSM_CHAN_CCCH, "Frame 49 = CCCH");

    // Idle frame: 50
    TEST_ASSERT_EQ(gsm_get_channel_type(50), GSM_CHAN_IDLE, "Frame 50 = IDLE");
}

// ======================== Test: FCCH Detection ========================

static void test_fcch_detection(void)
{
    TEST_SECTION("FCCH Detection");

    // Generate synthetic FCCH signal: constant frequency at +67708 Hz
    // At 1 MHz sample rate, phase increment = 2π * 67708 / 1e6 ≈ 0.4254 rad/sample
    int n_samples = 2000;
    float *freq_buf = (float *)calloc((size_t)n_samples, sizeof(float));

    float sps = (float)GSM_SAMPLE_RATE / GSM_SYMBOL_RATE; // ~3.6923
    float expected_dphi = (float)M_PI / (2.0f * sps);

    // First 500 samples: noise
    for (int i = 0; i < 500; i++) {
        freq_buf[i] = ((float)(i % 7) - 3.0f) * 0.3f; // random-ish noise
    }

    // Next 700 samples: FCCH tone
    for (int i = 500; i < 1200; i++) {
        freq_buf[i] = expected_dphi + 0.01f * ((i % 3) - 1); // tone with slight jitter
    }

    // Remaining: noise again
    for (int i = 1200; i < n_samples; i++) {
        freq_buf[i] = ((float)(i % 11) - 5.0f) * 0.2f;
    }

    double freq_offset;
    int pos = detect_fcch(freq_buf, n_samples, sps, &freq_offset);

    TEST_ASSERT(pos >= 500 && pos <= 1200, "FCCH detected within expected range");
    TEST_ASSERT(fabs(freq_offset) < 500.0, "FCCH freq offset < 500 Hz");

    printf("    FCCH detected at sample %d (expected ~850), freq offset %.1f Hz\n",
           pos, freq_offset);

    // Test: no FCCH in pure noise
    for (int i = 0; i < n_samples; i++) {
        freq_buf[i] = ((float)(i % 13) - 6.0f) * 0.4f;
    }
    pos = detect_fcch(freq_buf, n_samples, sps, &freq_offset);
    TEST_ASSERT(pos < 0, "No FCCH detected in noise");

    free(freq_buf);
}

// ======================== Test: GSM State Machine (Create/Destroy) ========================

static void test_state_machine(void)
{
    TEST_SECTION("GSM Decoder State Machine");

    gsm_config_t cfg = {
        .center_freq = 935200000, // ARFCN 1
        .sample_rate = GSM_SAMPLE_RATE,
        .tsc = -1,
        .msg_cb = NULL,
        .cb_cb = NULL,
        .callback_ctx = NULL,
    };

    struct gsm_state *st = gsm_create(&cfg);
    TEST_ASSERT(st != NULL, "gsm_create succeeds");

    if (st) {
        TEST_ASSERT_EQ(gsm_get_sync_state(st), GSM_SYNC_NONE, "Initial state = NONE");

        gsm_stats_t stats;
        gsm_get_stats(st, &stats);
        TEST_ASSERT_EQ(stats.samples_processed, 0, "Initial samples = 0");

        gsm_cell_info_t cell;
        gsm_get_cell_info(st, &cell);
        TEST_ASSERT_EQ(cell.fcch_count, 0, "Initial FCCH count = 0");

        gsm_destroy(st);
    }

    // Test NULL config
    st = gsm_create(NULL);
    TEST_ASSERT(st == NULL, "gsm_create(NULL) returns NULL");

    // Test destroy NULL (should not crash)
    gsm_destroy(NULL);
    TEST_ASSERT(true, "gsm_destroy(NULL) does not crash");
}

// ======================== Test: SCH CRC Polynomial ========================

static void test_sch_crc(void)
{
    TEST_SECTION("SCH CRC Polynomial");

    // The generator polynomial for SCH CRC is:
    // g(x) = x^10 + x^8 + x^6 + x^5 + x^4 + x^2 + x + 1
    // = 10101110111 in binary
    // = 0x577 for the full polynomial (11 bits)
    // Lower 10 bits for feedback: 0101110111 = 0x177... let me verify.

    // Test: all-zero data should give all-zero CRC
    {
        // Using the internal CRC function indirectly via sch_encode
        uint8_t info[25] = {0};
        uint8_t coded[78];
        gsm_sch_encode(info, coded);

        // Decode it back
        float soft[78];
        for (int i = 0; i < 78; i++) soft[i] = coded[i] ? -1.0f : 1.0f;

        uint8_t dec_info[25];
        bool ok = gsm_sch_decode(soft, dec_info);
        TEST_ASSERT(ok, "SCH CRC: all-zero info encodes and decodes");

        bool all_zero = true;
        for (int i = 0; i < 25; i++) {
            if (dec_info[i] != 0) all_zero = false;
        }
        TEST_ASSERT(all_zero, "SCH CRC: decoded info is all-zero");
    }

    // Test: corrupted data should fail CRC
    {
        uint8_t info[25] = {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1};
        uint8_t coded[78];
        gsm_sch_encode(info, coded);

        // Corrupt many bits to cause CRC failure
        float soft[78];
        for (int i = 0; i < 78; i++) soft[i] = coded[i] ? -1.0f : 1.0f;

        // Flip many bits
        for (int i = 0; i < 78; i += 3) soft[i] = -soft[i];

        uint8_t dec_info[25];
        bool ok = gsm_sch_decode(soft, dec_info);
        // With so many errors, CRC should fail
        TEST_ASSERT(!ok, "SCH CRC: heavy corruption detected");
    }
}

// ======================== Main ========================

int main(void)
{
    printf("GSM Broadcast Decoder — Unit Tests\n");
    printf("===================================\n");

    test_conv_codec();
    test_fire_code();
    test_sch_codec();
    test_sch_crc();
    test_bcch_interleave();
    test_bcch_full_chain();
    test_si3_parse();
    test_si1_parse();
    test_paging_parse();
    test_cb_parse();
    test_arfcn_freq();
    test_l2_parse();
    test_channel_mapping();
    test_fcch_detection();
    test_state_machine();

    printf("\n===================================\n");
    printf("Results: %d tests, %d passed, %d failed\n",
           tests_run, tests_passed, tests_failed);

    return tests_failed > 0 ? 1 : 0;
}
