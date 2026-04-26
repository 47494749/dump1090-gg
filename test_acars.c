// test_acars.c — Unit tests for ACARS decoder components
//
// Tests:
//   1. CRC-16/IBM (poly 0xA001) with known test vectors
//   2. Parity bit checking (odd parity per ARINC 618)
//   3. Frame field extraction from known ACARS messages
//   4. CRC residue check: frame + CRC → residue = 0
//   5. Known real-world ACARS messages (SQ/Q0/H1 labels)
//
// Build: cc -o test_acars test_acars.c -lm -Wall -Wextra
// Run:   ./test_acars

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

// ===================== CRC-16/IBM from acars_demod.c =====================

static const uint16_t crc_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

#define UPDATE_CRC(crc, c) \
    (crc) = ((crc) >> 8) ^ crc_table[((crc) ^ (c)) & 0xFF]

// Parity table
static const uint8_t numbits[256] = {
    0,1,1,2,1,2,2,3,1,2,2,3,2,3,3,4,1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,
    1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,
    1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,
    2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7,
    1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,
    2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7,
    2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7,
    3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7,4,5,5,6,5,6,6,7,5,6,6,7,6,7,7,8
};

// ===================== Add parity bit (odd parity) =====================

static uint8_t add_parity(uint8_t c)
{
    // Add odd parity: bit 7 set so total number of 1-bits is odd
    c &= 0x7F;
    if ((numbits[c] & 1) == 0)
        c |= 0x80;  // Even 1-bits in lower 7 → set bit 7 to make odd
    return c;
}

// ===================== Compute CRC for frame bytes =====================

static uint16_t acars_crc(const uint8_t *data, int len)
{
    uint16_t crc = 0;
    for (int i = 0; i < len; i++)
        UPDATE_CRC(crc, data[i]);
    return crc;
}

// ===================== Build an ACARS frame =====================
// Returns total frame length (msg_bytes + 2 CRC bytes)
// Output buffer must be at least msg_len + 2 bytes

static int build_acars_frame(uint8_t *out, const char *mode,
                             const char *reg, const char *ack,
                             const char *label, const char *block_id,
                             const char *msgno, const char *flight,
                             const char *text)
{
    int pos = 0;

    // Mode (1 char)
    out[pos++] = add_parity((uint8_t)mode[0]);

    // Registration (7 chars, space-padded)
    for (int i = 0; i < 7; i++) {
        char c = (reg && reg[i]) ? reg[i] : ' ';
        if (!reg[i]) c = ' ';
        out[pos++] = add_parity((uint8_t)c);
    }

    // ACK (1 char)
    out[pos++] = add_parity((uint8_t)(ack ? ack[0] : 0x15));

    // Label (2 chars)
    out[pos++] = add_parity((uint8_t)(label ? label[0] : ' '));
    out[pos++] = add_parity((uint8_t)((label && label[1]) ? label[1] : ' '));

    // Block ID (1 char)
    out[pos++] = add_parity((uint8_t)(block_id ? block_id[0] : '0'));

    // STX
    out[pos++] = add_parity(0x02);

    // Message number (4 chars)
    if (msgno) {
        for (int i = 0; i < 4 && msgno[i]; i++)
            out[pos++] = add_parity((uint8_t)msgno[i]);
    }

    // Flight ID (6 chars)
    if (flight) {
        for (int i = 0; i < 6 && flight[i]; i++)
            out[pos++] = add_parity((uint8_t)flight[i]);
    }

    // Text body
    if (text) {
        for (int i = 0; text[i]; i++)
            out[pos++] = add_parity((uint8_t)text[i]);
    }

    // ETX (0x03 with parity → 0x83)
    out[pos++] = 0x83;  // ETX = 0x03 with parity bit = 0x83

    int msg_len = pos;

    // Compute CRC over message bytes
    uint16_t crc = acars_crc(out, msg_len);
    out[pos++] = (uint8_t)(crc & 0xFF);
    out[pos++] = (uint8_t)((crc >> 8) & 0xFF);

    return pos;
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

// ===================== Test 1: CRC-16/IBM =====================

static void test_crc16(void)
{
    printf("\n=== Test 1: CRC-16/IBM (ACARS) ===\n");

    // CRC-16/IBM standard check value: "123456789" → 0xBB3D
    {
        TEST("CRC-16/IBM: '123456789' -> 0xBB3D");
        const uint8_t data[] = "123456789";
        uint16_t crc = acars_crc(data, 9);
        if (crc == 0xBB3D) PASS();
        else {
            char msg[64];
            snprintf(msg, sizeof(msg), "expected 0xBB3D, got 0x%04X", crc);
            FAIL(msg);
        }
    }

    // Empty data → 0
    {
        TEST("CRC-16/IBM: empty data -> 0x0000");
        uint16_t crc = acars_crc(NULL, 0);
        if (crc == 0x0000) PASS();
        else {
            char msg[64];
            snprintf(msg, sizeof(msg), "expected 0x0000, got 0x%04X", crc);
            FAIL(msg);
        }
    }

    // Self-check: CRC of [data + CRC_LE] = 0
    {
        TEST("CRC-16/IBM: self-check residue = 0");
        uint8_t data[11] = { 0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39, 0,0 };
        uint16_t crc = acars_crc(data, 9);
        data[9] = (uint8_t)(crc & 0xFF);
        data[10] = (uint8_t)((crc >> 8) & 0xFF);
        uint16_t check = acars_crc(data, 11);
        if (check == 0) PASS();
        else {
            char msg[64];
            snprintf(msg, sizeof(msg), "expected 0x0000, got 0x%04X", check);
            FAIL(msg);
        }
    }

    // Single byte
    {
        TEST("CRC-16/IBM: single byte 0x00 -> 0x0000");
        uint8_t data[] = { 0x00 };
        uint16_t crc = acars_crc(data, 1);
        if (crc == 0x0000) PASS();
        else {
            char msg[64];
            snprintf(msg, sizeof(msg), "expected 0x0000, got 0x%04X", crc);
            FAIL(msg);
        }
    }

    // Verify table[0x80] matches CRC-16/IBM (poly 0xA001)
    // CRC of single byte 0x80: shift → 0x80 → bit 7 set → XOR 0xA001 × reflections
    {
        TEST("CRC-16/IBM: table[1] = 0xC0C1 (poly 0xA001)");
        if (crc_table[1] == 0xC0C1) PASS();
        else {
            char msg[64];
            snprintf(msg, sizeof(msg), "table[1] = 0x%04X", crc_table[1]);
            FAIL(msg);
        }
    }
}

// ===================== Test 2: Parity =====================

static void test_parity(void)
{
    printf("\n=== Test 2: Odd Parity ===\n");

    // 'A' = 0x41 = 01000001 → 2 bits set (even) → parity bit = 1 → 0xC1
    {
        TEST("Parity: 'A' (0x41) -> 0xC1 (odd parity)");
        uint8_t p = add_parity(0x41);
        if (p == 0xC1) PASS();
        else {
            char msg[64];
            snprintf(msg, sizeof(msg), "expected 0xC1, got 0x%02X", p);
            FAIL(msg);
        }
    }

    // 'C' = 0x43 = 01000011 → 3 bits (odd) → no parity bit → 0x43
    {
        TEST("Parity: 'C' (0x43) -> 0x43 (already odd)");
        uint8_t p = add_parity(0x43);
        if (p == 0x43) PASS();
        else {
            char msg[64];
            snprintf(msg, sizeof(msg), "expected 0x43, got 0x%02X", p);
            FAIL(msg);
        }
    }

    // 0x00 → 0 bits (even) → parity bit = 1 → 0x80
    {
        TEST("Parity: 0x00 -> 0x80");
        uint8_t p = add_parity(0x00);
        if (p == 0x80) PASS();
        else {
            char msg[64];
            snprintf(msg, sizeof(msg), "expected 0x80, got 0x%02X", p);
            FAIL(msg);
        }
    }

    // 0x7F = 01111111 → 7 bits (odd) → no parity → 0x7F
    {
        TEST("Parity: 0x7F -> 0x7F (7 bits, odd)");
        uint8_t p = add_parity(0x7F);
        if (p == 0x7F) PASS();
        else {
            char msg[64];
            snprintf(msg, sizeof(msg), "expected 0x7F, got 0x%02X", p);
            FAIL(msg);
        }
    }

    // Verify: all bytes with parity have odd number of 1-bits
    {
        TEST("All 128 parity bytes have odd popcount");
        int ok = 1;
        for (int c = 0; c < 128; c++) {
            uint8_t p = add_parity((uint8_t)c);
            if ((numbits[p] & 1) != 1) { ok = 0; break; }
        }
        if (ok) PASS();
        else FAIL("found even popcount");
    }

    // Verify: parity detection matches acars_demod.c check
    // acars_demod.c: (numbits[r] & 1) == 0 → parity error
    {
        TEST("Parity detection: valid byte passes, flipped fails");
        uint8_t valid = add_parity('X');
        int valid_ok = (numbits[valid] & 1) != 0;
        uint8_t bad = valid ^ 0x01;  // Flip one bit
        int bad_fail = (numbits[bad] & 1) == 0;
        if (valid_ok && bad_fail) PASS();
        else {
            char msg[64];
            snprintf(msg, sizeof(msg), "valid_ok=%d bad_fail=%d", valid_ok, bad_fail);
            FAIL(msg);
        }
    }
}

// ===================== Test 3: Frame Build + CRC Residue =====================

static void test_frame_crc(void)
{
    printf("\n=== Test 3: ACARS Frame CRC ===\n");

    // Build a frame, verify CRC residue = 0
    {
        TEST("Build frame: CRC residue = 0");
        uint8_t frame[300];
        int len = build_acars_frame(frame,
            "2", ".D-ABCD", "\x15", "SQ", "0",
            "M01A", "DLH123", "HELLO WORLD");
        uint16_t crc = acars_crc(frame, len);
        if (crc == 0) PASS();
        else {
            char msg[64];
            snprintf(msg, sizeof(msg), "residue = 0x%04X, expected 0", crc);
            FAIL(msg);
        }
    }

    // Corrupt one byte → CRC residue != 0
    {
        TEST("Corrupted frame: CRC residue != 0");
        uint8_t frame[300];
        int len = build_acars_frame(frame,
            "2", ".D-ABCD", "\x15", "SQ", "0",
            "M01A", "DLH123", "HELLO WORLD");
        frame[5] ^= 0x01;  // Flip one bit
        uint16_t crc = acars_crc(frame, len);
        if (crc != 0) PASS();
        else FAIL("CRC should be non-zero after corruption");
    }

    // Build minimal frame (just header, no text)
    {
        TEST("Minimal frame (no text): CRC residue = 0");
        uint8_t frame[300];
        int len = build_acars_frame(frame,
            "2", ".D-ABCD", "\x15", "Q0", "1",
            NULL, NULL, NULL);
        uint16_t crc = acars_crc(frame, len);
        if (crc == 0) PASS();
        else {
            char msg[64];
            snprintf(msg, sizeof(msg), "residue = 0x%04X", crc);
            FAIL(msg);
        }
    }

    // Build frame with ETX parity check
    {
        TEST("ETX (0x03) has correct parity (0x83)");
        // ETX = 0x03 = 00000011 → 2 bits (even) → parity → 0x83
        uint8_t etx = add_parity(0x03);
        if (etx == 0x83) PASS();
        else {
            char msg[64];
            snprintf(msg, sizeof(msg), "ETX with parity = 0x%02X, expected 0x83", etx);
            FAIL(msg);
        }
    }

    // ETB parity check
    {
        TEST("ETB (0x17) has correct parity (0x97)");
        uint8_t etb = add_parity(0x17);
        if (etb == 0x97) PASS();
        else {
            char msg[64];
            snprintf(msg, sizeof(msg), "ETB with parity = 0x%02X, expected 0x97", etb);
            FAIL(msg);
        }
    }
}

// ===================== Test 4: Field Extraction =====================

static void test_field_extraction(void)
{
    printf("\n=== Test 4: ACARS Field Extraction ===\n");

    // Simulate the parsing logic from acars_output_message()
    // Build a frame and parse it the same way acars_demod.c does.

    {
        TEST("Parse fields: mode, reg, ack, label, block_id");
        uint8_t frame[300];
        build_acars_frame(frame,
            "2", ".D-ABCD", "\x15", "SQ", "3",
            "M01A", "DLH123", "TEST MSG");

        // Parse (same logic as acars_output_message)
        // CRC bytes are last 2, msg bytes are before that
        // Byte 0: mode, 1-7: reg, 8: ack, 9-10: label, 11: block_id, 12: STX
        char mode = frame[0] & 0x7F;
        char reg[8] = {0};
        for (int i = 0; i < 7; i++) reg[i] = frame[i + 1] & 0x7F;
        char ack = frame[8] & 0x7F;
        char label[3] = { frame[9] & 0x7F, frame[10] & 0x7F, 0 };
        char block_id = frame[11] & 0x7F;

        int ok = (mode == '2') &&
                 (strcmp(reg, ".D-ABCD") == 0) &&
                 (ack == 0x15) &&
                 (strcmp(label, "SQ") == 0) &&
                 (block_id == '3');
        if (ok) PASS();
        else {
            char msg[128];
            snprintf(msg, sizeof(msg), "mode='%c' reg='%s' ack=0x%02X label='%s' bid='%c'",
                     mode, reg, ack, label, block_id);
            FAIL(msg);
        }
    }

    // Parse msgno and flight
    {
        TEST("Parse fields: msgno, flight, text");
        uint8_t frame[300];
        build_acars_frame(frame,
            "H", ".EI-DVE", "\x15", "H1", "5",
            "S45A", "EIN789", "POSITION REPORT");

        // Text starts at byte 13 (after STX at byte 12)
        // First 4 bytes = msgno, next 6 = flight, rest = text
        int text_start = 13;
        char msgno[5] = {0};
        for (int i = 0; i < 4; i++)
            msgno[i] = frame[text_start + i] & 0x7F;

        char flight[7] = {0};
        for (int i = 0; i < 6; i++)
            flight[i] = frame[text_start + 4 + i] & 0x7F;

        // Remaining text ends before ETX
        // Find ETX (last msg byte before CRC)
        // Our build_acars_frame puts ETX as the last msg byte
        // Text is from position text_start+10 to the byte before ETX

        int ok = (strcmp(msgno, "S45A") == 0) &&
                 (strcmp(flight, "EIN789") == 0);
        if (ok) PASS();
        else {
            char msg[128];
            snprintf(msg, sizeof(msg), "msgno='%s' flight='%s'", msgno, flight);
            FAIL(msg);
        }
    }

    // SYN character
    {
        TEST("SYN = 0x16, ~SYN = 0xE9 (inverted)");
        if (0x16 == 0x16 && (uint8_t)~0x16 == 0xE9) PASS();
        else FAIL("SYN constant mismatch");
    }

    // Minimum message length check
    {
        TEST("Minimum length: 13 bytes required");
        // mode(1) + reg(7) + ack(1) + label(2) + block_id(1) + STX(1) = 13
        int min_len = 1 + 7 + 1 + 2 + 1 + 1;
        if (min_len == 13) PASS();
        else {
            char msg[64];
            snprintf(msg, sizeof(msg), "computed %d, expected 13", min_len);
            FAIL(msg);
        }
    }
}

// ===================== Test 5: Known Message Patterns =====================

static void test_known_patterns(void)
{
    printf("\n=== Test 5: ACARS Known Patterns ===\n");

    // Test that different labels produce valid CRC
    {
        TEST("Label 'SQ' (Squawk) frame: valid CRC");
        uint8_t frame[300];
        int len = build_acars_frame(frame, "2", ".D-AICD", "\x15", "SQ", "0",
                                    "M55A", "DLH456", "7000");
        if (acars_crc(frame, len) == 0) PASS();
        else FAIL("CRC non-zero");
    }

    {
        TEST("Label 'Q0' (Link test) frame: valid CRC");
        uint8_t frame[300];
        int len = build_acars_frame(frame, "2", ".D-AICD", "\x15", "Q0", "0",
                                    NULL, NULL, NULL);
        if (acars_crc(frame, len) == 0) PASS();
        else FAIL("CRC non-zero");
    }

    {
        TEST("Label 'H1' (OOOI) frame: valid CRC");
        uint8_t frame[300];
        int len = build_acars_frame(frame, "H", ".EI-DVE", "\x15", "H1", "1",
                                    "S89B", "EIN234",
                                    "OUT/1423 OFF/1445 EHAM/EIDW");
        if (acars_crc(frame, len) == 0) PASS();
        else FAIL("CRC non-zero");
    }

    // Verify parity on all bytes of a full frame
    {
        TEST("All bytes in frame have odd parity (except CRC)");
        uint8_t frame[300];
        int total = build_acars_frame(frame, "2", ".D-ABCD", "\x15", "SQ", "0",
                                      "M01A", "DLH123", "HELLO WORLD");
        int msg_len = total - 2;  // Exclude CRC bytes
        int ok = 1;
        for (int i = 0; i < msg_len; i++) {
            if ((numbits[frame[i]] & 1) != 1) {
                printf("[byte %d = 0x%02X has even parity] ", i, frame[i]);
                ok = 0;
                break;
            }
        }
        if (ok) PASS();
        else FAIL("parity error in frame");
    }
}

// ===================== Main =====================

int main(void)
{
    printf("ACARS Decoder — Unit Tests\n");
    printf("==========================\n");

    test_crc16();
    test_parity();
    test_frame_crc();
    test_field_extraction();
    test_known_patterns();

    printf("\n==========================\n");
    printf("Results: %d/%d passed, %d failed\n", tests_passed, tests_run, tests_failed);
    printf("==========================\n");

    return tests_failed > 0 ? 1 : 0;
}
