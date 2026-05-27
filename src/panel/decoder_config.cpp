// Part of dump1090-gg-light
//
// decoder_config.c: Per-decoder configuration management
//
// Loads and saves decoder-specific configurations from /etc/dump1090-gg/decoders.json
// Each decoder type has its own section with its specific options.

#include "decoder_config.h"
#include "dump1090_defs.h"
#include <stdint.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctype.h>
#include <cerrno>
#include "gg_format.h"

#if MODES_ENABLE_DIAGNOSTICS
#define DECODER_CONFIG_DIAG(...) gg::eprint(__VA_ARGS__)
#else
#define DECODER_CONFIG_DIAG(...) do {} while (0)
#endif

// Global decoder config instance
all_decoder_configs_t DecoderConfigs;

// ======================== Defaults ========================

void decoderConfigInit(void)
{
    DecoderConfigs = {};

    // ADS-B defaults
    DecoderConfigs.adsb.adaptive_range = true;
    DecoderConfigs.adsb.adaptive_burst = true;
    DecoderConfigs.adsb.adaptive_min_gain = 0.0f;
    DecoderConfigs.adsb.adaptive_max_gain = 99999.0f;
    DecoderConfigs.adsb.crc_rescue = true;
    DecoderConfigs.adsb.fix_crc = 1;
    DecoderConfigs.adsb.mode_ac = false;
    DecoderConfigs.adsb.mode_ac_auto = true;

    // FLARM defaults
    DecoderConfigs.flarm.enabled = false;
    DecoderConfigs.flarm.ogn_only = false;
    snprintf(DecoderConfigs.flarm.ogn_server, sizeof(DecoderConfigs.flarm.ogn_server),
             "aprs.glidernet.org");
    DecoderConfigs.flarm.ogn_port = 14580;
    DecoderConfigs.flarm.keys_loaded = false;
    snprintf(DecoderConfigs.flarm.keys_file, sizeof(DecoderConfigs.flarm.keys_file),
             "/etc/dump1090-gg/flarm_keys.conf");

    // ACARS defaults
    DecoderConfigs.acars.enabled = true;
    DecoderConfigs.acars.num_channels = 5;
    DecoderConfigs.acars.channel_freqs[0] = 131550000;
    DecoderConfigs.acars.channel_freqs[1] = 130025000;
    DecoderConfigs.acars.channel_freqs[2] = 131725000;
    DecoderConfigs.acars.channel_freqs[3] = 130450000;
    DecoderConfigs.acars.channel_freqs[4] = 129125000;
    DecoderConfigs.acars.center_freq = 130425000;

    // VDL2 defaults
    DecoderConfigs.vdl2.enabled = true;
    DecoderConfigs.vdl2.num_channels = 3;
    DecoderConfigs.vdl2.channel_freqs[0] = 136975000;
    DecoderConfigs.vdl2.channel_freqs[1] = 136875000;
    DecoderConfigs.vdl2.channel_freqs[2] = 136775000;
    DecoderConfigs.vdl2.center_freq = 136875000;
    DecoderConfigs.vdl2.squelch_level = -32.0f;

    // Radiosonde defaults
    DecoderConfigs.radiosonde.enabled = true;
    DecoderConfigs.radiosonde.sondehub_upload = true;
    DecoderConfigs.radiosonde.radiosondy_upload = false;
    DecoderConfigs.radiosonde.wettersonde_upload = false;
    DecoderConfigs.radiosonde.center_freq = 403000000;

    // POCSAG defaults
    DecoderConfigs.pocsag.enabled = true;
    DecoderConfigs.pocsag.output_enabled = true;
    DecoderConfigs.pocsag.num_channels = 3;
    DecoderConfigs.pocsag.channel_freqs[0] = 466075000;
    DecoderConfigs.pocsag.channel_freqs[1] = 466175000;
    DecoderConfigs.pocsag.channel_freqs[2] = 466225000;
    DecoderConfigs.pocsag.center_freq = 466150000;

    // GSM defaults
    DecoderConfigs.gsm.enabled = true;
    DecoderConfigs.gsm.output_enabled = true;
    DecoderConfigs.gsm.arfcn_freq = 947000000;
    DecoderConfigs.gsm.tsc = -1;

    // LTE defaults
    DecoderConfigs.lte.enabled = true;
    DecoderConfigs.lte.output_enabled = true;
    DecoderConfigs.lte.hop_enabled = true;
    DecoderConfigs.lte.center_freq = 806000000;

    // IoT 868 defaults
    DecoderConfigs.iot868.enabled = true;
    DecoderConfigs.iot868.output_enabled = true;
    DecoderConfigs.iot868.center_freq = 868300000;

    // FANET defaults
    DecoderConfigs.fanet.enabled = true;
    DecoderConfigs.fanet.output_enabled = true;
    DecoderConfigs.fanet.center_freq = 868200000;

    // Sarsat defaults
    DecoderConfigs.sarsat.enabled = true;
    DecoderConfigs.sarsat.output_enabled = true;
    DecoderConfigs.sarsat.center_freq = 406040000;
}

// ======================== JSON Helpers ========================

static void skip_ws(const char **p) { while (**p && isspace((uint8_t)**p)) (*p)++; }

static bool match_key(const char **p, const char *key)
{
    skip_ws(p);
    if (**p != '"') return false;
    (*p)++;
    size_t klen = strlen(key);
    if (strncmp(*p, key, klen) != 0) return false;
    *p += klen;
    if (**p != '"') return false;
    (*p)++;
    skip_ws(p);
    if (**p != ':') return false;
    (*p)++;
    skip_ws(p);
    return true;
}

static bool parse_bool(const char **p)
{
    skip_ws(p);
    if (strncmp(*p, "true", 4) == 0) { *p += 4; return true; }
    if (strncmp(*p, "false", 5) == 0) { *p += 5; return false; }
    return false;
}

static double parse_number(const char **p)
{
    skip_ws(p);
    char *end;
    double val = strtod(*p, &end);
    *p = end;
    return val;
}

static int32_t parse_int(const char **p)
{
    return (int32_t)parse_number(p);
}

static void parse_string(const char **p, char *out, int32_t maxlen)
{
    skip_ws(p);
    if (**p != '"') { out[0] = '\0'; return; }
    (*p)++;
    int32_t i = 0;
    while (**p && **p != '"' && i < maxlen - 1) {
        if (**p == '\\' && *(*p + 1)) {
            (*p)++;
            if (**p == 'n') out[i++] = '\n';
            else if (**p == 't') out[i++] = '\t';
            else out[i++] = **p;
        } else {
            out[i++] = **p;
        }
        (*p)++;
    }
    out[i] = '\0';
    if (**p == '"') (*p)++;
}

static uint32_t parse_hex(const char **p)
{
    skip_ws(p);
    // skip optional "0x" prefix or opening quote
    if (**p == '"') (*p)++;
    if ((*p)[0] == '0' && ((*p)[1] == 'x' || (*p)[1] == 'X')) *p += 2;
    char *end;
    uint32_t val = (uint32_t)strtoul(*p, &end, 16);
    *p = end;
    if (**p == '"') (*p)++;
    return val;
}

// Skip a JSON value (string, number, bool, null, object, array)
static void skip_value(const char **p)
{
    skip_ws(p);
    if (**p == '"') {
        (*p)++;
        while (**p && **p != '"') { if (**p == '\\') (*p)++; (*p)++; }
        if (**p == '"') (*p)++;
    } else if (**p == '{') {
        int32_t depth = 1; (*p)++;
        while (**p && depth > 0) {
            if (**p == '{') depth++;
            else if (**p == '}') depth--;
            else if (**p == '"') { (*p)++; while (**p && **p != '"') { if (**p == '\\') (*p)++; (*p)++; } }
            (*p)++;
        }
    } else if (**p == '[') {
        int32_t depth = 1; (*p)++;
        while (**p && depth > 0) {
            if (**p == '[') depth++;
            else if (**p == ']') depth--;
            else if (**p == '"') { (*p)++; while (**p && **p != '"') { if (**p == '\\') (*p)++; (*p)++; } }
            (*p)++;
        }
    } else {
        while (**p && **p != ',' && **p != '}' && **p != ']') (*p)++;
    }
}

// ======================== Parse Sections ========================

static void parse_adsb(const char **p)
{
    skip_ws(p);
    if (**p != '{') return;
    (*p)++;

    while (**p && **p != '}') {
        skip_ws(p);
        if (**p == ',') { (*p)++; continue; }
        if (match_key(p, "adaptive_range")) DecoderConfigs.adsb.adaptive_range = parse_bool(p);
        else if (match_key(p, "adaptive_burst")) DecoderConfigs.adsb.adaptive_burst = parse_bool(p);
        else if (match_key(p, "adaptive_min_gain")) DecoderConfigs.adsb.adaptive_min_gain = (float)parse_number(p);
        else if (match_key(p, "adaptive_max_gain")) DecoderConfigs.adsb.adaptive_max_gain = (float)parse_number(p);
        else if (match_key(p, "crc_rescue")) DecoderConfigs.adsb.crc_rescue = parse_bool(p);
        else if (match_key(p, "fix_crc")) DecoderConfigs.adsb.fix_crc = parse_int(p);
        else if (match_key(p, "mode_ac")) DecoderConfigs.adsb.mode_ac = parse_bool(p);
        else if (match_key(p, "mode_ac_auto")) DecoderConfigs.adsb.mode_ac_auto = parse_bool(p);
        else { skip_value(p); }
    }
    if (**p == '}') (*p)++;
}

static void parse_flarm(const char **p)
{
    skip_ws(p);
    if (**p != '{') return;
    (*p)++;

    while (**p && **p != '}') {
        skip_ws(p);
        if (**p == ',') { (*p)++; continue; }
        if (match_key(p, "enabled")) DecoderConfigs.flarm.enabled = parse_bool(p);
        else if (match_key(p, "ogn_only")) DecoderConfigs.flarm.ogn_only = parse_bool(p);
        else if (match_key(p, "ogn_server")) parse_string(p, DecoderConfigs.flarm.ogn_server, sizeof(DecoderConfigs.flarm.ogn_server));
        else if (match_key(p, "ogn_port")) DecoderConfigs.flarm.ogn_port = parse_int(p);
        else if (match_key(p, "ogn_station")) parse_string(p, DecoderConfigs.flarm.ogn_station, sizeof(DecoderConfigs.flarm.ogn_station));
        else if (match_key(p, "keys_file")) parse_string(p, DecoderConfigs.flarm.keys_file, sizeof(DecoderConfigs.flarm.keys_file));
        else if (match_key(p, "key_table")) {
            // Parse "hex,hex,hex,..." or [hex,hex,...]
            skip_ws(p);
            if (**p == '"') {
                (*p)++;
                for (int32_t i = 0; i < 12; i++) {
                    while (**p && !isxdigit((uint8_t)**p) && **p != '"') (*p)++;
                    if (**p == '"') break;
                    char *end;
                    DecoderConfigs.flarm.key_table[i] = (uint32_t)strtoul(*p, &end, 16);
                    *p = end;
                }
                if (**p == '"') (*p)++;
            } else if (**p == '[') {
                (*p)++;
                for (int32_t i = 0; i < 12; i++) {
                    skip_ws(p);
                    if (**p == ']') break;
                    if (**p == ',') (*p)++;
                    DecoderConfigs.flarm.key_table[i] = parse_hex(p);
                }
                skip_ws(p);
                if (**p == ']') (*p)++;
            }
        }
        else if (match_key(p, "key2")) DecoderConfigs.flarm.key2 = parse_hex(p);
        else if (match_key(p, "key3")) DecoderConfigs.flarm.key3 = parse_hex(p);
        else if (match_key(p, "key4")) DecoderConfigs.flarm.key4 = parse_hex(p);
        else if (match_key(p, "key5")) {
            skip_ws(p);
            if (**p == '"') {
                (*p)++;
                for (int32_t i = 0; i < 4; i++) {
                    while (**p && !isxdigit((uint8_t)**p) && **p != '"') (*p)++;
                    if (**p == '"') break;
                    char *end;
                    DecoderConfigs.flarm.key5[i] = (uint32_t)strtoul(*p, &end, 16);
                    *p = end;
                }
                if (**p == '"') (*p)++;
            } else if (**p == '[') {
                (*p)++;
                for (int32_t i = 0; i < 4; i++) {
                    skip_ws(p);
                    if (**p == ']') break;
                    if (**p == ',') (*p)++;
                    DecoderConfigs.flarm.key5[i] = parse_hex(p);
                }
                skip_ws(p);
                if (**p == ']') (*p)++;
            }
        }
        else { skip_value(p); }
    }
    if (**p == '}') (*p)++;

    // Check if keys are present
    bool has_keys = false;
    for (int32_t i = 0; i < 12; i++) { if (DecoderConfigs.flarm.key_table[i]) { has_keys = true; break; } }
    if (has_keys && DecoderConfigs.flarm.key2) DecoderConfigs.flarm.keys_loaded = true;
}

static void parse_acars(const char **p)
{
    skip_ws(p);
    if (**p != '{') return;
    (*p)++;

    while (**p && **p != '}') {
        skip_ws(p);
        if (**p == ',') { (*p)++; continue; }
        if (match_key(p, "enabled")) DecoderConfigs.acars.enabled = parse_bool(p);
        else if (match_key(p, "center_freq")) DecoderConfigs.acars.center_freq = parse_number(p);
        else if (match_key(p, "channels")) {
            skip_ws(p);
            if (**p == '[') {
                (*p)++;
                DecoderConfigs.acars.num_channels = 0;
                while (**p && **p != ']' && DecoderConfigs.acars.num_channels < 8) {
                    skip_ws(p);
                    if (**p == ',') { (*p)++; continue; }
                    DecoderConfigs.acars.channel_freqs[DecoderConfigs.acars.num_channels++] = parse_number(p);
                }
                if (**p == ']') (*p)++;
            }
        }
        else { skip_value(p); }
    }
    if (**p == '}') (*p)++;
}

static void parse_vdl2(const char **p)
{
    skip_ws(p);
    if (**p != '{') return;
    (*p)++;

    while (**p && **p != '}') {
        skip_ws(p);
        if (**p == ',') { (*p)++; continue; }
        if (match_key(p, "enabled")) DecoderConfigs.vdl2.enabled = parse_bool(p);
        else if (match_key(p, "center_freq")) DecoderConfigs.vdl2.center_freq = parse_number(p);
        else if (match_key(p, "squelch_level")) DecoderConfigs.vdl2.squelch_level = (float)parse_number(p);
        else if (match_key(p, "channels")) {
            skip_ws(p);
            if (**p == '[') {
                (*p)++;
                DecoderConfigs.vdl2.num_channels = 0;
                while (**p && **p != ']' && DecoderConfigs.vdl2.num_channels < 8) {
                    skip_ws(p);
                    if (**p == ',') { (*p)++; continue; }
                    DecoderConfigs.vdl2.channel_freqs[DecoderConfigs.vdl2.num_channels++] = parse_number(p);
                }
                if (**p == ']') (*p)++;
            }
        }
        else { skip_value(p); }
    }
    if (**p == '}') (*p)++;
}

static void parse_radiosonde(const char **p)
{
    skip_ws(p);
    if (**p != '{') return;
    (*p)++;

    while (**p && **p != '}') {
        skip_ws(p);
        if (**p == ',') { (*p)++; continue; }
        if (match_key(p, "enabled")) DecoderConfigs.radiosonde.enabled = parse_bool(p);
        else if (match_key(p, "sondehub_upload")) DecoderConfigs.radiosonde.sondehub_upload = parse_bool(p);
        else if (match_key(p, "radiosondy_upload")) DecoderConfigs.radiosonde.radiosondy_upload = parse_bool(p);
        else if (match_key(p, "wettersonde_upload")) DecoderConfigs.radiosonde.wettersonde_upload = parse_bool(p);
        else if (match_key(p, "callsign")) parse_string(p, DecoderConfigs.radiosonde.callsign, sizeof(DecoderConfigs.radiosonde.callsign));
        else if (match_key(p, "center_freq")) DecoderConfigs.radiosonde.center_freq = parse_number(p);
        else { skip_value(p); }
    }
    if (**p == '}') (*p)++;
}

static void parse_pocsag(const char **p)
{
    skip_ws(p);
    if (**p != '{') return;
    (*p)++;

    while (**p && **p != '}') {
        skip_ws(p);
        if (**p == ',') { (*p)++; continue; }
        if (match_key(p, "enabled")) DecoderConfigs.pocsag.enabled = parse_bool(p);
        else if (match_key(p, "output_enabled")) DecoderConfigs.pocsag.output_enabled = parse_bool(p);
        else if (match_key(p, "center_freq")) DecoderConfigs.pocsag.center_freq = parse_number(p);
        else if (match_key(p, "channels")) {
            skip_ws(p);
            if (**p == '[') {
                (*p)++;
                DecoderConfigs.pocsag.num_channels = 0;
                while (**p && **p != ']' && DecoderConfigs.pocsag.num_channels < 8) {
                    skip_ws(p);
                    if (**p == ',') { (*p)++; continue; }
                    DecoderConfigs.pocsag.channel_freqs[DecoderConfigs.pocsag.num_channels++] = parse_number(p);
                }
                if (**p == ']') (*p)++;
            }
        }
        else { skip_value(p); }
    }
    if (**p == '}') (*p)++;
}

static void parse_gsm(const char **p)
{
    skip_ws(p);
    if (**p != '{') return;
    (*p)++;

    while (**p && **p != '}') {
        skip_ws(p);
        if (**p == ',') { (*p)++; continue; }
        if (match_key(p, "enabled")) DecoderConfigs.gsm.enabled = parse_bool(p);
        else if (match_key(p, "output_enabled")) DecoderConfigs.gsm.output_enabled = parse_bool(p);
        else if (match_key(p, "arfcn_freq")) DecoderConfigs.gsm.arfcn_freq = parse_number(p);
        else if (match_key(p, "tsc")) DecoderConfigs.gsm.tsc = parse_int(p);
        else { skip_value(p); }
    }
    if (**p == '}') (*p)++;
}

static void parse_lte(const char **p)
{
    skip_ws(p);
    if (**p != '{') return;
    (*p)++;

    while (**p && **p != '}') {
        skip_ws(p);
        if (**p == ',') { (*p)++; continue; }
        if (match_key(p, "enabled")) DecoderConfigs.lte.enabled = parse_bool(p);
        else if (match_key(p, "output_enabled")) DecoderConfigs.lte.output_enabled = parse_bool(p);
        else if (match_key(p, "hop_enabled")) DecoderConfigs.lte.hop_enabled = parse_bool(p);
        else if (match_key(p, "center_freq")) DecoderConfigs.lte.center_freq = parse_number(p);
        else { skip_value(p); }
    }
    if (**p == '}') (*p)++;
}

static void parse_iot868(const char **p)
{
    skip_ws(p);
    if (**p != '{') return;
    (*p)++;

    while (**p && **p != '}') {
        skip_ws(p);
        if (**p == ',') { (*p)++; continue; }
        if (match_key(p, "enabled")) DecoderConfigs.iot868.enabled = parse_bool(p);
        else if (match_key(p, "output_enabled")) DecoderConfigs.iot868.output_enabled = parse_bool(p);
        else if (match_key(p, "center_freq")) DecoderConfigs.iot868.center_freq = parse_number(p);
        else { skip_value(p); }
    }
    if (**p == '}') (*p)++;
}

static void parse_fanet(const char **p)
{
    skip_ws(p);
    if (**p != '{') return;
    (*p)++;

    while (**p && **p != '}') {
        skip_ws(p);
        if (**p == ',') { (*p)++; continue; }
        if (match_key(p, "enabled")) DecoderConfigs.fanet.enabled = parse_bool(p);
        else if (match_key(p, "output_enabled")) DecoderConfigs.fanet.output_enabled = parse_bool(p);
        else if (match_key(p, "center_freq")) DecoderConfigs.fanet.center_freq = parse_number(p);
        else { skip_value(p); }
    }
    if (**p == '}') (*p)++;
}

static void parse_sarsat(const char **p)
{
    skip_ws(p);
    if (**p != '{') return;
    (*p)++;

    while (**p && **p != '}') {
        skip_ws(p);
        if (**p == ',') { (*p)++; continue; }
        if (match_key(p, "enabled")) DecoderConfigs.sarsat.enabled = parse_bool(p);
        else if (match_key(p, "output_enabled")) DecoderConfigs.sarsat.output_enabled = parse_bool(p);
        else if (match_key(p, "center_freq")) DecoderConfigs.sarsat.center_freq = parse_number(p);
        else { skip_value(p); }
    }
    if (**p == '}') (*p)++;
}

// ======================== Load ========================

bool decoderConfigLoad(void)
{
    FILE *f = fopen(DECODER_CONFIG_PATH, "r");
    if (!f) return false;

    fseek(f, 0, SEEK_END);
    int64_t fsize = ftell(f);
    if (fsize <= 0 || fsize > 65536) { fclose(f); return false; }
    fseek(f, 0, SEEK_SET);

    char *data = static_cast<char*>(malloc((size_t)fsize + 1));
    if (!data) { fclose(f); return false; }
    size_t rd = fread(data, 1, (size_t)fsize, f);
    fclose(f);
    data[rd] = '\0';

    const char *p = data;
    skip_ws(&p);
    if (*p != '{') { free(data); return false; }
    p++;

    while (*p && *p != '}') {
        skip_ws(&p);
        if (*p == ',') { p++; continue; }
        if (match_key(&p, "adsb")) parse_adsb(&p);
        else if (match_key(&p, "flarm")) parse_flarm(&p);
        else if (match_key(&p, "acars")) parse_acars(&p);
        else if (match_key(&p, "vdl2")) parse_vdl2(&p);
        else if (match_key(&p, "radiosonde")) parse_radiosonde(&p);
        else if (match_key(&p, "pocsag")) parse_pocsag(&p);
        else if (match_key(&p, "gsm")) parse_gsm(&p);
        else if (match_key(&p, "lte")) parse_lte(&p);
        else if (match_key(&p, "iot868")) parse_iot868(&p);
        else if (match_key(&p, "fanet")) parse_fanet(&p);
        else if (match_key(&p, "sarsat")) parse_sarsat(&p);
        else { skip_value(&p); }
    }

    free(data);

    // If keys not loaded from JSON, try keys file
    if (!DecoderConfigs.flarm.keys_loaded && DecoderConfigs.flarm.keys_file[0]) {
        decoderConfigLoadFlarmKeys(DecoderConfigs.flarm.keys_file);
    }

    DECODER_CONFIG_DIAG("decoder_config: loaded from %s\n", DECODER_CONFIG_PATH);
    return true;
}

// ======================== Save ========================

bool decoderConfigSave(void)
{
    FILE *f = fopen(DECODER_CONFIG_PATH, "w");
    if (!f) {
        gg::eprint("decoder_config: cannot write %s: %s\n", DECODER_CONFIG_PATH, strerror(errno));
        return false;
    }

    gg::fprint(f, "{\n");

    // ADS-B
    gg::fprint(f, "  \"adsb\": {\n");
    gg::fprint(f, "    \"adaptive_range\": %s,\n", DecoderConfigs.adsb.adaptive_range ? "true" : "false");
    gg::fprint(f, "    \"adaptive_burst\": %s,\n", DecoderConfigs.adsb.adaptive_burst ? "true" : "false");
    gg::fprint(f, "    \"adaptive_min_gain\": %.1f,\n", DecoderConfigs.adsb.adaptive_min_gain);
    gg::fprint(f, "    \"adaptive_max_gain\": %.1f,\n", DecoderConfigs.adsb.adaptive_max_gain);
    gg::fprint(f, "    \"crc_rescue\": %s,\n", DecoderConfigs.adsb.crc_rescue ? "true" : "false");
    gg::fprint(f, "    \"fix_crc\": %d,\n", DecoderConfigs.adsb.fix_crc);
    gg::fprint(f, "    \"mode_ac\": %s,\n", DecoderConfigs.adsb.mode_ac ? "true" : "false");
    gg::fprint(f, "    \"mode_ac_auto\": %s\n", DecoderConfigs.adsb.mode_ac_auto ? "true" : "false");
    gg::fprint(f, "  },\n");

    // FLARM
    gg::fprint(f, "  \"flarm\": {\n");
    gg::fprint(f, "    \"enabled\": %s,\n", DecoderConfigs.flarm.enabled ? "true" : "false");
    gg::fprint(f, "    \"ogn_only\": %s,\n", DecoderConfigs.flarm.ogn_only ? "true" : "false");
    gg::fprint(f, "    \"ogn_server\": \"%s\",\n", DecoderConfigs.flarm.ogn_server);
    gg::fprint(f, "    \"ogn_port\": %d,\n", DecoderConfigs.flarm.ogn_port);
    gg::fprint(f, "    \"ogn_station\": \"%s\",\n", DecoderConfigs.flarm.ogn_station);
    gg::fprint(f, "    \"keys_file\": \"%s\",\n", DecoderConfigs.flarm.keys_file);
    if (DecoderConfigs.flarm.keys_loaded) {
        fprintf(f, "    \"key_table\": \"%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x\",\n",
                DecoderConfigs.flarm.key_table[0], DecoderConfigs.flarm.key_table[1],
                DecoderConfigs.flarm.key_table[2], DecoderConfigs.flarm.key_table[3],
                DecoderConfigs.flarm.key_table[4], DecoderConfigs.flarm.key_table[5],
                DecoderConfigs.flarm.key_table[6], DecoderConfigs.flarm.key_table[7],
                DecoderConfigs.flarm.key_table[8], DecoderConfigs.flarm.key_table[9],
                DecoderConfigs.flarm.key_table[10], DecoderConfigs.flarm.key_table[11]);
        gg::fprint(f, "    \"key2\": \"%08x\",\n", DecoderConfigs.flarm.key2);
        gg::fprint(f, "    \"key3\": \"%08x\",\n", DecoderConfigs.flarm.key3);
        gg::fprint(f, "    \"key4\": \"%08x\",\n", DecoderConfigs.flarm.key4);
        fprintf(f, "    \"key5\": \"%08x,%08x,%08x,%08x\"\n",
                DecoderConfigs.flarm.key5[0], DecoderConfigs.flarm.key5[1],
                DecoderConfigs.flarm.key5[2], DecoderConfigs.flarm.key5[3]);
    } else {
        gg::fprint(f, "    \"key_table\": \"\",\n");
        gg::fprint(f, "    \"key2\": \"\",\n");
        gg::fprint(f, "    \"key3\": \"\",\n");
        gg::fprint(f, "    \"key4\": \"\",\n");
        gg::fprint(f, "    \"key5\": \"\"\n");
    }
    gg::fprint(f, "  },\n");

    // ACARS
    gg::fprint(f, "  \"acars\": {\n");
    gg::fprint(f, "    \"enabled\": %s,\n", DecoderConfigs.acars.enabled ? "true" : "false");
    gg::fprint(f, "    \"center_freq\": %.0f,\n", DecoderConfigs.acars.center_freq);
    gg::fprint(f, "    \"channels\": [");
    for (int32_t i = 0; i < DecoderConfigs.acars.num_channels; i++)
        gg::fprint(f, "%s%.0f", i ? "," : "", DecoderConfigs.acars.channel_freqs[i]);
    gg::fprint(f, "]\n  },\n");

    // VDL2
    gg::fprint(f, "  \"vdl2\": {\n");
    gg::fprint(f, "    \"enabled\": %s,\n", DecoderConfigs.vdl2.enabled ? "true" : "false");
    gg::fprint(f, "    \"center_freq\": %.0f,\n", DecoderConfigs.vdl2.center_freq);
    gg::fprint(f, "    \"squelch_level\": %.1f,\n", DecoderConfigs.vdl2.squelch_level);
    gg::fprint(f, "    \"channels\": [");
    for (int32_t i = 0; i < DecoderConfigs.vdl2.num_channels; i++)
        gg::fprint(f, "%s%.0f", i ? "," : "", DecoderConfigs.vdl2.channel_freqs[i]);
    gg::fprint(f, "]\n  },\n");

    // Radiosonde
    gg::fprint(f, "  \"radiosonde\": {\n");
    gg::fprint(f, "    \"enabled\": %s,\n", DecoderConfigs.radiosonde.enabled ? "true" : "false");
    gg::fprint(f, "    \"sondehub_upload\": %s,\n", DecoderConfigs.radiosonde.sondehub_upload ? "true" : "false");
    gg::fprint(f, "    \"radiosondy_upload\": %s,\n", DecoderConfigs.radiosonde.radiosondy_upload ? "true" : "false");
    gg::fprint(f, "    \"wettersonde_upload\": %s,\n", DecoderConfigs.radiosonde.wettersonde_upload ? "true" : "false");
    gg::fprint(f, "    \"callsign\": \"%s\",\n", DecoderConfigs.radiosonde.callsign);
    gg::fprint(f, "    \"center_freq\": %.0f\n", DecoderConfigs.radiosonde.center_freq);
    gg::fprint(f, "  },\n");

    // POCSAG
    gg::fprint(f, "  \"pocsag\": {\n");
    gg::fprint(f, "    \"enabled\": %s,\n", DecoderConfigs.pocsag.enabled ? "true" : "false");
    gg::fprint(f, "    \"output_enabled\": %s,\n", DecoderConfigs.pocsag.output_enabled ? "true" : "false");
    gg::fprint(f, "    \"center_freq\": %.0f,\n", DecoderConfigs.pocsag.center_freq);
    gg::fprint(f, "    \"channels\": [");
    for (int32_t i = 0; i < DecoderConfigs.pocsag.num_channels; i++)
        gg::fprint(f, "%s%.0f", i ? "," : "", DecoderConfigs.pocsag.channel_freqs[i]);
    gg::fprint(f, "]\n  },\n");

    // GSM
    gg::fprint(f, "  \"gsm\": {\n");
    gg::fprint(f, "    \"enabled\": %s,\n", DecoderConfigs.gsm.enabled ? "true" : "false");
    gg::fprint(f, "    \"output_enabled\": %s,\n", DecoderConfigs.gsm.output_enabled ? "true" : "false");
    gg::fprint(f, "    \"arfcn_freq\": %.0f,\n", DecoderConfigs.gsm.arfcn_freq);
    gg::fprint(f, "    \"tsc\": %d\n", DecoderConfigs.gsm.tsc);
    gg::fprint(f, "  },\n");

    // LTE
    gg::fprint(f, "  \"lte\": {\n");
    gg::fprint(f, "    \"enabled\": %s,\n", DecoderConfigs.lte.enabled ? "true" : "false");
    gg::fprint(f, "    \"output_enabled\": %s,\n", DecoderConfigs.lte.output_enabled ? "true" : "false");
    gg::fprint(f, "    \"hop_enabled\": %s,\n", DecoderConfigs.lte.hop_enabled ? "true" : "false");
    gg::fprint(f, "    \"center_freq\": %.0f\n", DecoderConfigs.lte.center_freq);
    gg::fprint(f, "  },\n");

    // IoT 868
    gg::fprint(f, "  \"iot868\": {\n");
    gg::fprint(f, "    \"enabled\": %s,\n", DecoderConfigs.iot868.enabled ? "true" : "false");
    gg::fprint(f, "    \"output_enabled\": %s,\n", DecoderConfigs.iot868.output_enabled ? "true" : "false");
    gg::fprint(f, "    \"center_freq\": %.0f\n", DecoderConfigs.iot868.center_freq);
    gg::fprint(f, "  },\n");

    // FANET
    gg::fprint(f, "  \"fanet\": {\n");
    gg::fprint(f, "    \"enabled\": %s,\n", DecoderConfigs.fanet.enabled ? "true" : "false");
    gg::fprint(f, "    \"output_enabled\": %s,\n", DecoderConfigs.fanet.output_enabled ? "true" : "false");
    gg::fprint(f, "    \"center_freq\": %.0f\n", DecoderConfigs.fanet.center_freq);
    gg::fprint(f, "  },\n");

    // Sarsat
    gg::fprint(f, "  \"sarsat\": {\n");
    gg::fprint(f, "    \"enabled\": %s,\n", DecoderConfigs.sarsat.enabled ? "true" : "false");
    gg::fprint(f, "    \"output_enabled\": %s,\n", DecoderConfigs.sarsat.output_enabled ? "true" : "false");
    gg::fprint(f, "    \"center_freq\": %.0f\n", DecoderConfigs.sarsat.center_freq);
    gg::fprint(f, "  }\n");

    gg::fprint(f, "}\n");
    fclose(f);

    DECODER_CONFIG_DIAG("decoder_config: saved to %s\n", DECODER_CONFIG_PATH);
    return true;
}

// ======================== FLARM Keys file ========================

bool decoderConfigLoadFlarmKeys(const char *path)
{
    if (!path || !path[0]) return false;

    FILE *f = fopen(path, "r");
    if (!f) return false;

    char line[512];
    bool has_table = false, has_k2 = false, has_k3 = false, has_k4 = false, has_k5 = false;

    while (fgets(line, sizeof(line), f)) {
        // Skip comments and empty lines
        char *s = line;
        while (*s && isspace((uint8_t)*s)) s++;
        if (!*s || *s == '#') continue;

        if (strncasecmp(s, "key_table=", 10) == 0 || strncasecmp(s, "key_table =", 11) == 0) {
            char *val = strchr(s, '=') + 1;
            while (*val && isspace((uint8_t)*val)) val++;
            for (int32_t i = 0; i < 12; i++) {
                while (*val && !isxdigit((uint8_t)*val)) val++;
                if (!*val) break;
                char *end;
                DecoderConfigs.flarm.key_table[i] = (uint32_t)strtoul(val, &end, 16);
                val = end;
            }
            has_table = true;
        } else if (strncasecmp(s, "key2=", 5) == 0 || strncasecmp(s, "key2 =", 6) == 0) {
            char *val = strchr(s, '=') + 1;
            while (*val && !isxdigit((uint8_t)*val)) val++;
            DecoderConfigs.flarm.key2 = (uint32_t)strtoul(val, NULL, 16);
            has_k2 = true;
        } else if (strncasecmp(s, "key3=", 5) == 0 || strncasecmp(s, "key3 =", 6) == 0) {
            char *val = strchr(s, '=') + 1;
            while (*val && !isxdigit((uint8_t)*val)) val++;
            DecoderConfigs.flarm.key3 = (uint32_t)strtoul(val, NULL, 16);
            has_k3 = true;
        } else if (strncasecmp(s, "key4=", 5) == 0 || strncasecmp(s, "key4 =", 6) == 0) {
            char *val = strchr(s, '=') + 1;
            while (*val && !isxdigit((uint8_t)*val)) val++;
            DecoderConfigs.flarm.key4 = (uint32_t)strtoul(val, NULL, 16);
            has_k4 = true;
        } else if (strncasecmp(s, "key5=", 5) == 0 || strncasecmp(s, "key5 =", 6) == 0) {
            char *val = strchr(s, '=') + 1;
            while (*val && isspace((uint8_t)*val)) val++;
            for (int32_t i = 0; i < 4; i++) {
                while (*val && !isxdigit((uint8_t)*val)) val++;
                if (!*val) break;
                char *end;
                DecoderConfigs.flarm.key5[i] = (uint32_t)strtoul(val, &end, 16);
                val = end;
            }
            has_k5 = true;
        }
    }
    fclose(f);

    DecoderConfigs.flarm.keys_loaded = (has_table && has_k2 && has_k3 && has_k4 && has_k5);
    if (DecoderConfigs.flarm.keys_loaded) {
        DECODER_CONFIG_DIAG("decoder_config: FLARM keys loaded from %s\n", path);
    }
    return DecoderConfigs.flarm.keys_loaded;
}

bool decoderConfigSaveFlarmKeys(const char *path)
{
    if (!path || !path[0]) return false;
    if (!DecoderConfigs.flarm.keys_loaded) return false;

    FILE *f = fopen(path, "w");
    if (!f) return false;

    gg::fprint(f, "# FLARM decryption keys\n");
    fprintf(f, "key_table=%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x\n",
            DecoderConfigs.flarm.key_table[0], DecoderConfigs.flarm.key_table[1],
            DecoderConfigs.flarm.key_table[2], DecoderConfigs.flarm.key_table[3],
            DecoderConfigs.flarm.key_table[4], DecoderConfigs.flarm.key_table[5],
            DecoderConfigs.flarm.key_table[6], DecoderConfigs.flarm.key_table[7],
            DecoderConfigs.flarm.key_table[8], DecoderConfigs.flarm.key_table[9],
            DecoderConfigs.flarm.key_table[10], DecoderConfigs.flarm.key_table[11]);
    gg::fprint(f, "key2=%08x\n", DecoderConfigs.flarm.key2);
    gg::fprint(f, "key3=%08x\n", DecoderConfigs.flarm.key3);
    gg::fprint(f, "key4=%08x\n", DecoderConfigs.flarm.key4);
    fprintf(f, "key5=%08X,%08X,%08X,%08X\n",
            DecoderConfigs.flarm.key5[0], DecoderConfigs.flarm.key5[1],
            DecoderConfigs.flarm.key5[2], DecoderConfigs.flarm.key5[3]);

    fclose(f);
    return true;
}

// ======================== Utilities ========================

static const char *decoder_type_names[DECODER_TYPE_COUNT] = {
    [DECODER_ADSB]       = "adsb",
    [DECODER_FLARM]      = "flarm",
    [DECODER_ACARS]      = "acars",
    [DECODER_VDL2]       = "vdl2",
    [DECODER_RADIOSONDE] = "radiosonde",
    [DECODER_POCSAG]     = "pocsag",
    [DECODER_GSM]        = "gsm",
    [DECODER_LTE]        = "lte",
    [DECODER_IOT868]     = "iot868",
    [DECODER_FANET]      = "fanet",
    [DECODER_SARSAT]     = "sarsat",
};

const char *decoderTypeName(decoder_type_t type)
{
    if (type < DECODER_TYPE_COUNT)
        return decoder_type_names[type];
    return "unknown";
}

decoder_type_t decoderTypeFromName(const char *name)
{
    if (!name) return DECODER_TYPE_COUNT;
    for (int32_t i = 0; i < DECODER_TYPE_COUNT; i++) {
        if (strcasecmp(name, decoder_type_names[i]) == 0)
            return (decoder_type_t)i;
    }
    return DECODER_TYPE_COUNT;
}

bool decoderConfigParseJson(const char *json)
{
    if (!json) return false;
    const char *p = json;
    skip_ws(&p);
    if (*p != '{') return false;
    p++;

    while (*p && *p != '}') {
        skip_ws(&p);
        if (*p == ',') { p++; continue; }
        if (match_key(&p, "adsb")) parse_adsb(&p);
        else if (match_key(&p, "flarm")) parse_flarm(&p);
        else if (match_key(&p, "acars")) parse_acars(&p);
        else if (match_key(&p, "vdl2")) parse_vdl2(&p);
        else if (match_key(&p, "radiosonde")) parse_radiosonde(&p);
        else if (match_key(&p, "pocsag")) parse_pocsag(&p);
        else if (match_key(&p, "gsm")) parse_gsm(&p);
        else if (match_key(&p, "lte")) parse_lte(&p);
        else if (match_key(&p, "iot868")) parse_iot868(&p);
        else if (match_key(&p, "fanet")) parse_fanet(&p);
        else if (match_key(&p, "sarsat")) parse_sarsat(&p);
        else { skip_value(&p); }
    }
    return true;
}
