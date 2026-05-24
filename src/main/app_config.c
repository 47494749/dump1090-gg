// Part of dump1090-gg, a Mode S message decoder.
//
// app_config.c: Global configuration instances and sync logic
//
// During migration, appConfigSyncFromModes() copies values from the legacy
// Modes struct into the new config structs. Once all consumers read from
// the new structs, this file will own initialization directly.

#include <string.h>
#include <stdint.h>
#include "station_config.h"
#include "network_config.h"
#include "display_config.h"
#include "json_config.h"
#include "decoder_config.h"
#include "dump1090.h"

// ======================== Global instances ========================

station_config_t  StationConfig;
network_config_t  NetworkConfig;
display_config_t  DisplayConfig;
json_config_t     JsonConfig;

// ======================== Sync from legacy Modes ========================

void appConfigSyncFromModes(void)
{
    // ---- Station ----
    StationConfig.latitude   = Modes.fUserLat;
    StationConfig.longitude  = Modes.fUserLon;
    StationConfig.max_range_m = Modes.maxRange;
    // altitude and name come from MLAT/OGN config — will be migrated later

    // ---- Network ----
    NetworkConfig.net_enabled = Modes.net;
    NetworkConfig.net_only    = Modes.net_only;
    NetworkConfig.sndbuf_size = Modes.net_sndbuf_size;
    NetworkConfig.output_flush_size = Modes.net_output_flush_size;
    NetworkConfig.output_flush_interval_ms = Modes.net_output_flush_interval;
    NetworkConfig.heartbeat_interval_ms = Modes.net_heartbeat_interval;
    NetworkConfig.verbatim = Modes.net_verbatim;
    NetworkConfig.forward_mlat = Modes.forward_mlat;

    if (Modes.net_output_raw_ports)
        snprintf(NetworkConfig.raw_out_ports, sizeof(NetworkConfig.raw_out_ports),
                 "%s", Modes.net_output_raw_ports);
    if (Modes.net_input_raw_ports)
        snprintf(NetworkConfig.raw_in_ports, sizeof(NetworkConfig.raw_in_ports),
                 "%s", Modes.net_input_raw_ports);
    if (Modes.net_output_sbs_ports)
        snprintf(NetworkConfig.sbs_out_ports, sizeof(NetworkConfig.sbs_out_ports),
                 "%s", Modes.net_output_sbs_ports);
    if (Modes.net_output_stratux_ports)
        snprintf(NetworkConfig.stratux_out_ports, sizeof(NetworkConfig.stratux_out_ports),
                 "%s", Modes.net_output_stratux_ports);
    if (Modes.net_output_beast_ports)
        snprintf(NetworkConfig.beast_out_ports, sizeof(NetworkConfig.beast_out_ports),
                 "%s", Modes.net_output_beast_ports);
    if (Modes.net_input_beast_ports)
        snprintf(NetworkConfig.beast_in_ports, sizeof(NetworkConfig.beast_in_ports),
                 "%s", Modes.net_input_beast_ports);
    if (Modes.net_bind_address)
        snprintf(NetworkConfig.bind_address, sizeof(NetworkConfig.bind_address),
                 "%s", Modes.net_bind_address);

    // Beast feeds
    NetworkConfig.feed_count = Modes.beast_feed_count;
    for (int32_t i = 0; i < Modes.beast_feed_count && i < MAX_BEAST_FEEDS; i++) {
        snprintf(NetworkConfig.feeds[i].name, sizeof(NetworkConfig.feeds[i].name),
                 "%s", Modes.beast_feeds[i].name);
        if (Modes.beast_feeds[i].host)
            snprintf(NetworkConfig.feeds[i].host, sizeof(NetworkConfig.feeds[i].host),
                     "%s", Modes.beast_feeds[i].host);
        NetworkConfig.feeds[i].port = Modes.beast_feeds[i].port;
        NetworkConfig.feeds[i].format = Modes.beast_feeds[i].format;
        NetworkConfig.feeds[i].enabled = Modes.beast_feeds[i].enabled;
    }

    if (Modes.adsbhub_ckey)
        snprintf(NetworkConfig.adsbhub_ckey, sizeof(NetworkConfig.adsbhub_ckey),
                 "%s", Modes.adsbhub_ckey);

    // ---- Display ----
    DisplayConfig.quiet = Modes.quiet;
    DisplayConfig.interactive = Modes.interactive;
    DisplayConfig.interactive_ttl_ms = Modes.interactive_display_ttl;
    DisplayConfig.interactive_rows = Modes.interactive_display_size;
    DisplayConfig.show_distance = Modes.interactive_show_distance;
    DisplayConfig.distance_units = (distance_unit_t)Modes.interactive_distance_units;
    DisplayConfig.show_only = Modes.show_only;
    DisplayConfig.onlyaddr = Modes.onlyaddr;
    DisplayConfig.tisb_verbose = Modes.tisb_verbose;
    DisplayConfig.metric = Modes.metric;
    DisplayConfig.raw = Modes.raw;
    if (Modes.interactive_callsign_filter)
        snprintf(DisplayConfig.callsign_filter, sizeof(DisplayConfig.callsign_filter),
                 "%s", Modes.interactive_callsign_filter);

    // ---- JSON ----
    if (Modes.json_dir)
        snprintf(JsonConfig.dir, sizeof(JsonConfig.dir), "%s", Modes.json_dir);
    JsonConfig.interval_ms = Modes.json_interval;
    JsonConfig.stats_interval_ms = Modes.json_stats_interval;
    JsonConfig.location_accuracy = Modes.json_location_accuracy;

    // ---- ADS-B decoder options ----
    DecoderConfigs.adsb.fix_crc = Modes.nfix_crc;
    DecoderConfigs.adsb.check_crc = Modes.check_crc;
    DecoderConfigs.adsb.fix_df = Modes.fix_df;
    DecoderConfigs.adsb.enable_df24 = Modes.enable_df24;
    DecoderConfigs.adsb.mode_ac = Modes.mode_ac;
    DecoderConfigs.adsb.mode_ac_auto = Modes.mode_ac_auto;
    DecoderConfigs.adsb.crc_rescue = Modes.crc_rescue;
    DecoderConfigs.adsb.use_gnss = Modes.use_gnss;
    DecoderConfigs.adsb.mlat = Modes.mlat;

    // Adaptive gain
    DecoderConfigs.adsb.adaptive_min_gain = Modes.adaptive_min_gain_db;
    DecoderConfigs.adsb.adaptive_max_gain = Modes.adaptive_max_gain_db;
    DecoderConfigs.adsb.adaptive_duty_cycle = Modes.adaptive_duty_cycle;
    DecoderConfigs.adsb.adaptive_burst = Modes.adaptive_burst_control;
    DecoderConfigs.adsb.adaptive_burst_alpha = Modes.adaptive_burst_alpha;
    DecoderConfigs.adsb.adaptive_burst_change_delay = Modes.adaptive_burst_change_delay;
    DecoderConfigs.adsb.adaptive_burst_loud_rate = Modes.adaptive_burst_loud_rate;
    DecoderConfigs.adsb.adaptive_burst_loud_runlength = Modes.adaptive_burst_loud_runlength;
    DecoderConfigs.adsb.adaptive_burst_quiet_rate = Modes.adaptive_burst_quiet_rate;
    DecoderConfigs.adsb.adaptive_burst_quiet_runlength = Modes.adaptive_burst_quiet_runlength;
    DecoderConfigs.adsb.adaptive_range = Modes.adaptive_range_control;
    DecoderConfigs.adsb.adaptive_range_alpha = Modes.adaptive_range_alpha;
    DecoderConfigs.adsb.adaptive_range_percentile = Modes.adaptive_range_percentile;
    DecoderConfigs.adsb.adaptive_range_target = Modes.adaptive_range_target;
    DecoderConfigs.adsb.adaptive_range_change_delay = Modes.adaptive_range_change_delay;
    DecoderConfigs.adsb.adaptive_range_scan_delay = Modes.adaptive_range_scan_delay;
    DecoderConfigs.adsb.adaptive_range_rescan_delay = Modes.adaptive_range_rescan_delay;
}
