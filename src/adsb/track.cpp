// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// track.c: aircraft state tracking
//
// Copyright (c) 2014-2016 Oliver Jowett <oliver@mutability.co.uk>
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.
//
// This file is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

// This file incorporates work covered by the following copyright and
// permission notice:
//
//   Copyright (C) 2012 by Salvatore Sanfilippo <antirez@gmail.com>
//
//   All rights reserved.
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions are
//   met:
//
//    *  Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//    *  Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//   HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "dump1090.h"
#include <stdint.h>
#include <inttypes.h>
#include "gg_format.h"

/* #define DEBUG_CPR_CHECKS */

uint32_t modeAC_count[4096];
uint32_t modeAC_lastcount[4096];
uint32_t modeAC_match[4096];
uint32_t modeAC_age[4096];

//
// Return a new aircraft structure for the linked list of tracked
// aircraft
//
static struct aircraft *trackCreateAircraft(struct modesMessage *mm) {
    static struct aircraft zeroAircraft;
    struct aircraft *a = (struct aircraft *) malloc(sizeof(*a));
    int32_t i;

    // Default everything to zero/NULL
    *a = zeroAircraft;

    // Now initialise things that should not be 0/NULL to their defaults
    a->addr = mm->addr;
    a->addrtype = mm->addrtype;
    for (i = 0; i < 8; ++i)
        a->signalLevel[i] = 1e-5;
    a->signalNext = 0;

    // defaults until we see a message otherwise
    a->adsb_version = -1;
    a->adsb_hrd = HEADING_MAGNETIC;
    a->adsb_tah = HEADING_GROUND_TRACK;

    // prime FATSV defaults we only emit on change

    // start off with the "last emitted" ACAS RA being blank (just the BDS 3,0
    // or ES type code)
    a->fatsv_emitted_bds_30[0] = 0x30;
    a->fatsv_emitted_es_acas_ra[0] = 0xE2;
    a->fatsv_emitted_adsb_version = -1;
    a->fatsv_emitted_addrtype = ADDR_UNKNOWN;

    // don't immediately emit, let some data build up
    a->fatsv_last_emitted = a->fatsv_last_force_emit = messageNow();

    // initialize data validity ages
#define F(f,s,e) do { a->f##_valid.stale_interval = (s) * 1000; a->f##_valid.expire_interval = (e) * 1000; } while (0)
    F(callsign,        60, 70);  // ADS-B or Comm-B
    F(altitude_baro,   15, 70);  // ADS-B or Mode S
    F(altitude_geom,   60, 70);  // ADS-B only
    F(geom_delta,      60, 70);  // ADS-B only
    F(gs,              60, 70);  // ADS-B or Comm-B
    F(ias,             60, 70);  // ADS-B (rare) or Comm-B
    F(tas,             60, 70);  // ADS-B (rare) or Comm-B
    F(mach,            60, 70);  // Comm-B only
    F(track,           60, 70);  // ADS-B or Comm-B
    F(track_rate,      60, 70);  // Comm-B only
    F(roll,            60, 70);  // Comm-B only
    F(mag_heading,     60, 70);  // ADS-B (rare) or Comm-B
    F(true_heading,    60, 70);  // ADS-B only (rare)
    F(baro_rate,       60, 70);  // ADS-B or Comm-B
    F(geom_rate,       60, 70);  // ADS-B or Comm-B
    F(squawk,          15, 70);  // ADS-B or Mode S
    F(emergency,       60, 70);  // ADS-B only
    F(airground,       15, 70);  // ADS-B or Mode S
    F(nav_qnh,         60, 70);  // Comm-B only
    F(nav_altitude_mcp, 60, 70);  // ADS-B or Comm-B
    F(nav_altitude_fms, 60, 70);  // ADS-B or Comm-B
    F(nav_altitude_src, 60, 70); // ADS-B or Comm-B
    F(nav_heading,     60, 70);  // ADS-B or Comm-B
    F(nav_modes,       60, 70);  // ADS-B or Comm-B
    F(cpr_odd,         60, 70);  // ADS-B only
    F(cpr_even,        60, 70);  // ADS-B only
    F(position,        60, 70);  // ADS-B only
    F(nic_a,           60, 70);  // ADS-B only
    F(nic_c,           60, 70);  // ADS-B only
    F(nic_baro,        60, 70);  // ADS-B only
    F(nac_p,           60, 70);  // ADS-B only
    F(nac_v,           60, 70);  // ADS-B only
    F(sil,             60, 70);  // ADS-B only
    F(gva,             60, 70);  // ADS-B only
    F(sda,             60, 70);  // ADS-B only
    F(mrar_source,     60, 70);  // Comm-B only
    F(wind,            60, 70);  // Comm-B only
    F(temperature,     60, 70);  // Comm-B only
    F(pressure,        60, 70);  // Comm-B only
    F(turbulence,      60, 70);  // Comm-B only
    F(humidity,        60, 70);  // Comm-B only
    F(mhar_turbulence, 60, 70);  // Comm-B only
    F(mhar_windshear,  60, 70);  // Comm-B only
    F(mhar_microburst, 60, 70);  // Comm-B only
    F(mhar_icing,      60, 70);  // Comm-B only
    F(mhar_wake,       60, 70);  // Comm-B only
    F(mhar_sat,        60, 70);  // Comm-B only
    F(mhar_asp,        60, 70);  // Comm-B only
    F(mhar_rh,         60, 70);  // Comm-B only
    F(waypoint,        60, 70);  // Comm-B only
    F(waypoint_pos,    60, 70);  // Comm-B only
    F(waypoint_info,   60, 70);  // Comm-B only
    F(acas_ra,         60, 70);  // Comm-B or ADS-B
    F(opstatus,        60, 70);  // ADS-B only
#undef F

    Modes.stats_current.unique_aircraft++;

    return (a);
}

//
//=========================================================================
//
// Return the aircraft with the specified address, or NULL if no aircraft
// exists with this address.
//
static struct aircraft *trackFindAircraft(uint32_t addr) {
    struct aircraft *a = Modes.aircrafts;

    while(a) {
        if (a->addr == addr) return (a);
        a = a->next;
    }
    return (NULL);
}

// Should we accept some new data from the given source?
// If so, update the validity and return 1
static int32_t accept_data(data_validity *d, datasource_t source)
{
    if (messageNow() < d->updated)
        return 0;

    if (source < d->source && messageNow() < d->stale)
        return 0;

    d->source = source;
    d->updated = messageNow();
    d->stale = messageNow() + (d->stale_interval ? d->stale_interval : 60000);
    d->expires = messageNow() + (d->expire_interval ? d->expire_interval : 70000);
    return 1;
}

// Given two datasources, produce a third datasource for data combined from them.
static void combine_validity(data_validity *to, const data_validity *from1, const data_validity *from2) {
    if (from1->source == SOURCE_INVALID) {
        *to = *from2;
        return;
    }

    if (from2->source == SOURCE_INVALID) {
        *to = *from1;
        return;
    }

    to->source = (from1->source < from2->source) ? from1->source : from2->source;        // the worse of the two input sources
    to->updated = (from1->updated > from2->updated) ? from1->updated : from2->updated;   // the *later* of the two update times
    to->stale = (from1->stale < from2->stale) ? from1->stale : from2->stale;             // the earlier of the two stale times
    to->expires = (from1->expires < from2->expires) ? from1->expires : from2->expires;   // the earlier of the two expiry times
}

static int32_t compare_validity(const data_validity *lhs, const data_validity *rhs) {
    if (messageNow() < lhs->stale && lhs->source > rhs->source)
        return 1;
    else if (messageNow() < rhs->stale && lhs->source < rhs->source)
        return -1;
    else if (lhs->updated > rhs->updated)
        return 1;
    else if (lhs->updated < rhs->updated)
        return -1;
    else
        return 0;
}

//
// CPR position updating
//

// Distance between points on a spherical earth.
// This has up to 0.5% error because the earth isn't actually spherical
// (but we don't use it in situations where that matters)
double greatcircle(double lat0, double lon0, double lat1, double lon1)
{
    double dlat, dlon;

    lat0 = lat0 * M_PI / 180.0;
    lon0 = lon0 * M_PI / 180.0;
    lat1 = lat1 * M_PI / 180.0;
    lon1 = lon1 * M_PI / 180.0;

    dlat = fabs(lat1 - lat0);
    dlon = fabs(lon1 - lon0);

    // use haversine for small distances for better numerical stability
    if (dlat < 0.001 && dlon < 0.001) {
        double a = sin(dlat/2) * sin(dlat/2) + cos(lat0) * cos(lat1) * sin(dlon/2) * sin(dlon/2);
        return 6371e3 * 2 * atan2(sqrt(a), sqrt(1.0 - a));
    }

    // spherical law of cosines
    return 6371e3 * acos(sin(lat0) * sin(lat1) + cos(lat0) * cos(lat1) * cos(dlon));
}

double get_bearing(double lat0, double lon0, double lat1, double lon1)
{
    double dlon;

    lat0 = lat0 * M_PI / 180.0;
    lon0 = lon0 * M_PI / 180.0;
    lat1 = lat1 * M_PI / 180.0;
    lon1 = lon1 * M_PI / 180.0;

    dlon = (lon1 - lon0);

    double x = (cos(lat0) * sin(lat1)) -
                     (sin(lat0) * cos(lat1) * cos(dlon));
    double y = sin(dlon) * cos(lat1);
    double degree = atan2(y, x) * 180 / M_PI;

    return (degree >= 0)? degree : (degree + 360);
}

// Magnetic declination using tilted dipole model (WMM2020 epoch, approximate)
// Returns declination in degrees, positive = east
static float get_mag_declination(double lat, double lon)
{
    // Main dipole Gauss coefficients (nT), approximate for 2024 epoch
    // g10 â‰ˆ -29405, g11 â‰ˆ -1451, h11 â‰ˆ 4653 (WMM2020 base, close enough)
    static const double g10 = -29405.0;
    static const double g11 = -1451.0;
    static const double h11 = 4653.0;

    double lat_r = lat * M_PI / 180.0;
    double lon_r = lon * M_PI / 180.0;

    // North and East components of the dipole field at the surface
    double Bx = -g10 * cos(lat_r) + (g11 * cos(lon_r) + h11 * sin(lon_r)) * sin(lat_r);
    double By = g11 * sin(lon_r) - h11 * cos(lon_r);

    return (float)(atan2(By, Bx) * 180.0 / M_PI);
}

// Derive wind speed and direction from TAS, heading, GS, and track
// Based on the triangle of velocities:
//   aircraft_vector (TAS along heading) = ground_vector (GS along track) + wind_vector
static void calc_wind(struct aircraft *a, uint64_t now)
{
    if (!trackDataValid(&a->gs_valid) ||
        !trackDataValid(&a->tas_valid) ||
        !trackDataValid(&a->track_valid))
        return;

    // Need at least one heading source
    if (!trackDataValid(&a->true_heading_valid) &&
        !trackDataValid(&a->mag_heading_valid))
        return;

    // All inputs must be recent (within TRACK_WT_TIMEOUT ms)
    if ((now - a->gs_valid.updated) > TRACK_WT_TIMEOUT ||
        (now - a->tas_valid.updated) > TRACK_WT_TIMEOUT ||
        (now - a->track_valid.updated) > TRACK_WT_TIMEOUT)
        return;

    float heading;
    if (trackDataValid(&a->true_heading_valid) &&
        (now - a->true_heading_valid.updated) <= TRACK_WT_TIMEOUT) {
        heading = a->true_heading;
    } else if (trackDataValid(&a->mag_heading_valid) &&
               (now - a->mag_heading_valid.updated) <= TRACK_WT_TIMEOUT) {
        // Convert magnetic heading to true heading using declination
        if (trackDataValid(&a->position_valid)) {
            heading = a->mag_heading + a->mag_declination;
        } else {
            return; // can't convert without position
        }
    } else {
        return;
    }

    float gs = a->gs;
    float tas = (float)a->tas;
    float trk = a->track;

    if (gs < 10 || tas < 10)
        return; // too slow, unreliable

    // Wind triangle: wind = aircraft_air_vector - ground_vector
    float trk_r = trk * (float)(M_PI / 180.0);
    float hdg_r = heading * (float)(M_PI / 180.0);

    // Ground vector components (north, east)
    float gn = gs * cosf(trk_r);
    float ge = gs * sinf(trk_r);

    // Air vector components (north, east)
    float an = tas * cosf(hdg_r);
    float ae = tas * sinf(hdg_r);

    // Wind = air - ground
    float wn = an - gn;
    float we = ae - ge;

    float ws = sqrtf(wn * wn + we * we);
    float wd = atan2f(we, wn) * (float)(180.0 / M_PI);

    // Wind direction is where wind comes FROM, so add 180
    wd += 180.0f;
    if (wd < 0) wd += 360.0f;
    if (wd >= 360.0f) wd -= 360.0f;

    // Sanity check
    if (ws > 250.0f)
        return; // implausible

    a->derived_wind_speed = ws;
    a->derived_wind_dir = wd;
    a->derived_wind_updated = now;
    if (trackDataValid(&a->altitude_baro_valid))
        a->derived_wind_altitude = a->altitude_baro;
}

// Derive OAT and TAT from TAS and Mach number
// Based on: TAS = Mach * speed_of_sound, speed_of_sound = 661.47 * sqrt(T/288.15) kts
// So: T = 288.15 * (TAS / (661.47 * Mach))^2
static void calc_temp(struct aircraft *a, uint64_t now)
{
    if (!trackDataValid(&a->tas_valid) ||
        !trackDataValid(&a->mach_valid))
        return;

    if ((now - a->tas_valid.updated) > TRACK_WT_TIMEOUT ||
        (now - a->mach_valid.updated) > TRACK_WT_TIMEOUT)
        return;

    float mach = (float)a->mach;
    float tas = (float)a->tas;

    if (mach < 0.395f || tas < 100.0f)
        return; // too slow / too low Mach for reliable calculation

    float fraction = tas / (661.47f * mach);
    float oat = fraction * fraction * 288.15f - 273.15f;

    // Sanity check: OAT should be between -90 and +60
    if (oat < -90.0f || oat > 60.0f)
        return;

    float tat = (oat + 273.15f) * (1.0f + 0.2f * mach * mach) - 273.15f;

    a->oat = oat;
    a->tat = tat;
    a->oat_updated = now;
}

static void update_range_histogram(double lat, double lon)
{
    if (Modes.stats_range_histo && (Modes.bUserFlags & MODES_USER_LATLON_VALID)) {
        double range = greatcircle(Modes.fUserLat, Modes.fUserLon, lat, lon);
        int32_t bucket = round(range / Modes.maxRange * RANGE_BUCKET_COUNT);

        if (bucket < 0)
            bucket = 0;
        else if (bucket >= RANGE_BUCKET_COUNT)
            bucket = RANGE_BUCKET_COUNT-1;

        ++Modes.stats_current.range_histogram[bucket];
    }
}

// return true if it's OK for the aircraft to have travelled from its last known position
// to a new position at (lat,lon,surface) at a time of now.
static int32_t speed_check(struct aircraft *a, double lat, double lon, int32_t surface)
{
    uint64_t elapsed;
    double distance;
    double range;
    int32_t speed;
    int32_t inrange;

    if (!trackDataValid(&a->position_valid))
        return 1; // no reference, assume OK

    elapsed = trackDataAge(&a->position_valid);

    if (trackDataValid(&a->gs_valid))
        speed = a->gs;
    else if (trackDataValid(&a->tas_valid))
        speed = a->tas * 4 / 3;
    else if (trackDataValid(&a->ias_valid))
        speed = a->ias * 2;
    else
        speed = surface ? 100 : 600; // guess

    // Work out a reasonable speed to use:
    //  current speed + 1/3
    //  surface speed min 20kt, max 150kt
    //  airborne speed min 200kt, no max
    speed = speed * 4 / 3;
    if (surface) {
        if (speed < 20)
            speed = 20;
        if (speed > 150)
            speed = 150;
    } else {
        if (speed < 200)
            speed = 200;
    }

    // 100m (surface) or 500m (airborne) base distance to allow for minor errors,
    // plus distance covered at the given speed for the elapsed time + 1 second.
    range = (surface ? 0.1e3 : 0.5e3) + ((elapsed + 1000.0) / 1000.0) * (speed * 1852.0 / 3600.0);

    // find actual distance
    distance = greatcircle(a->lat, a->lon, lat, lon);

    inrange = (distance <= range);
#ifdef DEBUG_CPR_CHECKS
    if (!inrange) {
        fprintf(stderr, "Speed check failed: %06x: %.3f,%.3f -> %.3f,%.3f in %.1f seconds, max speed %d kt, range %.1fkm, actual %.1fkm\n",
                a->addr, a->lat, a->lon, lat, lon, elapsed/1000.0, speed, range/1000.0, distance/1000.0);
    }
#endif

    return inrange;
}

// return 1 if left_rc is worse (less accurate) than right_rc
static int32_t rcIsWorse(int32_t left_rc, int32_t right_rc)
{
    if (left_rc == 0 && right_rc == 0) // both unknown
        return 0;
    if (left_rc == 0)
        return 1; // left unknown < right known
    if (right_rc == 0)
        return 0; // left known > right unknown
    return (left_rc > right_rc);
}

static int32_t doGlobalCPR(struct aircraft *a, struct modesMessage *mm, double *lat, double *lon, uint32_t *nic, uint32_t *rc)
{
    int32_t result;
    int32_t fflag = mm->cpr_odd;
    int32_t surface = (mm->cpr_type == CPR_SURFACE);

    // derive NIC, Rc from the worse of the two position
    // smaller NIC is worse
    *nic = (a->cpr_even_nic < a->cpr_odd_nic ? a->cpr_even_nic : a->cpr_odd_nic);
    *rc = (rcIsWorse(a->cpr_even_rc, a->cpr_odd_rc) ? a->cpr_even_rc : a->cpr_odd_rc);

    if (surface) {
        // surface global CPR
        // find reference location
        double reflat, reflon;

        if (trackDataValid(&a->position_valid)) { // Ok to try aircraft relative first
            reflat = a->lat;
            reflon = a->lon;
        } else if (Modes.bUserFlags & MODES_USER_LATLON_VALID) {
            reflat = Modes.fUserLat;
            reflon = Modes.fUserLon;
        } else {
            // No local reference, give up
            return (-1);
        }

        result = decodeCPRsurface(reflat, reflon,
                                  a->cpr_even_lat, a->cpr_even_lon,
                                  a->cpr_odd_lat, a->cpr_odd_lon,
                                  fflag,
                                  lat, lon);
    } else {
        // airborne global CPR
        result = decodeCPRairborne(a->cpr_even_lat, a->cpr_even_lon,
                                   a->cpr_odd_lat, a->cpr_odd_lon,
                                   fflag,
                                   lat, lon);
    }

    if (result < 0) {
#ifdef DEBUG_CPR_CHECKS
        gg::eprint("CPR: decode failure for %06X (%d).\n", a->addr, result);
        fprintf(stderr, "  even: %d %d   odd: %d %d  fflag: %s\n",
                a->cpr_even_lat, a->cpr_even_lon,
                a->cpr_odd_lat, a->cpr_odd_lon,
                fflag ? "odd" : "even");
#endif
        return result;
    }

    // check max range
    if (Modes.maxRange > 0 && (Modes.bUserFlags & MODES_USER_LATLON_VALID)) {
        double range = greatcircle(Modes.fUserLat, Modes.fUserLon, *lat, *lon);
        if (range > Modes.maxRange) {
#ifdef DEBUG_CPR_CHECKS
            fprintf(stderr, "Global range check failed: %06x: %.3f,%.3f, max range %.1fkm, actual %.1fkm\n",
                    a->addr, *lat, *lon, Modes.maxRange/1000.0, range/1000.0);
#endif

            Modes.stats_current.cpr_global_range_checks++;
            return (-2); // we consider an out-of-range value to be bad data
        }
    }

    // for mlat results, skip the speed check
    if (mm->source == SOURCE_MLAT)
        return result;

    // check speed limit
    if (trackDataValid(&a->position_valid) && a->pos_nic >= *nic && !rcIsWorse(a->pos_rc, *rc) && !speed_check(a, *lat, *lon, surface)) {
        Modes.stats_current.cpr_global_speed_checks++;
        return -2;
    }

    return result;
}

static int32_t doLocalCPR(struct aircraft *a, struct modesMessage *mm, double *lat, double *lon, uint32_t *nic, uint32_t *rc)
{
    // relative CPR
    // find reference location
    double reflat, reflon;
    double range_limit = 0;
    int32_t result;
    int32_t fflag = mm->cpr_odd;
    int32_t surface = (mm->cpr_type == CPR_SURFACE);

    if (fflag) {
        *nic = a->cpr_odd_nic;
        *rc = a->cpr_odd_rc;
    } else {
        *nic = a->cpr_even_nic;
        *rc = a->cpr_even_rc;
    }

    if (trackDataValid(&a->position_valid)) {
        reflat = a->lat;
        reflon = a->lon;

        if (a->pos_nic < *nic)
            *nic = a->pos_nic;
        if (rcIsWorse(a->pos_rc, *rc))
            *rc = a->pos_rc;

        range_limit = 50e3;
    } else if (!surface && (Modes.bUserFlags & MODES_USER_LATLON_VALID)) {
        reflat = Modes.fUserLat;
        reflon = Modes.fUserLon;

        // The cell size is at least 360NM, giving a nominal
        // max range of 180NM (half a cell).
        //
        // If the receiver range is more than half a cell
        // then we must limit this range further to avoid
        // ambiguity. (e.g. if we receive a position report
        // at 200NM distance, this may resolve to a position
        // at (200-360) = 160NM in the wrong direction)

        if (Modes.maxRange == 0) {
            return (-1); // Can't do receiver-centered checks at all
        } else if (Modes.maxRange <= 1852*180) {
            range_limit = Modes.maxRange;
        } else if (Modes.maxRange < 1852*360) {
            range_limit = (1852*360) - Modes.maxRange;
        } else {
            return (-1); // Can't do receiver-centered checks at all
        }
    } else {
        // No local reference, give up
        return (-1);
    }

    result = decodeCPRrelative(reflat, reflon,
                               mm->cpr_lat,
                               mm->cpr_lon,
                               fflag, surface,
                               lat, lon);
    if (result < 0) {
        return result;
    }

    // check range limit
    if (range_limit > 0) {
        double range = greatcircle(reflat, reflon, *lat, *lon);
        if (range > range_limit) {
            Modes.stats_current.cpr_local_range_checks++;
            return (-1);
        }
    }

    // check speed limit
    if (trackDataValid(&a->position_valid) && a->pos_nic >= *nic && !rcIsWorse(a->pos_rc, *rc) && !speed_check(a, *lat, *lon, surface)) {
#ifdef DEBUG_CPR_CHECKS
        gg::eprint("Speed check for %06X with local decoding failed\n", a->addr);
#endif
        Modes.stats_current.cpr_local_speed_checks++;
        return -1;
    }

    return 0;
}

static uint64_t time_between(uint64_t t1, uint64_t t2)
{
    if (t1 >= t2)
        return t1 - t2;
    else
        return t2 - t1;
}

static void updatePosition(struct aircraft *a, struct modesMessage *mm)
{
    int32_t location_result = -1;
    uint64_t max_elapsed;
    double new_lat = 0, new_lon = 0;
    uint32_t new_nic = 0;
    uint32_t new_rc = 0;
    int32_t surface;

    surface = (mm->cpr_type == CPR_SURFACE);

    if (surface) {
        ++Modes.stats_current.cpr_surface;

        // Surface: 25 seconds if >25kt or speed unknown, 50 seconds otherwise
        if (mm->gs_valid && mm->gs.selected <= 25)
            max_elapsed = 50000;
        else
            max_elapsed = 25000;
    } else {
        ++Modes.stats_current.cpr_airborne;

        // Airborne: 10 seconds
        max_elapsed = 10000;
    }

    // If we have enough recent data, try global CPR
    if (trackDataValid(&a->cpr_odd_valid) && trackDataValid(&a->cpr_even_valid) &&
        a->cpr_odd_valid.source == a->cpr_even_valid.source &&
        a->cpr_odd_type == a->cpr_even_type &&
        time_between(a->cpr_odd_valid.updated, a->cpr_even_valid.updated) <= max_elapsed) {

        location_result = doGlobalCPR(a, mm, &new_lat, &new_lon, &new_nic, &new_rc);

        if (location_result == -2) {
#ifdef DEBUG_CPR_CHECKS
            gg::eprint("global CPR failure (invalid) for (%06X).\n", a->addr);
#endif
            // Global CPR failed because the position produced implausible results.
            // This is bad data. Discard both odd and even messages and wait for a fresh pair.
            // Also disable aircraft-relative positions until we have a new good position (but don't discard the
            // recorded position itself)
            Modes.stats_current.cpr_global_bad++;
            a->cpr_odd_valid.source = a->cpr_even_valid.source = a->position_valid.source = SOURCE_INVALID;

            return;
        } else if (location_result == -1) {
#ifdef DEBUG_CPR_CHECKS
            if (mm->source == SOURCE_MLAT) {
                gg::eprint("CPR skipped from MLAT (%06X).\n", a->addr);
            }
#endif
            // No local reference for surface position available, or the two messages crossed a zone.
            // Nonfatal, try again later.
            Modes.stats_current.cpr_global_skipped++;
        } else {
            if (accept_data(&a->position_valid, mm->source)) {
                Modes.stats_current.cpr_global_ok++;
            } else {
                Modes.stats_current.cpr_global_skipped++;
                location_result = -2;
            }
        }
    }

    // Otherwise try relative CPR.
    if (location_result == -1) {
        location_result = doLocalCPR(a, mm, &new_lat, &new_lon, &new_nic, &new_rc);

        if (location_result == 0 && accept_data(&a->position_valid, mm->source)) {
            Modes.stats_current.cpr_local_ok++;
            mm->cpr_relative = 1;
        } else {
            Modes.stats_current.cpr_local_skipped++;
            location_result = -1;
        }
    }

    if (location_result == 0) {
        // If we sucessfully decoded, back copy the results to mm so that we can print them in list output
        mm->cpr_decoded = 1;
        mm->decoded_lat = new_lat;
        mm->decoded_lon = new_lon;
        mm->decoded_nic = new_nic;
        mm->decoded_rc = new_rc;

        // Compute calc_track from consecutive positions
        // a->lat/lon still holds the previous position at this point
        if (a->prev_pos_updated > 0) {
            double dist = greatcircle(a->prev_lat, a->prev_lon, new_lat, new_lon);
            if (dist >= TRACK_CALC_TRACK_MIN_DIST) {
                a->calc_track = (float)get_bearing(a->prev_lat, a->prev_lon, new_lat, new_lon);
                a->calc_track_updated = messageNow();
            }
        }
        a->prev_lat = new_lat;
        a->prev_lon = new_lon;
        a->prev_pos_updated = messageNow();

        // Update aircraft state
        a->lat = new_lat;
        a->lon = new_lon;
        a->pos_nic = new_nic;
        a->pos_rc = new_rc;

        update_range_histogram(new_lat, new_lon);
    }
}

static uint32_t compute_nic(uint32_t metype, uint32_t version, uint32_t nic_a, uint32_t nic_b, uint32_t nic_c)
{
    switch (metype) {
    case 5: // surface
    case 9: // airborne
    case 20: // airborne, GNSS altitude
        return 11;

    case 6: // surface
    case 10: // airborne
    case 21: // airborne, GNSS altitude
        return 10;

    case 7: // surface
        if (version == 2) {
            if (nic_a && !nic_c) {
                return 9;
            } else {
                return 8;
            }
        } else if (version == 1) {
            if (nic_a) {
                return 9;
            } else {
                return 8;
            }
        } else {
            return 8;
        }

    case 8: // surface
        if (version == 2) {
            if (nic_a && nic_c) {
                return 7;
            } else if (nic_a && !nic_c) {
                return 6;
            } else if (!nic_a && nic_c) {
                return 6;
            } else {
                return 0;
            }
        } else {
            return 0;
        }

    case 11: // airborne
        if (version == 2) {
            if (nic_a && nic_b) {
                return 9;
            } else {
                return 8;
            }
        } else if (version == 1) {
            if (nic_a) {
                return 9;
            } else {
                return 8;
            }
        } else {
            return 8;
        }

    case 12: // airborne
        return 7;

    case 13: // airborne
        return 6;

    case 14: // airborne
        return 5;

    case 15: // airborne
        return 4;

    case 16: // airborne
        if (nic_a && nic_b) {
            return 3;
        } else {
            return 2;
        }

    case 17: // airborne
        return 1;

    default:
        return 0;
    }
}

static uint32_t compute_rc(uint32_t metype, uint32_t version, uint32_t nic_a, uint32_t nic_b, uint32_t nic_c)
{
    // ED-102 Table 2-14, Table N-4, Table N-11

    switch (metype) {
    case 5: // surface
    case 9: // airborne
    case 20: // airborne, GNSS altitude
        return 8; // 7.5m

    case 6: // surface
    case 10: // airborne
    case 21: // airborne, GNSS altitude
        return 25;

    case 7: // surface
        if (version == 2) {
            if (nic_a && !nic_c) {
                return 75;
            } else {
                return 186; // 185.2m, 0.1NM
            }
        } else if (version == 1) {
            if (nic_a) {
                return 75;
            } else {
                return 186; // 185.2m, 0.1NM
            }
        } else {
            return 186; // 185.2m, 0.1NM
        }

    case 8: // surface
        if (version == 2) {
            if (nic_a && nic_c) {
                return 371; // 370.4m, 0.2NM
            } else if (nic_a && !nic_c) {
                return 556; // 555.6m, 0.3NM
            } else if (!nic_a && nic_c) {
                return 1111; // 1111m, 0.6NM
            } else {
                return RC_UNKNOWN;
            }
        } else {
            return RC_UNKNOWN;
        }

    case 11: // airborne
        if (version == 2) {
            if (nic_a && nic_b) {
                return 75;
            } else {
                return 186; // 185.2m, 0.1NM
            }
        } else if (version == 1) {
            if (nic_a) {
                return 75;
            } else {
                return 186; // 185.2m, 0.1NM
            }
        } else {
            return 186; // 185.2m, 0.1NM
        }

    case 12: // airborne
        return 371; // 370.4m, 0.2NM

    case 13: // airborne
        if (version == 2) {
            if (!nic_a && nic_b) {
                return 556; // 555.6m, 0.3NM
            } else if (!nic_a && !nic_b) {
                return 926; // 926m, 0.5NM
            } else if (nic_a && nic_b) {
                return 1112; // 1111.2m, 0.6NM
            } else {
                return RC_UNKNOWN; // bad combination
            }
        } else if (version == 1) {
            if (nic_a) {
                return 1112; // 1111.2m, 0.6NM
            } else {
                return 926; // 926m, 0.5NM
            }
        } else {
            return 926; // 926m, 0.5NM
        }

    case 14: // airborne
        return 1852; // 1.0NM

    case 15: // airborne
        return 3704; // 2NM

    case 16: // airborne
        if (version == 2) {
            if (nic_a && nic_b) {
                return 7408; // 4NM
            } else {
                return 14816; // 8NM
            }
        } else if (version == 1) {
            if (nic_a) {
                return 7408; // 4NM
            } else {
                return 14816; // 8NM
            }
        } else {
            return 18520; // 10NM
        }

    case 17: // airborne
        return 37040; // 20NM

    default:
        return RC_UNKNOWN;
    }
}

// Map ADS-B v0 position message type to NACp value
// returned computed NACp, or -1 if not a suitable message type
static int32_t compute_v0_nacp(struct modesMessage *mm)
{
    if (mm->msgtype != 17 && mm->msgtype != 18) {
        return -1;
    }

    // ED-102A Table N-7
    switch (mm->metype) {
    case 0: return 0;
    case 5: return 11;
    case 6: return 10;
    case 7: return 8;
    case 8: return 0;
    case 9: return 11;
    case 10: return 10;
    case 11: return 8;
    case 12: return 7;
    case 13: return 6;
    case 14: return 5;
    case 15: return 4;
    case 16: return 1;
    case 17: return 1;
    case 18: return 0;
    case 20: return 11;
    case 21: return 10;
    case 22: return 0;
    default: return -1;
    }
}

// Map ADS-B v0 position message type to SIL value
// returned computed SIL, or -1 if not a suitable message type
static int32_t compute_v0_sil(struct modesMessage *mm)
{
    if (mm->msgtype != 17 && mm->msgtype != 18) {
        return -1;
    }

    // ED-102A Table N-8
    switch (mm->metype) {
    case 0:
        return 0;

    case 5:
    case 6:
    case 7:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
    case 16:
    case 17:
        return 2;

    case 18:
        return 0;

    case 20:
    case 21:
        return 2;

    case 22:
        return 0;

    default:
        return -1;
    }
}

static void compute_nic_rc_from_message(struct modesMessage *mm, struct aircraft *a, uint32_t *nic, uint32_t *rc)
{
    int32_t nic_a = (trackDataValid(&a->nic_a_valid) && a->nic_a);
    int32_t nic_b = (mm->accuracy.nic_b_valid && mm->accuracy.nic_b);
    int32_t nic_c = (trackDataValid(&a->nic_c_valid) && a->nic_c);

    *nic = compute_nic(mm->metype, a->adsb_version, nic_a, nic_b, nic_c);
    *rc = compute_rc(mm->metype, a->adsb_version, nic_a, nic_b, nic_c);
}

static int32_t altitude_to_feet(int32_t raw, altitude_unit_t unit)
{
    switch (unit) {
    case UNIT_METERS:
        return raw / 0.3048;
    case UNIT_FEET:
        return raw;
    default:
        return 0;
    }
}

//
//=========================================================================
//
// Receive new messages and update tracked aircraft state
//

struct aircraft *trackUpdateFromMessage(struct modesMessage *mm)
{
    struct aircraft *a;
    uint32_t cpr_new = 0;


    if (mm->msgtype == 32) {
        // Mode A/C, just count it (we ignore SPI)
        modeAC_count[modeAToIndex(mm->squawk)]++;
        return NULL;
    }

    if (mm->addr == 0) {
        // junk address, don't track it
        return NULL;
    }

    _messageNow = mm->sysTimestampMsg;

    // Lookup our aircraft or create a new one
    a = trackFindAircraft(mm->addr);
    if (!a) {                              // If it's a currently unknown aircraft....
        a = trackCreateAircraft(mm);       // ., create a new record for it,
        a->next = Modes.aircrafts;         // .. and put it at the head of the list
        Modes.aircrafts = a;
    }

    if (mm->signalLevel > 0) {
        a->signalLevel[a->signalNext] = mm->signalLevel;
        a->signalNext = (a->signalNext + 1) & 7;
    }
    a->seen      = messageNow();
    a->messages++;

    // count reliable messages we receive; use them as a metric to
    // decide when this is a real aircraft, not noise
    if (mm->msgtype == 11 && mm->reliable) {
        ++a->reliableDF11;
    }

    if (mm->msgtype == 17 && mm->reliable) {
        ++a->reliableDF17;
    }

    if (mm->msgtype == 19) {
        a->seen_df19 = 1;
    }

    if (a->reliableDF11 >= TRACK_RELIABLE_DF11_MESSAGES || a->reliableDF17 >= TRACK_RELIABLE_DF17_MESSAGES || a->messages >= TRACK_RELIABLE_ANY_MESSAGES) {
        a->reliable = 1;
    }

    if (!mm->reliable && !a->reliable) {
        // no further update from this message as we don't trust it
        ++a->discarded;
        return a;
    }

    // update addrtype, we only ever go towards "more direct" types
    if (mm->addrtype < a->addrtype)
        a->addrtype = mm->addrtype;

    // decide on where to stash the version
    int32_t dummy_version = -1; // used for non-adsb/adsr/tisb messages
    int32_t *message_version;

    switch (mm->source) {
    case SOURCE_ADSB:
        message_version = &a->adsb_version;
        break;
    case SOURCE_TISB:
        message_version = &a->tisb_version;
        break;
    case SOURCE_ADSR:
        message_version = &a->adsr_version;
        break;
    default:
        message_version = &dummy_version;
        break;
    }

    // assume version 0 until we see something else
    if (*message_version < 0)
        *message_version = 0;

    // category shouldn't change over time, don't bother with metadata
    if (mm->category_valid) {
        a->category = mm->category;
    }

    // operational status message
    // done early to update version / HRD / TAH
    if (mm->opstatus.valid) {
        *message_version = mm->opstatus.version;

        if (mm->opstatus.hrd != HEADING_INVALID) {
            a->adsb_hrd = mm->opstatus.hrd;
        }
        if (mm->opstatus.tah != HEADING_INVALID) {
            a->adsb_tah = mm->opstatus.tah;
        }
    }

    // fill in ADS-B v0 NACp, SIL from position message type
    if (*message_version == 0 && !mm->accuracy.nac_p_valid) {
        int32_t computed_nacp = compute_v0_nacp(mm);
        if (computed_nacp != -1) {
            mm->accuracy.nac_p_valid = 1;
            mm->accuracy.nac_p = computed_nacp;
        }
    }

    if (*message_version == 0 && mm->accuracy.sil_type == SIL_INVALID) {
        int32_t computed_sil = compute_v0_sil(mm);
        if (computed_sil != -1) {
            mm->accuracy.sil_type = SIL_UNKNOWN;
            mm->accuracy.sil = computed_sil;
        }
    }

    if (mm->altitude_baro_valid) {
        uint64_t prev_alt_updated = a->altitude_baro_valid.updated;
        if (accept_data(&a->altitude_baro_valid, mm->source)) {
            int32_t alt = altitude_to_feet(mm->altitude_baro, mm->altitude_baro_unit);
            if (a->modeC_hit) {
                int32_t new_modeC = (a->altitude_baro + 49) / 100;
                int32_t old_modeC = (alt + 49) / 100;
                if (new_modeC != old_modeC) {
                    a->modeC_hit = 0;
                }
            }

            // Altitude reliability scoring
            if (a->alt_reliable > 0 && a->altitude_baro != 0 && prev_alt_updated > 0) {
                uint64_t elapsed = messageNow() - prev_alt_updated;
                if (elapsed > 0 && elapsed < 30000) {
                    int32_t delta = abs(alt - a->altitude_baro);
                    int32_t fpm = (int32_t)((int64_t)delta * 60000 / (int64_t)elapsed);
                    if (fpm > 10000) {
                        a->alt_reliable = a->alt_reliable / 2;
                    }
                }
            }
            if (a->alt_reliable < ALTITUDE_RELIABLE_MAX) {
                if (mm->source >= SOURCE_MODE_S_CHECKED)
                    a->alt_reliable += 2;
                else
                    a->alt_reliable += 1;
                if (a->alt_reliable > ALTITUDE_RELIABLE_MAX)
                    a->alt_reliable = ALTITUDE_RELIABLE_MAX;
            }

            a->altitude_baro = alt;
        }
    }

    if (mm->squawk_valid && accept_data(&a->squawk_valid, mm->source)) {
        if (mm->squawk != a->squawk) {
            // Squawk debounce: require a new squawk to be seen twice
            // (at least 250ms apart) before accepting it.
            // First squawk (a->squawk == 0) is accepted immediately.
            if (a->squawk == 0) {
                a->squawk = mm->squawk;
            } else if (mm->squawk != a->squawkTentative) {
                // First time seeing this new squawk â€” mark tentative
                a->squawkTentative = mm->squawk;
                a->squawkTentativeChanged = messageNow();
            } else if ((messageNow() - a->squawkTentativeChanged) >= 250) {
                // Confirmed: same tentative squawk seen again after 250ms
                uint32_t old_squawk = a->squawk;
                a->modeA_hit = 0;
                a->squawk = mm->squawk;
                a->squawkTentative = 0;
                a->squawkTentativeChanged = 0;

                // Log squawk changes
                if (PanelState.enabled) {
                    int32_t rxid = -1;
                    for (int32_t ri = 0; ri < SdrManager.count; ri++)
                        if (SdrManager.receivers[ri].config.role == SDR_ROLE_ADSB) { rxid = SdrManager.receivers[ri].dev_index; break; }
                    const char *cs = trackDataValid(&a->callsign_valid) ? a->callsign : "?";
                    if (mm->squawk == 0x7500 || mm->squawk == 0x7600 || mm->squawk == 0x7700) {
                        const char *desc = mm->squawk == 0x7500 ? "HIJACK" : mm->squawk == 0x7600 ? "NORDO" : "EMERGENCY";
                        panelLogMessage("[ADSB rx%d] \xf0\x9f\x9a\xa8 %06X [%s] \xe2\x86\x92 SQUAWK %04x %s (was %04x)",
                                        rxid, a->addr, cs, mm->squawk, desc, old_squawk);
                    } else {
                        panelLogMessage("[ADSB rx%d] %06X [%s] \xe2\x86\x92 SQUAWK %04x (was %04x)",
                                        rxid, a->addr, cs, mm->squawk, old_squawk);
                    }
                }
            }
            // else: same tentative squawk seen too quickly, wait for next
        } else {
            // Squawk unchanged, clear any tentative
            a->squawkTentative = 0;
            a->squawkTentativeChanged = 0;
        }

#if 0   // Disabled for now as it obscures the origin of the data
        // Handle 7x00 without a corresponding emergency status
        if (!mm->emergency_valid) {
            emergency_t squawk_emergency;
            switch (mm->squawk) {
            case 0x7500:
                squawk_emergency = EMERGENCY_UNLAWFUL;
                break;
            case 0x7600:
                squawk_emergency = EMERGENCY_NORDO;
                break;
            case 0x7700:
                squawk_emergency = EMERGENCY_GENERAL;
                break;
            default:
                squawk_emergency = EMERGENCY_NONE;
                break;
            }

            if (squawk_emergency != EMERGENCY_NONE && accept_data(&a->emergency_valid, mm->source)) {
                a->emergency = squawk_emergency;
            }
        }
#endif
    }

    if (mm->emergency_valid && accept_data(&a->emergency_valid, mm->source)) {
        if (PanelState.enabled && mm->emergency != EMERGENCY_NONE && mm->emergency != a->emergency) {
            static const char *emerg_names[] = {
                "NONE","GENERAL","LIFEGUARD","MINFUEL","NORDO","HIJACK","DOWNED","RESERVED"
            };
            const char *ename = (mm->emergency < 8) ? emerg_names[mm->emergency] : "UNKNOWN";
            const char *cs = trackDataValid(&a->callsign_valid) ? a->callsign : "?";
            int32_t rxid = -1;
            for (int32_t ri = 0; ri < SdrManager.count; ri++)
                if (SdrManager.receivers[ri].config.role == SDR_ROLE_ADSB) { rxid = SdrManager.receivers[ri].dev_index; break; }
            panelLogMessage("[ADSB rx%d] \xf0\x9f\x9a\xa8 %06X [%s] \xe2\x86\x92 EMERGENCY %s",
                            rxid, a->addr, cs, ename);
        }
        a->emergency = mm->emergency;
    }

    if (mm->altitude_geom_valid && accept_data(&a->altitude_geom_valid, mm->source)) {
        a->altitude_geom = altitude_to_feet(mm->altitude_geom, mm->altitude_geom_unit);
    }

    if (mm->geom_delta_valid && accept_data(&a->geom_delta_valid, mm->source)) {
        a->geom_delta = mm->geom_delta;
    }

    if (mm->heading_valid) {
        heading_type_t htype = mm->heading_type;
        if (htype == HEADING_MAGNETIC_OR_TRUE) {
            htype = a->adsb_hrd;
        } else if (htype == HEADING_TRACK_OR_HEADING) {
            htype = a->adsb_tah;
        }

        if (htype == HEADING_GROUND_TRACK && accept_data(&a->track_valid, mm->source)) {
            a->track = mm->heading;
        } else if (htype == HEADING_MAGNETIC && accept_data(&a->mag_heading_valid, mm->source)) {
            a->mag_heading = mm->heading;
        } else if (htype == HEADING_TRUE && accept_data(&a->true_heading_valid, mm->source)) {
            a->true_heading = mm->heading;
        }
    }

    if (mm->track_rate_valid && accept_data(&a->track_rate_valid, mm->source)) {
        a->track_rate = mm->track_rate;
    }

    if (mm->roll_valid && accept_data(&a->roll_valid, mm->source)) {
        a->roll = mm->roll;
    }

    if (mm->gs_valid) {
        mm->gs.selected = (*message_version == 2 ? mm->gs.v2 : mm->gs.v0);
        if (accept_data(&a->gs_valid, mm->source)) {
            a->gs = mm->gs.selected;
        }
    }

    if (mm->ias_valid && accept_data(&a->ias_valid, mm->source)) {
        a->ias = mm->ias;
    }

    if (mm->tas_valid && accept_data(&a->tas_valid, mm->source)) {
        a->tas = mm->tas;
    }

    if (mm->mach_valid && accept_data(&a->mach_valid, mm->source)) {
        a->mach = mm->mach;
    }

    if (mm->baro_rate_valid && accept_data(&a->baro_rate_valid, mm->source)) {
        a->baro_rate = mm->baro_rate;
    }

    if (mm->geom_rate_valid && accept_data(&a->geom_rate_valid, mm->source)) {
        a->geom_rate = mm->geom_rate;
    }

    if (mm->airground != AG_INVALID) {
        // If our current state is UNCERTAIN, accept new data as normal
        // If our current state is certain but new data is not, only accept the uncertain state if the certain data has gone stale
        if (mm->airground != AG_UNCERTAIN ||
            (mm->airground == AG_UNCERTAIN && !trackDataFresh(&a->airground_valid))) {
            if (accept_data(&a->airground_valid, mm->source)) {
                a->airground = mm->airground;
            }
        }
    }

    if (mm->callsign_valid && accept_data(&a->callsign_valid, mm->source)) {
        if (strcmp(a->callsign, mm->callsign) != 0) {
            // The callsign changed so tell interactive to
            // re-evaluate its callsign filter regex if it has one
            a->callsign_matched = 0;

            if (PanelState.enabled && mm->callsign[0]) {
                int32_t rxid = -1;
                for (int32_t ri = 0; ri < SdrManager.count; ri++)
                    if (SdrManager.receivers[ri].config.role == SDR_ROLE_ADSB) { rxid = SdrManager.receivers[ri].dev_index; break; }
                panelLogMessage("[ADSB rx%d] %06X \xe2\x86\x92 IDENT %s", rxid, a->addr, mm->callsign);
            }
        }
        memcpy(a->callsign, mm->callsign, sizeof(a->callsign));
    }

    if (mm->nav.mcp_altitude_valid && accept_data(&a->nav_altitude_mcp_valid, mm->source)) {
        a->nav_altitude_mcp = mm->nav.mcp_altitude;
    }

    if (mm->nav.fms_altitude_valid && accept_data(&a->nav_altitude_fms_valid, mm->source)) {
        a->nav_altitude_fms = mm->nav.fms_altitude;
    }

    if (mm->nav.altitude_source != NAV_ALT_INVALID && accept_data(&a->nav_altitude_src_valid, mm->source)) {
        a->nav_altitude_src = mm->nav.altitude_source;
    }

    if (mm->nav.heading_valid && accept_data(&a->nav_heading_valid, mm->source)) {
        a->nav_heading = mm->nav.heading;
    }

    if (mm->nav.modes_valid && accept_data(&a->nav_modes_valid, mm->source)) {
        a->nav_modes = mm->nav.modes;
    }

    if (mm->nav.qnh_valid && accept_data(&a->nav_qnh_valid, mm->source)) {
        a->nav_qnh = mm->nav.qnh;
    }

    // CPR, even
    if (mm->cpr_valid && !mm->cpr_odd && accept_data(&a->cpr_even_valid, mm->source)) {
        a->cpr_even_type = mm->cpr_type;
        a->cpr_even_lat = mm->cpr_lat;
        a->cpr_even_lon = mm->cpr_lon;
        compute_nic_rc_from_message(mm, a, &a->cpr_even_nic, &a->cpr_even_rc);
        cpr_new = 1;
    }

    // CPR, odd
    if (mm->cpr_valid && mm->cpr_odd && accept_data(&a->cpr_odd_valid, mm->source)) {
        a->cpr_odd_type = mm->cpr_type;
        a->cpr_odd_lat = mm->cpr_lat;
        a->cpr_odd_lon = mm->cpr_lon;
        compute_nic_rc_from_message(mm, a, &a->cpr_odd_nic, &a->cpr_odd_rc);
        cpr_new = 1;
    }

    if (mm->accuracy.sda_valid && accept_data(&a->sda_valid, mm->source)) {
        a->sda = mm->accuracy.sda;
    }

    if (mm->accuracy.nic_a_valid && accept_data(&a->nic_a_valid, mm->source)) {
        a->nic_a = mm->accuracy.nic_a;
    }

    if (mm->accuracy.nic_c_valid && accept_data(&a->nic_c_valid, mm->source)) {
        a->nic_c = mm->accuracy.nic_c;
    }

    if (mm->accuracy.nic_baro_valid && accept_data(&a->nic_baro_valid, mm->source)) {
        a->nic_baro = mm->accuracy.nic_baro;
    }

    if (mm->accuracy.nac_p_valid && accept_data(&a->nac_p_valid, mm->source)) {
        a->nac_p = mm->accuracy.nac_p;
    }

    if (mm->accuracy.nac_v_valid && accept_data(&a->nac_v_valid, mm->source)) {
        a->nac_v = mm->accuracy.nac_v;
    }

    if (mm->accuracy.sil_type != SIL_INVALID && accept_data(&a->sil_valid, mm->source)) {
        a->sil = mm->accuracy.sil;
        if (a->sil_type == SIL_INVALID || mm->accuracy.sil_type != SIL_UNKNOWN) {
            a->sil_type = mm->accuracy.sil_type;
        }
    }

    if (mm->accuracy.gva_valid && accept_data(&a->gva_valid, mm->source)) {
        a->gva = mm->accuracy.gva;
    }

    if (mm->accuracy.sda_valid && accept_data(&a->sda_valid, mm->source)) {
        a->sda = mm->accuracy.sda;
    }

    if (mm->mrar_source_valid && accept_data(&a->mrar_source_valid, mm->source)) {
        a->mrar_source = mm->mrar_source;
    }

    if (mm->wind_valid && accept_data(&a->wind_valid, mm->source)) {
        a->wind_speed = mm->wind_speed;
        a->wind_dir = mm->wind_dir;
    }

    if (mm->temperature_valid && accept_data(&a->temperature_valid, mm->source)) {
        a->temperature = mm->temperature;
    }

    if (mm->pressure_valid && accept_data(&a->pressure_valid, mm->source)) {
        a->pressure = mm->pressure;
    }

    if (mm->turbulence_valid && accept_data(&a->turbulence_valid, mm->source)) {
        a->turbulence = mm->turbulence;
    }

    if (mm->humidity_valid && accept_data(&a->humidity_valid, mm->source)) {
        a->humidity = mm->humidity;
    }

    // MHAR (BDS 4,5)
    if (mm->mhar_turbulence_valid && accept_data(&a->mhar_turbulence_valid, mm->source)) {
        a->mhar_turbulence = mm->mhar_turbulence;
    }
    if (mm->mhar_windshear_valid && accept_data(&a->mhar_windshear_valid, mm->source)) {
        a->mhar_windshear = mm->mhar_windshear;
    }
    if (mm->mhar_microburst_valid && accept_data(&a->mhar_microburst_valid, mm->source)) {
        a->mhar_microburst = mm->mhar_microburst;
    }
    if (mm->mhar_icing_valid && accept_data(&a->mhar_icing_valid, mm->source)) {
        a->mhar_icing = mm->mhar_icing;
    }
    if (mm->mhar_wake_valid && accept_data(&a->mhar_wake_valid, mm->source)) {
        a->mhar_wake = mm->mhar_wake;
    }
    if (mm->mhar_sat_valid && accept_data(&a->mhar_sat_valid, mm->source)) {
        a->mhar_sat = mm->mhar_sat;
    }
    if (mm->mhar_asp_valid && accept_data(&a->mhar_asp_valid, mm->source)) {
        a->mhar_asp = mm->mhar_asp;
    }
    if (mm->mhar_rh_valid && accept_data(&a->mhar_rh_valid, mm->source)) {
        a->mhar_rh = mm->mhar_rh;
    }

    // Waypoints (BDS 4,1 / 4,2 / 4,3)
    if (mm->waypoint_valid && accept_data(&a->waypoint_valid, mm->source)) {
        memcpy(a->waypoint_id, mm->waypoint_id, 9);
    }
    if ((mm->waypoint_lat_valid || mm->waypoint_lon_valid) && accept_data(&a->waypoint_pos_valid, mm->source)) {
        a->waypoint_lat = mm->waypoint_lat;
        a->waypoint_lon = mm->waypoint_lon;
        a->waypoint_alt = mm->waypoint_alt_valid ? mm->waypoint_alt : 0;
    }
    if ((mm->waypoint_crossing_alt_valid || mm->waypoint_crossing_speed_valid) && accept_data(&a->waypoint_info_valid, mm->source)) {
        a->waypoint_crossing_alt = mm->waypoint_crossing_alt_valid ? mm->waypoint_crossing_alt : 0;
        a->waypoint_crossing_speed = mm->waypoint_crossing_speed_valid ? mm->waypoint_crossing_speed : 0;
    }

    // ACAS RA (BDS 3,0 from Comm-B or TC28/sub2 from ADS-B)
    if (mm->acas_ra_valid && accept_data(&a->acas_ra_valid, mm->source)) {
        a->acas_ara = mm->acas_ara;
        a->acas_rac = mm->acas_rac;
        a->acas_rat = mm->acas_rat;
        a->acas_mte = mm->acas_mte;
        a->acas_tti = mm->acas_tti;
        a->acas_threat_id = mm->acas_threat_id;
    }

    // Operational Status (TC31)
    if (mm->opstatus.valid && accept_data(&a->opstatus_valid, mm->source)) {
        a->opstatus_version = mm->opstatus.version;
        a->opstatus_om_acas_ra = mm->opstatus.om_acas_ra;
        a->opstatus_om_ident = mm->opstatus.om_ident;
        a->opstatus_om_atc = mm->opstatus.om_atc;
        a->opstatus_om_saf = mm->opstatus.om_saf;
        a->opstatus_cc_acas = mm->opstatus.cc_acas;
        a->opstatus_cc_cdti = mm->opstatus.cc_cdti;
        a->opstatus_cc_1090_in = mm->opstatus.cc_1090_in;
        a->opstatus_cc_arv = mm->opstatus.cc_arv;
        a->opstatus_cc_ts = mm->opstatus.cc_ts;
        a->opstatus_cc_tc = mm->opstatus.cc_tc;
        a->opstatus_cc_uat_in = mm->opstatus.cc_uat_in;
        a->opstatus_cc_poa = mm->opstatus.cc_poa;
        a->opstatus_cc_b2_low = mm->opstatus.cc_b2_low;
        a->opstatus_cc_lw = mm->opstatus.cc_lw;
        a->opstatus_cc_antenna_offset = mm->opstatus.cc_antenna_offset;
    }

    // Now handle derived data

    // derive geometric altitude if we have baro + delta
    if (compare_validity(&a->altitude_baro_valid, &a->altitude_geom_valid) > 0 &&
        compare_validity(&a->geom_delta_valid, &a->altitude_geom_valid) > 0) {
        // Baro and delta are both more recent than geometric, derive geometric from baro + delta
        a->altitude_geom = a->altitude_baro + a->geom_delta;
        combine_validity(&a->altitude_geom_valid, &a->altitude_baro_valid, &a->geom_delta_valid);
    }

    // If we've got a new cpr_odd or cpr_even
    if (cpr_new) {
        updatePosition(a, mm);
    }

    // Update magnetic declination when we have a position
    if (trackDataValid(&a->position_valid) &&
        (a->mag_declination_updated == 0 || (messageNow() - a->mag_declination_updated) > 30000)) {
        a->mag_declination = get_mag_declination(a->lat, a->lon);
        a->mag_declination_updated = messageNow();
    }

    // Derive wind and temperature from available data
    calc_wind(a, messageNow());
    calc_temp(a, messageNow());

    // GPS integrity monitoring
    // Evaluate whenever NIC or NACp is updated
    {
        int32_t new_integrity = 0; // normal
        uint32_t cur_nic = a->pos_nic;
        uint32_t cur_nac_p = a->nac_p;

        if (trackDataValid(&a->position_valid)) {
            if (trackDataValid(&a->nac_p_valid) && cur_nac_p <= 1)
                new_integrity = 2; // suspect: very low accuracy
            else if (trackDataValid(&a->nac_p_valid) && cur_nac_p <= 3)
                new_integrity = (new_integrity > 1) ? new_integrity : 1; // degraded

            if (cur_nic <= 1 && trackDataValid(&a->position_valid))
                new_integrity = 2; // suspect: very low integrity
            else if (cur_nic <= 3)
                new_integrity = (new_integrity > 1) ? new_integrity : 1; // degraded

            // Detect sudden NIC drop (from >=7 to <=2)
            if (a->prev_nic >= 7 && cur_nic <= 2)
                new_integrity = 2; // suspect: sudden integrity drop

            // Detect sudden NACp drop
            if (a->prev_nac_p >= 7 && cur_nac_p <= 2)
                new_integrity = 2; // suspect: sudden accuracy drop
        }

        a->gps_integrity = new_integrity;
        a->gps_integrity_updated = messageNow();

        // Save current values for next comparison
        if (trackDataValid(&a->nac_p_valid))
            a->prev_nac_p = cur_nac_p;
        if (trackDataValid(&a->position_valid))
            a->prev_nic = cur_nic;
    }

    // Circling detection: record heading and check for >360Â° cumulative change
    if (trackDataValid(&a->track_valid)) {
        uint64_t now_ms = messageNow();
        int32_t idx = a->heading_history_idx;

        a->heading_history[idx] = a->track;
        a->heading_history_time[idx] = now_ms;
        a->heading_history_idx = (idx + 1) % CIRCLING_HISTORY_SIZE;
        if (a->heading_history_count < CIRCLING_HISTORY_SIZE)
            a->heading_history_count++;

        // Compute cumulative heading change over the window
        if (a->heading_history_count >= 3) {
            float cumulative = 0;
            int32_t count = a->heading_history_count;
            int32_t start = (a->heading_history_idx - count + CIRCLING_HISTORY_SIZE) % CIRCLING_HISTORY_SIZE;

            // Only consider samples within last 120 seconds
            for (int32_t i = 0; i < count - 1; i++) {
                int32_t ci = (start + i) % CIRCLING_HISTORY_SIZE;
                int32_t ni = (start + i + 1) % CIRCLING_HISTORY_SIZE;

                if ((now_ms - a->heading_history_time[ci]) > 120000)
                    continue; // too old

                float delta = a->heading_history[ni] - a->heading_history[ci];
                // Normalize to -180..+180
                if (delta > 180) delta -= 360;
                if (delta < -180) delta += 360;
                cumulative += delta;
            }

            a->circling = (fabsf(cumulative) >= 360.0f) ? 1 : 0;
            a->circling_updated = now_ms;
        }
    }

    return (a);
}

//
// Periodic updates of tracking state
//

// Periodically match up mode A/C results with mode S results
static void trackMatchAC(uint64_t now)
{
    // clear match flags
    for (uint32_t i = 0; i < 4096; ++i) {
        modeAC_match[i] = 0;
    }

    // scan aircraft list, look for matches
    for (struct aircraft *a = Modes.aircrafts; a; a = a->next) {
        if ((now - a->seen) > 5000) {
            continue;
        }

        // match on Mode A
        if (trackDataValid(&a->squawk_valid)) {
            uint32_t i = modeAToIndex(a->squawk);
            if ((modeAC_count[i] - modeAC_lastcount[i]) >= TRACK_MODEAC_MIN_MESSAGES) {
                a->modeA_hit = 1;
                modeAC_match[i] = (modeAC_match[i] ? 0xFFFFFFFF : a->addr);
            }
        }

        // match on Mode C (+/- 100ft)
        if (trackDataValid(&a->altitude_baro_valid)) {
            int32_t modeC = (a->altitude_baro + 49) / 100;

            uint32_t modeA = modeCToModeA(modeC);
            uint32_t i = modeAToIndex(modeA);
            if (modeA && (modeAC_count[i] - modeAC_lastcount[i]) >= TRACK_MODEAC_MIN_MESSAGES) {
                a->modeC_hit = 1;
                modeAC_match[i] = (modeAC_match[i] ? 0xFFFFFFFF : a->addr);
            }

            modeA = modeCToModeA(modeC + 1);
            i = modeAToIndex(modeA);
            if (modeA && (modeAC_count[i] - modeAC_lastcount[i]) >= TRACK_MODEAC_MIN_MESSAGES) {
                a->modeC_hit = 1;
                modeAC_match[i] = (modeAC_match[i] ? 0xFFFFFFFF : a->addr);
            }

            modeA = modeCToModeA(modeC - 1);
            i = modeAToIndex(modeA);
            if (modeA && (modeAC_count[i] - modeAC_lastcount[i]) >= TRACK_MODEAC_MIN_MESSAGES) {
                a->modeC_hit = 1;
                modeAC_match[i] = (modeAC_match[i] ? 0xFFFFFFFF : a->addr);
            }
        }
    }

    // reset counts for next time
    for (uint32_t i = 0; i < 4096; ++i) {
        if (!modeAC_count[i])
            continue;

        if ((modeAC_count[i] - modeAC_lastcount[i]) < TRACK_MODEAC_MIN_MESSAGES) {
            if (++modeAC_age[i] > 15) {
                // not heard from for a while, clear it out
                modeAC_lastcount[i] = modeAC_count[i] = modeAC_age[i] = 0;
            }
        } else {
            // this one is live
            // set a high initial age for matches, so they age out rapidly
            // and don't show up on the interactive display when the matching
            // mode S data goes away or changes
            if (modeAC_match[i]) {
                modeAC_age[i] = 10;
            } else {
                modeAC_age[i] = 0;
            }
        }

        modeAC_lastcount[i] = modeAC_count[i];
    }
}

//
//=========================================================================
//
// If we don't receive new nessages within TRACK_AIRCRAFT_TTL
// we remove the aircraft from the list.
//
static void trackRemoveStaleAircraft(uint64_t now)
{
    struct aircraft *a = Modes.aircrafts;
    struct aircraft *prev = NULL;

    while(a) {
        if ((now - a->seen) > TRACK_AIRCRAFT_TTL || (!a->reliable && (now - a->seen) > TRACK_AIRCRAFT_UNRELIABLE_TTL)) {
            // Count aircraft where we saw only one message before reaping them.
            // These are likely to be due to messages with bad addresses.
            if (a->messages == 1)
                Modes.stats_current.single_message_aircraft++;
            if (!a->reliable)
                Modes.stats_current.unreliable_aircraft++;

            // Remove the element from the linked list, with care
            // if we are removing the first element
            if (!prev) {
                Modes.aircrafts = a->next; free(a); a = Modes.aircrafts;
            } else {
                prev->next = a->next; free(a); a = prev->next;
            }
        } else {

#define EXPIRE(_f) do { if (a->_f##_valid.source != SOURCE_INVALID && now >= a->_f##_valid.expires) { a->_f##_valid.source = SOURCE_INVALID; } } while (0)
            EXPIRE(callsign);
            EXPIRE(altitude_baro);
            EXPIRE(altitude_geom);
            EXPIRE(geom_delta);
            EXPIRE(gs);
            EXPIRE(ias);
            EXPIRE(tas);
            EXPIRE(mach);
            EXPIRE(track);
            EXPIRE(track_rate);
            EXPIRE(roll);
            EXPIRE(mag_heading);
            EXPIRE(true_heading);
            EXPIRE(baro_rate);
            EXPIRE(geom_rate);
            EXPIRE(squawk);
            EXPIRE(emergency);
            EXPIRE(airground);
            EXPIRE(nav_qnh);
            EXPIRE(nav_altitude_mcp);
            EXPIRE(nav_altitude_fms);
            EXPIRE(nav_altitude_src);
            EXPIRE(nav_heading);
            EXPIRE(nav_modes);
            EXPIRE(cpr_odd);
            EXPIRE(cpr_even);
            EXPIRE(position);
            EXPIRE(nic_a);
            EXPIRE(nic_c);
            EXPIRE(nic_baro);
            EXPIRE(nac_p);
            EXPIRE(nac_v);
            EXPIRE(sil);
            EXPIRE(gva);
            EXPIRE(sda);
            EXPIRE(mrar_source);
            EXPIRE(wind);
            EXPIRE(temperature);
            EXPIRE(pressure);
            EXPIRE(turbulence);
            EXPIRE(humidity);
            EXPIRE(mhar_turbulence);
            EXPIRE(mhar_windshear);
            EXPIRE(mhar_microburst);
            EXPIRE(mhar_icing);
            EXPIRE(mhar_wake);
            EXPIRE(mhar_sat);
            EXPIRE(mhar_asp);
            EXPIRE(mhar_rh);
            EXPIRE(waypoint);
            EXPIRE(waypoint_pos);
            EXPIRE(waypoint_info);
            EXPIRE(acas_ra);
            EXPIRE(opstatus);
#undef EXPIRE
            prev = a; a = a->next;
        }
    }
}


//
// Entry point for periodic updates
//

void trackPeriodicUpdate()
{
    static uint64_t next_update;
    uint64_t now = mstime();

    // Only do updates once per second
    if (now >= next_update) {
        next_update = now + 1000;
        trackRemoveStaleAircraft(now);
        trackMatchAC(now);
    }
}

// ======================== Decoder-sourced aircraft update ========================
// Used by FLARM, FANET, OGNTP, P3I, ADS-L: position is already resolved,
// no CPR decode needed. Updates the aircraft struct directly.

struct aircraft *trackUpdateFromDecoder(const aircraft_update_t *upd)
{
    if (upd->addr == 0) return NULL;

    _messageNow = upd->timestamp_ms;

    // Use SOURCE_TISB priority (same as synthetic DF18 used before)
    datasource_t src = SOURCE_TISB;

    struct aircraft *a = trackFindAircraft(upd->addr);
    if (!a) {
        // Create a new aircraft manually (no modesMessage available)
        static struct aircraft zero;
        a = (struct aircraft *) malloc(sizeof(*a));
        *a = zero;
        a->addr = upd->addr;
        a->addrtype = ADDR_TISB_OTHER;
        for (int32_t i = 0; i < 8; ++i)
            a->signalLevel[i] = 1e-5;
        a->adsb_version = -1;
        a->adsb_hrd = HEADING_MAGNETIC;
        a->adsb_tah = HEADING_GROUND_TRACK;
        a->fatsv_last_emitted = a->fatsv_last_force_emit = messageNow();

        // Initialize data validity stale/expire intervals
#define F(f,s,e) do { a->f##_valid.stale_interval = (s) * 1000; a->f##_valid.expire_interval = (e) * 1000; } while (0)
        F(callsign,        60, 70);
        F(altitude_baro,   15, 70);
        F(altitude_geom,   60, 70);
        F(geom_delta,      60, 70);
        F(gs,              60, 70);
        F(ias,             60, 70);
        F(tas,             60, 70);
        F(mach,            60, 70);
        F(track,           60, 70);
        F(track_rate,      60, 70);
        F(roll,            60, 70);
        F(mag_heading,     60, 70);
        F(true_heading,    60, 70);
        F(baro_rate,       60, 70);
        F(geom_rate,       60, 70);
        F(squawk,          15, 70);
        F(emergency,       60, 70);
        F(airground,       15, 70);
        F(nav_qnh,         60, 70);
        F(nav_altitude_mcp,60, 70);
        F(nav_altitude_fms,60, 70);
        F(nav_altitude_src,60, 70);
        F(nav_heading,     60, 70);
        F(nav_modes,       60, 70);
        F(cpr_odd,         60, 70);
        F(cpr_even,        60, 70);
        F(position,        60, 70);
        F(nic_a,           60, 70);
        F(nic_c,           60, 70);
        F(nic_baro,        60, 70);
        F(nac_p,           60, 70);
        F(nac_v,           60, 70);
        F(sil,             60, 70);
        F(gva,             60, 70);
        F(sda,             60, 70);
        F(wind,            60, 70);
        F(temperature,     60, 70);
        F(pressure,        60, 70);
        F(turbulence,      60, 70);
        F(humidity,        60, 70);
        F(mhar_turbulence, 60, 70);
        F(mhar_windshear,  60, 70);
        F(mhar_microburst, 60, 70);
        F(mhar_icing,      60, 70);
        F(mhar_wake,       60, 70);
        F(mhar_sat,        60, 70);
        F(mhar_asp,        60, 70);
        F(mhar_rh,         60, 70);
        F(waypoint,        60, 70);
        F(waypoint_pos,    60, 70);
        F(waypoint_info,   60, 70);
        F(acas_ra,         60, 70);
        F(opstatus,        60, 70);
        F(mrar_source,     60, 70);
#undef F

        a->next = Modes.aircrafts;
        Modes.aircrafts = a;
        Modes.stats_current.unique_aircraft++;
    }

    // Update signal level
    if (upd->signal_level > 0) {
        a->signalLevel[a->signalNext] = upd->signal_level;
        a->signalNext = (a->signalNext + 1) & 7;
    }
    a->seen = messageNow();
    a->messages++;

    // Decoder-sourced messages are reliable by definition (CRC already checked)
    a->reliable = 1;

    // Callsign
    if (upd->callsign_valid && accept_data(&a->callsign_valid, src)) {
        memcpy(a->callsign, upd->callsign, sizeof(a->callsign));
    }

    // Category
    if (upd->category_valid) {
        a->category = upd->category;
    }

    // Altitude
    if (upd->altitude_valid) {
        if (upd->altitude_is_baro) {
            if (accept_data(&a->altitude_baro_valid, src)) {
                if (a->alt_reliable < ALTITUDE_RELIABLE_MAX)
                    a->alt_reliable = ALTITUDE_RELIABLE_MAX;
                a->altitude_baro = upd->altitude_ft;
            }
        } else {
            if (accept_data(&a->altitude_geom_valid, src)) {
                a->altitude_geom = upd->altitude_ft;
            }
        }
    }

    // Position (already resolved â€” no CPR needed)
    if (upd->position_valid) {
        if (accept_data(&a->position_valid, src)) {
            // Compute calc_track from consecutive positions
            if (a->prev_pos_updated > 0) {
                double dist = greatcircle(a->prev_lat, a->prev_lon, upd->lat, upd->lon);
                if (dist >= TRACK_CALC_TRACK_MIN_DIST) {
                    a->calc_track = (float)get_bearing(a->prev_lat, a->prev_lon, upd->lat, upd->lon);
                    a->calc_track_updated = messageNow();
                }
            }
            a->prev_lat = upd->lat;
            a->prev_lon = upd->lon;
            a->prev_pos_updated = messageNow();

            a->lat = upd->lat;
            a->lon = upd->lon;
            a->pos_nic = 8;   // ~185m (conservative for FLARM GPS)
            a->pos_rc = 186;

            update_range_histogram(upd->lat, upd->lon);
        }
    }

    // Ground speed + track
    if (upd->velocity_valid) {
        if (accept_data(&a->gs_valid, src)) {
            a->gs = upd->ground_speed_kt;
        }
        if (accept_data(&a->track_valid, src)) {
            a->track = upd->heading_deg;
        }
        if (upd->vert_rate_fpm != 0) {
            if (accept_data(&a->geom_rate_valid, src)) {
                a->geom_rate = upd->vert_rate_fpm;
            }
        }
    }

    // Squawk
    if (upd->squawk_valid && accept_data(&a->squawk_valid, src)) {
        a->squawk = upd->squawk;
    }

    // Air/ground
    if (upd->air_ground != DECODE_AG_UNKNOWN) {
        if (accept_data(&a->airground_valid, src)) {
            a->airground = (upd->air_ground == DECODE_AG_GROUND) ? AG_GROUND : AG_AIRBORNE;
        }
    }

    // FLARM/FANET metadata
    if (upd->flarm_acft_type > 0) {
        a->flarm_acft_type = upd->flarm_acft_type;
    }
    a->flarm_addr_type = upd->flarm_addr_type;
    a->flarm_proto_version = upd->flarm_proto_version;

    // FANET HW info
    if (upd->hw_info.valid) {
        a->fanet_hwinfo.device_type = upd->hw_info.device_type;
        a->fanet_hwinfo.uptime_minutes = upd->hw_info.uptime_min;
        a->fanet_hwinfo.rssi = upd->hw_info.rssi;
        a->fanet_hwinfo.valid = 1;
    }

    // FANET thermal
    if (upd->thermal.valid) {
        a->fanet_thermal.latitude = upd->thermal.lat;
        a->fanet_thermal.longitude = upd->thermal.lon;
        a->fanet_thermal.altitude = upd->thermal.altitude_m;
        a->fanet_thermal.climb = upd->thermal.climb_ms;
        a->fanet_thermal.wind_speed = upd->thermal.wind_speed_kmh;
        a->fanet_thermal.wind_heading = upd->thermal.wind_heading_deg;
        a->fanet_thermal.confidence = upd->thermal.confidence;
        a->fanet_thermal.valid = 1;
    }

    // Radiosonde metadata
    if (upd->sonde.valid) {
        memcpy(a->sonde_info.serial, upd->sonde.serial, sizeof(a->sonde_info.serial));
        memcpy(a->sonde_info.sonde_type, upd->sonde.sonde_type, sizeof(a->sonde_info.sonde_type));
        a->sonde_info.frame_num = upd->sonde.frame_num;
        a->sonde_info.rs_errors = upd->sonde.rs_errors;
        a->sonde_info.satellites = upd->sonde.satellites;
        a->sonde_info.vel_v = upd->sonde.vel_v;
        a->sonde_info.freq_mhz = upd->sonde.freq_mhz;
        a->sonde_info.valid = 1;
    }

    // Magnetic declination
    if (trackDataValid(&a->position_valid) &&
        (a->mag_declination_updated == 0 || (messageNow() - a->mag_declination_updated) > 30000)) {
        a->mag_declination = get_mag_declination(a->lat, a->lon);
        a->mag_declination_updated = messageNow();
    }

    return a;
}
