// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// cpr.h - Compact Position Reporting prototypes
//
// Copyright (c) 2014,2015 Oliver Jowett <oliver@mutability.co.uk>
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

#ifndef DUMP1090_CPR_H
#define DUMP1090_CPR_H
#include <stdint.h>

int32_t decodeCPRairborne(int32_t even_cprlat, int32_t even_cprlon,
                      int32_t odd_cprlat, int32_t odd_cprlon,
                      int32_t fflag,
                      double *out_lat, double *out_lon);

int32_t decodeCPRsurface(double reflat, double reflon,
                     int32_t even_cprlat, int32_t even_cprlon,
                     int32_t odd_cprlat, int32_t odd_cprlon,
                     int32_t fflag,
                     double *out_lat, double *out_lon);

int32_t decodeCPRrelative(double reflat, double reflon,
                      int32_t cprlat, int32_t cprlon,
                      int32_t fflag, int32_t surface,
                      double *out_lat, double *out_lon);

#endif
