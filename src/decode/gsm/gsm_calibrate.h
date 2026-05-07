// gsm_calibrate.h: GSM-based PPM calibration for RTL-SDR dongles
//
// Replicates the algorithm from ogn-rf's gsm_scan tool natively in dump1090.
// Scans E-GSM-900 downlink band (920-960 MHz), finds GSM base station carriers,
// and measures crystal frequency error by analyzing the GMSK spectral peak offset.

#ifndef GSM_CALIBRATE_H
#define GSM_CALIBRATE_H

#include "sdr_backend.h"

typedef struct {
    int    success;           // 1 if calibration succeeded
    double corrected_ppm;     // final corrected PPM value (current + offset)
    double measured_offset;   // measured PPM offset from current setting
    double rms;               // RMS of measurements (lower = more consistent)
    int    samples;           // number of valid measurements used
    char   error[256];        // error/warning message
} gsm_cal_result_t;

// Run GSM frequency calibration on the specified SDR dongle.
// The dongle must NOT be in use (close it before calling).
// serial:      RTL-SDR serial number string
// current_ppm: current PPM correction setting
// gain_db:     tuner gain in dB (e.g. 49.6)
// backend:     which backend library to use (AUTO/RTLSDR/SDRGG)
gsm_cal_result_t gsm_calibrate(const char *serial, int current_ppm, float gain_db,
                                sdr_backend_type_t backend);

#endif
