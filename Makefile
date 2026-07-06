PROGNAME=dump1090

DUMP1090_VERSION ?= 1.0.7
DUMP1090_DIAGNOSTICS ?= no

# ======================== Directory layout ========================

SRCDIR_MAIN    := src/main
SRCDIR_ADSB    := src/adsb
SRCDIR_UTIL    := src/util
SRCDIR_NET     := src/net
SRCDIR_SDR     := src/sdr
SRCDIR_PANEL   := src/panel
SRCDIR_STUBS   := src/stubs
SRCDIR_FLARM   := src/decode/flarm
SRCDIR_ACARS   := src/decode/acars
SRCDIR_VDL2    := src/decode/vdl2
SRCDIR_SONDE   := src/decode/sonde
SRCDIR_POCSAG  := src/decode/pocsag
SRCDIR_GSM     := src/decode/gsm
SRCDIR_LTE     := src/decode/lte
SRCDIR_IOT     := src/decode/iot
SRCDIR_FANET   := src/decode/fanet
SRCDIR_SARSAT  := src/decode/sarsat
SRCDIR_DISPATCH := src/dispatch
INCLUDEDIR     := include

OBJDIR         := obj

# VPATH: where make looks for source files
VPATH := $(SRCDIR_MAIN):$(SRCDIR_ADSB):$(SRCDIR_UTIL):$(SRCDIR_NET):$(SRCDIR_SDR):$(SRCDIR_PANEL):$(SRCDIR_STUBS):$(SRCDIR_FLARM):$(SRCDIR_ACARS):$(SRCDIR_VDL2):$(SRCDIR_SONDE):$(SRCDIR_POCSAG):$(SRCDIR_GSM):$(SRCDIR_LTE):$(SRCDIR_IOT):$(SRCDIR_FANET):$(SRCDIR_SARSAT):$(SRCDIR_DISPATCH)

# ======================== Compiler flags ========================

CFLAGS ?= -O3 -g
DUMP1090_CFLAGS := -std=c11 -fno-common -Wall -Wmissing-declarations -Werror -Wformat-signedness -W

# C++ flags for all source files (entire project is now C++)
CXXFLAGS_ALL = -std=c++17 -O3 -g -Wall -Werror -Wno-write-strings

# Include paths: all source directories + include/ so #include "foo.h" works from anywhere
INCLUDE_DIRS := -I. -I$(INCLUDEDIR) -I$(SRCDIR_MAIN) -I$(SRCDIR_ADSB) -I$(SRCDIR_UTIL) -I$(SRCDIR_NET) -I$(SRCDIR_SDR) -I$(SRCDIR_PANEL) -I$(SRCDIR_STUBS) -I$(SRCDIR_FLARM) -I$(SRCDIR_ACARS) -I$(SRCDIR_VDL2) -I$(SRCDIR_SONDE) -I$(SRCDIR_POCSAG) -I$(SRCDIR_GSM) -I$(SRCDIR_LTE) -I$(SRCDIR_IOT) -I$(SRCDIR_FANET) -I$(SRCDIR_SARSAT) -I$(SRCDIR_DISPATCH)

DUMP1090_CPPFLAGS := $(INCLUDE_DIRS) -D_POSIX_C_SOURCE=200112L -DMODES_DUMP1090_VERSION=\"$(DUMP1090_VERSION)\" -DMODES_DUMP1090_VARIANT=\"dump1090-gg-light\"

ifeq ($(DUMP1090_DIAGNOSTICS),yes)
  DUMP1090_CPPFLAGS += -DMODES_ENABLE_DIAGNOSTICS=1
endif

CXX ?= g++
CXXFLAGS_DISPATCH = -std=c++17 -O3 -g -Wall -Werror -Wno-write-strings $(DUMP1090_CPPFLAGS)

LIBS = -lpthread -lm
SDR_OBJ = $(OBJDIR)/cpu.o $(OBJDIR)/sdr.o $(OBJDIR)/fifo.o $(OBJDIR)/sdr_ifile.o $(OBJDIR)/sdr_backend.o dsp/helpers/tables.o

# Try to autodetect available libraries via pkg-config if no explicit setting was used
PKGCONFIG=$(shell pkg-config --version >/dev/null 2>&1 && echo "yes" || echo "no")
ifeq ($(PKGCONFIG), yes)
  ifndef RTLSDR
    ifdef RTLSDR_PREFIX
      RTLSDR := yes
    else
      RTLSDR := $(shell pkg-config --exists librtlsdr && echo "yes" || echo "no")
    endif
  endif

  ifndef BLADERF
    BLADERF := $(shell pkg-config --exists libbladeRF && echo "yes" || echo "no")
  endif

  ifndef HACKRF
    HACKRF := $(shell pkg-config --exists libhackrf && echo "yes" || echo "no")
  endif

  ifndef LIMESDR
    LIMESDR := $(shell pkg-config --exists LimeSuite && echo "yes" || echo "no")
  endif

  ifndef SOAPYSDR
    SOAPYSDR := $(shell pkg-config --exists SoapySDR && echo "yes" || echo "no")
  endif
else
  # pkg-config not available. Only use explicitly enabled libraries.
  RTLSDR ?= no
  BLADERF ?= no
  HACKRF ?= no
  LIMESDR ?= no
  SOAPYSDR ?= no
endif

BUILD_UNAME := $(shell uname)
BUILD_ARCH := $(shell uname -m)

UNAME ?= $(BUILD_UNAME)
ARCH ?= $(BUILD_ARCH)

ifeq ($(UNAME), Linux)
  DUMP1090_CPPFLAGS += -D_DEFAULT_SOURCE
  LIBS += -lrt
  LIBS_USB += -lusb-1.0
  LIBS_CURSES := -lncurses
  CPUFEATURES ?= yes
endif

ifeq ($(UNAME), Darwin)
  ifneq ($(shell sw_vers -productVersion | egrep '^10\.([0-9]|1[01])\.'),) # Mac OS X ver <= 10.11
    DUMP1090_CPPFLAGS += -DMISSING_GETTIME
    COMPAT += compat/clock_gettime/clock_gettime.o
  endif
  DUMP1090_CPPFLAGS += -DMISSING_NANOSLEEP
  COMPAT += compat/clock_nanosleep/clock_nanosleep.o
  ifeq ($(PKGCONFIG), yes)
    LIBS_SDR += $(shell pkg-config --libs-only-L libusb-1.0)
  endif
  LIBS_USB += -lusb-1.0
  LIBS_CURSES := -lncurses
  # cpufeatures reportedly does not work (yet) on darwin arm64
  ifneq ($(ARCH),arm64)
    CPUFEATURES ?= yes
  endif
endif

ifeq ($(UNAME), OpenBSD)
  DUMP1090_CPPFLAGS += -DMISSING_NANOSLEEP
  COMPAT += compat/clock_nanosleep/clock_nanosleep.o
  LIBS_USB += -lusb-1.0
  LIBS_CURSES := -lncurses
endif

ifeq ($(UNAME), FreeBSD)
  DUMP1090_CPPFLAGS += -D_DEFAULT_SOURCE
  LIBS += -lrt
  LIBS_USB += -lusb
  LIBS_CURSES := -lncurses
endif

ifeq ($(UNAME), NetBSD)
  DUMP1090_CPPFLAGS += -D_DEFAULT_SOURCE
  LIBS += -lrt
  LIBS_USB += -lusb-1.0
  LIBS_CURSES := -lcurses
endif

CPUFEATURES ?= no

ifeq ($(CPUFEATURES),yes)
  include Makefile.cpufeatures
  DUMP1090_CPPFLAGS += -DENABLE_CPUFEATURES -Icpu_features/include
endif

RTLSDR ?= yes
BLADERF ?= yes

ifeq ($(RTLSDR), yes)
  SDR_OBJ += $(OBJDIR)/sdr_rtlsdr.o
  DUMP1090_CPPFLAGS += -DENABLE_RTLSDR

  ifdef RTLSDR_PREFIX
    DUMP1090_CPPFLAGS += -I$(RTLSDR_PREFIX)/include
    ifeq ($(STATIC), yes)
      LIBS_SDR += -L$(RTLSDR_PREFIX)/lib -Wl,-Bstatic -lrtlsdr -Wl,-Bdynamic $(LIBS_USB)
    else
      LIBS_SDR += -L$(RTLSDR_PREFIX)/lib -lrtlsdr $(LIBS_USB)
    endif
  else
    RTLSDR_CFLAGS := $(shell pkg-config --cflags librtlsdr)
    RTLSDR_CFLAGS := $(filter-out -std=%,$(RTLSDR_CFLAGS))
    RTLSDR_CFLAGS := $(filter-out -I/,$(RTLSDR_CFLAGS))
    DUMP1090_CPPFLAGS += $(RTLSDR_CFLAGS)

    RTLSDR_LFLAGS := $(shell pkg-config --libs-only-L librtlsdr)
    ifeq ($(RTLSDR_LFLAGS),-L)
      LIBS_SDR += $(shell pkg-config --libs-only-l --libs-only-other librtlsdr)
    else
      LIBS_SDR += $(shell pkg-config --libs librtlsdr)
    endif
  endif
endif

SDRGG ?= no

ifeq ($(SDRGG), yes)
  SDR_OBJ += $(OBJDIR)/sdr_backend_sdrgg.o
  DUMP1090_CPPFLAGS += -DENABLE_SDRGG
  CXX ?= g++
  CXXFLAGS = -std=c++2a -O3 -Wall $(DUMP1090_CPPFLAGS)

  ifdef SDRGG_PREFIX
    ifneq ($(wildcard $(SDRGG_PREFIX)/sdrgg.h),)
      DUMP1090_CPPFLAGS += -I$(SDRGG_PREFIX)
      SDRGG_LIBDIR := $(SDRGG_PREFIX)
    else
      DUMP1090_CPPFLAGS += -I$(SDRGG_PREFIX)/include
      SDRGG_LIBDIR := $(SDRGG_PREFIX)/lib
    endif
    LIBS_SDR += -L$(SDRGG_LIBDIR) -Wl,-rpath,$(SDRGG_LIBDIR) -lsdrgg -lstdc++
  else ifeq ($(PKGCONFIG), yes)
    ifeq ($(shell pkg-config --exists libsdrgg && echo "yes" || echo "no"), yes)
      DUMP1090_CPPFLAGS += $(shell pkg-config --cflags libsdrgg)
      LIBS_SDR += $(shell pkg-config --libs libsdrgg) -lstdc++
    else
      LIBS_SDR += -lsdrgg -lstdc++
    endif
  else
    LIBS_SDR += -lsdrgg -lstdc++
  endif
endif

ifeq ($(BLADERF), yes)
  SDR_OBJ += $(OBJDIR)/sdr_bladerf.o
  DUMP1090_CPPFLAGS += -DENABLE_BLADERF
  DUMP1090_CPPFLAGS += $(shell pkg-config --cflags libbladeRF)
  LIBS_SDR += $(shell pkg-config --libs libbladeRF)
endif

ifeq ($(HACKRF), yes)
  SDR_OBJ += $(OBJDIR)/sdr_hackrf.o
  DUMP1090_CPPFLAGS += -DENABLE_HACKRF
  DUMP1090_CPPFLAGS += $(shell pkg-config --cflags libhackrf)
  LIBS_SDR += $(shell pkg-config --libs libhackrf)
endif

ifeq ($(LIMESDR), yes)
  SDR_OBJ += $(OBJDIR)/sdr_limesdr.o
  DUMP1090_CPPFLAGS += -DENABLE_LIMESDR
  DUMP1090_CPPFLAGS += $(shell pkg-config --cflags LimeSuite)
  LIBS_SDR += $(shell pkg-config --libs LimeSuite)
endif

ifeq ($(SOAPYSDR), yes)
  SDR_OBJ += $(OBJDIR)/sdr_soapy.o
  DUMP1090_CPPFLAGS += -DENABLE_SOAPYSDR
  DUMP1090_CPPFLAGS += $(shell pkg-config --cflags SoapySDR)
  LIBS_SDR += $(shell pkg-config --libs SoapySDR)
endif


##
## starch (runtime DSP code selection) mix, architecture-specific
##

ifneq ($(CPUFEATURES),yes)
  STARCH_MIX := generic
  DUMP1090_CPPFLAGS += -DSTARCH_MIX_GENERIC
else
  ifeq ($(ARCH),x86_64)
    STARCH_MIX := x86
    DUMP1090_CPPFLAGS += -DSTARCH_MIX_X86
  else ifeq ($(ARCH),amd64)
    STARCH_MIX := x86
    DUMP1090_CPPFLAGS += -DSTARCH_MIX_X86
  else ifeq ($(findstring aarch,$(ARCH)),aarch)
    STARCH_MIX := aarch64
    DUMP1090_CPPFLAGS += -DSTARCH_MIX_AARCH64
  else ifeq ($(findstring arm64,$(ARCH)),arm64)
    STARCH_MIX := aarch64
    DUMP1090_CPPFLAGS += -DSTARCH_MIX_AARCH64
  else ifeq ($(findstring arm,$(ARCH)),arm)
    STARCH_MIX := arm
    DUMP1090_CPPFLAGS += -DSTARCH_MIX_ARM
  else
    STARCH_MIX := generic
    DUMP1090_CPPFLAGS += -DSTARCH_MIX_GENERIC
  endif
endif

# ======================== Build targets ========================

all: showconfig dump1090 view1090 starch-benchmark

ALL_CCFLAGS := $(CPPFLAGS) $(DUMP1090_CPPFLAGS) $(CFLAGS) $(DUMP1090_CFLAGS)
ALL_CXXFLAGS := $(DUMP1090_CPPFLAGS) $(CXXFLAGS_ALL)

STARCH_COMPILE := $(CC) $(ALL_CCFLAGS) -c
include dsp/generated/makefile.$(STARCH_MIX)

showconfig:
	@echo "Building with:" >&2
	@echo "  Version string:   $(DUMP1090_VERSION)" >&2
	@echo "  Architecture:     $(ARCH)" >&2
	@echo "  DSP mix:          $(STARCH_MIX)" >&2
	@echo "  RTLSDR support:   $(RTLSDR)" >&2
	@echo "  BladeRF support:  $(BLADERF)" >&2
	@echo "  HackRF support:   $(HACKRF)" >&2
	@echo "  LimeSDR support:  $(LIMESDR)" >&2
	@echo "  SoapySDR support: $(SOAPYSDR)" >&2
	@echo "  libsdrgg support: $(SDRGG)" >&2
  @echo "  Diagnostics:      $(DUMP1090_DIAGNOSTICS)" >&2

# Create obj directory
$(OBJDIR):
	mkdir -p $(OBJDIR)

# Pattern rule: compile .cpp from VPATH into obj/
$(OBJDIR)/%.o: %.cpp | $(OBJDIR)
	$(CXX) $(ALL_CXXFLAGS) -c $< -o $@

# Pattern rule: compile .c from VPATH into obj/ (for C99-only files)
$(OBJDIR)/%.o: %.c | $(OBJDIR)
	$(CC) $(ALL_CCFLAGS) -c $< -o $@

# DSP helpers (still C, compiled in-place)
dsp/helpers/tables.o: dsp/helpers/tables.c
	$(CC) $(ALL_CCFLAGS) -c $< -o $@

# cpu_features (still C)
cpu_features/src/%.o: cpu_features/src/%.c
	$(CC) $(ALL_CCFLAGS) -c $< -o $@

# compat (still C)
compat/clock_gettime/%.o: compat/clock_gettime/%.c
	$(CC) $(ALL_CCFLAGS) -c $< -o $@

compat/clock_nanosleep/%.o: compat/clock_nanosleep/%.c
	$(CC) $(ALL_CCFLAGS) -c $< -o $@

# ======================== Object lists ========================

DUMP1090_OBJS := \
	$(OBJDIR)/dump1090.o \
	$(OBJDIR)/interactive.o \
	$(OBJDIR)/app_config.o \
	$(OBJDIR)/mode_ac.o \
	$(OBJDIR)/mode_s.o \
	$(OBJDIR)/comm_b.o \
	$(OBJDIR)/demod_2400.o \
	$(OBJDIR)/stats.o \
	$(OBJDIR)/cpr.o \
	$(OBJDIR)/icao_filter.o \
	$(OBJDIR)/track.o \
	$(OBJDIR)/adaptive.o \
	$(OBJDIR)/elm.o \
	$(OBJDIR)/util.o \
	$(OBJDIR)/crc.o \
	$(OBJDIR)/convert.o \
	$(OBJDIR)/ais_charset.o \
	$(OBJDIR)/cpdlc_decode.o \
	$(OBJDIR)/anet.o \
	$(OBJDIR)/net_io.o \
	$(OBJDIR)/feeder_thread.o \
	$(OBJDIR)/mlat_client.o \
	$(OBJDIR)/fa_mlat.o \
	$(OBJDIR)/piaware_client.o \
	$(OBJDIR)/planefinder_client.o \
	$(OBJDIR)/fr24_client.o \
	$(OBJDIR)/radarbox_client.o \
	$(OBJDIR)/opensky_client.o \
	$(OBJDIR)/ogn_client.o \
	$(OBJDIR)/sondehub_client.o \
	$(OBJDIR)/airframes_feed.o \
	$(OBJDIR)/flarm_decode.o \
	$(OBJDIR)/flarm_demod.o \
	$(OBJDIR)/flarm_reader.o \
	$(OBJDIR)/ogntp_decode.o \
	$(OBJDIR)/p3i_decode.o \
	$(OBJDIR)/p3i_demod.o \
	$(OBJDIR)/adsl_decode.o \
	$(OBJDIR)/acars_demod.o \
	$(OBJDIR)/acars_label.o \
	$(OBJDIR)/vdl2_demod.o \
	$(OBJDIR)/sonde_demod.o \
	$(OBJDIR)/pocsag_demod.o \
	$(OBJDIR)/gsm_calibrate.o \
	$(OBJDIR)/gsm_decode.o \
	$(OBJDIR)/gsm_tracker.o \
	$(OBJDIR)/lte_decode.o \
	$(OBJDIR)/lte_sib.o \
	$(OBJDIR)/lte_tracker.o \
	$(OBJDIR)/iot_decode.o \
	$(OBJDIR)/iot_tracker.o \
	$(OBJDIR)/fanet_decode.o \
	$(OBJDIR)/sarsat_decode.o \
	$(OBJDIR)/config_panel.o \
	$(OBJDIR)/decoder_config.o \
	$(OBJDIR)/sdr_receiver.o \
	$(OBJDIR)/dispatcher.o \
	$(OBJDIR)/msg_queue.o

VIEW1090_OBJS := \
	$(OBJDIR)/view1090.o \
	$(OBJDIR)/interactive.o \
	$(OBJDIR)/mode_ac.o \
	$(OBJDIR)/mode_s.o \
	$(OBJDIR)/comm_b.o \
	$(OBJDIR)/stats.o \
	$(OBJDIR)/cpr.o \
	$(OBJDIR)/icao_filter.o \
	$(OBJDIR)/track.o \
	$(OBJDIR)/elm.o \
	$(OBJDIR)/util.o \
	$(OBJDIR)/crc.o \
	$(OBJDIR)/ais_charset.o \
	$(OBJDIR)/cpdlc_decode.o \
	$(OBJDIR)/anet.o \
	$(OBJDIR)/net_io.o \
	$(OBJDIR)/mlat_client.o \
	$(OBJDIR)/fa_mlat.o \
	$(OBJDIR)/piaware_client.o \
	$(OBJDIR)/feeder_thread_stub.o \
	$(OBJDIR)/config_panel_stub.o \
	$(OBJDIR)/dispatcher_stub.o \
	$(OBJDIR)/sdr_stub.o \
	$(OBJDIR)/msg_queue.o

FAUP1090_OBJS := \
	$(OBJDIR)/faup1090.o \
	$(OBJDIR)/mode_ac.o \
	$(OBJDIR)/mode_s.o \
	$(OBJDIR)/comm_b.o \
	$(OBJDIR)/stats.o \
	$(OBJDIR)/cpr.o \
	$(OBJDIR)/icao_filter.o \
	$(OBJDIR)/track.o \
	$(OBJDIR)/elm.o \
	$(OBJDIR)/util.o \
	$(OBJDIR)/crc.o \
	$(OBJDIR)/ais_charset.o \
	$(OBJDIR)/cpdlc_decode.o \
	$(OBJDIR)/anet.o \
	$(OBJDIR)/net_io.o \
	$(OBJDIR)/piaware_client.o \
	$(OBJDIR)/fa_mlat.o \
	$(OBJDIR)/feeder_thread_stub.o \
	$(OBJDIR)/config_panel_stub.o \
	$(OBJDIR)/dispatcher_stub.o \
	$(OBJDIR)/sdr_stub.o \
	$(OBJDIR)/msg_queue.o

# ======================== Link targets ========================

dump1090: $(DUMP1090_OBJS) $(SDR_OBJ) $(COMPAT) $(CPUFEATURES_OBJS) $(STARCH_OBJS)
	$(CXX) -g -o $@ $^ $(LDFLAGS) $(LIBS) $(LIBS_SDR) $(LIBS_CURSES) -lssl -lcrypto -lz -lstdc++

view1090: $(VIEW1090_OBJS) $(COMPAT)
	$(CXX) -g -o $@ $^ $(LDFLAGS) $(LIBS) $(LIBS_CURSES) -lssl -lcrypto -lstdc++

faup1090: $(FAUP1090_OBJS) $(COMPAT)
	$(CXX) -g -o $@ $^ $(LDFLAGS) $(LIBS) -lssl -lcrypto -lstdc++

starch-benchmark: $(OBJDIR)/cpu.o dsp/helpers/tables.o $(CPUFEATURES_OBJS) $(STARCH_OBJS) $(STARCH_BENCHMARK_OBJ)
	$(CC) -g -o $@ $^ $(LDFLAGS) $(LIBS)

# ======================== Clean ========================

clean:
	rm -rf $(OBJDIR)
	rm -f compat/clock_gettime/*.o compat/clock_nanosleep/*.o cpu_features/src/*.o dsp/generated/*.o dsp/helpers/*.o $(CPUFEATURES_OBJS)
	rm -f dump1090 view1090 faup1090 cprtests crctests starch-benchmark
