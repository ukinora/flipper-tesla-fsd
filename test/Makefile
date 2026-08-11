# Host unit tests for the Tesla FSD protocol core.
# Compiles fsd_logic/fsd_handler.c on the host — no Flipper SDK / SPI needed.
#
#   make -C test check

CC      ?= cc
CFLAGS  ?= -std=c11 -Wall -Wextra -O0 -g
ROOT    := ..
SRC     := test_fsd_core.c $(ROOT)/fsd_logic/fsd_handler.c $(ROOT)/fsd_logic/fsd_profile.c $(ROOT)/fsd_logic/fsd_autonomy.c

test_fsd_core: $(SRC) $(ROOT)/fsd_logic/fsd_handler.h $(ROOT)/fsd_logic/fsd_types.h $(ROOT)/fsd_logic/fsd_checksum.h $(ROOT)/fsd_logic/fsd_can_ops.h $(ROOT)/fsd_logic/fsd_state.h $(ROOT)/fsd_logic/fsd_capture.h $(ROOT)/fsd_logic/fsd_profile.h $(ROOT)/fsd_logic/fsd_events.h $(ROOT)/fsd_logic/fsd_blackbox_summary.h $(ROOT)/fsd_logic/fsd_blackbox_filter.h $(ROOT)/fsd_logic/fsd_capability.h $(ROOT)/fsd_logic/fsd_profile_db.h $(ROOT)/fsd_logic/fsd_autonomy.h
	$(CC) $(CFLAGS) -I$(ROOT)/fsd_logic $(SRC) -o $@ -lm

# Speed-profile convergence machine. Standalone: it deliberately depends on
# nothing but its own header, so it builds without the rest of the core.
SP_SRC := test_speed_profile.c $(ROOT)/fsd_logic/fsd_speed_profile.c

test_speed_profile: $(SP_SRC) $(ROOT)/fsd_logic/fsd_speed_profile.h
	$(CC) $(CFLAGS) -I$(ROOT)/fsd_logic $(SP_SRC) -o $@

# On-module camera database + approach judgement. Reads a real camera.bin
# produced by camera-db/pack.py, so this doubles as a format compatibility
# check between the Python writer and the C reader.
CAM_SRC := test_camera.c $(ROOT)/fsd_logic/fsd_camera.c $(ROOT)/fsd_logic/fsd_cam_track.c $(ROOT)/fsd_logic/fsd_cam_policy.c $(ROOT)/fsd_logic/fsd_speed_profile.c

test_camera: $(CAM_SRC) $(ROOT)/fsd_logic/fsd_camera.h $(ROOT)/fsd_logic/fsd_cam_track.h $(ROOT)/fsd_logic/fsd_cam_policy.h
	$(CC) $(CFLAGS) -I$(ROOT)/fsd_logic $(CAM_SRC) -o $@ -lm

check: test_fsd_core test_speed_profile test_camera
	./test_fsd_core
	./test_speed_profile
	./test_camera

clean:
	rm -rf test_fsd_core test_speed_profile test_camera *.dSYM

.PHONY: check clean
