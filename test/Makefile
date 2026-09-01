# Host unit tests for the Tesla FSD protocol core.
# Compiles fsd_logic/fsd_handler.c on the host — no Flipper SDK / SPI needed.
#
#   make -C test check

CC      ?= cc
CFLAGS  ?= -std=c11 -Wall -Wextra -O0 -g
ROOT    := ..
SRC     := test_fsd_core.c $(ROOT)/fsd_logic/fsd_handler.c $(ROOT)/fsd_logic/fsd_profile.c $(ROOT)/fsd_logic/fsd_autonomy.c $(ROOT)/fsd_logic/fsd_power.c $(ROOT)/fsd_logic/fsd_owner.c $(ROOT)/fsd_logic/fsd_selftest.c

test_fsd_core: $(SRC) $(ROOT)/fsd_logic/fsd_handler.h $(ROOT)/fsd_logic/fsd_types.h $(ROOT)/fsd_logic/fsd_checksum.h $(ROOT)/fsd_logic/fsd_can_ops.h $(ROOT)/fsd_logic/fsd_state.h $(ROOT)/fsd_logic/fsd_capture.h $(ROOT)/fsd_logic/fsd_profile.h $(ROOT)/fsd_logic/fsd_events.h $(ROOT)/fsd_logic/fsd_blackbox_summary.h $(ROOT)/fsd_logic/fsd_blackbox_filter.h $(ROOT)/fsd_logic/fsd_bushealth.h $(ROOT)/fsd_logic/fsd_canquar.h $(ROOT)/fsd_logic/fsd_readcache.h $(ROOT)/fsd_logic/fsd_capability.h $(ROOT)/fsd_logic/fsd_profile_db.h $(ROOT)/fsd_logic/fsd_autonomy.h $(ROOT)/fsd_logic/fsd_power.h $(ROOT)/fsd_logic/fsd_owner.h $(ROOT)/fsd_logic/fsd_selftest.h
	$(CC) $(CFLAGS) -I$(ROOT)/fsd_logic $(SRC) -o $@ -lm

# Speed-profile convergence machine. Standalone: it deliberately depends on
# nothing but its own header, so it builds without the rest of the core.
SP_SRC := test_speed_profile.c $(ROOT)/fsd_logic/fsd_speed_profile.c

test_speed_profile: $(SP_SRC) $(ROOT)/fsd_logic/fsd_speed_profile.h
	$(CC) $(CFLAGS) -I$(ROOT)/fsd_logic $(SP_SRC) -o $@

# On-module camera database + approach judgement. Reads a real camera.bin
# produced by camera-db/pack.py, so this doubles as a format compatibility
# check between the Python writer and the C reader.
CAM_SRC := test_camera.c $(ROOT)/fsd_logic/fsd_camera.c $(ROOT)/fsd_logic/fsd_cam_track.c $(ROOT)/fsd_logic/fsd_cam_policy.c $(ROOT)/fsd_logic/fsd_speed_profile.c $(ROOT)/fsd_logic/fsd_gps.c

test_camera: $(CAM_SRC) $(ROOT)/fsd_logic/fsd_camera.h $(ROOT)/fsd_logic/fsd_cam_track.h $(ROOT)/fsd_logic/fsd_cam_policy.h $(ROOT)/fsd_logic/fsd_gps.h
	$(CC) $(CFLAGS) -I$(ROOT)/fsd_logic $(CAM_SRC) -o $@ -lm

# The car's GPS off the bus. Links fsd_camera.c for fsd_cam_distance_m(), which
# the freeze detector uses to ask whether the reported position actually moved.
GPS_SRC := test_gps.c $(ROOT)/fsd_logic/fsd_gps.c $(ROOT)/fsd_logic/fsd_camera.c

test_gps: $(GPS_SRC) $(ROOT)/fsd_logic/fsd_gps.h $(ROOT)/fsd_logic/fsd_camera.h
	$(CC) $(CFLAGS) -I$(ROOT)/fsd_logic $(GPS_SRC) -o $@ -lm

# Body control: the authority axis plus the T1/T2 detectors. Standalone — none
# of these files constructs a CAN frame, so there is nothing to link but them.
BODY_SRC := test_body.c $(ROOT)/fsd_logic/fsd_body.c $(ROOT)/fsd_logic/fsd_body_t1.c $(ROOT)/fsd_logic/fsd_body_t2.c

test_body: $(BODY_SRC) $(ROOT)/fsd_logic/fsd_body.h $(ROOT)/fsd_logic/fsd_body_t1.h $(ROOT)/fsd_logic/fsd_body_t2.h $(ROOT)/fsd_logic/fsd_autonomy.h
	$(CC) $(CFLAGS) -I$(ROOT)/fsd_logic $(BODY_SRC) -o $@

# The TX chokepoint, at bit granularity. Separate binary from test_body because
# it answers a different question: not "may this action happen" but "may this
# frame carry this action". Links fsd_body.c only for fsd_body_action_str().
#
# 🔴 NOT "WIRE_SRC": that name is already taken by test_wire further down, and
# make expands variables lazily — the later assignment wins, so this rule
# silently built test_body_wire out of test_wire.c. It reported "141 passed",
# which looked entirely normal, because test_wire has 141 tests.
#
# Only mutation testing caught it: ten deliberate holes punched in the
# chokepoint and not one of them turned a test red. Both of this repo's
# recurring traps at once — a plausible number that was measuring something
# else, and silence read as success.
BODYWIRE_SRC := test_body_wire.c $(ROOT)/fsd_logic/fsd_body_wire.c $(ROOT)/fsd_logic/fsd_body.c

test_body_wire: $(BODYWIRE_SRC) $(ROOT)/fsd_logic/fsd_body_wire.h $(ROOT)/fsd_logic/fsd_body.h $(ROOT)/fsd_logic/fsd_autonomy.h
	$(CC) $(CFLAGS) -I$(ROOT)/fsd_logic $(BODYWIRE_SRC) -o $@

# Named signals pulled out of CAN frames. The test frames are real ones, copied
# out of the 2026-09-01 capture — a test built from the table could only prove
# the table agrees with itself.
SIGNAL_SRC := test_signal.c $(ROOT)/fsd_logic/fsd_signal.c

test_signal: $(SIGNAL_SRC) $(ROOT)/fsd_logic/fsd_signal.h
	$(CC) $(CFLAGS) -I$(ROOT)/fsd_logic $(SIGNAL_SRC) -o $@

# Button press classification. Timing only — everything radio-specific is in
# esp32/.firmware/ble_central.cpp and needs a button to test.
BTN_SRC := test_button.c $(ROOT)/fsd_logic/fsd_button.c

test_button: $(BTN_SRC) $(ROOT)/fsd_logic/fsd_button.h
	$(CC) $(CFLAGS) -I$(ROOT)/fsd_logic $(BTN_SRC) -o $@

# The J6 remote's report decoder. Pure bytes-in, button-out — the vectors in
# test_btn_j6.c were captured from the remote, so this checks the decoder
# against the DEVICE rather than against my reading of it.
J6_SRC := test_btn_j6.c $(ROOT)/fsd_logic/fsd_btn_j6.c

test_btn_j6: $(J6_SRC) $(ROOT)/fsd_logic/fsd_btn_j6.h $(ROOT)/fsd_logic/fsd_button.h
	$(CC) $(CFLAGS) -I$(ROOT)/fsd_logic $(J6_SRC) -o $@

# BLE payload packing. Also GENERATES fixtures/wire_vectors.json, which the
# Android app's JVM tests parse — so the two implementations check each other
# instead of both checking the prose in BLE-GATT-프로토콜.md.
WIRE_SRC := test_wire.c $(ROOT)/fsd_logic/fsd_wire.c

test_wire: $(WIRE_SRC) $(ROOT)/fsd_logic/fsd_wire.h
	$(CC) $(CFLAGS) -I$(ROOT)/fsd_logic $(WIRE_SRC) -o $@

# The Capability document's renderer. Pure C on purpose: this document has
# outgrown the 512-byte ATT limit three times, and every time the overflow was
# found on hardware because the builder could not be exercised anywhere else.
# Now the worst case it can ever produce is measured here, on a desk.
CAPJSON_SRC := test_cap_json.c $(ROOT)/fsd_logic/fsd_cap_json.c

test_cap_json: $(CAPJSON_SRC) $(ROOT)/fsd_logic/fsd_cap_json.h
	$(CC) $(CFLAGS) -I$(ROOT)/fsd_logic $(CAPJSON_SRC) -o $@

check: test_fsd_core test_speed_profile test_camera test_gps test_body test_body_wire test_signal test_button test_btn_j6 test_wire test_cap_json
	./test_fsd_core
	./test_speed_profile
	./test_camera
	./test_gps
	./test_body
	./test_body_wire
	./test_signal
	./test_button
	./test_btn_j6
	./test_wire
	./test_cap_json

clean:
	rm -rf test_fsd_core test_speed_profile test_camera test_gps test_body test_body_wire test_signal test_button test_btn_j6 test_wire test_cap_json *.dSYM

.PHONY: check clean
