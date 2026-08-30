#include "prefs.h"
#ifdef BLE_SERVER_ENABLED
#include "ble_owner.h"
#include <NimBLEDevice.h>
#endif
#include "blackbox.h"   // BLACKBOX_DEFAULT_ENABLED
#include <Preferences.h>

static Preferences g_prefs;
static const char *NS = "fsd";

void prefs_load(FSDState *state) {
    g_prefs.begin(NS, /*readOnly=*/true);
    if (!g_prefs.isKey("ok")) {
        Serial.println("[NVS] No saved settings found (first boot)");
        g_prefs.end();
        return;
    }
    // 🔴 Not restored when the feature is compiled out. A board that ever ran a
    // build with the nag killer enabled still has "nag"=true sitting in its NVS,
    // and reading that back would raise the flag on a firmware that is supposed
    // to have no nag killer at all. The compile-time switch has to beat anything
    // the flash remembers. (fsd_handle_nag_killer() refuses regardless — this is
    // so the flag we report over BLE matches what the code will actually do.)
    state->nag_killer               = FSD_NAG_KILLER_ENABLED
                                        ? g_prefs.getBool("nag", true) : false;
    /* 🔴 켤 방법이 없는 기능은 **플래시가 기억해도 복원하지 않는다**
     * (2026-08-31 레드팀 ⑧). 바로 위 nag_killer 와 같은 규칙이다.
     *
     * `contap`(0x229 기어 스토크)과 `precond`(CAN_ID_TRIP_PLANNING)를 켤 수
     * 있었던 것은 **웹 대시보드 하나뿐**이고, 그것은 PR #28 에서 이미지에서
     * 사라졌다. 이 빌드에는 BLE 명령도 시리얼 명령도 없다 — 전수로 확인했다.
     *
     * 그런데 **NVS 키는 재플래시를 넘어 살아남는다.** WiFi 빌드를 한 번이라도
     * 돌린 보드는 이 값이 true 인 채로 부팅하고, 그러면 Active 로 올리는
     * 순간 **아무도 요청한 적 없는 프레임이 나간다.**
     *
     * 🔴 2026-08-31 에 이 표면이 넓어졌다 — `FSD_DISABLED_BUS_MASK` 를 지우면서
     * `can0` 이 살아났고, 생성 프레임의 기본 경로가 바로 그 `can0` 이다.
     *
     * 0x229 는 send_on_bus() 가 이미 무조건 거부한다. precondition 은 그런
     * 그물이 없었다 — 그래서 여기서 뿌리를 끊는다. 다른 보드(WiFi 가 살아
     * 있는)는 그대로 복원한다. */
#if defined(FSD_NO_WIFI)
    state->continuous_ap            = false;
    state->precondition             = false;
#else
    state->continuous_ap            = g_prefs.getBool("contap", false);
#endif
    state->ap_first                 = g_prefs.getBool("apfirst",false);
    state->ap_first_edge            = g_prefs.getBool("apfe",   false);
    state->ap_first_minimal         = g_prefs.getBool("apmi",   false);
    state->nag_epas_faithful        = g_prefs.getBool("nagf",   false);
    state->soft_engage              = g_prefs.getBool("soft",   false);
    state->nag_burst                = g_prefs.getBool("nagb",   false);
    state->abort_guard              = g_prefs.getBool("abortg", false);
    state->suppress_speed_chime     = g_prefs.getBool("chime",  true);
    state->ignore_ota               = g_prefs.getBool("ignota", false);
    state->fsd_unlock               = g_prefs.getBool("unlock", false);
    state->force_fsd                = g_prefs.getBool("force",  false);
    state->china_mode               = g_prefs.getBool("china",  false);
    state->tlssc_restore            = g_prefs.getBool("tlssc",  false);
#if !defined(FSD_NO_WIFI)
    state->precondition             = g_prefs.getBool("precond",false);
#endif
    state->emergency_vehicle_detect = g_prefs.getBool("emrg",   false);
    state->summon_unlock            = g_prefs.getBool("summon", false);
    state->continue_on_green        = g_prefs.getBool("cog",    false);
    state->assist_rhd_override       = g_prefs.getBool("rhd",    false);
    state->assist_telemetry_off      = g_prefs.getBool("teloff", false);
    state->apmv3_branch             = g_prefs.getUChar("apmv3", 0xFF);  // AP branch/tier selector, 0xFF = OFF
    state->bms_output               = g_prefs.getBool("bms",    false);
    state->firmware_14x_warning     = g_prefs.getBool("14x",    true);
    state->blackbox_enabled         = g_prefs.getBool("bbx",    BLACKBOX_DEFAULT_ENABLED);
    // Operator intent for unattended camera response. Persisted BECAUSE it
    // grants nothing by itself: without a gear of D and a latched belt it
    // produces no transmission at all. op_mode remains unrestored — see the
    // PERSIST_OP_MODE note below.
    state->autonomy_enabled         = g_prefs.getBool("auton",  false);
#if defined(BOARD_TTGO_DISPLAY)
    state->display_enabled          = g_prefs.getBool("disp",   true);
    state->display_brightness       = g_prefs.getUChar("disp_br", 50);
    state->display_timeout_s        = g_prefs.getUInt("disp_to",  60);
#endif
    state->sleep_idle_ms            = g_prefs.getUInt("sleep",  SLEEP_IDLE_MS);

    // WiFi
    if (g_prefs.isKey("wss")) g_prefs.getString("wss").toCharArray(state->wifi_ssid, sizeof(state->wifi_ssid));
    if (g_prefs.isKey("wsp")) g_prefs.getString("wsp").toCharArray(state->wifi_pass, sizeof(state->wifi_pass));
    state->wifi_hidden = g_prefs.getBool("wsh", false);
    if (g_prefs.isKey("stas")) g_prefs.getString("stas").toCharArray(state->wifi_sta_ssid, sizeof(state->wifi_sta_ssid));
    if (g_prefs.isKey("stap")) g_prefs.getString("stap").toCharArray(state->wifi_sta_pass, sizeof(state->wifi_sta_pass));

#if PERSIST_OP_MODE
    state->op_mode = (OpMode)g_prefs.getUChar("mode", (uint8_t)OpMode_ListenOnly);
#else
    // Boards that live permanently wired into a car boot Listen-Only no matter
    // what was saved. Otherwise the module wakes transmit-capable every time the
    // car wakes, with nobody present — see PERSIST_OP_MODE in prefs.h.
    state->op_mode = OpMode_ListenOnly;
#endif
    // Manual HW selection (#110); TeslaHW_Unknown = auto-detect.
    state->hw_override = (TeslaHWVersion)g_prefs.getUChar("hwov", (uint8_t)TeslaHW_Unknown);

    // Configurable nag-context signal mapping (#122)
    state->cfg_das_id        = g_prefs.getUShort("cdid",  0);
    state->cfg_apstate_byte  = g_prefs.getUChar("capb",   0);
    state->cfg_apstate_shift = g_prefs.getUChar("caps",   0);
    state->cfg_apstate_mask  = g_prefs.getUChar("capm",   0x0F);
    state->cfg_handson_byte  = g_prefs.getUChar("chob",   0);
    state->cfg_handson_shift = g_prefs.getUChar("chos",   0);
    state->cfg_handson_mask  = g_prefs.getUChar("chom",   0x0F);
    state->cfg_steer_id      = g_prefs.getUShort("csid",  0);
    state->cfg_steer_hi      = g_prefs.getUChar("cshi",   1);
    state->cfg_steer_lo      = g_prefs.getUChar("cslo",   0);

    /* 🔴 THIS LINE IS THE ONLY WAY TO SEE WHAT A BOARD IS CARRYING.
     *
     * Every flag below can be RESTORED from NVS but not SET: the only code that
     * ever wrote them true lived in web_dashboard.cpp, which this build does not
     * compile (PR #28). So a board flashed from a WiFi-era build can boot with
     * one of them on and there is no longer any way to turn it off — a red-team
     * finding on 2026-08-19, and the reason this line gets read before a car
     * visit.
     *
     * Precond and TLSSC were MISSING here, so two of the six could not be seen
     * at all. A diagnostic that answers two thirds of its own question sends
     * people to a multimeter for the rest. */
    Serial.printf("[NVS] Loaded: FSDUnlock=%d NAG=%d ContinuousAP=%d Precond=%d TLSSC=%d IgnoreOTA=%d China=%d Chime=%d Summon=%d COG=%d RHD=%d TelOff=%d APMv3=%d Sleep=%u AP=\"%s\" STA=\"%s\" HIDDEN=%d\n",
                  state->fsd_unlock, state->nag_killer, state->continuous_ap,
                  state->precondition, state->tlssc_restore, state->ignore_ota,
                  state->china_mode, state->suppress_speed_chime, state->summon_unlock,
                  state->continue_on_green, state->assist_rhd_override, state->assist_telemetry_off,
                  state->apmv3_branch, state->sleep_idle_ms, state->wifi_ssid, state->wifi_sta_ssid,
                  state->wifi_hidden);
    g_prefs.end();
}

void prefs_clear() {
    g_prefs.begin(NS, /*readOnly=*/false);
    g_prefs.clear();
    g_prefs.end();

    // 🔴 "All settings erased" used to be false. This namespace holds operator
    // settings; the BLE owner lives in its own ("bleowner") and the pairing keys
    // live in NimBLE's store, so a factory reset left the previous phone
    // enrolled AND bonded. Someone handing the car on, or selling the module,
    // would have had no way to know -- the log said everything was gone.
    //
    // A reset has to mean the next phone starts from nothing.
#ifdef BLE_SERVER_ENABLED
    ble_owner_erase_now();   // synchronous: a reboot follows in 200 ms
    NimBLEDevice::deleteAllBonds();
    Serial.println("[NVS] BLE owner and bonds erased too");
#endif
    Serial.println("[NVS] All settings erased — factory reset");
}

void prefs_save(const FSDState *state) {
    g_prefs.begin(NS, /*readOnly=*/false);
    g_prefs.putBool("ok",     true);
    g_prefs.putBool("nag",    state->nag_killer);
    g_prefs.putBool("contap", state->continuous_ap);
    g_prefs.putBool("apfirst",state->ap_first);
    g_prefs.putBool("apfe",   state->ap_first_edge);
    g_prefs.putBool("apmi",   state->ap_first_minimal);
    g_prefs.putBool("nagf",   state->nag_epas_faithful);
    g_prefs.putBool("soft",   state->soft_engage);
    g_prefs.putBool("nagb",   state->nag_burst);
    g_prefs.putBool("abortg", state->abort_guard);
    g_prefs.putBool("chime",  state->suppress_speed_chime);
    g_prefs.putBool("ignota", state->ignore_ota);
    g_prefs.putBool("unlock", state->fsd_unlock);
    g_prefs.putBool("force",  state->force_fsd);
    g_prefs.putBool("china",  state->china_mode);
    g_prefs.putBool("tlssc",  state->tlssc_restore);
    g_prefs.putBool("precond",state->precondition);
    g_prefs.putBool("emrg",   state->emergency_vehicle_detect);
    g_prefs.putBool("summon", state->summon_unlock);
    g_prefs.putBool("cog",    state->continue_on_green);
    g_prefs.putBool("rhd",    state->assist_rhd_override);
    g_prefs.putBool("teloff", state->assist_telemetry_off);
    g_prefs.putUChar("apmv3", state->apmv3_branch);  // AP branch/tier selector, 0xFF = OFF
    g_prefs.putBool("bms",    state->bms_output);
    g_prefs.putBool("14x",    state->firmware_14x_warning);
    g_prefs.putBool("bbx",    state->blackbox_enabled);
    g_prefs.putBool("auton",  state->autonomy_enabled);
#if defined(BOARD_TTGO_DISPLAY)
    g_prefs.putBool("disp",   state->display_enabled);
    g_prefs.putUChar("disp_br", state->display_brightness);
    g_prefs.putUInt("disp_to",  state->display_timeout_s);
#endif
    g_prefs.putUInt("sleep",  state->sleep_idle_ms);

    // WiFi
    g_prefs.putString("wss",  state->wifi_ssid);
    g_prefs.putString("wsp",  state->wifi_pass);
    g_prefs.putBool("wsh",    state->wifi_hidden);
    g_prefs.putString("stas", state->wifi_sta_ssid);
    g_prefs.putString("stap", state->wifi_sta_pass);

    g_prefs.putUChar("mode",  (uint8_t)state->op_mode);
    g_prefs.putUChar("hwov",  (uint8_t)state->hw_override);   // manual HW selection (#110)

    // Configurable nag-context signal mapping (#122)
    g_prefs.putUShort("cdid", state->cfg_das_id);
    g_prefs.putUChar("capb",  state->cfg_apstate_byte);
    g_prefs.putUChar("caps",  state->cfg_apstate_shift);
    g_prefs.putUChar("capm",  state->cfg_apstate_mask);
    g_prefs.putUChar("chob",  state->cfg_handson_byte);
    g_prefs.putUChar("chos",  state->cfg_handson_shift);
    g_prefs.putUChar("chom",  state->cfg_handson_mask);
    g_prefs.putUShort("csid", state->cfg_steer_id);
    g_prefs.putUChar("cshi",  state->cfg_steer_hi);
    g_prefs.putUChar("cslo",  state->cfg_steer_lo);

    // Same fields as the Loaded line above, and for the same reason — the two
    // are read side by side when something looks wrong.
    Serial.printf("[NVS] Saved: FSDUnlock=%d NAG=%d ContinuousAP=%d Precond=%d TLSSC=%d IgnoreOTA=%d China=%d Chime=%d Summon=%d COG=%d RHD=%d TelOff=%d APMv3=%d Sleep=%u AP=\"%s\" STA=\"%s\" HIDDEN=%d\n",
                  state->fsd_unlock, state->nag_killer, state->continuous_ap,
                  state->precondition, state->tlssc_restore, state->ignore_ota,
                  state->china_mode, state->suppress_speed_chime, state->summon_unlock,
                  state->continue_on_green, state->assist_rhd_override, state->assist_telemetry_off,
                  state->apmv3_branch, state->sleep_idle_ms, state->wifi_ssid, state->wifi_sta_ssid,
                  state->wifi_hidden);
    g_prefs.end();
}
