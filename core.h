// core.h — общее ядро проекта. Инклудится из каждого app-модуля.
// Содержит: внешние либы, cfg, алиасы шрифтов, extern-объявления глобалов
// и прототипы общих функций (helpers, oui, gps, ble, notif, manager).
// Определения глобалов и общих функций — в core.cpp.
#pragma once

#include <LilyGoLib.h>
#include <LV_Helper.h>
#include <arduinoFFT.h>
#include <NimBLEDevice.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <SensorPCF8563.hpp>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_mac.h>
#include <math.h>
#include <inttypes.h>

#define watch instance

// Шрифт объявлен здесь, определён в jetbrains_14.c (линкуется в скетч)
LV_FONT_DECLARE(jetbrains);
#define UI_FONT    (&jetbrains)
#define BIG_FONT   (&lv_font_montserrat_48)
#define NOTIF_FONT UI_FONT

// ─── Конфигурация ────────────────────────────────────────────────────────────
namespace cfg {
    constexpr uint32_t POMODORO_MS    = 40UL * 60 * 1000;
    constexpr uint32_t POMODORO_STEPS = 5;
    constexpr int      WORK_START     = 9 * 60 + 30;
    constexpr int      WORK_END       = 17 * 60 + 30;

    constexpr float    LPD_START_MHZ  = 433.050f;
    constexpr float    LPD_STEP_MHZ   = 0.0125f;
    constexpr int      LPD_CHANS      = 140;
    constexpr int      LPD_PEAKS      = 3;
    constexpr float    LPD_RX_BW      = 14.6f;
    constexpr int      LPD_SETTLE_US  = 1000;

    constexpr int      FFT_N          = 1024;
    constexpr float    FFT_SAMPLE_HZ  = 48000.0f;

    constexpr uint32_t SCREEN_DIM_MS   = 30000;
    constexpr uint32_t SCREEN_OFF_MS   = 40000;
    constexpr uint8_t  BRIGHTNESS_FULL = 64;
    constexpr uint8_t  BRIGHTNESS_DIM  = 20;

    constexpr char     TZ[]           = "UTC0";
    constexpr long     UTC_OFFSET_SEC = 3 * 3600;

    constexpr int      GPS_PIN_RX     = 42;
    constexpr int      GPS_PIN_TX     = 41;
    constexpr uint32_t GPS_BAUD       = 38400;
    constexpr uint32_t GPS_KEEP_MS    = 5UL * 60 * 1000;

    constexpr bool     WAKE_ON_TILT   = true;
    constexpr bool     WAKE_ON_FLIP   = true;

    constexpr char     BLE_NOTIF_NAME[] = "InfiniTime";
    constexpr bool     NOTIF_VIBRO    = true;
    constexpr bool     NOTIF_BEEP     = true;

    constexpr int      BATT_SAMPLES   = 120;
    constexpr uint32_t BATT_SAMPLE_MS = 60000;

    constexpr int      STATUSBAR_H     = 20;
    constexpr int      CONTENT_TOP     = 24;
}

// ─── Глобальное состояние (определения в core.cpp) ───────────────────────────
namespace state {
    extern volatile int      curScreen;
    extern volatile bool     scrChanged;
    extern volatile uint32_t lastActivity;
    extern bool              screenDimmed;
    extern bool              screenOff;

    extern bool     gpsActive;
    extern bool     gpsSynced;
    extern float    speedKmh;
    extern uint8_t  gpsVisible;
    extern uint8_t  gpsMaxSnr;
    extern uint32_t gpsChars;
    extern uint32_t gpsFailCsum;
    extern uint32_t gpsLastCharMs;

    extern uint8_t  gpsFix;
    extern double   gpsLat, gpsLon;
    extern float    gpsAlt;
    extern float    gpsPdop;
    extern double   distanceM;
    extern double   gpsPrevLat, gpsPrevLon;
    extern bool     gpsHasPrev;

    extern uint32_t stepCount;
    extern uint32_t pomStart;
    extern uint32_t stepsAtStart;

    extern int16_t lpdRssi[cfg::LPD_CHANS];
    extern bool    lpdDirty;

    extern volatile int wifiChannel;

    extern uint8_t       apBssid[6];
    extern char          apSsid[20];
    extern volatile bool apSelected;

    extern uint8_t  battHist[cfg::BATT_SAMPLES];
    extern int      battCount;
    extern uint32_t battLastSample;
}

// Внешнее железо
extern SFE_UBLOX_GNSS gnss;
extern bool gnssOk;

// FFT (определён в core.cpp)
extern float vReal[cfg::FFT_N];
extern float vImag[cfg::FFT_N];
extern ArduinoFFT<float> FFT;

// Фоновые задачи
extern TaskHandle_t hAudio;
extern TaskHandle_t hLPD;

// Статусбар
extern lv_obj_t *lbl_sb_left;
extern lv_obj_t *lbl_sb_mid;
extern lv_obj_t *lbl_sb_right;

#include "helpers.h"   // inline-утилиты (makeBar/heatColor/makeLabel/...)

// ─── Прототипы общих функций ─────────────────────────────────────────────────
// GPS
void gpsSyncTime();
void gpsPowerOn();
void gpsPowerOff();
void readGPS();
const char *ouiVendor(const uint8_t *mac);
// Battery
void batterySample();
// Statusbar
void buildStatusbar();
void updateStatusbar();
const char *batterySymbol(int pct);
// BLE scan
void bleInit();
void bleStart();
void bleStop();
void bleRadioSuspend();   // заглушить BLE-радио (WiFi-сниф в Recon)
void bleRadioResume();
// Notifications
void notifInit();
void notifBeep();
void notifServiceTick();
void notifPush(uint8_t cat, const char *title, const char *body);

// Менеджер приложений (Screen/RES_/screens[]/appEnter/appExit) живёт в главном
// .ino — он связан с навигацией и tileview, которые там же. Модулям он не нужен:
// они лишь определяют build()/update()/onEnter()/onExit().
