/**
 * T-Watch S3 Plus
 *
 * Навигация (мобильная схема):
 *   свайп влево/вправо       = экран в группе (сам tileview)
 *   свайп вверх из низа      = открыть drawer (меню групп)
 *   свайп вниз / кнопка BOOT = домой (часы), либо закрыть drawer
 *   тап слева/справа         = канал на RF-экранах
 *
 * Группы:
 *   Time   - часы
 *   Nav    - спидометр, GPS
 *   Audio  - спектр
 *   Radio  - LPD433
 *   WiFi   - разведка (drill-down), точки, клиенты, CSI, радар, deauth, pkt-rate, finder
 *   System - батарея
 *
 * Статусбар (сверху): иконки слева, батарея справа
 * Фон: помодоро-шагомер пн-пт 9:30–17:30
 *
 * Добавление экрана: namespace c build()/update() (+ опц. onEnter/onExit),
 * затем строка в массиве screens[].
 *
 * Arduino IDE:
 *   Board: LilyGo T-Watch-S3,  Rev: SX1262,  USB CDC: Enabled
 * Доп. либа: arduinoFFT (kosme)
 */

#ifndef ARDUINO_T_WATCH_S3
#define ARDUINO_T_WATCH_S3
#endif
#ifndef ARDUINO_LILYGO_LORA_SX1262
#define ARDUINO_LILYGO_LORA_SX1262
#endif

#include <LilyGoLib.h>
#include <LV_Helper.h>
#include <arduinoFFT.h>
#include <NimBLEDevice.h>   // доп. либа: NimBLE-Arduino (h2zero), API 2.x
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>   // UBX вместо NMEA — стабильнее
#include <SensorPCF8563.hpp>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_mac.h>
#include <math.h>
#include <inttypes.h>   // для PRIu32

#define watch instance

// ─── Кириллический шрифт ──────────────────────────────────────────
// Для отображения кириллики в уведомлениях нужен custom шрифт.
// Сгенерируйте: https://lvgl.io/tools/fontconverter
//   Font: любой TTF с кириллицей (напр. Roboto-Regular.ttf)
//   Size: 14, Bpp: 1, Output: font_cyrillic_14.c
//   Range: 0x0020-0x007E,0x0400-0x04FF (ASCII + кирилл.)
// Поместите font_cyrillic_14.c рядом с .ino и раскомментируйте:
#include "jetbrains_14.c"
LV_FONT_DECLARE(jetbrains);
// Единые алиасы шрифтов. Мелкий текст — jetbrains (кириллица+иконки).
// Крупный — пока montserrat_48 (цифры/иконки, кириллица там не нужна);
// сменишь на &jetbrains_48, когда сгенеришь 48px-версию.
#define UI_FONT  (&jetbrains)
#define BIG_FONT (&lv_font_montserrat_48)
#define NOTIF_FONT UI_FONT

SFE_UBLOX_GNSS gnss;
static bool gnssOk = false;

// ─── Конфигурация ──────────────────────────────────────────────────────────
namespace cfg {
    constexpr uint32_t POMODORO_MS    = 40UL * 60 * 1000;
    constexpr uint32_t POMODORO_STEPS = 5;
    constexpr int      WORK_START     = 9 * 60 + 30;
    constexpr int      WORK_END       = 17 * 60 + 30;

    constexpr float    LPD_START_MHZ  = 433.050f;
    constexpr float    LPD_STEP_MHZ   = 0.0125f;  // 12.5кГц: перекрытие с BW, без дыр охвата
    constexpr int      LPD_CHANS      = 140;      // охват 433.05–434.79 (весь LPD433)
    constexpr int      LPD_PEAKS      = 3;        // показываемых пиков
    constexpr float    LPD_RX_BW      = 14.6f;    // уже = ниже пол шума, шире = охват/канал
    constexpr int      LPD_SETTLE_US  = 1000;     // оседание RX/AGC; ниже ~700 RSSI плющит в пол

    constexpr int      FFT_N          = 1024;
    constexpr float    FFT_SAMPLE_HZ  = 48000.0f;  // Найквист 24кГц — ловим УЗ выше слуха

    constexpr uint32_t SCREEN_DIM_MS   = 30000;
    constexpr uint32_t SCREEN_OFF_MS   = 40000;
    constexpr uint8_t  BRIGHTNESS_FULL = 64;
    constexpr uint8_t  BRIGHTNESS_DIM  = 20;

    // Время храним местное и в RTC, и в системе; дисплей читает как есть (UTC0).
    // Смещение нужно только для перевода UTC от GPS в местное.
    constexpr char     TZ[]           = "UTC0";
    constexpr long     UTC_OFFSET_SEC = 3 * 3600;   // Москва UTC+3; меняй под себя

    constexpr int      GPS_PIN_RX     = 42;       // RX чипа GNSS (UART)
    constexpr int      GPS_PIN_TX     = 41;       // TX чипа GNSS
    constexpr uint32_t GPS_BAUD       = 38400;    // дефолт M10; если begin падает — попробуй 9600

    constexpr uint32_t GPS_KEEP_MS    = 5UL * 60 * 1000;  // не гасим GPS сразу — даём дочекать фикс

    constexpr bool     WAKE_ON_TILT   = true;   // встроенный детектор поднятия руки (BMA423 FEATURE_TILT)
    constexpr bool     WAKE_ON_FLIP   = true;   // переворот кисти «туда-обратно» (deliberate-жест)

    constexpr char     BLE_NOTIF_NAME[] = "InfiniTime";   // имя для распознавания Gadgetbridge
    constexpr bool     NOTIF_VIBRO    = true;   // вибро на приход уведомления
    constexpr bool     NOTIF_BEEP     = true;   // звуковой бип на приход

    constexpr int      CSI_MAX_SUBC   = 64;        // поднесущих (HT20)
    constexpr int      CSI_CHANNEL    = 1;         // канал для прослушки
    constexpr int      CSI_BARS       = 56;        // визуальных баров спектра

    constexpr int      BATT_SAMPLES   = 120;       // точек истории заряда
    constexpr uint32_t BATT_SAMPLE_MS = 60000;     // интервал выборки (1 мин)

    constexpr int      STATUSBAR_H     = 20;
    constexpr int      CONTENT_TOP     = 24;       // отступ под статусбаром
}

#include "helpers.h"   // makeBar/heatColor/makeLabel/isWorkTime/nmeaField/haversineM

// ─── Глобальное состояние ───────────────────────────────────────────────────
namespace state {
    volatile int      curScreen    = 0;
    volatile bool     scrChanged   = false;
    volatile uint32_t lastActivity = 0;
    bool              screenDimmed = false;
    bool              screenOff    = false;

    bool     gpsActive     = false;  // питание GPS включено (на экранах 1/4)
    bool     gpsSynced     = false;
    float    speedKmh      = 0.0f;
    uint8_t  gpsVisible    = 0;      // = SIV (спутников в решении)
    uint8_t  gpsMaxSnr     = 0;      // (UBX PVT не даёт, оставлено 0)
    uint32_t gpsChars      = 0;
    uint32_t gpsFailCsum   = 0;
    uint32_t gpsLastCharMs = 0;

    uint8_t  gpsFix        = 0;      // 0 нет, 2 2D, 3 3D
    double   gpsLat        = 0, gpsLon = 0;
    float    gpsAlt        = 0;      // м (MSL)
    float    gpsPdop       = 99.9f;
    double   distanceM     = 0;      // накопленная дистанция по трекам, м
    double   gpsPrevLat    = 0, gpsPrevLon = 0;
    bool     gpsHasPrev    = false;

    uint32_t stepCount    = 0;
    uint32_t pomStart     = 0;
    uint32_t stepsAtStart = 0;

    int16_t lpdRssi[cfg::LPD_CHANS];
    bool   lpdDirty = false;

    // CSI анализатор канала
    volatile bool    csiReady = false;          // новый кадр готов
    int8_t   csiRaw[cfg::CSI_MAX_SUBC * 2];      // сырые real/imag из callback
    int      csiLen     = 0;                     // число валидных int8 в csiRaw
    int8_t   csiRssi    = 0;                     // RSSI последнего пакета
    float    csiAmp[cfg::CSI_MAX_SUBC];          // амплитуды поднесущих
    float    csiPrevAmp[cfg::CSI_MAX_SUBC];      // предыдущий кадр (для motion)
    int      csiSubc    = 0;                     // число поднесущих
    float    csiFlatness = 0;                    // 0..100%
    float    csiMotion   = 0;                    // 0..100% (air disturbance)
    uint32_t csiPackets  = 0;                    // packet counter
    int      csiChannel  = cfg::CSI_CHANNEL;     // current listen channel 1-13
    int      csiChanRequest = 0;                  // 0 = no change, else target channel
    volatile int wifiChannel = cfg::CSI_CHANNEL;  // общий канал WiFi-экранов (источник истины)

    // Выбранная точка-цель: инструменты считают трафик к/от неё, а не по каналу
    uint8_t       apBssid[6] = {0};
    char          apSsid[20] = "";
    volatile bool apSelected = false;

    // История заряда батареи (кольцевой буфер)
    uint8_t  battHist[cfg::BATT_SAMPLES];
    int      battCount      = 0;                  // заполнено точек
    uint32_t battLastSample = 0;
}

// ─── FFT ────────────────────────────────────────────────────────────────────
static float vReal[cfg::FFT_N];
static float vImag[cfg::FFT_N];
ArduinoFFT<float> FFT(vReal, vImag, cfg::FFT_N, cfg::FFT_SAMPLE_HZ);

// ─── Хэндлы фоновых задач ────────────────────────────────────────────────────
static TaskHandle_t hAudio = NULL;
static TaskHandle_t hLPD   = NULL;

// ─── Статусбар ────────────────────────────────────────────────────────────────
static lv_obj_t *lbl_sb_left;   // иконки слева
static lv_obj_t *lbl_sb_mid;    // часы по центру
static lv_obj_t *lbl_sb_right;  // батарея справа


// ════════════════════════════════════════════════════════════════════════════
//  GPS
// ════════════════════════════════════════════════════════════════════════════

static uint8_t accVisible = 0;
static uint8_t accMaxSnr  = 0;

static void parseGSV(const char *line)
{
    char buf[8];
    int msgNum = 0;
    if (nmeaField(line, 2, buf, sizeof(buf)) > 0) msgNum = atoi(buf);
    if (msgNum == 1 && nmeaField(line, 3, buf, sizeof(buf)) > 0)
        accVisible += (uint8_t)atoi(buf);
    for (int sat = 0; sat < 4; sat++) {
        if (nmeaField(line, 7 + sat * 4, buf, sizeof(buf)) > 0) {
            int snr = atoi(buf);
            if (snr > accMaxSnr) accMaxSnr = snr;
        }
    }
}

static void gpsSyncTime()
{
    if (state::gpsSynced) return;
    if (!gnss.getTimeValid() || !gnss.getDateValid()) return;
    int year = gnss.getYear();
    if (year <= 2020) return;

    // GPS даёт UTC -> местное добавлением смещения. Дальше RTC, система и
    // дисплей живут в местном времени (TZ=UTC0), без двойных пересчётов.
    struct tm u = {};
    u.tm_year = year - 1900; u.tm_mon = gnss.getMonth() - 1; u.tm_mday = gnss.getDay();
    u.tm_hour = gnss.getHour(); u.tm_min = gnss.getMinute(); u.tm_sec = gnss.getSecond();
    u.tm_isdst = 0;
    time_t local = mktime(&u) + cfg::UTC_OFFSET_SEC;   // TZ=UTC0 -> mktime трактует поля как UTC

    struct tm lt; gmtime_r(&local, &lt);               // поля местного времени
    watch.rtc.setDateTime(lt.tm_year + 1900, lt.tm_mon + 1, lt.tm_mday,
                          lt.tm_hour, lt.tm_min, lt.tm_sec);
    struct timeval tv = { .tv_sec = local };
    settimeofday(&tv, NULL);
    state::gpsSynced = true;
    Serial.println("GPS->RTC+SYS synced (local)");
}

static void gpsPowerOn()
{
    if (state::gpsActive) return;
    watch.powerControl(POWER_GPS, true);   // питание модуля (BLDO1)
    delay(50);
    Serial1.setRxBufferSize(1024);         // запас под UBX-пакеты
    Serial1.begin(cfg::GPS_BAUD, SERIAL_8N1, cfg::GPS_PIN_RX, cfg::GPS_PIN_TX);
    gnssOk = gnss.begin(Serial1);
    if (gnssOk) {
        gnss.setUART1Output(COM_TYPE_UBX);   // только UBX — никакого NMEA-шума
        gnss.setNavigationFrequency(4);      // 4 Гц
        gnss.setAutoPVT(true);               // модуль сам шлёт PVT, getPVT() не блокирует
    }
    state::gpsActive = true;
    Serial.printf("GPS power ON, gnss=%d\n", gnssOk);
}

static void gpsPowerOff()
{
    if (!state::gpsActive) return;
    Serial1.end();
    watch.powerControl(POWER_GPS, false);
    gnssOk = false;
    state::gpsActive = false;
    state::gpsFix = 0;
    state::gpsVisible = 0;
    state::gpsHasPrev = false;             // следующий заход — новый трек
    Serial.println("GPS power OFF");
}

// Читаем GPS только когда питание включено (экраны 1/4)
static void readGPS()
{
    if (!state::gpsActive || !gnssOk) return;
    static uint32_t last = 0;
    if (millis() - last < 250) return;
    last = millis();

    if (!gnss.getPVT()) return;            // нет свежего решения

    uint8_t fix = gnss.getFixType();
    state::gpsFix     = fix;
    state::gpsVisible = gnss.getSIV();
    state::gpsPdop    = gnss.getPDOP() / 100.0f;

    if (fix >= 2) {
        double lat = gnss.getLatitude()  * 1e-7;
        double lon = gnss.getLongitude() * 1e-7;
        state::gpsLat   = lat;
        state::gpsLon   = lon;
        state::gpsAlt   = gnss.getAltitudeMSL() / 1000.0f;     // мм -> м
        state::speedKmh = gnss.getGroundSpeed() * 0.0036f;     // мм/с -> км/ч

        if (state::gpsHasPrev) {
            double d = haversineM(state::gpsPrevLat, state::gpsPrevLon, lat, lon);
            if (d > 1.5 && d < 200.0)      // фильтр джиттера и выбросов
                state::distanceM += d;
        }
        state::gpsPrevLat = lat;
        state::gpsPrevLon = lon;
        state::gpsHasPrev = true;
    }
    gpsSyncTime();
}

// ════════════════════════════════════════════════════════════════════════════
//  CSI — анализ состояния радиоканала (Channel State Information)
// ════════════════════════════════════════════════════════════════════════════

// Callback из контекста WiFi-драйвера. Только копируем, без тяжёлой обработки.
// Без IRAM_ATTR: memcpy может быть не в IRAM, что вызывает краш при cache miss.
static void csiCallback(void *ctx, wifi_csi_info_t *info)
{
    if (!info || !info->buf) return;
    if (info->len <= 0) return;
    if (state::csiReady) return;  // прошлый кадр не обработан — пропускаем

    int len = info->len;
    if (len > cfg::CSI_MAX_SUBC * 2) len = cfg::CSI_MAX_SUBC * 2;
    if (len < 2) return;

    memcpy(state::csiRaw, info->buf, len);
    state::csiLen   = len;
    state::csiRssi  = info->rx_ctrl.rssi;
    __sync_synchronize();         // барьер: данные записаны до выставления флага
    state::csiReady = true;
}

static void csiStart()
{
    state::csiChannel = state::wifiChannel;   // берём общий канал
    // Чистим состояние прошлой сессии чтобы не залипал старый спектр
    for (int i = 0; i < cfg::CSI_MAX_SUBC; i++) {
        state::csiAmp[i]     = 0;
        state::csiPrevAmp[i] = 0;
    }
    state::csiSubc    = 0;
    state::csiMotion  = 0;
    state::csiPackets = 0;
    state::csiReady   = false;

    // Минимальная конфигурация: только LLTF (64 поднесущих = 128 байт).
    // Включение htltf/stbc/ltf_merge даёт большой буфер и грузит драйвер —
    // источник нестабильности и порчи кучи. Оставляем самое простое.
    wifi_csi_config_t csi_config = {};
    csi_config.lltf_en         = true;
    csi_config.htltf_en        = false;
    csi_config.stbc_htltf2_en  = false;
    csi_config.ltf_merge_en    = false;
    csi_config.channel_filter_en = true;
    csi_config.manu_scale      = false;
    csi_config.shift           = 0;

    esp_wifi_set_csi_config(&csi_config);
    esp_wifi_set_csi_rx_cb(csiCallback, NULL);
    esp_wifi_set_csi(true);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(state::csiChannel, WIFI_SECOND_CHAN_NONE);
    Serial.printf("CSI started ch%d\n", state::csiChannel);
}

// Запрос смены канала — фактическое переключение в csiProcess (вне LVGL контекста)
static uint32_t csiLastChanChange = 0;

static void csiRequestChannel(int ch)
{
    // Дебаунс: не чаще раза в 400мс, иначе драйвер WiFi не успевает
    if (millis() - csiLastChanChange < 400) return;
    if (ch < 1) ch = 13;
    if (ch > 13) ch = 1;
    state::csiChanRequest = ch;
}

static void csiApplyChannel()
{
    int ch = state::csiChanRequest;
    if (ch == 0) return;
    state::csiChanRequest = 0;
    csiLastChanChange = millis();

    // Полная тишина на время смены канала: снимаем колбэк, гасим CSI и
    // промискуитет, меняем канал, поднимаем обратно. Иначе колбэк в полёте
    // пересекается с переконфигурацией драйвера и рушит кучу.
    esp_wifi_set_csi(false);
    esp_wifi_set_csi_rx_cb(NULL, NULL);
    state::csiReady = false;
    esp_wifi_set_promiscuous(false);

    esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);

    esp_wifi_set_promiscuous(true);
    esp_wifi_set_csi_rx_cb(csiCallback, NULL);
    esp_wifi_set_csi(true);

    state::csiChannel = ch;
    state::wifiChannel = ch;
    state::csiPackets = 0;
    state::csiSubc = 0;
    for (int i = 0; i < cfg::CSI_MAX_SUBC; i++) {
        state::csiAmp[i] = 0;
        state::csiPrevAmp[i] = 0;
    }
    Serial.printf("CSI channel -> %d\n", ch);
}

static void csiStop()
{
    esp_wifi_set_csi(false);
    esp_wifi_set_csi_rx_cb(NULL, NULL);   // снимаем колбэк полностью
    esp_wifi_set_promiscuous(false);
    state::csiReady = false;
    Serial.println("CSI stopped");
}
// Валидные поднесущие LLTF (HT20): DC и guard 27..37 — null, исключаем.
// Порядок частот: -26..-1 (бины 38..63), затем +1..+26 (бины 1..26).
static const int8_t CSI_LLTF_MAP[52] = {
    38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,
     1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26
};

static void csiProcess()
{
    csiApplyChannel();
    if (!state::csiReady) return;

    static int8_t local[cfg::CSI_MAX_SUBC * 2];
    int len = state::csiLen;
    if (len < 2) { state::csiReady = false; return; }
    if (len > cfg::CSI_MAX_SUBC * 2) len = cfg::CSI_MAX_SUBC * 2;
    memcpy(local, state::csiRaw, len);
    int8_t rssi = state::csiRssi;
    __sync_synchronize();
    state::csiReady = false;

    int rawSubc = len / 2;

    // Прошлый кадр для motion (до перезаписи csiAmp)
    for (int i = 0; i < state::csiSubc; i++)
        state::csiPrevAmp[i] = state::csiAmp[i];

    // Амплитуды только по валидным поднесущим, в порядке частот.
    int   subc;
    float sum = 0, sumSq = 0;
    if (rawSubc >= 64) {
        subc = 52;
        for (int i = 0; i < 52; i++) {
            int b = CSI_LLTF_MAP[i];
            float re = local[2 * b], im = local[2 * b + 1];
            float amp = sqrtf(re * re + im * im);
            state::csiAmp[i] = amp;
            sum += amp; sumSq += amp * amp;
        }
    } else {                          // нестандартный кадр — линейно, как было
        subc = rawSubc > cfg::CSI_MAX_SUBC ? cfg::CSI_MAX_SUBC : rawSubc;
        for (int i = 0; i < subc; i++) {
            float re = local[2 * i], im = local[2 * i + 1];
            float amp = sqrtf(re * re + im * im);
            state::csiAmp[i] = amp;
            sum += amp; sumSq += amp * amp;
        }
    }
    if (subc < 1) return;
    state::csiSubc = subc;
    state::csiRssi = rssi;

    float mean = sum / subc;

    // Flatness: 1 - CV. Высокая = плоский канал (LOS), низкая = многолучёвка.
    float variance = sumSq / subc - mean * mean;
    if (variance < 0) variance = 0;
    float cv = (mean > 0.1f) ? sqrtf(variance) / mean : 1.0f;
    state::csiFlatness = constrain((1.0f - cv) * 100.0f, 0.0f, 100.0f);

    // Motion: средняя относит. дельта на поднесущую. Нормировка по mean
    // убирает разницу мощности между пакетами от разных передатчиков.
    float meanPrev = 0;
    for (int i = 0; i < subc; i++) meanPrev += state::csiPrevAmp[i];
    meanPrev /= subc;

    float diff = 0;
    if (mean > 0.1f && meanPrev > 0.1f)
        for (int i = 0; i < subc; i++)
            diff += fabsf(state::csiAmp[i] / mean - state::csiPrevAmp[i] / meanPrev);
    float motionPct = diff / subc * 100.0f;
    state::csiMotion = state::csiMotion * 0.7f + motionPct * 0.3f;

    state::csiPackets++;
}

// ════════════════════════════════════════════════════════════════════════════
//  БАТАРЕЯ — выборка истории заряда (вызывается из loop, на всех экранах)
// ════════════════════════════════════════════════════════════════════════════

static void batterySample()
{
    if (state::battCount > 0
        && millis() - state::battLastSample < cfg::BATT_SAMPLE_MS) return;
    state::battLastSample = millis();

    int pct = watch.pmu.getBatteryPercent();
    if (pct < 0 || pct > 100) return;   // нет данных/батареи

    if (state::battCount < cfg::BATT_SAMPLES) {
        state::battHist[state::battCount++] = (uint8_t)pct;
    } else {
        memmove(state::battHist, state::battHist + 1,
                (cfg::BATT_SAMPLES - 1) * sizeof(uint8_t));
        state::battHist[cfg::BATT_SAMPLES - 1] = (uint8_t)pct;
    }
}

// ════════════════════════════════════════════════════════════════════════════
//  WIFI СНИФФЕР — промискуитетный приём, разбор 802.11, таблица MAC, счётчики
// ════════════════════════════════════════════════════════════════════════════

// Полная база вендоров по OUI (24-бит префикс), сгенерирована из IEEE/nmap.
// Отсортирована по возрастанию -> бинарный поиск. См. oui_db.h рядом с .ino.
#include "oui_db.h"

static const char *ouiVendor(const uint8_t *mac)
{
    if (!(mac[0] & 0x02)) {                // не рандомный — бинарный поиск по OUI
        uint32_t key = ((uint32_t)mac[0] << 16) | (mac[1] << 8) | mac[2];
        int lo = 0, hi = OUI_DB_COUNT - 1;
        while (lo <= hi) {
            int m = (lo + hi) >> 1;
            const OuiEntry &e = OUI_DB[m];
            uint32_t v = ((uint32_t)e.p[0] << 16) | (e.p[1] << 8) | e.p[2];
            if (v == key) return e.name;
            if (v <  key) lo = m + 1; else hi = m - 1;
        }
    }
    // нет в базе — показываем хвост MAC; рандомные адреса метим '~'
    static char buf[12];
    snprintf(buf, sizeof(buf), "%s%02X%02X%02X",
             (mac[0] & 0x02) ? "~" : "", mac[3], mac[4], mac[5]);
    return buf;
}

// ── Счётчики типов фреймов (обновляются в колбэке) ──
namespace sniff {
    volatile uint32_t cntTotal = 0, cntMgmt = 0, cntCtrl = 0, cntData = 0;
    volatile uint32_t cntProbe = 0, cntBeacon = 0, cntDeauth = 0, cntDisassoc = 0;
    volatile uint32_t apFrom = 0, apTo = 0, apDeauth = 0;   // трафик выбранной точки

    // Таблица недавно виденных устройств
    struct MacEntry {
        uint8_t  mac[6];
        int8_t   rssi;
        uint8_t  ch;
        bool     isRandom;     // рандомный (локально-админский) MAC
        uint32_t lastSeen;
    };
    static const int TABLE_SIZE = 128;
    MacEntry table[TABLE_SIZE];
    volatile int  count = 0;
    portMUX_TYPE  mux = portMUX_INITIALIZER_UNLOCKED;

    bool     hopping  = false;
    bool     started  = false;   // guard от двойного esp_wifi_start (вызов из неск. экранов)
    int      channel  = 1;
    uint32_t hopLast  = 0;
    int      chanReq  = 0;   // запрос смены канала (применяется в loop)

    // Клиенты выбранной точки (заполняется при apSelected)
    struct ClientEntry { uint8_t mac[6]; int8_t rssi; uint32_t lastSeen; };
    static const int CLIENTS_MAX = 32;
    ClientEntry  clients[CLIENTS_MAX];
    volatile int clientCount = 0;
}

// Добавить/обновить MAC в таблице (вызывается из колбэка, под спинлоком)
static void sniffTouchMac(const uint8_t *mac, int8_t rssi, uint8_t ch)
{
    bool isRand = (mac[0] & 0x02);   // локально-админский = рандомный

    portENTER_CRITICAL(&sniff::mux);
    int found = -1;
    int oldest = 0;          uint32_t oldestT = 0xFFFFFFFF;
    int oldestRand = -1;     uint32_t oldestRandT = 0xFFFFFFFF;
    for (int i = 0; i < sniff::count; i++) {
        if (memcmp(sniff::table[i].mac, mac, 6) == 0) { found = i; break; }
        if (sniff::table[i].lastSeen < oldestT) {
            oldestT = sniff::table[i].lastSeen; oldest = i;
        }
        if (sniff::table[i].isRandom && sniff::table[i].lastSeen < oldestRandT) {
            oldestRandT = sniff::table[i].lastSeen; oldestRand = i;
        }
    }
    int idx;
    if (found >= 0)                            idx = found;
    else if (sniff::count < sniff::TABLE_SIZE) idx = sniff::count++;
    else if (oldestRand >= 0)                  idx = oldestRand; // рандомные вытесняем первыми
    else                                       idx = oldest;

    memcpy(sniff::table[idx].mac, mac, 6);
    sniff::table[idx].rssi     = rssi;
    sniff::table[idx].ch       = ch;
    sniff::table[idx].isRandom = isRand;
    sniff::table[idx].lastSeen = millis();
    portEXIT_CRITICAL(&sniff::mux);
}

// Учёт клиента точки (вызывается из колбэка, под спинлоком)
static void sniffTouchClient(const uint8_t *mac, int8_t rssi)
{
    portENTER_CRITICAL(&sniff::mux);
    int found = -1, oldest = 0;
    uint32_t oldestT = 0xFFFFFFFF;
    for (int i = 0; i < sniff::clientCount; i++) {
        if (memcmp(sniff::clients[i].mac, mac, 6) == 0) { found = i; break; }
        if (sniff::clients[i].lastSeen < oldestT) { oldestT = sniff::clients[i].lastSeen; oldest = i; }
    }
    int idx;
    if (found >= 0)                                idx = found;
    else if (sniff::clientCount < sniff::CLIENTS_MAX) idx = sniff::clientCount++;
    else                                           idx = oldest;
    memcpy(sniff::clients[idx].mac, mac, 6);
    sniff::clients[idx].rssi     = rssi;
    sniff::clients[idx].lastSeen = millis();
    portEXIT_CRITICAL(&sniff::mux);
}

// Промискуитетный колбэк — вызывается из WiFi-задачи (не ISR)
static void snifferCb(void *buf, wifi_promiscuous_pkt_type_t type)
{
    wifi_promiscuous_pkt_t *p = (wifi_promiscuous_pkt_t *)buf;
    if (!p) return;
    int len = p->rx_ctrl.sig_len;
    if (len < 16) return;                    // мало для разбора заголовка
    const uint8_t *fr = p->payload;

    uint8_t ftype    = (fr[0] >> 2) & 0x03;  // 0=mgmt 1=ctrl 2=data
    uint8_t fsubtype = (fr[0] >> 4) & 0x0F;

    sniff::cntTotal++;
    if (ftype == 0) {
        sniff::cntMgmt++;
        if      (fsubtype == 4)  sniff::cntProbe++;     // probe request
        else if (fsubtype == 8)  sniff::cntBeacon++;    // beacon
        else if (fsubtype == 12) sniff::cntDeauth++;    // deauth
        else if (fsubtype == 10) sniff::cntDisassoc++;  // disassoc
    } else if (ftype == 1) {
        sniff::cntCtrl++;
    } else if (ftype == 2) {
        sniff::cntData++;
    }

    // addr2 (источник) — смещение 10 для mgmt/data фреймов
    if (ftype == 0 || ftype == 2)
        sniffTouchMac(fr + 10, p->rx_ctrl.rssi, p->rx_ctrl.channel);

    // Счёт трафика выбранной точки: addr1=получатель, addr2=отправитель
    if (state::apSelected) {
        const uint8_t *b = state::apBssid;
        bool from = (memcmp(fr + 10, b, 6) == 0);   // от точки (downlink)
        bool to   = (memcmp(fr + 4,  b, 6) == 0);   // к точке (uplink)
        if (from) sniff::apFrom++;
        if (to)   sniff::apTo++;
        if (ftype == 0 && (fsubtype == 12 || fsubtype == 10) && (from || to))
            sniff::apDeauth++;

        // Клиент точки из data-фреймов по DS-битам (addr1=получатель, addr2=отправитель)
        if (ftype == 2) {
            uint8_t ds = fr[1] & 0x03;          // bit0=ToDS, bit1=FromDS
            const uint8_t *cli = nullptr;
            if      (ds == 0x01 && to)   cli = fr + 10;  // client->AP: client=addr2
            else if (ds == 0x02 && from) cli = fr + 4;   // AP->client: client=addr1
            if (cli && !(cli[0] & 0x01) && memcmp(cli, b, 6) != 0)
                sniffTouchClient(cli, p->rx_ctrl.rssi);
        }
    }
}

static void snifferResetCounters()
{
    sniff::cntTotal = sniff::cntMgmt = sniff::cntCtrl = sniff::cntData = 0;
    sniff::cntProbe = sniff::cntBeacon = sniff::cntDeauth = sniff::cntDisassoc = 0;
    sniff::apFrom = sniff::apTo = sniff::apDeauth = 0;
    portENTER_CRITICAL(&sniff::mux);
    sniff::count = 0;
    sniff::clientCount = 0;
    portEXIT_CRITICAL(&sniff::mux);
}

static void snifferStart(bool hop, int fixedCh = 1)
{
    if (sniff::started) return;   // idempotent: повторный esp_wifi_start ломает драйвер
    snifferResetCounters();
    sniff::hopping = hop;
    sniff::channel = fixedCh;
    sniff::hopLast = millis();

    esp_wifi_start();
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(snifferCb);
    wifi_promiscuous_filter_t filt = { .filter_mask = WIFI_PROMIS_FILTER_MASK_ALL };
    esp_wifi_set_promiscuous_filter(&filt);
    esp_wifi_set_channel(sniff::channel, WIFI_SECOND_CHAN_NONE);
    sniff::started = true;
}

static void snifferStop()
{
    if (!sniff::started) return;
    esp_wifi_set_promiscuous(false);
    sniff::hopping = false;
    esp_wifi_stop();
    sniff::started = false;
}

// Прыжки по каналам — вызывается из loop (безопасный контекст)
static void snifferHopTick()
{
    if (!sniff::hopping) return;
    if (millis() - sniff::hopLast < 250) return;   // ~250мс на канал
    sniff::hopLast = millis();
    int ch = sniff::channel + 1;
    if (ch > 13) ch = 1;
    sniff::channel = ch;
    esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
}

// Применить запрос смены канала (из loop — безопасный контекст)
static void snifferApplyChanReq()
{
    if (!sniff::chanReq) return;
    int ch = sniff::chanReq;
    sniff::chanReq = 0;
    sniff::channel = ch;
    state::wifiChannel = ch;
    esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
}

// Подсчёт устройств за окно: real = с настоящим OUI, rand = рандомные MAC
static void sniffDeviceCount(uint32_t windowMs, int *real, int *rnd)
{
    uint32_t now = millis();
    int r = 0, rd = 0;
    portENTER_CRITICAL(&sniff::mux);
    for (int i = 0; i < sniff::count; i++) {
        if (now - sniff::table[i].lastSeen > windowMs) continue;
        if (sniff::table[i].isRandom) rd++;
        else                          r++;
    }
    portEXIT_CRITICAL(&sniff::mux);
    *real = r;
    *rnd  = rd;
}


// Данные уведомлений/Gadgetbridge — рано, до статусбара (он читает connected/unread).
namespace notif {
    struct Item { char title[28]; char body[80]; uint32_t when; uint8_t cat; };
    static const int MAX = 16;
    Item          items[MAX];
    volatile int  count = 0, head = 0, unread = 0;
    volatile bool connected = false;
    volatile bool pushNow = false;            // форс отдачи батареи/шагов при коннекте
    volatile bool arrived = false;            // флаг для loop: побудка + вибро
    portMUX_TYPE  mux = portMUX_INITIALIZER_UNLOCKED;

    // Сервисы Gadgetbridge
    NimBLECharacteristic *chBatt = nullptr;   // battery level (notify)
    NimBLECharacteristic *chStep = nullptr;   // step count (notify)
    volatile bool     timeSet = false;        // запрос установки времени из CTS
    volatile uint16_t tYear = 0;
    volatile uint8_t  tMon = 0, tDay = 0, tHour = 0, tMin = 0, tSec = 0;
}

static void buildStatusbar()
{
    lv_obj_t *bg = lv_obj_create(lv_layer_top());
    lv_obj_set_size(bg, LV_HOR_RES, cfg::STATUSBAR_H);
    lv_obj_set_pos(bg, 0, 0);
    lv_obj_set_style_bg_color(bg, lv_color_hex(0x111111), 0);
    lv_obj_set_style_border_width(bg, 0, 0);
    lv_obj_set_style_radius(bg, 0, 0);
    lv_obj_set_style_pad_all(bg, 0, 0);
    lv_obj_clear_flag(bg, LV_OBJ_FLAG_CLICKABLE);

    // Слева: статус-иконки (BT, уведомления, шаги)
    lbl_sb_left = makeLabel(bg, UI_FONT, 0xAAAAAA,
                            LV_ALIGN_LEFT_MID, 6, 0);
    lv_label_set_text(lbl_sb_left, LV_SYMBOL_LOOP " 0");

    // По центру: часы — видны на любом экране
    lbl_sb_mid = makeLabel(bg, UI_FONT, 0xFFFFFF,
                           LV_ALIGN_CENTER, 0, 0);
    lv_label_set_text(lbl_sb_mid, "--:--");

    // Справа: батарея
    lbl_sb_right = makeLabel(bg, UI_FONT, 0xAAAAAA,
                             LV_ALIGN_RIGHT_MID, -6, 0);
    lv_label_set_text(lbl_sb_right, LV_SYMBOL_BATTERY_FULL " 0%");
}

static const char *batterySymbol(int pct)
{
    if (watch.pmu.isCharging())   return LV_SYMBOL_CHARGE;
    if (pct > 80) return LV_SYMBOL_BATTERY_FULL;
    if (pct > 55) return LV_SYMBOL_BATTERY_3;
    if (pct > 30) return LV_SYMBOL_BATTERY_2;
    if (pct > 10) return LV_SYMBOL_BATTERY_1;
    return LV_SYMBOL_BATTERY_EMPTY;
}

static void updateStatusbar()
{
    static uint32_t last = 0;
    if (millis() - last < 2000) return;
    last = millis();

    char buf[80];

    // Слева: BT (если подключён) + непрочитанные уведомления + шаги
    int unread;
    portENTER_CRITICAL(&notif::mux);
    unread = notif::unread;
    portEXIT_CRITICAL(&notif::mux);
    int o = 0;
    if (notif::connected)
        o += snprintf(buf + o, sizeof(buf) - o, LV_SYMBOL_BLUETOOTH " ");
    if (unread > 0)
        o += snprintf(buf + o, sizeof(buf) - o, LV_SYMBOL_BELL "%d ", unread);
    o += snprintf(buf + o, sizeof(buf) - o, LV_SYMBOL_LOOP "%lu",
                  (unsigned long)state::stepCount);
    lv_label_set_text(lbl_sb_left, buf);

    // Центр: часы
    struct tm t; time_t now = time(NULL); localtime_r(&now, &t);
    snprintf(buf, sizeof(buf), "%02d:%02d", t.tm_hour, t.tm_min);
    lv_label_set_text(lbl_sb_mid, buf);

    // Справа: батарея
    int pct = watch.pmu.getBatteryPercent();
    snprintf(buf, sizeof(buf), "%s %d%%", batterySymbol(pct), pct);
    lv_label_set_text(lbl_sb_right, buf);
}

// ════════════════════════════════════════════════════════════════════════════
//  ЭКРАН 0: ЧАСЫ
// ════════════════════════════════════════════════════════════════════════════
// Данные уведомлений объявлены выше (до статусбара).
// Сервер/парсер/экран уведомлений — ниже по файлу (после BLE-сканера).

namespace scrClock {
    static lv_obj_t *root, *lblTime, *lblSec, *lblDate, *lblNotif;

    // Латиница — кириллицы нет в стоковом шрифте
    static const char *WDAY[] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};

    void build(lv_obj_t *parent) {
        root = parent;
        lv_obj_set_style_bg_color(root, lv_color_black(), 0);

        lblTime = makeLabel(root, BIG_FONT, 0xFFFFFF,
                            LV_ALIGN_CENTER, 0, -20);
        lv_label_set_text(lblTime, "--:--");

        // Секунды под временем по центру, мелким приглушённым
        lblSec = makeLabel(root, UI_FONT, 0x00CCFF,
                           LV_ALIGN_CENTER, 0, 18);
        lv_label_set_text(lblSec, "--");

        lblDate = makeLabel(root, UI_FONT, 0xAAAAAA,
                            LV_ALIGN_CENTER, 0, 52);
        lv_label_set_text(lblDate, "");

        // Баннер последнего уведомления внизу (тап по часам → экран уведомлений)
        lblNotif = makeLabel(root, NOTIF_FONT, 0xFFAA00,
                             LV_ALIGN_BOTTOM_MID, 0, -6);
        lv_obj_set_width(lblNotif, LV_HOR_RES - 16);
        lv_label_set_long_mode(lblNotif, LV_LABEL_LONG_DOT);
        lv_label_set_text(lblNotif, "");
    }

    void update() {
        static uint32_t last = 0;
        if (millis() - last < 1000) return;   // контент посекундный
        last = millis();

        struct tm t;
        time_t now = time(NULL);
        localtime_r(&now, &t);
        char buf[40];
        snprintf(buf, sizeof(buf), "%02d:%02d", t.tm_hour, t.tm_min);
        lv_label_set_text(lblTime, buf);
        snprintf(buf, sizeof(buf), "%02d", t.tm_sec);
        lv_label_set_text(lblSec, buf);
        snprintf(buf, sizeof(buf), "%s  %02d.%02d.%04d",
                 WDAY[t.tm_wday], t.tm_mday, t.tm_mon + 1, t.tm_year + 1900);
        lv_label_set_text(lblDate, buf);

        // Последнее уведомление + число непрочитанных
        char lt[28] = ""; int c, u;
        portENTER_CRITICAL(&notif::mux);
        c = notif::count; u = notif::unread;
        if (c > 0) {
            int idx = (notif::head - 1 + notif::MAX) % notif::MAX;
            const char *s = notif::items[idx].title[0] ? notif::items[idx].title
                                                       : notif::items[idx].body;
            strncpy(lt, s, sizeof(lt) - 1); lt[sizeof(lt) - 1] = 0;
        }
        portEXIT_CRITICAL(&notif::mux);
        char nb[64];
        if (c > 0) snprintf(nb, sizeof(nb), "%s %d  %s", LV_SYMBOL_BELL, u, lt);
        else       nb[0] = 0;
        lv_label_set_text(lblNotif, nb);
    }
}

// ════════════════════════════════════════════════════════════════════════════
//  ЭКРАН 1: СПИДОМЕТР
// ════════════════════════════════════════════════════════════════════════════
namespace scrSpeed {
    static lv_obj_t *root, *lblSpeed, *lblFix;

    void build(lv_obj_t *parent) {
        root = parent;
        lv_obj_set_style_bg_color(root, lv_color_black(), 0);
        lblSpeed = makeLabel(root, BIG_FONT, 0x00CCFF,
                             LV_ALIGN_CENTER, 0, 0);
        lv_label_set_text(lblSpeed, "0.0");
        lv_obj_t *u = makeLabel(root, UI_FONT, 0x555555,
                                LV_ALIGN_CENTER, 0, 52);
        lv_label_set_text(u, "km/h");
        lblFix = makeLabel(root, UI_FONT, 0xFF4444,
                           LV_ALIGN_TOP_MID, 0, cfg::CONTENT_TOP);
        lv_label_set_text(lblFix, LV_SYMBOL_GPS " NO FIX");
    }

    void update() {
        static uint32_t last = 0;
        if (millis() - last < 1000) return;   // спидометр посекундный
        last = millis();

        char buf[40];
        snprintf(buf, sizeof(buf), "%.1f", state::speedKmh);
        lv_label_set_text(lblSpeed, buf);

        uint32_t sats = state::gpsVisible;
        if (state::gpsFix >= 2 && sats >= 3) {
            snprintf(buf, sizeof(buf), LV_SYMBOL_GPS " OK  sats:%lu", sats);
            lv_obj_set_style_text_color(lblFix, lv_color_hex(0x00FF88), 0);
        } else {
            uint32_t s = millis() / 1000;
            snprintf(buf, sizeof(buf), LV_SYMBOL_GPS " %02lu:%02lu  sats:%lu",
                     s / 60, s % 60, sats);
            lv_obj_set_style_text_color(lblFix, lv_color_hex(0xFF4444), 0);
        }
        lv_label_set_text(lblFix, buf);
    }
}

// ════════════════════════════════════════════════════════════════════════════
//  ЭКРАН 2: АУДИО ВОДОПАД (waterfall)
//  Время сверху вниз, частота слева направо (0..24кГц), цвет = амплитуда.
// ════════════════════════════════════════════════════════════════════════════
namespace scrAudio {
    static lv_obj_t *root, *lblPeaks, *canvas;
    static uint16_t *wf       = nullptr;   // буфер водопада в PSRAM, RGB565
    static int       wfStride = 0;         // пикселей в строке (берём у LVGL)

    static const int WF_W      = 240;
    static const int WF_H      = 184;
    static const int STRIDE_PX = 256;      // запас под выравнивание (240*2=480Б)

    // Живой пол шума: тишина садится на пол -> синий, сигнал поднимается по палитре
    static float       dbFloor     = 20.0f;   // сглаженная оценка пола, дБ
    static const float WF_DB_SPAN  = 46.0f;   // окно над полом до «красного»

    void build(lv_obj_t *parent) {
        root = parent;
        lv_obj_set_style_bg_color(root, lv_color_black(), 0);
        lv_obj_set_style_pad_all(root, 0, 0);
        lv_obj_t *t = makeLabel(root, UI_FONT, 0x444444,
                                LV_ALIGN_TOP_MID, 0, cfg::CONTENT_TOP);
        lv_label_set_text(t, LV_SYMBOL_AUDIO " WATERFALL");

        lblPeaks = makeLabel(root, UI_FONT, 0x00FF88,
                             LV_ALIGN_TOP_MID, 0, cfg::CONTENT_TOP + 16);
        lv_label_set_text(lblPeaks, "...");

        canvas = lv_canvas_create(root);
        wf = (uint16_t *)ps_malloc((size_t)WF_H * STRIDE_PX * 2);
        if (wf) {
            memset(wf, 0, (size_t)WF_H * STRIDE_PX * 2);
            lv_canvas_set_buffer(canvas, wf, WF_W, WF_H, LV_COLOR_FORMAT_RGB565);
            lv_obj_align(canvas, LV_ALIGN_BOTTOM_MID, 0, 0);
            // Реальный шаг строки берём у LVGL (зависит от выравнивания)
            lv_image_dsc_t *d = (lv_image_dsc_t *)lv_canvas_get_image(canvas);
            wfStride = d ? (d->header.stride / 2) : WF_W;
            if (wfStride < WF_W || wfStride > STRIDE_PX) wfStride = WF_W;
        }
    }

    void update() {
        if (!wf) return;

        static uint32_t last = 0;
        if (millis() - last < 30) return;   // ~33 строки/сек — читаемый скролл
        last = millis();

        int halfN = cfg::FFT_N / 2;

        // Топ-3 пика по бинам -> текст
        const int NPEAKS = 3;
        int   pkBin[NPEAKS];
        float pkMag[NPEAKS];
        for (int p = 0; p < NPEAKS; p++) { pkBin[p] = -1; pkMag[p] = 0; }
        for (int i = 2; i < halfN - 1; i++) {
            float m = vReal[i];
            if (m <= vReal[i-1] || m <= vReal[i+1]) continue;  // лок. максимум
            for (int p = 0; p < NPEAKS; p++) {
                if (m > pkMag[p]) {
                    for (int q = NPEAKS - 1; q > p; q--) {
                        pkMag[q] = pkMag[q-1];
                        pkBin[q] = pkBin[q-1];
                    }
                    pkMag[p] = m; pkBin[p] = i;
                    break;
                }
            }
        }
        char buf[60];
        int off = 0;
        float binHz = cfg::FFT_SAMPLE_HZ / cfg::FFT_N;
        for (int p = 0; p < NPEAKS; p++) {
            if (pkBin[p] < 0) continue;
            float f = pkBin[p] * binHz;
            if (f >= 1000.0f)
                off += snprintf(buf + off, sizeof(buf) - off,
                                (p == 0) ? "%.1fk" : " %.1fk", f / 1000.0f);
            else
                off += snprintf(buf + off, sizeof(buf) - off,
                                (p == 0) ? "%.0f" : " %.0f", f);
        }
        if (off == 0) snprintf(buf, sizeof(buf), "---");
        lv_label_set_text(lblPeaks, buf);

        // Скролл водопада вниз на 1 строку
        size_t rowB = (size_t)wfStride * 2;
        memmove((uint8_t *)wf + rowB, wf, rowB * (WF_H - 1));

        // Верхняя строка: dB по колонкам, пол — по минимуму кадра
        static float col[WF_W];
        float mn = 1e9f;
        for (int x = 0; x < WF_W; x++) {
            int b0 = 1 + (int)((float)x       * (halfN - 2) / (WF_W - 1));
            int b1 = 1 + (int)((float)(x + 1) * (halfN - 2) / (WF_W - 1));
            if (b1 <= b0) b1 = b0 + 1;
            float m = 0;
            for (int b = b0; b < b1 && b < halfN; b++)
                if (vReal[b] > m) m = vReal[b];
            float dB = 20.0f * log10f(m + 1.0f);
            col[x] = dB;
            if (dB < mn) mn = dB;
        }
        dbFloor = dbFloor * 0.95f + mn * 0.05f;   // медленное сглаживание пола
        for (int x = 0; x < WF_W; x++)
            wf[x] = heatColor((col[x] - dbFloor) / WF_DB_SPAN);
        lv_obj_invalidate(canvas);
    }

    void onEnter() { if (hAudio) vTaskResume(hAudio); }
    void onExit()  { if (hAudio) vTaskSuspend(hAudio); }
}

// ════════════════════════════════════════════════════════════════════════════
//  ЭКРАН 3: LPD433 ВОДОПАД (waterfall)
//  Время сверху вниз, частота слева направо, цвет = RSSI над живым полом шума.
// ════════════════════════════════════════════════════════════════════════════
namespace scrLpd {
    static lv_obj_t *root, *lblPeaks, *canvas;
    static uint16_t *wf       = nullptr;   // буфер водопада в PSRAM, RGB565
    static int       wfStride = 0;
    static float     noiseFloor = -110.0f; // сглаженная оценка пола, дБм

    static const int WF_W      = 240;
    static const int WF_H      = 184;
    static const int STRIDE_PX = 256;
    static const float LPD_DB_SPAN = 25.0f;  // окно над полом до «красного» (меньше = контрастнее)

    void build(lv_obj_t *parent) {
        root = parent;
        lv_obj_set_style_bg_color(root, lv_color_black(), 0);
        lv_obj_set_style_pad_all(root, 0, 0);
        lv_obj_t *t = makeLabel(root, UI_FONT, 0x444444,
                                LV_ALIGN_TOP_MID, 0, cfg::CONTENT_TOP);
        lv_label_set_text(t, LV_SYMBOL_WIFI " LPD433 WF");

        lblPeaks = makeLabel(root, UI_FONT, 0xFFAA00,
                             LV_ALIGN_TOP_MID, 0, cfg::CONTENT_TOP + 16);
        lv_label_set_text(lblPeaks, "...");

        canvas = lv_canvas_create(root);
        wf = (uint16_t *)ps_malloc((size_t)WF_H * STRIDE_PX * 2);
        if (wf) {
            memset(wf, 0, (size_t)WF_H * STRIDE_PX * 2);
            lv_canvas_set_buffer(canvas, wf, WF_W, WF_H, LV_COLOR_FORMAT_RGB565);
            lv_obj_align(canvas, LV_ALIGN_BOTTOM_MID, 0, 0);
            lv_image_dsc_t *d = (lv_image_dsc_t *)lv_canvas_get_image(canvas);
            wfStride = d ? (d->header.stride / 2) : WF_W;
            if (wfStride < WF_W || wfStride > STRIDE_PX) wfStride = WF_W;
        }
    }

    void update() {
        if (!wf) return;
        if (!state::lpdDirty) return;   // одна строка на свежий свип
        state::lpdDirty = false;

        // Авто-пол: минимум RSSI по диапазону, медленное сглаживание
        int16_t mn = state::lpdRssi[0];
        for (int i = 1; i < cfg::LPD_CHANS; i++)
            if (state::lpdRssi[i] < mn) mn = state::lpdRssi[i];
        noiseFloor = noiseFloor * 0.9f + (float)mn * 0.1f;

        // Топ-N пиков по RSSI -> текст
        int    peakIdx[cfg::LPD_PEAKS];
        int16_t peakVal[cfg::LPD_PEAKS];
        for (int p = 0; p < cfg::LPD_PEAKS; p++) { peakIdx[p] = -1; peakVal[p] = -200; }
        for (int i = 0; i < cfg::LPD_CHANS; i++) {
            int16_t v = state::lpdRssi[i];
            for (int p = 0; p < cfg::LPD_PEAKS; p++) {
                if (v > peakVal[p]) {
                    for (int q = cfg::LPD_PEAKS - 1; q > p; q--) {
                        peakVal[q] = peakVal[q-1];
                        peakIdx[q] = peakIdx[q-1];
                    }
                    peakVal[p] = v; peakIdx[p] = i;
                    break;
                }
            }
        }
        char buf[60];
        int off = 0;
        for (int p = 0; p < cfg::LPD_PEAKS; p++) {
            if (peakIdx[p] < 0) continue;
            float f = cfg::LPD_START_MHZ + peakIdx[p] * cfg::LPD_STEP_MHZ;
            off += snprintf(buf + off, sizeof(buf) - off,
                            (p == 0) ? "%.3f" : " %.3f", f);
        }
        if (off == 0) snprintf(buf, sizeof(buf), "---");
        lv_label_set_text(lblPeaks, buf);

        // Скролл вниз + верхняя строка из RSSI относительно живого пола
        size_t rowB = (size_t)wfStride * 2;
        memmove((uint8_t *)wf + rowB, wf, rowB * (WF_H - 1));

        for (int x = 0; x < WF_W; x++) {
            // x -> канал (линейная интерполяция CHANS -> ширина)
            float src = (float)x * (cfg::LPD_CHANS - 1) / (WF_W - 1);
            int   i0  = (int)src;
            int   i1  = (i0 < cfg::LPD_CHANS - 1) ? i0 + 1 : i0;
            float frac = src - i0;
            float rssi = state::lpdRssi[i0] * (1 - frac) + state::lpdRssi[i1] * frac;

            float tn = (rssi - noiseFloor) / LPD_DB_SPAN;
            wf[x] = heatColor(tn);
        }
        lv_obj_invalidate(canvas);
    }

    void onEnter() {
        radio.standby();                               // выводим из sleep перед конфигом
        radio.setRxBandwidth(cfg::LPD_RX_BW);
        radio.setFrequency(cfg::LPD_START_MHZ, true);  // калибровка образа — один раз
        radio.startReceive();
        if (hLPD) vTaskResume(hLPD);
    }
    void onExit() {
        if (hLPD) vTaskSuspend(hLPD);
        radio.sleep();
    }
}

// ════════════════════════════════════════════════════════════════════════════
//  ЭКРАН 4: GPS ДИАГНОСТИКА + СПУТНИКИ
// ════════════════════════════════════════════════════════════════════════════
namespace scrGps {
    static lv_obj_t *root, *lblFix, *lblSats;

    void build(lv_obj_t *parent) {
        root = parent;
        lv_obj_set_style_bg_color(root, lv_color_black(), 0);

        lblFix = makeLabel(root, UI_FONT, 0x00FF88,
                           LV_ALIGN_TOP_LEFT, 4, cfg::CONTENT_TOP);
        lv_obj_set_width(lblFix, LV_HOR_RES - 8);
        lv_label_set_long_mode(lblFix, LV_LABEL_LONG_WRAP);
        lv_label_set_text(lblFix, "waiting...");

        // Список спутников ниже
        lblSats = makeLabel(root, UI_FONT, 0xAAAAAA,
                            LV_ALIGN_TOP_LEFT, 4, cfg::CONTENT_TOP + 90);
        lv_obj_set_width(lblSats, LV_HOR_RES - 8);
        lv_label_set_long_mode(lblSats, LV_LABEL_LONG_WRAP);
        lv_label_set_text(lblSats, "");
    }

    void update() {
        static uint32_t last = 0;
        if (millis() - last < 1000) return;   // диагностика посекундная
        last = millis();

        char buf[256];

        // Сводка фикса
        if (state::gpsFix >= 2) {
            snprintf(buf, sizeof(buf),
                LV_SYMBOL_GPS " FIX %dD  SIV:%u\n"
                "lat: %.6f\n"
                "lon: %.6f\n"
                "alt: %.0fm  spd: %.1f\n"
                "pDOP: %.1f  dist: %.0fm",
                state::gpsFix, state::gpsVisible,
                state::gpsLat, state::gpsLon,
                state::gpsAlt, state::speedKmh,
                state::gpsPdop, state::distanceM);
            lv_obj_set_style_text_color(lblFix, lv_color_hex(0x00FF88), 0);
        } else {
            uint32_t s = millis() / 1000;
            snprintf(buf, sizeof(buf),
                LV_SYMBOL_GPS " NO FIX  SIV:%u\n"
                "search: %02lu:%02lu\n"
                "pDOP: %.1f\n"
                "gnss: %s  synced: %s",
                state::gpsVisible,
                s / 60, s % 60,
                state::gpsPdop,
                gnssOk ? "ok" : "DOWN",
                state::gpsSynced ? "Y" : "N");
            lv_obj_set_style_text_color(lblFix, lv_color_hex(0xFF4444), 0);
        }
        lv_label_set_text(lblFix, buf);

        // Сводка: спутники в решении и дистанция
        char sbuf[80];
        snprintf(sbuf, sizeof(sbuf),
            "SIV: %u\nDistance: %.2f km\npDOP: %.1f",
            state::gpsVisible, state::distanceM / 1000.0, state::gpsPdop);
        lv_label_set_text(lblSats, sbuf);
    }
}

// ════════════════════════════════════════════════════════════════════════════
//  ЭКРАН 5: CSI АНАЛИЗАТОР КАНАЛА
//  Channel State Information — амплитуды поднесущих, плоскость спектра (flatness),
//  детектор возмущений эфира (motion). Слушает весь трафик на канале CSI_CHANNEL.
// ════════════════════════════════════════════════════════════════════════════
namespace scrCsi {
    static lv_obj_t *root, *lblTitle, *lblMetrics, *lblStatus;
    static lv_obj_t *bars[cfg::CSI_BARS];

    void build(lv_obj_t *parent) {
        root = parent;
        lv_obj_set_style_bg_color(root, lv_color_black(), 0);
        lv_obj_set_style_pad_all(root, 0, 0);

        lblTitle = makeLabel(root, UI_FONT, 0x888888,
                             LV_ALIGN_TOP_MID, 0, cfg::CONTENT_TOP);
        lv_label_set_text(lblTitle, LV_SYMBOL_WIFI " CSI ch1 " LV_SYMBOL_LEFT LV_SYMBOL_RIGHT);

        lblMetrics = makeLabel(root, UI_FONT, 0x00CCFF,
                               LV_ALIGN_TOP_LEFT, 4, cfg::CONTENT_TOP + 18);
        lv_obj_set_width(lblMetrics, LV_HOR_RES - 8);
        lv_label_set_long_mode(lblMetrics, LV_LABEL_LONG_WRAP);
        lv_label_set_text(lblMetrics, "waiting for packets...");

        lblStatus = makeLabel(root, UI_FONT, 0x00FF88,
                              LV_ALIGN_TOP_MID, 0, cfg::CONTENT_TOP + 72);
        lv_label_set_text(lblStatus, "");

        // Бары спектра поднесущих на всю ширину
        for (int i = 0; i < cfg::CSI_BARS; i++) {
            int x0 = barX(i, cfg::CSI_BARS);
            int x1 = barX(i + 1, cfg::CSI_BARS);
            bars[i] = makeBar(root, lv_color_hex(0x00AAFF));
            lv_obj_set_size(bars[i], (x1 - x0) > 1 ? (x1 - x0) : 1, 2);
            lv_obj_set_pos(bars[i], x0, LV_VER_RES - 2);
        }
    }

    void update() {
        // Process fresh CSI frame if any
        csiProcess();

        // Title with current channel
        char tbuf[40];
        snprintf(tbuf, sizeof(tbuf),
                 LV_SYMBOL_WIFI " CSI ch%d " LV_SYMBOL_LEFT LV_SYMBOL_RIGHT,
                 state::csiChannel);
        lv_label_set_text(lblTitle, tbuf);

        int subc = state::csiSubc;
        if (subc <= 0) {
            lv_label_set_text(lblMetrics, "no CSI packets\n(need WiFi traffic nearby)");
            // Сбрасываем бары в плоскую линию чтобы не висел старый спектр
            for (int i = 0; i < cfg::CSI_BARS; i++) {
                int x0 = barX(i, cfg::CSI_BARS);
                int x1 = barX(i + 1, cfg::CSI_BARS);
                lv_obj_set_size(bars[i], (x1 - x0) > 1 ? (x1 - x0) : 1, 2);
                lv_obj_set_pos(bars[i], x0, LV_VER_RES - 2);
            }
            return;
        }

        // Спектр амплитуд: интерполяция subc поднесущих на CSI_BARS баров
        int maxH = LV_VER_RES - (cfg::CONTENT_TOP + 100); 
        float amax = 1.0f;
        for (int i = 0; i < subc; i++)
            if (state::csiAmp[i] > amax) amax = state::csiAmp[i];

        for (int i = 0; i < cfg::CSI_BARS; i++) {
            float src = (subc > 1) ? (float)i * (subc - 1) / (cfg::CSI_BARS - 1) : 0;
            int   i0 = (int)src;
            if (i0 < 0) i0 = 0;
            if (i0 > subc - 1) i0 = subc - 1;
            int   i1 = (i0 < subc - 1) ? i0 + 1 : i0;
            float frac = src - i0;
            float amp = state::csiAmp[i0] * (1 - frac) + state::csiAmp[i1] * frac;

            int x0 = barX(i, cfg::CSI_BARS);
            int x1 = barX(i + 1, cfg::CSI_BARS);
            int w  = x1 - x0;
            int h  = 2 + (int)(amp / amax * (maxH - 2));
            h = constrain(h, 2, maxH);

            lv_obj_set_size(bars[i], w > 1 ? w : 1, h);
            lv_obj_set_pos(bars[i], x0, LV_VER_RES - h);

            // цвет по высоте
            uint8_t g = (uint8_t)(amp / amax * 255);
            lv_obj_set_style_bg_color(bars[i], lv_color_make(0, g, 255 - g/2), 0);
        }

        // Метрики текстом
        char buf[120];
        snprintf(buf, sizeof(buf),
                 LV_SYMBOL_WIFI " RSSI: %d dBm\n"
                 "Flatness: %.0f%%  (subc:%d)\n"
                 "Motion: %.0f%%  pkt:%lu",
                 state::csiRssi,
                 state::csiFlatness, subc,
                 state::csiMotion,
                 (unsigned long)state::csiPackets);
        lv_label_set_text(lblMetrics, buf);

        // Air status by motion
        if (state::csiMotion < 8) {
            lv_label_set_text(lblStatus, "Air: stable");
            lv_obj_set_style_text_color(lblStatus, lv_color_hex(0x00FF88), 0);
        } else if (state::csiMotion < 20) {
            lv_label_set_text(lblStatus, "Air: activity");
            lv_obj_set_style_text_color(lblStatus, lv_color_hex(0xFFAA00), 0);
        } else {
            lv_label_set_text(lblStatus, "Air: noisy");
            lv_obj_set_style_text_color(lblStatus, lv_color_hex(0xFF4444), 0);
        }
    }

    void onEnter() {
        esp_wifi_start();     // запускаем радио (конфиг сохранён с setup)
        delay(50);
        csiStart();
    }
    void onExit()  {
        csiStop();
        esp_wifi_stop();      // останавливаем радио без deinit — экономия + стабильность
    }
}

// ════════════════════════════════════════════════════════════════════════════
//  ЭКРАН 6: БАТАРЕЯ — кривая разряда и прогноз времени
// ════════════════════════════════════════════════════════════════════════════
namespace scrBattery {
    static lv_obj_t *root, *lblInfo, *chart;
    static lv_chart_series_t *ser;

    void build(lv_obj_t *parent) {
        root = parent;
        lv_obj_set_style_bg_color(root, lv_color_black(), 0);

        lblInfo = makeLabel(root, UI_FONT, 0x00FF88,
                            LV_ALIGN_TOP_LEFT, 4, cfg::CONTENT_TOP);
        lv_obj_set_width(lblInfo, LV_HOR_RES - 8);
        lv_label_set_long_mode(lblInfo, LV_LABEL_LONG_WRAP);
        lv_label_set_text(lblInfo, "collecting data...");

        // График кривой разряда
        chart = lv_chart_create(root);
        lv_obj_set_size(chart, LV_HOR_RES - 16, 90);
        lv_obj_align(chart, LV_ALIGN_BOTTOM_MID, 0, -8);
        lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
        lv_chart_set_point_count(chart, cfg::BATT_SAMPLES);
        lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 100);
        lv_obj_set_style_bg_color(chart, lv_color_hex(0x111111), 0);
        lv_obj_set_style_border_width(chart, 0, 0);
        lv_chart_set_div_line_count(chart, 3, 0);
        ser = lv_chart_add_series(chart, lv_color_hex(0x00FF88),
                                  LV_CHART_AXIS_PRIMARY_Y);
        // Скрываем точки на линии (только линия) — стиль точек серии
        lv_obj_set_style_width(chart, 0, LV_PART_INDICATOR);
        lv_obj_set_style_height(chart, 0, LV_PART_INDICATOR);
    }

    void update() {
        int n = state::battCount;
        int pct = watch.pmu.getBatteryPercent();
        uint16_t mv = watch.pmu.getBattVoltage();
        bool charging = watch.pmu.isCharging();

        // График трогаем только при появлении новой точки (раз в минуту)
        static int lastN = -1;
        if (n != lastN) {
            for (int i = 0; i < cfg::BATT_SAMPLES; i++)
                lv_chart_set_value_by_id(chart, ser, i,
                    i < n ? state::battHist[i] : LV_CHART_POINT_NONE);
            lv_chart_refresh(chart);
            lastN = n;
        }

        char buf[160];
        if (n < 2) {
            snprintf(buf, sizeof(buf),
                     LV_SYMBOL_BATTERY_FULL " %d%%  %umV\n%s\ncollecting data...",
                     pct, mv, charging ? "charging" : "discharging");
            lv_label_set_text(lblInfo, buf);
            return;
        }

        // Скорость: разница крайних точек за прошедшее время
        float hours = (float)(n - 1) * cfg::BATT_SAMPLE_MS / 3600000.0f;
        int   dPct  = (int)state::battHist[n - 1] - (int)state::battHist[0];
        float rate  = (hours > 0) ? (dPct / hours) : 0;   // %/час (знак: + заряд)

        char est[48];
        if (charging || rate > 0.5f) {
            float toFull = (rate > 0.1f) ? (100 - pct) / rate : 0;
            snprintf(est, sizeof(est), "to full: ~%.1fh", toFull);
        } else if (rate < -0.5f) {
            float toEmpty = pct / (-rate);
            int h = (int)toEmpty;
            int m = (int)((toEmpty - h) * 60);
            snprintf(est, sizeof(est), "to empty: ~%dh %dm", h, m);
        } else {
            snprintf(est, sizeof(est), "rate: stable");
        }

        snprintf(buf, sizeof(buf),
                 LV_SYMBOL_BATTERY_FULL " %d%%  %umV  %s\n"
                 "rate: %.1f %%/h\n"
                 "%s\n"
                 "window: %.1fh (%d pts)",
                 pct, mv, charging ? "CHG" : "",
                 rate,
                 est,
                 hours, n);
        lv_label_set_text(lblInfo, buf);
    }
}

// ════════════════════════════════════════════════════════════════════════════

// ════════════════════════════════════════════════════════════════════════════
//  ЭКРАН: WIFI РАДАР — счётчик устройств вокруг (channel hopping)
// ════════════════════════════════════════════════════════════════════════════
namespace scrRadar {
    static lv_obj_t *root, *lblBig, *lblList;

    void build(lv_obj_t *parent) {
        root = parent;
        lv_obj_set_style_bg_color(root, lv_color_black(), 0);

        lv_obj_t *t = makeLabel(root, UI_FONT, 0x444444,
                                LV_ALIGN_TOP_MID, 0, cfg::CONTENT_TOP);
        lv_label_set_text(t, LV_SYMBOL_WIFI " RADAR (all ch)");

        lblBig = makeLabel(root, BIG_FONT, 0x00CCFF,
                           LV_ALIGN_TOP_MID, 0, cfg::CONTENT_TOP + 18);
        lv_label_set_text(lblBig, "0");

        lblList = makeLabel(root, UI_FONT, 0xAAAAAA,
                            LV_ALIGN_TOP_LEFT, 4, cfg::CONTENT_TOP + 78);
        lv_obj_set_width(lblList, LV_HOR_RES - 8);
        lv_label_set_long_mode(lblList, LV_LABEL_LONG_WRAP);
        lv_label_set_text(lblList, "scanning...");
    }

    void update() {
        static uint32_t last = 0;
        if (millis() - last < 300) return;   // цифра меняется редко, 50Гц не нужно
        last = millis();

        int real = 0, rnd = 0;
        sniffDeviceCount(30000, &real, &rnd);   // за 30 сек

        // Большая цифра = реальные устройства (с настоящим OUI)
        char buf[16];
        snprintf(buf, sizeof(buf), "%d", real);
        lv_label_set_text(lblBig, buf);

        // Топ реальных устройств по сигналу (рандомные пропускаем)
        struct { int8_t rssi; uint8_t mac[6]; uint8_t ch; } top[5];
        int tn = 0;
        uint32_t now = millis();
        portENTER_CRITICAL(&sniff::mux);
        for (int i = 0; i < sniff::count; i++) {
            if (now - sniff::table[i].lastSeen > 30000) continue;
            if (sniff::table[i].isRandom) continue;     // только реальные
            int8_t r = sniff::table[i].rssi;
            int pos = tn;
            for (int p = 0; p < tn; p++) if (r > top[p].rssi) { pos = p; break; }
            if (tn < 5) tn++;
            for (int q = (tn < 5 ? tn - 1 : 4); q > pos; q--) top[q] = top[q-1];
            if (pos < 5) {
                top[pos].rssi = r;
                memcpy(top[pos].mac, sniff::table[i].mac, 6);
                top[pos].ch = sniff::table[i].ch;
            }
        }
        portEXIT_CRITICAL(&sniff::mux);

        char list[200];
        int off = snprintf(list, sizeof(list),
                           "real devices /30s\n+%d random (phones)\n", rnd);
        for (int i = 0; i < tn && off < (int)sizeof(list) - 1; i++) {
            off += snprintf(list + off, sizeof(list) - off,
                            "%s %ddBm ch%d\n",
                            ouiVendor(top[i].mac), top[i].rssi, top[i].ch);
        }
        lv_label_set_text(lblList, list);
    }

    void onEnter() { snifferStart(true); }    // channel hopping
    void onExit()  { snifferStop(); }
}

// ════════════════════════════════════════════════════════════════════════════
//  ЭКРАН: DEAUTH ДЕТЕКТОР — ловит deauth/disassoc фреймы (channel hopping)
// ════════════════════════════════════════════════════════════════════════════
namespace scrDeauth {
    static lv_obj_t *root, *lblTitle, *lblStatus, *lblStats;
    static uint32_t baseDeauth = 0, rateDeauth = 0, lastT = 0;
    static uint32_t baseAp = 0, rateAp = 0;

    void build(lv_obj_t *parent) {
        root = parent;
        lv_obj_set_style_bg_color(root, lv_color_black(), 0);

        lblTitle = makeLabel(root, UI_FONT, 0x444444,
                             LV_ALIGN_TOP_MID, 0, cfg::CONTENT_TOP);
        lv_label_set_text(lblTitle, LV_SYMBOL_WARNING " DEAUTH ch1");

        lblStatus = makeLabel(root, BIG_FONT, 0x00FF88,
                              LV_ALIGN_CENTER, 0, -10);
        lv_label_set_text(lblStatus, "OK");

        lblStats = makeLabel(root, UI_FONT, 0xAAAAAA,
                             LV_ALIGN_BOTTOM_MID, 0, -16);
        lv_obj_set_width(lblStats, LV_HOR_RES - 8);
        lv_label_set_long_mode(lblStats, LV_LABEL_LONG_WRAP);
        lv_label_set_text(lblStats, "");
    }

    void update() {
        uint32_t now = millis();
        if (now - lastT < 1000) return;   // весь экран обновляем раз в секунду
        lastT = now;

        // Цель выбрана — считаем deauth/disassoc с её участием, иначе по каналу
        uint32_t rate;
        char tb[48];
        if (state::apSelected) {
            rateAp = sniff::apDeauth - baseAp; baseAp = sniff::apDeauth;
            rate = rateAp;
            snprintf(tb, sizeof(tb), LV_SYMBOL_WARNING " %.12s", state::apSsid);
        } else {
            rateDeauth = sniff::cntDeauth - baseDeauth; baseDeauth = sniff::cntDeauth;
            rate = rateDeauth;
            snprintf(tb, sizeof(tb), LV_SYMBOL_WARNING " DEAUTH ch%d " LV_SYMBOL_LEFT LV_SYMBOL_RIGHT,
                     sniff::channel);
        }
        lv_label_set_text(lblTitle, tb);

        if (rate > 5) {
            lv_label_set_text(lblStatus, "ATTACK");
            lv_obj_set_style_text_color(lblStatus, lv_color_hex(0xFF4444), 0);
        } else if (rate > 0) {
            lv_label_set_text(lblStatus, "!");
            lv_obj_set_style_text_color(lblStatus, lv_color_hex(0xFFAA00), 0);
        } else {
            lv_label_set_text(lblStatus, "OK");
            lv_obj_set_style_text_color(lblStatus, lv_color_hex(0x00FF88), 0);
        }

        char buf[120];
        snprintf(buf, sizeof(buf),
                 "deauth/s: %lu\ntotal deauth: %lu\ndisassoc: %lu",
                 (unsigned long)rate,
                 (unsigned long)sniff::cntDeauth,
                 (unsigned long)sniff::cntDisassoc);
        lv_label_set_text(lblStats, buf);
    }

    void onEnter() {
        snifferStart(false, state::wifiChannel);   // фикс. канал, без прыжков
        baseDeauth = rateDeauth = 0;
        baseAp = rateAp = 0;
        lastT = millis();
    }
    void onExit()  { snifferStop(); }
}

// ════════════════════════════════════════════════════════════════════════════
//  ЭКРАН: PKT RATE — пульс эфира на канале (фикс. канал, свайп вверх/вниз)
// ════════════════════════════════════════════════════════════════════════════
namespace scrPktRate {
    static lv_obj_t *root, *lblTitle, *lblRate, *lblBreak;
    static uint32_t baseTotal = 0, baseMgmt = 0, baseData = 0, baseCtrl = 0;
    static uint32_t rTotal = 0, rMgmt = 0, rData = 0, rCtrl = 0;
    static uint32_t baseFrom = 0, baseTo = 0, rFrom = 0, rTo = 0;
    static uint32_t lastT = 0;

    void build(lv_obj_t *parent) {
        root = parent;
        lv_obj_set_style_bg_color(root, lv_color_black(), 0);

        lblTitle = makeLabel(root, UI_FONT, 0x444444,
                             LV_ALIGN_TOP_MID, 0, cfg::CONTENT_TOP);
        lv_label_set_text(lblTitle, LV_SYMBOL_WIFI " PKT RATE");

        lblRate = makeLabel(root, BIG_FONT, 0x00CCFF,
                            LV_ALIGN_CENTER, 0, -10);
        lv_label_set_text(lblRate, "0");

        lblBreak = makeLabel(root, UI_FONT, 0xAAAAAA,
                             LV_ALIGN_BOTTOM_MID, 0, -16);
        lv_obj_set_width(lblBreak, LV_HOR_RES - 8);
        lv_label_set_long_mode(lblBreak, LV_LABEL_LONG_WRAP);
        lv_label_set_text(lblBreak, "");
    }

    void update() {
        uint32_t now = millis();
        if (now - lastT < 1000) return;   // весь экран обновляем раз в секунду
        lastT = now;

        // Смена режима цель/канал — обнуляем базы, пропускаем тик (без выброса)
        static bool wasTgt = false;
        if (state::apSelected != wasTgt) {
            wasTgt = state::apSelected;
            baseTotal = sniff::cntTotal; baseMgmt = sniff::cntMgmt;
            baseData  = sniff::cntData;  baseCtrl = sniff::cntCtrl;
            baseFrom  = sniff::apFrom;   baseTo   = sniff::apTo;
            return;
        }

        char tb[48], rb[16], bb[100];
        if (state::apSelected) {
            rFrom = sniff::apFrom - baseFrom; baseFrom = sniff::apFrom;
            rTo   = sniff::apTo   - baseTo;   baseTo   = sniff::apTo;
            snprintf(tb, sizeof(tb), LV_SYMBOL_WIFI " %.12s", state::apSsid);
            snprintf(rb, sizeof(rb), "%lu", (unsigned long)(rFrom + rTo));
            snprintf(bb, sizeof(bb), "pkt/s AP c%d\nfrom:%lu to:%lu",
                     state::wifiChannel, (unsigned long)rFrom, (unsigned long)rTo);
        } else {
            rTotal = sniff::cntTotal - baseTotal; baseTotal = sniff::cntTotal;
            rMgmt  = sniff::cntMgmt  - baseMgmt;  baseMgmt  = sniff::cntMgmt;
            rData  = sniff::cntData  - baseData;  baseData  = sniff::cntData;
            rCtrl  = sniff::cntCtrl  - baseCtrl;  baseCtrl  = sniff::cntCtrl;
            snprintf(tb, sizeof(tb), LV_SYMBOL_WIFI " PKT ch%d " LV_SYMBOL_LEFT LV_SYMBOL_RIGHT,
                     sniff::channel);
            snprintf(rb, sizeof(rb), "%lu", (unsigned long)rTotal);
            snprintf(bb, sizeof(bb), "pkt/s total\nmgmt:%lu data:%lu ctrl:%lu",
                     (unsigned long)rMgmt, (unsigned long)rData, (unsigned long)rCtrl);
        }
        lv_label_set_text(lblTitle, tb);
        lv_label_set_text(lblRate, rb);
        lv_label_set_text(lblBreak, bb);
    }

    void onEnter() {
        snifferStart(false, state::wifiChannel);   // фикс. канал, без прыжков
        baseTotal = baseMgmt = baseData = baseCtrl = 0;
        baseFrom = baseTo = 0;
        lastT = millis();
    }
    void onExit()  { snifferStop(); }
}

// ════════════════════════════════════════════════════════════════════════════
//  ЭКРАН: FINDER — «горячо-холодно» по RSSI устройств на канале
// ════════════════════════════════════════════════════════════════════════════
namespace scrFinder {
    static const int ROWS = 5;
    static lv_obj_t *root, *lblTitle;
    static lv_obj_t *rName[ROWS], *rRssi[ROWS], *rBar[ROWS];

    void build(lv_obj_t *parent) {
        root = parent;
        lv_obj_set_style_bg_color(root, lv_color_black(), 0);

        lblTitle = makeLabel(root, UI_FONT, 0x444444,
                             LV_ALIGN_TOP_MID, 0, cfg::CONTENT_TOP);
        lv_label_set_text(lblTitle, LV_SYMBOL_GPS " FINDER");

        // Строка на устройство: имя | полоса сигнала | RSSI — всё в одну линию
        const int top = cfg::CONTENT_TOP + 24;
        const int rh  = 30;
        for (int i = 0; i < ROWS; i++) {
            int y = top + i * rh;
            rName[i] = makeLabel(root, UI_FONT, 0xCCCCCC, LV_ALIGN_TOP_LEFT, 4, y);
            rRssi[i] = makeLabel(root, UI_FONT, 0x00FF88, LV_ALIGN_TOP_RIGHT, -4, y);

            lv_obj_t *bar = lv_bar_create(root);
            lv_obj_set_size(bar, LV_HOR_RES - 8, 8);
            lv_obj_align(bar, LV_ALIGN_TOP_LEFT, 4, y + 18);
            lv_bar_set_range(bar, -100, -30);
            lv_obj_set_style_bg_color(bar, lv_color_hex(0x222222), LV_PART_MAIN);
            lv_obj_set_style_bg_color(bar, lv_color_hex(0x00FF88), LV_PART_INDICATOR);
            lv_obj_set_style_radius(bar, 2, LV_PART_MAIN);
            lv_obj_set_style_radius(bar, 2, LV_PART_INDICATOR);
            rBar[i] = bar;
        }
    }

    static void showRow(int i, bool on) {
        if (on) { lv_obj_clear_flag(rName[i], LV_OBJ_FLAG_HIDDEN);
                  lv_obj_clear_flag(rRssi[i], LV_OBJ_FLAG_HIDDEN);
                  lv_obj_clear_flag(rBar[i],  LV_OBJ_FLAG_HIDDEN); }
        else    { lv_obj_add_flag(rName[i], LV_OBJ_FLAG_HIDDEN);
                  lv_obj_add_flag(rRssi[i], LV_OBJ_FLAG_HIDDEN);
                  lv_obj_add_flag(rBar[i],  LV_OBJ_FLAG_HIDDEN); }
    }

    void update() {
        static uint32_t last = 0;
        if (millis() - last < 300) return;
        last = millis();

        char tb[40];
        snprintf(tb, sizeof(tb), LV_SYMBOL_GPS " FINDER ch%d " LV_SYMBOL_LEFT LV_SYMBOL_RIGHT,
                 sniff::channel);
        lv_label_set_text(lblTitle, tb);

        // топ-5 устройств на текущем канале по RSSI
        struct { int8_t rssi; uint8_t mac[6]; } top[ROWS];
        int tn = 0;
        uint32_t now = millis();
        portENTER_CRITICAL(&sniff::mux);
        for (int i = 0; i < sniff::count; i++) {
            if (sniff::table[i].ch != sniff::channel) continue;
            if (now - sniff::table[i].lastSeen > 10000) continue;
            int8_t r = sniff::table[i].rssi;
            int pos = tn;
            for (int p = 0; p < tn; p++) if (r > top[p].rssi) { pos = p; break; }
            if (tn < ROWS) tn++;
            for (int q = (tn < ROWS ? tn - 1 : ROWS - 1); q > pos; q--) top[q] = top[q-1];
            if (pos < ROWS) { top[pos].rssi = r; memcpy(top[pos].mac, sniff::table[i].mac, 6); }
        }
        portEXIT_CRITICAL(&sniff::mux);

        for (int i = 0; i < ROWS; i++) {
            if (i < tn) {
                lv_label_set_text(rName[i], ouiVendor(top[i].mac));
                char rb[8]; snprintf(rb, sizeof(rb), "%d", top[i].rssi);
                lv_label_set_text(rRssi[i], rb);
                lv_bar_set_value(rBar[i], constrain((int)top[i].rssi, -100, -30), LV_ANIM_OFF);
                showRow(i, true);
            } else {
                showRow(i, false);
            }
        }
        if (tn == 0) {
            showRow(0, true);
            lv_obj_add_flag(rBar[0],  LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(rRssi[0], LV_OBJ_FLAG_HIDDEN);
            lv_label_set_text(rName[0], "no devices");
        }
    }

    void onEnter() { snifferStart(false, state::wifiChannel); }
    void onExit()  { snifferStop(); }
}

// ════════════════════════════════════════════════════════════════════════════
//  ЭКРАН: ТОЧКИ ДОСТУПА — SSID, рабочий канал, RSSI; сортировка по RSSI (сильнее = выше)
//  Активное сканирование всех каналов через WiFi STA scan (asyncn), без промиска.
// ════════════════════════════════════════════════════════════════════════════
namespace scrAp {
    static lv_obj_t *root, *lblTitle, *lblList;

    static const int AP_MAX_SHOW = 12;                  // влезает в высоту экрана
    static const int AP_LIST_TOP = cfg::CONTENT_TOP + 20;
    static const int AP_LINE_H   = 18;                  // высота строки montserrat_14

    struct Row { uint8_t bssid[6]; uint8_t ch; int8_t rssi; uint8_t enc; char ssid[18]; };
    static Row rows[AP_MAX_SHOW];
    static int rowCount = 0;

    // Короткое имя типа шифрования
    static const char *encShort(uint8_t m) {
        switch (m) {
            case WIFI_AUTH_OPEN:          return "open";
            case WIFI_AUTH_WEP:           return "WEP";
            case WIFI_AUTH_WPA_PSK:       return "WPA";
            case WIFI_AUTH_WPA2_PSK:      return "WPA2";
            case WIFI_AUTH_WPA_WPA2_PSK:  return "WPA2";
            case WIFI_AUTH_WPA3_PSK:      return "WPA3";
            case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA3";
            default:                      return "ent?";
        }
    }

    void build(lv_obj_t *parent) {
        root = parent;
        lv_obj_set_style_bg_color(root, lv_color_black(), 0);

        lblTitle = makeLabel(root, UI_FONT, 0x444444,
                             LV_ALIGN_TOP_MID, 0, cfg::CONTENT_TOP);
        lv_label_set_text(lblTitle, LV_SYMBOL_WIFI " ACCESS POINTS");

        lblList = makeLabel(root, UI_FONT, 0x00CCFF,
                            LV_ALIGN_TOP_LEFT, 4, AP_LIST_TOP);
        lv_obj_set_width(lblList, LV_HOR_RES - 8);
        lv_label_set_long_mode(lblList, LV_LABEL_LONG_WRAP);
        lv_label_set_text(lblList, "scanning...");
    }

    // Стоковый шрифт LVGL — только латиница, чистим прочие байты
    static void asciiCopy(char *dst, int dstlen, const char *src) {
        int i = 0;
        for (; src[i] && i < dstlen - 1; i++) {
            char c = src[i];
            dst[i] = (c >= 0x20 && c < 0x7F) ? c : '?';
        }
        dst[i] = 0;
    }

    static void kickScan() {
        WiFi.scanDelete();
        WiFi.scanNetworks(true, true);   // async, со скрытыми
    }

    static void render() {
        char list[480];
        int off = 0;
        if (rowCount == 0) off = snprintf(list, sizeof(list), "no APs");
        for (int i = 0; i < rowCount && off < (int)sizeof(list) - 1; i++) {
            bool tgt = state::apSelected && memcmp(rows[i].bssid, state::apBssid, 6) == 0;
            off += snprintf(list + off, sizeof(list) - off, "%s%4d %2d %-4s %.10s\n",
                            tgt ? ">" : " ", rows[i].rssi, rows[i].ch,
                            encShort(rows[i].enc), rows[i].ssid);
        }
        lv_label_set_text(lblList, list);
    }

    // Тап по строке: выбрать точку целью (повтор по цели — снять)
    void tapSelect(int y) {
        // Новая цель/снятие — клиенты прошлой точки больше не релевантны
        portENTER_CRITICAL(&sniff::mux);
        sniff::clientCount = 0;
        portEXIT_CRITICAL(&sniff::mux);

        int i = (y - AP_LIST_TOP) / AP_LINE_H;
        if (i < 0 || i >= rowCount) { state::apSelected = false; render(); return; }
        bool same = state::apSelected && memcmp(rows[i].bssid, state::apBssid, 6) == 0;
        if (same) {
            state::apSelected = false;
        } else {
            memcpy(state::apBssid, rows[i].bssid, 6);
            strncpy(state::apSsid, rows[i].ssid, sizeof(state::apSsid) - 1);
            state::apSsid[sizeof(state::apSsid) - 1] = 0;
            state::wifiChannel = rows[i].ch;   // инструменты пойдут на канал точки
            state::apSelected  = true;
        }
        render();
    }

    void update() {
        static uint32_t last = 0;
        if (millis() - last < 500) return;
        last = millis();

        int n = WiFi.scanComplete();
        if (n == WIFI_SCAN_RUNNING) return;
        if (n < 0) { kickScan(); return; }   // не запущен/ошибка — пере-старт

        // Индексы, сортировка вставками по RSSI убыванием
        int idx[64];
        int m = n > 64 ? 64 : n;
        for (int i = 0; i < m; i++) idx[i] = i;
        for (int i = 1; i < m; i++) {
            int k = idx[i], j = i - 1;
            while (j >= 0 && WiFi.RSSI(idx[j]) < WiFi.RSSI(k)) { idx[j+1] = idx[j]; j--; }
            idx[j+1] = k;
        }

        rowCount = 0;
        for (int i = 0; i < m && rowCount < AP_MAX_SHOW; i++) {
            int e = idx[i];
            Row &r = rows[rowCount++];
            memcpy(r.bssid, WiFi.BSSID(e), 6);
            r.ch   = WiFi.channel(e);
            r.rssi = WiFi.RSSI(e);
            r.enc  = WiFi.encryptionType(e);
            String s = WiFi.SSID(e);
            asciiCopy(r.ssid, sizeof(r.ssid), s.length() ? s.c_str() : "<hidden>");
        }
        render();

        char tb[48];
        if (state::apSelected)
            snprintf(tb, sizeof(tb), LV_SYMBOL_WIFI " %d  " LV_SYMBOL_OK " %.10s", n, state::apSsid);
        else
            snprintf(tb, sizeof(tb), LV_SYMBOL_WIFI " APs (%d)  tap=target", n);
        lv_label_set_text(lblTitle, tb);

        kickScan();   // следующий проход
    }

    void onEnter() {
        esp_wifi_set_mode(WIFI_MODE_STA);
        esp_wifi_start();        // реально поднимаем STA (Arduino-флаг рассинхронен с raw stop)
        delay(50);
        WiFi.scanDelete();
        WiFi.scanNetworks(true, true);
    }
    void onExit() {
        esp_wifi_scan_stop();
        esp_wifi_set_mode(WIFI_MODE_AP);   // как после setup
        esp_wifi_stop();
    }
}

// ════════════════════════════════════════════════════════════════════════════
//  ЭКРАН: КЛИЕНТЫ ТОЧКИ — сидим на канале выбранной точки, считаем её станции
//  Цель выбирается тапом на экране ACCESS POINTS. Без цели — подсказка.
// ════════════════════════════════════════════════════════════════════════════
namespace scrClients {
    static lv_obj_t *root, *lblTitle, *lblBig, *lblList;
    static const uint32_t CLIENT_WIN = 30000;   // окно «живых» клиентов

    void build(lv_obj_t *parent) {
        root = parent;
        lv_obj_set_style_bg_color(root, lv_color_black(), 0);

        lblTitle = makeLabel(root, UI_FONT, 0x444444,
                             LV_ALIGN_TOP_MID, 0, cfg::CONTENT_TOP);
        lv_label_set_text(lblTitle, LV_SYMBOL_WIFI " CLIENTS");

        lblBig = makeLabel(root, BIG_FONT, 0x00CCFF,
                           LV_ALIGN_TOP_MID, 0, cfg::CONTENT_TOP + 18);
        lv_label_set_text(lblBig, "-");

        lblList = makeLabel(root, UI_FONT, 0xAAAAAA,
                            LV_ALIGN_TOP_LEFT, 4, cfg::CONTENT_TOP + 76);
        lv_obj_set_width(lblList, LV_HOR_RES - 8);
        lv_label_set_long_mode(lblList, LV_LABEL_LONG_WRAP);
        lv_label_set_text(lblList, "");
    }

    void update() {
        static uint32_t last = 0;
        if (millis() - last < 700) return;
        last = millis();

        if (!state::apSelected) {
            lv_label_set_text(lblTitle, LV_SYMBOL_WIFI " CLIENTS");
            lv_label_set_text(lblBig, "-");
            lv_label_set_text(lblList, "pick AP target\non ACCESS POINTS");
            return;
        }

        // Снимок живых клиентов под спинлоком, сортировка/вывод — вне него
        struct { int8_t rssi; uint8_t mac[6]; } snap[sniff::CLIENTS_MAX];
        int sn = 0;
        uint32_t now = millis();
        portENTER_CRITICAL(&sniff::mux);
        for (int i = 0; i < sniff::clientCount; i++) {
            if (now - sniff::clients[i].lastSeen > CLIENT_WIN) continue;
            snap[sn].rssi = sniff::clients[i].rssi;
            memcpy(snap[sn].mac, sniff::clients[i].mac, 6);
            sn++;
        }
        portEXIT_CRITICAL(&sniff::mux);

        // Сортировка по RSSI убыванием (ближе — выше)
        for (int i = 1; i < sn; i++) {
            auto k = snap[i]; int j = i - 1;
            while (j >= 0 && snap[j].rssi < k.rssi) { snap[j+1] = snap[j]; j--; }
            snap[j+1] = k;
        }

        char tb[48];
        snprintf(tb, sizeof(tb), LV_SYMBOL_WIFI " %.10s c%d", state::apSsid, state::wifiChannel);
        lv_label_set_text(lblTitle, tb);

        char nb[8];
        snprintf(nb, sizeof(nb), "%d", sn);
        lv_label_set_text(lblBig, nb);

        char list[300];
        int off = 0;
        if (sn == 0) off = snprintf(list, sizeof(list), "no clients seen");
        for (int i = 0; i < sn && i < 7 && off < (int)sizeof(list) - 1; i++) {
            off += snprintf(list + off, sizeof(list) - off, "%-9s %d\n",
                            ouiVendor(snap[i].mac), snap[i].rssi);
        }
        lv_label_set_text(lblList, list);
    }

    void onEnter() {
        if (state::apSelected) snifferStart(false, state::wifiChannel);
    }
    void onExit() { snifferStop(); }
}

// ════════════════════════════════════════════════════════════════════════════
//  WiFi РАЗВЕДКА (drill-down): точки -> клиенты точки -> узел.
//  Единая пассивная таблица (promiscuous + хопинг). Один шаблон списка,
//  разные фильтры. Сортировка по RSSI. Пакеты на узел = активность.
// ════════════════════════════════════════════════════════════════════════════
namespace recon {
    enum Kind : uint8_t { K_UNKNOWN = 0, K_STA = 1, K_AP = 2 };
    enum Enc  : uint8_t { E_OPEN = 0, E_WEP, E_WPA, E_WPA2, E_WPA3 };

    struct Dev {
        uint8_t  mac[6];
        uint8_t  bssid[6];     // STA: AP ассоциации; AP: == mac
        char     ssid[20];     // AP
        uint32_t packets;
        uint32_t lastSeen;
        int8_t   rssi;
        uint8_t  ch;
        uint8_t  kind;
        uint8_t  enc;
        bool     hasBssid;
        bool     beacon;       // SSID/enc подтверждены маяком (иначе AP выведена из трафика)
    };
    static const int MAX = 96;
    static Dev devs[MAX];
    static volatile int count = 0;
    static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

    // Лог probe-запросов: какой MAC какую сеть ищет (профиль устройства)
    struct Probe { uint8_t mac[6]; char ssid[20]; int8_t rssi; uint32_t lastSeen; };
    static const int PROBE_MAX = 64;
    static Probe probes[PROBE_MAX];
    static volatile int probeN = 0;

    static volatile bool started = false;
    static volatile bool hopping = true;
    static volatile int  channel = 1;
    static uint32_t hopLast = 0;

    static const char *encStr(uint8_t e) {
        switch (e) { case E_OPEN: return "open"; case E_WEP: return "WEP";
                     case E_WPA: return "WPA"; case E_WPA2: return "WPA2";
                     default: return "WPA3"; }
    }

    // запись probe (под спинлоком): дедуп по mac+ssid
    static void probeLog(const uint8_t *mac, const char *ssid, int8_t rssi) {
        for (int i = 0; i < probeN; i++)
            if (memcmp(probes[i].mac, mac, 6) == 0 && strcmp(probes[i].ssid, ssid) == 0) {
                probes[i].rssi = rssi; probes[i].lastSeen = millis(); return;
            }
        int i;
        if (probeN < PROBE_MAX) i = probeN++;
        else { uint32_t ot = 0xFFFFFFFF; i = 0;
               for (int k = 0; k < probeN; k++) if (probes[k].lastSeen < ot) { ot = probes[k].lastSeen; i = k; } }
        memcpy(probes[i].mac, mac, 6);
        strncpy(probes[i].ssid, ssid, sizeof(probes[i].ssid) - 1);
        probes[i].ssid[sizeof(probes[i].ssid) - 1] = 0;
        probes[i].rssi = rssi; probes[i].lastSeen = millis();
    }

    // create=false: только обновить уже известный узел (data-кадр не плодит фантом AP).
    // Возврат пропавшего узла: ищем по MAC — найдём, обновим lastSeen, снова виден.
    static void touch(const uint8_t *mac, uint8_t kind, int8_t rssi, uint8_t ch,
                      const uint8_t *bssid, const char *ssid, uint8_t enc, bool create,
                      bool beacon = false) {
        if (mac[0] & 0x01) return;            // мультикаст/бродкаст — не узел
        if (ch == 0 || ch > 14) return;       // битый канал — мусорный кадр

        int i = -1;
        for (int k = 0; k < count; k++)
            if (memcmp(devs[k].mac, mac, 6) == 0) { i = k; break; }
        if (i < 0) {
            if (!create) return;
            if (count < MAX) i = count++;
            else {
                uint32_t ot = 0xFFFFFFFF;
                for (int k = 0; k < count; k++)
                    if (devs[k].lastSeen < ot) { ot = devs[k].lastSeen; i = k; }
            }
            memset(&devs[i], 0, sizeof(Dev)); memcpy(devs[i].mac, mac, 6);
        }
        Dev &d = devs[i];
        d.rssi = rssi; d.ch = ch; d.lastSeen = millis(); d.packets++;
        if (kind > d.kind) d.kind = kind;
        if (bssid) { memcpy(d.bssid, bssid, 6); d.hasBssid = true; }
        if (ssid && ssid[0]) { strncpy(d.ssid, ssid, sizeof(d.ssid)-1); d.ssid[sizeof(d.ssid)-1]=0; }
        if (kind == K_AP) {
            if (beacon) { d.beacon = true; if (enc > d.enc) d.enc = enc; }
            if (!d.hasBssid) { memcpy(d.bssid, mac, 6); d.hasBssid = true; }
        }
    }

    // IE маяка: SSID + грубое шифрование (open/WEP/WPA/WPA2/WPA3)
    static void parseBeacon(const uint8_t *body, int len, char *ssidOut, uint8_t *encOut) {
        ssidOut[0] = 0;
        uint8_t cap0 = (len >= 11) ? body[10] : 0;
        bool privacy = cap0 & 0x10, rsn = false, wpa = false, sae = false;
        int p = 12;
        while (p + 2 <= len) {
            uint8_t id = body[p], l = body[p+1];
            if (p + 2 + l > len) break;
            const uint8_t *v = body + p + 2;
            if (id == 0) {
                int n = l < 19 ? l : 19;
                for (int k = 0; k < n; k++) ssidOut[k] = (v[k]>=0x20 && v[k]<0x7F) ? v[k] : '?';
                ssidOut[n] = 0;
            } else if (id == 48) {            // RSN -> WPA2; AKM SAE -> WPA3
                rsn = true;
                if (l >= 8) {
                    int pc = v[6] | (v[7] << 8);
                    int off = 8 + pc * 4;
                    if (off + 2 <= l) {
                        int ac = v[off] | (v[off+1] << 8), aoff = off + 2;
                        for (int s = 0; s < ac && aoff + 4 <= l; s++, aoff += 4)
                            if (v[aoff]==0x00 && v[aoff+1]==0x0F && v[aoff+2]==0xAC && v[aoff+3]==0x08)
                                sae = true;
                    }
                }
            } else if (id == 221 && l >= 4 &&
                       v[0]==0x00 && v[1]==0x50 && v[2]==0xF2 && v[3]==0x01) {
                wpa = true;
            }
            p += 2 + l;
        }
        *encOut = sae ? E_WPA3 : rsn ? E_WPA2 : wpa ? E_WPA : privacy ? E_WEP : E_OPEN;
    }

    // Только SSID из tagged-параметров (для assoc/reassoc — раскрывает скрытую AP)
    static void parseSsidOnly(const uint8_t *body, int len, char *out) {
        out[0] = 0;
        int p = 0;
        while (p + 2 <= len) {
            uint8_t id = body[p], l = body[p+1];
            if (p + 2 + l > len) break;
            if (id == 0) {
                int n = l < 19 ? l : 19;
                for (int k = 0; k < n; k++)
                    out[k] = (body[p+2+k] >= 0x20 && body[p+2+k] < 0x7F) ? body[p+2+k] : '?';
                out[n] = 0;
                return;
            }
            p += 2 + l;
        }
    }

    static void cb(void *buf, wifi_promiscuous_pkt_type_t type) {
        const wifi_promiscuous_pkt_t *p = (const wifi_promiscuous_pkt_t *)buf;
        const uint8_t *fr = p->payload;
        int len = p->rx_ctrl.sig_len;
        if (len < 24) return;
        int8_t rssi = p->rx_ctrl.rssi;
        uint8_t ch = p->rx_ctrl.channel;
        uint8_t ftype = (fr[0] >> 2) & 3, fsub = (fr[0] >> 4) & 0xF;
        const uint8_t *a1 = fr + 4, *a2 = fr + 10, *a3 = fr + 16;

        if (ftype == 0 && (fsub == 8 || fsub == 5)) {     // beacon / probe-resp -> AP (подтверждена)
            char ssid[20]; uint8_t enc = E_OPEN;
            parseBeacon(fr + 24, len - 24, ssid, &enc);
            portENTER_CRITICAL(&mux);
            touch(a2, K_AP, rssi, ch, a3, ssid, enc, true, true);
            portEXIT_CRITICAL(&mux);
        } else if (ftype == 0 && (fsub == 0 || fsub == 2)) {   // (re)assoc req -> имя скрытой AP
            int off = (fsub == 0) ? 4 : 10;
            char ssid[20]; parseSsidOnly(fr + 24 + off, len - 24 - off, ssid);
            portENTER_CRITICAL(&mux);
            if (ssid[0]) touch(a3, K_AP,  rssi, ch, a3, ssid, 0, false);
            touch(a2, K_STA, rssi, ch, a3, nullptr, 0, true);
            portEXIT_CRITICAL(&mux);
        } else if (ftype == 0 && fsub == 4) {              // probe request -> что ищет клиент
            char ssid[20]; parseSsidOnly(fr + 24, len - 24, ssid);
            portENTER_CRITICAL(&mux);
            touch(a2, K_STA, rssi, ch, nullptr, nullptr, 0, true);
            if (ssid[0]) probeLog(a2, ssid, rssi);         // направленный probe = сеть из PNL
            portEXIT_CRITICAL(&mux);
        } else if (ftype == 2) {                          // data: STA<->AP (AP выводим из трафика)
            uint8_t ds = fr[1] & 3;
            portENTER_CRITICAL(&mux);
            if (ds == 1)      { touch(a2, K_STA, rssi, ch, a1, nullptr, 0, true);
                                touch(a1, K_AP,  rssi, ch, a1, nullptr, 0, true); }
            else if (ds == 2) { touch(a1, K_STA, rssi, ch, a2, nullptr, 0, true);
                                touch(a2, K_AP,  rssi, ch, a2, nullptr, 0, true); }
            portEXIT_CRITICAL(&mux);
        } else {                                          // прочее: только обновить известных
            portENTER_CRITICAL(&mux);
            touch(a2, K_UNKNOWN, rssi, ch, nullptr, nullptr, 0, false);
            portEXIT_CRITICAL(&mux);
        }
    }

    static void start(int fixedCh) {
        if (started) return;
        portENTER_CRITICAL(&mux); count = 0; probeN = 0; portEXIT_CRITICAL(&mux);
        hopping = (fixedCh == 0);
        channel = fixedCh ? fixedCh : 1;
        hopLast = millis();
        esp_wifi_start();
        esp_wifi_set_promiscuous(true);
        esp_wifi_set_promiscuous_rx_cb(cb);
        wifi_promiscuous_filter_t f = { .filter_mask = WIFI_PROMIS_FILTER_MASK_ALL };
        esp_wifi_set_promiscuous_filter(&f);
        esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
        started = true;
    }
    static void stop() {
        if (!started) return;
        esp_wifi_set_promiscuous(false);
        esp_wifi_stop();
        started = false;
    }
    static void setLock(int ch) {        // 0 = хоп; иначе фикс. канал
        hopping = (ch == 0);
        if (ch) { channel = ch; esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE); }
    }
    static void hopTick() {              // из loop (безопасный контекст)
        if (!started || !hopping) return;
        if (millis() - hopLast < 250) return;
        hopLast = millis();
        int c = channel + 1; if (c > 13) c = 1;
        channel = c;
        esp_wifi_set_channel(c, WIFI_SECOND_CHAN_NONE);
    }
}

namespace scrRecon {
    enum View { V_APS, V_DEVS, V_AP, V_STA };   // точки / все устройства / клиенты AP / досье
    static lv_obj_t *root, *lblTitle, *lblHdr, *lblList;
    static int     view = V_APS;
    static int     staFrom = V_AP;        // откуда вошли в V_STA (для back)
    static uint8_t selBssid[6], selSta[6];
    static int     scrollRow = 0;
    static bool    forceRedraw = false;   // мгновенная перерисовка при drill/назад

    static const int LINE_H      = 20;
    static const int ROW_H       = 40;                      // две строки на элемент
    static const int TITLE_BOT   = cfg::CONTENT_TOP + 18;   // зона тапа = "назад"
    static const int LIST_APS_Y  = cfg::CONTENT_TOP + 20;   // список под титулом
    static const int CHART_Y     = cfg::CONTENT_TOP + 38;
    static const int CHART_H     = 34;
    static const int LIST_AP_Y   = cfg::CONTENT_TOP + 78;   // список под графиком
    static const int ROWS_APS    = 4;
    static const int ROWS_AP     = 3;
    static const uint32_t FRESH  = 20000;                   // свежий (норм. цвет)
    static const uint32_t ACTIVE = 60000;                   // окно подсчёта клиентов
    static const uint32_t KEEP   = 300000;                  // держать в списке 5 мин (серым)

    void update();
    bool back();
    void scroll(int d);

    // снимок таблицы (копия под спинлоком, обработка — вне)
    static recon::Dev snap[recon::MAX];
    static int        snapN = 0;
    static int  vis[recon::MAX];
    static int  visN = 0;

    // снимок probe-лога (для досье V_STA и счётчика PNL в V_DEVS)
    static recon::Probe probeSnap[recon::PROBE_MAX];
    static int          probeSnapN = 0;

    // график активности выбранного фильтра (пакетов/с)
    static lv_obj_t *chart = nullptr;
    static lv_chart_series_t *ser = nullptr;
    static uint32_t lastSampPkts = 0;
    static int      chartMax = 10;
    static bool     sampInit = false;
    static uint32_t sampMs = 0;
    static lv_obj_t *sbTrack = nullptr, *sbThumb = nullptr;   // скроллбар

    static bool withGraph() { return view == V_AP || view == V_STA; }
    static int listTop()  { return withGraph() ? LIST_AP_Y : LIST_APS_Y; }
    static int visRows()  { return withGraph() ? ROWS_AP : ROWS_APS; }

    // шифрование: для выведенной из трафика точки (без маяка) — неизвестно
    static const char *encOf(const recon::Dev &d) {
        return d.beacon ? recon::encStr(d.enc) : "?";
    }
    static int pnlCount(const uint8_t *mac) {
        int n = 0;
        for (int i = 0; i < probeSnapN; i++)
            if (memcmp(probeSnap[i].mac, mac, 6) == 0) n++;
        return n;
    }
    // явная подпись устройства: рандомный MAC -> "rnd XXXXXX", иначе вендор/хвост
    static void devLabel(const uint8_t *mac, char *out, int n) {
        if (mac[0] & 0x02) snprintf(out, n, "rnd %02X%02X%02X", mac[3], mac[4], mac[5]);
        else snprintf(out, n, "%.*s", n - 1, ouiVendor(mac));
    }

    void build(lv_obj_t *parent) {
        root = parent;
        lv_obj_set_style_bg_color(root, lv_color_black(), 0);
        lblTitle = makeLabel(root, UI_FONT, 0x00CCFF, LV_ALIGN_TOP_MID, 0, cfg::CONTENT_TOP);
        lv_label_set_text(lblTitle, LV_SYMBOL_WIFI " RECON");
        lblHdr = makeLabel(root, UI_FONT, 0xFFAA00, LV_ALIGN_TOP_LEFT, 4, cfg::CONTENT_TOP + 18);
        lv_obj_set_width(lblHdr, LV_HOR_RES - 8);
        lv_label_set_long_mode(lblHdr, LV_LABEL_LONG_WRAP);
        lv_label_set_text(lblHdr, "");

        chart = lv_chart_create(root);
        lv_obj_set_size(chart, LV_HOR_RES - 8, CHART_H);
        lv_obj_set_pos(chart, 4, CHART_Y);
        lv_chart_set_type(chart, LV_CHART_TYPE_BAR);
        lv_chart_set_point_count(chart, 30);
        lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, chartMax);
        lv_chart_set_div_line_count(chart, 0, 0);
        lv_obj_set_style_bg_opa(chart, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(chart, 0, 0);
        lv_obj_set_style_pad_all(chart, 0, 0);
        ser = lv_chart_add_series(chart, lv_color_hex(0x00FF88), LV_CHART_AXIS_PRIMARY_Y);
        lv_obj_add_flag(chart, LV_OBJ_FLAG_HIDDEN);

        lblList = makeLabel(root, UI_FONT, 0xCCCCCC, LV_ALIGN_TOP_LEFT, 4, LIST_APS_Y);
        lv_obj_set_width(lblList, LV_HOR_RES - 10);
        lv_label_set_long_mode(lblList, LV_LABEL_LONG_WRAP);
        lv_label_set_recolor(lblList, true);     // пропавшие строки красим серым
        lv_label_set_text(lblList, "scanning...");

        sbTrack = lv_obj_create(root);
        lv_obj_remove_style_all(sbTrack);
        lv_obj_set_style_bg_color(sbTrack, lv_color_hex(0x222222), 0);
        lv_obj_set_style_bg_opa(sbTrack, LV_OPA_COVER, 0);
        sbThumb = lv_obj_create(root);
        lv_obj_remove_style_all(sbThumb);
        lv_obj_set_style_bg_color(sbThumb, lv_color_hex(0x00CCFF), 0);
        lv_obj_set_style_bg_opa(sbThumb, LV_OPA_COVER, 0);
        lv_obj_add_flag(sbTrack, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(sbThumb, LV_OBJ_FLAG_HIDDEN);
    }

    static void updateScrollbar() {
        int vr = visRows();
        if (visN <= vr) {
            lv_obj_add_flag(sbTrack, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(sbThumb, LV_OBJ_FLAG_HIDDEN);
            return;
        }
        int top = listTop(), H = 226 - top;
        int th = H * vr / visN; if (th < 12) th = 12;
        int ty = top + (H - th) * scrollRow / (visN - vr);
        lv_obj_set_pos(sbTrack, LV_HOR_RES - 4, top); lv_obj_set_size(sbTrack, 3, H);
        lv_obj_set_pos(sbThumb, LV_HOR_RES - 4, ty);  lv_obj_set_size(sbThumb, 3, th);
        lv_obj_clear_flag(sbTrack, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(sbThumb, LV_OBJ_FLAG_HIDDEN);
    }

    static void resetChart() {
        sampInit = false; chartMax = 10; sampMs = 0;
        if (chart) { lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, chartMax);
                     lv_chart_set_all_value(chart, ser, 0); }
    }

    static int clientsOf(const uint8_t *bssid, uint32_t now) {
        int n = 0;
        for (int i = 0; i < snapN; i++)
            if (snap[i].kind == recon::K_STA && snap[i].hasBssid &&
                memcmp(snap[i].bssid, bssid, 6) == 0 && now - snap[i].lastSeen < ACTIVE) n++;
        return n;
    }

    static void rebuild() {
        portENTER_CRITICAL(&recon::mux);
        snapN = recon::count;
        memcpy(snap, recon::devs, snapN * sizeof(recon::Dev));
        probeSnapN = recon::probeN;
        memcpy(probeSnap, recon::probes, probeSnapN * sizeof(recon::Probe));
        portEXIT_CRITICAL(&recon::mux);

        uint32_t now = millis();
        visN = 0;

        if (view == V_STA) {                   // vis[] индексирует probeSnap (досье: что ищет MAC)
            for (int i = 0; i < probeSnapN; i++)
                if (memcmp(probeSnap[i].mac, selSta, 6) == 0) vis[visN++] = i;
        } else {
            for (int i = 0; i < snapN; i++) {
                recon::Dev &d = snap[i];
                if (now - d.lastSeen > KEEP) continue;
                bool take = false;
                if (view == V_APS)       take = (d.kind == recon::K_AP);
                else if (view == V_DEVS) take = (d.kind == recon::K_STA);
                else if (view == V_AP)   take = (d.kind == recon::K_STA && d.hasBssid &&
                                                 memcmp(d.bssid, selBssid, 6) == 0);
                if (take) vis[visN++] = i;     // порядок таблицы = порядок обнаружения, новые в конец
            }
        }
        int maxOff = visN - visRows();
        if (scrollRow > maxOff) scrollRow = maxOff;
        if (scrollRow < 0) scrollRow = 0;
    }

    static int findMac(const uint8_t *mac) {
        for (int i = 0; i < snapN; i++) if (memcmp(snap[i].mac, mac, 6) == 0) return i;
        return -1;
    }

    static void apName(const recon::Dev &a, char *out, int n) {
        if (a.ssid[0]) snprintf(out, n, "%.*s", n - 1, a.ssid);
        else snprintf(out, n, "%02X%02X%02X", a.mac[3], a.mac[4], a.mac[5]);
    }

    static void renderHeader(uint32_t now) {
        char t[44], h[120];
        if (view == V_APS) {
            snprintf(t, sizeof(t), LV_SYMBOL_WIFI " RECON ch%d (%d) " LV_SYMBOL_RIGHT "dev",
                     recon::channel, visN);
            lv_label_set_text(lblTitle, t);
            lv_label_set_text(lblHdr, "");
        } else if (view == V_DEVS) {
            snprintf(t, sizeof(t), LV_SYMBOL_GPS " DEVICES (%d) " LV_SYMBOL_RIGHT "ap", visN);
            lv_label_set_text(lblTitle, t);
            lv_label_set_text(lblHdr, "");
        } else if (view == V_AP) {
            int ai = findMac(selBssid);
            if (ai >= 0) {
                recon::Dev &a = snap[ai];
                char nm[18]; apName(a, nm, sizeof(nm));
                snprintf(t, sizeof(t), LV_SYMBOL_LEFT " %s", nm);
                snprintf(h, sizeof(h), "%.10s c%d %s %ddBm u%d",
                         ouiVendor(a.mac), a.ch, encOf(a), a.rssi, clientsOf(selBssid, now));
            } else { snprintf(t, sizeof(t), LV_SYMBOL_LEFT " AP lost"); h[0] = 0; }
            lv_label_set_text(lblTitle, t);
            lv_label_set_text(lblHdr, h);
        } else {   // V_STA — титул всегда из selSta (MAC не пропадёт при переписывании)
            snprintf(t, sizeof(t), LV_SYMBOL_LEFT " %02X:%02X:%02X:%02X:%02X:%02X%s",
                     selSta[0],selSta[1],selSta[2],selSta[3],selSta[4],selSta[5],
                     (selSta[0] & 0x02) ? " rnd" : "");
            int si = findMac(selSta);
            if (si >= 0) {
                recon::Dev &s = snap[si];
                int ai = findMac(s.bssid);
                char apn[12];
                if (s.hasBssid && ai >= 0) apName(snap[ai], apn, sizeof(apn));
                else if (s.hasBssid)       snprintf(apn, sizeof(apn), "%02X%02X%02X",
                                                    s.bssid[3], s.bssid[4], s.bssid[5]);
                else                       snprintf(apn, sizeof(apn), "-");
                uint32_t age = (now - s.lastSeen) / 1000;
                snprintf(h, sizeof(h), "%.8s %ddBm AP:%s %lus",
                         ouiVendor(selSta), s.rssi, apn, (unsigned long)age);
            } else {
                snprintf(h, sizeof(h), "%.8s  (gone)", ouiVendor(selSta));
            }
            lv_label_set_text(lblTitle, t);
            lv_label_set_text(lblHdr, h);
        }
    }

    // выборка пакеты/с выбранного узла -> график
    static void sampleActivity(uint32_t now) {
        int idx = (view == V_AP) ? findMac(selBssid) : findMac(selSta);
        uint32_t cur = (idx >= 0) ? snap[idx].packets : lastSampPkts;
        if (now - sampMs < 1000) return;
        sampMs = now;
        int32_t rate = sampInit ? (int32_t)(cur - lastSampPkts) : 0;
        if (rate < 0) rate = 0;
        lastSampPkts = cur; sampInit = true;
        if (rate > chartMax) { chartMax = rate;
            lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, 0, chartMax); }
        lv_chart_set_next_value(chart, ser, rate);
    }

    void update() {
    static uint32_t last = 0;
    uint32_t now = millis();                              // [FIX 1] один вызов millis()
    if (!forceRedraw && now - last < 500) return;
    forceRedraw = false;
    last = now;                                           // [FIX 1] last = now, не millis() снова

    rebuild();
    renderHeader(now);

    if (withGraph()) { lv_obj_clear_flag(chart, LV_OBJ_FLAG_HIDDEN); sampleActivity(now); }
    else             { lv_obj_add_flag(chart, LV_OBJ_FLAG_HIDDEN); }

    lv_obj_set_y(lblList, listTop());
    updateScrollbar();

    static const int LIST_SZ = 600;                      // [FIX 2] константа размера
    char list[LIST_SZ];
    list[0] = '\0';                                       // [FIX 4] явная инициализация
    int off = 0;
    int vr = visRows();

    // --- досье: какие сети ищет этот MAC (V_STA) ---
    if (view == V_STA) {
        if (visN == 0) { lv_label_set_text(lblList, "no probes seen"); return; }
        for (int r = 0; r < vr && scrollRow + r < visN; r++) {
            recon::Probe &pr = probeSnap[vis[scrollRow + r]];
            uint32_t age = (now - pr.lastSeen) / 1000;
            if (age > 9999) age = 9999;

            char buf[80];
            snprintf(buf, sizeof(buf), "%-.20s\n  %4ddBm %4" PRIu32 "s ago",  // [FIX 3] PRIu32
                     pr.ssid, pr.rssi, age);
            buf[sizeof(buf) - 1] = '\0';                 // [FIX 4] гарантия нуль-терминации

            for (char *p = buf; *p; p++) if (*p == '#') *p = '_';

            bool stale = (now - pr.lastSeen) > FRESH;
            int w = snprintf(list + off, LIST_SZ - off,
                             stale ? "~ %s\n" : "%s\n", buf);
            if (w <= 0 || off + w >= LIST_SZ - 1) break; // [FIX 2] защита от переполнения
            off += w;
        }
        lv_label_set_text(lblList, list);
        return;
    }

    // --- пустые состояния ---
    if (visN == 0) {
        lv_label_set_text(lblList,
            view == V_AP   ? "no clients yet" :
            view == V_DEVS ? "no devices yet" : "no APs yet");
        return;
    }

    // --- основной список ---
    for (int r = 0; r < vr && scrollRow + r < visN; r++) {
        recon::Dev &d = snap[vis[scrollRow + r]];
        uint32_t age = (now - d.lastSeen) / 1000;
        if (age > 999) age = 999;

        char buf[96];
        buf[0] = '\0';                                    // [FIX 4]

        if (view == V_APS) {
            char nm[18]; apName(d, nm, sizeof(nm));
            snprintf(buf, sizeof(buf), "%-.18s\n%4d ch%-2d %-4s u%2" PRIu32 " %3" PRIu32 "s",
                     nm, d.rssi, d.ch, encOf(d),
                     clientsOf(d.mac, now), age);         // [FIX 3] PRIu32 вместо %lu
        } else if (view == V_DEVS) {
            char dl[18]; devLabel(d.mac, dl, sizeof(dl));
            snprintf(buf, sizeof(buf), "%-.18s\n%4d %6" PRIu32 "p %2dnet %3" PRIu32 "s",
                     dl, d.rssi, d.packets, pnlCount(d.mac), age);
        } else {   // V_AP — клиенты точки
            char dl[18]; devLabel(d.mac, dl, sizeof(dl));
            snprintf(buf, sizeof(buf), "%-.18s\n%4d %6" PRIu32 "p %3" PRIu32 "s",
                     dl, d.rssi, d.packets, age);
        }

        buf[sizeof(buf) - 1] = '\0';                     // [FIX 4]
        for (char *p = buf; *p; p++) if (*p == '#') *p = '_';

        bool stale = (now - d.lastSeen) > FRESH;
        int w = snprintf(list + off, LIST_SZ - off,
                         stale ? "~ %s\n" : "%s\n", buf);
        if (w <= 0 || off + w >= LIST_SZ - 1) break;     // [FIX 2] обрываем, не пишем мусор
        off += w;
    }

    lv_label_set_text(lblList, list);
}

    // Тап: ТОЛЬКО титул -> переключение/назад; строка списка -> drill
    void tap(int y) {
        if (y < TITLE_BOT) {                           // тап строго по титулу
            if      (view == V_APS)  { view = V_DEVS; scrollRow = 0; forceRedraw = true; update(); }
            else if (view == V_DEVS) { view = V_APS;  scrollRow = 0; forceRedraw = true; update(); }
            else back();
            return;
        }
        if (view == V_STA) return;                     // в досье строки не кликабельны
        int lt = listTop();
        if (y < lt) return;                            // мета/график — не реагируем
        int r = (y - lt) / ROW_H + scrollRow;          // строка = два текстовых ряда
        if (r < 0 || r >= visN) return;
        recon::Dev &d = snap[vis[r]];
        if (view == V_APS) {
            memcpy(selBssid, d.mac, 6); view = V_AP; scrollRow = 0;
            recon::setLock(d.ch); resetChart();
        } else if (view == V_DEVS) {
            memcpy(selSta, d.mac, 6); view = V_STA; staFrom = V_DEVS; scrollRow = 0;
            recon::setLock(d.ch); resetChart();
        } else if (view == V_AP) {
            memcpy(selSta, d.mac, 6); view = V_STA; staFrom = V_AP; scrollRow = 0; resetChart();
        }
        forceRedraw = true; update();
    }

    void scroll(int d) {                  // тащим список (строк); работает и в досье (PNL)
        scrollRow += d;                   // зажим к диапазону сделает rebuild()
        forceRedraw = true; update();
    }

    bool back() {
        if (view == V_STA) {
            view = staFrom; scrollRow = 0; resetChart();
            if (staFrom == V_DEVS) recon::setLock(0);   // вернулись на верхний уровень — хоп
            forceRedraw = true; update(); return true;
        }
        if (view == V_AP)   { view = V_APS;  scrollRow = 0; recon::setLock(0); resetChart(); forceRedraw = true; update(); return true; }
        if (view == V_DEVS) { view = V_APS;  scrollRow = 0; forceRedraw = true; update(); return true; }
        return false;                                   // V_APS -> домой
    }

    void onEnter() { view = V_APS; scrollRow = 0; resetChart(); recon::start(0); }
    void onExit()  { recon::stop(); }
}

// ════════════════════════════════════════════════════════════════════════════
//  BLE СКАНЕР — пассивный сбор advertising, таблица устройств
// ════════════════════════════════════════════════════════════════════════════
namespace ble {
    struct Dev {
        char     addr[18];     // "XX:XX:XX:XX:XX:XX"
        char     name[20];
        int8_t   rssi;
        bool     isRandom;     // приватный (random) адрес
        uint32_t lastSeen;
    };
    static const int TABLE_SIZE = 64;
    Dev          table[TABLE_SIZE];
    volatile int count = 0;
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    volatile uint32_t advTotal = 0;
    bool         inited   = false;
    bool         scanning = false;
}

static NimBLEScan *bleScan = nullptr;

// Добавить/обновить устройство (host-задача NimBLE, не ISR)
static void bleTouch(const NimBLEAdvertisedDevice *dev)
{
    char addr[18];
    strncpy(addr, dev->getAddress().toString().c_str(), sizeof(addr) - 1);
    addr[sizeof(addr) - 1] = 0;

    char name[20] = {0};
    std::string n = dev->getName();
    if (!n.empty()) strncpy(name, n.c_str(), sizeof(name) - 1);

    bool   isRand = dev->getAddress().getType() & 1;   // 1,3 = random
    int8_t rssi   = dev->getRSSI();

    ble::advTotal++;

    portENTER_CRITICAL(&ble::mux);
    int found = -1, oldest = 0;
    uint32_t oldestT = 0xFFFFFFFF;
    for (int i = 0; i < ble::count; i++) {
        if (strcmp(ble::table[i].addr, addr) == 0) { found = i; break; }
        if (ble::table[i].lastSeen < oldestT) { oldestT = ble::table[i].lastSeen; oldest = i; }
    }
    int idx = (found >= 0)                   ? found
            : (ble::count < ble::TABLE_SIZE) ? ble::count++ : oldest;

    strncpy(ble::table[idx].addr, addr, sizeof(ble::table[idx].addr));
    if (name[0])        strncpy(ble::table[idx].name, name, sizeof(ble::table[idx].name));
    else if (found < 0) ble::table[idx].name[0] = 0;   // новый слот без имени
    ble::table[idx].rssi     = rssi;
    ble::table[idx].isRandom = isRand;
    ble::table[idx].lastSeen = millis();
    portEXIT_CRITICAL(&ble::mux);
}

class BleScanCb : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice *dev) override { bleTouch(dev); }
};
static BleScanCb bleCb;

static void bleInit()
{
    if (ble::inited) return;

    // Сдвигаем базовый MAC до инициализации контроллера: для телефона часы
    // становятся новым устройством — мимо протухшего GATT-кеша и старого бонда.
    // Стабильно (всегда заводской MAC XOR 1). Если связь наладится — можно убрать.
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    mac[5] ^= 0x01;
    esp_base_mac_addr_set(mac);

    NimBLEDevice::init("");
    bleScan = NimBLEDevice::getScan();
    bleScan->setScanCallbacks(&bleCb, false);   // false: объект статический
    bleScan->setActiveScan(true);               // active = ловим имена (scan rsp)
    bleScan->setInterval(80);
    bleScan->setWindow(60);
    bleScan->setMaxResults(0);                  // только callback, без буфера
    bleScan->setDuplicateFilter(false);         // повторы нужны для свежего RSSI
    ble::inited = true;
}

static void bleStart()
{
    if (ble::scanning) return;   // idempotent: повторный start скана — undefined в NimBLE
    bleInit();
    portENTER_CRITICAL(&ble::mux);
    ble::count = 0;
    portEXIT_CRITICAL(&ble::mux);
    ble::advTotal = 0;
    bleScan->start(0, false);   // 0 = непрерывно
    ble::scanning = true;
}

static void bleStop()
{
    if (!ble::scanning) return;
    bleScan->stop();
    bleScan->clearResults();
    ble::scanning = false;
}

namespace scrBle {
    static lv_obj_t *root, *lblBig, *lblList;

    void build(lv_obj_t *parent) {
        root = parent;
        lv_obj_set_style_bg_color(root, lv_color_black(), 0);

        lv_obj_t *t = makeLabel(root, UI_FONT, 0x444444,
                                LV_ALIGN_TOP_MID, 0, cfg::CONTENT_TOP);
        lv_label_set_text(t, LV_SYMBOL_BLUETOOTH " BLE SCAN");

        lblBig = makeLabel(root, BIG_FONT, 0x00CCFF,
                           LV_ALIGN_TOP_MID, 0, cfg::CONTENT_TOP + 18);
        lv_label_set_text(lblBig, "0");

        lblList = makeLabel(root, UI_FONT, 0xAAAAAA,
                            LV_ALIGN_TOP_LEFT, 4, cfg::CONTENT_TOP + 78);
        lv_obj_set_width(lblList, LV_HOR_RES - 8);
        lv_label_set_long_mode(lblList, LV_LABEL_LONG_WRAP);
        lv_label_set_text(lblList, "scanning...");
    }

    void update() {
        static uint32_t last = 0;
        if (millis() - last < 300) return;   // список меняется редко, 50Гц не нужно
        last = millis();

        uint32_t now = millis();
        const uint32_t WIN = 30000;

        struct { int8_t rssi; bool rnd; char addr[18]; char name[20]; } top[5];
        int tn = 0, total = 0, rnd = 0, named = 0;

        portENTER_CRITICAL(&ble::mux);
        for (int i = 0; i < ble::count; i++) {
            if (now - ble::table[i].lastSeen > WIN) continue;
            total++;
            if (ble::table[i].isRandom) rnd++;
            if (ble::table[i].name[0])  named++;
            int8_t r = ble::table[i].rssi;
            int pos = tn;
            for (int p = 0; p < tn; p++) if (r > top[p].rssi) { pos = p; break; }
            if (tn < 5) tn++;
            for (int q = (tn < 5 ? tn - 1 : 4); q > pos; q--) top[q] = top[q-1];
            if (pos < 5) {
                top[pos].rssi = r;
                top[pos].rnd  = ble::table[i].isRandom;
                memcpy(top[pos].addr, ble::table[i].addr, sizeof(top[pos].addr));
                memcpy(top[pos].name, ble::table[i].name, sizeof(top[pos].name));
            }
        }
        portEXIT_CRITICAL(&ble::mux);

        char buf[16];
        snprintf(buf, sizeof(buf), "%d", total);
        lv_label_set_text(lblBig, buf);

        char list[260];
        int off = snprintf(list, sizeof(list),
                           "devices /30s\nnamed:%d random:%d\n", named, rnd);
        for (int i = 0; i < tn && off < (int)sizeof(list) - 1; i++) {
            const char *id = top[i].name[0] ? top[i].name : top[i].addr;
            off += snprintf(list + off, sizeof(list) - off,
                            "%s %ddBm%s\n", id, top[i].rssi, top[i].rnd ? " r" : "");
        }
        lv_label_set_text(lblList, list);
    }

    void onEnter() { bleStart(); }
    void onExit()  { bleStop(); }
}

// ════════════════════════════════════════════════════════════════════════════
//  УВЕДОМЛЕНИЯ ПО BLE — постоянный GATT-сервер, сервис Alert Notification (0x1811).
//  Мост на телефоне — Gadgetbridge (тип устройства PineTime/InfiniTime): он пишет
//  уведомления в New Alert (0x2A46) как [категория][счёт][title\0body].
//  (данные notif:: объявлены выше — их читает экран часов)
// ════════════════════════════════════════════════════════════════════════════

// UTF-8 совместимая очистка текста; ASCII + кириллица (и любой UTF-8) проходят.
// Управляющие символы (< 0x20) выкидываются. nullToSpace: внутренние 0x00 → пробел.
static void cleanText(char *dst, int cap, const char *src, int n, bool nullToSpace)
{
    int j = 0;
    for (int i = 0; i < n && j < cap - 1; i++) {
        unsigned char c = (unsigned char)src[i];
        if (c == 0) { if (nullToSpace && j > 0) dst[j++] = ' '; continue; }
        // Пропускаем управляющие символы кроме пробела, но оставляем UTF-8
        if (c < 0x20) continue;   // управляющие — пропустить
        dst[j++] = (char)c;       // ASCII и UTF-8 multi-byte — пропустить как есть
    }
    while (j > 0 && dst[j - 1] == ' ') j--;   // trim хвоста
    dst[j] = 0;
}

// Оставляем только то, что есть в шрифте: ASCII-печатный, перевод строки и
// кириллицу. Прочее (эмодзи, спецпунктуация, \r, управляющие) выкидываем,
// иначе рисуется глиф-заглушка (квадрат).
static void sanitizeText(const char *in, char *out, int outsz)
{
    int o = 0;
    const uint8_t *s = (const uint8_t *)in;
    while (*s && o < outsz - 1) {
        uint8_t c = *s;
        if (c == '\n')                       { out[o++] = '\n'; s++; }
        else if (c >= 0x20 && c <= 0x7E)     { out[o++] = c;    s++; }
        else if (c == 0xD0 && s[1]) {                 // кириллица: Ё, А..п
            uint8_t c2 = s[1];
            if ((c2 == 0x81 || (c2 >= 0x90 && c2 <= 0xBF)) && o < outsz - 2)
                { out[o++] = c; out[o++] = c2; }
            s += 2;
        } else if (c == 0xD1 && s[1]) {               // кириллица: р..я, ё
            uint8_t c2 = s[1];
            if ((c2 == 0x91 || (c2 >= 0x80 && c2 <= 0x8F)) && o < outsz - 2)
                { out[o++] = c; out[o++] = c2; }
            s += 2;
        } else if (c >= 0xC0) {                        // прочий многобайт — пропустить целиком
            s++;
            while ((*s & 0xC0) == 0x80) s++;
        } else s++;                                    // прочие управляющие (\r, \t)
    }
    out[o] = 0;
}

static void notifPush(uint8_t cat, const char *title, const char *body)
{
    char st[28], sb[80];
    sanitizeText(title, st, sizeof(st));
    sanitizeText(body,  sb, sizeof(sb));
    portENTER_CRITICAL(&notif::mux);
    notif::Item &it = notif::items[notif::head];
    it.cat = cat; it.when = millis();
    strncpy(it.title, st, sizeof(it.title) - 1); it.title[sizeof(it.title) - 1] = 0;
    strncpy(it.body,  sb, sizeof(it.body)  - 1); it.body[sizeof(it.body)   - 1] = 0;
    notif::head = (notif::head + 1) % notif::MAX;
    if (notif::count  < notif::MAX) notif::count++;
    if (notif::unread < 99)         notif::unread++;
    portEXIT_CRITICAL(&notif::mux);
    notif::arrived = true;
}

// Запись в New Alert от Gadgetbridge (контекст host-задачи NimBLE)
class NewAlertCb : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &) override {
        NimBLEAttValue v = c->getValue();
        const uint8_t *d = v.data();
        int n = (int)v.length();
        if (n < 2) return;
        uint8_t cat = d[0];                    // d[1] = число новых
        int start = 2;
        int sep = -1;                          // первый 0x00 делит title|body
        for (int k = start; k < n; k++) if (d[k] == 0) { sep = k; break; }

        char title[28] = "", body[80] = "";
        if (sep > start) {
            cleanText(title, sizeof(title), (const char *)d + start, sep - start, false);
            cleanText(body,  sizeof(body),  (const char *)d + sep + 1, n - sep - 1, true);
        } else {
            cleanText(body, sizeof(body), (const char *)d + start, n - start, true);
        }
        if (!body[0] && title[0]) { strcpy(body, title); title[0] = 0; }  // всё ушло в title
        notifPush(cat, title, body);
    }
};

class NotifSrvCb : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *, NimBLEConnInfo &) override {
        notif::connected = true; notif::pushNow = true;
        Serial.println("[BLE] connect");
    }
    void onDisconnect(NimBLEServer *, NimBLEConnInfo &, int) override {
        notif::connected = false;
        Serial.println("[BLE] disconnect");
        NimBLEDevice::startAdvertising();      // снова видимы для реконнекта
    }
};

// Логирует подписку/чтение клиентом (GB) — видно, трогает ли он характеристику
class SubLogCb : public NimBLECharacteristicCallbacks {
    const char *tag;
public:
    SubLogCb(const char *t) : tag(t) {}
    void onSubscribe(NimBLECharacteristic *, NimBLEConnInfo &, uint16_t v) override {
        Serial.printf("[BLE] subscribe %s = %u\n", tag, v);
    }
    void onRead(NimBLECharacteristic *, NimBLEConnInfo &) override {
        Serial.printf("[BLE] read %s\n", tag);
    }
};

// Current Time — Gadgetbridge пишет местное время телефона. Применяем в loop
// (избегаем I2C к RTC из host-задачи NimBLE).
class CtsCb : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *c, NimBLEConnInfo &) override {
        NimBLEAttValue v = c->getValue();
        const uint8_t *d = v.data();
        if (v.length() < 7) return;
        int year = d[0] | (d[1] << 8);
        if (year < 2020) return;
        notif::tYear = year; notif::tMon = d[2]; notif::tDay = d[3];
        notif::tHour = d[4]; notif::tMin = d[5]; notif::tSec = d[6];
        notif::timeSet = true;
    }
};

static void notifInit()
{
    NimBLEDevice::setDeviceName(cfg::BLE_NOTIF_NAME);
    NimBLEDevice::setSecurityAuth(false, false, false);   // без бонда: переключ. не ломает связь

    NimBLEServer *srv = NimBLEDevice::createServer();
    srv->setCallbacks(new NotifSrvCb());

    NimBLEService *ans = srv->createService((uint16_t)0x1811);
    uint8_t catmask[2] = { 0xFF, 0x03 };                // поддерживаем все категории
    ans->createCharacteristic((uint16_t)0x2A47, NIMBLE_PROPERTY::READ)->setValue(catmask, 2);
    ans->createCharacteristic((uint16_t)0x2A46,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR)->setCallbacks(new NewAlertCb());
    ans->start();

    // Device Information — Gadgetbridge по версии прошивки опознаёт InfiniTime
    NimBLEService *dis = srv->createService((uint16_t)0x180A);
    dis->createCharacteristic((uint16_t)0x2A29, NIMBLE_PROPERTY::READ)->setValue("InfiniTime");
    dis->createCharacteristic((uint16_t)0x2A26, NIMBLE_PROPERTY::READ)->setValue("1.14.0");
    dis->start();

    // Battery — уровень заряда (Gadgetbridge показывает)
    NimBLEService *bat = srv->createService((uint16_t)0x180F);
    notif::chBatt = bat->createCharacteristic((uint16_t)0x2A19,
                        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    int p0 = watch.pmu.getBatteryPercent();
    uint8_t b0 = (uint8_t)constrain(p0, 0, 100);   // реальный заряд, не 0
    notif::chBatt->setValue(&b0, 1);
    notif::chBatt->setCallbacks(new SubLogCb("batt"));
    bat->start();

    // Current Time — приём времени телефона
    NimBLEService *cts = srv->createService((uint16_t)0x1805);
    cts->createCharacteristic((uint16_t)0x2A2B,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY)
        ->setCallbacks(new CtsCb());
    cts->start();

    // InfiniTime Motion Service — счётчик шагов (основа активности/дистанции в GB)
    NimBLEService *mot = srv->createService(NimBLEUUID("00030000-78fc-48fe-8e23-433b3a1942d0"));
    notif::chStep = mot->createCharacteristic(
        NimBLEUUID("00030001-78fc-48fe-8e23-433b3a1942d0"),
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    uint32_t z32 = 0; notif::chStep->setValue((uint8_t *)&z32, 4);
    notif::chStep->setCallbacks(new SubLogCb("step"));
    mot->start();

    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    adv->setName(cfg::BLE_NOTIF_NAME);       // имя в основном пакете — видно при любом скане
    adv->enableScanResponse(false);
    bool advOk = NimBLEDevice::startAdvertising();
    Serial.printf("[BLE] svc: ans=%d dis=%d bat=%d cts=%d mot=%d adv=%d\n",
                  ans != nullptr, dis != nullptr, bat != nullptr,
                  cts != nullptr, mot != nullptr, advOk);
}

// Короткий бип: строим WAV в памяти один раз и проигрываем через кодек.
static void notifBeep()
{
    static const int SR = 16000, MS = 120, FREQ = 2300;
    static const int N  = SR * MS / 1000;
    static uint8_t   wav[44 + (16000 * 120 / 1000) * 2];
    static bool      built = false;
    if (!built) {
        uint32_t dataLen = (uint32_t)N * 2, u32; uint16_t u16;
        memcpy(wav, "RIFF", 4);
        u32 = 36 + dataLen;          memcpy(wav + 4,  &u32, 4);
        memcpy(wav + 8, "WAVEfmt ", 8);
        u32 = 16;                    memcpy(wav + 16, &u32, 4);   // размер fmt
        u16 = 1;                     memcpy(wav + 20, &u16, 2);   // PCM
        u16 = 1;                     memcpy(wav + 22, &u16, 2);   // моно
        u32 = SR;                    memcpy(wav + 24, &u32, 4);
        u32 = SR * 2;                memcpy(wav + 28, &u32, 4);   // байт/с
        u16 = 2;                     memcpy(wav + 32, &u16, 2);   // выравнивание
        u16 = 16;                    memcpy(wav + 34, &u16, 2);   // бит
        memcpy(wav + 36, "data", 4); memcpy(wav + 40, &dataLen, 4);
        int16_t *pcm = (int16_t *)(wav + 44);
        int fade = SR / 50;          // ~20мс спад в конце
        for (int i = 0; i < N; i++) {
            float env = (i < N - fade) ? 1.0f : (float)(N - i) / fade;
            pcm[i] = (int16_t)(7000.0f * env * sinf(2.0f * 3.14159265f * FREQ * i / SR));
        }
        built = true;
    }
    watch.powerControl(POWER_SPEAK, true);
    watch.player.playWAV(wav, sizeof(wav));
    watch.powerControl(POWER_SPEAK, false);
}

// Периодическая отдача батареи и шагов в Gadgetbridge (вызывается из loop)
static void notifServiceTick()
{
    if (!notif::connected) return;
    uint32_t now = millis();
    static uint32_t lastB = 0, lastS = 0;
    bool force = notif::pushNow; notif::pushNow = false;   // отдать сразу после коннекта
    if (notif::chBatt && (force || now - lastB > 30000)) {
        lastB = now;
        int p = watch.pmu.getBatteryPercent();
        uint8_t pct = (uint8_t)constrain(p, 0, 100);
        notif::chBatt->setValue(&pct, 1);
        bool ok = notif::chBatt->notify();
        Serial.printf("[BLE] batt notify %u%% ok=%d\n", pct, ok);
    }
    if (notif::chStep && (force || now - lastS > 10000)) {
        lastS = now;
        uint32_t steps = watch.sensor.getPedometerCounter();
        notif::chStep->setValue((uint8_t *)&steps, 4);
        bool ok = notif::chStep->notify();
        Serial.printf("[BLE] step notify %lu ok=%d\n", (unsigned long)steps, ok);
    }
}

// ════════════════════════════════════════════════════════════════════════════
//  ЭКРАН: УВЕДОМЛЕНИЯ — список последних, статус подключения
// ════════════════════════════════════════════════════════════════════════════
namespace scrNotif {
    static lv_obj_t *root, *lblTitle, *lblList;
    static const int LIST_TOP = cfg::CONTENT_TOP + 20;
    static const int PER_VIEW = 4;        // записей на экран
    static int  scrollRow = 0;
    static int  lastSn = 0;
    static bool force = false;

    void update();

    void build(lv_obj_t *parent) {
        root = parent;
        lv_obj_set_style_bg_color(root, lv_color_black(), 0);

        lblTitle = makeLabel(root, UI_FONT, 0x444444,
                             LV_ALIGN_TOP_MID, 0, cfg::CONTENT_TOP);
        lv_label_set_text(lblTitle, LV_SYMBOL_BELL " NOTIFS");

        lblList = makeLabel(root, NOTIF_FONT, 0xCCCCCC,
                            LV_ALIGN_TOP_LEFT, 4, LIST_TOP);
        lv_obj_set_width(lblList, LV_HOR_RES - 8);
        lv_label_set_long_mode(lblList, LV_LABEL_LONG_WRAP);
        lv_label_set_text(lblList, "waiting...");
    }

    static void clearAll() {
        portENTER_CRITICAL(&notif::mux);
        notif::count = notif::head = notif::unread = 0;
        portEXIT_CRITICAL(&notif::mux);
        scrollRow = 0;
    }

    // Тап по заголовку (верхняя полоса) — очистить всё
    void tap(int y) {
        if (y < LIST_TOP) clearAll();
    }

    void scroll(int d) {
        scrollRow += d;
        if (scrollRow > lastSn - 1) scrollRow = lastSn - 1;
        if (scrollRow < 0) scrollRow = 0;
        force = true; update();
    }

    void update() {
        static uint32_t last = 0;
        if (!force && millis() - last < 500) return;
        force = false;
        last = millis();

        char tb[48];
        snprintf(tb, sizeof(tb), "%s %s  tap=clear", LV_SYMBOL_BELL,
                 notif::connected ? LV_SYMBOL_BLUETOOTH " on" : "off");
        lv_label_set_text(lblTitle, tb);

        // Снимок под спинлоком (новые сверху)
        struct { char title[28]; char body[80]; uint32_t when; } snap[notif::MAX];
        int sn = 0;
        uint32_t now = millis();
        portENTER_CRITICAL(&notif::mux);
        int c = notif::count, h = notif::head;
        for (int i = 0; i < c; i++) {
            int idx = (h - 1 - i + notif::MAX) % notif::MAX;
            memcpy(snap[sn].title, notif::items[idx].title, sizeof(snap[sn].title));
            memcpy(snap[sn].body,  notif::items[idx].body,  sizeof(snap[sn].body));
            snap[sn].when = notif::items[idx].when;
            sn++;
        }
        portEXIT_CRITICAL(&notif::mux);

        lastSn = sn;
        if (scrollRow > sn - 1) scrollRow = sn > 0 ? sn - 1 : 0;

        char list[600];
        int off = 0;
        if (sn == 0)
            off = snprintf(list, sizeof(list),
                           notif::connected ? "no notifs" : "not connected\nuse Gadgetbridge");
        for (int i = scrollRow; i < sn && i < scrollRow + PER_VIEW && off < (int)sizeof(list) - 1; i++) {
            uint32_t age = (now - snap[i].when) / 1000;
            char agebuf[12];
            if (age < 60)       snprintf(agebuf, sizeof(agebuf), "%lus", (unsigned long)age);
            else if (age < 3600) snprintf(agebuf, sizeof(agebuf), "%lum", (unsigned long)(age / 60));
            else                 snprintf(agebuf, sizeof(agebuf), "%luh", (unsigned long)(age / 3600));
            off += snprintf(list + off, sizeof(list) - off, "%d/%d %s  %s\n%s\n\n",
                            i + 1, sn, snap[i].title[0] ? snap[i].title : "-", agebuf, snap[i].body);
        }
        lv_label_set_text(lblList, list);
    }

    void onEnter() { notif::unread = 0; scrollRow = 0; }   // прочитали
    void onExit()  {}
}


// Эксклюзивное железо. WIFI и BLE делят один радиоблок ESP32 — оба одновременно
// не живут. AUDIO (I2S) и RADIO (SX1262) независимы. GPS тут НЕ учитываем:
// им рулит applyGpsPower по группе Nav, отдельным механизмом.
enum {
    RES_NONE  = 0,
    RES_AUDIO = 1 << 0,
    RES_RADIO = 1 << 1,   // SX1262 (LPD433)
    RES_WIFI  = 1 << 2,
    RES_BLE   = 1 << 3,
};

struct Screen {
    lv_obj_t  **root;        // указатель на tile экрана (заполняется в setup)
    void (*build)(lv_obj_t *parent);
    void (*update)();
    void (*onEnter)();
    void (*onExit)();
    const char *app_name;        // короткое имя для drawer / заголовков
    const char *app_icon;        // LVGL символ (LV_SYMBOL_*)
    const char *group_name;      // имя группы (совпадающие = одна группа)
    uint8_t     needs;           // битмаска RES_*: какое железо захватывает onEnter
};

static Screen screens[] = {
    { &scrClock::root,   scrClock::build,   scrClock::update,   nullptr,             nullptr,            "Clock",   LV_SYMBOL_LOOP,         "Time", RES_NONE },
    { &scrSpeed::root,   scrSpeed::build,   scrSpeed::update,   nullptr,             nullptr,            "Speed",   LV_SYMBOL_GPS,          "Nav", RES_NONE },
    { &scrAudio::root,   scrAudio::build,   scrAudio::update,   scrAudio::onEnter,   scrAudio::onExit,   "Audio",   LV_SYMBOL_AUDIO,        "Audio", RES_AUDIO },
    { &scrLpd::root,     scrLpd::build,     scrLpd::update,     scrLpd::onEnter,     scrLpd::onExit,     "LPD433",  LV_SYMBOL_WIFI,         "Radio", RES_RADIO },
    { &scrGps::root,     scrGps::build,     scrGps::update,     nullptr,             nullptr,            "GPS",     LV_SYMBOL_GPS,          "Nav", RES_NONE },
    { &scrRecon::root,   scrRecon::build,   scrRecon::update,   scrRecon::onEnter,   scrRecon::onExit,   "Recon",   LV_SYMBOL_WIFI,         "WiFi", RES_WIFI },
    { &scrAp::root,      scrAp::build,      scrAp::update,      scrAp::onEnter,      scrAp::onExit,      "APs",     LV_SYMBOL_WIFI,         "WiFi", RES_WIFI },
    { &scrClients::root, scrClients::build, scrClients::update, scrClients::onEnter, scrClients::onExit, "Clients", LV_SYMBOL_WIFI,         "WiFi", RES_WIFI },
    { &scrCsi::root,     scrCsi::build,     scrCsi::update,     scrCsi::onEnter,     scrCsi::onExit,     "CSI",     LV_SYMBOL_WIFI,         "WiFi", RES_WIFI },
    { &scrRadar::root,   scrRadar::build,   scrRadar::update,   scrRadar::onEnter,   scrRadar::onExit,   "Radar",   LV_SYMBOL_WIFI,         "WiFi", RES_WIFI },
    { &scrDeauth::root,  scrDeauth::build,  scrDeauth::update,  scrDeauth::onEnter,  scrDeauth::onExit,  "Deauth",  LV_SYMBOL_WARNING,      "WiFi", RES_WIFI },
    { &scrPktRate::root, scrPktRate::build, scrPktRate::update, scrPktRate::onEnter, scrPktRate::onExit, "PktRate", LV_SYMBOL_WIFI,         "WiFi", RES_WIFI },
    { &scrFinder::root,  scrFinder::build,  scrFinder::update,  scrFinder::onEnter,  scrFinder::onExit,  "Finder",  LV_SYMBOL_GPS,          "WiFi", RES_WIFI },
    { &scrBattery::root, scrBattery::build, scrBattery::update, nullptr,             nullptr,            "Battery", LV_SYMBOL_BATTERY_FULL, "System", RES_NONE },
    { &scrBle::root,     scrBle::build,     scrBle::update,     scrBle::onEnter,     scrBle::onExit,     "BLE",     LV_SYMBOL_BLUETOOTH,    "BLE", RES_BLE },
    { &scrNotif::root,   scrNotif::build,   scrNotif::update,   scrNotif::onEnter,   scrNotif::onExit,   "Notifs",  LV_SYMBOL_BELL,         "BLE", RES_BLE },
};
static const int SCREEN_COUNT = sizeof(screens) / sizeof(screens[0]);

// Индексы экранов в screens[] для читаемости группировки
enum {
    SCR_CLOCK = 0, SCR_SPEED, SCR_AUDIO, SCR_LPD, SCR_GPS,
    SCR_RECON, SCR_AP, SCR_CLIENTS, SCR_CSI, SCR_RADAR, SCR_DEAUTH, SCR_PKT, SCR_FINDER, SCR_BATTERY, SCR_BLE, SCR_NOTIF
};

// GPS питаем пока активен экран Nav-группы. Поллинг из loop вместо onEnter/onExit:
// свайп Speed<->GPS не дёргает питание (gpsPowerOn/Off идемпотентны), а на
// потухшем экране GPS не гасится сам собой — curScreen не меняется.
static void applyGpsPower()
{
    static uint32_t leftAt = 0;
    bool want = (state::curScreen == SCR_SPEED || state::curScreen == SCR_GPS);
    if (want) { leftAt = 0; gpsPowerOn(); return; }
    if (!state::gpsActive) return;
    if (leftAt == 0) leftAt = millis();
    if (millis() - leftAt > cfg::GPS_KEEP_MS) gpsPowerOff();  // не рвём фикс на каждом свайпе
}

// ════════════════════════════════════════════════════════════════════════════
//  ГРУППЫ ЭКРАНОВ (2D сетка: вертикаль = группа, горизонталь = экран в группе)
//  Добавить экран: вписать в screens[] с нужным group_name. buildGroups() строит группы автоматически.
// ════════════════════════════════════════════════════════════════════════════

struct Group {
    const char *name;
    const char *icon;   // иконка группы = иконка первого экрана в ней
    int  screen[8];   // индексы в screens[]
    int  count;
};

// Динамически построенные группы из screens[].group_name
static Group   groups[16];     // максимум 16 групп
static int     GROUP_COUNT = 0;

static lv_obj_t *tileview;
static lv_obj_t *tiles[SCREEN_COUNT];     // плитка для каждого экрана
static int       tileScreen[SCREEN_COUNT];// tiles[k] -> индекс экрана
static int       tileGroup[SCREEN_COUNT]; // tiles[k] -> группа (row)
static int       tilePos[SCREEN_COUNT];   // tiles[k] -> позиция в группе (col)
static int       tileTotal = 0;

// Индикаторы
static lv_obj_t *screenDots[8];               // горизонтальные точки экрана внизу
static lv_obj_t *chevL = nullptr, *chevR = nullptr;   // стрелки краёв группы

static int curGroup = 0, curPos = 0;
static int16_t navPressX = 0, navPressY = 0;   // координаты касания для свайп/тап

// App Drawer (mobile OS)
static bool     drawerOpen  = false;
static lv_obj_t *drawerPanel = nullptr;
static lv_obj_t *drawerIcons[16];    // тап-зоны иконок групп (макс 16)

// ════════════════════════════════════════════════════════════════════════════
//  УПРАВЛЕНИЕ ЭКРАНАМИ (2D tileview)
// ════════════════════════════════════════════════════════════════════════════

static void updateIndicators()
{
    // Точки экранов группы внизу (y=228): активная — «пилюля», остальные — точки.
    int n = groups[curGroup].count;
    const int dot = 5, gap = 7, activeW = 14, y = 228;

    int total = 0;
    for (int i = 0; i < n; i++) total += (i == curPos ? activeW : dot) + (i ? gap : 0);
    int x0 = (LV_HOR_RES - total) / 2;

    int x = x0;
    for (int i = 0; i < 8; i++) {
        if (i < n) {
            int w = (i == curPos ? activeW : dot);
            lv_obj_clear_flag(screenDots[i], LV_OBJ_FLAG_HIDDEN);
            lv_obj_set_size(screenDots[i], w, dot);           // радиус CIRCLE -> пилюля
            lv_obj_set_style_bg_color(screenDots[i],
                lv_color_hex(i == curPos ? 0xFFFFFF : 0x444444), 0);
            lv_obj_set_pos(screenDots[i], x, y);
            x += w + gap;
        } else {
            lv_obj_add_flag(screenDots[i], LV_OBJ_FLAG_HIDDEN);
        }
    }

    // Стрелки: только в ту сторону, где есть следующий экран группы
    if (curPos > 0) {
        lv_obj_clear_flag(chevL, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_pos(chevL, x0 - 20, y - 6);
    } else lv_obj_add_flag(chevL, LV_OBJ_FLAG_HIDDEN);

    if (curPos < n - 1) {
        lv_obj_clear_flag(chevR, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_pos(chevR, x0 + total + 8, y - 6);
    } else lv_obj_add_flag(chevR, LV_OBJ_FLAG_HIDDEN);
}

// ─── Менеджер активного приложения ───────────────────────────────────────────
// Единственная точка входа/выхода. Идемпотентность снимает классы багов:
// повторный onEnter без onExit (был от wakeScreen) и двойной onExit.
static int     activeApp  = -1;   // кто сейчас владеет железом; -1 = никто
static uint8_t heldRes     = 0;   // битмаска RES_* занятого сейчас железа

// Вход в приложение scr. Если уже активно — no-op (защита от двойного onEnter).
static void appEnter(int scr)
{
    if (scr == activeApp) return;
    if (activeApp >= 0 && screens[activeApp].onExit) screens[activeApp].onExit();
    heldRes &= ~(activeApp >= 0 ? screens[activeApp].needs : 0);

    // Конфликт = новое приложение требует железо, которое не отпустил предыдущий
    // onExit. Не должно случаться при парных enter/exit; ловим регрессию.
    if (heldRes & screens[scr].needs) {
        Serial.printf("WARN res leak: held=%d needs=%d app=%s\n",
                      (int)heldRes, (int)screens[scr].needs, screens[scr].app_name);
    }

    activeApp = scr;
    state::curScreen = scr;
    heldRes |= screens[scr].needs;
    if (screens[scr].onEnter) screens[scr].onEnter();
}

// Выход из текущего приложения (для сна). Idempotent.
static void appExit()
{
    if (activeApp < 0) return;
    if (screens[activeApp].onExit) screens[activeApp].onExit();
    heldRes &= ~screens[activeApp].needs;
    activeApp = -1;
}

// Переключение на экран по его tile-индексу k
static void activateTile(int k)
{
    int scr = tileScreen[k];
    curGroup = tileGroup[k];
    curPos   = tilePos[k];
    appEnter(scr);
    updateIndicators();
}

static void tileEventCb(lv_event_t *e)
{
    lv_obj_t *act = lv_tileview_get_tile_act(tileview);
    for (int k = 0; k < tileTotal; k++)
        if (tiles[k] == act) { activateTile(k); break; }
}

// Прыжок на конкретный экран по его индексу (для быстрых переходов)
static void gotoScreen(int scr)
{
    for (int k = 0; k < tileTotal; k++)
        if (tileScreen[k] == scr) {
            lv_obj_set_tile(tileview, tiles[k], LV_ANIM_OFF);  // мгновенно, без прокрутки
            activateTile(k);   // VALUE_CHANGED при ANIM_OFF не гарантирован
            break;
        }
}

// Тап по краю экрана меняет канал на RF-экранах (x-зона: лево=−, право=+)
static void dispatchChannelTap(int x)
{
    int dir = (x < LV_HOR_RES / 3) ? -1 : (x > LV_HOR_RES * 2 / 3) ? +1 : 0;
    if (dir == 0) return;
    state::apSelected = false;          // ручная смена канала снимает цель
    int ch = state::wifiChannel + dir;
    if (ch > 13) ch = 1;
    if (ch < 1)  ch = 13;
    state::wifiChannel = ch;            // общий источник истины
    switch (state::curScreen) {
        case SCR_CSI:    csiRequestChannel(ch); break;
        case SCR_DEAUTH:
        case SCR_PKT:
        case SCR_FINDER: sniff::chanReq = ch; break;
        default: break;
    }
}

static void wakeScreen()
{
    state::screenOff    = false;
    state::screenDimmed = false;
    watch.setBrightness(cfg::BRIGHTNESS_FULL);
    appEnter(state::curScreen);   // парный к appExit() при засыпании
}

// ════════════════════════════════════════════════════════════════════════════
//  APP DRAWER — выдвижная шторка групп в стиле мобильных OS
// ════════════════════════════════════════════════════════════════════════════

static void openDrawer();
static void closeDrawer();

// Создаёт полноэкранную панель drawer поверх tileview через lv_layer_top()
static void buildDrawer()
{
    drawerPanel = lv_obj_create(lv_layer_top());
    lv_obj_set_size(drawerPanel, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_pos(drawerPanel, 0, 0);
    lv_obj_set_style_bg_color(drawerPanel, lv_color_hex(0x0A0A0A), 0);
    lv_obj_set_style_bg_opa(drawerPanel, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(drawerPanel, 0, 0);
    lv_obj_set_style_border_width(drawerPanel, 0, 0);
    lv_obj_set_style_pad_all(drawerPanel, 0, 0);
    lv_obj_add_flag(drawerPanel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(drawerPanel, LV_OBJ_FLAG_SCROLLABLE);

    // Заголовок «APPS» вверху
    lv_obj_t *title = lv_label_create(drawerPanel);
    lv_label_set_text(title, "APPS");
    lv_obj_set_style_text_font(title, UI_FONT, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0x666666), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 6);

    // Лаунчер-сетка без прокрутки: всё на один экран. Не более 2 рядов для ≤8
    // групп — по вертикали просторно, иконки крупные и не режутся.
    const int COLS    = (GROUP_COUNT > 6) ? 4 : 3;
    const int ICON_W  = (COLS == 4) ? 54 : 66;   // ширина с запасом под глиф 48px
    const int CELL_H  = 78;                       // иконка 48 + подпись + воздух
    const int TOP     = 26;
    const int GAP_X   = (LV_HOR_RES - COLS * ICON_W) / (COLS + 1);
    int rows = (GROUP_COUNT + COLS - 1) / COLS;
    int gapY = (LV_VER_RES - TOP - rows * CELL_H) / (rows + 1);
    if (gapY < 6) gapY = 6;

    for (int i = 0; i < GROUP_COUNT; i++) {
        int row      = i / COLS;
        int colInRow = i % COLS;
        int rowItems = GROUP_COUNT - row * COLS;
        if (rowItems > COLS) rowItems = COLS;
        // центрируем неполный последний ряд
        int rowW   = rowItems * ICON_W + (rowItems - 1) * GAP_X;
        int startX = (LV_HOR_RES - rowW) / 2;
        int x = startX + colInRow * (ICON_W + GAP_X);
        int y = TOP + gapY + row * (CELL_H + gapY);

        // Ячейка иконки
        lv_obj_t *cell = lv_obj_create(drawerPanel);
        lv_obj_set_size(cell, ICON_W, CELL_H);
        lv_obj_set_pos(cell, x, y);
        lv_obj_set_style_bg_color(cell, lv_color_hex(0x1E1E1E), 0);
        lv_obj_set_style_bg_opa(cell, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(cell, 14, 0);
        lv_obj_set_style_border_width(cell, 0, 0);
        lv_obj_set_style_pad_all(cell, 0, 0);
        lv_obj_set_style_clip_corner(cell, false, 0);   // глиф не режется радиусом
        lv_obj_clear_flag(cell, LV_OBJ_FLAG_SCROLLABLE);
        drawerIcons[i] = cell;

        // Символ группы
        lv_obj_t *icon = lv_label_create(cell);
        lv_label_set_text(icon, groups[i].icon);
        lv_obj_set_style_text_font(icon, BIG_FONT, 0);
        lv_obj_set_style_text_color(icon, lv_color_hex(0xFFFFFF), 0);
        lv_obj_align(icon, LV_ALIGN_TOP_MID, 0, 6);

        // Подпись
        lv_obj_t *lbl = lv_label_create(cell);
        lv_label_set_text(lbl, groups[i].name);
        lv_obj_set_style_text_font(lbl, UI_FONT, 0);
        lv_obj_set_style_text_color(lbl, lv_color_hex(0xCCCCCC), 0);
        lv_obj_align(lbl, LV_ALIGN_BOTTOM_MID, 0, -4);
    }
}


static void openDrawer()
{
    if (drawerPanel) lv_obj_clear_flag(drawerPanel, LV_OBJ_FLAG_HIDDEN);
    drawerOpen = true;
}

static void closeDrawer()
{
    if (drawerPanel) lv_obj_add_flag(drawerPanel, LV_OBJ_FLAG_HIDDEN);
    drawerOpen = false;
}

// Индикаторы: горизонтальные точки экрана внизу + home indicator + App Drawer
static void buildNavDots()
{
    // Точки экранов — горизонтально внизу, y=228 (над home indicator)
    for (int i = 0; i < 8; i++) {
        screenDots[i] = lv_obj_create(lv_layer_top());
        lv_obj_set_size(screenDots[i], 5, 5);
        lv_obj_set_style_radius(screenDots[i], LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_border_width(screenDots[i], 0, 0);
        lv_obj_set_style_bg_color(screenDots[i], lv_color_hex(0x444444), 0);
        lv_obj_clear_flag(screenDots[i], LV_OBJ_FLAG_CLICKABLE);
    }

    // Home indicator — тонкая белая полоска 60×3px по центру внизу, y=232
    lv_obj_t *homeBar = lv_obj_create(lv_layer_top());
    lv_obj_set_size(homeBar, 60, 3);
    lv_obj_set_pos(homeBar, (LV_HOR_RES - 60) / 2, 232);
    lv_obj_set_style_radius(homeBar, 2, 0);
    lv_obj_set_style_border_width(homeBar, 0, 0);
    lv_obj_set_style_bg_color(homeBar, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_opa(homeBar, (lv_opa_t)(255 * 80 / 100), 0);  // 80% opacity
    lv_obj_clear_flag(homeBar, LV_OBJ_FLAG_CLICKABLE);

    // Стрелки краёв группы — рядом с точками, видны только при наличии соседа
    chevL = lv_label_create(lv_layer_top());
    lv_label_set_text(chevL, LV_SYMBOL_LEFT);
    lv_obj_set_style_text_font(chevL, UI_FONT, 0);
    lv_obj_set_style_text_color(chevL, lv_color_hex(0x888888), 0);
    lv_obj_clear_flag(chevL, LV_OBJ_FLAG_CLICKABLE);

    chevR = lv_label_create(lv_layer_top());
    lv_label_set_text(chevR, LV_SYMBOL_RIGHT);
    lv_obj_set_style_text_font(chevR, UI_FONT, 0);
    lv_obj_set_style_text_color(chevR, lv_color_hex(0x888888), 0);
    lv_obj_clear_flag(chevR, LV_OBJ_FLAG_CLICKABLE);

    // Создаём App Drawer поверх всего
    buildDrawer();
}

// Строит массив groups[] динамически из screens[].group_name
// Вызывать один раз в setup() перед buildTileview()
static void buildGroups()
{
    GROUP_COUNT = 0;
    for (int s = 0; s < SCREEN_COUNT; s++) {
        const char *gname = screens[s].group_name;
        // Ищем существующую группу
        int g = -1;
        for (int i = 0; i < GROUP_COUNT; i++)
            if (strcmp(groups[i].name, gname) == 0) { g = i; break; }
        if (g < 0) {
            // Новая группа
            g = GROUP_COUNT++;
            groups[g].name  = gname;
            groups[g].count = 0;
            // Иконка группы = иконка первого экрана в ней
            groups[g].icon  = screens[s].app_icon;
        }
        if (groups[g].count < 8)
            groups[g].screen[groups[g].count++] = s;
    }
}

// Создаём 2D tileview: row = группа, col = экран в группе.
// Вертикальная навигация только по col 0 (избегаем рваной сетки).
static void buildTileview()
{
    tileview = lv_tileview_create(lv_scr_act());
    lv_obj_set_style_bg_color(tileview, lv_color_black(), 0);
    lv_obj_set_scrollbar_mode(tileview, LV_SCROLLBAR_MODE_OFF);

    int k = 0;
    for (int g = 0; g < GROUP_COUNT; g++) {
        for (int p = 0; p < groups[g].count; p++) {
            // Свайп листает экраны внутри группы. Разрешаем только те стороны,
            // где реально есть сосед — иначе можно «прокрутить» за последний экран.
            int dir = LV_DIR_NONE;
            if (p > 0)                   dir |= LV_DIR_LEFT;
            if (p < groups[g].count - 1) dir |= LV_DIR_RIGHT;
            lv_obj_t *tile = lv_tileview_add_tile(tileview, p, g, (lv_dir_t)dir);
            lv_obj_set_style_pad_all(tile, 0, 0);

            int scr = groups[g].screen[p];
            tiles[k]      = tile;
            tileScreen[k] = scr;
            tileGroup[k]  = g;
            tilePos[k]    = p;
            *screens[scr].root = tile;
            screens[scr].build(tile);
            k++;
        }
    }
    tileTotal = k;

    lv_obj_set_style_anim_duration(tileview, 90, 0);    // снап быстрый и плавный
    lv_obj_add_event_cb(tileview, tileEventCb, LV_EVENT_VALUE_CHANGED, NULL);
}

// ════════════════════════════════════════════════════════════════════════════
//  ФОНОВЫЕ ЗАДАЧИ
// ════════════════════════════════════════════════════════════════════════════

static void taskPomodoro(void *)
{
    for (;;) {
        if (isWorkTime() && (millis() - state::pomStart >= cfg::POMODORO_MS)) {
            uint32_t delta = state::stepCount - state::stepsAtStart;
            if (delta < cfg::POMODORO_STEPS) {
                uint32_t cur = state::stepCount;
                while (state::stepCount == cur) {
                    // Strong sustained buzz burst (DRV2605 effect 47 = Buzz 100%)
                    watch.setHapticEffects(47);
                    for (int i = 0; i < 4; i++) {
                        watch.vibrator();
                        vTaskDelay(pdMS_TO_TICKS(200));
                    }
                    vTaskDelay(pdMS_TO_TICKS(600));
                }
            }
            state::pomStart     = millis();
            state::stepsAtStart = state::stepCount;
        }
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

static void taskAudio(void *)
{
    static int16_t buf[cfg::FFT_N];

    // Инициализация микрофона в контексте этой задачи
    watch.mic.end();
    delay(50);
    watch.mic.setPinsPdmRx(MIC_SCK, MIC_DAT);
    bool ok = watch.mic.begin(I2S_MODE_PDM_RX, (uint32_t)cfg::FFT_SAMPLE_HZ,
                              I2S_DATA_BIT_WIDTH_16BIT,
                              I2S_SLOT_MODE_MONO);
    Serial.printf("[AUDIO] mic.begin = %d @ %dHz\n", ok, (int)cfg::FFT_SAMPLE_HZ);

    for (;;) {
        size_t rd = watch.mic.readBytes((char *)buf, cfg::FFT_N * sizeof(int16_t));
        size_t n = rd / sizeof(int16_t);

        if (n > 0) {
            float mean = 0;
            for (size_t i = 0; i < n; i++) mean += buf[i];
            mean /= n;
            for (int i = 0; i < cfg::FFT_N; i++) {
                vReal[i] = (i < (int)n) ? (buf[i] - mean) : 0.0f;
                vImag[i] = 0.0f;
            }
            FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
            FFT.compute(FFTDirection::Forward);
            FFT.complexToMagnitude();
        }
        vTaskDelay(pdMS_TO_TICKS(5));   // темп задаёт набор окна (~21мс на 1024@48к)
    }
}

static void taskLPD(void *)
{
    for (;;) {
        for (int i = 0; i < cfg::LPD_CHANS; i++) {
            // SetRfFrequency валиден только в standby (не в RX). calibrate=false —
            // образ откалиброван один раз в onEnter, хоп быстрый.
            radio.standby();
            radio.setFrequency(cfg::LPD_START_MHZ + i * cfg::LPD_STEP_MHZ, false);
            radio.startReceive();
            delayMicroseconds(cfg::LPD_SETTLE_US);   // оседание RX/AGC
            state::lpdRssi[i] = (int16_t)radio.getRSSI(false);
            if ((i & 0x0F) == 0) vTaskDelay(1);      // кормим WDT/idle на core 0
        }
        state::lpdDirty = true;
    }
}

// ════════════════════════════════════════════════════════════════════════════
//  КОЛБЭК СОБЫТИЙ
// ════════════════════════════════════════════════════════════════════════════

static void deviceEventCb(DeviceEvent_t event, void *params, void *user_data)
{
    if (event == SENSOR_EVENT &&
        watch.getSensorEventType(params) == SENSOR_STEPS_UPDATED)
    {
        state::stepCount = watch.sensor.getPedometerCounter();
    }
}

// ════════════════════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════════════════════

void setup()
{
    Serial.begin(115200);

    watch.begin();
    watch.setBrightness(cfg::BRIGHTNESS_FULL);
    // Микрофон инициализируется в taskAudio (контекст задачи нужен для I2S)

    watch.sensor.configAccelerometer();
    watch.sensor.enableAccelerometer();
    watch.sensor.enablePedometer();
    watch.sensor.resetPedometer();
    watch.sensor.enableFeature(SensorBMA423::FEATURE_TILT, true);   // «поднятие руки»
    watch.sensor.enableTiltIRQ();
    watch.onEvent(deviceEventCb);
    watch.setHapticEffects(14);

    beginLvglHelper(watch);

    buildStatusbar();
    buildGroups();     // динамически строит группы из screens[].group_name
    buildTileview();   // создаёт плитки и строит все экраны
    buildNavDots();

    state::pomStart     = millis();
    state::lastActivity = millis();

    lv_indev_t *indev = lv_indev_get_next(NULL);
    if (indev) {
        // Нажатие: запоминаем точку, будим экран
        lv_indev_add_event_cb(indev, [](lv_event_t *) {
            lv_point_t p; lv_indev_get_point(lv_indev_get_act(), &p);
            navPressX = p.x; navPressY = p.y;
            state::lastActivity = millis();
            if (state::screenOff) wakeScreen();
        }, LV_EVENT_PRESSED, NULL);

        // Отпускание: мобильная OS схема навигации.
        // Горизонтальный свайп обрабатывает сам tileview (экраны в группе).
        lv_indev_add_event_cb(indev, [](lv_event_t *) {
            lv_point_t p; lv_indev_get_point(lv_indev_get_act(), &p);
            int dx = p.x - navPressX;
            int dy = p.y - navPressY;
            bool vert = abs(dy) > abs(dx);
            // Экраны со списками: вертикальный драг = прокрутка.
            // Выход с них — кнопкой BOOT (домой) или тапом по шапке (recon: уровень вверх).
            bool listScr = (state::curScreen == SCR_RECON || state::curScreen == SCR_NOTIF);

            if (drawerOpen) {
                if (dy > 40 && vert) { closeDrawer(); return; }
                if (abs(dx) < 20 && abs(dy) < 20) {
                    bool hit = false;
                    for (int i = 0; i < GROUP_COUNT; i++) {
                        if (!drawerIcons[i]) continue;
                        lv_area_t area; lv_obj_get_coords(drawerIcons[i], &area);
                        if (navPressX >= area.x1 && navPressX <= area.x2 &&
                            navPressY >= area.y1 && navPressY <= area.y2) {
                            gotoScreen(groups[i].screen[0]); closeDrawer(); hit = true; break;
                        }
                    }
                    if (!hit) closeDrawer();
                }
                return;
            }

            if (dy < -40 && vert && navPressY > LV_VER_RES - 40) {
                openDrawer();                            // свайп вверх от нижнего края
            } else if (listScr && vert && abs(dy) > 25) {
                int rows = -dy / 22;                     // тащим вверх -> вниз по списку
                if (rows == 0) rows = (dy < 0) ? 1 : -1;
                if (state::curScreen == SCR_RECON) scrRecon::scroll(rows);
                else                               scrNotif::scroll(rows);
            } else if (dy > 40 && vert) {
                if (state::curScreen == SCR_RECON && scrRecon::back()) { /* уровень вверх */ }
                else gotoScreen(SCR_CLOCK);
            } else if (abs(dx) < 20 && abs(dy) < 20) {
                if      (state::curScreen == SCR_RECON) scrRecon::tap(navPressY);
                else if (state::curScreen == SCR_AP)    scrAp::tapSelect(navPressY);
                else if (state::curScreen == SCR_NOTIF) scrNotif::tap(navPressY);
                else if (state::curScreen == SCR_CLOCK) { if (notif::count > 0) gotoScreen(SCR_NOTIF); }
                else                                    dispatchChannelTap(navPressX);
            }
        }, LV_EVENT_RELEASED, NULL);
    }

    // Кнопка GPIO0/BOOT:
    //   потухший экран → wakeScreen
    //   drawer открыт  → закрыть drawer
    //   иначе          → домой (SCR_CLOCK)
    attachInterrupt(0, []() IRAM_ATTR {
        static uint32_t last = 0;
        uint32_t now = millis();
        if (now - last > 300) {
            last = now;
            state::lastActivity = now;
            state::scrChanged = true;   // обработается в loop
        }
    }, FALLING);

    setenv("TZ", cfg::TZ, 1);
    tzset();

    // 4.8, 5.8, 7.3, 9.7, 11.7, 14.6, 19.5, 23.4, 29.3, 39.0
    radio.beginFSK(434.0, 0.6, 5, 14.6);
    radio.sleep();

    // GPS выключаем — стартовый экран часы, GPS не нужен.
    // Включится в onEnter спидометра/GPS-экрана.
    watch.powerControl(POWER_GPS, false);
    state::gpsActive = false;

    // WiFi инициализируем один раз, но радио сразу останавливаем (экономия).
    // На CSI-экране радио запускается через esp_wifi_start (без полного deinit,
    // иначе при переинициализации меняется раскладка поднесущих CSI).
    WiFi.mode(WIFI_AP);
    esp_wifi_stop();

    bleInit();      // поднимаем NimBLE один раз
    notifInit();    // постоянный сервер уведомлений (advertising с загрузки)

    xTaskCreatePinnedToCore(taskPomodoro, "pom",   2048, NULL, 1, NULL,    0);
    xTaskCreatePinnedToCore(taskAudio,    "audio", 4096, NULL, 2, &hAudio, 1);
    xTaskCreatePinnedToCore(taskLPD,      "lpd",   4096, NULL, 1, &hLPD,   0);
    vTaskSuspend(hAudio);
    vTaskSuspend(hLPD);

    // tileview уже на активном экране (группа 0, экран 0 = часы).
    activateTile(0);
}

// ════════════════════════════════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════════════════════════════════

static void manageBrightness()
{
    uint32_t idle = millis() - state::lastActivity;
    if (idle >= cfg::SCREEN_OFF_MS) {
        if (!state::screenOff) {
            state::screenOff    = true;
            state::screenDimmed = false;
            watch.setBrightness(0);
            appExit();   // приложение само гасит свои ресурсы (audio/lpd/wifi/ble)
            // GPS НЕ трогаем: его питанием рулит applyGpsPower по группе Nav,
            // чтобы на велосипеде при потухшем экране не было холодного старта.
        }
    } else if (idle >= cfg::SCREEN_DIM_MS) {
        if (!state::screenDimmed) {
            state::screenDimmed = true;
            watch.setBrightness(cfg::BRIGHTNESS_DIM);
        }
    } else if (state::screenDimmed) {
        state::screenDimmed = false;
        watch.setBrightness(cfg::BRIGHTNESS_FULL);
    }
}

// Жест-побудка: быстрый поворот кисти «туда-обратно». Ось уходит за порог в
// одну сторону и возвращается за другую в окне времени — одиночный наклон
// (тряска на велосипеде) не срабатывает.
namespace gesture {
    static const float    FLIP_TH  = 0.6f;   // порог отклонения оси, доля g
    static const uint32_t FLIP_WIN = 700;    // окно «туда-обратно», мс
    static int      dir0 = 0;
    static uint32_t t0   = 0;

    static bool flip(float a) {
        uint32_t now = millis();
        int d = (a > FLIP_TH) ? 1 : (a < -FLIP_TH) ? -1 : 0;
        if (d == 0) return false;
        if (dir0 == 0 || now - t0 > FLIP_WIN) { dir0 = d; t0 = now; return false; }
        if (d == -dir0) { dir0 = 0; return true; }   // вернулись в другую сторону
        return false;
    }
}

static void pollWakeGesture()
{
    static uint32_t last = 0;
    if (millis() - last < 40) return;   // ~25Гц хватает на быстрый жест
    last = millis();

    bool wake = false;

    // Аппаратный детектор наклона BMA423 — поднятие руки. Статус латчится,
    // редкий опрос его не теряет.
    if (cfg::WAKE_ON_TILT) {
        watch.sensor.readIrqStatus();
        if (watch.sensor.isTilt()) wake = true;
    }

    // Программный переворот «туда-обратно» — нужна частая выборка
    if (cfg::WAKE_ON_FLIP) {
        int16_t x, y, z;
        if (watch.sensor.getAccelerometer(x, y, z)) {
            float mag = sqrtf((float)x * x + (float)y * y + (float)z * z);
            if (mag >= 1 && gesture::flip((float)x / mag)) wake = true;  // ось X
        }
    }

    if (wake) {
        state::lastActivity = millis();           // снимает и dim
        if (state::screenOff) wakeScreen();
    }
}

void loop()
{
    applyGpsPower();
    readGPS();
    batterySample();
    snifferHopTick();
    snifferApplyChanReq();
    recon::hopTick();
    watch.loop();
    lv_task_handler();

    if (state::screenOff) pollWakeGesture();   // на включённом экране жест не нужен
    manageBrightness();

    if (notif::arrived) {                  // пришло уведомление
        notif::arrived = false;
        state::lastActivity = millis();
        if (state::screenOff) wakeScreen();
        if (cfg::NOTIF_VIBRO) watch.setHapticEffects(47);   // короткий buzz
        if (cfg::NOTIF_BEEP)  notifBeep();
    }

    notifServiceTick();                    // батарея/шаги в Gadgetbridge

    if (notif::timeSet) {                  // время из CTS (телефон) — местное
        notif::timeSet = false;
        struct tm t = {};
        t.tm_year = notif::tYear - 1900; t.tm_mon = notif::tMon - 1; t.tm_mday = notif::tDay;
        t.tm_hour = notif::tHour; t.tm_min = notif::tMin; t.tm_sec = notif::tSec; t.tm_isdst = 0;
        watch.rtc.setDateTime(notif::tYear, notif::tMon, notif::tDay,
                              notif::tHour, notif::tMin, notif::tSec);
        time_t epoch = mktime(&t);         // TZ=UTC0 -> поля как есть (местное)
        struct timeval tv = { .tv_sec = epoch };
        settimeofday(&tv, NULL);
        state::gpsSynced = true;
    }

    if (state::scrChanged) {
        state::scrChanged = false;
        if (state::screenOff) {
            wakeScreen();
        } else if (drawerOpen) {
            closeDrawer();
        } else {
            gotoScreen(SCR_CLOCK);   // домой
        }
    }

    if (!state::screenOff) {
        if (!drawerOpen) {              // drawer перекрывает экран и статусбар
            updateStatusbar();
            screens[state::curScreen].update();
        }
        delay(20);
    } else {
        // Экран потух — крутим реже, но достаточно часто для wake-жеста.
        // GPS-парсинг не теряем: readGPS вызывается в начале loop.
        delay(60);
    }
}