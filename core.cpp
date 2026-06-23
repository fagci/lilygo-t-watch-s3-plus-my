// core.cpp — определения ядра: state, FFT, общие функции (gps/csi/sniff/batt/sb/notif/ble).
// Объявления — core.h; разделяемые namespace-переменные — core_state.h.
#include "core.h"
#include "core_state.h"

SFE_UBLOX_GNSS gnss;
bool gnssOk = false;

float vReal[cfg::FFT_N];
float vImag[cfg::FFT_N];
ArduinoFFT<float> FFT(vReal, vImag, cfg::FFT_N, cfg::FFT_SAMPLE_HZ);

TaskHandle_t hAudio = NULL;
TaskHandle_t hLPD   = NULL;

lv_obj_t *lbl_sb_left;
lv_obj_t *lbl_sb_mid;
lv_obj_t *lbl_sb_right;

namespace state {
    volatile int      curScreen=0; volatile bool scrChanged=false;
    volatile uint32_t lastActivity=0; bool screenDimmed=false, screenOff=false;
    bool gpsActive=false,gpsSynced=false; float speedKmh=0;
    uint8_t gpsVisible=0,gpsMaxSnr=0; uint32_t gpsChars=0,gpsFailCsum=0,gpsLastCharMs=0;
    uint8_t gpsFix=0; double gpsLat=0,gpsLon=0; float gpsAlt=0,gpsPdop=99.9f;
    double distanceM=0,gpsPrevLat=0,gpsPrevLon=0; bool gpsHasPrev=false;
    uint32_t stepCount=0,pomStart=0,stepsAtStart=0;
    int16_t lpdRssi[cfg::LPD_CHANS]; bool lpdDirty=false;
    volatile bool csiReady=false; int8_t csiRaw[cfg::CSI_MAX_SUBC*2]; int csiLen=0; int8_t csiRssi=0;
    float csiAmp[cfg::CSI_MAX_SUBC],csiPrevAmp[cfg::CSI_MAX_SUBC]; int csiSubc=0;
    float csiFlatness=0,csiMotion=0; uint32_t csiPackets=0;
    int csiChannel=cfg::CSI_CHANNEL,csiChanRequest=0; volatile int wifiChannel=cfg::CSI_CHANNEL;
    uint8_t apBssid[6]={0}; char apSsid[20]=""; volatile bool apSelected=false;
    uint8_t battHist[cfg::BATT_SAMPLES]; int battCount=0; uint32_t battLastSample=0;
}

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

void gpsSyncTime()
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

void gpsPowerOn()
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

void gpsPowerOff()
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
void readGPS()
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

void csiStart()
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

void csiRequestChannel(int ch)
{
    // Дебаунс: не чаще раза в 400мс, иначе драйвер WiFi не успевает
    if (millis() - csiLastChanChange < 400) return;
    if (ch < 1) ch = 13;
    if (ch > 13) ch = 1;
    state::csiChanRequest = ch;
}

void csiApplyChannel()
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

void csiStop()
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

void csiProcess()
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

void batterySample()
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

const char *ouiVendor(const uint8_t *mac)
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

void snifferResetCounters()
{
    sniff::cntTotal = sniff::cntMgmt = sniff::cntCtrl = sniff::cntData = 0;
    sniff::cntProbe = sniff::cntBeacon = sniff::cntDeauth = sniff::cntDisassoc = 0;
    sniff::apFrom = sniff::apTo = sniff::apDeauth = 0;
    portENTER_CRITICAL(&sniff::mux);
    sniff::count = 0;
    sniff::clientCount = 0;
    portEXIT_CRITICAL(&sniff::mux);
}

void snifferStart(bool hop, int fixedCh)
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

void snifferStop()
{
    if (!sniff::started) return;
    esp_wifi_set_promiscuous(false);
    sniff::hopping = false;
    esp_wifi_stop();
    sniff::started = false;
}

// Прыжки по каналам — вызывается из loop (безопасный контекст)
void snifferHopTick()
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
void snifferApplyChanReq()
{
    if (!sniff::chanReq) return;
    int ch = sniff::chanReq;
    sniff::chanReq = 0;
    sniff::channel = ch;
    state::wifiChannel = ch;
    esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
}

// Подсчёт устройств за окно: real = с настоящим OUI, rand = рандомные MAC
void sniffDeviceCount(uint32_t windowMs, int *real, int *rnd)
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

void buildStatusbar()
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

const char *batterySymbol(int pct)
{
    if (watch.pmu.isCharging())   return LV_SYMBOL_CHARGE;
    if (pct > 80) return LV_SYMBOL_BATTERY_FULL;
    if (pct > 55) return LV_SYMBOL_BATTERY_3;
    if (pct > 30) return LV_SYMBOL_BATTERY_2;
    if (pct > 10) return LV_SYMBOL_BATTERY_1;
    return LV_SYMBOL_BATTERY_EMPTY;
}

void updateStatusbar()
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

void notifPush(uint8_t cat, const char *title, const char *body)
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

void notifInit()
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
void notifBeep()
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
void notifServiceTick()
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

void bleInit()
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

void bleStart()
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

void bleStop()
{
    if (!ble::scanning) return;
    bleScan->stop();
    bleScan->clearResults();
    ble::scanning = false;
}

