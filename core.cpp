// core.cpp — определения ядра: state, FFT, общие функции (gps/oui/batt/sb/notif/ble).
// Объявления — core.h; разделяемые namespace-переменные — core_state.h.
#include "core.h"
#include "core_state.h"

SFE_UBLOX_GNSS_SERIAL gnss;   // v3: UART-вариант класса
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
    uint8_t gpsVisible=0;
    uint8_t gpsFix=0; double gpsLat=0,gpsLon=0; float gpsAlt=0,gpsPdop=99.9f,gpsHacc=0;
    uint32_t gpsBaud=0; int8_t gpsRxPin=-1; uint16_t gpsRawBytes=0; bool gpsSawUbx=false,gpsSawNmea=false;
    uint8_t gpsProtVer=0,gpsSivView=0; GpsSat gpsSats[GPS_SAT_MAX]={}; uint8_t gpsSatCount=0;
    double distanceM=0,gpsPrevLat=0,gpsPrevLon=0; bool gpsHasPrev=false;
    uint32_t stepCount=0,pomStart=0,stepsAtStart=0;
    int16_t lpdRssi[cfg::LPD_CHANS]; bool lpdDirty=false;
    volatile int wifiChannel=1;   // активный WiFi-канал (общий источник истины)
    uint8_t apBssid[6]={0}; char apSsid[20]=""; volatile bool apSelected=false;
    uint8_t battHist[cfg::BATT_SAMPLES]; int battCount=0; uint32_t battLastSample=0;
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

// Конфигурируем модуль ТОЛЬКО через интерфейс ключей (VALSET). MIA-M10Q —
// это протокол 34, где legacy-сообщения UBX-CFG-PRT/RATE/MSG/NAV5 удалены и
// модуль на них отвечает NAK. Поэтому старые хелперы (setUART1Output,
// setNavigationFrequency, setAutoPVT, setDynamicModel) и даже gnss.begin()
// (он зондирует CFG-PRT в isConnected) на M10 молча проваливались — отсюда
// «нет фикса вовсе». Конфиг кладём в RAM-слой: модуль обесточивается по
// GPS_KEEP_MS, флеш не насилуем, на каждом включении настраиваем заново.
static uint32_t gpsLastPvtMs = 0;          // момент последнего NAV-PVT (watchdog)

// Сырая проба UART: читаем напрямую N мс, считаем байты и ищем признаки
// протокола (UBX 0xB5, NMEA '$'). Работает даже если чип НЕ u-blox — на экране
// сразу видно, жив ли модуль и тот ли бод. Бод не подходит → байтов ноль/мусор.
static uint16_t gpsProbeRaw(uint16_t ms, bool &ubx, bool &nmea)
{
    while (Serial1.available()) Serial1.read();   // слить хвост от смены бода
    uint16_t bytes = 0; ubx = false; nmea = false;
    uint32_t t = millis();
    // Окно должно перекрывать период вывода: NMEA по умолчанию идёт пачкой
    // ~1 раз/с, короткое окно может её проспать и зря сменить пины/бод.
    while (millis() - t < ms) {
        while (Serial1.available()) {
            uint8_t b = Serial1.read();
            if (bytes < 0xFFFF) bytes++;
            if (b == 0xB5) ubx  = true;
            if (b == '$')  nmea = true;
        }
        if (bytes >= 80) break;                   // поток явно есть — не ждём всё окно
    }
    return bytes;
}

// Кандидаты распиновки UART. Имена констант — со стороны ЧИПА, поэтому
// «правильный» порядок: ESP RX <- GNSS TX, ESP TX -> GNSS RX. Второй вариант
// (перевёрнутый) — страховка под иную плату/распайку, подберётся пробой.
struct GpsPins { int8_t rx, tx; };
static const GpsPins GPS_PINS[] = {
    { (int8_t)cfg::GPS_PIN_TX, (int8_t)cfg::GPS_PIN_RX },   // ESP RX=41, TX=42
    { (int8_t)cfg::GPS_PIN_RX, (int8_t)cfg::GPS_PIN_TX },   // перевёрнутый
};
static uint8_t gpsPinIdx = 0;

// (Пере)открыть Serial1 на фиксированном боде (MIA-M10Q заводской 38400)
// с текущим кандидатом распиновки.
static void gpsOpenUart()
{
    Serial1.end();
    Serial1.setRxBufferSize(2048);
    Serial1.begin(cfg::GPS_BAUD, SERIAL_8N1,
                  GPS_PINS[gpsPinIdx].rx, GPS_PINS[gpsPinIdx].tx);
    state::gpsBaud  = cfg::GPS_BAUD;
    state::gpsRxPin = GPS_PINS[gpsPinIdx].rx;
}

// Следующий кандидат линка — другая распиновка (бод фиксирован 38400).
static void gpsAdvanceScan()
{
    gpsPinIdx ^= 1;
}

static bool gpsConfigure()
{
    static uint8_t failStreak = 0;

    // 1) Открыть UART (бод фиксирован 38400) и попробовать линк. Тишина →
    //    другая распиновка. На молчащей линии VALSET слать смысла нет.
    gpsOpenUart();
    delay(20);
    bool ubx, nmea;
    uint16_t raw = gpsProbeRaw(1100, ubx, nmea);
    state::gpsRawBytes = raw;
    state::gpsSawUbx = ubx; state::gpsSawNmea = nmea;
    if (raw < 20) {                                // нет потока — следующий кандидат
        gpsAdvanceScan();
        failStreak = 0;
        gnssOk = false;
        return false;
    }

    // 2) begin лишь инициализирует порт/буферы. Его проверку связи не ждём:
    //    на M10 она зондирует CFG-PRT и всегда фейлит (3×maxWait впустую),
    //    результат не нужен — о готовности судим по ACK на VALSET ниже.
    gnss.begin(Serial1, 25, true);

    // 3) Критичная транзакция: вывод UBX по UART1, NAV-PVT и NAV-SAT, частота,
    //    динамическая модель. По её ACK судим, что модуль жив и настроен.
    gnss.newCfgValset(VAL_LAYER_RAM);
    gnss.addCfgValset8 (UBLOX_CFG_UART1OUTPROT_UBX,       1);
    gnss.addCfgValset8 (UBLOX_CFG_UART1OUTPROT_NMEA,      0);   // глушим NMEA-шум
    gnss.addCfgValset8 (UBLOX_CFG_UART1INPROT_UBX,        1);
    gnss.addCfgValset8 (UBLOX_CFG_MSGOUT_UBX_NAV_PVT_UART1, 1); // PVT каждую эпоху
    gnss.addCfgValset8 (UBLOX_CFG_MSGOUT_UBX_NAV_SAT_UART1, 4); // спутники ~1 Гц
    gnss.addCfgValset16(UBLOX_CFG_RATE_MEAS, cfg::GPS_MEAS_MS);
    gnss.addCfgValset16(UBLOX_CFG_RATE_NAV,  cfg::GPS_NAV_RATIO);
    gnss.addCfgValset8 (UBLOX_CFG_NAVSPG_DYNMODEL, DYN_MODEL_PORTABLE);
    bool ok = gnss.sendCfgValset(300);

    // Все созвездия + SBAS — отдельной транзакцией: даже если модуль отвергнет
    // какую-то комбинацию, базовый вывод PVT выше уже применён. M10 тянет
    // GPS+GLONASS+Galileo+BeiDou одновременно, плюс QZSS и SBAS-аугментацию.
    gnss.newCfgValset(VAL_LAYER_RAM);
    gnss.addCfgValset8(UBLOX_CFG_SIGNAL_GPS_ENA,  1);
    gnss.addCfgValset8(UBLOX_CFG_SIGNAL_GAL_ENA,  1);
    gnss.addCfgValset8(UBLOX_CFG_SIGNAL_BDS_ENA,  1);
    gnss.addCfgValset8(UBLOX_CFG_SIGNAL_GLO_ENA,  1);
    gnss.addCfgValset8(UBLOX_CFG_SIGNAL_QZSS_ENA, 1);
    gnss.addCfgValset8(UBLOX_CFG_SIGNAL_SBAS_ENA, 1);
    gnss.sendCfgValset(300);

    // На M10 setAutoPVT/NAVSAT (CFG-MSG) недоступны — вывод уже включён ключами
    // выше. assume* говорит библиотеке «данные приходят сами»: get* лишь
    // разбирают буфер и не шлют legacy-поллы.
    gnss.assumeAutoPVT(true);
    gnss.assumeAutoNAVSAT(true);

    gnssOk = ok;
    if (ok) {
        failStreak = 0;
        gpsLastPvtMs = millis();           // дать модулю время до watchdog-проверки
        state::gpsProtVer = gnss.getProtocolVersionHigh(300);   // тип/версия чипа
        return true;
    }

    // Поток на линии есть, но VALSET не подтвердился. Это либо ещё не очнувшийся
    // модуль (даём ему пару попыток на той же комбинации), либо мусор не с того
    // бода/распиновки — после нескольких неудач всё же двигаем скан.
    if (++failStreak >= 3) { failStreak = 0; gpsAdvanceScan(); }
    return false;
}

void gpsPowerOn()
{
    if (state::gpsActive) return;
    watch.powerControl(POWER_GPS, true);   // питание модуля (BLDO1)
    delay(200);                            // M10 нужно время на старт после подачи питания
    state::gpsActive = true;
    gpsConfigure();                        // сам откроет UART (пины+бод) и настроит
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
    state::gpsHacc = 0;
    state::gpsSivView = 0;
    state::gpsSatCount = 0;
    state::gpsHasPrev = false;             // следующий заход — новый трек
    Serial.println("GPS power OFF");
}

// Снимок UBX-NAV-SAT в state: список спутников (созвездие/svId/cno/used),
// отсортированный по cno (сильные сверху). Вызывается из readGPS.
static void readNavSat()
{
    if (!gnss.getNAVSAT() || gnss.packetUBXNAVSAT == NULL) return;
    uint8_t n = gnss.packetUBXNAVSAT->data.header.numSvs;
    state::gpsSivView = n;
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < n && cnt < GPS_SAT_MAX; i++) {
        UBX_NAV_SAT_block_t &b = gnss.packetUBXNAVSAT->data.blocks[i];
        if (b.cno == 0 && !b.flags.bits.svUsed) continue;   // не тратим место на «пустые»
        state::gpsSats[cnt++] = { b.gnssId, b.svId, b.cno,
                                  (uint8_t)b.flags.bits.svUsed };
    }
    // вставкой по убыванию cno — массив маленький
    for (uint8_t i = 1; i < cnt; i++) {
        GpsSat k = state::gpsSats[i];
        int j = i - 1;
        while (j >= 0 && state::gpsSats[j].cno < k.cno) {
            state::gpsSats[j + 1] = state::gpsSats[j]; j--;
        }
        state::gpsSats[j + 1] = k;
    }
    state::gpsSatCount = cnt;
    gnss.flushNAVSAT();        // пометить прочитанным: getNAVSAT даст true лишь на свежем
}

// Читаем GPS только когда питание включено (экраны 1/4)
void readGPS()
{
    if (!state::gpsActive) return;

    // Не настроились (холодный старт / конфиг не лёг) — периодически пробуем.
    // Сюда же попадаем, если NAV-PVT молчит >5с: значит конфиг не применился.
    if (!gnssOk || (millis() - gpsLastPvtMs > 5000)) {
        static uint32_t lastTry = 0;
        if (millis() - lastTry < 3000) return;
        lastTry = millis();
        gpsConfigure();                    // короткие таймауты внутри — UI не вешаем
        return;
    }

    static uint32_t last = 0;
    if (millis() - last < 250) return;
    last = millis();

    readNavSat();                          // снимок спутников (если пришёл NAV-SAT)

    if (!gnss.getPVT()) return;            // нет свежего решения
    gpsLastPvtMs = millis();

    uint8_t fix = gnss.getFixType();
    state::gpsFix     = fix;
    state::gpsVisible = gnss.getSIV();
    state::gpsPdop    = gnss.getPDOP() / 100.0f;
    state::gpsHacc    = gnss.getHorizontalAccEst() / 1000.0f;   // мм -> м

    if (fix >= 2) {
        double lat = gnss.getLatitude()  * 1e-7;
        double lon = gnss.getLongitude() * 1e-7;
        state::gpsLat   = lat;
        state::gpsLon   = lon;
        state::gpsAlt   = gnss.getAltitudeMSL() / 1000.0f;     // мм -> м

        // Скорость: стоя на месте модуль шумит и иногда выдаёт фантом (192 км/ч).
        // Гасим в ноль, если фикс невалиден или скорость не превышает свою же
        // оценку точности (sAcc) — это и есть «стою на месте».
        int32_t  gs  = gnss.getGroundSpeed();                  // мм/с
        uint32_t sa  = gnss.getSpeedAccEst();                  // оценка точности, мм/с
        if (!gnss.getGnssFixOk() || gs < 0 || (uint32_t)gs <= sa)
            state::speedKmh = 0;
        else
            state::speedKmh = gs * 0.0036f;                    // мм/с -> км/ч

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
//  OUI — поиск вендора по MAC (общий для статусбара и Recon)
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
        // Не перезапускаем рекламу, если BLE намеренно заглушён на время WiFi-снифа
        if (!notif::bleSuspended) NimBLEDevice::startAdvertising();   // снова видимы для реконнекта
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

// Заглушить BLE-радио на время WiFi-снифа (Recon): WiFi и BLE делят одну
// антенну ESP32-S3, и постоянная реклама/коннект уведомлений съедает эфир.
// Снимаем рекламу и рвём активный коннект; onDisconnect не переподнимет рекламу
// из-за флага notif::bleSuspended. Парный bleRadioResume() возвращает рекламу.
void bleRadioSuspend()
{
    if (notif::bleSuspended) return;
    notif::bleSuspended = true;
    bleStop();                                   // на всякий — стоп скана
    NimBLEDevice::stopAdvertising();
    NimBLEServer *srv = NimBLEDevice::getServer();
    if (srv) for (uint16_t h : srv->getPeerDevices()) srv->disconnect(h);
}
void bleRadioResume()
{
    if (!notif::bleSuspended) return;
    notif::bleSuspended = false;
    NimBLEDevice::startAdvertising();
}

