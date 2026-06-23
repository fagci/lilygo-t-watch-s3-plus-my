// scr_wifi.cpp — WiFi: разведка, точки, клиенты, CSI, радар, deauth, pkt-rate, finder.
#include "scr_wifi.h"

namespace scrCsi {
    lv_obj_t *root;
    static lv_obj_t *lblTitle, *lblMetrics, *lblStatus;
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

namespace scrRadar {
    lv_obj_t *root;
    static lv_obj_t *lblBig, *lblList;
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

namespace scrDeauth {
    lv_obj_t *root;
    static lv_obj_t *lblTitle, *lblStatus, *lblStats;
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

namespace scrPktRate {
    lv_obj_t *root;
    static lv_obj_t *lblTitle, *lblRate, *lblBreak;
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

namespace scrFinder {
    static const int ROWS = 5;
    lv_obj_t *root;
    static lv_obj_t *lblTitle;
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

namespace scrAp {
    lv_obj_t *root;
    static lv_obj_t *lblTitle, *lblList;
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

namespace scrClients {
    lv_obj_t *root;
    static lv_obj_t *lblTitle, *lblBig, *lblList;
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
    void hopTick() {              // из loop (безопасный контекст)
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
    lv_obj_t *root;
    static lv_obj_t *lblTitle, *lblHdr, *lblList;
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
