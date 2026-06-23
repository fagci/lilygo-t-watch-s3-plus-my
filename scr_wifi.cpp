// scr_wifi.cpp — WiFi: единый инструмент Recon (точки/устройства/клиенты/досье),
// со встроенным учётом deauth и RSSI-индикаторами в списке.
#include "scr_wifi.h"

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
        uint64_t tsf;          // TSF из последнего маяка (мкс) = внутренние часы AP
        bool     tsfReset;     // TSF откатился назад -> перезагрузка/подмена маяка
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

    // Рёбра графа: линк STA<->AP с весом по трафику (data-кадры). Основа будущего
    // визуального графа; пока показываем текстом (план LINKS).
    struct Edge { uint8_t sta[6]; uint8_t ap[6]; uint32_t packets; uint32_t lastSeen; };
    static const int EDGE_MAX = 96;
    static Edge edges[EDGE_MAX];
    static volatile int edgeN = 0;

    static volatile bool started = false;
    static volatile bool hopping = true;
    static volatile int  channel = 1;
    static uint32_t hopLast = 0;

    // Учёт атак: deauth/disassoc-фреймы за сессию (из собственного колбэка)
    static volatile uint32_t cntDeauth = 0, cntDisassoc = 0;
    // Сводка по принятым кадрам — понять природу трафика (норм/мусор/далёкие одиночки)
    static volatile uint32_t cntTotal = 0, cntMgmt = 0, cntData = 0, cntCtrl = 0;
    static volatile uint32_t cntBeacon = 0, cntProbe = 0, cntWeak = 0;   // weak = RSSI < -85

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
                      bool beacon = false, uint64_t tsf = 0) {
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
            if (beacon) {
                d.beacon = true; if (enc > d.enc) d.enc = enc;
                // TSF только растёт; откат назад (>1 c) = ребут точки или подменный маяк
                if (tsf) {
                    if (d.tsf && tsf + 1000000ULL < d.tsf) d.tsfReset = true;
                    d.tsf = tsf;
                }
            }
            if (!d.hasBssid) { memcpy(d.bssid, mac, 6); d.hasBssid = true; }
        }
    }

    // Учёт ребра STA<->AP (вызывается под спинлоком из cb на data-кадрах)
    static void edgeTouch(const uint8_t *sta, const uint8_t *ap) {
        if ((sta[0] & 0x01) || (ap[0] & 0x01)) return;   // только юникаст-узлы
        for (int i = 0; i < edgeN; i++)
            if (!memcmp(edges[i].sta, sta, 6) && !memcmp(edges[i].ap, ap, 6)) {
                edges[i].packets++; edges[i].lastSeen = millis(); return;
            }
        int i;
        if (edgeN < EDGE_MAX) i = edgeN++;
        else { uint32_t ot = 0xFFFFFFFF; i = 0;
               for (int k = 0; k < edgeN; k++) if (edges[k].lastSeen < ot) { ot = edges[k].lastSeen; i = k; } }
        memcpy(edges[i].sta, sta, 6); memcpy(edges[i].ap, ap, 6);
        edges[i].packets = 1; edges[i].lastSeen = millis();
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

        // Сводка по природе трафика (без лока — статистика, гонки терпимы)
        cntTotal++;
        if (rssi < -85) cntWeak++;                        // слабый = далёкий/интерференция
        if      (ftype == 0) { cntMgmt++; if (fsub == 8) cntBeacon++; else if (fsub == 4) cntProbe++; }
        else if (ftype == 1) cntCtrl++;
        else if (ftype == 2) cntData++;

        if (ftype == 0 && (fsub == 8 || fsub == 5)) {     // beacon / probe-resp -> AP (подтверждена)
            // Фикс. поля тела: timestamp(8) + beacon interval(2) + capability(2).
            // Без них кадр битый — не плодим фантомные точки.
            if (len < 36) return;
            uint16_t bint = fr[32] | (fr[33] << 8);       // beacon interval, TU
            if (bint < 1 || bint > 10000) return;         // неправдоподобно -> мусор
            char ssid[20]; uint8_t enc = E_OPEN;
            parseBeacon(fr + 24, len - 24, ssid, &enc);
            // TSF — первые 8 байт тела (uint64 LE, мкс). Кламп абсурда (>10 лет = мусор)
            uint64_t tsf = 0;
            for (int b = 0; b < 8; b++) tsf |= (uint64_t)fr[24 + b] << (8 * b);
            if (tsf > 315360000000000ULL) tsf = 0;
            portENTER_CRITICAL(&mux);
            touch(a2, K_AP, rssi, ch, a3, ssid, enc, true, true, tsf);
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
        } else if (ftype == 0 && (fsub == 12 || fsub == 10)) {   // deauth / disassoc
            if (fsub == 12) cntDeauth++; else cntDisassoc++;
            portENTER_CRITICAL(&mux);
            touch(a2, K_UNKNOWN, rssi, ch, nullptr, nullptr, 0, false);
            portEXIT_CRITICAL(&mux);
        } else if (ftype == 2) {                          // data: STA<->AP (AP выводим из трафика)
            uint8_t ds = fr[1] & 3;
            portENTER_CRITICAL(&mux);
            if (ds == 1)      { touch(a2, K_STA, rssi, ch, a1, nullptr, 0, true);
                                touch(a1, K_AP,  rssi, ch, a1, nullptr, 0, true);
                                edgeTouch(a2, a1); }            // STA=a2 -> AP=a1
            else if (ds == 2) { touch(a1, K_STA, rssi, ch, a2, nullptr, 0, true);
                                touch(a2, K_AP,  rssi, ch, a2, nullptr, 0, true);
                                edgeTouch(a1, a2); }            // STA=a1 <- AP=a2
            portEXIT_CRITICAL(&mux);
        } else {                                          // прочее: только обновить известных
            portENTER_CRITICAL(&mux);
            touch(a2, K_UNKNOWN, rssi, ch, nullptr, nullptr, 0, false);
            portEXIT_CRITICAL(&mux);
        }
    }

    static void start(int fixedCh) {
        if (started) return;
        portENTER_CRITICAL(&mux); count = 0; probeN = 0; edgeN = 0; portEXIT_CRITICAL(&mux);
        cntDeauth = cntDisassoc = 0;
        cntTotal = cntMgmt = cntData = cntCtrl = cntBeacon = cntProbe = cntWeak = 0;
        hopping = (fixedCh == 0);
        channel = fixedCh ? fixedCh : 1;
        hopLast = millis();
        // Промискуитет должен работать в STA, а не в AP (на старте выставлен AP):
        // в AP-режиме радио занято собственным маяком, приём кадров рваный.
        esp_wifi_set_mode(WIFI_MODE_STA);
        esp_wifi_start();
        esp_wifi_set_ps(WIFI_PS_NONE);   // КЛЮЧЕВОЕ: без сна модема, иначе приёмник
                                         // дремлет и пропускает большинство кадров
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
    enum View { V_APS, V_DEVS, V_AP, V_STA, V_LINKS, V_STATS };   // +сводка по кадрам
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

    // снимок рёбер графа (для плана LINKS)
    static recon::Edge  edgeSnap[recon::EDGE_MAX];
    static int          edgeSnapN = 0;

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
        edgeSnapN = recon::edgeN;
        memcpy(edgeSnap, recon::edges, edgeSnapN * sizeof(recon::Edge));
        portEXIT_CRITICAL(&recon::mux);

        uint32_t now = millis();
        visN = 0;

        if (view == V_STA) {                   // vis[] индексирует probeSnap (досье: что ищет MAC)
            for (int i = 0; i < probeSnapN; i++)
                if (memcmp(probeSnap[i].mac, selSta, 6) == 0) vis[visN++] = i;
        } else if (view == V_LINKS) {          // vis[] индексирует edgeSnap, сорт по трафику
            for (int i = 0; i < edgeSnapN; i++)
                if (now - edgeSnap[i].lastSeen <= KEEP) vis[visN++] = i;
            for (int i = 1; i < visN; i++) {   // вставками по packets убыванием
                int k = vis[i], j = i - 1;
                while (j >= 0 && edgeSnap[vis[j]].packets < edgeSnap[k].packets) { vis[j+1] = vis[j]; j--; }
                vis[j+1] = k;
            }
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

    // Компактный индикатор RSSI: 4 ячейки '•' (есть сигнал) / '·' (нет).
    // -90dBm и слабее -> пусто, -34dBm и сильнее -> полный (символы из шрифта).
    static void rssiBar(int8_t rssi, char *out, int outsz) {
        const int cells = 4;
        int f = (rssi + 90) / 14;
        if (f < 0) f = 0; if (f > cells) f = cells;
        int o = 0;
        for (int i = 0; i < cells && o < outsz - 3; i++)
            o += snprintf(out + o, outsz - o, "%s", i < f ? "\xE2\x80\xA2" : "\xC2\xB7");
        out[o] = 0;
    }

    // Аптайм точки из TSF (мкс) -> компактно: "3d4h" / "5h12m" / "47m" / "-"
    static void fmtUptime(uint64_t tsf, char *out, int outsz) {
        if (!tsf) { snprintf(out, outsz, "-"); return; }
        uint32_t s = (uint32_t)(tsf / 1000000ULL);
        uint32_t d = s / 86400, h = (s % 86400) / 3600, m = (s % 3600) / 60;
        if (d)      snprintf(out, outsz, "%lud%luh", (unsigned long)d, (unsigned long)h);
        else if (h) snprintf(out, outsz, "%luh%lum", (unsigned long)h, (unsigned long)m);
        else        snprintf(out, outsz, "%lum", (unsigned long)m);
    }

    static void renderHeader(uint32_t now) {
        char t[64], h[120];
        if (view == V_APS) {
            // Счётчик атак deauth/disassoc прямо в титуле (предупреждение, если есть)
            char dw[24] = "";
            uint32_t dt = recon::cntDeauth + recon::cntDisassoc;
            if (dt) snprintf(dw, sizeof(dw), " " LV_SYMBOL_WARNING "%lu", (unsigned long)dt);
            // REFRESH = идёт перебор каналов (хоп) по всему диапазону
            snprintf(t, sizeof(t), LV_SYMBOL_WIFI " " LV_SYMBOL_REFRESH "ch%d (%d)%s " LV_SYMBOL_RIGHT "dev",
                     recon::channel, visN, dw);
            lv_label_set_text(lblTitle, t);
            lv_label_set_text(lblHdr, "");
        } else if (view == V_DEVS) {
            snprintf(t, sizeof(t), LV_SYMBOL_GPS " " LV_SYMBOL_REFRESH "DEVICES (%d) " LV_SYMBOL_RIGHT "link", visN);
            lv_label_set_text(lblTitle, t);
            lv_label_set_text(lblHdr, "");
        } else if (view == V_LINKS) {
            snprintf(t, sizeof(t), LV_SYMBOL_SHUFFLE " " LV_SYMBOL_REFRESH "LINKS (%d) " LV_SYMBOL_RIGHT "stat", visN);
            lv_label_set_text(lblTitle, t);
            lv_label_set_text(lblHdr, "");
        } else if (view == V_STATS) {
            snprintf(t, sizeof(t), LV_SYMBOL_LIST " " LV_SYMBOL_REFRESH "FRAME STATS " LV_SYMBOL_RIGHT "ap");
            lv_label_set_text(lblTitle, t);
            lv_label_set_text(lblHdr, "");
        } else if (view == V_AP) {
            int ai = findMac(selBssid);
            if (ai >= 0) {
                recon::Dev &a = snap[ai];
                char nm[18]; apName(a, nm, sizeof(nm));
                snprintf(t, sizeof(t), LV_SYMBOL_LEFT " %s", nm);
                // [cN] = залочены на канал AP (не хопим); up = аптайм из TSF; ⚠ = откат TSF
                char up[12]; fmtUptime(a.tsf, up, sizeof(up));
                snprintf(h, sizeof(h), "%s%.8s [c%d] %s %ddBm u%d up%s",
                         a.tsfReset ? LV_SYMBOL_WARNING " " : "",
                         ouiVendor(a.mac), a.ch, encOf(a), a.rssi, clientsOf(selBssid, now), up);
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

    // --- план связей: топ линков STA<->AP по трафику (V_LINKS) ---
    if (view == V_LINKS) {
        if (visN == 0) { lv_label_set_text(lblList, "no links yet"); return; }
        for (int r = 0; r < vr && scrollRow + r < visN; r++) {
            recon::Edge &e = edgeSnap[vis[scrollRow + r]];
            uint32_t age = (now - e.lastSeen) / 1000; if (age > 999) age = 999;
            char sl[16]; devLabel(e.sta, sl, sizeof(sl));
            char an[16]; int ai = findMac(e.ap);
            if (ai >= 0) apName(snap[ai], an, sizeof(an));
            else snprintf(an, sizeof(an), "%02X%02X%02X", e.ap[3], e.ap[4], e.ap[5]);

            char buf[80];
            snprintf(buf, sizeof(buf), "%-.13s " LV_SYMBOL_RIGHT " %-.13s\n  %6" PRIu32 "p %3" PRIu32 "s",
                     sl, an, e.packets, age);
            buf[sizeof(buf) - 1] = '\0';
            for (char *p = buf; *p; p++) if (*p == '#') *p = '_';

            bool stale = (now - e.lastSeen) > FRESH;
            int w = snprintf(list + off, LIST_SZ - off, stale ? "~ %s\n" : "%s\n", buf);
            if (w <= 0 || off + w >= LIST_SZ - 1) break;
            off += w;
        }
        lv_label_set_text(lblList, list);
        return;
    }

    // --- сводка по кадрам: природа трафика (V_STATS) ---
    if (view == V_STATS) {
        static uint32_t baseTot = 0, baseMs = 0; static uint32_t rate = 0;
        uint32_t tot = recon::cntTotal;
        if (now - baseMs >= 1000) {                       // темп раз в секунду
            rate = (baseMs && tot >= baseTot) ? (tot - baseTot) : 0;
            baseTot = tot; baseMs = now;
        }
        uint32_t weakPct = tot ? (recon::cntWeak * 100 / tot) : 0;
        snprintf(list, LIST_SZ,
                 "total %lu  (%lu/s)\n"
                 "mgmt %lu  data %lu  ctrl %lu\n"
                 "beacon %lu  probe %lu\n"
                 "deauth %lu  disas %lu\n"
                 "weak<-85: %lu (%lu%%)\n"
                 "nodes %d  links %d",
                 (unsigned long)tot, (unsigned long)rate,
                 (unsigned long)recon::cntMgmt, (unsigned long)recon::cntData, (unsigned long)recon::cntCtrl,
                 (unsigned long)recon::cntBeacon, (unsigned long)recon::cntProbe,
                 (unsigned long)recon::cntDeauth, (unsigned long)recon::cntDisassoc,
                 (unsigned long)recon::cntWeak, (unsigned long)weakPct,
                 snapN, edgeSnapN);
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

        char buf[120];
        buf[0] = '\0';                                    // [FIX 4]
        char bar[16]; rssiBar(d.rssi, bar, sizeof(bar));  // мини-индикатор сигнала (finder)

        if (view == V_APS) {
            char nm[18]; apName(d, nm, sizeof(nm));
            // ⚠ перед именем = у точки откатывался TSF (ребут/подменный маяк)
            snprintf(buf, sizeof(buf), "%s%-.18s\n%s %4d ch%-2d %-4s u%2" PRIu32 " %3" PRIu32 "s",
                     d.tsfReset ? LV_SYMBOL_WARNING " " : "",
                     nm, bar, d.rssi, d.ch, encOf(d),
                     clientsOf(d.mac, now), age);         // [FIX 3] PRIu32 вместо %lu
        } else if (view == V_DEVS) {
            char dl[18]; devLabel(d.mac, dl, sizeof(dl));
            snprintf(buf, sizeof(buf), "%-.18s\n%s %4d %6" PRIu32 "p %2dnet %3" PRIu32 "s",
                     dl, bar, d.rssi, d.packets, pnlCount(d.mac), age);
        } else {   // V_AP — клиенты точки
            char dl[18]; devLabel(d.mac, dl, sizeof(dl));
            snprintf(buf, sizeof(buf), "%-.18s\n%s %4d %6" PRIu32 "p %3" PRIu32 "s",
                     dl, bar, d.rssi, d.packets, age);
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
        if (y < TITLE_BOT) {                           // тап по титулу = цикл верхних планов
            if      (view == V_APS)   { view = V_DEVS;  scrollRow = 0; forceRedraw = true; update(); }
            else if (view == V_DEVS)  { view = V_LINKS; scrollRow = 0; forceRedraw = true; update(); }
            else if (view == V_LINKS) { view = V_STATS; scrollRow = 0; forceRedraw = true; update(); }
            else if (view == V_STATS) { view = V_APS;   scrollRow = 0; forceRedraw = true; update(); }
            else back();
            return;
        }
        if (view == V_STA || view == V_LINKS || view == V_STATS) return;  // нет кликабельных строк
        int lt = listTop();
        if (y < lt) return;                            // мета/график — не реагируем
        int r = (y - lt) / ROW_H + scrollRow;          // строка = два текстовых ряда
        if (r < 0 || r >= visN) return;
        recon::Dev &d = snap[vis[r]];
        if (view == V_APS) {
            memcpy(selBssid, d.mac, 6); view = V_AP; scrollRow = 0;
            recon::setLock(d.ch); resetChart();
            // Drill в точку = выбор глобальной цели для Deauth/PktRate/Finder и сниффера
            state::apSelected = true;
            memcpy(state::apBssid, d.mac, 6);
            apName(d, state::apSsid, sizeof(state::apSsid));
            state::wifiChannel = d.ch;
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
        if (view == V_LINKS){ view = V_APS;  scrollRow = 0; forceRedraw = true; update(); return true; }
        if (view == V_STATS){ view = V_APS;  scrollRow = 0; forceRedraw = true; update(); return true; }
        return false;                                   // V_APS -> домой
    }

    void onEnter() { bleRadioSuspend(); view = V_APS; scrollRow = 0; resetChart(); recon::start(0); }
    void onExit()  { recon::stop(); bleRadioResume(); }
}
