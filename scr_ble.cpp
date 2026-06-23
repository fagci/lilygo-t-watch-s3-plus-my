// scr_ble.cpp — BLE: сканер и уведомления.
#include "scr_ble.h"

namespace scrBle {
    lv_obj_t *root;
    static lv_obj_t *lblBig, *lblList;
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

namespace scrNotif {
    lv_obj_t *root;
    static lv_obj_t *lblTitle, *lblList;
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
