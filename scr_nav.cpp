// scr_nav.cpp — Nav: спидометр и GPS.
#include "scr_nav.h"

namespace scrSpeed {
    lv_obj_t *root;
    static lv_obj_t *lblSpeed, *lblFix;
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

namespace scrGps {
    lv_obj_t *root;
    static lv_obj_t *lblFix, *lblSats;
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
                            LV_ALIGN_TOP_LEFT, 4, cfg::CONTENT_TOP + 112);
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
                "pDOP: %.1f  acc: %.1fm\n"
                "dist: %.0fm",
                state::gpsFix, state::gpsVisible,
                state::gpsLat, state::gpsLon,
                state::gpsAlt, state::speedKmh,
                state::gpsPdop, state::gpsHacc, state::distanceM);
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
