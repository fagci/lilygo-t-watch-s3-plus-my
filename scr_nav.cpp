// scr_nav.cpp — Nav: спидометр и GPS.
#include "scr_nav.h"

namespace scrSpeed {
    lv_obj_t *root;
    static lv_obj_t *lblSpeed, *lblFix;
    static uint32_t lastUpd = 0;
    void onEnter() { lastUpd = 0; }        // войдя на экран — обновиться сразу
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
        if (millis() - lastUpd < 1000) return;   // спидометр посекундный
        lastUpd = millis();

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
    static uint32_t lastUpd = 0;
    void onEnter() { lastUpd = 0; }        // войдя на экран — обновиться сразу

    // gnssId -> буква созвездия (G GPS, R ГЛОНАСС, E Galileo, B BeiDou,
    // Q QZSS, S SBAS).
    static char gnssLetter(uint8_t id) {
        switch (id) {
            case 0: return 'G'; case 1: return 'S'; case 2: return 'E';
            case 3: return 'B'; case 5: return 'Q'; case 6: return 'R';
            default: return '?';
        }
    }

    void build(lv_obj_t *parent) {
        root = parent;
        lv_obj_set_style_bg_color(root, lv_color_black(), 0);

        lblFix = makeLabel(root, UI_FONT, 0x00FF88,
                           LV_ALIGN_TOP_LEFT, 4, cfg::CONTENT_TOP);
        lv_obj_set_width(lblFix, LV_HOR_RES - 8);
        lv_label_set_long_mode(lblFix, LV_LABEL_LONG_WRAP);
        lv_label_set_text(lblFix, "waiting...");

        // Список спутников ниже шапки
        lblSats = makeLabel(root, UI_FONT, 0xAAAAAA,
                            LV_ALIGN_TOP_LEFT, 4, cfg::CONTENT_TOP + 86);
        lv_obj_set_width(lblSats, LV_HOR_RES - 8);
        lv_label_set_long_mode(lblSats, LV_LABEL_LONG_WRAP);
        lv_label_set_text(lblSats, "");
    }

    void update() {
        if (millis() - lastUpd < 1000) return;   // диагностика посекундная
        lastUpd = millis();

        char buf[256];

        // ── Линк мёртв: показываем диагностику UART, чтобы было НЕ глухо ──
        if (!gnssOk) {
            snprintf(buf, sizeof(buf),
                LV_SYMBOL_GPS " LINK DOWN\n"
                "rx-pin: %d\n"
                "raw: %u B\n"
                "ubx: %s   nmea: %s\n"
                "scanning pins...",
                state::gpsRxPin, state::gpsRawBytes,
                state::gpsSawUbx ? "Y" : "N", state::gpsSawNmea ? "Y" : "N");
            lv_obj_set_style_text_color(lblFix, lv_color_hex(0xFF4444), 0);
            lv_label_set_text(lblFix, buf);
            lv_label_set_text(lblSats,
                state::gpsRawBytes ? "поток есть, ждём UBX..."
                                   : "тишина на UART — ищем бод/питание");
            return;
        }

        // ── Шапка: фикс/поиск + ключевые цифры ──
        if (state::gpsFix >= 2) {
            snprintf(buf, sizeof(buf),
                LV_SYMBOL_GPS " FIX %dD  use:%u/%u\n"
                "%.6f %.6f\n"
                "alt:%.0fm  hdg:%.0f  spd:%.1f\n"
                "hAcc:%.1f vAcc:%.1f sAcc:%.1f\n"
                "pDOP:%.1f dist:%.0fm pv:%u",
                state::gpsFix, state::gpsVisible, state::gpsSivView,
                state::gpsLat, state::gpsLon,
                state::gpsAlt, state::gpsHeading, state::speedKmh,
                state::gpsHacc, state::gpsVacc, state::gpsSacc,
                state::gpsPdop, state::distanceM, state::gpsProtVer);
            lv_obj_set_style_text_color(lblFix, lv_color_hex(0x00FF88), 0);
        } else {
            uint32_t s = millis() / 1000;
            snprintf(buf, sizeof(buf),
                LV_SYMBOL_GPS " NO FIX  use:%u/%u\n"
                "search: %02lu:%02lu\n"
                "pDOP:%.1f  pv:%u  sync:%s",
                state::gpsVisible, state::gpsSivView,
                s / 60, s % 60,
                state::gpsPdop, state::gpsProtVer,
                state::gpsSynced ? "Y" : "N");
            lv_obj_set_style_text_color(lblFix, lv_color_hex(0xFFCC44), 0);
        }
        lv_label_set_text(lblFix, buf);

        // ── Спутники: сводка по созвездиям + список (svId/cno, * = в фиксе) ──
        char sbuf[512];
        int p = 0;
        uint8_t vCnt[8] = {0}, uCnt[8] = {0};
        uint16_t cnoSum = 0; uint8_t cnoMax = 0, cnoUsedN = 0;
        for (uint8_t i = 0; i < state::gpsSatCount; i++) {
            const GpsSat &sv = state::gpsSats[i];
            if (sv.gnss < 8) { vCnt[sv.gnss]++; if (sv.used) uCnt[sv.gnss]++; }
            if (sv.cno > cnoMax) cnoMax = sv.cno;
            if (sv.used) { cnoSum += sv.cno; cnoUsedN++; }
        }
        // строка качества сигнала: лучший и средний C/N0 по используемым спутникам
        p += snprintf(sbuf + p, sizeof(sbuf) - p, "cno max:%u avg:%u dB\n",
                      cnoMax, cnoUsedN ? cnoSum / cnoUsedN : 0);
        // сводная строка по созвездиям с непустым счётчиком (used/view)
        for (uint8_t g = 0; g < 8 && p < (int)sizeof(sbuf) - 16; g++) {
            if (!vCnt[g]) continue;
            p += snprintf(sbuf + p, sizeof(sbuf) - p, "%c%u/%u ",
                          gnssLetter(g), uCnt[g], vCnt[g]);
        }
        p += snprintf(sbuf + p, sizeof(sbuf) - p, "\n");
        // сами спутники, сильные сверху (массив уже отсортирован по cno).
        // Формат "G05:48*" — буква созвездия, svId, cno, * если в фиксе.
        // 3 в ряд; экран узкий, показываем сколько влезает в буфер.
        for (uint8_t i = 0; i < state::gpsSatCount && i < 24 && p < (int)sizeof(sbuf) - 12; i++) {
            const GpsSat &sv = state::gpsSats[i];
            p += snprintf(sbuf + p, sizeof(sbuf) - p, "%c%02u:%02u%c%s",
                          gnssLetter(sv.gnss), sv.sv, sv.cno,
                          sv.used ? '*' : ' ',
                          (i % 3 == 2) ? "\n" : " ");
        }
        if (state::gpsSatCount == 0)
            snprintf(sbuf + p, sizeof(sbuf) - p, "no sats yet");
        lv_label_set_text(lblSats, sbuf);
    }
}
