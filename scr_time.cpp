// scr_time.cpp — Time: часы.
#include "scr_time.h"

namespace scrClock {
    lv_obj_t *root;
    static lv_obj_t *lblTime, *lblSec, *lblDate, *lblNotif;
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
