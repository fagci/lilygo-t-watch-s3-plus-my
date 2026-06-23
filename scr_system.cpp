// scr_system.cpp — группа System.
#include "scr_system.h"

namespace scrBattery {
    lv_obj_t *root;
    static lv_obj_t *lblInfo, *chart;
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
