// scr_radio.cpp — Radio: LPD433.
#include "scr_radio.h"

namespace scrLpd {
    lv_obj_t *root;
    static lv_obj_t *lblPeaks, *canvas;
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
