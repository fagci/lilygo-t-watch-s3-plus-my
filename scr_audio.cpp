// scr_audio.cpp — Audio: спектр.
#include "scr_audio.h"

namespace scrAudio {
    lv_obj_t *root;
    static lv_obj_t *lblPeaks, *canvas;
    static uint16_t *wf       = nullptr;   // буфер водопада в PSRAM, RGB565
    static int       wfStride = 0;         // пикселей в строке (берём у LVGL)

    static const int WF_W      = 240;
    static const int WF_H      = 184;
    static const int STRIDE_PX = 256;      // запас под выравнивание (240*2=480Б)

    // Живой пол шума: тишина садится на пол -> синий, сигнал поднимается по палитре
    static float       dbFloor     = 20.0f;   // сглаженная оценка пола, дБ
    static const float WF_DB_SPAN  = 46.0f;   // окно над полом до «красного»

    void build(lv_obj_t *parent) {
        root = parent;
        lv_obj_set_style_bg_color(root, lv_color_black(), 0);
        lv_obj_set_style_pad_all(root, 0, 0);
        lv_obj_t *t = makeLabel(root, UI_FONT, 0x444444,
                                LV_ALIGN_TOP_MID, 0, cfg::CONTENT_TOP);
        lv_label_set_text(t, LV_SYMBOL_AUDIO " WATERFALL");

        lblPeaks = makeLabel(root, UI_FONT, 0x00FF88,
                             LV_ALIGN_TOP_MID, 0, cfg::CONTENT_TOP + 16);
        lv_label_set_text(lblPeaks, "...");

        canvas = lv_canvas_create(root);
        wf = (uint16_t *)ps_malloc((size_t)WF_H * STRIDE_PX * 2);
        if (wf) {
            memset(wf, 0, (size_t)WF_H * STRIDE_PX * 2);
            lv_canvas_set_buffer(canvas, wf, WF_W, WF_H, LV_COLOR_FORMAT_RGB565);
            lv_obj_align(canvas, LV_ALIGN_BOTTOM_MID, 0, 0);
            // Реальный шаг строки берём у LVGL (зависит от выравнивания)
            lv_image_dsc_t *d = (lv_image_dsc_t *)lv_canvas_get_image(canvas);
            wfStride = d ? (d->header.stride / 2) : WF_W;
            if (wfStride < WF_W || wfStride > STRIDE_PX) wfStride = WF_W;
        }
    }

    void update() {
        if (!wf) return;

        static uint32_t last = 0;
        if (millis() - last < 30) return;   // ~33 строки/сек — читаемый скролл
        last = millis();

        int halfN = cfg::FFT_N / 2;

        // Топ-3 пика по бинам -> текст
        const int NPEAKS = 3;
        int   pkBin[NPEAKS];
        float pkMag[NPEAKS];
        for (int p = 0; p < NPEAKS; p++) { pkBin[p] = -1; pkMag[p] = 0; }
        for (int i = 2; i < halfN - 1; i++) {
            float m = vReal[i];
            if (m <= vReal[i-1] || m <= vReal[i+1]) continue;  // лок. максимум
            for (int p = 0; p < NPEAKS; p++) {
                if (m > pkMag[p]) {
                    for (int q = NPEAKS - 1; q > p; q--) {
                        pkMag[q] = pkMag[q-1];
                        pkBin[q] = pkBin[q-1];
                    }
                    pkMag[p] = m; pkBin[p] = i;
                    break;
                }
            }
        }
        char buf[60];
        int off = 0;
        float binHz = cfg::FFT_SAMPLE_HZ / cfg::FFT_N;
        for (int p = 0; p < NPEAKS; p++) {
            if (pkBin[p] < 0) continue;
            float f = pkBin[p] * binHz;
            if (f >= 1000.0f)
                off += snprintf(buf + off, sizeof(buf) - off,
                                (p == 0) ? "%.1fk" : " %.1fk", f / 1000.0f);
            else
                off += snprintf(buf + off, sizeof(buf) - off,
                                (p == 0) ? "%.0f" : " %.0f", f);
        }
        if (off == 0) snprintf(buf, sizeof(buf), "---");
        lv_label_set_text(lblPeaks, buf);

        // Скролл водопада вниз на 1 строку
        size_t rowB = (size_t)wfStride * 2;
        memmove((uint8_t *)wf + rowB, wf, rowB * (WF_H - 1));

        // Верхняя строка: dB по колонкам, пол — по минимуму кадра
        static float col[WF_W];
        float mn = 1e9f;
        for (int x = 0; x < WF_W; x++) {
            int b0 = 1 + (int)((float)x       * (halfN - 2) / (WF_W - 1));
            int b1 = 1 + (int)((float)(x + 1) * (halfN - 2) / (WF_W - 1));
            if (b1 <= b0) b1 = b0 + 1;
            float m = 0;
            for (int b = b0; b < b1 && b < halfN; b++)
                if (vReal[b] > m) m = vReal[b];
            float dB = 20.0f * log10f(m + 1.0f);
            col[x] = dB;
            if (dB < mn) mn = dB;
        }
        dbFloor = dbFloor * 0.95f + mn * 0.05f;   // медленное сглаживание пола
        for (int x = 0; x < WF_W; x++)
            wf[x] = heatColor((col[x] - dbFloor) / WF_DB_SPAN);
        lv_obj_invalidate(canvas);
    }

    void onEnter() { if (hAudio) vTaskResume(hAudio); }
    void onExit()  { if (hAudio) vTaskSuspend(hAudio); }
}
