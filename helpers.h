// helpers.h — стабильные утилиты UI/математики. Зависят только от cfg и LVGL.
// Вынесено из основного .ino для сокращения размера. Менять редко.
#pragma once


static inline lv_obj_t *makeBar(lv_obj_t *parent, lv_color_t col)
{
    lv_obj_t *b = lv_obj_create(parent);
    lv_obj_set_style_border_width(b, 0, 0);
    lv_obj_set_style_radius(b, 0, 0);
    lv_obj_set_style_bg_color(b, col, 0);
    lv_obj_set_style_pad_all(b, 0, 0);
    return b;
}

// X-координата левого края i-го бара из n на всю ширину экрана
static inline int barX(int i, int n) { return (int)((float)i * LV_HOR_RES / n); }

// Тепловая палитра t[0..1] -> RGB565: чёрный->синий->циан->зелёный->жёлтый->красный
static inline uint16_t heatColor(float t)
{
    if (t < 0) t = 0; if (t > 1) t = 1;
    float r = 0, g = 0, b = 0;
    if      (t < 0.20f) { float u = t / 0.20f;           b = u; }
    else if (t < 0.40f) { float u = (t - 0.20f) / 0.20f; g = u; b = 1; }
    else if (t < 0.60f) { float u = (t - 0.40f) / 0.20f; g = 1; b = 1 - u; }
    else if (t < 0.80f) { float u = (t - 0.60f) / 0.20f; r = u; g = 1; }
    else                { float u = (t - 0.80f) / 0.20f; r = 1; g = 1 - u; }
    uint16_t r5 = (uint16_t)(r * 31), g6 = (uint16_t)(g * 63), b5 = (uint16_t)(b * 31);
    return (r5 << 11) | (g6 << 5) | b5;
}

static inline lv_obj_t *makeLabel(lv_obj_t *parent, const lv_font_t *font,
                           uint32_t color, lv_align_t align, int x, int y)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(color), 0);
    lv_obj_align(l, align, x, y);
    return l;
}

static inline bool isWorkTime()
{
    struct tm t;
    time_t now = time(NULL);
    localtime_r(&now, &t);
    if (t.tm_wday == 0 || t.tm_wday == 6) return false;
    int mins = t.tm_hour * 60 + t.tm_min;
    return (mins >= cfg::WORK_START && mins < cfg::WORK_END);
}

static inline int nmeaField(const char *s, int n, char *out, int outLen)
{
    int f = 0;
    while (*s && f < n) { if (*s++ == ',') f++; }
    int i = 0;
    while (*s && *s != ',' && *s != '*' && i < outLen - 1)
        out[i++] = *s++;
    out[i] = 0;
    return i;
}

// ─── Геометрия ──────────────────────────────────────────────────────
static inline double haversineM(double lat1, double lon1, double lat2, double lon2)
{
    const double R = 6371000.0, d2r = 3.14159265358979 / 180.0;
    double dlat = (lat2 - lat1) * d2r, dlon = (lon2 - lon1) * d2r;
    double a = sin(dlat / 2) * sin(dlat / 2) +
               cos(lat1 * d2r) * cos(lat2 * d2r) * sin(dlon / 2) * sin(dlon / 2);
    return R * 2 * atan2(sqrt(a), sqrt(1 - a));
}