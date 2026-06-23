// scr_wifi.h — WiFi: универсальная разведка (Recon) + дашборды radar/deauth/pkt/finder.
// Интерфейс для screens[].
#pragma once
#include "core_state.h"

namespace scrRadar {
    extern lv_obj_t *root;
    void build(lv_obj_t *parent);
    void update();
    void onEnter();
    void onExit();
}

namespace scrDeauth {
    extern lv_obj_t *root;
    void build(lv_obj_t *parent);
    void update();
    void onEnter();
    void onExit();
}

namespace scrPktRate {
    extern lv_obj_t *root;
    void build(lv_obj_t *parent);
    void update();
    void onEnter();
    void onExit();
}

namespace scrFinder {
    extern lv_obj_t *root;
    void build(lv_obj_t *parent);
    void update();
    void onEnter();
    void onExit();
}

namespace recon {
    void hopTick();
}

namespace scrRecon {
    extern lv_obj_t *root;
    void build(lv_obj_t *parent);
    void update();
    void onEnter();
    void onExit();
    void scroll(int d);
    bool back();
    void tap(int y);
}
