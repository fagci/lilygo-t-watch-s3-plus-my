// scr_wifi.h — WiFi: разведка, точки, клиенты, CSI, радар, deauth, pkt-rate, finder.
// Сгенерировано выделением экранов из главного .ino. Интерфейс для screens[].
#pragma once
#include "core_state.h"

namespace scrCsi {
    extern lv_obj_t *root;
    void build(lv_obj_t *parent);
    void update();
    void onEnter();
    void onExit();
}

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

namespace scrAp {
    extern lv_obj_t *root;
    void build(lv_obj_t *parent);
    void update();
    void onEnter();
    void onExit();
    void tapSelect(int y);
}

namespace scrClients {
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
