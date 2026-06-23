// scr_wifi.h — WiFi: единый универсальный инструмент Recon (точки/устройства/
// клиенты/досье; deauth-счётчик и RSSI-бары встроены). Интерфейс для screens[].
#pragma once
#include "core_state.h"

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
