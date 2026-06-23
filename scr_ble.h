// scr_ble.h — BLE: сканер и уведомления.
// Сгенерировано выделением экранов из главного .ino. Интерфейс для screens[].
#pragma once
#include "core_state.h"

namespace scrBle {
    extern lv_obj_t *root;
    void build(lv_obj_t *parent);
    void update();
    void onEnter();
    void onExit();
}

namespace scrNotif {
    extern lv_obj_t *root;
    void build(lv_obj_t *parent);
    void update();
    void onEnter();
    void onExit();
    void scroll(int d);
    void tap(int y);
}
