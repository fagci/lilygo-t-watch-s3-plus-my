// scr_nav.h — Nav: спидометр и GPS.
// Сгенерировано выделением экранов из главного .ino. Интерфейс для screens[].
#pragma once
#include "core_state.h"

namespace scrSpeed {
    extern lv_obj_t *root;
    void build(lv_obj_t *parent);
    void update();
    void onEnter();
}

namespace scrGps {
    extern lv_obj_t *root;
    void build(lv_obj_t *parent);
    void update();
    void onEnter();
}
