// scr_radio.h — Radio: LPD433.
// Сгенерировано выделением экранов из главного .ino. Интерфейс для screens[].
#pragma once
#include "core_state.h"

namespace scrLpd {
    extern lv_obj_t *root;
    void build(lv_obj_t *parent);
    void update();
    void onEnter();
    void onExit();
}
