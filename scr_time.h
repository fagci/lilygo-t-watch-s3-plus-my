// scr_time.h — Time: часы.
// Сгенерировано выделением экранов из главного .ino. Интерфейс для screens[].
#pragma once
#include "core_state.h"

namespace scrClock {
    extern lv_obj_t *root;
    void build(lv_obj_t *parent);
    void update();
}
