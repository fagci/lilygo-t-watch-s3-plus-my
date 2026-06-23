// scr_system.h — группа System: экран Battery.
// Главный .ino включает это, чтобы собрать таблицу screens[].
#pragma once
#include "core.h"

namespace scrBattery {
    extern lv_obj_t *root;
    void build(lv_obj_t *parent);
    void update();
}
