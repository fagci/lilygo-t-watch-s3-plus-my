// scr_audio.h — Audio: спектр.
// Сгенерировано выделением экранов из главного .ino. Интерфейс для screens[].
#pragma once
#include "core_state.h"

namespace scrAudio {
    extern lv_obj_t *root;
    void build(lv_obj_t *parent);
    void update();
    void onEnter();
    void onExit();
}
