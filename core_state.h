// core_state.h — разделяемые namespace ble/notif.
// Переменные inline (C++17): одно определение на всю программу, живёт в заголовке,
// доступно и ядру (core.cpp), и модулям (scr_*.cpp), и главному .ino.
#pragma once
#include "core.h"

// ─── BLE-сканер ──────────────────────────────────────────────────────────────
namespace ble {
    struct Dev {
        char     addr[18];
        char     name[20];
        int8_t   rssi;
        bool     isRandom;
        uint32_t lastSeen;
    };
    inline constexpr int TABLE_SIZE = 64;
    inline Dev          table[TABLE_SIZE];
    inline volatile int count = 0;
    inline portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    inline volatile uint32_t advTotal = 0;
    inline bool         inited   = false;
    inline bool         scanning = false;
}

// ─── Уведомления (Gadgetbridge / InfiniTime) ─────────────────────────────────
namespace notif {
    struct Item { char title[28]; char body[80]; uint32_t when; uint8_t cat; };
    inline constexpr int MAX = 16;
    inline Item          items[MAX];
    inline volatile int  count = 0, head = 0, unread = 0;
    inline volatile bool connected = false;
    inline volatile bool pushNow = false;
    inline volatile bool arrived = false;
    inline volatile bool bleSuspended = false;   // BLE заглушён (WiFi-сниф в Recon)
    inline portMUX_TYPE  mux = portMUX_INITIALIZER_UNLOCKED;

    inline NimBLECharacteristic *chBatt = nullptr;
    inline NimBLECharacteristic *chStep = nullptr;
    inline volatile bool     timeSet = false;
    inline volatile uint16_t tYear = 0;
    inline volatile uint8_t  tMon = 0, tDay = 0, tHour = 0, tMin = 0, tSec = 0;
}
