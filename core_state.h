// core_state.h — разделяемые namespace sniff/ble/notif.
// Переменные inline (C++17): одно определение на всю программу, живёт в заголовке,
// доступно и ядру (core.cpp), и модулям (scr_*.cpp), и главному .ino.
#pragma once
#include "core.h"

// ─── Снифферный учёт (WiFi) ──────────────────────────────────────────────────
namespace sniff {
    inline volatile uint32_t cntTotal = 0, cntMgmt = 0, cntCtrl = 0, cntData = 0;
    inline volatile uint32_t cntProbe = 0, cntBeacon = 0, cntDeauth = 0, cntDisassoc = 0;
    inline volatile uint32_t apFrom = 0, apTo = 0, apDeauth = 0;

    struct MacEntry {
        uint8_t  mac[6];
        int8_t   rssi;
        uint8_t  ch;
        bool     isRandom;
        uint32_t lastSeen;
    };
    inline constexpr int TABLE_SIZE = 128;
    inline MacEntry      table[TABLE_SIZE];
    inline volatile int  count = 0;
    inline portMUX_TYPE  mux = portMUX_INITIALIZER_UNLOCKED;

    // Клиенты выбранной точки (отдельный учёт от общей таблицы MAC)
    struct Client {
        uint8_t  mac[6];
        int8_t   rssi;
        uint32_t lastSeen;
    };
    inline constexpr int CLIENTS_MAX = 64;
    inline Client        clients[CLIENTS_MAX];
    inline volatile int  clientCount = 0;

    inline bool     hopping  = false;
    inline bool     started  = false;   // guard от двойного esp_wifi_start
    inline int      channel  = 1;
    inline uint32_t hopLast  = 0;
    inline int      chanReq  = 0;        // запрос смены канала из тача (0 = нет)
}

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
    inline portMUX_TYPE  mux = portMUX_INITIALIZER_UNLOCKED;

    inline NimBLECharacteristic *chBatt = nullptr;
    inline NimBLECharacteristic *chStep = nullptr;
    inline volatile bool     timeSet = false;
    inline volatile uint16_t tYear = 0;
    inline volatile uint8_t  tMon = 0, tDay = 0, tHour = 0, tMin = 0, tSec = 0;
}
