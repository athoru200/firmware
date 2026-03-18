/**
 * @file nrf_jammer.cpp
 * @brief Enhanced 2.4 GHz jammer with 12 modes and dual strategy.
 *
 * Features over original:
 *  - 12 jamming mode presets with tuned channel lists
 *  - Data flooding via writeFast() for packet collision attacks
 *  - Constant carrier (CW) for FHSS disruption
 *  - Per-mode configurable PA, data rate, dwell time
 *  - Config persistence via LittleFS
 *  - Random hopping for FHSS targets (BT, Drone)
 *  - Live mode/channel switching during operation
 *  - Improved UI with adaptive layout
 *  - UART support preserved for external NRF modules
 *
 * Hardware: E01-ML01SP2 (NRF24L01+ PA+LNA, +20dBm effective).
 */

#include "nrf_jammer.h"
#include "core/display.h"
#include "core/mykeyboard.h"
#include <LittleFS.h>
#include <globals.h>

// ── Garbage payload for data flooding ───────────────────────────
// 32 bytes maximises TX duty cycle per burst at 2Mbps
static const uint8_t JAM_FLOOD_DATA[32] = {0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0xDE, 0xAD, 0xBE,
                                           0xEF, 0xCA, 0xFE, 0xBA, 0xBE, 0xFF, 0x00, 0xFF, 0x00, 0xA5, 0x5A,
                                           0xA5, 0x5A, 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};

// Pattern khusus untuk BLE (karakter 'x' berulang)
static const uint8_t BLE_JAM_PATTERN[32] = {
    'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
    'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
    'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',
    'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x'
};

// Pattern khusus untuk WiFi (0xAA/0x55)
static const uint8_t WIFI_JAM_PATTERN[32] = {
    0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55,
    0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55,
    0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55,
    0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55
};

// ── FULL CHANNEL LIST (0-124) ─────────────────────────────────
static const uint8_t FULL_CHANNELS[] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
    20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
    40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59,
    60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79,
    80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99,
    100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115,
    116, 117, 118, 119, 120, 121, 122, 123, 124
};

// BLE Advertising channels (37,38,39)
static const uint8_t BLE_ADV_CHANNELS[] = {37, 38, 39};

// BLE Data channels (0-36)
static const uint8_t BLE_DATA_CHANNELS[] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
    20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36
};

// WiFi ch 1,6,11: each spans 22MHz, sub-channels cover bandwidth
static const uint8_t CH_WIFI[] = {
    1,  3,  5,  7,  9,  11, 13, 15, 17, 19, 21, 23, // WiFi ch 1
    26, 28, 30, 32, 34, 36, 38, 40, 42,             // WiFi ch 6
    51, 53, 55, 57, 59, 61, 63, 65, 67, 69, 71, 73  // WiFi ch 11
};

// USB wireless dongles
static const uint8_t CH_USB[] = {40, 50, 60};

// Video streaming (upper ISM band)
static const uint8_t CH_VIDEO[] = {70, 75, 80};

// RC controllers (low channels)
static const uint8_t CH_RC[] = {1, 3, 5, 7};

// Zigbee ch 11-26: 3 nRF sub-channels per Zigbee channel (±1MHz)
static const uint8_t CH_ZIGBEE[] = {
    4,  5,  6,  // ch11
    9,  10, 11, // ch12
    14, 15, 16, // ch13
    19, 20, 21, // ch14
    24, 25, 26, // ch15
    29, 30, 31, // ch16
    34, 35, 36, // ch17
    39, 40, 41, // ch18
    44, 45, 46, // ch19
    49, 50, 51, // ch20
    54, 55, 56, // ch21
    59, 60, 61, // ch22
    64, 65, 66, // ch23
    69, 70, 71, // ch24
    74, 75, 76, // ch25
    79, 80, 81  // ch26
};

// ── Config persistence ──────────────────────────────────────────
static const char *NRF_JAM_CFG_PATH = "/nrf_jam_cfg.bin";
#define NRF_JAM_CFG_VERSION 3

// ── Per-mode default configs ────────────────────────────────────
static NrfJamConfig jamConfigs[NRF_JAM_MODE_COUNT] = {
    /* FULL       */ {3, 1, 0, 0},
    /* WIFI       */ {3, 1, 0, 0},
    /* BLE        */ {3, 1, 0, 0},
    /* BLE_ADV    */ {3, 1, 0, 0},
    /* BLUETOOTH  */ {3, 1, 0, 0},
    /* USB        */ {3, 1, 0, 0},
    /* VIDEO      */ {3, 1, 0, 0},
    /* RC         */ {3, 1, 0, 0},
    /* ZIGBEE     */ {3, 1, 0, 0},
    /* DRONE      */ {3, 1, 0, 0},
};

// ── Mode information table ──────────────────────────────────────
static const NrfJamModeInfo MODE_INFO[NRF_JAM_MODE_COUNT] = {
    {"Full Spectrum",   "Full Spec"},
    {"WiFi 2.4GHz",     "WiFi 2.4"},
    {"BLE Data",        "BLE Data"},
    {"BLE Advertising", "BLE Adv"},  // Mode baru
    {"BT Classic",      "BT Classic"},
    {"USB Dongles",     "USB Dongle"},
    {"Video/FPV",       "Video FPV"},
    {"RC Controllers",  "RC Ctrl"},
    {"Zigbee",          "Zigbee"},
    {"Drone FHSS",      "Drone"},
};

// ── Channel list accessor (MODIFIED) ───────────────────────────
static const uint8_t *getChannelList(NrfJamMode mode, size_t &count) {
    switch (mode) {
        case NRF_JAM_FULL:
        case NRF_JAM_WIFI:
        case NRF_JAM_BLE:
        case NRF_JAM_BLUETOOTH:
        case NRF_JAM_DRONE:
            count = sizeof(FULL_CHANNELS);
            return FULL_CHANNELS;  // Semua mode pake full channel!
            
        case NRF_JAM_BLE_ADV:
            count = sizeof(BLE_ADV_CHANNELS);
            return BLE_ADV_CHANNELS;  // Khusus 3 channel advertising
            
        case NRF_JAM_USB: count = sizeof(CH_USB); return CH_USB;
        case NRF_JAM_VIDEO: count = sizeof(CH_VIDEO); return CH_VIDEO;
        case NRF_JAM_RC: count = sizeof(CH_RC); return CH_RC;
        case NRF_JAM_ZIGBEE: count = sizeof(CH_ZIGBEE); return CH_ZIGBEE;
        
        default: count = 0; return nullptr;
    }
}

// ── Config persistence ──────────────────────────────────────────
static void loadJamConfigs() {
    if (!LittleFS.exists(NRF_JAM_CFG_PATH)) return;

    File f = LittleFS.open(NRF_JAM_CFG_PATH, "r");
    if (!f) return;

    uint8_t version = f.read();
    if (version != NRF_JAM_CFG_VERSION) {
        f.close();
        return;
    }

    size_t expected = sizeof(NrfJamConfig) * NRF_JAM_MODE_COUNT;
    size_t bytesRead = f.read((uint8_t *)jamConfigs, expected);
    f.close();

    if (bytesRead != expected) return;

    // Clamp values
    for (int i = 0; i < NRF_JAM_MODE_COUNT; i++) {
        if (jamConfigs[i].paLevel > 3) jamConfigs[i].paLevel = 3;
        if (jamConfigs[i].dataRate > 2) jamConfigs[i].dataRate = 1;
        if (jamConfigs[i].dwellTimeMs > 200) jamConfigs[i].dwellTimeMs = 200;
        if (jamConfigs[i].useFlooding > 1) jamConfigs[i].useFlooding = 1;
    }
    Serial.println("[JAM] Configs loaded from flash");
}

static void saveJamConfigs() {
    File f = LittleFS.open(NRF_JAM_CFG_PATH, FILE_WRITE);
    if (!f) return;
    f.write(NRF_JAM_CFG_VERSION);
    f.write((uint8_t *)jamConfigs, sizeof(NrfJamConfig) * NRF_JAM_MODE_COUNT);
    f.close();
    Serial.println("[JAM] Configs saved to flash");
}

// ── Apply RF config to hardware ─────────────────────────────────
static void applyJamConfig(const NrfJamConfig &cfg, bool flooding) {
    rf24_pa_dbm_e paLevels[] = {RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX};
    rf24_datarate_e dataRates[] = {RF24_1MBPS, RF24_2MBPS, RF24_250KBPS};

    NRFradio.setPALevel(paLevels[cfg.paLevel & 3]);

    if (!NRFradio.setDataRate(dataRates[cfg.dataRate <= 2 ? cfg.dataRate : 1])) {
        Serial.println("[JAM] Warning: setDataRate failed");
    }

    NRFradio.setAutoAck(false);
    NRFradio.setRetries(0, 0);
    NRFradio.disableCRC();

    if (flooding) {
        NRFradio.setPayloadSize(32);
        NRFradio.setAddressWidth(3);
        uint8_t txAddr[] = {0xE7, 0xE7, 0xE7};
        NRFradio.openWritingPipe(txAddr);
        NRFradio.stopListening();
    }
}

// ── Data flooding on a channel ──────────────────────────────────
static void floodChannel(uint8_t ch, uint16_t dwellMs, bool isBLE) {
    digitalWrite(bruceConfigPins.NRF24_bus.io0, LOW);
    NRFradio.flush_tx();
    NRFradio.setChannel(ch);

    const uint8_t* pattern = isBLE ? BLE_JAM_PATTERN : WIFI_JAM_PATTERN;

    if (dwellMs == 0) {
        // Turbo: fill FIFO (3 packets) and fire
        NRFradio.writeFast(pattern, 32, true);
        NRFradio.writeFast(pattern, 32, true);
        NRFradio.writeFast(pattern, 32, true);
        delayMicroseconds(500);
        return;
    }

    unsigned long startMs = millis();
    while ((millis() - startMs) < dwellMs) {
        if (!NRFradio.writeFast(pattern, 32, true)) { delayMicroseconds(10); }
    }
}

// ── CW initialization helper ────────────────────────────────────
static void initCW(int channel) {
    NRFradio.powerDown();
    delay(100);  // Lebih lama untuk discharge kapasitor
    NRFradio.powerUp();
    delay(5);
    
    NRFradio.setPALevel(RF24_PA_MAX);
    NRFradio.setDataRate(RF24_2MBPS);
    NRFradio.setChannel(channel);
    NRFradio.setAutoAck(false);
    NRFradio.disableCRC();
    NRFradio.setRetries(0, 0);
    NRFradio.stopListening();
    
    NRFradio.flush_tx();
    NRFradio.flush_rx();
    
    NRFradio.startConstCarrier(RF24_PA_MAX, channel);
    
    Serial.printf("CW started on channel %d\n", channel);
}

// ── CW on a channel ─────────────────────────────────────────────
static void cwChannel(uint8_t ch, uint16_t dwellMs) {
    NRFradio.setChannel(ch);
    
    if (dwellMs > 0) {
        // For compatibility, but we'll use non-blocking timing in main loop
    }
}

// ══════════════════════════════════════════════════════════════════
// ═══════════════ CONFIG EDIT UI ═══════════════════════════════
// ══════════════════════════════════════════════════════════════════

static void editModeConfig(NrfJamMode mode) {
    NrfJamConfig &cfg = jamConfigs[(uint8_t)mode];
    const char *paLabels[] = {"MIN (-18dBm)", "LOW (-12dBm)", "HIGH (-6dBm)", "MAX (0/+20dBm)"};
    const char *drLabels[] = {"1 Mbps", "2 Mbps", "250 Kbps"};
    const char *stratLabels[] = {"Constant Carrier", "Data Flooding"};

    int menuIdx = 0;
    bool editing = false;
    bool redraw = true;

    while (true) {
        if (check(EscPress)) {
            saveJamConfigs();
            break;
        }

        if (redraw) {
            drawMainBorderWithTitle(MODE_INFO[(uint8_t)mode].shortName);
            tft.setTextSize(FP);
            tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);

            int y = BORDER_PAD_Y + FM * LH + 4;
            int lineH = max(14, tftHeight / 10);

            const char *items[] = {"PA Level", "Data Rate", "Dwell (ms)", "Strategy", "Save & Back"};
            String values[] = {
                paLabels[cfg.paLevel & 3],
                drLabels[cfg.dataRate <= 2 ? cfg.dataRate : 1],
                String(cfg.dwellTimeMs),
                stratLabels[cfg.useFlooding & 1],
                ""
            };

            for (int i = 0; i < 5; i++) {
                int itemY = y + i * lineH;
                uint16_t fg = (i == menuIdx) ? bruceConfig.bgColor : bruceConfig.priColor;
                uint16_t bg = (i == menuIdx) ? bruceConfig.priColor : bruceConfig.bgColor;

                tft.fillRect(7, itemY, tftWidth - 14, lineH - 2, bg);
                tft.setTextColor(fg, bg);
                String line = String(items[i]);
                if (values[i].length() > 0) { line += ": " + values[i]; }
                tft.drawString(line, 12, itemY + 2, 1);

                if (editing && i == menuIdx && i < 4) {
                    tft.setTextColor(TFT_YELLOW, bg);
                    tft.drawRightString("<>", tftWidth - 12, itemY + 2, 1);
                }
            }
            redraw = false;
        }

        if (check(NextPress)) {
            if (editing) {
                switch (menuIdx) {
                    case 0: cfg.paLevel = (cfg.paLevel + 1) % 4; break;
                    case 1: cfg.dataRate = (cfg.dataRate + 1) % 3; break;
                    case 2: cfg.dwellTimeMs = min(200, (int)cfg.dwellTimeMs + 1); break;
                    case 3: cfg.useFlooding = !cfg.useFlooding; break;
                }
            } else {
                menuIdx = (menuIdx + 1) % 5;
            }
            redraw = true;
        }

        if (check(PrevPress)) {
            if (editing) {
                switch (menuIdx) {
                    case 0: cfg.paLevel = (cfg.paLevel + 3) % 4; break;
                    case 1: cfg.dataRate = (cfg.dataRate + 2) % 3; break;
                    case 2: cfg.dwellTimeMs = max(0, (int)cfg.dwellTimeMs - 1); break;
                    case 3: cfg.useFlooding = !cfg.useFlooding; break;
                }
            } else {
                menuIdx = (menuIdx + 4) % 5;
            }
            redraw = true;
        }

        if (check(SelPress)) {
            if (menuIdx == 4) {
                saveJamConfigs();
                break;
            }
            editing = !editing;
            redraw = true;
        }

        delay(50);
    }
}

// ══════════════════════════════════════════════════════════════════
// ═══════════════ JAMMER STATUS UI ═════════════════════════════
// ══════════════════════════════════════════════════════════════════

static void drawJammerStatus(NrfJamMode mode, int currentCh, uint8_t nrfOnline, bool initial) {
    const NrfJamConfig &cfg = jamConfigs[(uint8_t)mode];

    if (initial) { drawMainBorderWithTitle("NRF JAMMER"); }

    int y = BORDER_PAD_Y + FM * LH + 4;
    int lineH = max(14, tftHeight / 10);

    tft.setTextSize(FP);

    tft.fillRect(7, y, tftWidth - 14, lineH, bruceConfig.bgColor);
    tft.setTextColor(TFT_GREEN, bruceConfig.bgColor);
    tft.drawString(MODE_INFO[(uint8_t)mode].shortName, 12, y + 2, 1);

    y += lineH;

    tft.fillRect(7, y, tftWidth - 14, lineH, bruceConfig.bgColor);
    tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
    char buf[40];
    snprintf(buf, sizeof(buf), "Status: %d ACTIVE", nrfOnline);
    tft.drawString(buf, 12, y + 2, 1);

    y += lineH;

    tft.fillRect(7, y, tftWidth - 14, lineH, bruceConfig.bgColor);
    tft.setTextColor(TFT_YELLOW, bruceConfig.bgColor);
    int freq = 2400 + currentCh;
    snprintf(buf, sizeof(buf), "CH:%d  %dMHz", currentCh, freq);
    tft.drawString(buf, 12, y + 2, 1);

    y += lineH;

    tft.fillRect(7, y, tftWidth - 14, lineH, bruceConfig.bgColor);
    tft.setTextColor(TFT_DARKGREY, bruceConfig.bgColor);
    snprintf(buf, sizeof(buf), "%s dwell:%
