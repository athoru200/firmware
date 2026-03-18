#include "nrf_common.h"
#include "../../core/mykeyboard.h"

RF24 NRFradio(bruceConfigPins.NRF24_bus.io0, bruceConfigPins.NRF24_bus.cs);
HardwareSerial NRFSerial = HardwareSerial(2); // Uses UART2 for External NRF's
SPIClass *NRFSPI;

// Current radio settings
static uint8_t currentPALevel = 3; // MAX
static uint8_t currentDataRate = 1; // 2Mbps

void nrf_info() {
    tft.fillScreen(bruceConfig.bgColor);
    tft.setTextSize(FM);
    tft.setTextColor(TFT_RED, bruceConfig.bgColor);
    tft.drawCentreString("_Disclaimer_", tftWidth / 2, 10, 1);
    tft.setTextColor(TFT_WHITE, bruceConfig.bgColor);
    tft.setTextSize(FP);
    tft.setCursor(15, 33);
    padprintln("These functions were made to be used in a controlled environment for STUDY only.");
    padprintln("");
    padprintln("DO NOT use these functions to harm people or companies, you can go to jail!");
    tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
    padprintln("");
    padprintln(
        "This device is VERY sensible to noise, so long wires or passing near VCC line can make "
        "things go wrong."
    );
    delay(1000);
    while (!check(AnyKeyPress));
}

// Fungsi baru untuk nonaktifkan device lain di SPI bus
static void disableOtherSPIDevices() {
    // Matikan CC1101 jika ada
    if (bruceConfigPins.CC1101_bus.cs != GPIO_NUM_NC) {
        pinMode(bruceConfigPins.CC1101_bus.cs, OUTPUT);
        digitalWrite(bruceConfigPins.CC1101_bus.cs, HIGH);
    }
    // Matikan W5500 jika ada
    if (bruceConfigPins.W5500_bus.cs != GPIO_NUM_NC) {
        pinMode(bruceConfigPins.W5500_bus.cs, OUTPUT);
        digitalWrite(bruceConfigPins.W5500_bus.cs, HIGH);
    }
    // Matikan SD Card jika ada
    if (bruceConfigPins.SDCARD_bus.cs != GPIO_NUM_NC) {
        pinMode(bruceConfigPins.SDCARD_bus.cs, OUTPUT);
        digitalWrite(bruceConfigPins.SDCARD_bus.cs, HIGH);
    }
}

bool nrf_start(NRF24_MODE mode) {
    bool result = false;
    if (mode == NRF_MODE_DISABLED) return false;

    if (CHECK_NRF_UART(mode)) {
        if (USBserial.getSerialOutput() == &Serial1) {
            displayError("(E) UART already in use", true);
            return false;
        }
        NRFSerial.begin(115200, SERIAL_8N1, bruceConfigPins.uart_bus.rx, bruceConfigPins.uart_bus.tx);
        Serial.println("NRF24 on Serial Started");
        result = true;
    };

    if (!CHECK_NRF_SPI(mode)) return result;
    
    // MATIKAN DEVICE LAIN DI SPI BUS
    disableOtherSPIDevices();
    
    pinMode(bruceConfigPins.NRF24_bus.cs, OUTPUT);
    digitalWrite(bruceConfigPins.NRF24_bus.cs, HIGH);
    pinMode(bruceConfigPins.NRF24_bus.io0, OUTPUT);
    
    // POWER CYCLE YANG BENAR UNTUK MODUL PA+LNA
    digitalWrite(bruceConfigPins.NRF24_bus.io0, LOW);
    delay(10);
    digitalWrite(bruceConfigPins.NRF24_bus.io0, HIGH);
    delay(100); // Lebih lama untuk discharge kapasitor

    // PILIH SPI BUS YANG TEPAT
    if (bruceConfigPins.NRF24_bus.mosi == (gpio_num_t)TFT_MOSI &&
        bruceConfigPins.NRF24_bus.mosi != GPIO_NUM_NC) {
#if TFT_MOSI > 0
        NRFSPI = &tft.getSPIinstance();
#else
        NRFSPI = &SPI;
#endif
    } else if (bruceConfigPins.NRF24_bus.mosi == bruceConfigPins.SDCARD_bus.mosi) {
        NRFSPI = &sdcardSPI;
    } else if (bruceConfigPins.NRF24_bus.mosi == bruceConfigPins.CC1101_bus.mosi &&
               bruceConfigPins.NRF24_bus.mosi != bruceConfigPins.SDCARD_bus.mosi) {
        NRFSPI = &CC_NRF_SPI;
    } else {
        NRFSPI = &SPI;
    }
    
    // INISIALISASI SPI
    NRFSPI->begin(
        (int8_t)bruceConfigPins.NRF24_bus.sck,
        (int8_t)bruceConfigPins.NRF24_bus.miso,
        (int8_t)bruceConfigPins.NRF24_bus.mosi
    );
    delay(10);

    // INISIALISASI RADIO
    if (NRFradio.begin(
            NRFSPI,
            rf24_gpio_pin_t(bruceConfigPins.NRF24_bus.io0),
            rf24_gpio_pin_t(bruceConfigPins.NRF24_bus.cs)
        )) {
        result = true;
        
        // KONFIGURASI DASAR UNTUK JAMMING
        rf24_pa_dbm_e paLevels[] = {RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX};
        rf24_datarate_e dataRates[] = {RF24_1MBPS, RF24_2MBPS, RF24_250KBPS};
        
        NRFradio.setPALevel(paLevels[currentPALevel]);
        NRFradio.setDataRate(dataRates[currentDataRate]);
        NRFradio.setAutoAck(false);
        NRFradio.disableCRC();
        NRFradio.setRetries(0, 0);
        NRFradio.stopListening();
        
        // Flush buffers
        NRFradio.flush_tx();
        NRFradio.flush_rx();
        
        Serial.println("NRF24 initialized successfully for jamming");
    } else {
        Serial.println("NRF24 begin FAILED!");
        return false;
    }
    return result;
}

NRF24_MODE nrf_setMode() {
    NRF24_MODE mode = NRF_MODE_DISABLED;
    options = {
        {"SPI Mode",  [&]() { mode = NRF_MODE_SPI; } },
        {"SPI UART",  [&]() { mode = NRF_MODE_UART; }},
        {"SPI BOTH",  [&]() { mode = NRF_MODE_BOTH; }},
        {"Main Menu", [=]() { returnToMenu = true; } }
    };
    loopOptions(options);
    return mode;
}

// Fungsi untuk mengatur PA Level
void nrf_set_pa_level(uint8_t level) {
    if (level > 3) level = 3;
    currentPALevel = level;
    rf24_pa_dbm_e paLevels[] = {RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX};
    NRFradio.setPALevel(paLevels[level]);
    Serial.printf("PA Level set to: %d\n", level);
}

// Fungsi untuk mengatur Data Rate
void nrf_set_data_rate(uint8_t rate) {
    if (rate > 2) rate = 1;
    currentDataRate = rate;
    rf24_datarate_e dataRates[] = {RF24_1MBPS, RF24_2MBPS, RF24_250KBPS};
    NRFradio.setDataRate(dataRates[rate]);
    Serial.printf("Data Rate set to: %d\n", rate);
}

// Fungsi untuk mendapatkan PA Level saat ini
uint8_t nrf_get_pa_level() {
    return currentPALevel;
}

// Fungsi untuk mendapatkan Data Rate saat ini
uint8_t nrf_get_data_rate() {
    return currentDataRate;
}
