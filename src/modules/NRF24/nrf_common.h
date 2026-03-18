#ifndef __NRF_COMMON_H
#define __NRF_COMMON_H

#include <RF24.h>
#include <globals.h>

// Define the Macros case it hasn't been declared
#ifndef NRF24_CE_PIN
#define NRF24_CE_PIN -1
#endif
#ifndef NRF24_SS_PIN
#define NRF24_SS_PIN -1
#endif

enum NRF24_MODE {
    NRF_MODE_DISABLED, // 0b00
    NRF_MODE_SPI,      // 0b01
    NRF_MODE_UART,     // 0b10
    NRF_MODE_BOTH      // 0b11
};

#define CHECK_NRF_SPI(mode) (mode & NRF_MODE_SPI)
#define CHECK_NRF_UART(mode) (mode & NRF_MODE_UART)
#define CHECK_NRF_BOTH(mode) (mode == NRF_MODE_BOTH)

extern RF24 NRFradio;
extern HardwareSerial NRFSerial; // Uses UART2 for External NRF's

NRF24_MODE nrf_setMode();
bool nrf_start(NRF24_MODE mode);
void nrf_info();

// ============================================================
// FUNGSI BARU UNTUK KONFIGURASI NRF24
// ============================================================

/**
 * @brief Mengatur Power Amplifier (PA) Level
 * @param level 0=MIN, 1=LOW, 2=HIGH, 3=MAX
 */
void nrf_set_pa_level(uint8_t level);

/**
 * @brief Mengatur Data Rate
 * @param rate 0=1Mbps, 1=2Mbps, 2=250Kbps
 */
void nrf_set_data_rate(uint8_t rate);

/**
 * @brief Mendapatkan PA Level saat ini
 * @return uint8_t 0-3
 */
uint8_t nrf_get_pa_level();

/**
 * @brief Mendapatkan Data Rate saat ini
 * @return uint8_t 0-2
 */
uint8_t nrf_get_data_rate();

/**
 * @brief Menampilkan menu konfigurasi NRF24 di layar
 */
void nrf_config_menu();

/**
 * @brief Menyimpan konfigurasi ke LittleFS
 */
void nrf_save_config();

/**
 * @brief Memuat konfigurasi dari LittleFS
 */
void nrf_load_config();

/**
 * @brief Reset konfigurasi ke default
 */
void nrf_reset_config();

#endif // __NRF_COMMON_H
