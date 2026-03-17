// File: nrf_pin_config.h
// Tambahkan di folder yang sama dengan nrf_jammer.cpp

#ifndef NRF_PIN_CONFIG_H
#define NRF_PIN_CONFIG_H

// ====================================================
// KONFIGURASI PIN NRF24 UNTUK TTGO T-DISPLAY
// Berdasarkan analisis pinout F6CZV dan board Anda
// ====================================================

// CE Pin - Chip Enable (AMAN: GPIO15 tidak dipakai)
#ifndef NRF24_CE_PIN
#define NRF24_CE_PIN 15
#endif

// CSN Pin - Chip Select (AMAN: GPIO2 tidak dipakai)
#ifndef NRF24_CSN_PIN
#define NRF24_CSN_PIN 2
#endif

// SPI Pins - Sesuai board Anda (dari file pins)
#ifndef NRF24_SCK_PIN
#define NRF24_SCK_PIN 25
#endif

#ifndef NRF24_MOSI_PIN
#define NRF24_MOSI_PIN 26
#endif

#ifndef NRF24_MISO_PIN
#define NRF24_MISO_PIN 27
#endif

#endif
