#include "core/powerSave.h"
#include "core/utils.h"
#include <Button.h>

#include <globals.h>
#include <interface.h>
volatile bool nxtPress = false;
volatile bool prvPress = false;
volatile bool ecPress = false;
volatile bool slPress = false;
static void onButtonSingleClickCb1(void *button_handle, void *usr_data) { nxtPress = true; }
static void onButtonDoubleClickCb1(void *button_handle, void *usr_data) { slPress = true; }
static void onButtonHoldCb1(void *button_handle, void *usr_data) { slPress = true; }

static void onButtonSingleClickCb2(void *button_handle, void *usr_data) { prvPress = true; }
static void onButtonDoubleClickCb2(void *button_handle, void *usr_data) { ecPress = true; }
static void onButtonHoldCb2(void *button_handle, void *usr_data) { ecPress = true; }

Button *btn1;
Button *btn2;

/***************************************************************************************
** Function name: _setup_gpio()
** Description:   initial setup for the device
***************************************************************************************/
void _setup_gpio() {
    // setup buttons
    pinMode(DW_BTN, INPUT_PULLUP);
    pinMode(UP_BTN, INPUT_PULLUP);
    button_config_t bt1 = {
        .type = BUTTON_TYPE_GPIO,
        .long_press_time = 600,
        .short_press_time = 120,
        .gpio_button_config = {
                               .gpio_num = DW_BTN,
                               .active_level = 0,
                               },
    };
    button_config_t bt2 = {
        .type = BUTTON_TYPE_GPIO,
        .long_press_time = 600,
        .short_press_time = 120,
        .gpio_button_config = {
                               .gpio_num = UP_BTN,
                               .active_level = 0,
                               },
    };

    btn1 = new Button(bt1);
    btn1->attachSingleClickEventCb(&onButtonSingleClickCb1, NULL);
    btn1->attachDoubleClickEventCb(&onButtonDoubleClickCb1, NULL);
    btn1->attachLongPressStartEventCb(&onButtonHoldCb1, NULL);

    btn2 = new Button(bt2);
    btn2->attachSingleClickEventCb(&onButtonSingleClickCb2, NULL);
    btn2->attachDoubleClickEventCb(&onButtonDoubleClickCb2, NULL);
    btn2->attachLongPressStartEventCb(&onButtonHoldCb2, NULL);

    // setup POWER pin required by the vendor
    pinMode(ADC_EN, OUTPUT);
    digitalWrite(ADC_EN, HIGH);

    // Start with default IR, RF and RFID Configs, replace old
    bruceConfigPins.rfModule = CC1101_SPI_MODULE;
    bruceConfigPins.rfidModule = PN532_I2C_MODULE;

    bruceConfigPins.irRx = RXLED;
    bruceConfigPins.irTx = TXLED;

    Serial.begin(115200);
}

/*********************************************************************
**  Function: setBrightness
**  set brightness value
**********************************************************************/
void _setBrightness(uint8_t brightval) {
    if (brightval == 0) {
        analogWrite(TFT_BL, brightval);
    } else {
        int bl = MINBRIGHT + round(((255 - MINBRIGHT) * brightval / 100));
        analogWrite(TFT_BL, bl);
    }
}

/*********************************************************************
** Function: InputHandler
** Handles the variables PrevPress, NextPress, SelPress, AnyKeyPress and EscPress
**********************************************************************/

void InputHandler(void) {
    static unsigned long tm = 0;
    static bool btn_pressed = false;
    if (nxtPress || prvPress || ecPress || slPress) btn_pressed = true;

    if (millis() - tm > 200 || LongPress) {
        if (btn_pressed) {
            btn_pressed = false;
            tm = millis();
            if (!wakeUpScreen()) AnyKeyPress = true;
            else return;
            SelPress = slPress;
            EscPress = ecPress;
            NextPress = nxtPress;
            PrevPress = prvPress;

            nxtPress = false;
            prvPress = false;
            ecPress = false;
            slPress = false;
        }
    }
}

void powerOff() {
    // Tampilkan menu pilihan
    tft.fillScreen(bruceConfig.bgColor);
    tft.setTextColor(TFT_WHITE, bruceConfig.bgColor);
    tft.setTextSize(1);
    
    tft.drawString("Pilih mode mati:", 20, 40);
    tft.drawString("1. Tekan UP = Sleep (bisa bangun)", 20, 70);
    tft.drawString("2. Tekan DW = Mati Total", 20, 100);
    tft.drawString("3. Tekan SELECT = Batal", 20, 130);
    
    // Tunggu input user (sederhana, loop manual)
    unsigned long startWait = millis();
    bool menuActive = true;
    
    while (menuActive && (millis() - startWait < 10000)) { // timeout 10 detik
        if (prvPress) { // UP_BTN = Sleep biasa
            prvPress = false;
            // Mode sleep biasa (bisa bangun)
            tft.fillScreen(bruceConfig.bgColor);
            tft.drawString("Mode Sleep...", 20, 60);
            delay(1000);
            
            digitalWrite(TFT_BL, LOW);
            tft.writecommand(0x10);
            esp_sleep_enable_ext0_wakeup((gpio_num_t)DW_BTN, BTN_ACT);
            esp_deep_sleep_start();
            return;
        }
        else if (nxtPress) { // DW_BTN = Mati total
            nxtPress = false;
            // Mode mati total
            tft.fillScreen(bruceConfig.bgColor);
            tft.drawString("Mati total...", 20, 60);
            tft.drawString("Tekan RST untuk hidup", 20, 90);
            delay(2000);
            
            digitalWrite(TFT_BL, LOW);
            tft.writecommand(0x10);
            esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
            esp_deep_sleep_start();
            return;
        }
        else if (slPress) { // SELECT = Batal
            slPress = false;
            tft.fillScreen(bruceConfig.bgColor);
            tft.drawString("Dibatalkan", 20, 60);
            delay(1000);
            return; // Kembali ke menu utama
        }
        
        delay(50); // kecilin CPU usage
    }
    
    // Kalau timeout, default ke sleep biasa
    digitalWrite(TFT_BL, LOW);
    tft.writecommand(0x10);
    esp_sleep_enable_ext0_wakeup((gpio_num_t)DW_BTN, BTN_ACT);
    esp_deep_sleep_start();
}
