/*! Adafruit Feather M0 3176 test
*   library reference: https://www.airspayce.com/mikem/arduino/RadioHead/classRH__RF69.html
*   radio setup based on: https://github.com/adafruit/RadioHead/blob/master/examples/feather/RadioHead69_RawDemo_TX/RadioHead69_RawDemo_TX.ino
*   adafruit getting started tutorial: https://learn.adafruit.com/adafruit-feather-m0-radio-with-rfm69-packet-radio/
*   sniffing example: https://www.embeddedrelated.com/showarticle/626.php
*/

#include "rf69.h"

#define BAUDRATE 115200
/* for feather m0  */
#define RFM69_CS    8
#define RFM69_INT   3
#define RFM69_RST   4
#define LED         13
#define VBATPIN     A7

// 862 - 890 MHz
#define RF69_FREQ 868.1

const uint8_t sync_val[] = {0x00};
const uint8_t MED[] = {0x22, 0x0f, 0xe2, 0x00};
#define PACKET_LENGTH

// Singleton instance of the radio driver
//
//RH_RF69 rf69(RFM69_CS, RFM69_INT);
Radio radio(RFM69_CS, RFM69_INT, RFM69_RST);

float measureBat();

void setup() {
    Serial.begin(BAUDRATE);

    if(radio.rf69_init(sizeof(sync_val), 4, sync_val, 2))
        Serial.println("radio initialised");
}

void loop() {
    radio.rf69_transmit(MED, 4, false);
    delay(1000);
}

float measureBat() {
    float measuredvbat = analogRead(VBATPIN);
    measuredvbat *= 2;    // we divided by 2, so multiply back
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1024; // convert to voltage
    return measuredvbat;
}