/*! Adafruit Feather M0 3176 test
*   library reference: https://www.airspayce.com/mikem/arduino/RadioHead/classRH__RF69.html
*   radio setup based on: https://github.com/adafruit/RadioHead/blob/master/examples/feather/RadioHead69_RawDemo_TX/RadioHead69_RawDemo_TX.ino
*   adafruit getting started tutorial: https://learn.adafruit.com/adafruit-feather-m0-radio-with-rfm69-packet-radio/
*   sniffing example: https://www.embeddedrelated.com/showarticle/626.php
*/

#include <Arduino.h>
#include <SPI.h>
#include <RH_RF69.h>

/* for feather m0  */
#define RFM69_CS    8
#define RFM69_INT   3
#define RFM69_RST   4
#define LED         13
#define VBATPIN     A7

// 862 - 890 MHz
#define RF69_FREQ 868.0

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

float measureBat();

void setup() {
    Serial.begin(115200);
    while (!Serial)
        delay(1); // wait until serial console is open, remove if not tethered to computer

    pinMode(LED, OUTPUT);     
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, LOW);

    // manual reset
    digitalWrite(RFM69_RST, HIGH);
    delay(10);
    digitalWrite(RFM69_RST, LOW);
    delay(10);

    if (!rf69.init()) {
        Serial.println("RFM69 radio init failed");
        while (1);
    }
    Serial.println("RFM69 radio init OK!");

    // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
    // No encryption
    if (!rf69.setFrequency(RF69_FREQ))
        Serial.println("setFrequency failed");

    // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
    // ishighpowermodule flag set like this:
    rf69.setTxPower(14, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

    rf69.mode(OOK_Rb1Bw1);
    
    // The encryption key has to be the same as the one in the server
    //uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    //                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    //rf69.setEncryptionKey(key);
    
    pinMode(LED, OUTPUT);

    Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
    }

void loop() {
  // put your main code here, to run repeatedly:
}

float measureBat() {
    float measuredvbat = analogRead(VBATPIN);
    measuredvbat *= 2;    // we divided by 2, so multiply back
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1024; // convert to voltage
    //Serial.print("VBat: " ); 
    //Serial.println(measuredvbat);
    return measuredvbat;
}