/*! Adafruit Feather M0 3176 test
*   library reference: https://www.airspayce.com/mikem/arduino/RadioHead/classRH__RF69.html
*   radio setup based on: https://github.com/adafruit/RadioHead/blob/master/examples/feather/RadioHead69_RawDemo_TX/RadioHead69_RawDemo_TX.ino
*   adafruit getting started tutorial: https://learn.adafruit.com/adafruit-feather-m0-radio-with-rfm69-packet-radio/
*   sniffing example: https://www.embeddedrelated.com/showarticle/626.php
*/

#include "rf69.h"

#define BAUDRATE 115200
/* for feather m0  */
#define RF69_CS     8
#define RF69_INT    3
#define RF69_RST    4
#define LED         13
#define VBATPIN     A7

// 862 - 890 MHz
#define RF69_FREQ 868.1

//const uint8_t sync_val[] = {0x00};
//const uint8_t sync_val[] = { 0x6c, 0xb6, 0xcb, 0x2c, 0x92, 0xd9 };
const uint8_t sync_val[] = {0x01};
//const uint8_t MED[] = {0x22, 0x0f, 0xe2, 0x00};
//const uint8_t MED[] = {0x84,0x3c,0x84,0x21,0xe4,0x21,0x08,0x43,0xcf,0x79,0xe7,0x9e,0x79,0x08,0x43,0xc8, 0x04};
const uint8_t preamb[] = {0x01,0x00,0x00,0x00,0x00};
const uint8_t _MED[] = {0x00,0x00,0x00,0x00,0x84,0x3c,0x84,0x21,0xe4,0x21,0x08,0x43,0xcf,0x79,0xe7,0x9e,0x79,0x08,0x43,0xc8,0x40};
const uint8_t MED[] = {0x84,0x3c,0x84,0x21,0xe4,0x21,0x08,0x43,0xcf,0x79,0xe7,0x9e,0x79,0x08,0x43,0xc8,0x40};
#define PACKET_LENGTH

// Singleton instance of the radio driver
//
//RH_RF69 rf69(RFM69_CS, RFM69_INT);
Radio radio(RF69_CS, RF69_INT, RF69_RST);

void setup() {
    Serial.begin(BAUDRATE);
    while (!Serial)
        delay(1); // wait until serial console is open, remove if not tethered to computer
    Serial.println("serial started");

    if(!radio.rf69_init(sizeof(sync_val), 8, sync_val, 2)) {
        Serial.println("radio init failed");
        while(1);
    }

    Serial.println("radio initialised");
}

void loop() {
    radio.rf69_transmit((uint8_t*)_MED, sizeof(_MED), false);
    for(int i = 0; i < 3; i++) {
        delay(8);
        radio.rf69_transmit((uint8_t*)MED, sizeof(MED), true);
    }
    Serial.println("data sent");
    //delay(1000);

    for(int i = 0; i < 10; i++) {
        radio.rf69_receiveDone();
        delay(400);
    }

    // uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    // uint8_t len = sizeof(buf);

    // if (rf69.waitAvailableTimeout(500)) {
    //     // Should be a reply message for us now
    //     if (rf69.recv(buf, &len)) {
    //         Serial.print("got reply: ");
    //         Serial.println((char*)buf);
    //     }
    //     else {
    //         Serial.println("recv failed");
    //     }
    // }
    // else {
    //     Serial.println("No reply, is rf69_server running?");
    // }
    // delay(400);

    // uint8_t data[30];
    // uint8_t data_sz = sizeof data;

    // if (radio.rf69_receiveDone(data, &data_sz)) {
    //     for(int i = 0; i < data_sz; i++) {
    //         Serial.print(data[i]);
    //     }
    // }
}