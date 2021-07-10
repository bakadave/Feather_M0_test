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
#define FREQ 868.1

// Singleton instance of the radio driver
//
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
    if (!rf69.setFrequency(FREQ))
        Serial.println("setFrequency failed");

    // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
    // ishighpowermodule flag set like this:
    rf69.setTxPower(14, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

    //rf69.setModemRegisters(FSK_Rb38_4Fd76_8);
    rf69.setModemConfig(RH_RF69::OOK_Rb9_6Bw19_2);

    // The encryption key has to be the same as the one in the server
    //uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    //                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    //rf69.setEncryptionKey(key);

    pinMode(LED, OUTPUT);

    Serial.print("RFM69 radio @");  Serial.print((int)FREQ);  Serial.println(" MHz");

    /*! @brief rtl_433 output
    *   Guessing modulation: Pulse Width Modulation with multiple packets
    *   Attempting demodulation... short_width: 1228, long_width: 2484, reset_limit: 5180, sync_width: 0
    *   Use a flex decoder with -X 'n=name,m=OOK_PWM,s=1228,l=2484,r=5180,g=2552,t=500,y=0'
    *   pulse_demod_pwm(): Analyzer Device
    *   bitbuffer:: Number of rows: 6
    *   [00] {27} ff 9f 3d c0 : 11111111 10011111 00111101 110
    *   [01] {27} ff 9f 3d c0 : 11111111 10011111 00111101 110
    *   [02] {27} ff 9f 3d c0 : 11111111 10011111 00111101 110
    *   [03] {27} ff 9f 3d c0 : 11111111 10011111 00111101 110
    *   [04] {27} ff 9f 3d c0 : 11111111 10011111 00111101 110
    *   [05] {27} ff 9f 3d c0 : 11111111 10011111 00111101 110
    */
    uint8_t data[4] = {0xFF, 0x9F, 0x3D, 0xC0};
    rf69.send((uint8_t*)data, 4);

    Serial.println("data sent");
}

void loop() {
    uint8_t data[] = {0x22, 0x0F, 0xe2};
    //for(int i = 0; i < 5; i++) {
        rf69.send((uint8_t*)data, sizeof(data));
        rf69.waitPacketSent();
    //}
    Serial.println("data sent");
    delay(1000);

    // if (rf69.available()) {
    //     // Should be a message for us now
    //     uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    //     uint8_t len = sizeof(buf);
    //     if (rf69.recv(buf, &len)) {
    //     if (!len) return;
    //     buf[len] = 0;
    //     Serial.print("Received [");
    //     Serial.print(len);
    //     Serial.print("]: ");
    //     Serial.println((char*)buf);
    //     Serial.print("RSSI: ");
    //     Serial.println(rf69.lastRssi(), DEC);

    //     if (strstr((char *)buf, "Hello World")) {
    //         // Send a reply!
    //         uint8_t data[] = "And hello back to you";
    //         rf69.send(data, sizeof(data));
    //         rf69.waitPacketSent();
    //         Serial.println("Sent a reply");
    //         //Blink(LED, 40, 3); //blink LED 3 times, 40ms between blinks
    //     }
    //     } else {
    //     Serial.println("Receive failed");
    //     }
    // }
}

float measureBat() {
    float measuredvbat = analogRead(VBATPIN);
    measuredvbat *= 2;    // we divided by 2, so multiply back
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1024; // convert to voltage
    return measuredvbat;
}