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

#define BAUDRATE 115200

//const uint8_t sync_val[] = {0x00};
//const uint8_t sync_val[] = { 0x6c, 0xb6, 0xcb, 0x2c, 0x92, 0xd9 };
const uint8_t sync_val[] = {0x01};
const uint8_t sync_tol = 0x02;
const uint8_t recv_packet_len = 2;
//const uint8_t MED[] = {0x22, 0x0f, 0xe2, 0x00};
//const uint8_t MED[] = {0x84,0x3c,0x84,0x21,0xe4,0x21,0x08,0x43,0xcf,0x79,0xe7,0x9e,0x79,0x08,0x43,0xc8, 0x04};
const uint8_t preamb[] = {0x01,0x00,0x00,0x00,0x00};
const uint8_t _MED[] = {0x00,0x00,0x00,0x00,0x84,0x3c,0x84,0x21,0xe4,0x21,0x08,0x43,0xcf,0x79,0xe7,0x9e,0x79,0x08,0x43,0xc8,0x40};
const uint8_t MED[] = {0x84,0x3c,0x84,0x21,0xe4,0x21,0x08,0x43,0xcf,0x79,0xe7,0x9e,0x79,0x08,0x43,0xc8,0x40};

// Singleton instance of the radio driver
//
RH_RF69 rf69(RFM69_CS, RFM69_INT);

void setup() {
    Serial.begin(BAUDRATE);
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

    // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
    // ishighpowermodule flag set like this:
    rf69.setTxPower(14, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

    ATOMIC_BLOCK_START;

    /* Ensure the module is initialised before we try to configure it: */
    do {
		rf69.spiWrite(RH_RF69_REG_2F_SYNCVALUE1, 0xaa);
	}
	while (rf69.spiRead(RH_RF69_REG_2F_SYNCVALUE1) != 0xaa);
    do {
        rf69.spiWrite(RH_RF69_REG_2F_SYNCVALUE1, 0x55);
    }
	while (rf69.spiRead(RH_RF69_REG_2F_SYNCVALUE1) != 0x55);

    rf69.setOpMode(RH_RF69_OPMODE_MODE_STDBY);
    rf69.setOpMode(RH_RF69_OPMODE_SEQUENCEROFF);

    /* Initialize registers */
	rf69.spiWrite(RH_RF69_REG_02_DATAMODUL,     /*RH_RF69_DATAMODUL_DATAMODE_CONT_WITHOUT_SYNC | */RH_RF69_DATAMODUL_MODULATIONTYPE_OOK | RH_RF69_DATAMODUL_MODULATIONSHAPING_OOK_NONE);  /* pakcet mode, OOK, no shaping */

//BITRATE
    uint32_t bitrate = RH_RF69_FXOSC / 3800;
	rf69.spiWrite(RH_RF69_REG_03_BITRATEMSB,    bitrate >> 8);
	rf69.spiWrite(RH_RF69_REG_04_BITRATELSB,    bitrate);

	// rf69.spiWrite(RH_RF69_REG_05_FDEVMSB,       0x01);
	// rf69.spiWrite(RH_RF69_REG_06_FDEVLSB,       0x9a);  /* frequency deviation - 25Khz */

//FREQUENCY
    uint32_t freqHz = 868112500;
    freqHz /= RH_RF69_FSTEP;
	rf69.spiWrite(RH_RF69_REG_07_FRFMSB,        freqHz >> 16);
	rf69.spiWrite(RH_RF69_REG_08_FRFMID,        freqHz >> 8);
	rf69.spiWrite(RH_RF69_REG_09_FRFLSB,        freqHz);
    //rf69.setFrequency(868.1);

//BANDWIDTH
    //uint8_t bw = 0x15;  /* 5.2 kHz*/
    //uint8_t bw = 0x00;  /* 250 kHz*/
    //uint8_t bw = 0x15;
	//rf69.spiWrite(RH_RF69_REG_19_RXBW,          (rf69.spiRead(RH_RF69_REG_19_RXBW) & 0xE0) | bw);

//THRESHOLD
    rf69.spiWrite(RH_RF69_REG_1D_OOKFIX,        30);

    rf69.spiWrite(RH_RF69_REG_37_PACKETCONFIG1, RH_RF69_PACKETCONFIG1_DCFREE_NONE);  /* No packet filtering */
	rf69.spiWrite(RH_RF69_REG_28_IRQFLAGS2,     RH_RF69_IRQFLAGS2_FIFOOVERRUN);  /* Clear FIFO and flags */
	rf69.spiWrite(RH_RF69_REG_2D_PREAMBLELSB,   0x00);  /* We generate our own preamble */

//SYNC VAL
    rf69.spiWrite(RH_RF69_REG_2E_SYNCCONFIG,
		(1 << 7) /* sync on */ | ((sizeof(sync_val) - 1) << 3) /* 6 bytes */ | sync_tol /* error tolerance of 2 */
	);
	for (uint16_t i = 0; i < sizeof(sync_val); i++) {
		rf69.spiWrite(RH_RF69_REG_2F_SYNCVALUE1 + i, sync_val[i]);
	}

    /* Using fixed packet size for receive: */
	rf69.spiWrite(RH_RF69_REG_38_PAYLOADLENGTH, recv_packet_len);
	rf69.spiWrite(RH_RF69_REG_3C_FIFOTHRESH,    RH_RF69_FIFOTHRESH_TXSTARTCONDITION_NOTEMPTY);  /* transmit as soon as FIFO non-empty */
	rf69.spiWrite(RH_RF69_REG_6F_TESTDAGC,      RH_RF69_TESTDAGC_CONTINUOUSDAGC_IMPROVED_LOWBETAON);

    Serial.println("starting receive mode");
	/* Start in receive mode */
	rf69.setOpMode(RH_RF69_OPMODE_MODE_RX);
	ATOMIC_BLOCK_END;

    Serial.println("init done");
}

void loop() {
    rf69.send((uint8_t*)_MED, sizeof(_MED));
    for(int i = 0; i < 3; i++) {
        delay(8);
        rf69.send((uint8_t*)MED, sizeof(MED));
        rf69.waitPacketSent();
    }
    Serial.println("data sent");
    //delay(5000);

    if (rf69.available()) {
        // Should be a message for us now
        uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);
        if (rf69.recv(buf, &len)) {
        if (!len) return;
        buf[len] = 0;
        Serial.print("Received [");
        Serial.print(len);
        Serial.print("]: ");
        Serial.println((char*)buf);
        Serial.print("RSSI: ");
        Serial.println(rf69.lastRssi(), DEC);

        if (strstr((char *)buf, "Hello World")) {
            // Send a reply!
            uint8_t data[] = "And hello back to you";
            rf69.send(data, sizeof(data));
            rf69.waitPacketSent();
            Serial.println("Sent a reply");
            //Blink(LED, 40, 3); //blink LED 3 times, 40ms between blinks
        }
        } else {
        Serial.println("Receive failed");
        }
    }
    delay(500);
}