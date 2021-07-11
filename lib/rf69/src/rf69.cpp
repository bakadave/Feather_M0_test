/* Simple driver for RF69 module for use with integration projects.  Note that
 * this assumes a single RF69 attached to the Arduino.
 *
 * Intended to be a very simple, easy to use implementation, that's enough to
 * provide necessary functionality to interact with Danfoss systems. */

#include <SPI.h>
#include <RH_RF69.h>

#include "rf69.h"

/* SPI settings for RF69.  Note the 4MHz bus speed is lower than the 10MHz max
 * supported by the chip, but other libraries seem to operate stably at this
 * speed and since our data rate is low it seems wise to choose a safe value.
 **/
static SPISettings rf69_spi_settings(4000000, MSBFIRST, SPI_MODE0);


// Radio::Radio(uint8_t _pin_cs, uint8_t _pin_int, uint8_t _pin_rst) {
//     pin_cs = _pin_cs;
//     pin_int = _pin_int;
//     pin_rst = _pin_rst;
// }

bool Radio::rf69_init(uint8_t sync_len, uint8_t recv_packet_len, const uint8_t *sync_val, uint8_t sync_tol) {
	pinMode(pin_rst, OUTPUT);
	digitalWrite(pin_rst, LOW);

    //sync value of 0x00 is forbidden
    for(int i = 0; i < sync_len; i++) {
        if(sync_val[i] == 0x00) {
            Serial.println("illegal sync value");
            return false;
        }
    }

    // manual reset
    digitalWrite(pin_rst, HIGH);
    delay(10);
    digitalWrite(pin_rst, LOW);
    delay(10);

    Serial.println("module reset, starting init");

	if (!rf69.init()) {
        Serial.println("rf69.init() failed");
		return false;
	}
    Serial.println("setting TX power");
	rf69.setTxPower(14, true);

	ATOMIC_BLOCK_START;

    Serial.println("TX power set, checking module initialisation");

	/* Ensure the module is initialised before we try to configure it: */
	do {
		rf69.spiWrite(RH_RF69_REG_2F_SYNCVALUE1, 0xaa);
	}
	while (rf69.spiRead(RH_RF69_REG_2F_SYNCVALUE1) != 0xaa);
    do {
        rf69.spiWrite(RH_RF69_REG_2F_SYNCVALUE1, 0x55);
    }
	while (rf69.spiRead(RH_RF69_REG_2F_SYNCVALUE1) != 0x55);

    Serial.println("module initialisation check pass");

	rf69.setOpMode(RH_RF69_OPMODE_MODE_STDBY);
    rf69.setOpMode(RH_RF69_OPMODE_SEQUENCEROFF);

    Serial.println("opmode set");

	/* Initialize registers */
	rf69.spiWrite(RH_RF69_REG_02_DATAMODUL,     RH_RF69_DATAMODUL_DATAMODE_CONT_WITHOUT_SYNC | RH_RF69_DATAMODUL_MODULATIONTYPE_OOK | RH_RF69_DATAMODUL_MODULATIONSHAPING_OOK_NONE);  /* pakcet mode, OOK, no shaping */

//BITRATE
    uint32_t bitrate = RH_RF69_FXOSC / 3000;
	rf69.spiWrite(RH_RF69_REG_03_BITRATEMSB,    bitrate >> 8);
	rf69.spiWrite(RH_RF69_REG_04_BITRATELSB,    bitrate);

	// rf69.spiWrite(RH_RF69_REG_05_FDEVMSB,       0x01);
	// rf69.spiWrite(RH_RF69_REG_06_FDEVLSB,       0x9a);  /* frequency deviation - 25Khz */

//FREQUENCY
    uint32_t freqHz = 868100000;
    freqHz /= RH_RF69_FSTEP;
	rf69.spiWrite(RH_RF69_REG_07_FRFMSB,        freqHz >> 16);
	rf69.spiWrite(RH_RF69_REG_08_FRFMID,        freqHz >> 8);
	rf69.spiWrite(RH_RF69_REG_09_FRFLSB,        freqHz);
    //rf69.setFrequency(868.1);

//BANDWIDTH
    //uint8_t bw = 0x00;
	//rf69.spiWrite(RH_RF69_REG_19_RXBW,          (rf69.spiRead(RH_RF69_REG_19_RXBW) & 0xE0) | bw);

//THRESHOLD
    rf69.spiWrite(RH_RF69_REG_1D_OOKFIX,        30);

    rf69.spiWrite(RH_RF69_REG_37_PACKETCONFIG1, RH_RF69_PACKETCONFIG1_DCFREE_NONE);  /* No packet filtering */
	rf69.spiWrite(RH_RF69_REG_28_IRQFLAGS2,     RH_RF69_IRQFLAGS2_FIFOOVERRUN);  /* Clear FIFO and flags */
	rf69.spiWrite(RH_RF69_REG_2D_PREAMBLELSB,   0x00);  /* We generate our own preamble */
//SYNC VAL
    // rf69.spiWrite(RH_RF69_REG_2E_SYNCCONFIG,
	// 	(1 << 7) /* sync on */ | ((sync_len - 1) << 3) /* 6 bytes */ | sync_tol /* error tolerance of 2 */
	// );
	// for (uint16_t i = 0; i < sync_len; i++) {
	// 	rf69.spiWrite(RH_RF69_REG_2F_SYNCVALUE1 + i, sync_val[i]);
	// }

	/* Using fixed packet size for receive: */
	rf69.spiWrite(RH_RF69_REG_38_PAYLOADLENGTH, recv_packet_len);
	rf69.spiWrite(RH_RF69_REG_3C_FIFOTHRESH,    RH_RF69_FIFOTHRESH_TXSTARTCONDITION_NOTEMPTY);  /* transmit as soon as FIFO non-empty */
	rf69.spiWrite(RH_RF69_REG_6F_TESTDAGC,      RH_RF69_TESTDAGC_CONTINUOUSDAGC_IMPROVED_LOWBETAON);

    Serial.println("starting receive mode");
	/* Start in receive mode */
	rf69.setOpMode(RH_RF69_OPMODE_MODE_RX);
	ATOMIC_BLOCK_END;

    Serial.println("init done");
	return true;
}

void Radio::rf69_receiveDone(/*uint8_t *out, uint8_t *sz*/)
{
	// ATOMIC_BLOCK_START;
	// if (rf69.spiRead(RH_RF69_REG_28_IRQFLAGS2) & 4 /* PayloadReady*/) {
	// 	size_t max_sz = *sz, b = 0;

	// 	/* Read data out of FIFO */
	// 	while (b < max_sz && rf69.spiRead(RH_RF69_REG_28_IRQFLAGS2) & 0x20 /* fifo not empty */) {
	// 		out[b++] = rf69.spiRead(RH_RF69_REG_00_FIFO);
	// 	}

	// 	*sz = b;
	// 	return true;
	// }


	// ATOMIC_BLOCK_END;
	// return false;

    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf69.waitAvailableTimeout(500)) {
        // Should be a reply message for us now
        if (rf69.recv(buf, &len)) {
            Serial.print("got reply: ");
            Serial.println((char*)buf);
        }
        else {
            Serial.println("recv failed");
        }
    }
    else {
        Serial.println("No reply, is rf69_server running?");
    }
}

void Radio::rf69_transmit(const uint8_t *data, size_t len, bool no_sync)
{
	uint8_t orig_payload_len, orig_syncconf = 0, pos;

	ATOMIC_BLOCK_START;
	/* Clear any current receive in progress */
	rf69.setOpMode(RH_RF69_OPMODE_MODE_STDBY);
	rf69.spiWrite(RH_RF69_REG_28_IRQFLAGS2, RH_RF69_IRQFLAGS2_FIFOOVERRUN);  /* Clear FIFO and flags */

	/* Disable sync word? */
	if (no_sync) {
		orig_syncconf = rf69.spiRead(RH_RF69_REG_2E_SYNCCONFIG);
		rf69.spiWrite(RH_RF69_REG_2E_SYNCCONFIG, RH_RF69_SYNCCONFIG_SYNCSIZE_1);
	}

	/* Populate the FIFO with the data.  We assume len < size of fifo (66
	* bytes) */
	orig_payload_len = rf69.spiRead(RH_RF69_REG_38_PAYLOADLENGTH);
	rf69.spiWrite(RH_RF69_REG_38_PAYLOADLENGTH, len);
	SPI.beginTransaction(rf69_spi_settings);
	digitalWrite(pin_cs, LOW);
	SPI.transfer(RH_RF69_REG_00_FIFO | 0x80);
	for (pos = 0; pos < len; pos++) {
		SPI.transfer(data[pos]);
	}
	digitalWrite(pin_cs, HIGH);
	SPI.endTransaction();

	/* Transition to transmit state, then poll for transmission complete */
	rf69.setOpMode(RH_RF69_OPMODE_MODE_TX);
	while (!(rf69.spiRead(RH_RF69_REG_28_IRQFLAGS2) & 8 /* PacketSent */)) {
		delay(100);
	}

	/* Re-enable sync word? */
	if (no_sync) {
		rf69.setOpMode(RH_RF69_OPMODE_MODE_STDBY);
		rf69.spiWrite(RH_RF69_REG_2E_SYNCCONFIG, orig_syncconf);
	}

	/* Now back to receive mode */
	rf69.spiWrite(RH_RF69_REG_38_PAYLOADLENGTH, orig_payload_len);
	rf69.setOpMode(RH_RF69_OPMODE_MODE_RX);
	ATOMIC_BLOCK_END;
}
