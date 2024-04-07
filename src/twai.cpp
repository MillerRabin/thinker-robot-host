#include "twai.h"

CanFrame rxFrame;

/*void sendObdFrame(uint8_t obdId) {
	CanFrame obdFrame = { 0 };
	obdFrame.identifier = 0x7DF; // Default OBD2 address;
	obdFrame.extd = 0;
	obdFrame.data_length_code = 8;
	obdFrame.data[0] = 2;
	obdFrame.data[1] = 1;
	obdFrame.data[2] = obdId;
	obdFrame.data[3] = 0xAA;    // Best to use 0xAA (0b10101010) instead of 0
	obdFrame.data[4] = 0xAA;    // CAN works better this way as it needs
	obdFrame.data[5] = 0xAA;    // to avoid bit-stuffing
	obdFrame.data[6] = 0xAA;
	obdFrame.data[7] = 0xAA;
    // Accepts both pointers and references 
    ESP32Can.writeFrame(obdFrame);  // timeout defaults to 1 ms
}*/

void TWAI_Setup() {
    if(ESP32Can.begin(ESP32Can.convertSpeed(1000), CAN_TX, CAN_RX, 5, 5)) {
        Serial.println("CAN bus started!");
    } else {
        Serial.println("CAN bus failed!");
    }
}

void TWAI_GetData() {
    if(ESP32Can.readFrame(rxFrame, 1000)) {
        Serial.printf("Received frame: %03X  \r\n", rxFrame.identifier);
    }
}