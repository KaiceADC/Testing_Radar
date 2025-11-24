#include <SPI.h>
#include "mcp2515_can.h"

const int SPI_CS_PIN = 9;             // Or your CS pin!
mcp2515_can CAN(SPI_CS_PIN);

void setup() {
  Serial.begin(115200);
  while (!Serial) ;
  if (CAN_OK != CAN.begin(CAN_500KBPS)) {
    while (1); // hang if CAN fails
  }
}

void loop() {
  unsigned char len = 0;
  unsigned char buf[8];
  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    CAN.readMsgBuf(&len, buf);
    unsigned long canId = CAN.getCanId();
    // Send CAN ID (4 bytes, little endian)
    Serial.write((uint8_t*)&canId, 4);
    // Send message data (8 bytes, even if not used)
    Serial.write(buf, 8);
    // Send message length (1 byte, typically 8 for radar messages)
    Serial.write(len);
  }
}
