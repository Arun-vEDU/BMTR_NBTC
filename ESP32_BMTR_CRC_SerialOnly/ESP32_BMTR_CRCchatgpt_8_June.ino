#define PIN_SCK   4
#define PIN_MOSI  6
#define PIN_MISO  5
#define PIN_CS    7

uint8_t recvBuf[22];  // Start marker + 20 data bytes + 1 CRC
bool headerPrinted = false;

// Variables to hold received data
int time_start = 0;
float time_start_sec = 0.0;
float reaction_torque_input = 0.0;
float desired_torque = 0.0;
float theta1 = 0.0;
float theta2 = 0.0;

void setup() {
  pinMode(PIN_SCK, INPUT);
  pinMode(PIN_MOSI, INPUT);
  pinMode(PIN_MISO, OUTPUT);
  pinMode(PIN_CS, INPUT_PULLUP);

  digitalWrite(PIN_MISO, LOW);
  Serial.begin(115200);
}

uint8_t ReadByteSPI(unsigned long timeout_us = 2000) {
  uint8_t receivedByte = 0;
  uint8_t toSend = 0xA5;

  for (int i = 0; i < 8; i++) {
    unsigned long startTime = micros();
    while (digitalRead(PIN_SCK) == LOW) {
      if (micros() - startTime > timeout_us) return 0xFF;
    }

    delayMicroseconds(1);
    int bit = digitalRead(PIN_MOSI);
    receivedByte = (receivedByte << 1) | (bit & 0x01);

    digitalWrite(PIN_MISO, (toSend & 0x80) ? HIGH : LOW);
    toSend <<= 1;

    startTime = micros();
    while (digitalRead(PIN_SCK) == HIGH) {
      if (micros() - startTime > timeout_us) return 0xFF;
    }
  }
  return receivedByte;
}

bool WaitForCSLow(unsigned long timeout_ms = 100) {
  unsigned long start = millis();
  while (digitalRead(PIN_CS) == HIGH) {
    if (millis() - start > timeout_ms) return false;
  }
  return true;
}

bool WaitForCSHigh(unsigned long timeout_ms = 100) {
  unsigned long start = millis();
  while (digitalRead(PIN_CS) == LOW) {
    if (millis() - start > timeout_ms) return false;
  }
  return true;
}

uint8_t CalculateCRC(uint8_t* data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
  }
  return crc;
}

void loop() {
  if (!WaitForCSLow()) return;

  memset(recvBuf, 0, sizeof(recvBuf));

  uint8_t startByte = 0;
  do {
    startByte = ReadByteSPI();
  } while (startByte != 0xAA && digitalRead(PIN_CS) == LOW);

  if (startByte != 0xAA || digitalRead(PIN_CS) == HIGH) return;

  recvBuf[0] = startByte;
  for (int i = 1; i < 22; i++) {
    uint8_t byte = ReadByteSPI();
    if (byte == 0xFF && digitalRead(PIN_CS) == LOW) return;
    recvBuf[i] = byte;
  }

  if (!WaitForCSHigh()) return;

  uint8_t receivedCRC = recvBuf[21];
  uint8_t computedCRC = CalculateCRC(recvBuf, 21);

  if (receivedCRC != computedCRC) {
    // CRC mismatch, ignore packet
    return;
  }

  // Extract values only if CRC is OK
  memcpy(&time_start,             &recvBuf[1],  sizeof(int));
  memcpy(&reaction_torque_input, &recvBuf[5],  sizeof(float));
  memcpy(&desired_torque,        &recvBuf[9],  sizeof(float));
  memcpy(&theta1,                &recvBuf[13], sizeof(float));
  memcpy(&theta2,                &recvBuf[17], sizeof(float));

  time_start_sec = time_start / 1000.0;

  if (!headerPrinted) {
    Serial.println("time_sec\treaction_torque\tdesired_torque\ttheta1\ttheta2");
    headerPrinted = true;
  }

  Serial.print(time_start_sec, 2);         Serial.print("\t");
  Serial.print(reaction_torque_input, 2);  Serial.print("\t");
  Serial.print(desired_torque, 2);         Serial.print("\t");
  Serial.print(theta1, 2);                 Serial.print("\t");
  Serial.println(theta2, 2);

  delay(100);
}
