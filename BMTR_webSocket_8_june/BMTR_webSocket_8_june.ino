#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <WebServer.h>

#define PIN_SCK   4
#define PIN_MOSI  6
#define PIN_MISO  5
#define PIN_CS    7

uint8_t recvBuf[22];
bool headerPrinted = false;

int time_start = 0;
unsigned long time_start_ms = 0;
float reaction_torque_input = 0.0;
float desired_torque = 0.0;
float theta1 = 0.0;
float theta2 = 0.0;

unsigned long last_timestamp_ms = 0;

const char* ssid = "BMTR";
const char* password = "bmtr";

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head><title>ESP32 SPI Monitor</title></head>
<body>
  <h2>Real-Time Data</h2>
  <div id="data" style="font-family:monospace;"></div>
<script>
  const ws = new WebSocket("ws://" + location.hostname + ":81");
  ws.onmessage = (event) => {
    const d = JSON.parse(event.data);
    document.getElementById("data").innerHTML = 
      "Timestamp: " + d.timestamp_ms + " ms<br>" +
      "Start Time: " + d.time_start_ms + " ms<br>" +
      "Reaction Torque: " + d.reaction_torque + "<br>" +
      "Commanded Torque: " + d.commanded_torque + "<br>" +
      "Theta1: " + d.theta1 + "<br>" +
      "Theta2: " + d.theta2 + "<br>" +
      "Delta_ms: " + d.delta_ms + " ms<br>";
  };
</script>
</body>
</html>
)rawliteral";

void setup() {
  pinMode(PIN_SCK, INPUT);
  pinMode(PIN_MOSI, INPUT);
  pinMode(PIN_MISO, OUTPUT);
  pinMode(PIN_CS, INPUT_PULLUP);
  digitalWrite(PIN_MISO, LOW);

  Serial.begin(115200);

  WiFi.softAP(ssid, password);
  delay(500);
  Serial.println("Access Point started");
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", []() {
    server.send_P(200, "text/html", index_html);
  });
  server.begin();

  webSocket.begin();
  webSocket.onEvent([](uint8_t num, WStype_t type, uint8_t *payload, size_t len) {
    if (type == WStype_CONNECTED)
      Serial.printf("[WS] Client %u connected\n", num);
    else if (type == WStype_DISCONNECTED)
      Serial.printf("[WS] Client %u disconnected\n", num);
  });

  if (!headerPrinted) {
    Serial.println("timestamp_ms\ttime_start_ms\treaction_torque\tcommanded_torque\ttheta1\ttheta2\tdelta_ms");
    headerPrinted = true;
  }
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
  server.handleClient();
  webSocket.loop();

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
  if (receivedCRC != computedCRC) return;

  memcpy(&time_start,             &recvBuf[1],  sizeof(int));
  memcpy(&reaction_torque_input, &recvBuf[5],  sizeof(float));
  memcpy(&desired_torque,        &recvBuf[9],  sizeof(float));
  memcpy(&theta1,                &recvBuf[13], sizeof(float));
  memcpy(&theta2,                &recvBuf[17], sizeof(float));
  time_start_ms = (unsigned long)time_start;

  unsigned long timestamp_ms = millis();
  unsigned long delta_ms = timestamp_ms - last_timestamp_ms;
  last_timestamp_ms = timestamp_ms;

  // Print to Serial
  Serial.printf("%lu\t%lu\t%.2f\t%.2f\t%.2f\t%.2f\t%lu\n",
                timestamp_ms, time_start_ms, reaction_torque_input,
                desired_torque, theta1, theta2, delta_ms);

  // Send over WebSocket
  char msg[200];
  snprintf(msg, sizeof(msg),
           "{\"timestamp_ms\":%lu,\"time_start_ms\":%lu,\"reaction_torque\":%.2f,\"commanded_torque\":%.2f,\"theta1\":%.2f,\"theta2\":%.2f,\"delta_ms\":%lu}",
           timestamp_ms, time_start_ms, reaction_torque_input,
           desired_torque, theta1, theta2, delta_ms);
  webSocket.broadcastTXT(msg);

  delay(50);  // optional rate limiter
}
