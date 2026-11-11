#include <WiFi.h>
void setup() {
Serial.begin(115200);
WiFi.mode(WIFI_STA); // necesario para que la MAC no salga en ceros
delay(100);
}
void loop() {
uint8_t m[6];
WiFi.macAddress(m);
Serial.printf("MAC STA: %s\n", WiFi.macAddress().c_str());
Serial.printf("Para MAC_GATEWAY: {0x %02X, 0x %02X, 0x %02X, 0x %02X, 0x %02X, 0x %02X
}\n\n",
m[0], m[1], m[2], m[3], m[4], m[5]);