#include <Arduino.h>

constexpr uint8_t LED_PIN = LED_BUILTIN;

void setup()
{
    Serial.begin(115200);
    delay(500);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    Serial.println();
    Serial.println("Hello, World from ESP8266 NodeMCU!");
}

void loop()
{
    static int aliveCounter = 0;
    aliveCounter++;
    digitalWrite(LED_PIN, LOW);
    Serial.print("Hello, ESP8266! Alive cycles: ");
    Serial.println(aliveCounter);
    delay(1000);

    digitalWrite(LED_PIN, HIGH);
    delay(1000);
}