#include <Arduino.h>

#define LED1 3
#define LED2 46

void setup() {
    Serial.begin(115200);
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, HIGH);
}

void loop() {
    Serial.println("Hello, world!");
    digitalWrite(LED1, !digitalRead(LED1));
    digitalWrite(LED2, !digitalRead(LED2));
    delay(1000);
}