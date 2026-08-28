#include <Arduino.h>

void setup() {
    pinMode(2, OUTPUT); digitalWrite(2, LOW);
    pinMode(3, OUTPUT); digitalWrite(3, LOW);
    pinMode(4, OUTPUT); digitalWrite(4, LOW);
    pinMode(5, OUTPUT); digitalWrite(5, LOW);
    pinMode(6, OUTPUT); digitalWrite(6, LOW);
    pinMode(7, OUTPUT); digitalWrite(7, LOW);
    pinMode(13, OUTPUT); digitalWrite(13, LOW);
    Serial.begin(115200);
    Serial.println(F("[SYSTEM] Stopped / Idle"));
}

void loop() {
    delay(1000);
}

