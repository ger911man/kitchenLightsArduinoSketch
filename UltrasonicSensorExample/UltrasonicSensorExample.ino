/* HC-SR04 example sketch
        *
        * https://create.arduino.cc/projecthub/Isaac100/getting-started-with-the-hc-sr04-ultrasonic-sensor-036380
*
* by Isaac100
*/

#include "Arduino.h"

#define PIN_TRIG 9                  // send/trig
#define PIN_ECHO 10                  // receive

float duration, distance;

void setup() {
    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);
    Serial.begin(115200);
}

void loop() {
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);

    duration = pulseIn(PIN_ECHO, HIGH);     //very spase consuming func
    distance = (duration*.0343)/2;
    Serial.print("Distance: ");
    Serial.println(distance);
    delay(100);
}