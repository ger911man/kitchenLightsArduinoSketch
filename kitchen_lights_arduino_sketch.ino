// Created by Gera 11/04/2023

#include <Arduino.h>

// ultrasonic pins
#define PIN_TRIG 3          // send
#define PIN_ECHO 2          // receive

// relay pins
#define PIN_RELAY_EYE_LEVEL 5     // kitchen eye-level light
#define PIN_RELAY_CEILING 6       // kitchen sealing light

#define MAX_MEASURING_DISTANCE 600              // max distance to measure via ultrasonic distance sensor in mm
#define SMOOTHNESS_COEFFICIENT 0.9              // makes distance measures smoother [0.1 ... 0.9]
#define DELAY_BETWEEN_SWITCHING 1000               // delay after light switching in millis
#define MAX_COUNTER 3

// Ceiling lights need's to be switched ON/OFF if you put your hand between MIN_CEILING_SWITCHING_IST and MAX_CEILING_SWITCHING_DIST
#define MIN_CEILING_SWITCHING_IST 0           // min distance eye level lights switches
#define MAX_CEILING_SWITCHING_DIST 100           // max distance eye level lights switches

// Eye level lights need's to be switched ON/OFF if you put your hand between MIN_EYE_LEVEL_SWITCHING_IST and MIN_EYE_LEVEL_SWITCHING_DIST
#define MIN_EYE_LEVEL_SWITCHING_DIST 100        // min distance eye level lights switches
#define MAX_EYE_LEVEL_SWITCHING_DIST 200        // max distance eye level lights switches

#define DEBUG_SERIAL_PLOTTER                          // if defined - print logs to serial port ( plotter format supported )

boolean internalLedState = false;
boolean eyeLevelLightState = false;
boolean ceilingLightState = false;

float dist;
float prevDist;
float rawDist;

uint8_t eyeLevelLightCounter = 0;
uint8_t ceilingLightCounter = 0;

class DistanceSensor {
public:
    /// <summary>
    /// Class for ultrasonic distance sensor "HC-SR04"
    /// </summary>
    DistanceSensor(uint8_t pin_trig, uint8_t pin_echo);
    float getDistance();

private:
    uint8_t pin_trig;
    uint8_t pin_echo;
    unsigned long distance;
};

DistanceSensor::DistanceSensor(uint8_t pin_trig, uint8_t pin_echo){
    this->pin_trig = pin_trig;
    this->pin_echo = pin_echo;
    this->distance = 0;
}

float DistanceSensor::getDistance() {
    /// <summary>
    /// Method returns distance in mm
    /// </summary>
    digitalWrite(pin_trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(pin_trig, LOW);

    distance = pulseIn(pin_echo, HIGH) / 5.82;
    if (distance > MAX_MEASURING_DISTANCE) {
        return MAX_MEASURING_DISTANCE;
    }
    else {
        return distance;
    }
}

void setup() {
    //builtin LED
    pinMode(LED_BUILTIN, OUTPUT);
    //relays
    pinMode(PIN_RELAY_EYE_LEVEL, OUTPUT);
    pinMode(PIN_RELAY_CEILING, OUTPUT);
    //sensor
    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);

#ifdef DEBUG_SERIAL_PLOTTER
    Serial.begin(115200);
    Serial.println("Raw:, Smooth:");
#endif

    dist = MAX_MEASURING_DISTANCE;
    prevDist = dist;
    rawDist = dist;
}

DistanceSensor sensor = DistanceSensor(PIN_TRIG, PIN_ECHO);

void loop() {
    rawDist = sensor.getDistance();
    dist += (rawDist - prevDist) * SMOOTHNESS_COEFFICIENT;

    if (MIN_CEILING_SWITCHING_IST < dist && dist < MAX_CEILING_SWITCHING_DIST) {
        ceilingLightCounter++;
        if (eyeLevelLightCounter > 0) eyeLevelLightCounter--;
        if (ceilingLightCounter >= MAX_COUNTER) {
            ceilingLightState = !ceilingLightState;
            digitalWrite(PIN_RELAY_CEILING, ceilingLightState);
            delay(DELAY_BETWEEN_SWITCHING);
            ceilingLightCounter = 0;
        }
    }
    else if (MIN_EYE_LEVEL_SWITCHING_DIST <= dist && dist < MAX_EYE_LEVEL_SWITCHING_DIST) {
        eyeLevelLightCounter ++;
        if(ceilingLightCounter > 0) ceilingLightCounter--;
        if(eyeLevelLightCounter > MAX_COUNTER) {
            eyeLevelLightState = !eyeLevelLightState;
            digitalWrite(PIN_RELAY_EYE_LEVEL, eyeLevelLightState);
            delay(DELAY_BETWEEN_SWITCHING);
            eyeLevelLightCounter = 0;
        }
    } else {
        if(ceilingLightCounter > 0) ceilingLightCounter--;
        if(eyeLevelLightCounter > 0) eyeLevelLightCounter--;
        delay(150);
    }

#ifdef DEBUG_SERIAL_PLOTTER
    digitalWrite(LED_BUILTIN, internalLedState);
    internalLedState = !internalLedState;
    Serial.print(rawDist);
    Serial.print(",");
    Serial.print(dist);
    Serial.println();
#endif

    prevDist = dist;
    delay(50);
}