// Created by Gera 11/04/2023

#include <Arduino.h>

// ultrasonic pins
#define PIN_ECHO 2                  // receive
#define PIN_TRIG 3                  // send/trig

// relay pins
//TODO figure out why pin #4 is acting weird on wavgat avga328p pro mini
#define PIN_RELAY_EYE_LEVEL 5       // kitchen eye-level light
#define PIN_RELAY_CEILING 6          // kitchen sealing light

#define MAX_MEASURING_DISTANCE 600              // max distance to measure via ultrasonic distance sensor in mm
#define SMOOTHNESS_COEFFICIENT 0.9              // makes distance measures smoother [0.1 ... 0.9]
#define DELAY_BETWEEN_SWITCHING 1000            // delay after light switching in millis
#define MAX_COUNTER 4

// Ceiling lights need's to be switched ON/OFF if you put your hand between MIN_CEILING_SWITCHING_IST and MAX_CEILING_SWITCHING_DIST
#define MIN_CEILING_SWITCHING_IST 0             // min distance eye level lights switches
#define MAX_CEILING_SWITCHING_DIST 80          // max distance eye level lights switches

// Eye level lights need's to be switched ON/OFF if you put your hand between MIN_EYE_LEVEL_SWITCHING_IST and MIN_EYE_LEVEL_SWITCHING_DIST
#define MIN_EYE_LEVEL_SWITCHING_DIST 80        // min distance eye level lights switches
#define MAX_EYE_LEVEL_SWITCHING_DIST 160        // max distance eye level lights switches

//#define DEBUG_SERIAL_PLOTTER                  // if defined - print logs to serial port ( plotter format supported ) Wavgat avga328p not supporting serial
#define DEBUG_BUILTIN_LED

#ifdef DEBUG_BUILTIN_LED
boolean internalLedState = false;
#endif

class LightSwitcher{
public:
    /// <summary>
    /// Class for switching ceiling and eye-level lights based on measured distance by sensor "HC-SR04"
    /// </summary>
    LightSwitcher();
    void processDistance(unsigned long distance);

private:
    uint8_t pin_eyeLevel;
    uint8_t pin_ceiling;
    float distance;
    float prevDist;
    float rawDist;
    uint8_t eyeLevelLightCounter;
    uint8_t ceilingLightCounter;
    boolean eyeLevelLightState;
    boolean ceilingLightState ;
};

LightSwitcher::LightSwitcher() {
    this->pin_eyeLevel = PIN_RELAY_EYE_LEVEL;
    this->pin_ceiling = PIN_RELAY_CEILING;
    this->distance = MAX_MEASURING_DISTANCE;
    this->prevDist = MAX_MEASURING_DISTANCE;
    this->rawDist = MAX_MEASURING_DISTANCE;
    this->eyeLevelLightCounter = 0;
    this->ceilingLightCounter = 0;
    this->eyeLevelLightState = true;
    this->ceilingLightState = true;
    digitalWrite(pin_ceiling, ceilingLightState);
    digitalWrite(pin_eyeLevel, eyeLevelLightState);
}

void LightSwitcher::processDistance(unsigned long dist) {
    rawDist = dist;
    distance += (rawDist - prevDist) * SMOOTHNESS_COEFFICIENT;

    if (MIN_CEILING_SWITCHING_IST < dist && dist < MAX_CEILING_SWITCHING_DIST) {
        ceilingLightCounter++;
        if (eyeLevelLightCounter > 0) eyeLevelLightCounter--;
        if (ceilingLightCounter >= MAX_COUNTER) {
            ceilingLightState = !ceilingLightState;
            digitalWrite(pin_ceiling, ceilingLightState);
            delay(DELAY_BETWEEN_SWITCHING);
            ceilingLightCounter = 0;
        }
    }
    else if (MIN_EYE_LEVEL_SWITCHING_DIST <= dist && dist < MAX_EYE_LEVEL_SWITCHING_DIST) {
        eyeLevelLightCounter ++;
        if(ceilingLightCounter > 0) ceilingLightCounter--;
        if(eyeLevelLightCounter > MAX_COUNTER) {
            eyeLevelLightState = !eyeLevelLightState;
            digitalWrite(pin_eyeLevel, eyeLevelLightState);
            delay(DELAY_BETWEEN_SWITCHING);
            eyeLevelLightCounter = 0;
        }
    } else {
        if(ceilingLightCounter > 0) ceilingLightCounter--;
        if(eyeLevelLightCounter > 0) eyeLevelLightCounter--;
        delay(400);
    }
}

class DistanceSensor {
public:
    /// <summary>
    /// Class for ultrasonic distance sensor "HC-SR04"
    /// </summary>
    DistanceSensor();
    float getDistance();

private:
    uint8_t pin_trig;
    uint8_t pin_echo;
    unsigned long distance;
};

DistanceSensor::DistanceSensor(){
    this->pin_trig = PIN_TRIG;
    this->pin_echo = PIN_ECHO;
    this->distance = MAX_MEASURING_DISTANCE;
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

    //relays
    pinMode(PIN_RELAY_EYE_LEVEL, OUTPUT);
    pinMode(PIN_RELAY_CEILING, OUTPUT);
    //sensor
    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);

#ifdef DEBUG_BUILTIN_LED
    pinMode(LED_BUILTIN, OUTPUT);
#endif
#ifdef DEBUG_SERIAL_PLOTTER
    Serial.begin(115200);
    Serial.println("Raw:, Smooth:");
#endif
}

DistanceSensor sensor = DistanceSensor();
LightSwitcher switcher = LightSwitcher();



void loop() {
    switcher.processDistance(sensor.getDistance());

#ifdef DEBUG_SERIAL_PLOTTER
    Serial.print(rawDist);
    Serial.print(",");
    Serial.print(dist);
    Serial.println();
#endif
#ifdef DEBUG_BUILTIN_LED
    digitalWrite(LED_BUILTIN, internalLedState);
    internalLedState = !internalLedState;
#endif
    delay(10);
}