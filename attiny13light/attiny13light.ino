// Created by Gera 11/04/2023
// Attiny13 have only 1024 bytes of program storage space standard arduino libraries for HC-SR04 is too huge for Tiny13
// cores 4 attiny: MicroCore - OK (unable to flash via YUN, used Nano instead), DIY Attiny - core have glitches

//********** examples from inet *********************
// #1
//no pulseIn
//int ultrasonic(byte TrigPin, byte EchoPin){
//    PORTB |= (1<<TrigPin);
//    delayMicroseconds(10);
//    PORTB &= ~(1<<TrigPin);
//    uint32_t micros_old_Ult = micros();
//    while(!(PINB&(1<<EchoPin)) && micros()-micros_old_Ult < 500);
//    micros_old_Ult = micros();
//    while((PINB&(1<<EchoPin)) && micros()-micros_old_Ult < 20000);
//    return (micros() - micros_old_Ult)/58;
//}
// #2
//standard pulseIn func
//unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout)
//{
//    uint8_t bit = digitalPinToBitMask(pin);
//    uint8_t port = digitalPinToPort(pin);
//    uint8_t stateMask = (state ? bit : 0);
//    unsigned long maxloops = microsecondsToClockCycles(timeout)/16;
//    unsigned long width = countPulseASM(portInputRegister(port), bit, stateMask, maxloops);
//    if (width)
//        return clockCyclesToMicroseconds(width * 16 + 16);
//    else
//        return 0;
//}

// ATMEL ATTINY / ARDUINO
//
//                           +=._.=+
//  D5/A0/Reset/ADC0/PB5   1=|     |=8  VCC
//  D3/A3/XTAL1/ADC3/PB3~  2=|     |=7  PB2/SCL/SCK/D2(A1)
//  D4/A2/XTAL2/ADC2/PB4~  3=|     |=6  PB1~/MISO/D1
//                    Gnd  4=`-----'=5  PB0~/MOSI/SDA/AREF/D0
// "~" - PWM pins
// Attiny13: Has PWM only on PB0 and PB1
//           No AREF
//           PB3 and PB4 - the same timer


#include <Arduino.h>

//#define DEBUG_SERIAL_PLOTTER

// ultrasonic pins
#define PIN_ECHO 4                  // receive
#define PIN_TRIG 3                  // send/trig

// relay pin
#define PIN_RELAY_CEILING 0          // kitchen sealing light

// LED pin
#define PIN_DEBUG_LED 1              // led used as operate/debug indicator

#define DELAY_BETWEEN_SWITCHING 1000             // delay after light switching in millis
#define MAX_SWITCHING_COUNTER 5

// Ceiling lights need's to be switched ON/OFF if you put your hand between MIN_CEILING_SWITCHING_IST and MAX_CEILING_SWITCHING_DIST
#define MAX_CEILING_SWITCHING_DIST 100          // max distance eye level lights switches

boolean debugLedState = true;
boolean ceilingLightState = true; //true/HIGH - light switched OFF

void setup() {
    pinMode(PIN_DEBUG_LED, OUTPUT);
    pinMode(PIN_RELAY_CEILING, OUTPUT);
    digitalWrite(PIN_RELAY_CEILING, ceilingLightState);
    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);
#ifdef DEBUG_SERIAL_PLOTTER
    Serial.begin(9600);
#endif
}

float ultrasonic(){
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);
    uint32_t micros_old_Ult = micros();
    while(!digitalRead(PIN_ECHO) && micros() - micros_old_Ult < 500);
    micros_old_Ult = micros();
    while(digitalRead(PIN_ECHO) && micros() - micros_old_Ult < 20000);
    return (micros() - micros_old_Ult)/6;
}

//float ultrasonicPulseIn(){
//    digitalWrite(PIN_TRIG, HIGH);
//    delayMicroseconds(10);
//    digitalWrite(PIN_TRIG, LOW);
//    return pulseIn(PIN_ECHO, HIGH)/5.82;     //very spase consuming func caused by data type conversion
//}

unsigned char switchingCounter = 0;
unsigned long distance;

void loop() {
    distance = ultrasonic();

    if(distance < MAX_CEILING_SWITCHING_DIST){
        switchingCounter++;
        if(switchingCounter >= MAX_SWITCHING_COUNTER){
            digitalWrite(PIN_RELAY_CEILING, ceilingLightState = !ceilingLightState);
            switchingCounter = 0;
            delay(DELAY_BETWEEN_SWITCHING);
        }
    }
    else {
        if(switchingCounter > 0) switchingCounter--;
        delay(400);
    }

    digitalWrite(PIN_DEBUG_LED, debugLedState);
    debugLedState = !debugLedState;
    delay(10);

#ifdef DEBUG_SERIAL_PLOTTER
    Serial.print(distance);
    Serial.println();
#endif
}