#include "Arduino.h"

#define EchoInputCount 4
#define TriggerPin  2

#define DelayBetweenPings 50 // it works to about 5 milliseconds between pings

volatile  unsigned long PingTime[EchoInputCount];
volatile int Counter = EchoInputCount;
volatile  unsigned long edgeTime;
volatile  uint8_t PCintLast;
int PinMask = B1000; // pin 3
float Measurements[EchoInputCount];
void PintTimer()
{
    uint8_t pin;
    static unsigned long cTime;
    cTime = micros();         // micros() return a uint32_t
    pin = PIND >> 3 & 1;      // Quickly get the state of  pin 3
    if (pin)edgeTime = cTime; //Pulse went HIGH store the start time
    else { // Pulse Went low calculate the duratoin
        PingTime[Counter % EchoInputCount] = cTime - edgeTime; // Calculate the change in time  NOTE: the "% EchoInputCount" prevents the count from overflowing the array look up % remainder calculation
        Counter++;
    }
}
void debug()
{
    char S[20];
    static unsigned long PingTimer;
    if ((unsigned long)(millis() - PingTimer) >= 1) {
        PingTimer = millis();
        for (int c = 0; c < EchoInputCount; c++) {
            Serial.print(dtostrf(Measurements[c], 6, 1, S));
        }
        Serial.println();
    }
}

float microsecondsToInches(long microseconds)
{
    return (float) microseconds / 74 / 2;
}

float microsecondsToCentimeters(long microseconds)
{
    return (float)microseconds / 29 / 2;
}

void PingTrigger(int Pin)
{

    digitalWrite(Pin, LOW);
    delayMicroseconds(1);
    digitalWrite(Pin, HIGH); // Trigger another pulse
    delayMicroseconds(10);
    digitalWrite(Pin, LOW);
}

void PingIt()
{
    unsigned long PT[EchoInputCount];
    static unsigned long PingTimer;
    if (Counter >= EchoInputCount) {
        if ( ((unsigned long)(millis() - PingTimer) >= DelayBetweenPings)) {
            PingTimer = millis();
            cli ();         // clear interrupts flag
            for (int c = 0; c < EchoInputCount; c++) {
                PT[c] = PingTime[c];
            }
            sei ();         // set interrupts flag
            for (int c = 0; c < EchoInputCount; c++) {
                if (PT[c] < 43200) Measurements[c] = (float) (microsecondsToCentimeters(PT[c]));
            }
            //      Measurements = (float) (microsecondsToInches(PT));
            debug();
            delay(10);
            PingTrigger(TriggerPin); // Send another ping
            Counter = 0;
        }
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Ping Test");
    pinMode(3, INPUT);
    pinMode(2, OUTPUT);
    attachInterrupt(1, PintTimer, CHANGE );

}

void loop()
{
    PingIt(); // Manage ping data

    if ( ((unsigned long)(millis() - edgeTime) >= 1000)) {
        PingTrigger(TriggerPin); // Send another ping
        Counter = 0;
        delay(100);
    }

}
