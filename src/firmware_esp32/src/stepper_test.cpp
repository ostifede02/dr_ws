#include <Arduino.h>
#include "ShiftRegister74HC595.h"

#define PIN_STEP 1
#define PIN_DIR 2


void do_step(void);


// create a global shift register object
#define NUMBER_SHIFT_REG 1
#define I2S_DATA_PIN 21
#define I2S_CLOCK_PIN 16
#define I2S_LATCH_PIN 17

ShiftRegister74HC595<NUMBER_SHIFT_REG> sr(I2S_DATA_PIN, I2S_CLOCK_PIN, I2S_LATCH_PIN);


void setup() {
    Serial.begin(115200);
    pinMode(I2S_DATA_PIN, OUTPUT);
    pinMode(I2S_DATA_PIN, OUTPUT);
    pinMode(I2S_LATCH_PIN, OUTPUT);
}

int direction = HIGH;

void loop() {
    
    for(int i=0; i < 3200; ++i){
        do_step();
    }

    if (direction == HIGH){
        direction = LOW;
    } else {
        direction = HIGH;
    }
    // digitalWrite(PIN_DIR, direction);
    sr.set(PIN_DIR, direction);
    delay(200);
}

void do_step(void)
{
    // digitalWrite(PIN_STEP, HIGH);
    sr.set(PIN_STEP, HIGH);
    delayMicroseconds(40);
    // digitalWrite(PIN_STEP, LOW);
    sr.set(PIN_STEP, LOW);
    delayMicroseconds(40);
    return;
}