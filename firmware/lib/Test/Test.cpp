#include <Global.h>
using namespace constants;

// Encoder Variables 
volatile boolean rotating;  // debounce management
boolean A_state, B_state;  // state flag

int16_t pos; // counter for dial

void isr() {
    if (rotating) {
        delayMicroseconds(DEBOUNCE_SPAN);
 
        if (digitalRead(2) != A_state) {
            A_state = !A_state;
            if (A_state && !B_state)
                pos += 1;
            else if (A_state && B_state)
                pos -=1;
             
        }
        rotating = false;
    }    
}


void estart() {
    rotating = false;
    A_state = false;
    B_state = false;
    pos = 0; 

    pinMode(2, INPUT_PULLUP);
    pinMode(3, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(2), isr, RISING);     
}

// Update per loop
void update() {
    noInterrupts();
    rotating = true; 
    Serial.println(pos);
    interrupts();
}
 
