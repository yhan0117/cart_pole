#include <Arduino.h>
#include <Wire.h>
#include <Global.h>
#include <IMU.h>
using namespace constants;

/* to dos
  test motor stuff
  test calibiration
*/
// #define CART 1
#define AXIS 1
// #define PMRY 1 
// #define TEST 1

#ifdef CART
  #include "Encoder.h"
  extern EBoard Cart;
#elif AXIS
  #include "Encoder.h"
  extern EBoard Axis;
#elif PMRY
  #include "Primary.h"
  states curState;
  IMU myIMU;
#elif TEST
  #include "Test.cpp"
#endif

void setup() {
  Serial.begin(115200); // debugging use
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz

  // built-in LED for debug
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);  // dim LED on default
  
  #ifdef CART // board on cart responsible for generating angular data  
    Cart.begin();
  #elif AXIS  // board on right end responsible for measuring position  
    Axis.begin();
  #elif PMRY // primary board reponsible for motor control and communication with computer 
    primary_begin(&curState, myIMU);
  #elif TEST
    estart();
  #endif
}

void loop() {
	#ifdef CART 
    Cart.update();
	#elif AXIS   
		Axis.update();
    Serial.println(Axis.pos);
	#elif PMRY
    // if(Serial.available()) serial_receive(curState);
    // // imu_update(&curState, &myIMU);
    // if(Serial.available()) serial_receive(curState);
    // // cart_update(&curState);
    // if(Serial.available()) serial_receive(curState);
    // axis_update(&curState);
  #elif TEST
    update();
	#endif
}
