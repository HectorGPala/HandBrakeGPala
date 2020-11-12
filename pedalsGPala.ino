#include <Joystick.h>
#include "HX711.h"
#include <Oversample.h>

enum PEDAL_TYPE {
  HALL_SENSOR,
  LOAD_CELL,
  NONE
};

struct PEDAL {
  String description;
  int pin;
  float value;
  PEDAL_TYPE type;
  float minVal;
  float maxVal;
  int minRes;
  int maxRes;
  Oversample* sampler;

  PEDAL(int const p, PEDAL_TYPE t, String s){
    description = String(s);
    pin = p;
    value = 0.0;
    type = t;
    minVal = 9999999.0;
    maxVal = 0.0;
    minRes = 0;
    maxRes = 1023;
    sampler = NULL;
  }
};

//----------------
// Config Zone
//----------------

bool debug = false;
const int deadZone = 20;

struct PEDAL throttle(A3, HALL_SENSOR, "THROTTLE");
struct PEDAL brake(1, LOAD_CELL, "BRAKE");
struct PEDAL clutch(A0, HALL_SENSOR, "CLUTCH");

//----------------------------------------------------
// Extra Data for Load Cell with Hx711
//----------------------------------------------------
int DATA_PIN = 16;
int CLK_PIN = 10;
#define CELL_SCALE 77
int TARE_SAMPLES = 100;

//----------------------------------------------------
// Joystick simulation to Windows
//----------------------------------------------------

uint8_t hidReportId = 0x05;
uint8_t joystickType = JOYSTICK_TYPE_GAMEPAD;
uint8_t buttonCount = 0;
uint8_t hatSwitchCount = 0;
bool includeXAxis = true; // clutch
bool includeYAxis = false;
bool includeZAxis = false;
bool includeRxAxis = false;
bool includeRyAxis = false;
bool includeRzAxis = false;
bool includeRudder = false;
bool includeThrottle = true;
bool includeAccelerator = false;
bool includeBrake = true;
bool includeSteering = false;

Joystick_ joy(hidReportId,
              joystickType,
              buttonCount,
              hatSwitchCount,
              includeXAxis,
              includeYAxis,
              includeZAxis,
              includeRxAxis,
              includeRyAxis,
              includeRzAxis,
              includeRudder,
              includeThrottle,
              includeAccelerator,
              includeBrake,
              includeSteering);


//----------------------------------------------------
// Load Cell interface
//----------------------------------------------------
HX711 cell;

//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------

//------------
//   SETUP
//------------

void setup_pedal(PEDAL &p){
  switch(p.type){
    case HALL_SENSOR:
      pinMode(p.pin, INPUT);
      p.sampler = new Oversample(p.pin, 12);
      break;
    case LOAD_CELL:
      
      break;
    default:
      break;  
  }
}

void setupPedals(){
  setup_pedal(throttle);
  setup_pedal(brake);
  setup_pedal(clutch);
//  throttle.minVal = 51215.72;
//  throttle.maxVal = 222466.03;
//  clutch.minVal = 59556.68;
//  clutch.maxVal = 175548.14;
}

void setup(){
  if(!debug){
    joy.begin(false);
    joy.setThrottleRange(throttle.minRes, throttle.maxRes);
    joy.setBrakeRange(brake.minRes, brake.maxRes);
    joy.setXAxisRange(clutch.minRes, clutch.maxRes);
  }
  else{
    Serial.begin(9600);
    Serial.println("Debug mode is ready...");  
    delay(1000);
  }
  setupPedals();
  cell.begin(DATA_PIN, CLK_PIN);
  cell.set_scale(CELL_SCALE);
  cell.tare(TARE_SAMPLES);
}  

void readPedal(PEDAL &p){
  switch(p.type){
    case LOAD_CELL:
      p.value = cell.read();
      p.minVal = 270703.00;
      p.maxVal = 1760304.00;
      //updateMinMax(p);
      //Serial.println("Read...");
      break;
    case HALL_SENSOR:
      // Read analog voltage
      int32_t oversampling = 0;
      double hallVolt = 0.0;
//      for(int i = 0; i < 255; i++){
        hallVolt = p.sampler->read();
//        oversampling += hallVolt;
//      }
//      hallVolt = (oversampling>>4);
      // Convert to mV
      float halt_mV = hallVolt * 5000.0 / 1023;
      //float halt_mV = hallVolt / 1023;
      // Convert to magnet flux 49E
      float hallFlux = halt_mV * 53.33 - 133.3;
      p.value = hallFlux;
      updateMinMax(p);
      break;
    default:
      break;  
  }
}

void updateMinMax(PEDAL &p){
  if (p.value < p.minVal)
    p.minVal = p.value;
  else if (p.value > p.maxVal)
    p.maxVal = p.value;
}

void readPedals(){
  readPedal(throttle);
  readPedal(brake);
  readPedal(clutch);  
}

void logPedal(PEDAL p){
  Serial.print(p.description);
  Serial.print(" MAX: ");
  Serial.print(p.maxVal);
  Serial.print(" MIN: ");
  Serial.print(p.minVal);
  Serial.print(" VAL: ");
  Serial.println(p.value);  
}

void plotPedals(){
  plotPedal(throttle);
  plotPedal(brake);
  plotPedal(clutch);  
}

void plotPedal(PEDAL p){
  Serial.print(p.description);
  Serial.print(" MAX: ");
  Serial.print(p.maxRes);
  Serial.print(" MIN: ");
  Serial.print(p.minRes);
  Serial.print(" VAL: ");
  int pedalVal = map(p.value, p.minVal, p.maxVal, p.minRes, p.maxRes);
  if(pedalVal < deadZone)
    pedalVal = 0;
  Serial.println(pedalVal);  
}

void chivatoPedals(){
  chivatoPedal(throttle);
  chivatoPedal(brake);
  chivatoPedal(clutch);  
}
void chivatoPedal(PEDAL p){
  int pedalVal = map(p.value, p.minVal, p.maxVal, p.minRes, p.maxRes);
  if(pedalVal >= deadZone){
  Serial.print(p.description);
  Serial.print(" MAX: ");
  Serial.print(p.maxRes);
  Serial.print(" MIN: ");
  Serial.print(p.minRes);
  Serial.print(" VAL: ");
  
  Serial.println(pedalVal);
  } 
}
void logPedals(){
  //logPedal(throttle);
  //logPedal(brake);
  logPedal(clutch);
}

int convertToJoystick(PEDAL p){
  if (p.value > p.maxVal)
    p.value = p.maxVal;
  int pedalVal = map(p.value, p.minVal, p.maxVal, p.minRes, p.maxRes);
  if(pedalVal < deadZone)
    pedalVal = 0;

  return pedalVal;    
}

void updateJoystick(){
  int throttleVal = convertToJoystick(throttle);
  joy.setThrottle(throttleVal);

  int brakeVal = convertToJoystick(brake);
  joy.setBrake(brakeVal);

  int clutchVal = convertToJoystick(clutch);
  joy.setXAxis(clutchVal);

  joy.sendState();
}

void pedal_loop(){
  readPedals();
  if(debug)
    logPedals();
  else
    updateJoystick();
    //chivatoPedals();
  //plotPedals();
}

void loop() {
  pedal_loop();
}
