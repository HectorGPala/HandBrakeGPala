#include <Joystick.h>
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
    minVal = 9999999;
    maxVal = -9999999;
    minRes = 0;
    maxRes = 1023;
    sampler = NULL;
  }
};

//----------------
// Config Zone
//----------------

bool logPedalsFlag = true;
bool debugBrake = false;
bool debugFlag = debugBrake;
const int deadZone = 20;
int lastBrakeVal = 0;
int switchPin = 6;

// Brake
struct PEDAL brake(A1, HALL_SENSOR, "BRAKE");

//----------------------------------------------------
// Joystick simulation to Windows
//----------------------------------------------------

uint8_t hidReportId = 0x06;
uint8_t joystickType = JOYSTICK_TYPE_JOYSTICK;
uint8_t buttonCount = 1;
uint8_t hatSwitchCount = 0;
bool includeXAxis = false;
bool includeYAxis = false;
bool includeZAxis = false;
bool includeRxAxis = false;
bool includeRyAxis = false;
bool includeRzAxis = false;
bool includeRudder = false;
bool includeThrottle = false;
bool includeAccelerator = false;
bool includeBrake = true;
bool includeSteering = false;

bool analogUse = true;

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
      p.sampler = new Oversample(p.pin, 16);
      break;
    case LOAD_CELL:
		p.minRes = -32767;
		p.maxRes = 32767;
		if(!logPedalsFlag){
			p.minVal = 290299.00;//305960.00;
			p.maxVal = 3015929.00;//1760304.00;
		}
      break;
    default:
      break;  
  }
}

void setupPedals(){
  setup_pedal(brake);
}

void setupSwitch(){
	pinMode(switchPin, INPUT);
	pinMode(switchPin-1, OUTPUT);
	pinMode(switchPin+1, OUTPUT);
	digitalWrite(switchPin-1, HIGH);
	digitalWrite(switchPin+1, LOW);
}

void setup(){
  if(!debugFlag){
    joy.begin(false);
	setupPedals();
	setupSwitch();
    joy.setBrakeRange(brake.minRes, brake.maxRes);
  }
  else{
    Serial.begin(9600);
    Serial.println("Debug mode is ready...");  
    delay(1000);
  }
} 

void detectAnalog(){
	if(digitalRead(switchPin) != 0)
		analogUse = false;
	else
		analogUse = true;
}

void readPedal(PEDAL &p){
  switch(p.type){
     case HALL_SENSOR:
      // Read analog voltage
      double hallVolt;
      hallVolt = p.sampler->read();
      // Convert to mV
      float halt_mV;
      halt_mV = hallVolt * 5000.0 / 1023;
      // Convert to magnet flux 49E
      float hallFlux;
      hallFlux = halt_mV * 53.33 - 133.3;
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
  readPedal(brake);
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

double mapFixed(long x, long in_min, long in_max, long out_min, long out_max) {	
	if(x < in_min)
		return out_min;
	else if(x > in_max)
		return out_max;
	else {
		double slope = 1.0 * (out_max - out_min) / (in_max - in_min);
		return out_min + slope * (x - in_min);
	}
}

void plotPedals(){
  if(debugBrake)
	plotPedal(brake);
}

void plotPedal(PEDAL p){
  Serial.print(p.description);
  Serial.print(" MAX: ");
  Serial.print(p.maxRes);
  Serial.print(" MIN: ");
  Serial.print(p.minRes);
  Serial.print(" VAL: ");
  int pedalVal = mapFixed(p.value, p.minVal, p.maxVal, p.minRes, p.maxRes);
  if(pedalVal < p.minRes + deadZone)
    pedalVal = p.minRes;
  Serial.println(pedalVal);  
}

void logPedals(){
  if(debugBrake)
	logPedal(brake);
}




int convertToJoystick(PEDAL p){
	int pedalVal = 0;
	switch(p.type){
    case LOAD_CELL:
	  pedalVal = mapFixed(p.value, p.minVal, p.maxVal, p.minRes, p.maxRes);
      break;
    case HALL_SENSOR:
		pedalVal = mapFixed(p.value, p.minVal, p.maxVal, p.minRes, p.maxRes);
      break;
    default:
      break;  
	}  
	if(pedalVal < p.minRes + deadZone)
		pedalVal = p.minRes;
  
	return pedalVal;    
}

void updateJoystick(){
  int brakeVal = convertToJoystick(brake);
  if(analogUse){
	joy.setBrake(brake.maxRes-brakeVal);
	joy.setButton(0, 0);
  }
  else {
	joy.setBrake(0);
    if(brakeVal > brake.maxRes * 0.2)
		joy.setButton(0, 0); // digital brake
	else
		joy.setButton(0, 1); // digital dont brake
  }

  joy.sendState();
}

void pedal_loop(){
  readPedals();
  if(debugFlag)
	if(logPedalsFlag)
		logPedals();
	else
		plotPedals();
  else {
	detectAnalog();
    updateJoystick();
  }
}

void loop() {
  pedal_loop();
}
