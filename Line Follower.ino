#include <QTRSensors.h>
#include <EEPROM.h>

#define GO_LEFT 1
#define COME_BACK_LEFT 2
#define GO_RIGHT 3
#define COME_BACK_RIGHT 4

const int m11Pin = 7;
const int m12Pin = 6;
const int m21Pin = 5;
const int m22Pin = 4;
const int m1Enable = 11;
const int m2Enable = 10;

const int buttonPin = 2;

int m1Speed = 0;
int m2Speed = 0;

// PID kp ki kd
float kp = 0.26;
float ki = 0; //0.00001;
float kd = 2.55; 

int p = 0;
int i = 0;
int d = 0;

int error = 0;
int lastError = 0;

const int maxSpeed = 255;
const int minSpeed = -100;
const int baseSpeed = 230;

const int calibrateWindow = 5000;
const int calibrationAddress = 0;

QTRSensors qtr;

const int sensorCount = 6;
int sensorValues[sensorCount];

void setup() {
  Serial.begin(9600);

  // pinMode setup
  pinMode(m11Pin, OUTPUT);
  pinMode(m12Pin, OUTPUT);
  pinMode(m21Pin, OUTPUT);
  pinMode(m22Pin, OUTPUT);
  pinMode(m1Enable, OUTPUT);
  pinMode(m2Enable, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  
  pinMode(buttonPin, INPUT_PULLUP);
  
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, sensorCount);

  setMotorSpeed(0, 0);

  // the user has 5 seconds to press the calibrate button
  while (millis() < calibrateWindow) {
    if (buttonPressed()) {
      calibrateQTR();
      break;      
    }  
  }

  getCalibrateFromEEPROM();

  setMotorSpeed(0, 0);
  delay(100);
}

void getCalibrateFromEEPROM() {
  int address = calibrationAddress;
  
  qtr.calibrate();
  // EEPROM.get(calibrationAddress, qtr.calibrationOn);

  EEPROM.get(address, qtr.calibrationOn.initialized);
  address += sizeof(qtr.calibrationOn.initialized);
  
  for (int i=0; i<sensorCount; i++){
    EEPROM.get(address, qtr.calibrationOn.minimum[i]);
    address += sizeof(qtr.calibrationOn.minimum[i]);

    EEPROM.get(address, qtr.calibrationOn.maximum[i]);
    address += sizeof(qtr.calibrationOn.maximum[i]);    
  }

  qtr.calibrate();
}

void saveCalibrateToEEPROM() {
  int address = calibrationAddress;
  
  EEPROM.put(address, qtr.calibrationOn.initialized);
  address += sizeof(qtr.calibrationOn.initialized);
  
  for (int i=0; i<sensorCount; i++){
    EEPROM.put(address, qtr.calibrationOn.minimum[i]);
    address += sizeof(qtr.calibrationOn.minimum[i]);

    EEPROM.put(address, qtr.calibrationOn.maximum[i]);
    address += sizeof(qtr.calibrationOn.maximum[i]);    
  }  
}

// function that checks if the button was pressed using debounce
bool buttonPressed() {
  static const int debounceDelay = 50;
  static int lastDebounceTime = 0;
  static int lastReading = HIGH;
  static int buttonState = HIGH;
  int reading = digitalRead(buttonPin);

  if (reading != lastReading) {
    lastDebounceTime = millis();
  }
  lastReading = reading;

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == HIGH) {
        return true;
      }
    }
  }

  return false;
}

// function that calibrates the QTR sensor
void calibrateQTR() {
  static const int totalCycles = 5;
  static const int blackValueLimit = 600;
  int calibrationSpeedLeft = -200;
  int calibrationSpeedRight = 200;
  int currentState = 1;
  int cycles = 0;

  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  while (cycles < totalCycles) {
    qtr.calibrate();
    qtr.read(sensorValues);

    bool isBlack = false;
    for (int j = 0; j < sensorCount; j ++) {
      if (sensorValues[j] > blackValueLimit) {
        isBlack = true;        
      }
    }
  
    switch (currentState) {
      case GO_LEFT :
        setMotorSpeed(calibrationSpeedLeft, calibrationSpeedRight);
        if (!isBlack) {
          currentState = COME_BACK_LEFT;        
        }
        break;
        
      case COME_BACK_LEFT :
        setMotorSpeed(-calibrationSpeedLeft, -calibrationSpeedRight);
        if (isBlack) {
          currentState = GO_RIGHT;        
        }
        break;

      case GO_RIGHT :
        setMotorSpeed(-calibrationSpeedLeft, -calibrationSpeedRight);
        if (!isBlack) {
          currentState = COME_BACK_RIGHT;        
        }
        break;
        
      case COME_BACK_RIGHT :
        setMotorSpeed(calibrationSpeedLeft, calibrationSpeedRight);
        if (isBlack) {
          currentState = GO_LEFT;  
          cycles ++;      
        }
        break;
    }
  }

  saveCalibrateToEEPROM();

  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  computePID();
  computeMotorSpeed();
}

// function that computes the p,i and d
void computePID() {
  const static int midPosition = 2500;
  int position = qtr.readLineBlack(sensorValues);
  int error = position - midPosition; // [0,5000] -> 2500 is the middle

  p = error;
  i = i + error;
  d = error - lastError;
  lastError = error;
}

// function that computes the m1 and m2 speed and sets the motors
void computeMotorSpeed() {
  int motorSpeed = kp * p + ki * i + kd * d;
  
  m1Speed = baseSpeed;
  m2Speed = baseSpeed;

  m1Speed += motorSpeed;
  m2Speed -= motorSpeed;

  m1Speed = constrain(m1Speed, minSpeed, maxSpeed);
  m2Speed = constrain(m2Speed, minSpeed, maxSpeed);

  setMotorSpeed(m1Speed, m2Speed);
  
  //  DEBUGGING
  //  Serial.print("Error: ");
  //  Serial.println(error);
  //  Serial.print("M1 speed: ");
  //  Serial.println(m1Speed);
  //
  //  Serial.print("M2 speed: ");
  //  Serial.println(m2Speed);
  //
  //  delay(250);
}

// each arguments takes values between -255 and 255. The negative values represent the motor speed in reverse.
void setMotorSpeed(int motor1Speed, int motor2Speed) {
  // remove comment if any of the motors are going in reverse 
  motor1Speed = -motor1Speed;
  motor2Speed = -motor2Speed;
  if (motor1Speed == 0) {
    digitalWrite(m11Pin, LOW);
    digitalWrite(m12Pin, LOW);
    analogWrite(m1Enable, motor1Speed);
  }
  else {
    if (motor1Speed > 0) {
      digitalWrite(m11Pin, HIGH);
      digitalWrite(m12Pin, LOW);
      analogWrite(m1Enable, motor1Speed);
    }
    if (motor1Speed < 0) {
      digitalWrite(m11Pin, LOW);
      digitalWrite(m12Pin, HIGH);
      analogWrite(m1Enable, -motor1Speed);
    }
  }
  if (motor2Speed == 0) {
    digitalWrite(m21Pin, LOW);
    digitalWrite(m22Pin, LOW);
    analogWrite(m2Enable, motor2Speed);
  }
  else {
    if (motor2Speed > 0) {
      digitalWrite(m21Pin, HIGH);
      digitalWrite(m22Pin, LOW);
      analogWrite(m2Enable, motor2Speed);
    }
    if (motor2Speed < 0) {
      digitalWrite(m21Pin, LOW);
      digitalWrite(m22Pin, HIGH);
      analogWrite(m2Enable, -motor2Speed);
    } 
  }
}

