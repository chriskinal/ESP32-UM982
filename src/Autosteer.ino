#include <Arduino.h>
#include <TaskScheduler.h>
#include <elapsedMillis.h>
#include <EEPROM.h>
#include <Wire.h>
#include "zNMEAParser.h"

#include <WiFiManager.h>
#include "AsyncUDP.h"

/*  PWM Frequency ->
     490hz (default) = 0
     122hz = 1
     3921hz = 2
*/
#define PWM_Frequency 0

/////////////////////////////////////////////

// if not in eeprom, overwrite
#define EEP_Ident 2400

//   ***********  Motor drive connections  **************888
//Connect ground only for cytron, Connect Ground and +5v for IBT2

//Dir1 for Cytron Dir, Both L and R enable for IBT2
#define DIR1_RL_ENABLE 27

//PWM1 for Cytron PWM, Left PWM for IBT2
#define PWM1_LPWM 13

//Not Connected for Cytron, Right PWM for IBT2
#define PWM2_RPWM 14

//--------------------------- Switch Input Pins ------------------------
#define STEERSW_PIN 25
#define WORKSW_PIN 26
#define REMOTE_PIN 33

#define CONST_180_DIVIDED_BY_PI 57.2957795130823

//How many degrees before decreasing Max PWM
#define LOW_HIGH_DEGREES 3.0

#define AUTOSTEER_INTERVAL 100  //read GPIO millisec 50hz

//loop time variables in microseconds
const uint16_t LOOP_TIME = 20;  //40Hz
const uint16_t WATCHDOG_THRESHOLD = 100;
const uint16_t WATCHDOG_FORCE_VALUE = WATCHDOG_THRESHOLD + 2;  // Should be greater than WATCHDOG_THRESHOLD
uint8_t watchdogTimer = WATCHDOG_FORCE_VALUE;

//Define sensor pin for current or pressure sensor
#define LOAD_SENSOR_PIN 39
#define WAS_SENSOR_PIN 36

Scheduler ts;

IPAddress myip;

  // Create WiFiManager object
  WiFiManager wfm;

void imuTask();

void gpsStream();

void inputHandler();

void autosteerLoop();

void commandHandler();

void autoSteerPacketPerser(AsyncUDPPacket udpPacket);

Task imuTS(TASK_IMMEDIATE, TASK_FOREVER, &imuTask, &ts, false);

Task t2(TASK_IMMEDIATE, TASK_FOREVER, &gpsStream, &ts, true);

Task t3(LOOP_TIME, TASK_FOREVER, &inputHandler, &ts, true);

Task t4(AUTOSTEER_INTERVAL, TASK_FOREVER, &autosteerLoop, &ts, true);

Task t5(1000, TASK_FOREVER, &commandHandler, &ts, true);

uint8_t aog2Count = 0;
float sensorReading;
float sensorSample;

elapsedMillis gpsSpeedUpdateTimer = 0;

//EEPROM
int16_t EEread = 0;

//Relays
bool isRelayActiveHigh = true;
uint8_t relay = 0, relayHi = 0, uTurn = 0;
uint8_t tram = 0;

//Switches
uint8_t remoteSwitch = 0, workSwitch = 0, steerSwitch = 1, switchByte = 0;

//On Off
uint8_t guidanceStatus = 0;
uint8_t prevGuidanceStatus = 0;
bool guidanceStatusChanged = false;

//speed sent as *10
float gpsSpeed = 0;
bool GGA_Available = false;  //Do we have GGA on correct port?

const bool invertRoll = true;  //Used for IMU with dual antenna

//pwm variables
int16_t pwmDrive = 0, pwmDisplay = 0;
float pValue = 0;
float errorAbs = 0;
float highLowPerDeg = 0;

//steering variables
float steerAngleActual = 0;
float steerAngleSetPoint = 0;  //the desired angle from AgOpen
int16_t steeringPosition = 0;  //from steering sensor
float steerAngleError = 0;     //setpoint - actual
int16_t helloSteerPosition = 0;

//Steer switch button  ***********************************************************************************************************
uint8_t currentState = 1, reading, previous = 0;
uint8_t pulseCount = 0;  // Steering Wheel Encoder
bool encEnable = false;  //debounce flag
uint8_t thisEnc = 0, lastEnc = 0;


//Variables for settings
struct Storage {
  uint8_t Kp = 40;      // proportional gain
  uint8_t lowPWM = 10;  // band of no action
  int16_t wasOffset = 0;
  uint8_t minPWM = 9;
  uint8_t highPWM = 60;  // max PWM value
  float steerSensorCounts = 30;
  float AckermanFix = 1;  // sent as percent
};
Storage steerSettings;  // 11 bytes

//Variables for settings - 0 is false
struct Setup {
  uint8_t InvertWAS = 0;
  uint8_t IsRelayActiveHigh = 0;  // if zero, active low (default)
  uint8_t MotorDriveDirection = 0;
  uint8_t SingleInputWAS = 1;
  uint8_t CytronDriver = 1;
  uint8_t SteerSwitch = 0;  // 1 if switch selected
  uint8_t SteerButton = 0;  // 1 if button selected
  uint8_t ShaftEncoder = 0;
  uint8_t PressureSensor = 0;
  uint8_t CurrentSensor = 0;
  uint8_t PulseCountMax = 5;
  uint8_t IsDanfoss = 0;
  uint8_t IsUseY_Axis = 0;  //Set to 0 to use X Axis, 1 to use Y avis
};
Setup steerConfig;  // 9 bytes

void steerSettingsInit() {
  // for PWM High to Low interpolator
  highLowPerDeg = ((float)(steerSettings.highPWM - steerSettings.lowPWM)) / LOW_HIGH_DEGREES;
}

void autosteerSetup() {
  //PWM rate settings. Set them both the same!!!!
  /*  PWM Frequency ->
       490hz (default) = 0
       122hz = 1
       3921hz = 2
  */
  if (PWM_Frequency == 0) {
    ledcSetup(PWM1_LPWM, 490, 8);
    ledcSetup(PWM2_RPWM, 490, 8);
  } else if (PWM_Frequency == 1) {
    ledcSetup(PWM1_LPWM, 122, 8);
    ledcSetup(PWM2_RPWM, 122, 8);
  } else if (PWM_Frequency == 2) {
    ledcSetup(PWM1_LPWM, 3921, 8);
    ledcSetup(PWM2_RPWM, 3921, 8);
  }

  pinMode(DIR1_RL_ENABLE, OUTPUT);

  // Disable digital inputs for analog input pins
  //pinMode(CURRENT_SENSOR_PIN, INPUT_DISABLE);
  //pinMode(PRESSURE_SENSOR_PIN, INPUT_DISABLE);

  EEPROM.begin(60);
  EEPROM.get(0, EEread);  // read identifier

  if (EEread != EEP_Ident)  // check on first start and write EEPROM
  {
    EEPROM.put(0, EEP_Ident);
    EEPROM.put(10, steerSettings);
    EEPROM.put(40, steerConfig);
    EEPROM.commit();
  } else {
    EEPROM.get(10, steerSettings);  // read the Settings
    EEPROM.get(40, steerConfig);
  }

}  // End of Setup

void setup() {
  // Setup Serial Monitor
  Serial.begin(115200);
  Serial2.begin(115200);

  initUDP();
  initHandler();

  initIMU();

  initInput();

  autosteerSetup();
}

void loop() {
  ts.execute();
}