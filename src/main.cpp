#include <param.h>
#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cd_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>
#include <SPI.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <cmath>
#include <bits/stdc++.h> 

using namespace std; 

// Parameters (not user choice)

// Distance sensor
#define DEV_I2C Wire
VL53L4CD sensor_vl53l4cd_sat(&DEV_I2C, A1);
VL53L4CD_Result_t TOF_sensor;
uint8_t posStatus; // is 0 if TOF_sensor senser see something

// Accelero
Adafruit_LSM6DSOX sox;

// PID
double time_ck[2] = {0, 0};
double priError_p = 0;
double priError_v = 0;
double toError_p = 0;
double toError_v = 0;
float setP_v; // Velocity setpoint for second PID
float pid;

double Pvalue_p;
double Ivalue_p;
double Dvalue_p;

float velocity = 0;
float velocityFiltered = 0;
float oldVelocity = 0;

double Pvalue_v;
double Ivalue_v;
double Dvalue_v;

double a = 0.000098393;
double b = -0.2099975;
double c = 115.6317857;

// Lowpass
double oldPosition = setP_p;
double positionFiltered;
double position;

double dFiltered;
double dFilteredOld;

// ESC
Servo ESC[4]; // create servo object to control the ESC
int* pwm; // Array of pwms

// Time interrupt
hw_timer_t *mainTimer = NULL;
bool mainStatus = 0;

// Function that run on time interrupt
void IRAM_ATTR onMainTimer(){
  mainStatus = 1;
}

// Lowpass filter
double lowpass(double dataInput, double fcut, double oldData, double dt) {

    double alpha = dt / (1.0 / fcut + dt); // calculate alpha
    double filteredData = (1 - alpha) * oldData + dataInput * alpha; // update value

    return filteredData;
}

// Function that read accelerometer and update sensor object with new data
void getAccelerometer(){ // TODO Essayer de ne pas déclarer les variables à chaque loop
  // Gather Accelerometer data
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sox.getEvent(&accel, &gyro, &temp);
}

// Function that read TOF_sensor sensor and return sensor object
VL53L4CD_Result_t getPosition(){
// Gather distance from distance sensor
  uint8_t NewDataReady = 0;
  VL53L4CD_Result_t results;
  uint8_t status;

  do {
    status = sensor_vl53l4cd_sat.VL53L4CD_CheckForDataReady(&NewDataReady);
  } while (!NewDataReady);

  if ((!status) && (NewDataReady != 0)) {

    // (Mandatory) Clear HW interrupt to restart measurements
    sensor_vl53l4cd_sat.VL53L4CD_ClearInterrupt();

    // Read measured distance. RangeStatus = 0 means valid data
    sensor_vl53l4cd_sat.VL53L4CD_GetResult(&results);
  }

  return results;
}

// Function that calculate PID (in Newton needed to accelerate the drone toward setpoint)
float PID(float setP, float actualData, double *Pvalue, double *Ivalue, double *Dvalue, float kp, float kd, float ki, float saturationI, float pidSaturation, float deadzone, float fcut_d, double *priError, double *toError) {
  double error = setP - actualData; 
  float PIDvalue = 0;

  *Pvalue = error * kp;
  *Ivalue = *toError * ki;

  double dPreGain = error - *priError;

  if (kd != 0){
    dFiltered = lowpass(dPreGain, fcut_d, dFilteredOld, dt);
    dFilteredOld = dFiltered;
  } else{
    dFiltered = dPreGain;
  }
  
  *Dvalue = dFiltered * kd;
 
  if(*Ivalue <= -saturationI && error < 0){
    *Ivalue = -saturationI;
  }else if (*Ivalue >= saturationI && error > 0){
    *Ivalue = saturationI;
  }else if (*Ivalue <= -saturationI && error > 0){
    *Ivalue = -saturationI;
    *toError += error;
  }else if (*Ivalue >= saturationI && error < 0){
    *Ivalue = saturationI;
    *toError += error;
  }else{
    *toError += error;
  }

  if (fabs(error) >= deadzone){
    PIDvalue = *Pvalue + *Ivalue + *Dvalue;
  }else{
    PIDvalue = 0;
  }

  *priError = error;

  if(PIDvalue < -pidSaturation){
    PIDvalue = -pidSaturation;
  }else if (PIDvalue > pidSaturation){
    PIDvalue = pidSaturation;
  }

  return PIDvalue;
}

// Function that convert the PID (force) in motor command (PWM) that produces that force
int* pidToPwm(float pid) {
  int* result = new int[4];
  int pwm = minPwm;

  if (std::abs(pid) > 3.6)
      pwm = -(b - std::sqrt(b * b - 4 * a * (c - std::abs(pid)))) / (2 * a);
  else
      pwm = 1100;
  
  if (pid < 0 && pwm > minPwm && pwm < maxPwm){
    result[0] = pwm;
    result[1] = minPwm;
    result[2] = minPwm;
    result[3] = pwm;
  } else if (pid > 0 && pwm > minPwm & pwm < maxPwm){
    result[0] = minPwm;
    result[1] = pwm;
    result[2] = pwm;
    result[3] = minPwm;
  }else if (pid > 0 && pwm > minPwm & pwm > maxPwm){
    result[0] = minPwm;
    result[1] = maxPwm;
    result[2] = maxPwm;
    result[3] = minPwm;
  }else if (pid < 0 && pwm > minPwm && pwm > maxPwm){
    result[0] = maxPwm;
    result[1] = minPwm;
    result[2] = minPwm;
    result[3] = maxPwm;
  }else{
    result[0] = minPwm;
    result[1] = minPwm;
    result[2] = minPwm;
    result[3] = minPwm;
  }
  
  return result;

}

// Functions that print all relevent data in the serial monitor
void logData(){
  Serial.print((double)millis()/1000);
  Serial.print(",");
  Serial.print(positionFiltered - setP_p, 5);
  Serial.print(",");
  Serial.print(pid);
  Serial.print(",");
  Serial.print(pwm[0]);
  Serial.print(",");
  Serial.print(pwm[1]);
  Serial.print(",");
  Serial.print(pwm[2]);
  Serial.print(",");
  Serial.print(pwm[3]);
  Serial.print(",");
  Serial.print(sox.accX);
  Serial.print(",");
  Serial.print(sox.accY);
  Serial.print(",");
  Serial.print(sox.accZ);
  Serial.print(",");
  Serial.print(Pvalue_p);
  Serial.print(",");
  Serial.print(velocityFiltered);
  Serial.print(",");
  Serial.print(Pvalue_v);
  Serial.print(",");
  Serial.print(Ivalue_v);
  Serial.print(",");
  Serial.print(Dvalue_v);
  Serial.print("\n");
}

void logHeader(){ 
  Serial.print(kp_p);
  Serial.print(",");
  Serial.print(kp_v);
  Serial.print(",");
  Serial.print(ki_v);
  Serial.print(",");
  Serial.print(kd_v);
  Serial.print(",");
  Serial.println(setP_p);
  //Serial.println("Time (ms),Distance (mm),pid (N),pwm 1,pwm 2,pwm 3,pwm 4,acc x (m/s^2),acc y (m/s^2),acc z (m/s^2)");
}

void setup() {
  Serial.begin(115200);

  // log p-i-d gains
  logHeader();

  // Distance sensor settup
  DEV_I2C.begin();
  sensor_vl53l4cd_sat.begin();
  sensor_vl53l4cd_sat.VL53L4CD_Off();
  sensor_vl53l4cd_sat.InitSensor();
  sensor_vl53l4cd_sat.VL53L4CD_SetRangeTiming(10, 0); // Program the highest possible TimingBudget
  sensor_vl53l4cd_sat.VL53L4CD_StartRanging();

  // Accelerometer setup
  if (!sox.begin_I2C()) {
    while (1) {
      delay(10);
    }
  }

  sox.setAccelRange(LSM6DS_ACCEL_RANGE_2_G); // Accelerometer and gyro range and frequency configuration (See doc for choices)
  sox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );
  sox.setAccelDataRate(LSM6DS_RATE_416_HZ);
  sox.setGyroDataRate(LSM6DS_RATE_416_HZ);

  // ESC
  for (int i = 0; i < 4; i++){
    pinMode(ESCpin[i], OUTPUT);
    ESC[i].setPeriodHertz(50);
    ESC[i].attach(ESCpin[i],1000,2000); // (pin, min pulse width, max pulse width in microseconds)
    ESC[i].write(1000);    // Send the signal to the ESC
  }
  delay(2000);

  // Time interrupt
  mainTimer = timerBegin(1, 80, true); // Prescaler set to 80 (80MHz / 80 = 1MHz)
  timerAttachInterrupt(mainTimer, &onMainTimer, true); // Attach onMainTimer function to the timer interrupt
  timerAlarmWrite(mainTimer, 10000, true); // Set timer to 1 / 10 000 micro seconds (100 Hz). If prescaler change, this will change too
  timerAlarmEnable(mainTimer);

  // Setpoint
  if (setP_p == 0){ // If setP is 0, set setP to initial TOF_sensor
    setP_p = (float)getPosition().distance_mm/1000;
    oldPosition = setP_p * 1000;   // Set initial TOF_sensor for the TOF_sensor to the actual TOF_sensor
  }
}

void loop() {
  if(mainStatus == 1){ // Trigger loop at time interrupt
    // mainStatus update
    mainStatus = 0;
    time_ck[0] = (double)millis()/1000;

    TOF_sensor = getPosition(); // Get TOF_sensor
    position = (double)TOF_sensor.distance_mm/1000;
    positionFiltered = lowpass((double)position, fcut_p, oldPosition, dt); // Filter the TOF_sensor

    velocity = (positionFiltered-oldPosition)/(time_ck[0]-time_ck[1]);
    velocityFiltered = lowpass((double)velocity, fcut_v, oldVelocity, dt); // Filter the velocity

    if (TOF_sensor.range_status == 0){ // Make sure the TOF_sensor sensor see the drone
      getAccelerometer(); // Gather accelerometer data
      setP_v = PID(setP_p, positionFiltered, &Pvalue_p, &Ivalue_p, &Dvalue_p, kp_p, kd_p, ki_p, saturationI_p, pidSaturation_p, deadzone_p, fcut_d_p, &priError_p, &toError_p); // Calculate PID on TOF_sensor (used as velocity setpoint)
      pid = PID(setP_v, velocityFiltered, &Pvalue_v, &Ivalue_v, &Dvalue_v, kp_v, kd_v, ki_v, saturationI_v, pidSaturation_v, deadzone_v, fcut_d_v, &priError_v, &toError_v); // Calculate PID on velocity
      pwm = pidToPwm(pid); // Convert PID to motor commands

      if((double)millis()/1000 > 5){
        // Send motor commands to ESC
        for (int i = 0; i < 4; i++){
          ESC[i].write(pwm[i]);
        }
      }

      // Log data
      logData();

      oldPosition = positionFiltered; // log old position
      oldVelocity = velocityFiltered; // log old velocity
      time_ck[1] = time_ck[0]; // log old velocity

    }else{ // If TOF sensor doesn't see the drone, run motors at idle
      for (int i = 0; i < 4; i++){
        ESC[i].write(0);
      }
    }
  }
}