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
VL53L4CD_Result_t position;
uint8_t posStatus; // is 0 if position senser see something

// Accelero
Adafruit_LSM6DSOX sox;

// PID
double priError = 0;
double toError = 0;
float pid;

double Pvalue;
double Ivalue;
double Dvalue;

// Lowpass
double filteredDataOld;  // Replace with the initial value of previously filtered data
double positionFiltered;

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

// Function that read accelerometer and update sensor object with new data
void getAccelerometer(){ // TODO Essayer de ne pas déclarer les variables à chaque loop
  // Gather Accelerometer data
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sox.getEvent(&accel, &gyro, &temp);
}

// Function that read position sensor and return sensor object
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
float PID(float dis, double *Pvalue, double *Ivalue, double *Dvalue) {
  float disM = (float)dis/1000;
  double error = setP - disM; 
  float PIDvalue = 0;

  *Pvalue = error * kp;
  *Ivalue = toError * ki;
  *Dvalue = (error - priError) * kd;

  if(*Ivalue <= -saturationI && error < 0){
    *Ivalue = -saturationI;
  }else if (*Ivalue >= saturationI && error > 0){
    *Ivalue = saturationI;
  }else if (*Ivalue <= -saturationI && error > 0){
    *Ivalue = -saturationI;
    toError += error;
  }else if (*Ivalue >= saturationI && error < 0){
    *Ivalue = saturationI;
    toError += error;
  }else{
    toError += error;
  }

  if (fabs(error) >= deadzone){
    PIDvalue = *Pvalue + *Ivalue + *Dvalue;
  }else{
    PIDvalue = 0;
  }

  priError = error;

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

  if (fabs(pid) > 0.52){
    float rpm = (50.0 / 149.0) * (833.0 + std::sqrt(-15338511.0 + 59600000.0 * (fabs(pid))/2)); // Correspond to the curve of our motor thrust in function of RPM
    pwm = 0.0451*rpm + 986; // Correspond to the curve of our motor PWM in function of RPM
  }
  
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
  Serial.print(millis()/1000);
  Serial.print(",");
  Serial.print(positionFiltered/1000-setP);
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
  //Serial.print(sox.accX);
  //Serial.print(",");
  //Serial.print(sox.accY);
  //Serial.print(",");
  //Serial.println(sox.accZ);
  Serial.print(Pvalue);
  Serial.print(",");
  Serial.print(Ivalue);
  Serial.print(",");
  Serial.println(Dvalue);
}

void logHeader(){ 
  Serial.print(kp);
  Serial.print(",");
  Serial.print(ki);
  Serial.print(",");
  Serial.print(kd);
  Serial.print(",");
  Serial.print(setP);
  Serial.print(",");
  Serial.print(0);
  Serial.print(",");
  Serial.print(0);
  Serial.print(",");
  Serial.print(0);
  Serial.print(",");
  Serial.print(0);
  Serial.print(",");
  Serial.print(0);
  Serial.print(",");
  Serial.println(0);
  //Serial.println("Time (ms),Distance (mm),pid (N),pwm 1,pwm 2,pwm 3,pwm 4,acc x (m/s^2),acc y (m/s^2),acc z (m/s^2)");
}

// Lowpass filter
double lowpass(double dataInput, double fcut, double filteredDataOld, double dt) {

    double alpha = dt / (1.0 / fcut + dt); // calculate alpha
    double filteredData = (1 - alpha) * filteredDataOld + dataInput * alpha; // update value

    return filteredData;
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
  if (setP == 0){ // If setP is 0, set setP to initial position
    setP = (float)getPosition().distance_mm/1000;
  }
}

void loop() {
  if(mainStatus == 1){ // Trigger loop at time interrupt
    // mainStatus update
    mainStatus = 0;

    position = getPosition(); // Get position
    positionFiltered = lowpass((double)position.distance_mm, fcut, filteredDataOld, dt); // Filter the position
    Serial.println(positionFiltered);
    filteredDataOld = positionFiltered; // log data to use with the filter next loop

    if (position.range_status == 0){ // Make sure the position sensor see the drone
      getAccelerometer(); // Gather accelerometer data
      pid = PID(positionFiltered, &Pvalue, &Ivalue, &Dvalue); // Calculate PID
      pwm = pidToPwm(pid); // Convert PID to motor commands

      // Send motor commands to ESC
      for (int i = 0; i < 4; i++){
        ESC[i].write(pwm[i]);
      }

      // Log data
      logData();

    }else{ // If position sensor doesn't see the drone, run motors at idle
      for (int i = 0; i < 4; i++){
        ESC[i].write(0);
      }
    }
  }
}
