#ifndef PARAM_H
#define PARAM_H

// Parameters 

// PID
float kp_p = 100; // P gain on position
float ki_p = 0; // I gain on position
float kd_p = 0; // D gain on position

float setP_p = 0; // Setpoint (distance from position sensor in m) (if 0, auto setP is enabled. It means it will use initial position as setP)

float deadzone_p = 0.01; // PID active only if out of +/- deadzone (m) WARNING : The bigger the deadzone, the bigger the step the motor will do when it starts
float pidSaturation_p = 22; // Max PID value. In our case, 22 corresponds to the force (N) that the motors can produce for each side
float saturationI_p = 10; // Max I value. You can see it as the number of cm that can be accumulated by the I factor for the error
float fcut_d_p = 15;

float kp_v = 100; // P gain on velocity
float ki_v = 0; // I gain on velocity
float kd_v = 0; // D gain on velocity

float deadzone_v = 0.01; // PID active only if out of +/- deadzone (m) WARNING : The bigger the deadzone, the bigger the step the motor will do when it starts
float pidSaturation_v = 22; // Max PID value. In our case, 22 corresponds to the force (N) that the motors can produce for each side
float saturationI_v = 10; // Max I value. You can see it as the number of cm that can be accumulated by the I factor for the error
float fcut_d_v = 15;

// lowpass
double fcut_p = 15;        // Replace with actual filter cutoff frequency (rad/s)
double fcut_v = 10;        // Replace with actual filter cutoff frequency (rad/s)
double dt = 0.01;        // Replace with actual data timestep

// ESC
const int ESCpin[4] = {15, 33, 27, 12}; // pin for each ESC {ESC1, ESC2, ESC3, ESC4}

#define minPwm 1100 // Pwm for all the motors cannot go under that value
#define maxPwm 1500 // Pwm for all the motors cannot go over that value

#endif // PARAM_H
