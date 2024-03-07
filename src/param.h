#ifndef PARAM_H
#define PARAM_H

// Parameters 

// PID
float kp = 150; // P gain
float ki = 0; // I gain
float kd = 5000; // D gain

#define deadzone 0.05 // PID active only if out of +/- deadzone (cm) WARNING : The bigger the deadzone, the bigger the step the motor will do when it starts

#define pidSaturation 35 // Max PID value. In our case, 35 corresponds to the force (N) that the motors can produce for each side
#define saturationI 0.16 // Max I value. You can see it as the number of cm that can be accumulated by the I factor for the error

float setP = 0.4; // Setpoint (distance from position sensor in m) (if 0, auto setP is enabled. It means it will use initial position as setP)

// lowpass
double fcut = 20;        // Replace with actual filter cutoff frequency (rad/s)
double dt = 0.01;        // Replace with actual data timestep

// ESC
const int ESCpin[4] = {15, 33, 27, 12}; // pin for each ESC {ESC1, ESC2, ESC3, ESC4}

#define minPwm 1100 // Pwm for all the motors cannot go under that value
#define maxPwm 1500 // Pwm for all the motors cannot go over that value

#endif // PARAM_H
