#include "fw_pid.h"
#include "pid.h"
#include "ahrs.h"
#include "ibus.h"
#include "maths.h"
#include "pwmwrite.h"

#define MAX_TILT_ANGLE 45  
#define MAX_ANGLULAR_VELOCITY 100 //deg/s
#define MAX_I_VAL  200
#define Dt 0.002f

uint16_t servoLeft;
uint16_t servoRight;

float pitch_I;
float pitchKp = 1;
float pitchKi = 0;
//
float roll_I;
float rollKp = 1;
float rollKi = 0;

// ppi controller
void attitudeController(float target_roll,float target_pitch){
    //pitch
    float pitch_error_p = target_pitch - AHRS.pitch;
    float pitch_error_v = pitch_error_p  -  AHRS.pitch_velocity; 
    float P_pitch = pitch_error_v*pitchKp;
    pitch_I += pitch_error_v*pitchKi*Dt;
    pitch_I = constrainf(pitch_I,-MAX_I_VAL,MAX_I_VAL);
    P_pitch += pitch_I;
    //roll
    float roll_error_p = target_roll - AHRS.roll;
    float roll_error_v = roll_error_p  -  AHRS.roll_velocity; 
    float P_roll = roll_error_v*rollKp;
    roll_I += roll_error_v*rollKi*Dt;
    roll_I = constrainf(roll_I,-MAX_I_VAL,MAX_I_VAL);
    P_roll += roll_I;
    //
    servoLeft  = 1500  - P_roll + P_pitch;
    servoRight = 1500  + P_roll + P_pitch;
    servoLeft  = constrain(servoLeft,1000,2000);
    servoRight = constrain(servoRight,1000,2000);
    writePwm(0,servoLeft);
    writePwm(1,servoRight);
}
void attitudeCrtlReset(){
   roll_I = 0.0f;
   pitch_I = 0.0f;
}
