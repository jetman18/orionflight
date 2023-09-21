#include "position.h"
#include "../lib/imu.h"
#include "../lib/axis.h"
#include "../lib/maths.h"
#include "../lib/ahrs.h"
#include "../lib/pid.h"
#include "../lib/gps.h"
#include "../lib/bmp280.h"
#include "scheduler.h"
#define TOSEC (1e-06f)
/***************const parameters****************/
const float gravity_earth = 9.81;              // (m/s^2)
const float force_gain = 0.005;                // (N)
const float quad_weigh = 0.8;                  // (kg)
const float propeller_drag_coefficient = 0;

typedef struct {
   float acc;
   float velocity;
   float displacement;
}es_asix;

es_asix North,East,Up;
float longitude,latitude,altitude;
float acc_lon,acc_lat,acc_down;
float DT_update; 
/*
void estimate_position(){
    float Force_lift,moto_total;
    float acc_temp,delta_acc;
    static float last_acc,acc; 
    static uint32_t last_time;

    DT_update = micros() - last_time;
    DT_update *= TOSEC;
    last_time = micros();
    moto_total = moto1 + moto2 + moto3 + moto4 - 4000;
    Force_lift = moto_total*force_gain - quad_weigh*gravity_earth;
    acc_temp = Force_lift / quad_weigh;
    // ---- use acceleration correct
   // delta_acc = acc_temp - last_acc;
    //last_acc = acc_temp;
    //acc += delta_acc * DT_update;
    // Up axis estimate acceleration
   // Up.acc  = acc * dcm[2][2];
    //Up.acc  = Up.acc * 0.9f - AHRS.acc_z*0.1f; // correct
    // Z axis estimate displacement
    //Up.displacement +=  Up.velocity*DT_update + Up.acc * sq(DT_update)/2.0f;
    //Up.displacement  = Up.displacement * 0.9f - bmp280_altitude * 0.1f; // correct
    // Z axis estimate velocity
    //Up.velocity += Up.acc*DT_update;
    //Up.velocity  = Up.velocity * 0.9f - bmp280_velocity*0.1f; // correct
    
    // North axis estimate acceleration
    // North.acc  = acc * dcm[1][2];
    // North.acc  = North.acc * 0.9f - AHRS.acc_y*0.1f; // correct
    // North axis estimate velocity
    // North.velocity += North.acc*DT_update;
    // North.velocity  = North.velocity * 0.9f;// - gps_altitude*0.1f; // correct
    // North axis estimate displacement
    // North.displacement += North.velocity*DT_update;
    // North.displacement  = North.displacement * 0.9f;// - gps_altitude*0.1f; // correct


}

static void distance_from_cordinate(){

}
static float velocity_from_2cordinate(){
    
}
*/
