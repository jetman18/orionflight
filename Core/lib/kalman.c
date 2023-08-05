#include "kalman.h"
#include "imu.h"
#include "axis.h"
#include "maths.h"
#include "ahrs.h"
#include "pid.h"
#include "gps.h"
#include "bmp280.h"
/***************const parameters****************/
const float gravity_earth = 9.81;              // (m/s^2)
const float force_gain = 0.005;                // (N)
const float quad_weigh = 0.8;                  // (kg)
const float DT_update = 0.002;                 // (Sec)
const float propeller_drag_coefficient = 0;

typedef struct {
   float acc;
   float velocity;
   float displacement;
}es_asix;

es_asix x,y,z;
float longitude,latitude,altitude;
float acc_lon,acc_lat,acc_down;
void estimate_position(){
    float Force_lift,moto_total;
    float acc_temp,delta_acc;
    static float last_acc,acc; 
    /************************************************************/
    moto_total = ((moto1 + moto2 + moto3 + moto4) - 4000);
    Force_lift = moto_total*force_gain - quad_weigh*gravity_earth;
    acc_temp = Force_lift / quad_weigh;
    delta_acc = acc_temp - last_acc;
    last_acc = acc_temp;
    acc += delta_acc * DT_update;
    /* Z axis estimate acceleration */
    z.acc *= acc * dcm[2][2];
    z.acc  = z.acc * 0.9f - AHRS.acc_z*0.1f; // correction
    /* Z axis estimate velocity */
    z.velocity += z.acc*DT_update;
    z.velocity  = z.velocity * 0.9f - bmp280_velocity*0.1f; // correction
    /* Z axis estimate displacement*/
    z.displacement += z.velocity*DT_update;
    z.displacement  = z.displacement * 0.9f - bmp280_altitude*0.1f; // correction

    /* Z axis estimate acceleration */
    y.acc *= acc * dcm[1][2];
    y.acc  = y.acc * 0.9f - AHRS.acc_y*0.1f; // correction
    /* Z axis estimate velocity */
    y.velocity += y.acc*DT_update;
    y.velocity  = y.velocity * 0.9f;// - gps_altitude*0.1f; // correction
    /* Z axis estimate displacement*/
    y.displacement += y.velocity*DT_update;
    y.displacement  = y.displacement * 0.9f;// - gps_altitude*0.1f; // correction
}

