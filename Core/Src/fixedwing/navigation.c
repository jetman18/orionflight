#include "navigation.h"
#include "math.h"
#include "fw_pid.h"
#define DEG  57.295779f
#define MAX_POINT 50

int coord_counter = 0;

typedef struct coord{
   float x;
   float y;
}coord;

float plane_heading;  // 0-360
coord Coordinate[MAX_POINT];
coord GPS_data;
static float courseOverGround(coord p1, coord p2);
static float angleToNextPoint(coord P);
static float distanceAirplaneToNextLine(coord pos1,coord pos2, coord P);
static float distanceBetweenTwoPoint(coord pos1, coord pos2);


// Goc bay toi toa do tiep theo, la diem dau
// cua duong thang tao boi hai diem
static float angleToNextPoint(coord P){
   float target_line_angle,angle_plane_to_line;
   target_line_angle = courseOverGround(Coordinate[coord_counter],Coordinate[coord_counter + 1]);
   angle_plane_to_line = courseOverGround(GPS_data,Coordinate[coord_counter]);
}
// Huong di chuyen so voi mat dat tinh tu hai toa do 
static float courseOverGround(coord p1, coord p2){
     float dx,dy,angle;
     float atanI,atanII;
     dx = p2.x - p1.x;
     dy = p2.y - p1.y;
     atanI  = fabs(atan2(dx,dy)*DEG);
     atanII = fabs(atan2(dy,dx)*DEG);
     if(dx >= 0 && dy >= 0){
        angle = atanI;
        return angle;
     }
     if(dx >= 0 && dy <= 0){
        angle = 90 + atanII;
        return angle;
     }
     if(dx <= 0 && dy <= 0){
        angle = 90 + atanII;
        return angle;
     }
     if(dx <= 0 && dy >= 0){
        angle = 360 - atanI;
        return angle;
     }
}
// khoang cach tu may bay den duong thang tao boi
// hai toa do tiep theo , don vi (m)
static float distanceAirplaneToNextLine(coord pos1,coord pos2, coord P){
   float a_coefficient,b_coefficient,c_coefficient,dis; 
   a_coefficient =  pos2.y -  pos1.y;
   b_coefficient =  pos1.x -  pos2.x;
   c_coefficient = -a_coefficient* pos1.x - b_coefficient* pos1.y;
   dis  = fabs(a_coefficient*P.x + b_coefficient*P.y + c_coefficient);
   dis /= sqrtf(a_coefficient*a_coefficient + b_coefficient*b_coefficient);
   return dis;
}

//khoang cach giua hai diem toa do , don vi (m)
static float distanceBetweenTwoPoint(coord pos1, coord pos2){
   float a_temp = pos2.x - pos1.x;
   float b_temp = pos2.y - pos1.y;
   float dis = sqrtf(a_temp*a_temp + b_temp*b_temp);
   return dis;
}

