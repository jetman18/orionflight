#include <stdio.h>
#include <math.h>
#define MAX_CORDINATE 50
#define DEG  57.295779f
typedef struct coord{
   float x;
   float y;
}coord;
int pos_index = 0;
coord Coordinate[] ={
    {1,1},
    {10,10}
};
static float course_over_ground(coord p1, coord p2);
static float distance_airplane_to_next_line(coord P);

int main(){
    coord a ={0,0};
    coord b ={-100,8000};
    float dis = course_over_ground(a, b);

    printf("%f\n",dis);
    return 0;
}

static float distance_airplane_to_next_line(coord P){
    float a_coefficient,b_coefficient,c_coefficient,dis; 
    a_coefficient = Coordinate[pos_index + 1].y - Coordinate[pos_index].y;
    printf("a %f\n",a_coefficient);
    b_coefficient = Coordinate[pos_index].x - Coordinate[pos_index + 1].x;
    printf("b %f\n",b_coefficient);
    c_coefficient = -a_coefficient*Coordinate[pos_index].x - b_coefficient*Coordinate[pos_index].y;
    printf("c %f\n",c_coefficient);
    dis  = fabs(a_coefficient*P.x + b_coefficient*P.y + c_coefficient);
    dis /= sqrtf(a_coefficient*a_coefficient + b_coefficient*b_coefficient);
    return dis;
}
static float course_over_ground(coord p1, coord p2){
     float dx,dy,angle,tan_angle1,tan_angle2;
     dx = p2.x - p1.x;
     dy = p2.y - p1.y;
     tan_angle1 = fabs(atan2(dx,dy)*DEG);
     tan_angle2 = fabs(atan2(dy,dx)*DEG);
     if(dx >= 0 && dy >= 0){
        angle = tan_angle1;
        return angle;
     }
     if(dx >= 0 && dy <= 0){
        angle = 90 + tan_angle2;
        return angle;
     }
     if(dx <= 0 && dy <= 0){
        angle = 90 +  tan_angle2;
        return angle;
     }
     if(dx <= 0 && dy >= 0){
        angle = 360 - tan_angle1;
        return angle;
     }
}