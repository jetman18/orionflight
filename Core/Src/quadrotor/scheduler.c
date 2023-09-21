//#include "scheduler.h"
//#include "iqr_handler.h"
//#include "log.h"
//#include "imu.h"
//#include "qmc5883.h"
//#include "usart.h"
//#include "gpio.h"
//#include "scheduler.h"
//#include "pwmwrite.h"
//#include "pid.h"
//#include "ms5611.h"
//#include "ibus.h"
//#include "gps.h"
//#include "hc_sr04.h"
//#include "opticalflow.h"
//#include "config.h"
//#include "mavlink_handler.h"
//#include "mpu6500.h"
//#include "ms5611.h"
//#include "position.h"
//#include "ahrs.h"
//#include  "sensordetect.h"
//#define LOOP_US  2000U
//#define MAX_LOOP_BREAK_US  1800U
//
//uint32_t max_excution_time_us = 0;
//uint32_t num_tasks = 0;
//
//
//task_t task[]={
//    {ahrs_update,      0,0,0,1}, /*  imu task 500 hz*/
//
//  //{pidUpdate,       0,0,0,1},/*  pid task  500 hz*/
//
//	//{ms5611_start,         0,0,0,2},/*  pid task  5 hz*/
//
//  //{pwm2esc,         0,0,0,1},/*  esc task  500 hz*/
//
//  //{ibusGet,         0,0,0,10},/*  receiver task  50 hz*/
//
// // {optical_flow_run,0,0,0,13},/*  optflow task  40 hz*/
//
// // {hc_sr04_run,     0,0,0,13},/*  sr-hc04  task  40 hz*/
//};
//
//int16_t gyx_offset;
//void init_sche(){
//	i2cDectect(&hi2c2);
//	//ms5611_init(&hi2c2);
//	initPWM(&htim3);
//	//ibusInit(&huart2,115200);
//	//mavlinkInit(SYS_ID,0,&huart2,115200);
//	mpu_init();
//	motoIdle();
//	//qmc5883_init(&hi2c2);
//    //bmp280_init();
//	PID_init_param();
//	num_tasks  = sizeof(task)/sizeof(task_t);
//}
//
//static void wait(){
//    static uint32_t time_us;
//    while(( micros() - time_us )<LOOP_US);
//    time_us = micros();
//}
//
//
//void start_scheduler() {
//  static int counter = 0;
//  uint32_t time_1;
//  uint32_t total_execution_time_us = 0;
//  for (int i = 0; i < num_tasks; i++){
//      if((task[i].exec != NULL) && (counter % task[i].period == 0)){
//        time_1 = micros();
//        task[i].execution_cycle_us = micros() - task[i].last_exec_time_us;
//        task[i].last_exec_time_us = time_1;
//        task[i].exec();
//        task[i].execution_time_us = micros() - time_1;
//        total_execution_time_us += task[i].execution_time_us;
//        if(total_execution_time_us > MAX_LOOP_BREAK_US){
//          break;
//        }
//      }
//  }
//  max_excution_time_us = total_execution_time_us;
//  counter ++;
//  if(counter == 499) counter = 0;
//  wait();
//}
//
