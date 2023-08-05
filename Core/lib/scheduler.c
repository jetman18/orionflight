#include "scheduler.h"
#include "iqr_handler.h"
#include "log.h"
#include "i2c.h"
#include "imu.h"
#include "qmc5883.h"
#include "usart.h"
#include "gpio.h"
#include "scheduler.h"
#include "pwmwrite.h"
#include "pid.h"
#include "bmp280.h"
#include "ibus.h"
#include "gps.h"
#include "hc_sr04.h"
#include "opticalflow.h"
#include "../quadrotor/config.h"
#include "mavlink_handler.h"
#include "mpu6500.h"
#include "ms5611.h"
#include "kalman.h"
#include "ahrs.h"
#define LOOP_US  2000U
#define MAX_LOOP_BREAK_US  1700U

bootTime_t t;
TIM_HandleTypeDef *htimmz;
uint32_t micross;
uint32_t max_excution_time_us = 0;
uint32_t num_tasks = 0;

/*|Tasks list
 *|pr1 function ptr
 *|pr2 execution_time_us
 *|pr3 execution_cycle_us
 *|pr4 last_exec_time_us
 *|pr5 period
 ************************************************/
task_t task[]={ 
    {ahrs_update,      0,0,0,1}, /*  imu task 500 hz*/

  //{pidUpdate,       0,0,0,1},/*  pid task  500 hz*/

  //{pwm2esc,         0,0,0,1},/*  esc task  500 hz*/

  //{ibusGet,         0,0,0,10},/*  receiver task  50 hz*/

 // {optical_flow_run,0,0,0,13},/*  optflow task  40 hz*/

 // {hc_sr04_run,     0,0,0,13},/*  sr-hc04  task  40 hz*/
};

void init_sche(TIM_HandleTypeDef *htimz){
	htimmz = htimz;
	HAL_TIM_Base_Start_IT(htimmz);
	initPWM(&htim4);
	ibusInit(&huart2,115200);
	//mavlinkInit(SYS_ID,0,&huart2,115200);
	mpu6050_init();
	motoIdle();
	//qmc5883_init(&hi2c1);
  bmp280_init();
	PID_init_param();
	//flowInit(&huart1,19200);
	num_tasks  = sizeof(task)/sizeof(task_t);
}

static void wait(){
    static uint32_t time_us;
    while(( micros() - time_us )<LOOP_US);
    time_us = micros();
}

static uint16_t setoverFlow(int val,int flow_val){
    uint8_t k,l;
    l =flow_val + 1;
    k = val/l;
    k = val - (l*k);
    return k;
}
bootTime_t getBootTime(){
	static uint16_t sec_L  =0;
  sec_L = millis()/1000;
	t.sec   = setoverFlow(sec_L,59); 
	t.min   = setoverFlow((sec_L/60),59); 
	t.hour  = setoverFlow((sec_L/3600),23);
	return t;
}

void delay_ms(uint32_t val)
{
	delay_us(val*1000);
}

void delay_us(uint32_t val){
	static uint32_t time_us;
  time_us = micros();
  while((micros() - time_us)<val);
}



void start_scheduler() {
  static int counter = 0;
  uint32_t time_1;
  uint32_t total_execution_time_us = 0;
  for (int i = 0; i < num_tasks; i++){
      if((task[i].exec != NULL) && (counter % task[i].period == 0)){
        time_1 = micros();
        task[i].execution_cycle_us = micros() - task[i].last_exec_time_us;
        task[i].last_exec_time_us = time_1;
        task[i].exec();
        task[i].execution_time_us = micros() - time_1;
        total_execution_time_us += task[i].execution_time_us;
        if(total_execution_time_us > MAX_LOOP_BREAK_US){
          break;
        }
      }
  }
  max_excution_time_us = total_execution_time_us;
  counter ++;
  if(counter == 499) counter = 0;
  wait();   //2000 us
}
