
#include "scheduler.h"
void main_loop(){
	 init_sche(&htim3);
	while(1){
		start_scheduler();
	}
}
