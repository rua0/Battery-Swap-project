basic functions

// determine the timers to use
#define ENCL_TIMER              TIM2
#define ENCL_TIMER_CLK          RCC_APB1Periph_TIM2
#define ENCR_TIMER              TIM4
#define ENCR_TIMER_CLK          RCC_APB1Periph_TIM4

#define LEFT_COUNT()            ENCL_TIMER->CNT
#define RIGHT_COUNT()           ENCR_TIMER->CNT


void gloabal_initial();//initial useful gloabal variables
	//get the following variables
	int? zero_position;
	int? one_position;
	int? step_size;//the number of encoder counts correspondding to one step
	//TBD, tentative

void trigger();//toggle a pin high, start an interrupt to set it to low and disable interrupt
	//for prototyping, just delay works


--------------------
simple means, step without correction/closed-loop
void simple_step(int num,(direction,step_duration));//trigger with a step delay，duration (in ms) 

void simple_step(int num,(direction,step_duration)){
	int step_duration;
	//not change to step direction
	//for each num left 
		//trigger high
		//wait for step duration
}


-----------------------------------
Guaranteed stepping functions

!!!: min_step_duration shouldn't be smaller than the global_min_step_duration obtained from doing a calibration/test, but it could be. And if it is, then the stepper is running at max_speed.

void g_one_step((direction,min_step_duration));//move one step, blockingly; min_step_duration is the least time it takes for a step and it might be longer because a step might take a bit time to complete

void g_step(int num,(direction,step_duration));//move $(num) step blockingly， duration (in ms) 

void g_step_setup();//set up functions for interrupts stuffs; right now only a place holder



//move one step, blockingly; min_step_duration is the least time it takes for a step and it might be longer because a step might take a bit time to complete

void g_one_step((direction,min_step_duration)){
	//variable stuffs
	int? init_cnt=get_encoder_count();
	int? count_inc;
	int? min_inc=step_size*margin;//potentially declare as global
	//int? init_cnt=TIM_ENCODER->CNT;

	//place holder
	g_step_setup();

	trigger();//step trigger, trigger a step movement, automaticall reset
	HAL_Delay(min_step_duration);

	//is this gonna be positive?
	count_inc=get_encoder_count()-init_cnt;
	//assume postive, wait until a step is complete
	while(count_inc<min_inc) ;
}


//move $(num) step blockingly， duration (in ms) 

void g_step(int num,(direction,min_step_duration)){
	//variable stuffs
	int? init_cnt=get_encoder_count();
	int? count_inc;
	int? min_inc=step_size*margin;//potentially declare as global
	//int? init_cnt=TIM_ENCODER->CNT;

	//place holder
	g_step_setup();

	for(int i=0;i<num;i++){
		trigger();//step trigger, trigger a step movement, automaticall reset
		HAL_Delay(min_step_duration);
	
		//is this gonna be positive?
		count_inc=get_encoder_count()-init_cnt;
		//assume postive, wait until a step is complete
		while(count_inc<min_inc) ;
	}
}

---------------------
communication
void? decode_msg(char* buf);
void change_mode();

{
	
}


lower level is the hal
--------------------------


--------------------------
stepper_go_to cmd

--------------------------