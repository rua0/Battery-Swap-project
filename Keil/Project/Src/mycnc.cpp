
#include "mycnc.h"

//-----------------stepper functions

Stepper::Stepper (int _ID): stepper_id(_ID) {};

void Stepper::set_dir(int dir){
	//write direction to dir_pin and port
	HAL_GPIO_WritePin(Dir_pin_GPIO_Port, Dir_pin_Pin,direction);

}

//blockingly run one step
void Stepper::one_step(int duration){
	this.trigger();
	HAL_Delay(duration);
}

void Stepper::simple_step(int num,int dir,int duration){
	//set direction
	this.set_dir(dir);
	//give $num step triggers
	for (int i = 0; i < num; ++i)
    {
      this.one_step(duration);
    } 
}

void Stepper::get_ID() {
	//print ID
	//cout<<"rua";
}

void Stepper::trigger() {
	//toggle the pin high
    //HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,GPIO_PIN_SET);
    //LED lights as well, because they use the same pin
    HAL_GPIO_WritePin(Step_pin_GPIO_Port, Step_pin_Pin,GPIO_PIN_SET);
    //start timer 9 to toggle the trigger low
    HAL_TIM_Base_Start_IT(&htim9);
}


//-----------------CNC functions