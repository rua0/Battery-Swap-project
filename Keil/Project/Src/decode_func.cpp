#include "decode_func.h"

void test(){
	//light up an LED maybe
}


void decode_msg(uint8_t end_char_index);
/*
void decode_msg(uint8_t end_char_index){
  int new_cmd_id=CMD_buf[0];
  char debug_msg[40];
  sprintf (debug_msg, "line 162 decode_msg called %d\n",end_char_index);
  HAL_UART_Transmit(&huart2, (uint8_t *)debug_msg, strlen(debug_msg) ,9999);//30 is buffer size
  //if short cmd such as ask status
  if(isShort(new_cmd_id)){
    exe_short_cmd();
    return;
  }
  //not short cmd

  //if there is an ongoing cmd
  if (Cur_CMD)
  {
    if (!areCompatible(Cur_CMD,new_cmd_id))
    {
      //causing conflict
      //print error msg
      HAL_UART_Transmit(&huart2, (uint8_t *)Cmd_cflt_err_msg, strlen(Cmd_cflt_err_msg),9999);
      //it really should be transmit_IT here to nonblockingly transmit, 
        //but using blocking transmit 
        //and to see this communicatio delay would be interesting
      //return keep executing current cmd
      return;
    }else{
      //is compatible
      Cur_CMD=new_cmd_id;
    }
  }else{
    //if no ongoing cmd
    //??activate CMD_TO_EXE flag?
    //is this needed
    //CMD_TO_EXE=1;
    Cur_CMD=new_cmd_id;
  }
  //if cmd is executed here, then parameter can be updated
  update_target(Cur_CMD);
  }
*/
bool isShort(int cmd_id);
/*
bool isShort(int cmd_id){
    switch(cmd_id){
      case STEP_CMD_ID:
        return false;
      case TRIGGER_CMD_ID:
        return false;
      //step cmd with all parameters
      case SIMPLE_STEP_ALL_CMD_ID:
        return false;
      default: 
        return false;
    }
*/
void exe_cmd(void);

/*
void exe_cmd(){
    #ifdef EXE_CMD_DEBUG
    //debug mode: print the received cmd
      char debug_msg[40];
      sprintf (debug_msg, "CMD executing %d\n",cmd_buf[0]);
      HAL_UART_Transmit(&huart2, (uint8_t *)debug_msg, strlen(debug_msg) ,9999);//30 is buffer size
    #endif
    switch(Cur_CMD){
      case TRIGGER_CMD_ID:
        trigger();
        Cur_CMD=0;//reset the current cmd
      break;
      case SIMPLE_STEP_ALL_CMD_ID:{
				uint32_t step_duration=CMD_buf[3]*CMD_buf[4];
        simple_step(CMD_buf[1],CMD_buf[2],step_duration);
        //(int num,uint8_t direction,int step_duration) 
			}
      break;
      case SIMPLE_GO_TO_CMD_ID:{
        int target_step=CMD_buf[1];
        int index=2;
        while(CMD_buf[index]!='$'){
        	target_step=target_step*10+CMD_buf[index];
        	index++;
        }

        go_to_step(target_step);
      }
      break;
      case CALIBRATION_CMD_ID:
      calibration();
      break;
    }
		//finished current CMD
		//Reset the CMD TO EXE
		Cur_CMD=0;
		char done_msg[]="$";
		HAL_UART_Transmit_IT(&huart2, (uint8_t *)done_msg, 1);//
  }
*/

// void exe_short_cmd(){
  
//   }

bool areCompatible(int cur_cmd_id, int new_cmd_id);
/*
bool areCompatible(int cur_cmd_id, int new_cmd_id){
  
    return false;
  }
*/

void update_target(int cmd_id);
/*
void update_target(int cmd_id){
    switch(cmd_id){
      case STEP_CMD_ID:
        Step_left=CMD_buf[1];
      break;
      case SIMPLE_STEP_ALL_CMD_ID:
        Step_left=CMD_buf[1];
      break;
    }
  }
*/