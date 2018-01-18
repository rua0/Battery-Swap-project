#define CMD_MAX_SIZE 30
unsigned char Rx_cb_indx, CMD_buf[CMD_MAX_SIZE], Rx_data[2], Transfer_cplt;

void clear_cmd_buf(){
	for (uint8_t i=0;i<CMD_MAX_SIZE;i++)//clear rx buffer
	{
		CMD_buf[i]=0;											// clear Rx_buf before receiving new data
	}
}
//Rx Complete Callback function
//which will be executed at the end of receiving a byte  
//and copy it to the CMD buffer
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t i;//max 255, or 511?
	//my_toggleled();
	//do this to indicate that this routine is entered
	//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

	if (huart->Instance == USART2)             //is current&USB uart?
	{
		//if data is not being received
		if (Rx_cb_indx == 0)                       
		{
			//****thinking abt moving this to after decode msg
			clear_cmd_buf();
			// for (i=0;i<CMD_MAX_SIZE;i++)//clear rx buffer
			// {
			// 	CMD_buf[i]=0;											// clear Rx_buf before receiving new data
			// }
		}
		//if received data is different from the end character
		if (Rx_data[0]!=MSG_END_CHAR)                     
		{
			CMD_buf[Rx_cb_indx++]= Rx_data[0];     // store data in buffer
			
		}
		else 																	// if received data = 13
		{
			//at the end, decode received msg
			decode_msg(Rx_cb_indx);
			Rx_cb_indx= 0;//re-init index to zero when an end is detected
			#ifdef RX_CB_END_DEBUG
			//debug mode: blockingly echo received bytes array 
			//when the end character detected
			HAL_UART_Transmit(&huart2, (uint8_t *)CMD_buf, CMD_buf_SIZE,99999);
				//buffer size could also be strlen(CMD_buf)
			#endif
			//maybe clear_cmd_buf() here
		}
		// activate receive in background
		HAL_UART_Receive_IT (&huart2, Rx_data, 1);     
	}
}


//high level commands
void my_printf(){

}

bool isShort(int cmd_id){
	return false;
}
void exe_short_cmd(){

}
bool areCompatible(int cur_cmd_id, int new_cmd_id){

	return false;
}

#define STEP_CMD_ID 100
void update_target(int cmd_id){
	switch(cmd_id){
		case STEP_CMD_ID:
			Step_left=CMD_buf[1];
		break;
	}
}


char Cmd_cflt_err_msg="1, CMD conflict";
//incompatible command error message
//conflict with current command

static int Cur_CMD=0;
//uint8_t CMD_TO_EXE=0;
//Probably not needed
void decode_msg(uint8_t end_char_index){
	int new_cmd_id=CMD_buf[0];
	//if short cmd such as ask status
	if(--isShort(new_cmd_id)){
		--exe_short_cmd();
		return;
	}
	//not short cmd

	//if there is an ongoing cmd
	if (Cur_CMD)
	{
		if (!--areCompatible(Cur_CMD,new_cmd_id))
		{
			//causing conflict
			//print error msg
			HAL_UART_Transmit(&huart2, (uint8_t *)Cmd_cflt_err_msg, strlen(Cmd_cflt_err_msg));
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
	--update_target(Cur_CMD);
}






























//----------------------------------------------
//------------------LED functions---------------

