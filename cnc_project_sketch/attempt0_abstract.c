//attempt0 abstraction


//Constant declaration:
#define ZERO_POS POS_UNKNOWN //need test
#define STEP_SIZE_INIT $(some empirical value)
#define FULL_LENGTH $(some measured value)

#define MSG_END_CHAR '$'

#define CMD_MAX_SIZE 10
#define Rx_buf_SIZE 30
//Global variable declaration:
static int step_size=STEP_SIZE_INIT;
static int pos_dir;
static int cur_cmd;


//need declaration of
unsigned char Rx_cb_indx, Rx_buf[Rx_buf_SIZE], Rx_data[CMD_MAX_SIZE+2], Transfer_cplt;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t i;//max 255, or 511?
	//my_toggleled();
	//do this to indicate that this routine is entered
	//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

	if (huart->Instance == USART2)             //is current&USB uart?
	{
		if (Rx_cb_indx == 0)                       //if data is not being received
		{
			for (i=0;i<Rx_buf_SIZE;i++)//clear rx buffer
			{
				Rx_buf[i]=0;											// clear Rx_buf before receiving new data
			}
		}
		if (Rx_data[0]!=MSG_END_CHAR)                     //if received data different from ascii 13 (enter)
		{
			Rx_buf[Rx_cb_indx++]= Rx_data[0];     // store data in buffer
			
		}
		else 																	// if received data = 13
		{
			//str_cpy here, so that the cmd is not executed in the call back
			decode_msg(Rx_cb_indx);
			Rx_cb_indx= 0;//re-init index to zero when an end is detected
			//return received byte if not the end character
			//HAL_UART_Transmit(&huart2, (uint8_t *)Rx_buf, 30,99999);//30 is buffer size
		}
		HAL_UART_Receive_IT (&huart2, Rx_data, 1);     // activate receive
	}
}

int main(void){
	
	my_init();
	//now after initialization, either the button or the message received will make the board do something
	//the button ir is disabled

	//!!!!send initialized msg
	trans_init_cplt_msg();
	//HAL_UART_Transmit(&huart2, (uint8_t *)Rx_buf, 30,99999);//30 is buffer size

	while(1){
		//when a MSG is received,
		if (CMD_TO_EXE)//remember to declare the variable and function underneath
		{
			execute_cmd();//execute cmd based on cmd_buffer
			//is blocking
			CMD_TO_EXE=0;
		}
	}
}

void my_init(){
	Hardware_init();
	//initialize and config HAL
		//TIMER CONFIG
			//quadrature decoder
		//GPIO & EXTI
			//output: dir, step
			//input: usr_but
			//input (interrupt): limit switch 0_pos&1_pos
		//UART
	
	Global_init();
	//initialize and config global variables

	
}

//called when a msg_end_char is detected
void decode_msg(int end_index){
	//determine if cmd is valid and change CMD_TO_EXE accordingly
	uint8_t i=0;
	#ifdef MY_DEBUG
	//debug mode: print the received cmd
		HAL_UART_Transmit(&huart2, "Decode Received: ", 17,9999);
		char buf_trans[];
		memcpy(buf_trans,Rx_buf,end_index);
		buf_trans[end_index]='\n';
		HAL_UART_Transmit(&huart2, (uint8_t *)buf_trans, 30,9999);//30 is buffer size
	#endif
	memcpy(cmd_buf,Rx_buf,end_index);
	if (cmd_buf[0]<200)
	{
		CMD_TO_EXE=cmd_buf[0];
	}else{
		char status_msg[]="status cmd received\n";
		HAL_UART_Transmit(&huart2, status_msg, strlen(status_msg)+1,9999);
	}
}

//execute a cmd based on the id
void execute_cmd(){
	#ifdef EXE_CMD_DEBUG
	//debug mode: print the received cmd
		char debug_msg[40];
		sprintf (debug_msg, "CMD executing %d\n",cmd_buf[0]);
		HAL_UART_Transmit(&huart2, (uint8_t *)debug_msg, strlen(debug_msg) ,9999);//30 is buffer size
	#endif
	switch(cmd_buf[0]){
		case 110:
			simple_one_step();
			break;
		case 111://step with no arguments
			simple_step(1,1?,dft_?);
			break;
		default: ret_invalid();
	}
}
