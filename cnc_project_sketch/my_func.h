void gloabal_initial();
void trigger();

void simple_one_step();
void simple_step(int num,(direction,step_duration));
void g_one_step((direction,min_step_duration));
//move one step, blockingly;

void g_step(int num,(direction,step_duration));//move $(num) step blockingly，with each step duration (in ms) 

void g_step_setup();//set up functions for interrupts stuffs; right now only a place holder

void calib_step_size();
void calib_zero_pos();
void calib_zero_pos_return();

void decode_msg();//decode a received message, entered if msg end char is received






