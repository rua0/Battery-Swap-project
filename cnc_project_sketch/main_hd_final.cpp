/****************************************************
/ Functions for the coordinator
/ Author: Handi, Matthew
****************************************************/
#include "mbed.h"
#include "stm32f401xe.h"
#include <math.h>
// Debug
//#define DEBUG
#define CHILD
// setups
int speedHigh = 4000; // 5ms
int speedLimit = 2000;
int speedStep = 50000; // 50ms
int speedSet[1000]; // 5ms
int idSpeed = 0;
//static double disPerStep = 0.182502; //0.182502 mm per step 
#ifndef CHILD
 static double disFromEndToEnd = 171.443333;
#else
 static double disFromEndToEnd = 168.84;
#endif
static double cntPerMM = 109.6309887;
static double cntPerStep = 19.997;
int encoderInit = 30000;
static int countIntn = 5;
// Objects
static DigitalOut myled(LED2);
static DigitalOut dirOut(PB_3); // dir
static DigitalOut comOut(PB_0); // communicate between boards
static DigitalOut pwmOut(PA_10);
static DigitalIn in0(PC_0); 
static DigitalIn in1(PC_1); 
Serial pc(SERIAL_TX, SERIAL_RX);
// Golbal variables
#ifndef CHILD
#else
 int TIM9_PSC = 100;
 int TIM9_ARR = 42000;
#endif
static double cntx[10]; // target distance
static double cnty[10]; // target distance
static int fullSpeed[10]; // which one is in full speed
static int idx=0;
static int idxn=0;
static int lastCnt = encoderInit;
static int lastCntx = encoderInit;
static int lastCnty = encoderInit;
//static double disCurr = 0.0;   // current position
static int countInt = 0;
static bool state = true;  // state of the coordinator: 1: waiting; 2: monitoring
static bool firstInterrupt = true; // if first interrupt: start timer3 (the PWMs of two board are synchonized in this way)
static bool finishMon = false;
static bool hitEnd = false;
static bool hitTheOtherEnd = false;
static bool calibFlag = true;
#ifndef CHILD
 static bool childIsReady = false;
#endif
// Functions
void system_init();
void calib();
void move_init();
void state_waiting_for_command_from_serial();
void state_monitoring_the_motors();
void read_serial();
void go(int dir, int speed);
void goStep(int dir, int step, double speed);
void timer3_init();
void timer3_enable();
void timer3_disable();
void pwm_setPeroid(int32_t pwm_timerPeriods_per_period);
#ifndef CHILD
void timer9_init();
void timer9_enable();
void timer9_disable();
void timer9_period(int period);
#endif
void initializeEncoderCounter(); // timer2
void exti_init();
void exti0_init();
void exti1_init();
/****************************
/ Main function for the coordinator
/  init and go to state 1 (wait for command)
/ Author: Handi, Matthew
*****************************/
int main()
{
 pc.printf("Hello World.\n"); 
 #ifndef CHILD
  pc.printf("Father.\n");
 #else
  pc.printf("Child.\n");
 #endif
 while (1){
  char ch = pc.getc();
  if (ch == '#')
    break;
 }
 
 system_init();
 // test code
// int step = 800;
// double period = 2000;
//  goStep(1, step, period);
// double cnt = TIM2 -> CNT; // Reset the count to 0
// pc.printf("Step: %d, time: %f, Encoder Count: %f\n", step, period/1000000.0*step, cnt);
 //pc.getc();
 //
 while(1) {
  if (state) state_waiting_for_command_from_serial();
  else state_monitoring_the_motors();
 }
}
/***********************************************************
/ Init the whole system
/ 1. to the init position
/ 2. calibration
/ Author: Handi
****************************/
void system_init(){
 comOut = 0;
 // init the communication from switcher
 exti0_init();
 exti1_init();
 // init timers  
 timer3_init();
 #ifndef CHILD
  timer9_init();
 #endif
 
 // calibration
 calib();
 // init the communication between the boards
 exti_init();
}
/**************************
/ Init the whole system
/ 1. to the init position
/ 2. calibration
/ Author: Handi
****************************/
void calib(){
 calibFlag = true;
 NVIC_DisableIRQ(EXTI0_IRQn);
 
 hitTheOtherEnd = false;
 if (in1 == 0) {
  go(1, speedHigh); // 5ms
  // wait for the interrupt exti0 (hit the original point)
  while (1){
   if (hitTheOtherEnd) break;
  }
  pc.printf("To the other end. Back.\n");
 }
 goStep(0,40,speedHigh);
 hitTheOtherEnd = false;
 // wait for the interrupt exti0 (hit the original point)
 while (1) {
  if (hitTheOtherEnd) break;
  goStep(1, 1, speedStep);
 }
 NVIC_DisableIRQ(EXTI1_IRQn);
 NVIC_EnableIRQ(EXTI0_IRQn);
 initializeEncoderCounter();
 
 hitEnd = false;
 // go(dir, speed/10000)
 go(0, speedHigh); // 5ms
 // wait for the interrupt exti0 (hit the original point)
 while (1){
  if (hitEnd) break;
 }
 pc.printf("Go back.\n");
 
 // goStep(dir, steps, speed/ms)
 goStep(1,40,speedHigh);
 hitEnd = false;
 // wait for the interrupt exti0 (hit the original point)
 while (1) {
  if (hitEnd) break;
  goStep(0, 1, speedStep);
 }
 double cnt = encoderInit - double(TIM2 -> CNT); // cnt from one end to the other
 pc.printf("Manual calibration, cnt per MM is %f, auto calibration ... is %f. \n", cntPerMM , cnt/disFromEndToEnd);
 //cntPerMM = (cntPerMM + cnt/disFromEndToEnd)/2;
 cntPerMM = cnt/disFromEndToEnd;
 
 initializeEncoderCounter();
 //NVIC_DisableIRQ(EXTI0_IRQn);
 NVIC_EnableIRQ(EXTI1_IRQn);
 calibFlag = false;
}
/****************************
/ interrupt from child when child is ready
/ Author: Handi
*****************************/
extern "C" void EXTI0_IRQHandler(void) {
 if(EXTI->PR & EXTI_PR_PR0){  
  if (in0 == 1) {
  timer3_disable();
  hitEnd = true;
  }
 
 //#ifdef DEBUG
  pc.printf("Get to original point.\n");
  pc.printf("Signal from pin C0: %d\n",int(in0));
 //#endif 
 
  EXTI->PR |= (EXTI_PR_PR0);
 }
}
/****************************
/ interrupt from child when child is ready
/ Author: Handi
*****************************/
extern "C" void EXTI1_IRQHandler(void) {
 if(EXTI->PR & EXTI_PR_PR1){  
  if (in1 == 1) {
  timer3_disable();
  hitTheOtherEnd = true;
  }
 
 #ifdef DEBUG
  pc.printf("Get to the End point.\n");
  pc.printf("Signal from pin C0: %d\n",int(in1));
 #endif
 
  EXTI->PR |= (EXTI_PR_PR1);
 }
}
/***********************************************************
/ State 1 of the coordinator: waiting for signal from
/ 1. Read serial message from touch screen
/ 2. Init the movement info
/ 3. Go to state 2 (monitoring)
/ Author: Handi
*****************************/
void state_waiting_for_command_from_serial(){
  #ifdef DEBUG
  pc.printf("State 1.\n");
 #endif
  myled = 1;
 
 // will stuck in this function,
 // if no correct signal from serial
 if (idx<idxn) idx++;
 else { idx = 0; read_serial();}
 
 state = false;
}
/************************************************
/ Read signal from serial
/ Author: Handi
************************************************/
void read_serial(){  
 int id = 0;
 int disx[10]; // target distance
 int disy[10]; // target distance
 while (1) {
   
  char c = pc.getc();
  //if ( c=='*' ){
   // calibration
   //calib();
  //}
  
  // Command from touch screen starts with a '$'
  if ( c == '$') {
   // get distance
   disx[id] = 0;
   for(int i = 0; i < 3; i++){
   c = pc.getc();
   disx[id] = disx[id] *10 + (c - '0');
   }
   // get distance
   disy[id] = 0;
   for(int i = 0; i < 3; i++){
   c = pc.getc();
   disy[id] = disy[id] *10 + (c - '0');
   }
  
  
   // calculations
   cntx[id] = disx[id]*cntPerMM + encoderInit;
   cnty[id] = disy[id]*cntPerMM + encoderInit;
   if (id == 0) {
    if (abs(cntx[id] - lastCntx)>abs(cnty[id] - lastCnty)){
      fullSpeed[id] = 0;
    }else{
      fullSpeed[id] = 1;
    }
   }else{
     if (abs(cntx[id]-cntx[id-1])>abs(cnty[id]-cnty[id-1])){
      fullSpeed[id] = 0;
    }else{
      fullSpeed[id] = 1;
    }
   }
  //#ifdef DEBUG
   pc.printf("Move %d. disx: %d, disy: %d, fullSpeed: %d.\n",id+1, disx[id], disy[id], fullSpeed[id]);
  //#endif
	
   id +=1;
   continue;
  }
  
   if ( id >= 10 ) {
   idxn = id-1;
   //#ifdef DEBUG
    pc.printf("Up to 10 points. %d Points in total.\n", idxn);
   //#endif
   break;
  }
  
  if ( c == '%' ) {
   idxn = id-1;
   //#ifdef DEBUG
    pc.printf("%d Points in total.\n", idxn+1);
   //#endif
   break;
  }
 }
}
/***********************************************************
/ State 2 of the coordinator: monitoring the motors
/ 1. Monitor the motors
/ 2. Send PWM signal to motors
/ 3. Go to state 1 (waiting)
/ Author: Handi
*****************************/
void state_monitoring_the_motors(){
 //#ifdef DEBUG
  pc.printf("State 2.\n");
 //#endif
 myled = 0;
 
 // init the movement.
 // will stuck in this function,
 // if no signal from child
 move_init();
 
 while(1){
  if (finishMon) {
   state = true;
   break;
  }
 }
 lastCntx = cntx[idx];
 lastCnty = cnty[idx];
 #ifndef CHILD 
  lastCnt = cntx[idx];
  pc.printf("x: cnt: %f, expected dis: %f, dis (calculated from cnt): %f.\n", double(TIM2->CNT)-encoderInit, (cntx[idx] - encoderInit)/cntPerMM, (double(TIM2->CNT)-encoderInit)/cntPerMM);
 #else
  lastCnt = cnty[idx];
  pc.printf("y: cnt: %f, expected dis: %f, dis (calculated from cnt): %f.\n", double(TIM2->CNT)-encoderInit, (cnty[idx] - encoderInit)/cntPerMM, (double(TIM2->CNT)-encoderInit)/cntPerMM);
 #endif
 
 pc.printf("speed:");
 for (int i=0;i<countIntn; i++){
  pc.printf("%d ", speedSet[i]);
 }
 idSpeed = 0;
 wait(2);
 
}
/****************************
/ Init the moving action
/ 1. set timer (monitor)
/ 2. set speed
/ Author: Handi
*****************************/
void move_init(){
 
 finishMon = false;
 firstInterrupt = true;
 
 #ifndef CHILD 
  // output direction to motor
  if (cntx[idx] > double(TIM2 -> CNT)) dirOut = 1;
  else dirOut = 0;
  if (fullSpeed[idx]==0) speedSet[idSpeed] = speedHigh;
  else speedSet[idSpeed] = abs(speedHigh*((cnty[idx]- lastCnty)/(cntx[idx]- lastCntx)));
  if (speedSet[idSpeed]<speedLimit) speedSet[idSpeed] = speedLimit;
 
  pc.printf("x,dir:%d, cntn:%f, cntC:%f\n", int(dirOut), cntx[idx] - encoderInit, double(TIM2->CNT)-encoderInit);
  pc.printf("peroid:%f\n", speedSet[idSpeed]/1000.0);
  //double stepPerInt = (TIM9->PSC*TIM9->ARR)/(TIM3->PSC*TIM3->ARR);
  //double cntPerInt = stepPerInt * cntPerStep;
  //countIntn = ceil((cntx[idx] - TIM2 -> CNT)/cntPerInt);
 
  // wait until child is ready
  while (!childIsReady){
  } 
  childIsReady = false;
  // start timer9 for Interrupt
  countInt = 0;
  double peroidAll;
  if (fullSpeed[idx]==0) peroidAll = speedHigh * (cntx[idx]- lastCntx)/cntPerStep;
  else peroidAll = speedHigh * (cnty[idx]- lastCnty)/cntPerStep;
  
  int peroidPerInt = ceil( abs(peroidAll) / countIntn);
  pc.printf("peroidPerInt:%f\n", peroidPerInt/1000.0);
  timer9_period(peroidPerInt);
  timer9_enable();
 #else
    
  // output direction to motor
  if (cnty[idx] > double(TIM2 -> CNT)) dirOut = 1;
  else dirOut = 0;
  if (fullSpeed[idx]==1) speedSet[idSpeed] = speedHigh;
  else speedSet[idSpeed] = abs(speedHigh*((cntx[idx]- lastCntx)/(cnty[idx]- lastCnty)));
  if (speedSet[idSpeed]<speedLimit) speedSet[idSpeed] = speedLimit;
  
  pc.printf("y,dir:%d, cntn:%f, cntC:%f\n", int(dirOut), cnty[idx] - encoderInit, double(TIM2->CNT) - encoderInit);
  pc.printf("period:%f\n", speedSet[idSpeed]/1000.0);
  countInt = 0;
  // send signal to father 
  if (comOut == 1) comOut = 0;
  else comOut = 1;
 #endif
}
#ifndef CHILD
/****************************
/ interrupt of the monitor
/ 1. Monitor the motors position
/ 2. Change motor speed by PWM Peroid
/ 3. Deside when to stop
/ Author: Handi
*****************************/
extern "C" void TIM1_BRK_TIM9_IRQHandler(void) {
 
 if(TIM9->SR & 0xfffe){   // if UIF flag is set
   // output interrupt to child
  if (comOut == 1) comOut = 0;
  else comOut = 1;
 
  double cntCurr = double(TIM2->CNT);
 
  //double coef = );
  if (firstInterrupt){
   firstInterrupt = false; 
   pwm_setPeroid(speedSet[idSpeed]);
   timer3_enable();
   #ifdef DEBUG
    pc.printf("First!!\n");
   #endif
  }else{
   countInt ++;
   if (countInt<countIntn){
    double cntEachIntExpected = (cntx[idx] - lastCntx)/countIntn;
    double cntExpected = countInt*(cntx[idx]-lastCntx)/countIntn + lastCntx;
    double cntEachIntCurr = cntEachIntExpected + (cntCurr - cntExpected);
    //pc.printf("cntEachIntExpected: %f, cntCurr: %f, cntExpected: %f, cntEachIntCurr: %f.\n",cntEachIntExpected, cntCurr, cntExpected, cntEachIntCurr);
    //pc.printf("%d\n",speedSet[idSpeed]);
    speedSet[idSpeed+1] = speedSet[idSpeed] * cntEachIntCurr / cntEachIntExpected;
    idSpeed++;
    if (speedSet[idSpeed]<speedLimit) speedSet[idSpeed] = speedLimit;
    //pc.printf("%d\n",speedSet[idSpeed]);
    pwm_setPeroid(speedSet[idSpeed]);
    
    #ifdef DEBUG
     myled = ~myled;
     pc.printf("comOut:%d, dir:%d, dis:%f, disCurr:%f\n ", int(comOut), int(dirOut), (cntx[idx]-encoderInit)/cntPerMM,(cntCurr-encoderInit)/cntPerMM);
     pc.printf("cntExpected:%f, cntCurr:%f, \n", cntExpected, cntCurr);
     pc.printf("new peroid:%d, \n",speedSet[idSpeed]);
    #endif
   }
  }
  
  TIM9->SR = TIM9->SR & 0xfffe;   // clear UIF flag
 }
}
/****************************
/ interrupt from child when child is ready
/ Author: Handi
*****************************/
extern "C" void EXTI4_IRQHandler(void) {
 if(EXTI->PR & EXTI_PR_PR4){  
 #ifdef DEBUG
  pc.printf("Get from child: ready.\n");
 #endif 
  childIsReady = true;
  EXTI->PR |= (EXTI_PR_PR4);
 }
}
#else
/****************************
/ interrupt of the monitor from father
/ 1. Monitor the motors position
/ 2. Change motor speed by PWM Peroid
/ 3. Deside when to stop
/ Author: Handi
*****************************/
extern "C" void EXTI4_IRQHandler(void) {
 if(EXTI->PR & EXTI_PR_PR4){  
      
   double cntCurr = double(TIM2->CNT);
 
  //double coef = (TIM9_PSC*TIM9_ARR)/(TIM3->PSC*TIM3->ARR);
  if (firstInterrupt){
   firstInterrupt = false;
   pwm_setPeroid(speedSet[idSpeed]);
   timer3_enable();   
   #ifdef DEBUG
    pc.printf("First!!\n");
   #endif
  }else{
   countInt ++;
   if (countInt<countIntn){
    
    double cntEachIntExpected = (cnty[idx] - lastCnty)/countIntn;
    double cntExpected = countInt*(cnty[idx]-lastCnty)/countIntn + lastCnty;
    double cntEachIntCurr = cntEachIntExpected + (cntCurr - cntExpected);
    //pc.printf("cntEachIntExpected: %f, cntCurr: %f, cntExpected: %f, cntEachIntCurr: %f.\n",cntEachIntExpected, cntCurr, cntExpected, cntEachIntCurr);
    //pc.printf("%d\n",speedSet[idSpeed]);
    speedSet[idSpeed+1] = speedSet[idSpeed] * cntEachIntCurr / cntEachIntExpected;
    idSpeed++;
    if (speedSet[idSpeed]<speedLimit) speedSet[idSpeed] = speedLimit;
    //pc.printf("%d\n",speedSet[idSpeed]);
    pwm_setPeroid(speedSet[idSpeed]);
    
    #ifdef DEBUG
     myled = ~myled;
     pc.printf("comOut:%d, dir:%d, dis:%f, disCurr:%f\n ", int(comOut), int(dirOut), (cnty[idx]-encoderInit)/cntPerMM,(cntCurr-encoderInit)/cntPerMM);
     pc.printf("cntExpected:%f, cntCurr:%f, \n", cntExpected, cntCurr);
     pc.printf("new peroid:%d, \n",speedSet[idSpeed]);
    #endif
   }
  }
  
  EXTI->PR |= (EXTI_PR_PR4);
 }
}
#endif
/**************************************************
/ Init timer3 for PWM signal
/ Author: Matthew
**************************************************/
void timer3_init(){
  RCC -> APB1ENR |= RCC_APB1ENR_TIM3EN; // enable clock for timer
  RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // enable clock for GPIOA pins
 
 TIM3->CR2 &= 0; // UG used as trigg output
 TIM3->PSC = 83;  // 83*10000 = 10ms
 //TIM3->ARR = 42000; //
 //TIM3->ARR = 72000; // 1kHz
 TIM3->EGR = TIM_EGR_UG; // reinit the counter
 TIM3->DIER = TIM_DIER_UIE; // enable update interrupts
 // enable timer
 //TIM3->CR1 |= TIM_CR1_CEN;
 NVIC->ISER[0] |= TIM3_IRQn;  // enable TIM4 interrupt in NVIC
 NVIC_EnableIRQ(TIM3_IRQn); // Enable interrupt from TIM3 (NVIC level)
  NVIC_SetPriority(TIM3_IRQn,1);
  return;
}
void timer3_enable(){
 TIM3->CR1 |= 0x0001;
}
void timer3_disable(){
 TIM3->CR1 &= 0x1110;
}
void pwm_setPeroid(int32_t pwm_timerPeriods_per_period){
 TIM3->ARR = pwm_timerPeriods_per_period/2;
  //TIM3->CCR1 = pwm_timerPeriods_per_period/2;
}
/****************************
/ interrupt of the monitor of each step
/ 1. Monitor the motors position
/ 2. Deside if it should wait for the other direction
/ 2. Deside when to stop
/ Author: Handi
*****************************/
extern "C" void TIM3_IRQHandler(void) {
 if(TIM3->SR & TIM_SR_UIF){  
  
  if (pwmOut==1) pwmOut=0;
  else pwmOut=1; 
 
  if (calibFlag==0) {
  // check if it is the final step
  #ifndef CHILD
    dirOut=cntx[idx]>TIM2->CNT;
   if (fabs(cntx[idx]-(TIM2->CNT)) < (cntPerStep/2.0)) {
    timer9_disable();
  #else
   dirOut=cnty[idx]>TIM2->CNT;
   if (fabs(cnty[idx]-(TIM2->CNT)) < (cntPerStep/2.0)) {
  #endif  
    timer3_disable();
    finishMon = true;
    TIM3->SR &= ~TIM_SR_UIF;
    #ifdef DEBUG
     pc.printf("Stop the timer.\n");
    #endif
    return;
   }
 }
 
 TIM3->SR &= ~TIM_SR_UIF;
 }
}
 
/************************************************
/ Init timer9 for monitor (interrupt mode)
/ Author: Handi
************************************************/
void timer9_init(){
  RCC -> APB2ENR |= RCC_APB2ENR_TIM9EN; // enable clock for timer
  RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // enable clock for GPIOA pins
 
 TIM9->PSC = 83*16;  // 50 ms per interrupt //100
  TIM9->CNT = 0x0000;
  //TIM9->ARR = 2000;
  //TIM9->CR1 |= 1;  // Enable timer
  TIM9->DIER |= 1;   // enable interrupt
  NVIC->ISER[0] |= TIM4_IRQn;  // enable TIM4 interrupt in NVIC
  NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
  NVIC_SetPriority(TIM1_BRK_TIM9_IRQn,1);
 
 //GPIOA ->OTYPER &= ~(GPIO_OTYPER_OT_2); //Makes the output type as push-pull by resetting the corresponding bit in output type register.
 //GPIOA ->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2_1;//Makes the output high speed.
 //GPIOA ->PUPDR &= ~(GPIO_PUPDR_PUPDR2);//No pull up or pull down.
 //GPIOA ->AFR[0] = GPIO_AF1_TIM2<<20;//Sets the AF1 bit in the AFRL(AFR[0]) register which links AF1 to pin A5
 
 //GPIOA -> MODER |= GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1;  // | GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1 | GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
  GPIOA -> AFR[0] |= 0x00003300;
  return;
}
void timer9_enable(){
 TIM9->CR1 |= 1;
}
void timer9_disable(){
 TIM9->CR1 &= 0;
}
void timer9_period(int period){
 // peroid max  = 2s/n = 400ms = 400000/16 = 50000
 //TIM9->PSC = 83*16;
  TIM9->ARR = period/16;
}
/************************************************
/ Init timer2 for encoder counter (interrupt mode)
/ Author: Handi
************************************************/
void initializeEncoderCounter(){
	
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	
	// Configure encoder outputs to digital inputs
	GPIOA -> MODER |= GPIO_MODER_MODER0_1;    	// Configure PA_0 as input
	GPIOA -> MODER |= GPIO_MODER_MODER1_1;    	// Configure PA_1 as input
	GPIOA -> PUPDR |= GPIO_PUPDR_PUPDR0_1;    	// PA_0 pull down networt
	GPIOA -> PUPDR |= GPIO_PUPDR_PUPDR1_1;     	// PA_1 pull down network
	GPIOA -> AFR[0]|= 0x00000011;
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	
	TIM2->CR1 = 0x0001;
	TIM2->SMCR = 0x0003;
	TIM2->CCMR1 = 0xF1F1;
	TIM2->CCMR2 = 0x0000;
	TIM2->CCER = 0x0011;
	TIM2->PSC = 0x0000;
	TIM2->ARR = 0xFFFFFFFF;
	TIM2->CNT = encoderInit;
	TIM2->DIER = 0x0001;
}
/************************************************
/ External interrupt (exti4 is A2)
/ Author: Handi
************************************************/
void exti_init(){
  RCC -> APB1ENR |= RCC_AHB1ENR_GPIOAEN; // enable clock for GPIOA pins
 
 RCC -> APB2ENR |= RCC_APB2ENR_SYSCFGEN;
 SYSCFG -> EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PA;
 EXTI->IMR |= EXTI_IMR_MR4;
 EXTI->RTSR |= EXTI_RTSR_TR4;
 EXTI->FTSR |= EXTI_FTSR_TR4;
 GPIOA->PUPDR |= GPIO_PUPDR_PUPDR4_1; // Enable internal pull down for PA5
 
 NVIC_EnableIRQ(EXTI4_IRQn);
 NVIC_SetPriority(EXTI4_IRQn,1);
  return;
}
/************************************************
/ External interrupt for original point
/ Author: Handi
************************************************/
void exti0_init(){
  RCC -> APB1ENR |= RCC_AHB1ENR_GPIOCEN; // enable clock for GPIOA pins
 
 RCC -> APB2ENR |= RCC_APB2ENR_SYSCFGEN;
 SYSCFG -> EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PC;
 EXTI->IMR |= EXTI_IMR_MR0;
 EXTI->RTSR |= EXTI_RTSR_TR0;
 EXTI->FTSR |= EXTI_FTSR_TR0;
 GPIOC->PUPDR |= GPIO_PUPDR_PUPDR0_1; // Enable internal pull down
 
 NVIC_EnableIRQ(EXTI0_IRQn);
 NVIC_SetPriority(EXTI0_IRQn,2);
  return;
}
/************************************************
/ External interrupt for original point
/ Author: Handi
************************************************/
void exti1_init(){
  RCC -> APB1ENR |= RCC_AHB1ENR_GPIOCEN; // enable clock for GPIOA pins
 
 RCC -> APB2ENR |= RCC_APB2ENR_SYSCFGEN;
 SYSCFG -> EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PC;
 EXTI->IMR |= EXTI_IMR_MR1;
 EXTI->RTSR |= EXTI_RTSR_TR1;
 EXTI->FTSR |= EXTI_FTSR_TR1;
 GPIOC->PUPDR |= GPIO_PUPDR_PUPDR1_1; // Enable internal pull down
 
 NVIC_EnableIRQ(EXTI1_IRQn);
 NVIC_SetPriority(EXTI1_IRQn,2);
  return;
}
/************************************************
/ Go in a given direction with a speed, stop with a flag
/ Author: Handi
************************************************/
void go(int dir, int speed){
 
 dirOut = dir;
 pwm_setPeroid(speed);
 timer3_enable();
 
}
/************************************************
/ Go given steps
/ Author: Handi
************************************************/
void goStep(int dir, int step, double speed){
 
 pwmOut = 0;
 dirOut = dir;
 speed /= 1000000;
 for (int i = 0; i<step; i++){
  pwmOut = 1;
  wait(speed/2);
  pwmOut = 0;
  wait(speed/2);
 }
}
