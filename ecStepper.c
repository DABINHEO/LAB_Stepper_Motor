#include "stm32f4xx.h"
#include "ecStepper.h"
#include "ecGPIO.h"
#include "ecSysTick.h"

//State number 
#define S0 0
#define S1 1
#define S2 2
#define S3 3
#define S4 4
#define S5 5
#define S6 6
#define S7 7


// Stepper Motor function
uint32_t direction = 1; 
uint32_t step_delay = 100; 
uint32_t step_per_rev = 64*32;
	 

// Stepper Motor variable
volatile Stepper_t myStepper; 


//FULL stepping sequence  - FSM
typedef struct {
	uint8_t out[4];
  uint32_t next[2];
} State_full_t;

State_full_t FSM_full[4] = {  
 {{1,1,0,0},{S1,S3}},  //S0
 {{0,1,1,0},{S2,S0}},	//S1
 {{0,0,1,1},{S3,S1}},	//S2
 {{1,0,0,1},{S0,S2}}		//S3
};

//HALF stepping sequence
typedef struct {
	uint8_t out[4];
  uint32_t next[2];
} State_half_t;

State_half_t FSM_half[8] = { 
 {{1,0,0,0},{S1,S7}},	//S0
 {{1,1,0,0},{S2,S0}},	//S1
 {{0,1,0,0},{S3,S1}},	//S2
 {{0,1,1,0},{S4,S2}},	//S3
 {{0,0,1,0},{S5,S3}},	//S4
 {{0,0,1,1},{S6,S4}},	//S5
 {{0,0,0,1},{S7,S5}},	//S6
 {{1,0,0,1},{S0,S6}}	//S7
 };



void Stepper_init(GPIO_TypeDef* port1, int pin1, GPIO_TypeDef* port2, int pin2, GPIO_TypeDef* port3, int pin3, GPIO_TypeDef* port4, int pin4){
	 
//  GPIO Digital Out Initiation
	 myStepper.port1 = port1;
   myStepper.pin1  = pin1;
	 myStepper.port1 = port2;
   myStepper.pin1  = pin2;
	 myStepper.port1 = port3;
   myStepper.pin1  = pin3;
	 myStepper.port1 = port4;
   myStepper.pin1  = pin4;
	 	
//  GPIO Digital Out Initiation
	 GPIO_init(port1, pin1, OUTPUT);
	 GPIO_init(port2, pin2, OUTPUT);
	 GPIO_init(port3, pin3, OUTPUT);
	 GPIO_init(port4, pin4, OUTPUT);
		// No pull-up Pull-down , Push-Pull, Fast	
	 GPIO_pupd(port1, pin1, EC_NUD);
	 GPIO_pupd(port2, pin2, EC_NUD);
	 GPIO_pupd(port3, pin3, EC_NUD);
	 GPIO_pupd(port4, pin4, EC_NUD);
	 GPIO_otype(port1, pin1, Push_Pull);
	 GPIO_otype(port2, pin2, Push_Pull);
	 GPIO_otype(port3, pin3, Push_Pull);
	 GPIO_otype(port4, pin4, Push_Pull);
	 GPIO_ospeed(port1, pin1, Fast);
	 GPIO_ospeed(port2, pin2, Fast);
	 GPIO_ospeed(port3, pin3, Fast);
	 GPIO_ospeed(port4, pin4, Fast);
}

void Stepper_pinOut (uint32_t state, int mode){
	
	   if (mode == FULL){         // FULL mode
/*		  GPIO_write(myStepper.port1, myStepper.pin1, FSM_full[state].out[0]);
			GPIO_write(myStepper.port2, myStepper.pin2, FSM_full[state].out[1]);
			GPIO_write(myStepper.port3, myStepper.pin3, FSM_full[state].out[2]);
			GPIO_write(myStepper.port4, myStepper.pin4, FSM_full[state].out[3]);
*/
			GPIO_write(GPIOB, 10, FSM_full[state].out[0]);
			GPIO_write(GPIOB, 4, FSM_full[state].out[1]);
			GPIO_write(GPIOB, 5, FSM_full[state].out[2]);
			GPIO_write(GPIOB, 3, FSM_full[state].out[3]);
			}	 
		 else if (mode == HALF){    // HALF mode
			/*GPIO_write(myStepper.port1, myStepper.pin1, FSM_half[state].out[0]);
			GPIO_write(myStepper.port2, myStepper.pin2, FSM_half[state].out[1]);
			GPIO_write(myStepper.port3, myStepper.pin3, FSM_half[state].out[2]);
			GPIO_write(myStepper.port4, myStepper.pin4, FSM_half[state].out[3]);
			 */
			 GPIO_write(GPIOB, 10, FSM_half[state].out[0]);
			GPIO_write(GPIOB, 4, FSM_half[state].out[1]);
			GPIO_write(GPIOB, 5, FSM_half[state].out[2]);
			GPIO_write(GPIOB, 3, FSM_half[state].out[3]);
			}
}


void Stepper_setSpeed (long whatSpeed){      // rpm
		step_delay = 60000 / 2048 / whatSpeed; //YOUR CODE   // Convert rpm to milli sec
}


void Stepper_step(int steps, int direction, int mode){
	 uint32_t state = 0;
	 myStepper._step_num = steps;

	for(; myStepper._step_num > 0; myStepper._step_num--){ // run for step size
				// YOUR CODE                                  		// delay (step_delay); 
		if (mode == FULL){
				state = FSM_full[state].next[direction];// YOUR CODE       // state = next state
				delay_ms(step_delay);
		}
		else if (mode == HALF){
				state = FSM_half[state].next[direction];// YOUR CODE       // state = next state
				delay_ms(step_delay / 2);
		}
		Stepper_pinOut(state, mode);
   }
}


void Stepper_stop (void){ 
     
    	myStepper._step_num = 0;    
			// All pins(Port1~4, Pin1~4) set as DigitalOut '0'
			 	GPIO_write(myStepper.port1, myStepper.pin1, 0);
				GPIO_write(myStepper.port2, myStepper.pin2, 0);
				GPIO_write(myStepper.port3, myStepper.pin3, 0);
				GPIO_write(myStepper.port4, myStepper.pin4, 0);
}

