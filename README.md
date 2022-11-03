### LAB : Stepper Motor

**Date:** 2022.11.03

**Author/Partner:** Heo Dabin/Ga Seungho

**Github:** [Github code](https://github.com/DABINHEO/LAB_Stepper_Motor)

**Demo Video:** [Youtube Link](https://youtu.be/9AHZsEXFOPc)

##            



### Introduction

In this LAB, a header code is created to control the stepper motor, and based on this, NECTO-F401RE will use four pins to input the stepper motor through the motor drive to control it. The rotational speed of the motor will allow the motor to rotate to an accurate degree at an accurate time, and the direction will also be controlled clockwise and counterclockwise. Furthermore, a moore FSM of the Final state machine was used for control.



### Requirement

#### Hardware

* MCU
  
  * NUCLEO-F401RE
* Actuator
   * 3Stepper Motor 28BYJ-48
   * Motor Driver ULN2003

   
   

#### Software

* Keil uVision, CMSIS, EC_HAL library

##          



### Problem1: Stepper Motor



#### Hardware Connection

The 28-BYJ48 Stepper Motors are one of the most commonly used stepper motors. The motor has a 4 coil unipolar arrangement and each coil is rated for +5V hence it is relatively easy to control with any basic microcontrollers. These motors has a stride angle of 5.625°/64, this means that the motor will have to make 64 steps to complete one rotation and for every step it will cover a 5.625° hence the level of control is also high. However, these motors run only on 5V and hence cannot provide high torque, for high torque application you should consider the other motors.

![image](https://user-images.githubusercontent.com/113574284/199484790-7dd1f99f-b0a0-4a4e-83c6-422ce2c59300.png)

**28BYJ-48 Stepper Motor Technical Specifications**

- Rated Voltage: 5V DC
- Number of Phases: 4
- Stride Angle: 5.625°/64
- Pull in torque: 300 gf.cm
- Insulated Power: 600VAC/1mA/1s
- Coil: Unipolar 5 lead coil



#### Stepper Motor Sequence

In order to control the stepper motor, each electrode must be input according to the following sequence.

**Full-stepping sequence**

![1](https://user-images.githubusercontent.com/113574284/199505066-75aa5834-8ed9-48ae-85fe-6d17c8286398.png)

**Half-stepping sequence**

![2](https://user-images.githubusercontent.com/113574284/199506623-5a951fe5-d783-42a5-9187-9be6f391d12b.png)

#### Finite State Machine

**Full-stepping sequence**

![3](https://user-images.githubusercontent.com/113574284/199507036-df860bee-e363-4ffc-9b69-3d5210d8ef62.png)

**Half-stepping sequence**

![4](https://user-images.githubusercontent.com/113574284/199507055-06a2d0fe-15a7-4763-9888-5ea6db76173d.png)



##          



### Problem2: RC Servo motor



#### Procedure

In this problem, we will create a header code to control the stepper motor and use it to control the motor. The rotational speed of the motor allows the motor to rotate at a certain time and to a certain degree, and the direction is also controlled clockwise and counterclockwise. In addition, the Moore FSM of the final state machine was used for control.

#### Configuration

![image](https://user-images.githubusercontent.com/113574284/199508057-8b34d59c-3dcf-4feb-ae5d-d12c8dde3411.png)



#### Circuit Diagram

![image](https://user-images.githubusercontent.com/113574284/199510623-64bb3c86-bf7a-4d50-81e1-b3ccded98fa6.png)



#### Discussion

1. Find out the trapezoid-shape velocity profile for stepper motor. When is this profile necessary?

   When using a stepper motor, we want to avoid running it immediately at full speed especially if a mass is attached to the motor since it is quite likely to induce miss-stepping. A motion profile provides the physical motion information and graphically depicts how the motor should behave during the movement (often in terms of position, velocity, and acceleration) and is used by the controller to determine what commands (voltages) to send to the motor. 

   

2. How would you change the code more efficiently for micro-stepping control? You don’t have to code this but need to explain your strategy.

   Instead of entering the number of steps directly, it would be more convenient to have the number of revolutions input and to code it so that it can be automatically adjusted in half mode and full mode.



#### Description with Code

* ecStepper.c  description

```c++
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

// Stepper Motor function, default setting
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
 {{1,1,0,0},{S1,S3}},   //S0
 {{0,1,1,0},{S2,S0}},	//S1
 {{0,0,1,1},{S3,S1}},	//S2
 {{1,0,0,1},{S0,S2}}	//S3
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
        GPIO_write(GPIOB, 10, FSM_full[state].out[0]);
        GPIO_write(GPIOB, 4, FSM_full[state].out[1]);
        GPIO_write(GPIOB, 5, FSM_full[state].out[2]);
        GPIO_write(GPIOB, 3, FSM_full[state].out[3]);
    }	 
    else if (mode == HALF){    // HALF mode
        GPIO_write(GPIOB, 10, FSM_half[state].out[0]);
        GPIO_write(GPIOB, 4, FSM_half[state].out[1]);
        GPIO_write(GPIOB, 5, FSM_half[state].out[2]);
        GPIO_write(GPIOB, 3, FSM_half[state].out[3]);
    }
}

void Stepper_setSpeed (long whatSpeed){        // whatSpeed = rpm
    step_delay = 60000 / 2048 / whatSpeed;     // Convert rpm to milli sec
}

void Stepper_step(int steps, int direction, int mode){
    uint32_t state = 0;
    myStepper._step_num = steps;

    for(; myStepper._step_num > 0; myStepper._step_num--){ 	// run for step size
        if (mode == FULL){
            state = FSM_full[state].next[direction];// state = next state
            delay_ms(step_delay);
        }
        else if (mode == HALF){
            state = FSM_half[state].next[direction];// state = next state
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
```

LAB_Stepper_Motor.c  description

```c++
#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecTIM.h"
#include "ecEXTI.h"
#include "ecSysTick.h"
#include "ecStepper.h"

void setup(void);
	
int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	int rev = 10; 						// in FULL mode, *2 int HALF mode
	Stepper_step(2048 * rev, 0, FULL);  // (Step : 2048, Direction : 0 or 1, Mode : FULL or HALF)
	// Inifinite Loop ----------------------------------------------------------
	while(1){;}
}

// Initialiization 
void setup(void)
{	
	RCC_PLL_init();                                 // System Clock = 84MHz
	SysTick_init();                                 // Systick init

	Stepper_init(GPIOB,10,GPIOB,4,GPIOB,5,GPIOB,3); // Stepper GPIO pin initialization
	Stepper_setSpeed(10);							// rpm
}
```





#### Results

![KakaoTalk_20221103_112116911](https://user-images.githubusercontent.com/113574284/199636485-f1fa6ce7-c902-4998-9486-51b68c0255a9.jpg)

[Youtube Link](https://youtu.be/9AHZsEXFOPc)



##          



### Reference

[LAB: Stepper Motor](https://ykkim.gitbook.io/ec/course/lab/lab-stepper-motor)

[28BYJ-48 - 5V Stepper Motor](https://components101.com/motors/28byj-48-stepper-motor)

[Trapezoidal velocity profile for a stepper motor](https://hofmannu.org/2022/01/06/trap-vel-stepper-motor/)

[What is a motion profile?](https://www.motioncontroltips.com/what-is-a-motion-profile/)

Class materials in Embedded Controller by Prof. Kim

RM0383 Reference manual

##          



### Troubleshooting

We tried to proceed by approaching it using binary and bitwise in FSM's state structure for motor control, but it did not work properly due to unknown cause. Therefore, we decided to give up the binary and approach it by making four arrays, and it worked well as intended.

There was a problem giving output through GPIO_write. It seems that there was also a problem accessing the structure, but it was solved by directly entering the used pin.
