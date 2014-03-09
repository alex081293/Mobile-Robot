/*	Project6  - Part A and B - IIR Filter in Fixed Point
*	University of South Florida
*
*	Sean Fleming
*	Timothy Lizardi
*	
*
*/

#include <hidef.h>			 /* common defines and macros */
#include "derivative.h"		 /* derivative-specific definitions */
#include <cstdio.h>
#include "filter.h"
#define MAX_SPEED 20
#define MIN_SPEED 250
#define FRONT 0
#define LEFT 1
#define RIGHT 2

#define LCD_DATA PORTK 
#define LCD_CTRL PORTK 
#define RS 0x01 
#define EN 0x02 

int a1[3] = {0x4000, 0x63E7, 0x290B};
int b1[3] = {0x0524, 0x0A48, 0x0524}; 
int front_prox = 0;
int left_prox = 0;
int right_prox = 0;
int old_left_prox_derivative = 0;
int old_right_prox_derivative = 0;
static int read_count=0;
static int front_temp = 0;
static int left_temp =  0;
static int right_temp =  0;

static int left_step_count = 0;
static int right_step_count = 0;
	
static int left_count=200; 	//count for left motor
static int right_count=200;	//count for right motor

static int ramp_count = 0;
	
static int left_speed_final = MAX_SPEED;
static int right_speed_final = MAX_SPEED;
	
static int left_speed  = MIN_SPEED-1;
static int right_speed = MIN_SPEED-1;
	
static int leftPos = 0;
static int rightPos = 0;
	
static int rampCoef = 250;

static int u_turn_count = 0;
static int u_turn_flag = 0;
static int turn_flag = 0;//Used to stop sensor reading and correction while turning

static int right_wheel_backwards = 0;//Sets right wheel motion backwards
static int left_wheel_backwards = 0;//Sets left wheel motion backwards

int ideal_distance = 62; 
int max_wall_distance = 45;
float p_value = .3;
float d_value = .5;

//static char right_chars[8] ={0x80, 0xC0, 0x40, 0x60, 0x20, 0x30, 0x10, 0x90};
static char right_chars[8] ={0xC0, 0x40, 0x60, 0x60, 0x30, 0x10, 0x90, 0x80};
static char left_chars[8] = {0x08, 0x09, 0x01, 0x03, 0x02, 0x06, 0x04, 0x0C};


void forward(){// forward one block
	//PTH = 0x40;
	right_step_count = 1000;//These values are guessed
	left_step_count = 1000;
	while(right_step_count >=0 && left_step_count >=0){}
	

}

void right_pivot_turn(){
	PTH = 0x24;
	turn_flag = 1;
	right_step_count = 0;//These values are guessed
	left_step_count = 212;
	while(right_step_count >=0 && left_step_count >=0){}
	turn_flag = 0;
}

void left_pivot_turn(){
	PTH = 0x02;
	turn_flag = 1;
	right_step_count = 212;//These values are guessed
	left_step_count = 0;
	while(right_step_count >=0 && left_step_count >=0){}
	turn_flag = 0;
}

void right_stationary_turn(){
	PTH = 0x79;
	turn_flag = 1;
	right_wheel_backwards = 1;
	right_step_count = 106;//These values are guessed
	left_step_count = 106;
	while(right_step_count >=0 && left_step_count >=0){}
	right_wheel_backwards = 0;
	turn_flag = 0;

}

void left_stationary_turn(){
	PTH = 0x12;
	turn_flag = 1;
	left_wheel_backwards = 1;
	right_step_count = 106;//These values are guessed
	left_step_count = 106;
	while(right_step_count >=0 && left_step_count >=0){}
	turn_flag = 0;
	left_wheel_backwards = 0;

}

void u_turn(){
	PTH = 0x19;
	turn_flag = 1;
	left_wheel_backwards = 1;
	right_step_count = 53;
	left_step_count = 53;
	while(right_step_count >=0 && left_step_count >=0){}
	turn_flag = 0;
	left_wheel_backwards = 0;
}

void right_rolling_turn(){
	PTH = 0x30;
	turn_flag = 1;
	right_speed = MAX_SPEED+100;
	left_speed = MAX_SPEED;
	right_step_count = 100;//These values are guessed
	left_step_count = 200;
	while(right_step_count >=0 && left_step_count >=0){}
	turn_flag = 0;	
}

void left_rolling_turn(){
	PTH = 0x78;
	turn_flag = 1;
	left_speed = MAX_SPEED+100;
	right_speed = MAX_SPEED;
	left_step_count = 100;//These values are guessed
	right_step_count = 200;
	while(right_step_count >=0 && left_step_count >=0){}
	turn_flag = 0;	
}

void main(void) {
	SYNR = 2;
	REFDV = 0;

	while(!(CRGFLG&0x08)){}
	CLKSEL |= 0x80;
	
	MCCTL = 0xCF;//Interrupt Enabled, Force Load, Enabled, P=16
	MCCNT = 150;
  
	DDRK=0xFF;
	DDRB=0xFF;
	DDRH=0xFF;
	
	ATD0CTL2 = 0x80; //Turns on ATD converter
	ATD0CTL3 = 0x18; //Length=3 FIFO=0
	ATD0CTL4 = 0x80; //8-bit resolution
	ATD0CTL5 = 0x33; //Left Justified, Unsigned, SCAN=1, MULTI=1, START=3

	EnableInterrupts;

	//forward(); 
    for(;;) {
		//Below is what our finished code will do
		//I cannot make anything happen in any kind of control statement in the main.
		//Even outside this for loop.
		//I can call funtions outside the loop, but if the funtion has a control statement
		//it gets stuck there
		
		forward();
		/*right_stationary_turn();
		forward();
		right_pivot_turn();
		forward();
		right_rolling_turn();
		forward();
		u_turn();
		forward();
		left_stationary_turn();
		forward();
		left_pivot_turn();
		forward();
		left_rolling_turn();
		forward();*/
		u_turn();
	
		 
		
    } /* loop forever */
  /* please make sure that you never leave main */
}


#pragma TRAP_PROC
#pragma CODE_SEG __SHORT_SEG NON_BANKED
interrupt VectorNumber_Vtimmdcu void mdcuInterrupt () {
	MCFLG |= 0x80;//Always Clear the Flag
	left_count++;
	right_count++;
	ramp_count++; 
	read_count++;

	
	//The Wall follow is commented out for debugging reasons
	/*if((read_count%100)==0 && turn_flag!=1){	// Get new reading every 10 ms
		old_left_prox_derivative = ideal_distance - left_prox; 
		old_right_prox_derivative = ideal_distance - right_prox; 
	    front_prox = front_butterworth(ATD0DR2H, a1, b1);
		left_prox = left_butterworth(ATD0DR1H, a1, b1);
		right_prox = right_butterworth(ATD0DR0H, a1, b1);
		read_count = 0;
		if (left_prox >= max_wall_distance && right_prox >= max_wall_distance){
			ideal_distance = (left_prox + right_prox)/2;	
		}
	}

	
	
	if (left_prox > max_wall_distance  && turn_flag!=1){	
		//Left Sensor Control
		if (left_prox < (ideal_distance)){//If too far from left wall correct	    
		   right_speed_final  = MAX_SPEED;
		   left_speed_final  = MAX_SPEED +(p_value * (ideal_distance - left_prox)) + (d_value * old_left_prox_derivative); 
		   //old_left_prox_derivative = (  ideal_distance - left_prox);
		   }else{	 //If too close to left wall correct   
			   left_speed_final  = MAX_SPEED;
			   right_speed_final  = MAX_SPEED +(-p_value * ( ideal_distance - left_prox)) + (-d_value * old_left_prox_derivative); 
			   //old_left_prox_derivative = ( ideal_distance - left_prox);
		}
	}
	
	if (left_prox < max_wall_distance && right_prox > max_wall_distance && turn_flag!=1){
		//Right Sensor Control 
		if (right_prox < (ideal_distance)){//If too far from right wall correct	    
		   left_speed_final  = MAX_SPEED;
		   right_speed_final  = MAX_SPEED +(p_value * (ideal_distance - right_prox)) + (d_value * old_right_prox_derivative); 
		   //old_right_prox_derivative = (ideal_distance - right_prox);
		   }else{	 //If too close to right wall correct   
			   right_speed_final  = MAX_SPEED;
			   left_speed_final  = MAX_SPEED +(-p_value * (ideal_distance - right_prox)) + (-d_value * old_right_prox_derivative); 
			   //old_right_prox_derivative = (ideal_distance - right_prox);
		}
	}

	if (left_prox <= max_wall_distance && right_prox <= max_wall_distance && turn_flag!=1){
	right_speed_final = MAX_SPEED+50;
	left_speed_final = MAX_SPEED+50;
	
	}
	
	//Front Sensor Control
	if (front_prox>80 && u_turn_flag!=1){
	   right_speed_final = MAX_SPEED+100;
	   left_speed_final  = MAX_SPEED+100; 
	   u_turn_flag = 1;}*/
	   
	if (ramp_count == rampCoef){//Ramping
	     
	    if(right_speed < right_speed_final){
	        right_speed +=10;    //slow it down
	    }else if(right_speed>right_speed_final){
	        right_speed -=5;    //speed it up
	    }  
	    if(left_speed < left_speed_final){
	        left_speed +=10;    //slow it down
	    }else if(left_speed>left_speed_final){
	        left_speed -=5;    //speed it up
	    }	    
	    ramp_count = 0;
	}
	
	
	
	//Increment Left Wheel
	if(left_count>=left_speed){
		if(left_wheel_backwards == 0){//Forwards
			leftPos++;
			if(leftPos==8)leftPos = 0;
			left_count=0;
			left_step_count--;
		}else{//Backwards
			leftPos--;
			if(leftPos<=0)leftPos = 8;
			left_count=0;
			left_step_count--;
			//u_turn_count++;
		}
	}
	 
	//Increment Right Wheel
	if(right_count>=right_speed){
		if(right_wheel_backwards == 0){//Forwards 
			rightPos++;
			if(rightPos==8)rightPos = 0;
			right_count=0;
			right_step_count--;
		}else{//Backwards
			rightPos--;
			if(rightPos<=0)rightPos = 8;
			right_count=0;
			right_step_count--;
			//u_turn_count++;
		}
	}
	
	/*if(u_turn_count > 425){//MAX_SPEED == 20 this = 425; 15, 410
	u_turn_count = 0;
	u_turn_flag = 0;
	//right_speed_final = MIN_SPEED;
	//left_speed_final  = MIN_SPEED;
	}*/
	 
	PORTB = left_chars[leftPos] + right_chars[rightPos];
}










