/*
*    
*    Final Project
*    Alex Moran
*    Sean Fleming
*    
*/

#include <hidef.h>             /* common defines and macros */
#include "derivative.h"         /* derivative-specific definitions */
#include <cstdio.h>
#include <stdlib.h>

// Emums for ease of read
#define MAX_SPEED 22
#define MIN_SPEED 250
#define FRONT 0
#define LEFT 10
#define RIGHT 20
#define NORTH 4
#define EAST 1
#define SOUTH 2
#define WEST 3

#define STATIONARYDISTANCE 530

#define UTURN 100
#define PIVOT 101
#define STATIONARY 102
#define ROLLING 103

#define LCD_DATA PORTK 
#define LCD_CTRL PORTK 
#define RS 0x01 
#define EN 0x02 

int a1[3] = {0x4000, 0x63E7, 0x290B};
int b1[3] = {0x0524, 0x0A48, 0x0524}; 
int front_filter[8];
int right_filter[8];
int left_filter[8];
int front_prox = 0;
int left_prox = 0;
int right_prox = 0;
int previousErrorL = 0;
int currentErrorL = 0;
int errorDirL = 0;
int previousErrorR = 0;
int currentErrorR = 0;
int errorDirR = 0;
int ave = 0;
int i = 0;
static int directionToChangeTo = 0;
static int read_count=0;
static int front_temp = 0;
static int left_temp =  0;
static int right_temp =  0;
static char popValue = ' ';

static int left_step_count = 0;
static int right_step_count = 0;
    
static int left_count=200;     //count for left motor
static int right_count=200;    //count for right motor

static int ramp_count = 0;
static int wait_count = 0;
    
static int left_speed_final = MAX_SPEED;
static int right_speed_final = MAX_SPEED;
    
static int left_speed  = MIN_SPEED-100;
static int right_speed = MIN_SPEED-100;
    
static int leftPos = 0;
static int rightPos = 0;
    
static int rampCoef = 35;

static int tracebackFlag = 0;

static int u_turn_count = 0;
static int u_turn_flag = 0;
static int turn_flag = 1;//Used to stop sensor reading and correction while turning

static unsigned int right_wheel_backwards = 0;//Sets right wheel motion backwards
static unsigned int left_wheel_backwards = 0;//Sets left wheel motion backwards

static int wait_time = 100;

static int turn;

static short int maze[10][10];
static short int directionMaze[10][10];
static char stack[100];
static int sTop = 0;
static int uTurnFlag;
static int x, y;
static int distance = 0;
static int direction = NORTH;
static int lastWall = LEFT;
static int dumbFlag = 0;
int ideal_distance = 63; 
int max_wall_distance = 45;
// .22 || .55
// .3  || .6
float p_value = .22;
float d_value = 46;

static char right_chars[8] ={0xC0, 0x40, 0x60, 0x60, 0x30, 0x10, 0x90, 0x80};
static char left_chars[8] = {0x08, 0x09, 0x01, 0x03, 0x02, 0x06, 0x04, 0x0C};

void resetFilters() {
        int j;                      
        
        for(i = 0; i < 8; i++) {
              front_filter[i] = ATD0DR2H;
              left_filter[i] = ATD0DR1H;
              right_filter[i] = ATD0DR0H;
        }
        
        front_prox = (front_filter[0] + front_filter[1] + front_filter[2] + front_filter[3] + front_filter[4] + front_filter[5] + front_filter[6] + front_filter[7])/8;
        left_prox = (left_filter[0] + left_filter[1] + left_filter[2] + left_filter[3] + left_filter[4] + left_filter[5] + left_filter[6] + left_filter[7])/8;
        right_prox = (right_filter[0] + right_filter[1] + right_filter[2] + right_filter[3] + right_filter[4] + right_filter[5] + right_filter[6] + right_filter[7])/8;
}
        



// Software wait                    
void wait(int time){
    while((wait_count % time) != 0){}
    wait_count = 0;
}

void resetValues() {
    turn_flag = 0;
    leftPos = rightPos = 4;
    right_step_count = left_step_count = 0; 
    right_wheel_backwards = left_wheel_backwards = 0;
    //right_speed = left_speed = MAX_SPEED;
    right_speed_final = left_speed_final = MAX_SPEED;
    
    wait(20); 
}

void left_stationary_turn(){
    turn_flag = 1;
    leftPos = rightPos = 4; //Reset Wheel Position
    right_speed_final = 30;
    left_speed_final = 30; 
    left_wheel_backwards = 1;
    right_step_count = left_step_count = 200;    
    while(right_step_count >=0 || left_step_count >=0){}
    
    if (direction == NORTH) {
       direction = EAST;
    } else if ((direction == EAST)) {
      direction = SOUTH;
    } else if ((direction == SOUTH) ) {
       direction = WEST;
    } else if ((direction == WEST) ) {
      direction = NORTH;
    } 
       
    resetValues();
    resetFilters(); 
    //wait(wait_time);    
    
}

void right_stationary_turn(){  /*
    turn_flag = 1; 
    leftPos = rightPos = 4; //Reset Wheel Position
    right_speed_final = 45;
    left_speed_final = 45;
    right_wheel_backwards = 1;
    right_step_count = left_step_count = 200;    
    while(right_step_count >=0 || left_step_count >=0){}    
    resetValues(); 
    //wait(wait_time); 
    */
    left_stationary_turn();
    left_stationary_turn();
    left_stationary_turn();   
    
}



void u_turn(){
    turn_flag = 1;
    right_speed_final = left_speed_final = 32;// To Minimize spillage
    turn_flag = 1;
    leftPos = rightPos = 4; //Reset Wheel Position
    right_speed_final = MAX_SPEED+20;
    left_speed_final = MAX_SPEED+20;    
    left_wheel_backwards = 1;
    right_step_count = 415;
    left_step_count = 415;
    while(right_step_count >=0 || left_step_count >=0){}
    resetValues(); 
    //wait(wait_time);    
    
}

               
int orientate(){
        if(
            (
             (direction == NORTH) &&
               (
                ((front_prox < 40) && (maze[x][y+1] == -1)) ||
                ((left_prox < 40) && (maze[x-1][y] == -1)) ||
                ((right_prox < 40) && (maze[x+1][y] == -1))  
               )
            ) ||
            (
             (direction == SOUTH) &&
               (
                ((front_prox < 40) && (maze[x][y-1] == -1)) ||
                ((left_prox < 40) && (maze[x+1][y] == -1)) ||
                ((right_prox < 40) && (maze[x-1][y] == -1))
               )
            ) ||                
            (
             (direction == WEST) &&
               (
                ((front_prox < 40) && (maze[x-1][y] == -1)) ||
                ((left_prox < 40) && (maze[x][y-1] == -1)) ||
                ((right_prox < 40) && (maze[x][y+1] == -1))
               )
            ) ||                
            (
             (direction == EAST) &&
               (
                ((front_prox < 40) && (maze[x+1][y] == -1)) ||
                ((left_prox < 40) && (maze[x][y+1] == -1)) ||
                ((right_prox < 40) && (maze[x][y-1] == -1))
               )
            )                
          ) {
               if(direction == NORTH) {
                switch(directionMaze[x][y]) {
                        case 3:
                                left_stationary_turn();
                                break;
                        case 2: 
                                u_turn();
                                break;
                        case 1:
                                right_stationary_turn();
                                break;
                        default:
                                break;               
                }
               } else if(direction == EAST) {
                switch(directionMaze[x][y]) {
                        case 4:
                                left_stationary_turn();
                                break;
                        case 3: 
                                u_turn();
                                break;
                        case 2:
                                right_stationary_turn();
                                break;
                        default:
                                break;               
                }
               } else if(direction == SOUTH) {
                switch(directionMaze[x][y]) {
                        case 1:
                                left_stationary_turn();
                                break;
                        case 4: 
                                u_turn();
                                break;
                        case 3:
                                right_stationary_turn();
                                break;
                        default:
                                break;               
                }
               } else if(direction == WEST) {
                switch(directionMaze[x][y]) {
                        case 2:
                                left_stationary_turn();
                                break;
                        case 1: 
                                u_turn();
                                break;
                        case 4:
                                right_stationary_turn();
                                break;
                        default:
                                break;               
                }
               }
             return 1;
          } else {
                return 0;
          }
      
}

char pop(){
 return stack[--sTop];       
}

int push(char temp){
        if(sTop == 99)
        return 0;//Fail, to big for array
        
        stack[sTop++] = temp;
        return 1;
}

void MSDelay(unsigned int itime)
{
  unsigned int i; unsigned int j;
  for(i=0;i<itime;i++)
  for(j=0;j<1000;j++);
}
  void COMWRT4(unsigned char command)
  {
        unsigned char x;
        
        x = (command & 0xF0) >> 2;         //shift high nibble to center of byte for Pk5-Pk2
        LCD_DATA =LCD_DATA & ~0x3C;          //clear bits Pk5-Pk2
        LCD_DATA = LCD_DATA | x;          //sends high nibble to PORTK
        MSDelay(1);
        LCD_CTRL = LCD_CTRL & ~RS;         //set RS to command (RS=0)
        MSDelay(1);
        LCD_CTRL = LCD_CTRL | EN;          //rais enable
        MSDelay(5);
        LCD_CTRL = LCD_CTRL & ~EN;         //Drop enable to capture command
        MSDelay(15);                       //wait
        x = (command & 0x0F)<< 2;          // shift low nibble to center of byte for Pk5-Pk2
        LCD_DATA =LCD_DATA & ~0x3C;         //clear bits Pk5-Pk2
        LCD_DATA =LCD_DATA | x;             //send low nibble to PORTK
        LCD_CTRL = LCD_CTRL | EN;          //rais enable
        MSDelay(5);
        LCD_CTRL = LCD_CTRL & ~EN;         //drop enable to capture command
        MSDelay(15);
  }

  void DATWRT4(unsigned char data)
  {
    unsigned char x;

        x = (data & 0xF0) >> 2;
        LCD_DATA =LCD_DATA & ~0x3C;                     
        LCD_DATA = LCD_DATA | x;
        MSDelay(1);
        LCD_CTRL = LCD_CTRL | RS;
        MSDelay(1);
        LCD_CTRL = LCD_CTRL | EN;
        MSDelay(1);
        LCD_CTRL = LCD_CTRL & ~EN;
        MSDelay(5);
       
        x = (data & 0x0F)<< 2;
        LCD_DATA =LCD_DATA & ~0x3C;                     
        LCD_DATA = LCD_DATA | x;
        LCD_CTRL = LCD_CTRL | EN;
        MSDelay(1);
        LCD_CTRL = LCD_CTRL & ~EN;
        MSDelay(15);
  }

// Moves forward for a given distance, if the front prox reaches a certain distance, it jumps out
void forward(int distance){
      
    leftPos = 4; //Reset Wheel Position
    rightPos = 4;
    right_step_count = left_step_count = distance; 
    while(right_step_count >=0 || left_step_count >=0){
        if(front_prox >= 62 && ATD0DR0H >= 62) right_step_count = left_step_count = 0;   
    }    
    resetValues();
    wait(wait_time);
}



void right_rolling_turn(){//Doesn't Work for some reason
    turn_flag = 1;    
    leftPos = rightPos = 4; //Reset Wheel Position
    right_speed = right_speed_final = MAX_SPEED*2.5;
    left_speed = left_speed_final = MAX_SPEED;    
    right_step_count = 235;
    left_step_count = right_step_count*2.5;
    while(right_step_count >=0 || left_step_count >=0){}
    resetValues();    
    wait(wait_time);
    turn = UTURN;
}

void left_rolling_turn(){ 
    turn_flag = 1;     
    leftPos = rightPos = 4; //Reset Wheel Position
    right_speed = right_speed_final = MAX_SPEED;
    left_speed = left_speed_final = MAX_SPEED*2.5; 
    left_step_count = 235;
    right_step_count = left_step_count*2.5;    
    while(right_step_count >=0 || left_step_count >=0){}
    resetValues();
    wait(wait_time);
    turn = UTURN;
}    

void explore(){
    int oldDirection = 0;
    int makeTurnFlag = 0;
    static char digits[10] = {0};
    int position;
    char directionWrite;
    if(maze[x][y]==-1) distance++;
    else distance--;
    
    

    COMWRT4(0x01); // Clears display
    MSDelay(1);
   
    if(right_prox>40) DATWRT4('|');
    else DATWRT4(' ');     
    if(front_prox>40) DATWRT4('-');
    else DATWRT4(' '); 
    if(left_prox>40) DATWRT4('|');
    else DATWRT4(' ');
    DATWRT4('[');            
    position = sprintf(digits,"%d", x);
    DATWRT4(digits[0]);
    DATWRT4(']');
    DATWRT4('[');
    position = sprintf(digits,"%d", y);
    DATWRT4(digits[0]);
    DATWRT4(']');
    COMWRT4(0xC0); // Moves to second line ;    
    switch(direction) {
      case NORTH:
        directionWrite = 'N';
        break;
      case EAST:
        directionWrite = 'E';
        break;
      case WEST:
        directionWrite = 'W';
        break;
      case SOUTH:
        directionWrite = 'S';
        break;
    }
    DATWRT4(directionWrite);
    DATWRT4(' ');
    
    right_speed_final = left_speed_final = right_speed = left_speed = 400;
    wait(2000);       
    if(front_prox < 40) {
        DATWRT4('F');
        directionMaze[x][y] = direction;
        if ((direction == NORTH) && (maze[x][y+1] == -1)) {
                y++;
                makeTurnFlag = 1;
        }
        else if ((direction == EAST) && (maze[x+1][y] == -1)) {
                x++;
                makeTurnFlag = 1;
        }
        else if ((direction == SOUTH) && (maze[x][y-1] == -1)) {
                y--;
                makeTurnFlag = 1;
        }
        else if ((direction == WEST) && (maze[x-1][y] == -1)) {
                x--;
                makeTurnFlag = 1;
        }
        if(makeTurnFlag == 1) {
                push('F');
                maze[x][y] = distance;
                tracebackFlag = 1;
                forward(STATIONARYDISTANCE); 
                explore();
                orientate();
                resetFilters();
        }
    }
    
    if(right_prox < 40){
        DATWRT4('R');  
                    
        if ((direction == NORTH) && maze[x+1][y] == -1) {
                makeTurnFlag = 1;
        } else if ((direction == EAST) && (maze[x][y+1] == -1)) {
                makeTurnFlag = 1;
        } else if ((direction == SOUTH) && (maze[x-1][y] == -1)) {
                makeTurnFlag = 1;
        } else if ((direction == WEST) && (maze[x][y-1] == -1)) {; 
                makeTurnFlag = 1;
        }
        if(makeTurnFlag == 1) {
                push('R');
                right_stationary_turn();
                explore();
                orientate();
                resetFilters();
        }
    } 
    
    if(left_prox < 40){
        DATWRT4('L');
        if (direction == NORTH && maze[x-1][y] == -1) {
                makeTurnFlag = 1;
        } else if ((direction == EAST) && (maze[x][y-1] == -1 )) {
                makeTurnFlag = 1;
        } else if ((direction == SOUTH) && (maze[x+1][y] == -1)) {
                makeTurnFlag = 1;
        } else if ((direction == WEST) && (maze[x][y+1] == -1)) {
                makeTurnFlag = 1;
        }
        if(makeTurnFlag == 1) {
                push('L');
                left_stationary_turn();
                explore();
                orientate();
                resetFilters();
                
        }
    }
    
  
    
    if(tracebackFlag == 1) {
        u_turn();
        tracebackFlag = 0;
        DATWRT4('U');
        
        if (direction == NORTH) {
                direction = SOUTH;
              
        } else if ((direction == EAST)) {
                direction = WEST;
               
        } else if ((direction == SOUTH) ) {
                direction = NORTH;
               
        } else if ((direction == WEST) ) {
                direction = EAST;
        }
    }
    
    COMWRT4(0x01); // Clears display
    MSDelay(1);
   
    if(right_prox>40) DATWRT4('|');
    else DATWRT4(' ');     
    if(front_prox>40) DATWRT4('-');
    else DATWRT4(' '); 
    if(left_prox>40) DATWRT4('|');
    else DATWRT4(' ');
    DATWRT4('[');            
    position = sprintf(digits,"%d", x);
    DATWRT4(digits[0]);
    DATWRT4(']');
    DATWRT4('[');
    position = sprintf(digits,"%d", y);
    DATWRT4(digits[0]);
    DATWRT4(']');
    COMWRT4(0xC0); // Moves to second line ;    
    switch(direction) {
      case NORTH:
        directionWrite = 'N';
        break;
      case EAST:
        directionWrite = 'E';
        break;
      case WEST:
        directionWrite = 'W';
        break;
      case SOUTH:
        directionWrite = 'S';
        break;
    }
    DATWRT4(directionWrite);
    DATWRT4(' ');
    
    popValue = pop();
    for(i = 7; i >= 0; i--){
            front_filter[i]=50;
        }
    right_speed_final = left_speed_final = right_speed = left_speed = 400;
    wait(2000);
    front_prox = 20;
    switch(popValue){
        case 'F': DATWRT4('F');   
                if ((direction == NORTH)) {
                        y++;
                }
                else if ((direction == EAST)) {
                        x++;
                }
                else if ((direction == SOUTH)) {
                        y--;
                }
                else if ((direction == WEST)) {
                        x--;
                }                         
                forward(STATIONARYDISTANCE); break;
        
        case 'L':
                DATWRT4('L');
                right_stationary_turn(); break;
        
        case 'R': DATWRT4('R');
                left_stationary_turn(); break;
    }
    
}

void main(void) {
    int j=0;
    x = y =0;
    maze[x][y] = 0;
    
    SYNR = 2;
    REFDV = 0;




    while(!(CRGFLG&0x08)){}
    CLKSEL |= 0x80;
    
    MCCTL |= 0x04;
    MCCNT = 150;
    MCCTL = 0xC7;//Interrupt Enabled, Force Load, Enabled, P=16

    DDRK=0xFF;
    DDRB=0xFF;
    DDRH=0xFF;
  
    DDRK = 0xFF;
    COMWRT4(0x33); //reset sequence provided by data sheet
    COMWRT4(0x32); //reset sequence provided by data sheet
    COMWRT4(0x28);;
    COMWRT4(0x06);
    COMWRT4(0x0E);
    COMWRT4(0x01);
    COMWRT4(0x80);

    ATD0CTL2 = 0x80; //Turns on ATD converter
    ATD0CTL3 = 0x18; //Length=3 FIFO=0
    ATD0CTL4 = 0x80; //8-bit resolution
    //ATD0CTL5 = 0x32;
    
    // Bertha Values
    ATD0CTL5 = 0x33; //Left Justified, Unsigned, SCAN=1, MULTI=1, START=3

    for(i = 6; i >= 0; i--){//Initialize maze to -1
        for(j = 6; j >= 0; j--){
            maze[i][j]= -1;
        }
    }
;
    EnableInterrupts;
    resetValues();
    explore();
    // Loops forever making the correct turns
    for(;;) { }
}






#pragma TRAP_PROC
#pragma CODE_SEG __SHORT_SEG NON_BANKED
interrupt VectorNumber_Vtimmdcu void mdcuInterrupt () {
    MCFLG |= 0x80;//Always Clear the Flag
    left_count++;
    right_count++;
    ramp_count++; 
    read_count++;
    wait_count++;
    
    //The Wall follow is commented out for debugging reasons
    if((read_count%100)==0 && turn_flag!=1){    // Get new reading every 10 ms

        /*
        front_prox = ATD0DR0H;
        left_prox = ATD0DR1H;
        right_prox = ATD0DR2H;
        */
        
        // Bertha Values
        front_prox = ATD0DR2H;
        left_prox = ATD0DR1H;
        right_prox = ATD0DR0H;
                
        front_filter[i] = front_prox;
        left_filter[i] = left_prox;
        right_filter[i] = right_prox;
        i++;
        front_prox = (front_filter[0] + front_filter[1] + front_filter[2] + front_filter[3] + front_filter[4] + front_filter[5] + front_filter[6] + front_filter[7])/8;
        left_prox = (left_filter[0] + left_filter[1] + left_filter[2] + left_filter[3] + left_filter[4] + left_filter[5] + left_filter[6] + left_filter[7])/8;
        right_prox = (right_filter[0] + right_filter[1] + right_filter[2] + right_filter[3] + right_filter[4] + right_filter[5] + right_filter[6] + right_filter[7])/8;
        if (i >= 8) i = 0;
        

        previousErrorR = currentErrorR;
        currentErrorR = ideal_distance - right_prox;
        errorDirR = currentErrorR - previousErrorR;


        previousErrorL = currentErrorL;
        currentErrorL = ideal_distance - left_prox;
        errorDirL = currentErrorL - previousErrorL;        

        read_count = 0;
        
     if(lastWall == LEFT) {                
                  if(left_prox > max_wall_distance && turn_flag != 1) {
                         lastWall = LEFT;
                        if(left_prox < ideal_distance) {
                                right_speed_final = MAX_SPEED;
                                left_speed_final = MAX_SPEED + (p_value * (ideal_distance - left_prox)) + (d_value * errorDirL);
                        } else {
                                left_speed_final  = MAX_SPEED;
                                right_speed_final  = MAX_SPEED + (-p_value * ( ideal_distance - left_prox)) + (-d_value * errorDirL);
                        }
                  } else if(left_prox < max_wall_distance && right_prox > max_wall_distance && turn_flag!=1) {
                        lastWall = RIGHT;
                        if (right_prox < (ideal_distance)){//If too far from right wall correct        
                                left_speed_final  = MAX_SPEED;
                                right_speed_final  = MAX_SPEED +(p_value * (ideal_distance - right_prox)) + (d_value * errorDirR); 
                        } else{     //If too close to right wall correct   
                                right_speed_final  = MAX_SPEED;
                                left_speed_final  = MAX_SPEED +(-p_value * (ideal_distance - right_prox)) + (-d_value * errorDirR); 
                        }
                 } else right_speed_final = left_speed_final = MAX_SPEED;
     }
        if(lastWall == RIGHT) { 
            if(right_prox > max_wall_distance && turn_flag != 1) {
                 lastWall = RIGHT;
                 if(right_prox < ideal_distance) {
                       left_speed_final  = MAX_SPEED;
                       right_speed_final  = MAX_SPEED +(p_value * (ideal_distance - right_prox)) + (d_value * errorDirR);
                 } else {     //If too close to right wall correct   
                       right_speed_final  = MAX_SPEED;
                       left_speed_final  = MAX_SPEED +(-p_value * (ideal_distance - right_prox)) + (-d_value * errorDirR);
                 }
              } else if(right_prox < max_wall_distance && left_prox > max_wall_distance && turn_flag != 1) {
                 lastWall = LEFT;
                 if(left_prox < ideal_distance) {
                        right_speed_final = MAX_SPEED;
                        left_speed_final = MAX_SPEED + (p_value * (ideal_distance - left_prox)) + (d_value * errorDirL);
                 } else {
                        left_speed_final  = MAX_SPEED;
                        right_speed_final  = MAX_SPEED +(-p_value * ( ideal_distance - left_prox)) + (-d_value * errorDirL);
                 }
            } else right_speed_final = left_speed_final = MAX_SPEED;     
        }
    }
    
    // Ramping   
    if (ramp_count == rampCoef){
         
        if(right_speed < right_speed_final){
            right_speed +=1;    //slow it down
        }else if(right_speed>right_speed_final){
            right_speed -=1;    //speed it up
        }  
        if(left_speed < left_speed_final){
            left_speed +=1;    //slow it down
        }else if(left_speed>left_speed_final){
            left_speed -=1;    //speed it up
        }        
        ramp_count = 0;
    }
    
    if(left_wheel_backwards != 0 && left_wheel_backwards != 1){ left_wheel_backwards = 0; left_speed = 150;}
    if(right_wheel_backwards != 0 && right_wheel_backwards != 1){ right_wheel_backwards = 0; right_speed =150;}

    //Increment Left Wheel
    if(left_count>=left_speed){
        if(left_wheel_backwards <= 0){//Forwards
            leftPos++;
            if(leftPos==8)leftPos = 0;
            left_count=0;
            left_step_count--;
        }else if(left_wheel_backwards >= 1){//Backwards
            leftPos--;
            if(leftPos<=0)leftPos = 8;
            left_count=0;
            left_step_count--;
            //u_turn_count++;
        }
    }
     
    //Increment Right Wheel
    if(right_count>=right_speed){
        if(right_wheel_backwards <= 0){//Forwards 
            rightPos++;
            if(rightPos==8)rightPos = 0;
            right_count=0;
            right_step_count--;
        }else if(right_wheel_backwards >= 1){//Backwards
            rightPos--;
            if(rightPos<=0)rightPos = 8;
            right_count=0;
            right_step_count--;
            //u_turn_count++;
        }
    }
     
  
  PORTB = left_chars[leftPos] + right_chars[rightPos];
    }