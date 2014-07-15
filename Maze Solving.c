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
#define MAX_SPEED 27
#define MIN_SPEED 250
#define FRONT 0
#define LEFT 10
#define RIGHT 20
#define NORTH 4
#define EAST 1
#define SOUTH 2
#define WEST 3
#define THREE 3
#define TWO 2
#define ONE 1

#define STATIONARYDISTANCE 506

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
int currentErrorL = 0;
int currentErrorR = 0;
static short int directionMaze[7][7];
static int distance = 0;
static int direction = NORTH;
static int doNotMove; 
static short int explored[7][7];
static int errorDirL = 0;
static int errorDirR = 0;
static int front_filter[8];
static int front_prox = 0;
static int goalX, goalY;
int i = 0;
static int ideal_distance = 64;
static int lastWall = LEFT;
static int left_filter[8];
static int left_prox = 0;
static int left_count=200;
static int left_speed_final = MAX_SPEED;
static int left_speed  = MIN_SPEED-100;
static int leftPos = 0;
static unsigned int left_wheel_backwards = 0;//Sets left wheel motion backwards
static short int maze[7][7];
static short int mazeTurn[7][7];
int max_wall_distance = 45;
static short int orientateFlag = 0;
static char popValue = ' ';
static int previousErrorL = 0;
static int previousErrorR = 0;
static int left_step_count = 0;
static int read_count=0;
static int right_filter[8];
static int right_prox = 0;
static int right_step_count = 0;
static int right_count=200;
static int ramp_count = 0;
static int right_speed_final = MAX_SPEED;
static int right_speed = MIN_SPEED-100;
static int rightPos = 0;
static int rampCoef = 35;
static unsigned int right_wheel_backwards = 0;//Sets right wheel motion backwards
static char stack[100];
static int sTop = 2;
static int tracebackFlag = 0;
static int turn_flag = 1;//Used to stop sensor reading and correction while turning
static int wait_count = 0;
static int wait_time = 100;
static char which_turn = ' ';
static int x, y;

 

// .22 || .55
// .3  || .6
// Changing pd to their values
float p_value = .2;//.175 best
float d_value = .375;//.327
static int PDREAD = 5.25;

static char right_chars[8] ={0xC0, 0x40, 0x60, 0x20, 0x30, 0x10, 0x90, 0x80};
static char left_chars[8] = {0x08, 0x09, 0x01, 0x03, 0x02, 0x06, 0x04, 0x0C};

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

void resetFilters() {

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
void wait(int time) {
        while ((wait_count % time) != 0) {}
        wait_count = 0;
}

// Helps our 
void resetValues() {
    turn_flag = 0;
    right_step_count = left_step_count = 0; 
    right_wheel_backwards = left_wheel_backwards = 0;;
    right_speed_final = left_speed_final = MAX_SPEED; 
}

// Moves forward for a given distance, if the front prox reaches a certain distance, it jumps out
void forward (int forwardDistance) {
     
    leftPos = 4; //Reset Wheel Position
    rightPos = 4;
    right_step_count = left_step_count = forwardDistance; 
    while (right_step_count >=0 || left_step_count >=0) {
        if (front_prox >= 62 && ATD0DR0H >= 62) right_step_count = left_step_count = 0;   
    }    
    resetValues();
 //   wait(wait_time);
}


void left_stationary_turn() {
    turn_flag = 1;
    right_speed = left_speed = 100;
    left_speed_final = right_speed_final = MAX_SPEED; 
    left_wheel_backwards = 1;
    right_step_count = left_step_count = 195;    
    while(right_step_count >=0 || left_step_count >=0){} 
    resetValues();
    resetFilters();
    forward(15);
    doNotMove = 1;
    if (right_prox < 40 || left_prox < 40) {
        doNotMove = 1;    
        wait(2000);
        doNotMove = 0;
    } 
    resetValues();
    resetFilters();    
}

void right_stationary_turn(){
    int changeDirection = direction; 
    
    turn_flag = 1;
    right_speed = left_speed = 100;
    right_speed_final = left_speed_final = MAX_SPEED; 
    right_wheel_backwards = 1;
    right_step_count = left_step_count = 195;    
    while (right_step_count >=0 || left_step_count >=0) {}
    resetValues();
    resetFilters();
    forward(15);
    doNotMove = 1;
    if(right_prox < 40 || left_prox < 40) {
        doNotMove = 1;    
        wait(2000);
        doNotMove = 0;
    } 
    resetValues();
    resetFilters();
      
}

void u_turn(){
    turn_flag = 1;
    right_speed_final = left_speed_final = 33;// To Minimize spillage
    turn_flag = 1;
    leftPos = rightPos = 4; //Reset Wheel Position
    right_speed_final = MAX_SPEED+10;
    left_speed_final = MAX_SPEED+10;    
    left_wheel_backwards = 1;
    right_step_count = 431;
    left_step_count = 431;
    while(right_step_count >=0 || left_step_count >=0){}
    resetValues();     
}
       

char pop(){
        char temp = stack[sTop - 1];
        stack[sTop - 1] = ' ';
        sTop--;

        return temp;
      
}

int push(char temp){
        if(sTop == 99)
        return 0;//Fail, to big for array
        
        stack[sTop++] = temp;
        return 1;
}

void explore(){
    int oldDirection = direction;
    int exploredSet = 0;
    int exploreFlag = 0;
    int makeTurnFlag = 0;
    static char digits[10] = {0};
    int position;
    char directionWrite;
    int oldY = y;
    int oldX = x;
    int i, j;
    
    resetFilters();
    for (i = 0; i < 7; i++) {
        for (j = 0; j < 7; j++) {
               if (i != 4 || j !=4) {
                //Check along left edge 
                if (maze[0][j] == -1 && maze[0][j+1] !=-1 && maze[0][j-1] !=-1)
                        maze[i][j] = 100;
                
                //Check along bottom edge
                if (maze[i][0] == -1 && maze[i+1][0] !=-1 && maze[i-1][0] !=-1) 
                        maze[i][j] = 100;
                
                //Check along right edge
                if (maze[6][j] == -1 && maze[6][j+1] !=-1 && maze[6][j-1] !=-1) 
                        maze[i][j] = 100;
                
                //Check along top edge
                if (maze[i][6] == -1 && maze[i+1][6] !=-1 && maze[i-1][6] !=-1) 
                        maze[i][j] = 100;
                
                if (maze[i][j] == -1 && maze[i+1][j] !=-1 && maze[i-1][j] !=-1  && maze[i][j+1] !=-1 && maze[i][j-1] !=-1)//Check insides 
                        maze[i][j] = 100;
            }
        }
    }

    COMWRT4(0x01); // Clears display
    MSDelay(1);
    DATWRT4(' ');
    if(right_prox>40) {
        DATWRT4('|');
    }
    else {
        DATWRT4(' '); 
    }
    if(front_prox>40) {
        DATWRT4('-');
    }
    else {
        DATWRT4(' ');
    }
    if(left_prox>40) {
        DATWRT4('|');
    }
    else {
        DATWRT4(' ');
    }
    if(mazeTurn[x][y] == -1) {
       mazeTurn[x][y] = 0;
       if(front_prox<40) mazeTurn[x][y]++;
       if(left_prox<40) mazeTurn[x][y]++;
       if(right_prox<40) mazeTurn[x][y]++;
    }
    if(x == 3 && y == 4) {
        if(direction == EAST && front_prox < 40) { forward(STATIONARYDISTANCE); x++; doNotMove = 1;}
        if(direction == NORTH && right_prox < 40) { right_stationary_turn(); direction = EAST;  forward(STATIONARYDISTANCE); x++;  explore(); }
        if(direction == SOUTH && left_prox < 40) { left_stationary_turn(); direction = EAST; forward(STATIONARYDISTANCE); x++; explore(); }
    }
    
    if(x == 5 && y == 4) {
        if(direction == WEST && front_prox < 40) { forward(STATIONARYDISTANCE); x--;  doNotMove = 1;}
        if(direction == SOUTH && right_prox < 40) { right_stationary_turn(); direction = WEST; forward(STATIONARYDISTANCE); x--; explore(); }
        if(direction == NORTH && left_prox < 40) { left_stationary_turn(); direction = WEST; forward(STATIONARYDISTANCE); x--; explore(); }
    }
    
    if(x == 4 && y == 5) {
        if(direction == SOUTH && front_prox < 40) { forward(STATIONARYDISTANCE); y--; doNotMove = 1;}
        if(direction == WEST && left_prox < 40) { left_stationary_turn(); direction = SOUTH; forward(STATIONARYDISTANCE); y--; explore(); }
        if(direction == EAST && right_prox < 40) { right_stationary_turn(); direction = NORTH; forward(STATIONARYDISTANCE); y--; explore(); }
    }
    
    if(x == 4 && y == 3) {
        if(direction == NORTH && front_prox < 40) { forward(STATIONARYDISTANCE); y++;  doNotMove = 1;}
        if(direction == EAST && left_prox < 40) { left_stationary_turn(); direction = NORTH; forward(STATIONARYDISTANCE); y++; explore(); }
        if(direction == WEST && right_prox < 40) { right_stationary_turn(); direction = NORTH; forward(STATIONARYDISTANCE); y++; explore(); }
    }
    
    if(x==4 && y == 4) {
        u_turn(); COMWRT4(0x10); DATWRT4('G');DATWRT4('O');DATWRT4('A');DATWRT4('L');  for(i=0; i<10; i++){if(i==8) i= 0;u_turn(); u_turn(); PORTK = 0x181; }
    }
    
    
    position = sprintf(digits,"%d", x);
    DATWRT4(digits[0]);
    DATWRT4('-');
    position = sprintf(digits,"%d", y);
    DATWRT4(digits[0]);
    COMWRT4(0xC0); // Moves to second line 
    DATWRT4(' ');;    
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

        
    if(right_prox > 40 && left_prox > 40 && front_prox > 40) {
        explored[x][y] = 1;
        maze[x][y] = 100;
    }

    else if(front_prox < 40) { 
        
        switch(direction) {
            case NORTH:
                if (((maze[x-1][y] != -1) || (left_prox > 40)) && ((maze[x+1][y] != -1) || (right_prox > 40)) && (explored[x][y] != 1)) {                                
                    exploreFlag = 1;
                    if (exploredSet ==0) {
                        popValue = pop();
                        switch(popValue){
                            case 'F':
                                push('F');
                                break;
                            case 'L':
                                push('R');
                                break;
                            case 'R':
                                push('L');
                                break;
                        }
                    }
                    exploredSet=1;
                }
                break;
            case SOUTH:
                 if (((maze[x-1][y] != -1) || (right_prox > 40)) && ((maze[x+1][y] != -1) || (left_prox > 40)) && (explored[x][y] != 1)) {
                    exploreFlag = 1;
                    if (exploredSet ==0) {
                        popValue = pop();
                        switch(popValue){
                            case 'F':
                                push('F');
                                break;
                            case 'L':
                                push('R');
                                break;
                            case 'R':
                                push('L');
                                break;
                        }
                    }
                    exploredSet=1;
                }
                break;                       
            case EAST:
                 if (((maze[x][y-1] != -1) || (right_prox > 40)) && ((maze[x][y+1] != -1) || (left_prox > 40)) && (explored[x][y] != 1)) {
                    exploreFlag = 1;
                    if (exploredSet ==0) {
                        popValue = pop();
                        switch(popValue){
                            case 'F':
                                push('F');
                                break;
                            case 'L':
                                push('R');
                                break;
                            case 'R':
                                push('L');
                                break;
                        }
                    }
                    exploredSet=1;
                 }
                break;
            case WEST:
                if (((maze[x][y-1] != -1) || (right_prox > 40))&& ((maze[x][y+1] != -1) || (right_prox > 40)) && (explored[x][y] != 1)) {
                    exploreFlag = 1;
                    if (exploredSet ==0) {
                        popValue = pop();
                        switch(popValue) {
                            case 'F':
                                push('F');
                                break;
                            case 'L':
                                push('R');
                                break;
                            case 'R':
                                push('L');
                                break;
                        }
                    }
                    exploredSet=1;
                }
                break;
        }
        
        if ((direction == NORTH) && (maze[x][y+1] == -1)) {
            if(right_prox > 40 && left_prox > 40 && explored[x][y] != 1){
                explored[x][y] = 1;
                if(exploredSet ==0){
                        popValue = pop();
                        switch(popValue){
                            case 'F':
                                push('F');
                                break;
                            case 'L':
                                push('R');
                                break;
                            case 'R':
                                push('L');
                                break;
                        }
                }
                exploredSet=1;
            }
            mazeTurn[x][y]--;
            maze[x][y] = ++distance;              
            y++;
            makeTurnFlag = 1;
        }
  
        else if ((direction == EAST) && (maze[x+1][y] == -1)) {
                 if(right_prox > 40 && left_prox > 40 && explored[x][y] != 1){
                        explored[x][y] = 1;
                        if(exploredSet ==0){
                                popValue = pop();
                                switch(popValue){
                                         case 'F':push('F');break;
                                         case 'L':push('R');break;
                                         case 'R':push('L');break;
                                }
                        }
                        exploredSet=1;
                }
                mazeTurn[x][y]--;
                maze[x][y] = ++distance;               
                x++;
                makeTurnFlag = 1; 
        }
        else if ((direction == SOUTH) && (maze[x][y-1] == -1)) {
                 if(right_prox > 40 && left_prox > 40 && explored[x][y] != 1){
                        explored[x][y] = 1;
                        if(exploredSet ==0){
                                popValue = pop();
                                switch(popValue){
                                         case 'F':push('F');break;
                                         case 'L':push('R');break;
                                         case 'R':push('L');break;
                                }
                        }
                        exploredSet=1;
                 }
                 mazeTurn[x][y]--;
                maze[x][y] = ++distance;
                y--;
                makeTurnFlag = 1;
        }
        
        else if ((direction == WEST) && (maze[x-1][y] == -1)) {
                if(right_prox > 40 && left_prox > 40 && explored[x][y] != 1){
                        explored[x][y] = 1;
                        
                        if(exploredSet ==0){
                                popValue = pop();
                                switch(popValue){
                                         case 'F':push('F');break;
                                         case 'L':push('R');break;
                                         case 'R':push('L');break;  
                                }
                        }
                        exploredSet=1;
                        
                }
                mazeTurn[x][y]--;
                maze[x][y] = ++distance;
                x--;
                makeTurnFlag = 1;
        }
                
        if(makeTurnFlag == 1) {
                DATWRT4('F');
        
                DATWRT4(' ');
                position = sprintf(digits,"%d", maze[x][y]);
                DATWRT4(digits[0]);
                
                push('F');
                directionMaze[x][y] = direction;
                tracebackFlag = 1;
                forward(STATIONARYDISTANCE);

                explore();
                if(explored[x][y] ==0 || mazeTurn[oldX][oldY]>0) {
                    if(exploreFlag == 1) {
                        explored[oldX][oldY] = 1;
                    }
                   explore();
                   
                   resetFilters();
                   wait(100);
                }          
                
                resetFilters();
                makeTurnFlag = 0;
       }
    }
    
    if(right_prox < 40 || left_prox < 40) {
        doNotMove = 1;    
        wait(2000);
        doNotMove = 0;
    }
    
    if(left_prox < 40){
        int makeTurnFlagL = 0;
        switch(direction) {
                case NORTH:
                        if(((maze[x-1][y] != -1) || (left_prox > 40)) && ((maze[x+1][y] != -1) || (right_prox > 40)) && (explored[x][y] != 1)){                                
                                explored[x][y] = 1;
                                if(exploredSet ==0){
                                        popValue = pop();
                                        switch(popValue){
                                                case 'F':push('F');break;
                                                case 'L':push('R');break;
                                                case 'R':push('L');break;
                                        }
                                }
                                exploredSet=1;
                        }
                        break;
                case SOUTH:
                         if(((maze[x-1][y] != -1) || (right_prox > 40)) && ((maze[x+1][y] != -1) || (left_prox > 40)) && (explored[x][y] != 1)){
                                explored[x][y] = 1;
                                if(exploredSet ==0){
                                        popValue = pop();
                                        switch(popValue){
                                                case 'F':push('F');break;
                                                case 'L':push('R');break;
                                                case 'R':push('L');break;
                                        }
                                }
                                exploredSet=1;
                        }
                        break;                       
                case EAST:
                         if(((maze[x][y-1] != -1) || (right_prox > 40)) && ((maze[x][y+1] != -1) || (left_prox > 40)) && (explored[x][y] != 1)) {
                                explored[x][y] = 1;
                                if(exploredSet ==0){
                                        popValue = pop();
                                        switch(popValue){
                                                case 'F':push('F');break;
                                                case 'L':push('R');break;
                                                case 'R':push('L');break;
                                        }
                                }
                                exploredSet=1;
                         }
                        break;
                case WEST:
                        if(((maze[x][y-1] != -1) || (right_prox > 40))&& ((maze[x][y+1] != -1) || (right_prox > 40)) && (explored[x][y] != 1)) {
                                explored[x][y] = 1;
                                if(exploredSet ==0){
                                        popValue = pop();
                                        switch(popValue){
                                                case 'F':push('F');break;
                                                case 'L':push('R');break;
                                                case 'R':push('L');break;
                                        }
                                }
                                exploredSet=1;
                        }
                        break;
        }
        
       
        if (direction == NORTH && maze[x-1][y] == -1) {
            makeTurnFlagL = 1;
            if(right_prox > 45  && explored[x][y] != 1){
                    explored[x][y] = 1;
                    if(exploredSet ==0){
                        popValue = pop();
                        switch(popValue){
                             case 'F':push('F');break;
                             case 'L':push('R');break;
                             case 'R':push('L');break;
                        }
                    exploredSet=1;
                    }
            }
            
        } else if ((direction == EAST) && (maze[x][y+1] == -1 )) {
                makeTurnFlagL = 1;
                if(right_prox > 45  && explored[x][y] != 1){
                    explored[x][y] = 1;
                    
                    if(exploredSet ==0){
                        popValue = pop();
                        switch(popValue){
                             case 'F':push('F');break;
                             case 'L':push('R');break;
                             case 'R':push('L');break;
                        }
                    exploredSet=1;
                    }
                }
                      
        } else if ((direction == SOUTH) && (maze[x+1][y] == -1)) {
            makeTurnFlagL = 1;
            if(right_prox > 45  && explored[x][y] != 1){
                explored[x][y] = 1;
                
                if(exploredSet ==0){
                    popValue = pop();
                    switch(popValue){
                         case 'F':push('F');break;
                         case 'L':push('R');break;
                         case 'R':push('L');break;
                    }
                    exploredSet=1;
                }   
            
            }
                    
        } else if ((direction == WEST) && (maze[x][y-1] == -1)) {
            makeTurnFlagL = 1;
            if(right_prox > 45 && explored[x][y] != 1 ){
                explored[x][y] = 1;
                if(exploredSet ==0){
                        popValue = pop();
                        switch(popValue){
                             case 'F':push('F');break;
                             case 'L':push('R');break;
                             case 'R':push('L');break;
                        }
                        exploredSet=1;
                }
            }    
        }
        if(makeTurnFlagL == 1){
            push('L');
            DATWRT4('L');
            left_stationary_turn();
            if (direction == NORTH) {
                direction = WEST;
            } else if ((direction == EAST)) {
                direction = NORTH;
            } else if ((direction == SOUTH) ) {
                direction = EAST;
            } else if ((direction == WEST) ) {
                direction = SOUTH;
            }           
            explore();
            resetFilters();
            
            if(explored[x][y] == 0) {
               explore();
               resetFilters();
            }
            makeTurnFlagL = 0;   
        }
    }
    
    if(right_prox < 40){
        int makeTurnFlagR = 0; 
        if (explored[x][y] != 1){
            explored[x][y] = 1;
            if(exploredSet ==0){
                popValue = pop();
                switch(popValue){
                     case 'F':push('F'); break;
                     case 'L':push('R'); break;
                     case 'R':push('L'); break;
                }
                exploredSet=1; 
            }   
        }             
        if ((direction == NORTH) && maze[x+1][y] == -1) {
            makeTurnFlagR = 1;
        } else if ((direction == EAST) && (maze[x][y-1] == -1)) {
                makeTurnFlagR = 1;
        } else if ((direction == SOUTH) && (maze[x-1][y] == -1)) {
                makeTurnFlagR = 1;
        } else if ((direction == WEST) && (maze[x][y+1] == -1)) { 
                makeTurnFlagR = 1;
        }
       if((makeTurnFlagR == 1)){
                DATWRT4('R');
                push('R');
                right_stationary_turn();
                if (direction == NORTH) {
                       direction = EAST;
                    } else if ((direction == EAST)) {
                      direction = SOUTH;
                    } else if ((direction == SOUTH) ) {
                       direction = WEST;
                    } else if ((direction == WEST) ) {
                      direction = NORTH;
                    }

                explore();
                resetFilters();
                makeTurnFlag = 0;
       }
    }

    
    if(tracebackFlag == 1) {
        DATWRT4('U');
        u_turn();
        if (direction == NORTH) {
                direction = SOUTH;
              
        } else if ((direction == EAST)) {
                direction = WEST;
               
        } else if ((direction == SOUTH) ) {
                direction = NORTH;
               
        } else if ((direction == WEST) ) {
                direction = EAST;
        }        
        tracebackFlag = 0;
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
    resetFilters();  
    
    switch(popValue){
        case 'F':
            distance--;   
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
            COMWRT4(0x01); // Clears display
            MSDelay(1);
                   
            DATWRT4(' ');
            if(right_prox>40) DATWRT4('|');
            else DATWRT4(' ');     
            if(front_prox>40) DATWRT4('-');
            else DATWRT4(' '); 
            if(left_prox>40) DATWRT4('|');
            else DATWRT4(' ');
            DATWRT4(' ');          
            position = sprintf(digits,"%d", x);
            DATWRT4(digits[0]);
            DATWRT4('-');
            position = sprintf(digits,"%d", y);
            DATWRT4(digits[0]);
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
            DATWRT4('P');
            DATWRT4('P'); 
            DATWRT4(' ');
            position = sprintf(digits,"%d", maze[x][y]);
            DATWRT4(digits[0]);              
            forward(STATIONARYDISTANCE);
            break;
            
        case 'R':                         
        case 'L':
            switch(direction) {
                case NORTH:
                
                    if (((maze[x-1][y] < maze[x+1][y]) && (left_prox < 40))) { 
                        left_stationary_turn();
                        which_turn = 'L';
                    } else {
                        right_stationary_turn();
                        which_turn = 'R';
                    }
                    break;                      
                case SOUTH:
                    if (((maze[x+1][y] < maze[x-1][y]) && (left_prox < 40))) { 
                        left_stationary_turn(); 
                        which_turn = 'L';
                    } else {
                        right_stationary_turn();
                        which_turn = 'R';
                    }                         
                    break;                       
                case EAST:
                    if (((maze[x][y+1] < maze[x][y-1]) && (left_prox < 40))) { 
                        left_stationary_turn();
                        which_turn = 'L';
                    } else {
                        right_stationary_turn();
                        which_turn = 'R';
                    }                         
                    break;
                case WEST:
                    if (((maze[x][y-1] < maze[x][y+1]) && (left_prox < 40))){ 
                        left_stationary_turn();
                        which_turn = 'L';
                    } else {
                        right_stationary_turn();
                        which_turn = 'R';
                    }                        
                    break;
            }
            
            if (which_turn == 'R') {
                DATWRT4('R');
                if (direction == NORTH) {
                   direction = EAST;
                } else if ((direction == EAST)) {
                  direction = SOUTH;
                } else if ((direction == SOUTH) ) {
                   direction = WEST;
                } else if ((direction == WEST) ) {
                  direction = NORTH;
                }       
            } else if (which_turn == 'L') {
                DATWRT4('L');
                if (direction == NORTH) {
                        direction = WEST;
                    } else if ((direction == EAST)) {
                        direction = NORTH;
                    } else if ((direction == SOUTH) ) {
                       direction = EAST;
                    } else if ((direction == WEST) ) {
                      direction = SOUTH;
                    }
                } 
        break;        
    }
    
    
     
        
 }

void main(void) {
    int j=0;
    x = y =0;
    goalX = goalY = 4;
    
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
    COMWRT4(0x28);
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
            explored[i][j] = 0;
            mazeTurn[i][j] = -1;
        }
    }
    maze[0][0] = 0;

    EnableInterrupts;      
    resetValues();
    push('F');
    explore();
    u_turn();
    direction = NORTH;
    turn_flag = 1;
    doNotMove = 1;
    COMWRT4(0x01); DATWRT4(' ');DATWRT4('D');DATWRT4('O');DATWRT4('N');DATWRT4('E');
    
    // Loops forever making the correct turns
    for(;;) {}
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

    //read_count = 0;
    
     if(right_prox > 40 && left_prox > 40) ideal_distance = (right_prox+left_prox)/2;
     else ideal_distance = 64;
     if((read_count%PDREAD)==0 && turn_flag!=1){    
     if(lastWall == LEFT) { 
                  read_count = 0;               
                  if(left_prox > max_wall_distance && turn_flag != 1) {
                         lastWall = LEFT;
                        if(left_prox < ideal_distance) {//Too far from left wall
                                right_speed_final = MAX_SPEED;
                                left_speed_final = MAX_SPEED + (p_value * (ideal_distance - left_prox)) + (d_value * errorDirL);
                        } else {
                                left_speed_final  = MAX_SPEED;//Too close to wall
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
            if(leftPos>7)leftPos = 0;
            left_count=0;
            left_step_count--;
        }else if(left_wheel_backwards >= 1){//Backwards
            leftPos--;
            if(leftPos<0)leftPos = 8;
            left_count=0;
            left_step_count--;
        }
    }
     
    //Increment Right Wheel
    if(right_count>=right_speed){
        if(right_wheel_backwards <= 0){//Forwards 
            rightPos++;
            if(rightPos>7)rightPos = 0;
            right_count=0;
            right_step_count--;
        }else if(right_wheel_backwards >= 1){//Backwards
            rightPos--;
            if(rightPos<0) rightPos = 8;
            right_count=0;
            right_step_count--;
        }
    }
     
  
  if(doNotMove != 1) PORTB = left_chars[leftPos] + right_chars[rightPos];
}