/*
 * Title:       Team 4 Robot State Machine Code
 * Filename:    EGR107S10_Team4_RobotStateMachineFINAL
 * Author(s):   Dustin Matthews
 * Date:        3/1/2020
 * Instructor:  Professor KC
 * 
 * Revised: 4/18/20
 * 
 * Notes: The code starts by declaring multiple global variables 
 * so that they can be passed around the functions easily, and 
 * should also make the code easier to modify. 
 *
 * In operation, the code first reads the sensors, which pass
 * their value back into the loop as a variable. The code 
 * then enters a switch case to control various motor and impeller
 * functions. All the functions will execute based on either 
 * timing via millis() or based on the triggering of a switch or sensor reading.
 *
 */

#include <Servo.h>

//Declare states for the state machine
enum State_enum {FWD, REV, FWD_TURN, REV_TURN, SCORE, SCORE2, RETREAT}; 

static unsigned int state; //switch state for main loop
static unsigned int function_state; //switch state for functions

int randNum; //used to determine random turns if sensors do not read values

//declare various functions for operating the robot
void motors_fwd();
void motors_rev();
void motors_right();
void motors_left();
void right_reverse();
void left_reverse();
void score();
void score_fwd();

//assign servo variables
Servo flapservo;
int ServoSpeed = 20;
byte servoPin = 9;

//assign distance sensor functions
float front_sensor();
float right_sensor();
float left_sensor();
  
//assign distance sensor pins
const int TRIG_1 = 11;
const int ECHO_1 = 5;
const int TRIG_2 = 13;
const int ECHO_2 = 12;
const int TRIG_3 = 16;
const int ECHO_3 = 17;

//assign switch pins for the microswitches
int BOT_SWITCH = 14;
int TOP_SWITCH = 15;

//assign motor pins
const int motorA1 = 4;
const int motorA2 = 2;
const int ENA = 3;
const int motorB1 = 7;
const int motorB2 = 8;
const int ENB = 6;

//assign timekeeping variables
unsigned long currentMillis;
unsigned long startMillis;
unsigned long functionMillis;
//timekeeping variables for different actions
const unsigned long interval = 6000;
const unsigned long rev_time = 1000;
const unsigned long turn_time = 900;

//assign distance to command the robot to drive forward and attempt a goal
float GOAL_DIST = 115;

//assign distance variables for initiating a turn
const int front_dist = 8;
const int side_dist = 8;

//Setup
void setup() {
  Serial.begin(9600); //initiate serial communication

//configure sensor pins
pinMode(TRIG_1,OUTPUT);
  digitalWrite(TRIG_1, LOW);
  
pinMode(TRIG_2,OUTPUT);
  digitalWrite(TRIG_2, LOW);

pinMode(TRIG_3,OUTPUT);
  digitalWrite(TRIG_3, LOW);

//assign motor pins as output
pinMode(motorA1,OUTPUT);
pinMode(motorA2,OUTPUT);
pinMode(motorB1,OUTPUT);
pinMode(motorB2,OUTPUT);
pinMode(ENA,OUTPUT);
pinMode(ENB,OUTPUT);

//configure servo pin
flapservo.attach(servoPin);

startMillis = millis(); //get the start time 

}

void loop() {
//Assign distance variables for sensors
float dist_f, dist_r, dist_l; 

//call sensors and return distances read to the loop in centimeters
dist_f = front_sensor(); 
dist_r = right_sensor();
dist_l = left_sensor();

BOT_SWITCH = digitalRead(14); //read state of BOT_SWITCH
TOP_SWITCH = digitalRead(15); //read state of TOP_SWITCH

currentMillis = millis(); //get time of loop start

//Serial.println(dist_f); //Used to determine an appropriate distance for the GOAL_DIST variable

switch (state) //enter switch case
{
    case FWD:
    
      flapservo.write(ServoSpeed); //initiate Servo to spin forward
      motors_fwd(); //initialize motors to propel the robot forward
      
      //if time interval is exceeded, enter the reverse state \
      if((unsigned long)(currentMillis - startMillis) > interval) 
      {   
        functionMillis = millis();
        state = REV;
      }
      //if front sensor reads a wall < 8 cm away, enter the reverse staete
      else if (dist_f <= front_dist)
      {
        Serial.println(dist_f);
        functionMillis = millis();
        state = REV;
      }
      else if (BOT_SWITCH == LOW) 
      //obstacle or wall encountered, enter the reverse turn state
      {
        Serial.println("BOTTOM SWITCH");
        randNum = random(1,3); //pass random number to REV_TURN
        functionMillis = millis(); //get current time, pass to REV_TURN
        state = REV_TURN;
      }
      else if (TOP_SWITCH == LOW)
      //top switch triggered, attempt goal
      {
        functionMillis = millis();
        state = SCORE;
      } 
      else if (dist_f > GOAL_DIST)
      //the front sensor detects a distance that may be a goal and attempts to drive toward it 
      {
        Serial.println("Driving toward goal");
        startMillis = millis();
        state = FWD;
      }
   
      break;
    
    case REV:
      motors_rev(); 
      flapservo.write(90); //halt the Servo when reversing to allow more current to get to the drive motors.
      //Halting the Servo also assists in allowing the robot to free itself from obstacles if the Impeller rotors are 
      //jammed up on them. The Servo by default spins forward which would have the tendency to pull the robot
      //further onto an obstacle. Halting the Servo gives it a better chance of escape.
          
      if ((unsigned long)(currentMillis - functionMillis) > rev_time)//reverse for rev_time, then enter the FWD_TURN state
      {
        randNum = random(1,3); //pass random number to the FWD_TURN
        functionMillis = millis(); //get current time, pass to FWD_TURN
        state = FWD_TURN;
      }
      break;
    
    case FWD_TURN:    
      flapservo.write(ServoSpeed); //re-engage the Servo
      //check if the side sensors are reading a wall nearby, then turn in the opposite direction
      if (dist_r < side_dist) 
      {
        motors_left();        
      }
      else if (dist_l < side_dist) 
      {
        motors_right();
      }
      else //if no walls detected, random turn
      {
        if (randNum == 1)
        {
        motors_right();     
        }
        else if (randNum == 2)
        {
          motors_left();   
        }
      }
      if((unsigned long)(currentMillis - functionMillis) > turn_time)//if the turn time is exceeded, re-enter the forward state
          {   
          startMillis = millis(); //get current time and pass to FWD
          state = FWD;
          }
      else if (BOT_SWITCH == LOW) 
          //obstacle or wall encountered, turn in reverse
          {
          Serial.println("Turn Interupted");
          randNum = random(1,3); //pass random number to REV_TURN
          functionMillis = millis(); //get current time, pass to REV_TURN
          state = REV_TURN;
          }
      else if (TOP_SWITCH == LOW)
          //Goal encountered, attempt to score
          {
          Serial.println("Forward turn score");
          functionMillis = millis(); //get current time, pass to SCORE
          state = SCORE;         
          }
      break;

    case REV_TURN:
      //decide which way to turn
        if (randNum == 1)
        {
        right_reverse();  
        }
        else if (randNum == 2)
        {
        left_reverse();
        }
      //turn for alotted time then re-enter FWD state  
      if((unsigned long)(currentMillis - functionMillis) > turn_time) 
          { 
          startMillis = millis(); //get current time, pass to FWD 
          state = FWD;
          }   
      break;

    case SCORE:
      //SCORE reverses the servo and the motors to eject the ball
        Serial.println("TOP SWITCH");
        flapservo.write(165); //reverse the Servo
        score(); //call the score function
          //remain in the Score function for rev_time
          if((unsigned long)(currentMillis - functionMillis) > rev_time)  
            {
            functionMillis = millis(); //get current time, pass to SCORE2
            state = SCORE2;  
            }
        break;

      case SCORE2:
      //SCORE2 calls the score_fwd function, which moves the robot forward 
      //while the impeller is still in reverse
      //to push the ball through the goal if it has not already gone through
        Serial.println("Push the ball");
        flapservo.write(125); //reverse the servo
        score_fwd(); 
        if((unsigned long)(currentMillis - functionMillis) > rev_time) 
            { 
            randNum = random(1,3); //get a random number, pass to RETREAT state  
            functionMillis = millis(); //get current time, pass to REV_TURN
            state = RETREAT;
            }
        else if ((BOT_SWITCH == LOW) || (TOP_SWITCH == LOW)) 
        //obstacle or wall encountered, turn in reverse. The robot has already attempted a goal
        //and should attempt to get away from it.
           {
           Serial.println("Interupt");
           randNum = random(1,3); //pass random number to REV_TURN
           functionMillis = millis(); //get current time, pass to REV_TURN
           state = REV_TURN; //reverse turn away from the goal
           }            
         break;
         
     case RETREAT:
      //The robot has already attempted a goal and should attempt to get away from it.
      //The drawback of utilizing a distance sensor to find the goal is that sometimes 
      //the robot will keep trying for a goal as it will continually read a suitable distance
      //when it is facing out the goal. This can result in it getting stuck, especially if it is 
      //in a spot of the arena where there is not enough traction. The RETREAT state is 
      //fairly reliable for getting the robot away from a goal after attempting to score.
      Serial.println("RETREAT");
        //using a random number, the robot will attempt to reverse turn either left or right
        if (randNum == 2)
          {
          right_reverse();
          }
          else if (randNum == 1)
          {
          left_reverse();
          }
        //After 1500ms have passed, the robot enters FWD_TURN. The random number just used is 
        //not changed, so the robot will turn forward in the opposite direction from its reverse turn.
        //This give the robot a better chance of not getting stuck facing a goal for too long.  
        if ((unsigned long)(currentMillis - functionMillis) > 1500)
          {
          functionMillis = millis(); //Get current time, pass to FWD_TURN
          state = FWD_TURN;
          }

          break;
                                   
   }
}

void motors_fwd()
{
 switch(function_state)
    {
      case 0:  
      //propel the robot forward
      Serial.println("forward");
      digitalWrite(motorA1,HIGH); 
      digitalWrite(motorA2,LOW);
      digitalWrite(motorB1,HIGH);
      digitalWrite(motorB2,LOW);
      analogWrite(ENA,95);
      analogWrite(ENB,95); 
    }
}

void motors_rev()
{
  switch(function_state)
    {
      case 0:
      //propel the robot in reverse
      Serial.println("reverse");
      digitalWrite(motorA1,LOW); 
      digitalWrite(motorA2,HIGH);
      digitalWrite(motorB1,LOW);
      digitalWrite(motorB2,HIGH);
      analogWrite(ENA,95);
      analogWrite(ENB,95); 

      break;
    }
}

void motors_right()
{
  switch(function_state)
      {
        case 0:
        //turn the robot right
        Serial.println ("turning right");
        digitalWrite(motorA1,HIGH); 
        digitalWrite(motorA2,LOW);
        digitalWrite(motorB1,HIGH);
        digitalWrite(motorB2,LOW);
        analogWrite(ENA,45);
        analogWrite(ENB,100);

        break;
      }
}

void motors_left()
{
  switch(function_state)
      {
        case 0:
        //turn the robot left
        Serial.println ("turning left");
        digitalWrite(motorA1,LOW); 
        digitalWrite(motorA2,HIGH);
        digitalWrite(motorB1,LOW);
        digitalWrite(motorB2,HIGH);
        analogWrite(ENA,100);
        analogWrite(ENB,45);

        break;          
      }
}

void right_reverse()
{
   switch(function_state)
      {
        case 0:
        //reverse turn right
        Serial.println("reverse right");
        digitalWrite(motorA1,HIGH); 
        digitalWrite(motorA2,LOW);
        digitalWrite(motorB1,HIGH);
        digitalWrite(motorB2,LOW);
        analogWrite(ENA,45);
        analogWrite(ENB,100);

        break;  
      }
}

void left_reverse()
{
  switch(function_state)
      {
        case 0:
        //reverse turn left
        Serial.println("reverse left");
        digitalWrite(motorA1,LOW); 
        digitalWrite(motorA2,HIGH);
        digitalWrite(motorB1,LOW);
        digitalWrite(motorB2,HIGH);
        analogWrite(ENA,100);
        analogWrite(ENB,45);

        break;  
      }  
}

void score()
{
  switch(function_state)
      {
        case 0:
        Serial.println("SCORE");
        //reverse the motors
        digitalWrite(motorA1,LOW); 
        digitalWrite(motorA2,HIGH);
        digitalWrite(motorB1,LOW);
        digitalWrite(motorB2,HIGH);
        analogWrite(ENA,85);
        analogWrite(ENB,85);  

        break;
      }
}

void score_fwd()
{
  switch(function_state)
      {
      case 0:
      //propel the robot forward at a slightly slower speed
      Serial.println("SCORE2");
      digitalWrite(motorA1,HIGH); 
      digitalWrite(motorA2,LOW);
      digitalWrite(motorB1,HIGH);
      digitalWrite(motorB2,LOW);
      analogWrite(ENA,85);
      analogWrite(ENB,85);

      break;
      } 
}

float front_sensor() //get distance reading from front sensor
{
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width; 
  float cm;
 
  digitalWrite(TRIG_1,HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_1,LOW);
  
  while(digitalRead(ECHO_1) == 0);
  t1 = micros();
  while(digitalRead(ECHO_1) == 1); 
  t2 = micros();
  pulse_width = t2 - t1;
  cm = pulse_width / 58.0;  
/*  Test front sensor output
  Serial.print("Front distance: ");
  Serial.print(cm);
  Serial.println(" cm"); 
*/   

  return cm;
}

float right_sensor() //get distance reading from right sensor
{
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width; 
  float cm;
 
  digitalWrite(TRIG_2,HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_2,LOW);
  
  while(digitalRead(ECHO_2) == 0);
  t1 = micros();
  while(digitalRead(ECHO_2) == 1); 
  t2 = micros();
  pulse_width = t2 - t1;
  cm = pulse_width / 58.0;  
/*Test right sensor output
  Serial.print("Right distance: ");
  Serial.print(cm);
  Serial.println(" cm"); 
*/   

  return cm;
}

float left_sensor() //get distance reading from left sensor
{
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width; 
  float cm;
 
  digitalWrite(TRIG_3,HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_3,LOW);
  
  while(digitalRead(ECHO_3) == 0);
  t1 = micros();
  while(digitalRead(ECHO_3) == 1); 
  t2 = micros();
  pulse_width = t2 - t1;
  cm = pulse_width / 58.0;  
/*Test left sensor output
  Serial.print("Left distance: ");
  Serial.print(cm);
  Serial.println(" cm"); 
*/

  return cm;
}
  
