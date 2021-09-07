/* Finish Line
 *  
 *  This is a program that will use IR LED Break beam sensors
 *  on two Matchbox Car tracks to identify the winner of a 
 *  race.  
 *  
 *  The circuit includes two Servo motors to raise a flag to
 *  identify which track hosts the winner of the race.
 *  
 *  There is also a button to reset the track for the next 
 *  set of racers. 
 *  
 *  Servos are on Digital Pins 10 and 11
 *  
 *  Race Reset button on Digital pin 0
 *  
 *  Break-beam IR sensors on Digital pins 2 and 3 configured
 *      as triggered inputs. 
 *      
 *  Future Considerations:
 *    - Light tree to indicate when it's time to go
 *    - Light to indicate winnner in addition to Servo
 *  
 */

#define DEBUG 1

#include <Servo.h>

#define PIN_SERVO_LANE_1     11
#define PIN_SERVO_LANE_2     10
#define PIN_SERVO_LANE_3     9
#define PIN_SERVO_LANE_4     6  
#define PIN_RESET_BUTTON     0
#define PIN_IR_SENSOR_LANE_1 5
#define PIN_IR_SENSOR_LANE_2 4
#define PIN_IR_SENSOR_LANE_3 3
#define PIN_IR_SENSOR_LANE_4 2

#define WINNER_NONE   0
#define WINNER_LANE_1 1
#define WINNER_LANE_2 2
#define WINNER_LANE_3 4
#define WINNER_LANE_4 8

#define PWM_LEFT_TOP     90
#define PWM_LEFT_MIDDLE  45
#define PWM_LEFT_START   0
#define PWM_RIGHT_TOP    90
#define PWM_RIGHT_MIDDLE 135
#define PWM_RIGHT_START  180

/* Switching to polling for 4 lanes.
void left_ir_interrupt();
void right_ir_interrupt();
*/
void poll_ir_sensors();

class FinishLine
{
  protected:
    Servo servo_lane_1;
    Servo servo_lane_2;
    Servo servo_lane_3;
    Servo servo_lane_4;

    char button_state;
    char running;
    int ir_state_lane_1;
    int ir_state_lane_2;
    int ir_state_lane_3;
    int ir_state_lane_4;

    unsigned long time_lane_1;
    unsigned long time_lane_2;
    unsigned long time_lane_3;
    unsigned long time_lane_4;

  void reset_timers() {
    #ifdef DEBUG
    Serial.println("Reset Timers");
    #endif

    this->time_lane_1 = 0;
    this->time_lane_2 = 0;
    this->time_lane_3 = 0;
    this->time_lane_4 = 0;
  }

  void attach_interrupts() {
    #ifdef DEBUG
    Serial.println("Attach Interrupts");
    #endif

    /* Switching to polling for 4 lanes.
    attachInterrupt(
      digitalPinToInterrupt(PIN_IR_SENSOR_LANE_1),
      left_ir_interrupt,
      CHANGE);
    attachInterrupt(
      digitalPinToInterrupt(PIN_IR_SENSOR_LANE_2),
      right_ir_interrupt,
      CHANGE);
    */
    this->running=1;
    this->ir_state_lane_1 = digitalRead(PIN_IR_SENSOR_LANE_1);
    this->ir_state_lane_2 = digitalRead(PIN_IR_SENSOR_LANE_2);
    this->ir_state_lane_3 = digitalRead(PIN_IR_SENSOR_LANE_3);
    this->ir_state_lane_4 = digitalRead(PIN_IR_SENSOR_LANE_4);
  }

  void detach_interrupts() {
    #ifdef DEBUG
    Serial.println("Detatch Interrupts");
    #endif

    /* Switching to polling for 4 lanes.
    detachInterrupt(digitalPinToInterrupt(PIN_IR_SENSOR_LANE_1));
    detachInterrupt(digitalPinToInterrupt(PIN_IR_SENSOR_LANE_2));
    */
    this->running=0;
  }
  
  public:
    FinishLine() {
      
    }

    void initialize() {
      #ifdef DEBUG
      Serial.println("Initialize");
      #endif

      // Intitialize Servos
      this->servo_lane_1.attach(PIN_SERVO_LANE_1);
      this->servo_lane_2.attach(PIN_SERVO_LANE_2);
      this->servo_lane_3.attach(PIN_SERVO_LANE_3);
      this->servo_lane_4.attach(PIN_SERVO_LANE_4);
  
      // Initialize Button
      this->button_state = 0;
      pinMode(PIN_RESET_BUTTON, INPUT_PULLUP);
  
      // Initialize triggers
      pinMode(PIN_IR_SENSOR_LANE_1, INPUT_PULLUP);
      pinMode(PIN_IR_SENSOR_LANE_2, INPUT_PULLUP);
      pinMode(PIN_IR_SENSOR_LANE_3, INPUT_PULLUP);
      pinMode(PIN_IR_SENSOR_LANE_4, INPUT_PULLUP);
  
      // Initialize internal LED
      pinMode(LED_BUILTIN, OUTPUT);
  
      this->reset_race();
    }
  

    void reset_race() {
      #ifdef DEBUG
      Serial.println("Reset Race");
      #endif

      // Move servos to reset position
      this->servo_lane_1.write(PWM_LEFT_START);
      this->servo_lane_2.write(PWM_LEFT_START);
      this->servo_lane_3.write(PWM_RIGHT_START);
      this->servo_lane_4.write(PWM_RIGHT_START);
      digitalWrite(LED_BUILTIN, LOW);
      
      // Reset IR triggers
      this->attach_interrupts();
      
      // Reset trigger timers
      this->reset_timers();
    }

    void start_race() {
      #ifdef DEBUG
      Serial.println("Start Race");
      #endif

      // Use Flags to indicate to the racers when to start
      delay(1000);
      this->servo_lane_1.write(PWM_LEFT_TOP);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100); 
      digitalWrite(LED_BUILTIN, LOW);

      delay(900);
      this->servo_lane_2.write(PWM_LEFT_TOP);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100); 
      digitalWrite(LED_BUILTIN, LOW);

      delay(900);
      this->servo_lane_3.write(PWM_RIGHT_TOP);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100); 
      digitalWrite(LED_BUILTIN, LOW);

      delay(900);
      this->servo_lane_4.write(PWM_RIGHT_TOP);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100); 
      digitalWrite(LED_BUILTIN, LOW);

      delay(900);
      this->servo_lane_1.write(PWM_LEFT_START);
      this->servo_lane_2.write(PWM_LEFT_START);
      this->servo_lane_3.write(PWM_RIGHT_START);
      this->servo_lane_4.write(PWM_RIGHT_START);
      digitalWrite(LED_BUILTIN, HIGH);

      this->reset_timers();
    }
    
    void announce_winner(int winner) {
      // Indicate the winner of the race
      // if there's a tie, then both flags raise
    
      if ( winner & WINNER_LANE_1 ) {
        // raise left flag
        this->servo_lane_1.write(PWM_LEFT_TOP);
      }
      else {
        this->servo_lane_1.write(PWM_LEFT_START);
      }
    
      if ( winner & WINNER_LANE_2 ) {
        // raise right flag
        this->servo_lane_2.write(PWM_LEFT_TOP);
      }
      else {
        this->servo_lane_2.write(PWM_LEFT_START);
      }

      if ( winner & WINNER_LANE_3 ) {
        // raise right flag
        this->servo_lane_3.write(PWM_RIGHT_TOP);
      }
      else {
        this->servo_lane_3.write(PWM_RIGHT_START);
      }
    
      if ( winner & WINNER_LANE_4 ) {
        // raise right flag
        this->servo_lane_4.write(PWM_RIGHT_TOP);
      }
      else {
        this->servo_lane_4.write(PWM_RIGHT_START);
      }    
    }
    
    void trigger_lane_1(unsigned long time_millis) {
      // save the time of the trigger
      if (!this->running) { return ; }
      int old_ir_state = this->ir_state_lane_1;
      this->ir_state_lane_1 = digitalRead(PIN_IR_SENSOR_LANE_1);
      if (this->ir_state_lane_1 != old_ir_state && this->ir_state_lane_1 == LOW) {
        if ( 0 == this->time_lane_1) {
          this->time_lane_1 = time_millis;
          this->servo_lane_1.write(PWM_LEFT_MIDDLE);
          #ifdef DEBUG
          Serial.print("Lane 1 IR trigger: trigger time = ");
          Serial.println(this->time_lane_1);
          Serial.print("Lane 1 IR trigger: old state = ");
          Serial.print(old_ir_state);
          Serial.print(" new state = ");
          Serial.println(this->ir_state_lane_1);
          #endif
        }
      }
    }
    void trigger_lane_2(unsigned long time_millis) {
      // save the time of the trigger
      if (!this->running) { return ; }
      int old_ir_state = this->ir_state_lane_2;
      this->ir_state_lane_2 = digitalRead(PIN_IR_SENSOR_LANE_2);
      if (this->ir_state_lane_2 != old_ir_state && this->ir_state_lane_2 == LOW) {
        if ( 0 == this->time_lane_2) {
          this->time_lane_2 = time_millis;
          this->servo_lane_2.write(PWM_LEFT_MIDDLE);
          #ifdef DEBUG
          Serial.print("Lane 2 IR trigger: trigger time = ");
          Serial.println(this->time_lane_2);
          Serial.print("Lane 2 IR trigger: old state = ");
          Serial.print(old_ir_state);
          Serial.print(" new state = ");
          Serial.println(this->ir_state_lane_2);
          #endif
        }
      }
    }
    void trigger_lane_3(unsigned long time_millis) {
      // save the time of the trigger
      if (!this->running) { return ; }
      int old_ir_state = this->ir_state_lane_3;
      this->ir_state_lane_3 = digitalRead(PIN_IR_SENSOR_LANE_3);
      if (this->ir_state_lane_3 != old_ir_state && this->ir_state_lane_3 == LOW) {
        if ( 0 == this->time_lane_3) {
          this->time_lane_3 = time_millis;
          this->servo_lane_3.write(PWM_RIGHT_MIDDLE);
          #ifdef DEBUG
          Serial.print("Lane 3 IR trigger: trigger time = ");
          Serial.println(this->time_lane_3);
          Serial.print("Lane 3 IR trigger: old state = ");
          Serial.print(old_ir_state);
          Serial.print(" new state = ");
          Serial.println(this->ir_state_lane_3);
          #endif
        }
      }
    }
    void trigger_lane_4(unsigned long time_millis) {
      // save the time of the trigger
      if (!this->running) { return ; }
      int old_ir_state = this->ir_state_lane_4;
      this->ir_state_lane_4 = digitalRead(PIN_IR_SENSOR_LANE_4);
      if (this->ir_state_lane_4 != old_ir_state && this->ir_state_lane_4 == LOW) {
        if ( 0 == this->time_lane_4) {
          this->time_lane_4 = time_millis;
          this->servo_lane_4.write(PWM_RIGHT_MIDDLE);
          #ifdef DEBUG
          Serial.print("Lane 4 IR trigger: trigger time = ");
          Serial.println(this->time_lane_4);
          Serial.print("Lane 4 IR trigger: old state = ");
          Serial.print(old_ir_state);
          Serial.print(" new state = ");
          Serial.println(this->ir_state_lane_4);
          #endif
        }
      }
    }
    
    int check_race_reset() {
      // check state of race reset button
      int old_state = this->button_state;
      this->button_state = digitalRead(PIN_RESET_BUTTON);

      if (old_state != this->button_state) {
        #ifdef DEBUG
        Serial.print("Button state change: ");
        Serial.print(old_state);
        Serial.print(" -> ");
        Serial.println(this->button_state);
        #endif

        return (LOW == this->button_state);
        }
      return 0;
    }
    
    void determine_winner() {
      // Compare the trigger times of the right and left
      // IR LED. Determine the winner based on the results.

      double p = 0.0;
      int winner =  WINNER_NONE;

      if ( 0 < this->time_lane_1 && 
           0 < this->time_lane_2 && 
           0 < this->time_lane_3 && 
           0 < this->time_lane_4 ) {
        this->detach_interrupts();

        int times[4] = { 
          this->time_lane_1, 
          this->time_lane_2,
          this->time_lane_3,
          this->time_lane_4
        };
        int rank[] = {0,0,0,0};
        int stage_1a[] = {0,0};
        int stage_1b[] = {0,0};
        int stage_4[] = {0,0};

        // Rank the lanes.  
        // 1. Compare 1/2 and 3/4.
        // 2. 1st = Compare the winners of 1/2 and 3/4
        // 3. 4th = Compare the loser of 1/2 and 3/4.
        // 4. 2nd/3rd = Compare remainging 2

        // stage 1a        
        if ( times[0] <= times[1]) {
          stage_1a[0] = 0;
          stage_1a[1] = 1;
        }
        else {
          stage_1a[0] = 1;
          stage_1a[1] = 0;
        }

        // stage 1b
        if ( times[2] <= times[3]) {
          stage_1b[0] = 2;
          stage_1b[1] = 3;
        }
        else {
          stage_1b[0] = 3;
          stage_1b[1] = 2;
        }

        // stage 2
        if ( times[stage_1a[0]] <= times[stage_1b[0]]) {
          rank[0] = stage_1a[0];
          stage_4[0] = stage_1b[0];
        }
        else {
          rank[0] = stage_1b[0];
          stage_4[0] = stage_1a[0];
        }

        // stage 3
        if ( times[stage_1a[1]] > times[stage_1b[1]]) {
          rank[3] = stage_1a[1];
          stage_4[1] = stage_1b[1];
        }
        else {
          rank[3] = stage_1b[1];
          stage_4[1] = stage_1a[1];
        }

        // stage 4
        if ( times[stage_4[0]] <= times[stage_4[1]]) {
          rank[1] = stage_4[0];
          rank[2] = stage_4[1];
        }
        else {
          rank[1] = stage_4[1];
          rank[2] = stage_4[0];
        }

        // Evaluate ranks
        // Ugh... rounding and floating points. adding 0.4 to help.
        winner = (int)(pow(2.0,rank[0])+0.4);
        // check to see if #2 tied
        if ( times[rank[0]] == times[rank[1]] ) {
          winner = winner + (int)(pow(2.0,rank[1])+0.4);
          // check to see if #3 tied (unlikely, but benefit of the doubt...)
          if (times[rank[0]] == times[rank[2]] ) {
            winner = winner + (int)(pow(2.0,rank[2])+0.4);
            // check to see if #4 tied too (really unlikely, but let's include anyway)
            if (times[rank[0]] == times[rank[3]] ) {
              winner = winner + (int)(pow(2.0,rank[3])+0.4);
            }
          }
        }


        #ifdef DEBUG
        Serial.println("Determining winnner:");
        Serial.print("  Ranks:"); 
        Serial.print( rank[0]); Serial.print( ", ");
        Serial.print( rank[1]); Serial.print( ", ");
        Serial.print( rank[2]); Serial.print( ", ");
        Serial.println( rank[3]);
        Serial.print("  Times:"); 
        Serial.print( times[rank[0]]); Serial.print( ", ");
        Serial.print( times[rank[1]]); Serial.print( ", ");
        Serial.print( times[rank[2]]); Serial.print( ", ");
        Serial.println( times[rank[3]]);
        Serial.print( "winner = "); Serial.println(winner);
        #endif

        this->announce_winner(winner);

        for ( int i = 0; i < 5; i++) {
          digitalWrite(LED_BUILTIN, HIGH);
          delay(100); 
          digitalWrite(LED_BUILTIN, LOW);
          delay(100); 
        }

        this->reset_timers();

      }
      // else, race isn't finished yet.
      
    }

}; 

FinishLine controller;

/*
void left_ir_interrupt() {
  // save the time of the trigger
  #ifdef DEBUG
  Serial.println("Left IR Interrupt");
  #endif

  controller.trigger_left();
}
    
void right_ir_interrupt() {
  #ifdef DEBUG
  Serial.println("Right IR Interrupt");
  #endif
  controller.trigger_right();
}
*/

void poll_ir_sensors(){
  unsigned long t = millis();
  controller.trigger_lane_1(t);
  controller.trigger_lane_2(t);
  controller.trigger_lane_3(t);
  controller.trigger_lane_4(t);
}


void setup() {
  // put your setup code here, to run once:

  #ifdef DEBUG
  Serial.begin(9600);
  #endif

  controller.initialize();

}

void loop() {
  // put your main code here, to run repeatedly:
  if ( controller.check_race_reset() ) {
    controller.reset_race();
    controller.start_race();
  }
  else {
    poll_ir_sensors();
    controller.determine_winner();
  }
}
