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

#define NUM_TRACKS  4 

#define PIN_SERVO_TRACK_1     11
#define PIN_SERVO_TRACK_2     10
#define PIN_SERVO_TRACK_3     9
#define PIN_SERVO_TRACK_4     6  
#define PIN_RESET_BUTTON     0
#define PIN_IR_SENSOR_TRACK_1 2
#define PIN_IR_SENSOR_TRACK_2 3
#define PIN_IR_SENSOR_TRACK_3 4
#define PIN_IR_SENSOR_TRACK_4 5

#define PWM_LEFT_TOP     90
#define PWM_LEFT_MIDDLE  45
#define PWM_LEFT_START   0
#define PWM_RIGHT_TOP    90
#define PWM_RIGHT_MIDDLE 135
#define PWM_RIGHT_START  180


class Track
{
  protected:

    // Sensors and actuators
    Servo servo;
    int ir_state;

    // Data
    unsigned long trigger_time;
    byte triggered;

    // Configuration
    byte track_number;
    byte servo_start;
    byte servo_middle;
    byte servo_top;
    byte servo_pin;
    byte ir_sensor_pin;
    
  public:
    Track(byte track_number,
          byte ir_sensor_pin,
          byte servo_pin, 
          byte servo_start,
          byte servo_middle,
          byte servo_top
          ) {
      // Save Configuration
      this->track_number = track_number;
      this->ir_sensor_pin = ir_sensor_pin;
      this->servo_pin = servo_pin;
      this->servo_start = servo_start;
      this->servo_middle = servo_middle;
      this->servo_top = servo_top;
    }

    void initialize() {
      // Initialize IR Sensor and Servo
      pinMode(this->ir_sensor_pin, INPUT);
      digitalWrite(this->ir_sensor_pin, HIGH); // Turn on the pull-up
      this->servo.attach(this->servo_pin);
      this->triggered = 0;
    }

    void reset() {
      this->set_servo_start();
      this->ir_state = digitalRead(this->ir_sensor_pin);
      this->trigger_time = 0;
      this->triggered = 0;

          #ifdef DEBUG
      Serial.print("Track #"); Serial.print(this->track_number);
      Serial.println(" Reset");
      #endif
}

    void set_servo_top(){
      this->servo.write(this->servo_top);
      #ifdef DEBUG
      Serial.print("Track #"); Serial.print(this->track_number);
      Serial.println(" Servo Top");
      #endif

    }

    void set_servo_middle(){
      this->servo.write(this->servo_middle);
      #ifdef DEBUG
      Serial.print("Track #"); Serial.print(this->track_number);
      Serial.println(" Servo Middle");
      #endif
    }
    
    void set_servo_start(){
      this->servo.write(this->servo_start);
      #ifdef DEBUG
      Serial.print("Track #"); Serial.print(this->track_number);
      Serial.println(" Servo Start");
      #endif
    }

    unsigned long get_trigger_time() {
      return this->trigger_time;
    }

    byte was_triggered() {
      return this->triggered;
    }


    byte check_for_ir_trigger(unsigned long time_millis) {
      // save the time of the trigger
      byte old_ir_state = this->ir_state;
      this->ir_state = digitalRead(this->ir_sensor_pin);
      if (this->triggered) {
        #ifdef DEBUG
        Serial.print(" end ");
        #endif
       return 0;
      }
      #ifdef DEBUG
      Serial.print(" ");
      Serial.print(old_ir_state);
      Serial.print("/");
      Serial.print(this->ir_state);
      Serial.print("  ");
      #endif
      if (this->ir_state != old_ir_state && this->ir_state == LOW) {
        if ( 0 == this->trigger_time) {
          this->trigger_time = time_millis;
          #ifdef DEBUG
          Serial.print("Track #"); Serial.println(this->track_number);
          Serial.print("IR trigger: trigger time = ");
          Serial.println(this->trigger_time);
          Serial.print("IR trigger: old state = ");
          Serial.print(old_ir_state);
          Serial.print(" new state = ");
          Serial.println(this->ir_state);
          #endif
          this->triggered = 1;
          return 1;
        }
      }
      return 0;
    }
};

class FinishLine
{
  protected:
    Track* track[NUM_TRACKS];
    int rank[NUM_TRACKS] = {0};
    byte rank_index = 0;

    byte button_state;
    byte running;

  void run() {
    #ifdef DEBUG
    Serial.println("Run");
    #endif

    this->running=1;
  }

  void stop() {
    #ifdef DEBUG
    Serial.println("Stop");
    #endif

    this->running=0;
  }
  
  public:
    FinishLine() {
      track[0] = new Track(1, PIN_IR_SENSOR_TRACK_1, PIN_SERVO_TRACK_1, 
        PWM_LEFT_START, PWM_LEFT_MIDDLE, PWM_LEFT_TOP);
      track[1] = new Track(2, PIN_IR_SENSOR_TRACK_2, PIN_SERVO_TRACK_2, 
        PWM_LEFT_START, PWM_LEFT_MIDDLE, PWM_LEFT_TOP);
      track[2] = new Track(3, PIN_IR_SENSOR_TRACK_3, PIN_SERVO_TRACK_3, 
        PWM_RIGHT_START, PWM_RIGHT_MIDDLE, PWM_RIGHT_TOP);
      track[3] = new Track(4, PIN_IR_SENSOR_TRACK_4, PIN_SERVO_TRACK_4, 
        PWM_RIGHT_START, PWM_RIGHT_MIDDLE, PWM_RIGHT_TOP);
    }

    void initialize() {
      #ifdef DEBUG
      Serial.println("Initialize");
      #endif

      // Initialize Tracks
      for (int idx = 0; idx < NUM_TRACKS; idx++) {
        this->track[idx]->initialize();
      }

      // Initialize Button
      this->button_state = 0;
      pinMode(PIN_RESET_BUTTON, INPUT_PULLUP);
    
      // Initialize internal LED
      pinMode(LED_BUILTIN, OUTPUT);
  
      this->reset_race();
    }
  
    void reset_race() {
      #ifdef DEBUG
      Serial.println("Reset Race");
      #endif

      // Reset Tracks to starting conditions and reset our ranking.
      for(int idx = 0; idx < NUM_TRACKS; idx++) {
        this->track[idx]->reset();
        this->rank[idx] = NUM_TRACKS;
      }
      this->rank_index=0;

      // Turn of LED
      digitalWrite(LED_BUILTIN, LOW);
      
      // Save the running state
      this->run();
    }

    void start_race() {
      #ifdef DEBUG
      Serial.println("Start Race");
      #endif

      // Use Flags to indicate to the racers when to start
      delay(1000);

      // Set the starting flag for each track as a count-down
      for(int idx = 0; idx < NUM_TRACKS; idx++) {
        this->track[idx]->set_servo_top();
        
        // illuminate the LED for 100ms
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100); 
        digitalWrite(LED_BUILTIN, LOW);
        delay(900);
      }
      // Put all the flags back down again
      for(int idx = 0; idx < NUM_TRACKS; idx++) {
        this->track[idx]->set_servo_start();
      }

      // Turn on the LED
      digitalWrite(LED_BUILTIN, HIGH);
    }
    

    byte check_tracks(unsigned long time_millis) {
      if (!this->running) { return ; }
      byte any_trigger = 0;
      byte track_triggered = 0;

      #ifdef DEBUG
      Serial.print("IR READ:  ");
      #endif

      // Check to see if any of the tracks have triggered
      // if so, remember that and return that value. 
      for(int idx = 0; idx < NUM_TRACKS; idx++) {
        track_triggered = this->track[idx]->check_for_ir_trigger(time_millis);
        if (track_triggered) {
          this->rank[rank_index] = idx;
          this->rank_index++;
          
          // Determine if we won and raise our flag accordingly
          if(this->track[idx]->get_trigger_time() == 
             this->track[this->rank[0]]->get_trigger_time())
            {
              // If we have the same time as the first ranked track then we won (or tied)
              // This could cover if we are the winner, or if there's a tie.
              this->track[idx]->set_servo_top();
            }
            else {
              // we didn't win...
              this->track[idx]->set_servo_middle();
            }
        }
        any_trigger |= track_triggered;
      }

      #ifdef DEBUG
      Serial.println();
      #endif

      return any_trigger;
    }
    
    byte check_race_reset() {
      // check state of race reset button
      byte old_state = this->button_state;
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
    
    void check_if_done() {
      boolean done = true; 

      if (!this->running) { return ; }

      // check to see if they've all triggered.
      for(int idx = 0; idx < NUM_TRACKS; idx++) {
        done &= this->track[idx]->was_triggered();
      }
      if (done) {
        this->stop();

        #ifdef DEBUG
        boolean comma = false;
        Serial.println("Determining winnner:");
        Serial.print("  Ranks:"); 
        for(int idx = 0; idx < NUM_TRACKS; idx++) {
          if (comma) { Serial.print( ", "); }
          Serial.print( rank[idx]); 
          comma = true;
        }
        Serial.println();
        Serial.print("  Times:"); 
        comma = false;
        for(int idx = 0; idx < NUM_TRACKS; idx++) {
          if (comma) { Serial.print( ", "); }
          Serial.print( this->track[rank[idx]]->get_trigger_time()); 
          comma = true;
        }
        Serial.println();
        #endif

      }
    }

}; 

FinishLine controller;

void setup() {
  // put your setup code here, to run once:

  #ifdef DEBUG
  Serial.begin(9600);
  #endif

  controller.initialize();
}

void loop() {
  if ( controller.check_race_reset() ) {
    // Check to see if a race reset has been requested.  If so, do it.
    controller.reset_race();
    controller.start_race();
  }
  else {
    // otherwise, we're off to the races!
    unsigned long t = millis();
    if (controller.check_tracks(t)) {
      controller.check_if_done();
    }
  }
}
