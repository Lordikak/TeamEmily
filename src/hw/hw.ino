#include <PID_v1.h>

#include <Encoder.h>
#include <Arduino.h>
#include <Servo.h>


#define right_forward_pin 11
#define right_backward_pin 10
#define left_forward_pin 8
#define left_backward_pin 9

#define BUTTON_PIN 0

#define PADDLE_PIN 7


#define speed 200
#define wheel_setoff 8.2

//#define DEBUG
//#define DEBUG3
//#define DEBUG_ENC
//#define DEBUG2
//#define DEBUGOUTPUT
#define DEBUGINTERMEDIATE

//#define DEBUGPID

#define ENCODER_USE_INTERRUPTS

Encoder right_enc(3,2);
Encoder left_enc(19,18);

double setpointr, inputr, outputr;
double rp = 36, ri = .5, rd = 1; 
double setpointl, inputl, outputl;
double lp = 36, li = .5, ld = 1;

PID pid_right(&inputr, &outputr, &setpointr, rp, ri, rd, 0);
PID pid_left(&inputl, &outputl, &setpointl, lp, li, ld, 0);

static const float ticks_per_cm = 1833 / (6.7* PI); // TODO: measure this value
static const float steps_per_cm = 50.0; // TODO: measure this value

static const int in_loop_delay = 0;

const int homePosition = 90;
int servoPin = PADDLE_PIN;
static Servo paddleServo;

void initPaddleServo(int pin) {
    servoPin = pin;
    paddleServo.attach(servoPin);
    resetPaddle();
}

void movePaddle(float degrees) {
    // Clamp degrees to [0, 180] for most servos
    degrees = homePosition-90;
    degrees=constrain(degrees, 0, 180);
    paddleServo.write(degrees);
}

void resetPaddle() {
    movePaddle(0);
}

void reset_r_PID(){
  inputr = 0;
  outputr = 0;
  setpointr = 0;
  pid_right.SetMode(MANUAL);
  pid_right.SetMode(AUTOMATIC);
  pid_right.SetOutputLimits(-255., 255);
}

void reset_l_PID(){
  inputl = 0;
  outputl = 0;
  setpointl = 0;
  pid_left.SetMode(MANUAL);
  pid_left.SetMode(AUTOMATIC);
  pid_left.SetOutputLimits(-255., 255);
}

float ticks_to_cm(long count){
  return count / ticks_per_cm;
}
int cm_to_ticks(float cm){
  return cm * steps_per_cm;
}
/***
 * Rotates the bot around a point on the axis of the wheels.
 * @param degrees the degrees of the rotation in degrees
 * @param distanceOnAxis the distance in cm from the center between the wheels, positive is to the right
 * @param speed the speed in %
 * @endcond rotation is done
 * @author Team Emily
 */


void drive_differential (float left_distance, float right_distance, float s = 50) {
  reset_l_PID();
  reset_r_PID();

  pid_right.SetMode(AUTOMATIC);
  pid_left.SetMode(AUTOMATIC);

  Serial.print("Left Distance: ");
  Serial.print(left_distance);
  Serial.print(" cm, Right Distance: ");
  Serial.print(right_distance);
  Serial.println(" cm");


  float total_dist = max(abs(left_distance), abs(right_distance));
  float left_ratio = left_distance / total_dist;
  float right_ratio = right_distance / total_dist;

  left_enc.write(0);
  right_enc.write(0);



  for(float step = 0; step <= total_dist; step+= .003){ //0.006 to 0.003
    setpointl = (step * left_ratio);
    setpointr = (step * right_ratio);

    {
      #ifdef DEBUG3
      
      Serial.print("Setpoint Left: ");
      Serial.print(setpoint2);
      Serial.print(" cm, Setpoint Right: ");
      Serial.print(setpoint1);
      Serial.println(" cm");
      #endif
    }   

    inputl = ticks_to_cm(left_enc.read());
    inputr = ticks_to_cm(right_enc.read());

{
    #ifdef DEBUG2
    Serial.print("Input Left: ");
    Serial.print(inputl);
    Serial.print(" cm, Input Right: ");
    Serial.print(inputr);
    Serial.println(" cm");

    //Ticks
    Serial.print("Ticks Left: ");
    Serial.print(left_enc.read());
    Serial.print(", Ticks Right: ");
    Serial.print(right_enc.read());
    Serial.println();
    #endif
}
    pid_left.Compute();
    pid_right.Compute();

    #ifdef DEBUGOUTPUT
    Serial.print("Output Left: ");
    Serial.print(outputl);
    Serial.print(", Output Right: ");
    Serial.println(outputr);
    #endif

    #ifdef DEBUGPID
    Serial.print("PID Left in: ");
    Serial.print(inputl);
    Serial.print(", out: ");
    Serial.print(outputl);
    Serial.print(", setpoint: ");
    Serial.println(setpointl);
    Serial.print("PID Right in: ");
    Serial.print(inputr);
    Serial.print(", out: ");
    Serial.print(outputr);
    Serial.print(", setpoint: ");
    Serial.println(setpointr);
    #endif

    
    if(outputr > 0){
      analogWrite(right_forward_pin, constrain(abs(outputr), 0, 254)); // TODO: some translation
      analogWrite(right_backward_pin, 0);
    }else{
      analogWrite(right_backward_pin, constrain(abs(outputr), 0, 254));
      analogWrite(right_forward_pin, 0);
    }

    if(outputl > 0){
      analogWrite(left_forward_pin, constrain(abs(outputl), 0, 255)); // TODO: some translation
      analogWrite(left_backward_pin, 0);
    }else{
      analogWrite(left_backward_pin, constrain(abs(outputl), 0, 255));
      analogWrite(left_forward_pin, 0);  
    }

    delay(in_loop_delay);
  }
  setpointl = left_distance;
  setpointr = right_distance;

  #ifdef DEBUGINTERMEDIATE
  Serial.print("PID Left in: ");
  Serial.print(inputl);
  Serial.print(", out: ");
  Serial.print(outputl);
  Serial.print(", setpoint: ");
  Serial.println(setpointl);
  Serial.print("PID Right in: ");
  Serial.print(inputr);
  Serial.print(", out: ");
  Serial.print(outputr);
  Serial.print(", setpoint: ");
  Serial.println(setpointr);

  // Encoder values
  Serial.print("Ticks Left: ");
  Serial.print(left_enc.read());
  Serial.print(", Ticks Right: ");
  Serial.print(right_enc.read());
  Serial.println();
  #endif

  bool reached = true;
  unsigned long reached_time = 0;

  while (true) {
    inputl = ticks_to_cm(left_enc.read());
    inputr = ticks_to_cm(right_enc.read());

    pid_left.Compute();
    pid_right.Compute();

    if(outputr > 0){
      analogWrite(right_forward_pin, constrain(abs(outputr), 0, 255));
      analogWrite(right_backward_pin, 0);
    }else{
      analogWrite(right_backward_pin, constrain(abs(outputr), 0, 255));
      analogWrite(right_forward_pin, 0);
    }

    if(outputl > 0){
      analogWrite(left_forward_pin, constrain(abs(outputl), 0, 255));
      analogWrite(left_backward_pin, 0);
    }else{
      analogWrite(left_backward_pin, constrain(abs(outputl), 0, 255));
      analogWrite(left_forward_pin, 0);  
    }

    if (!reached && abs(inputl - setpointl) <= 0.1 && abs(inputr - setpointr) <= 0.1) {
      reached = true;
      reached_time = millis();
    }

    // If reached, check if 2 seconds have passed
    if (reached && (millis() - reached_time >= 5000)) {
      break;
    }

    delay(in_loop_delay);
  }
  
  analogWrite(left_forward_pin, 0);
  analogWrite(right_forward_pin, 255);
  analogWrite(left_backward_pin, 0);
  analogWrite(right_backward_pin, 255);

  #ifdef DEBUG
  Serial.println("Done Rotating");
  #endif

}

void rotate (float deg, float dist = 0, float s = 50) {
  

  float left_wheel_circ = (wheel_setoff + dist) * 2 * PI;
  float right_wheel_circ = (-wheel_setoff + dist) * 2 * PI; 
  float middle_circ = dist * 2 * PI; // maybe for later


  float left_dist = left_wheel_circ * (deg / 360);
  float right_dist = right_wheel_circ * (deg / 360);

  drive_differential(left_dist, right_dist, s);

}




void drive(float dist, float s = 50){
  drive_differential(dist, dist, s);
}




void setup() {
  // put your setup code here, to run once:
  pid_left.SetOutputLimits(-255, 255);
  pid_right.SetOutputLimits(-255, 255);


  Serial.begin(9600);

  Serial.println("Init done");

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  pinMode(right_forward_pin, OUTPUT);
  pinMode(right_backward_pin, OUTPUT);
  pinMode(left_forward_pin, OUTPUT);
  pinMode(left_backward_pin, OUTPUT);

  reset_l_PID();
  reset_r_PID();

  initPaddleServo(PADDLE_PIN);

  //delay(2000);
  //test_slalom();

}

void loop() {
  // put your main code here, to run repeatedly:
  test_drive_serial();

  //if (digitalRead(BUTTON_PIN) == LOW) {
  //  Serial.println("Button pressed! Executing slalom test...");
  //  test_slalom();
  //  Serial.println("Done slalom.");
  //  // Simple debounce
  //  delay(500);
  //}

#ifdef DEBUG_ENC
  int re = right_enc.read();
  int le = left_enc.read();
  Serial.print("Right Encoder: ");
  Serial.print(re);
  Serial.print(", Left Encoder: ");
  Serial.println(le);
  #endif
  //delay(100);
}

void test_rotate_serial() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    float dist = input.toFloat();
    if (dist != 0.0) {
      Serial.print("Rotating: ");
      Serial.println(dist);
      drive_differential(dist,dist );
      Serial.println("Done rotating.");
    }
    // print overshoot
    delay(2000);
    float distl = ticks_to_cm(left_enc.read());
    float distr = ticks_to_cm(right_enc.read());
    Serial.print("Left Distance: ");
    Serial.print(distl);
    Serial.print(" cm, Right Distance: ");
    Serial.print(distr);
    Serial.println(" cm");
  }
  
}

void test_drive_serial() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    // If input is 's' or 'S', run slalom test
    if (input.equalsIgnoreCase("s")) {
      Serial.println("Executing slalom test...");
      test_slalom();
      Serial.println("Done slalom.");
      return;
    }

    char mode = input.charAt(0);
    // Check for rotate with two parameters: e.g. "r90,20" or "r 90 20"
    if (mode == 'r' || mode == 'R') {
      // Remove the 'r' or 'R'
      String params = input.substring(1);
      params.trim();
      float angle = 0, dist = 0;
      int commaIdx = params.indexOf(',');
      int spaceIdx = params.indexOf(' ');
      if (commaIdx > 0) {
        angle = params.substring(0, commaIdx).toFloat();
        dist = params.substring(commaIdx + 1).toFloat();
      } else if (spaceIdx > 0) {
        angle = params.substring(0, spaceIdx).toFloat();
        dist = params.substring(spaceIdx + 1).toFloat();
      } else {
        angle = params.toFloat();
        dist = 0;
      }
      Serial.print("Rotating: ");
      Serial.print(angle);
      Serial.print(" deg, dist: ");
      Serial.println(dist);
      rotate(angle, dist);
      Serial.println("Done rotating.");
    } else if (mode == 'd' || mode == 'D') {
      // Default: drive (with or without 'd' prefix)
      float value;
      if (mode == 'd' || mode == 'D') {
        value = input.substring(1).toFloat();
      } else {
        value = input.toFloat();
      }
      Serial.print("Driving: ");
      Serial.println(value);
      drive(value);
      Serial.println("Done driving.");
    }else if (mode == 'p' || mode == 'P') {
      // Paddle control
      String paddleCommand = input.substring(1);
      paddleCommand.trim();
      if (paddleCommand.length() > 0) {
        float degrees = paddleCommand.toFloat();
        Serial.print("Moving paddle to: ");
        Serial.print(degrees);
        Serial.println(" degrees");
        movePaddle(degrees);
      } else {
        Serial.println("No paddle command provided.");
      }
    }
    
    else {
      Serial.println("Unknown command. Use 'r' for rotate or 'd' for drive.");
      return;
    }

    // print overshoot
    delay(2000);
    float distl = ticks_to_cm(left_enc.read());
    float distr = ticks_to_cm(right_enc.read());
    Serial.print("Left Distance: ");
    Serial.print(distl);
    Serial.print(" cm, Right Distance: ");
    Serial.print(distr);
    Serial.println(" cm");
  }
}

void test_slalom() {
  //drive(40);
  delay(2000);
  rotate(90, 20);
  // Print distance after first rotate
  Serial.print("After rotate(90, 20): L=");
  Serial.print(ticks_to_cm(left_enc.read()));
  Serial.print(" cm, R=");
  Serial.print(ticks_to_cm(right_enc.read()));
  Serial.println(" cm");

  rotate(-180, -20);
  Serial.print("After rotate(-180, -20): L=");
  Serial.print(ticks_to_cm(left_enc.read()));
  Serial.print(" cm, R=");
  Serial.print(ticks_to_cm(right_enc.read()));
  Serial.println(" cm");

  rotate(180, 20);
  Serial.print("After rotate(180, 20): L=");
  Serial.print(ticks_to_cm(left_enc.read()));
  Serial.print(" cm, R=");
  Serial.print(ticks_to_cm(right_enc.read()));
  Serial.println(" cm");

  rotate(-180, -20);
  Serial.print("After rotate(-180, -20): L=");
  Serial.print(ticks_to_cm(left_enc.read()));
  Serial.print(" cm, R=");
  Serial.print(ticks_to_cm(right_enc.read()));
  Serial.println(" cm");

  rotate(180, 20);
  Serial.print("After rotate(180, 20): L=");
  Serial.print(ticks_to_cm(left_enc.read()));
  Serial.print(" cm, R=");
  Serial.print(ticks_to_cm(right_enc.read()));
  Serial.println(" cm");
  
  rotate(-90, -20);
  Serial.print("After rotate(-90, -20): L=");
  Serial.print(ticks_to_cm(left_enc.read()));
  Serial.print(" cm, R=");
  Serial.print(ticks_to_cm(right_enc.read()));
  Serial.println(" cm");
  drive(40);
}
