#include <Arduino.h>
#include <Encoder.h>
#include <PID_v1.h>
#include <Servo.h>
#include <VL53L0X.h>

#define right_forward_pin 11
#define right_backward_pin 10
#define left_forward_pin 8
#define left_backward_pin 9

#define PADDLE_PIN 7

#define BUTTON_SPORT_MODE 1  // D1
#define BUTTON_LAUNCH 0      // D0

#define LED_RUNNING 13
#define LED_SPORT_FOOTBALL 4
#define LED_SPORT_GOLF 5
#define LED_SPORT_SLALOM 6

#define XSHUT1 16 // D16 für Sensor 1
#define XSHUT2 17 // D17 für Sensor 2

const int sportLEDs[] = {
    LED_SPORT_FOOTBALL,
    LED_SPORT_GOLF,
    LED_SPORT_SLALOM};
const int numModes = sizeof(sportLEDs) / sizeof(sportLEDs[0]);

int state = 1;  // Aktueller Modus (0=football, 1=golf, 2=slalom)

bool lastModeButton = HIGH;
bool lastLaunchButton = HIGH;

#define speed 200
#define wheel_setoff 8.2

// #define DEBUG
// #define DEBUG3
// #define DEBUG_ENC
// #define DEBUG2
// #define DEBUGOUTPUT
#define DEBUGINTERMEDIATE
// #define COMMAND

// #define DEBUGPID

#define ENCODER_USE_INTERRUPTS

Encoder right_enc(3, 2);
Encoder left_enc(19, 18);

VL53L0X sensor1;
VL53L0X sensor2;


double setpointr, inputr, outputr;
double rp = 36, ri = 2, rd = 1;
double setpointl, inputl, outputl;
double lp = 36, li = 2, ld = 1;

PID pid_right(&inputr, &outputr, &setpointr, rp, ri, rd, 0);
PID pid_left(&inputl, &outputl, &setpointl, lp, li, ld, 0);

static const float ticks_per_cm = 1833 / (6.7 * PI);  // TODO: measure this value
static const float steps_per_cm = 50.0;               // TODO: measure this value

static const int in_loop_delay = 0;

// const int homePosition = 94; // aussen max 20p
const int homePosition = 123;  // innen max 42
int servoPin = PADDLE_PIN;
static Servo paddleServo;

void initPaddleServo(int pin) {
    servoPin = pin;
    paddleServo.attach(servoPin);
    resetPaddle();
}

void movePaddle(float degrees) {
    // Clamp degrees to [0, 180] for most servos
    degrees = homePosition - degrees;
    degrees = constrain(degrees, 0, 180);
    Serial.println(degrees);
    paddleServo.write(degrees);
}

void resetPaddle() {
    movePaddle(0);
}

void reset_r_PID() {
    inputr = 0;
    outputr = 0;
    setpointr = 0;
    pid_right.SetMode(MANUAL);
    pid_right.SetMode(AUTOMATIC);
    pid_right.SetOutputLimits(-255., 255);
}

void reset_l_PID() {
    inputl = 0;
    outputl = 0;
    setpointl = 0;
    pid_left.SetMode(MANUAL);
    pid_left.SetMode(AUTOMATIC);
    pid_left.SetOutputLimits(-255., 255);
}

float ticks_to_cm(long count) {
    return count / ticks_per_cm;
}
int cm_to_ticks(float cm) {
    return cm * steps_per_cm;
}

static float cor_time = 0;
/***
 * Rotates the bot around a point on the axis of the wheels.
 * @param degrees the degrees of the rotation in degrees
 * @param distanceOnAxis the distance in cm from the center between the wheels, positive is to the right
 * @param speed the speed in %
 * @endcond rotation is done
 * @author Team Emily
 */


void drive_differential(float left_distance, float right_distance, float s = 50, bool wait_for_ball = false, float threshold = 10) {
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

    float increment = .006 /100* s;

    for (float step = 0; step <= total_dist; step += increment) {  // 0.006 to 0.003
        setpointl = (step * left_ratio);
        setpointr = (step * right_ratio);

        if (wait_for_ball && ( (int) (step/increment) ) % 80 == 0) {
            if (downDistance() < threshold) {
                break;
            }
        }

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

            // Ticks
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

        if (outputr > 0) {
            analogWrite(right_forward_pin, constrain(abs(outputr), 0, 254));  // TODO: some translation
            analogWrite(right_backward_pin, 0);
        } else {
            analogWrite(right_backward_pin, constrain(abs(outputr), 0, 254));
            analogWrite(right_forward_pin, 0);
        }

        if (outputl > 0) {
            analogWrite(left_forward_pin, constrain(abs(outputl), 0, 255));  // TODO: some translation
            analogWrite(left_backward_pin, 0);
        } else {
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
    unsigned long reached_time = millis();

    while (true) {
        if (reached && (millis() - reached_time >= cor_time)) {
            break;
        }
        inputl = ticks_to_cm(left_enc.read());
        inputr = ticks_to_cm(right_enc.read());

        pid_left.Compute();
        pid_right.Compute();

        if (outputr > 0) {
            analogWrite(right_forward_pin, constrain(abs(outputr), 0, 255));
            analogWrite(right_backward_pin, 0);
        } else {
            analogWrite(right_backward_pin, constrain(abs(outputr), 0, 255));
            analogWrite(right_forward_pin, 0);
        }

        if (outputl > 0) {
            analogWrite(left_forward_pin, constrain(abs(outputl), 0, 255));
            analogWrite(left_backward_pin, 0);
        } else {
            analogWrite(left_backward_pin, constrain(abs(outputl), 0, 255));
            analogWrite(left_forward_pin, 0);
        }

        if (!reached && abs(inputl - setpointl) <= 0.1 && abs(inputr - setpointr) <= 0.1) {
            reached = true;
            reached_time = millis();
        }

        // If reached, check if 2 seconds have passed
        

        delay(in_loop_delay);
    }


    analogWrite(left_forward_pin, 0);
    analogWrite(right_forward_pin, 0);
    analogWrite(left_backward_pin, 0);
    analogWrite(right_backward_pin, 0);

#ifdef DEBUG
    Serial.println("Done Rotating");
#endif
}

void rotate(float deg, float dist = 0, float s = 50) {
    float left_wheel_circ = (wheel_setoff + dist) * 2 * PI;
    float right_wheel_circ = (-wheel_setoff + dist) * 2 * PI;
    float middle_circ = dist * 2 * PI;  // maybe for later

    float left_dist = left_wheel_circ * (deg / 360);
    float right_dist = right_wheel_circ * (deg / 360);

    drive_differential(left_dist, right_dist, s);
}

float forwardDistance(){
    int dist = sensor2.readRangeSingleMillimeters();
    // conv to cm
    float fdist = ((float) dist) / 10.0;
    return fdist-5.2;
}

float downDistance(){
    int dist = sensor1.readRangeSingleMillimeters();
    // conv to cm
    float fdist = ((float) dist) / 10.0;
    return fdist;
}

void setupSensor(){
  Wire.begin();

  pinMode(XSHUT1, OUTPUT);
  pinMode(XSHUT2, OUTPUT);

  digitalWrite(XSHUT1, LOW);
  digitalWrite(XSHUT2, LOW);
  delay(10);

  // Sensor 1 aktivieren und Adresse setzen
  digitalWrite(XSHUT1, HIGH);
  delay(10);
  sensor1.init();
  sensor1.setAddress(0x30);  // neue Adresse für Sensor 1

    // Sensor 2 aktivieren und Adresse setzen

  digitalWrite(XSHUT2, HIGH);
  delay(10);
  sensor2.init();
  sensor2.setAddress(0x31);  // neue Adresse für Sensor 2

 // Sensoren starten
  sensor1.setTimeout(500);
  sensor2.setTimeout(500);


}

void drive(float dist, float s = 50) {
    drive_differential(dist, dist, s);
}

void t(bool drive = false, int driveSpeed = 0) {
    
    //delay(700);

    if(drive){
        analogWrite(left_backward_pin, 0);
        analogWrite(right_backward_pin, 0);
        analogWrite(left_forward_pin,driveSpeed);
        analogWrite(right_forward_pin,driveSpeed);
    }
    movePaddle(90);
    delay(230);
    movePaddle(37);
    delay(200);
    movePaddle(0);
    if(drive){
        analogWrite(left_forward_pin, 0);
        analogWrite(right_forward_pin, 0);
        analogWrite(left_backward_pin, 0);
        analogWrite(right_backward_pin, 0);
    }
}

void setup() {
    // put your setup code here, to run once:
    pid_left.SetOutputLimits(-255, 255);
    pid_right.SetOutputLimits(-255, 255);

    Serial.begin(9600);

    

    pinMode(right_forward_pin, OUTPUT);
    pinMode(right_backward_pin, OUTPUT);
    pinMode(left_forward_pin, OUTPUT);
    pinMode(left_backward_pin, OUTPUT);

    reset_l_PID();
    reset_r_PID();

    initPaddleServo(PADDLE_PIN);

    // delay(2000);
    // test_slalom();

    pinMode(BUTTON_SPORT_MODE, INPUT_PULLUP);
    pinMode(BUTTON_LAUNCH, INPUT_PULLUP);

    pinMode(LED_RUNNING, OUTPUT);
    pinMode(LED_SPORT_FOOTBALL, OUTPUT);
    pinMode(LED_SPORT_GOLF, OUTPUT);
    pinMode(LED_SPORT_SLALOM, OUTPUT);

    updateLEDs();
    Serial.println("Setting up Sensor");
    setupSensor();
    Serial.println("Init done");
}

void updateLEDs() {
    for (int i = 0; i < numModes; i++) {
        digitalWrite(sportLEDs[i], (i == state) ? HIGH : LOW);
    }
}

//#define COMMAND

void loop() {
    //Serial.print("TEST");
// put your main code here, to run repeatedly:
#ifdef COMMAND
    test_drive_serial();
#else

    bool modeButton = digitalRead(BUTTON_SPORT_MODE);
    if (modeButton == LOW && lastModeButton == HIGH) {
        state = (state + 1) % numModes;
        Serial.print("Modus gewechselt zu: ");
        Serial.println(state);
        updateLEDs();
        delay(200);  // Entprellen
    }
    lastModeButton = modeButton;

    // --- Startknopf (D0) ---
    bool launchButton = digitalRead(BUTTON_LAUNCH);
    if (launchButton == LOW && lastLaunchButton == HIGH) {
        Serial.print("Starte Modus: ");
        Serial.println(state);

        digitalWrite(LED_RUNNING, HIGH);  // AN während Sport läuft

        switch (state) {
            case 0:
                startFootball();
                break;
            case 1:
                startGolf();
                break;
            case 2:
                startSlalom();
                break;
        }

        digitalWrite(LED_RUNNING, LOW);  // AUS nach Ende
        delay(200);                      // Entprellen
    }
    lastLaunchButton = launchButton;

#endif

    // if (digitalRead(BUTTON_PIN) == LOW) {
    //   Serial.println("Button pressed! Executing slalom test...");
    //   test_slalom();
    //   Serial.println("Done slalom.");
    //   // Simple debounce
    //   delay(500);
    // }

#ifdef DEBUG_ENC
    int re = right_enc.read();
    int le = left_enc.read();
    Serial.print("Right Encoder: ");
    Serial.print(re);
    Serial.print(", Left Encoder: ");
    Serial.println(le);
#endif
    // delay(100);
}

void test_rotate_serial() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        float dist = input.toFloat();
        if (dist != 0.0) {
            Serial.print("Rotating: ");
            Serial.println(dist);
            drive_differential(dist, dist);
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
        } else if (mode == 'p' || mode == 'P') {
            // Paddle control
            //delay(10000);
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
        } else if (mode == 't') {
            // analogWrite(left_forward_pin, 255);
            // analogWrite(right_forward_pin, 255);
            t();
            // analogWrite(left_forward_pin, 0);
            // analogWrite(right_forward_pin, 0);
        }else if (mode == 'f'){
            startFootball();
        } else if (mode == 'o'){
          float dist = forwardDistance();
          Serial.print("Distance Front: ");
          Serial.print(dist);
          Serial.println();
        } else if (mode == 'b'){
          float dist = downDistance();
          Serial.print("Distance bottom: ");
          Serial.print(dist);
          Serial.println();
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
    // drive(40);

    float s = 70;
    float r = 20.5;

    float first_dist = 8;
    delay(2000);

    drive(r+first_dist,s);
    cor_time = 500;
    rotate(-90,0,s);
    cor_time = 0;

    rotate(180, r,s);
    // Print distance after first rotate
    Serial.print("After rotate(90, 20): L=");
    Serial.print(ticks_to_cm(left_enc.read()));
    Serial.print(" cm, R=");
    Serial.print(ticks_to_cm(right_enc.read()));
    Serial.println(" cm");

    rotate(-180, -r,s);
    Serial.print("After rotate(-180, -20): L=");
    Serial.print(ticks_to_cm(left_enc.read()));
    Serial.print(" cm, R=");
    Serial.print(ticks_to_cm(right_enc.read()));
    Serial.println(" cm");

    rotate(180, r,s);
    Serial.print("After rotate(180, 20): L=");
    Serial.print(ticks_to_cm(left_enc.read()));
    Serial.print(" cm, R=");
    Serial.print(ticks_to_cm(right_enc.read()));
    Serial.println(" cm");

    rotate(-180, -r,s);
    Serial.print("After rotate(-180, -20): L=");
    Serial.print(ticks_to_cm(left_enc.read()));
    Serial.print(" cm, R=");
    Serial.print(ticks_to_cm(right_enc.read()));
    Serial.println(" cm");

    rotate(100, r,s);
    Serial.print("After rotate(180, 20): L=");
    Serial.print(ticks_to_cm(left_enc.read()));
    Serial.print(" cm, R=");
    Serial.print(ticks_to_cm(right_enc.read()));
    Serial.println(" cm");


    drive(4*r,s);
}

void startFootball(){
    cor_time = 700;
    float threashold = 45.0;
    float fdist = forwardDistance();
    Serial.print("Distance Front: ");
    Serial.print(fdist);
    Serial.println(" cm");
    if(fdist > threashold && false) {
        Serial.println("Shooting straight");
        t();
        cor_time=0;
        return;
    }

    float threashold2 = 45.0;
    float deg = 12;
    float extra = 4;
    rotate(deg, -wheel_setoff,40);
    delay(200);
    fdist = forwardDistance();
    Serial.print("Distance Front after rotate: ");
    Serial.print(fdist);
    Serial.println(" cm");
    if (fdist > threashold2) {
        Serial.println("Shooting right");
        rotate(extra, -wheel_setoff, 40);
        t(true, 100);
        cor_time = 0;
        return;
    }

    rotate(-2*deg-extra, -wheel_setoff, 40);
    Serial.println("Shooting left");
    t(true, 100);
    cor_time=0;
}
void startGolf() {
    Serial.println("🏌️ Starte Golf-Modus");
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_SPORT_GOLF, LOW);
        delay(300);
        digitalWrite(LED_SPORT_GOLF, HIGH);
        delay(300);
    }
    
    // while (downDistance() > 4.8) {
    //     float distance = 2.0;
    //     int super_speed = 5;
    //     drive_differential(distance, distance, super_speed);
    // }
    // drive_differential(0, 0, 10);
    // delay(30);
    // t();

    float distance = 15.0;
    drive_differential(distance, distance, 20, true, 10);
    
    delay(30);
    t(true, 50);
    

}

void startSlalom() {
    Serial.println("⛷️ Starte Slalom-Modus");
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_SPORT_SLALOM, LOW);
        delay(300);
        digitalWrite(LED_SPORT_SLALOM, HIGH);
        delay(300);
    }
    test_slalom();
}


