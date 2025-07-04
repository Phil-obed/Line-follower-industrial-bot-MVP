// #include <Arduino.h>
// #include <Servo.h>

// /* =====================  PIN MAP  =================================== */
// // IR sensors (active‑LOW)
// #define IR_L       8
// #define IR_C       9
// #define IR_R      10

// // Ultrasonics
// #define USL_TRIG  A1
// #define USL_ECHO  A0
// #define USR_TRIG  A3
// #define USR_ECHO  A2

// // L293D motor driver
// #define ENA      11      // left motor enable  (PWM)
// #define ENB       3      // right motor enable (PWM)
// #define IN1       4
// #define IN2       5
// #define IN3       6
// #define IN4       7

// // Servo (any free digital pin)
// #define SERVO_PIN 12

// /* =====================  BIT PATTERNS  ============================== */
// #define STOP    B0000
// #define FORWARD B1010
// #define REVERSE B0101
// #define LEFT    B1001
// #define RIGHT   B0110


// const uint8_t FAST = 150;   // straight‑line speed
// const uint8_t SLOW = 90;    // inside wheel on a turn


// class Bot {
// private:
//     uint8_t motorPins[4] = {IN1, IN2, IN3, IN4};
//     uint8_t _current_Speed = 0;
//     uint8_t _target_Speed  = 0;
//     uint8_t _ramp_Step     = 5;
//     uint8_t _ramp_Interval = 30;
//     unsigned long _last_UpdatedT = 0;
//     uint8_t _current_Pattern = STOP;
//     /* ‑‑‑‑‑‑‑‑‑‑‑ fixed mapping ‑‑‑‑‑‑‑‑‑‑‑ */
//     const uint8_t _enL = ENA;  // left enable  -> pin 11
//     const uint8_t _enR = ENB;  // right enable -> pin 3
//     Servo _servo;

// public:
//     Bot();
//     void setSpeed(uint8_t s);
//     void updateSpeed();
//     void move(uint8_t pattern);
//     uint8_t readLineBits();
//     void check_line();
// };

// Bot::Bot() {
//     for (uint8_t p : motorPins) pinMode(p, OUTPUT);
//     pinMode(_enL, OUTPUT);
//     pinMode(_enR, OUTPUT);

//     pinMode(IR_L, INPUT_PULLUP);
//     pinMode(IR_C, INPUT_PULLUP);
//     pinMode(IR_R, INPUT_PULLUP);

//     pinMode(USL_TRIG, OUTPUT); pinMode(USL_ECHO, INPUT);
//     pinMode(USR_TRIG, OUTPUT); pinMode(USR_ECHO, INPUT);

//     _servo.attach(SERVO_PIN);
//     _servo.write(0);
// }

// void Bot::setSpeed(uint8_t s) {
//     _target_Speed = constrain(s, 0, 255);
// }

// void Bot::updateSpeed() {
//     unsigned long now = millis();
//     if (now - _last_UpdatedT < _ramp_Interval) return;
//     _last_UpdatedT = now;

//     if (_current_Speed < _target_Speed)
//     _current_Speed = constrain(_current_Speed + _ramp_Step, 0, _target_Speed);
// else if (_current_Speed > _target_Speed)
//     _current_Speed = constrain(_current_Speed - _ramp_Step, _target_Speed, 255);


//     analogWrite(_enL, _current_Speed);   // left PWM  (pin 11)
//     analogWrite(_enR, _current_Speed);   // right PWM (pin 3)
// }

// void Bot::move(uint8_t pattern) {
//     uint8_t leftPwm  = 0;
//     uint8_t rightPwm = 0;

//     switch (pattern) {
//         case FORWARD:
//             leftPwm  = FAST;
//             rightPwm = FAST;
//             digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);    // L fwd
//             digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);    // R fwd
//             break;

//         case LEFT:   // gentle left: slow left wheel
//             leftPwm  = SLOW;
//             rightPwm = FAST;
//             digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);    // L fwd
//             digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);    // R fwd
//             break;

//         case RIGHT:  // gentle right: slow right wheel
//             leftPwm  = FAST;
//             rightPwm = SLOW;
//             digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);    // L fwd
//             digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);    // R fwd
//             break;

//         default:     // STOP
//             leftPwm  = 0;
//             rightPwm = 0;
//             digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);
//             digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);
//             break;
//     }

//     analogWrite(ENA, leftPwm);   // left enable  (pin 11)
//     analogWrite(ENB, rightPwm);  // right enable (pin 3)
// }



// uint8_t Bot::readLineBits() {
//     return (digitalRead(IR_L) << 2) |
//            (digitalRead(IR_C) << 1) |
//             digitalRead(IR_R);
// }

// void Bot::check_line() {
//     switch (readLineBits()) {
//         case 0b010:  // Center only — go straight
//             move(FORWARD);
//             break;

//         case 0b100:  // Left edge detected — correct right
//         case 0b110:
//             move(LEFT);         // Slow right wheel
//             delay(300);          // Brief correction
//             move(FORWARD);       // Back to normal
//             break;

//         case 0b001:  // Right edge detected — correct left
//         case 0b011:
//             move(RIGHT);          
//             delay(300);
//             move(FORWARD);
//             break;

//         case 0b000:  // All black — junction maybe
//         case 0b111:  // All white — off the line
//         case 0b101:  // Left + right only — undefined/confused
//         default:
//             move(STOP);
//             break;
//     }
// }



// /* =====================  MAIN ====================================== */
// Bot robot;

// void setup() {
//     Serial.begin(9600);
//     robot.setSpeed(120);
// }

// void loop() {
//     robot.updateSpeed();
//     robot.check_line();
// }













































#include <Arduino.h>
#include <Servo.h>

/* =====================  PIN MAP  =================================== */
// IR sensors (active‑LOW)
#define IR_L       8
#define IR_C       9
#define IR_R      10

// Motor driver (L293D)
#define ENA      11      // left motor enable  (PWM)
#define ENB       3      // right motor enable (PWM)
#define IN1       4
#define IN2       5
#define IN3       6
#define IN4       7

// Servo (optional – not used here, but initialized)
#define SERVO_PIN 12

/* =====================  MOTION CONFIG ============================== */
const uint8_t FAST = 150;   // normal forward speed
const uint8_t SLOW = 90;    // inside wheel during gentle turn
const uint16_t CORRECTION_TIME = 200; // ms correction duration

/* =====================  CLASS ===================================== */
class Bot {
private:
    Servo _servo;

public:
    Bot();
    void moveForward();
    void stop();
    void moveWithOffset(uint8_t leftPWM, uint8_t rightPWM, uint16_t duration = CORRECTION_TIME);
    void check_line();
    uint8_t readLineBits();
};

Bot::Bot() {
    pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

    pinMode(IR_L, INPUT_PULLUP);
    pinMode(IR_C, INPUT_PULLUP);
    pinMode(IR_R, INPUT_PULLUP);

    _servo.attach(SERVO_PIN);
    _servo.write(0);
}

void Bot::moveForward() {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  // Left forward
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  // Right forward
    analogWrite(ENA, FAST);
    analogWrite(ENB, FAST);
}

void Bot::stop() {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}

void Bot::moveWithOffset(uint8_t leftPWM, uint8_t rightPWM, uint16_t duration) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  // Left forward
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  // Right forward

    analogWrite(ENA, leftPWM);
    analogWrite(ENB, rightPWM);

    delay(duration); // short correction window

    analogWrite(ENA, FAST);  // back to full speed
    analogWrite(ENB, FAST);
}

uint8_t Bot::readLineBits() {
    return (digitalRead(IR_L) << 2) |
           (digitalRead(IR_C) << 1) |
           (digitalRead(IR_R));
}

void Bot::check_line() {
    switch (readLineBits()) {
        case 0b010:  // Only center sees white
            moveForward();
            break;

        case 0b100:  // Left only sees white
        case 0b110:  // Left + center white
            moveWithOffset(SLOW, FAST);  // gentle right turn
            break;

        case 0b001:  // Right only sees white
        case 0b011:  // Right + center white
            moveWithOffset(FAST, SLOW);  // gentle left turn
            break;

        case 0b000:  // All black
        case 0b111:  // All white
        default:
            stop();  // uncertain or lost — stop
            break;
    }
}

/* =====================  MAIN ====================================== */
Bot robot;

void setup() {
    Serial.begin(9600);
    robot.stop();
}

void loop() {
    robot.check_line();
}
