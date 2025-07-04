#include <Arduino.h>
#include <Servo.h>

/* =====================  PIN MAP  =================================== */
// IR sensors
#define IR_L   6
#define IR_C   7
#define IR_R   8

// Ultrasonics
#define USL_TRIG 12
#define USL_ECHO 13
#define USR_TRIG 14
#define USR_ECHO 15

// L293D motor driver (PWM-compatible)
#define ENA 10     // PWM slice 5A
#define IN1 2
#define IN2 3
#define ENB 11     // PWM slice 5B
#define IN3 4
#define IN4 5

// Servo
#define SERVO_PIN 16    // PWM slice 0A

/* =====================  BIT PATTERNS  ============================== */
#define STOP    B0000
#define FORWARD B1010
#define REVERSE B0101
#define LEFT    B1001
#define RIGHT   B0110

class Bot {
private:
    uint8_t _motors = 2;
    uint8_t _speed;
    uint8_t motorPins[4];
    uint8_t _current_Speed = 0;
    uint8_t _target_Speed = 0;
    uint8_t _ramp_Step = 5;
    uint8_t _ramp_Interval = 30;
    unsigned long _last_UpdatedT = 0;
    uint8_t _current_Pattern = STOP;
    uint8_t _enR = ENA;
    uint8_t _enL = ENB;
    Servo _servo;
    uint8_t _servo_Target = 0;

public:
    Bot();
    void check_collision();
    bool obstacleAhead(uint16_t threshold = 300);
    void updateSpeed();
    void check_line();
    void offload(bool drop);
    void move(uint8_t pattern);
    void setSpeed(uint8_t speed);
    uint8_t readLineBits();
};

Bot::Bot() {
    // Motor pins (bit order)
    motorPins[0] = IN1;
    motorPins[1] = IN2;
    motorPins[2] = IN3;
    motorPins[3] = IN4;

    for (uint8_t i = 0; i < 4; i++) pinMode(motorPins[i], OUTPUT);
    pinMode(_enL, OUTPUT);
    pinMode(_enR, OUTPUT);

    // IRs
    pinMode(IR_L, INPUT_PULLUP);
    pinMode(IR_C, INPUT_PULLUP);
    pinMode(IR_R, INPUT_PULLUP);

    // Ultrasound
    pinMode(USL_TRIG, OUTPUT);
    pinMode(USL_ECHO, INPUT);
    pinMode(USR_TRIG, OUTPUT);
    pinMode(USR_ECHO, INPUT);

    _servo.attach(SERVO_PIN);
    _servo.write(0);
}

void Bot::setSpeed(uint8_t speed) {
    _target_Speed = constrain(speed, 0, 255);
}

void Bot::move(uint8_t pattern) {
    _current_Pattern = pattern;
    for (uint8_t i = 0; i < 4; i++) {
        bool state = (_current_Pattern >> (3 - i)) & 0x01;
        digitalWrite(motorPins[i], state ? HIGH : LOW);
    }
    analogWrite(_enL, _current_Speed);
    analogWrite(_enR, _current_Speed);
}

void Bot::updateSpeed() {
    unsigned long now = millis();
    if (now - _last_UpdatedT >= _ramp_Interval) {
        _last_UpdatedT = now;

        if (_current_Speed < _target_Speed) {
            _current_Speed += _ramp_Step;
            if (_current_Speed > _target_Speed) _current_Speed = _target_Speed;
        } else if (_current_Speed > _target_Speed) {
            _current_Speed -= _ramp_Step;
            if (_current_Speed < _target_Speed) _current_Speed = _target_Speed;
        }

        analogWrite(_enL, _current_Speed);
        analogWrite(_enR, _current_Speed);
    }
}

bool Bot::obstacleAhead(uint16_t threshold) {
    auto readUS = [](uint8_t trig, uint8_t echo) -> uint16_t {
        digitalWrite(trig, LOW);
        delayMicroseconds(2);
        digitalWrite(trig, HIGH);
        delayMicroseconds(10);
        digitalWrite(trig, LOW);
        unsigned long duration = pulseIn(echo, HIGH, 25000UL);
        return duration / 58;  // microseconds to mm
    };

    uint16_t left = readUS(USL_TRIG, USL_ECHO);
    uint16_t right = readUS(USR_TRIG, USR_ECHO);
    return (left && left < threshold) || (right && right < threshold);
}

void Bot::check_collision() {
    if (obstacleAhead()) {
        move(STOP);
        setSpeed(0);
    }
}

uint8_t Bot::readLineBits() {
    return (digitalRead(IR_L) << 2) | (digitalRead(IR_C) << 1) | digitalRead(IR_R);
}

void Bot::check_line() {
    uint8_t pattern;
    switch (readLineBits()) {
        case 0b010: pattern = FORWARD; break;
        case 0b110: pattern = LEFT;    break;
        case 0b011: pattern = RIGHT;   break;
        case 0b000: pattern = STOP;    break;  // maybe junction
        default:    pattern = FORWARD; break;
    }
    move(pattern);
}

void Bot::offload(bool drop) {
    uint8_t target = drop ? 180 : 0;
    uint8_t current = _servo.read();
    if (current != target) {
        if (current < target) current += 2;
        else current -= 2;
        _servo.write(current);
    }
}

/* =====================  MAIN ====================================== */

Bot robot;

void setup() {
    analogWriteResolution(8); // Ensure resolution is 8-bit for PWM
}

void loop() {
    robot.setSpeed(200);
    robot.updateSpeed();
    robot.check_collision();
    robot.check_line();
    robot.offload(false); // Change to true to simulate dropping
}
