#include "AFMotor.h"
#include <NewPing.h>

// Communication Protocol
#define START_BYTE 0xAA
#define END_BYTE   0x55
#define ID_SENSOR_RANGE 0x01
#define ID_ENCODERS 0x02
#define ID_CMD_RPM 0x10

// Ultrasonic sensor
#define MAX_DIST  300
#define TRIG_PIN  A0
#define ECHO_PIN  A1  
uint8_t range[2];
uint16_t dist = 0;
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DIST);

// Motors
AF_DCMotor motor_left(1);
AF_DCMotor motor_right(2);

int motor_left_pwm = 0;
int motor_left_state = RELEASE;
int motor_right_pwm = 0;
int motor_right_state = RELEASE;
int motor_watch_dog = 0;


//Encoders
uint8_t rpms[2];
volatile byte pulses_left;
volatile byte pulses_right;
unsigned long last_encoders_time;
#define PULSE_PER_TURN 20
#define ENCODER_LEFT A2
#define ENCODER_RIGHT A3


// Others
#define LOOP_PERIOD_MS  50
unsigned long last_loop_time = 0;

void sendMessage(uint8_t id, uint8_t* data, uint8_t len) {
  Serial.write(START_BYTE);
  Serial.write(id);
  Serial.write(len);
  uint8_t checksum = id ^ len;
  for (uint8_t i = 0; i < len; i++) {
    Serial.write(data[i]);
    checksum ^= data[i];
  }
  Serial.write(checksum);
  Serial.write(END_BYTE);
}

void receiveMessage() {
  static enum { WAIT_START, READ_ID, READ_LEN, READ_DATA, READ_CHK, WAIT_END } state = WAIT_START;
  static uint8_t buffer[32];
  static uint8_t idx = 0;
  static uint8_t id, len, checksum;

  while (Serial.available()) {
    uint8_t byte = Serial.read();
    switch (state) {
      case WAIT_START:
        if (byte == START_BYTE) state = READ_ID;
        break;
      case READ_ID:
        id = byte;
        checksum = id;
        state = READ_LEN;
        break;
      case READ_LEN:
        len = byte;
        checksum ^= len;
        idx = 0;
        state = READ_DATA;
        break;
      case READ_DATA:
        buffer[idx++] = byte;
        checksum ^= byte;
        if (idx >= len) state = READ_CHK;
        break;
      case READ_CHK:
        if (byte == checksum) state = WAIT_END;
        else state = WAIT_START;
        break;
      case WAIT_END:
        if (byte == END_BYTE) {
          if (id == ID_CMD_RPM && len == 4) {
            processMotorCMD(buffer);
          }
        }
        state = WAIT_START;
        break;
    }
  }
}

void processMotorCMD(uint8_t* data) {
  motor_left_state  = (data[0] == 0) ? BACKWARD : FORWARD;
  motor_left_pwm    = data[1];
  motor_right_state = (data[2] == 0) ? BACKWARD : FORWARD;
  motor_right_pwm   = data[3];
  motor_watch_dog = 10;
}

void writeMotors() {
  motor_left.run(motor_left_state);
  motor_left.setSpeed(motor_left_pwm);
  motor_right.run(motor_right_state);
  motor_right.setSpeed(motor_right_pwm);
}

void count_left(){
  pulses_left++;
}

void count_right(){
  pulses_right++;
}


/*void ecoCheck() {
  if (sonar.check_timer()) {
    dist = sonar.ping_result / US_ROUNDTRIP_CM;
  }
}*/


void setup() {
  //Init Serial
  Serial.begin(9600);
  
  // Init Motors
  writeMotors();
  
  // Init encoders
  pulses_left = 0;
  pulses_right = 0;
  last_encoders_time = millis();
  pinMode(ENCODER_LEFT, INPUT);
  pinMode(ENCODER_RIGHT, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT), count_left, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), count_right, FALLING);
}

void loop() {
  unsigned long now = millis();

  if (now - last_loop_time >= (LOOP_PERIOD_MS)) {
    last_loop_time = now;
    // sonar.ping_timer(ecoCheck);
    unsigned int dist = sonar.ping_cm();
    range[0] = dist & 0xFF;        // byte baix (LSB)
    range[1] = (dist >> 8) & 0xFF; // byte alt (MSB)
    sendMessage(ID_SENSOR_RANGE, range, 2);
    
    receiveMessage();
    writeMotors();
    if (motor_watch_dog <= 0) {
      motor_left_pwm = 0;
      motor_left_state = RELEASE;
      motor_right_pwm = 0;
      motor_right_state = RELEASE;
    }
    else {
      motor_watch_dog = motor_watch_dog - 1;
    }

    // Read Encoders
    detachInterrupt(digitalPinToInterrupt(ENCODER_LEFT));
    detachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT));
    rpms[0] = (60*LOOP_PERIOD_MS/PULSE_PER_TURN)/(now - last_loop_time) * pulses_left;
    rpms[1] = (60*LOOP_PERIOD_MS/PULSE_PER_TURN)/(now - last_loop_time) * pulses_right;
    pulses_left = 0;
    pulses_right = 0;
    sendMessage(ID_ENCODERS, rpms, 2);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT), count_left, FALLING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), count_right, FALLING);
  } 
}
