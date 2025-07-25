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
#define ENCODER_LEFT A2
#define ENCODER_RIGHT A3
volatile byte pulses_left;
volatile byte pulses_right;
bool prev_left_state = HIGH;
bool prev_right_state = HIGH;
uint8_t pulses[2];

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


void setup() {
  //Init Serial
  Serial.begin(9600);
  
  // Init Motors
  writeMotors();
  
  // Init encoders
  pulses_left = 0;
  pulses_right = 0;
  pinMode(ENCODER_LEFT, INPUT);
  pinMode(ENCODER_RIGHT, INPUT);
}

void loop() {
  unsigned long now = millis();

  // At every 50 ms:
  if (now - last_loop_time >= (LOOP_PERIOD_MS)) {
    // read range sensor
    last_loop_time = now;
    unsigned int dist = sonar.ping_cm();
    range[0] = dist & 0xFF;        // byte baix (LSB)
    range[1] = (dist >> 8) & 0xFF; // byte alt (MSB)
    sendMessage(ID_SENSOR_RANGE, range, 2);
    
    // Send Encoder Pulses
    pulses[0] = pulses_left;
    pulses[1] = pulses_right;
    pulses_left = 0;
    pulses_right = 0;
    sendMessage(ID_ENCODERS, pulses, 2);
    
    // Process motor commands
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
  } 

  // At Every iteration: Read encoder pulses bu polling
  bool current_left_state = digitalRead(ENCODER_LEFT);
  bool current_right_state = digitalRead(ENCODER_RIGHT);

  if (prev_left_state == HIGH && current_left_state == LOW) {
    pulses_left++;
  }
  if (prev_right_state == HIGH && current_right_state == LOW) {
    pulses_right++;
  }
  prev_left_state = current_left_state;
  prev_right_state = current_right_state;
}
