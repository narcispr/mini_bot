#include "AFMotor.h"
#include <NewPing.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

// Communication Protocol
#define START_BYTE 0xAA
#define END_BYTE   0x55
#define ID_SENSOR_RANGE 0x01
#define ID_SENSOR_ENCODERS 0x02
#define ID_SENSOR_COMPASS 0x03
#define ID_CMD_RPM 0x10

// Crear l'objecte del sensor
Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified(12345);
uint8_t angle[2];
// CONNECT the compass to the I2C bus
// SCL --> A5
// SDA --> A4

// Ultrasonic sensor
#define MAX_DIST  300
#define TRIG_PIN  A0
#define ECHO_PIN  A1  
uint8_t range[2];
uint16_t dist = 0;
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DIST);

// Motors
AF_DCMotor motor_left(3);
AF_DCMotor motor_right(4);

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

// Divideix un valor de 16 o 32 bits en un array de bytes
void splitToBytes(uint32_t value, uint8_t* buffer, uint8_t num_bytes) {
  for (uint8_t i = 0; i < num_bytes; i++) {
    buffer[i] = (value >> (8 * i)) & 0xFF;
  }
}

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
  Serial.begin(115200);
  
  // Init Motors
  writeMotors();
  
  // Init encoders
  pulses_left = 0;
  pulses_right = 0;
  pinMode(ENCODER_LEFT, INPUT);
  pinMode(ENCODER_RIGHT, INPUT);

  // Init compass
  Wire.begin();

  // Inicialitzar el sensor
  if (!compass.begin()) {
    splitToBytes(0xFFFF, angle, 2); 
    sendMessage(ID_SENSOR_COMPASS, angle, 2); // Envia un valor d'angle que no pot ser...
  } 
}

void loop() {
  unsigned long now = millis();

  // At every 50 ms:
  if (now - last_loop_time >= (LOOP_PERIOD_MS)) {
    // read range sensor
    last_loop_time = now;
    unsigned int dist = sonar.ping_cm();
    splitToBytes(dist, range, 2);
    sendMessage(ID_SENSOR_RANGE, range, 2);
    
    // Send Encoder Pulses
    pulses[0] = pulses_left;
    pulses[1] = pulses_right;
    pulses_left = 0;
    pulses_right = 0;
    sendMessage(ID_SENSOR_ENCODERS, pulses, 2);
    
    // Process motor commands
    receiveMessage();
    writeMotors();
    if (motor_watch_dog <= 0) {
      motor_left_pwm = 0;
      motor_left_state = BRAKE;
      motor_right_pwm = 0;
      motor_right_state = BRAKE;
    }
    else {
      motor_watch_dog = motor_watch_dog - 1;
    }

    // Read compass
    sensors_event_t event;
    compass.getEvent(&event);
    if (event.magnetic.x == 0 && event.magnetic.y == 0 && event.magnetic.z == 0) {
      splitToBytes(0xFFFF, angle, 2); // Envia un valor d'angle que no pot ser...
    } else {
      int16_t heading = atan2(event.magnetic.y, event.magnetic.x) * 180 / PI; // Convert to degrees
      if (heading < 0) heading += 360; // Normalize to 0-359
      splitToBytes(heading, angle, 2);
    }
    sendMessage(ID_SENSOR_COMPASS, angle, 2);
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
