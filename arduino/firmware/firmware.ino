#include "AFMotor.h"

#define START_BYTE 0xAA
#define END_BYTE   0x55
#define ID_SENSOR_RANGE 0x01
#define ID_CMD_RPM 0x10

// Range sensor
const int Trigger = A0;
const int Echo = A1;  
uint8_t range[2];

AF_DCMotor motor_left(1);
AF_DCMotor motor_right(2);

int motor_left_pwm = 0;
int motor_left_state = RELEASE;
int motor_right_pwm = 0;
int motor_right_state = RELEASE;
int motor_watch_dog = 0;

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

long readRange() {
  long t; //timepo que demora en llegar el eco
 
  digitalWrite(Trigger, HIGH);
  delayMicroseconds(10);          //Enviamos un pulso de 10us
  digitalWrite(Trigger, LOW);
  
  t = pulseIn(Echo, HIGH, 17700); //obtenemos el ancho del pulso (rang màxim 3m)
  return t/59;             //escalamos el tiempo a una distancia en cm
}

void setup() {
  //Init Serial
  Serial.begin(9600);
  // Init Motors
  writeMotors();
  // Setup range sensor
  pinMode(Trigger, OUTPUT); //pin como salida
  pinMode(Echo, INPUT);  //pin como entrada
  digitalWrite(Trigger, LOW);//Inicializamos el pin con 0
}

void loop() {
  // Serial.println("Hola!");
  receiveMessage();
  writeMotors();
  delay(50); 
  uint16_t r = readRange();
  range[0] = r & 0xFF;        // byte baix (LSB)
  range[1] = (r >> 8) & 0xFF; // byte alt (MSB)
  sendMessage(ID_SENSOR_RANGE, range, 2);
  
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
