#include <Wire.h>
#include <ZumoMotors.h>

#define I2C_SLAVE_ADDRESS  0x03

ZumoMotors motors;

struct {
  int16_t rightSpeed;
  int16_t leftSpeed;
} motorSpeed;


void setup() {
  Serial.begin(9600);

  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.onReceive(receiveData);

  motorSpeed.rightSpeed = 0;
  motorSpeed.leftSpeed  = 0;

  motors.flipRightMotor(true);
  motors.flipLeftMotor(false);

  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
}

void loop() {
  motors.setSpeeds(motorSpeed.rightSpeed, motorSpeed.leftSpeed);
}

// callback for received data
void receiveData(int byteCount) {

  // Serial.print("byte count: ");
  // Serial.println(byteCount);

  // discard the first byte. For some reason we are getting and extra one
  Wire.read();

  motorSpeed.rightSpeed = (((uint16_t)Wire.read()) << 8) | ((uint16_t)Wire.read());
  motorSpeed.leftSpeed  = (((uint16_t)Wire.read()) << 8) | ((uint16_t)Wire.read());

  // Serial.print("right: "); Serial.print(motorSpeed.rightSpeed);
  // Serial.print(" left: "); Serial.print(motorSpeed.leftSpeed);
  // Serial.println();
  
}
