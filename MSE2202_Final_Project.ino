#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>

Servo servo_RightMotor;
Servo servo_LeftMotor;
Servo servo_ArmRotationMotor;
Servo servo_ArmFlipMotor;
Servo servo_GripMotor;
Servo servo_ForkliftRotationMotor;
Servo servo_ForkliftGripMotor;
Servo servo_ForkliftRaiseMotor;

I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;

const int ci_Ultrasonic_Left_Ping = 2;   //input plug
const int ci_Ultrasonic_Left_Data = 3;   //output plug
const int ci_Ultrasonic_Front_Ping = 4;
const int ci_Ultrasonic_Front_Data = 5;

const int ci_Right_Motor = 8;
const int ci_Left_Motor = 9;
const int ci_Arm_Rotation_Motor = 10;
const int ci_Arm_Flip_Motor = 11;
const int ci_Grip_Motor = 12;
const int ci_Forklift_Rotation_Motor = 13
const int ci_Forklift_Grip_Motor = 6
const int ci_Forklift_Raise_Motor = 7

const int ci_I2C_SDA = A4;         // I2C data = white
const int ci_I2C_SCL = A5;         // I2C clock = yellow

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
