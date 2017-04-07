#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>

Servo servo_ForkliftRotationMotor;
Servo servo_ForkliftGripMotor;
Servo servo_ForkliftRaiseMotor;
Servo servo_RightMotor;
Servo servo_LeftMotor;
Servo servo_ArmRotationMotor;
Servo servo_ArmFlipMotor;
Servo servo_GripMotor;

Servo servo_LeftPlatformServo;
Servo servo_RightPlatformServo;

I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;

// Uncomment keywords to enable debugging output

//#define DEBUG_MOTORS
//#define DEBUG_ULTRASONIC_LEFT
//#define DEBUG_ULTRASONIC_FRONT
//#define DEBUG_ENCODERS
//#define DEBUG_MOTOR_CALIBRATION

const int ci_Left_Motor_Offset_Address_L = 12;
const int ci_Left_Motor_Offset_Address_H = 13;
const int ci_Right_Motor_Offset_Address_L = 14;
const int ci_Right_Motor_Offset_Address_H = 15;

const int ci_Ultrasonic_Left_Ping = 2;   //input plug
const int ci_Ultrasonic_Left_Data = 3;   //output plug
const int ci_Ultrasonic_Front_Ping = 4;
const int ci_Ultrasonic_Front_Data = 5;

const int ci_Forklift_Rotation_Motor = 6;
const int ci_Forklift_Grip_Motor = 7;
const int ci_Forklift_Raise_Motor = 10;
const int ci_Right_Motor = 8;
const int ci_Left_Motor = 9;
const int ci_Arm_Rotation_Motor = 11;
const int ci_Arm_Flip_Motor = 12;
const int ci_Grip_Motor = 13;

const int ci_Left_Platform_Servo = A0;
const int ci_Right_Platform_Servo = A1;

const int ci_Course_Switch = A2;

const int ci_Hall_Effect = A3;

const int ci_I2C_SDA = A4;         // I2C data = white
const int ci_I2C_SCL = A5;         // I2C clock = yellow
/*IR Sensor will be connected directly to recieve port*/

const int ci_Forklift_Rotate_Front = 200;
const int ci_Forklift_Rotate_Back = 0;
const int ci_Forklift_Grip_Open = 140;
const int ci_Forklift_Grip_Closed = 90;
const int ci_Forklift_Raise_Drive = 10;
const int ci_Forklift_Raise_Bottom = 0;
const int ci_Forklift_Raise_Platform = 50;
const int ci_Left_Motor_Stop = 1500;        // 200 for brake mode; 1500 for stop
const int ci_Right_Motor_Stop = 1500;
const int ci_Arm_Rotate_Out = 90;      //  "
const int ci_Arm_Rotate_In = 0;      //  "
const int ci_Arm_Flip_Up = 0;
const int ci_Arm_Flip_Down = 60;
const int ci_Grip_Motor_Open = 150;         // Experiment to determine appropriate value
const int ci_Grip_Motor_Closed = 80;        //  "

const int ci_Motor_Calibration_Cycles = 3;
const int ci_Motor_Calibration_Time = 5000;

unsigned long ul_Left_Echo_Time;
unsigned long ul_Front_Echo_Time;

unsigned int ui_Motors_Speed = 1900;        // Default run speed
unsigned int ui_Left_Motor_Speed;
unsigned int ui_Right_Motor_Speed;
long l_Left_Motor_Position;
long l_Right_Motor_Position;

int Left_Platform_Servo_Pos = 0;
int Right_Platform_Servo_Pos = 0;

unsigned long ul_Display_Time;
unsigned long ul_Calibration_Time;
unsigned long ui_Left_Motor_Offset;
unsigned long ui_Right_Motor_Offset;

boolean bt_Cal_Initialized = false;

int stage = 0;
unsigned int LeftPingTime;
unsigned int FrontPingTime;
bool RobotStop = false;
double sensorVal;

int counter = 0;
int counter1 = 0;

bool pyramidFound = false;
bool Initialized = false;

int closeEnough = 0;

void setup() {
  Wire.begin();
  Serial.begin(2400);

  servo_LeftPlatformServo.attach(ci_Left_Platform_Servo);
  servo_RightPlatformServo.attach(ci_Right_Platform_Servo);

  //set up ultrasonic
  pinMode(ci_Ultrasonic_Left_Ping, OUTPUT);
  pinMode(ci_Ultrasonic_Left_Data, INPUT);

  pinMode(ci_Ultrasonic_Front_Ping, OUTPUT);
  pinMode(ci_Ultrasonic_Front_Data, INPUT);

  //set up drive motors
  pinMode(ci_Right_Motor, OUTPUT);
  servo_RightMotor.attach(ci_Right_Motor);
  pinMode(ci_Left_Motor, OUTPUT);
  servo_LeftMotor.attach(ci_Left_Motor);

  //set up forklift motors
  pinMode(ci_Forklift_Rotation_Motor, OUTPUT);
  servo_ForkliftRotationMotor.attach(ci_Forklift_Rotation_Motor);
  pinMode(ci_Forklift_Grip_Motor, OUTPUT);
  servo_ForkliftGripMotor.attach(ci_Forklift_Grip_Motor);
  pinMode(ci_Forklift_Raise_Motor, OUTPUT);
  servo_ForkliftRaiseMotor.attach(ci_Forklift_Raise_Motor);

  //set up arm motors
  pinMode(ci_Arm_Rotation_Motor, OUTPUT);
  servo_ArmRotationMotor.attach(ci_Arm_Rotation_Motor);
  pinMode(ci_Arm_Flip_Motor, OUTPUT);
  servo_ArmFlipMotor.attach(ci_Arm_Flip_Motor);
  pinMode(ci_Grip_Motor, OUTPUT);
  servo_GripMotor.attach(ci_Grip_Motor);

  // set up encoders. Must be initialized in order that they are chained together,
  // starting with the encoder directly connected to the Arduino. See I2CEncoder docs
  // for more information
  encoder_LeftMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_RightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_RightMotor.setReversed(true);  // adjust for positive count when moving forward
  encoder_LeftMotor.zero();
  encoder_RightMotor.zero();
}

void loop() {
  // put your main code here, to run repeatedly:
//  Serial.println(stage);
  
  
  
  if (stage == 0)
  {
    servo_ArmRotationMotor.detach();
    servo_ArmFlipMotor.write(ci_Arm_Flip_Up);
    delay(1000);
    servo_ArmFlipMotor.detach();
    servo_GripMotor.write(ci_Grip_Motor_Open);
    delay(1000);
    servo_GripMotor.detach();
    servo_ForkliftRotationMotor.write(ci_Forklift_Rotate_Front);
    delay(1000);
    servo_ForkliftRotationMotor.detach();
    servo_ForkliftGripMotor.write(ci_Forklift_Grip_Open);
    delay(1000);
    servo_ForkliftGripMotor.detach();
    servo_ForkliftRaiseMotor.write(ci_Forklift_Raise_Drive);
    delay(1000);
    servo_ForkliftRaiseMotor.detach();
    servo_LeftMotor.writeMicroseconds(1900);
    servo_RightMotor.writeMicroseconds(1900);
    stage++;
  }

  if (stage == 1)
  {
        if(!RobotStop)
        {
          PingLeft();
//          Serial.println(ul_Left_Echo_Time);
          if(ul_Left_Echo_Time > 380)
          {
            servo_RightMotor.writeMicroseconds(2000);
            servo_LeftMotor.writeMicroseconds(1650);
          }
          else if (ul_Left_Echo_Time < 260)
          {
            servo_LeftMotor.writeMicroseconds(2000);
            servo_RightMotor.writeMicroseconds(1650);
          }
          else
          {
            servo_LeftMotor.writeMicroseconds(1650);
            servo_RightMotor.writeMicroseconds(1650);
          }
        }

    sensorVal = analogRead(ci_Hall_Effect);
    Serial.println(sensorVal);
    
    if (sensorVal < 500 || sensorVal > 520)
    {
        RobotStop = true;
        servo_LeftMotor.writeMicroseconds(1500);
        servo_RightMotor.writeMicroseconds(1500);
        delay(1500);
        servo_ArmFlipMotor.attach(ci_Arm_Flip_Motor);
        servo_ArmFlipMotor.write(ci_Arm_Flip_Down);
        delay(1500);
        servo_GripMotor.attach(ci_Grip_Motor);
        servo_GripMotor.write(ci_Grip_Motor_Closed);
        delay(1000);
        servo_GripMotor.detach();
        servo_ArmFlipMotor.write(ci_Arm_Flip_Up);
        delay(1000);
        servo_ArmRotationMotor.attach(ci_Arm_Rotation_Motor);
        servo_ArmRotationMotor.write(ci_Arm_Rotate_In);
        delay(1000);
        servo_GripMotor.attach(ci_Grip_Motor);
        servo_GripMotor.write(ci_Grip_Motor_Closed);
        delay(1000);
        servo_ArmRotationMotor.write(ci_Arm_Rotate_Out);
        delay(1000);
        servo_ArmFlipMotor.write(ci_Arm_Flip_Down);
        delay(100);
        servo_ArmRotationMotor.detach();
        servo_ArmFlipMotor.detach();
        servo_GripMotor.detach();
  
        stage++;
    }
  }

  
  if(stage == 2)
  {
    Serial.println(counter);
    if(!pyramidFound)
    {
       if(counter <= 225)
       {
         servo_RightMotor.writeMicroseconds(1100);
         servo_LeftMotor.writeMicroseconds(2100);
         counter++;
       }
       else if(counter > 225 && counter <= 425)
       {
         servo_RightMotor.writeMicroseconds(1900);
         servo_LeftMotor.writeMicroseconds(1900);
         counter++;
       }
       else
       {
         PingFront();
         if(!Initialized)
         {
           servo_RightMotor.writeMicroseconds(1700);
           servo_LeftMotor.writeMicroseconds(1500);
         }
         if(ReadIRsensor() == 1)
         {
           servo_RightMotor.writeMicroseconds(1700);
           servo_LeftMotor.writeMicroseconds(1700);
           counter1 = 0;

           if((ul_Front_Echo_Time/148) > 0)
           { 
             if((ul_Front_Echo_Time/148) <= 2)
             {
               pyramidFound = true;
             }
           }
           
//           Serial.println("Read");
         }
         else if (ReadIRsensor() == -1)
         { 
           counter1 = 0;
//           Serial.println("NotRight");
         }
         else if (ReadIRsensor() == 0 && counter1 >= 500)
         {
           servo_LeftMotor.writeMicroseconds(1500);
           servo_RightMotor.writeMicroseconds(1700);
//           Serial.println("NotRead");

           if((ul_Front_Echo_Time/148) > 0)
           { 
             if((ul_Front_Echo_Time/148) <= 2)
             {
               pyramidFound = true;
               stage++;
             }
           }
         }
         else
         {
           counter1++;
         }
       }
    //    Serial.println(ReadIRsensor());
     }
     else 
     {
       servo_RightMotor.writeMicroseconds(1500);
       servo_LeftMotor.writeMicroseconds(1500);
     }
  }

  if(stage == 4)
  {
    servo_ForkliftRotationMotor.attach(ci_Forklift_Rotation_Motor);
    servo_ForkliftGripMotor.attach(ci_Forklift_Grip_Motor);
    servo_ForkliftRaiseMotor.attach(ci_Forklift_Raise_Motor);

    servo_ForkliftRaiseMotor.write(ci_Forklift_Raise_Bottom);
    servo_ForkliftGripMotor.write(ci_Forklift_Grip_Closed);
    servo_ForkliftRaiseMotor.write((ci_Forklift_Raise_Platform) + 20);
    servo_ForkliftRotationMotor.write(ci_Forklift_Rotate_Back);
    servo_ForkliftRaiseMotor.write(ci_Forklift_Raise_Platform); 
    servo_ForkliftGripMotor.write(ci_Forklift_Grip_Open);
    servo_ForkliftRaiseMotor.write((ci_Forklift_Raise_Platform) + 20);
    servo_ForkliftRotationMotor.write(ci_Forklift_Rotate_Front);

    analogWrite(ci_Left_Platform_Servo, 0);
    analogWrite(ci_Right_Platform_Servo, 0);    
  }  
}

void CalibrateMotors()
{
  if (!bt_Cal_Initialized)
  {
    bt_Cal_Initialized = true;
    encoder_LeftMotor.zero();
    encoder_RightMotor.zero();
    ul_Calibration_Time = millis();
    servo_LeftMotor.writeMicroseconds(ui_Motors_Speed);
    servo_RightMotor.writeMicroseconds(ui_Motors_Speed);
  }
  else if ((millis() - ul_Calibration_Time) > ci_Motor_Calibration_Time)
  {
    servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
    servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
    l_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
    l_Right_Motor_Position = encoder_RightMotor.getRawPosition();
    if (l_Left_Motor_Position > l_Right_Motor_Position)
    {
      // May have to update this if different calibration time is used
      ui_Right_Motor_Offset = 0;
      ui_Left_Motor_Offset = (l_Left_Motor_Position - l_Right_Motor_Position) / 4;
      stage++;
    }
    else
    {
      // May have to update this if different calibration time is used
      ui_Right_Motor_Offset = (l_Right_Motor_Position - l_Left_Motor_Position) / 4;
      ui_Left_Motor_Offset = 0;
      stage++;
    }

#ifdef DEBUG_MOTOR_CALIBRATION
    Serial.print("Motor Offsets: Left = ");
    Serial.print(ui_Left_Motor_Offset);
    Serial.print(", Right = ");
    Serial.println(ui_Right_Motor_Offset);
#endif
    EEPROM.write(ci_Right_Motor_Offset_Address_L, lowByte(ui_Right_Motor_Offset));
    EEPROM.write(ci_Right_Motor_Offset_Address_H, highByte(ui_Right_Motor_Offset));
    EEPROM.write(ci_Left_Motor_Offset_Address_L, lowByte(ui_Left_Motor_Offset));
    EEPROM.write(ci_Left_Motor_Offset_Address_H, highByte(ui_Left_Motor_Offset));
  }

#ifdef DEBUG_MOTOR_CALIBRATION
  Serial.print("Encoders L: ");
  Serial.print(encoder_LeftMotor.getRawPosition());
  Serial.print(", R: ");
  Serial.println(encoder_RightMotor.getRawPosition());
#endif
}


// measure distance to target using ultrasonic sensor
void PingLeft()
{
  //Ping Ultrasonic
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  digitalWrite(ci_Ultrasonic_Left_Ping, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ci_Ultrasonic_Left_Ping, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW
  ul_Left_Echo_Time = pulseIn(ci_Ultrasonic_Left_Data, HIGH, 10000);

  // Print Sensor Readings
#ifdef DEBUG_ULTRASONIC_LEFT
  Serial.print("Time (microseconds): ");
  Serial.print(ul_Left_Echo_Time, DEC);
  Serial.print(", Inches: ");
  Serial.print(ul_Left_Echo_Time / 148); //divide time by 148 to get distance in inches
  Serial.print(", cm: ");
  Serial.println(ul_Left_Echo_Time / 58); //divide time by 58 to get distance in cm
#endif
}

// measure distance to target using ultrasonic sensor
void PingFront()
{
  //Ping Ultrasonic
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  digitalWrite(ci_Ultrasonic_Front_Ping, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ci_Ultrasonic_Front_Ping, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW
  ul_Front_Echo_Time = pulseIn(ci_Ultrasonic_Front_Data, HIGH, 10000);

  // Print Sensor Readings
#ifdef DEBUG_ULTRASONIC_FRONT
  Serial.print("Time (microseconds): ");
  Serial.print(ul_Front_Echo_Time, DEC);
  Serial.print(", Inches: ");
  Serial.print(ul_Front_Echo_Time / 148); //divide time by 148 to get distance in inches
  Serial.print(", cm: ");
  Serial.println(ul_Front_Echo_Time / 58); //divide time by 58 to get distance in cm
#endif
}


int ReadIRsensor()
{
  if (Serial.available())
  {
    if (analogRead(ci_Course_Switch) == 1023 && (Serial.read() == 65 || Serial.read() == 69)) //A&E switch up
    {
      return 1;
    }
    else if (analogRead(ci_Course_Switch) == 0 && (Serial.read() == 73 || Serial.read() == 79)) //I&O switch down
    {
      return 1;
    }
    else
    {
      return -1;
    }
  }
  else
  {
    return 0;
  }
}

