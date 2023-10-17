#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#define STOP      0
#define FORWARD   1
#define BACKWARD  2
#define TURNLEFT  3
#define TURNRIGHT 4
// 设置 LCD 地址为 0x27
#define LCD_I2C_ADDR 0x27
// 设置LCD列数和行数
#define LCD_COLUMNS 16
#define LCD_ROWS 2
// 初始化LCD对象
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLUMNS, LCD_ROWS);

int leftMotor1 = 2;
int leftMotor2 = 3;
int rightMotor1 = 4;
int rightMotor2 = 5;

int leftPWM = 6;
int rightPWM = 7;

int leftSensorTriggerPin = 8;   // 左边超声波trigger引脚
int leftSensorEchoPin = 9;      // 左边超声波echo引脚
int frontSensorTriggerPin = 10; // 前方超声波trigger引脚
int frontSensorEchoPin = 11;    // 前方超声波echo引脚
int rightSensorTriggerPin = 12; // 右边超声波trigger引脚
int rightSensorEchoPin = 13;    // 右边超声波echo引脚

void setup() {
  // 串口初始化
  Serial.begin(9600);
  
  lcd.begin(16, 2);
   lcd.print("Hello world!");
  // 初始化电机控制引脚
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(leftPWM, OUTPUT);
  pinMode(rightPWM, OUTPUT);

  // 初始化超声波控制引脚
  pinMode(leftSensorTriggerPin, OUTPUT);
  pinMode(leftSensorEchoPin, INPUT);
  pinMode(frontSensorTriggerPin, OUTPUT);
  pinMode(frontSensorEchoPin, INPUT);
  pinMode(rightSensorTriggerPin, OUTPUT);
  pinMode(rightSensorEchoPin, INPUT);
  // 初始化I2C通信
  Wire.begin();

  // 初始化LCD
  lcd.begin(LCD_COLUMNS, LCD_ROWS);

  // 打开背光
  lcd.backlight();

  // 在第一行显示消息
  lcd.setCursor(2, 0);
  lcd.print("Hello,world!");
  lcd.setCursor(2,1);
  lcd.print("20 30%");
  
}



void loop() {
  // 主要代码循环
  avoidance();
}

void motorRun(int cmd, int value) {
  analogWrite(leftPWM, value);
  analogWrite(rightPWM, value);

  switch (cmd) {
    case FORWARD:
      Serial.println("FORWARD");
      digitalWrite(leftMotor1, HIGH);
      digitalWrite(leftMotor2, LOW);
      digitalWrite(rightMotor1, HIGH);
      digitalWrite(rightMotor2, LOW);
      break;
    case BACKWARD:
      Serial.println("BACKWARD");
      digitalWrite(leftMotor1, LOW);
      digitalWrite(leftMotor2, HIGH);
      digitalWrite(rightMotor1, LOW);
      digitalWrite(rightMotor2, HIGH);
      break;
    case TURNLEFT:
     /* Serial.println("TURN  LEFT");
      digitalWrite(leftMotor1, LOW);
      digitalWrite(leftMotor2, HIGH);
      digitalWrite(rightMotor1, HIGH);
      digitalWrite(rightMotor2, LOW);*/

       Serial.println("TURN  LEFT");
      digitalWrite(leftMotor1, HIGH);
      digitalWrite(leftMotor2, LOW);
      digitalWrite(rightMotor1, LOW);
      digitalWrite(rightMotor2, HIGH);

      break;
    case TURNRIGHT:
    /*  Serial.println("TURN  RIGHT");
      digitalWrite(leftMotor1, HIGH);
      digitalWrite(leftMotor2, LOW);
      digitalWrite(rightMotor1, LOW);
      digitalWrite(rightMotor2, HIGH);*/

   Serial.println("TURN  RIGHT");
      digitalWrite(leftMotor1, LOW);
      digitalWrite(leftMotor2, HIGH);
      digitalWrite(rightMotor1, HIGH);
      digitalWrite(rightMotor2, LOW);
      
      break;
    default:
      Serial.println("STOP");
      digitalWrite(leftMotor1, LOW);
      digitalWrite(leftMotor2, LOW);
      digitalWrite(rightMotor1, LOW);
      digitalWrite(rightMotor2, LOW);
  }
}

void avoidance() {
  int distances[3];

  // 前进
  motorRun(FORWARD, 80);

  // 获取传感器距离
  distances[0] = getDistance(leftSensorTriggerPin, leftSensorEchoPin);   // 左边
  distances[1] = getDistance(frontSensorTriggerPin, frontSensorEchoPin); // 前方
  distances[2] = getDistance(rightSensorTriggerPin, rightSensorEchoPin); // 右边

  if (distances[1] < 10) {
    motorRun(STOP, 0);
    delay(200);
    distances[0] = getDistance(leftSensorTriggerPin, leftSensorEchoPin);   // 左边
    delay(100);
    distances[1] = getDistance(frontSensorTriggerPin, frontSensorEchoPin); // 前方
    delay(100);
    distances[2] = getDistance(rightSensorTriggerPin, rightSensorEchoPin); // 右边


    

    if (distances[0] < distances[2]) { // 右边距离障碍物更远
      // 右转
      // lcd.setCursor(0, 0);
      // lcd.print("TURN RIGHT");
       
          if(distances[0]<8&&distances[1]<8){
             
             Serial.println("warning!");
          motorRun(BACKWARD,200);
           delay(200);
            motorRun(STOP,0);
           delay(200);
           motorRun(TURNRIGHT,200);
           delay(150);
         }
         else{
                  motorRun(BACKWARD,150);
                  delay(100);
                    motorRun(STOP,0);
           delay(200);
                  motorRun(TURNRIGHT, 170);
                  delay(270);
          
         }     
    } 
 
     else if(distances[2]<distances[0]){  // 右边距离障碍物更近
      // 左转
             //lcd.setCursor(0, 0);
             //lcd.print("TURN LEFT");
          if(distances[1]<8&&distances[2]<8){
             
             Serial.println("warning!warning!");
             
          motorRun(BACKWARD,200);
           delay(200);
           motorRun(STOP,0);
           delay(200);
           motorRun(TURNLEFT,200);
           delay(150);
         }//小转
         else{
      motorRun(BACKWARD,150);
        delay(100);
          motorRun(STOP,0);
           delay(200);
      motorRun(TURNLEFT, 170);
      delay(270);
       //大转 
      }
       }

       
      }
  if(distances[0]<4){
    Serial.println("warning!warning!warning!");
     
      motorRun(BACKWARD, 130);
      delay(200);

      motorRun(TURNRIGHT, 180);
      delay(200);
       motorRun(FORWARD,130);
      delay(250);


  }
  if(distances[2]<4){
    Serial.println("warning!warning!warning!");
     
      motorRun(BACKWARD, 130);
      delay(200); 
      motorRun(TURNLEFT, 180);
      delay(200);
      motorRun(FORWARD,130);
      delay(250);
     


  }

      
  }




int getDistance(int triggerPin, int echoPin) {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  int duration = pulseIn(echoPin, HIGH);
  int distance = duration / 58;
  Serial.println("distances:");
  Serial.println(distance);

  if (distance >= 50||distance <0) {
    return 50;
  }
   
  else {
    return distance;
  }
}
