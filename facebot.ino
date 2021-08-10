/***********************************************************************************************************************************************************
 *  TITLE: This is an example skecth that will help you interface a servo to an Arduino.
 *
 *  By Frenoy Osburn
 *  YouTube Video: https://youtu.be/NoqjQKcRkVw
 ***********************************************************************************************************************************************************/

  /********************************************************************************************************************
 *  Board Settings:
 *  Board: "Arduino Nano"
 *  Processor: "ATmega328P (Old Bootloader)"
 *  COM Port: Depends *On Your System*
 *********************************************************************************************************************/
 
#include <Servo.h>

#define SERVO_PIN_OX    11  //PWM pin that is connected to the servo
#define SERVO_PIN_OY    10  //PWM pin that is connected to the servo
#define RESET_PIN 9
#define ANALOG_OY A4
#define ANALOG_OX A5

#define INITIAL_X 90
#define INITIAL_Y 90
#define LIMIT_MIN_Y 25
#define LIMIT_MAX_Y 140

#define RAD 57.2957795

Servo servo_ox;        //create a servo object
Servo servo_oy;        //create a servo object

int servoAngle = 20;     //servo angle which can vary from 0 - 180
int threshold = 20;
int dtime = 50;

long last_command = 0;
long command_timeout = 10 * 1000;

bool ready = false;
void setup() 
{
  pinMode(RESET_PIN, INPUT_PULLUP);
  pinMode(ANALOG_OX, INPUT);
  pinMode(ANALOG_OY, INPUT);
  servo_ox.attach(SERVO_PIN_OX);  //attach the pin to the object so that we can send the signal to it
  servo_oy.attach(SERVO_PIN_OY);  //attach the pin to the object so that we can send the signal to 

  servo_ox.write(INITIAL_X);
  servo_oy.write(INITIAL_Y);

  Serial.begin(115200);
}

int x = INITIAL_X, y = INITIAL_Y;

void move_servo(int dx, int dy, int dtime) {
  /*
  Serial.print(x);
  Serial.print(" ");
  Serial.print(y);
  Serial.print(" | ");
  Serial.print(dx);
  Serial.print(" ");
  Serial.println(dy);
  */
  x += dx;
  y += dy;
  
  x = max(0, x);
  x = min(x, 180);
  y = max(LIMIT_MIN_Y, y);
  y = min(y, LIMIT_MAX_Y);

  /*
  if (x < 0) {
    x = 180 + x % 180;
    y = 180 - y;
  } else if (x > 180) {
    x = x % 180;
    y = 180 - y;
  }

  y = max(0, y);
  y = min(y, 180);
  */

  servo_ox.write(x);                                   //move servo to 0 deg
  servo_oy.write(y);                                   //move servo to 0 deg
  
  delay(dtime);                                           //wait for dtime ms 
}

void joystick_loop() {
  int dx = analogRead(ANALOG_OX);
  int dy = analogRead(ANALOG_OY);
  if (dx > 512+threshold || dx < 512-threshold) {
    dx = map(dx, 0, 1023, -servoAngle, servoAngle);
  }
  else dx = 0;
  if (dy > 512+threshold || dy < 512-threshold) {
    dy = map(dy, 0, 1023, -servoAngle, servoAngle);
  }
  else dy = 0;
  if (dx || dy)
  move_servo(dx,dy,dtime);
  if (digitalRead(RESET_PIN) == LOW) {
    dx = -x;
    dy = -y;
    move_servo(dx,dy,dtime);
  }
}

void rotation_loop() {
  move_servo(1,0,dtime/10);
}

bool check_command() {
  if (Serial.available() >= 4) {
    last_command = millis();
    return true;
  }
  return false;
}

void move_servo_smooth(int dx, int dy, int dtime) {
  if (dx > 0)
    for (int i = 0; i < dx && !check_command(); i ++)
      move_servo(1,0,dtime);
  else
    for (int i = 0; i > dx && !check_command(); i --)
      move_servo(-1,0,dtime);

  if (dy > 0)
    for (int i = 0; i < dy && !check_command(); i ++)
      move_servo(0,1,dtime);
  else
    for (int i = 0; i > dy && !check_command(); i --)
      move_servo(0,-1,dtime);
}

void inspection_loop() {
  move_servo_smooth(-x+90,-y+90,dtime);
  for(int i = 90; i < 180 && !check_command(); i ++) {
    move_servo(1,sin(i*4/RAD)*1.5,dtime);
  }
  for(int i = 180; i > 0 && !check_command(); i --) {
    move_servo(-1,sin((180-i)*4/RAD)*1.5,dtime);
  }
  for(int i = 0; i < 90 && !check_command(); i ++) {
    move_servo(1,sin(i*4/RAD)*1.5,dtime);
  }
}

void serial_loop() {
  int dx,dy,sx,sy;
  while(Serial.available() >= 4){
    // fill array
    sx = Serial.read();
    sy = Serial.read();
    dx = Serial.read();
    dy = Serial.read();

    if (sx) dx *= -1;
    if (sy) dy *= -1;
    
    // use the values
    move_servo(dx,dy,dtime);
    last_command = millis();

    Serial.print(dx);
    Serial.print(" and ");
    Serial.println(dy);
  }
  
}

void loop() 
{
  if (!ready) {
    ready = true;
    Serial.println("Initial");
  }
  delay(100);
  if (millis() - last_command < command_timeout)
    serial_loop();
  else
    inspection_loop();
    
}
