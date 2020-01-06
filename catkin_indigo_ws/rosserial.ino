/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 * Add servo controll
 */

#include <ros.h>
#include <std_msgs/Empty.h>
#include <Servo.h>
ros::NodeHandle nh;
#define DEFAULT_BAUDRATE 9600
#define DEFAULT_SERIALPORT "/dev/rfcomm9"

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  if(digitalRead(13) == 0)
  {
    for (pos = 0; pos <= 20; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(150);                       // waits 15ms for the servo to reach the position
    }
  }
  if(digitalRead(13) == 1)
  {
    for (pos = 20; pos >= 0; pos -= 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(150);                       // waits 15ms for the servo to reach the position
    }
  }
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );

void setup()
{
  pinMode(13, OUTPUT);
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  Serial.begin(9600);
      nh.getHardware()->setBaud(9600);
      nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
