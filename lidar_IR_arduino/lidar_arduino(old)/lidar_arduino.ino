#define USE_USBCON // Arduino Micro board
#include <ros.h>
#include <std_msgs/Int32.h>

ros::NodeHandle  nh;

unsigned long pW;

std_msgs::Int32 pulseWidth;
ros::Publisher lidar("lidar", &pulseWidth);

void setup()
{
  nh.initNode();
  nh.advertise(lidar);

  pinMode(2, OUTPUT); // Set pin 2 as trigger pin
  digitalWrite(2, LOW); // Set trigger LOW for continuous read
  pinMode(3, INPUT); // Set pin 3 as monitor pin
}

void loop()
{
    pW = pulseIn(3, HIGH); // Count how long the pulse is high in microseconds
    pW = pW / 10; // 10usec = 1 cm of distance
    pulseWidth.data = pW;
    lidar.publish(&pulseWidth);
    nh.spinOnce();
    delay(10);
}
