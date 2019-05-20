#define USE_USBCON // Arduino Micro board
#include <ros.h>
#include <std_msgs/UInt32.h>

ros::NodeHandle  nh;

unsigned long pW;

static long analogPinTimer = 0; 
// Set the sampling time
#define ANALOG_PIN_TIMER_INTERVAL 2 // milliseconds
unsigned long thisMillis_old;

int fc = 5; // cutoff frequency 5~10 Hz 정도 사용해보시기 바랍니다
double dt = ANALOG_PIN_TIMER_INTERVAL/1000.0; // sampling time
double lambda = 2*PI*fc*dt;
double x1 = 0.0;
double x1_f = 0.0;
double x1_fold = 0.0;
double x2 = 0.0;
double x2_f = 0.0;
double x2_fold = 0.0;

std_msgs::UInt32 lidar;
std_msgs::UInt32 ir1;
std_msgs::UInt32 ir2;
ros::Publisher Lidar("Lidar", &lidar);
ros::Publisher IR1("IR1", &ir1);
ros::Publisher IR2("IR2", &ir2);


void setup()
{
  nh.initNode();
  nh.advertise(Lidar);
  nh.advertise(IR1);
  nh.advertise(IR2);
  pinMode(2, OUTPUT); // Set pin 2 as trigger pin
  digitalWrite(2, LOW); // Set trigger LOW for continuous read
  pinMode(3, INPUT); // Set pin 3 as monitor pin
}

void loop()
{
  pW = pulseIn(3, HIGH); // Count how long the pulse is high in microseconds
  pW = pW / 10; // 10usec = 1 cm of distance
  lidar.data = pW;
  unsigned long deltaMillis = 0; // clear last result
  unsigned long thisMillis = millis();  
  if (thisMillis != thisMillis_old) { 
    deltaMillis = thisMillis-thisMillis_old; 
    thisMillis_old = thisMillis;   
  } 
  
  analogPinTimer -= deltaMillis; 
  
  if (analogPinTimer <= 0) {  
    analogPinTimer += ANALOG_PIN_TIMER_INTERVAL; 

  // sensing loop start!! 
  x1 = analogRead(A0); // 아날로그값 읽기
  x1_f = lambda/(1+lambda)*x1+1/(1+lambda)*x1_fold; //필터된 값
  x1_fold = x1_f; // 센서 필터 이전값 업데이트
  x2 = analogRead(A1); // 아날로그값 읽기
  x2_f = lambda/(1+lambda)*x2+1/(1+lambda)*x2_fold; //필터된 값
  x2_fold = x2_f; // 센서 필터 이전값 업데이트
  ir1.data = x1_f;
  ir2.data = x2_f;
  Lidar.publish(&lidar);
  IR1.publish(&ir1);
  IR2.publish(&ir2);
  nh.spinOnce();
  delay(10);
  }
}
