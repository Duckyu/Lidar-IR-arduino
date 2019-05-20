#define USE_USBCON // Arduino Micro board
#include <ros.h>
#include <std_msgs/Float64.h>

ros::NodeHandle  nh;

float thre_ir1[16] = {751.1,514.5,394.2,319.9,277.7,244.5,221.6,202.8,188.5,174.7,165.0,156.1,149.6,141.0,134.9,127.9};
float thre_ir2[16] = {760.5,527.4,408.7,339.2,294.6,261.4,232.5,208.9,187.6,171.8,160.1,149.7,142.1,136.2,134.1,130.5};

double pW = 0;
double pW_f = 0;
double pW_fold = 0;

static long analogPinTimer = 0; 
// Set the sampling time
#define ANALOG_PIN_TIMER_INTERVAL 2 // milliseconds
unsigned long thisMillis_old;

int fc = 5; // cutoff frequency 5~10 Hz 정도 사용해보시기 바랍니다
double dt = ANALOG_PIN_TIMER_INTERVAL/1000.0; // sampling time
double lambda = 2*PI*fc*dt;
//double lambda = 19; // 9 - 0.9, 19 - 0.95
double x1 = 0.0;
double x1_f = 0.0;
double x1_fold = 0.0;
double x2 = 0.0;
double x2_f = 0.0;
double x2_fold = 0.0;

std_msgs::Float64 lidar;
std_msgs::Float64 ir1;
std_msgs::Float64 ir2;
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
  
  unsigned long deltaMillis = 0; // clear last result
  unsigned long thisMillis = millis();  
  if (thisMillis != thisMillis_old) { 
    deltaMillis = thisMillis-thisMillis_old; 
    thisMillis_old = thisMillis;   
  } 
  
  analogPinTimer -= deltaMillis; 
  
  if (analogPinTimer <= 0) {  
    analogPinTimer += ANALOG_PIN_TIMER_INTERVAL; 

  pW = pulseIn(3, HIGH); // Count how long the pulse is high in microseconds
  pW = pW / 10; // 10usec = 1 cm of distance
  pW_f = lambda/(1+lambda)*pW+1/(1+lambda)*pW_fold; //필터된 값
  pW_fold = pW_f; // 센서 필터 이전값 업데이트
  
  // sensing loop start!!
  x1 = analogRead(A0); // 아날로그값 읽기
  x1_f = lambda/(1+lambda)*x1+1/(1+lambda)*x1_fold; //필터된 값
  x1_fold = x1_f; // 센서 필터 이전값 업데이트
  x2 = analogRead(A1); // 아날로그값 읽기
  x2_f = lambda/(1+lambda)*x2+1/(1+lambda)*x2_fold; //필터된 값
  x2_fold = x2_f; // 센서 필터 이전값 업데이트
//  ir1.data = x1_f;
//  ir2.data = x2_f;
  int index1 = 0;
  int index2 = 0;
  for(int i=0;i<9;i++){
    index1 += (x1_f<thre_ir1[i]);
    index2 += (x2_f<thre_ir2[i]);
  }
  lidar.data = pW_f;
  switch(index1){
    case 0:
      ir1.data = 0;
      break;
    case 9:
      ir1.data = 50;
      break;
    default:
      ir1.data = 5+5*index1+5*(thre_ir1[index1-1]-x1_f)/(thre_ir1[index1-1]-thre_ir1[index1]);
      break;
  }
  switch(index2){
    case 0:
      ir2.data = 0;
      break;
    case 9:
      ir2.data = 50;
      break;
    default:
      ir2.data = 5+5*index2+5*(thre_ir2[index2-1]-x2_f)/(thre_ir2[index2-1]-thre_ir2[index2]);
      break;
  }
  Lidar.publish(&lidar);
  IR1.publish(&ir1);
  IR2.publish(&ir2);
  nh.spinOnce();
  delay(10);
  }
}
