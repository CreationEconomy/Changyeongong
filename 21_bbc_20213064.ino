#include <Servo.h>
#define PIN_SERVO 10
#define PIN_IR A0
#define _DUTY_MIN 700
#define _DUTY_NEU 1400
#define _DUTY_MAX 2050
float dist_filter;
int cnt1 = 0, cnt2 = 0;
Servo myservo;

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(A0));
  val = ((6762.0/(volt-9.0))-4) * 10.0;
  float dist_cali = 300.0/258.0*(val-86.0)+100;
  if(dist_cali > 430) dist_cali=430;
  dist_filter = 0.2*dist_cali + 0.8*dist_filter;
  return dist_filter;
}

void setup() {
  myservo.attach(PIN_SERVO); 
  Serial.begin(57600);
}

void loop() {
  float dis = ir_distance();
  Serial.println("Min: 100, ");
  Serial.println("Dis: ");
  Serial.println(dis);
  Serial.println(", Max: 450, ");
  if(dis<250) cnt1++;
  else if(dis>250)cnt2++;
  if(cnt1>10){
    myservo.writeMicroseconds(_DUTY_NEU + 200);
    cnt1 = 0;
  }
  if(cnt2>10){
    myservo.writeMicroseconds(_DUTY_NEU - 200);
    cnt2 = 0;
  }
}
