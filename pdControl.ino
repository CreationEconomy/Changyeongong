#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0

// Framework setting
#define _DIST_TARGET 255
#define _DIST_MIN 100
#define _DIST_MAX 410

// Distance sensor
#define _DIST_ALPHA 0.3    

// Servo range
#define _DUTY_MIN 700
#define _DUTY_NEU 1350
#define _DUTY_MAX 2050

// Servo speed control
#define _SERVO_ANGLE 30
#define _SERVO_SPEED 600
  
// Event periods
#define _INTERVAL_DIST 20
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100

// PID parameters
#define _KP 2.5
#define _KD 150.0

//////////////////////
// global variables //
//////////////////////

// Servo instance 
Servo myservo;

// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema;    //측정값, ema필터 적용값

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial; //거리, 서보, 시리얼 측정 여부

// Servo speed control
int duty_chg_per_interval; // 한 주기 당 제어할 최대 duty값
int duty_target, duty_curr; // 목표 pulse주기값, 현재 pulse주기값

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

// 중간값 필터
#define midFilter 9
float dist_arr[midFilter];
int idx = 0;

//r
int period = 30;


void setup() {
  randomSeed(analogRead(5));
// initialize GPIO pins for LED and attach servo 
pinMode(PIN_LED,OUTPUT);
myservo.attach(PIN_SERVO);

// move servo to neutral position
duty_target = duty_curr = _DUTY_NEU; // 초기화
myservo.writeMicroseconds(duty_curr); // 서보 중립위치

// initialize serial port
Serial.begin(57600);

// initialize global variables 
dist_target = _DIST_TARGET;
dist_raw = dist_ema = dist_arr[idx] = ir_distance();
++idx;
error_curr = error_prev = dist_target - dist_raw;

last_sampling_time_dist = 0; // last_sampling_time_dist 초기화
last_sampling_time_servo = 0;
last_sampling_time_serial = 0;

event_dist = event_servo = event_serial = false;

// convert angle speed into duty change per interval.
  duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / ((float)_SERVO_ANGLE) * _INTERVAL_SERVO / 1000; //한 주기 당 제어할 최대 duty값 초기화
}
  


void loop() {
/////////////////////
// Event generator //
/////////////////////
  unsigned long time_curr = millis();
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        period = time_curr - last_sampling_time_dist;
        event_dist = true;
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
  } 



////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
     event_dist = false;
  // get a distance reading from the distance sensor
      dist_raw = ir_distance_filtered();
      dist_ema = (1 - _DIST_ALPHA) * dist_ema + _DIST_ALPHA * dist_raw;

  // PID control logic
    error_curr = dist_target - dist_ema;
    pterm = _KP * error_curr;
    dterm = _KD * (error_curr - error_prev);
    
    //dterm /= period;
    control = pterm + dterm;

  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control;

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
  if(duty_target < _DUTY_MIN){
    duty_target = _DUTY_MIN;
  }
  if(duty_target > _DUTY_MAX){
    duty_target = _DUTY_MAX;
  }

   error_prev = error_curr;  // error update
  }
  
  if(event_servo) {
event_servo = false;

    // adjust duty_curr toward duty_target by duty_chg_per_interval

    // update servo position

// adjust duty_curr toward duty_target by duty_chg_per_interval
  if(duty_target > duty_curr) {
  duty_curr += duty_chg_per_interval;
  if(duty_curr > duty_target) duty_curr = duty_target;
  }
  else {
  duty_curr -= duty_chg_per_interval;
  if(duty_curr < duty_target) duty_curr = duty_target;
  
  }
  
}
  // update servo position
  myservo.writeMicroseconds(duty_curr);
  

  if(event_serial) {
    event_serial = false;
//    Serial.print("raw_dist:");
//    Serial.print(ir_distance());
    Serial.print("dist_ir:");
    Serial.print(dist_ema);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",dterm:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
// Min :측정 최소거리, Max : 최대거리, Low : 목표구역 최소거리, dist_target :기준이 되는 거리, High : 목표구역 최대거리 값을 시리얼 모니터에 표시

  }
}

float ir_distance(void){ // return value unit: mm
  int a = 87;
  int b = 315;
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  val = 100 + 300.0 / (b - a) * (val - a);
  return val;
}

float median(float a, float b, float c){
    if (a > b){
        if (b > c)          return b;
        else if (a > c)     return c;
        else                return a;
    }
    else{
        if (a > c)          return a;
        else if (b > c)     return c;
        else                return b;
    }
}

float ir_distance_filtered(void){ // return value unit: mm
  float raw =  ir_distance();
  dist_arr[idx] = raw;
  ++idx;
  if(idx>=midFilter) idx = 0;
  float a = median(dist_arr[0], dist_arr[1], dist_arr[2]);
  float b = median(dist_arr[3], dist_arr[4], dist_arr[5]);
  float c = median(dist_arr[6], dist_arr[7], dist_arr[8]);
  return median(a, b, c);
}
