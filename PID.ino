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
#define _DIST_ALPHA 0.0 // EMA filter is disabled
// Servo range
#define _DUTY_MIN 1000
#define _DUTY_NEU 1450
#define _DUTY_MAX 2000
// Servo speed control
#define _SERVO_ANGLE 30 // angle b/w DUTY_MAX and DUTY_MIN
#define _SERVO_SPEED 100 // servo speed limit (deg/sec)
// Event periods
#define _INTERVAL_DIST 20 // distance sensor interval (ms)
#define _INTERVAL_SERVO 20 // servo interval (ms)
#define _INTERVAL_SERIAL 100 // serial interval (ms)
// PID parameters
#define _KP 0.55
#define _KD 40
#define _KI 0.002 // proportional gain *****
//////////////////////
// global variables //
//////////////////////
// Servo instance
Servo myservo;
// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema;
// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo,
last_sampling_time_serial;
bool event_dist, event_servo, event_serial;
// Servo speed control
int duty_chg_per_interval; // maximum duty difference per interval
int duty_target, duty_curr;
// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

void setup() {
// initialize GPIO pins for LED and attach servo
  pinMode(PIN_LED, OUTPUT);
  
// initialize global variables
duty_curr = _DUTY_MIN;
dist_target = _DIST_TARGET;

// move servo to neutral position
  myservo.attach(PIN_SERVO);
  duty_target = duty_curr = _DUTY_NEU;
  myservo.writeMicroseconds(duty_curr);

last_sampling_time_dist = last_sampling_time_servo = last_sampling_time_serial = 0;
event_dist = event_servo = event_serial = false;
// initialize serial port
Serial.begin(57600);
// convert angle speed into duty change per interval.
duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / 180 * _INTERVAL_SERVO/ 1000;
}

void loop() {
/////////////////////
// Event generator //
/////////////////////
unsigned long time_curr = millis();
if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
}
if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO){
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
}
if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL){
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
}

////////////////////
// Event handlers //
////////////////////
  if(event_dist) {
event_dist= false;
// get a distance reading from the distance sensor

  dist_raw = ir_distance();
  dist_raw = ir_distance_filtered();

// PID control logic
error_curr = dist_target - dist_raw;
pterm = _KP* error_curr;
dterm = _KD * (error_curr - error_prev);
iterm += _KI * error_curr;
control = pterm + dterm + iterm;
// duty_target = f(duty_neutral, control)
duty_target =_DUTY_NEU + control;
// keep duty_target value within the range of
// [_DUTY_MIN, _DUTY_MAX]
if(duty_target > _DUTY_MAX){duty_target = _DUTY_MAX;}
if(duty_target < _DUTY_MIN){duty_target = _DUTY_MIN;}
error_prev = error_curr;
}
if(event_servo) {
 event_servo =false;
// adjust duty_curr toward duty_target by duty_chg_per_interval
if(duty_target > duty_curr) {
  duty_curr += duty_chg_per_interval;
  if(duty_curr > duty_target) duty_curr = duty_target;
}
else{
  duty_curr -= duty_chg_per_interval;
  if(duty_curr < duty_target) duty_curr = duty_target;
}
// update servo position
myservo.writeMicroseconds(duty_curr);
}
if(event_serial) {
event_serial = false;
Serial.print("IR:");
Serial.print(dist_raw);
Serial.print(",T:");
Serial.print(dist_target);
Serial.print(",P:");
Serial.print(map(pterm,-1000,1000,510,610));
Serial.print(",D:");
Serial.print(map(dterm,-1000,1000,510,610));
Serial.print(", I:");
Serial.print(map(iterm, -1000,1000,510,610));
Serial.print(",DTT:");
Serial.print(map(duty_target,1000,2000,410,510));
Serial.print(",DTC:");
Serial.print(map(duty_curr,1000,2000,410,510));
Serial.println(",,-G:245,+G:265,m:0,M:800");
}}

float ir_distance(void){ // return value unit: mm
  float val;
  int a = 100;
  int b = 400;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  val = 100 + 300.0 / (_DIST_MAX - _DIST_MIN) *(val - _DIST_MIN);//[20213092] 오류수정
  val = 100 + 300.0 / (b-a) * (val - a);
  return val;
}
float ir_distance_filtered(void){ // return value unit: mm
// for now, just use ir_distance() without noise filter.
return ir_distance();
}
