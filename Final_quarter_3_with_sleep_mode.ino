#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <RadarSensor.h>

#define RX_PIN 16
#define TX_PIN 17
unsigned long prevsensorread = 0;
const unsigned long READ_INTERVAL   = 500;
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);
RadarSensor radar(&Serial2);

//Sleep Mode
unsigned long lastInRangeTime = 0;          // last millis() when target was in 0.7–2.0 m
const unsigned long SLEEP_TIMEOUT = 40000;  // 15 seconds
bool sleepMode = false;

unsigned long targetDetectedTime = 0;
bool targetConfirmed = false;
bool waitingForConfirmation = false;
String receivedMessage = "";
float x_target = 0.0;
float y_target = 0.0;
float target_distance=0.0;
float target_angle;
float angle_tolerance=4;
#define SERVOMIN 150       
#define SERVOMAX 600      
#define SERVO_FREQ 50     
#define NUM_CHICKENS 32
#define UPDATE_INTERVAL 10 
#define DEBOUNCE_INTERVAL 200 
unsigned long ledTimer = 0;
typedef struct {
  float x_coordinate;
  float y_coordinate;
  float chicken_Angle;
} chicken_t;

struct chickenState {
  int currentPos;         
  int targetPos;           
  int8_t direction;       
};

static const chicken_t points[NUM_CHICKENS] = {
{  0.0f,     0.26f,   90.0f },//0x40
  {  0.0f,     0.46f,   90.0f  },
  {  -0.145f,   0.442f,  108.16f },
  {  -0.143f,   0.34f,   112.81f },
  {  -0.0f,     0.36f,   90.0f },
  {  -0.140f,   0.235f,  120.8f  },
  {  -0.135f,   0.13f,   136.0f },
  {  0.0f,     0.0f,   0.0f  },
  { 0.0f,   0.0f,  0.0f  },
  {0.0f,     0.16f,   90.0f },
  {  -0.245f,   0.046f,  179.4f },
  {  -0.264f,   0.166f,  148.0f }, 
  {  -0.276f,   0.28f,   134.6f },
  {  -0.387f,   0.187f,  154.2f },
  {  -0.282f,   0.391f,  125.8f  },
  {  -0.405f,   0.309f,  142.6f },
  { 0.405f,   0.309f,  37.4f },//0x41 
  { 0.387f,   0.187f,  25.8f  },//sign inveted for all
  { 0.276f,   0.28f,   45.5f },
  { 0.125f,   0.02f,   9.1f  },
  { 0.264f,   0.165f,  32.2f  },
  { 0.245f,   0.046f,  10.7f },
  { 0.135f,   0.13f,   43.9f },
  {  0.0f,   0.0f,  0.0f },
  {  0.0f,   0.0f,  0.0f  },
   {  -0.125f,   0.02f,   9.1f  },
  {  0.0f,     0.06f,   90.0f  },
  {  0.0f,   0.0f,  0.0f  },
  { 0.14f,    0.235f,  59.2f },
  { 0.143f,   0.34f,   67.2f },
  { 0.282f,   0.39f,  54.2f  },  
  { 0.145f,   0.442f,  71.8f  }
 
 };
// static const chicken_t points[NUM_CHICKENS] = {
//   {  0.125f,   0.02f,   9.090f  },//top layer
//   {  0.0f,     0.06f,   90.0f  },
//   { -0.125f,   0.02f,   170.90f  },
//   {  0.245f,   0.046f,  10.6f },//layer2
//   {  0.135f,   0.13f,   44.04f },
//   {  0.0f,     0.16f,   90.0f },
//   { -0.135f,   0.13f,   136.1f },
//   { -0.245f,   0.046f,  169.36f },
//   {  0.264f,   0.166f,  32.1f },//layer3
//   {  0.140f,   0.235f,  59.2f  },
//   {  0.0f,     0.26f,   90.0f },
//   { -0.14f,    0.235f,  120.8f },
//   { -0.264f,   0.165f,  147.8f  },
//   {  0.387f,   0.187f,  25.8f },  //layer 4
//   {  0.276f,   0.28f,   45.4f },
//   {  0.143f,   0.34f,   67.18f },
//   {  0.0f,     0.36f,   90.0f },
//   { -0.143f,   0.34f,   112.8f },
//   { -0.276f,   0.28f,   134.58f },
//   { -0.387f,   0.187f,  154.2f  },
//   {  0.405f,   0.309f,  37.33f },//layer 5
//   {  0.282f,   0.391f,  54.2f  },
//   {  0.145f,   0.442f,  71.8f },
//   {  0.0f,     0.46f,   90.0f  },
//   { -0.145f,   0.442f,  108.16f  },
//   { -0.282f,   0.39f,  125.8f  },
//   { -0.405f,   0.309f,  142.65f }
//  };
chickenState servos[NUM_CHICKENS];

void enterSleepMode() {
  if (!sleepMode) {
    sleepMode = true;
    Serial.println("SLEEP MODE: No target for 15s — parking all servos.");

    for (uint8_t i = 0; i < NUM_CHICKENS; i++) {

      if(i==7 || i==8 || i==23 || i==24 || i==27) continue;

      servos[i].targetPos = SERVOMIN;

      if(servos[i].currentPos > SERVOMIN)
        servos[i].direction = -2;
      else
        servos[i].direction = 0;

    }
  }
}

unsigned long lastUpdate = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  

  //Radar Sensor Initialization

  Serial.println("Initializing RD-03D Radar...");
  
  // Initialize Serial2 hardware
  Serial2.begin(256000, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial2.setRxBufferSize(512);
  
  // Begin radar (baud already set in Serial2.begin)
  radar.begin(256000);
  
  Serial.println("Radar initialized!");




  // Initialize PCA9685
  pwm1.begin();
 pwm2.begin();
  pwm1.setPWMFreq(SERVO_FREQ);
  pwm2.setPWMFreq(SERVO_FREQ);
  // Initialize all servo states
  for (uint8_t i = 0; i < NUM_CHICKENS; i++) {
    if(i==7 || i==8 || i==23 || i==24 || i==27) continue;
    servos[i].currentPos = SERVOMAX;
    servos[i].targetPos = SERVOMIN;
    servos[i].direction = -2;  // Start moving forward
    
    // Set initial position for all servos
    if(i<=15){
      pwm1.setPWM(i, 0, SERVOMIN);
      }
    
  else {pwm2.setPWM(i-16, 0, SERVOMIN);}
    
  }
  lastInRangeTime = millis();
  Serial.println("All 27 servos initialized and running simultaneously");
}



void Target_read(){

    if(radar.update()) {
    RadarTarget tgt = radar.getTarget();
    
    if(tgt.detected) {
      x_target=tgt.x/1000.0;
      y_target=tgt.y/1000.0;
      target_angle=atan2(y_target, x_target)*(180.0/PI);
      Serial.print("X: "); 
      Serial.print(x_target);
      Serial.print(" meters | Y: "); 
      Serial.print(y_target);
      Serial.print(" meters | Dist: "); 
      Serial.print(target_angle);
      Serial.println(" Degree"); 
      
    } else {
      x_target=10.0;
      y_target=10.0;
      Serial.println("No target detected");
    }
    }
}


void loop() {
unsigned long now = millis();
if (now - prevsensorread>= READ_INTERVAL) {
    prevsensorread = now;
    Target_read();
  }
    

     
  if (now - lastUpdate < UPDATE_INTERVAL) return;
  lastUpdate = now;
 target_distance=sqrt(x_target*x_target+y_target*y_target);

 // SLEEP MODE: refresh timer whenever target is in range
  if (target_distance >= 0.7f && target_distance <= 2.0f) {
    lastInRangeTime = now;
    if (sleepMode) {
      sleepMode = false;
      targetConfirmed      = false;
      waitingForConfirmation = false;
      Serial.println("SLEEP MODE OFF: Target back in range.");
    }
  }

  // SLEEP MODE: check if timeout has elapsed
  if (!sleepMode && (now - lastInRangeTime >= SLEEP_TIMEOUT)) {
    enterSleepMode();
  }

  // SLEEP MODE: skip all motion logic while sleeping
  if (sleepMode) {

  for (uint8_t i = 0; i < NUM_CHICKENS; i++) {

    if(i==7 || i==8 || i==23 || i==24 || i==27) continue;

    chickenState &servo = servos[i];

    if (servo.currentPos > servo.targetPos) {
      servo.currentPos -= 2;
      if (servo.currentPos < servo.targetPos)
        servo.currentPos = servo.targetPos;
    }

    if(i<=15)
      pwm1.setPWM(i, 0, servo.currentPos);
    else
      pwm2.setPWM(i-16, 0, servo.currentPos);
  }

  return;
}

    if(target_distance<0.7 || target_distance>2.0){
          targetConfirmed = false;
    waitingForConfirmation = false;
    Serial.println("Target Search Mode");
    for (uint8_t channel = 0; channel < NUM_CHICKENS; channel++) {
      if(channel==7 || channel==8 ||channel==23 || channel==24 || channel==27 ) continue;
      chicken_normal_rotate(channel);
      
    }}
    else{
//Debounce added 

if(!waitingForConfirmation) {
      // First detection - start waiting
      targetDetectedTime = now;
      waitingForConfirmation = true;
      targetConfirmed = false;
      Serial.println("Target detected - waiting for confirmation...");
      
      // Keep searching while waiting
      Serial.println("Target Search Mode");
      for (uint8_t i = 0; i < NUM_CHICKENS; i++) {
         if(i==7 || i==8 || i==23 || i==24 || i==27) continue;
        chicken_normal_rotate(i);
      }
    }
else if(!targetConfirmed && (now - targetDetectedTime >= DEBOUNCE_INTERVAL)) {
      // After 1 second, check if still detected
      if(target_distance>=0.7 && target_distance <= 2.0) {
        targetConfirmed = true;
        Serial.println("Target CONFIRMED - tracking mode activated!");
      }
    }
    
    if(targetConfirmed) {
      // Confirmed target - track it
      
         for (uint8_t i=0;i<NUM_CHICKENS;i++){
    chickenState &servo = servos[i];
 if(i==7 || i==8 || i==23 || i==24 || i==27) continue;
    
    float CtoT_x=x_target-points[i].x_coordinate;
    float CtoT_y=y_target-points[i].y_coordinate;
   
    float final_angle=atan2(CtoT_y, CtoT_x)*(180.0/PI);
    final_angle=180-final_angle;
    if (final_angle > 90.0f) {
    final_angle = 90.0f + (final_angle - 90.0f) * 0.86f;
}
    
    float current_degree=(servo.currentPos-150)/2.5f;
    
    
   
    if (current_degree > final_angle+angle_tolerance ) {
      
    servo.currentPos -=2;// (final_angle*2.611)+140;
    servo.direction = -2;  // Reverse to backward
  } else if (current_degree < final_angle-angle_tolerance) {
     
    servo.currentPos +=2;// (final_angle*2.611)+140;
    servo.direction = 2;
  }
  else { 
    servo.direction = 0;   // Reverse to forward
  }
 if(i<=15){
    pwm1.setPWM(i, 0, servo.currentPos);
    }
    
  else {pwm2.setPWM(i-16, 0, servo.currentPos);}
    
  
  if (timer(1000, ledTimer)) {
    Serial.print("Chicken ");
    Serial.print(i);
    Serial.print(" to target x_axis");
Serial.println(CtoT_x);
Serial.print("Chicken ");
Serial.print(i);
Serial.print(" to target y_axis");
Serial.println(CtoT_y);
Serial.print("Final Angle: ");
Serial.println(final_angle);
Serial.print("Current_angle: ");
Serial.println(current_degree);
  }

        }
      
    }
    else {
      // Still waiting for confirmation - keep searching
      for (uint8_t i = 0; i < NUM_CHICKENS; i++) {
         if(i==7 || i==8 || i==23 || i==24 || i==27) continue;
        chicken_normal_rotate(i);
      }
    }
  }
}


    

  


void chicken_normal_rotate(uint8_t channel) {
  chickenState &servo = servos[channel];  // Reference to avoid copying
 // Serial.println("Target Search Mode");
  // Move servo one step in current direction
 if(servo.direction!=2 && servo.direction!=-2 ){
  servo.direction=2;
 }
    
    
  servo.currentPos += servo.direction;
  
  // Check if target reached and reverse direction
  if (servo.currentPos >= SERVOMAX) {
    servo.currentPos = SERVOMAX;
    servo.direction = -2;  // Reverse to backward
  } else if (servo.currentPos <= SERVOMIN) {
    servo.currentPos = SERVOMIN;
    servo.direction = 2;   // Reverse to forward
  }
 
  // Update PWM for this channel
   if(channel<=15){
    pwm1.setPWM(channel, 0, servo.currentPos);
    }
    
  else {pwm2.setPWM(channel-16, 0, servo.currentPos);}
  int degree=(servo.currentPos-150)/2.5f;
  if(channel==0 && degree%20==0 ){
  Serial.println(degree);
  }
}

  bool timer(unsigned long interval, unsigned long &lastTime) {
  unsigned long now = millis();
  if (now - lastTime >= interval) {
    lastTime = now;
    return true;
  }
  return false;
}