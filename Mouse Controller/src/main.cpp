#include <Arduino.h>
#include <Adafruit_MCP3008.h> 
#include <Adafruit_MPU6050.h>
#include <Encoder.h>
#include "WiFi.h"
#include "HTTPClient.h"
#include "WebSocketsServer.h"

#define GEAR_RATIO 29.86 //motor spec
#define ROTATION 360 //ticks for one wheel rotation
#define WHEEL_RADIUS 16 //mm
#define THRESHOLD 630 //differentiate white line from gray foam

#define NUM_CALIBRATION_SAMPLES 100

#define STRAIGHT_THRESHOLD 6.0 //cm
#define STRAIGHT_DISTANCE 15.0 //cm
//#define CURVE_DISTANCE 15.0*2.0*PI/4.0
#define CURVE_DISTANCE 5.0 + (10.0*2.0*PI/4.0) + 5.0 //cm
#define NODE_READ_DISTANCE 2.5 //cm
#define WHEEL_TO_NODE_DISTANCE 8.5 //cm

#define F_PATH 0
#define FL_PATH 1
#define FR_PATH 2

#define LOWER_0 315
#define UPPER_0 45
#define LOWER_90 45
#define UPPER_90 135
#define LOWER_180 135
#define UPPER_180 225
#define LOWER_270 225
#define UPPER_270 315

int mouse = 1;  //select which mouse is running

Adafruit_MPU6050 mpu;
Adafruit_MCP3008 adc1;
Adafruit_MCP3008 adc2;

const unsigned int BUZZ = 26;
const unsigned int BUZZ_CHANNEL = 0;
const unsigned int octave = 4;

const unsigned int ADC_1_CS = 2;
const unsigned int ADC_2_CS = 17;

const unsigned int M_LEFT_ENC_A = 39;
const unsigned int M_LEFT_ENC_B = 38;
const unsigned int M_RIGHT_ENC_A = 37;
const unsigned int M_RIGHT_ENC_B = 36;

const unsigned int M_LEFT_IN_1 = 13;
const unsigned int M_LEFT_IN_2 = 12;
const unsigned int M_RIGHT_IN_1 = 25;
const unsigned int M_RIGHT_IN_2 = 14;

const unsigned int M_LEFT_IN_1_CHANNEL = 1;
const unsigned int M_LEFT_IN_2_CHANNEL = 2;
const unsigned int M_RIGHT_IN_1_CHANNEL = 3;
const unsigned int M_RIGHT_IN_2_CHANNEL = 4;

const unsigned int M_LEFT_I_SENSE = 35;
const unsigned int M_RIGHT_I_SENSE = 34;

const unsigned int MAX_PWM_VALUE = 255; // Max PWM given 8 bit resolution
const unsigned int MIN_PWM_VALUE = 60;

const float M_I_COUNTS_TO_A = (3.3 / 1024.0) / 0.120;

const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;

const char* ssid = "DerienHotspot";
const char* password =  "ENEE408I";

//PID Constants
float Kp_R = 0;
float Ki_R = 0;
float Kd_R = 0;

float Kp_L = 0;
float Ki_L = 0;
float Kd_L = 0;

int bit_buf[14]; //Reflectance Sensor Array
int leftSensors = 0;
int rightSensors = 0;
int emptyCheck = 0;

float kP_line = 0.02;
int line_error = 0;
float dist_adjust = 0.0;

sensors_event_t a, g, temp;

WebSocketsServer webSocket = WebSocketsServer(80);
HTTPClient http;




/* BEGIN Basic motor movements *************************************/

void M_RIGHT_backward(int pwm) {
  ledcWrite(M_RIGHT_IN_1_CHANNEL, pwm);
  ledcWrite(M_RIGHT_IN_2_CHANNEL, 0);
}

void M_RIGHT_forward(int pwm) {
  ledcWrite(M_RIGHT_IN_1_CHANNEL, 0);
  ledcWrite(M_RIGHT_IN_2_CHANNEL, pwm);
}

void M_LEFT_backward(int pwm) {
  ledcWrite(M_LEFT_IN_1_CHANNEL, pwm);
  ledcWrite(M_LEFT_IN_2_CHANNEL, 0);
}

void M_LEFT_forward(int pwm) {
  ledcWrite(M_LEFT_IN_1_CHANNEL, 0);
  ledcWrite(M_LEFT_IN_2_CHANNEL, pwm);
}

void stopMove() {
  // Stop right motor
  ledcWrite(M_RIGHT_IN_1_CHANNEL, 0);
  ledcWrite(M_RIGHT_IN_2_CHANNEL, 0);
  // Stop left motor
  ledcWrite(M_LEFT_IN_1_CHANNEL, 0);
  ledcWrite(M_LEFT_IN_2_CHANNEL, 0);
}

/* END Basic motor movements *************************************/




/* BEGIN Rotational Movement *************************************/

float calibrateMouseZ() {
  delay(500);
  float zOffset = 0.0;

  for (int i = 0; i < NUM_CALIBRATION_SAMPLES; i++) {
    mpu.getEvent(&a, &g, &temp);
    zOffset = zOffset + (g.gyro.z / NUM_CALIBRATION_SAMPLES);
    delay(5);
  }

  return zOffset;
}

float calibrateMouseX() {
  delay(500);
  float xOffset = 0.0;

  for (int i = 0; i < NUM_CALIBRATION_SAMPLES; i++) {
    mpu.getEvent(&a, &g, &temp);
    xOffset = xOffset + (a.acceleration.x / NUM_CALIBRATION_SAMPLES);
    delay(5);
  }

  return xOffset;
}

float calibrateMouseY() {
  delay(500);
  float yOffset = 0.0;

  for (int i = 0; i < NUM_CALIBRATION_SAMPLES; i++) {
    mpu.getEvent(&a, &g, &temp);
    yOffset = yOffset + (a.acceleration.y / NUM_CALIBRATION_SAMPLES);
    delay(5);
  }

  return yOffset;
}

void rotateLeft(int deg, Encoder &encL, Encoder &encR) {
  float zPosition = 0.0;
  float timeElapsed = 0.0;
  float zVel = 0.0;
  float zOffset = calibrateMouseZ();
  unsigned long prevTime_Rot = millis();

  /*PID Control*/
  float targetVel_L = 10;  //in cm/s
  float targetVel_R = 10;  //in cm/s
  float targetPos_L = 0;
  float targetPos_R = 0;

  float currentError_L = 0;
  float integral_L = 0;
  float derivative_L = 0;
  float prevError_L = 0;

  float currentError_R = 0;
  float integral_R = 0;
  float derivative_R = 0;
  float prevError_R = 0;

  long prevTime = micros();  //get start time in us
  encL.write(0); encR.write(0); //clear encoders
  delay(10);

  while (1) {
    long currentTime = micros(); //time in us
    int currentPos_L = encL.read() * -1; //backwards for left turn
    int currentPos_R = -encR.read();
    float deltaTime = ((float) (currentTime - prevTime))/1.0e6; //delta time in s

    //calculate desired position (ticks)
    float deltaPos_L = (targetVel_L*ROTATION/(2*PI*WHEEL_RADIUS/10))*deltaTime; //pos increment if going at this speed
    float deltaPos_R = (targetVel_R*ROTATION/(2*PI*WHEEL_RADIUS/10))*deltaTime; //pos increment if going at this speed

    //increment target position
    targetPos_L = targetPos_L + deltaPos_L;
    targetPos_R = targetPos_R + deltaPos_R;

    //calculate control values
    currentError_L = targetPos_L - currentPos_L;
    integral_L = integral_L + currentError_L*deltaTime;
    derivative_L = (currentError_L - prevError_L)/deltaTime;

    currentError_R = targetPos_R - currentPos_R;
    integral_R = integral_R + currentError_R*deltaTime;
    derivative_R = (currentError_R - prevError_R)/deltaTime;

    float u_L = Kp_L*currentError_L + Ki_L*integral_L + Kd_L*derivative_L;
    float u_R = Kp_R*currentError_R + Ki_R*integral_R + Kd_R*derivative_R;

    //update variables
    prevTime = currentTime;
    prevError_L = currentError_L;
    prevError_R = currentError_R;
    
    //set motor power
    if (u_L > MAX_PWM_VALUE) { //if too large, cap
      u_L = MAX_PWM_VALUE;
    }
    else if (u_L <= 0) {
      u_L = 0;
    }

    //set motor power
    if (u_R > MAX_PWM_VALUE) { //if too large, cap
      u_R = MAX_PWM_VALUE;
    }
    else if (u_R <= 0) {
      u_R = 0;
    }

    //drive motors
    M_RIGHT_forward(u_R); 
    M_LEFT_backward(u_L);  //rotate left backwards
    
    mpu.getEvent(&a, &g, &temp);
    zVel = (g.gyro.z - zOffset);
    timeElapsed = (millis() - prevTime_Rot);
    zPosition = zPosition + (zVel * timeElapsed / 1000);
    prevTime_Rot = millis();
    //Serial.println(zPosition);
    if (zPosition >= deg*DEG_TO_RAD) {
      stopMove();
      return;
    }
    delay(10);
  }
}

void rotateRight(int deg, Encoder &encL, Encoder &encR) {
  float zPosition = 0.0;
  float timeElapsed = 0.0;
  float zVel = 0.0;
  float zOffset = calibrateMouseZ();
  unsigned long prevTime_Rot = millis();

  /*PID Control*/
  float targetVel_L = 10;  //in cm/s
  float targetVel_R = 10;  //in cm/s
  float targetPos_L = 0;
  float targetPos_R = 0;

  float currentError_L = 0;
  float integral_L = 0;
  float derivative_L = 0;
  float prevError_L = 0;

  float currentError_R = 0;
  float integral_R = 0;
  float derivative_R = 0;
  float prevError_R = 0;

  long prevTime = micros();  //get start time in us
  encL.write(0); encR.write(0); //clear encoders
  delay(10);

  while (1) {
    long currentTime = micros(); //time in us
    int currentPos_L = encL.read();
    int currentPos_R = -encR.read() * -1; //backwards for right turn
    float deltaTime = ((float) (currentTime - prevTime))/1.0e6; //delta time in s

    //calculate desired position (ticks)
    float deltaPos_L = (targetVel_L*ROTATION/(2*PI*WHEEL_RADIUS/10))*deltaTime; //pos increment if going at this speed
    float deltaPos_R = (targetVel_R*ROTATION/(2*PI*WHEEL_RADIUS/10))*deltaTime; //pos increment if going at this speed

    //increment target position
    targetPos_L = targetPos_L + deltaPos_L;
    targetPos_R = targetPos_R + deltaPos_R;

    //calculate control values
    currentError_L = targetPos_L - currentPos_L;
    integral_L = integral_L + currentError_L*deltaTime;
    derivative_L = (currentError_L - prevError_L)/deltaTime;

    currentError_R = targetPos_R - currentPos_R;
    integral_R = integral_R + currentError_R*deltaTime;
    derivative_R = (currentError_R - prevError_R)/deltaTime;

    float u_L = Kp_L*currentError_L + Ki_L*integral_L + Kd_L*derivative_L;
    float u_R = Kp_R*currentError_R + Ki_R*integral_R + Kd_R*derivative_R;

    //update variables
    prevTime = currentTime;
    prevError_L = currentError_L;
    prevError_R = currentError_R;
    
    //set motor power
    if (u_L > MAX_PWM_VALUE) { //if too large, cap
      u_L = MAX_PWM_VALUE;
    }
    else if (u_L <= 0) {
      u_L = 0;
    }

    //set motor power
    if (u_R > MAX_PWM_VALUE) { //if too large, cap
      u_R = MAX_PWM_VALUE;
    }
    else if (u_R <= 0) {
      u_R = 0;
    }

    //drive motors
    M_LEFT_forward(u_L); 
    M_RIGHT_backward(u_R);  //rotate right backwards
    
    mpu.getEvent(&a, &g, &temp);
    zVel = (g.gyro.z - zOffset);
    timeElapsed = (millis() - prevTime_Rot);
    zPosition = zPosition + (zVel * timeElapsed / 1000);
    prevTime_Rot = millis();
    //Serial.println(zPosition);
    if (zPosition <= -deg*DEG_TO_RAD) {
      stopMove();
      return;
    }
    delay(10);
  }
}

/* END Rotational Movement *************************************/




/* BEGIN Linear Movement *************************************/

void senseLine(int *bit_buf) {
  leftSensors = 0;
  rightSensors = 0;
  emptyCheck = 0;
  int adc_buf[14];
  //read line sensor values, indices correspond to numbered sensors on the mouse
  for (int i = 0, j = 1; i < 8; i++) {
    adc_buf[j] = adc1.readADC(i);
    adc_buf[j+1] = adc2.readADC(i);
    j = j+2;
  }
  
  //convert to binary values, 0 = gray, 1 = white
  for (int i = 1; i < 14; i++) {
    if(adc_buf[i] < THRESHOLD) { //below threshold, white line
      bit_buf[i] = 1;
      if(i > 7) {
        leftSensors++;
      }
      else if (i < 7) {
        rightSensors++;
      }
    }
    else {
      bit_buf[i] = 0;
      emptyCheck++;
    }
  }
}

void PIDForward(float distance, Encoder &encL, Encoder &encR) {
  /*PID Control*/
  float targetVel_L = 10;  //in cm/s
  float targetVel_R = 10;  //in cm/s
  float targetPos_L = 0;
  float targetPos_R = 0;
  float finalPos_L = 0; //desired endpoint in encoder ticks
  float finalPos_R = 0; //desired endpoint in encoder ticks

  float currentError_L = 0;
  float integral_L = 0;
  float derivative_L = 0;
  float prevError_L = 0;

  float currentError_R = 0;
  float integral_R = 0;
  float derivative_R = 0;
  float prevError_R = 0;

  int prevPos_L = 0;  //get start position
  int prevPos_R = 0;  //get start position
  long prevTime = micros();  //get start time in us
  finalPos_L = (distance + dist_adjust)/(2*PI*WHEEL_RADIUS/10)*ROTATION + prevPos_L; //convert desired distance to encoder value
  finalPos_R = (distance + dist_adjust)/(2*PI*WHEEL_RADIUS/10)*ROTATION + prevPos_R; //convert desired distance to encoder value
  encL.write(0); encR.write(0); //clear encoders
  delay(10);

  while (1) {
    long currentTime = micros(); //time in us
    int currentPos_L = encL.read();
    int currentPos_R = -encR.read(); 
    float deltaTime = ((float) (currentTime - prevTime))/1.0e6; //delta time in s

    //calculate desired position (ticks)
    float deltaPos_L = (targetVel_L*ROTATION/(2*PI*WHEEL_RADIUS/10))*deltaTime; //pos increment if going at this speed
    float deltaPos_R = (targetVel_R*ROTATION/(2*PI*WHEEL_RADIUS/10))*deltaTime; //pos increment if going at this speed

    //increment target position
    targetPos_L = targetPos_L + deltaPos_L;
    targetPos_R = targetPos_R + deltaPos_R;

    //calculate control values
    currentError_L = targetPos_L - currentPos_L;
    integral_L = integral_L + currentError_L*deltaTime;
    derivative_L = (currentError_L - prevError_L)/deltaTime;

    currentError_R = targetPos_R - currentPos_R;
    integral_R = integral_R + currentError_R*deltaTime;
    derivative_R = (currentError_R - prevError_R)/deltaTime;

    float u_L = Kp_L*currentError_L + Ki_L*integral_L + Kd_L*derivative_L;
    float u_R = Kp_R*currentError_R + Ki_R*integral_R + Kd_R*derivative_R;

    //update variables
    prevTime = currentTime;
    prevPos_L = currentPos_L;
    prevPos_R = currentPos_R;
    prevError_L = currentError_L;
    prevError_R = currentError_R;
    
    //set motor power
    if (u_L > MAX_PWM_VALUE) { //if too large, cap
      u_L = MAX_PWM_VALUE;
    }
    else if (u_L <= 0) {
      u_L = 0;
    }

    //set motor power
    if (u_R > MAX_PWM_VALUE) { //if too large, cap
      u_R = MAX_PWM_VALUE;
    }
    else if (u_R <= 0) {
      u_R = 0;
    }

    //drive motors
    M_RIGHT_forward(u_R); 
    M_LEFT_forward(u_L);  

    //read reflectance sensors to stay on line
    senseLine(bit_buf);
    //proportional control for line following correction
    int sum = 0;
    int tally = 0;  //number of sensors active
    for(int i = 1; i < 14; i++) { //tally up all reflectance sensors
      if(bit_buf[i] == 1) {
        sum = sum + i;  //if that sensor is on, add its index
        tally++;
      }
    }
    float line_pos = (float)sum/tally; //average position on the line
    //Serial.print("Line Position: "); Serial.print(line_pos); Serial.println();
    int target_line = 7; //desired average is the center
    line_error = target_line - line_pos; //positive if mouse is too far left (right sensors predominant) 
    //Serial.print("Error: "); Serial.print(line_error); Serial.println();
    float adjust_vel = kP_line * line_error;
    //Serial.print("Adjustment: "); Serial.print(adjust_vel); Serial.println();
    if(line_error == 0 || emptyCheck < 3) {  //on track, reset
      targetVel_L = 10;
      targetVel_R = 10;
    }
    else {
      targetVel_L += adjust_vel;  //adjust is pos --> left needs to be pos.
      targetVel_R -= adjust_vel;  //adjust is neg --> right needs to be pos. (double neg)
    }
    if(emptyCheck == 13) { //no sensors active, off line, stop
      targetVel_L = 10;
      targetVel_R = 10;
    }
    emptyCheck = 0;
    
    currentPos_L = encL.read(); 
    currentPos_R = -encR.read(); 
    if (currentPos_R >= finalPos_R || currentPos_L >= finalPos_L) {
      stopMove();
      return;
    }
    delay(10);
  }
}

void PIDBackward(float distance, Encoder &encL, Encoder &encR) {
  /*PID Control*/
  float targetVel_L = -10;  //in cm/s
  float targetVel_R = -10;  //in cm/s
  float targetPos_L = 0;
  float targetPos_R = 0;
  float finalPos_L = 0; //desired endpoint in encoder ticks
  float finalPos_R = 0; //desired endpoint in encoder ticks

  float currentError_L = 0;
  float integral_L = 0;
  float derivative_L = 0;
  float prevError_L = 0;

  float currentError_R = 0;
  float integral_R = 0;
  float derivative_R = 0;
  float prevError_R = 0;

  int prevPos_L = 0;  //get start position
  int prevPos_R = 0;  //get start position
  long prevTime = micros();  //get start time in us
  finalPos_L = -1 * ((distance + dist_adjust)/(2*PI*WHEEL_RADIUS/10)*ROTATION + prevPos_L); //convert desired distance to encoder value
  finalPos_R = -1 * ((distance + dist_adjust)/(2*PI*WHEEL_RADIUS/10)*ROTATION + prevPos_R); //convert desired distance to encoder value
  encL.write(0); encR.write(0); //clear encoders
  delay(10);

  while (1) {
    long currentTime = micros(); //time in us
    int currentPos_L = encL.read(); 
    int currentPos_R = -encR.read();
    float deltaTime = ((float) (currentTime - prevTime))/1.0e6; //delta time in s

    //calculate desired position (ticks)
    float deltaPos_L = (targetVel_L*ROTATION/(2*PI*WHEEL_RADIUS/10))*deltaTime; //pos increment if going at this speed
    float deltaPos_R = (targetVel_R*ROTATION/(2*PI*WHEEL_RADIUS/10))*deltaTime; //pos increment if going at this speed

    //increment target position
    targetPos_L = targetPos_L + deltaPos_L; //subtracts delta position for backwards movement
    targetPos_R = targetPos_R + deltaPos_R;

    //calculate control values
    currentError_L = currentPos_L - targetPos_L;
    integral_L = integral_L + currentError_L*deltaTime;
    derivative_L = (currentError_L - prevError_L)/deltaTime;

    currentError_R = currentPos_R - targetPos_R;
    integral_R = integral_R + currentError_R*deltaTime;
    derivative_R = (currentError_R - prevError_R)/deltaTime;

    float u_L = Kp_L*currentError_L + Ki_L*integral_L + Kd_L*derivative_L;
    float u_R = Kp_R*currentError_R + Ki_R*integral_R + Kd_R*derivative_R;

    //update variables
    prevTime = currentTime;
    prevPos_L = currentPos_L;
    prevPos_R = currentPos_R;
    prevError_L = currentError_L;
    prevError_R = currentError_R;
    
    //set motor power
    if (u_L > MAX_PWM_VALUE) { //if too large, cap
      u_L = MAX_PWM_VALUE;
    }
    else if (u_L <= 0) {
      u_L = 0;
    }

    //set motor power
    if (u_R > MAX_PWM_VALUE) { //if too large, cap
      u_R = MAX_PWM_VALUE;
    }
    else if (u_R <= 0) {
      u_R = 0;
    }

    //drive motors
    M_RIGHT_backward(u_R); 
    M_LEFT_backward(u_L);  
    
    currentPos_L = encL.read(); 
    currentPos_R = -encR.read(); 
    if (currentPos_R <= finalPos_R || currentPos_L <= finalPos_L) {
      stopMove();
      return;
    }
    delay(10);
  }
}


/* END Linear Movement *************************************/




/* BEGIN Instruction Handling *************************************/

void instructionHandler(char instruction, Encoder &encL, Encoder &encR){
  switch (instruction) {
    case 'F':
      PIDForward(3, encL, encR); //Move off node
      break;
    case 'L':
      PIDForward(WHEEL_TO_NODE_DISTANCE, encL, encR);
      rotateLeft(88,encL,encR);
      PIDBackward(WHEEL_TO_NODE_DISTANCE - 3, encL, encR);
      break;
    case 'R':
      PIDForward(WHEEL_TO_NODE_DISTANCE, encL, encR);
      rotateRight(88,encL,encR);
      PIDBackward(WHEEL_TO_NODE_DISTANCE - 3, encL, encR);
      break;
    case 'B':
      PIDForward(WHEEL_TO_NODE_DISTANCE, encL, encR);
      rotateRight(178,encL,encR);
      PIDBackward(WHEEL_TO_NODE_DISTANCE - 3, encL, encR);
      break;
    case 'Y': 
      ledcWriteNote(BUZZ_CHANNEL, NOTE_B, octave);
      delay(150);
      ledcWriteNote(BUZZ_CHANNEL, NOTE_E, octave + 1);
      delay(150);
      ledcWrite(BUZZ_CHANNEL, 0);
      delay(5000);
      break;
    case 'N':
      ledcWriteNote(BUZZ_CHANNEL, NOTE_A, octave);
      delay(100);
      ledcWrite(BUZZ_CHANNEL, 0);
      delay(50);
      ledcWriteNote(BUZZ_CHANNEL, NOTE_A, octave);
      delay(100);
      ledcWrite(BUZZ_CHANNEL, 0);
      delay(5000);
      break;
    case 'S':
      ledcWriteNote(BUZZ_CHANNEL, NOTE_B, octave);
      delay(150);
      ledcWriteNote(BUZZ_CHANNEL, NOTE_E, octave + 1);
      delay(150);
      ledcWriteNote(BUZZ_CHANNEL, NOTE_B, octave);
      delay(150);
      ledcWriteNote(BUZZ_CHANNEL, NOTE_E, octave + 1);
      delay(150);
      ledcWriteNote(BUZZ_CHANNEL, NOTE_B, octave);
      delay(150);
      ledcWriteNote(BUZZ_CHANNEL, NOTE_E, octave + 1);
      delay(150);
      delay(5000);
      break;
    case 'W':
      //TODO: Add W sound
      delay(5000);
      break;
  }
}

/* END Instruction Handling *************************************/




/* SETUP *************************************/

void setup() {
  // Sound
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);
  delay(100);
  ledcAttachPin(BUZZ, BUZZ_CHANNEL);
  
  // Serial
  Serial.begin(115200);

  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  
  // Motor
  ledcSetup(M_RIGHT_IN_1_CHANNEL, freq, resolution);
  ledcSetup(M_RIGHT_IN_2_CHANNEL, freq, resolution);
  ledcSetup(M_LEFT_IN_1_CHANNEL, freq, resolution);
  ledcSetup(M_LEFT_IN_2_CHANNEL, freq, resolution);

  ledcAttachPin(M_RIGHT_IN_1, M_RIGHT_IN_1_CHANNEL);
  ledcAttachPin(M_RIGHT_IN_2, M_RIGHT_IN_2_CHANNEL);
  ledcAttachPin(M_LEFT_IN_1, M_LEFT_IN_1_CHANNEL);
  ledcAttachPin(M_LEFT_IN_2, M_LEFT_IN_2_CHANNEL);

  pinMode(M_RIGHT_I_SENSE, INPUT);
  pinMode(M_LEFT_I_SENSE, INPUT);

  // ADC
  adc1.begin(ADC_1_CS);  
  adc2.begin(ADC_2_CS);

  // MPU
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  //switch to select mouse
  switch (mouse) {
    case 1: 
      Kp_R = 1.02;
      Ki_R = 0.8;
      Kd_R = 0.1;

      Kp_L = 1.86;
      Ki_L = 0.5;
      Kd_L = 0.1;
      dist_adjust = -0.2;
      break;

    case 2:
      Kp_R = 0.96;
      Ki_R = 1;
      Kd_R = 0.1;

      Kp_L = 1.62;
      Ki_L = 1;
      Kd_L = 0.1;
      dist_adjust = -0.5;
      break;

    case 3:
      Kp_R = 1.56;
      Ki_R = 1;
      Kd_R = 0.1;

      Kp_L = 1.62;
      Ki_L = 1.2;
      Kd_L = 0.1;
      dist_adjust = -0.3;
      break;

    default: 
      break;
  }

  // WiFi
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}




/* MAIN LOOP *************************************/

void loop() {

  Encoder encR(M_RIGHT_ENC_A, M_RIGHT_ENC_B); //right wheel, forward neg
  Encoder encL(M_LEFT_ENC_A, M_LEFT_ENC_B); //left wheel, forward pos

  auto onWebSocketEvent = [&encL = encL, 
                          &encR = encR]
                          (uint8_t num, WStype_t type, uint8_t * payload, size_t length) {

    float zOffset = calibrateMouseZ();
    float zPosition = 0.0;
    float xPosition = 0.0;

    switch (type) {
      case WStype_DISCONNECTED: {
        //Serial.printf("[%u] Disconnected!", num);
        break;
      }

      case WStype_CONNECTED: {
        IPAddress ip = webSocket.remoteIP(num);
        //Serial.printf("[%u] Connection from ", num);
        //Serial.printf(ip.toString().c_str());
        break;
      }

      case WStype_TEXT: {
        //Serial.printf("[%u] Text: %s\n", num, payload);
        //Serial.println();
        
        String instructions = String((char *)payload);
        bool endFlag = false;

        switch (instructions[0]) { //Handle first instruction
          case 'E':
            instructions.remove(0);
            PIDForward(7, encL, encR);
            break;
          
          case 'Y':
          case 'N':
          case 'S':
            instructionHandler(instructions[0], encL, encR);
            instructions.remove(0);
            if (instructions.length() == 0) {
              endFlag = true;
            } else {
              PIDForward(4, encL, encR);
              instructionHandler(instructions[0], encL, encR);
              instructions.remove(0);
            }
            break;
          
          case 'L':
          case 'R':
          case 'B':
            PIDForward(4, encL, encR);
            instructionHandler(instructions[0], encL, encR);
            instructions.remove(0);
            break;
          
          case 'F':
            PIDForward(7, encL, encR);
            instructions.remove(0);
            break;


          default:
            break;
        }

        String path = "";

        while (!endFlag) {
          int pathType = F_PATH;
          int orientation = 0;
          bool straightDistanceReset = true;
          bool curveDistanceReset = true;

          float timeElapsed = 0.0;
          float zVel = 0.0;
          unsigned long prevTime_Rot = millis();

          //PID Control
          float targetVel_L = 10;  //in cm/s
          float targetVel_R = 10;  //in cm/s
          float targetPos_L = 0;
          float targetPos_R = 0;

          float currentError_L = 0;
          float integral_L = 0;
          float derivative_L = 0;
          float prevError_L = 0;

          float currentError_R = 0;
          float integral_R = 0;
          float derivative_R = 0;
          float prevError_R = 0;

          long prevTime = micros();  //get start time in us
          encL.write(0); encR.write(0); //clear encoders
          float totalDistance = 0.0;
          delay(10);

          //Move forward until interupt or until path needs to be sent

          while (1) {
            long currentTime = micros(); //time in us
            int currentPos_L = encL.read(); //actually right
            int currentPos_R = -encR.read(); // actually left
            float deltaTime = ((float) (currentTime - prevTime))/1.0e6; //delta time in s

            //calculate desired position (ticks)
            float deltaPos_L = (targetVel_L*ROTATION/(2*PI*WHEEL_RADIUS/10))*deltaTime; //pos increment if going at this speed
            float deltaPos_R = (targetVel_R*ROTATION/(2*PI*WHEEL_RADIUS/10))*deltaTime; //pos increment if going at this speed

            //increment target position
            targetPos_L = targetPos_L + deltaPos_L;
            targetPos_R = targetPos_R + deltaPos_R;

            //calculate control values
            currentError_L = targetPos_L - currentPos_L;
            integral_L = integral_L + currentError_L*deltaTime;
            derivative_L = (currentError_L - prevError_L)/deltaTime;

            currentError_R = targetPos_R - currentPos_R;
            integral_R = integral_R + currentError_R*deltaTime;
            derivative_R = (currentError_R - prevError_R)/deltaTime;

            float u_L = Kp_L*currentError_L + Ki_L*integral_L + Kd_L*derivative_L;
            float u_R = Kp_R*currentError_R + Ki_R*integral_R + Kd_R*derivative_R;

            //update variables
            prevTime = currentTime;
            prevError_L = currentError_L;
            prevError_R = currentError_R;
            
            //set motor power
            if (u_L > MAX_PWM_VALUE) { //if too large, cap
              u_L = MAX_PWM_VALUE;
            }
            else if (u_L <= 0) {
              u_L = 0;
            }

            //set motor power
            if (u_R > MAX_PWM_VALUE) { //if too large, cap
              u_R = MAX_PWM_VALUE;
            }
            else if (u_R <= 0) {
              u_R = 0;
            }

            //drive motors
            M_RIGHT_forward(u_R); 
            M_LEFT_forward(u_L);  

            //read reflectance sensors to stay on line
            senseLine(bit_buf);
            //proportional control for line following correction
            int sum = 0;
            int tally = 0;  //number of sensors active
            for(int i = 1; i < 14; i++) { //tally up all reflectance sensors
              if(bit_buf[i] == 1) {
                sum = sum + i;  //if that sensor is on, add its index
                tally++;
              }
            }
            float line_pos = (float)sum/tally; //average position on the line
            //Serial.print("Line Position: "); Serial.print(line_pos); Serial.println();
            int target_line = 7; //desired average is the center
            line_error = target_line - line_pos; //positive if mouse is too far left (right sensors predominant) 
            //Serial.print("Error: "); Serial.print(line_error); Serial.println();
            float adjust_vel = kP_line * line_error;
            //Serial.print("Adjustment: "); Serial.print(adjust_vel); Serial.println();
            if(line_error == 0 || emptyCheck < 3) {  //on track, reset
              targetVel_L = 10;
              targetVel_R = 10;
            }
            else {
              targetVel_L += adjust_vel;  //adjust is pos --> left needs to be pos.
              targetVel_R -= adjust_vel;  //adjust is neg --> right needs to be pos. (double neg)
            }
            
            currentPos_L = encL.read();
            currentPos_R = -encR.read();

            //Get gyro measurements
            mpu.getEvent(&a, &g, &temp);
            timeElapsed = (millis() - prevTime_Rot);
            prevTime_Rot = millis();

            zVel = g.gyro.z - zOffset;
            zPosition = zPosition + (zVel * timeElapsed / 1000.0);

            xPosition = (currentPos_L + currentPos_R) * 0.5 / ROTATION * (2*PI*WHEEL_RADIUS/10) - dist_adjust;

            //Serial.print("X: "); Serial.print(xPosition); Serial.println();

            //If mouse hits junction with left or right path and no instructions
            if ((leftSensors > 3 || rightSensors > 3) && instructions.length() == 0) {
              //Serial.println("J w/o I");
              stopMove();
              delay(10);
              PIDBackward(4, encL, encR);
              emptyCheck = 0;
              path = path + "J"; //Junction
              endFlag = true;
              break;

            //If mouse hits junction with left or right path and has instructions
            } else if (leftSensors > 3 || rightSensors > 3) {
              //Serial.println("J w/ I");
              stopMove();
              char instruction = instructions[0];
              instructionHandler(instruction,encL,encR);
              instructions.remove(0);
              emptyCheck = 0;
              if ((instruction == 'Y' || instruction == 'N' || instruction == 'S') && instructions.length() == 0) {
                endFlag = true;
              }
              break;
            
            //If mouse hits dead end
            } else if (emptyCheck == 13) {
              //Serial.println("D");
              stopMove();
              delay(10);
              PIDBackward(4, encL, encR);
              emptyCheck = 0;
              path = path + "D"; //Dead end
              endFlag = true;
              break;
            
            //If no junctions, dead ends, or instructions, move forward
            } else if (instructions.length() == 0) {
              //Serial.println("F");
              int zPositionCheck = (int)(zPosition*RAD_TO_DEG)%360;

              if (curveDistanceReset && orientation == 0 && LOWER_270 < zPositionCheck && zPositionCheck < UPPER_270) {
                pathType = FR_PATH;
                curveDistanceReset = false;
                straightDistanceReset = true;
                path = path + "R";
                //Serial.println("R instruction");
                orientation = 270;
              } else if (curveDistanceReset && orientation == 0 && LOWER_90 < zPositionCheck && zPositionCheck < UPPER_90) {
                pathType = FL_PATH;
                curveDistanceReset = false;
                straightDistanceReset = true;
                path = path + "L";
                //Serial.println("L instruction");
                orientation = 90;
              } else if (curveDistanceReset && orientation == 90 && LOWER_0 < zPositionCheck && zPositionCheck < UPPER_0) {
                pathType = FR_PATH;
                curveDistanceReset = false;
                straightDistanceReset = true;
                path = path + "R";
                //Serial.println("R instruction");
                orientation = 0;
              } else if (curveDistanceReset && orientation == 90 && LOWER_180 < zPositionCheck && zPositionCheck < UPPER_180) {
                pathType = FL_PATH;
                curveDistanceReset = false;
                straightDistanceReset = true;
                path = path + "L";
                //Serial.println("L instruction");
                orientation = 180;
              } else if (curveDistanceReset && orientation == 180 && LOWER_90 < zPositionCheck && zPositionCheck < UPPER_90) {
                pathType = FR_PATH;
                curveDistanceReset = false;
                straightDistanceReset = true;
                path = path + "R";
                //Serial.println("R instruction");
                orientation = 90;
              } else if (curveDistanceReset && orientation == 180 && LOWER_270 < zPositionCheck && zPositionCheck < UPPER_270) {
                pathType = FL_PATH;
                curveDistanceReset = false;
                straightDistanceReset = true;
                path = path + "L";
                //Serial.println("L instruction");
                orientation = 270;
              } else if (curveDistanceReset && orientation == 270 && LOWER_180 < zPositionCheck && zPositionCheck < UPPER_180) {
                pathType = FR_PATH;
                curveDistanceReset = false;
                straightDistanceReset = true;
                path = path + "R";
                //Serial.println("R instruction");
                orientation = 180;
              } else if (curveDistanceReset && orientation == 270 && LOWER_0 < zPositionCheck && zPositionCheck < UPPER_0) {
                pathType = FL_PATH;
                curveDistanceReset = false;
                straightDistanceReset = true;
                path = path + "L";
                //Serial.println("L instruction");
                orientation = 0;
              } else if (straightDistanceReset && pathType == F_PATH && xPosition > (STRAIGHT_THRESHOLD + totalDistance)) {
                path = path + "F";
                curveDistanceReset = true;
                straightDistanceReset = false;
              } else if (!straightDistanceReset && pathType == F_PATH && xPosition > (STRAIGHT_DISTANCE + totalDistance)) {
                //Serial.println("Reset distance");
                totalDistance += STRAIGHT_DISTANCE;
                curveDistanceReset = true;
                straightDistanceReset = true;
              } else if (!curveDistanceReset && pathType != F_PATH && xPosition > (CURVE_DISTANCE + totalDistance)) {
                //Serial.println("Reset distance");
                totalDistance += CURVE_DISTANCE;
                curveDistanceReset = true;
                straightDistanceReset = true;
                pathType = F_PATH;
              }

              emptyCheck = 0;
              //Serial.print("Reset: "); Serial.print(curveDistanceReset); Serial.println();
              //Serial.print("Path: "); Serial.print(path); Serial.println();
            }
            
            delay(10);
          }
        }

        //Serial.print("Fin: "); Serial.print(path); Serial.println();
        webSocket.sendTXT(num, path);
        delay(100);
        break;
      }
      
      default:
        break;
    }
  };

  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  while(true) {
    webSocket.loop();
  }

}