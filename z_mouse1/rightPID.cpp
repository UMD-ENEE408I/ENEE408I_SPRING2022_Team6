#include <Arduino.h>
#include <Adafruit_MCP3008.h> 
#include <Adafruit_MPU6050.h>
#include <Encoder.h>

#define GEAR_RATIO 29.86 //motor spec
#define ROTATION 360 //ticks for one wheel rotation
#define WHEEL_RADIUS 16 //mm

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

const float M_I_COUNTS_TO_A = (3.3 / 1024.0) / 0.120;

const unsigned int MAX_PWM_VALUE = 255; // Max PWM given 8 bit resolution
const unsigned int MIN_PWM_VALUE = 60;
unsigned int BASE_PWM = 100;

const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;

/*BEGIN basic motor movements*/
void M_LEFT_backward(int pwm) {
  ledcWrite(M_LEFT_IN_1_CHANNEL, pwm);
  ledcWrite(M_LEFT_IN_2_CHANNEL, 0);
}

void M_LEFT_forward(int pwm) {
  ledcWrite(M_LEFT_IN_1_CHANNEL, 0);
  ledcWrite(M_LEFT_IN_2_CHANNEL, pwm);
}

void M_LEFT_stop() {
  ledcWrite(M_LEFT_IN_1_CHANNEL, 0);
  ledcWrite(M_LEFT_IN_2_CHANNEL, 0);
}

void M_RIGHT_backward(int pwm) {
  ledcWrite(M_RIGHT_IN_1_CHANNEL, pwm);
  ledcWrite(M_RIGHT_IN_2_CHANNEL, 0);
}

void M_RIGHT_forward(int pwm) {
  ledcWrite(M_RIGHT_IN_1_CHANNEL, 0);
  ledcWrite(M_RIGHT_IN_2_CHANNEL, pwm);
}

void M_RIGHT_stop() {
  ledcWrite(M_RIGHT_IN_1_CHANNEL, 0);
  ledcWrite(M_RIGHT_IN_2_CHANNEL, 0);
}

void stopMove() {
  M_LEFT_stop();
  M_RIGHT_stop();
}

void moveForward(int pwm) {
  M_LEFT_forward(pwm);
  M_RIGHT_forward(pwm);
}

void moveBackward(int pwm) {
  M_LEFT_backward(pwm);
  M_RIGHT_backward(pwm);
}

void turnLeft(int pwm) {
  M_LEFT_stop();
  M_RIGHT_forward(pwm);
}

void turnRight(int pwm) {
  M_RIGHT_stop();
  M_LEFT_forward(pwm);
}
/*END basic motor movements*/

/*PID Control*/
long prevTime = 0;
int prevPos = 0;

float targetVel = 10;  //in cm/s
float targetPos = 0;
int targetDist = 50; //desired distance in cm
float finalPos = 0; //desired endpoint in encoder ticks

float currentError = 0;
float integral = 0;
float derivative = 0;
float prevError = 0;

boolean initialStop = true;

int endFlag = 0;

//PID constants
float Kp = 1.02;
float Ki = 0.8;
float Kd = 0.1;

void setup() {
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);
  delay(100);
  
  Serial.begin(115200);
  
  ledcSetup(M_LEFT_IN_1_CHANNEL, freq, resolution);
  ledcSetup(M_LEFT_IN_2_CHANNEL, freq, resolution);
  ledcSetup(M_RIGHT_IN_1_CHANNEL, freq, resolution);
  ledcSetup(M_RIGHT_IN_2_CHANNEL, freq, resolution);

  ledcAttachPin(M_LEFT_IN_1, M_LEFT_IN_1_CHANNEL);
  ledcAttachPin(M_LEFT_IN_2, M_LEFT_IN_2_CHANNEL);
  ledcAttachPin(M_RIGHT_IN_1, M_RIGHT_IN_1_CHANNEL);
  ledcAttachPin(M_RIGHT_IN_2, M_RIGHT_IN_2_CHANNEL);

  pinMode(M_LEFT_I_SENSE, INPUT);
  pinMode(M_RIGHT_I_SENSE, INPUT);
  
  delay(3000);
  prevPos = 0;  //get start position
  prevTime = micros();  //get start time in us
  finalPos = targetDist/(2*PI*WHEEL_RADIUS/10)*ROTATION + prevPos; //convert desired distance to encoder value
}

void loop() {
  //create encoder objects in loop to avoid throwing exception
  Encoder encL(M_LEFT_ENC_A, M_LEFT_ENC_B); //left wheel, forward pos
  Encoder encR(M_RIGHT_ENC_A, M_RIGHT_ENC_B); //right wheel, forward neg

  encR.write(0);
  delay(10);
   
  while(endFlag != 1) {  //"actual main loop"
    long currentTime = micros(); //time in us
    int currentPos = -encR.read();
    float deltaTime = ((float) (currentTime - prevTime))/1.0e6; //delta time in s
    float currentVel = ((float)(currentPos - prevPos)/ROTATION)/deltaTime; //rev per sec
    float metricVel = currentVel*2*PI*WHEEL_RADIUS/10; //in cm/s
    if(metricVel != 0) { //don't change setpoint until we start moving 
      initialStop = false;
    }
    Serial.print(metricVel); Serial.print(','); //for tuning PID
    Serial.print(targetVel); Serial.println();

    //calculate desired position (ticks)
    float deltaPos = (targetVel*ROTATION/(2*PI*WHEEL_RADIUS/10))*deltaTime; //pos increment if going at this speed
    targetPos = targetPos + deltaPos;

    //calculate control values
    currentError = targetPos - currentPos;
    integral = integral + currentError*deltaTime;
    //derivative = (currentError - prevError)/deltaTime;
    derivative = -(currentPos-prevPos)/deltaTime;

    float u = Kp*currentError + Ki*integral + Kd*derivative;
  
    /*Serial.print("Current Position: "); Serial.print(currentPos); Serial.println();
    Serial.print("Delta Position: "); Serial.print(deltaPos); Serial.println();
    Serial.print("Target Position: "); Serial.print(targetPos); Serial.println();
    Serial.print("Error: "); Serial.print(currentError); Serial.println();
    Serial.print("Velocity: "); Serial.print(currentVel); Serial.println();*/
  
    //update variables
    prevPos = currentPos;
    prevTime = currentTime;
    prevError = currentError;

    //set motor power
    if (u > MAX_PWM_VALUE) { //if too large, cap
      u = MAX_PWM_VALUE;
    }
    else if (u <= 0) {
      u = 0;
    }

    //Serial.print("Control: "); Serial.print(u); Serial.println();
    //Serial.println(); Serial.println();
    M_RIGHT_forward(u);  
    delay(10);
    currentPos = -encR.read(); 
    /*if (currentPos >= finalPos) {
      stopMove();
      endFlag = 1;
    }*/
  }
}