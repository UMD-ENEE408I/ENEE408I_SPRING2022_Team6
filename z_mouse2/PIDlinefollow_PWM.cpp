#include <Arduino.h>
#include <Adafruit_MCP3008.h> 
#include <Adafruit_MPU6050.h>
#include <Encoder.h>

#define GEAR_RATIO 29.86 //motor spec
#define ROTATION 360 //ticks for one wheel rotation
#define WHEEL_RADIUS 16 //mm
#define threshold 630 //differentiate white line from gray foam 

Adafruit_MCP3008 adc1;
Adafruit_MCP3008 adc2;

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

const float M_I_COUNTS_TO_A = (3.3 / 1024.0) / 0.120;

const unsigned int MAX_PWM_VALUE = 255; // Max PWM given 8 bit resolution
const unsigned int MIN_PWM_VALUE = 60;
unsigned int BASE_PWM = 100;

const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;

/*BEGIN basic motor movements*/
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

void stopMove() {
  M_LEFT_stop();
  M_RIGHT_stop();
}

void moveForward(int pwm) {
  M_RIGHT_forward(pwm);
  M_LEFT_forward(pwm);
}

void moveBackward(int pwm) {
  M_RIGHT_backward(pwm);
  M_LEFT_backward(pwm);
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

//declaration
void senseLine(int *bit_buf);

/*Reflectance Sensor Array*/
int bit_buf[14];
int leftSide = 0;
int rightSide = 0;
int emptyCheck = 0;

/*PID Control*/
long prevTime = 0;
int prevPos_L = 0;
int prevPos_R = 0;

float targetVel = 10;  //in cm/s
int targetDist = 50; //desired distance in cm
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

boolean initialStop = true;

int endFlag = 0;

//PID constants
float Kp_R = 0.96;
float Ki_R = 1;
float Kd_R = 0.25;

float Kp_L = 1.62;
float Ki_L = 1;
float Kd_L = 0.25;

void setup() {
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);
  delay(100);
  
  Serial.begin(115200);
  
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

  adc1.begin(ADC_1_CS);  
  adc2.begin(ADC_2_CS);
  
  delay(3000);
  prevPos_L = 0;  //get start position
  prevPos_R = 0;  //get start position
  prevTime = micros();  //get start time in us
  finalPos_L = targetDist/(2*PI*WHEEL_RADIUS/10)*ROTATION + prevPos_L; //convert desired distance to encoder value
  finalPos_R = targetDist/(2*PI*WHEEL_RADIUS/10)*ROTATION + prevPos_R; //convert desired distance to encoder value
}

void loop() {
  //create encoder objects in loop to avoid throwing exception
  Encoder encR(M_RIGHT_ENC_A, M_RIGHT_ENC_B); //right wheel, forward neg
  Encoder encL(M_LEFT_ENC_A, M_LEFT_ENC_B); //left wheel, forward pos

  encL.write(0); encR.write(0); //clear encoders
  delay(10);
   
  while(endFlag != 1) {  //"actual main loop"
    long currentTime = micros(); //time in us
    int currentPos_L = encL.read(); //actually right
    int currentPos_R = -encR.read(); // actually left
    float deltaTime = ((float) (currentTime - prevTime))/1.0e6; //delta time in s
    float currentVel_L = ((float)(currentPos_L - prevPos_L)/ROTATION)/deltaTime; //rev per sec
    float metricVel_L = currentVel_L*2*PI*WHEEL_RADIUS/10; //in cm/s
    float currentVel_R = ((float)(currentPos_R - prevPos_R)/ROTATION)/deltaTime; //rev per sec
    float metricVel_R = currentVel_R*2*PI*WHEEL_RADIUS/10; //in cm/s
    /*Serial.print(metricVel_L); Serial.print(','); //for tuning PID
    Serial.print(metricVel_R); Serial.print(',');
    Serial.print(targetVel); Serial.println();*/

    //calculate desired position (ticks)
    float deltaPos = (targetVel*ROTATION/(2*PI*WHEEL_RADIUS/10))*deltaTime; //pos increment if going at this speed

    //increment target position
    targetPos_L = targetPos_L + deltaPos;
    targetPos_R = targetPos_R + deltaPos;

    //calculate control values
    currentError_L = targetPos_L - currentPos_L;
    integral_L = integral_L + currentError_L*deltaTime;
    derivative_L = (currentError_L - prevError_L)/deltaTime;
  
    currentError_R = targetPos_R - currentPos_R;
    integral_R = integral_R + currentError_R*deltaTime;
    derivative_R = (currentError_R - prevError_R)/deltaTime;

    float u_L = Kp_L*currentError_L + Ki_L*integral_L + Kd_L*derivative_L;
    float u_R = Kp_R*currentError_R + Ki_R*integral_R + Kd_R*derivative_R;
  
    /*Serial.print("Current Position_L: "); Serial.print(currentPos_L); Serial.println();
    Serial.print("Current Position_R: "); Serial.print(currentPos_R); Serial.println();    
    Serial.print("Delta Position: "); Serial.print(deltaPos); Serial.println();
    Serial.print("Final Position_L: "); Serial.print(finalPos_L); Serial.println();
    Serial.print("Final Position_R: "); Serial.print(finalPos_R); Serial.println();
    
    Serial.print("Error_L: "); Serial.print(currentError_L); Serial.println();
    Serial.print("Error_R: "); Serial.print(currentError_R); Serial.println();    
    Serial.print("Velocity_L: "); Serial.print(currentVel_L); Serial.println();
    Serial.print("Velocity_R: "); Serial.print(currentVel_R); Serial.println();*/
  
    //update variables
    prevTime = currentTime;
    prevPos_L = currentPos_L;
    prevPos_R = currentPos_R;
    prevError_L = currentError_L;
    prevError_R = currentError_R;
    
    //read reflectance sensors to stay on line
    senseLine(bit_buf);
    
    if(bit_buf[6] || bit_buf[7] || bit_buf[8]) {
      
    }
    else if(bit_buf[9] == 1 || bit_buf[10] == 1 || bit_buf[11] == 1) { //too far right, veer left
      u_R += 20;
      u_L -= 20;
    }
    else if(bit_buf[5] == 1 || bit_buf[4] == 1 || bit_buf[3] == 1) { //too far left, veer right
      u_R -= 10;
      u_L += 10;
    }
    else if(emptyCheck == 13) { //no sensors active, off line, stop
      stopMove();
      endFlag = 1;
    }
    leftSide = 0;
    rightSide = 0;
    emptyCheck = 0;

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
    /*Serial.print("Control: "); Serial.print(u_R); Serial.println();
    Serial.println(); Serial.println();*/

    //drive motors
    M_RIGHT_forward(u_R); 
    M_LEFT_forward(u_L);  
    
    currentPos_L = encL.read(); 
    currentPos_R = -encR.read(); 
    /*if (currentPos_R >= finalPos_R || currentPos_L >= finalPos_L) {
      stopMove();
      endFlag = 1;
    }*/
  }
}

void senseLine(int *bit_buf) {
  int adc_buf[14];
  //read line sensor values, indices correspond to numbered sensors on the mouse
  for (int i = 0, j = 1; i < 8; i++) {
    adc_buf[j] = adc1.readADC(i);
    adc_buf[j+1] = adc2.readADC(i);
    j = j+2;
  }
  
  //convert to binary values, 0 = gray, 1 = white
  for (int i = 1; i < 14; i++) {
    if(adc_buf[i] < threshold) { //below threshold, white line
      bit_buf[i] = 1;
      if(i > 6) { //left 
        leftSide++;
      }
      if(i < 6) {
        rightSide++;
      }
    }
    else {
      bit_buf[i] = 0;
      emptyCheck++;
    }
  }
}