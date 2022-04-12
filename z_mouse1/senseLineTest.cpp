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

const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;

void senseLine(int *bit_buf);

/*Reflectance Sensor Array*/
int bit_buf[14];

/*PID Control*/
long prevTime = 0;
int prevPos_L = 0;
int prevPos_R = 0;

float targetVel = 15;  //in cm/s
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
float Kp_R = 0.3;
float Ki_R = 0.08;
float Kd_R = 0.06;

float Kp_L = 0.3; //needs adjusting
float Ki_L = 0.08;
float Kd_L = 0.06;

void setup() {
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);
  delay(100);
  
  Serial.begin(115200);

  adc1.begin(ADC_1_CS);  
  adc2.begin(ADC_2_CS);
  
  delay(3000);
}

void loop() {  
  senseLine(bit_buf);
  //print values
  for (int i = 1; i < 14; i++) {
    Serial.print(bit_buf[i]); Serial.print("\t");
  }
  Serial.println();
  delay(100);
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
    }
    else {
      bit_buf[i] = 0;
    }
  }
}