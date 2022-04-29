#include <Arduino.h>
#include <Adafruit_MCP3008.h> 
#include <Adafruit_MPU6050.h>
#include <Encoder.h>
#include "WiFi.h"
#include "HTTPClient.h"

#define GEAR_RATIO 29.86 //motor spec
#define ROTATION 360 //ticks for one wheel rotation
#define WHEEL_RADIUS 16 //mm
#define THRESHOLD 630 //differentiate white line from gray foam

#define NUM_CALIBRATION_SAMPLES 100

#define CENTER_ON_NODE_DISTANCE 9 //cm
#define STRAIGHT_DISTANCE 15 - CENTER_ON_NODE_DISTANCE //cm
#define CURVE_DISTANCE 15*TWO_PI/4 - CENTER_ON_NODE_DISTANCE //cm

int mouse = 2;  //select which mouse is running

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

const char* ssid = "GoTerps";
const char* password =  "goterps2022";

//PID Constants
float Kp_R = 0;
float Ki_R = 0;
float Kd_R = 0;

float Kp_L = 0;
float Ki_L = 0;
float Kd_L = 0;

int bit_buf[14]; //Reflectance Sensor Array
int emptyCheck = 0;

float kP_line = 0.03;
int line_error = 0;
float dist_adjust = 0.0;

sensors_event_t a, g, temp;

WiFiServer server(80);
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

float calibrateMouse() {
  delay(500);
  float zOffset = 0.0;

  for (int i = 0; i < NUM_CALIBRATION_SAMPLES; i++) {
    mpu.getEvent(&a, &g, &temp);
    zOffset = zOffset + (g.gyro.z / NUM_CALIBRATION_SAMPLES);
    delay(5);
  }

  return zOffset;
}

void rotateLeft(int deg, Encoder &encL, Encoder &encR) {
  float zPosition = 0.0;
  float timeElapsed = 0.0;
  float zVel = 0.0;
  float zOffset = calibrateMouse();
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

  int prevPos_L = 0;  //get start position
  int prevPos_R = 0;  //get start position
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
    M_LEFT_backward(u_L);  //rotate left backwards
    
    mpu.getEvent(&a, &g, &temp);
    zVel = (g.gyro.z - zOffset);
    timeElapsed = (millis() - prevTime_Rot);
    zPosition = zPosition + (zVel * timeElapsed / 1000);
    prevTime_Rot = millis();
    Serial.println(zPosition);
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
  float zOffset = calibrateMouse();
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

  int prevPos_L = 0;  //get start position
  int prevPos_R = 0;  //get start position
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
    M_LEFT_forward(u_L); 
    M_RIGHT_backward(u_R);  //rotate right backwards
    
    mpu.getEvent(&a, &g, &temp);
    zVel = (g.gyro.z - zOffset);
    timeElapsed = (millis() - prevTime_Rot);
    zPosition = zPosition + (zVel * timeElapsed / 1000);
    prevTime_Rot = millis();
    Serial.println(zPosition);
    if (zPosition <= -deg*DEG_TO_RAD) {
      stopMove();
      return;
    }
    delay(10);
  }
}

/* END Rotational Movement *************************************/




/* BEGIN Forward Movement *************************************/

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
    if(adc_buf[i] < THRESHOLD) { //below threshold, white line
      bit_buf[i] = 1;
    }
    else {
      bit_buf[i] = 0;
      emptyCheck++;
    }
  }
}

void PIDForward(int distance, Encoder &encL, Encoder &encR) {
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
  finalPos_L = distance/(2*PI*WHEEL_RADIUS/10)*ROTATION + prevPos_L - dist_adjust; //convert desired distance to encoder value
  finalPos_R = distance/(2*PI*WHEEL_RADIUS/10)*ROTATION + prevPos_R - dist_adjust; //convert desired distance to encoder value
  encL.write(0); encR.write(0); //clear encoders
  delay(10);

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
    Serial.print("Line Position: "); Serial.print(line_pos); Serial.println();
    int target_line = 7; //desired average is the center
    line_error = target_line - line_pos; //positive if mouse is too far left (right sensors predominant) 
    Serial.print("Error: "); Serial.print(line_error); Serial.println();
    float adjust_vel = kP_line * line_error;
    Serial.print("Adjustment: "); Serial.print(adjust_vel); Serial.println();
    if(line_error == 0) {  //on track, reset
      targetVel_L = 10;
      targetVel_R = 10;
    }
    else {
      targetVel_L += adjust_vel;  //adjust is pos --> left needs to be pos.
      targetVel_R -= adjust_vel;  //adjust is neg --> right needs to be pos. (double neg)
    }
    if(emptyCheck == 13) { //no sensors active, off line, stop
      stopMove();
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

void PIDForwardNoLine(int distance, Encoder &encL, Encoder &encR) {
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
  finalPos_L = distance/(2*PI*WHEEL_RADIUS/10)*ROTATION + prevPos_L - dist_adjust; //convert desired distance to encoder value
  finalPos_R = distance/(2*PI*WHEEL_RADIUS/10)*ROTATION + prevPos_R - dist_adjust; //convert desired distance to encoder value
  encL.write(0); encR.write(0); //clear encoders
  delay(10);

  while (1) {
    long currentTime = micros(); //time in us
    int currentPos_L = encL.read(); //actually right
    int currentPos_R = -encR.read(); // actually left
    float deltaTime = ((float) (currentTime - prevTime))/1.0e6; //delta time in s
    //float currentVel_L = ((float)(currentPos_L - prevPos_L)/ROTATION)/deltaTime; //rev per sec
    //float metricVel_L = currentVel_L*2*PI*WHEEL_RADIUS/10; //in cm/s
    //float currentVel_R = ((float)(currentPos_R - prevPos_R)/ROTATION)/deltaTime; //rev per sec
    //float metricVel_R = currentVel_R*2*PI*WHEEL_RADIUS/10; //in cm/s

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
    /*Serial.print("Control: "); Serial.print(u_R); Serial.println();
    Serial.println(); Serial.println();*/

    //drive motors
    M_RIGHT_forward(u_R); 
    M_LEFT_forward(u_L);  
    
    currentPos_L = encL.read(); 
    currentPos_R = -encR.read(); 
    if (currentPos_R >= finalPos_R || currentPos_L >= finalPos_L) {
      stopMove();
      return;
    }
    delay(10);
  }
}

/* END Forward Movement *************************************/




/* BEGIN Instruction Handling *************************************/

void instructionHandler(char instruction, Encoder &encL, Encoder &encR){
  switch (instruction) {
    case 'L': rotateLeft(90); break;
    case 'R': rotateRight(90); break;
    case 'B': rotateRight(180); break;
    case 'F': PIDForward(STRAIGHT_DISTANCE, encL, encR); break;
    case 'C': PIDForward(CURVE_DISTANCE, encL, encR); break;
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
    case 'S': delay(5000); break; // TODO: "Success" sound
  }
}

void pingJetson(String serverName){
  http.begin(serverName);

  int httpResponseCode = http.GET(); // Send HTTP GET request
  String payload = "{}"; 

  if (httpResponseCode>0) {
    payload = http.getString();
  }

  Serial.println(httpResponseCode);
  Serial.println(payload);

  http.end(); // Free resources
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
      dist_adjust = 0;
      Kp_R = 1.02;
      Ki_R = 0.8;
      Kd_R = 0.1;

      Kp_L = 1.86;
      Ki_L = 0.5;
      Kd_L = 0.1;
      break;

    case 2:
      Kp_R = 0.96;
      Ki_R = 1;
      Kd_R = 0.25;

      Kp_L = 1.62;
      Ki_L = 1;
      Kd_L = 0.25;
      dist_adjust = 1.3;
      break;

    case 3:
      Kp_R = 1.56;
      Ki_R = 1;
      Kd_R = 0.1;

      Kp_L = 1.62;
      Ki_L = 1.2;
      Kd_L = 0.1;
      dist_adjust = 0;
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

  server.begin();
  
}




/* MAIN LOOP *************************************/

void loop() {
  Encoder encR(M_RIGHT_ENC_A, M_RIGHT_ENC_B); //right wheel, forward neg
  Encoder encL(M_LEFT_ENC_A, M_LEFT_ENC_B); //left wheel, forward pos

  while(true) {
    WiFiClient client = server.available();   // listen for incoming clients

    if (client) {                           
      String currentLine = "";                
      while (client.connected()) {            // loop while the client's connected
        if (client.available()) {             // if there's bytes to read
          char c = client.read();             // read a byte
          if (c == '\n' && currentLine.length() != 0) {    
            // newline -> clear currentLine:
            currentLine = "";
          } else if (c == '\n' && currentLine.length() == 0) {
            // blank line = end of the client HTTP request -> send a response
            client.println("HTTP/1.1 200 OK");        // response code
            client.println("Content-type:text/html"); // content-type
            client.println();                         // end of header
            client.print("<p>Connected!</p><br>");     // response content
            client.println();                         // end of response content
            break;
          } else if (c != '\r') {
            // anything but a carriage return character -> add to currentLine
            currentLine += c;
          }
          
          if (currentLine.endsWith("/instruct")) {
            String instructions = currentLine.substring(currentLine.indexOf("GET /") + 5, currentLine.indexOf("/instruct"));

            PIDForwardNoLine(CENTER_ON_NODE_DISTANCE, encL, encR); // move up to the node

            // left final instruction out of the loop (final forward/curve instructions are handled differently)
            for (byte i = 0; i < sizeof(instructions) - 1; i = i + 1) {
              char instruction = instructions[i];
              instructionHandler(instruction, encL, encR);

              // for forward/curve instructions, move up to the node to finish the instruction
              if (instruction == 'F' || instruction == 'C') {
                PIDForwardNoLine(CENTER_ON_NODE_DISTANCE, encL, encR);
              }
            }
            
            // the last instruction in the set doesn't move the mouse up to the node yet
            instructionHandler(instructions[sizeof(instructions) - 1], encL, encR);

            // Send GET request to instruction server
            pingJetson("http://" + client.remoteIP().toString() + ":8000/next");
          }
        }
      }

      // close the connection:
      client.stop();
      Serial.println("Client Disconnected.");
    }
  }
}