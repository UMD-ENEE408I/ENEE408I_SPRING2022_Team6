#include <Adafruit_MCP3008.h>

#define threshold 630 //differentiate line from foam 

Adafruit_MCP3008 adc1;
Adafruit_MCP3008 adc2;

const unsigned int ADC_1_CS = A3;
const unsigned int ADC_2_CS = A2;
const unsigned int RF_CS = A4;

int adc_buf[14]; //raw adc values, size for index matching
int bit_buf[14]; //binary array from threshold

int sum = 0;

int center_bit;
int left_bit;
int right_bit;

const unsigned int M1_IN_1 = 2;
const unsigned int M1_IN_2 = 3;
const unsigned int M2_IN_1 = 5;
const unsigned int M2_IN_2 = 4;

const unsigned int M1_I_SENSE = A1;
const unsigned int M2_I_SENSE = A0;

const float M_I_COUNTS_TO_A = (3.3 / 1024.0) / 0.120;
const unsigned int PWM_VALUE = 60;

/*BEGIN basic motor movements*/
void M1_backward() {
  analogWrite(M1_IN_1, PWM_VALUE);
  analogWrite(M1_IN_2, 0);
}

void M1_forward() {
  analogWrite(M1_IN_1, 0);
  analogWrite(M1_IN_2, PWM_VALUE);
}

void M1_stop() {
  analogWrite(M1_IN_1, 0);
  analogWrite(M1_IN_2, 0);
}

void M2_backward() {
  analogWrite(M2_IN_1, PWM_VALUE);
  analogWrite(M2_IN_2, 0);
}

void M2_forward() {
  analogWrite(M2_IN_1, 0);
  analogWrite(M2_IN_2, PWM_VALUE);
}

void M2_stop() {
  analogWrite(M2_IN_1, 0);
  analogWrite(M2_IN_2, 0);
}

void stopMove() {
  M1_stop();
  M2_stop();
}

void moveForward() {
  M1_forward();
  M2_forward();
}

void moveBackward() {
  M1_backward();
  M2_backward();
}

void turnLeft() {
  M1_forward();
  M2_stop();
}

void turnRight() {
  M1_stop();
  M2_forward();
}
/*END basic motor movements*/

void setup() {
  Serial.begin(115200);

  pinMode(M1_IN_1, OUTPUT);
  pinMode(M1_IN_2, OUTPUT);
  pinMode(M2_IN_1, OUTPUT);
  pinMode(M2_IN_2, OUTPUT);

  pinMode(RF_CS, OUTPUT);
  digitalWrite(RF_CS, HIGH); // Without this the nRF24 will write to the SPI bus
                             // while the ADC's are also talking
  adc1.begin(ADC_1_CS);  
  adc2.begin(ADC_2_CS);  
}

//line following
void loop() {
  senseLine();  //get readings
  while (sum != 0) {
    if (center_bit == 1) { //on line
      moveForward();
    }
    else if (center_bit != 1) {
      if (left_bit == 1) { //too far right
        turnLeft();
      }
      if (right_bit == 1) { //too far left
        turnRight();
      }
    }
    else {
      stopMove();
    }
    senseLine();
  }
  stopMove();
}

void senseLine() {
  sum = 0;
  //read line sensor values
  for (int i = 0, j = 1; i < 8; i++) {
    adc_buf[j] = adc1.readADC(i);
    adc_buf[j+1] = adc2.readADC(i);
    j = j+2;
  }
  
  //convert to binary values, 0 = gray, 1 = white
  for (int i = 1; i < 14; i++) {
    if(adc_buf[i] < threshold) { //below threshold, white line
      bit_buf[i] = 1;
      sum++;
    }
    else {
      bit_buf[i] = 0;
    }
  }

  //store important values
  center_bit = bit_buf[7];
  left_bit = bit_buf[9];
  right_bit = bit_buf[5];

  
  //print values
  for (int i = 1; i < 14; i++) {
    Serial.print(bit_buf[i]); Serial.print("\t");
  }
  Serial.println();
}
