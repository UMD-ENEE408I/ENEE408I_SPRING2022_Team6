void PIDForwardNoLine(int distance, Encoder &encR, Encoder &encL) {
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

  //PID constants
  float Kp_R = 0.96;
  float Ki_R = 1;
  float Kd_R = 0.25;

  float Kp_L = 1.62;
  float Ki_L = 1;
  float Kd_L = 0.25;

  int prevPos_L = 0;  //get start position
  int prevPos_R = 0;  //get start position
  long prevTime = micros();  //get start time in us
  finalPos_L = distance/(2*PI*WHEEL_RADIUS/10)*ROTATION + prevPos_L; //convert desired distance to encoder value
  finalPos_R = distance/(2*PI*WHEEL_RADIUS/10)*ROTATION + prevPos_R; //convert desired distance to encoder value
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