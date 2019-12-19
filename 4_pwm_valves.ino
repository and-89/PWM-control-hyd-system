//control system for two hydraulic motors with for PWM valves
const byte joysticYA = A0; //Analog Jostick Y axis
const byte joysticXA = A1; //Analog Jostick X axis

const byte PWMleftFA = 10; //PWM FORWARD PIN for OSMC Controller A (left motor)
const byte PWMleftRA = 9;  //PWM REVERSE PIN for OSMC Controller A (left motor)
const byte PWMrightFB = 6;  //PWM FORWARD PIN for OSMC Controller B (right motor)
const byte PWMrightRB = 5;  //PWM REVERSE PIN for OSMC Controller B (right motor)
const byte disablePin = 2; //OSMC disable, pull LOW to enable motor controller

int analogTmp = 0; //temporary variable to store 
int throttle, direction = 0; //throttle (Y axis) and direction (X axis) 

int leftMotor,leftMotorScaled = 0; //left Motor helper variables
float leftMotorScale = 0;

int rightMotor,rightMotorScaled = 0; //right Motor helper variables
float rightMotorScale = 0;

float maxMotorScale = 0; //holds the mixed output scaling factor

int deadZone = 5; //jostick dead zone 

void setup()  { 

  //initialization of pins  
  Serial.begin(19200);
  pinMode(PWMleftFA, OUTPUT);
  pinMode(PWMleftRA, OUTPUT);
  pinMode(PWMrightFB, OUTPUT);
  pinMode(PWMrightRB, OUTPUT);  

  pinMode(disablePin, OUTPUT);
  digitalWrite(disablePin, LOW);
} 

void loop()  { 
  //analog input rescale the 0..1023 range to -255..255 range
  analogTmp = analogRead(joysticYA);
  throttle = (512-analogTmp)/2;

  delayMicroseconds(100);
  //...and  the same for X axis
  analogTmp = analogRead(joysticXA);
  direction = (512-analogTmp)/2;

  //mix throttle and direction
  leftMotor = throttle;
  rightMotor = direction;

  //print the initial mix results
  Serial.print("LIN:"); Serial.println( leftMotor, DEC);
  Serial.print(", RIN:"); Serial.print( rightMotor, DEC);

  //calculate the scale of the results in comparision base 8 bit PWM resolution
  leftMotorScale =  leftMotor/255.0;
  leftMotorScale = abs(leftMotorScale);
  rightMotorScale =  rightMotor/255.0;
  rightMotorScale = abs(rightMotorScale);

  Serial.print("| LSCALE:"); Serial.print( leftMotorScale,2);
  Serial.print(", RSCALE:"); Serial.print( rightMotorScale,2);

  //choose the max scale value if it is above 1
  maxMotorScale = max(leftMotorScale,rightMotorScale);
  maxMotorScale = max(1,maxMotorScale);

  //and apply it to the mixed values
  leftMotorScaled = constrain(leftMotor/maxMotorScale,-120,185); //(x,y)joystik lewy do tyłu x, do przodu y
  rightMotorScaled = constrain(rightMotor/maxMotorScale,-65,70);  //(x,y)joystik prawy do tyłu x, do przodu y

  Serial.print("| LOUT:"); Serial.print( leftMotorScaled);
  Serial.print(", ROUT:"); Serial.print( rightMotorScaled);

  Serial.print(" |");

  //apply the results to appropriate uC PWM outputs for the LEFT motor:
  if(abs(leftMotorScaled)>deadZone)
  {

    if (leftMotorScaled > 0)
    {
      Serial.print("F");
      Serial.print(abs(leftMotorScaled),DEC);

      analogWrite(PWMleftRA,0);
      analogWrite(PWMleftFA,abs(leftMotorScaled));            
    }
    else 
    {
      Serial.print("R");
      Serial.print(abs(leftMotorScaled),DEC);

      analogWrite(PWMleftFA,0);
      analogWrite(PWMleftRA,abs(leftMotorScaled));  
    }
  }  
  else 
  {
  Serial.print("IDLE");
  analogWrite(PWMleftFA,0);
  analogWrite(PWMleftRA,0);
  } 

  if(abs(rightMotorScaled)>deadZone)
  {

    if (rightMotorScaled > 0)
    {
      Serial.print("F");
      Serial.print(abs(rightMotorScaled),DEC);

      analogWrite(PWMrightRB,0);
      analogWrite(PWMrightFB,abs(rightMotorScaled));            
    }
    else 
    {
      Serial.print("R");
      Serial.print(abs(rightMotorScaled),DEC);

      analogWrite(PWMrightFB,0);
      analogWrite(PWMrightRB,abs(rightMotorScaled));  
    }
  }  
  else 
  {
  Serial.print("IDLE");
  analogWrite(PWMrightFB,0);
  analogWrite(PWMrightRB,0);
  } 

  Serial.println("");


  delay(10);

}
  
  
