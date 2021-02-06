#include <PID_v1.h>
#include <Wire.h>

float incomingFloat;
float val =0.0;
int cooler = 3;
int heater = 5;
int tempSensor = A0;
double currentTemp;
double desiredTemp;
double PID_value = 0;
double prev_val;


//Specify the links and initial tuning parameters
PID tempPID(&currentTemp, &PID_value, &desiredTemp,2.6,2,0.5, DIRECT); //constants in order kp, ki, kd

void setup() {
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
  pinMode(heater, OUTPUT);
  pinMode(cooler, OUTPUT);
  //Time = millis(); 

  //Initialize variables
  currentTemp= analogRead(tempSensor);
  currentTemp= ((5*currentTemp/1023)-0.5)*100; //celsius
  currentTemp = (currentTemp * 9.0 / 5.0) + 32.0 ;//fahrenheit
  desiredTemp= currentTemp+1;

  //turn the PID on
  tempPID.SetMode(AUTOMATIC);
  
}

void loop() {

  //Read heater and cooler element temperatures
  currentTemp= analogRead(tempSensor);
  currentTemp= ((5*currentTemp/1023)-0.5)*100; //celsius
  currentTemp = (currentTemp * 9.0 / 5.0) + 32.0 ;//fahrenheit
  Serial.print("current:");
  Serial.println(currentTemp);
  
   //Safety Thressholds
   if(currentTemp>=140 || currentTemp<=60){
    analogWrite(heater, 0);
    return;  
   }
   
  //Read incoming temperature reading
  if (Serial.available() > 0) {
       
    //incomingByte = Serial.readString();
    incomingFloat = Serial.parseFloat();
    // say what you got:

    if(incomingFloat!=0){
          desiredTemp = incomingFloat;
          Serial.flush();
          Serial.print("desired:");
          Serial.println(desiredTemp);
    }
    }

      // PID:
    if(desiredTemp!=0){
       
        //Adjust heater/cooler
       if (desiredTemp>currentTemp && currentTemp<150){
          tempPID.SetControllerDirection(DIRECT); //Reset direction if reversed
          tempPID.Compute();
          //write the PWM signal to the mosfet on digital pin
          analogWrite(heater,PID_value);
          Serial.print("HOutput: ");
          Serial.println(PID_value);
       }
       if (desiredTemp<currentTemp && currentTemp>40){
          tempPID.SetControllerDirection(REVERSE);  //Reverse direction of controller (increase cooling to decrease temperature)
          tempPID.Compute();
          //write the PWM signal to the mosfet on digital pin
          analogWrite(cooler,PID_value);
          Serial.print("COutput: "); 
          Serial.println(PID_value);
       } 

       delay(300);

    }else{
      analogWrite(heater, 0);
      analogWrite(cooler, 0);
  }
            
delay(500);
}
