#include <PID_v1.h>
#include <Wire.h>

int incomingInt;
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
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
 // Serial.setTimeout(300);
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
  pinMode(heater, OUTPUT);
  pinMode(cooler, OUTPUT);
  analogReference(EXTERNAL);
  //Time = millis(); 

  //Initialize variables
  currentTemp= analogRead(tempSensor);
  currentTemp= ((3.3*currentTemp/1023)-0.5)*100; //celsius
  currentTemp = (currentTemp * 9.0 / 5.0) + 32.0 ;//fahrenheit
  desiredTemp= currentTemp+1;

  //turn the PID on
  tempPID.SetMode(AUTOMATIC);
  
}

void loop() {  

  //Read heater and cooler element temperatures
  currentTemp= analogRead(tempSensor);
  currentTemp= ((3.3*currentTemp/1024)-0.5)*100; //celsius
  currentTemp = (currentTemp * 9.0 / 5.0) + 32.0 ;//fahrenheit
 // Serial.print("current:");
  //Serial.println(currentTemp);
  
   //Safety Thresholds
   if(currentTemp>=140 || currentTemp<=60){
    analogWrite(heater, 0);
    return;  
   }
    Serial.read();
    delay(200);
  //Read incoming temperature reading
  if (Serial.available() > 0) {
       
    //incomingByte = Serial.readString();
    incomingInt = Serial.parseInt();
    
    // say what you got:
    //Serial.print("desired:");
    //Serial.println(incomingInt);

    if(incomingInt>60){ //thresholding temperatures at 60F
          desiredTemp = incomingInt;
          digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
          delay(10);                       // wait for a second
          digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW 
          //Serial.print("desired:");
          //Serial.println(desiredTemp);
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
          //Serial.print("HOutput: ");
          //Serial.println(PID_value);
       }
       if (desiredTemp<currentTemp && currentTemp>40){
          tempPID.SetControllerDirection(REVERSE);  //Reverse direction of controller (increase cooling to decrease temperature)
          tempPID.Compute();
          //write the PWM signal to the mosfet on digital pin
          analogWrite(cooler,PID_value);
          //Serial.print("COutput: "); 
          //Serial.println(PID_value);
       } 

       delay(100);

    }else{
      analogWrite(heater, 0);
      analogWrite(cooler, 0);
  }


}
