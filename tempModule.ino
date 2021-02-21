#include <PID_v1.h>
#include <Wire.h>
#include <FIR.h>

int incomingInt;
float incomingFloat;
float val =0.0;
int cooler = 3;
int heater = 5;
int tempSensor = A0;
double currentTemp;
double desiredTemp;
double PID_value = 0;
double PID_value2 = 0;
double prev_val;


//Specify the links and initial tuning parameters
//Heater PID
PID heatPID(&currentTemp, &PID_value, &desiredTemp,12.5,2.5,0.65, DIRECT); //constants in order kp, ki, kd
//Cooler PID
PID coolPID(&currentTemp, &PID_value2, &desiredTemp,12.75,3.85,0.9, REVERSE); //constants in order kp, ki, kd

//Moving average filter, five samples
FIR<float, 5> fir_avg;

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
  heatPID.SetMode(AUTOMATIC);
  coolPID.SetMode(AUTOMATIC);

  //filter coefficients
  float coef_avg[5] = {1., 1., 1., 1., 1.};

  // Set the coefficients
  fir_avg.setFilterCoeffs(coef_avg);

  // Set the gain
 // Serial.print("Moving Average Filter Gain: ");
 // Serial.println(fir_avg.getGain());
}

void loop() {

  //Read heater and cooler element temperatures
  currentTemp= analogRead(tempSensor);
  currentTemp= ((3.3*currentTemp/1023)-0.5)*100; //celsius
  currentTemp = (currentTemp * 9.0 / 5.0) + 32.0 ;//fahrenheit
  
   //Safety Thressholds
   if(currentTemp>=140 || currentTemp<60){
    analogWrite(heater, 0);
    return;  
   }
   //Serial.read();
   //delay(200);
   
   currentTemp = fir_avg.processReading(currentTemp);
   //Serial.print("current:");
   Serial.println(currentTemp);
   
  //Read incoming temperature reading
  if (Serial.available() > 0) {
       
    //incomingByte = Serial.readString();
    incomingInt = Serial.parseInt();
    //incomingFloat = Serial.parseFloat();
    
    // say what you got:

    if(incomingInt>=50){ //thresholding temperatures at 60F
          desiredTemp = incomingInt;
          digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
          delay(10);                       // wait for a second
          digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW 
          //Serial.print("desired:");
          Serial.println(desiredTemp);
     }
    }

      // PID:
    if(desiredTemp>=50){
       
        //Adjust heater/cooler
       if (desiredTemp>currentTemp && currentTemp<150){
          //heatPID.SetControllerDirection(DIRECT); //As temp increases heat output can decrease?
          heatPID.Compute();
          //write the PWM signal to the mosfet on digital pin
          analogWrite(cooler, 0);
          if(PID_value>255){
            analogWrite(heater, 255);
          }else{
            analogWrite(heater,PID_value);
          }
          //Serial.print("HOutput: ");
          //Serial.println(PID_value);
       }
       if (desiredTemp<currentTemp && currentTemp>40){
          //heatPID.SetControllerDirection(DIRECT); //As temp increases cold output should increase
          coolPID.Compute();
          //write the PWM signal to the mosfet on digital pin
          analogWrite(heater, 0);
          if(PID_value2>255){
            analogWrite(cooler, 255);
          }else{
            analogWrite(cooler,PID_value2);
          }
         //Serial.print("COutput: "); 
         //Serial.println(PID_value2);
       } 

       delay(100);

    }else{
      analogWrite(heater, 0);
      analogWrite(cooler, 0);
  }
            
delay(500);
}
